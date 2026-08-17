//
//  Decoder.cpp
//  AVDecoder
//
//  Created by Grishka on 29.08.2025.
//

#include "Decoder.hpp"
#include "Filters.hpp"
#include <stdio.h>
#include <mach/mach_time.h>

// https://www.batsocks.co.uk/readme/video_timing.htm
#define DEFAULT_LINE_DURATION 1280 // 64 us
#define MIN_LINE_DURATION 1250
#define MAX_LINE_DURATION 1300
#define DEFAULT_FRAME_DURATION (DEFAULT_LINE_DURATION*625)
#define DEFAULT_FIELD_DURATION (DEFAULT_FRAME_DURATION/2)
#define MIN_FRAME_DURATION (DEFAULT_LINE_DURATION*620)
#define MAX_FRAME_DURATION (DEFAULT_LINE_DURATION*626)
#define LINE_LONG_SYNC_DURATION 94 // 4.7 us
#define LINE_SYNC_DURATION 47 // 2.35 us
#define FIELD_SYNC_DURATION 546 // 27.3 us
#define LINE_SYNC_WINDOW (LINE_SYNC_DURATION*3/6)
#define FIELD_SYNC_WINDOW (FIELD_SYNC_DURATION*3/4)

#define MIN_SYNC_DURATION 35 // 1.75 us
#define LINE_SYNC_MIN_DURATION 70 // 3.5 us
#define LINE_SYNC_MAX_DURATION 129 // 6.45 us

class BaseSignalBuffers{
public:
	float *luminance;
	float *chrominance[2];
	float *filteredLuminance;
	float *raw;
	
	void setFrom(BaseSignalBuffers *other, int offset){
		luminance=other->luminance+offset;
		chrominance[0]=other->chrominance[0]+offset;
		chrominance[1]=other->chrominance[1]+offset;
		filteredLuminance=other->filteredLuminance+offset;
		raw=other->raw+offset;
	}
};

class SignalBuffers: public BaseSignalBuffers{
public:
	SignalBuffers(){
	}
	
	virtual ~SignalBuffers(){
		free(luminance);
		free(chrominance[0]);
		free(chrominance[1]);
		free(filteredLuminance);
		free(raw);
	}
	
	void init(){
		int size=getBufferSize();
		luminance=(float*)malloc(size*sizeof(float));
		chrominance[0]=(float*)malloc(size*sizeof(float));
		chrominance[1]=(float*)malloc(size*sizeof(float));
		filteredLuminance=(float*)malloc(size*sizeof(float));
		raw=(float*)malloc(size*sizeof(float));
	}
	
protected:
	virtual int getBufferSize(){
		return BUFFER_SIZE+1;
	}
};

class VideoField: public SignalBuffers{
public:
	std::vector<VideoLine> lines;
	bool isBottom;
	float syncLevel;
	float blackLevel;
	int numSamples=0;

	void appendSamples(BaseSignalBuffers *buf, int srcOffset, int count){
		assert(numSamples+count<=getBufferSize());
		
		memcpy(luminance+numSamples, buf->luminance+srcOffset, count*sizeof(float));
		memcpy(chrominance[0]+numSamples, buf->chrominance[0]+srcOffset, count*sizeof(float));
		memcpy(chrominance[1]+numSamples, buf->chrominance[1]+srcOffset, count*sizeof(float));
		memcpy(filteredLuminance+numSamples, buf->filteredLuminance+srcOffset, count*sizeof(float));
		memcpy(raw+numSamples, buf->raw+srcOffset, count*sizeof(float));
		numSamples+=count;
	}
	
protected:
	virtual int getBufferSize() override{
		return MAX_LINE_DURATION*625;
	}
};

class VideoLine: public BaseSignalBuffers{
public:
	std::vector<Decoder::SyncPulse> sync;
	int offsetInBuffer;
	int numSamples;
	
	VideoLine(){}
	
	VideoLine(BaseSignalBuffers *buf, int offset, int length, vector<Decoder::SyncPulse> sync):sync(sync), numSamples(length){
		setFrom(buf, offset);
	}
};

Decoder::Decoder(void* bitmapData, unsigned int bitmapWidth, unsigned int bitmapHeight, unsigned int bitmapStride, CFRunLoopSourceRef runLoopSource) :bitmapData(bitmapData), bitmapWidth(bitmapWidth), bitmapHeight(bitmapHeight), bitmapStride(bitmapStride/4), decoderThread(std::bind(&Decoder::runDecoderThread, this)), runLoopSource(runLoopSource){
	
	currentOutputBuffer=outputBufferPool.Get();
	mainThreadRunLoop=CFRunLoopGetMain();
	interpolatedField=new VideoField();
	interpolatedField->init();
	for(int i=0;i<625;i++){
		VideoLine line;
		line.setFrom(interpolatedField, DEFAULT_LINE_DURATION*i);
		line.numSamples=DEFAULT_LINE_DURATION;
		interpolatedField->lines.push_back(line);
	}

	decoderThread.SetName("Decoder");
	decoderThread.Start();
}

Decoder::~Decoder(){
	delete colorDecoder;
	delete interpolatedField;
}

void Decoder::handleSampleData(uint8_t* samples, size_t count){
	blockingSemaphore.Acquire();
	tgvoip::Buffer buf;
	try{
		buf=bufferPool.Get();
	}catch(std::bad_alloc& x){
		printf("Can't keep up with real time, dropping buffer!\n");
		blockingSemaphore.Release();
		return;
	}
	buf.CopyFrom(samples, 0, count);
	blockingSemaphore.Release();
	newlyAcquiredDataBuffers.Put(std::move(buf));
}

void Decoder::runDecoderThread(){
	blackLevel=1;
	
	float *samples=(float*)calloc(BUFFER_SIZE, sizeof(float));
	float *nextSamples=(float*)calloc(BUFFER_SIZE, sizeof(float));
	
	vector<SyncPulse> currentFieldSyncPulses;
	vector<SyncPulse> syncPulseLocations;
	vector<SyncPulse> fixedSyncPulseLocations;
	
	vector<VideoLine> lineBuffer;
	
	SignalBuffers *b=new SignalBuffers();
	b->init();
	SignalBuffers *prevBuf=new SignalBuffers();
	prevBuf->init();
	SignalBuffers *nextBuf=new SignalBuffers();
	nextBuf->init();
	
	for(int i=0;i<6;i++){
		fieldPool.push_back(new VideoField());
		fieldPool[i]->init();
	}
	
	VideoField *currentField=fieldPool.front();
	fieldPool.pop_front();
	
	BiquadFilter syncLowpass(-1.69096225, 0.73269106, 0.01043220, 0.02086441, 0.01043220);
	BiquadFilter luminanceLowpass(-0.76129177, 0.27595249, 0.18599880, 0.23921641, 0.08944551);

	int bufferCount=0;
	
	int lastLongSyncPulseLocation=0;
	int earliestNextLongSyncPulseLocation=-1;
	bool nextFieldIsBottom=false;
	
	while(running){
		if(syncPulseLocations.size()>10){
			syncPulseLocations.erase(syncPulseLocations.begin(), syncPulseLocations.end()-10);
			for(SyncPulse& sp:syncPulseLocations){
				sp.location-=BUFFER_SIZE;
			}
		}else{
			while(syncPulseLocations.size()<10)
				syncPulseLocations.push_back({-1, 0});
		}
		tgvoip::Buffer buf=newlyAcquiredDataBuffers.GetBlocking();
		
		//blockingSemaphore.Acquire();
		tgvoip::MutexGuard mutexGuard(colorDecoderMutex);

		SignalBuffers *tmpB=prevBuf;
		prevBuf=b;
		b=nextBuf;
		nextBuf=tmpB;
		
		float *tmp=nextSamples;
		nextSamples=samples;
		samples=tmp;
		
		memcpy(nextBuf->raw, nextSamples, BUFFER_SIZE*sizeof(float));
		
		for(int i=0;i<BUFFER_SIZE;i++){
			int32_t sampleInt=(int32_t)buf[i];
			nextSamples[i]=sampleInt/255.0;
		}
		
		colorDecoder->separateSubcarrier(nextBuf, nextSamples);
		
		syncLowpass.process(nextBuf->luminance, nextBuf->filteredLuminance, BUFFER_SIZE);
		luminanceLowpass.process(nextBuf->luminance, nextBuf->luminance, BUFFER_SIZE);

		colorDecoder->demodulateSubcarrier(nextBuf);

		float minLevels[128];
		for(int i=0;i<128;i++)
			minLevels[i]=1;
		for(int i=0;i<BUFFER_SIZE;i++){
			float sample=b->luminance[i];
			int index=i>>12;
			minLevels[index]=std::min(sample, minLevels[index]);
		}
		float minLevel=0;
		for(int i=0;i<128;i++)
			minLevel+=minLevels[i];
		minLevel/=128.0f;
		//printf("minLevel %f, maxLevel %f\n", minLevel, maxLevel);
		float threshold=minLevel+0.1;
		float lowSampleSum=0;
		int lowSampleCount=0;
		for(int i=0;i<BUFFER_SIZE;i++){
			float s=b->luminance[i];
			if(s<threshold){
				lowSampleSum+=s;
				lowSampleCount++;
			}
		}
		if(lowSampleCount>0){
			syncLevel=lowSampleSum/lowSampleCount;
			syncThreshold=syncLevel+0.08;
		}
		int samplesForBlackLevel=0;
		int samplesUntilBlackLevelIsSampled=0;
		float blackSampleSum[128]={0};
		int blackSampleCount[128]={0};
		for(int i=0;i<BUFFER_SIZE;i++){
			float s=b->luminance[i];
			if(s<syncThreshold){
				samplesUntilBlackLevelIsSampled=15;
			}else if(samplesUntilBlackLevelIsSampled>0){
				samplesUntilBlackLevelIsSampled--;
				samplesForBlackLevel=40;
			}else if(samplesForBlackLevel>0){
				samplesForBlackLevel--;
				if(s<0.5){
					int index=i>>12;
					blackSampleSum[index]+=s;
					blackSampleCount[index]++;
				}
			}
		}
		float blackSampleAverage=0;
		int blackSampleNonZeroCount=0;
		for(int i=0;i<128;i++){
			if(blackSampleCount[i]){
				blackSampleSum[i]/=(float)blackSampleCount[i];
				blackSampleAverage+=blackSampleSum[i];
				blackSampleNonZeroCount++;
			}
		}
		if(blackSampleNonZeroCount>0){
			blackSampleAverage/=blackSampleNonZeroCount;
			float finalBlackSum=0;
			int finalBlackCount=0;
			for(int i=0;i<128;i++){
				if(blackSampleSum[i]<=blackSampleAverage){
					finalBlackSum+=blackSampleSum[i];
					finalBlackCount++;
				}
			}
			if(finalBlackCount){
				float newBlackLevel=finalBlackSum/finalBlackCount;
				syncThreshold=syncLevel+(newBlackLevel-syncLevel)*0.25;
				float newVisibleBrightnessRange=std::min((newBlackLevel-syncLevel)/whiteLevelRatio, 0.99f-newBlackLevel);
				if(fabsf(newBlackLevel-blackLevel)>=0.005f || fabsf(newVisibleBrightnessRange-visibleBrightnessRange)>=0.005f){
					blackLevel=newBlackLevel;
					visibleBrightnessRange=newVisibleBrightnessRange;
					//printf("sync %f, black %f, white %f\n", syncLevel, blackLevel, blackLevel+visibleBrightnessRange);
				}
			}
		}
		
		bool insidePulse=false;
		int pulseStart=0;
		for(int i=0;i<BUFFER_SIZE+DEFAULT_LINE_DURATION;i++){
			float s;
			if(i<BUFFER_SIZE)
				s=b->filteredLuminance[i];
			else
				s=nextBuf->filteredLuminance[i-BUFFER_SIZE];
			if(insidePulse){
				if(s>syncThreshold){
					insidePulse=false;
					int length=i-pulseStart;
					if(length>=MIN_SYNC_DURATION){
						syncPulseLocations.push_back({pulseStart, length});
					}
				}
			}else{
				if(s<syncThreshold){
					insidePulse=true;
					pulseStart=i;
				}
			}
		}
		
		for(int i=10;i<syncPulseLocations.size();i++){
			int loc=syncPulseLocations[i].location;
			if(loc>=BUFFER_SIZE)
				break;
			if(syncPulseLocations[i].length>LINE_SYNC_MAX_DURATION && loc>=earliestNextLongSyncPulseLocation){
				int prevLineSyncPos=0;
				for(int j=1;j<10;j++){
					int length=syncPulseLocations[i-j].length;
					if(length>LINE_SYNC_MIN_DURATION && length<LINE_SYNC_MAX_DURATION){
						prevLineSyncPos=syncPulseLocations[i-j].location;
						break;
					}
				}
				float linesSinceLastLineSync=(loc-prevLineSyncPos)/(float)DEFAULT_LINE_DURATION;
				float phaseRelativeToLineSync=linesSinceLastLineSync-(int)linesSinceLastLineSync;
				int fieldDuration=loc-lastLongSyncPulseLocation;
				bool nextIsBottom=phaseRelativeToLineSync>0.25f && phaseRelativeToLineSync<0.75f;
				if(nextIsBottom)
					fieldDuration-=DEFAULT_LINE_DURATION/2;
				fieldDuration=std::min(MAX_LINE_DURATION*625, fieldDuration);
				//printf("long pulse: %d samples (%f / %d lines) location %d last %d, %s field, add %d samples\n", fieldDuration, fieldDuration/(float)DEFAULT_LINE_DURATION, (int)roundf(fieldDuration/(float)DEFAULT_LINE_DURATION), loc, lastLongSyncPulseLocation, nextFieldIsBottom ? "bottom" : "top", lastLongSyncPulseLocation+fieldDuration);
				
				VideoField *nextField=fieldPool.front();
				fieldPool.pop_front();
				assert(nextField->numSamples==0);
				if(lastLongSyncPulseLocation+fieldDuration<0){
					int samplesToMove=-(lastLongSyncPulseLocation+fieldDuration);
					//printf("moving %d samples\n", samplesToMove);
					int offset=currentField->numSamples-samplesToMove;
					nextField->appendSamples(currentField, offset, samplesToMove);
					currentField->numSamples-=samplesToMove;
				}else{
					int offset=std::max(0, lastLongSyncPulseLocation);
					int count=fieldDuration+std::min(lastLongSyncPulseLocation, 0);
					int maxCount=(MAX_LINE_DURATION*625)-currentField->numSamples;
					currentField->appendSamples(b, offset, std::min(count, maxCount));
				}
				assert(currentField->numSamples==fieldDuration);
				for(int j=10;j<i;j++){
					if(syncPulseLocations[j].location>=lastLongSyncPulseLocation)
						currentFieldSyncPulses.push_back(syncPulseLocations[j].offset(-lastLongSyncPulseLocation));
				}
				vector<VideoLine> field=processField(currentField, currentFieldSyncPulses, syncLevel, blackLevel, visibleBrightnessRange, nextFieldIsBottom);
				currentField->numSamples=0;
				fieldPool.push_back(currentField);
				currentField=nextField;
				currentFieldSyncPulses.clear();
				
				nextFieldIsBottom=nextIsBottom;
				lastLongSyncPulseLocation+=fieldDuration;
				earliestNextLongSyncPulseLocation=loc+DEFAULT_LINE_DURATION*310;
				if(earliestNextLongSyncPulseLocation>=BUFFER_SIZE)
					break;
			}
		}
		
		int unprocessedSamplesRemain=BUFFER_SIZE-lastLongSyncPulseLocation;
		//printf("remaining: %d, last %d\n", unprocessedSamplesRemain, lastLongSyncPulseLocation);
		int nextFieldDuration;
		if(nextFieldIsBottom)
			nextFieldDuration=DEFAULT_LINE_DURATION*313;
		else
			nextFieldDuration=DEFAULT_LINE_DURATION*312;
		while(unprocessedSamplesRemain>=nextFieldDuration){
			int offset=std::max(0, lastLongSyncPulseLocation);
			int count=nextFieldDuration+std::min(lastLongSyncPulseLocation, 0);

			currentField->appendSamples(b, offset, count);

			assert(currentField->numSamples==nextFieldDuration);
			for(int j=10;j<syncPulseLocations.size();j++){
				if(syncPulseLocations[j].location>=std::min(BUFFER_SIZE, lastLongSyncPulseLocation+nextFieldDuration))
					break;
				if(syncPulseLocations[j].location>=lastLongSyncPulseLocation)
					currentFieldSyncPulses.push_back(syncPulseLocations[j].offset(-lastLongSyncPulseLocation));
			}
			vector<VideoLine> field=processField(currentField, currentFieldSyncPulses, syncLevel, blackLevel, visibleBrightnessRange, nextFieldIsBottom);
			currentField->numSamples=0;
			currentFieldSyncPulses.clear();

			lastLongSyncPulseLocation+=nextFieldDuration;
			earliestNextLongSyncPulseLocation=lastLongSyncPulseLocation+DEFAULT_LINE_DURATION*10;
			unprocessedSamplesRemain-=nextFieldDuration;
			//printf("desync, adding field, last location %d, remain %d, %s field\n", lastLongSyncPulseLocation, unprocessedSamplesRemain, nextFieldIsBottom ? "bottom" : "top");
			nextFieldIsBottom=!nextFieldIsBottom;
			if(nextFieldIsBottom)
				nextFieldDuration=DEFAULT_LINE_DURATION*313;
			else
				nextFieldDuration=DEFAULT_LINE_DURATION*312;
		}
		if(unprocessedSamplesRemain){
			int offset=BUFFER_SIZE-unprocessedSamplesRemain;
			currentField->appendSamples(b, offset, unprocessedSamplesRemain);

			for(int j=10;j<syncPulseLocations.size();j++){
				if(syncPulseLocations[j].location>=BUFFER_SIZE)
					break;
				if(syncPulseLocations[j].location>=lastLongSyncPulseLocation)
					currentFieldSyncPulses.push_back(syncPulseLocations[j].offset(-lastLongSyncPulseLocation));
			}
		}
		
		lastLongSyncPulseLocation-=BUFFER_SIZE;
		earliestNextLongSyncPulseLocation-=BUFFER_SIZE;
		
		//blockingSemaphore.Release();
		bufferCount++;
	}
	
	delete b;
	delete prevBuf;
	delete nextBuf;
	free(samples);
	free(nextSamples);
	for(VideoField* f:fieldQueue){
		delete f;
	}
	for(VideoField* f:fieldPool){
		delete f;
	}
}

VideoLine Decoder::joinLines(VideoLine a, VideoLine b){
	assert(a.luminance+a.numSamples==b.luminance);
	assert(a.chrominance[0]+a.numSamples==b.chrominance[0]);
	assert(a.filteredLuminance+a.numSamples==b.filteredLuminance);
	assert(a.raw+a.numSamples==b.raw);
	
	vector<SyncPulse> sync=a.sync;
	for(SyncPulse sp:b.sync){
		sync.push_back(sp.offset(a.numSamples));
	}
	//printf("join %d + %d -> %d\n", (int)a.luminance.size(), (int)b.luminance.size(), (int)luminance.size());
	return VideoLine(&a, 0, a.numSamples+b.numSamples, sync);
}

std::vector<VideoLine> Decoder::splitLine(VideoLine line, int numParts){
	float partLength=(float)line.numSamples/numParts;
	//printf("split into %d parts %f samples each\n", numParts, partLength);
	vector<VideoLine> parts;
	for(int i=0;i<numParts;i++){
		int offset=(int)(partLength*i);
		int length=i==numParts-1 ? (int)line.numSamples-offset : (int)partLength;
		assert(length>0);
		vector<SyncPulse> sync;
		for(SyncPulse sp:line.sync){
			if(sp.location>=offset && sp.location<offset+length)
				sync.push_back(sp);
		}
		parts.emplace_back(&line, offset, length, sync);
	}
	return parts;
}

vector<VideoLine> Decoder::processField(VideoField *field, std::vector<SyncPulse> sync, float syncLevel, float blackLevel, float visibleBrightnessRange, bool isBottom){
	SyncPulse nextSyncPulse=sync.size() ? sync[0] : SyncPulse{0, 0};
	int syncPulseIndex=0;
	VideoLine currentLine(field, 0, 0, {});
	vector<VideoLine> lines;
	int nextLineStartEarliestLocation=MIN_LINE_DURATION;
	float syncLevelSum=0, blackLevelSum=0;
	int syncLevelSampleCount=0, syncSamplesToAdd=0, blackLevelSampleCount=0, blackSamplesToAdd=0, blackSampleDelay=0;
	for(int i=0;i<field->numSamples;i++){
		currentLine.numSamples++;
		if(i==nextSyncPulse.location){
			if(nextSyncPulse.location>=nextLineStartEarliestLocation){
				lines.push_back(currentLine);
				currentLine=VideoLine(field, i+1, 0, {});
				nextLineStartEarliestLocation=nextSyncPulse.location+MIN_LINE_DURATION;
			}
			syncSamplesToAdd=nextSyncPulse.length;
			if(nextSyncPulse.length<=LINE_SYNC_MAX_DURATION){
				blackSamplesToAdd=40;
				blackSampleDelay=nextSyncPulse.length+15;
			}
			syncPulseIndex++;
			if(syncPulseIndex<sync.size())
				nextSyncPulse=sync[syncPulseIndex];
		}
		if(syncSamplesToAdd){
			syncLevelSum+=field->filteredLuminance[i];
			syncSamplesToAdd--;
			syncLevelSampleCount++;
		}
		if(blackSamplesToAdd){
			if(blackSampleDelay){
				blackSampleDelay--;
			}else{
				blackLevelSum+=field->filteredLuminance[i];
				blackSamplesToAdd--;
				blackLevelSampleCount++;
			}
		}
	}
	if(currentLine.numSamples>=MIN_LINE_DURATION){
		lines.push_back(currentLine);
	}
	
	int j=0;
	for(auto line=lines.begin();line!=lines.end() && j<313;line++, j++){
		if(line->numSamples>MAX_LINE_DURATION){
			//printf("line of length %d offset %d\n", (int)line->luminance.size(), line->offsetInBuffer);
			VideoLine joined=*line;
			vector<VideoLine>::iterator replaceBegin=line, replaceEnd=line+1;
			if(line!=lines.begin()){
				VideoLine prev=*(line-1);
				if(prev.numSamples>MAX_LINE_DURATION){
					joined=joinLines(prev, joined);
					replaceBegin=line-1;
				}
			}
			int numParts=(int)roundf(joined.numSamples/(float)DEFAULT_LINE_DURATION);
			if(numParts>1){
				vector<VideoLine> parts=splitLine(joined, numParts);
				line=lines.erase(replaceBegin, replaceEnd);
				line=lines.insert(line, parts.begin(), parts.end());
			}
		}
	}
	
	if(lines.size()>313){
		//printf("field too long with %d lines\n", (int)lines.size());
		lines.erase(lines.begin()+313, lines.end());
	}
	
	field->lines=lines;
	field->isBottom=isBottom;
	field->syncLevel=syncLevelSampleCount ? syncLevelSum/(float)syncLevelSampleCount : syncLevel;
	field->blackLevel=blackLevelSampleCount ? blackLevelSum/(float)blackLevelSampleCount : blackLevel;
	fieldQueue.push_back(field);
	
	if(fieldQueue.size()==4){
		bool allAreTop=true, allAreBottom=true;
		for(VideoField *field:fieldQueue){
			if(field->isBottom)
				allAreTop=false;
			else
				allAreBottom=false;
		}
		if(allAreTop){
			fieldQueue[1]->isBottom=true;
			fieldQueue[3]->isBottom=true;
		}else if(allAreBottom){
			fieldQueue[0]->isBottom=false;
			fieldQueue[2]->isBottom=false;
		}else if(fieldQueue[0]->isBottom==fieldQueue[1]->isBottom && fieldQueue[1]->isBottom==fieldQueue[2]->isBottom && fieldQueue[2]->isBottom!=fieldQueue[3]->isBottom){
			// top/top/top/bottom (or inverse) -- the 2nd field was misdetected
			fieldQueue[1]->isBottom=!fieldQueue[1]->isBottom;
		}
		VideoField *field=fieldQueue.front();
		fieldQueue.pop_front();
		float defaultWhiteLevel=field->blackLevel+std::min((field->blackLevel-field->syncLevel)/whiteLevelRatio, 0.99f-field->blackLevel);
		float whiteLevel=fieldsWithoutVITS>5 ? defaultWhiteLevel : detectedWhiteLevel;
		fieldsWithoutVITS++;
		// Try to sample white level from VITS signals transmitted by most channels
		int vitsLineIndex=field->isBottom ? 17 : 16;
		if(field->lines.size()>vitsLineIndex){
			float sumForWhiteLevel=0;
			int sampleCount=0;
			VideoLine line=field->lines[vitsLineIndex];
			for(int k=120;k<line.numSamples;k++){
				if(line.filteredLuminance[k]>defaultWhiteLevel){
					sumForWhiteLevel+=line.filteredLuminance[k];
					sampleCount++;
				}
			}
			if(sampleCount>5){
				detectedWhiteLevel=whiteLevel=sumForWhiteLevel/(float)sampleCount;
				fieldsWithoutVITS=0;
			}
		}
		float spikeThreshold=whiteLevel+(whiteLevel-field->blackLevel)*0.25f;

		// Precisely align lines relative to each other by offsetting and interpolating them such that the edges of the sync pulses either end of the line
		// end up at exact known X coordinates in the framebuffer
		float leadingThreshold=field->syncLevel+(field->blackLevel-field->syncLevel)*0.2f;
		float trailingThreshold=field->syncLevel+(field->blackLevel-field->syncLevel)*0.6f;
		float lineLeadingOffsets[field->lines.size()], lineTrailingOffsets[field->lines.size()], lineTrailingAlignPositions[field->lines.size()];
		int lineLeadingAlignDestinations[field->lines.size()];
		for(int j=0;j<field->lines.size();j++){
			int lineIndex=j+(field->isBottom ? 312 : 0);
			if(lineIndex<625){
				VideoLine& line=field->lines[j];
				int numSamples=line.numSamples;
				
				// Find and remove (interpolate over) spikes in the signal
				int spikeStartIndex=-1;
				for(int i=0;i<numSamples;i++){
					if(line.raw[i]>spikeThreshold){
						spikeStartIndex=i;
					}else if(spikeStartIndex!=-1){
						if(i-spikeStartIndex<10){
							int interpStart=std::max(0, spikeStartIndex-5);
							int interpEnd=std::min(numSamples, i+5);
							for(int j=interpStart;j<interpEnd;j++){
								float k=(j-interpStart)/(interpEnd-interpStart);
								line.luminance[j]=line.luminance[interpStart]*(1.0f-k)+line.luminance[interpEnd]*k;
								line.chrominance[0][j]=line.chrominance[0][interpStart]*(1.0f-k)+line.chrominance[0][interpEnd]*k;
								line.chrominance[1][j]=line.chrominance[1][interpStart]*(1.0f-k)+line.chrominance[1][interpEnd]*k;
							}
						}
						spikeStartIndex=-1;
					}
				}

				float leadingOffset=0;
				float trailingOffset=0;
				float leadingAlign;
				float trailingAlign;
				int leadingAlignDest, trailingAlignDest;
				if((lineIndex>4 && lineIndex<310) || (lineIndex>317 && lineIndex<623)){
					// These lines contain field sync pulses, which are shorter than normal line sync
					leadingAlignDest=LINE_LONG_SYNC_DURATION-LINE_SYNC_WINDOW+15;
				}else{
					leadingAlignDest=LINE_SYNC_DURATION-LINE_SYNC_WINDOW+15;
				}
				leadingAlign=numSamples*(leadingAlignDest/(float)DEFAULT_LINE_DURATION);
				trailingAlignDest=DEFAULT_LINE_DURATION-12;
				trailingAlign=numSamples*(trailingAlignDest/(float)DEFAULT_LINE_DURATION);
				
				// Find the rising edge of the leading sync pulse, going right to left
				//      _______
				// ____/ <----
				for(int i=leadingAlign+15;i>std::max(1, (int)leadingAlign-30);i--){
					float prevSample=line.luminance[i-1];
					float curSample=line.luminance[i];
					if(curSample>leadingThreshold && prevSample<=leadingThreshold){
						leadingOffset=i-leadingAlign+(leadingThreshold-prevSample)/(curSample-prevSample);
						break;
					}
				}
				
				// Find the falling edge of the trailing sync pulse, going left to right
				// _____
				// ---> \____
				for(int i=trailingAlign-5;i<numSamples;i++){
					float prevSample=line.luminance[i-1];
					float curSample=line.luminance[i];
					if(curSample<trailingThreshold && prevSample>=trailingThreshold){
						trailingOffset=i-trailingAlign+(trailingThreshold-prevSample)/(curSample-prevSample);
						break;
					}
				}
				
				lineLeadingOffsets[j]=leadingOffset;
				lineTrailingOffsets[j]=trailingOffset;
				lineTrailingAlignPositions[j]=trailingAlign;
				lineLeadingAlignDestinations[j]=leadingAlignDest;
			}
		}
		
		for(int j=0;j<field->lines.size();j++){
			int lineIndex=j+(field->isBottom ? 312 : 0);
			if(lineIndex<625){
				interpolateLine(field->lines[j], interpolatedField->lines[j], lineLeadingOffsets[j], lineTrailingOffsets[j], lineLeadingAlignDestinations[j], lineTrailingAlignPositions[j]);
			}
		}
		
		interpolatedField->isBottom=field->isBottom;
		interpolatedField->blackLevel=field->blackLevel;
		interpolatedField->syncLevel=field->syncLevel;
		colorDecoder->decodeColor(interpolatedField);
		
		for(int j=0;j<field->lines.size();j++){
			int lineIndex=j+(field->isBottom ? 312 : 0);
			if(lineIndex<625){
				processLine(interpolatedField->lines[j], lineIndex, field->syncLevel, field->blackLevel, whiteLevel);
				if(lineIndex==scopeLineIndex){
					scopeData1.clear();
					VideoLine &line=interpolatedField->lines[j];
					for(int i=0;i<DEFAULT_LINE_DURATION;i++){
						float v=line.luminance[i];
						scopeData1.push_back((float)v);
					}
					scopeData2.clear();
					for(int i=0;i<DEFAULT_LINE_DURATION;i++){
						float v=line.chrominance[0][i];
						scopeData2.push_back((float)v/2.0f+0.5f);
					}
					scopeLines.clear();
					scopeLines.push_back(syncLevel);
					scopeLines.push_back(blackLevel);
					scopeLines.push_back(whiteLevel);
				}
			}
		}

		if(vbiDataCallback && field->lines.size()>32){
			uint8_t vbiData[DEFAULT_LINE_DURATION*16];
			int offset=0;
			for(int i=0;i<16;i++){
				VideoLine line=field->lines[(field->isBottom ? 14 : 13)+i];
				int lineLength=std::min(line.numSamples, DEFAULT_LINE_DURATION);
				for(int j=0;j<lineLength;j++){
					vbiData[offset+j]=(uint8_t)roundf(std::clamp((line.raw[j]-blackLevel)/(whiteLevel-blackLevel)*0.8f+0.2f, 0.0f, 1.0f)*255.0f);
				}
				offset+=DEFAULT_LINE_DURATION;
			}
			vbiDataCallback(vbiData, DEFAULT_LINE_DURATION*16);
		}
		
		if(field->isBottom && outputEnabled){
			outputCapturedFrame();
		}
		CFRunLoopSourceSignal(runLoopSource);
		CFRunLoopWakeUp(mainThreadRunLoop);
	}

	return lines;
}

void Decoder::interpolateLine(VideoLine const& src, VideoLine const& dst, float leadingOffset, float trailingOffset, int leadingAlignDest, float trailingAlign){
	float interpolationIndexes[DEFAULT_LINE_DURATION];
	
	for(int x=0;x<DEFAULT_LINE_DURATION;x++){
		float sampleOffsetK=std::clamp((x-leadingAlignDest)/(float)trailingAlign, 0.0f, 1.0f);
		float sampleOffset=trailingOffset*sampleOffsetK+leadingOffset*(1.0f-sampleOffsetK);
		interpolationIndexes[x]=std::clamp(x/(float)DEFAULT_LINE_DURATION*src.numSamples+sampleOffset, 0.0f, (float)src.numSamples-1);
	}
	vDSP_vlint(src.luminance, interpolationIndexes, 1, dst.luminance, 1, DEFAULT_LINE_DURATION, src.numSamples);
	vDSP_vlint(src.chrominance[0], interpolationIndexes, 1, dst.chrominance[0], 1, DEFAULT_LINE_DURATION, src.numSamples);
	vDSP_vlint(src.chrominance[1], interpolationIndexes, 1, dst.chrominance[1], 1, DEFAULT_LINE_DURATION, src.numSamples);
	if(colorMode==ColorDisplayMode::Raw){
		vDSP_vlint(src.raw, interpolationIndexes, 1, dst.raw, 1, DEFAULT_LINE_DURATION, src.numSamples);
	}
}

void Decoder::processLine(VideoLine line, int lineIndex, float syncLevel, float blackLevel, float whiteLevel){
	uint32_t* bitmapPixels=(uint32_t*)bitmapData;
	float visibleBrightnessRange=whiteLevel-blackLevel;
	
	uint32_t* bitmapLine;
	if(displayFieldsSequentially)
		bitmapLine=bitmapPixels+((lineIndex)*bitmapStride);
	else
		bitmapLine=bitmapPixels+((lineIndex*2%bitmapHeight)*bitmapStride);

	float* samplesForDisplay;
	switch(colorMode){
		case ColorDisplayMode::Full:
			samplesForDisplay=line.luminance;
			break;
		case ColorDisplayMode::Raw:
			samplesForDisplay=line.raw;
			break;
		case ColorDisplayMode::Y:
			samplesForDisplay=line.luminance;
			break;
		case ColorDisplayMode::Db:
			samplesForDisplay=line.chrominance[0];
			break;
		case ColorDisplayMode::Dr:
			samplesForDisplay=line.chrominance[1];
			break;
	}
	float* cbSamples=line.chrominance[0];
	float* crSamples=line.chrominance[1];

	for(int x=0;x<bitmapWidth;x++){
		float sampleIndex=std::clamp(x/(float)bitmapWidth*DEFAULT_LINE_DURATION, 0.0f, (float)DEFAULT_LINE_DURATION-2);
		float sample1=samplesForDisplay[((int)sampleIndex)];
		float sample2=samplesForDisplay[(int)sampleIndex+1];
		float k=sampleIndex-(int)sampleIndex;
		float interpolatedSample=sample2*k+sample1*(1.0f-k);
		
		if(colorMode==ColorDisplayMode::Full){
			float y=(interpolatedSample-blackLevel)/visibleBrightnessRange;
			float db1=cbSamples[(int)sampleIndex], db2=cbSamples[(int)sampleIndex+1];
			float db=db2*k+db1*(1.0f-k);
			float dr1=crSamples[(int)sampleIndex], dr2=crSamples[(int)sampleIndex+1];
			float dr=dr2*k+dr1*(1.0f-k);
			
			// http://avisynth.nl/index.php/Colorimetry
			const float kr=0.299, kg=0.587, kb=0.114;
			
			float r=std::clamp(y+dr*(1.0f-kr), 0.0f, 1.0f);
			float g=std::clamp(y-db*(1.0f-kb)*kb/kg-dr*(1.0f-kr)*kr/kg, 0.0f, 1.0f);
			float b=std::clamp(y+db*(1.0f-kb), 0.0f, 1.0f);

			int ri=std::round(r*255);
			int gi=std::round(g*255);
			int bi=std::round(b*255);
			bitmapLine[x]=ri | (gi << 8) | (bi << 16) | 0xff000000;
		}else{
			int pixelColor;
			
			if(colorMode==ColorDisplayMode::Dr || colorMode==ColorDisplayMode::Db){
				interpolatedSample=(interpolatedSample+1.0f)/2.0f;
				pixelColor=(int)(std::clamp(interpolatedSample, 0.0f, 1.0f)*255);
			}else{
				switch(levelsMode){
					case DisplayLevelsMode::Auto:
						pixelColor=(int)(std::clamp((interpolatedSample-blackLevel)/visibleBrightnessRange, 0.0f, 1.0f)*255);
						break;
					case DisplayLevelsMode::BlackIsSync:
						pixelColor=(int)(std::clamp((interpolatedSample-syncLevel)/(1.0f-syncLevel), 0.0f, 1.0f)*255);
						break;
					case DisplayLevelsMode::Raw:
						pixelColor=(int)(std::clamp(interpolatedSample, 0.0f, 1.0f)*255);
						break;
				}
			}
			bitmapLine[x]=pixelColor | (pixelColor << 8) | (pixelColor << 16) | 0xff000000;
		}
	}
	
	if(outputEnabled){
		int lineStartOffset=includeBlankingIntervalsInOutput ? 0 : 197;
		int lineLength=includeBlankingIntervalsInOutput ? 1280 : 1043;
		int outputWidth, outputHeight, yOffset;
		if(includeBlankingIntervalsInOutput){
			outputWidth=942;
			outputHeight=625;
			yOffset=0;
		}else{
			outputWidth=768;
			outputHeight=576;
			yOffset=44;
		}
		int y=lineIndex*2%625-yOffset;
		if(y>=0 && y<outputHeight){
			uint16_t *outputLineY=((uint16_t*)*currentOutputBuffer)+y*outputWidth;
			uint16_t *outputLineUV=((uint16_t*)*currentOutputBuffer)+outputWidth*outputHeight+y*outputWidth*2;
			for(int x=0;x<outputWidth;x++){
				float sampleIndex=std::clamp(x/(float)outputWidth*lineLength, 0.0f, (float)lineLength-2)+lineStartOffset;
				float sample1=line.luminance[((int)sampleIndex)];
				float sample2=line.luminance[(int)sampleIndex+1];
				float k=sampleIndex-(int)sampleIndex;
				float interpolatedSample=sample2*k+sample1*(1.0f-k);
				float y=(interpolatedSample-blackLevel)/visibleBrightnessRange;
				float db1=cbSamples[(int)sampleIndex], db2=cbSamples[(int)sampleIndex+1];
				float db=db2*k+db1*(1.0f-k);
				float dr1=crSamples[(int)sampleIndex], dr2=crSamples[(int)sampleIndex+1];
				float dr=dr2*k+dr1*(1.0f-k);
				
				outputLineY[x]=(int)(std::clamp(y, 0.0f, 1.0f)*65535);
				outputLineUV[x*2]=(int)(std::clamp((db+1)/2, 0.0f, 1.0f)*65535);
				outputLineUV[x*2+1]=(int)(std::clamp((dr+1)/2, 0.0f, 1.0f)*65535);
			}
		}
	}
}

tgvoip::Buffer Decoder::getOutputFrame(){
	return outputQueue.GetBlocking();
}

void Decoder::outputCapturedFrame(){
	outputQueue.Put(std::move(currentOutputBuffer));
	currentOutputBuffer=outputBufferPool.Get();
}

void Decoder::startOutput(){
	outputEnabled=true;
}

void Decoder::stopOutput(){
	outputEnabled=false;
	outputQueue.Put(tgvoip::Buffer(0));
}

void Decoder::replaceColorDecoder(ColorDecoder *newColorDecoder){
	tgvoip::MutexGuard mutexGuard(colorDecoderMutex);
	delete colorDecoder;
	colorDecoder=newColorDecoder;
}

Decoder::ColorDecoder::ColorDecoder(){
	for(int i=0;i<2048;i++){
		float angle=M_PI*2/2048.0f*i;
		sinLUT[i]=sin(angle);
		cosLUT[i]=cos(angle);
	}
}

#pragma mark - SECAM color decoder

/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 20000000 Hz

* 0 Hz - 400000 Hz
  gain = 1
  desired ripple = 0.01 dB
  actual ripple = 0.006705300620706349 dB

* 800000 Hz - 10000000 Hz
  gain = 0
  desired attenuation = -60 dB
  actual attenuation = -60.80818509918938 dB

*/
Decoder::ColorDecoderSECAM::ColorDecoderSECAM():chromaSeparationFilter({
	-0.0005289404960301208, -0.00015194834021620932, -0.00016064522389667017, -0.00015947495667639554, -0.00014669995651741624, -0.00012100545070936878, -0.00008159696499506645, -0.000028353048490032713, 0.00003807818829524216, 0.00011616362724136156,
	0.00020356822702490105, 0.0002969805677478761, 0.0003923180517311467, 0.00048480424126498876, 0.0005690686005356404, 0.0006394016653697397, 0.0006900439838754251, 0.0007154047124949425, 0.0007103942686820328, 0.0006708657471036658,
	0.0005937179282194615, 0.0004774014753530968, 0.0003220616805610796, 0.00012978304776424675, -0.00009529319955139381, -0.000347002181274705, -0.0006165928376910925, -0.0008952706930790662, -0.0011689074582295832, -0.0014266598276419296,
	-0.0016541422161819152, -0.0018367844723349738, -0.001961205662258954, -0.0020152456751155803, -0.0019882646426562533, -0.0018718810990902134, -0.0016608497534930965, -0.0013536235139110341, -0.0009529178011047729, -0.0004658200324138174,
	0.00009614702820258965, 0.0007170064586302902, 0.001376544335369247, 0.002050812291303941, 0.0027126945652568156, 0.003332719653891382, 0.003880318389978319, 0.004324726283844566, 0.004636521425314842, 0.004788827440094886,
	0.004758694431714366, 0.004528386151088643, 0.004086513525800587, 0.0034294152578191565, 0.002561181200092824, 0.0014957698045580292, 0.0002554307623225337, -0.0011278960314228127, -0.002613336584587735, -0.004152191520029391,
	-0.0056887425474737964, -0.007161539479185438, -0.008505233264615036, -0.00965264229530876, -0.010536749735523446, -0.011093039545439783, -0.011261724116637147, -0.010990019468917691, -0.010234408476711941, -0.008962638216015534,
	-0.007155233541660424, -0.004806880135395776, -0.0019275395965021108, 0.0014574543264773562, 0.005306986326917668, 0.009565026481345837, 0.01416150910756365, 0.019013903025916226, 0.024029172362971148, 0.029106073097884537,
	0.03413804779435603, 0.03901566070485198, 0.04363037952714531, 0.047876918478203176, 0.05165678439973822, 0.054881156429649615, 0.05747329400145506, 0.05937093858746132, 0.060528263037262595, 0.06091720281464091,
	0.060528263037262595, 0.05937093858746132, 0.05747329400145506, 0.054881156429649615, 0.05165678439973822, 0.047876918478203176, 0.04363037952714531, 0.03901566070485198, 0.03413804779435603, 0.029106073097884537,
	0.024029172362971148, 0.019013903025916226, 0.01416150910756365, 0.009565026481345837, 0.005306986326917668, 0.0014574543264773562, -0.0019275395965021108, -0.004806880135395776, -0.007155233541660424, -0.008962638216015534,
	-0.010234408476711941, -0.010990019468917691, -0.011261724116637147, -0.011093039545439783, -0.010536749735523446, -0.00965264229530876, -0.008505233264615036, -0.007161539479185438, -0.0056887425474737964, -0.004152191520029391,
	-0.002613336584587735, -0.0011278960314228127, 0.0002554307623225337, 0.0014957698045580292, 0.002561181200092824, 0.0034294152578191565, 0.004086513525800587, 0.004528386151088643, 0.004758694431714366, 0.004788827440094886,
	0.004636521425314842, 0.004324726283844566, 0.003880318389978319, 0.003332719653891382, 0.0027126945652568156, 0.002050812291303941, 0.001376544335369247, 0.0007170064586302902, 0.00009614702820258965, -0.0004658200324138174,
	-0.0009529178011047729, -0.0013536235139110341, -0.0016608497534930965, -0.0018718810990902134, -0.0019882646426562533, -0.0020152456751155803, -0.001961205662258954, -0.0018367844723349738, -0.0016541422161819152, -0.0014266598276419296,
	-0.0011689074582295832, -0.0008952706930790662, -0.0006165928376910925, -0.000347002181274705, -0.00009529319955139381, 0.00012978304776424675, 0.0003220616805610796, 0.0004774014753530968, 0.0005937179282194615, 0.0006708657471036658,
	0.0007103942686820328, 0.0007154047124949425, 0.0006900439838754251, 0.0006394016653697397, 0.0005690686005356404, 0.00048480424126498876, 0.0003923180517311467, 0.0002969805677478761, 0.00020356822702490105, 0.00011616362724136156,
	0.00003807818829524216, -0.000028353048490032713, -0.00008159696499506645, -0.00012100545070936878, -0.00014669995651741624, -0.00015947495667639554, -0.00016064522389667017, -0.00015194834021620932, -0.0005289404960301208
}),
chromaDeemphasisFilter(-0.9672101283716882, 0, 0.016394935814155933, 0.016394935814155933, 0)
{
	const int filterSize=chromaSeparationFilter.getSize();
	rawISamples=(float*)calloc(BUFFER_SIZE+filterSize*2, sizeof(float));
	rawQSamples=(float*)calloc(BUFFER_SIZE+filterSize*2, sizeof(float));
	prevFieldChrominance[0]=(float*)calloc(DEFAULT_LINE_DURATION*625, sizeof(float));
	prevFieldChrominance[1]=(float*)calloc(DEFAULT_LINE_DURATION*625, sizeof(float));
	angleIndex=0;
}

Decoder::ColorDecoderSECAM::~ColorDecoderSECAM(){
	free(rawISamples);
	free(rawQSamples);
	free(prevFieldChrominance[0]);
	free(prevFieldChrominance[1]);
}
	
void Decoder::ColorDecoderSECAM::separateSubcarrier(SignalBuffers *buf, float *nextBuffer){
	const int filterDelay=chromaSeparationFilter.getDelay();
	// Copy the tail of the previous buffer to the beginning of the current one
	memcpy(rawISamples, rawISamples+BUFFER_SIZE, filterDelay*sizeof(float));
	memcpy(rawQSamples, rawQSamples+BUFFER_SIZE, filterDelay*sizeof(float));
	unsigned int angleIndex=this->angleIndex;
	unsigned int initAngleIndex=this->angleIndex;
	for(int i=0;i<BUFFER_SIZE;i++){
		float angle=intermediateFreq*angleIndex/20000000.0*2048;
		int lutIndex=(int)angle%2048;
		rawISamples[i+filterDelay]=buf->raw[i]*sinLUT[lutIndex];
		rawQSamples[i+filterDelay]=buf->raw[i]*cosLUT[lutIndex];
		angleIndex++;
	}
	angleIndex%=277*1280;
	this->angleIndex=angleIndex;
	// +1 is for the demodulation stage because it needs to look ahead one sample
	for(int i=0;i<filterDelay+1;i++){
		float angle=intermediateFreq*angleIndex/20000000.0*2048;
		int lutIndex=(int)angle%2048;
		rawISamples[i+filterDelay+BUFFER_SIZE]=nextBuffer[i]*sinLUT[lutIndex];
		rawQSamples[i+filterDelay+BUFFER_SIZE]=nextBuffer[i]*cosLUT[lutIndex];
		angleIndex++;
	}
	
	chromaSeparationFilter.process(rawISamples, buf->chrominance[0], BUFFER_SIZE+1);
	chromaSeparationFilter.process(rawQSamples, buf->chrominance[1], BUFFER_SIZE+1);
	
	for(int i=0;i<BUFFER_SIZE;i++){
		float angle=intermediateFreq*(initAngleIndex+i)/20000000.0*2048;
		int lutIndex=(int)angle%2048;
		float carrier=buf->chrominance[0][i]*sinLUT[lutIndex]*2+buf->chrominance[1][i]*cosLUT[lutIndex]*2;
		buf->luminance[i]=buf->raw[i]-carrier;
	}
}

void Decoder::ColorDecoderSECAM::demodulateSubcarrier(SignalBuffers *buf){
	float prevI=this->prevI, prevQ=this->prevQ;
	for(int i=0;i<BUFFER_SIZE;i++){
		float iSample=buf->chrominance[0][i];
		float qSample=buf->chrominance[1][i];
		float nextI=buf->chrominance[0][i+1];
		float nextQ=buf->chrominance[1][i+1];
		
		float freq=(iSample*(nextQ-prevQ)-qSample*(nextI-prevI))/(iSample*iSample+qSample*qSample);
		freq*=20000000.0f/(4*M_PI);
		buf->chrominance[0][i]=std::clamp(freq+intermediateFreq, 3000000.0f, 5000000.0f);
		buf->chrominance[1][i]=hypotf(iSample, qSample);
		prevI=iSample;
		prevQ=qSample;
	}
	this->prevI=prevI;
	this->prevQ=prevQ;
	chromaDeemphasisFilter.process(buf->chrominance[0], buf->chrominance[0], BUFFER_SIZE);
}

void Decoder::ColorDecoderSECAM::decodeColor(VideoField *field){
	float subcarrierAmplitudeThreshold=(field->blackLevel-field->syncLevel)/0.43f*0.03f;
	int wrongLineCount=0;
	int syncLineCount=0;
	// Look at the "green lines" aka "green bottles" (Soviet literature calls them just "color sync signals")
	// in the VBI to determine which lines carry R-Y and which B-Y
	for(int i=(field->isBottom ? 7 : 6);i<(field->isBottom ? 15 : 16);i++){
		VideoLine &line=field->lines[i];
		int lineIndex=i+(field->isBottom ? 312 : 0);
		bool isRedLine=(lineIndex+colorLineOffset)%2==0;
		float centerSum=0;
		float min=10;
		float max=-10;
		for(int j=150;j<250;j++){
			centerSum+=line.chrominance[0][j];
			min=std::min(line.chrominance[0][j], min);
			max=std::max(line.chrominance[0][j], max);
		}
		float centerFreq=centerSum/100.0;
		float maxSum=0;
		for(int j=1000;j<1200;j++){
			maxSum+=line.chrominance[0][j];
		}
		float maxFreq=maxSum/200.0;
		float freqDiff=centerFreq-maxFreq;
		if(centerFreq>3500000 && centerFreq<4500000 && fabsf(freqDiff)>270000.0f){
			syncLineCount++;
			bool isActuallyRedLine=maxFreq>centerFreq;
			if(isActuallyRedLine!=isRedLine){
				wrongLineCount++;
			}
		}
	}
	
	if(syncLineCount<2){ // There were no green lines or they were all too garbled
		// So let's instead analyze some of the top active lines, since they *should* have some undeviated carrier in the front porch
		int start=field->isBottom ? 27 : 26;
		int end=field->isBottom ? 35 : 34;
		for(int i=start;i<end;i++){
			float avg=0;
			for(int j=150;j<190;j++){
				avg+=field->lines[i].chrominance[0][j];
			}
			avg/=40.0f;
			int lineIndex=i+(field->isBottom ? 312 : 0);
			bool isRedLine=(lineIndex+colorLineOffset)%2==0;
			bool isActuallyRedLine=avg>intermediateFreq;
			if(isRedLine!=isActuallyRedLine)
				wrongLineCount++;
		}
	}
	
	if(wrongLineCount>=2){ // Our current state is wrong, flip it
		colorLineOffset=colorLineOffset==1 ? 0 : 1;
	}
	
	for(int i=0;i<(field->isBottom ? 313 : 312);i++){
		int lineIndex=i+(field->isBottom ? 312 : 0);
		
		VideoLine &line=field->lines[i];
		bool isRedLine=(lineIndex+colorLineOffset)%2==0;
		if((lineIndex>=6 && lineIndex<15) || (lineIndex>=319 && lineIndex<328)){
			float centerSum=0;
			float min=10;
			float max=-10;
			for(int j=150;j<250;j++){
				centerSum+=line.chrominance[0][j];
				min=std::min(line.chrominance[0][j], min);
				max=std::max(line.chrominance[0][j], max);
			}
			float centerFreq=centerSum/100.0;
			float maxSum=0;
			for(int j=1000;j<1200;j++){
				maxSum+=line.chrominance[0][j];
			}
			float maxFreq=maxSum/200.0;
			float freqDiff=centerFreq-maxFreq;
			if(centerFreq>3500000 && centerFreq<4500000 && fabsf(freqDiff)>270000.0f){
				bool isActuallyRedLine=maxFreq>centerFreq;
				if(isActuallyRedLine!=isRedLine){
					colorLineOffset=colorLineOffset==1 ? 0 : 1;
					isRedLine=isActuallyRedLine;
				}
			}
		}
		float centerFreq=isRedLine ? redCenterFreq : blueCenterFreq;
		float maxDeviation=isRedLine ? redMaxDeviation : blueMaxDeviation;
		int chrominanceIndex=isRedLine ? 1 : 0;
		int prevLineChrominanceIndex=isRedLine ? 0 : 1;
		float coeff=isRedLine ? -1.902f : 1.505f;
		if(colorArtifactFilterEnabled){
			int remainingBadChromaSamples=0;
			for(int j=0;j<DEFAULT_LINE_DURATION;j++){
				if(line.chrominance[1][j]<subcarrierAmplitudeThreshold && j>=197 && j<DEFAULT_LINE_DURATION-40 && i>=22 && i<310){
					float fromPrevField=prevFieldChrominance[chrominanceIndex][DEFAULT_LINE_DURATION*(lineIndex+(frameCount%2==1 ? 0 : 1))+j];
					float fromPrevLine=field->lines[i-1].chrominance[chrominanceIndex][j];
					line.chrominance[chrominanceIndex][j]=(fromPrevField+fromPrevLine)/2.0f;
					remainingBadChromaSamples=50;
				}else if(remainingBadChromaSamples>0){
					remainingBadChromaSamples--;
					//line.chrominance[chrominanceIndex][j]=prevFieldChrominance[chrominanceIndex][DEFAULT_LINE_DURATION*(lineIndex+(frameCount%2==1 ? 0 : 1))+j];
					float fromPrevField=prevFieldChrominance[chrominanceIndex][DEFAULT_LINE_DURATION*(lineIndex+(frameCount%2==1 ? 0 : 1))+j];
					float fromPrevLine=field->lines[i-1].chrominance[chrominanceIndex][j];
					line.chrominance[chrominanceIndex][j]=(fromPrevField+fromPrevLine)/2.0f;
				}else{
					line.chrominance[chrominanceIndex][j]=std::clamp((line.chrominance[0][j]-centerFreq)/maxDeviation/coeff, -1.0f, 1.0f);
				}
			}
		}else{
			for(int j=0;j<DEFAULT_LINE_DURATION;j++){
				line.chrominance[chrominanceIndex][j]=std::clamp((line.chrominance[0][j]-centerFreq)/maxDeviation/coeff, -1.0f, 1.0f);
			}
		}
		if(i>0){
			memcpy(line.chrominance[prevLineChrominanceIndex], field->lines[i-1].chrominance[prevLineChrominanceIndex], DEFAULT_LINE_DURATION*sizeof(float));
		}
	}
	if(colorArtifactFilterEnabled){
		int lineCount=field->isBottom ? 313 : 312;
		int offset=field->isBottom ? 312 : 0;
		memcpy(prevFieldChrominance[0]+offset*DEFAULT_LINE_DURATION, field->chrominance[0], DEFAULT_LINE_DURATION*lineCount*sizeof(float));
		memcpy(prevFieldChrominance[1]+offset*DEFAULT_LINE_DURATION, field->chrominance[1], DEFAULT_LINE_DURATION*lineCount*sizeof(float));
	}
	if(field->isBottom){
		frameCount++;
		colorLineOffset=colorLineOffset==1 ? 0 : 1;
	}
}

#pragma mark - PAL color decoder

/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 20000000 Hz

* 0 Hz - 3430000 Hz
  gain = 1
  desired ripple = 0.1 dB
  actual ripple = 0.07209347141167058 dB

* 3630000 Hz - 5230000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = -40.19524184473034 dB

* 5430000 Hz - 10000000 Hz
  gain = 1
  desired ripple = 0.1 dB
  actual ripple = 0.07209347141167058 dB

*/
Decoder::ColorDecoderPAL::ColorDecoderPAL():chromaNotchFilter({
	-0.0013223890092425744, 0.002169197739003572, 0.0020713708101192177, -0.0002670120203169399, -0.001669488756435616, -0.000045479393273382667, 0.0014523631547638817, 0.0004674708436634332, -0.0005809180591022729, -0.00013910343694471084,
	-0.0001914360971700373, -0.0007906234887587319, 0.0003551152418230627, 0.0017804407408220295, 0.00019599213127478117, -0.002136154145467837, -0.0009509208519788754, 0.0015833933557721119, 0.0011247202872493546, -0.000552260992588332,
	-0.00028495848416094466, -0.00009265729722095198, -0.0012244051924595852, -0.00026724096774202017, 0.0024575350975470553, 0.001439529727985197, -0.002569175639335625, -0.0024552113411834262, 0.0015422151198342753, 0.0022800940910669126,
	-0.00030728499725103957, -0.0006824414160277245, 0.0000761272936366316, -0.001454604368204993, -0.001341503247088603, 0.00271100626318093, 0.003339884493485329, -0.002264404116655807, -0.004512757832264982, 0.0006356113041455242,
	0.0037095497961184554, 0.0005587789100787759, -0.0012001801617747964, 0.00013778946647017323, -0.0013634433352795292, -0.0027637447369171036, 0.0021531001954769803, 0.005693338973660847, -0.0006823771161729551, -0.0067642108614997705,
	-0.001615337036773915, 0.00500410422270167, 0.002353201248848628, -0.0015878195944386908, -0.00009243852837451847, -0.0008870218490090111, -0.0043029098367272, 0.00040849548571992574, 0.008061676045785313, 0.0026722219134499335,
	-0.008553179859861973, -0.005622507317312688, 0.00547928753525117, 0.005306864958930067, -0.0014019103304672275, -0.0007729180102652681, -0.00005493850034653831, -0.0056367690584228245, -0.0028664607545010575, 0.00977640848979656,
	0.008297906405354083, -0.00888825110676341, -0.011775702385540843, 0.004077264791942513, 0.009591834720016859, 0.00010665335771685467, -0.001998624596475931, 0.0009839655848009239, -0.006417729413350083, -0.008155396124933902,
	0.009907806437818294, 0.01703878447579458, -0.006180818767615026, -0.02084438274107957, -0.0011166686183928058, 0.015574835693370773, 0.00444573616305343, -0.00380595583420094, 0.0019971884880576017, -0.006291161785837853,
	-0.01704980316831708, 0.006723472808413564, 0.03206044317749449, 0.003547662727655479, -0.03634916879227786, -0.016020523302918945, 0.02547759931965321, 0.01706492681700458, -0.006472224954168771, 0.0027341384747225114,
	-0.004481440660600159, -0.04035121645593691, -0.008512252800423345, 0.07931612562378414, 0.050027941035901316, -0.09771779171349738, -0.10806359491773884, 0.08096150855606563, 0.15873490162338408, -0.03142221692555693,
	0.8211858937594392, -0.03142221692555693, 0.15873490162338408, 0.08096150855606563, -0.10806359491773884, -0.09771779171349738, 0.050027941035901316, 0.07931612562378414, -0.008512252800423345, -0.04035121645593691,
	-0.004481440660600159, 0.0027341384747225114, -0.006472224954168771, 0.01706492681700458, 0.02547759931965321, -0.016020523302918945, -0.03634916879227786, 0.003547662727655479, 0.03206044317749449, 0.006723472808413564,
	-0.01704980316831708, -0.006291161785837853, 0.0019971884880576017, -0.00380595583420094, 0.00444573616305343, 0.015574835693370773, -0.0011166686183928058, -0.02084438274107957, -0.006180818767615026, 0.01703878447579458,
	0.009907806437818294, -0.008155396124933902, -0.006417729413350083, 0.0009839655848009239, -0.001998624596475931, 0.00010665335771685467, 0.009591834720016859, 0.004077264791942513, -0.011775702385540843, -0.00888825110676341,
	0.008297906405354083, 0.00977640848979656, -0.0028664607545010575, -0.0056367690584228245, -0.00005493850034653831, -0.0007729180102652681, -0.0014019103304672275, 0.005306864958930067, 0.00547928753525117, -0.005622507317312688,
	-0.008553179859861973, 0.0026722219134499335, 0.008061676045785313, 0.00040849548571992574, -0.0043029098367272, -0.0008870218490090111, -0.00009243852837451847, -0.0015878195944386908, 0.002353201248848628, 0.00500410422270167,
	-0.001615337036773915, -0.0067642108614997705, -0.0006823771161729551, 0.005693338973660847, 0.0021531001954769803, -0.0027637447369171036, -0.0013634433352795292, 0.00013778946647017323, -0.0012001801617747964, 0.0005587789100787759,
	0.0037095497961184554, 0.0006356113041455242, -0.004512757832264982, -0.002264404116655807, 0.003339884493485329, 0.00271100626318093, -0.001341503247088603, -0.001454604368204993, 0.0000761272936366316, -0.0006824414160277245,
	-0.00030728499725103957, 0.0022800940910669126, 0.0015422151198342753, -0.0024552113411834262, -0.002569175639335625, 0.001439529727985197, 0.0024575350975470553, -0.00026724096774202017, -0.0012244051924595852, -0.00009265729722095198,
	-0.00028495848416094466, -0.000552260992588332, 0.0011247202872493546, 0.0015833933557721119, -0.0009509208519788754, -0.002136154145467837, 0.00019599213127478117, 0.0017804407408220295, 0.0003551152418230627, -0.0007906234887587319,
	-0.0001914360971700373, -0.00013910343694471084, -0.0005809180591022729, 0.0004674708436634332, 0.0014523631547638817, -0.000045479393273382667, -0.001669488756435616, -0.0002670120203169399, 0.0020713708101192177, 0.002169197739003572,
	-0.0013223890092425744
}),
verticalDiffLowpass(-1.34891824, 0.51392633, 0.04125202, 0.08250405, 0.04125202),
ycDiffLowpass(-1.34891824, 0.51392633, 0.04125202, 0.08250405, 0.04125202),
/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 20000000 Hz

* 0 Hz - 1300000 Hz
  gain = 1
  desired ripple = 0.1 dB
  actual ripple = 0.06330149551265106 dB

* 2000000 Hz - 10000000 Hz
  gain = 0
  desired attenuation = -60 dB
  actual attenuation = -61.324869774648064 dB

*/
chromaLowpassFilter({
	0.0007531764721153182, 0.0007519874408971961, 0.0008729569988541416, 0.0007663799237776646, 0.00036879314744475765, -0.0003049347095003807, -0.0011395378732079375, -0.0019271769896894693, -0.0024080972682461455, -0.00234250098284755,
	-0.0015943264953982939, -0.00020366148820846527, 0.0015789910528071964, 0.0033175633133526667, 0.004480905777870153, 0.004583816854226088, 0.0033463363899978674, 0.000825704498443918, -0.0025257417852139587, -0.0059096969960359975,
	-0.008344496945319678, -0.008915713811573618, -0.007057813193061558, -0.002789953297410818, 0.003170813418377668, 0.009473103019323201, 0.014385341068687055, 0.01620427188452137, 0.013732813449343097, 0.006710296549959943,
	-0.003926015644950164, -0.016040482243346765, -0.02659669821484103, -0.03221921797003905, -0.029917076727535838, -0.017812841202317516, 0.0042947916063085885, 0.034679307991646695, 0.06982680636866369, 0.10497164259121904,
	0.13492869210263547, 0.1550564948941928, 0.16214608317804943, 0.1550564948941928, 0.13492869210263547, 0.10497164259121904, 0.06982680636866369, 0.034679307991646695, 0.0042947916063085885, -0.017812841202317516,
	-0.029917076727535838, -0.03221921797003905, -0.02659669821484103, -0.016040482243346765, -0.003926015644950164, 0.006710296549959943, 0.013732813449343097, 0.01620427188452137, 0.014385341068687055, 0.009473103019323201,
	0.003170813418377668, -0.002789953297410818, -0.007057813193061558, -0.008915713811573618, -0.008344496945319678, -0.0059096969960359975, -0.0025257417852139587, 0.000825704498443918, 0.0033463363899978674, 0.004583816854226088,
	0.004480905777870153, 0.0033175633133526667, 0.0015789910528071964, -0.00020366148820846527, -0.0015943264953982939, -0.00234250098284755, -0.0024080972682461455, -0.0019271769896894693, -0.0011395378732079375, -0.0003049347095003807,
	0.00036879314744475765, 0.0007663799237776646, 0.0008729569988541416, 0.0007519874408971961, 0.0007531764721153182
}){
	const int filterSize=chromaNotchFilter.getSize();
	rawI=(float*)calloc(BUFFER_SIZE+filterSize*2, sizeof(float));
	rawQ=(float*)calloc(BUFFER_SIZE+filterSize, sizeof(float));
	prevSubcarrier=(float*)calloc(DEFAULT_LINE_DURATION*2, sizeof(float));
	
	yNotch=(float*)calloc(BUFFER_SIZE+filterSize*2+DEFAULT_LINE_DURATION*4, sizeof(float));
	yNotchRaw=(float*)calloc(BUFFER_SIZE+filterSize*2+DEFAULT_LINE_DURATION*4, sizeof(float));
	yComb=(float*)calloc(BUFFER_SIZE+filterSize*2, sizeof(float));
	cComb=(float*)calloc(BUFFER_SIZE+filterSize*2, sizeof(float));
	verticalDiff=(float*)calloc(BUFFER_SIZE+filterSize*2, sizeof(float));
	ycDiff=(float*)calloc(BUFFER_SIZE+filterSize*2, sizeof(float));
}

Decoder::ColorDecoderPAL::~ColorDecoderPAL(){
	free(rawI);
	free(rawQ);
	free(prevSubcarrier);
	
	free(yNotch);
	free(yNotchRaw);
	free(yComb);
	free(cComb);
	free(verticalDiff);
	free(ycDiff);
}

void Decoder::ColorDecoderPAL::separateSubcarrier(SignalBuffers *buf, float *nextBuffer){
	const int filterDelay=chromaNotchFilter.getDelay();
	memcpy(yNotchRaw, yNotchRaw+BUFFER_SIZE, (filterDelay+DEFAULT_LINE_DURATION*2)*sizeof(float));
	memcpy(yNotchRaw+filterDelay+DEFAULT_LINE_DURATION*2, buf->raw, BUFFER_SIZE*sizeof(float));
	memcpy(yNotchRaw+filterDelay+BUFFER_SIZE+DEFAULT_LINE_DURATION*2, nextBuffer, (filterDelay+DEFAULT_LINE_DURATION*2)*sizeof(float));
	
	chromaNotchFilter.process(yNotchRaw, yNotch, BUFFER_SIZE+DEFAULT_LINE_DURATION*4);
	// At this point, yNotch contains: 2 lines worth of previous buffer, then buf->raw, then 2 lines worth of next buffer
	
	for(int i=0;i<BUFFER_SIZE;i++){
		float s=buf->raw[i];
		// yNotchRaw contains buf->raw delayed by 2 lines with delay padded with previous and next buffers
		// (this makes it easier by not needing another loop to process this tail)
		float delayed=yNotchRaw[i+filterDelay];
		yComb[i]=(s+delayed)/2.0f;
		cComb[i]=(s-delayed)/2.0f;
		verticalDiff[i]=fabsf(yNotch[i+DEFAULT_LINE_DURATION*2]-yNotch[i]);
		ycDiff[i]=fabsf(yNotch[i+DEFAULT_LINE_DURATION*2]-s);
	}
	
	verticalDiffLowpass.process(verticalDiff, verticalDiff, BUFFER_SIZE);
	ycDiffLowpass.process(ycDiff, ycDiff, BUFFER_SIZE);
	
	const float vertLow=0.015f;
	const float vertHigh=0.05f;
	const float ycLow=0.005f;
	const float ycHigh=0.01f;
	for(int i=0;i<BUFFER_SIZE;i++){
		float k=std::clamp((verticalDiff[std::min(i+1, BUFFER_SIZE-1)]-vertLow)/(vertHigh-vertLow), 0.0f, 1.0f);
		float filtered=(1.0f-k)*yComb[i]+k*yNotch[i+DEFAULT_LINE_DURATION*2];
		float raw=buf->raw[i];
		float ycBlend=std::clamp((ycDiff[std::min(i+3, BUFFER_SIZE-1)]-ycLow)/(ycHigh-ycLow), 0.0f, 1.0f);
		float finalSample=(1.0f-ycBlend)*raw+ycBlend*filtered;
		buf->luminance[i]=finalSample;
		buf->chrominance[0][i]=raw-finalSample;
	}
}

void Decoder::ColorDecoderPAL::demodulateSubcarrier(SignalBuffers *buf){
	double samplesPerPeriod=20000000.0/fsc;
	const int lowpassDelay=chromaLowpassFilter.getDelay();
	memcpy(rawI, rawI+BUFFER_SIZE, lowpassDelay*sizeof(float));
	memcpy(rawQ, rawQ+BUFFER_SIZE, lowpassDelay*sizeof(float));
	for(int i=0;i<BUFFER_SIZE;i++){
		float angle=(1.0/samplesPerPeriod)*sampleCount*2048;
		int lutIndex=(int)angle%2048;
		float subcarrierSample=buf->chrominance[0][i];
		rawI[i+lowpassDelay]=subcarrierSample*sinLUT[lutIndex];
		rawQ[i+lowpassDelay]=subcarrierSample*cosLUT[lutIndex];

		sampleCount++;
	}
	sampleCount%=2500*DEFAULT_LINE_DURATION;
	
	for(int i=0;i<lowpassDelay;i++){
		rawI[i+BUFFER_SIZE+lowpassDelay]=rawI[BUFFER_SIZE+lowpassDelay-i];
		rawQ[i+BUFFER_SIZE+lowpassDelay]=rawQ[BUFFER_SIZE+lowpassDelay-i];
	}
	
	chromaLowpassFilter.process(rawI, buf->chrominance[0], BUFFER_SIZE);
	chromaLowpassFilter.process(rawQ, buf->chrominance[1], BUFFER_SIZE);
}

void Decoder::ColorDecoderPAL::decodeColor(VideoField *field){
	float prevLineBurstAngle=0;
	for(int i=0;i<(field->isBottom ? 313 : 312);i++){
		VideoLine &line=field->lines[i];
		float burstI=0, burstQ=0;
		for(int j=115;j<156;j++){
			burstI+=line.chrominance[0][j];
			burstQ+=line.chrominance[1][j];
		}
		burstI/=41.0f;
		burstQ/=41.0f;
		float burstAngle=atan2f(burstQ, burstI);
		float burstAmplitude=hypotf(burstI, burstQ);
		if(burstAmplitude<0.005f){
			memset(line.chrominance[0], 0, DEFAULT_LINE_DURATION*sizeof(float));
			memset(line.chrominance[1], 0, DEFAULT_LINE_DURATION*sizeof(float));
			continue;
		}
		
		float overallChrominanceScale=1.0f/(burstAmplitude*3.5f);
		float targetAngle=M_PI*0.75f; // 135 degrees
		float angleDiff=burstAngle-prevLineBurstAngle;
		if(angleDiff>M_PI)
			angleDiff-=2*M_PI;
		else if(angleDiff<=-M_PI)
			angleDiff+=2*M_PI;
		
		float vScale=1;
		if(angleDiff<0){
			targetAngle=-targetAngle;
			vScale=-1;
		}
		
		float c=(double)cos(targetAngle-burstAngle);
		float s=(double)sin(targetAngle-burstAngle);
		for(int j=0;j<DEFAULT_LINE_DURATION;j++){
			float re=line.chrominance[0][j];
			float im=line.chrominance[1][j];
			line.chrominance[0][j]=std::clamp(-(re*s+im*c)*vScale*overallChrominanceScale/0.493f, -1.0f, 1.0f);
			line.chrominance[1][j]=std::clamp(-(re*c-im*s)*overallChrominanceScale/0.877f, -1.0f, 1.0f);
		}
		
		prevLineBurstAngle=burstAngle;
	}
	
	// Average the chrominance of each line with the previous line to reduce noise
	// and probably correct phase errors, but I'm not sure if that works like this
	for(int i=(field->isBottom ? 313 : 312)-1;i>0;i--){
		VideoLine &line=field->lines[i];
		VideoLine &prevLine=field->lines[i-1];
		for(int j=0;j<DEFAULT_LINE_DURATION;j++){
			line.chrominance[0][j]=(line.chrominance[0][j]+prevLine.chrominance[0][j])/2.0f;
			line.chrominance[1][j]=(line.chrominance[1][j]+prevLine.chrominance[1][j])/2.0f;
		}
	}
}
