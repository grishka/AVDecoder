//
//  Decoder.hpp
//  AVDecoder
//
//  Created by Grishka on 29.08.2025.
//

#ifndef Decoder_hpp
#define Decoder_hpp

#define BUFFER_SIZE 524288

#include <stdlib.h>
#include <stdint.h>
#include <vector>
#include <deque>
#include <functional>
#include <CoreFoundation/CoreFoundation.h>

#include "Buffers.h"
#include "threading.h"
#include "BlockingQueue.h"
#include "Filters.hpp"

enum class ColorDisplayMode{
	Full,
	Y,
	Db,
	Dr,
	Raw
};

enum class DisplayLevelsMode{
	Auto,
	BlackIsSync,
	Raw
};

class VideoField;
class VideoLine;
class SignalBuffers;

class Decoder{
public:
	class ColorDecoder;
	Decoder(void* bitmapData, unsigned int bitmapWidth, unsigned int bitmapHeight, unsigned int bitmapStride, CFRunLoopSourceRef runLoopSource);
	~Decoder();
	void handleSampleData(uint8_t* samples, size_t count);
	tgvoip::Buffer getOutputFrame();
	void startOutput();
	void stopOutput();
	void replaceColorDecoder(ColorDecoder *newColorDecoder);
	
	ColorDisplayMode colorMode=ColorDisplayMode::Full;
	DisplayLevelsMode levelsMode=DisplayLevelsMode::Auto;
	bool displayFieldsSequentially=false;
	std::vector<float> scopeData1;
	std::vector<float> scopeData2;
	std::vector<float> scopeLines;
	bool includeBlankingIntervalsInOutput=false;
	bool outputEnabled=false;
	int scopeLineIndex=16;
	float whiteLevelRatio=0.43; // ratio between sync to black and black to white
	std::function<void(uint8_t*, size_t)> vbiDataCallback;
	ColorDecoder *colorDecoder;

	struct SyncPulse{
		int location;
		int length;
		
		SyncPulse offset(int amount){
			return SyncPulse{
				.location=location+amount,
				.length=length
			};
		}
	};
	
	class ColorDecoder{
	public:
		ColorDecoder();
		virtual ~ColorDecoder(){};
		virtual void separateSubcarrier(SignalBuffers *buf, float *nextBuffer)=0;
		virtual void demodulateSubcarrier(SignalBuffers *buf)=0;
		virtual void decodeColor(VideoField *field)=0;
	protected:
		float sinLUT[2048];
		float cosLUT[2048];
	};
	
	class ColorDecoderSECAM: public ColorDecoder{
	public:
		ColorDecoderSECAM();
		virtual ~ColorDecoderSECAM();
		virtual void separateSubcarrier(SignalBuffers *buf, float *nextBuffer);
		virtual void demodulateSubcarrier(SignalBuffers *buf);
		virtual void decodeColor(VideoField *field);
		bool colorArtifactFilterEnabled=true;
	private:
		FIRFilter chromaSeparationFilter;
		BiquadFilter chromaDeemphasisFilter;
		float *rawISamples;
		float *rawQSamples;
		float *prevFieldChrominance[2];
		unsigned int frameCount=0;
		unsigned int angleIndex=0;
		float prevI=0, prevQ=0;
		
		static constexpr float redCenterFreq=4406250;
		static constexpr float blueCenterFreq=4250000;
		static constexpr float redMaxDeviation=280000;
		static constexpr float blueMaxDeviation=230000;
		static constexpr float intermediateFreq=(redCenterFreq+blueCenterFreq)/2.0f;
		int colorLineOffset=0;
	};
	
	class ColorDecoderPAL: public ColorDecoder{
	public:
		ColorDecoderPAL();
		virtual ~ColorDecoderPAL();
		virtual void separateSubcarrier(SignalBuffers *buf, float *nextBuffer);
		virtual void demodulateSubcarrier(SignalBuffers *buf);
		virtual void decodeColor(VideoField *field);
	private:
		FIRFilter chromaNotchFilter;
		BiquadFilter verticalDiffLowpass;
		BiquadFilter ycDiffLowpass;
		FIRFilter chromaLowpassFilter;
		
		float *yNotchRaw;
		float *yNotch;
		float *yComb;
		float *cComb;
		float *verticalDiff;
		float *ycDiff;
		
		float *rawI;
		float *rawQ;
		float *prevSubcarrier;
		int sampleCount=0;
		
		static constexpr float fsc=4433618.75f;
	};

private:
	void runDecoderThread();
	std::vector<VideoLine> processField(VideoField* field, std::vector<SyncPulse> sync, float syncLevel, float blackLevel, float visibleBrightnessRange, bool isBottom);
	void interpolateLine(VideoLine const& src, VideoLine const& dst, float leadingOffset, float trailingOffset, int leadingAlignDest, float trailingAlign);
	void processLine(VideoLine line, int lineIndex, float syncLevel, float blackLevel, float whiteLevel);
	void outputCapturedFrame();
	VideoLine joinLines(VideoLine a, VideoLine b);
	std::vector<VideoLine> splitLine(VideoLine line, int numParts);
	
	tgvoip::Thread decoderThread;
	tgvoip::BlockingQueue<tgvoip::Buffer> newlyAcquiredDataBuffers=tgvoip::BlockingQueue<tgvoip::Buffer>(10);
	tgvoip::BufferPool<BUFFER_SIZE, 10> bufferPool;
	tgvoip::Semaphore blockingSemaphore=tgvoip::Semaphore(1, 1);
	tgvoip::BufferPool<942*625*2*3, 10> outputBufferPool;
	tgvoip::BlockingQueue<tgvoip::Buffer> outputQueue=tgvoip::BlockingQueue<tgvoip::Buffer>(10);
	tgvoip::Buffer currentOutputBuffer=tgvoip::Buffer(0);
	tgvoip::Mutex colorDecoderMutex;
	
	std::deque<VideoField*> fieldPool;
	std::deque<VideoField*> fieldQueue;
	VideoField *interpolatedField;
	void* bitmapData;
	unsigned int bitmapWidth, bitmapHeight, bitmapStride;
	bool running=true;
	int currentLineIndex=0;
	unsigned int currentLineIndexTotal=0;
	float syncLevel=0;
	float syncThreshold=0;
	float blackLevel=0;
	float visibleBrightnessRange=0.99f;
	float nextSyncLevel=0;
	float detectedWhiteLevel=0;
	int fieldsWithoutVITS=10;

	int frameCount=0;
	uint64_t lastFrameTime;
	
	CFRunLoopSourceRef runLoopSource;
	CFRunLoopRef mainThreadRunLoop;
};

#endif /* Decoder_hpp */
