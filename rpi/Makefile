RPI_MODEL := $(shell cat /proc/device-tree/model 2>/dev/null)

ifneq (,$(findstring Raspberry Pi 4,$(RPI_MODEL)))
	CFLAGS += -DRPI_4
else ifneq (,$(findstring Raspberry Pi 3,$(RPI_MODEL)))
	CFLAGS += -DRPI_3
else ifneq (,$(findstring Raspberry Pi 2,$(RPI_MODEL)))
	CFLAGS += -DRPI_2
endif

all: adc_stream usb_init

adc_stream: adc_stream.c rpi_dma_utils.c
	gcc adc_stream.c rpi_dma_utils.c -O3 -o adc_stream

usb_init: usb_init.c
	gcc usb_init.c -o usb_init

clean:
	rm -f *.o adc_stream usb_init
