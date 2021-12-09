CC := gcc

ifneq ($(wildcard /usr/bin/ccache),)
    CCACHE = ccache
endif

TARGETS = rpi_pixleds rpi_smi_adc_test rpi_smi_dac_test rpi_adc_dma_test

LDFLAGS_exe = -rdynamic

all: $(TARGETS)

rpi_pixleds: rpi_pixleds.o rpi_dma_utils.o

rpi_smi_adc_test: rpi_smi_adc_test.o rpi_dma_utils.o

rpi_smi_dac_test: rpi_smi_dac_test.o rpi_dma_utils.o

rpi_adc_dma_test: rpi_adc_dma_test.o rpi_dma_utils.o

%.o: %.c Makefile
	$(CCACHE) $(CC) $(CFLAGS) $(CFLAGS_$@) -c $< -o $@

clean:
	rm -rf $(TARGETS) *.o


