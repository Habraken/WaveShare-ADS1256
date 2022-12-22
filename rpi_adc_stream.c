// Derived from "Raspberry Pi MCP3202 ADC streaming interface"; see https://iosoft.blog for details
//
// Copyright (c) 2020 Jeremy P Bentham
// Copyright (c) 2022 Jan Habraken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// v0.20 JPB 16/11/20 Tidied up for first Github release
//
//
// v0.30 JH  16/09/22 modified for WaveShare's High-Precision AD/DA Board

#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <ctype.h>
#include <sys/stat.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include "rpi_dma_utils.h"

#define VERSION "0.30"

// Default & max sample rate (samples/sec)
#define SAMPLE_RATE 3267 // a for drate=0xF0 = default
#define MAX_SAMPLE_RATE 3267

// PWM definitions: divisor, and reload value
#define PWM_FREQ 1000000
#define PWM_VALUE 2

// ADC sample size (ADS1256 = 3 bytes, with 24 data bits, 2's complement)
#define ADC_RAW_LEN 3 // used in get_sample method only, not in the DMA method

// Conversion macro's
#define ADC_VOLTAGE(n) (((n)*5.0) / 8388607.0) // (2^23 - 1) for ADS1256. There is a one LSB error on one half of the scale.
#define ADC_MILLIVOLTS(n) ((int)((((n)*3300) + 1024) / 2048))
#define ADC_RAW_VAL(d) (((uint16_t)(d) << 8 | (uint16_t)(d) >> 8) & 0x7ff)

// Non-cached memory size
#define MAX_SAMPLES 4096
#define SAMPLE_SIZE 4 // number of bytes to be read from fifo. Requires equal number of bytes to be clocked out
#define BUFFER_LEN (MAX_SAMPLES * SAMPLE_SIZE)
#define MAX_BUFFERS 2
#define VC_MEM_SIZE (PAGE_SIZE + (BUFFER_LEN * MAX_BUFFERS))

// DMA control block macros
#define NUM_CBS 28
#define REG(r, a) REG_BUS_ADDR(r, a)
#define MEM(m, a) MEM_BUS_ADDR(m, a)
#define CBS(n) MEM_BUS_ADDR(mp, &dp->cbs[(n)])

// DMA transfer information for PWM and SPI
#define PWM_TI (DMA_DEST_DREQ | (DMA_PWM_DREQ << 16) | DMA_WAIT_RESP)
#define SPI_RX_TI (DMA_SRCE_DREQ | (DMA_SPI_RX_DREQ << 16) | DMA_WAIT_RESP | DMA_CB_DEST_INC)
#define SPI_TX_TI (DMA_DEST_DREQ | (DMA_SPI_TX_DREQ << 16) | DMA_WAIT_RESP | DMA_CB_SRCE_INC)

/* SPI clock frequency
 * The SPI_FREQ or SCLK depends ultimately on the core frequency which is adjusted by the OS.
 * Use vcgencmd measure_clock core other options are: [arm h264 isp v3d uart pwm emmc pixel vec hdmi dpi]
 * sudo apt install cpufrequtils
 * Use "cpufreq-info -g" to get a list of available governors
 * Use "sudo cpufreq-set -g powersave" to force the system clock to a stable minimum value: 200 MHz (Pi4)
 */
#define MIN_SPI_FREQ 300000
#define MAX_SPI_FREQ 1920000
#define SPI_FREQ 1920000

// SPI0 pin definitions > BCM numbering, WaveShare board
#define SPI0_CE0_PIN 8   // P_CE0 - not used by the WaveShare board
#define SPI0_CE1_PIN 7   // P_CE1 - not used by the WaveShare board
#define SPI0_MISO_PIN 9  // P_MISO
#define SPI0_MOSI_PIN 10 // P_MOSI
#define SPI0_SCLK_PIN 11 // P_SCK

// ADS1256 pin definitions > BCM number, Waveshare board
#define ADC_CE0_PIN 22   // P3
#define DAC_CE1_PIN 23   // P4
#define ADC_DRDY_PIN 17  // P0
#define ADC_RESET_PIN 18 // P1
#define ADC_SYNC_PIN 27  // P2
#define LED0 21          // P29

// SPI registers and constants
#define SPI0_BASE (PHYS_REG_BASE + 0x204000)
#define SPI_CS 0x00
#define SPI_FIFO 0x04
#define SPI_CLK 0x08
#define SPI_DLEN 0x0c
#define SPI_DC 0x14
#define SPI_FIFO_CLR (3 << 4)
#define SPI_RX_FIFO_CLR (1 << 5)
#define SPI_TX_FIFO_CLR (1 << 4)
#define SPI_TFR_ACT (1 << 7)
#define SPI_DMA_EN (1 << 8)
#define SPI_AUTO_CS (1 << 11)
#define SPI_RXD (1 << 17)
#define SPI_CPOL (0 << 3) // SPI SCLK Polarity
#define SPI_CPHA (1 << 2) // SPI SClk Phase
#define SPI_CE_NUM 0      // 0, 1, 2 no effect as WaveShare board doesn't use the P_CE0 and P_CE1 for the SPI bus

// SPI register strings
char *spi_regstrs[] = {"CS", "FIFO", "CLK", "DLEN", "LTOH", "DC", ""};

// ADC ADS1256 commands
#define ADS1256_WAKEUP 0xff
#define ADS1256_RSTCMD 0xfe
#define ADS1256_SYNC 0xfc
#define ADS1256_RDATA 0x01
#define ADS1256_RREG 0x10
#define ADS1256_WREG 0x50
#define ADS1256_RDATAC 0x03
#define ADS1256_SDATAC 0x0f
#define ADS1256_STANDBY 0xfd
#define ADS1256_SELFCAL 0xf0

// ADC ADS1256 registers
#define ADS1256_STATUS 0x00
#define ADS1256_MUX 0x01
#define ADS1256_ADCON 0x02
#define ADA1256_DRATE 0x03
#define ADS1256_IO 0x04
#define ADS1256_OFC0 0x05
#define ADS1256_OFC1 0x06
#define ADS1256_OFC2 0x07
#define ADS1256_FSCO 0x08
#define ADS1256_FSC1 0x09
#define ADS1256_FSC2 0x0a

// ADC ADS1256 register strings
char *adc_regstrs[] = {"STATUS", "MUX", "ADCON", "DRATE", "IO", "OFC0", "OFC1", "OFC2", "FSC0", "FSC1", "FSC2", ""};

// Microsecond timer
#define USEC_BASE (PHYS_REG_BASE + 0x3000)
#define USEC_TIME 0x04
uint32_t usec_start;

// Buffer for streaming output, and raw Rx data
#define STREAM_BUFFLEN 10000
char stream_buff[STREAM_BUFFLEN];
uint32_t rx_buff[MAX_SAMPLES];

// fcntl constant to get free FIFO length
#define F_GETPIPE_SZ 1032

// Virtual memory pointers to acceess peripherals & memory
extern MEM_MAP gpio_regs, dma_regs, clk_regs, pwm_regs;
MEM_MAP vc_mem, spi_regs, usec_regs;

// File descriptor for FIFO
int fifo_fd;

// Data formats for -f option
#define FMT_USEC 1

// DMA block timing
#define TIME_BLOCK0 32 * 2 // 32 us for TRX of 4 bytes
#define TIME_BLOCK1 16 * 2 // 16 us to define pauze between TRX's
#define TIME_BLOCK2 32 * 2 // 32 us for TRX of 4 bytes
#define TIME_BLOCK3 16 * 2 // 16 us to define pulse width for SYNC
// time_block4 is defined by the ADC settling_time for a given data rate

// Command-line variables
int in_chans = 1, sample_count = 0, sample_rate = SAMPLE_RATE;
int data_format, testmode, verbose, lockstep;
uint8_t drate = 0xF0, pga = 0x00, bufen = 0x00, mux = 0x01;                    // ADS1256 ADC default settings
uint32_t pwm_range, samp_total, overrun_total, fifo_size, settling_time = 210; // settling_time in micro seconds (ADS1256 datasheet, t18)
char *fifo_name;

void terminate(int sig);
void map_devices(void);
void get_uncached_mem(MEM_MAP *mp, int size);
int adc_get_sample(int chan);
float test_pwm_frequency(MEM_MAP *mp);
float test_spi_frequency(MEM_MAP *mp);
void adc_dma_init(MEM_MAP *mp, int ns, int single, uint32_t settling_time, uint8_t mux);
void adc_stream_start(void);
void adc_stream_wait(void);
void adc_stream_stop(void);
int adc_stream_csv(MEM_MAP *mp, char *vals, int maxlen, int nsamp);
void dma_wait(int chan);
void do_streaming(MEM_MAP *mp, char *vals, int maxlen, int nsamp);
int create_fifo(char *fname);
int open_fifo_write(char *fname);
int write_fifo(int fd, void *data, int dlen);
uint32_t fifo_freespace(int fd);
void destroy_fifo(char *fname, int fd);
int init_spi(int hz);
void spi_clear(void);
void spi_cs(int set);
void spi_xfer(uint8_t *txd, uint8_t *rxd, int len);
void reset_adc_hard(void);                                             // hard reset with RST pin of ADS1256
void reset_adc_soft(void);                                             // soft reset with cmd over spi
void init_adc(uint8_t drate, uint8_t pga, uint8_t bufen, uint8_t mux); // init ADS1256
void self_cal_adc(void);                                               // self calibrate ADS1256
void spi_disable(void);
void disp_spi(void); // display current spi registers
void disp_adc(void); // display current adc registers
void conversion_test(void);

typedef struct
{
    char channel[3];
    uint8_t channel_code;
} CHANNEL_INFO;

typedef struct
{
    uint32_t data_rate, settling_time;
    uint8_t drate;
} DATA_RATE_INFO;

// Main program
int main(int argc, char *argv[])
{
    int args = 0, f, val = 0, drate_num, mux_num;
    float freq;

    // ADS1256 datasheet table 11 'valid drate register setting' and table 13 'settling time (t18)'
    static const DATA_RATE_INFO data_rate_lut[16] =
        {
            {.data_rate = 30000, .drate = 0xF0, .settling_time = 210}, // 0   num_avg = 1 (bypassed) default
            {.data_rate = 15000, .drate = 0xE0, .settling_time = 250}, // 1   num_avg = 2
            {.data_rate = 7500, .drate = 0xD0, .settling_time = 310},  // 2   num_avg = 4
            {.data_rate = 3750, .drate = 0xC0, .settling_time = 440},  // 3   num_avg = 8
            {.data_rate = 2000, .drate = 0xB0, .settling_time = 680},  // 4   num_avg = 15
            {.data_rate = 1000, .drate = 0xA1, .settling_time = 1180}, // 5   num_avg = 30
            {.data_rate = 500, .drate = 0x92, .settling_time = 2180},  // 6   num_avg = 60
            {.data_rate = 100, .drate = 0x82, .settling_time = 10180}, // 7   num_avg = 300
            {.data_rate = 60, .drate = 0x72, .settling_time = 16840},  // 8   num_avg = 500
            {.data_rate = 50, .drate = 0x63, .settling_time = 20180},  // 9   num_avg = 600
            {.data_rate = 30, .drate = 0x53, .settling_time = 33510},  // 10  num_avg = 1000
            {.data_rate = 25, .drate = 0x43, .settling_time = 40180},  // 11  num_avg = 1200
            {.data_rate = 15, .drate = 0x33, .settling_time = 66840},  // 12  num_avg = 2000
            {.data_rate = 10, .drate = 0x23, .settling_time = 100180}, // 13  num_avg = 3000
            {.data_rate = 5, .drate = 0x13, .settling_time = 200180},  // 14  num_avg = 6000
            {.data_rate = 2, .drate = 0x03, .settling_time = 400180}   // 15  num_avg = 12000
        };

    // ADS1256 datasheet page 31, MUX: Input Multiplexer Control Register (Address 0x01)
    static const CHANNEL_INFO channel_lut[12] =
        {
            {.channel = "D0", .channel_code = 0x01}, // 0  differential default
            {.channel = "D1", .channel_code = 0x23}, // 1  differential
            {.channel = "D2", .channel_code = 0x45}, // 2  differential
            {.channel = "D3", .channel_code = 0x67}, // 3  differential
            {.channel = "S0", .channel_code = 0x08}, // 4  single-ended
            {.channel = "S1", .channel_code = 0x18}, // 5  single-ended
            {.channel = "S2", .channel_code = 0x28}, // 6  single-ended
            {.channel = "S3", .channel_code = 0x38}, // 7  single-ended
            {.channel = "S4", .channel_code = 0x48}, // 8  single-ended
            {.channel = "S5", .channel_code = 0x58}, // 9  single-ended
            {.channel = "S6", .channel_code = 0x68}, // 10 single-ended
            {.channel = "S7", .channel_code = 0x78}  // 11 single-ended
        };

    printf("RPi ADC streamer v" VERSION "\n");
    while (argc > ++args) // Process command-line args
    {
        if (argv[args][0] == '-')
        {
            switch (toupper(argv[args][1]))
            {
            case 'F': // -F: output format
                if (args >= argc - 1 || !isdigit((int)argv[args + 1][0]))
                    fprintf(stderr, "Error: no format value\n");
                else
                    data_format = atoi(argv[++args]);
                break;
            case 'I': // -I: number of input channels
                if (args >= argc - 1 || !isdigit((int)argv[args + 1][0]))
                    fprintf(stderr, "Error: no input chan count\n");
                else
                    in_chans = atoi(argv[++args]);
                break;
            case 'L': // -L: lockstep streaming
                lockstep = 1;
                break;
            case 'N': // -N: number of samples per block
                if (args >= argc - 1 || !isdigit((int)argv[args + 1][0]) ||
                    (sample_count = atoi(argv[++args])) < 1)
                    fprintf(stderr, "Error: no sample count\n");
                else if (sample_count > MAX_SAMPLES)
                {
                    fprintf(stderr, "Error: maximum sample count %u\n", MAX_SAMPLES);
                    sample_count = MAX_SAMPLES;
                }
                break;
            case 'S': // -S: stream into named pipe (FIFO)
                if (args >= argc - 1 || !argv[args + 1][0])
                    fprintf(stderr, "Error: no FIFO name\n");
                else
                    fifo_name = argv[++args];
                break;
            case 'T': // -T: test mode
                testmode = 1;
                break;
            case 'V': // -V: verbose mode (display hex data)
                verbose = 1;
                break;
            case 'B': // -B: enable buffer amplifier
                bufen = 0x01;
                break;
            case 'G': // -G: programmable gain
                if (args >= argc - 1 || !isdigit((int)argv[args + 1][0]))
                    fprintf(stderr, "Error: no gain setting\n");
                else if ((pga = atoi(argv[++args])) > 6)
                {
                    fprintf(stderr, "Error: invalid gain setting\n");
                    pga = 0x00;
                }
                else
                    pga = (uint8_t)atoi(argv[args]);
                break;
            case 'C': // -C: input channel
                if (args >= argc - 1 || !isdigit((int)argv[args + 1][0]))
                    fprintf(stderr, "Error: no channel selection\n");
                else if ((mux_num = atoi(argv[++args])) > 11)
                {
                    fprintf(stderr, "Error: invalid channel selection\n");
                    mux = 0x01;
                }
                else
                    mux = channel_lut[atoi(argv[args])].channel_code;
                break;
            case 'D': // -D: data rate
                if (args >= argc - 1 || !isdigit((int)argv[args + 1][0]))
                    fprintf(stderr, "Error: no data rate selection\n");
                else if ((drate_num = atoi(argv[++args])) > 15)
                {
                    fprintf(stderr, "Error: invalid data rate selection\n");
                    drate = 0xF0;
                    settling_time = 210;
                    sample_rate = (int)(2 * 1e6 / (TIME_BLOCK0 + TIME_BLOCK1 + TIME_BLOCK2 + TIME_BLOCK3 + (settling_time * 2)));
                }
                else
                {
                    drate = data_rate_lut[atoi(argv[args])].drate;
                    settling_time = data_rate_lut[atoi(argv[args])].settling_time;
                    sample_rate = (int)(2 * 1e6 / (TIME_BLOCK0 + TIME_BLOCK1 + TIME_BLOCK2 + TIME_BLOCK3 + (settling_time * 2)));
                }
                break;
            default:
                printf("Error: unrecognised option '%s'\n", argv[args]);
                exit(1);
            }
        }
    }
    map_devices();
    map_uncached_mem(&vc_mem, VC_MEM_SIZE);
    signal(SIGINT, terminate);
    pwm_range = (PWM_FREQ * 2) / (sample_rate * 2);
    f = init_spi(SPI_FREQ);
    reset_adc_hard();
    init_adc(drate, pga, bufen, mux);
    disp_adc();
    self_cal_adc();
    if (testmode)
    {
        printf("Testing %1.3f MHz SPI frequency: ", f / 1e6);
        freq = test_spi_frequency(&vc_mem);
        printf("%7.3f MHz\n", freq);
        printf("Testing %5u Hz  PWM frequency: ", sample_rate);
        freq = test_pwm_frequency(&vc_mem);
        printf("%7.3f Hz\n", freq / 2);
    }
    else if (sample_count == 0)
    {
        printf("SPI frequency %u Hz\n", f);
        val = adc_get_sample(0x67);
        usleep(1000);
        val = adc_get_sample(0x01);
        usleep(1000);
        val = adc_get_sample(0x23);
        usleep(1000);
        val = adc_get_sample(0x45);
        printf("0x01 ADC value = %08X = % 9d = % 1.9fV\n", val, val, ADC_VOLTAGE(val));
        usleep(1000);
        val = adc_get_sample(0x67);
        printf("0x23 ADC value = %08X = % 9d = % 1.9fV\n", val, val, ADC_VOLTAGE(val));
        usleep(1000);
        val = adc_get_sample(0x01);
        printf("0x45 ADC value = %08X = % 9d = % 1.9fV\n", val, val, ADC_VOLTAGE(val));
        usleep(1000);
        val = adc_get_sample(0x23);
        printf("0x67 ADC value = %08X = % 9d = % 1.9fV\n", val, val, ADC_VOLTAGE(val));
        usleep(1000);
        // disp_adc();
    }
    else if (fifo_name)
    {
        if (create_fifo(fifo_name))
        {
            printf("Created FIFO '%s'\n", fifo_name);
            printf("Streaming %u samples per block at %u S/s %s\n",
                   sample_count, sample_rate, lockstep ? "(lockstep)" : "");
            // printf("PWM Range: %d\n", pwm_range);
            adc_dma_init(&vc_mem, sample_count, 0, settling_time, mux);
            adc_stream_start();
            while (1)
                do_streaming(&vc_mem, stream_buff, STREAM_BUFFLEN, sample_count);
        }
    }
    else
    {
        printf("Reading %u samples at %u S/s\n", sample_count, sample_rate);
        adc_dma_init(&vc_mem, sample_count, 1, settling_time, mux);
        adc_stream_start();
        adc_stream_wait();
        adc_stream_stop();
        adc_stream_csv(&vc_mem, stream_buff, STREAM_BUFFLEN, sample_count);
        printf("%s", stream_buff);
    }
    terminate(0);
}

// Catastrophic failure in initial setup
void fail(char *s)
{
    printf(s);
    terminate(0);
}

// Free memory & peripheral mapping and exit
void terminate(int sig)
{
    printf("Closing\n");
    spi_disable();
    stop_dma(DMA_CHAN_A);
    stop_dma(DMA_CHAN_B);
    stop_dma(DMA_CHAN_C);
    unmap_periph_mem(&vc_mem);
    unmap_periph_mem(&usec_regs);
    unmap_periph_mem(&pwm_regs);
    unmap_periph_mem(&clk_regs);
    unmap_periph_mem(&spi_regs);
    unmap_periph_mem(&dma_regs);
    unmap_periph_mem(&gpio_regs);
    if (fifo_name)
        destroy_fifo(fifo_name, fifo_fd);
    if (samp_total)
        printf("Total samples %u, overruns %u\n", samp_total, overrun_total);
    exit(0);
}

// Map GPIO, DMA and SPI registers into virtual mem (user space)
// If any of these fail, program will be terminated
void map_devices(void)
{
    map_periph(&gpio_regs, (void *)GPIO_BASE, PAGE_SIZE);
    map_periph(&dma_regs, (void *)DMA_BASE, PAGE_SIZE);
    map_periph(&spi_regs, (void *)SPI0_BASE, PAGE_SIZE);
    map_periph(&clk_regs, (void *)CLK_BASE, PAGE_SIZE);
    map_periph(&pwm_regs, (void *)PWM_BASE, PAGE_SIZE);
    map_periph(&usec_regs, (void *)USEC_BASE, PAGE_SIZE);
}

// Get uncached memory
void get_uncached_mem(MEM_MAP *mp, int size)
{
    if (!map_uncached_mem(mp, size))
        fail("Error: can't allocate uncached memory\n");
}

// Fetch single sample from ADC diff. channel 0, 1, 2, 3
int adc_get_sample(int chan)
{
    uint8_t txdata0[] = {0xFF, 0xFF, 0xFF};                                     // read ADC value
    uint8_t txdata1[] = {ADS1256_WREG | ADS1256_MUX, 0x00, chan, ADS1256_SYNC}; // set MUX, prepare SYNC
    uint8_t txdata2[] = {0xFF, ADS1256_RDATA};                                  // sync ADC, request ADC value
    uint8_t rxdata[] = {0x01, 0x02, 0x03};
    uint8_t rxdummy[] = {0x00, 0x00, 0x00};
    int32_t val;

    spi_cs(1);
    gpio_out(ADC_CE0_PIN, 0); // enable ADC
    spi_xfer(txdata0, rxdata, sizeof(txdata0));
    spi_cs(0);
    gpio_out(ADC_CE0_PIN, 1); // disable ADC
    usleep(2);
    spi_cs(1);
    gpio_out(ADC_CE0_PIN, 0); // enable ADC
    spi_xfer(txdata1, rxdummy, sizeof(txdata1));
    spi_cs(0);
    gpio_out(ADC_CE0_PIN, 1); // disable ADC
    usleep(2);
    spi_cs(1);
    gpio_out(ADC_CE0_PIN, 0); // enable ADC
    spi_xfer(txdata2, rxdummy, sizeof(txdata2));
    spi_cs(1);
    gpio_out(ADC_CE0_PIN, 0); // keep ADC enabled
    usleep(500);              // Datasheet ADS1256 t18: after changing mux, ADC settling time
    // spi_clear();
    if (verbose)
    {
        for (int i = 0; i < ADC_RAW_LEN; i++)
            printf("%02X ", rxdata[i]);
        printf("\n");
    }
    val = (rxdata[0] & 0x80 ? 0xFF : 0x00) << 24 | rxdata[0] << 16 | rxdata[1] << 8 | rxdata[2]; // convert 3 bytes rx data to 32 bit int
    return val;
}

// Definitions for SPI frequency test
#define SPI_TEST_TI (DMA_DEST_DREQ | (DMA_SPI_TX_DREQ << 16) | DMA_WAIT_RESP | DMA_CB_SRCE_INC)
#define TEST_NSAMPS 10

typedef struct
{
    DMA_CB cbs[NUM_CBS];
    uint32_t txd[TEST_NSAMPS], val;
    volatile uint32_t usecs[2];
} TEST_DMA_DATA;

// Test SPI frequency
float test_spi_frequency(MEM_MAP *mp)
{
    TEST_DMA_DATA *dp = mp->virt;
    TEST_DMA_DATA dma_data =
        {
            .txd = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, .usecs = {0, 0}, .cbs = {
            // Tx output: 2 initial transfers, then 10 timed transfers
            {SPI_TEST_TI, MEM(mp, dp->txd), REG(spi_regs, SPI_FIFO), 2 * 4, 0, CBS(1), 0},           // 0
            {SPI_TEST_TI, REG(usec_regs, USEC_TIME), MEM(mp, &dp->usecs[0]), 4, 0, CBS(2), 0},       // 1
            {SPI_TEST_TI, MEM(mp, dp->txd), REG(spi_regs, SPI_FIFO), TEST_NSAMPS * 4, 0, CBS(3), 0}, // 2
            {SPI_TEST_TI, REG(usec_regs, USEC_TIME), MEM(mp, &dp->usecs[1]), 4, 0, 0, 0},            // 3
        }};
    memcpy(dp, &dma_data, sizeof(dma_data));                         // Copy DMA data into uncached memory
    *REG32(spi_regs, SPI_DC) = (8 << 24) | (1 << 16) | (8 << 8) | 1; // Set DMA priorities
    *REG32(spi_regs, SPI_CS) = SPI_FIFO_CLR;                         // Clear SPI FIFOs
    start_dma(mp, DMA_CHAN_A, &dp->cbs[0], 0);                       // Start SPI Tx DMA
    *REG32(spi_regs, SPI_DLEN) = (TEST_NSAMPS + 2) * 4;              // Set data length, and SPI flags
    *REG32(spi_regs, SPI_CS) = SPI_TFR_ACT | SPI_DMA_EN;
    dma_wait(DMA_CHAN_A);                    // Wait until complete
    *REG32(spi_regs, SPI_CS) = SPI_FIFO_CLR; // Clear accumulated Rx data
    return (dp->usecs[1] > dp->usecs[0] ? 32.0 * TEST_NSAMPS / (dp->usecs[1] - dp->usecs[0]) : 0);
}

// Test PWM frequency
float test_pwm_frequency(MEM_MAP *mp)
{
    TEST_DMA_DATA *dp = mp->virt;
    TEST_DMA_DATA dma_data = {
        .val = pwm_range, .usecs = {0, 0}, .cbs = {
        // Tx output: 2 initial transfers, then timed transfer
        {PWM_TI, MEM(mp, &dp->val), REG(pwm_regs, PWM_FIF1), 4, 0, CBS(1), 0},        // 0
        {PWM_TI, MEM(mp, &dp->val), REG(pwm_regs, PWM_FIF1), 4, 0, CBS(2), 0},        // 1
        {PWM_TI, REG(usec_regs, USEC_TIME), MEM(mp, &dp->usecs[0]), 4, 0, CBS(3), 0}, // 2
        {PWM_TI, MEM(mp, &dp->val), REG(pwm_regs, PWM_FIF1), 4, 0, CBS(4), 0},        // 3
        {PWM_TI, REG(usec_regs, USEC_TIME), MEM(mp, &dp->usecs[1]), 4, 0, 0, 0},      // 4
    }};
    memcpy(dp, &dma_data, sizeof(dma_data));                         // Copy DMA data into uncached memory
    *REG32(spi_regs, SPI_DC) = (8 << 24) | (1 << 16) | (8 << 8) | 1; // Set DMA priorities
    init_pwm(PWM_FREQ, pwm_range, PWM_VALUE);                        // Initialise PWM
    *REG32(pwm_regs, PWM_DMAC) = PWM_DMAC_ENAB | PWM_ENAB;           // Enable PWM DMA
    start_dma(mp, DMA_CHAN_A, &dp->cbs[0], 0);                       // Start DMA
    start_pwm();                                                     // Start PWM
    dma_wait(DMA_CHAN_A);                                            // Wait until complete
    stop_pwm();                                                      // Stop PWM
    return (dp->usecs[1] > dp->usecs[0] ? 1e6 / (dp->usecs[1] - dp->usecs[0]) : 0);
}

typedef struct
{
    DMA_CB cbs[NUM_CBS];
    uint32_t txd1_size, txd2_size, adc_csd1, adc_csd2, pin1, pin2;
    uint32_t pwm_val, pwm_rng0, pwm_rng1, pwm_rng2, pwm_rng3, pwm_rng4;
    uint8_t txd0[8], txd1[8];
    volatile uint32_t usecs[2], states[2], rxd1[MAX_SAMPLES], rxd2[MAX_SAMPLES];
} ADC_DMA_DATA;

// Initialise PWM-paced DMA for ADC sampling
void adc_dma_init(MEM_MAP *mp, int nsamp, int single, uint32_t settling_time, uint8_t mux)
{
    ADC_DMA_DATA *dp = mp->virt;
    ADC_DMA_DATA dma_data = {
        .txd1_size = 4, .txd2_size = 4, .pwm_val = 1, 
        .pwm_rng0 = TIME_BLOCK0, .pwm_rng1 = TIME_BLOCK1, 
        .pwm_rng2 = TIME_BLOCK2, .pwm_rng3 = TIME_BLOCK3, 
        .pwm_rng4 = (settling_time * 2), 
        .txd0 = {0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF}, // primer / dummy data
        .txd1 = {ADS1256_WREG | ADS1256_MUX, 0x00, mux, ADS1256_RDATA, 0x00, 0x00, 0x00, 0x00},
        .pin1 = 1 << ADC_SYNC_PIN, .pin2 = 1 << ADC_CE0_PIN,
        .adc_csd1 = SPI_TFR_ACT | SPI_AUTO_CS | SPI_DMA_EN | SPI_CPHA | SPI_CE_NUM,
        .adc_csd2 = SPI_TFR_ACT | SPI_AUTO_CS | SPI_DMA_EN | SPI_FIFO_CLR | SPI_CPHA | SPI_CE_NUM,
        .usecs = {0, 0}, .states = {0, 0}, .rxd1 = {0}, .rxd2 = {0},
        .cbs = {
            // Rx input: read data from usec clock and SPI, into 2 ping-pong buffers
            {SPI_RX_TI, REG(usec_regs, USEC_TIME), MEM(mp, &dp->usecs[0]),    4, 0, CBS(1),  0}, // 0
            {SPI_RX_TI, REG(spi_regs, SPI_FIFO),   MEM(mp, dp->rxd1), nsamp * 4, 0, CBS(2),  0}, // 1
            {SPI_RX_TI, REG(spi_regs, SPI_CS),     MEM(mp, &dp->states[0]),   4, 0, CBS(3),  0}, // 2
            {SPI_RX_TI, REG(usec_regs, USEC_TIME), MEM(mp, &dp->usecs[1]),    4, 0, CBS(4),  0}, // 3
            {SPI_RX_TI, REG(spi_regs, SPI_FIFO),   MEM(mp, dp->rxd2), nsamp * 4, 0, CBS(5),  0}, // 4
            {SPI_RX_TI, REG(spi_regs, SPI_CS),     MEM(mp, &dp->states[1]),   4, 0, CBS(0),  0}, // 5
            // Tx output: prime the SPI BUS then repeat the same 8 bytes to set the MUX and get the data
            {SPI_TX_TI, MEM(mp, dp->txd0),         REG(spi_regs, SPI_FIFO),   8, 0, CBS(7),  0}, // 6  primer / dummy data
            {SPI_TX_TI, MEM(mp, dp->txd1),         REG(spi_regs, SPI_FIFO),   8, 0, CBS(7),  0}, // 7
            // PWM ADC trigger: wait for PWM, set sample length, trigger SPI, trigger pin changes
            // time_block0 :: 0..32 us
            {PWM_TI,    MEM(mp, &dp->pwm_rng0),    REG(pwm_regs, PWM_RNG1),   4, 0, CBS(9),  0}, // 8  set timer TRX 1
            {PWM_TI,    MEM(mp, &dp->pwm_val),     REG(pwm_regs, PWM_FIF1),   4, 0, CBS(10), 0}, // 9  start timer
            {PWM_TI,    MEM(mp, &dp->pin1),        REG(gpio_regs, GPIO_SET0), 4, 0, CBS(11), 0}, // 10 ensure SYNC=1
            {PWM_TI,    MEM(mp, &dp->pin2),        REG(gpio_regs, GPIO_SET0), 4, 0, CBS(12), 0}, // 11 ensure CE0=1
            {PWM_TI,    MEM(mp, &dp->pin2),        REG(gpio_regs, GPIO_CLR0), 4, 0, CBS(13), 0}, // 12 set    CE0=0
            {PWM_TI,    MEM(mp, &dp->txd1_size),   REG(spi_regs, SPI_DLEN),   4, 0, CBS(14), 0}, // 13 set number of bytes for TRX1
            {PWM_TI,    MEM(mp, &dp->adc_csd1),    REG(spi_regs, SPI_CS),     4, 0, CBS(15), 0}, // 14 start  TRX1
            // time_block1 :: 32..48 us
            {PWM_TI,    MEM(mp, &dp->pwm_rng1),    REG(pwm_regs, PWM_RNG1),   4, 0, CBS(16), 0}, // 15 set the delay timer between 2x TRX
            {PWM_TI,    MEM(mp, &dp->pwm_val),     REG(pwm_regs, PWM_FIF1),   4, 0, CBS(17), 0}, // 16 start timer
            // time_block2 :: 48..80 us
            {PWM_TI,    MEM(mp, &dp->pwm_rng2),    REG(pwm_regs, PWM_RNG1),   4, 0, CBS(18), 0}, // 17 set timer TRX 2
            {PWM_TI,    MEM(mp, &dp->pwm_val),     REG(pwm_regs, PWM_FIF1),   4, 0, CBS(19), 0}, // 18 start timer
            {PWM_TI,    MEM(mp, &dp->txd2_size),   REG(spi_regs, SPI_DLEN),   4, 0, CBS(20), 0}, // 19 set number of bytes for TRX2
            {PWM_TI,    MEM(mp, &dp->adc_csd2),    REG(spi_regs, SPI_CS),     4, 0, CBS(21), 0}, // 20 start  TRX2
            // time_block3 :: 80..96 us
            {PWM_TI,    MEM(mp, &dp->pwm_rng3),    REG(pwm_regs, PWM_RNG1),   4, 0, CBS(22), 0}, // 21 set the SYNC pulse width timer
            {PWM_TI,    MEM(mp, &dp->pwm_val),     REG(pwm_regs, PWM_FIF1),   4, 0, CBS(23), 0}, // 22 start timer
            {PWM_TI,    MEM(mp, &dp->pin1),        REG(gpio_regs, GPIO_CLR0), 4, 0, CBS(24), 0}, // 23 set    SYNC=0 start conversion
            {PWM_TI,    MEM(mp, &dp->pin2),        REG(gpio_regs, GPIO_SET0), 4, 0, CBS(25), 0}, // 24 set    CE0=1
            // time_block4 :: 96..xx us
            {PWM_TI,    MEM(mp, &dp->pwm_rng4),    REG(pwm_regs, PWM_RNG1),   4, 0, CBS(26), 0}, // 25 set wait timer for conversion
            {PWM_TI,    MEM(mp, &dp->pwm_val),     REG(pwm_regs, PWM_FIF1),   4, 0, CBS(27), 0}, // 26 start timer
            {PWM_TI,    MEM(mp, &dp->pin1),        REG(gpio_regs, GPIO_SET0), 4, 0, CBS(8),  0}, // 27 set    SYNC=1
        }};
    if (single) // If single-shot, stop after first Rx block
        dma_data.cbs[2].next_cb = 0;
    memcpy(dp, &dma_data, sizeof(dma_data));  // Copy DMA data into uncached memory
    init_pwm(PWM_FREQ, pwm_range, PWM_VALUE); // Initialise PWM, with DMA
    *REG32(pwm_regs, PWM_DMAC) = PWM_DMAC_ENAB | (0x01 << 8) | 0x01;
    *REG32(spi_regs, SPI_DC) = (0x30 << 24) | (0x20 << 16) | (0x00 << 8) | 0x04; // Set DMA priorities
    *REG32(spi_regs, SPI_CS) = SPI_FIFO_CLR;                                     // Clear SPI FIFOs
    start_dma(mp, DMA_CHAN_C, &dp->cbs[6], 0);                                   // Start SPI Tx DMA
    start_dma(mp, DMA_CHAN_B, &dp->cbs[0], 0);                                   // Start SPI Rx DMA
    start_dma(mp, DMA_CHAN_A, &dp->cbs[8], 0);                                   // Start PWM DMA, for SPI trigger
}

// Manage streaming output
void do_streaming(MEM_MAP *mp, char *vals, int maxlen, int nsamp)
{
    int n;

    if (!fifo_fd)
    {
        if ((fifo_fd = open_fifo_write(fifo_name)) > 0)
        {
            printf("Started streaming to FIFO '%s'\n", fifo_name);
            fifo_size = fifo_freespace(fifo_fd);
        }
    }
    if (fifo_fd)
    {
        if ((n = adc_stream_csv(mp, vals, maxlen, nsamp)) > 0)
        {
            if (!write_fifo(fifo_fd, vals, n))
            {
                printf("Stopped streaming\n");
                close(fifo_fd);
                fifo_fd = 0;
                usleep(100000);
            }
        }
        else
            usleep(10);
    }
}

// Start ADC data acquisition
void adc_stream_start(void)
{
    start_pwm();
}

// Wait until a (single) DMA cycle is complete
void adc_stream_wait(void)
{
    dma_wait(DMA_CHAN_B);
}

// Stop ADC data acquisition
void adc_stream_stop(void)
{
    stop_pwm();
}

// Fetch samples from ADC buffer, return comma-delimited integer values
// If in lockstep mode, discard new data if FIFO isn't empty
int adc_stream_csv(MEM_MAP *mp, char *vals, int maxlen, int nsamp)
{
    ADC_DMA_DATA *dp = mp->virt;
    uint32_t i, n, usec, slen = 0;
    int32_t val[nsamp * 4];
    for (n = 0; n < 2 && slen == 0; n++)
    {
        if (dp->states[n])
        {
            samp_total += nsamp;
            memcpy(rx_buff, n ? (void *)dp->rxd2 : (void *)dp->rxd1, nsamp * 4);
            usec = dp->usecs[n];
            if (dp->states[n ^ 1])
            {
                dp->states[0] = dp->states[1] = 0;
                overrun_total++;
                break;
            }
            dp->states[n] = 0;
            if (usec_start == 0)
                usec_start = usec;
            if (!lockstep || fifo_freespace(fifo_fd) >= fifo_size)
            {
                if (data_format == FMT_USEC)
                    slen += sprintf(&vals[slen], "%u", usec - usec_start);
                
                for (i = 0; i < nsamp * 4 && slen + 20 < maxlen; i += 8)
                {
                    /* This is a hack ( I know ! ) 
                     * This is a hack, to handle the occoasional out of sync situation where data of the ADC ends up in the first
                     * 4 byte word insted of the second 4 byte word. Somehow the DMA sequnce gets of of sync? The reason for this 
                     * is (still) unknown. 
                     */
                    if ( *(((uint8_t *)rx_buff) + i + 0) + *(((uint8_t *)rx_buff) + i + 1) + *(((uint8_t *)rx_buff) + i + 2) + *(((uint8_t *)rx_buff) + i + 3) == 0)
                    {
                        val[i] = (*(((uint8_t *)rx_buff) + i + 4) & 0x80 ? 0xFF : 0x00) << 24 | *(((uint8_t *)rx_buff) + i + 4) << 16 | *(((uint8_t *)rx_buff) + i + 5) << 8 | *(((uint8_t *)rx_buff) + i + 6); // convert 3 bytes rx data to 32 bit int
                    }
                    else
                    {
                        val[i] = (*(((uint8_t *)rx_buff) + i + 0) & 0x80 ? 0xFF : 0x00) << 24 | *(((uint8_t *)rx_buff) + i + 0) << 16 | *(((uint8_t *)rx_buff) + i + 1) << 8 | *(((uint8_t *)rx_buff) + i + 2); // convert 3 bytes rx data to 32 bit int
                    }
                    slen += sprintf(&vals[slen], "%s % 1.9f", slen ? "," : "", (double)ADC_VOLTAGE(val[i]));
                }
                slen += sprintf(&vals[slen], "\n");
                
                if (verbose)
                {
                    for (int i = 0; i < nsamp * 4; i++)
                        printf("%02X ", *(((uint8_t *)rx_buff) + i));
                    printf("\n");
                }
            }
        }
    }
    vals[slen] = 0;
    return (slen);
}

// Test of SPI write cycles
// Redundant code, kept in as an explanation of SPI data length
int spi_tx_test(MEM_MAP *mp, uint16_t *buff, int nsamp)
{
    uint32_t n, a = 0;

    nsamp = 8;
    *REG32(spi_regs, SPI_CS) = SPI_FIFO_CLR;
#if 1
    // Write data length to DLEN reg (with ACT clear)
    for (n = 0; n < nsamp; n++)
    {
        *REG32(spi_regs, SPI_DLEN) = 2;
        *REG32(spi_regs, SPI_CS) = SPI_TFR_ACT | SPI_AUTO_CS | SPI_DMA_EN | SPI_FIFO_CLR;
        *REG32(spi_regs, SPI_FIFO) = n;
        usleep(5);
        a += *REG32(spi_regs, SPI_FIFO);
        // *REG32(spi_regs, SPI_CS) = SPI_FIFO_CLR; // Not needed, as ACT is already clear
    }
#else
    // Write data length to FIFO (with ACT set)
    *REG32(spi_regs, SPI_CS) = SPI_TFR_ACT | SPI_AUTO_CS | SPI_DMA_EN | SPI_FIFO_CLR;
    for (n = 0; n < nsamp; n++)
    {
        *REG32(spi_regs, SPI_FIFO) = (2 << 16) | SPI_TFR_ACT | SPI_FIFO_CLR;
        *REG32(spi_regs, SPI_FIFO) = n;
        usleep(5);
        a += *REG32(spi_regs, SPI_FIFO);
        // *REG32(spi_regs, SPI_CS) = SPI_FIFO_CLR; // Clearing ACT would disrupt comms
    }
#endif
    return (0);
}

// Wait until DMA is complete
void dma_wait(int chan)
{
    int n = 60000;

    do
    {
        usleep(10);
    } while (dma_transfer_len(chan) && --n);
    if (n == 0)
        printf("DMA transfer timeout\n");
}

// Create a FIFO (named pipe)
int create_fifo(char *fname)
{
    int ok = 0;

    umask(0);
    if (mkfifo(fname, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH) < 0 && errno != EEXIST)
        printf("Can't open FIFO '%s'\n", fname);
    else
        ok = 1;
    return (ok);
}

// Open a FIFO for writing, return 0 if there is no reader
int open_fifo_write(char *fname)
{
    int f = open(fname, O_WRONLY | O_NONBLOCK);

    return (f == -1 ? 0 : f);
}

// Write to FIFO, return 0 if no reader
int write_fifo(int fd, void *data, int dlen)
{
    struct pollfd pollfd = {fd, POLLOUT, 0};

    poll(&pollfd, 1, 0);
    if (pollfd.revents & POLLOUT && !(pollfd.revents & POLLERR))
        return (fd ? write(fd, data, dlen) : 0);
    return (0);
}

// Return the free space in FIFO
uint32_t fifo_freespace(int fd)
{
    return (fcntl(fd, F_GETPIPE_SZ));
}

// Remove the FIFO
void destroy_fifo(char *fname, int fd)
{
    if (fd > 0)
        close(fd);
    unlink(fname);
}

// Initialise SPI0, given desired clock freq; return actual value
int init_spi(int hz)
{
    int sclk_freq, div = (SPI_CLOCK_HZ / hz + 1) & ~1;

    gpio_set(SPI0_CE0_PIN, GPIO_ALT0, GPIO_PULLUP);
    gpio_set(SPI0_CE1_PIN, GPIO_ALT0, GPIO_PULLUP);
    gpio_set(SPI0_MISO_PIN, GPIO_ALT0, GPIO_PULLDN);
    gpio_set(SPI0_MOSI_PIN, GPIO_ALT0, GPIO_PULLDN);
    gpio_set(SPI0_SCLK_PIN, GPIO_ALT0, GPIO_NOPULL);
    gpio_set(ADC_CE0_PIN, GPIO_OUT, GPIO_NOPULL);   // WaveShare board
    gpio_set(DAC_CE1_PIN, GPIO_OUT, GPIO_NOPULL);   // WaveShare board
    gpio_set(ADC_RESET_PIN, GPIO_OUT, GPIO_NOPULL); // WaveShare board
    gpio_set(ADC_SYNC_PIN, GPIO_OUT, GPIO_NOPULL);  // WaveShare board
    gpio_set(ADC_DRDY_PIN, GPIO_IN, GPIO_NOPULL);   // WaveShare board
    gpio_set(LED0, GPIO_OUT, GPIO_NOPULL);
    while (div == 0 || (sclk_freq = SPI_CLOCK_HZ / div) > MAX_SPI_FREQ)
        div += 2;
    printf("frequency: %d, divider: %d \n", sclk_freq, div);
    *REG32(spi_regs, SPI_CS) = SPI_FIFO_CLR | SPI_CPHA;
    *REG32(spi_regs, SPI_CLK) = div;
    gpio_out(ADC_CE0_PIN, 0);  // enable ADC
    gpio_out(ADC_SYNC_PIN, 1); // wakeup ADC
    return (sclk_freq);
}

// Clear SPI FIFOs
void spi_clear(void)
{
    *REG32(spi_regs, SPI_CS) = SPI_FIFO_CLR | SPI_TFR_ACT;
}

// Set / clear SPI chip select
void spi_cs(int set)
{
    uint32_t csval = *REG32(spi_regs, SPI_CS);

    *REG32(spi_regs, SPI_CS) = set ? csval | 0x80 : csval & ~0x80;
}

// Transfer SPI bytes
void spi_xfer(uint8_t *txd, uint8_t *rxd, int len)
{
    while (len--)
    {
        *REG8(spi_regs, SPI_FIFO) = *txd++;
        while ((*REG32(spi_regs, SPI_CS) & (1 << 17)) == 0)
            ; // wait until bit 17 of SPI_CS is set
        *rxd++ = *REG32(spi_regs, SPI_FIFO);
    }
}

// Disable SPI
void spi_disable(void)
{
    *REG32(spi_regs, SPI_CS) = SPI_FIFO_CLR;
    *REG32(spi_regs, SPI_CS) = 0;
}

// Display SPI registers
void disp_spi(void)
{
    volatile uint32_t *p = REG32(spi_regs, SPI_CS);
    int i = 0;

    printf("\n");
    while (spi_regstrs[i][0])
        printf("%-4s %08X ", spi_regstrs[i++], *p++);
    printf("\n");
}

// Display ADC registers
void disp_adc(void)
{
    uint8_t txdata[] = {ADS1256_RREG | ADS1256_STATUS, 0x0B - 1};
    uint8_t rxdata[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t dummydata[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    spi_cs(1);
    spi_xfer(txdata, rxdata, sizeof(txdata)); // send rreg cmd
    spi_cs(0);
    usleep(10);
    spi_cs(1);
    spi_xfer(dummydata, rxdata, sizeof(dummydata)); // receive rreg data
    spi_cs(0);
    usleep(10000);
    printf("\nADS1256 register data:\n");
    for (int i = 0; i < 0x0b; i++)
        printf("%-6s 0x%02X \n", adc_regstrs[i], rxdata[i]);
    printf("\n");
}

// Soft Reset ADS1256
void reset_adc_soft(void)
{
    uint8_t txdata[] = {0xfe}; // ADS1256 reset command
    uint8_t rxdata[1];

    spi_cs(1);
    spi_xfer(txdata, rxdata, sizeof(txdata));
    spi_cs(0);
    usleep(10000); // 10 ms
}

// Soft Reset ADS1256
void self_cal_adc(void)
{
    uint8_t txdata[] = {0xf0}; // ADS1256 self calibrate command
    uint8_t rxdata[1];

    spi_cs(1);
    spi_xfer(txdata, rxdata, sizeof(txdata));
    spi_cs(0);
    usleep(800000); // 800 ms
}

// Init ADS1256: set sample rate
void init_adc(uint8_t drate, uint8_t pga, uint8_t bufen, uint8_t mux)
{
    uint8_t status = bufen << 1;
    uint8_t adcon = 0x20 | pga;
    uint8_t txdata[] = {ADS1256_WREG, 0x03, status, mux, adcon, drate}; // ADS1256 wreg command, 4 bytes
    uint8_t rxdata[6];

    spi_cs(1);
    spi_xfer(txdata, rxdata, sizeof(txdata));
    spi_cs(0);
    usleep(10000); // 10ms
}

// Hard Reset ADS1256
void reset_adc_hard(void)
{
    gpio_out(ADC_RESET_PIN, 0);
    usleep(10); // t16 ADS1256 datasheet > min. 4x tau_clkin = 0.5 usec
    gpio_out(ADC_RESET_PIN, 1);
    usleep(10000); // 10ms
}

// Testing conversion arithmatic
void conversion_test(void)
{
    int32_t val;
    uint8_t data[5][3] = {{0x80, 0x00, 0x00},
                          {0xFF, 0xFF, 0xFF},
                          {0x00, 0x00, 0x00},
                          {0x00, 0x00, 0x01},
                          {0x7F, 0xFF, 0xFF}};
    printf("\n Conversion Test \n");
    for (int i = 0; i < 5; i++)
    {
        val = (data[i][0] & 0x80 ? 0xFF : 0x00) << 24 | data[i][0] << 16 | data[i][1] << 8 | data[i][2];
        printf("%08X, % d \n", val, val);
    }
    printf("\n");
}
// EOF