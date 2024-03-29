# NOTES

Here I will keep track of my notes of this project. 
E.g. this is a good place to list and describe de arguments to the adc streaming program.

## credits

This program is depends on and is derived from an great DMA example by:

Jeremy P Bentham, "Raspberry Pi ADC data streaming"

See https://iosoft.blog/streaming-analog-data-raspberry-pi for details.

So far this program has been tested on a RPI4 running Bullseye. However is should work on any RPI. I intend to test it on a RPI 0 and 3 as well. More later...

---

## compiling the program

Please do set your type of RPI (0..4) in the top of rpi_dma_utils.h:

``` c
// Raspberry Pi hardware version (0 to 4)
#define RPI_VERSION     4
```

Create the executable from the three files:
- rpi_adc-stream.c
- rpi_dma_utils.h
- rpi_dma_utils.c

with the following command:

```
gcc -Wall -g -o rpi_adc_stream rpi_adc_stream.c rpi_dma_utils.c
```

## pre-requisites

For this program to work correctly the the cpu frequency of the RPI needs to be fixed, as DMA is used to drive the SPI bus and GPIO. A varying RPI clock frequency will alter the timing of the SPI bus and GPIO signals that control the ADS1256 and render communication with the ADS1256 impossible. Needless to say there should be no other device using the SPI bus and it might be a good idea to disable the SPI bus in the raspi-config.

The cpu frequency is normally controlled by the OS and is scaled up and down based on the workload. You can also instruct the OS not to do this. For that you need to install the cpufrequtils if not already present:

```
sudo apt install cpufrequtils
```

once installed, you can use the commands below:

- to set the RPI freq to it's lowest state run:

```
sudo cpufreq-set -g powersave
```

- when you are done you can restore the RPI to it's default state with:

```
sudo cpufreq-set -g ondemand
```


---


## first test

in the directory run the command:

```
sudo ./rpi_adc_stream -t
```

The output should look like this:

```
RPi ADC streamer v0.30
VC mem handle 11, phys 0xbebc6000, virt 0xb6f75000
frequency: 1886792, divider: 106 

ADS1256 register data:
STATUS 0x30 
MUX    0x01 
ADCON  0x20 
DRATE  0xF0 
IO     0xE1 
OFC0   0x10 
OFC1   0xF7 
OFC2   0xFF 
FSC0   0xB4 
FSC1   0x19 
FSC2   0x45 

Conversion Test (PGA=0, gain=1)
FF800000,  -8388608,  -5.000000596 
FFFFFFFF,        -1,  -0.000000596 
00000000,         0,   0.000000000 
00000001,         1,   0.000000596 
007FFFFF,   8388607,   5.000000000 

Testing 1.887 MHz SPI frequency:   1.882 MHz
Testing  3267 Hz  PWM frequency: 3267.974 Hz
Closing
```
If the ADS1256 register data is only showing 0x00 values, than this is an indication that there is a problem with the ADS1256. Did you forget to run the ```sudo cpufreq-set -g powersave``` command?

## basic usage

```
sudo ./rpi_adc_stream -f 1 -n 2 -v -s /tmp/adc.fifo
```
-f 1 = output timestamps
-n 2 = output 2 samples
-v   = show raw data received from the ADS1256
-s   = steam data to /tmp/adc.fifo

In a separate terminal you can view the output with

```
cat /tmp/adc.fifo
```


---


## command arguments

-t = Test mode

-b = ADC input buffer enabled. When omitted it goes to default = not enabled. 
The -b command argument will significantly increase the input impedance of the ADC and can be be modeled as a resistor.
The effective impedance (resistive) depends on the data rate.
At data rates > 1000 S/s this is a minimum of about 10M $\Omega$, for data rates <= 50 S/s this is 80M $\Omega$.[<sup>1</sup>]. If the buffer is not enabled (default) then the input impedance doesn't depend on the data rate, but on the gain (PGA) setting -g. (See below.) For -g = [0..4] (= PGA [1..16]) the impedance = 150kΩ/PGA. For -g = [5, 6] (= PGA [32, 64]) the impedance is 4.7kΩ[<sup>1</sup>]. Please refer to the ADC input impedance verification test section below for more details.  

-g [0..6] = Programmable Gain Amplifier (PGA) setting. 
0 = default. PGA is programmed in the lower 3 bits of the ADCON register of the ADC. The output is scaled according to the gain setting.

 Value | Binary | Gain   | FSR (Full Scale Range) | R<sub>eff</sub>
-------|--------|--------|------------------------|----------------
 0 | 0b000 | 1 (default) | ± 5 V                  | 150.0k $\Omega$
 1 | 0b001 | 2           | ± 2.5 V                |  75.0k $\Omega$
 2 | 0b010 | 4           | ± 1.25 V               |  37.5k $\Omega$
 3 | 0b011 | 8           | ± 0.625 V              |  18.8k $\Omega$
 4 | 0b100 | 16          | ± 0.3125 V             |   9.4k $\Omega$
 5 | 0b101 | 32          | ± 0.15625 V            |   4.7k $\Omega$
 6 | 0b11x | 64          | ± 0.078125 V           |   4.7k $\Omega$

-f 1 = enable time stamps, when omitted no time stamps are written to fifo. 1 is the only option and it shows the number of microseconds passed since the start of the capture. If you intend to analyze the data by using the data_analysis.ipynb Jupyter Notebook, you should omit this command argument as the the notebook can not (yet) process and use the timestamps. 

-c [0..11] = ADC channel selector. Table for valid channel numbers

Value | Channel | Hex | Type
------|---------|-----|-----
 0 | D0 | 0x01 | differential (default)
 1 | D1 | 0x23 | differential
 2 | D2 | 0x45 | differential
 3 | D3 | 0x67 | differential
 4 | S0 | 0x08 | single-ended
 5 | S1 | 0x18 | single-ended
 6 | S2 | 0x28 | single-ended
 7 | S3 | 0x38 | single-ended
 8 | S4 | 0x48 | single-ended
 9 | S5 | 0x58 | single-ended
10 | S6 | 0x68 | single-ended
11 | S7 | 0x78 | single-ended

The current version can only handle a singe channel at a time.

-s = specifies the path and the name of the file to serve as the fifo buffer.

-n [0..4096]= number of samples per block. Normally this is 2 samples for a single channel. (This is due to nature of the SPI communications between the ADC and the Raspberry Pi: the spi interface receives and transmits at the same time. The clock is only active during tx, so to rx a byte you have to tx a byte. The communication with the AD1256 is aligned to 4 bytes, therefore each 'interaction' consists of 2x4 bytes where the first 4 bytes are 0x00 and 3 bytes containing the 3 byte value (2's complement), while the last byte = don't care and could be non-zero.)

-v = verbose output. Shows the byte values received from the ADS1256.

-d [0..15] = data rate. The number of samples per second. Table for valid data rate numbers

 Value | S/s |  Hex | Settling time in $\mu$ s    | Averaging | R<sub>eff</sub> 
-------|-----|------|-----------------------------|-----------|-------------------
0  | 30000 | 0xF0 | 210    | 1 (bypassed) default | 10 M $\Omega$ 
1  | 15000 | 0xE0 | 250    | 2       | 10 M $\Omega$
2  |  7500 | 0xD0 | 310    | 4       | 10 M $\Omega$
3  |  3750 | 0xC0 | 440    | 8       | 10 M $\Omega$
4  |  2000 | 0xB0 | 680    | 15      | 10 M $\Omega$
5  |  1000 | 0xA1 | 1180   | 30      | 20 M $\Omega$
6  |   500 | 0x92 | 2180   | 60      | 40 M $\Omega$
7  |   100 | 0x82 | 10180  | 300     | 40 M $\Omega$
8  |    60 | 0x72 | 16840  | 500     | 40 M $\Omega$
9  |    50 | 0x63 | 20180  | 600     | 80 M $\Omega$
10 |    30 | 0x53 | 33510  | 1000    | 80 M $\Omega$
11 |    25 | 0x43 | 40180  | 1200    | 80 M $\Omega$
12 |    15 | 0x33 | 66840  | 2000    | 80 M $\Omega$
13 |    10 | 0x23 | 100180 | 3000    | 80 M $\Omega$
14 |     5 | 0x13 | 200180 | 6000    | 80 M $\Omega$
15 |     2 | 0x03 | 400180 | 12000   | 80 M $\Omega$

The table above reflects the parameters of the ADC. The actual sample or data rate is in reality lower due to communication overhead.


---


## ADC input impedance verification test

### Buffer = on

Test with ADC in series with 2 x 10MΩ 0.1% resisters and a 2.500 V voltage source. Below are the different median voltage readings for various settings of -d. The other relevant parameters were: -g 0, -b, -c 1

U2     = 2.500 V   
R1, R2 =  10e6 Ω (0.1%)  

R<sub>eff</sub>(U1)  =  U1/((U2-U1)/(R1+R2))  
R<sub>eff</sub> => (U1xR1 + U1xR2)/(U2 - U1)

data rate | U1 median  | U1 $\sigma$| R<sub>eff</sub>
----------|------------|------------|----------------
-d = 11   | 2.010486 V | 0.000283 V | 82,142,124 Ω
-d =  9   | 2.009339 V | 0.000317 V | 81,903,350 Ω
-d =  7   | 1.682762 V | 0.000173 V | 41,181,687 Ω
-d =  5   | 1.264918 V | 0.000184 V | 20,483,142 Ω
-d =  3   | 0.853077 V | 0.000092 V | 10,359,646 Ω

### Buffer = off (default)

Test with ADC in series with 2 x 10kΩ 1% resisters and a 2.500 V voltage source. Below are the different median voltage readings for various settings of -g. The other relevant parameters were: -d 7, -b, -c 2 (I do choose a different channel, as the differential channel between in AIN4 and AIN5 seems to be a little less noisier.)

U2   = 0.250 V  
R1   = 11.12e3 Ω #10.12k + 1k on the WaveShare board  
R2   = 11.09e3 Ω #10.09k + 1k on the WaveShare board  
R<sub>eff</sub>(U1)  =  U1/((U2-U1)/(R1+R2))  
R<sub>eff</sub> => (U1*R1 + U1*R2)/(U2 - U1)  
### Expectations

For PGA settings between from 1 to 16] we expect:  
Vexp(PGA) = U2 / (R1+(150kohm/PGA)+R2) * (150kohm/PGA):  
Vexp( 1) in V => 0.217757 V  
Vexp( 2) in V => 0.192881 V  
Vexp( 4) in V => 0.157009 V  
Vexp( 8) in V => 0.114441 V  
Vexp(16) in V => 0.074205 V  

For PGA settings 32 and 64 we expect:  
Vexp(PGA) = U2 / (R1+(4.7kohm)+R2) * (4.7kohm)  
Vexp(32) in V => 0.043664 V  
Vexp(64) in V => 0.043664 V  

### Results

d=7 is 97 S/s, 100s measurement, gives about 9700 samples. 
U2 voltage source: 0.250 V DC.
  -g (PGA)  |  U1 $\mu$  | U1 $\sigma$| R<sub>eff</sub>
------------|------------|------------|----------------
-g = 0 ( 1) | 0.21909 V  |  0.120 mV  | 157,424.422517Ω
-g = 1 ( 2) | 0.19474 V  |  0.096 mV  | 78,269.551212Ω
-g = 2 ( 4) | 0.16178 V  |  0.073 mV  | 40,729.242802Ω
-g = 3 ( 8) | 0.12074 V  |  0.049 mV  | 20,746.057558Ω
-g = 4 (16) | 0.08017 V  |  0.032 mV  | 10,484.459165Ω
-g = 5 (32) | 0.04769 V  |  0.019 mV  | 5,235.504424Ω
-g = 6 (64) | 0.04762 V  |  0.019 mV  | 5,226.011464Ω

### Conclusion
All the test data fit well with the data sheet.

### A few important things to note
- When the buffer is enabled the voltage on any analog input should not exceed AGND or AVDD - 2 V with respect to AGND. This means for the WaveShare board 0..3V. Exceeding these limits will effect the ADC's performance, especially it's linearity[<sup>1</sup>].

- When the buffer is not enabled the voltage on any analog input should not exceed AGND -0.1 V and AVDD + 0.1 V with respect to AGND[<sup>1</sup>].

- And finally on the WaveShare board, each of the analog inputs pins is preceded by a RC low pass filter consisting of a 1kΩ resistor and a 100nF capacitor[<sup>10</sup>]. The cutoff or corner frequency of this filter is therefore influenced either by enabling the buffer and the data rate or if the buffer is disabled by the PGA setting.


---  





[<sup>1</sup>]: https://www.ti.com/lit/ds/symlink/ads1256.pdf?ts=1666512818664&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FADS1256

[<sup>2</sup>]: https://www.vishay.com/docs/81521/bpw34.pdf

[<sup>3</sup>]: https://docs.rs-online.com/cc04/0900766b81563867.pdf

[<sup>4</sup>]: http://www.osioptoelectronics.com/application-notes/AN-Photodiode-Parameters-Characteristics.pdf

[<sup>5</sup>]: https://dammedia.osram.info/media/resource/hires/osram-dam-5488305/BPW%2034_EN.pdf

[<sup>6</sup>]: https://dammedia.osram.info/media/resource/hires/osram-dam-6026228/SFH%202401_EN.pdf

[<sup>7</sup>]: https://www.osram.com/ecat/DIL%20BPW%2034%20B/com/en/class_pim_web_catalog_103489/prd_pim_device_2219537/

[<sup>8</sup>]: https://qtwork.tudelft.nl/~schouten/linkload/phdiode.pdf

[<sup>9</sup>]: https://www.ti.com/lit/an/sbaa532/sbaa532.pdf?ts=1670166223461&ref_url=https%253A%252F%252Fwww.google.com%252F

[<sup>10</sup>]: https://www.waveshare.com/w/upload/2/29/High-Precision-AD-DA-board.pdf





