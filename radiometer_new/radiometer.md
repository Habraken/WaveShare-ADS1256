# radiometer_new

Notes specific to the radiometer_new

I will try to split the repository in two parts:
- WaveShare-ADS1256
- radiometer_new

## Description:

The WaveShare-ADS1256 part|folder will be just about the WaveShare board and specifically the ADS1256 ADC on that board.

The radiometer 'sub folder' will be used to migrate all the radiometer related info and files away from the WaveShare-ADS1256

## Schematic (concept & simulations) 



## Schematic (for the purpose of creating the pcb)

Schematic 

![image](radiometer_new_sch.png)

PCB top side

![image](radiometer_new_pcb_t.png)

PCB bottom side

![image](radiometer_new_pcb_b.png)

## Tests with the BPW34(S)

The BPW34 is in series with the input terminals of the ADC. The buffer amplifier of the ADC needs to be enabled for this and is used as the series resistor for the BPW34. De pending on the data rate the effective resistance varies between 10M $\Omega$ and 80M $\Omega$. When testing the BPW34 is important to observe the limits imposed by the ADC. With this simple setup this means that the maximum reverse voltage is limited to a maximum of 3 volt. As can be concluded from the BPW34(S) data sheet[<sup>2</sup>],[<sup>5</sup>] the reverse voltage has no great influence on the sensitivity of the diode.

## I<sub>dark</sub>

Dark current single diodes.
(Reverse voltage 2.5 V, data rate = 11 (24 S/s))

 Diode  | U<sub>ADC</sub>  | I<sub>dark</sub>
--------|----|------------------
 1      |    |                  
 2      |    |                  
 3      |    |                  
 Array1 |    |                  
 Array2 |    |                    

## I<sub>light</sub>

Light current single diodes. 
(Reverse voltage 2.5 V, data rate = 11 (24 S/s), light source a green LED reflected of a white paper)

 Diode  | U<sub>ADC</sub>  | I<sub>light</sub> 
--------|----|------------------
 1      |    |                  
 2      |    |                  
 3      |    |                  
 Array1 |    |                  
 Array2 |    |                    


An alternative diode could be the SFH2401 as suggested on the Osram webpage[<sup>7</sup>] for the BPW34, as the Osram BPW34 is discontinued.
The SFH2401[<sup>6</sup>], is a SMD, has a similar sensitivity as the BPW34, but a smaller dark current, and a lower NEP.

The test do show a strong interference signal.

To eliminate the interference a differential setup was created: where the pin diodes arrays were connected in series to two high precisions 0.1% 10M $\Omega$ resistors[<sup>3</sup>] in a "Wheatstone Bridge" fashion[<sup>9</sup>]. A small reverse bias is applied. This bias of 2.5 V ensures that the ADC will always operate at maximum linearity.

![image](./Screenshot%202022-12-05%20at%2019.11.17.png)

The differential setup hopes to eliminate (common mode) interference as well as temperature drift. The original radiometer used the photodiodes in pv mode, this mode is not well documented and the data sheet (see Vishay BPW34[<sup>2</sup>]) shows a lower photo current for a similar illumination.

## Modeling the BPW34

A model BPW34 based on general information from these sources.[<sup>4</sup>]<sup>,</sup>[<sup>8</sup>]

## First test results with the photo-diode resistor bridge

![image](./Screenshot%202022-12-07%20at%2023.45.39.png)

## LTSpice simulation

Below is an example of a circuit that uses two diodes in the illuminated branch and two diodes in the dark branch. The effect of the additional photo current is reflected by the factor 2 in the current source. An attempt has been made to model the diode characteristics but this is still not very realistic. The simulation does demonstrate though that the temperature dependence of the photo current has been compensated by the bridge arrangement.

![image](./Screenshot%202022-12-14%20at%2022.49.55.png)

The R<sub>eff</sub> is changed in accordance with the R<sub>eff</sub> of the ADC. See tables above for the R<sub>eff</sub> relation to the sample rate.

![image](./Screenshot%202022-12-14%20at%2022.48.20.png)

For each value of R<sub>eff</sub> the illumination is varied form 0 to 2.5 lux (1V=1LUX)

Step | R<sub>eff</sub> | Plot color
-----|-----------------|-----------
@1   | 10M $\Omega$    | green
@2   | 20M $\Omega$    | blue
@3   | 40M $\Omega$    | red
@4   | 80M $\Omega$    | cyan

# A new round of lab testing the designed circuit

This is the diagram of the circuit implemented in the aluminum box, with the Raspberry Pi4 and the WaveShare high precision ADDA board:

![image](./Screenshot%202022-12-23%20at%2000.11.08.png)

Please note: the circuit around the green led has no purpose for the simulations, but is shown here to describe the lab test setup.

The diode model for the BPW34 is not very accurate... but it will do for now. (As the circuit is balanced anyway, all the temperature drift stuff will not have an effect anyway, however I would like to get the capacitance and noise contribution accurate.)

Real lab test results:

Histogram example:

![image](./notes_histogram.png)

Spectrogram example:

![image](./notes_spectrogram.png)

Both plots were created with the data_analysis.ipynb

After the ADC was initialized with the rpi_adc_streaming command:
```
rpi_adc_streaming -n 2 -b -g 0 -c 0 -d 2 -s /tmp/adc.fifo
```
(2 samples per block, buffer enabled, no gain, diff channel 0, data rate = 2463 S/s, streaming to /tmp/adc.fifo)

The data was then saved to the adc_data.csv, which is read by the data_analysis.ipynb.

To time the measurement sessions (2 seconds) I used the following command (in a separate terminal):
```
timeout 2s cat /tmp/adc.fifo > /tmp/adc.data.csv
```

The signal generator for driving the led was setup as follows: 990mV DC offset with a 660mVpp AC at various frequencies.
In this test the led is a 3mm green led that is reflected of a white sheet of paper on the underside of the lid of the aluminum box.
A little shroud is place around the led to avoid direct light hitting the diodes.

To get an idea of the bandwidth of the system I created the following table:

freq [Hz] | ampl [uV] | comment
----------|----------------|--------
200  | 13.9 |
400  | 6.6  |
600  | 5.4  |
800  | 2.5  |
1000 | 2.7  |
1200 | 3.0  |
1400 | 3.9  | @ 1063 Hz
1600 | 3.6  | @  863 Hz
1800 | 1.4  | @  663 Hz
2000 | 2.3  | @  463 Hz
2200 | 1.0  | @  263 Hz

As you can see there is som aliasing happening... A filter is required.
