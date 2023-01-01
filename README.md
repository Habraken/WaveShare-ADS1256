# WaveShare-ADS1256

A utility to stream micro second accurate samples from the Raspberry Pi High-Precision AD/DA Expansion Board by WaveShare.   

Please see the notes.md for instructions and hints on how to compile and use the program.

This is a work in progress and it is likely that things will change without prior notice. This is a hobby project and and therefore it should be needless to say: it comes with out any warranty of any kind. It is provided here in case it might be useful to someone...

Recent update:
I added the appropriate scaling of the output voltage based on the gain setting. (-g 0..6)

To Do's:
- Update the timing of the DMA blocks for a higher ARM clock speed. The clock speed will still need to be fixed to a single number
- Scan four differential channels in a sequence
- Scan eight single channels in a sequence  

## Credits

This program depends on, and is derived from, a great DMA example by:
Jeremy P Bentham, "Raspberry Pi ADC data streaming".

See https://iosoft.blog/streaming-analog-data-raspberry-pi for details.
