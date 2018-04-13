# BlueMagic
BluePill/BlackMagicProbe support for CSR Programming

This is a fork of https://github.com/Frans-Willem/CsrSpiDrivers

You can program the code onto a bluepill that has a bootloader or a black magic probe.

The pinouts are as follows:

| BluePill/STM32 PIN |BlackMagicProbe|CSR-SPI|
|---------------------|---------------|-------|
|PA3|TDI|MOSI|
|PA4|TMS|CS|
|PA5|TCK|CLK|
|PA6|TDO|MISO|


## Note:
The Blue Pill can only talk at 3.3V. However the BMP should be capable of talking to a 1.8V target in the following manner:
Prior to powering up the black magic probe, connect both SPI_EN and a targets 1.8V test point to the Black Magic's VCC.
On power up, if the BMP sees that there is already voltage greater that 1V at VCC it will leave it's power supply off.
In this manner, the 1.8V of the target is used to power the translator in the BMP and the communication happens at 1.8V.
