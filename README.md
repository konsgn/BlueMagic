# BlueMagic
BluePill/BlackMagicProbe support for CSR Programming

This is a fork of https://github.com/Frans-Willem/CsrSpiDrivers

You can program the code onto a bluepill that has a bootloader or a black magic probe.

The pinouts are as follows:

| BluePill/STM32 PIN |BlackMagicProbe|CSR-SPI|
|---------------------|---------------|-------|
|PA3|TDI|MISO|
|PA4|TMS|CS|
|PA5|TCK|CLK|
|PA6|TDO|MOSI|
