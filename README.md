# mcmboot
Bootloader written for and used in mcmega (AVR ATMega64), supports Intel Hex up- &amp; download

The binary is compiled and linked to start at (byte-)address 0xF000 (Linker option "-Ttext=0xF000") - which is the start-address for 4 kB Bootloader in a ATMega64.
In order to work properly, one has to set the correct FUSES to invoke the bootloader during reset:

Fuse High Byte is configured as follows:

Name | Bit no | Description | Default | __required by mcmboot__
---|---|---|---|---|
OCDEN|7| Enable on-chip debug| 1 | don't care
JTAGEN|6| Enable JTAG| 0 | don't care
SPIEN|5| Enable SPI Serial Program and Data Downloading| 0 | 0
CKOPT|4| Oscillator Option | 1 | depends on your clock-source
EESAVE|3| EEPROM memory is preserved during chip-erase| 1 | don't care
BOOTSZ1|2| Select bootloader size (1)| 0 | 1
BOOTSZ2|1| Select bootloader size (0)| 0 | 1
BOOTRST|0| Select reset vector | 1 | 0

