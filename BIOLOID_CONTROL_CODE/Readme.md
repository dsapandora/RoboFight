Necesitamos utilizar
avr-gcc para compilar este codigo

Linux distributions have an AVR-GCC toolchain. It may not, however, be installed by default. For instructions about installing AVR-GCC for Linux, see the instructions for your particular distribution.

For linux compile use:

-x c -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -mrelax -Wall -mmcu=atmega2561 -c -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" 


Usually multiple packages need to be installed. Here an exemplary list for the Ubuntu distribution:

gcc-avr	Compiler & Assembler
binutils-avr	Linker and some other useful tools
gdb-avr	Debugger for AVR Targets
avr-libc	The standard C library for the AVR series, including the required include files
avrdude	Programm to upload applications onto a AVR device

WINDOWS

Descargar AMTEL- STUDIO
