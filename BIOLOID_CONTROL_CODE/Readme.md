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


Para compilar el skelteton tracker
instalar

Necesitan igualmente el driver de la kinect



libgtk-3-dev
libmutter-dev
libwnck-3-dev
libgnome-menu-3-dev
libupower-glib-dev
gobject-introspection
Also, I am not sure these two packages are needed because they already installed in my system:

libglib3.0-cil-dev
libgtk3.0-cil-dev


./configure --disable-shared --enable-staticfatal error: gfreenect.h


git clone https://github.com/OpenKinect/libfreenect
mkdir build
cmake /L ..
make
make install
sudo apt-get install gtk-doc-tools


GFreenect:
wget https://github.com/elima/GFreenect/archive/master.zip
unzip master.zip 
cd GFreenect-master
./autogen.sh
./configure --prefix=/usr
make
make install

Skeltrack:
wget https://github.com/joaquimrocha/Skeltrack/archive/master.zip
unzip master.zip 
cd Skeltrack-master
./autogen.sh 
./configure --prefix=/usr --enable-examples=yes --enable-tests=yes
make
make install

install cairo
sudo apt-get install libcairo2-dev
