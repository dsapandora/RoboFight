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

Grab the packages you'll need to compile libusb and libfreenect:

sudo apt-get install git cmake build-essential
sudo apt-get install freeglut3-dev libxmu-dev libxi-dev
sudo apt-get install libudev-dev

Remove the existing libusb, if it's there:

sudo apt-get remove libusb-1.0-0-dev

Grab the sources for libusb-1.0.18:

wget http://sourceforge.net/projects/libusb/files/libusb-1.0/libusb-1.0.18/libusb-1.0.18.tar.bz2
tar -xvf libusb-1.0.18.tar.bz2

Build and install the updated libusb:

cd libusb-1.0.18/
./configure --prefix=/usr --disable-static
make
sudo make install

Ugly hack to convince libfreenect to use the new libusb library:

sudo ln -s /usr/lib/libusb-1.0.so /usr/lib/arm-linux-gnueabihf/libusb-1.0.so

Then you should be able to build libfreenect.

git clone https://github.com/OpenKinect/libfreenect
mkdir build
cmake /L ..
make
make install
sudo apt-get install gtk-doc-tools


GFreenect:
wget https://github.com/elima/GFreenect/archive/master.zip
unzip master.zip Grab the packages you'll need to compile libusb and libfreenect:

sudo apt-get install git cmake build-essential
sudo apt-get install freeglut3-dev libxmu-dev libxi-dev
sudo apt-get install libudev-dev

Remove the existing libusb, if it's there:

sudo apt-get remove libusb-1.0-0-dev

Grab the sources for libusb-1.0.18:

wget http://sourceforge.net/projects/libusb/files/libusb-1.0/libusb-1.0.18/libusb-1.0.18.tar.bz2
tar -xvf libusb-1.0.18.tar.bz2

Build and install the updated libusb:

cd libusb-1.0.18/
./configure --prefix=/usr --disable-static
make
sudo make install

Ugly hack to convince libfreenect to use the new libusb library:

sudo ln -s /usr/lib/libusb-1.0.so /usr/lib/arm-linux-gnueabihf/libusb-1.0.so

Then you should be able to build libfreenect.

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
