# stm32f072b-freertos
Test task with libopencm3, FreeRTOS and freemodbus for STM32F072B-DISCO board.

Manual

Step Zero) Download and install toolchain.

Go to https://launchpad.net/gcc-arm-embedded, or click direct download link https://launchpad.net/gcc-arm-embedded/5.0/5-2016-q3-update/+download/gcc-arm-none-eabi-5_4-2016q3-20160926-linux.tar.bz2
Unpack it, for example to /opt directory.

I'm using Linux x64, all steps are tested here.

Test your toolchain:
$ /opt/gcc-arm-none-eabi-5_4-2016q3/bin/arm-none-eabi-gcc --version
arm-none-eabi-gcc (GNU Tools for ARM Embedded Processors) 5.4.1 20160919 (release) [ARM/embedded-5-branch revision 240496]
Copyright (C) 2015 Free Software Foundation, Inc.
This is free software; see the source for copying conditions.  There is NO
warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

You should have output similar to mine.


Step 1) Install Eclipse CDT IDE

Download and install Eclipse CDT from official website (I'm using Eclipse Neon).
Launch and select name for your workspace, e.g. "workspace-new".

Search and install GNU ARM Eclipse plugin http://gnuarmeclipse.github.io/
In Eclipse, go Help->Eclipse marketplace, find and install GNU ARM Eclipse plugin

Download OpenOCD from https://github.com/gnuarmeclipse/openocd/releases
Follow http://gnuarmeclipse.github.io/openocd/install/ installation instructions.

Step 2) Download libraries and FreeRTOS sources

Download FreeRTOS from official website http://www.freertos.org/a00104.html or directly from SourceForge.
I used FreeRTOS 9.0.0 https://sourceforge.net/projects/freertos/files/latest/download?source=files
Unzip archive contents somewhere to your home dir, e.g. ~/libraries/

Download (clone) freemodbus from GitHub https://github.com/chrismerck/freemodbus to ~/libraries/

Download (clone) libopencm3 from GitHub https://github.com/libopencm3/libopencm3 to ~/libraries/

Now all library sources are downloaded.
~/libraries/FreeRTOSv9.0.0
~/libraries/freemodbus
~/libraries/libopencm3

In terminal, TEMPORARY add toolchain to your system PATH
$ PATH=/opt/gcc-arm-none-eabi-5_4-2016q3/bin:$PATH
This will be valid only for current terminal session.
Test toolchain again: $ arm-none-eabi-gcc --version, now from any directory you must see that it is available and working.

Go to ~/libraries/libopencm3 and run make.
Library will be built, and you will get libopencm3/lib/libopencm3_stm32f0.a
Running arm-none-eabi-readelf -A ~/libraries/libopencm3/lib/libopencm3_stm32f0.a will show some library contents :)

Step 3) Importing stm32f072b-freertos project from GitHub to Eclipse
In Eclipse, with opened workspace, File->Inport->Git->Projects from Git enter URI https://github.com/dellangard/stm32f072b-freertos.git and other params to import project.
IDE will clone it to local repo.
Now you must edit paths in Eclipse to add external libraries/sources to project.

Step 4) Modifying library paths in Eclipse project
In Eclipse, go to Project properties - > linked resources, Path variables tab.
Edit FREERTOS_PATH, FREEMODBUS_PATH, LIBOPENCM3_PATH values, for example:
FREERTOS_PATH /home/your_username/libraries/FreeRTOSv9.0.0/FreeRTOS
FREEMODBUS_PATH /home/your_username/libraries/freemodbus
LIBOPENCM3_PATH /home/your_username/libraries/libopencm3

After that, go to C/C++ Build -> Build variables. Find the same variables, and assign them corresponding path values.

Modify C/C++ Build -> Tools Paths -> Toolchain folder to your toolchain path (/opt/gcc-arm-none-eabi-5_4-2016q3/bin)
Toolchain installing and setting it up in Eclipse is described here http://gnuarmeclipse.github.io/toolchain/install/

Step 5) Project Build
After connecting libraries to project, run Project->Clean and Project->Build.
