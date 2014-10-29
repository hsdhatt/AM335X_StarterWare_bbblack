AM335X_StarterWare_bbblack
==========================
This repository contains the AM335X_StarterWare port to run on BeagleBone Black. 
I have tested it on REVA5C, as it is the only board I have.

1. The key difference is only regarding the DDR initialization. 
   Therefore, the only file that has been modified is 
   bootloader/src/armv7a/am335x/bl_platform.c 

2. Alongwith that, need to add a "bb_black" CFLAG in the 
   build/armv7a/gcc/am335x/beaglebone/bootloader/makefile
   to compile specifically for the beaglebone black. 

Compilation
===========

cd build/armv7a/gcc/am335x/beaglebone/bootloader/
make clean
make release

Booting
=======

Copy to the primary partition the file
../../../../../..//binary/armv7a/gcc/am335x/beaglebone/bootloader/Release_MMCSD/boot_ti.bin
and rename it to MLO. This renaming is crucial because the Public ROM Code on the 
AM335X SoC looks for a bootloader named MLO. The exact meaning of the word "MLO" remains 
unknown. 

In order to boot using MMCSD card,
hold down the S2 and then plug in power(either DC or USB). 

Enjoy playing with the starterware applications on Beaglebone Black.

NOTE
====
I have only tested the MLO using the MMCSD boot option.

