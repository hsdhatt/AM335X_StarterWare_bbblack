AM335X_StarterWare_bbblack
==========================
This repository contains the AM335X_StarterWare port to run on BeagleBone Black. 
I have tested it on REVA5C, as it is the only board I have.

The key difference is only regarding the DDR initialization. Therefore, the only file that needs modification is
bl_platform.c
