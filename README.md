# Quatro3D
A simple 3D geometry library for 3D vectors (point translations) and quaternions (rotations), built for a Drone project, but not limited in its usage.

## Setup instructions

**Download source code :**
 * `git clone https://github.com/RohanVDvivedi/Quatro3D.git`

**Build from source :**
 * `cd Quatro3D`
 * `make clean all`

**Install from the build :**
 * `sudo make install`
 * ***Once you have installed from source, you may discard the build by*** `make clean`

## Using The library
 * add `-lquatro3d -lm` linker flag, while compiling your application
 * do not forget to include appropriate public api headers as and when needed. this includes
   * `#include<quatro3d/quatro.h>`

## Instructions for uninstalling library

**Uninstall :**
 * `cd Quatro3D`
 * `sudo make uninstall`
