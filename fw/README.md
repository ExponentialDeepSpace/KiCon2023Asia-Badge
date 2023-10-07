# KiCon 2023 Asia Badge Firmware

## Preparation
git clone --recursive git@github.com:ExponentialDeepSpace/KiCon2023Asia-Badge.git

## Build
1. Download and Install [ARM GNU Toolchain](https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain)
1. Setup Toolchain environment path
1. make

## Debug

1. Install [pyOCD](pyocd.io/) ( or [openocd](openocd.org) with customization for N32L406 )
1. install `gdb-multiarch` ( or some gdb which supports ARM Cortex-M and python scripting )
1. pyocd gdbserver
1. gdb-multiarch

### Comments
I cannot get gdb from ARM GNU Toolchain arm-gnu-toolchain-11.3.rel1-x86_64-arm-none-eabi works with PyCortexMDebug (failed to load math extension from custom built Python3.8.11, like this issue: [1](https://github.com/clearlinux/distribution/issues/2234)). I installed gdb-multiarch shipped with Debian 11.

## License
KiCon 2023 Asia Badge Firmware is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

KiCon 2023 Asia Badge Firmware is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with KiCon 2023 Asia Badge Firmware. If not, see <https://www.gnu.org/licenses/>. 
