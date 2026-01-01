# OpenFirmwareMower

test firmware to confirm the different functions

# Prerequisites

Clone this repo and pull submodules, if the repo is already cloned, you just need to pull the submodules by executing:

`git submodule update --init --recursive`.

Install cross-compiler

```bash

sudo dnf update -y
sudo dnf install arm-none-eabi-gcc-cs -y
sudo dnf install arm-none-eabi-newlib -y
sudo dnf install gcc-aarch64-linux-gnu -y
sudo dnf install gcc-arm-linux-gnu -y

```

# Build

```bash

mkdir build && cd build
cmake .. && make

```