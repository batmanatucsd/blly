#!/bin/bash
RCol='\e[0m'  # Text Reset
Red='\e[0;31m'; BRed='\e[1;31m'; URed='\e[4;31m';
Gre='\e[0;32m'; BGre='\e[1;32m'; UGre='\e[4;32m';
Cya='\e[0;36m'; BCya='\e[1;36m'; UCya='\e[4;36m';
Wht='\e[0;37m'; BWht='\e[1;37m'; UWht='\e[4;37m';
BOOT_IMG_FILE='boot-ifc6410-20140526-15.img.gz'
FIRM_IMG_FILE='firmware-ifc6410-20140526-15.img.gz'
UBUNTU_IMG_FILE='linaro-trusty-gnome-ifc6410-20140526-15.img.gz'
KERNEL_REPO='https://git.linaro.org/landing-teams/working/qualcomm/kernel.git'
TOOLCHAIN_NAME='gcc-linaro-arm-linux-gnueabihf-4.9-2014.05_linux.tar.xz'
TOOLCHAIN_FILE_NAME='gcc-linaro-arm-linux-gnueabihf-4.9-2014.05_linux'

function header {
    echo -e "\n$Gre#########################################"
    echo -e "## $BWht$1"
    echo -e "$Gre########################################$RCol"
}

function error {
    echo -e "\n$Red#######################################"
    echo -e "## $BWht$1"
    echo -e "$Gre########################################$RCol"
}

echo -e "\n$Gre##########################################"
echo -e "## $BCya AUTOMATIC Dragonboard Setup Script $Gre ##"
echo -e "##########################################$RCol"

header "Setting up build directory"
mkdir -p kernel_build
cd kernel_build
mkdir -p build
echo -e "Done"

header "Downloading images..."
mkdir -p raw_image
cd raw_image
wget -nc http://releases.linaro.org/14.05/ubuntu/ifc6410/$BOOT_IMG_FILE
wget -nc http://releases.linaro.org/14.05/ubuntu/ifc6410/$FIRM_IMG_FILE
wget -nc http://releases.linaro.org/14.05/ubuntu/ifc6410/$UBUNTU_IMG_FILE
cd ../
cp raw_image/*.img.gz .
echo -e "Done"

header "Extracting images..."
gunzip *.img.gz
echo -e "Done"

header "Start Rebuilding The Linux Kernel"
header "Cloning the Linaro Trusty repo"
git clone https://git.linaro.org/landing-teams/working/qualcomm/kernel.git linaro-trusty
cd linaro-trusty
git checkout ubuntu-ifc6410-14.05
cd ../
header "Grabbing the copy of the toolchain"
wget -nc http://releases.linaro.org/14.05/components/toolchain/binaries/$TOOLCHAIN_NAME
tar xf $TOOLCHAIN_NAME
TOOLCHAIN_PATH="$(pwd)/$TOOLCHAIN_FILE_NAME"
cd linaro-trusty

header "Set the tolchain to use the default configuration for the IFC6410"
LD_LIBRARY_PATH=$TOOLCHAIN_PATH/arm-linux-gnueabihf/lib make ARCH=arm CROSS_COMPILE=$TOOLCHAIN_PATH/bin/arm-linux-gnueabihf- defconfig ifc6410_defconfig

header "Starting kernel configuration menu..."
LD_LIBRARY_PATH=$TOOLCHAIN_PATH/arm-linux-gnueabihf/lib make ARCH=arm CROSS_COMPILE=$TOOLCHAIN_PATH/bin/arm-linux-gnueabihf- menuconfig

header "Building the kernel image..."
LD_LIBRARY_PATH=$TOOLCHAIN_PATH/arm-linux-gnueabihf/lib make ARCH=arm CROSS_COMPILE=$TOOLCHAIN_PATH/bin/arm-linux-gnueabihf- -j8 zImage

header "DONEEEEEEE"
