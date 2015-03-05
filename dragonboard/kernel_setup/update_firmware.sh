#!/bin/bash
Red='\e[0;31m'; BRed='\e[1;31m'; URed='\e[4;31m';

BINARY_BLOB='Inforce-IFC6410_Android_BSP_Rel_V_1.5.zip'
PROP_FILE='proprietary.tar.gz'

if [ -e $PROP_FILE ] && [ -e $BINARY_BLOB ]
then 
    mkdir image
    sudo mount -o loop firmware-ifc6410-${VERSION}.img image
    sudo tar xzf $PROP_FILE -C image/ --strip-components 8 proprietary/prebuilt/target/product/msm8960/system/etc/firmware/
    sudo umount image
    rmdir image
else 
    echo -e "$BRed:ERROR: Propriatary file and/or binary blob was not found"
fi


