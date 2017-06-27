#!/bin/sh

#echo "Remove 64 bytes file head"
#dd if=uramdisk.image.gz of=ramdisk.image.gz bs=64 skip=1

#echo "general ramdisk.image"
#gunzip ramdisk.image.gz

#mkdir -p tmp

#mount -o loop,rw ramdisk.image tmp

#cp rootfs/* tmp -r

#cd tmp
#sync
#cd ..


genext2fs -b 160384 -N 1000 -d $1 ramdisk.image
if [ $? -ne 0 ]; then
	echo "general ramdisk.image failed! make sure your PC has installed genext2fs and check the param of genext2fs"
	exit 1
fi

echo "general ramdisk.image.gz"
gzip -9 ramdisk.image

if [ $? -ne 0 ]; then
	echo "general ramdisk.image.gz failed!"
	exit 1
else
	echo "add file head"
	mkimage -A arm -T ramdisk -C gzip -n Ramdisk -d ramdisk.image.gz uramdisk.image.gz
	rm ramdisk.image.gz
    cp uramdisk.image.gz ../../amp_system/linux_image/
fi
