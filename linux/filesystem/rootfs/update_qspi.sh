#/bin/sh

# the purpose of this script is to flash u-boot, Linux, 
# and the Linux ramdisk to QSPI flash

path=$1

if [ $# -ne 1 ] ; then
	echo "usage: update_qspi.sh <path to images>"
	exit
fi

printf "\nWriting BOOT.bin Image To QSPI Flash\n\n"

flashcp -v $path/BOOT.bin /dev/mtd0

printf "\nWriting Linux Image To QSPI Flash\n\n"

flashcp -v $path/uImage /dev/mtd1

printf "\nWriting Device Tree To QSPI Flash\n\n"

flashcp -v $path/devicetree.dtb /dev/mtd2

printf "\nWriting Ramdisk Image To QSPI Flash\n\n"

flashcp -v $path/uramdisk.image.gz /dev/mtd3
