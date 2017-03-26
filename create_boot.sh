#/bin/bash

if [ $# != 1 ]
then
	echo "usage:./create_boot.sh [bif file path]"
	exit 1
fi

bootgen -image $1 -o i BOOT.bin -w on

