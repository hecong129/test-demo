#!/bin/sh

# **************************************************
UPPATH=/mnt/sdcard
UPFILE=$UPPATH/update.zip
UPIMGFILE=$UPPATH/upimg.sh
UPPATH_TAR=$UPPATH/tmp_up
UPLOGO=$UPPATH_TAR/page_1.bmp
UPKERNEL=$UPPATH_TAR/zImage
UPFASTBOOT=$UPPATH_TAR/Spinandboot_37xxc.bin
UPMAINAPP=$UPPATH_TAR/mainapp
UPRESOURCE=$UPPATH_TAR/resource
UPSHELL=$UPPATH_TAR/update.sh
OLDMAINPP=/usr/mainapp/mainapp
OLDRESOURCE=/usr/mainapp/resource


if [ -f "$UPFILE" ];then
	echo "update.zip find update ..."
	mount -o remount rw /
	cd $UPPATH
	if [ -f $UPPATH_TAR ]; then
		echo "del tmp_up"
		rm -rf $UPPATH_TAR
	fi
	tar -xvf $UPFILE
	chmod 777 $UPPATH_TAR -R
	
	if [ -f $UPIMGFILE ]; then
		$UPIMGFILE
	fi
fi

