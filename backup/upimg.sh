#!/bin/sh

# **************************************************
UPPATH=/mnt/sdcard
UPFILE=$UPPATH/update.zip
UPPATH_TAR=$UPPATH/tmp_up
UPIMGFILE=$UPPATH_TAR/upimg.sh
UPLOGO=$UPPATH_TAR/page_1.bmp
UPKERNEL=$UPPATH_TAR/zImage
UPFASTBOOT=$UPPATH_TAR/Spinandboot_37xxc.bin
UPMAINAPP=$UPPATH_TAR/mainapp
UPRESOURCE=$UPPATH_TAR/resource
UPSHELL=$UPPATH_TAR/update.sh
OLDMAINPP=/usr/mainapp/mainapp
OLDRESOURCE=/usr/mainapp/resource


if [ $1 = $1 ];then
	echo "exece update img ..."
	cd $UPPATH
	if [ -f "$UPLOGO" ];then
		echo "update logo"
		echo "1" > _logo_
		
			if [ -f "$UPKERNEL" ];then
				echo "and update kernel"
				echo "1" > _kernel_
					if [ -f "$UPFASTBOOT" ];then
						echo "1" > _boot_
						updater local L=$UPPATH_TAR/page_1.bmp  K=$UPPATH_TAR/zImage B=$UPPATH_TAR/Spinandboot_37xxc.bin
					else	
						updater local L=$UPPATH_TAR/page_1.bmp  K=$UPPATH_TAR/zImage
					fi
			else
					if [ -f "$UPFASTBOOT" ];then
						echo "1" > _boot_
						updater local L=$UPPATH_TAR/page_1.bmp  B=$UPPATH_TAR/Spinandboot_37xxc.bin
					else	
						updater local L=$UPPATH_TAR/page_1.bmp  
					fi
			fi	

	else
		if [ -f "$UPKERNEL" ];then
			echo "update kernel"
			echo "1" > _kernel_
			if [ -f "$UPFASTBOOT" ];then
				echo "1" > _boot_
				updater local B=$UPPATH_TAR/Spinandboot_37xxc.bin K=$UPPATH_TAR/zImage
			else	
				updater local K=$UPPATH_TAR/zImage
			fi	
		else
			if [ -f "$UPFASTBOOT" ];then
				echo "1" > _boot_
				updater local B=$UPPATH_TAR/Spinandboot_37xxc.bin
			fi
		fi	
	fi
		
	if [ -f "$UPMAINAPP" ];then
		echo "update app"
		cp -rf $UPMAINAPP /usr/mainapp/mainapp
		echo "1" > _mainapp_
	fi
	
	if [ -d "$UPRESOURCE" ];then
		echo "update resource"
		rm -rf /usr/mainapp/resource
		cp -avrf $UPRESOURCE /usr/mainapp/
		echo "1" > _resource_
	fi
	
	if [ -f "$UPSHELL" ];then
		echo "exec other update shell"
		$UPSHELL
	fi

else
	echo "update.zip not find"

fi

# **************************************************
echo " update  finish...reboot..."
# ****************** finish ************************
