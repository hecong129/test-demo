#!/bin/sh


TOPDIR=`pwd`
ROOT_DIR=$TOPDIR/../
APPDIR=$ROOT_DIR/app
APPBIN=$APPDIR/mainapp/BUILD_mainapp_EXEC/mainapp
KERNELDIR=$ROOT_DIR/kernel
KERNELBIN=$KERNELDIR/build/arch/arm/boot/zImage
ROOTFSDIR=$ROOT_DIR/rootfs
ROOTFSBIN=$ROOTFSDIR/platform/output/2k_root.yaffs2
ROOTFSBIN_WR=$ROOTFSDIR/platform_wr/output/2k_root_wr.yaffs2
RESOURCEDIR=$ROOT_DIR/resource
BOOLTOORDIR=$TOPDIR/burntool_kit_V5.0.48
FASTBOOTBIN=$BOOLTOORDIR/Spinandboot_37xxc.bin

# add by kx
RESOURCEPAGE_1=$RESOURCEDIR/img/comm/page_1.bmp
ROOTFSLIB=$ROOTFSDIR/platform/rootfs/rootfs/lib
SOFTWAREINI=$TOPDIR/software.ini
THIRDPARTY=$APPDIR/thirdparty
ANYKALIB=$THIRDPARTY/anykalib/*
ROOTFSVERSION=$ROOTFSDIR/platform_wr/rootfs/rootfs/

# __readINI [配置文件路径+名称] [节点名] [键值]
function __readINI() {
	INIFILE=$1; SECTION=$2; ITEM=$3
	_readIni=`awk -F '=' '/\['$SECTION'\]/{a=1}a==1&&$1~/'$ITEM'/{print $2;exit}' $INIFILE`
	echo ${_readIni}
}

# 使用方法:
# GET VERSION
_AK_VER=( $( __readINI $SOFTWAREINI gobal AK_VER ) ) 
#echo ${_AK_VER}

AK_VERSION=9208SC_AK_${_AK_VER}.zip
AK_UPDATE=9208SC_UPDATE_AK_${_AK_VER}.zip
#echo $AK_VERSION
#echo $AK_UPDATE
#sleep 10
# add end

AK_FASTBOOT=1
AK_KERNEL=1
AK_LOGO=1
AK_APP=1
BUILD_UPZIP=0

source ./"update.ini"

if [ $BUILD_UPZIP = 1 ]; then
echo "make update.zip ."
else
echo "\033[31m load update configs, shold use green cmd as follows:\033[0m"
echo "\033[32m source build.sh \033[0m"
echo "\033[31m will use dafault config to build after 4s < CTRL+C to end ? >\033[0m "
sleep 4
fi

# **************************************************
echo "building app..."
cd $APPDIR
make clean
make

# **************************************************
echo "building kernel..."
cd $KERNELDIR
if [ -d "build" ]; then
	rm -rf "build"	
fi
if [ -d "kernel" ]; then
	rm -rf "kernel"	
fi
tar -zxvf kernel_V1.1.02.tar.gz
cd kernel
mkdir ../build
make O=../build 9208sc_defconfig
make O=../build

# **************************************************
echo "makeing rootfs..."
cd $ROOTFSDIR
if [ -d "platform" ]; then
	rm -rf "platform"	
fi
tar -zxvf aimer37c_rootfs_V1.2.00.tar.gz
cd platform
#mkdir ./rootfs/rootfs/usr/mainapp
cp $APPBIN ./rootfs/rootfs/usr/mainapp
cp -rf $RESOURCEDIR ./rootfs/rootfs/usr/mainapp

# add by kx
cd $THIRDPARTY
if [ -d "antkalib" ]; then
	rm -rf "anykalib"
fi
tar -zxvf anykalib.tar.gz
cp -rf $ANYKALIB $ROOTFSLIB
# add end


cd $ROOTFSDIR/platform
./build.sh
# **************************************************
echo "makeing rootfs_wr..."
cd $ROOTFSDIR
if [ -d "platform_wr" ]; then
	rm -rf "platform_wr"	
fi
tar -zxvf aimer37c_rootfs_wr_V1.2.00.tar.gz

# add by kx
cp -rf $TOPDIR/software.ini $ROOTFSVERSION
# add end

cd platform_wr
./build.sh

# **************************************************
cd $BOOLTOORDIR
cp $ROOTFSBIN ./
cp $ROOTFSBIN_WR ./
cp $KERNELBIN ./
# **************************************************

echo "make update.zip .."

if [ -d "tmp_up" ]; then
	rm -rf "tmp_up"	
fi
mkdir tmp_up
## fastbbot
echo " FASTBOOT = " $AK_FASTBOOT
if [ $AK_KERNEL = 1 ]; then
	echo "cp fastboot"	
	cp $FASTBOOTBIN tmp_up
fi

## logo

# add by kx   update logo
cp -rf $RESOURCEPAGE_1 $BOOLTOORDIR
# add end

echo " LOGO = " $AK_LOGO
if [ $AK_KERNEL = 1 ]; then
	cp $BOOLTOORDIR/page_1.bmp tmp_up

fi
## kernel
echo " KERNEL = " $AK_KERNEL
if [ $AK_KERNEL = 1 ]; then
	cp $KERNELBIN tmp_up
fi
## app
echo " APP = " $AK_APP
if [ $AK_APP = 1 ]; then
	cp $APPBIN tmp_up
fi

echo " Update Shell"
if [ -f "update.sh" ]; then
	cp update.sh  tmp_up
fi

echo "make update.zip..."

if [ -d "update.zip" ]; then
	rm -rf "update.zip"	
fi
if [ -f "checkup.sh" ]; then
	cp -vf checkup.sh ./tmp	
fi
tar -cvf update.zip ./tmp_up
ls -il tmp_up
echo "make update.zip end"

# add by kx
zip -r $AK_UPDATE update.zip
mv $AK_UPDATE ..
# add end

cd ..

if [ -d "build.tar.gz" ]; then
	rm -rf "build.tar.gz"	
fi
#tar -czvf build.tar.gz burntool_kit_V5.0.48

# add by kx
zip -r $AK_VERSION burntool_kit_V5.0.48
# add end

#检查文件完整性，to be done
echo "finish................"
# ****************** finish ************************
