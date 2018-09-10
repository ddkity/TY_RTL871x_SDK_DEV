#!/bin/sh
if [ -z "$1" ];then
        echo "please input the app bin name(no suffix \".bin\")!!!"
        exit 1
else
        APP_BIN_NAME=$1
fi

if [ -z "$2" ];then
        echo "please input the app sw version(for example:the format is "1.1.1")!!!"
        exit 1
else
        USER_SW_VER=$2
fi
#mkdir tuya_user/$APP_BIN_NAME/output/$USER_SW_VER
# $3 compile parameter command as user set,for example clean and so on.
USER_DEF_CMD=$3

echo ""
echo "start..."
echo ""
set -e
make APP_BIN_NAME=$APP_BIN_NAME USER_SW_VER=$USER_SW_VER $USER_DEF_CMD -C ./GCC-RELEASE
if [ -z "$3" ];then
		make APP_BIN_NAME=$APP_BIN_NAME USER_SW_VER=$USER_SW_VER ota_idx=2 -C ./GCC-RELEASE
fi
