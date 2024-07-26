#!/bin/sh

############################
# OB_DEPLOY UPGRADE SCRIPT #
############################

TMP_DIR=/tmp
TMP_FILE_NAME=$TMP_DIR/ob_deploy_upgrade.tar.gz
UPGRADE_DIR=$TMP_DIR/ob_deploy_upgrade

PRE_UPGRADE_SCRIPT=$UPGRADE_DIR/pre_upgrade.sh
UPGRADE_SCRIPT=$UPGRADE_DIR/upgrade.sh
POST_UPGRADE_SCRIPT=$UPGRADE_DIR/post_upgrade.sh

beginswith() { case $2 in "$1"*) true;; *) false;; esac; }

PACKAGE_URL=$1

if [ -z $PACKAGE_URL ]; then
    echo "usage: $0 <url>"
    exit 1
fi

# Clear previous upgrade package
if [ -e $TMP_FILE_NAME ]; then
    rm $TMP_FILE_NAME
fi

# Prepare current upgrade package
if beginswith http:// $PACKAGE_URL || beginswith https:// $PACKAGE_URL ; then
    echo "Downloading upgrade package..."

    wget $PACKAGE_URL -O $TMP_FILE_NAME

    if [ $? -gt 0 ]; then
        echo "Error when download package."
        exit 1
    fi
else
    if [ ! -e $PACKAGE_URL ]; then
        echo "$PACKAGE_URL does not exists."
        exit 1
    fi

    echo "Copying upgrade package..."
    cp $PACKAGE_URL $TMP_FILE_NAME

    if [ $? -gt 0 ]; then
        echo "Error when copy upgrade package."
        exit 1
    fi
fi

# Check upgrade package exists.
if [ ! -e $TMP_FILE_NAME ]; then
    echo "$TMP_FILE_NAME does not exists."
    exit 1
fi

# Prepare upgrade package dir
if [ -d $UPGRADE_DIR ]; then
    rm -rf $UPGRADE_DIR
fi
mkdir -p $UPGRADE_DIR

# Decompress
echo "Decompressing upgrade package."
tar -C $UPGRADE_DIR -xf $TMP_FILE_NAME

if [ $? -gt 0 ]; then
    echo "Error when decompress upgrade package."
    exit 1
fi

cd $UPGRADE_DIR

# check upgrade script exists
if [ ! -e $UPGRADE_SCRIPT ]; then
    echo "Upgrade script $UPGRADE_SCRIPT does not exists, exit!"
    exit 1
fi

# execute upgrade scripts
if [ -e $PRE_UPGRADE_SCRIPT ]; then 
    echo "Execute PRE upgrade script."
    sh $PRE_UPGRADE_SCRIPT

    if [ $? -gt 0 ]; then
        echo "Error when execute pre upgrade scripts."
        exit 1
    fi
fi

echo "Execute upgrade script."
sh $UPGRADE_SCRIPT

if [ $? -gt 0 ]; then
    echo "Error when execute upgrade scripts."
    exit 1
fi

if [ -e $POST_UPGRADE_SCRIPT ]; then 
    echo "Execute post upgrade script."
    sh $POST_UPGRADE_SCRIPT

    if [ $? -gt 0 ]; then
        echo "Error when execute post upgrade scripts."
        exit 1
    fi
fi

echo "Upgrade success!!"
