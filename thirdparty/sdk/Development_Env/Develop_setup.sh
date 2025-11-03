#!/bin/sh

OS_NAME='linux'
OS_ARCH=`arch`
if [ "$OS_ARCH"x = "x86_64"x ]; then
    ARCHITECTURE='x86_64'
    PLATFORM='x64'
fi
if [ "$OS_ARCH"x = "i686"x ]; then
    ARCHITECTURE='x86_64'
    PLATFORM='x86'
fi
if [ "$OS_ARCH"x = "aarch64"x ]; then
    ARCHITECTURE='arm'
    PLATFORM='aarch64'
fi

SETUP_ALLOW='no'

if [ "$ARCHITECTURE"x = "x86_64"x ]; then
    if [ "$PLATFORM"x = "x64"x ]; then
        SETUP_ALLOW='yes'
    fi
    if [ "$PLATFORM"x = "x86"x ]; then
        SETUP_ALLOW='yes'
    fi
fi

if [ "$ARCHITECTURE"x = "arm"x ]; then
    if [ "$PLATFORM"x = "aarch64"x ]; then
        SETUP_ALLOW='yes'
    fi
fi

if [ "$SETUP_ALLOW" = "no" ]; then
    echo "Error: Sdk develop setup script do not support current architecture & platform."
    return -1
fi

if [ ! -n "$1" ]; then
    echo "Error: Sdk script do not have enough parameters,please use Develop_setup.sh --help for more information."
    return -1
fi
if [ "$1x" != "--helpx" ] && [ "$1x" != "--installx" ] && [ "$1x" != "--uninstallx" ] && [ "$1x" != "--setupx" ]; then
    echo "Error: Sdk setup script has invalid parameter,please use Develop_setup.sh --help for more information."
    return -1
fi
_mode=$1
if [ "$2x" = "xenomaix" ]; then
    _os_type='xenomai'
    if [ "${_os_type}x" = "xenomaix" ]; then
        if [ ! -n "$XENOMAI_ROOT" ]; then
            echo "Error: You do not set XENOMAI_ROOT, /usr/xenomai will be used."
            export XENOMAI_ROOT=/usr/xenomai
        fi
        if [ ! -d $XENOMAI_ROOT ]; then
            echo "Error: Can not found xenomai root: $XENOMAI_ROOT."
            return -1
        fi
    fi
else
    _os_type='linux'
fi

if [ "${_mode}x" = "--helpx" ]; then
    echo 'Usage: Develop_setup.sh mode [type]'
    echo 'mode could be the follow value:'
    echo '  --help                  display help'
    echo '  --install                install this package for current user'
    echo '  --uninstall            uninstall this package for current user'
    echo '  --setup                setup environment'
    echo 'type is option value and it could be the follow value: linux or xenomai'
elif [ "${_mode}x" = "--installx" ]; then
    workdir=$(cd $(dirname `readlink -f $0`); pwd)
    installLine=`grep GENERIC_DEVELOPMENT_ENV_PLACEHOLDER ~/.bashrc`
    if [ ! -n "$installLine" ]; then
        echo . $workdir/Develop_setup.sh --setup $_os_type >> ~/.bashrc
    else
        sed -i -e "s#GENERIC_DEVELOPMENT_ENV_PLACEHOLDER#. $workdir/Develop_setup.sh --setup $_os_type#g" ~/.bashrc
    fi
elif [ "${_mode}x" = "--uninstallx" ]; then
    workdir=$(cd $(dirname `readlink -f $0`); pwd)
    sed -i -e "s#. $workdir/Develop_setup.sh --setup $_os_type#GENERIC_DEVELOPMENT_ENV_PLACEHOLDER#g" ~/.bashrc
else
    workdir=$(cd $(dirname $BASH_SOURCE[0]); pwd)
    export GENERIC_DEVELOPMENT_ENVIRONMENT_ROOT=$workdir
    export OS_NAME
    export PLATFORM
    export ARCHITECTURE
    if [ "${_os_type}x" = "xenomaix" ]; then
        export USE_XENOMAI_ENV=1
    else
        unset USE_XENOMAI_ENV
    fi
fi
