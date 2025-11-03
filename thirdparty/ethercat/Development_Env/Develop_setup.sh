#!/bin/sh

if [ "$1x" = "--installx" ]; then
    if [ "$GENERIC_DEVELOPMENT_ENVIRONMENT_ROOT"x = ""x ]; then
        echo Error: you should install sdk development environment first. 
        exit -1
    fi
fi

SETUP_ALLOW='no'

if [ "$ARCHITECTURE"x = "x86_64"x ]; then
    if [ "$PLATFORM"x = "x64"x ]; then
        SETUP_ALLOW='yes'
    fi
fi

if [ "$ARCHITECTURE"x = "arm"x ]; then
    if [ "$PLATFORM"x = "x86"x ]; then
        SETUP_ALLOW='yes'
    fi
fi

if [ "$SETUP_ALLOW" = "no" ]; then
    echo "Error: Ethercat develop setup script do not support current architecture & platform."
    return -1
fi

if [ ! -n "$1" ]; then
    echo "Error: Ethercat script do not have enough parameters,please use Develop_setup.sh --help for more information."
    return -1
fi
if [ "$1x" != "--helpx" ] && [ "$1x" != "--installx" ] && [ "$1x" != "--uninstallx" ] && [ "$1x" != "--setupx" ]; then
    echo "Error: Ethercat setup script has invalid parameter,please use Develop_setup.sh --help for more information."
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

if [ "$3x" = "1.6.0x" ]; then
    _igh_ver='1.6.0'
else
    _igh_ver='1.5.2'
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
    installLine=`grep ETHERCAT_DEVELOPMENT_ENV_PLACEHOLDER ~/.bashrc`
    if [ ! -n "$installLine" ]; then
        echo . $workdir/Develop_setup.sh --setup $_os_type $_igh_ver >> ~/.bashrc
    else
        sed -i -e "s#ETHERCAT_DEVELOPMENT_ENV_PLACEHOLDER#. $workdir/Develop_setup.sh --setup $_os_type $_igh_ver#g" ~/.bashrc
    fi
elif [ "${_mode}x" = "--uninstallx" ]; then
    workdir=$(cd $(dirname `readlink -f $0`); pwd)
    sed -i -e "s#. $workdir/Develop_setup.sh --setup $_os_type $_igh_ver#ETHERCAT_DEVELOPMENT_ENV_PLACEHOLDER#g" ~/.bashrc
else
    workdir=$(cd $(dirname $BASH_SOURCE[0]); pwd)
    export ETHERCAT_DEVELOPMENT_ENVIRONMENT_ROOT=$workdir
fi

