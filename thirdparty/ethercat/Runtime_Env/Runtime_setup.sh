#!/bin/sh

if [ "$1x" = "--installx" ]; then
    if [ "$GENERIC_RUNTIME_ENVIRONMENT_ROOT"x = ""x ]; then
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
    echo "Error: Ethercat runtime setup script do not support current architecture & platform."
    return -1
fi

if [ ! -n "$1" ]; then
    echo "Error: Ethercat setup script do not have enough parameters,please use Runtime_setup.sh --help for more information."
    return -1
fi
if [ "$1x" != "--helpx" ] && [ "$1x" != "--installx" ] && [ "$1x" != "--uninstallx" ] && [ "$1x" != "--setupx" ]; then
    echo "Error: Ethercat setup script has invalid parameter,please use Runtime_setup.sh --help for more information."
    return -1
fi
_mode=$1
if [ "$2x" = "xenomaix" ]; then
    _os_type='xenomai'
    _igh_os='xenomai'
else
    _os_type='linux'
    _igh_os='normal'
fi

if [ "$3x" = "1.6.0x" ]; then
    _igh_ver='1.6.0'
else
    _igh_ver='1.5.2'
fi

if [ "$ARCHITECTURE"x != "x86_64"x ] || [ "$PLATFORM"x != "x64"x ]; then
    echo Error: Ethercat runtime package only support x86_64 architecture and x64 OS. 
    return -1
fi

ETHERLAB_SUPPORT='no'
if [ `uname -r` = '4.16.18-rt12' ]; then
    ETHERLAB_SUPPORT='yes'
elif [ `uname -r` = '5.10.214-xeno-3.2.4+' ]; then
    ETHERLAB_SUPPORT='yes'
elif [ `uname -r` = '6.1.83-xeno-3.2.4+' ]; then
    ETHERLAB_SUPPORT='yes'
elif [ `uname -r` = '6.1.0-32-amd64' ]; then
    ETHERLAB_SUPPORT='yes'
elif [ `uname -r` = '6.1.0-32-rt-amd64' ]; then
    ETHERLAB_SUPPORT='yes'
elif [ `uname -r` = '6.1.0-1036-oem' ]; then
    ETHERLAB_SUPPORT='yes'
fi
if [ "$ETHERLAB_SUPPORT" = "yes" ]; then
    if [ "${_mode}x" = "--helpx" ]; then
        echo 'Usage: Runtime_setup.sh mode [type]'
        echo 'mode could be the follow value:'
        echo '  --help                  display help'
        echo '  --install                install this package for current user'
        echo '  --uninstall            uninstall this package for current user'
        echo '  --setup                setup environment'
        echo 'type is option value and it could be the follow value: linux or xenomai'
    elif [ "${_mode}x" = "--installx" ]; then
        workdir=$(cd $(dirname `readlink -f $0`); pwd)
        installLine=`grep ETHERCAT_RUNTIME_ENV_PLACEHOLDER ~/.bashrc`
        if [ ! -n "$installLine" ]; then
            echo . $workdir/Runtime_setup.sh --setup $_os_type $_igh_ver >> ~/.bashrc
        else
            sed -i -e "s#ETHERCAT_RUNTIME_ENV_PLACEHOLDER#. $workdir/Runtime_setup.sh --setup $_os_type $_igh_ver#g" ~/.bashrc
        fi
        $workdir/linux/$ARCHITECTURE/$PLATFORM/Etherlab/$_igh_os/$_igh_ver/Etherlab_setup.sh --install
        if [ "${_igh_os}x" = "xenomaix" ]; then
            etherlab=$workdir/linux/$ARCHITECTURE/$PLATFORM/xenomai-3.2.4/$_igh_ver
        else
            etherlab=$workdir/linux/$ARCHITECTURE/$PLATFORM/normal/$_igh_ver
        fi
        echo `echo $etherlab` >> /etc/ld.so.conf.d/Etherlab.conf
    elif [ "${_mode}x" = "--uninstallx" ]; then
        workdir=$(cd $(dirname `readlink -f $0`); pwd)
        $workdir/linux/$ARCHITECTURE/$PLATFORM/Etherlab/$_igh_os/$_igh_ver/Etherlab_setup.sh --uninstall
        sed -i -e "s#. $workdir/Runtime_setup.sh --setup $_os_type $_igh_ver#ETHERCAT_RUNTIME_ENV_PLACEHOLDER#g" ~/.bashrc
    else
        workdir=$(cd $(dirname $BASH_SOURCE[0]); pwd)
        export ETHERCAT_RUNTIME_ENVIRONMENT_ROOT=$workdir
        export ETHERCAT_ACTIVE_ENVIRONMENT_ROOT=$ETHERCAT_RUNTIME_ENVIRONMENT_ROOT/$GENERIC_BIN_DIR
        export PATH=$ETHERCAT_ACTIVE_ENVIRONMENT_ROOT/Etherlab/$_igh_os/$_igh_ver/bin:$PATH
        export LD_LIBRARY_PATH=$ETHERCAT_ACTIVE_ENVIRONMENT_ROOT/$_igh_os/$_igh_ver:$LD_LIBRARY_PATH
        . $workdir/linux/$ARCHITECTURE/$PLATFORM/Etherlab/$_igh_os/$_igh_ver/Etherlab_setup.sh --setup
    fi
else
    if [ "${_mode}x" = "--installx" ]; then
        echo Warning: Ethercat package do not support current kernel, you can only use it for building process.
        workdir=$(cd $(dirname `readlink -f $0`); pwd)
        ethercatlib=$workdir/linux/$ARCHITECTURE/$PLATFORM/Etherlab/$_igh_os/$_igh_ver
        if [ "${_os_type}x" = "xenomaix" ]; then
            etherlab=$workdir/linux/$ARCHITECTURE/$PLATFORM/xenomai-3.2.4/$_igh_ver
        else
            etherlab=$workdir/linux/$ARCHITECTURE/$PLATFORM/normal/$_igh_ver
        fi
        rm /etc/ld.so.conf.d/Etherlab.conf -f
        echo `echo $ethercatlib/lib` >> /etc/ld.so.conf.d/Etherlab.conf
        echo `echo $etherlab` >> /etc/ld.so.conf.d/Etherlab.conf
        ldconfig
    elif [ "${_mode}x" = "--uninstallx" ]; then
        workdir=$(cd $(dirname `readlink -f $0`); pwd)
        rm /etc/ld.so.conf.d/Etherlab.conf -f
        ldconfig
    else
        workdir=$(cd $(dirname $BASH_SOURCE[0]); pwd)
        export ETHERCAT_RUNTIME_ENVIRONMENT_ROOT=$workdir
        export ETHERCAT_ACTIVE_ENVIRONMENT_ROOT=$ETHERCAT_RUNTIME_ENVIRONMENT_ROOT/$GENERIC_BIN_DIR
        export PATH=$ETHERCAT_ACTIVE_ENVIRONMENT_ROOT/Etherlab/$_igh_os/$_igh_ver/bin:$PATH
        export LD_LIBRARY_PATH=$ETHERCAT_ACTIVE_ENVIRONMENT_ROOT/$_igh_os/$_igh_ver:$LD_LIBRARY_PATH
    fi
fi

