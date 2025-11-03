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


if [ "$SETUP_ALLOW" = "no" ]; then
    echo Error: Amw runtime setup script do not support current architecture & platform.
    return -1
fi

if [ ! -n "$1" ]; then
    echo "Error: Amw setup script do not have enough parameters,please use Runtime_setup.sh --help for more information."
    return -1
fi
if [ "$1x" != "--helpx" ] && [ "$1x" != "--installx" ] && [ "$1x" != "--uninstallx" ] && [ "$1x" != "--setupx" ]; then
    echo "Error: Amw setup script has invalid parameter,please use Runtime_setup.sh --help for more information."
    return -1
fi
_mode=$1
if [ "$2x" = "xenomaix" ]; then
    _os_type='xenomai'
else
    _os_type='linux'
fi

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
    installLine=`grep AMW_RUNTIME_ENV_PLACEHOLDER ~/.bashrc`
    if [ ! -n "$installLine" ]; then
        echo . $workdir/Runtime_setup.sh --setup $_os_type >> ~/.bashrc
    else
        sed -i -e "s#AMW_RUNTIME_ENV_PLACEHOLDER#. $workdir/Runtime_setup.sh --setup $_os_type#g" ~/.bashrc
    fi
elif [ "${_mode}x" = "--uninstallx" ]; then
    workdir=$(cd $(dirname `readlink -f $0`); pwd)
    sed -i -e "s#. $workdir/Runtime_setup.sh --setup $_os_type#AMW_RUNTIME_ENV_PLACEHOLDER#g" ~/.bashrc
else
    workdir=$(cd $(dirname $BASH_SOURCE[0]); pwd)
    export AMW_RUNTIME_ENVIRONMENT_ROOT=$workdir
    export AMW_ACTIVE_ENVIRONMENT_ROOT=$AMW_RUNTIME_ENVIRONMENT_ROOT/$GENERIC_BIN_DIR
    export PATH=$AMW_ACTIVE_ENVIRONMENT_ROOT:$PATH
    export LD_COMPONENTS_PATH=$AMW_ACTIVE_ENVIRONMENT_ROOT/components:$LD_COMPONENTS_PATH
    export LD_LIBRARY_PATH=$AMW_ACTIVE_ENVIRONMENT_ROOT:$AMW_ACTIVE_ENVIRONMENT_ROOT/components:$LD_LIBRARY_PATH
    export GENERATE_BASE_PATH=$AMW_RUNTIME_ENVIRONMENT_ROOT/generate
    if [ "$AMW_SERVICE_CENTER_URL"x = ""x ]; then
        #配置默认的ServiceCenter地址
        export AMW_SERVICE_CENTER_URL=tcp://127.0.0.1:15000
    fi
    if [ "$AMW_IDF_PATH"x = ""x ]; then
        #配置默认的消息文件目录
        export AMW_IDF_PATH=$AMW_RUNTIME_ENVIRONMENT_ROOT/amw_msgs
    fi
    if [ "$HOST_STARTUP_CONFIG"x = ""x ]; then
        #配置默认的Host启动配置
        export HOST_STARTUP_CONFIG=$AMW_RUNTIME_ENVIRONMENT_ROOT/config
    fi
fi


