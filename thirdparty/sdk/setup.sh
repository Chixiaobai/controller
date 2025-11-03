#!/bin/sh

if [ ! -n "$1" ]; then
    echo "not enough parameter,please use setup.sh --help for more information"
    return -1
fi
if [ "$1x" != "--helpx" ] && [ "$1x" != "--installx" ] && [ "$1x" != "--uninstallx" ] && [ "$1x" != "--setupx" ]; then
    echo "invalid parameter: $1,please use setup.sh --help for more information."
    return -1
fi
_mode=$1
if [ "$2x" = "xenomaix" ]; then
    _os_type='xenomai'
else
    _os_type='linux'
fi

if [ "${_mode}x" = "--helpx" ]; then
    echo 'Usage: setup.sh mode [type]'
    echo 'mode could be the follow value:'
    echo '  --help                  display help'
    echo '  --install                install this package for current user'
    echo '  --uninstall            uninstall this package for current user'
    echo '  --setup                setup environment'
    echo 'type is option value and it could be the follow value: linux or xenomai'
elif [ "${_mode}x" != "--setupx" ]; then
    workdir=$(cd $(dirname `readlink -f $0`); pwd)
    $workdir/Development_Env/Develop_setup.sh $_mode $_os_type
    workdir=$(cd $(dirname `readlink -f $0`); pwd)
    $workdir/Runtime_Env/Runtime_setup.sh $_mode $_os_type
else
    workdir=$(cd $(dirname $BASH_SOURCE[0]); pwd)
    . $workdir/Development_Env/Develop_setup.sh $_mode $_os_type
    workdir=$(cd $(dirname $BASH_SOURCE[0]); pwd)
    . $workdir/Runtime_Env/Runtime_setup.sh $_mode $_os_type
fi
