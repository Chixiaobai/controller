#!/bin/sh

if [ ! -n "$1" ]; then
    echo 'Error: Etherlab setup script do not have enough parameters,please use Runtime_setup.sh --help for more information.'
else
    if [ "$1x" = "--helpx" ]; then
        echo 'Usage: setup.sh mode'
        echo 'mode could be the follow value:'
        echo '  --help                  display help'
        echo '  --install                install this package for current user'
        echo '  --uninstall            uninstall this package for current user'
        echo '  --setup                setup environment'
    elif [ "$1x" = "--installx" ]; then
        workdir=$(cd $(dirname `readlink -f $0`); pwd)
        rm /etc/ld.so.conf.d/Etherlab.conf -f
        echo `echo $workdir/lib` >> /etc/ld.so.conf.d/Etherlab.conf
        cp $workdir/kernel/lib/modules/* /lib/modules/ -rf
        mkdir /etc/sysconfig -p
        cp $workdir/etc/sysconfig/ethercat /etc/sysconfig/ethercat
        rm /etc/init.d/ethercat -f
        ln -s $workdir/etc/init.d/ethercat /etc/init.d/ethercat
        sudo depmod
        rm /etc/udev/rules.d/99-EtherCAT.rules -f
        echo KERNEL==\"EtherCAT[0-9]*\", MODE=\"0666\" > /etc/udev/rules.d/99-EtherCAT.rules
        echo 'Info: Please run gedit /etc/sysconfig/ethercat to set MAC and then reboot system.'
    elif [ "$1x" = "--uninstallx" ]; then
        workdir=$(cd $(dirname `readlink -f $0`); pwd)
        rm /etc/ld.so.conf.d/Etherlab.conf -f
        rm /etc/sysconfig/ethercat -f
        rm /etc/init.d/ethercat -f
        rm /etc/udev/rules.d/99-EtherCAT.rules -f
        rm /lib/modules/`uname -r`/ethercat -rf
        sudo depmod
    elif [ "$1x" = "--setupx" ]; then
        workdir=$(cd $(dirname $BASH_SOURCE[0]); pwd)
        export ETHERLAB_ROOT=$workdir
    else
        echo 'Error: Etherlab setup script has invalid parameter,please use Runtime_setup.sh --help for more information.'
    fi
fi

