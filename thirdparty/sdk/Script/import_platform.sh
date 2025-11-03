#!/bin/bash

_scriptdir=$(cd $(dirname $BASH_SOURCE[0]); pwd)

#判断在当前环境下该脚本是否是第一次执行
if [ "${PlatformInited}x" = "x" ]; then
    echo Platform is initing!

    source $_scriptdir/set_build_type.sh Release
    source $_scriptdir/initool.sh

    PlatformIniFile=$1
    export Architecture=`initool $PlatformIniFile general Architecture`
    export Platform=`initool $PlatformIniFile general Platform`
    export ProcessorNum=`initool $PlatformIniFile general ProcessorNum`
    export AdditionCMakeArg=`initool $PlatformIniFile general AdditionCMakeArg`
    export Toolchain=`initool $PlatformIniFile linux Toolchain`
    export CompilePrefix=`initool $PlatformIniFile linux CompilePrefix`
    export ACROSS_C_COMPILER="$CompilePrefix-gcc"
    export ACROSS_CXX_COMPILER="$CompilePrefix-g++"

    if [ ! -n "$Architecture" ]; then
        export Architecture=x86_64
    fi

    if [ ! -n "$Platform" ]; then
        if [ $(getconf WORD_BIT) = '32' ] && [ $(getconf LONG_BIT) = '64' ] ; then
            export Platform='x64'
        else
            export Platform='x86'
        fi
    fi

    if [ ! -n "$ProcessorNum" ]; then
        export ProcessorNum=`cat /proc/cpuinfo | grep "processor id" | sort | uniq | wc -l`
    fi

    if [ "$Architecture" = "arm" ]; then
        #check across compile environment
        if [ ! -n "$ACROSS_C_COMPILER" ]; then
            echo "do not set  ACROSS_C_COMPILER"
            exit
        fi
        if [ ! -n "$ACROSS_CXX_COMPILER" ]; then
            echo "do not set  ACROSS_CXX_COMPILER"
            exit
        fi
        if [ -n "$SYS_ROOT" ]; then
            export AdditionCMakeArg="-DSYS_ROOT=$SYS_ROOT $AdditionCMakeArg"
        fi
        export ADDITION_CMAKE_ARGUMENTS="-DACROSS_C_COMPILER=$ACROSS_C_COMPILER -DACROSS_CXX_COMPILER=$ACROSS_CXX_COMPILER $AdditionCMakeArg"
        echo "ACROSS_C_COMPILER="$ACROSS_C_COMPILER
        echo "ACROSS_CXX_COMPILER="$ACROSS_CXX_COMPILER
        echo "AdditionCMakeArg"=$AdditionCMakeArg
    fi

    export PlatformInited=1
fi

