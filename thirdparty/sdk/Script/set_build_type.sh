#!/bin/sh

export Configuration=$1
if [ ! -n "$Configuration" ]; then
    export Configuration=Release
fi

if [ "$Configuration" = "Debug" ]; then
    export LIBRARY_POSTFIX=d
    export Q_Configuration=debug
else
    export LIBRARY_POSTFIX=""
    export Q_Configuration=release
fi

