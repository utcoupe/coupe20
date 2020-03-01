#!/bin/bash

# Shortcut for a working arm-none-eabi based GNU Debugger

COMPILER_VERSION="6-2017-q2-update"


if [[ -z $UTCOUPE_WORKSPACE ]]; then
    echo '$UTCOUPE_WORKSPACE must be defined!'
    exit 1
fi

"$UTCOUPE_WORKSPACE/libs/gcc-arm-none-eabi-$COMPILER_VERSION/bin/arm-none-eabi-gdb" $@
