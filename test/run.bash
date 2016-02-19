#!/bin/bash

export LD_LIBRARY_PATH=".:$LD_LIBRARY_PATH"
vktrace -o ./test.vktrace -p ./../LuaJIT/src/luajit -a vulkan_test.lua -w .
vkreplay -t test.vktrace