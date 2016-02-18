#!/bin/bash

export LD_LIBRARY_PATH=".:$LD_LIBRARY_PATH"
./../LuaJIT/src/luajit vulkan_test.lua