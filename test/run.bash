#!/bin/bash

rm ../examples/glfw/libglfw.lua && make -C ../examples/glfw/
rm ../examples/vulkan/libvulkan.lua && make -C ../examples/vulkan/
rm ../examples/freeimage/libfreeimage.lua && make -C ../examples/freeimage/

export LD_LIBRARY_PATH=".:$LD_LIBRARY_PATH"
./../LuaJIT/src/luajit vulkan_test.lua

#vktrace -o ./test.vktrace -p ./../LuaJIT/src/luajit -a vulkan_test.lua -w .
#vkreplay -t test.vktrace