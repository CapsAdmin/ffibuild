package.path = package.path .. ";../../?.lua"
local ffibuild = require("ffibuild")


ffibuild.BuildSharedLibrary(
	"bullet",
	"https://github.com/AndresTraks/BulletSharpPInvoke.git",
	"cd libbulletc && " .. 
    "git clone https://github.com/bulletphysics/bullet3 --depth 1 && cd bullet3 && mkdir build && cd build && " ..
    "cmake .. -DCMAKE_CXX_FLAGS=\"-fPIC\" -DCMAKE_C_FLAGS=\"-fPIC\" -DBUILD_BULLET3=off -DBUILD_PYBULLET=off -DBUILD_CPU_DEMOS=off -DBUILD_BULLET2_DEMOS=off -DBUILD_UNIT_TESTS=off && "..
    "make && cd ../../ && " .. 
    "cmake . -DBULLET_INCLUDE_DIR=bullet3/src && make"
)

local header = ffibuild.BuildCHeader([[
	#include "bulletc.h"

]], "-I./repo/libbulletc/src/")

header = header:gsub("extern \"C\"%s-{", "")
header = header:gsub("}", "")

local meta_data = ffibuild.GetMetaData(header)

local header = meta_data:BuildMinimalHeader(function(name) return name:find("^bt%u") end, function(name) return name:find("^bt%u") end, true, true)

local lua = ffibuild.StartLibrary(header)

lua = lua .. "library = " .. meta_data:BuildFunctions("^bt(%u.+)")
lua = lua .. "library.e = " .. meta_data:BuildEnums("^bt(%u.+)")

ffibuild.EndLibrary(lua, header)

