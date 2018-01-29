package.path = package.path .. ";../?.lua"
local ffibuild = require("ffibuild")

os.checkcmds("meson", "ninja", "git")

os.execute("git clone https://github.com/bkaradzic/bx --depth 1")
os.execute("git clone https://github.com/bkaradzic/bimg --depth 1")

ffibuild.ManualBuild(
	"bgfx",
	"https://github.com/bkaradzic/bgfx.git",
	"make linux-release64"
)

local header = ffibuild.ProcessSourceFileGCC([[
	#include "bgfx/c99/bgfx.h"
]], "-I./repo/include/ -I./bx/include/")

local meta_data = ffibuild.GetMetaData(header)
local header = meta_data:BuildMinimalHeader(function(name) return name:find("^bgfx_") end, function(name) return name:find("^BGFX_") end, true, true)
local lua = ffibuild.StartLibrary(header)

lua = lua .. "library = " .. meta_data:BuildFunctions("^bgfx_(.+)", "foo_bar", "FooBar")
lua = lua .. "library.e = " .. meta_data:BuildEnums("^BGFX_(.+)", "./repo/include/bgfx/defines.h", "BGFX_")

ffibuild.EndLibrary(lua, header)
