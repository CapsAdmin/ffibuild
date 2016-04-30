package.path = package.path .. ";../../?.lua"
local ffibuild = require("ffibuild")

os.execute("git clone https://github.com/bkaradzic/bx")

ffibuild.BuildSharedLibrary(
	"bgfx",
	"https://github.com/bkaradzic/bgfx.git",
	"make linux-release64"
)

local header = ffibuild.BuildCHeader([[
	#include "bgfx/c99/bgfx.h"
]], "-I./repo/include/")

local meta_data = ffibuild.GetMetaData(header)
local header = meta_data:BuildMinimalHeader(function(name) return name:find("^bgfx_") end, function(name) return name:find("^BGFX_") end, true, true)
local lua = ffibuild.StartLibrary(header)

lua = lua .. "library = " .. meta_data:BuildFunctions("^bgfx_(.+)", "foo_bar", "FooBar")
lua = lua .. "library.e = " .. meta_data:BuildEnums("^BGFX_(.+)", "./repo/include/bgfx/bgfxdefines.h", "BGFX_")

ffibuild.EndLibrary(lua, header)