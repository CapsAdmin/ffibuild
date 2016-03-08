package.path = package.path .. ";../../?.lua"
local ffibuild = require("ffibuild")


ffibuild.BuildSharedLibrary(
	"sndfile",
	"https://github.com/erikd/libsndfile.git",
	"./autogen.sh && ./configure && make"
)

local header = ffibuild.BuildCHeader([[
	#include "sndfile.h"
]], "-I./repo/src")

local meta_data = ffibuild.GetMetaData(header)
local header = meta_data:BuildMinimalHeader(function(name) return name:find("^sf_") end, function(name) return name:find("^SF") end, true, true)

local lua = ffibuild.StartLibrary(header)

lua = lua .. "library = " .. meta_data:BuildFunctions("^sf_(.+)")
lua = lua .. "library.e = " .. meta_data:BuildEnums("^SF_(.+)")

ffibuild.EndLibrary(lua, header)