package.path = package.path .. ";../../?.lua"
local ffibuild = require("ffibuild")


ffibuild.BuildSharedLibrary(
	"assimp",
	"git clone https://github.com/assimp/assimp repo",
	"cd repo && cmake . && make && cd .."
)

local header = ffibuild.BuildCHeader([[
	#include "assimp/postprocess.h"
	#include "assimp/scene.h"
	#include "assimp/version.h"
	#include "assimp/config.h"
	#include "assimp/cimport.h"
	#include "assimp/cexport.h"
]], "-I./repo/include")


local meta_data = ffibuild.GetMetaData(header)
meta_data.functions.aiGetImporterDesc = nil

local header = meta_data:BuildMinimalHeader(function(name) return name:find("^ai") end, function(name) return name:find("^ai") end, true, true)
local lua = ffibuild.StartLibrary(header)

lua = lua .. "library = " .. meta_data:BuildFunctions("^ai(.+)")
lua = lua .. "library.e = " .. meta_data:BuildEnums("^ai.-_(%u.+)")

ffibuild.EndLibrary(lua, header)