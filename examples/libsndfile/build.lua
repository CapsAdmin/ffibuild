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

local extra = ""

for name, struct in pairs(meta_data.structs) do
	if name:find("^struct SF_") and not header:find(name) then
		extra = extra .. name .. struct:GetDeclaration(meta_data) .. ";\n"
	end
end

header = header .. extra

local lua = ffibuild.StartLibrary(header)

lua = lua .. "library = " .. meta_data:BuildFunctions("^sf_(.+)", "foo_bar", "FooBar")
lua = lua .. "library.e = " .. meta_data:BuildEnums("^SF[CMD]?_(.+)")

ffibuild.EndLibrary(lua, header)