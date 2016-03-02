package.path = package.path .. ";../../?.lua"
local ffibuild = require("ffibuild")


ffibuild.BuildSharedLibrary(
	"sdl",
	"hg clone http://hg.libsdl.org/SDL repo",
	"cd repo && mkdir build && cd build && ../configure && make && cd ../../ && cp repo/build/.libs/libSDL2-2.0.so.0.4.0 libSDL2.so"
)

local header = ffibuild.BuildCHeader([[
	#include "SDL.h"
]], "$(pkg-config --cflags sdl2)")

header = "struct SDL_BlitMap {};\n" .. header

local meta_data = ffibuild.GetMetaData(header)
local header = meta_data:BuildMinimalHeader(function(name) return name:find("^SDL_") end, function(name) return name:find("^SDL_") or name:find("^KMOD_") end, true)

local lua = ffibuild.BuildGenericLua(header, "sdl")

lua = lua .. "library = {\n"

for func_name, func_type in pairs(meta_data.functions) do
	if func_name:find("^SDL_") then
		local friendly_name = func_name:match("SDL_(.+)")
		lua = lua .. "\t" .. ffibuild.BuildLuaFunction(friendly_name, func_type.name, func_type) .. ",\n"
	end
end

lua = lua .. "}\n"

do -- enums
	lua = lua .. "library.e = {\n"
	for basic_type, type in pairs(meta_data.enums) do
		for i, enum in ipairs(type.enums) do
			lua =  lua .. "\t" .. enum.key .. " = ffi.cast(\""..basic_type.."\", \""..enum.key.."\"),\n"
		end
	end
	lua = lua .. "}\n"
end


lua = lua .. "return library\n"

ffibuild.OutputAndValidate("sdl", lua, header)