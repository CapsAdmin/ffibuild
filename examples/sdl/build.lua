local ffibuild = dofile("../../ffibuild.lua")

local header = ffibuild.BuildCHeader([[
	#include "SDL.h"
]], "$(pkg-config --cflags sdl2)")

header = "struct SDL_BlitMap {};\n" .. header

local meta_data = ffibuild.GetMetaData(header)
local header = meta_data:BuildMinimalHeader(function(name) return name:find("^SDL_") end, function(name) return name:find("^SDL_") or name:find("^KMOD_") end, true)

local lua = ffibuild.BuildGenericLua(header, "sdl", "chars_to_string")

lua = lua .. "library = {\n"

for func_name, func_type in pairs(meta_data.functions) do
	if func_name:find("^SDL_") then
		local friendly_name = func_name:match("SDL_(.+)")
		lua = lua .. "\t" .. ffibuild.BuildLuaFunction(friendly_name, func_type.name, func_type) .. ",\n"
	end
end

lua = lua .. "}\n"

lua = lua .. "return library\n"

ffibuild.OutputAndValidate(lua, header)