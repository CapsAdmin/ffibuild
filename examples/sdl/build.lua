package.path = package.path .. ";../../?.lua"
local ffibuild = require("ffibuild")


ffibuild.BuildSharedLibrary(
	"SDL2",
	"hg clone http://hg.libsdl.org/SDL repo",
	"cd repo && mkdir build && cd build && ../configure && make && cd ../../"
)

local header = ffibuild.BuildCHeader([[
	#include "SDL.h"
]], "$(pkg-config --cflags sdl2)")

header = "struct SDL_BlitMap {};\n" .. header

local meta_data = ffibuild.GetMetaData(header)
local header = meta_data:BuildMinimalHeader(function(name) return name:find("^SDL_") end, function(name) return name:find("^SDL_") or name:find("^KMOD_") end, true)

header = header:gsub("SDL_MESSAGEBOX_COLOR_MAX", "5") -- TODO


local lua = ffibuild.StartLibrary(header)

lua = lua .. "library = " .. meta_data:BuildFunctions("^SDL_(.+)")
lua = lua .. "library.e = " .. meta_data:BuildEnums()

ffibuild.EndLibrary(lua, header)