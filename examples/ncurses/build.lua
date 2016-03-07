package.path = package.path .. ";../../?.lua"
local ffibuild = require("ffibuild")

ffibuild.BuildSharedLibrary(
	"ncurses",
	"git clone git://ncurses.scripts.mit.edu/ncurses.git repo --depth 1",
	"cd repo/ && ./configure --with-shared --without-normal --without-debug && make && cd ../"
)

do return end -- TODO

local header = ffibuild.BuildCHeader([[
	#include "curses.h"
]], "-I./repo/include")

local meta_data = ffibuild.GetMetaData(header)
local header = meta_data:BuildMinimalHeader(nil, nil, true)

local lua = ffibuild.StartLibrary(header)
-- curses is extremely macro heavy and has no library prefix
lua = lua .. "library = " .. meta_data:BuildFunctions()
lua = lua .. [[
function library.freeconsole()
	if jit.os == "Windows" then
		ffi.cdef("int FreeConsole();")
		ffi.C.FreeConsole()
	end
end
]]
ffibuild.EndLibrary(lua, header)