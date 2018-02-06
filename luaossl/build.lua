package.path = package.path .. ";../?.lua"
local ffibuild = require("ffibuild")

os.checkcmds("cmake", "git", "make")

local target = jit.os == "OSX" and "macosx" or "linux"
local ext = jit.os == "OSX" and ".dylib" or ".so"
local luajit_src = io.popen("echo $(realpath ../luajit/repo/src)", "r"):read("*all"):sub(0,-2)
local luajit_lib = "libluajit.a"
local inc = "-I" .. luajit_src
local lib = "-l:" .. luajit_lib .. " " .. "-L" .. luajit_src

ffibuild.ManualBuild(
	"luaossl",
	"https://github.com/wahern/luaossl.git",
	"make CPPFLAGS=\""..inc.."\" LDFLAGS=\""..lib.."\"",
	"cp repo/src/5.1/openssl.so _openssl.so && mkdir -p openssl && cp -r repo/src/*.lua openssl/ && cd openssl && for f in *.lua; do mv \"$f\" \"${f#openssl.}\"; done"
)

if os.isfile("_openssl.so") then
    os.execute("../luajit/repo/src/luajit -e \"for k,v in pairs(require('openssl.hmac')) do print(k,v) end\"")
end
