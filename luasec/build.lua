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
	"luasec",
	"https://github.com/brunoos/luasec.git",
	"make " .. (jit.os == "Linux" and "linux" or jit.os == "OSX" and "macosx") .. " INC_PATH=\""..inc.."\" LIB_PATH=\""..lib.."\"",
	"cp repo/src/ssl.so . && mkdir -p ssl && cp -r repo/src/*.lua ssl/"
)

if os.isfile("ssl.so") then
    os.execute("../luajit/repo/src/luajit -e \"for k,v in pairs(require('ssl.ssl')) do print(k,v) end\"")
end
