package.path = package.path .. ";../?.lua"
local ffibuild = require("ffibuild")

ffibuild.NixBuild({
	package_name = "luajitPackages.luasocket",
	library_name = "lua/5.1/*"
})