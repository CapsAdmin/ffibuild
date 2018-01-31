package.path = package.path .. ";../?.lua"
local ffibuild = require("ffibuild")

ffibuild.NixBuild({
	package_name = "luajitPackages.luasec",
	library_name = "lua/5.1/ssl"
})
