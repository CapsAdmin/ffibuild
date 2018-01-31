package.path = package.path .. ";../?.lua"
local ffibuild = require("ffibuild")

ffibuild.NixBuild({
	package_name = "openssl",
	library_name = "libssl"
})
