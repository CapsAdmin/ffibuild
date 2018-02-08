package.path = package.path .. ";../?.lua"
local ffibuild = require("ffibuild")

ffibuild.NixBuild({
	package_name = "pkgs.luajitPackages.luasec.overrideAttrs (oldAttrs: {src = fetchFromGitHub {owner = \"brunoos\"; repo = \"luasec\";};  buildInputs = [ luajit openssl_1_1_0 ];})",
	custom = [[

	]],
	library_name = "lua/5.1/ssl"
})
