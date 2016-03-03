package.path = package.path .. ";../../?.lua"
local ffibuild = require("ffibuild")


ffibuild.BuildSharedLibrary(
	"openal",
	"git clone https://github.com/kcat/openal-soft repo",
	"cd repo && cmake . && make && cd .. && cp repo/libopenal.so.1.17.2 libopenal.so"
)

for lib_name, enum_name in pairs({al = "AL_", alc = "ALC_"}) do

	ffibuild.lib_name = lib_name

	local headers = {
		"alc",
		"alext",
		"al",
		"efx",
	}

	local c_source = ""

	for _, name in ipairs(headers) do
		c_source = c_source .. "#include \"" .. name .. ".h\"\n"
	end

	local header = ffibuild.BuildCHeader(c_source, "-I./repo/include/AL")

	do
		local args = {}

		for _, name in ipairs(headers) do
			table.insert(args, {"./repo/include/AL/" .. name .. ".h", enum_name})
		end
	end

	local meta_data = ffibuild.GetMetaData(header)
	local header = meta_data:BuildMinimalHeader(function(name) return name:find("^"..lib_name.."%u") end, function(name) return name:find("^" .. enum_name) end, true, true)

	ffibuild.lib_name = "openal"
	local lua = ffibuild.StartLibrary(header)
	ffibuild.lib_name = lib_name

	lua = lua .. "library = " .. meta_data:BuildFunctions("^"..lib_name.."(%u.+)")

	local args = {}

	for _, name in ipairs(headers) do
		table.insert(args, {"./repo/include/AL/" .. name .. ".h", enum_name})
	end

	lua = lua .. "library.e = " .. meta_data:BuildEnums("^"..enum_name.."(.+)", args)

	ffibuild.EndLibrary(lua, header)
end