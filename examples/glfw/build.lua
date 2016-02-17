local ffibuild = dofile("../../ffibuild.lua")

local header = ffibuild.BuildCHeader([[
	#include "GLFW/glfw3.h"
]], "-I./include")

do
	local temp = io.open("./include/GLFW/glfw3.h")
	local raw_header = temp:read("*all")
	temp:close()

	local fake_enums = "enum {\n"

	raw_header:gsub("#define (GLFW_.-)\n", function(chunk)
		local enum, found = chunk:gsub("^(%S-)(%s+)(.+)", function(a, b, c) return a .. " = " .. c .. "," end)
		if found > 0 then
			fake_enums = fake_enums .. enum .. "\n"
		end
	end)

	fake_enums = fake_enums .. "};\n"

	header = fake_enums .. header
end

local meta_data = ffibuild.GetMetaData(header)
local header = meta_data:BuildMinimalHeader(function(name) return name:find("^glfw") end, function(name) return name:find("^GLFW") end, true)

local lua = ffibuild.BuildGenericLua(header, "glfw3")

lua = lua .. "library = {\n"

for func_name, func_type in pairs(meta_data.functions) do
	if func_name:find("^glfw") then
		local friendly_name = func_name:match("glfw(.+)")
		lua = lua .. "\t" .. ffibuild.BuildLuaFunction(friendly_name, func_type.name, func_type) .. ",\n"
	end
end

lua = lua .. "}\n"

lua = lua .. "return library\n"

ffibuild.OutputAndValidate(lua, header)