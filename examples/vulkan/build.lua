local ffibuild = dofile("../../ffibuild.lua")

local header = ffibuild.BuildCHeader([[
	#include "/usr/include/vulkan/vulkan.h"
]])

local meta_data = ffibuild.GetMetaData(header)
local header = meta_data:BuildMinimalHeader(function(name) return name:find("^vk") end, function(name) return name:find("^VK_") end, true)

local lua = ffibuild.BuildGenericLua(header, "vulkan")

lua = lua .. "library = {\n"

for func_name, func_type in pairs(meta_data.functions) do
	local friendly_name = func_name:match("^vk(.+)")
	lua = lua .. "\t" .. ffibuild.BuildLuaFunction(friendly_name, func_type.name, func_type) .. ",\n"
end

lua = lua .. "}\n"

lua = lua .. "return library\n"

ffibuild.OutputAndValidate(lua, header)