local ffibuild = dofile("../../ffibuild.lua")

local header = ffibuild.BuildCHeader([[
	#include "FreeImage.h"
]], "-I./freeimage/FreeImage/Source/")


local meta_data = ffibuild.GetMetaData(header)
local header = meta_data:BuildMinimalHeader(function(name) return name:find("^FreeImage_") end, function(name) return name:find("^FI") end, true, true)
local lua = ffibuild.BuildGenericLua(header, "freeimage")

lua = lua .. "library = {\n"

for func_name, func_type in pairs(meta_data.functions) do
	if func_name:find("^FreeImage_") and func_name ~= "FreeImage_RegisterExternalPlugin" then
		local friendly_name = func_name:match("FreeImage_(.+)")
		lua = lua .. "\t" .. ffibuild.BuildLuaFunction(friendly_name, func_type.name, func_type) .. ",\n"
	end
end

lua = lua .. "}\n"

do -- enums
	lua = lua .. "library.e = {\n"
	for basic_type, type in pairs(meta_data.enums) do
		for i, enum in ipairs(type.enums) do
			local friendly = enum.key:match("^FI(.+)")
			if friendly:find("^T_") then
				friendly = friendly:gsub("^T", "IMAGE_TYPE")
			elseif friendly:find("^CC_") then
				friendly = friendly:gsub("^CC", "COLOR_CHANNEL")
			elseif friendly:find("^C_") then
				friendly = friendly:gsub("^C", "COLOR_TYPE")
			elseif friendly:find("^F_") then
				friendly = friendly:gsub("^F", "FORMAT")
			elseif friendly:find("^Q_") then
				friendly = friendly:gsub("^Q", "QUANTIZE")
			elseif friendly:find("^LTER_") then
				friendly = friendly:gsub("^LTER", "IMAGE_FILTER")
			elseif friendly:find("^D_") then
				friendly = friendly:gsub("^D", "DITHER")
			elseif friendly:find("^MD_") then
				friendly = friendly:gsub("^MD", "METADATA")
			elseif friendly:find("^DT_") then
				friendly = friendly:gsub("^DT", "METADATA_TYPE")
			elseif friendly:find("^JPEG_OP_") then
				friendly = friendly:gsub("^JPEG_OP", "JPEG_OPERATION")
			elseif friendly:find("^JPEG_OP_") then
				friendly = friendly:gsub("^JPEG_OP", "JPEG_OPERATION")
			elseif friendly:find("^TMO_") then
				friendly = friendly:gsub("^TMO", "TONEMAP_OPERATOR")
			end
			lua =  lua .. "\t" .. friendly .. " = ffi.cast(\""..basic_type.."\", \""..enum.key.."\"),\n"
		end
	end
	lua = lua .. "}\n"
end


lua = lua .. "return library\n"

ffibuild.OutputAndValidate(lua, header)