local extensions = {
	EXT = true,
	KHR = true
}

local ffibuild = dofile("../../ffibuild.lua")

local header = ffibuild.BuildCHeader([[
	#include "/usr/include/vulkan/vulkan.h"
]])

local meta_data = ffibuild.GetMetaData(header)
local header = meta_data:BuildMinimalHeader(function(name) return name:find("^vk") end, function(name) return name:find("^VK_") end, true)

local lua = ffibuild.BuildGenericLua(header, "vulkan", "safe_clib_index", "metatables")

lua = lua .. "library = {\n"

for func_name, func_type in pairs(meta_data.functions) do
	local friendly_name = func_name:match("^vk(.+)")
	lua = lua .. "\t" .. ffibuild.BuildLuaFunction(friendly_name, func_type.name, func_type, nil, nil, nil, nil, "SAFE_INDEX(CLIB)") .. ",\n"
end

lua = lua .. "}\n"

do -- utilities
	lua = lua .. "library.util = {}\n"
	lua = lua .. [[function library.util.StringList(tbl)
	return ffi.new("const char * const ["..#tbl.."]", tbl), #tbl
end
]]
end

do -- macros
	lua = lua .. "library.macros = {}\n"
	lua = lua .. "library.macros.MAKE_VERSION = function(major, minor, patch) return bit.bor(bit.lshift(major, 22), bit.lshift(minor, 12) , patch) end\n"
end

local helper_functions = {}

do -- enumerate helpers so you don't have to make boxed count and array values
	for func_name, func_type in pairs(meta_data.functions) do
		if func_name:find("^vkEnumerate") then
			local friendly = func_name:match("^vkEnumerate(.+)")
			local parameters, call = func_type:GetParameters(nil, nil, -2)

			if #func_type.arguments ~= 2 then
				call = call .. ", "
			end

			lua = lua .. [[function library.Get]] .. friendly .. [[(]] .. parameters .. [[)
	local count = ffi.new("uint32_t[1]")
	local array = ffi.new("]] .. func_type.arguments[#func_type.arguments]:GetDeclaration(meta_data):gsub("(.+)%*", "%1[128]") .. [[")
	local status = CLIB.]] .. func_name .. [[(]] .. call .. [[count, array)

	if status == 0 then
		local out = {}

		for i = 0, count[0] - 1 do
			out[i + 1] = array[i]
		end

		return out
	end
	return nil, status
end
]]
			helper_functions[func_name] = "library.Get" .. friendly
		end
	end
end

do -- get helpers so you don't have to make a boxed value
	for func_name, func_type in pairs(meta_data.functions) do
		if func_name:find("^vkGet") then
			local ret_basic_type = func_type.return_type:GetBasicType(meta_data)
			if ret_basic_type == "enum VkResult" or ret_basic_type == "void" then
				local type = func_type.arguments[#func_type.arguments]
				if type:GetDeclaration(meta_data):sub(-1) == "*" then
					local friendly = func_name:match("^vk(.+)")

					if func_type.arguments[#func_type.arguments - 1]:GetDeclaration(meta_data)  == "unsigned int *" then
						local parameters, call = func_type:GetParameters(nil, nil, -2)

						call = call .. ", "


lua = lua .. [[function library.]] .. friendly .. [[(]] .. parameters .. [[)
	local count = ffi.new("uint32_t[1]")
	local array = ffi.new("]] .. func_type.arguments[#func_type.arguments]:GetDeclaration(meta_data):gsub("(.+)%*", "%1[256]") .. [[")
]]
						if ret_basic_type == "enum VkResult" then
lua = lua .. [[
	local status = CLIB.]] .. func_name .. [[(]] .. call .. [[count, array)

	if status == 0 then
		local out = {}

		for i = 0, count[0] - 1 do
			out[i + 1] = array[i]
		end

		return out
	end
	return nil, status
end
]]
						elseif ret_basic_type == "void" then
lua = lua .. [[
	CLIB.]] .. func_name .. [[(]] .. call .. [[count, array)

	local out = {}

	for i = 0, count[0] - 1 do
		out[i + 1] = array[i]
	end

	return out
end
]]
						end
					else
						local parameters, call = func_type:GetParameters(nil, nil, -1)

						call = call .. ", "

lua = lua .. [[function library.]] .. friendly .. [[(]] .. parameters .. [[)
	local box = ffi.new("]] .. type:GetDeclaration(meta_data):gsub("(.+)%*", "%1[1]") .. [[")
]]

						if ret_basic_type == "enum VkResult" then
lua = lua .. [[
	local status = CLIB.]] .. func_name .. [[(]] .. call .. [[box)

	if status == 0 then
		return box[0]
	end

	return nil, status
end
]]
						elseif ret_basic_type == "void" then
lua = lua .. [[
	CLIB.]] .. func_name .. [[(]] .. call .. [[box)
	return box[0]
end
]]
						end
					end
					helper_functions[func_name] = "library." .. friendly
				end
			end
		end
	end
end

do -- *Create helpers so you don't have to make a boxed value
	for func_name, func_type in pairs(meta_data.functions) do
		local parameters, call = func_type:GetParameters(nil, nil, #func_type.arguments - 1)
		if #func_type.arguments ~= 1 then call = call .. ", " end

		if func_name:find("^vkCreate") then
			local friendly = func_name:match("^vk(.+)")
			lua = lua .. [[function library.]]..friendly..[[(]]..parameters..[[)
	local box = ffi.new("]]..func_type.arguments[#func_type.arguments]:GetDeclaration(meta_data):gsub("(.+)%*", "%1[1]")..[[")
	local status = CLIB.]]..func_name..[[(]]..call..[[box)

	if status == 0 then
		return box[0]
	end

	return nil, status
end
]]
			helper_functions[func_name] = "library." .. friendly
		end
	end
end

do -- struct creation helpers
	lua = lua .. "library.structs = {}\n"
	for i, info in ipairs(meta_data.enums["enum VkStructureType"].enums) do
		local name = info.key:match("VK_STRUCTURE_TYPE_(.+)")
		local friendly = ffibuild.ChangeCase(name:lower(), "foo_bar", "FooBar")

		if extensions[friendly:sub(-3):upper()] then
			friendly = friendly:gsub("^(.+)(%u%l%l)$", function(a, s) return a .. s:upper() end)
		end

		local struct = meta_data.structs["struct Vk" .. friendly]

		if struct then
			lua = lua .. "function library.structs." .. friendly .. "(tbl) tbl.sType = \"" .. info.key .. "\" return ffi.new(\"struct Vk" .. friendly .. "\", tbl) end\n"
		else
			lua = lua .. "function library.structs." .. friendly .. "(tbl) return ffi.new(\"struct Vk" .. friendly .. "\", tbl) end\n"
		end
	end
end

do
	local objects = {}

	for alias, type in pairs(meta_data.typedefs) do
		local basic_type = type:GetBasicType(meta_data)
		local friendly_type_name = basic_type:match("^struct Vk(.+)_T$")
		if friendly_type_name then
			objects[basic_type] = {meta_name = friendly_type_name, declaration = type:GetBasicType(meta_data), functions = {}}
			for func_name, func_type in pairs(meta_data:GetFunctionsStartingWithType(type)) do
				local friendly_name = func_name:match("^vk(.+)")
				friendly_name = friendly_name:gsub(friendly_type_name, "")


				-- INCONSISTENCIES!!!!!
				if friendly_type_name == "CommandBuffer" then
					friendly_name = friendly_name:gsub("Cmd", "")
				end

				for ext in pairs(extensions) do
					friendly_name = friendly_name:gsub(ext.."$", "")
				end

				if helper_functions[func_name] then
					friendly_name = friendly_name:gsub("^Enumerate", "Get")
					objects[basic_type].functions[friendly_name] = helper_functions[func_name]
				else
					objects[basic_type].functions[friendly_name] = func_type
				end
			end
		end
	end

	for _, info in pairs(objects) do
		if next(info.functions) then
			lua = lua .. ffibuild.BuildLuaMetaTable(info.meta_name, info.declaration, info.functions, nil, nil, meta_data, "SAFE_INDEX(CLIB)", true)
		end
	end
end

lua = lua .. "return library\n"

if RELOAD then
	vfs.Write("/media/caps/ssd_840_120gb/goluwa/src/lua/libraries/graphics/ffi/libvulkan.lua", lua)
	return
end

ffibuild.OutputAndValidate(lua, header)