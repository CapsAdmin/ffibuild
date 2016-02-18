local ffibuild = dofile("../../ffibuild.lua")

local header = ffibuild.BuildCHeader([[
	#include "GLFW/glfw3.h"
	//#include "GLFW/glfw3native.h"

	void *glfwGetInstanceProcAddress(void *instance, const char* procname);
	int glfwGetPhysicalDevicePresentationSupport(void *instance, void *device, uint32_t queuefamily);
	unsigned int glfwCreateWindowSurface(void *instance, GLFWwindow* window, const void* allocator, void* surface);
]], "-I./glfw/include")

do
	local temp = io.open("./glfw/include/GLFW/glfw3.h")
	local raw_header = temp:read("*all")
	temp:close()

	local fake_enums = "typedef enum {\n"

	raw_header:gsub("#define (GLFW_.-)\n", function(chunk)
		local enum, found = chunk:gsub("^(%S-)(%s+)(.+)", function(a, b, c) return a .. " = " .. c .. "," end)
		if found > 0 then
			fake_enums = fake_enums .. enum .. "\n"
		end
	end)

	fake_enums = fake_enums:sub(0, -3) .. "} GLFWenum;\n"

	header = fake_enums .. header
end

local meta_data = ffibuild.GetMetaData(header)
local header = meta_data:BuildMinimalHeader(function(name) return name:find("^glfw") end, function(name) return name:find("^GLFW") end, true, true)

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

if RELOAD then
	vfs.Write("/media/caps/ssd_840_120gb/goluwa/src/lua/libraries/graphics/ffi/libglfw3.lua", lua)
	return
end

ffibuild.OutputAndValidate(lua, header)