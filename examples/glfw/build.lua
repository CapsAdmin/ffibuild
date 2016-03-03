package.path = package.path .. ";../../?.lua"
local ffibuild = require("ffibuild")


ffibuild.BuildSharedLibrary(
	"glfw",
	"git clone https://github.com/glfw/glfw repo",
	"cd repo && cmake -DBUILD_SHARED_LIBS=1 . && make && cd .. && cp repo/src/libglfw.so.3.2 libglfw.so"
)

local header = ffibuild.BuildCHeader([[
	#include "vulkan.h"
	#include "GLFW/glfw3.h"
	#include "GLFW/glfw3native.h"

	//void *glfwGetInstanceProcAddress(void *instance, const char* procname);
	//int glfwGetPhysicalDevicePresentationSupport(void *instance, void *device, uint32_t queuefamily);
	//unsigned int glfwCreateWindowSurface(void *instance, GLFWwindow* window, const void* allocator, void** surface);
]], "-I./repo/include -I./../vulkan/repo/include/vulkan")

do
	local temp = io.open("./repo/include/GLFW/glfw3.h")
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
header = header:gsub("\nstruct Vk.-\n", "\n")
header = header:gsub("\nstruct Vk.-\n", "\n")
header = header:gsub("\nstruct Vk.-\n", "\n")

local lua = ffibuild.StartLibrary(header)

lua = lua .. "library = " .. meta_data:BuildFunctions("^glfw(.+)")
lua = lua .. "library.e = " .. meta_data:BuildEnums()


lua = lua .. [[
function library.GetRequiredInstanceExtensions()
	local count = ffi.new("uint32_t[1]")
	local array = CLIB.glfwGetRequiredInstanceExtensions(count)
	local out = {}
	for i = 0, count[0] - 1 do
		table.insert(out, ffi.string(array[i]))
	end
	return out
end

function library.CreateWindowSurface(instance, window, huh)
	local box = ffi.new("struct VkSurfaceKHR_T * [1]")
	local status = CLIB.glfwCreateWindowSurface(instance, window, huh, box)
	if status == "VK_SUCCESS" then
		return box[0]
	end
	return nil, status
end
]]

ffibuild.EndLibrary(lua, header)