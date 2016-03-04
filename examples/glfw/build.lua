package.path = package.path .. ";../../?.lua"
local ffibuild = require("ffibuild")


ffibuild.BuildSharedLibrary(
	"glfw",
	"git clone https://github.com/glfw/glfw repo",
	"cd repo && cmake -DBUILD_SHARED_LIBS=1 . && make && cd .."
)

local header = ffibuild.BuildCHeader([[
	#include "vulkan.h"
	#include "GLFW/glfw3.h"
	#include "GLFW/glfw3native.h"
]], "-I./repo/include -I./../vulkan/repo/include/vulkan")

local meta_data = ffibuild.GetMetaData(header)
local header = meta_data:BuildMinimalHeader(function(name) return name:find("^glfw") end, function(name) return name:find("^GLFW") end, true, true)

header = header:gsub("^struct VkInstance_T.-struct VkSurfaceKHR_T {};", "") -- TODO

local lua = ffibuild.StartLibrary(header)

lua = lua .. "library = " .. meta_data:BuildFunctions("^glfw(.+)")
lua = lua .. "library.e = " .. meta_data:BuildEnums(nil, "./repo/include/GLFW/glfw3.h", "GLFW_")

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