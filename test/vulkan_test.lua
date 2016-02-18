local ffi = require("ffi")

package.path = package.path .. ";./../examples/?.lua"

local vk = require("vulkan/libvulkan")

_G.FFI_LIB = "../examples/glfw/glfw/src/libglfw.so"
local glfw = require("glfw/libglfw")
_G.FFI_LIB = nil

glfw.SetErrorCallback(function(_, str) print(ffi.string(str)) end)
glfw.Init()

local validate = false

local extensions = {
	VK_KHR_surface = true,
	VK_KHR_xcb_surface = true,
	VK_EXT_debug_report = validate,
}

do
	local count = ffi.new("uint32_t[1]")
	local array = glfw.GetRequiredInstanceExtensions(count)
	for i = 0, count[0] - 1 do
		extensions[ffi.string(array[i])] = true
	end
end

table.print(extensions)

local layers = {
	"VK_LAYER_LUNARG_threading",
	"VK_LAYER_LUNARG_mem_tracker",
	"VK_LAYER_LUNARG_object_tracker",
	"VK_LAYER_LUNARG_draw_state",
	"VK_LAYER_LUNARG_param_checker",
	"VK_LAYER_LUNARG_swapchain",
	"VK_LAYER_LUNARG_device_limits",
	"VK_LAYER_LUNARG_image",
	"VK_LAYER_GOOGLE_unique_objects",
}

local temp = {} for k,v in pairs(extensions) do if v then table.insert(temp, k) end end extensions = temp

local instance = vk.CreateInstance({
	pApplicationInfo = vk.structs.ApplicationInfo({
		pApplicationName = "goluwa",
		pEngineName = "goluwa",
		apiVersion = vk.macros.MAKE_VERSION(1, 0, 2),
	}),

	enabledLayerCount = validate and #layers or 0,
	ppEnabledLayerNames = validate and vk.util.StringList(layers) or nil,

	enabledExtensionCount = #extensions,
	ppEnabledExtensionNames = vk.util.StringList(extensions),
})

instance:LoadProcAddr("vkGetPhysicalDeviceSurfaceSupportKHR")
instance:LoadProcAddr("vkGetPhysicalDeviceSurfaceCapabilitiesKHR")

glfw.WindowHint(ffi.cast("enum GLFWenum", "GLFW_CLIENT_API"), ffi.cast("enum GLFWenum", "GLFW_NO_API"))
local window = glfw.CreateWindow(1024, 768, "vulkan", nil, nil)
--glfw.SetWindowRefreshCallback(function() end)
--glfw.SetFramebufferSizeCallback(function() end)
--glfw.SetKeyCallback(function() end)

local surface = ffi.new("struct VkSurfaceKHR_T[1]")
glfw.CreateWindowSurface(instance, window, nil, surface)

for _, gpu in ipairs(instance:GetPhysicalDevices()) do
	for i, info in ipairs(gpu:GetQueueFamilyProperties()) do

		if bit.band(info.queueFlags, ffi.cast("enum VkQueueFlagBits", "VK_QUEUE_GRAPHICS_BIT")) ~= 0 then -- todo: public ffi use is bad!

			local queue_index = i - 1

			local device = gpu:CreateDevice({
				queueCreateInfoCount = 1,
				pQueueCreateInfos = vk.structs.DeviceQueueCreateInfo({
					queueFamilyIndex = queue_index,
					queueCount = 1,
					pQueuePriorities = ffi.new("float[1]", 0), -- todo: public ffi use is bad!
					pEnabledFeatures = nil,

				})
			})

			local queue = device:GetQueue(queue_index, 0)
			local cmd_pool = device:CreateCommandPool({
				queueFamilyIndex = queue_index,
			})

			--local ok, err = gpu:GetSurfaceCapabilitiesKHR(surface)
			--print(tonumber(err))
			--gpu:GetSurfaceSupportKHR(queue_index, surface)

			return
		end
	end
end

