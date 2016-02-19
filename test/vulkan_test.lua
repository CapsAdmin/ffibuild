local ffi = require("ffi")

package.path = package.path .. ";./../examples/?.lua"

local vk = require("vulkan/libvulkan")

_G.FFI_LIB = "../examples/glfw/glfw/src/libglfw.so"
local glfw = require("glfw/libglfw")
_G.FFI_LIB = nil

local function set_image_layout(setup_cmd, image, aspectMask, old_image_layout, new_image_layout)
	local mask = 0

	if new_image_layout == "VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL" then
		mask = "VK_ACCESS_TRANSFER_READ_BIT"
	elseif new_image_layout == "VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL" then
		mask = "VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT"
	elseif new_image_layout == "VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL" then
		mask = "VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT"
	elseif new_image_layout == "VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL" then
		mask = bit.bor(ffi.cast("enum VkAccessFlagBits", "VK_ACCESS_SHADER_READ_BIT"), ffi.cast("enum VkAccessFlagBits", "VK_ACCESS_INPUT_ATTACHMENT_READ_BIT"))
	end

	setup_cmd:PipelineBarrier("VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT", "VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT", 0, 0, nil, 0, nil, 1, vk.structs.ImageMemoryBarrier({
		srcAccessMask = 0,
		dstAccessMask = 0,
		oldLayout = old_image_layout,
		newLayout = new_image_layout,
		image = image,
		subresourceRange = {aspectMask, 0, 1, 0, 1},
	}))
end

--demo_init_connection
glfw.SetErrorCallback(function(_, str) print(ffi.string(str)) end)
glfw.Init()

--demo_init_vk
local validate = false

local extensions = {
	VK_KHR_surface = true,
	VK_KHR_xlib_surface = true,
	VK_EXT_debug_report = validate,
}

do
	local count = ffi.new("uint32_t[1]")
	local array = glfw.GetRequiredInstanceExtensions(count)
	for i = 0, count[0] - 1 do
		extensions[ffi.string(array[i])] = true
	end
end

local validation_layers = {
	"VK_LAYER_LUNARG_mem_tracker",
	"VK_LAYER_GOOGLE_unique_objects",
}

local temp = {} for k,v in pairs(extensions) do if v then table.insert(temp, k) end end extensions = temp

local instance = vk.CreateInstance({
	pApplicationInfo = vk.structs.ApplicationInfo({
		pApplicationName = "goluwa",
		applicationVersion = 0,
		pEngineName = "goluwa",
		engineVersion = 0,
		apiVersion = vk.macros.MAKE_VERSION(1, 0, 2),
	}),

	enabledLayerCount = validate and #validation_layers or 0,
	ppEnabledLayerNames = validate and vk.util.StringList(validation_layers) or nil,

	enabledExtensionCount = #extensions,
	ppEnabledExtensionNames = vk.util.StringList(extensions),
})

instance:LoadProcAddr("vkCreateDebugReportCallbackEXT")
instance:LoadProcAddr("vkGetPhysicalDeviceSurfaceCapabilitiesKHR")
instance:LoadProcAddr("vkGetPhysicalDeviceSurfaceFormatsKHR")
instance:LoadProcAddr("vkGetPhysicalDeviceSurfacePresentModesKHR")
instance:LoadProcAddr("vkGetPhysicalDeviceSurfaceSupportKHR")
instance:LoadProcAddr("vkCreateSwapchainKHR")
instance:LoadProcAddr("vkDestroySwapchainKHR")
instance:LoadProcAddr("vkGetSwapchainImagesKHR")
instance:LoadProcAddr("vkAcquireNextImageKHR")
instance:LoadProcAddr("vkQueuePresentKHR")

if validate then
	instance:CreateDebugReportCallbackEXT({
		flags = bit.bor(ffi.cast("enum VkDebugReportFlagBitsEXT", "VK_DEBUG_REPORT_ERROR_BIT_EXT"), ffi.cast("enum VkDebugReportFlagBitsEXT", "VK_DEBUG_REPORT_WARNING_BIT_EXT")),
		pfnCallback = function(msgFlags, objType, srcObject, location, msgCode, pLayerPrefix, pMsg, pUserData)
			print(msgFlags, objType, srcObject, location, msgCode, pLayerPrefix, pMsg, pUserData)

			return 0
		end,
	})
end

glfw.WindowHint(ffi.cast("enum GLFWenum", "GLFW_CLIENT_API"), ffi.cast("enum GLFWenum", "GLFW_NO_API"))
local window = glfw.CreateWindow(1024, 768, "vulkan", nil, nil)

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

			local draw_cmd = device:AllocateCommandBuffers({
					commandPool = cmd_pool,
					level = "VK_COMMAND_BUFFER_LEVEL_PRIMARY",
					commandBufferCount = 1,
			})

			local surface = ffi.new("struct VkSurfaceKHR_T * [1]")
			glfw.CreateWindowSurface(instance, window, nil, surface)

			local surface_formats = gpu:GetSurfaceFormatsKHR(surface[0])
			local surface_capabilities = gpu:GetSurfaceCapabilitiesKHR(surface[0])
			local memory_properties = gpu:GetMemoryProperties()

			local format = surface_formats[1].format ~= "VK_FORMAT_UNDEFINED" and surface_formats[1].format or "VK_FORMAT_B8G8R8A8_UNORM"

			local w, h = surface_capabilities.currentExtent.width, surface_capabilities.currentExtent.height

			if w == 0xFFFFFFFF then
				w = 1024
				h = 768
			end

			local swap_chain = device:CreateSwapchainKHR({
				surface = surface[0],
				minImageCount = math.min(surface_capabilities.minImageCount + 1, surface_capabilities.maxImageCount == 0 and math.huge or surface_capabilities.maxImageCount),
				imageFormat = format,
				imagecolorSpace = surface_formats[1].colorSpace,
				imageExtent = {w, h},
				imageUse = "VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT",
				preTransform = bit.band(surface_capabilities.supportedTransforms, ffi.cast("enum VkSurfaceTransformFlagBitsKHR", "VK_SURFACE_TRANSFORM_IDENTITY_BIT_KHR")) ~= 0 and "VK_SURFACE_TRANSFORM_IDENTITY_BIT_KHR" or surface_capabilities.currentTransform,
				compositeAlpha = "VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR",
				imageArrayLayers = 1,
				imageSharingMode = "VK_SHARING_MODE_EXCLUSIVE",
				queueFamilyIndexCount = 0,
				pQueueFamilyIndices = nil,
				presentMode = "VK_PRESENT_MODE_FIFO_KHR",
				oldSwapchain = nil,
				clipped = true,
			})

			local setup_cmd = device:AllocateCommandBuffers({
					commandPool = cmd_pool,
					level = "VK_COMMAND_BUFFER_LEVEL_PRIMARY",
					commandBufferCount = 1,
			})

			setup_cmd:Begin(vk.structs.CommandBufferBeginInfo{
				flags = 0,
				pInheritanceInfo = vk.structs.CommandBufferInheritanceInfo({
					renderPass = nil,
					subpass = 0,
					framebuffer = nil,
					offclusionQueryEnable = 0,
					queryFlags = 0,
					pipelineStatistics = 0,
				 })
			})

			local buffers = {}

			for i, image in ipairs(device:GetSwapchainImagesKHR(swap_chain)) do
				local view = device:CreateImageView({
					format = format,
					components = {
						r = "VK_COMPONENT_SWIZZLE_R",
						g = "VK_COMPONENT_SWIZZLE_G",
						b = "VK_COMPONENT_SWIZZLE_B",
						a = "VK_COMPONENT_SWIZZLE_A"
					},
					subresourceRange = {
						aspectMask = tonumber(ffi.new("enum VkImageAspectFlagBits", "VK_IMAGE_ASPECT_COLOR_BIT")),
						baseMipLevel = 0,
						levelCount = 1,
						baseArrayLayer = 0,
						layerCount = 1,
					},
					viewType = "VK_IMAGE_VIEW_TYPE_2D",
					image = image,
					flags = 0,
				})
				print(i, view, image)
				buffers[i] = {view = view, image = image}
			end

			--print(tonumber(err))
			--gpu:GetSurfaceSupportKHR(queue_index, surface[0])

			return
		end
	end
end

