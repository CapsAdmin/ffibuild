collectgarbage("stop")

local ffi = require("ffi")

package.path = package.path .. ";./../examples/?.lua"

local vk = require("vulkan/libvulkan")

_G.FFI_LIB = "../examples/glfw/glfw/src/libglfw.so"
local glfw = require("glfw/libglfw")
_G.FFI_LIB = nil

local DEMO = {}

DEMO.Extensions = {
	"VK_EXT_debug_report",
}

DEMO.ValidationLayers = {
	"VK_LAYER_LUNARG_threading",
	"VK_LAYER_LUNARG_mem_tracker",
	"VK_LAYER_LUNARG_object_tracker",
	"VK_LAYER_LUNARG_draw_state",
	"VK_LAYER_LUNARG_param_checker",
	"VK_LAYER_LUNARG_swapchain",
	"VK_LAYER_LUNARG_device_limits",
	"VK_LAYER_LUNARG_image",
}

function DEMO:PrepareWindow()
	glfw.SetErrorCallback(function(_, str) io.write(string.format("GLFW Error: %s\n", ffi.string(str))) end)
	glfw.Init()
	glfw.WindowHint(glfw.e.GLFW_CLIENT_API, glfw.e.GLFW_NO_API)

	for _, ext in ipairs(glfw.GetRequiredInstanceExtensions()) do
		table.insert(self.Extensions, ext)
	end

	self.Window = glfw.CreateWindow(1024, 768, "vulkan", nil, nil)
end

function DEMO:PrepareInstance()
	local instance = vk.CreateInstance({
		pApplicationInfo = vk.structs.ApplicationInfo({
			pApplicationName = "goluwa",
			applicationVersion = 0,
			pEngineName = "goluwa",
			engineVersion = 0,
			apiVersion = vk.macros.MAKE_VERSION(1, 0, 2),
		}),

		enabledLayerCount = #self.ValidationLayers,
		ppEnabledLayerNames = vk.util.StringList(self.ValidationLayers),

		enabledExtensionCount = #self.Extensions,
		ppEnabledExtensionNames = vk.util.StringList(self.Extensions),
	})

	if instance:LoadProcAddr("vkCreateDebugReportCallbackEXT") then
		instance:CreateDebugReportCallback({
			flags = bit.bor(
				vk.e.VK_DEBUG_REPORT_INFORMATION_BIT_EXT,
				vk.e.VK_DEBUG_REPORT_PERFORMANCE_WARNING_BIT_EXT,
				vk.e.VK_DEBUG_REPORT_ERROR_BIT_EXT,
				vk.e.VK_DEBUG_REPORT_WARNING_BIT_EXT
			),
			pfnCallback = function(msgFlags, objType, srcObject, location, msgCode, pLayerPrefix, pMsg, pUserData)
				io.write(string.format("Vulkan Debug Report: %s: %s\n", ffi.string(pLayerPrefix), ffi.string(pMsg)))

				return 0
			end,
		})
	end

	instance:LoadProcAddr("vkGetPhysicalDeviceSurfaceCapabilitiesKHR")
	instance:LoadProcAddr("vkGetPhysicalDeviceSurfaceFormatsKHR")
	instance:LoadProcAddr("vkGetPhysicalDeviceSurfacePresentModesKHR")
	instance:LoadProcAddr("vkGetPhysicalDeviceSurfaceSupportKHR")
	instance:LoadProcAddr("vkCreateSwapchainKHR")
	instance:LoadProcAddr("vkDestroySwapchainKHR")
	instance:LoadProcAddr("vkGetSwapchainImagesKHR")
	instance:LoadProcAddr("vkAcquireNextImageKHR")
	instance:LoadProcAddr("vkQueuePresentKHR")

	self.Instance = instance
end

function DEMO:PrepareDevice()
	for _, gpu in ipairs(self.Instance:GetPhysicalDevices()) do
		for i, info in ipairs(gpu:GetQueueFamilyProperties()) do

			if bit.band(info.queueFlags, vk.e.VK_QUEUE_GRAPHICS_BIT) ~= 0 then

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

				self.Device = device
				self.GPU = gpu
				self.DeviceQueueIndex = queue_index

				self.DeviceMemoryProperties = self.GPU:GetMemoryProperties()
				self.DeviceQueue = self.Device:GetQueue(self.DeviceQueueIndex, 0)
				self.DeviceCommandPool = self.Device:CreateCommandPool({
					queueFamilyIndex = self.DeviceQueueIndex,
				})
				return
			end
		end
	end
end

function DEMO:PrepareSurface()
	self.Surface = glfw.CreateWindowSurface(self.Instance, self.Window, nil)
	self.SurfaceFormats = self.GPU:GetSurfaceFormats(self.Surface)
	self.SurfaceCapabilities = self.GPU:GetSurfaceCapabilities(self.Surface)

	local prefered_format = self.SurfaceFormats[1].format ~= vk.e.VK_FORMAT_UNDEFINED and self.SurfaceFormats[1].format or vk.e.VK_FORMAT_B8G8R8A8_UNORM

	local w, h = self.SurfaceCapabilities.currentExtent.width, self.SurfaceCapabilities.currentExtent.height

	if w == 0xFFFFFFFF then
		w = 1024
		h = 768
	end

	self.Width = w
	self.Height = w

	self.SurfaceBuffers = {}

	local swap_chain = self.Device:CreateSwapchain({
		surface = self.Surface,
		minImageCount = math.min(self.SurfaceCapabilities.minImageCount + 1, self.SurfaceCapabilities.maxImageCount == 0 and math.huge or self.SurfaceCapabilities.maxImageCount),
		imageFormat = prefered_format,
		imagecolorSpace = self.SurfaceFormats[1].colorSpace,
		imageExtent = {self.Width, self.Height},
		imageUse = vk.e.VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT,
		preTransform = bit.band(self.SurfaceCapabilities.supportedTransforms, vk.e.VK_SURFACE_TRANSFORM_IDENTITY_BIT_KHR) ~= 0 and vk.e.VK_SURFACE_TRANSFORM_IDENTITY_BIT_KHR or self.SurfaceCapabilities.currentTransform,
		compositeAlpha = vk.e.VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR,
		imageArrayLayers = 1,
		imageSharingMode = vk.e.VK_SHARING_MODE_EXCLUSIVE,
		queueFamilyIndexCount = 0,
		pQueueFamilyIndices = nil,
		presentMode = vk.e.VK_PRESENT_MODE_FIFO_KHR,
		oldSwapchain = nil,
		clipped = 1,
	})

	for i, image in ipairs(self.Device:GetSwapchainImages(swap_chain)) do
		self.SurfaceBuffers[i] = {}

		self:SetImageLayout(image, vk.e.VK_IMAGE_ASPECT_COLOR_BIT, vk.e.VK_IMAGE_LAYOUT_UNDEFINED, vk.e.VK_IMAGE_LAYOUT_PRESENT_SRC_KHR)

		local view = self.Device:CreateImageView({
			viewType = vk.e.VK_IMAGE_VIEW_TYPE_2D,
			image = image,
			format = prefered_format,
			flags = 0,
			components = {
				r = vk.e.VK_COMPONENT_SWIZZLE_R,
				g = vk.e.VK_COMPONENT_SWIZZLE_G,
				b = vk.e.VK_COMPONENT_SWIZZLE_B,
				a = vk.e.VK_COMPONENT_SWIZZLE_A
			},
			subresourceRange = {
				aspectMask = tonumber(vk.e.VK_IMAGE_ASPECT_COLOR_BIT),
				baseMipLevel = 0,
				levelCount = 1,
				baseArrayLayer = 0,
				layerCount = 1,
			},
		})

		self.SurfaceBuffers[i].view = view
		self.SurfaceBuffers[i].image = image
	end
end

function DEMO:CreateImage(width, height, format, usage, tiling, required_props)
	local image = self.Device:CreateImage({
		imageType = vk.e.VK_IMAGE_TYPE_2D,
		format = format,
		extent = {width, height, 1},
		mipLevels = 1,
		arrayLayers = 1,
		samples = vk.e.VK_SAMPLE_COUNT_1_BIT,
		tiling = tiling,
		usage = usage,
		flags = 0,
		queueFamilyIndexCount = 0,
	})

	local memory_requirements = self.Device:GetImageMemoryRequirements(image)

	local memory = self.Device:AllocateMemory({
		allocationSize = memory_requirements.size,
		memoryTypeIndex = self:GetMemoryTypeFromProperties(memory_requirements.memoryTypeBits, required_props),
	})

	self.Device:BindImageMemory(image, memory, 0)

	return image, memory, memory_requirements
end

function DEMO:PrepareDepthTexture()
	self.DepthBuffer = {}

	local format = vk.e.VK_FORMAT_D16_UNORM

	local image, memory = self:CreateImage(self.Width, self.Height, format, vk.e.VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT, vk.e.VK_IMAGE_TILING_OPTIMAL, 0)

	self:SetImageLayout(image, vk.e.VK_IMAGE_ASPECT_DEPTH_BIT, vk.e.VK_IMAGE_LAYOUT_UNDEFINED, vk.e.VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL)

	local view = self.Device:CreateImageView({
		viewType = vk.e.VK_IMAGE_VIEW_TYPE_2D,
		image = image,
		format = format,
		flags = 0,
		subresourceRange = {
			aspectMask = tonumber(vk.e.VK_IMAGE_ASPECT_DEPTH_BIT),
			baseMipLevel = 0,
			levelCount = 1,
			baseArrayLayer = 0,
			layerCount = 1,
		},
	})

	self.DepthBuffer.image = image
	self.DepthBuffer.memory = memory
	self.DepthBuffer.view = view
	self.DepthBuffer.format = format
end

do
	local function prepare(self, tex, tiling, usage, required_props)
		tex.width = 2
		tex.height = 2

		local image, memory, memory_requirements = self:CreateImage(tex.width, tex.height, vk.e.VK_FORMAT_B8G8R8A8_UNORM, usage, tiling, required_props)

		tex.memory = memory
		tex.image = image

		if bit.band(required_props, vk.e.VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT) ~= 0 then
			tex.imageLayout = tonumber(vk.e.VK_IMAGE_ASPECT_COLOR_BIT)

			self.Device:MapMemory(memory, 0, memory_requirements.size, 0, "uint32_t", function(data)
				for y = 0, tex.height - 1 do
					for x = 0, tex.width - 1 do
						data[x * y] = math.random(0xFFFFFFFF)
					end
				end
			end)
		end

		self:SetImageLayout(image, vk.e.VK_IMAGE_ASPECT_COLOR_BIT, vk.e.VK_IMAGE_LAYOUT_UNDEFINED, vk.e.VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL)
	end

	function DEMO:CreateTexture()
		local texture = {}

		local format = vk.e.VK_FORMAT_B8G8R8A8_UNORM

		local staging_texture = {}
		prepare(self, staging_texture, vk.e.VK_IMAGE_TILING_LINEAR, vk.e.VK_IMAGE_USAGE_TRANSFER_SRC_BIT, vk.e.VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT)
		self:SetImageLayout(staging_texture.image, vk.e.VK_IMAGE_ASPECT_COLOR_BIT, staging_texture.imageLayout, vk.e.VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL)

		prepare(self, texture, vk.e.VK_IMAGE_TILING_OPTIMAL, bit.bor(vk.e.VK_IMAGE_USAGE_TRANSFER_DST_BIT, vk.e.VK_IMAGE_USAGE_SAMPLED_BIT), vk.e.VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT)
		self:SetImageLayout(texture.image, vk.e.VK_IMAGE_ASPECT_COLOR_BIT, texture.imageLayout, vk.e.VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL)

		self.SetupCMD:CopyImage(staging_texture.image, vk.e.VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, texture.image, vk.e.VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, ffi.new("struct VkImageCopy", {
			srcSubresource = {tonumber(vk.e.VK_IMAGE_ASPECT_COLOR_BIT), 0, 0, 1},
			srcOffset = {0, 0, 0},
			dstSubresource = {tonumber(vk.e.VK_IMAGE_ASPECT_COLOR_BIT), 0, 0, 1},
			dstOffset = {0, 0, 0},
			extent = {
				staging_texture.width,
				staging_texture.height,
				1
			},
		}))

		self:SetImageLayout(texture.image, vk.e.VK_IMAGE_ASPECT_COLOR_BIT, vk.e.VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, texture.imageLayout)
		self:FlushSetupCMD()

		self.Device:DestroyImage(staging_texture.image, nil)
		self.Device:FreeMemory(staging_texture.memory, nil)

		texture.sampler = self.Device:CreateSampler({
			magFilter = vk.e.VK_FILTER_NEAREST,
			minFilter = vk.e.VK_FILTER_NEAREST,
			mipmapMode = vk.e.VK_SAMPLER_MIPMAP_MODE_NEAREST,
			addressModeU = vk.e.VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE,
			addressModeV = vk.e.VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE,
			addressModeW = vk.e.VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE,
			ipLodBias = 0.0,
			anisotropyEnable = 0,
			maxAnisotropy = 1,
			compareOp = vk.e.VK_COMPARE_OP_NEVER,
			minLod = 0.0,
			maxLod = 0.0,
			borderColor = vk.e.VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE,
			unnormalizedCoordinates = 0,
		})

		texture.view = self.Device:CreateImageView({
			viewType = vk.e.VK_IMAGE_VIEW_TYPE_2D,
			image = texture.image,
			format = format,
			flags = 0,
			components = {
				r = vk.e.VK_COMPONENT_SWIZZLE_R,
				g = vk.e.VK_COMPONENT_SWIZZLE_G,
				b = vk.e.VK_COMPONENT_SWIZZLE_B,
				a = vk.e.VK_COMPONENT_SWIZZLE_A
			},
			subresourceRange = {
				aspectMask = tonumber(vk.e.VK_IMAGE_ASPECT_COLOR_BIT),
				0,
				1,
				0,
				1
			},
		})

		return texture
	end
end

function DEMO:PrepareTextures()
	self.Texture = self:CreateTexture()
end

function DEMO:Initialize()
	self:PrepareWindow() -- create a window
	self:PrepareInstance() -- create a vulkan instance
	self:PrepareDevice() -- find and gpu and use it
	self:PrepareSurface() -- create the window surface to render on
	self:PrepareDepthTexture() -- create the depth buffer
	self:PrepareTextures()

	self.DrawCMD = self.Device:AllocateCommandBuffers({
		commandPool = self.DeviceCommandPool,
		level = vk.e.VK_COMMAND_BUFFER_LEVEL_PRIMARY,
		commandBufferCount = 1,
	})

	io.write("reached end of demo\n")

	local function table_print(tbl, level)
		level = level or 0
		for k,v in pairs(tbl) do
			if type(v) == "table" then
				io.write(("\t"):rep(level)..tostring(k).." = {\n")
				level = level + 1
				table_print(v, level)
				level = level - 1
				io.write(("\t"):rep(level).."}\n")
			elseif type(v) ~= "function" then
				io.write(string.format(("\t"):rep(level) .. "%s = %s\n", tostring(k), tostring(v)))
			end
		end
	end

	table_print(self)
end

function DEMO:GetMemoryTypeFromProperties(type_bits, requirements_mask)
	for i = 0, 32 - 1 do
		if bit.band(type_bits, 1) == 1 then
			if bit.band(self.DeviceMemoryProperties.memoryTypes[i].propertyFlags, requirements_mask) == requirements_mask then
				return i
			end
		end
		type_bits = bit.rshift(type_bits, 1)
	end
end

function DEMO:SetImageLayout(image, aspectMask, old_image_layout, new_image_layout)

	if not self.SetupCMD then
		self.SetupCMD = self.Device:AllocateCommandBuffers({
				commandPool = self.DeviceCommandPool,
				level = vk.e.VK_COMMAND_BUFFER_LEVEL_PRIMARY,
				commandBufferCount = 1,
		})

		self.SetupCMD:Begin(vk.structs.CommandBufferBeginInfo{
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
	end

	local mask = 0

	if new_image_layout == vk.e.VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL then
		mask = vk.e.VK_ACCESS_TRANSFER_READ_BIT
	elseif new_image_layout == vk.e.VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL then
		mask = vk.e.VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT
	elseif new_image_layout == vk.e.VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL then
		mask = vk.e.VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT
	elseif new_image_layout == vk.e.VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL then
		mask = bit.bor(vk.e.VK_ACCESS_SHADER_READ_BIT, vk.e.VK_ACCESS_INPUT_ATTACHMENT_READ_BIT)
	end

	self.SetupCMD:PipelineBarrier(
		tonumber(vk.e.VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT),
		tonumber(vk.e.VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT),
		0,
		0,
		nil,
		0,
		nil,
		1,
		vk.structs.ImageMemoryBarrier({
			srcAccessMask = 0,
			dstAccessMask = tonumber(mask),
			oldLayout = old_image_layout,
			newLayout = new_image_layout,
			image = image,
			subresourceRange = {
				tonumber(aspectMask),
				0,
				1,
				0,
				1
			},
		})
	)
end

function DEMO:FlushSetupCMD()
	self.SetupCMD:End()
	self.DeviceQueue:Submit(1, vk.structs.SubmitInfo({
		waitSemaphoreCount = 0,
		pWaitSemaphores = nil,
		pWaitDstStageMask = nil,
		commandBufferCount = 1,
		pCommandBuffers = ffi.new("struct VkCommandBuffer_T *[1]", self.SetupCMD),
		signalSemaphoreCount = 0,
		pSignalSemaphores = nil
	}), nil)
	self.DeviceQueue:WaitIdle()
	self.Device:FreeCommandBuffers(self.DeviceCommandPool, 1, ffi.new("struct VkCommandBuffer_T *[1]", self.SetupCMD))
	self.SetupCMD = nil
end

DEMO:Initialize()