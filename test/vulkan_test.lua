collectgarbage("stop")

local ffi = require("ffi")

package.path = package.path .. ";./../examples/?.lua"

local vk = require("vulkan/libvulkan")

_G.FFI_LIB = "../examples/glfw/glfw/src/libglfw.so"
local glfw = require("glfw/libglfw")
_G.FFI_LIB = nil

local setup_cmd

local function memory_type_from_properties(memory_properties, typeBits, requirements_mask, typeIndex)
	for i = 0, 32 - 1 do
		if bit.band(typeBits, 1) == 1 then
			if bit.band(memory_properties.memoryTypes[i].propertyFlags, requirements_mask) == requirements_mask then
				return i
			end
		end
		typeBits = bit.rshift(typeBits, 1)
	end
end

local function set_image_layout(device, cmd_pool, image, aspectMask, old_image_layout, new_image_layout)

	if not setup_cmd then
		setup_cmd = device:AllocateCommandBuffers({
				commandPool = cmd_pool,
				level = vk.e.VK_COMMAND_BUFFER_LEVEL_PRIMARY,
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

	setup_cmd:PipelineBarrier(
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

local function flush_setup_cmd(cmd_pool, device, queue)
	setup_cmd:End()
	queue:Submit(1, vk.structs.SubmitInfo({
		waitSemaphoreCount = 0,
		pWaitSemaphores = nil,
		pWaitDstStageMask = nil,
		commandBufferCount = 1,
		pCommandBuffers = ffi.new("struct VkCommandBuffer_T *[1]", setup_cmd),
		signalSemaphoreCount = 0,
		pSignalSemaphores = nil
	}), nil)
	queue:WaitIdle()
	device:FreeCommandBuffers(cmd_pool, 1, ffi.new("struct VkCommandBuffer_T *[1]", setup_cmd))
	setup_cmd = nil
end

local function prepare_texture_image(device, tex_colors, tex, tiling, usage, required_props, memory_properties)
	tex.width = 2
	tex.height = 2

	local image = device:CreateImage({
		imageType = vk.e.VK_IMAGE_TYPE_2D,
		format = vk.e.VK_FORMAT_B8G8R8A8_UNORM,
		extent = {tex.width, tex.height, 1},
		mipLevels = 1,
		arrayLayers = 1,
		samples = vk.e.VK_SAMPLE_COUNT_1_BIT,
		tiling = tiling,
	})

	tex.image = image

	local memory_requirements = device:GetImageMemoryRequirements(image)

	local memory = device:AllocateMemory({
		allocationSize = memory_requirements.size,
		memoryTypeIndex = memory_type_from_properties(memory_properties, memory_requirements.memoryTypeBits, required_props),
	})

	tex.memory = memory

	device:BindImageMemory(image, memory, 0)


	if bit.band(required_props, vk.e.VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT) ~= 0 then
		local layout = device:GetImageSubresourceLayout(image, ffi.new("struct VkImageSubresource[0]", {
			aspectMask = vk.e.VK_IMAGE_ASPECT_COLOR_BIT,
			mipLevel = 0,
			arrayLayer = 0,
		}))

		tex.imageLayout = tonumber(vk.e.VK_IMAGE_ASPECT_COLOR_BIT)

		local data = ffi.new("void *[1]")

		device:MapMemory(memory, 0, memory_requirements.size, 0, data)

		--[[for y = 0, tex.height - 1 do
			local row = data[0] + layout.rowPitch * y
			for x = 0, tex.width - 1 do
				row[0] = tex_colors[bit.xor(bit.band(x, 1), bit.band(y, 1)) + 1]
			end
		end]]

		device:UnmapMemory(memory)
	end

	set_image_layout(device, cmd_pool, image, vk.e.VK_IMAGE_ASPECT_COLOR_BIT, vk.e.VK_IMAGE_LAYOUT_UNDEFINED, vk.e.VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL)
end

local extensions = {
	"VK_EXT_debug_report",
}

local validation_layers = {
	"VK_LAYER_LUNARG_threading",
	"VK_LAYER_LUNARG_mem_tracker",
	"VK_LAYER_LUNARG_object_tracker",
	"VK_LAYER_LUNARG_draw_state",
	"VK_LAYER_LUNARG_param_checker",
	"VK_LAYER_LUNARG_swapchain",
	"VK_LAYER_LUNARG_device_limits",
	"VK_LAYER_LUNARG_image",
}

glfw.SetErrorCallback(function(_, str) print(ffi.string(str)) end)
glfw.Init()
glfw.WindowHint(glfw.e.GLFW_CLIENT_API, glfw.e.GLFW_NO_API)
local window = glfw.CreateWindow(1024, 768, "vulkan", nil, nil)

for _, ext in ipairs(glfw.GetRequiredInstanceExtensions()) do
	table.insert(extensions, ext)
end

local instance = assert(vk.CreateInstance({
	pApplicationInfo = vk.structs.ApplicationInfo({
		pApplicationName = "goluwa",
		applicationVersion = 0,
		pEngineName = "goluwa",
		engineVersion = 0,
		apiVersion = vk.macros.MAKE_VERSION(1, 0, 2),
	}),

	enabledLayerCount = #validation_layers,
	ppEnabledLayerNames = vk.util.StringList(validation_layers),

	enabledExtensionCount = #extensions,
	ppEnabledExtensionNames = vk.util.StringList(extensions),
}))

if instance:LoadProcAddr("vkCreateDebugReportCallbackEXT") then
	instance:CreateDebugReportCallback({
		flags = bit.bor(
			vk.e.VK_DEBUG_REPORT_INFORMATION_BIT_EXT,
			vk.e.VK_DEBUG_REPORT_PERFORMANCE_WARNING_BIT_EXT,
			vk.e.VK_DEBUG_REPORT_ERROR_BIT_EXT,
			vk.e.VK_DEBUG_REPORT_WARNING_BIT_EXT
		),
		pfnCallback = function(msgFlags, objType, srcObject, location, msgCode, pLayerPrefix, pMsg, pUserData)
			print(msgFlags, objType, srcObject, location, msgCode, ffi.string(pLayerPrefix), ffi.string(pMsg), pUserData)

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

for _, gpu in ipairs(instance:GetPhysicalDevices()) do
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

			local queue = device:GetQueue(queue_index, 0)
			local cmd_pool = device:CreateCommandPool({
				queueFamilyIndex = queue_index,
			})

			local draw_cmd = device:AllocateCommandBuffers({
					commandPool = cmd_pool,
					level = vk.e.VK_COMMAND_BUFFER_LEVEL_PRIMARY,
					commandBufferCount = 1,
			})

			local surface = glfw.CreateWindowSurface(instance, window, nil)

			local surface_formats = gpu:GetSurfaceFormats(surface)
			local surface_capabilities = gpu:GetSurfaceCapabilities(surface)
			local memory_properties = gpu:GetMemoryProperties()

			local prefered_format = surface_formats[1].format ~= vk.e.VK_FORMAT_UNDEFINED and surface_formats[1].format or vk.e.VK_FORMAT_B8G8R8A8_UNORM

			local w, h = surface_capabilities.currentExtent.width, surface_capabilities.currentExtent.height

			if w == 0xFFFFFFFF then
				w = 1024
				h = 768
			end

			local depth = {}
			local screen_buffers = {}
			local textures = {{color = 0xffff0000}, {color = 0xff00ff00}}

			do -- surface buffers
				local swap_chain = device:CreateSwapchain({
					surface = surface,
					minImageCount = math.min(surface_capabilities.minImageCount + 1, surface_capabilities.maxImageCount == 0 and math.huge or surface_capabilities.maxImageCount),
					imageFormat = prefered_format,
					imagecolorSpace = surface_formats[1].colorSpace,
					imageExtent = {w, h},
					imageUse = vk.e.VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT,
					preTransform = bit.band(surface_capabilities.supportedTransforms, vk.e.VK_SURFACE_TRANSFORM_IDENTITY_BIT_KHR) ~= 0 and vk.e.VK_SURFACE_TRANSFORM_IDENTITY_BIT_KHR or surface_capabilities.currentTransform,
					compositeAlpha = vk.e.VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR,
					imageArrayLayers = 1,
					imageSharingMode = vk.e.VK_SHARING_MODE_EXCLUSIVE,
					queueFamilyIndexCount = 0,
					pQueueFamilyIndices = nil,
					presentMode = vk.e.VK_PRESENT_MODE_FIFO_KHR,
					oldSwapchain = nil,
					clipped = 1,
				})

				for i, image in ipairs(device:GetSwapchainImages(swap_chain)) do
					screen_buffers[i] = {}

					set_image_layout(
						device,
						cmd_pool,
						image,
						vk.e.VK_IMAGE_ASPECT_COLOR_BIT,
						vk.e.VK_IMAGE_LAYOUT_UNDEFINED,
						vk.e.VK_IMAGE_LAYOUT_PRESENT_SRC_KHR
					)

					-- FAILS TO REMAP ^^^^ enums?

					local view = device:CreateImageView({
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

					screen_buffers[i].view = view
					screen_buffers[i].image = image
				end
			end

			do -- depth
				local format = vk.e.VK_FORMAT_D16_UNORM

				local image = device:CreateImage({
					imageType = vk.e.VK_IMAGE_TYPE_2D,
					format = format,
					extent = {w, h, 1},
					mipLevels = 1,
					arrayLayers = 1,
					samples = vk.e.VK_SAMPLE_COUNT_1_BIT,
					tiling = vk.e.VK_IMAGE_TILING_OPTIMAL,
					usage = vk.e.VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT,
					flags = 0,
					queueFamilyIndexCount = 0,
				})

				local memory_requirements = device:GetImageMemoryRequirements(image)

				local memory = device:AllocateMemory({
					allocationSize = memory_requirements.size,
					memoryTypeIndex = memory_type_from_properties(memory_properties, memory_requirements.memoryTypeBits, 0),
				})

				device:BindImageMemory(image, memory, 0)

				set_image_layout(device, cmd_pool, image, vk.e.VK_IMAGE_ASPECT_DEPTH_BIT, vk.e.VK_IMAGE_LAYOUT_UNDEFINED, vk.e.VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL)

				local view = device:CreateImageView({
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

				depth.image = image
				depth.memory = memory
				depth.view = view
				depth.format = format
			end

			do -- textures
				local format = vk.e.VK_FORMAT_B8G8R8A8_UNORM
				local props = gpu:GetFormatProperties(format)

				for _, tex_info in ipairs(textures) do

					local staging_texture = {}
					prepare_texture_image(
						device,
						tex_info.color,
						staging_texture,
						vk.e.VK_IMAGE_TILING_LINEAR,
						vk.e.VK_IMAGE_USAGE_TRANSFER_SRC_BIT,
						vk.e.VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT,
						memory_properties
					)

					set_image_layout(
						device,
						cmd_pool,
						staging_texture.image,
						vk.e.VK_IMAGE_ASPECT_COLOR_BIT,
						staging_texture.imageLayout,
						vk.e.VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL
					)

					prepare_texture_image(
						device,
						tex_info.color,
						tex_info,
						VK_IMAGE_TILING_OPTIMAL,
						bit.bor(vk.e.VK_IMAGE_USAGE_TRANSFER_DST_BIT, vk.e.VK_IMAGE_USAGE_SAMPLED_BIT),
						vk.e.VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
						memory_properties
					)
					set_image_layout(
						device,
						cmd_pool,
						tex_info.image,
						vk.e.VK_IMAGE_ASPECT_COLOR_BIT,
						tex_info.imageLayout,
						vk.e.VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL
					)

					setup_cmd:CopyImage(staging_texture.image, vk.e.VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, tex_info.image, vk.e.VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, ffi.new("struct VkImageCopy", {
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
					set_image_layout(
						device,
						cmd_pool,
						tex_info.image,
						vk.e.VK_IMAGE_ASPECT_COLOR_BIT,
						vk.e.VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
						tex_info.imageLayout
					)
					flush_setup_cmd(cmd_pool, device, queue)
					device:DestroyImage(staging_texture.image, nil)
					device:FreeMemory(staging_texture.memory, nil)
					tex_info.sampler = device:CreateSampler({
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

					tex_info.view = device:CreateImageView({
						viewType = vk.e.VK_IMAGE_VIEW_TYPE_2D,
						image = tex_info.image,
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
					print(i, "????????")
				end
			end

			print("!!!")

			return
		end
	end
end

