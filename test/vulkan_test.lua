collectgarbage("stop") -- TODO

local ffi = require("ffi")

package.path = package.path .. ";./../examples/?.lua"

local vk = require("vulkan/libvulkan")

_G.FFI_LIB = "../examples/glfw/glfw/src/libglfw.so"
local glfw = require("glfw/libglfw")
_G.FFI_LIB = nil

local DEMO = {}

DEMO.InstanceExtensions = {
	"VK_EXT_debug_report",
}

DEMO.DeviceExtensions = {
	"VK_NV_glsl_shader",
}

DEMO.ValidationLayers = {
	"VK_LAYER_LUNARG_threading",
	"VK_LAYER_LUNARG_mem_tracker",
	"VK_LAYER_LUNARG_object_tracker",
	--"VK_LAYER_LUNARG_draw_state",
	"VK_LAYER_LUNARG_param_checker",
	"VK_LAYER_LUNARG_swapchain",
	"VK_LAYER_LUNARG_device_limits",
	"VK_LAYER_LUNARG_image",
	--"VK_LAYER_LUNARG_api_dump",
}


DEMO.DebugFlags = {
	vk.e.DEBUG_REPORT_INFORMATION_BIT_EXT,
	vk.e.DEBUG_REPORT_WARNING_BIT_EXT,
	vk.e.DEBUG_REPORT_PERFORMANCE_WARNING_BIT_EXT,
	vk.e.DEBUG_REPORT_ERROR_BIT_EXT,
	vk.e.DEBUG_REPORT_DEBUG_BIT_EXT,
}

function DEMO:PrepareWindow()
	glfw.SetErrorCallback(function(_, str) io.write(string.format("GLFW Error: %s\n", ffi.string(str))) end)
	glfw.Init()
	glfw.WindowHint(glfw.e.GLFW_CLIENT_API, glfw.e.GLFW_NO_API)

	for _, ext in ipairs(glfw.GetRequiredInstanceExtensions()) do
		table.insert(self.InstanceExtensions, ext)
	end

	self.Window = glfw.CreateWindow(1024, 768, "vulkan", nil, nil)
end

function DEMO:PrepareInstance()
	local instance = vk.CreateInstance({
		pApplicationInfo = vk.s.ApplicationInfo({
			pApplicationName = "goluwa",
			applicationVersion = 0,
			pEngineName = "goluwa",
			engineVersion = 0,
			apiVersion = vk.macros.MAKE_VERSION(1, 0, 2),
		}),

		enabledLayerCount = #self.ValidationLayers,
		ppEnabledLayerNames = vk.util.StringList(self.ValidationLayers),

		enabledExtensionCount = #self.InstanceExtensions,
		ppEnabledExtensionNames = vk.util.StringList(self.InstanceExtensions),
	})

	if instance:LoadProcAddr("vkCreateDebugReportCallbackEXT") then
		instance:CreateDebugReportCallback({
			flags = bit.bor(unpack(self.DebugFlags)),
			pfnCallback = function(msgFlags, objType, srcObject, location, msgCode, pLayerPrefix, pMsg, pUserData)

				local level = 3
				local info = debug.getinfo(level, "Sln")
				local lines = {}
				for i = 3, 10 do
					local info = debug.getinfo(i, "Sln")
					if not info or info.currentline == -1 then break end
					table.insert(lines, info.currentline)
				end
				io.write(string.format("Line %s %s: %s: %s\n", table.concat(lines, ", "), info.name or "unknown", ffi.string(pLayerPrefix), ffi.string(pMsg)))

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
			if bit.band(info.queueFlags, vk.e.QUEUE_GRAPHICS_BIT) ~= 0 then

				local queue_index = i - 1

				local device = gpu:CreateDevice({
					queueCreateInfoCount = 1,

					enabledLayerCount = #self.ValidationLayers,
					ppEnabledLayerNames = vk.util.StringList(self.ValidationLayers),

					enabledExtensionCount = #self.DeviceExtensions,
					ppEnabledExtensionNames = vk.util.StringList(self.DeviceExtensions),

					pQueueCreateInfos = vk.s.DeviceQueueCreateInfo({
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

	local prefered_format = self.SurfaceFormats[1].format ~= vk.e.FORMAT_UNDEFINED and self.SurfaceFormats[1].format or vk.e.FORMAT_B8G8R8A8_UNORM

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
		imageUse = vk.e.IMAGE_USAGE_COLOR_ATTACHMENT_BIT,
		preTransform = bit.band(self.SurfaceCapabilities.supportedTransforms, vk.e.SURFACE_TRANSFORM_IDENTITY_BIT_KHR) ~= 0 and vk.e.SURFACE_TRANSFORM_IDENTITY_BIT_KHR or self.SurfaceCapabilities.currentTransform,
		compositeAlpha = vk.e.COMPOSITE_ALPHA_OPAQUE_BIT_KHR,
		imageArrayLayers = 1,
		imageSharingMode = vk.e.SHARING_MODE_EXCLUSIVE,

		queueFamilyIndexCount = 0,
		pQueueFamilyIndices = nil,

		presentMode = vk.e.PRESENT_MODE_FIFO_KHR,
		oldSwapchain = nil,
		clipped = 1,
	})

	for i, image in ipairs(self.Device:GetSwapchainImages(swap_chain)) do
		self.SurfaceBuffers[i] = {}

		self:SetImageLayout(image, vk.e.IMAGE_ASPECT_COLOR_BIT, vk.e.IMAGE_LAYOUT_UNDEFINED, vk.e.IMAGE_LAYOUT_PRESENT_SRC_KHR)

		local view = self.Device:CreateImageView({
			viewType = vk.e.IMAGE_VIEW_TYPE_2D,
			image = image,
			format = prefered_format,
			flags = 0,
			components = {
				r = vk.e.COMPONENT_SWIZZLE_R,
				g = vk.e.COMPONENT_SWIZZLE_G,
				b = vk.e.COMPONENT_SWIZZLE_B,
				a = vk.e.COMPONENT_SWIZZLE_A
			},
			subresourceRange = {vk.e.IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1},
		})

		self.SurfaceBuffers[i].view = view
		self.SurfaceBuffers[i].image = image
	end

	self.SwapChain = swap_chain
end

function DEMO:CreateImage(width, height, format, usage, tiling, required_props)
	local image = self.Device:CreateImage({
		imageType = vk.e.IMAGE_TYPE_2D,
		format = format,
		extent = {width, height, 1},
		mipLevels = 1,
		arrayLayers = 1,
		samples = vk.e.SAMPLE_COUNT_1_BIT,
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

	local format = vk.e.FORMAT_D16_UNORM

	local image, memory = self:CreateImage(self.Width, self.Height, format, vk.e.IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT, vk.e.IMAGE_TILING_OPTIMAL, 0)

	self:SetImageLayout(image, vk.e.IMAGE_ASPECT_DEPTH_BIT, vk.e.IMAGE_LAYOUT_UNDEFINED, vk.e.IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL)

	local view = self.Device:CreateImageView({
		viewType = vk.e.IMAGE_VIEW_TYPE_2D,
		image = image,
		format = format,
		flags = 0,
		subresourceRange = {vk.e.IMAGE_ASPECT_DEPTH_BIT, 0, 1, 0, 1},
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

		local image, memory, memory_requirements = self:CreateImage(tex.width, tex.height, vk.e.FORMAT_B8G8R8A8_UNORM, usage, tiling, required_props)

		tex.memory = memory
		tex.image = image

		if bit.band(required_props, vk.e.MEMORY_PROPERTY_HOST_VISIBLE_BIT) ~= 0 then
			tex.imageLayout = vk.e.IMAGE_ASPECT_COLOR_BIT

			self.Device:MapMemory(memory, 0, memory_requirements.size, 0, "uint32_t", function(data)
				for y = 0, tex.height - 1 do
					for x = 0, tex.width - 1 do
						data[x * y] = math.random(0xFFFFFFFF)
					end
				end
			end)
		end

		self:SetImageLayout(image, vk.e.IMAGE_ASPECT_COLOR_BIT, vk.e.IMAGE_LAYOUT_UNDEFINED, vk.e.IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL)
	end

	function DEMO:CreateTexture()
		local texture = {}

		local staging_texture = {}
		prepare(self, staging_texture, vk.e.IMAGE_TILING_LINEAR, vk.e.IMAGE_USAGE_TRANSFER_SRC_BIT, vk.e.MEMORY_PROPERTY_HOST_VISIBLE_BIT)
		self:SetImageLayout(staging_texture.image, vk.e.IMAGE_ASPECT_COLOR_BIT, staging_texture.imageLayout, vk.e.IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL)

		prepare(self, texture, vk.e.IMAGE_TILING_OPTIMAL, bit.bor(vk.e.IMAGE_USAGE_TRANSFER_DST_BIT, vk.e.IMAGE_USAGE_SAMPLED_BIT), vk.e.MEMORY_PROPERTY_DEVICE_LOCAL_BIT)
		self:SetImageLayout(texture.image, vk.e.IMAGE_ASPECT_COLOR_BIT, texture.imageLayout, vk.e.IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL)

		self.SetupCMD:CopyImage(
			staging_texture.image,
			vk.e.IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
			texture.image,
			vk.e.IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,

			1,
			vk.s.ImageCopyArray{
				srcSubresource = {vk.e.IMAGE_ASPECT_COLOR_BIT, 0, 0, 1},
				srcOffset = {0, 0, 0},
				dstSubresource = {vk.e.IMAGE_ASPECT_COLOR_BIT, 0, 0, 1},
				dstOffset = {0, 0, 0},
				extent = {
					staging_texture.width,
					staging_texture.height,
					1
				},
			}
		)

		self:SetImageLayout(texture.image, vk.e.IMAGE_ASPECT_COLOR_BIT, vk.e.IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, texture.imageLayout)
		self:FlushSetupCMD()

		self.Device:DestroyImage(staging_texture.image, nil)
		self.Device:FreeMemory(staging_texture.memory, nil)

		texture.sampler = self.Device:CreateSampler({
			magFilter = vk.e.FILTER_NEAREST,
			minFilter = vk.e.FILTER_NEAREST,
			mipmapMode = vk.e.SAMPLER_MIPMAP_MODE_NEAREST,
			addressModeU = vk.e.SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE,
			addressModeV = vk.e.SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE,
			addressModeW = vk.e.SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE,
			ipLodBias = 0.0,
			anisotropyEnable = 0,
			maxAnisotropy = 1,
			compareOp = vk.e.COMPARE_OP_NEVER,
			minLod = 0.0,
			maxLod = 0.0,
			borderColor = vk.e.BORDER_COLOR_FLOAT_OPAQUE_WHITE,
			unnormalizedCoordinates = 0,
		})

		texture.view = self.Device:CreateImageView({
			viewType = vk.e.IMAGE_VIEW_TYPE_2D,
			image = texture.image,
			format = self.TextureFormat,
			flags = 0,
			components = {
				r = vk.e.COMPONENT_SWIZZLE_R,
				g = vk.e.COMPONENT_SWIZZLE_G,
				b = vk.e.COMPONENT_SWIZZLE_B,
				a = vk.e.COMPONENT_SWIZZLE_A
			},
			subresourceRange = {vk.e.IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1},
		})

		return texture
	end
end

function DEMO:PrepareTextures()
	self.TextureFormat = vk.e.FORMAT_B8G8R8A8_UNORM
	self.Texture = self:CreateTexture()
end

function DEMO:PrepareVertices()
	local vb = ffi.new("float[3][5]", {
	--  position             	texcoord
		{ -1.0, -1.0,  0.25,	0.0, 0.0 },
		{  1.0, -1.0,  0.25,	1.0, 0.0 },
		{  0.0,  1.0,  1.0,		0.5, 1.0 },
	})
	local buffer = self.Device:CreateBuffer({
		size = ffi.sizeof(vb),
		usage = vk.e.BUFFER_USAGE_VERTEX_BUFFER_BIT,
	})

	local memory_requirements = self.Device:GetBufferMemoryRequirements(buffer)

	local memory = self.Device:AllocateMemory({
		allocationSize = memory_requirements.size,
		memoryTypeIndex = self:GetMemoryTypeFromProperties(memory_requirements.memoryTypeBits, vk.e.MEMORY_PROPERTY_HOST_VISIBLE_BIT),
	})

	self.Device:MapMemory(memory, 0, memory_requirements.size, 0, "float", function(data)
		ffi.copy(data, vb, memory_requirements.size)
	end)

	self.Device:BindBufferMemory(buffer, memory, 0)

	self.Vertices = vk.s.PipelineVertexInputStateCreateInfo({
		vertexBindingDescriptionCount = 1,
		pVertexBindingDescriptions = vk.s.VertexInputBindingDescriptionArray{
			{
				binding = 0,
				stride = ffi.sizeof(vb[0]),
				inputRate = vk.e.VERTEX_INPUT_RATE_VERTEX
			}
		},
		vertexAttributeDescriptionCount = 2,
		pVertexAttributeDescriptions = vk.s.VertexInputAttributeDescriptionArray{
			{
				binding = 0,
				location = 0,
				format = vk.e.FORMAT_R32G32B32_SFLOAT,
				offset = 0,
			},
			{
				binding = 0,
				location = 1,
				format = vk.e.FORMAT_R32G32_SFLOAT,
				offset = ffi.sizeof("float") * 3,
			}
		},
	})
	self.VerticesBuffer = buffer
	self.VerticesMemory = memory
end

function DEMO:PrepareDescriptorLayout()
	self.DescriptorLayout = self.Device:CreateDescriptorSetLayout({
		bindingCount = 1,
		pBindings = vk.s.DescriptorSetLayoutBindingArray{
			{
				binding = 0,
				descriptorType = vk.e.DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
				descriptorCount = 1,
				stageFlags = vk.e.SHADER_STAGE_FRAGMENT_BIT,
				pImmutableSamplers = nil,
			}
		},
	})

	self.PipelineLayout = self.Device:CreatePipelineLayout({
		setLayoutCount = 1,
		pSetLayouts = vk.s.DescriptorSetLayoutArray{
			self.DescriptorLayout
		},
	})
end

function DEMO:PrepareRenderPass()
	self.RenderPass = self.Device:CreateRenderPass({
		attachmentCount = 2,
		pAttachments = vk.s.AttachmentDescriptionArray{
			{
				format = self.TextureFormat,
				samples = vk.e.SAMPLE_COUNT_1_BIT,
				loadOp = vk.e.ATTACHMENT_LOAD_OP_CLEAR,
				storeOp = vk.e.ATTACHMENT_STORE_OP_STORE,
				stencilLoadOp = vk.e.ATTACHMENT_LOAD_OP_DONT_CARE,
				stencilStoreOp = vk.e.ATTACHMENT_STORE_OP_DONT_CARE,
				initialLayout = vk.e.IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
				finalLayout = vk.e.IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
			},
			{
				format = self.DepthBuffer.format,
				samples = vk.e.SAMPLE_COUNT_1_BIT,
				loadOp = vk.e.ATTACHMENT_LOAD_OP_CLEAR,
				storeOp = vk.e.ATTACHMENT_STORE_OP_DONT_CARE,
				stencilLoadOp = vk.e.ATTACHMENT_LOAD_OP_DONT_CARE,
				stencilStoreOp = vk.e.ATTACHMENT_STORE_OP_DONT_CARE,
				initialLayout = vk.e.IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
				finalLayout = vk.e.IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
			}
		},
		subpassCount = 1,
		pSubpasses = vk.s.SubpassDescriptionArray{
			{
				pipelineBindPoint = vk.e.PIPELINE_BIND_POINT_GRAPHICS,
				flags = 0,

				inputAttachmentCount = 0,
				pInputAttachments = nil,

				colorAttachmentCount = 1,
				pColorAttachments = vk.s.AttachmentReferenceArray{
					{
						attachment = 0,
						layout = vk.e.IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
					}
				},

				pResolveAttachments = nil,
				pDepthStencilAttachment = vk.s.AttachmentReference{
					attachment = 0,
					layout = vk.e.IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
				},

				preserveAttachmentCount = 0,
				pPreserveAttachments = nil,
			}
		},

		dependencyCount = 0,
		pDependencies = nil,
	})
end

function DEMO:PreparePipeline()
	local vertex_code = [[#version 400
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable
layout (location = 0) in vec4 pos;
layout (location = 1) in vec2 attr;
layout (location = 0) out vec2 texcoord;
void main() {
   texcoord = attr;
   gl_Position = pos;
}]]

	local fragment_code = [[#version 400
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable
layout (binding = 0) uniform sampler2D tex;
layout (location = 0) in vec2 texcoord;
layout (location = 0) out vec4 uFragColor;
void main() {
   uFragColor = texture(tex, texcoord);
}]]

	local cache = self.Device:CreatePipelineCache({})

	self.Pipeline = self.Device:CreateGraphicsPipelines(cache, 1, {
		{
			layout = self.PipelineLayout,
			pVertexInputState = self.Vertices,
			renderPass = self.RenderPass,

			stageCount = 2,
			pStages = vk.s.PipelineShaderStageCreateInfoArray{
				{
					stage = vk.e.SHADER_STAGE_VERTEX_BIT,
					module = self.Device:CreateShaderModule({
						codeSize = #vertex_code,
						pCode = ffi.cast("uint32_t *", vertex_code),
					}),
					pName = "main",
				},
				{
					stage = vk.e.SHADER_STAGE_FRAGMENT_BIT,
					module = self.Device:CreateShaderModule({
						codeSize = #fragment_code,
						pCode = ffi.cast("uint32_t *", fragment_code),
					}),
					pName = "main",
				},
			},

			pInputAssemblyState = vk.s.PipelineInputAssemblyStateCreateInfo{
				topology = vk.e.PRIMITIVE_TOPOLOGY_TRIANGLE_LIST
			},

			pRasterizationState = vk.s.PipelineRasterizationStateCreateInfo{
				polygonMode = vk.e.POLYGON_MODE_FILL,
				cullMode = vk.e.CULL_MODE_BACK_BIT,
				frontFace = vk.e.FRONT_FACE_CLOCKWISE,
				depthClampEnable = 0,
				rasterizerDiscardEnable = 0,
				depthBiasEnable = 0,
			},

			pColorBlendStage = vk.s.PipelineColorBlendStateCreateInfo{
				attachmentCount = 1,
				pAttachments = vk.s.PipelineColorBlendAttachmentStateArray{
					{
						colorWriteMask = 0xf,
						blendEnable = 0,
					}
				}
			},

			pMultisampleState = vk.s.PipelineMultisampleStateCreateInfo{
				pSampleMask = nil,
				rasterizationSamples = vk.e.SAMPLE_COUNT_1_BIT,
			},

			pViewportState = vk.s.PipelineViewportStateCreateInfo
			{
				viewportCount = 1,
				scissorCount = 1,
			},

			pDepthStencilState = vk.s.PipelineDepthStencilStateCreateInfo{
				depthTestEnable = 1,
				depthWriteEnable = 1,
				depthCompareOp = vk.e.COMPARE_OP_LESS_OR_EQUAL,
				depthBoundsTestEnable = 0,
				stencilTestEnable = 0,
				back = {
					failOp = vk.e.STENCIL_OP_KEEP,
					passOp = vk.e.STENCIL_OP_KEEP,
					compareOp = vk.e.COMPARE_OP_ALWAYS,
				},
				front = {
					failOp = vk.e.STENCIL_OP_KEEP,
					passOp = vk.e.STENCIL_OP_KEEP,
					compareOp = vk.e.COMPARE_OP_ALWAYS,
				},
			},

			pDynamicState = vk.s.PipelineDynamicStateCreateInfo{
				dynamicStateCount = 2,
				pDynamicStates = ffi.new("enum VkDynamicState[2]",
					vk.e.DYNAMIC_STATE_VIEWPORT,
					vk.e.DYNAMIC_STATE_SCISSOR
				),
			},
		}
	})

	self.Device:DestroyPipelineCache(cache, nil)
end

function DEMO:PrepareDescriptorPool()
	self.DescriptorPool = self.Device:CreateDescriptorPool({
		maxSets = 1,

		poolSizeCount = 1,
		pPoolSizes = vk.s.DescriptorPoolSizeArray{
			{
				type = vk.e.DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
				descriptorCount = 1
			},
		}
	})
end

function DEMO:PrepareDescriptorSet()
	self.DescriptorSet = self.Device:AllocateDescriptorSets({
		descriptorPool = self.DescriptorPool,

		descriptorSetCount = 1,
		pSetLayouts = vk.s.DescriptorSetLayoutArray{
			self.DescriptorLayout
		},
	})

	self.Device:UpdateDescriptorSets(1, vk.s.WriteDescriptorSet({
		dstSet = self.DescriptorSet,
		descriptorCount = 1,
		descriptorType = vk.e.DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
		pImageInfo = vk.s.DescriptorImageInfoArray{
			{
				sampler = self.Texture.sampler,
				imageView = self.Texture.view,
				imageLayout = vk.e.IMAGE_LAYOUT_GENERAL,
			}
		}
	}), 0, nil)
end

function DEMO:PrepareFramebuffers()
	self.Framebuffers = {}

	for i, buffer in ipairs(self.SurfaceBuffers) do
		self.Framebuffers[i] = self.Device:CreateFramebuffer({
			renderPass = self.RenderPass,

			attachmentCount = 2,
			pAttachments = vk.s.ImageViewArray{
				buffer.view,
				self.DepthBuffer.view
			},

			width = self.Width,
			height = self.Height,
			layers = 1,
		})
	end
end

function DEMO:Initialize()
	self:PrepareWindow() -- create a window
	self:PrepareInstance() -- create a vulkan instance
	self:PrepareDevice() -- find and gpu and use it
	self:PrepareSurface() -- create the window surface to render on
	self:PrepareDepthTexture() -- create the depth buffer
	self:PrepareTextures()
	self:PrepareVertices()
	self:PrepareDescriptorLayout()
	self:PrepareRenderPass()
	self:PreparePipeline()
	self:PrepareDescriptorPool()
	self:PrepareDescriptorSet()
	self:PrepareFramebuffers()

	self.DrawCMD = self.Device:AllocateCommandBuffers({
		commandPool = self.DeviceCommandPool,
		level = vk.e.COMMAND_BUFFER_LEVEL_PRIMARY,
		commandBufferCount = 1,
	})
	--while glfw.WindowShouldClose(self.Window) == 0 do
	do
		glfw.PollEvents()

		local semaphore = self.Device:CreateSemaphore({})
		local index = ffi.new("unsigned int[1]") local status = vk.AcquireNextImageKHR(self.Device, self.SwapChain, 0xFFFFFFFFFFFFFFFFULL, semaphore, nil, index) index = index[0] + 1
		print(index, "!!!", status)

		self:SetImageLayout(self.SurfaceBuffers[index].image, vk.e.IMAGE_ASPECT_COLOR_BIT, vk.e.IMAGE_LAYOUT_PRESENT_SRC_KHR, vk.e.IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL)
		self:FlushSetupCMD()

			self.DrawCMD:Begin(vk.s.CommandBufferBeginInfo({
				flags = 0,
				pInheritanceInfo = vk.s.CommandBufferInheritanceInfo({
					renderPass = nil,
					subpass = 0,
					framebuffer = nil,
					offclusionQueryEnable = 0,
					queryFlags = 0,
					pipelineStatistics = 0,
				 })
			}))

				self.DrawCMD:BeginRenderPass(vk.s.RenderPassBeginInfo{
					renderPass = self.RenderPass,
					framebuffer = self.Framebuffers[index],
					renderArea = {offset = {0, 0}, extent = {self.Width, self.Height}},
					clearValueCount = 2,
					pClearValues = ffi.new("union VkClearValue[2]", {color = {float32 = {0.2, 0.2, 0.2, 0.2}}}, {depthStencil = {1, 0}}),
				}, vk.e.SUBPASS_CONTENTS_INLINE)

					self.DrawCMD:BindPipeline(vk.e.PIPELINE_BIND_POINT_GRAPHICS, self.Pipeline)
					self.DrawCMD:BindDescriptorSets(vk.e.PIPELINE_BIND_POINT_GRAPHICS, self.PipelineLayout, 0, 1, vk.s.DescriptorSetArray{self.DescriptorSet}, 0, nil)
					self.DrawCMD:SetViewport(0,1, vk.s.Viewport{0,0,self.Height,self.Width, 0,1,})
					self.DrawCMD:SetScissor(0,1, vk.s.Rect2D{offset = {0, 0}, extent = {self.Height, self.Width}})
					self.DrawCMD:BindVertexBuffers(0, 1, vk.s.BufferArray{self.VerticesBuffer}, ffi.new("unsigned long[1]", 0))
					self.DrawCMD:Draw(3,1,0,0)

				self.DrawCMD:EndRenderPass()

				self.DrawCMD:PipelineBarrier(
					vk.e.PIPELINE_STAGE_ALL_COMMANDS_BIT,
					vk.e.PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT,
					0,

					0,
					nil,

					0,
					nil,

					1,
					vk.s.ImageMemoryBarrierArray{
						{
							srcAccessMask = vk.e.ACCESS_COLOR_ATTACHMENT_WRITE_BIT,
							dstAccessMask = vk.e.ACCESS_MEMORY_READ_BIT,
							oldLayout = vk.e.IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
							newLayout = vk.e.IMAGE_LAYOUT_PRESENT_SRC_KHR,
							srcQueueFamilyIndex = bit.bnot(0ULL),--VK_QUEUE_FAMILY_IGNORED,
							dstQueueFamilyIndex = bit.bnot(0ULL),--VK_QUEUE_FAMILY_IGNORED,
							subresourceRange = {vk.e.IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1},
						}
					}
				)

			self.DrawCMD:End()

		self.DeviceQueue:Submit(1, vk.s.SubmitInfo({
			pWaitDstStageMask = ffi.new("enum VkPipelineStageFlagBits [1]", vk.e.PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT),

			waitSemaphoreCount = 1,
			pWaitSemaphores = vk.s.SemaphoreArray{
				semaphore
			},

			signalSemaphoreCount = 0,
			pSignalSemaphores = nil,

			commandBufferCount = 1,
			pCommandBuffers = vk.s.CommandBufferArray{
				self.DrawCMD
			},
		}), nil)

		vk.QueuePresentKHR(self.DeviceQueue, vk.s.PresentInfoKHR{
			pImageIndices = ffi.new("unsigned int [1]", index - 1),

			swapchainCount = 1,
			pSwapchains = vk.s.SwapchainKHRArray{
				self.SwapChain
			},
		})

		self.Device:WaitIdle()
		self.Device:DestroySemaphore(semaphore, nil)
	end


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
				level = vk.e.COMMAND_BUFFER_LEVEL_PRIMARY,
				commandBufferCount = 1,
		})

		self.SetupCMD:Begin(vk.s.CommandBufferBeginInfo{
			flags = 0,
			pInheritanceInfo = vk.s.CommandBufferInheritanceInfo({
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

	if new_image_layout == vk.e.IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL then
		mask = vk.e.ACCESS_TRANSFER_READ_BIT
	elseif new_image_layout == vk.e.IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL then
		mask = vk.e.ACCESS_COLOR_ATTACHMENT_WRITE_BIT
	elseif new_image_layout == vk.e.IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL then
		mask = vk.e.ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT
	elseif new_image_layout == vk.e.IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL then
		mask = bit.bor(vk.e.ACCESS_SHADER_READ_BIT, vk.e.ACCESS_INPUT_ATTACHMENT_READ_BIT)
	end

	self.SetupCMD:PipelineBarrier(
		vk.e.PIPELINE_STAGE_TOP_OF_PIPE_BIT,
		vk.e.PIPELINE_STAGE_TOP_OF_PIPE_BIT,
		0,
		0,
		nil,
		0,
		nil,
		1,
		vk.s.ImageMemoryBarrier({
			srcAccessMask = 0,
			dstAccessMask = mask,
			oldLayout = old_image_layout,
			newLayout = new_image_layout,
			image = image,
			subresourceRange = {aspectMask, 0, 1, 0, 1},
		})
	)
end

function DEMO:FlushSetupCMD()
	self.SetupCMD:End()
	self.DeviceQueue:Submit(1, vk.s.SubmitInfo({
		waitSemaphoreCount = 0,
		pWaitSemaphores = nil,
		pWaitDstStageMask = nil,

		commandBufferCount = 1,
		pCommandBuffers = vk.s.CommandBufferArray{
			self.SetupCMD
		},

		signalSemaphoreCount = 0,
		pSignalSemaphores = nil
	}), nil)

	self.DeviceQueue:WaitIdle()

	self.Device:FreeCommandBuffers(
		self.DeviceCommandPool,
		1,
		vk.s.CommandBufferArray{
			self.SetupCMD
		}
	)
	self.SetupCMD = nil
end

DEMO:Initialize()