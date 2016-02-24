collectgarbage("stop") -- TODO

local ffi = require("ffi")

package.path = package.path .. ";./../examples/?.lua"

local vk = require("vulkan/libvulkan")

_G.FFI_LIB = "../examples/glfw/glfw/src/libglfw.so"
local glfw = require("glfw/libglfw")
_G.FFI_LIB = nil

local matrix_type = require("matrix44")

local vector_type = ffi.typeof("float[3]")
local vertex_type = ffi.typeof("$[2]", vector_type)
local vertices_type = ffi.typeof("$[?]", vertex_type)

local indices_type = ffi.typeof("uint32_t[?]")

local uniforms_type = ffi.typeof("struct { $ projection; $ model; $ view; }", matrix_type, matrix_type, matrix_type)

local create_vertices = function(tbl) return vertices_type(#tbl, tbl) end
local create_indices = function(tbl) return indices_type(#tbl, tbl) end
local create_matrix = function(x,y,z) return matrix_type(1,0,0,0, 0,1,0,0, 0,0,1,0, x or 0, y or 0, z or 0,1) end
local create_uniforms = uniforms_type

local DEMO = {}

DEMO.InstanceExtensions = {
	"VK_EXT_debug_report",
}

DEMO.InstanceValidationLayers = {}

DEMO.DeviceExtensions = {}

DEMO.DeviceValidationLayers = {
	--"VK_LAYER_LUNARG_threading",
	--"VK_LAYER_LUNARG_mem_tracker",
	--"VK_LAYER_LUNARG_object_tracker",
	"VK_LAYER_LUNARG_draw_state",
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

if GLSL then
	table.insert(DEMO.DeviceExtensions, "VK_NV_glsl_shader")
end

for k,v in pairs(DEMO.DeviceValidationLayers) do table.insert(DEMO.InstanceValidationLayers, v) end

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
		pApplicationInfo = vk.s.ApplicationInfo{
			pApplicationName = "goluwa",
			applicationVersion = 0,
			pEngineName = "goluwa",
			engineVersion = 0,
			apiVersion = vk.macros.MAKE_VERSION(1, 0, 2),
		},

		enabledLayerCount = #self.InstanceValidationLayers,
		ppEnabledLayerNames = vk.util.StringList(self.InstanceValidationLayers),

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
	for _, physical_device in ipairs(self.Instance:GetPhysicalDevices()) do			-- get a list of vulkan capable hardware
		for i, info in ipairs(physical_device:GetQueueFamilyProperties()) do			-- get a list of queues the hardware supports
			if bit.band(info.queueFlags, vk.e.QUEUE_GRAPHICS_BIT) ~= 0 then			-- if this queue supports graphics use it

				self.PhysicalDevice = physical_device
				self.DeviceMemoryProperties = self.PhysicalDevice:GetMemoryProperties()
				self.DeviceQueueIndex = i - 1

				self.Device = physical_device:CreateDevice({
					enabledLayerCount = #self.DeviceValidationLayers,
					ppEnabledLayerNames = vk.util.StringList(self.DeviceValidationLayers),

					enabledExtensionCount = #self.DeviceExtensions,
					ppEnabledExtensionNames = vk.util.StringList(self.DeviceExtensions),

					queueCreateInfoCount = 1,
					pQueueCreateInfos = vk.s.DeviceQueueCreateInfoArray{
						{
							queueFamilyIndex = self.DeviceQueueIndex,
							queueCount = 1,
							pQueuePriorities = ffi.new("float[1]", 0), -- todo: public ffi use is bad!
							pEnabledFeatures = nil,
						}
					}
				})

				self.DeviceQueue = self.Device:GetQueue(self.DeviceQueueIndex, 0)
				self.DeviceCommandPool = self.Device:CreateCommandPool({queueFamilyIndex = self.DeviceQueueIndex})

				return
			end
		end
	end
end

function DEMO:PrepareSurface()
	self.Surface = glfw.CreateWindowSurface(self.Instance, self.Window, nil)
	self.SurfaceFormats = self.PhysicalDevice:GetSurfaceFormats(self.Surface)
	self.SurfaceCapabilities = self.PhysicalDevice:GetSurfaceCapabilities(self.Surface)

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

function DEMO:PrepareData()
	do
		self.DepthBuffer = self:CreateImage(self.Width, self.Height, vk.e.FORMAT_D16_UNORM, vk.e.IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT, vk.e.IMAGE_TILING_OPTIMAL, 0)

		self:SetImageLayout(self.DepthBuffer.image, vk.e.IMAGE_ASPECT_DEPTH_BIT, vk.e.IMAGE_LAYOUT_UNDEFINED, vk.e.IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL)

		self.DepthBuffer.view = self.Device:CreateImageView({
			viewType = vk.e.IMAGE_VIEW_TYPE_2D,
			image = self.DepthBuffer.image,
			format = self.DepthBuffer.format,
			flags = 0,
			subresourceRange = {vk.e.IMAGE_ASPECT_DEPTH_BIT, 0, 1, 0, 1},
		})
	end

	self.Texture = self:CreateTexture(vk.e.FORMAT_B8G8R8A8_UNORM)

	self.Vertices = self:CreateBuffer(
		vk.e.BUFFER_USAGE_VERTEX_BUFFER_BIT,
		create_vertices
		{
			{ { 1.0,  1.0, 0.0 }, { 1.0, 0.0, 0.0 } },
			{ {-1.0,  1.0, 0.0 }, { 0.0, 1.0, 0.0 } },
			{ { 0.0, -1.0, 0.0 }, { 0.0, 0.0, 1.0 } },
		}
	)

	self.Indices = self:CreateBuffer(
		vk.e.BUFFER_USAGE_INDEX_BUFFER_BIT,
		create_indices
		{
			0,
			1,
			2,
		}
	)
	self.Indices.count = self.Indices.size / ffi.sizeof("uint32_t")

	self.Uniforms = self:CreateBuffer(
		vk.e.BUFFER_USAGE_UNIFORM_BUFFER_BIT,
		create_uniforms
		{
			projection = create_matrix():Perspective(math.rad(60), 0.1, 256, self.Width / self.Height),
			view = create_matrix(0,0,-30),
			model = create_matrix(),
		}
	)
end

function DEMO:PrepareDescriptors()
	self.DescriptorLayout = self.Device:CreateDescriptorSetLayout({
		bindingCount = 2,
		pBindings = vk.s.DescriptorSetLayoutBindingArray{
			{
				binding = 0,
				descriptorType = vk.e.DESCRIPTOR_TYPE_UNIFORM_BUFFER,
				descriptorCount = 1,
				stageFlags = vk.e.SHADER_STAGE_VERTEX_BIT,
				pImmutableSamplers = nil,
			},
			{
				binding = 1,
				descriptorType = vk.e.DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
				descriptorCount = 1,
				stageFlags = vk.e.SHADER_STAGE_FRAGMENT_BIT,
				pImmutableSamplers = nil,
			},
		},
	})

	self.PipelineLayout = self.Device:CreatePipelineLayout({
		setLayoutCount = 1,
		pSetLayouts = vk.s.DescriptorSetLayoutArray{
			self.DescriptorLayout,
		},
	})

	self.DescriptorSet = self.Device:AllocateDescriptorSets({
		descriptorPool = self.Device:CreateDescriptorPool({
			maxSets = 2,
			poolSizeCount = 2,
			pPoolSizes = vk.s.DescriptorPoolSizeArray{
				{
					type = vk.e.DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
					descriptorCount = 1
				},
				{
					type = vk.e.DESCRIPTOR_TYPE_UNIFORM_BUFFER,
					descriptorCount = 1
				},
			}
		}),

		descriptorSetCount = 1,
		pSetLayouts = vk.s.DescriptorSetLayoutArray{
			self.DescriptorLayout
		},
	})

	self.Device:UpdateDescriptorSets(
		2,
		vk.s.WriteDescriptorSetArray{
			{
				dstSet = self.DescriptorSet,
				descriptorType = vk.e.DESCRIPTOR_TYPE_UNIFORM_BUFFER,
				dstBinding = 0,

				descriptorCount = 1,
				pBufferInfo = vk.s.DescriptorBufferInfoArray{
					{
						buffer = self.Uniforms.buffer,
						offset = 0,
						range = self.Uniforms.size,
					}
				}
			},
			{
				dstSet = self.DescriptorSet,
				descriptorType = vk.e.DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
				dstBinding = 1,

				descriptorCount = 1,
				pImageInfo = vk.s.DescriptorImageInfoArray{
					{
						sampler = self.Texture.sampler,
						imageView = self.Texture.view,
						imageLayout = vk.e.IMAGE_LAYOUT_GENERAL,
					}
				}
			},
		},
		0,
		nil
	)
end

function DEMO:PrepareRenderPass()
	self.RenderPass = self.Device:CreateRenderPass({
		attachmentCount = 2,
		pAttachments = vk.s.AttachmentDescriptionArray{
			{
				format = self.Texture.format,
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
			},
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
					},
				},

				pResolveAttachments = nil,
				pDepthStencilAttachment = vk.s.AttachmentReference{
					attachment = 0,
					layout = vk.e.IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
				},

				preserveAttachmentCount = 0,
				pPreserveAttachments = nil,
			},
		},

		dependencyCount = 0,
		pDependencies = nil,
	})
end

function DEMO:PreparePipeline()
	local vertex_code, vertex_size
	local fragment_code, fragment_size

	if GLSL then
		vertex_code = [[
			#version 400
			#extension GL_ARB_separate_shader_objects : enable
			#extension GL_ARB_shading_language_420pack : enable
			layout (location = 0) in vec4 pos;
			layout (location = 1) in vec2 attr;
			layout (location = 0) out vec2 texcoord;
			void main() {
			   texcoord = attr;
			   gl_Position = pos;
			}
		]]

		vertex_size = #vertex_code
		vertex_code = ffi.cast("uint32_t *", vertex_code)

		fragment_code = [[
			#version 400
			#extension GL_ARB_separate_shader_objects : enable
			#extension GL_ARB_shading_language_420pack : enable
			layout (binding = 0) uniform sampler2D tex;
			layout (location = 0) in vec2 texcoord;
			layout (location = 0) out vec4 uFragColor;
			void main() {
			   uFragColor = texture(tex, texcoord);
			}
		]]
		fragment_size = #fragment_code
		fragment_code = ffi.cast("uint32_t *", fragment_code)
	else
		vertex_code = {
			0x03, 0x02, 0x23, 0x07, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x08, 0x00,
			0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x00, 0x02, 0x00,
			0x01, 0x00, 0x00, 0x00, 0x0b, 0x00, 0x06, 0x00, 0x01, 0x00, 0x00, 0x00,
			0x47, 0x4c, 0x53, 0x4c, 0x2e, 0x73, 0x74, 0x64, 0x2e, 0x34, 0x35, 0x30,
			0x00, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x01, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x04, 0x00, 0x00, 0x00, 0x6d, 0x61, 0x69, 0x6e, 0x00, 0x00, 0x00, 0x00,
			0x09, 0x00, 0x00, 0x00, 0x0b, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00,
			0x17, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x1d, 0x00, 0x00, 0x00,
			0x03, 0x00, 0x03, 0x00, 0x02, 0x00, 0x00, 0x00, 0x90, 0x01, 0x00, 0x00,
			0x04, 0x00, 0x09, 0x00, 0x47, 0x4c, 0x5f, 0x41, 0x52, 0x42, 0x5f, 0x73,
			0x65, 0x70, 0x61, 0x72, 0x61, 0x74, 0x65, 0x5f, 0x73, 0x68, 0x61, 0x64,
			0x65, 0x72, 0x5f, 0x6f, 0x62, 0x6a, 0x65, 0x63, 0x74, 0x73, 0x00, 0x00,
			0x04, 0x00, 0x09, 0x00, 0x47, 0x4c, 0x5f, 0x41, 0x52, 0x42, 0x5f, 0x73,
			0x68, 0x61, 0x64, 0x69, 0x6e, 0x67, 0x5f, 0x6c, 0x61, 0x6e, 0x67, 0x75,
			0x61, 0x67, 0x65, 0x5f, 0x34, 0x32, 0x30, 0x70, 0x61, 0x63, 0x6b, 0x00,
			0x05, 0x00, 0x04, 0x00, 0x04, 0x00, 0x00, 0x00, 0x6d, 0x61, 0x69, 0x6e,
			0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x05, 0x00, 0x09, 0x00, 0x00, 0x00,
			0x74, 0x65, 0x78, 0x63, 0x6f, 0x6f, 0x72, 0x64, 0x00, 0x00, 0x00, 0x00,
			0x05, 0x00, 0x04, 0x00, 0x0b, 0x00, 0x00, 0x00, 0x61, 0x74, 0x74, 0x72,
			0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x06, 0x00, 0x11, 0x00, 0x00, 0x00,
			0x67, 0x6c, 0x5f, 0x50, 0x65, 0x72, 0x56, 0x65, 0x72, 0x74, 0x65, 0x78,
			0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x06, 0x00, 0x11, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x67, 0x6c, 0x5f, 0x50, 0x6f, 0x73, 0x69, 0x74,
			0x69, 0x6f, 0x6e, 0x00, 0x06, 0x00, 0x07, 0x00, 0x11, 0x00, 0x00, 0x00,
			0x01, 0x00, 0x00, 0x00, 0x67, 0x6c, 0x5f, 0x50, 0x6f, 0x69, 0x6e, 0x74,
			0x53, 0x69, 0x7a, 0x65, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x07, 0x00,
			0x11, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x67, 0x6c, 0x5f, 0x43,
			0x6c, 0x69, 0x70, 0x44, 0x69, 0x73, 0x74, 0x61, 0x6e, 0x63, 0x65, 0x00,
			0x05, 0x00, 0x03, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x05, 0x00, 0x03, 0x00, 0x17, 0x00, 0x00, 0x00, 0x70, 0x6f, 0x73, 0x00,
			0x05, 0x00, 0x05, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x67, 0x6c, 0x5f, 0x56,
			0x65, 0x72, 0x74, 0x65, 0x78, 0x49, 0x44, 0x00, 0x05, 0x00, 0x06, 0x00,
			0x1d, 0x00, 0x00, 0x00, 0x67, 0x6c, 0x5f, 0x49, 0x6e, 0x73, 0x74, 0x61,
			0x6e, 0x63, 0x65, 0x49, 0x44, 0x00, 0x00, 0x00, 0x47, 0x00, 0x04, 0x00,
			0x09, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x47, 0x00, 0x04, 0x00, 0x0b, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00,
			0x01, 0x00, 0x00, 0x00, 0x48, 0x00, 0x05, 0x00, 0x11, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x48, 0x00, 0x05, 0x00, 0x11, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
			0x0b, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x48, 0x00, 0x05, 0x00,
			0x11, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x0b, 0x00, 0x00, 0x00,
			0x03, 0x00, 0x00, 0x00, 0x47, 0x00, 0x03, 0x00, 0x11, 0x00, 0x00, 0x00,
			0x02, 0x00, 0x00, 0x00, 0x47, 0x00, 0x04, 0x00, 0x17, 0x00, 0x00, 0x00,
			0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x47, 0x00, 0x04, 0x00,
			0x1c, 0x00, 0x00, 0x00, 0x0b, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00,
			0x47, 0x00, 0x04, 0x00, 0x1d, 0x00, 0x00, 0x00, 0x0b, 0x00, 0x00, 0x00,
			0x06, 0x00, 0x00, 0x00, 0x13, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00, 0x00,
			0x21, 0x00, 0x03, 0x00, 0x03, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00,
			0x16, 0x00, 0x03, 0x00, 0x06, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00,
			0x17, 0x00, 0x04, 0x00, 0x07, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00,
			0x02, 0x00, 0x00, 0x00, 0x20, 0x00, 0x04, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x03, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x3b, 0x00, 0x04, 0x00,
			0x08, 0x00, 0x00, 0x00, 0x09, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00,
			0x20, 0x00, 0x04, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
			0x07, 0x00, 0x00, 0x00, 0x3b, 0x00, 0x04, 0x00, 0x0a, 0x00, 0x00, 0x00,
			0x0b, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x17, 0x00, 0x04, 0x00,
			0x0d, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00,
			0x15, 0x00, 0x04, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x2b, 0x00, 0x04, 0x00, 0x0e, 0x00, 0x00, 0x00,
			0x0f, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x04, 0x00,
			0x10, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00,
			0x1e, 0x00, 0x05, 0x00, 0x11, 0x00, 0x00, 0x00, 0x0d, 0x00, 0x00, 0x00,
			0x06, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x20, 0x00, 0x04, 0x00,
			0x12, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00,
			0x3b, 0x00, 0x04, 0x00, 0x12, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00,
			0x03, 0x00, 0x00, 0x00, 0x15, 0x00, 0x04, 0x00, 0x14, 0x00, 0x00, 0x00,
			0x20, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x2b, 0x00, 0x04, 0x00,
			0x14, 0x00, 0x00, 0x00, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x20, 0x00, 0x04, 0x00, 0x16, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
			0x0d, 0x00, 0x00, 0x00, 0x3b, 0x00, 0x04, 0x00, 0x16, 0x00, 0x00, 0x00,
			0x17, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x20, 0x00, 0x04, 0x00,
			0x19, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x0d, 0x00, 0x00, 0x00,
			0x20, 0x00, 0x04, 0x00, 0x1b, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
			0x14, 0x00, 0x00, 0x00, 0x3b, 0x00, 0x04, 0x00, 0x1b, 0x00, 0x00, 0x00,
			0x1c, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x3b, 0x00, 0x04, 0x00,
			0x1b, 0x00, 0x00, 0x00, 0x1d, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
			0x36, 0x00, 0x05, 0x00, 0x02, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0xf8, 0x00, 0x02, 0x00,
			0x05, 0x00, 0x00, 0x00, 0x3d, 0x00, 0x04, 0x00, 0x07, 0x00, 0x00, 0x00,
			0x0c, 0x00, 0x00, 0x00, 0x0b, 0x00, 0x00, 0x00, 0x3e, 0x00, 0x03, 0x00,
			0x09, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x3d, 0x00, 0x04, 0x00,
			0x0d, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x17, 0x00, 0x00, 0x00,
			0x41, 0x00, 0x05, 0x00, 0x19, 0x00, 0x00, 0x00, 0x1a, 0x00, 0x00, 0x00,
			0x13, 0x00, 0x00, 0x00, 0x15, 0x00, 0x00, 0x00, 0x3e, 0x00, 0x03, 0x00,
			0x1a, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0xfd, 0x00, 0x01, 0x00,
			0x38, 0x00, 0x01, 0x00
		}
		vertex_size = #vertex_code
		vertex_code = ffi.cast("const unsigned int *", ffi.new("uint8_t[?]", vertex_size, vertex_code))

		fragment_code = {
			0x03, 0x02, 0x23, 0x07, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x08, 0x00,
			0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x00, 0x02, 0x00,
			0x01, 0x00, 0x00, 0x00, 0x0b, 0x00, 0x06, 0x00, 0x01, 0x00, 0x00, 0x00,
			0x47, 0x4c, 0x53, 0x4c, 0x2e, 0x73, 0x74, 0x64, 0x2e, 0x34, 0x35, 0x30,
			0x00, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x01, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x07, 0x00, 0x04, 0x00, 0x00, 0x00,
			0x04, 0x00, 0x00, 0x00, 0x6d, 0x61, 0x69, 0x6e, 0x00, 0x00, 0x00, 0x00,
			0x09, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00, 0x10, 0x00, 0x03, 0x00,
			0x04, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x03, 0x00, 0x03, 0x00,
			0x02, 0x00, 0x00, 0x00, 0x90, 0x01, 0x00, 0x00, 0x04, 0x00, 0x09, 0x00,
			0x47, 0x4c, 0x5f, 0x41, 0x52, 0x42, 0x5f, 0x73, 0x65, 0x70, 0x61, 0x72,
			0x61, 0x74, 0x65, 0x5f, 0x73, 0x68, 0x61, 0x64, 0x65, 0x72, 0x5f, 0x6f,
			0x62, 0x6a, 0x65, 0x63, 0x74, 0x73, 0x00, 0x00, 0x04, 0x00, 0x09, 0x00,
			0x47, 0x4c, 0x5f, 0x41, 0x52, 0x42, 0x5f, 0x73, 0x68, 0x61, 0x64, 0x69,
			0x6e, 0x67, 0x5f, 0x6c, 0x61, 0x6e, 0x67, 0x75, 0x61, 0x67, 0x65, 0x5f,
			0x34, 0x32, 0x30, 0x70, 0x61, 0x63, 0x6b, 0x00, 0x05, 0x00, 0x04, 0x00,
			0x04, 0x00, 0x00, 0x00, 0x6d, 0x61, 0x69, 0x6e, 0x00, 0x00, 0x00, 0x00,
			0x05, 0x00, 0x05, 0x00, 0x09, 0x00, 0x00, 0x00, 0x75, 0x46, 0x72, 0x61,
			0x67, 0x43, 0x6f, 0x6c, 0x6f, 0x72, 0x00, 0x00, 0x05, 0x00, 0x03, 0x00,
			0x0d, 0x00, 0x00, 0x00, 0x74, 0x65, 0x78, 0x00, 0x05, 0x00, 0x05, 0x00,
			0x11, 0x00, 0x00, 0x00, 0x74, 0x65, 0x78, 0x63, 0x6f, 0x6f, 0x72, 0x64,
			0x00, 0x00, 0x00, 0x00, 0x47, 0x00, 0x04, 0x00, 0x09, 0x00, 0x00, 0x00,
			0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x47, 0x00, 0x04, 0x00,
			0x0d, 0x00, 0x00, 0x00, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x47, 0x00, 0x04, 0x00, 0x0d, 0x00, 0x00, 0x00, 0x21, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x47, 0x00, 0x04, 0x00, 0x11, 0x00, 0x00, 0x00,
			0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x00, 0x02, 0x00,
			0x02, 0x00, 0x00, 0x00, 0x21, 0x00, 0x03, 0x00, 0x03, 0x00, 0x00, 0x00,
			0x02, 0x00, 0x00, 0x00, 0x16, 0x00, 0x03, 0x00, 0x06, 0x00, 0x00, 0x00,
			0x20, 0x00, 0x00, 0x00, 0x17, 0x00, 0x04, 0x00, 0x07, 0x00, 0x00, 0x00,
			0x06, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x20, 0x00, 0x04, 0x00,
			0x08, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00,
			0x3b, 0x00, 0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x09, 0x00, 0x00, 0x00,
			0x03, 0x00, 0x00, 0x00, 0x19, 0x00, 0x09, 0x00, 0x0a, 0x00, 0x00, 0x00,
			0x06, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x1b, 0x00, 0x03, 0x00, 0x0b, 0x00, 0x00, 0x00,
			0x0a, 0x00, 0x00, 0x00, 0x20, 0x00, 0x04, 0x00, 0x0c, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x0b, 0x00, 0x00, 0x00, 0x3b, 0x00, 0x04, 0x00,
			0x0c, 0x00, 0x00, 0x00, 0x0d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x17, 0x00, 0x04, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00,
			0x02, 0x00, 0x00, 0x00, 0x20, 0x00, 0x04, 0x00, 0x10, 0x00, 0x00, 0x00,
			0x01, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x3b, 0x00, 0x04, 0x00,
			0x10, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
			0x36, 0x00, 0x05, 0x00, 0x02, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0xf8, 0x00, 0x02, 0x00,
			0x05, 0x00, 0x00, 0x00, 0x3d, 0x00, 0x04, 0x00, 0x0b, 0x00, 0x00, 0x00,
			0x0e, 0x00, 0x00, 0x00, 0x0d, 0x00, 0x00, 0x00, 0x3d, 0x00, 0x04, 0x00,
			0x0f, 0x00, 0x00, 0x00, 0x12, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00,
			0x57, 0x00, 0x05, 0x00, 0x07, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00,
			0x0e, 0x00, 0x00, 0x00, 0x12, 0x00, 0x00, 0x00, 0x3e, 0x00, 0x03, 0x00,
			0x09, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00, 0xfd, 0x00, 0x01, 0x00,
			0x38, 0x00, 0x01, 0x00
		}
		fragment_size = #fragment_code
		fragment_code = ffi.cast("const unsigned int *", ffi.new("uint8_t[?]", fragment_size, fragment_code))
	end

	local cache = self.Device:CreatePipelineCache({})

	self.Pipeline = self.Device:CreateGraphicsPipelines(cache, 1, {
		{
			layout = self.PipelineLayout,
			pVertexInputState = vk.s.PipelineVertexInputStateCreateInfo({
				vertexBindingDescriptionCount = 1,
				pVertexBindingDescriptions = vk.s.VertexInputBindingDescriptionArray{
					{
						binding = 0,
						stride = ffi.sizeof(self.Vertices.data[0]),
						inputRate = vk.e.VERTEX_INPUT_RATE_VERTEX
					}
				},
				vertexAttributeDescriptionCount = 2,
				pVertexAttributeDescriptions = vk.s.VertexInputAttributeDescriptionArray{
					-- location 0: position
					{
						binding = 0,
						location = 0,
						format = vk.e.FORMAT_R32G32B32_SFLOAT,
						offset = 0,
					},
					-- location 1: color
					{
						binding = 0,
						location = 1,
						format = vk.e.FORMAT_R32G32B32_SFLOAT,
						offset = ffi.sizeof(self.Vertices.data[0][0]),
					}
				},
			}),
			renderPass = self.RenderPass,

			stageCount = 2,
			pStages = vk.s.PipelineShaderStageCreateInfoArray{
				{
					stage = vk.e.SHADER_STAGE_VERTEX_BIT,
					module = self.Device:CreateShaderModule({
						codeSize = vertex_size,
						pCode = vertex_code,
					}),
					pName = "main",
				},
				{
					stage = vk.e.SHADER_STAGE_FRAGMENT_BIT,
					module = self.Device:CreateShaderModule({
						codeSize = fragment_size,
						pCode = fragment_code,
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
				depthClampEnable = vk.e.FALSE,
				rasterizerDiscardEnable = vk.e.FALSE,
				depthBiasEnable = vk.e.FALSE,
			},

			pColorBlendStage = vk.s.PipelineColorBlendStateCreateInfo{
				attachmentCount = 1,
				pAttachments = vk.s.PipelineColorBlendAttachmentStateArray{
					{
						colorWriteMask = 0xf,
						blendEnable = vk.e.FALSE,
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
				depthTestEnable = vk.e.TRUE,
				depthWriteEnable = vk.e.TRUE,
				depthCompareOp = vk.e.COMPARE_OP_LESS_OR_EQUAL,
				depthBoundsTestEnable = vk.e.FALSE,
				stencilTestEnable = vk.e.FALSE,
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


function DEMO:Initialize()
	self:PrepareWindow() -- create a window
	self:PrepareInstance() -- create a vulkan instance
	self:PrepareDevice() -- find and physical_device and use it
	self:PrepareSurface() -- create the window surface to render on

	self:PrepareData() -- create depth buffers, framebuffers, textures, vertices, indices, etc
	self:PrepareDescriptors() -- setup the layout of the data like how to bind uniforms and textures

	self:PrepareRenderPass()

	do
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

	self:PreparePipeline()

	self:FlushSetupCMD()

	self.DrawCMDs = {}

	for i, surface_buffer in ipairs(self.SurfaceBuffers) do
		local draw_cmd = self.Device:AllocateCommandBuffers({
			commandPool = self.DeviceCommandPool,
			level = vk.e.COMMAND_BUFFER_LEVEL_PRIMARY,
			commandBufferCount = 1,
		})

		draw_cmd:Begin(vk.s.CommandBufferBeginInfo({
			flags = 0,
			pInheritanceInfo = vk.s.CommandBufferInheritanceInfo({
				renderPass = nil,
				subpass = 0,
				framebuffer = nil,
				offclusionQueryEnable = vk.e.FALSE,
				queryFlags = 0,
				pipelineStatistics = 0,
			 })
		}))

		draw_cmd:BeginRenderPass(vk.s.RenderPassBeginInfo{
			renderPass = self.RenderPass,
			framebuffer = self.Framebuffers[i],
			renderArea = {offset = {0, 0}, extent = {self.Width, self.Height}},
			clearValueCount = 2,
			pClearValues = vk.s.ClearValueArray{
				{color = {float32 = {0.2, 0.2, 0.2, 0.2}}},
				{depthStencil = {1, 0}}
			},
		}, vk.e.SUBPASS_CONTENTS_INLINE)

		draw_cmd:SetViewport(0,1, vk.s.Viewport{0,0,self.Height,self.Width, 0,1,})
		draw_cmd:SetScissor(0,1, vk.s.Rect2D{offset = {0, 0}, extent = {self.Height, self.Width}})

		draw_cmd:BindPipeline(vk.e.PIPELINE_BIND_POINT_GRAPHICS, self.Pipeline)

		draw_cmd:BindDescriptorSets(vk.e.PIPELINE_BIND_POINT_GRAPHICS, self.PipelineLayout, 0, 1, vk.s.DescriptorSetArray{self.DescriptorSet}, 0, nil)
		draw_cmd:BindVertexBuffers(0, 1, vk.s.BufferArray{self.Vertices.buffer}, ffi.new("unsigned long[1]", 0))
		draw_cmd:BindIndexBuffer(self.Indices.buffer, 0, vk.e.INDEX_TYPE_UINT32)

		draw_cmd:DrawIndexed(self.Indices.count, 1, 0, 0, 0)

		draw_cmd:EndRenderPass()

		draw_cmd:PipelineBarrier(
			vk.e.PIPELINE_STAGE_ALL_COMMANDS_BIT,
			vk.e.PIPELINE_STAGE_TOP_OF_PIPE_BIT,
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
					srcQueueFamilyIndex = vk.e.QUEUE_FAMILY_IGNORED,
					dstQueueFamilyIndex = vk.e.QUEUE_FAMILY_IGNORED,
					subresourceRange = {vk.e.IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1},
					image = surface_buffer.image,
				}
			}
		)

		draw_cmd:End()

		self.DrawCMDs[i] = draw_cmd
	end

	self.PostPresentCMD = self.Device:AllocateCommandBuffers({
		commandPool = self.DeviceCommandPool,
		level = vk.e.COMMAND_BUFFER_LEVEL_PRIMARY,
		commandBufferCount = #self.SurfaceBuffers,
	})

	--while glfw.WindowShouldClose(self.Window) == 0 do
	--for i = 1, 5 do
	do
		glfw.PollEvents()

		self.DeviceQueue:WaitIdle()

		local semaphore = self.Device:CreateSemaphore({
			flags = 0,
		})

		local index, status = self.Device:AcquireNextImage(self.SwapChain, vk.e.WHOLE_SIZE, semaphore, nil)
		index = index + 1

		self:SetImageLayout(self.SurfaceBuffers[index].image, vk.e.IMAGE_ASPECT_COLOR_BIT, vk.e.IMAGE_LAYOUT_PRESENT_SRC_KHR, vk.e.IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL)


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
				self.DrawCMDs[i]
			},
		}), nil)

		vk.QueuePresentKHR(self.DeviceQueue, vk.s.PresentInfoKHR{
			pImageIndices = ffi.new("unsigned int [1]", index - 1),

			swapchainCount = 1,
			pSwapchains = vk.s.SwapchainKHRArray{
				self.SwapChain
			},
		})

		self.Device:DestroySemaphore(semaphore, nil)



		self.PostPresentCMD:Begin({})

		self.PostPresentCMD:PipelineBarrier(
			vk.e.PIPELINE_STAGE_ALL_COMMANDS_BIT,
			vk.e.PIPELINE_STAGE_TOP_OF_PIPE_BIT,
			0,

			0,
			nil,

			0,
			nil,

			1,
			vk.s.ImageMemoryBarrierArray{
				{
					srcAccessMask = 0,
					dstAccessMask = vk.e.ACCESS_COLOR_ATTACHMENT_WRITE_BIT,
					oldLayout = vk.e.IMAGE_LAYOUT_PRESENT_SRC_KHR,
					newLayout = vk.e.IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
					srcQueueFamilyIndex = vk.e.QUEUE_FAMILY_IGNORED,
					dstQueueFamilyIndex = vk.e.QUEUE_FAMILY_IGNORED,
					subresourceRange = {vk.e.IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1},
					image = self.SurfaceBuffers[index].image,
				}
			}
		)

		self.PostPresentCMD:End()

		self.DeviceQueue:Submit(1, vk.s.SubmitInfo({
			commandBufferCount = 1,
			pCommandBuffers = vk.s.CommandBufferArray{
				self.PostPresentCMD,
			},
		}), nil)

		self.DeviceQueue:WaitIdle()
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

--	table_print(self)
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
				offclusionQueryEnable = vk.e.FALSE,
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

	return {
		image = image,
		memory = memory,
		size = memory_requirements.size,
		format = format,
	}
end

do
	local function prepare(self, tex, tiling, usage, required_props)
		tex.width = 2
		tex.height = 2

		local info = self:CreateImage(tex.width, tex.height, vk.e.FORMAT_B8G8R8A8_UNORM, usage, tiling, required_props)

		tex.memory = info.memory
		tex.image = info.image

		if bit.band(required_props, vk.e.MEMORY_PROPERTY_HOST_VISIBLE_BIT) ~= 0 then
			tex.imageLayout = vk.e.IMAGE_ASPECT_COLOR_BIT

			self.Device:MapMemory(info.memory, 0, info.size, 0, "uint32_t", function(data)
				for y = 0, tex.height - 1 do
					for x = 0, tex.width - 1 do
						data[x * y] = math.random(0xFFFFFFFF)
					end
				end
			end)
		end

		self:SetImageLayout(image, vk.e.IMAGE_ASPECT_COLOR_BIT, vk.e.IMAGE_LAYOUT_UNDEFINED, vk.e.IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL)
	end

	function DEMO:CreateTexture(format)
		local texture = {}

		prepare(self, texture, vk.e.IMAGE_TILING_OPTIMAL, bit.bor(vk.e.IMAGE_USAGE_TRANSFER_DST_BIT, vk.e.IMAGE_USAGE_SAMPLED_BIT), vk.e.MEMORY_PROPERTY_DEVICE_LOCAL_BIT)
		self:SetImageLayout(texture.image, vk.e.IMAGE_ASPECT_COLOR_BIT, texture.imageLayout, vk.e.IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL)

		do
			local staging = {}
			prepare(self, staging, vk.e.IMAGE_TILING_LINEAR, vk.e.IMAGE_USAGE_TRANSFER_SRC_BIT, vk.e.MEMORY_PROPERTY_HOST_VISIBLE_BIT)
			self:SetImageLayout(staging.image, vk.e.IMAGE_ASPECT_COLOR_BIT, staging.imageLayout, vk.e.IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL)

			self.SetupCMD:CopyImage(
				staging.image,
				vk.e.IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
				texture.image,
				vk.e.IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,

				1,
				vk.s.ImageCopyArray{
					{
						srcSubresource = {vk.e.IMAGE_ASPECT_COLOR_BIT, 0, 0, 1},
						srcOffset = {0, 0, 0},
						dstSubresource = {vk.e.IMAGE_ASPECT_COLOR_BIT, 0, 0, 1},
						dstOffset = {0, 0, 0},
						extent = {
							staging.width,
							staging.height,
							1
						},
					}
				}
			)

			self.Device:DestroyImage(staging.image, nil)
			self.Device:FreeMemory(staging.memory, nil)
		end

		self:SetImageLayout(texture.image, vk.e.IMAGE_ASPECT_COLOR_BIT, vk.e.IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, texture.imageLayout)

		texture.sampler = self.Device:CreateSampler({
			magFilter = vk.e.FILTER_NEAREST,
			minFilter = vk.e.FILTER_NEAREST,
			mipmapMode = vk.e.SAMPLER_MIPMAP_MODE_NEAREST,
			addressModeU = vk.e.SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE,
			addressModeV = vk.e.SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE,
			addressModeW = vk.e.SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE,
			ipLodBias = 0.0,
			anisotropyEnable = vk.e.FALSE,
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
			format = format,
			flags = 0,
			components = {
				r = vk.e.COMPONENT_SWIZZLE_R,
				g = vk.e.COMPONENT_SWIZZLE_G,
				b = vk.e.COMPONENT_SWIZZLE_B,
				a = vk.e.COMPONENT_SWIZZLE_A
			},
			subresourceRange = {vk.e.IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1},
		})

		texture.format = format

		return texture
	end
end

function DEMO:CreateBuffer(usage, data)
	local size = ffi.sizeof(data)
	local buffer = self.Device:CreateBuffer({
		size = size,
		usage = usage,
	})

	local memory = self.Device:AllocateMemory({
		allocationSize = size,
		memoryTypeIndex = self:GetMemoryTypeFromProperties(self.Device:GetBufferMemoryRequirements(buffer).memoryTypeBits, vk.e.MEMORY_PROPERTY_HOST_VISIBLE_BIT),
	})

	self.Device:MapMemory(memory, 0, size, 0, "float", function(cdata)
		ffi.copy(cdata, data, size)
	end)

	self.Device:BindBufferMemory(buffer, memory, 0)

	return {
		buffer = buffer,
		memory = memory,
		data = data,
		size = size,
	}
end

DEMO:Initialize()