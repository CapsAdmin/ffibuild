collectgarbage("stop")

local ffi = require("ffi")

package.path = package.path .. ";./../examples/?.lua"

local vk = require("vulkan/libvulkan")

_G.FFI_LIB = "../examples/glfw/glfw/src/libglfw.so"
local glfw = require("glfw/libglfw")

_G.FFI_LIB = "../examples/freeimage/freeimage/FreeImage/libfreeimage-3.18.0.so"
local freeimage = require("freeimage/libfreeimage")

_G.FFI_LIB = nil

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
	--"VK_LAYER_LUNARG_draw_state",
	"VK_LAYER_LUNARG_param_checker",
	--"VK_LAYER_LUNARG_swapchain",
	--"VK_LAYER_LUNARG_device_limits",
	--"VK_LAYER_LUNARG_image",
	--"VK_LAYER_LUNARG_api_dump",
}

DEMO.DebugFlags = {
	vk.e.DEBUG_REPORT_INFORMATION_BIT_EXT,
	vk.e.DEBUG_REPORT_WARNING_BIT_EXT,
	vk.e.DEBUG_REPORT_PERFORMANCE_WARNING_BIT_EXT,
	vk.e.DEBUG_REPORT_ERROR_BIT_EXT,
	vk.e.DEBUG_REPORT_DEBUG_BIT_EXT,
}

for k,v in pairs(DEMO.DeviceValidationLayers) do
	table.insert(DEMO.InstanceValidationLayers, v)
end

function DEMO:Initialize()

	do -- create glfw window
		glfw.SetErrorCallback(function(_, str) io.write(string.format("GLFW Error: %s\n", ffi.string(str))) end)
		glfw.Init()
		glfw.WindowHint(glfw.e.GLFW_CLIENT_API, glfw.e.GLFW_NO_API)

		for _, ext in ipairs(glfw.GetRequiredInstanceExtensions()) do
			table.insert(self.InstanceExtensions, ext)
		end

		self.Window = glfw.CreateWindow(1024, 768, "vulkan", nil, nil)
	end

	do -- create vulkan instance
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

		instance:LoadProcAddr("vkGetPhysicalDeviceSurfacePresentModesKHR")
		instance:LoadProcAddr("vkGetPhysicalDeviceSurfaceSupportKHR")
		instance:LoadProcAddr("vkCreateSwapchainKHR")
		instance:LoadProcAddr("vkDestroySwapchainKHR")
		instance:LoadProcAddr("vkGetSwapchainImagesKHR")
		instance:LoadProcAddr("vkAcquireNextImageKHR")
		instance:LoadProcAddr("vkQueuePresentKHR")
		instance:LoadProcAddr("vkGetPhysicalDeviceSurfaceCapabilitiesKHR")
		instance:LoadProcAddr("vkGetPhysicalDeviceSurfaceFormatsKHR")

		self.Instance = instance
	end

	do -- find and use a gpu
		for _, physical_device in ipairs(self.Instance:GetPhysicalDevices()) do			-- get a list of vulkan capable hardware
			for i, info in ipairs(physical_device:GetQueueFamilyProperties()) do			-- get a list of queues the hardware supports
				if bit.band(info.queueFlags, vk.e.QUEUE_GRAPHICS_BIT) ~= 0 then			-- if this queue supports graphics use it

					self.PhysicalDevice = physical_device
					self.DeviceMemoryProperties = self.PhysicalDevice:GetMemoryProperties()
					self.DeviceQueueIndex = i - 1

					self.Device = self.PhysicalDevice:CreateDevice({
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

					break
				end
			end
		end
	end

	do -- setup the glfw window buffer
		local surface = glfw.CreateWindowSurface(self.Instance, self.Window, nil)

		local formats = self.PhysicalDevice:GetSurfaceFormats(surface)
		local capabilities = self.PhysicalDevice:GetSurfaceCapabilities(surface)

		local prefered_format = formats[1].format

		if prefered_format == vk.e.FORMAT_UNDEFINED then
			prefered_format = vk.e.FORMAT_B8G8R8A8_UNORM
		end

		self.Width = capabilities.currentExtent.width
		self.Height = capabilities.currentExtent.height

		if self.Width == 0xFFFFFFFF then
			self.Width = 1024
			self.Height = 768
		end

		local present_mode

		for _, mode in ipairs(self.PhysicalDevice:GetSurfacePresentModes(surface)) do
			if mode == vk.e.PRESENT_MODE_FIFO_KHR then
				present_mode = mode
				break
			end
		end

		self.Surface = surface

		self.SwapChain = self.Device:CreateSwapchain({
			surface = surface,
			minImageCount = math.min(capabilities.minImageCount + 1, capabilities.maxImageCount == 0 and math.huge or capabilities.maxImageCount),
			imageFormat = prefered_format,
			imagecolorSpace = formats[1].colorSpace,
			imageExtent = {self.Width, self.Height},
			imageUse = vk.e.IMAGE_USAGE_COLOR_ATTACHMENT_BIT,
			preTransform = bit.band(capabilities.supportedTransforms, vk.e.SURFACE_TRANSFORM_IDENTITY_BIT_KHR) ~= 0 and vk.e.SURFACE_TRANSFORM_IDENTITY_BIT_KHR or capabilities.currentTransform,
			compositeAlpha = vk.e.COMPOSITE_ALPHA_OPAQUE_BIT_KHR,
			imageArrayLayers = 1,
			imageSharingMode = vk.e.SHARING_MODE_EXCLUSIVE,

			queueFamilyIndexCount = 0,
			pQueueFamilyIndices = nil,

			presentMode = present_mode or vk.e.PRESENT_MODE_IMMEDIATE_KHR,
			oldSwapchain = nil,
			clipped = vk.e.TRUE,
		})

		self:CreateSetupCMD()

		do -- depth buffer to use in render pass
			self.DepthBuffer = self:CreateImage(self.Width, self.Height, vk.e.FORMAT_D16_UNORM, vk.e.IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT, vk.e.IMAGE_TILING_OPTIMAL, 0)

			self:SetImageLayout(self.DepthBuffer.image, vk.e.IMAGE_ASPECT_DEPTH_BIT, vk.e.IMAGE_LAYOUT_UNDEFINED, vk.e.IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL)

			self.DepthBuffer.view = self.Device:CreateImageView({
				viewType = vk.e.IMAGE_VIEW_TYPE_2D,
				image = self.DepthBuffer.image,
				format = self.DepthBuffer.format,
				flags = 0,
				subresourceRange = {
					aspectMask = vk.e.IMAGE_ASPECT_DEPTH_BIT,

					levelCount = 1,
					baseMipLevel = 0,

					layerCount = 1,
					baseLayerLevel = 0
				},
			})
		end

		self.RenderPass = self.Device:CreateRenderPass({
			attachmentCount = 2,
			pAttachments = vk.s.AttachmentDescriptionArray{
				{
					format = prefered_format,
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
						attachment = 1,
						layout = vk.e.IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
					},

					preserveAttachmentCount = 0,
					pPreserveAttachments = nil,
				},
			},

			dependencyCount = 0,
			pDependencies = nil,
		})

		self.SwapChainBuffers = {}

		for i, image in ipairs(self.Device:GetSwapchainImages(self.SwapChain)) do
			self:SetImageLayout(image, vk.e.IMAGE_ASPECT_COLOR_BIT, vk.e.IMAGE_LAYOUT_UNDEFINED, vk.e.IMAGE_LAYOUT_PRESENT_SRC_KHR)

			local view = self.Device:CreateImageView({
				viewType = vk.e.IMAGE_VIEW_TYPE_2D,
				image = image,
				format = prefered_format,
				flags = 0,
				components = {
					vk.e.COMPONENT_SWIZZLE_R,
					vk.e.COMPONENT_SWIZZLE_G,
					vk.e.COMPONENT_SWIZZLE_B,
					vk.e.COMPONENT_SWIZZLE_A
				},
				subresourceRange = {
					aspectMask = vk.e.IMAGE_ASPECT_COLOR_BIT,

					levelCount = 1,
					baseMipLevel = 0,

					layerCount = 1,
					baseLayerLevel = 0
				},
			})

			self.SwapChainBuffers[i] = {
				cmd = self.Device:AllocateCommandBuffers({
					commandPool = self.DeviceCommandPool,
					level = vk.e.COMMAND_BUFFER_LEVEL_PRIMARY,
					commandBufferCount = 1,
				}),
				image = image,
				view = view,
				framebuffer = self.Device:CreateFramebuffer({
					renderPass = self.RenderPass,

					attachmentCount = 2,
					pAttachments = vk.s.ImageViewArray{
						view,
						self.DepthBuffer.view
					},

					width = self.Width,
					height = self.Height,
					layers = 1,
				})
			}
		end
	end

	do -- data layout
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
						type = vk.e.DESCRIPTOR_TYPE_UNIFORM_BUFFER,
						descriptorCount = 1
					},
					{
						type = vk.e.DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
						descriptorCount = 1
					},
				}
			}),

			descriptorSetCount = 1,
			pSetLayouts = vk.s.DescriptorSetLayoutArray{
				self.DescriptorLayout
			},
		})
	end

	do
		self.Texture = self:CreateTexture("./volcano.png", vk.e.FORMAT_B8G8R8A8_UNORM)
	end

	local vertices

	do
		local vertex_type = ffi.typeof([[
			struct
			{
				float pos[3];
				float uv[2];
				float color[3];
			}
		]])

		local vertices_type = ffi.typeof("$[?]", vertex_type)

		local create_vertices = function(tbl) return vertices_type(#tbl, tbl) end

		vertices = {
			{ pos = { -1, -1, -1 },  	uv = { 0, 0 } },
			{ pos = { -1, 1, 1 },  		uv = { 1, 1 } },
			{ pos = { -1, -1, 1 }, 		uv = { 1, 0 } },
			{ pos = { -1, 1, 1 },  		uv = { 1, 1 } },
			{ pos = { -1, -1, -1 },  	uv = { 0, 0 } },
			{ pos = { -1, 1, -1 },  	uv = { 0, 1 } },

			{ pos = { -1, -1, -1 },  	uv = { 1, 0 } },
			{ pos = { 1, -1, -1 },  	uv = { 0, 0 } },
			{ pos = { 1, 1, -1 },  		uv = { 0, 1 } },
			{ pos = { -1, -1, -1 },  	uv = { 1, 0 } },
			{ pos = { 1, 1, -1 },  		uv = { 0, 1 } },
			{ pos = { -1, 1, -1 },  	uv = { 1, 1 } },

			{ pos = { -1, -1, -1 },  	uv = { 1, 1 } },
			{ pos = { 1, -1, 1 },  		uv = { 0, 0 } },
			{ pos = { 1, -1, -1 },  	uv = { 1, 0 } },
			{ pos = { -1, -1, -1 },  	uv = { 1, 1 } },
			{ pos = { -1, -1, 1 },  	uv = { 0, 1 } },
			{ pos = { 1, -1, 1 },  		uv = { 0, 0 } },

			{ pos = { -1, 1, -1 },  	uv = { 1, 1 } },
			{ pos = { 1, 1, 1 },  		uv = { 0, 0 } },
			{ pos = { -1, 1, 1 },  		uv = { 0, 1 } },
			{ pos = { -1, 1, -1 }, 	 	uv = { 1, 1 } },
			{ pos = { 1, 1, -1 },  		uv = { 1, 0 } },
			{ pos = { 1, 1, 1 },  		uv = { 0, 0 } },

			{ pos = { 1, 1, -1 },  		uv = { 1, 1 } },
			{ pos = { 1, -1, 1 }, 	 	uv = { 0, 0 } },
			{ pos = { 1, 1, 1 },  		uv = { 0, 1 } },
			{ pos = { 1, -1, 1 },  		uv = { 0, 0 } },
			{ pos = { 1, 1, -1 }, 	 	uv = { 1, 1 } },
			{ pos = { 1, -1, -1 },  	uv = { 1, 0 } },

			{ pos = { -1, 1, 1 }, 	 	uv = { 0, 1 } },
			{ pos = { 1, 1, 1 },  		uv = { 1, 1 } },
			{ pos = { -1, -1, 1 }, 		uv = { 0, 0 } },
			{ pos = { -1, -1, 1 },  	uv = { 0, 0 } },
			{ pos = { 1, 1, 1 },  		uv = { 1, 1 } },
			{ pos = { 1, -1, 1 },  		uv = { 1, 0 } },
		}

		for k,v in ipairs(vertices) do
			v.color = {math.random(), math.random(), math.random()}
		end

		self.Vertices = self:CreateBuffer(vk.e.BUFFER_USAGE_VERTEX_BUFFER_BIT, create_vertices(vertices))
	end

	do -- indices
		local indices_type = ffi.typeof("uint32_t[?]")

		local create_indices = function(tbl) return indices_type(#tbl, tbl) end

		-- kind of pointless to use indices like this but whatever
		local indices = {}
		for i = 0, #vertices - 1 do
			table.insert(indices, i)
		end

		self.Indices = self:CreateBuffer(vk.e.BUFFER_USAGE_INDEX_BUFFER_BIT, create_indices(indices))
		self.Indices.count = #indices
	end

	do -- uniforms
		local matrix_type = require("matrix44")

		local uniforms_type = ffi.typeof("struct { $ projection; $ view; $ world; }", matrix_type, matrix_type, matrix_type)
		local create_uniforms = uniforms_type

		local create_matrix = function()
			return matrix_type(
				1,0,0,0,
				0,1,0,0,
				0,0,1,0,
				0,0,0,1
			)
		end

		local uniforms = create_uniforms
		{
			projection = create_matrix(),
			view = create_matrix(),
			world = create_matrix(),
		}

		self.ProjectionMatrix = uniforms.projection
		self.ViewMatrix = uniforms.view
		self.ModelMatrix = uniforms.world

		self.ProjectionMatrix:Perspective(math.rad(90), 32000, 0.1, self.Width / self.Height)
		self.ViewMatrix:Translate(0,0,-5)
		self.ModelMatrix:Rotate(0.5, 0,1,0)

		self.Uniforms = self:CreateBuffer(vk.e.BUFFER_USAGE_UNIFORM_BUFFER_BIT, uniforms)
		self.uniform_ref = uniforms


		--self.ViewMatrix:Translate(5,3,10)
		--self.ViewMatrix:Rotate(math.rad(90), 0,-1,0)
	end

	-- update uniforms
	self.Device:UpdateDescriptorSets(
		2, vk.s.WriteDescriptorSetArray{
			{
				dstSet = self.DescriptorSet,
				descriptorType = vk.e.DESCRIPTOR_TYPE_UNIFORM_BUFFER,
				dstBinding = 0,

				descriptorCount = 1,
				pBufferInfo = vk.s.DescriptorBufferInfoArray{
					{
						buffer = self.Uniforms.buffer,
						range = self.Uniforms.size,
						offset = 0,
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
		0, nil
	)

	local code, size = vk.util.GLSLToSpirV("vert", [[
		#version 450

		in layout(location = 0) vec3 position;
		in layout(location = 1) vec2 uv;
		in layout(location = 2) vec3 color;

		uniform layout (std140, binding = 0) matrices_t
		{
			mat4 projection;
			mat4 view;
			mat4 world;
		} matrices;

		// to fragment
		out layout(location = 0) vec2 frag_uv;
		out layout(location = 1) vec3 frag_color;

		void main()
		{
			gl_Position = (matrices.projection * matrices.view * matrices.world) * vec4(position, 1);

			// to fragment
			frag_uv = uv;
			frag_color = color;
		}
	]])

	self.VertexShader = self.Device:CreateShaderModule({
		codeSize = size,
		pCode = code,
	})

	local code, size = vk.util.GLSLToSpirV("frag", [[
		#version 450

		//from descriptor sets
		uniform layout(binding = 1) sampler2D tex;

		//from vertex
		in layout(location = 0) vec2 uv;
		in layout(location = 1) vec3 color;

		//to render pass (kinda)
		out layout(location = 0) vec4 frag_color;

		void main()
		{
			frag_color =  texture(tex, uv) * vec4(color, 1);
		}
	]])

	self.FragmentShader = self.Device:CreateShaderModule({
		codeSize = size,
		pCode = code,
	})

	self.Pipeline = self.Device:CreateGraphicsPipelines(nil, 1, {
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
				vertexAttributeDescriptionCount = 3,
				pVertexAttributeDescriptions = vk.s.VertexInputAttributeDescriptionArray{
					-- layout(location = 0) in vec3 position;
					{
						binding = 0,
						location = 0,
						format = vk.e.FORMAT_R32G32B32_SFLOAT,
						offset = 0,
					},
					-- layout(location = 1) in vec2 uv;
					{
						binding = 0,
						location = 1,
						format = vk.e.FORMAT_R32G32_SFLOAT,
						offset = ffi.sizeof(self.Vertices.data[0].pos),
					},
					-- layout(location = 2) in vec3 color;
					{
						binding = 0,
						location = 2,
						format = vk.e.FORMAT_R32G32B32_SFLOAT,
						offset = ffi.sizeof(self.Vertices.data[0].pos)  + ffi.sizeof(self.Vertices.data[0].uv),
					},
				},
			}),
			renderPass = self.RenderPass,

			stageCount = 2,
			pStages = vk.s.PipelineShaderStageCreateInfoArray{
				{
					stage = vk.e.SHADER_STAGE_VERTEX_BIT,
					module = self.VertexShader,
					pName = "main",
				},
				{
					stage = vk.e.SHADER_STAGE_FRAGMENT_BIT,
					module = self.FragmentShader,
					pName = "main",
				},
			},

			pInputAssemblyState = vk.s.PipelineInputAssemblyStateCreateInfo{
				topology = vk.e.PRIMITIVE_TOPOLOGY_TRIANGLE_LIST
			},

			pRasterizationState = vk.s.PipelineRasterizationStateCreateInfo{
				polygonMode = vk.e.POLYGON_MODE_FILL,
				cullMode = vk.e.CULL_MODE_NONE,--vk.e.CULL_MODE_BACK_BIT,
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

	self:FlushSetupCMD()

	for i, buffer in ipairs(self.SwapChainBuffers) do
		buffer.cmd:Begin(vk.s.CommandBufferBeginInfo{
			flags = 0,
			pInheritanceInfo = vk.s.CommandBufferInheritanceInfo{
				renderPass = nil,
				subpass = 0,
				framebuffer = nil,
				offclusionQueryEnable = vk.e.FALSE,
				queryFlags = 0,
				pipelineStatistics = 0,
			 }
		})

		buffer.cmd:BeginRenderPass(
			vk.s.RenderPassBeginInfo{
				renderPass = self.RenderPass,
				framebuffer = buffer.framebuffer,
				renderArea = {offset = {0, 0}, extent = {self.Width, self.Height}},
				clearValueCount = 2,
				pClearValues = vk.s.ClearValueArray{
					{color = {float32 = {0.2, 0.2, 0.2, 0.2}}},
					{depthStencil = {1, 0}}
				},
			},
			vk.e.SUBPASS_CONTENTS_INLINE
		)

		buffer.cmd:SetViewport(
			0,
			1, vk.s.ViewportArray{
				{0,0,self.Height,self.Width, 0,1}
			}
		)
		buffer.cmd:SetScissor(
			0,
			1, vk.s.Rect2DArray{
				{
					offset = {0, 0},
					extent = {self.Height, self.Width}
				}
			}
		)

		buffer.cmd:BindPipeline(vk.e.PIPELINE_BIND_POINT_GRAPHICS, self.Pipeline)

		buffer.cmd:BindDescriptorSets(vk.e.PIPELINE_BIND_POINT_GRAPHICS, self.PipelineLayout, 0, 1, vk.s.DescriptorSetArray{self.DescriptorSet}, 0, nil)
		buffer.cmd:BindVertexBuffers(0, 1, vk.s.BufferArray{self.Vertices.buffer}, ffi.new("unsigned long[1]", 0))
		buffer.cmd:BindIndexBuffer(self.Indices.buffer, 0, vk.e.INDEX_TYPE_UINT32)

		buffer.cmd:DrawIndexed(self.Indices.count, 1, 0, 0, 0)

		buffer.cmd:EndRenderPass()

		buffer.cmd:PipelineBarrier(
			vk.e.PIPELINE_STAGE_ALL_COMMANDS_BIT, vk.e.PIPELINE_STAGE_TOP_OF_PIPE_BIT, 0,
			0, nil,
			0, nil,
			1, vk.s.ImageMemoryBarrierArray{
				{
					srcAccessMask = vk.e.ACCESS_COLOR_ATTACHMENT_WRITE_BIT,
					dstAccessMask = vk.e.ACCESS_MEMORY_READ_BIT,
					oldLayout = vk.e.IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
					newLayout = vk.e.IMAGE_LAYOUT_PRESENT_SRC_KHR,
					srcQueueFamilyIndex = vk.e.QUEUE_FAMILY_IGNORED,
					dstQueueFamilyIndex = vk.e.QUEUE_FAMILY_IGNORED,
					subresourceRange = {
						aspectMask = vk.e.IMAGE_ASPECT_COLOR_BIT,

						levelCount = 1,
						baseMipLevel = 0,

						layerCount = 1,
						baseLayerLevel = 0
					},
					image = buffer.image,
				}
			}
		)

		buffer.cmd:End()
	end

	self:FlushSetupCMD()

	while glfw.WindowShouldClose(self.Window) == 0 do
		self.DeviceQueue:WaitIdle()
		self.Device:WaitIdle()

		glfw.PollEvents()

		local semaphore = self.Device:CreateSemaphore({
			flags = 0,
		})

		local index, status = self.Device:AcquireNextImage(self.SwapChain, vk.e.WHOLE_SIZE, semaphore, nil)
		index = index + 1

		if glfw.GetKey(self.Window, glfw.e.GLFW_KEY_W) == glfw.e.GLFW_PRESS then
			self.ViewMatrix:Translate(0,0,1)
		elseif glfw.GetKey(self.Window, glfw.e.GLFW_KEY_S) == glfw.e.GLFW_PRESS then
			self.ViewMatrix:Translate(0,0,-1)
		elseif glfw.GetKey(self.Window, glfw.e.GLFW_KEY_A) == glfw.e.GLFW_PRESS then
			self.ViewMatrix:Translate(1,0,0)
		elseif glfw.GetKey(self.Window, glfw.e.GLFW_KEY_D) == glfw.e.GLFW_PRESS then
			self.ViewMatrix:Translate(-1,0,0)
		end

		--self.ProjectionMatrix:Perspective(math.rad(90), 32000, 0.1, self.Width / self.Height)
		self:UpdateBuffer(self.Uniforms)

		self.DeviceQueue:Submit(
			1, vk.s.SubmitInfoArray{
				{
					pWaitDstStageMask = ffi.new("enum VkPipelineStageFlagBits [1]", vk.e.PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT),

					waitSemaphoreCount = 1,
					pWaitSemaphores = vk.s.SemaphoreArray{
						semaphore
					},

					signalSemaphoreCount = 0,
					pSignalSemaphores = nil,

					commandBufferCount = 1,
					pCommandBuffers = vk.s.CommandBufferArray{
						self.SwapChainBuffers[index].cmd
					},
				},
			},
			nil
		)

		self.DeviceQueue:Present(vk.s.PresentInfoKHR{
			swapchainCount = 1,
			pSwapchains = vk.s.SwapchainKHRArray{
				self.SwapChain
			},
			pImageIndices = ffi.new("unsigned int [1]", index - 1),
		})

		self.Device:DestroySemaphore(semaphore, nil)
	end
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

function DEMO:SetImageLayout(image, aspect_mask, old_image_layout, new_image_layout)
	local src_mask = 0
	local dst_mask = 0

    if old_image_layout == vk.e.IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL then
        src_mask = vk.e.ACCESS_COLOR_ATTACHMENT_WRITE_BIT
    end

    if new_image_layout == vk.e.IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL then
        dst_mask = vk.e.ACCESS_MEMORY_READ_BIT
    end

    if new_image_layout == vk.e.IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL then
        src_mask = bit.bor(vk.e.ACCESS_HOST_WRITE_BIT, vk.e.ACCESS_TRANSFER_WRITE_BIT)
        dst_mask = vk.e.ACCESS_SHADER_READ_BIT
    end

    if new_image_layout == vk.e.IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL then
        dst_mask = vk.e.ACCESS_COLOR_ATTACHMENT_READ_BIT
	end

    if new_image_layout == vk.e.IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL then
        dst_mask = vk.e.ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT
	end

	self.SetupCMD:PipelineBarrier(
		vk.e.PIPELINE_STAGE_TOP_OF_PIPE_BIT, vk.e.PIPELINE_STAGE_TOP_OF_PIPE_BIT, 0,
		0, nil,
		0, nil,
		1, vk.s.ImageMemoryBarrier({
			srcAccessMask = src_mask,
			dstAccessMask = dst_mask,
			oldLayout = old_image_layout,
			newLayout = new_image_layout,
			image = image,
			subresourceRange = {
				aspectMask = aspect_mask,

				levelCount = 1,
				baseMipLevel = 0,

				layerCount = 1,
				baseLayerLevel = 0
			},
		})
	)
end

function DEMO:CreateSetupCMD()
	self.SetupCMD = self.Device:AllocateCommandBuffers({
		commandPool = self.DeviceCommandPool,
		level = vk.e.COMMAND_BUFFER_LEVEL_PRIMARY,
		commandBufferCount = 1,
	})
end

function DEMO:BeginSetupCMD()
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

function DEMO:FlushSetupCMD()
	if not self.SetupCMD then return end

	self.SetupCMD:End()
	self.DeviceQueue:Submit(
		1, vk.s.SubmitInfoArray{
			{
				waitSemaphoreCount = 0,
				pWaitSemaphores = nil,
				pWaitDstStageMask = nil,

				commandBufferCount = 1,
				pCommandBuffers = vk.s.CommandBufferArray{
					self.SetupCMD
				},

				signalSemaphoreCount = 0,
				pSignalSemaphores = nil
			}
		},
		nil
	)

	self.DeviceQueue:WaitIdle()

	self.Device:FreeCommandBuffers(
		self.DeviceCommandPool,
		1, vk.s.CommandBufferArray{
			self.SetupCMD
		}
	)
	self.SetupCMD = nil
end


function DEMO:CreateImage(width, height, format, usage, tiling, required_props, levels)
	local image = self.Device:CreateImage({
		imageType = vk.e.IMAGE_TYPE_2D,
		format = format,
		extent = {width, height, 1},
		mipLevels = levels or 1,
		arrayLayers = 1,
		samples = vk.e.SAMPLE_COUNT_1_BIT,
		tiling = tiling,
		usage = usage,
		flags = 0,
		queueFamilyIndexCount = 0,
		sharingMode = vk.e.SHARING_MODE_EXCLUSIVE,
		initialLayout = vk.e.IMAGE_LAYOUT_UNDEFINED,
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
		width = width,
		height = height,
	}
end

function DEMO:CreateTexture(file_name, format)
	format = format or vk.e.FORMAT_B8G8R8A8_UNORM

	local image_infos = freeimage.LoadImage(file_name)

	local texture = {}

	texture.width = image_infos[1].width
	texture.height = image_infos[1].height
	texture.mip_levels = #image_infos

	local properties = self.PhysicalDevice:GetFormatProperties(format)

	if bit.band(properties.linearTilingFeatures, vk.e.FORMAT_FEATURE_SAMPLED_IMAGE_BIT) ~= 0 then
		local image = self:CreateImage(
			texture.width,
			texture.height,
			format,
			bit.bor(vk.e.IMAGE_USAGE_TRANSFER_DST_BIT, vk.e.IMAGE_USAGE_SAMPLED_BIT),
			vk.e.IMAGE_TILING_OPTIMAL,
			vk.e.MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
			texture.mip_levels
		)

		texture.image = image.image
		texture.memory = image.memory
		texture.size = image.size
		texture.width = image.width
		texture.height = image.height
		texture.format = image.format

		-- copy the mip maps into temporary images
		for i, image_info in ipairs(image_infos) do
			local image = self:CreateImage(
				image_info.width,
				image_info.height,
				format,
				vk.e.VK_IMAGE_USAGE_TRANSFER_SRC_BIT,
				vk.e.IMAGE_TILING_LINEAR,
				vk.e.MEMORY_PROPERTY_HOST_VISIBLE_BIT
			)

			self.Device:MapMemory(image.memory, 0, image.size, 0, "uint8_t", function(data)
				--for i = 0, tonumber(info.size) - 1 do data[i] = math.random(255) end
				ffi.copy(data, image_info.data, image.size)
			end)

			self:SetImageLayout(image.image, vk.e.IMAGE_ASPECT_COLOR_BIT, vk.e.IMAGE_LAYOUT_UNDEFINED, vk.e.IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL)

			image_infos[i] = image
		end

		self:SetImageLayout(texture.image, vk.e.IMAGE_ASPECT_COLOR_BIT, vk.e.IMAGE_LAYOUT_UNDEFINED, vk.e.IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL)

		-- copy from temporary mip map images to main image
		for i, mip_map in ipairs(image_infos) do
			self.SetupCMD:CopyImage(
				mip_map.image, vk.e.IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
				texture.image, vk.e.IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
				1, vk.s.ImageCopyArray{
					{
						extent = {mip_map.width, mip_map.height, 1},

						srcSubresource = {
							aspectMask = vk.e.IMAGE_ASPECT_COLOR_BIT,
							baseArrayLayer = 0,
							mipLevel = 0,
							layerCount = 1,
						},
						srcOffset = { 0, 0, 0 },

						dstSubresource = {
							aspectMask = vk.e.IMAGE_ASPECT_COLOR_BIT,
							baseArrayLayer = 0,
							mipLevel = i - 1,
							layerCount = 1,
						},
						dstOffset = { 0, 0, 0 },
					}
				}
			)

			self.Device:DestroyImage(mip_map.image, nil)
			self.Device:FreeMemory(mip_map.memory, nil)
		end

		self:SetImageLayout(texture.image, vk.e.IMAGE_ASPECT_COLOR_BIT, vk.e.IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, vk.e.IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL)

		self:FlushSetupCMD()
		self:CreateSetupCMD()
	else
		texture.mip_levels = 1

		local info = self:CreateImage(texture.width, texture.height, format, vk.e.IMAGE_USAGE_SAMPLED_BIT, vk.e.IMAGE_TILING_LINEAR, vk.e.MEMORY_PROPERTY_HOST_VISIBLE_BIT)

		texture.image = info.image
		texture.memory = info.memory
		texture.size = info.size
		texture.width = info.width
		texture.height = info.height
		texture.format = info.format

		self.Device:MapMemory(info.memory, 0, info.size, 0, "uint8_t", function(data)
			--for i = 0, tonumber(info.size) - 1 do data[i] = math.random(255) end
			ffi.copy(data, image_infos[1].data, image_infos[1].size)
		end)

		self:SetImageLayout(info.image, vk.e.IMAGE_ASPECT_COLOR_BIT, vk.e.IMAGE_LAYOUT_UNDEFINED, vk.e.IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL)
	end

	texture.sampler = self.Device:CreateSampler({
		magFilter = vk.e.FILTER_LINEAR,
		minFilter = vk.e.FILTER_LINEAR,
		mipmapMode = vk.e.SAMPLER_MIPMAP_MODE_LINEAR,
		addressModeU = vk.e.SAMPLER_ADDRESS_MODE_REPEAT,
		addressModeV = vk.e.SAMPLER_ADDRESS_MODE_REPEAT,
		addressModeW = vk.e.SAMPLER_ADDRESS_MODE_REPEAT,
		ipLodBias = 0.0,
		anisotropyEnable = vk.e.TRUE,
		maxAnisotropy = 8,
		compareOp = vk.e.COMPARE_OP_NEVER,
		minLod = 0.0,
		maxLod = texture.mip_levels,
		borderColor = vk.e.BORDER_COLOR_FLOAT_OPAQUE_WHITE,
		unnormalizedCoordinates = 0,
	})

	texture.view = self.Device:CreateImageView({
		viewType = vk.e.IMAGE_VIEW_TYPE_2D,
		image = texture.image,
		format = format,
		flags = 0,
		components = {
			vk.e.COMPONENT_SWIZZLE_R,
			vk.e.COMPONENT_SWIZZLE_G,
			vk.e.COMPONENT_SWIZZLE_B,
			vk.e.COMPONENT_SWIZZLE_A
		},
		subresourceRange = {
			aspectMask = vk.e.IMAGE_ASPECT_COLOR_BIT,

			levelCount = texture.mip_levels,
			baseMipLevel = 0,

			layerCount = 1,
			baseLayerLevel = 0
		},
	})

	texture.format = format

	return texture
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

function DEMO:UpdateBuffer(buffer)
	ffi.copy(self.Device:MapMemory(buffer.memory, 0, buffer.size, 0), buffer.data, buffer.size)
	self.Device:UnmapMemory(buffer.memory)
end

DEMO:Initialize()