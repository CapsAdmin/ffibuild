local ffi = require("ffi")

package.path = package.path .. ";./../../?.lua"

local vk = require("vulkan/libvulkan")

_G.FFI_LIB = "../../glfw/libglfw.so"
local glfw = require("glfw/libglfw")

_G.FFI_LIB = "../../freeimage/libfreeimage.so"
local freeimage = require("freeimage/libfreeimage")

_G.FFI_LIB = nil

local DEMO = {}

DEMO.InstanceValidationLayers = {
	--"VK_LAYER_LUNARG_threading",
	--"VK_LAYER_LUNARG_mem_tracker",
	--"VK_LAYER_LUNARG_object_tracker",
	--"VK_LAYER_LUNARG_draw_state",
	--"VK_LAYER_LUNARG_param_checker",
	--"VK_LAYER_LUNARG_swapchain",
	--"VK_LAYER_LUNARG_device_limits",
	--"VK_LAYER_LUNARG_image",
	--"VK_LAYER_LUNARG_api_dump",
}

DEMO.InstanceExtensions = {
	--"VK_EXT_debug_report",
}

DEMO.DeviceValidationLayers = DEMO.InstanceValidationLayers
DEMO.DeviceExtensions = {}--DEMO.InstanceExtensions

function DEMO:Initialize()

	local extensions

	do -- create glfw window
		glfw.SetErrorCallback(function(_, str) io.write(string.format("GLFW Error: %s\n", ffi.string(str))) end)
		glfw.Init()
		glfw.WindowHint(glfw.e.CLIENT_API, glfw.e.NO_API)

		extensions = glfw.GetRequiredInstanceExtensions()

		for _, ext in ipairs(self.InstanceExtensions) do
			table.insert(extensions, ext)
		end

		self.Window = glfw.CreateWindow(1024, 768, "vulkan", nil, nil)
	end

	do -- create vulkan instance
		local instance = vk.CreateInstance({
			pApplicationInfo = {
				pApplicationName = "goluwa",
				applicationVersion = 0,
				pEngineName = "goluwa",
				engineVersion = 0,
				apiVersion = vk.macros.MAKE_VERSION(1, 0, 2),
			},

			ppEnabledLayerNames = self.InstanceValidationLayers,
			ppEnabledExtensionNames = extensions,
		})

		if instance:LoadProcAddr("vkCreateDebugReportCallbackEXT") then
			instance:CreateDebugReportCallback({
				flags = {"information", "warning", "performance_warning", "error", "debug"},
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
				if bit.band(info.queueFlags, vk.e.queue.graphics) ~= 0 then			-- if this queue supports graphics use it

					self.PhysicalDevice = physical_device
					self.DeviceMemoryProperties = self.PhysicalDevice:GetMemoryProperties()
					self.DeviceQueueIndex = i - 1

					self.Device = self.PhysicalDevice:CreateDevice({
						ppEnabledLayerNames = self.DeviceValidationLayers,
						ppEnabledExtensionNames = self.DeviceExtensions,

						pQueueCreateInfos = {
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

		if prefered_format == vk.e.format.undefined then
			prefered_format = "b8g8r8a8_unorm"
		end

		self.Width = capabilities.currentExtent.width
		self.Height = capabilities.currentExtent.height

		if self.Width == 0xFFFFFFFF then
			self.Width = 1024
			self.Height = 768
		end

		local present_mode

		for _, mode in ipairs(self.PhysicalDevice:GetSurfacePresentModes(surface)) do
			if mode == vk.e.present_mode.fifo then
				present_mode = "fifo"
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
			imageUse = "color_attachment",
			preTransform = bit.band(capabilities.supportedTransforms, vk.e.surface_transform.identity) ~= 0 and "identity" or capabilities.currentTransform,
			compositeAlpha = "opaque",
			imageArrayLayers = 1,
			imageSharingMode = "exclusive",

			queueFamilyIndexCount = 0,
			pQueueFamilyIndices = nil,

			presentMode = present_mode or "immediate",
			oldSwapchain = nil,
			clipped = true,
		})

		self:CreateSetupCMD()

		do -- depth buffer to use in render pass
			local format = "d16_unorm"

			self.DepthBuffer = self:CreateImage({
				width = self.Width,
				height = self.Height,
				format = format,
				usage = {"depth_stencil_attachment", "transfer_src"},
				tiling = "optimal",
				required_props = "device_local"
			})

			self.DepthBuffer.format = format

			self:SetImageLayout(self.DepthBuffer.image, "depth", "undefined", "depth_stencil_attachment_optimal")

			self.DepthBuffer.view = self.Device:CreateImageView({
				viewType = "2d",
				image = self.DepthBuffer.image,
				format = format,
				flags = 0,
				subresourceRange = {
					aspectMask = {"depth", "stencil"},

					levelCount = 1,
					baseMipLevel = 0,

					layerCount = 1,
					baseLayerLevel = 0
				},
			})
		end

		self.RenderPass = self.Device:CreateRenderPass({
			pAttachments = {
				{
					format = prefered_format,
					samples = "1",
					loadop = "clear",
					storeop = "store",
					stencilloadop = "dont_care",
					stencilstoreop = "dont_care",
					initiallayout = "color_attachment_optimal",
					finallayout = "color_attachment_optimal",
				},
				{
					format = self.DepthBuffer.format,
					samples = "1",
					loadop = "clear",
					storeop = "dont_care",
					stencilloadop = "dont_care",
					stencilstoreop = "dont_care",
					initiallayout = "depth_stencil_attachment_optimal",
					finallayout = "depth_stencil_attachment_optimal",
				},
			},
			pSubpasses = {
				{
					pipelineBindPoint = "graphics",
					flags = 0,

					inputAttachmentCount = 0,
					pInputAttachments = nil,

					pColorAttachments = {
						{
							attachment = 0,
							layout = "color_attachment_optimal",
						},
					},

					pResolveAttachments = nil,

					pDepthStencilAttachment = {
						attachment = 1,
						layout = "depth_stencil_attachment_optimal",
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
			self:SetImageLayout(image, "color", "undefined", "present_src")

			local view = self.Device:CreateImageView({
				viewType = "2d",
				image = image,
				format = prefered_format,
				flags = 0,
				components = {r = "r", g = "g", b = "b", a = "a"},
				subresourceRange = {
					aspectMask = "color",

					levelCount = 1,
					baseMipLevel = 0,

					layerCount = 1,
					baseLayerLevel = 0
				},
			})

			self.SwapChainBuffers[i] = {
				cmd = self.Device:AllocateCommandBuffers({
					commandPool = self.DeviceCommandPool,
					level = "primary",
					commandBufferCount = 1,
				}),
				image = image,
				view = view,
				framebuffer = self.Device:CreateFramebuffer({
					renderPass = self.RenderPass,

					pAttachments = {
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
			pBindings = {
				{
					binding = 0,
					descriptorType = "uniform_buffer",
					descriptorCount = 1,
					stageFlags = "vertex",
					pImmutableSamplers = nil,
				},
				{
					binding = 1,
					descriptorType = "combined_image_sampler",
					descriptorCount = 1,
					stageFlags = "fragment",
					pImmutableSamplers = nil,
				},
			},
		})

		self.PipelineLayout = self.Device:CreatePipelineLayout({
			pSetLayouts = {
				self.DescriptorLayout,
			},
		})

		self.DescriptorSet = self.Device:AllocateDescriptorSets({
			descriptorPool = self.Device:CreateDescriptorPool({
				maxSets = 2,
				pPoolSizes = {
					{
						type = "uniform_buffer",
						descriptorCount = 1
					},
					{
						type = "combined_image_sampler",
						descriptorCount = 1
					},
				}
			}),

			pSetLayouts = {
				self.DescriptorLayout
			},
		})
	end

	do
		self.Texture = self:CreateTexture("./volcano.png", "b8g8r8a8_unorm")
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

		for _, vertex in ipairs(vertices) do
			vertex.color = {math.random(), math.random(), math.random()}
		end

		self.Vertices = self:CreateBuffer("vertex_buffer", create_vertices(vertices))
	end

	do -- indices
		local indices_type = ffi.typeof("uint32_t[?]")

		local create_indices = function(tbl) return indices_type(#tbl, tbl) end

		-- kind of pointless to use indices like this but whatever
		local indices = {}
		for i = 0, #vertices - 1 do
			table.insert(indices, i)
		end

		self.Indices = self:CreateBuffer("index_buffer", create_indices(indices))
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

		self.Uniforms = self:CreateBuffer("uniform_buffer", uniforms)
		self.uniform_ref = uniforms


		--self.ViewMatrix:Translate(5,3,10)
		--self.ViewMatrix:Rotate(math.rad(90), 0,-1,0)
	end

	-- update uniforms
	self.Device:UpdateDescriptorSets(
		nil,
		{
			{
				dstSet = self.DescriptorSet,
				descriptorType = "uniform_buffer",
				dstBinding = 0,

				pBufferInfo = {
					{
						buffer = self.Uniforms.buffer,
						range = self.Uniforms.size,
						offset = 0,
					}
				}
			},
			{
				dstSet = self.DescriptorSet,
				descriptorType = "combined_image_sampler",
				dstBinding = 1,

				pImageInfo = {
					{
						sampler = self.Texture.sampler,
						imageView = self.Texture.view,
						imageLayout = "general",
					}
				}
			},
		},
		0, nil
	)

	self.VertexShader = self.Device:CreateShaderModule(vk.util.GLSLToSpirV("vert", [[
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
	]]))

	self.FragmentShader = self.Device:CreateShaderModule(vk.util.GLSLToSpirV("frag", [[
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
	]]))

	self.Pipeline = self.Device:CreateGraphicsPipelines(nil, 1, {
		{
			layout = self.PipelineLayout,
			renderPass = self.RenderPass,

			pVertexInputState = {
				pVertexBindingDescriptions = {
					{
						binding = 0,
						stride = ffi.sizeof(self.Vertices.data[0]),
						inputRate = "vertex"
					}
				},
				pVertexAttributeDescriptions = {
					-- layout(location = 0) in vec3 position;
					{
						binding = 0,
						location = 0,
						format = "r32g32b32_sfloat",
						offset = 0,
					},
					-- layout(location = 1) in vec2 uv;
					{
						binding = 0,
						location = 1,
						format = "r32g32_sfloat",
						offset = ffi.sizeof(self.Vertices.data[0].pos),
					},
					-- layout(location = 2) in vec3 color;
					{
						binding = 0,
						location = 2,
						format = "r32g32b32_sfloat",
						offset = ffi.sizeof(self.Vertices.data[0].pos)  + ffi.sizeof(self.Vertices.data[0].uv),
					},
				},
			},

			pStages = {
				{
					stage = "vertex",
					module = self.VertexShader,
					pName = "main",
				},
				{
					stage = "fragment",
					module = self.FragmentShader,
					pName = "main",
				},
			},

			pInputAssemblyState = {
				topology = "triangle_list",
			},

			pRasterizationState = {
				polygonMode = "fill",
				cullMode = "none", -- "BACK",
				frontFace = "clockwise",
				depthClampEnable = false,
				rasterizerDiscardEnable = false,
				depthBiasEnable = false,
			},

			pColorBlendState = {
				pAttachments = {
					{
						colorWriteMask = 0xf,
						blendEnable = false,
					}
				}
			},

			pMultisampleState = {
				pSampleMask = nil,
				rasterizationSamples = "1",
			},

			pViewportState = {
				viewportCount = 1,
				scissorCount = 1,
			},

			pDepthStencilState = {
				depthTestEnable = true,
				depthWriteEnable = true,
				depthCompareOp = "less_or_equal",
				depthBoundsTestEnable = false,
				stencilTestEnable = false,
				back = {
					failOp = "keep",
					passOp = "keep",
					compareOp = "always",
				},
				front = {
					failOp = "keep",
					passOp = "keep",
					compareOp = "always",
				},
			},

			pDynamicState = {
				dynamicStateCount = 2,
				pDynamicStates = ffi.new("enum VkDynamicState[2]",
					vk.e.dynamic_state.viewport,
					vk.e.dynamic_state.scissor
				),
			},
		}
	})

	self:FlushSetupCMD()

	for _, buffer in ipairs(self.SwapChainBuffers) do
		buffer.cmd:Begin({
			flags = 0,
			pInheritanceInfo = {
				renderPass = nil,
				subpass = 0,
				framebuffer = nil,
				offclusionQueryEnable = false,
				queryFlags = 0,
				pipelineStatistics = 0,
			}
		})

		buffer.cmd:BeginRenderPass(
			{
				renderPass = self.RenderPass,
				framebuffer = buffer.framebuffer,
				renderArea = {offset = {0, 0}, extent = {self.Width, self.Height}},
				pClearValues = {
					{color = {float32 = {0.2, 0.2, 0.2, 0.2}}},
					{depthStencil = {1, 0}}
				},
			},
			"inline"
		)

		buffer.cmd:SetViewport(0, nil,
			{
				{0,0,self.Height,self.Width, 0,1}
			}
		)
		buffer.cmd:SetScissor(0, nil,
			{
				{
					offset = {0, 0},
					extent = {self.Height, self.Width}
				}
			}
		)

		buffer.cmd:BindPipeline("graphics", self.Pipeline)
		buffer.cmd:BindDescriptorSets("graphics", self.PipelineLayout, 0, nil, {self.DescriptorSet}, 0, nil)


		buffer.cmd:BindVertexBuffers(0, nil, {self.Vertices.buffer}, ffi.new("unsigned long[1]", 0))
		buffer.cmd:BindIndexBuffer(self.Indices.buffer, 0, "uint32")

		buffer.cmd:DrawIndexed(self.Indices.count, 1, 0, 0, 0)

		buffer.cmd:EndRenderPass()

		buffer.cmd:PipelineBarrier(
			"all_commands", "top_of_pipe", 0,
			0, nil,
			0, nil,
			nil, {
				{
					srcAccessMask = "color_attachment_write",
					dstAccessMask = "memory_read",
					oldLayout = "color_attachment_optimal",
					newLayout = "present_src",
					srcQueueFamilyIndex = vk.e.QUEUE_FAMILY_IGNORED,
					dstQueueFamilyIndex = vk.e.QUEUE_FAMILY_IGNORED,
					subresourceRange = {
						aspectMask = "color",

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

		local index = self.Device:AcquireNextImage(self.SwapChain, vk.e.WHOLE_SIZE, semaphore, nil)
		index = index + 1

		if glfw.GetKey(self.Window, glfw.e.KEY_W) == glfw.e.PRESS then
			self.ViewMatrix:Translate(0,0,0.1)
		elseif glfw.GetKey(self.Window, glfw.e.KEY_S) == glfw.e.PRESS then
			self.ViewMatrix:Translate(0,0,-0.1)
		elseif glfw.GetKey(self.Window, glfw.e.KEY_A) == glfw.e.PRESS then
			self.ViewMatrix:Translate(0.1,0,0)
		elseif glfw.GetKey(self.Window, glfw.e.KEY_D) == glfw.e.PRESS then
			self.ViewMatrix:Translate(-0.1,0,0)
		end

		--self.ProjectionMatrix:Perspective(math.rad(90), 32000, 0.1, self.Width / self.Height)
		self:UpdateBuffer(self.Uniforms)

		self.DeviceQueue:Submit(nil,
			{
				{
					pWaitDstStageMask = ffi.new("enum VkPipelineStageFlagBits [1]", vk.e.pipeline_stage.bottom_of_pipe),


					pWaitSemaphores = {
						semaphore
					},

					signalSemaphoreCount = 0,
					pSignalSemaphores = nil,


					pCommandBuffers = {
						self.SwapChainBuffers[index].cmd
					},
				},
			},
			nil
		)

		self.DeviceQueue:Present({
			pSwapchains = {
				self.SwapChain
			},
			pImageIndices = ffi.new("unsigned int [1]", index - 1),
		})

		self.Device:DestroySemaphore(semaphore, nil)
	end
end

function DEMO:GetMemoryTypeFromProperties(type_bits, requirements_mask)
	requirements_mask = vk.e.memory_property[requirements_mask] or requirements_mask
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

    if old_image_layout == "color_attachment_optimal" then
        src_mask = "color_attachment_write"
    end

    if new_image_layout == "transfer_dst_optimal" then
        dst_mask = "memory_read"
    end

    if new_image_layout == "shader_read_only_optimal" then
        src_mask = {"host_write", "transfer_write"}
        dst_mask = "shader_read"
    end

    if new_image_layout == "color_attachment_optimal" then
        dst_mask = "color_attachment_read"
	end

    if new_image_layout == "depth_stencil_attachment_optimal" then
        dst_mask = "depth_stencil_attachment_read"
	end

	self.SetupCMD:PipelineBarrier(
		"top_of_pipe", "top_of_pipe", 0,
		0, nil,
		0, nil,
		nil,
		{
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
		}
	)
end

function DEMO:CreateSetupCMD()
	self.SetupCMD = self.Device:AllocateCommandBuffers({
		commandPool = self.DeviceCommandPool,
		level = "primary",
		commandBufferCount = 1,
	})
end

function DEMO:BeginSetupCMD()
	self.SetupCMD:Begin({
		flags = 0,
		pInheritanceInfo = {
			renderPass = nil,
			subpass = 0,
			framebuffer = nil,
			offclusionQueryEnable = false,
			queryFlags = 0,
			pipelineStatistics = 0,
		}
	})
end

function DEMO:FlushSetupCMD()
	if not self.SetupCMD then return end

	self.SetupCMD:End()
	self.DeviceQueue:Submit(
		nil,
		{
			{
				waitSemaphoreCount = 0,
				pWaitSemaphores = nil,
				pWaitDstStageMask = nil,

				pCommandBuffers = {
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
		nil,
		{
			self.SetupCMD
		}
	)
	self.SetupCMD = nil
end


function DEMO:CreateImage(info)
	local image = self.Device:CreateImage({
		imageType = "2d",
		format = info.format,
		extent = {info.width, info.height, 1},
		mipLevels = info.levels or 1,
		arrayLayers = 1,
		samples = "1",
		tiling = info.tiling,
		usage = info.usage,
		flags = 0,
		queueFamilyIndexCount = 0,
		sharingMode = "exclusive",
		initialLayout = "undefined",
	})

	local memory_requirements = self.Device:GetImageMemoryRequirements(image)

	local memory = self.Device:AllocateMemory({
		allocationSize = memory_requirements.size,
		memoryTypeIndex = self:GetMemoryTypeFromProperties(memory_requirements.memoryTypeBits, info.required_props),
	})

	self.Device:BindImageMemory(image, memory, 0)

	return {
		image = image,
		memory = memory,
		size = memory_requirements.size,
	}
end

function DEMO:CreateTexture(file_name, format)
	format = format or "b8g8r8a8_unorm"

	local image_infos = freeimage.LoadImageMipMaps(file_name)

	local texture = {}

	texture.width = image_infos[1].width
	texture.height = image_infos[1].height
	texture.mip_levels = #image_infos

	local properties = self.PhysicalDevice:GetFormatProperties(format)

	if bit.band(properties.linearTilingFeatures, vk.e.format_feature.sampled_image) ~= 0 then
		local image = self:CreateImage({
			width = texture.width,
			height = texture.height,
			format = format,
			usage = {"transfer_dst", "sampled"},
			tiling = "optimal",
			required_props = "device_local",
			levels = texture.mip_levels,
		})

		texture.image = image.image
		texture.memory = image.memory
		texture.size = image.size

		-- copy the mip maps into temporary images
		for i, image_info in ipairs(image_infos) do
			local image = self:CreateImage({
				width = image_info.width,
				height = image_info.height,
				format = format,
				usage = "transfer_dst",
				tiling = "linear",
				required_props = "host_visible",
			})

			image.width = image_info.width
			image.height = image_info.height
			image.format = format

			self.Device:MapMemory(image.memory, 0, image.size, 0, "uint8_t", function(data)
				--for i = 0, tonumber(info.size) - 1 do data[i] = math.random(255) end
				ffi.copy(data, image_info.data, image.size)
			end)

			self:SetImageLayout(image.image, "color", "undefined", "transfer_src_optimal")

			image_infos[i] = image
		end

		self:SetImageLayout(texture.image, "color", "undefined", "transfer_src_optimal")

		-- copy from temporary mip map images to main image
		for i, mip_map in ipairs(image_infos) do
			self.SetupCMD:CopyImage(
				mip_map.image, "transfer_src_optimal",
				texture.image, "transfer_dst_optimal",
				nil,
				{
					{
						extent = {mip_map.width, mip_map.height, 1},

						srcSubresource = {
							aspectMask = "color",
							baseArrayLayer = 0,
							mipLevel = 0,
							layerCount = 1,
						},
						srcOffset = { 0, 0, 0 },

						dstSubresource = {
							aspectMask = "color",
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

		self:SetImageLayout(texture.image, "color", "transfer_dst_optimal", "shader_read_only_optimal")

		self:FlushSetupCMD()
		self:CreateSetupCMD()
	else
		texture.mip_levels = 1

		local info = self:CreateImage({
			width = texture.width,
			height = texture.height,
			format = format,
			usage = "sampled",
			tiling = "linear",
			required_props = "host_visible"
		})

		texture.image = info.image
		texture.memory = info.memory
		texture.size = info.size

		self.Device:MapMemory(info.memory, 0, info.size, 0, "uint8_t", function(data)
			ffi.copy(data, image_infos[1].data, image_infos[1].size)
		end)

		self:SetImageLayout(info.image, "color", "undefined", "shader_read_only_optimal")
	end

	texture.sampler = self.Device:CreateSampler({
		magFilter = "linear",
		minFilter = "linear",
		mipmapMode = "linear",
		addressModeU = "repeat",
		addressModeV = "repeat",
		addressModeW = "repeat",
		ipLodBias = 0.0,
		anisotropyEnable = true,
		maxAnisotropy = 8,
		compareOp = "never",
		minLod = 0.0,
		maxLod = texture.mip_levels,
		borderColor = "float_opaque_white",
		unnormalizedCoordinates = 0,
	})

	texture.view = self.Device:CreateImageView({
		viewType = "2d",
		image = texture.image,
		format = format,
		flags = 0,
		components = {r = "r", g = "g", b = "b", a = "a"},
		subresourceRange = {
			aspectMask = "color",

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
		memoryTypeIndex = self:GetMemoryTypeFromProperties(self.Device:GetBufferMemoryRequirements(buffer).memoryTypeBits, "host_visible"),
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