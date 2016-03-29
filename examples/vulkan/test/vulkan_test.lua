local ffi = require("ffi")

collectgarbage("stop")

package.path = package.path .. ";./../../?.lua"

local vk = require("vulkan/libvulkan")

_G.FFI_LIB = "../../glfw/libglfw.so"
local glfw = require("glfw/libglfw")

_G.FFI_LIB = "../../freeimage/libfreeimage.so"
local freeimage = require("freeimage/libfreeimage")

_G.FFI_LIB = nil

local context = {}
context.width = 1024
context.height = 768
context.device = nil
context.physical_device = nil
context.window = nil

do -- objects
	local function get_memory_type_from_properties(type_bits, requirements_mask)
		requirements_mask = vk.e.memory_property[requirements_mask] or requirements_mask
		if bit.band(type_bits, 1) == 1 then
			for i = 0, 32 - 1 do
				if bit.band(context.device_memory_properties.memoryTypes[i].propertyFlags, requirements_mask) == requirements_mask then
					return i
				end
			end
		else
			for i = 0, 32 - 1 do
				type_bits = bit.rshift(type_bits, 1)
			end
		end
		return type_bits
	end

	function Image(info)
		local image = context.device:CreateImage({
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

		local memory_requirements = context.device:GetImageMemoryRequirements(image)

		local memory = context.device:AllocateMemory({
			allocationSize = memory_requirements.size,
			memoryTypeIndex = get_memory_type_from_properties(memory_requirements.memoryTypeBits, info.required_props),
		})

		context.device:BindImageMemory(image, memory, 0)

		return {
			image = image,
			memory = memory,
			size = memory_requirements.size,
		}
	end

	function Texture(file_name, format)
		format = format or "b8g8r8a8_unorm"

		local image_infos = freeimage.LoadImageMipMaps(file_name)

		local self = {}

		self.width = image_infos[1].width
		self.height = image_infos[1].height
		self.mip_levels = #image_infos

		local properties = context.physical_device:GetFormatProperties(format)

		if bit.band(properties.linearTilingFeatures, vk.e.format_feature.sampled_image) ~= 0 then
			local image = Image({
				width = self.width,
				height = self.height,
				format = format,
				usage = {"transfer_dst", "sampled"},
				tiling = "optimal",
				required_props = "device_local",
				levels = self.mip_levels,
			})

			self.image = image.image
			self.memory = image.memory
			self.size = image.size

			-- copy the mip maps into temporary images
			for i, image_info in ipairs(image_infos) do
				local image = Image({
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

				context.device:MapMemory(image.memory, 0, image.size, 0, "uint8_t", function(data)
					ffi.copy(data, image_info.data, image.size)
				end)

				context.setup_cmd:SetImageLayout(image.image, "color", "undefined", "transfer_src_optimal")

				image_infos[i] = image
			end

			context.setup_cmd:SetImageLayout(self.image, "color", "undefined", "transfer_src_optimal")

			-- copy from temporary mip map images to main image
			for i, mip_map in ipairs(image_infos) do
				context.setup_cmd:CopyImage(mip_map.image, self.image, mip_map.width, mip_map.height, i - 1)
				context.device:DestroyImage(mip_map.image, nil)
				context.device:FreeMemory(mip_map.memory, nil)
			end

			context.setup_cmd:SetImageLayout(self.image, "color", "transfer_dst_optimal", "shader_read_only_optimal")

			context.setup_cmd:Flush()
			context.setup_cmd:Create()
		else
			self.mip_levels = 1

			local info = Image({
				width = self.width,
				height = self.height,
				format = format,
				usage = "sampled",
				tiling = "linear",
				required_props = "host_visible"
			})

			self.image = info.image
			self.memory = info.memory
			self.size = info.size

			context.device:MapMemory(info.memory, 0, info.size, 0, "uint8_t", function(data)
				ffi.copy(data, image_infos[1].data, image_infos[1].size)
			end)

			context.setup_cmd:SetImageLayout(info.image, "color", "undefined", "shader_read_only_optimal")
		end

		self.sampler = context.device:CreateSampler({
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
			maxLod = self.mip_levels,
			borderColor = "float_opaque_white",
			unnormalizedCoordinates = 0,
		})

		self.view = context.device:CreateImageView({
			viewType = "2d",
			image = self.image,
			format = format,
			flags = 0,
			components = {r = "r", g = "g", b = "b", a = "a"},
			subresourceRange = {
				aspectMask = "color",

				levelCount = self.mip_levels,
				baseMipLevel = 0,

				layerCount = 1,
				baseLayerLevel = 0
			},
		})

		self.format = format

		return self
	end

	function Buffer(usage, data)
		local size = ffi.sizeof(data)
		local buffer = context.device:CreateBuffer({
			size = size,
			usage = usage,
		})

		local memory = context.device:AllocateMemory({
			allocationSize = size,
			memoryTypeIndex = get_memory_type_from_properties(context.device:GetBufferMemoryRequirements(buffer).memoryTypeBits, "host_visible"),
		})

		context.device:MapMemory(memory, 0, size, 0, "float", function(cdata)
			ffi.copy(cdata, data, size)
		end)

		context.device:BindBufferMemory(buffer, memory, 0)

		local self = {
			buffer = buffer,
			memory = memory,
			data = data,
			size = size,
		}

		function self:UpdateBuffer()
			ffi.copy(context.device:MapMemory(self.memory, 0, self.size, 0), self.data, self.size)
			context.device:UnmapMemory(self.memory)
		end

		return self
	end

	do
		local matrix_type = require("matrix44")

		function Matrix44()
			return matrix_type(
				1,0,0,0,
				0,1,0,0,
				0,0,1,0,
				0,0,0,1
			)
		end
	end

	do -- static object
		context.setup_cmd = {}

		function context.setup_cmd:SetImageLayout(image, aspect_mask, old_layout, new_layout)
			local src_mask
			local dst_mask

			if old_layout == "color_attachment_optimal" then
				src_mask = "color_attachment_write"
			end

			if new_layout == "transfer_dst_optimal" then
				dst_mask = "memory_read"
			end

			if new_layout == "shader_read_only_optimal" then
				src_mask = {"host_write", "transfer_write"}
				dst_mask = "shader_read"
			end

			if new_layout == "color_attachment_optimal" then
				dst_mask = "color_attachment_read"
			end

			if new_layout == "depth_stencil_attachment_optimal" then
				dst_mask = "depth_stencil_attachment_read"
			end

			self.cmd:PipelineBarrier(
				"top_of_pipe", "top_of_pipe", 0,
				0, nil,
				0, nil,
				nil, {
					srcAccessMask = src_mask,
					dstAccessMask = dst_mask,
					oldLayout = old_layout,
					newLayout = new_layout,
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

		function context.setup_cmd:CopyImage(src, dst, w, h, mip_level)
			self.cmd:CopyImage(
				src, "transfer_src_optimal",
				dst, "transfer_dst_optimal",
				nil, {
					{
						extent = {w, h, 1},

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
							mipLevel = mip_level,
							layerCount = 1,
						},
						dstOffset = { 0, 0, 0 },
					}
				}
			)
		end

		function context.setup_cmd:Create()
			self.cmd = context.device:AllocateCommandBuffers({
				commandPool = context.device_command_pool,
				level = "primary",
				commandBufferCount = 1,
			})
		end

		function context.setup_cmd:Begin()
			self.cmd:Begin({
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

		function context.setup_cmd:Flush()
			if not self.cmd then return end

			self.cmd:End()
			context.device_queue:Submit(
				nil, {
					{
						waitSemaphoreCount = 0,
						pWaitSemaphores = nil,
						pWaitDstStageMask = nil,

						pCommandBuffers = {
							self.cmd
						},

						signalSemaphoreCount = 0,
						pSignalSemaphores = nil
					}
				},
				nil
			)

			context.device_queue:WaitIdle()

			context.device:FreeCommandBuffers(
				context.device_command_pool,
				nil, {
					self.cmd
				}
			)
			self.cmd = nil
		end
	end

end

do -- create glfw window
	glfw.SetErrorCallback(function(_, str) io.write(string.format("GLFW Error: %s\n", ffi.string(str))) end)
	glfw.Init()
	glfw.WindowHint(glfw.e.CLIENT_API, glfw.e.NO_API)

	context.window = glfw.CreateWindow(1024, 768, "vulkan", nil, nil)
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
		ppEnabledLayerNames = {
			--"VK_LAYER_LUNARG_threading",
			--"VK_LAYER_LUNARG_mem_tracker",
			--"VK_LAYER_LUNARG_object_tracker",
			--"VK_LAYER_LUNARG_draw_state",
			"VK_LAYER_LUNARG_param_checker",
			--"VK_LAYER_LUNARG_swapchain",
			--"VK_LAYER_LUNARG_device_limits",
			--"VK_LAYER_LUNARG_image",
			--"VK_LAYER_LUNARG_api_dump",
		},
		ppEnabledExtensionNames = glfw.GetRequiredInstanceExtensions({
			"VK_EXT_debug_report",
		}),
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

	context.instance = instance
end

do -- find and use a gpu
	for _, physical_device in ipairs(context.instance:GetPhysicalDevices()) do			-- get a list of vulkan capable hardware
		for queue_index, info in ipairs(physical_device:GetQueueFamilyProperties()) do			-- get a list of queues the hardware supports
			if bit.band(info.queueFlags, vk.e.queue.graphics) ~= 0 then				-- if this queue supports graphics use it
				queue_index = queue_index - 1

				local device, err = physical_device:CreateDevice({
					ppEnabledLayerNames = {
						--"VK_LAYER_LUNARG_threading",
						--"VK_LAYER_LUNARG_mem_tracker",
						--"VK_LAYER_LUNARG_object_tracker",
						--"VK_LAYER_LUNARG_draw_state",
						"VK_LAYER_LUNARG_param_checker",
						--"VK_LAYER_LUNARG_swapchain",
						--"VK_LAYER_LUNARG_device_limits",
						--"VK_LAYER_LUNARG_image",
						--"VK_LAYER_LUNARG_api_dump",
					},
					ppEnabledExtensionNames = {},
					pQueueCreateInfos = {
						{
							queueFamilyIndex = queue_index,
							queueCount = 1,
							pQueuePriorities = ffi.new("float[1]", 0), -- todo: public ffi use is bad!
							pEnabledFeatures = nil,
						}
					}
				})

				context.device_queue = device:GetQueue(queue_index, 0)
				context.device_command_pool = device:CreateCommandPool({queueFamilyIndex = queue_index})
				context.device_memory_properties = physical_device:GetMemoryProperties()
				context.physical_device = physical_device
				context.device = device

				break
			end
		end
	end
end

do -- setup the glfw window buffer
	local surface = glfw.CreateWindowSurface(context.instance, context.window, nil)
	local formats = context.physical_device:GetSurfaceFormats(surface)
	local capabilities = context.physical_device:GetSurfaceCapabilities(surface)

	local prefered_format = formats[1].format

	if prefered_format == vk.e.format.undefined then
		prefered_format = "b8g8r8a8_unorm"
	end

	if capabilities.currentExtent.width ~= 0xFFFFFFFF then
		context.width = capabilities.currentExtent.width
		context.height = capabilities.currentExtent.height
	end

	local present_mode = "immediate"

	for _, mode in ipairs(context.physical_device:GetSurfacePresentModes(surface)) do
		if mode == vk.e.present_mode.fifo then
			present_mode = "fifo"
			break
		end
	end

	context.surface = surface

	context.swap_chain = context.device:CreateSwapchain({
		surface = surface,
		minImageCount = math.min(capabilities.minImageCount + 1, capabilities.maxImageCount == 0 and math.huge or capabilities.maxImageCount),
		imageFormat = prefered_format,
		imagecolorSpace = formats[1].colorSpace,
		imageExtent = {context.width, context.height},
		imageUse = "color_attachment",
		preTransform = bit.band(capabilities.supportedTransforms, vk.e.surface_transform.identity) ~= 0 and "identity" or capabilities.currentTransform,
		compositeAlpha = "opaque",
		imageArrayLayers = 1,
		imageSharingMode = "exclusive",

		queueFamilyIndexCount = 0,
		pQueueFamilyIndices = nil,

		presentMode = present_mode,
		oldSwapchain = nil,
		clipped = true,
	})

	context.setup_cmd:Create()

	do -- depth buffer to use in render pass
		local format = "d16_unorm"

		local depth_buffer = Image({
			width = context.width,
			height = context.height,
			format = format,
			usage = {"depth_stencil_attachment", "transfer_src"},
			tiling = "optimal",
			required_props = "device_local"
		})

		context.setup_cmd:SetImageLayout(depth_buffer.image, "depth", "undefined", "depth_stencil_attachment_optimal")

		depth_buffer.view = context.device:CreateImageView({
			viewType = "2d",
			image = depth_buffer.image,
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

		depth_buffer.format = format
		context.depth_buffer = depth_buffer
	end

	context.render_pass = context.device:CreateRenderPass({
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
				format = context.depth_buffer.format,
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

	context.swap_chain_buffers = {}

	for i, image in ipairs(context.device:GetSwapchainImages(context.swap_chain)) do
		context.setup_cmd:SetImageLayout(image, "color", "undefined", "present_src")

		local view = context.device:CreateImageView({
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

		context.swap_chain_buffers[i] = {
			cmd = context.device:AllocateCommandBuffers({
				commandPool = context.device_command_pool,
				level = "primary",
				commandBufferCount = 1,
			}),
			framebuffer = context.device:CreateFramebuffer({
				renderPass = context.render_pass,

				pAttachments = {
					view,
					context.depth_buffer.view
				},

				width = context.width,
				height = context.height,
				layers = 1,
			}),
			image = image,
			view = view,
		}
	end
end

do -- data layout
	local descriptorsets_layout = context.device:CreateDescriptorSetLayout({
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

	context.pipeline_layout = context.device:CreatePipelineLayout({
		pSetLayouts = {
			descriptorsets_layout,
		},
	})

	context.descriptorsets = context.device:AllocateDescriptorSets({
		descriptorPool = context.device:CreateDescriptorPool({
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
			descriptorsets_layout
		},
	})
end

do
	context.texture = Texture("./volcano.png", "b8g8r8a8_unorm")
end

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

	local vertices = {
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

	context.vertices = Buffer("vertex_buffer", create_vertices(vertices))
	context.vertices.tbl = vertices
end

do -- indices
	local indices_type = ffi.typeof("uint32_t[?]")

	local create_indices = function(tbl) return indices_type(#tbl, tbl) end

	-- kind of pointless to use indices like this but whatever
	local indices = {}
	for i = 0, #context.vertices.tbl - 1 do
		table.insert(indices, i)
	end

	context.indices = Buffer("index_buffer", create_indices(indices))
	context.indices.tbl = indices
	context.indices.count = #indices
end

do -- uniforms
	local matrix_type = require("matrix44")

	local uniforms_type = ffi.typeof("struct { $ projection; $ view; $ world; }", matrix_type, matrix_type, matrix_type)
	local create_uniforms = uniforms_type

	local uniforms = create_uniforms
	{
		projection = Matrix44(),
		view = Matrix44(),
		world = Matrix44(),
	}

	context.projection_matrix = uniforms.projection
	context.view_matrix = uniforms.view
	context.model_matrix = uniforms.world

	context.projection_matrix:Perspective(math.rad(90), 32000, 0.1, context.width / context.height)
	context.view_matrix:Translate(0,0,-5)
	context.model_matrix:Rotate(0.5, 0,1,0)

	context.uniforms = Buffer("uniform_buffer", uniforms)

	--context.view_matrix:Translate(5,3,10)
	--context.view_matrix:Rotate(math.rad(90), 0,-1,0)
end

-- update uniforms
context.device:UpdateDescriptorSets(
	nil, {
		{
			dstSet = context.descriptorsets,
			descriptorType = "uniform_buffer",
			dstBinding = 0,

			pBufferInfo = {
				{
					buffer = context.uniforms.buffer,
					range = context.uniforms.size,
					offset = 0,
				}
			}
		},
		{
			dstSet = context.descriptorsets,
			descriptorType = "combined_image_sampler",
			dstBinding = 1,

			pImageInfo = {
				{
					sampler = context.texture.sampler,
					imageView = context.texture.view,
					imageLayout = "general",
				}
			}
		},
	},
	0, nil
)

context.pipeline = context.device:CreateGraphicsPipelines(nil, 1, {
	{
		layout = context.pipeline_layout,
		renderPass = context.render_pass,

		pVertexInputState = {
			pVertexBindingDescriptions = {
				{
					binding = 0,
					stride = ffi.sizeof(context.vertices.data[0]),
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
					offset = ffi.sizeof(context.vertices.data[0].pos),
				},
				-- layout(location = 2) in vec3 color;
				{
					binding = 0,
					location = 2,
					format = "r32g32b32_sfloat",
					offset = ffi.sizeof(context.vertices.data[0].pos)  + ffi.sizeof(context.vertices.data[0].uv),
				},
			},
		},

		pStages = {
			{
				stage = "vertex",
				module = context.device:CreateShaderModule(vk.util.GLSLToSpirV("vert", [[
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
				]])),
				pName = "main",
			},
			{
				stage = "fragment",
				module = context.device:CreateShaderModule(vk.util.GLSLToSpirV("frag", [[
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
				]])),
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

context.setup_cmd:Flush()

for _, buffer in ipairs(context.swap_chain_buffers) do
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
			renderPass = context.render_pass,
			framebuffer = buffer.framebuffer,
			renderArea = {offset = {0, 0}, extent = {context.width, context.height}},
			pClearValues = {
				{color = {float32 = {0.2, 0.2, 0.2, 0.2}}},
				{depthStencil = {1, 0}}
			},
		},
		"inline"
	)

	buffer.cmd:SetViewport(0, nil,
		{
			{0,0,context.height,context.width, 0,1}
		}
	)
	buffer.cmd:SetScissor(0, nil,
		{
			{
				offset = {0, 0},
				extent = {context.height, context.width}
			}
		}
	)

	buffer.cmd:BindPipeline("graphics", context.pipeline)
	buffer.cmd:BindDescriptorSets("graphics", context.pipeline_layout, 0, nil, {context.descriptorsets}, 0, nil)


	buffer.cmd:BindVertexBuffers(0, nil, {context.vertices.buffer}, ffi.new("unsigned long[1]", 0))
	buffer.cmd:BindIndexBuffer(context.indices.buffer, 0, "uint32")

	buffer.cmd:DrawIndexed(context.indices.count, 1, 0, 0, 0)

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

context.setup_cmd:Flush()

while glfw.WindowShouldClose(context.window) == 0 do
	context.device_queue:WaitIdle()
	context.device:WaitIdle()

	glfw.PollEvents()

	local semaphore = context.device:CreateSemaphore({
		flags = 0,
	})

	local index = context.device:AcquireNextImage(context.swap_chain, vk.e.WHOLE_SIZE, semaphore, nil)
	index = index + 1

	if glfw.GetKey(context.window, glfw.e.KEY_W) == glfw.e.PRESS then
		context.view_matrix:Translate(0,0,0.1)
	elseif glfw.GetKey(context.window, glfw.e.KEY_S) == glfw.e.PRESS then
		context.view_matrix:Translate(0,0,-0.1)
	elseif glfw.GetKey(context.window, glfw.e.KEY_A) == glfw.e.PRESS then
		context.view_matrix:Translate(0.1,0,0)
	elseif glfw.GetKey(context.window, glfw.e.KEY_D) == glfw.e.PRESS then
		context.view_matrix:Translate(-0.1,0,0)
	end

	--context.projection_matrix:Perspective(math.rad(90), 32000, 0.1, context.width / context.height)
	context.uniforms:Update()

	context.device_queue:Submit(nil,
		{
			{
				pWaitDstStageMask = ffi.new("enum VkPipelineStageFlagBits [1]", vk.e.pipeline_stage.bottom_of_pipe),


				pWaitSemaphores = {
					semaphore
				},

				signalSemaphoreCount = 0,
				pSignalSemaphores = nil,


				pCommandBuffers = {
					context.swap_chain_buffers[index].cmd
				},
			},
		},
		nil
	)

	context.device_queue:Present({
		pSwapchains = {
			context.swap_chain
		},
		pImageIndices = ffi.new("unsigned int [1]", index - 1),
	})

	context.device:DestroySemaphore(semaphore, nil)
end