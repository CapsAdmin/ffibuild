local ffi = require("ffi")

_G.FFI_LIB = "../../bgfx/libbgfx.so"
local bgfx = dofile("./../../bgfx/libbgfx.lua")
_G.FFI_LIB = nil

ffi.cdef("bool entry_process_events(uint32_t* _width, uint32_t* _height, uint32_t* _debug, uint32_t* _reset)")

local width  = ffi.new("uint32_t[1]", 1280)
local height = ffi.new("uint32_t[1]", 720)
local debug  = ffi.new("uint32_t[1]", bgfx.e.DEBUG_TEXT)
local reset  = ffi.new("uint32_t[1]", bgfx.e.RESET_VSYNC)

local cb = ffi.new("struct bgfx_callback_interface[1]", {
	{
		vtbl = ffi.new("struct bgfx_callback_vtbl[1]", {
			{ 
				fatal = print,
				trace_vargs = print,
			}
		})
	}
})

if bgfx.Init(bgfx.e.RENDERER_TYPE_OPENGL, bgfx.e.PCI_ID_NONE, 0, cb, nil) or true then
	bgfx.Reset(width[0], height[0], reset[0])

	-- Enable debug text.
	bgfx.SetDebug(debug[0])
	bgfx.SetViewClear(0, bit.bor(bgfx.e.CLEAR_COLOR, bgfx.e.CLEAR_DEPTH), 0x303030ff, 1.0, 0)


	while bgfx.clib.entry_process_events(width, height, debug, reset) ~= 0 do
		-- Set view 0 default viewport.
		bgfx.SetViewRect(0, 0, 0, width[0], height[0])

		-- This dummy draw call is here to make sure that view 0 is cleared
		-- if no other draw calls are submitted to view 0.
		bgfx.Touch(0)

		-- Use debug font to print information about this example.
		bgfx.DbgTextClear(0, false)
		--[[bgfx.DbgTextImage(
			uint16_max(width/2/8, 20)-20, 
			uint16_max(height/2/16, 6)-6, 
			40, 
			12, 
			s_logo, 
			160
		)]]
		bgfx.DbgTextPrintf(0, 1, 0x4, "bgfx/examples/25-c99")
		bgfx.DbgTextPrintf(0, 2, 0x6, "Description: Initialization and debug text with C99 API.")

		-- Advance to next frame. Rendering thread will be kicked to
		-- process submitted rendering primitives.
		bgfx.Frame()
	end

	bgfx.Shutdown()
else
	print("unable to init")
end
