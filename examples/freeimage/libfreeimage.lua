local ffi = require("ffi")
ffi.cdef([[]])
local CLIB = ffi.load(_G.FFI_LIB or "freeimage")
local library = {}
library = {
}
library.e = {
}
do
	do
		local function pow2ceil(n)
			return 2 ^ math.ceil(math.log(n) / math.log(2))
		end

		local function create_mip_map(bitmap, w, h, div)
			local width = pow2ceil(w)
			local height = pow2ceil(h)

			local size = width > height and width or height

			size = size / (2 ^ div)

			local new_bitmap = ffi.gc(library.Rescale(bitmap, size, size, library.e.IMAGE_FILTER_BILINEAR), library.Unload)

			return {
				data = library.GetBits(new_bitmap),
				size = library.GetMemorySize(new_bitmap),
				width = size,
				height = size,
				new_bitmap = new_bitmap,
			}
		end

		function library.LoadImageMipMaps(file_name, flags, format)
			local file = io.open(file_name, "rb")
			local data = file:read("*all")
			file:close()

			local buffer = ffi.cast("unsigned char *", data)

			local stream = library.OpenMemory(buffer, #data)
			local type = format or library.GetFileTypeFromMemory(stream, #data)

			local temp = library.LoadFromMemory(type, stream, flags or 0)
			local bitmap = library.ConvertTo32Bits(temp)


			local width = library.GetWidth(bitmap)
			local height = library.GetHeight(bitmap)

			local images = {}

			for level = 0, math.floor(math.log(math.max(width, height)) / math.log(2)) do
				images[level] = create_mip_map(bitmap, width, height, level)
			end

			library.Unload(bitmap)
			library.Unload(temp)

			library.CloseMemory(stream)

			return images
		end
	end

	function library.LoadImage(data, flags, format)
		local buffer = ffi.cast("unsigned char *", data)

		local stream = library.OpenMemory(buffer, #data)
		local type = format or library.GetFileTypeFromMemory(stream, #data)

		if type == library.e.FORMAT_UNKNOWN or type > library.e.FORMAT_RAW then -- huh...
			library.CloseMemory(stream)
			error("unknown format", 2)
		end

		local temp = library.LoadFromMemory(type, stream, flags or 0)
		local bitmap = library.ConvertTo32Bits(temp)
		library.Unload(temp)

		local data = library.GetBits(bitmap)
		local width = library.GetWidth(bitmap)
		local height = library.GetHeight(bitmap)

		ffi.gc(bitmap, library.Unload)

		library.CloseMemory(stream)

		return data, width, height
	end

	function library.LoadMultiPageImage(data, flags)
		local buffer = ffi.cast("const unsigned char *const ", data)

		local stream = library.OpenMemory(buffer, #data)
		local type = library.GetFileTypeFromMemory(stream, #data)

		local temp = library.LoadMultiBitmapFromMemory(type, stream, flags or 0)
		local count = library.GetPageCount(temp)

		local out = {}

		for page = 0, count - 1 do
			local temp = library.LockPage(temp, page)
			local bitmap = library.ConvertTo32Bits(temp)

			local tag = ffi.new("FI_TAG *[1]")
			library.GetMetadata(library.e.METADATA_ANIMATION, bitmap, "FrameLeft", tag)
			local x = tonumber(ffi.cast("int", library.GetTagValue(tag[0])))

			library.GetMetadata(library.e.METADATA_ANIMATION, bitmap, "FrameTop", tag)
			local y = tonumber(ffi.cast("int", library.GetTagValue(tag[0])))

			library.GetMetadata(library.e.METADATA_ANIMATION, bitmap, "FrameTime", tag)
			local ms = tonumber(ffi.cast("int", library.GetTagValue(tag[0]))) / 1000

			library.DeleteTag(tag[0])

			local data = library.GetBits(bitmap)
			local width = library.GetWidth(bitmap)
			local height = library.GetHeight(bitmap)

			ffi.gc(bitmap, library.Unload)

			table.insert(out, {w = width, h = height, x = x, y = y, ms = ms, data = data})
		end

		library.CloseMultiBitmap(temp, 0)

		return out
	end
end
library.clib = CLIB
return library
