package.path = package.path .. ";../../?.lua"
local ffibuild = require("ffibuild")

ffibuild.BuildSharedLibrary(
	"freetype",
	"git clone git://git.sv.nongnu.org/freetype/freetype2.git repo",
	"cd repo && mkdir build && cd build && cmake .. -DBUILD_SHARED_LIBS=1 && make && cd ../../ && cp repo/build/libfreetype.so.2.6.2 libfreetype.so"
)

local header = ffibuild.BuildCHeader([[
	#include <ft2build.h>

	typedef struct _FT_Glyph_Class {} FT_Glyph_Class;

	#include FT_CONFIG_CONFIG_H
	#include FT_LZW_H
	#include FT_CONFIG_STANDARD_LIBRARY_H
	#include FT_BZIP2_H
	#include FT_CONFIG_OPTIONS_H
	#include FT_WINFONTS_H
	#include FT_CONFIG_MODULES_H
	#include FT_GLYPH_H
	#include FT_FREETYPE_H
	#include FT_BITMAP_H
	#include FT_ERRORS_H
	#include FT_BBOX_H
	#include FT_MODULE_ERRORS_H
	#include FT_CACHE_H
	#include FT_SYSTEM_H
	#include FT_CACHE_IMAGE_H
	#include FT_IMAGE_H
	#include FT_CACHE_SMALL_BITMAPS_H
	#include FT_TYPES_H
	#include FT_CACHE_CHARMAP_H
	#include FT_LIST_H
	//#include FT_MAC_H
	#include FT_OUTLINE_H
	#include FT_BDF_H
	#include FT_MULTIPLE_MASTERS_H
	#include FT_SIZES_H
	#include FT_SFNT_NAMES_H
	#include FT_MODULE_H
	#include FT_OPENTYPE_VALIDATE_H
	#include FT_RENDER_H
	#include FT_GX_VALIDATE_H
	#include FT_AUTOHINTER_H
	#include FT_PFR_H
	#include FT_CFF_DRIVER_H
	#include FT_STROKER_H
	#include FT_TRUETYPE_DRIVER_H
	#include FT_SYNTHESIS_H
	#include FT_TYPE1_TABLES_H
	#include FT_FONT_FORMATS_H
	#include FT_TRUETYPE_IDS_H
	#include FT_TRIGONOMETRY_H
	#include FT_TRUETYPE_TABLES_H
	#include FT_LCD_FILTER_H
	#include FT_TRUETYPE_TAGS_H
	#include FT_UNPATENTED_HINTING_H
	#include FT_INCREMENTAL_H
	#include FT_CID_H
	#include FT_GASP_H
	#include FT_GZIP_H
	#include FT_ADVANCES_H
]], "-I./repo/include/")

print(header)


do return end


local meta_data = ffibuild.GetMetaData(header)
local header = meta_data:BuildMinimalHeader(function(name) return name:find("^FT_") end, function(name) return name:find("^FT_") or name:find("^BDF_") end, true, true)
local lua = ffibuild.BuildGenericLua(header, "freetype")

lua = lua .. "library = {\n"

for func_name, func_type in pairs(meta_data.functions) do
	if func_name:find("^FT_") then
		local friendly_name = func_name:match("FT_(.+)")
		lua = lua .. "\t" .. ffibuild.BuildLuaFunction(friendly_name, func_type.name, func_type) .. ",\n"
	end
end

lua = lua .. "}\n"

lua = lua .. "library.e = " .. ffibuild.BuildEnums(meta_data, "^FT_(.+)")

lua = lua .. "return library\n"

ffibuild.OutputAndValidate("freetype", lua, header)