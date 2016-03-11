package.path = package.path .. ";../../?.lua"
local ffibuild = require("ffibuild")


ffibuild.BuildSharedLibrary(
	"archive",
	"https://github.com/libarchive/libarchive.git",
	"cmake . && make"
)

local header = ffibuild.BuildCHeader([[
	#include "archive.h"

	#include "archive_rb.h"
	#include "archive_pack_dev.h"
	#include "archive_entry.h"

]], "-I./repo/libarchive")

local meta_data = ffibuild.GetMetaData(header)
meta_data.functions.archive_entry_acl_next_w = nil
local header = meta_data:BuildMinimalHeader(function(name) return name:find("^archive_") end, function(name) return name:find("^ARCHIVE_") end, true, true)

local lua = ffibuild.StartLibrary(header)

lua = lua .. "library = " .. meta_data:BuildFunctions("^archive_(.+)", "foo_bar", "FooBar")
lua = lua .. "library.e = " .. meta_data:BuildEnums("^ARCHIVE_(.+)", "./repo/libarchive/archive.h", "ARCHIVE_")

ffibuild.EndLibrary(lua, header)