package.path = package.path .. ";../?.lua"
local ffibuild = require("ffibuild")

local header = ffibuild.NixBuild({
	package_name = "libarchive",
	library_name = "libarchive",
	src = [[
	#include "archive.h"
	#include "archive_entry.h"
]]})

local meta_data = ffibuild.GetMetaData(header)
meta_data.functions.archive_entry_acl_next_w = nil
meta_data.functions.archive_write_open_FILE.arguments[2] = ffibuild.CreateType("type", "void *")
meta_data.functions.archive_read_open_FILE.arguments[2] = ffibuild.CreateType("type", "void *")
meta_data.functions.archive_entry_copy_stat.arguments[2] = ffibuild.CreateType("type", "void *")
meta_data.functions.archive_read_disk_entry_from_file.arguments[4] = ffibuild.CreateType("type", "const void *")
meta_data.functions.archive_entry_stat.return_type = ffibuild.CreateType("type", "void *")

local header = meta_data:BuildMinimalHeader(function(name) return name:find("^archive_") end, function(name) return name:find("^ARCHIVE_") end, true, true)

local lua = ffibuild.StartLibrary(header)

lua = lua .. "library = " .. meta_data:BuildFunctions("^archive_(.+)", "foo_bar", "FooBar")
lua = lua .. "library.e = " .. meta_data:BuildEnums("^ARCHIVE_(.+)", "./include/archive.h", "ARCHIVE_")

ffibuild.EndLibrary(lua, header)