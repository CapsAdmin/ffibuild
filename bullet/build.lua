if render then
	os.execute("cd /home/caps/goluwa/goluwa/data/ffibuild/bullet/ && bash make.sh")
	return
end

package.path = package.path .. ";../?.lua"
local ffibuild = require("ffibuild")

--ffibuild.Clone("https://github.com/bulletphysics/bullet3.git", "repo/bullet")
--ffibuild.Clone("https://github.com/AndresTraks/BulletSharpPInvoke.git", "repo/libbulletc")

--os.execute("mkdir -p repo/libbulletc/libbulletc/build")
--os.execute("cd repo/libbulletc/libbulletc/build && cmake .. && make")
--os.execute("cp repo/libbulletc/libbulletc/build/libbulletc.so .")

ffibuild.lib_name = "bullet"

local header = ffibuild.ProcessSourceFileGCC([[
	#include "bulletc.h"

]], "-I./repo/libbulletc/libbulletc/src/")

local meta_data = ffibuild.GetMetaData(header)

local objects = {}

for key, tbl in pairs(meta_data.functions) do
	if key:sub(1, 2) == "bt" then
		local t = key:match("^(.+)_[^_]+$")
		if t then
			objects[t] = objects[t] or {ctors = {}, functions = {}}
		else
--			print(key)
		end
	else
	--	print(key)
	end
end

for t, data in pairs(objects) do
	for key, tbl in pairs(meta_data.functions) do
		if key:find(t, 0, true) then
			if key:find("_new", nil, true) then
				table.insert(objects[t].ctors, key)
			else
				local friendly = key:sub(#t+2)
				if friendly == "" then friendly = key end
				table.insert(objects[t].functions, {func = key, friendly = friendly})
			end
		end
	end
end

local header = meta_data:BuildMinimalHeader(function(name) return name:find("^bt%u") end, function(name) return name:find("^bt%u") end, true, true)

local lua = ffibuild.StartLibrary(header)

local ffi = require("ffi")
local clib = ffi.load("./libbullet.so")
ffi.cdef(header)
lua = lua .. "library = " .. meta_data:BuildFunctions("^bt(%u.+)", nil, nil, nil, function(name)
	local ok, err = pcall(function() return clib[name] end)
	if not pcall(function() return clib[name] end) then
		return false
	end
end)
lua = lua .. "library.e = " .. meta_data:BuildEnums("^bt(%u.+)")

do
	collectgarbage()

	for k,v in pairs(objects) do
		local s = ""

		s = s .. "do -- " .. k .. "\n"
		s = s .. "\tlocal META = {}\n"
		s = s .. "\tMETA.__index = META\n"

		for i,v in ipairs(v.ctors) do
			if i == 1 then i = "" end
			s = s .. "\tfunction library.Create" .. k:sub(3) .. i .. "(...)\n"
			s = s .. "\t\tlocal self = setmetatable({}, META)\n"
			s = s .. "\t\tself.ptr = CLIB."..v.."(...)\n"
			s = s .. "\t\treturn self\n"
			s = s .. "\tend\n"
		end

		for k,v in ipairs(v.functions) do
			s = s .. "\tfunction META:" .. v.friendly .. "(...)\n"
			s = s .. "\t\treturn CLIB." .. v.func .. "(self.ptr, ...)\n"
			s = s .. "\tend\n"
		end

		s = s .. "end\n"

		lua = lua .. s
	end
end

ffibuild.EndLibrary(lua, header)
