local ffibuild = dofile("../../ffibuild.lua")

local header = ffibuild.GetHeader([[
	#define PURPLE_PLUGINS
	#include <libpurple/purple.h>
]], "$(pkg-config purple --cflags)")

local meta_data = ffibuild.GetMetaData(header)
local top, bottom = ffibuild.SplitHeader(header, {"_Purple", "purple"})
local header = ffibuild.StripHeader(bottom, meta_data, function(func_name) return func_name:find("^purple_") end, true)

local lua = ffibuild.BuildGenericHeader(header, "purple")
lua = lua .. ffibuild.BuildHelperFunctions("metatables", "chars_to_string")

lua = lua .. [[
local function glist_to_table(l, meta_name)
	if not metatables[meta_name] then
		error((meta_name or "nil") .. " is not a valid meta type", 3)
	end

	local out = {}

	for i = 1, math.huge do
		out[i] = ffi.cast(metatables[meta_name].ctype, l.data)
		out[i] = wrap_pointer(out[i], meta_name)
		l = l.next
		if l == nil then
			break
		end
	end

	return out
end

ffi.cdef("struct GList * g_list_insert(struct GList *, void *, int);\nstruct GList * g_list_alloc();")

local function table_to_glist(tbl)
	local list = CLIB.g_list_alloc()
	for i, v in ipairs(tbl) do
		CLIB.g_list_insert(list, v.ptr, -1)
	end
	return list
end

ffi.cdef([==[
	void *malloc(size_t size);
	void free(void *ptr);
]==])

local function replace_buffer(buffer, size, data)
	ffi.C.free(buffer[0])
	local chr = ffi.C.malloc(size)
	ffi.copy(chr, data)
	buffer[0] = chr
end

local function create_boxed_table(p, wrap_in, wrap_out)
	return setmetatable({p = p}, {__newindex = function(s, k, v) wrap_in(s.p, v) end, __index = function(s, k) return wrap_out(s.p) end})
end
]]

local objects = {}
local libraries = {}

local function argument_translate(declaration, name, type)
	if declaration == "sturct _GList *" or declaration == "struct _GList *" then
		return "table_to_glist(" .. name .. ")"
	elseif objects[type:GetBasicType()] then
		return name .. ".ptr"
	end
end

local function return_translate(declaration, type)
	if declaration == "const char *" or declaration == "char *" then
		return "v = chars_to_string(v)"
	elseif declaration == "sturct _GList *" or declaration == "struct _GList *" then
		return "v = glist_to_table(v, cast_type)", function(s) return s:gsub("^(.-)%)", function(s) if s:find(",") then return s .. ", cast_type)" else return s .. "cast_type)" end end) end
	elseif objects[type:GetBasicType()] then
		return "v = wrap_pointer(v, \"" .. objects[type:GetBasicType()].meta_name .. "\")"
	end
end

do -- metatables
	local prefix_translate = {
		ssl_connection = {"ssl"},
		conv_message = {"conversation_message"},
		request_fields = {"request_fields", "request_field"},
		srv_txt_query_data = {"srv_txt_query"},
		theme_manager = {"theme_loader"},
		stored_image = {"imgstore"},
		conversation = {"conv"},
		saved_status = {"savedstatus"},
	}

	for _, info in ipairs(ffibuild.GetStructTypes(meta_data, "^Purple(.+)")) do
		local basic_type = info.type:GetBasicType()
		objects[basic_type] = {meta_name = info.name, declaration = info.type:GetDeclaration(), functions = {}}

		local prefix = ffibuild.ChangeCase(basic_type:match("^struct[%s_]-Purple(.+)"), "FooBar", "foo_bar")

		for func_name, func_info in pairs(ffibuild.GetFunctionsStartingWithType(meta_data, info.type)) do
			local friendly = func_name:match("purple_" .. prefix .. "_(.+)")

			if not friendly and prefix_translate[prefix] then
				for _, prefix in ipairs(prefix_translate[prefix]) do
					friendly = func_name:match("purple_" .. prefix .. "_(.+)")
					if friendly then break end
				end
			end

			if friendly then
				friendly = ffibuild.ChangeCase(friendly, "foo_bar", "FooBar")
				objects[basic_type].functions[friendly] = func_info
				func_info.done = true
			end
		end

		if not next(objects[basic_type].functions) then
			objects[basic_type] = nil
		end
	end

	for _, info in pairs(objects) do
		lua = lua .. ffibuild.BuildMetaTable(info.meta_name, info.declaration, info.functions, argument_translate, return_translate, meta_data)
	end
end

do -- libraries
	for func_name, type in pairs(meta_data.functions) do
		if not type.done and func_name:find("^purple_") then
			local library_name, friendly_name = func_name:match("purple_(.-)_(.+)")

			if not library_name then
				library_name = "purple"
				friendly_name = func_name:match("purple_(.+)")
			end

			friendly_name = ffibuild.ChangeCase(friendly_name, "foo_bar", "FooBar")

			libraries[library_name] = libraries[library_name] or {}
			libraries[library_name][friendly_name] = type
		end
	end

	for library_name, functions in pairs(libraries) do
		lua = lua .. "library." .. library_name .. " = {\n"
		for friendly_name, func_type in pairs(functions) do
			lua = lua .. "\t" .. ffibuild.BuildFunction(friendly_name, func_type.name, func_type, argument_translate, return_translate, meta_data) .. ",\n"
		end
		lua = lua .. "}\n"
	end
end

do -- callbacks
	lua = lua .. "local callbacks = {}\n"

	local callbacks_meta_data = ffibuild.GetMetaData(dofile("callbacks.lua"))

	for func_name, func_type in pairs(callbacks_meta_data.functions) do
		func_type:MakePrimitive(meta_data)

		local arg_line =  {}
		local wrap_line = {}

		if func_type.arguments then
			for i, arg in ipairs(func_type.arguments) do
				local decl = arg:GetDeclaration()

				if decl == "const char *" or decl == "char *" then
					wrap_line[i] = "chars_to_string(_"..i..")"
				elseif decl == "sturct _GList *" or decl == "struct _GList *" then
					wrap_line[i] = "glist_to_table(_"..i..", 'void *')"
				elseif decl == "struct GList * *"then
					wrap_line[i] = "create_boxed_table(_"..i..", function(p, v) p[0] = table_to_glist(v) end, function(p) return glist_to_table(p[0]) end)"
				elseif decl == "char * *" then
					wrap_line[i] = "create_boxed_table(_"..i..", function(p, v) replace_buffer(p, #v, v) end, function(p) return ffi.string(p[0]) end)"
				elseif objects[arg:GetBasicType()] then
					wrap_line[i] = "wrap_pointer(_"..i..", \"" .. objects[arg:GetBasicType()].meta_name .. "\")"
				else
					wrap_line[i] = "_" .. i .. " --[["..decl.."]] "
				end

				arg_line[i] = "_" .. i
			end
		end

		table.insert(arg_line, 1, "callback")

		arg_line = table.concat(arg_line, ", ")
		wrap_line = table.concat(wrap_line, ", ")

		local ret_line = "return ret"

		if func_type.return_type:GetBasicType() ~= "void" then
			ret_line = " if ret == nil then return ffi.new(\"" .. func_type.return_type:GetBasicType() .. "\") end " .. ret_line
		end

		lua = lua .. "callbacks[\"" .. func_name:gsub("_", "-") .. "\"] = {\n"
		lua = lua .. "\twrap = function(" .. arg_line .. ") local ret = callback(" .. wrap_line .. ") " .. ret_line .. " end,\n"
		lua = lua .. "\tdefinition = \"" .. func_type:GetDeclaration(nil, true) .. "\",\n"
		lua = lua .. "}\n"
	end

	lua = lua .. [[
library.signal.Connect_real = library.signal.Connect

function library.signal.Connect(signal, callback)
	if not library.handles then error("unable to find handles (use purple.set_handles(plugin_handle, conversation_handle))", 2) end

	local info = callbacks[signal]

	if not info then error(signal .. " is not a valid signal!") end

	return library.signal.Connect_real(library.handles.conversation, signal, library.handles.plugin, ffi.cast("void(*)()", ffi.cast(info.definition, function(...)
		local ok, ret = pcall(info.wrap, callback, ...)

		if not ok then
			library.notify.Message(library.handles.plugin, "PURPLE_NOTIFY_MSG_INFO", "lua error", signal, ret, nil, nil)
			return default_value
		end

		return ret
	end)), nil)
end

function library.set_handles(plugin, conv)
	library.handles = {
		plugin = ffi.cast("struct _PurplePlugin *", plugin),
		conversation = conv,
	}
end

]]
end

lua = lua .. "return library\n"

if not RELOAD then
	io.write(lua) -- write output to make file

	-- check if this wokrs if possible
	if jit then
		require("ffi").cdef(header)
	end

	assert(loadstring(lua))
end
