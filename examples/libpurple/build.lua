local ffibuild = dofile("../../ffibuild.lua")

local header = ffibuild.GetHeader([[
	#define PURPLE_PLUGINS
	#include <libpurple/purple.h>
]], "$(pkg-config purple --cflags)")

-- these callbacks havebeen auto generated
header = header .. [[
void __signal_callback__buddy_status_changed ( PurpleBuddy * , PurpleStatus * , PurpleStatus * );
void __signal_callback__buddy_privacy_changed ( PurpleBuddy * );
void __signal_callback__buddy_idle_changed ( PurpleBuddy * , int , int );
void __signal_callback__buddy_signed_on ( PurpleBuddy * );
void __signal_callback__buddy_signed_off ( PurpleBuddy * );
void __signal_callback__buddy_got_login_time ( PurpleBuddy * );
void __signal_callback__blist_node_added ( PurpleBlistNode * );
void __signal_callback__blist_node_removed ( PurpleBlistNode * );
void __signal_callback__buddy_added ( PurpleBuddy * );
void __signal_callback__buddy_removed ( PurpleBuddy * );
void __signal_callback__buddy_icon_changed ( PurpleBuddy * );
void __signal_callback__update_idle (  );
void __signal_callback__blist_node_extended_menu ( PurpleBlistNode * , GList * *  );
void __signal_callback__blist_node_aliased ( PurpleBlistNode * , const char * );
void __signal_callback__buddy_caps_changed ( PurpleBuddy * , int , int );
void __signal_callback__certificate_stored ( PurpleCertificatePool * , const char * );
void __signal_callback__certificate_deleted ( PurpleCertificatePool * , const char * );
void __signal_callback__cipher_added ( PurpleCipher * );
void __signal_callback__cipher_removed ( PurpleCipher * );
void __signal_callback__cmd_added ( const char * , int , int );
void __signal_callback__cmd_removed ( const char * );
bool __signal_callback__writing_im_msg ( PurpleAccount * , const char * , char * * , PurpleConversation * , unsigned int );
void __signal_callback__wrote_im_msg ( PurpleAccount * , const char * , const char * , PurpleConversation * , unsigned int );
void __signal_callback__sent_attention ( PurpleAccount * , const char * , PurpleConversation * , unsigned int );
void __signal_callback__got_attention ( PurpleAccount * , const char * , PurpleConversation * , unsigned int );
void __signal_callback__sending_im_msg ( PurpleAccount * , const char * , char * * );
void __signal_callback__sent_im_msg ( PurpleAccount * , const char * , const char * );
bool __signal_callback__receiving_im_msg ( PurpleAccount * , char * * , char * * , PurpleConversation * , unsigned int * );
void __signal_callback__received_im_msg ( PurpleAccount * , const char * , const char * , PurpleConversation * , unsigned int );
void __signal_callback__blocked_im_msg ( PurpleAccount * , const char * , const char * , unsigned int , unsigned int );
bool __signal_callback__writing_chat_msg ( PurpleAccount * , const char * , char * * , PurpleConversation * , unsigned int );
void __signal_callback__wrote_chat_msg ( PurpleAccount * , const char * , const char * , PurpleConversation * , unsigned int );
void __signal_callback__sending_chat_msg ( PurpleAccount * , char * * , unsigned int );
void __signal_callback__sent_chat_msg ( PurpleAccount * , const char * , unsigned int );
bool __signal_callback__receiving_chat_msg ( PurpleAccount * , char * * , char * * , PurpleConversation * , unsigned int * );
void __signal_callback__received_chat_msg ( PurpleAccount * , const char * , const char * , PurpleConversation * , unsigned int );
void __signal_callback__conversation_created ( PurpleConversation * );
void __signal_callback__conversation_updated ( PurpleConversation * , unsigned int );
void __signal_callback__deleting_conversation ( PurpleConversation * );
void __signal_callback__buddy_typing ( PurpleAccount * , const char * );
void __signal_callback__buddy_typed ( PurpleAccount * , const char * );
void __signal_callback__buddy_typing_stopped ( PurpleAccount * , const char * );
bool __signal_callback__chat_buddy_joining ( PurpleConversation * , const char * , unsigned int );
void __signal_callback__chat_buddy_joined ( PurpleConversation * , const char * , unsigned int , bool );
void __signal_callback__chat_buddy_flags ( PurpleConversation * , const char * , unsigned int , unsigned int );
bool __signal_callback__chat_buddy_leaving ( PurpleConversation * , const char * , const char * );
void __signal_callback__chat_buddy_left ( PurpleConversation * , const char * , const char * );
void __signal_callback__deleting_chat_buddy ( PurpleConvChatBuddy * );
void __signal_callback__chat_inviting_user ( PurpleConversation * , const char * , char * * );
void __signal_callback__chat_invited_user ( PurpleConversation * , const char * , const char * );
int __signal_callback__chat_invited ( PurpleAccount * , const char * , const char * , const char * , void * );
void __signal_callback__chat_invite_blocked ( PurpleAccount * , const char * , const char * , const char * , GHashTable *  );
void __signal_callback__chat_joined ( PurpleConversation * );
void __signal_callback__chat_join_failed ( PurpleConnection * , void * );
void __signal_callback__chat_left ( PurpleConversation * );
void __signal_callback__chat_topic_changed ( PurpleConversation * , const char * , const char * );
void __signal_callback__cleared_message_history ( PurpleConversation * );
void __signal_callback__conversation_extended_menu ( PurpleConversation * , GList * *  );
bool __signal_callback__uri_handler ( const char * , const char * , GHashTable *  );
void __signal_callback__quitting (  );
bool __signal_callback__dbus_method_called ( void * , void * );
void __signal_callback__dbus_introspect ( void * * );
void __signal_callback__file_recv_accept ( PurpleXfer * );
void __signal_callback__file_send_accept ( PurpleXfer * );
void __signal_callback__file_recv_start ( PurpleXfer * );
void __signal_callback__file_send_start ( PurpleXfer * );
void __signal_callback__file_send_cancel ( PurpleXfer * );
void __signal_callback__file_recv_cancel ( PurpleXfer * );
void __signal_callback__file_send_complete ( PurpleXfer * );
void __signal_callback__file_recv_complete ( PurpleXfer * );
void __signal_callback__file_recv_request ( PurpleXfer * );
void __signal_callback__image_deleting ( PurpleStoredImage * );
const char * __signal_callback__log_timestamp ( PurpleLog * , int , int64_t , bool );
void __signal_callback__displaying_email_notification ( const char * , const char * , const char * , const char * );
void __signal_callback__displaying_emails_notification ( void * , void * , void * , void * , unsigned int );
void __signal_callback__displaying_userinfo ( PurpleAccount * , const char * , PurpleNotifyUserInfo * );
void __signal_callback__plugin_load ( PurplePlugin * );
void __signal_callback__plugin_unload ( PurplePlugin * );
void __signal_callback__savedstatus_changed ( PurpleSavedStatus * , PurpleSavedStatus * );
void __signal_callback__savedstatus_added ( PurpleSavedStatus * );
void __signal_callback__savedstatus_deleted ( PurpleSavedStatus * );
void __signal_callback__savedstatus_modified ( PurpleSavedStatus * );
bool __signal_callback__playing_sound_event ( int , PurpleAccount * );
void __signal_callback__account_modified ( PurpleAccount * );
void __signal_callback__gtkblist_hiding ( PurpleBuddyList * );
void __signal_callback__gtkblist_unhiding ( PurpleBuddyList * );
void __signal_callback__gtkblist_created ( PurpleBuddyList * );
void __signal_callback__drawing_tooltip ( PurpleBlistNode * , GString *  , bool );
const char * __signal_callback__drawing_buddy ( PurpleBuddy * );
const char * __signal_callback__conversation_timestamp ( PurpleConversation * , int , int64_t , bool );
bool __signal_callback__displaying_im_msg ( PurpleAccount * , const char * , char * * , PurpleConversation * , int );
void __signal_callback__displayed_im_msg ( PurpleAccount * , const char * , const char * , PurpleConversation * , int );
bool __signal_callback__displaying_chat_msg ( PurpleAccount * , const char * , char * * , PurpleConversation * , int );
void __signal_callback__displayed_chat_msg ( PurpleAccount * , const char * , const char * , PurpleConversation * , int );
void __signal_callback__conversation_switched ( PurpleConversation * );
bool __signal_callback__chat_nick_autocomplete ( PurpleConversation * );
bool __signal_callback__chat_nick_clicked ( PurpleConversation * , const char * , unsigned int );
]]

local meta_data = ffibuild.GetMetaData(header)
local top, bottom = ffibuild.SplitHeader(header, {"_Purple", "purple"})
local header = ffibuild.StripHeader(bottom, meta_data, function(func_name) return func_name:find("^purple_") or func_name:find("^__signal_callback__") end, nil, true)

local lua = ""

lua = lua .. "local header = [[" .. header .. "]]\n\n"
lua = lua ..
[[
local ffi = require("ffi")
ffi.cdef(header)
local CLIB = ffi.load("purple")
local purple = {}
local metatables = {}

-- todo: cache pointers?
local function wrap_object(ptr, meta_name)
	return setmetatable({ptr = ptr}, metatables[meta_name])
end

local function glist_to_table(l, meta_name)
	if not metatables[meta_name] then
		error((meta_name or "nil") .. " is not a valid meta type", 3)
	end

	local out = {}

	for i = 1, math.huge do
		out[i] = ffi.cast(metatables[meta_name].ctype, l.data)
		out[i] = wrap_object(out[i], meta_name)
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

local function chars_to_string(ctyp)
	if ctyp ~= nil then
		return ffi.string(ctyp)
	end
	return ""
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
	return setmetatable(
		{
			p = p
		},
		{
			__newindex = function(s, k, v)
				wrap_in(s.p, v)
			end,
			__index = function(s, k)
				return wrap_out(s.p)
			end
		}
	)
end

purple.wrap_object = wrap_object
purple.metatables = metatables
purple.P = P

]]

local meta_types = {}

local function get_meta_type(info)
	return meta_types[info.type]
end

local function argument_translate(declaration, name, type)
	if declaration == "sturct _GList" or declaration == "struct _GList" then
		return "table_to_glist(" .. name .. ")"
	elseif get_meta_type(type) then
		return name .. ".ptr"
	end
end

local function return_translate(declaration, type)
	if declaration == "const char*" or declaration == "char*" then
		return "v = chars_to_string(v)"
	elseif t == "sturct _GList" or t == "struct _GList" then
		return "v = glist_to_table(v, cast_type)", function(s) return s:gsub("^(.-)%)", function(s) if s:find(",") then return s .. ", cast_type)" else return s .. "cast_type)" end end) end
	elseif get_meta_type(type) then
		return "v = wrap_object(v, \"" .. get_meta_type(type) .. "\")"
	end
end

do -- metatables
	local prefix_translate = {
		PurpleSrvTxtQueryData = {"srv_txt_query"},
		PurpleConvMessage = {"conversation_message"},
		PurpleThemeManager = {"theme_loader"},
		PurpleStoredImage = {"imgstore"},
		PurpleConversation = {"conversation", "conv"},
		PurpleRequestFields = {"request_fields"},
		PurpleRequestFields = {"request_field"},
		PurpleSslConnection = {"ssl"},
		PurpleSavedStatus = {"savedstatus"},
	}

	local function starts_with(a, b)
		if type(b) == "table" then
			for _, b in ipairs(b) do
				if starts_with(a, b) then
					return true
				end
			end
		end

		return a:sub(0, #b) == b
	end

	local sorted = {}

	-- find all types that start with Purple and are structs
	for type_name, types in pairs(meta_data.typedefs) do
		if type_name:find("^Purple") then
			local type = types[#types]
			local name = type:GetDeclaration()
			if name:find("^struct ") then
				table.insert(sorted, name)
			end
		end
	end

	-- sort them by length to avoid functions like purple_>>conversation<<_foo_bar() to conflict with purple_>>conversation_im<<_foo_bar()
	table.sort(sorted, function(a, b) return #a > #b end)

	local objects = {}

	-- find all objects
	for _, type_name in ipairs(sorted) do
		local prefix = prefix_translate[type_name] or {ffibuild.ChangeCase(type_name:match("^struct[%s_]-Purple(.+)"), "FooBar", "foo_bar")}

		for func_name, info in pairs(meta_data.functions) do
			if
				info.arguments and
				not info.is_meta_function and
				starts_with(func_name:sub(#"purple_" + 1), prefix) and
				(info.arguments[1]:GetEvaluated(meta_data) or info.arguments[1]).type == type_name
			then
				local meta_name = prefix[1]

				local friendly_name = ffibuild.ChangeCase(func_name:sub(#("purple_" .. prefix[1]) + 2), "foo_bar", "FooBar")

				objects[meta_name] = objects[meta_name] or {functions = {}}
				objects[meta_name].tag = info.arguments[1]:GetDeclaration()
				objects[meta_name].functions[func_name] = objects[meta_name].functions[func_name] or info

				info.is_meta_function = true
				info.friendly_name = friendly_name

				meta_types[type_name] = meta_name
			end
		end
	end

	local s = ""

	for meta_name, info in pairs(objects) do
		s = s .. "do\n"
		s = s .. "\tlocal META = {\n"
		s = s .. "\t\tctype = ffi.typeof(\"" .. info.tag .. "\"),\n"
		for func_name, info in pairs(info.functions) do
			s = s .. "\t\t" .. ffibuild.BuildFunction(info.friendly_name, func_name, info, argument_translate, return_translate, meta_data, true) .. ",\n"
		end
		s = s .. "\t}\n"
		s = s .. "\tMETA.__index = META\n"
		s = s .. "\tmetatables." .. meta_name .. " = META\n"
		s = s .. "end\n"
	end

	lua = lua .. s
end

do -- libraries
	local libraries = {}

	for func_name, info in pairs(meta_data.functions) do
		if not info.is_meta_function and func_name:find("^purple_") then
			local library_name, friendly_name = func_name:match("purple_(.-)_(.+)")

			if not library_name then
				library_name = "purple"
				friendly_name = func_name:match("purple_(.+)")
			end

			info.friendly_name = ffibuild.ChangeCase(friendly_name, "foo_bar", "FooBar")

			libraries[library_name] = libraries[library_name] or {}
			libraries[library_name][func_name] = info
		end
	end

	local s = ""

	for library_name, functions in pairs(libraries) do
		s = s .. "purple." .. library_name .. " = {\n"
		for func_name, info in pairs(functions) do
			s = s .. "\t" .. ffibuild.BuildFunction(info.friendly_name, func_name, info, argument_translate, return_translate, meta_data) .. ",\n"
		end
		s = s .. "}\n"
	end

	lua = lua .. s
end

do -- callbacks
	local s = ""

	s = s .. "local callbacks = {}\n"

	for func_name, info in pairs(meta_data.functions) do
		if func_name:find("__signal_callback__") then
			local arg_line =  {}
			local wrap_line = {}

			if info.arguments then
				for i, arg in ipairs(info.arguments) do
					arg = arg:GetEvaluated(meta_data) or arg

					if arg.const and arg.type == "char" and arg.pointers == 1 then
						wrap_line[i] = "chars_to_string(_"..i..")"
					elseif arg.type == "sturct _GList" or arg.type == "struct _GList" then
						wrap_line[i] = "glist_to_table(_"..i..", 'void *')"
					elseif arg.type == "char" and arg.pointers == 2 then
						wrap_line[i] = "create_boxed_table(_"..i..", function(p, v) replace_buffer(p, #v, v) end, function(p) return ffi.string(p[0]) end)"
					elseif arg.type == "struct GList" and arg.pointers == 2 then
						wrap_line[i] = "create_boxed_table(_"..i..", function(p, v) p[0] = table_to_glist(v) end, function(p) return glist_to_table(p[0]) end)"
					elseif get_meta_type(arg) then
						wrap_line[i] = "wrap_object(_"..i..", \"" .. get_meta_type(arg) .. "\")"
					else
						wrap_line[i] = "_" .. i .. " --[["..arg.type.."]] "
					end


					arg_line[i] = "_" .. i
				end
			end

			table.insert(arg_line, 1, "callback")

			arg_line = table.concat(arg_line, ", ")
			wrap_line = table.concat(wrap_line, ", ")

			local ret_line = "return ret"

			if info.return_type.type ~= "void" then
				ret_line = " if ret == nil then return ffi.new(\"" .. info.return_type.type .. "\") end " .. ret_line
			end

			s = s .. "callbacks[\"" .. func_name:gsub("__signal_callback__", ""):gsub("_", "-") .. "\"] = {\n"
			s = s .. "\twrap = function(" .. arg_line .. ") local ret = callback(" .. wrap_line .. ") " .. ret_line .. " end,\n"
			s = s .. "\tdefinition = \"" .. info:GetDeclaration(true) .. "\",\n"
			s = s .. "}\n"
		end
	end

	lua = lua .. s .. [[
purple.signal.Connect_real = purple.signal.Connect

function purple.signal.Connect(signal, callback)
	if not purple.handles then error("unable to find handles (use purple.set_handles(plugin_handle, conversation_handle))", 2) end

	local info = callbacks[signal]

	if not info then error(signal .. " is not a valid signal!") end

	return purple.signal.Connect_real(purple.handles.conversation, signal, purple.handles.plugin, ffi.cast("void(*)()", ffi.cast(info.definition, function(...)
		local ok, ret = pcall(info.wrap, callback, ...)

		if not ok then
			purple.notify.Message(purple.handles.plugin, "PURPLE_NOTIFY_MSG_INFO", "lua error", signal, ret, nil, nil)
			return default_value
		end

		return ret
	end)), nil)
end

function purple.set_handles(plugin, conv)
	purple.handles = {
		plugin = ffi.cast("struct _PurplePlugin *", plugin),
		conversation = conv,
	}
end

]]
end

lua = lua .. "return purple\n"

if not RELOAD then
	io.write(lua) -- write output to make file

	-- check if this wokrs if possible
	if jit then
		require("ffi").cdef(header)
	end
end

assert(loadstring(lua))