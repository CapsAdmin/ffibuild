local ffibuild = dofile("../../ffibuild.lua")

local header = ffibuild.GetHeader([[
	#define PURPLE_PLUGINS
	#include <libpurple/purple.h>
]], "$(pkg-config purple --cflags)")

-- these callbacks have been auto generated
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
local header = ffibuild.StripHeader(bottom, meta_data, function(func_name) return func_name:find("^purple_") or func_name:find("^__signal_callback__") end, true)

local lua = ffibuild.BuildGenericHeader(header, "purple", "purple")
lua = lua .. ffibuild.BuildMetaTableFunctions()
lua = lua .. ffibuild.BuildHelperFunctions()

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

	for func_name, type in pairs(meta_data.functions) do
		if func_name:find("__signal_callback__") then
			local arg_line =  {}
			local wrap_line = {}

			if type.arguments then
				for i, arg in ipairs(type.arguments) do
					arg = arg:GetPrimitive(meta_data) or arg
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

			if type.return_type:GetBasicType() ~= "void" then
				ret_line = " if ret == nil then return ffi.new(\"" .. type.return_type:GetBasicType() .. "\") end " .. ret_line
			end

			lua = lua .. "callbacks[\"" .. func_name:gsub("__signal_callback__", ""):gsub("_", "-") .. "\"] = {\n"
			lua = lua .. "\twrap = function(" .. arg_line .. ") local ret = callback(" .. wrap_line .. ") " .. ret_line .. " end,\n"
			lua = lua .. "\tdefinition = \"" .. type:GetDeclaration(nil, true) .. "\",\n"
			lua = lua .. "}\n"
		end
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
