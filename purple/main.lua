local path, plugin_handle, conv_handle = ...
local plugin_dir = path:gsub("/main.lua", "")

package.path = package.path .. ";" .. plugin_dir .. "/?.lua"

local ffi = require("ffi")
local purple = require("libpurple")
purple.set_handles(plugin_handle, conv_handle)

do -- extended
	purple.buddy_list = purple.blist
	purple.blist = nil

	function purple.buddy_list.FindByName(name)
		for _, buddy in ipairs(purple.buddy_list.GetBuddies("buddy")) do
			if buddy:GetName() == name then
				return buddy
			end
		end
	end
end

do
	local easylua = {}

	local commands = {}

	commands.l = {
		help = "run lua",
		callback = function(...)
			return easylua.RunLua(...)
		end,
	}

	commands.print = {
		help = "print lua",
		callback = function(code, ...)
			return easylua.RunLua("print(" .. code .. ")", ...)
		end,
	}

	local function msg(str, account, who)
		purple.conversation.New("PURPLE_CONV_TYPE_IM", account, who):GetImData():Send(str)
	end

	function easylua.RunString(msg, account, who)
		local name, cmd = msg:match("^!(.-)%s+(.+)$")

		if commands[name] then
			easylua.PCall(
				commands[name].callback, account, who,
				cmd, account, who
			)
		end
	end

	function easylua.PCall(func, account, who, ...)
		local ret = {pcall(func, ...)}
		if ret[1] then
			table.remove(ret, 1)
			if #ret > 0 then
				for i, v in ipairs(ret) do
					ret[i] = tostring(v)
				end

				msg(table.concat(ret, ", "))
			end
		else
			msg(tostring(ret[2]), account, who)
		end
	end

	function easylua.RunLua(code, account, who)
		local env = {
			{"buddy", purple.find.Buddy(account, who)},
			{"who", who},
			{"account", account},
			{"print", function(...)
				local args = {}
				for i = 1, select("#", ...) do
					local val = (select(i, ...))
					if type(val) == "cdata" and tostring(ffi.typeof(val)):find("char") then
						val = purple.chars_to_string(val)
					else
						val = tostring(val)
					end
					table.insert(args, val)
				end
				msg(table.concat(args, ", "), account, who)
			end},
		}

		local locals = "local "
		for i, data in ipairs(env) do
			locals = locals .. data[1]
			if i ~= #env then
				locals = locals .. ", "
			end
		end

		local func = assert(loadstring(locals .. " = ... " .. code, code))

		local vars = {}
		for _, data in ipairs(env) do
			table.insert(vars, data[2])
		end

		return func(unpack(vars))
	end

	purple.signal.Connect("sent-im-msg", function(account, who, message)
		easylua.RunString(purple.markup.StripHtml(message), account, who)

		return false
	end)

	purple.signal.Connect("received-im-msg", function(account, who, message, conv, flags)
		local buddy = purple.find.Buddy(account, who)

		if buddy:GetGroup():GetName() == "lua" then
			easylua.RunString(purple.markup.StripHtml(message), account, who, conv)
		end
	end)
end

_G.purple = purple
_G.ffi = ffi

do
	purple.signal.Connect("received-im-msg", function(account, who, buffer, conv, flags)
		local buddy = purple.find.Buddy(account, who)
		local presence = buddy:GetPresence()

		local msg = {}
		msg.text = purple.markup.StripHtml(buffer)
		msg.remote_username = who
		msg.local_alias = account:GetAlias()
		msg.local_username = account:GetNameForDisplay()
		msg.remote_alias = buddy:GetAlias()
		msg.protocol_id = account:GetProtocolId()

		local group = buddy:GetGroup()

		if group.ptr ~= nil then
			msg.remote_from_group = group:GetName()
		end

		local local_status = account:GetActiveStatus()
		local local_status_type = local_status:GetType()

		msg.local_status_id = purple.primitive.GetIdFromType(local_status_type:GetPrimitive())
		if local_status_type:GetAttr("message") ~= nil then
			msg.local_status_msg = local_status:GetAttrString("message")
		else
			local savedstatus = savedstatus.GetCurrnet()
			if savedstatus.ptr ~= nil then
				msg.local_status_msg = savedstatus:GetMessage()
			end
		end

		local remote_status = presence:GetActiveStatus()
		local remote_status_type = remote_status:GetType()

		msg.remote_status_id = purple.primitive.GetIdFromType(remote_status_type:GetPrimitive())

		if remote_status_type:GetAttr("message") ~= nil then
			msg.local_status_msg = remote_status:GetAttrString("message")
		else
			msg.local_status_msg = nil
		end

		for k,v in pairs(msg) do
			print(k .. " = " .. tostring(v))
		end
	end)
end

do
	purple.signal.Connect("writing-im-msg", function(account, who, buffer, conv, flags)
		if buffer.str == "wow" then
			buffer.str = "bubu: SYSWOW32"
		end
	end)
end