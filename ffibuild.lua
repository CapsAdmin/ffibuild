local ffibuild = {}



function ffibuild.StripHeader(header, meta_data, check_function, empty_structs)
	local required = {}

	local bottom = ""

	for func_name, func_type in pairs(meta_data.functions) do
		if not check_function or check_function(func_name, func_type) then
			func_type:MakePrimitive(meta_data, required)
			bottom = bottom .. func_type:GetDeclaration() .. ";\n"
		end
	end

	local top = ""

	for _, type in ipairs(required) do
		local basic_type = type:GetBasicType()

		if type:GetSubType() == "struct" then
			top = top .. basic_type .. " " .. (empty_structs and "{}" or meta_data.structs[basic_type]) .. ";\n"
		elseif type:GetSubType() == "union" then
			top = top .. basic_type .. " " .. (empty_structs and "{}" or meta_data.unions[basic_type]) .. ";\n"
		elseif type:GetSubType() == "enum" then
			top = top .. "typedef " .. basic_type .. " " .. meta_data.enums[basic_type] .. ";\n"
		end
	end

	if meta_data.global_enums then
		top = meta_data.global_enums .. top
	end

	return top .. bottom
end

function ffibuild.ChangeCase(str, from, to)
	if from == "fooBar" then
		if to == "FooBar" then
			return str:sub(1, 1):upper() .. str:sub(2)
		elseif to == "foo_bar" then
			return ffibuild.ChangeCase(str:sub(1, 1):upper() .. str:sub(2), "FooBar", "foo_bar")
		end
	elseif from == "FooBar" then
		if to == "foo_bar" then
			return str:gsub("(%l)(%u)", function(a, b) return a.."_"..b:lower() end):lower()
		elseif to == "fooBar" then
			return str:sub(1, 1):lower() .. str:sub(2)
		end
	elseif from == "foo_bar" then
		if to == "FooBar" then
			return ("_" .. str):gsub("_(%l)", function(s) return s:upper() end)
		elseif to == "fooBar" then
			return ffibuild.ChangeCase(ffibuild.ChangeCase(str, "foo_bar", "FooBar"), "FooBar", "fooBar")
		end
	end
	return str
end

function ffibuild.BuildFunction(friendly_name, real_name, func_type, call_translate, return_translate, meta_data, first_argument_self, clib)
	clib = clib or "CLIB"

	local parameters, call = func_type:GetParameters(first_argument_self, function(type, name)
		type = type:GetPrimitive(meta_data)

		return
			call_translate and
			call_translate(type:GetDeclaration(), name, type, func_type) or
			name
	end)

	local s = ""
	s = s .. friendly_name .. " = function(" .. parameters .. ") "
	s = s .. "local v = " .. clib .. "." .. real_name .. "(" .. call .. ") "
	local return_type = func_type.return_type:GetPrimitive(meta_data) or func_type.return_type
	local declaration = return_type:GetDeclaration()

	if return_translate then
		local ret, func = return_translate(declaration, return_type, func_type)

		s = s .. (ret or "")

		if func then
			s = func(s)
		end
	end

	s = s .. " return v "
	s = s .. "end"

	return s
end

function ffibuild.FindFunctions(meta_data, pattern, from, to)
	local out = {}
	for func_name, func_type in pairs(meta_data.functions) do
		local capture = func_name:match(pattern)
		if capture then
			if from and to then
				capture = ffibuild.ChangeCase(capture, from, to)
			end
			out[capture] = func_type
		end
	end
	return out
end


function ffibuild.GetStructTypes(meta_data, pattern)
	local out = {}

	-- find all types that start with *pattern* and are also structs
	for type_name, type in pairs(meta_data.typedefs) do
		local name = type_name:match(pattern)
		if name and type:GetSubType() == "struct" then
			table.insert(out, {
				name = name,
				type = type,
			})
		end
	end

	-- sort them by length to avoid functions like purple_>>conversation<<_foo_bar() to conflict with purple_>>conversation_im<<_foo_bar()
	table.sort(out, function(a, b) return #a.name > #b.name end)

	return out
end

function ffibuild.GetFunctionsStartingWithType(meta_data, type)
	local out = {}

	for func_name, func_type in pairs(meta_data.functions) do
		if func_type.arguments then
			local evaluated = func_type.arguments[1]:GetPrimitive(meta_data) or func_type.arguments[1]
			if evaluated:GetBasicType() == type:GetBasicType() then
				out[func_name] = func_type
			end
		end
	end

	return out
end



function ffibuild.GetHeader(c_source, flags)
	flags = flags or ""
	local temp_name = os.tmpname()
	local temp_file = io.open(temp_name, "w")
	temp_file:write(c_source)
	temp_file:close()

	local gcc = io.popen("gcc -xc -E -P " .. flags .. " " .. temp_name)
	local header = gcc:read("*all")

	gcc:close()
	os.remove(temp_name)

	return header
end

function ffibuild.SplitHeader(header, stops)
	header = header:gsub("/%*.-%*/", "")
	header = header:gsub("#.-\n", "")

	local found = {}

	for _, what in pairs(stops) do
		local _, stop_pos = header:find(".-" .. what)

		if stop_pos then
			stop_pos = stop_pos - #what
		end
		table.insert(found, stop_pos)
	end

	table.sort(found, function(a, b) return a < b end)

	local stop = found[1]

	for i = 1, math.huge do
		local char = header:sub(stop - i, stop - i)
		if char == ";" or char == "}" then
			stop = stop - i + 1
			break
		end
	end

	return header:sub(0, stop), header:sub(stop)
end

do -- type metatables
	local metatables = {}

	for _, name in ipairs({"function", "type", "var_arg"}) do
		local META = {}
		META.__index = META
		META.MetaType = name

		--META.__tostring = function(s) return ("%s[%s]"):format(s:GetDeclaration(), name) end

		function META:GetDeclaration() end
		function META:GetPrimitive() return self end
		function META:MakePrimitive() end
		function META:GetCopy() end
		function META:GetBasicType() end
		function META:GetSubType() return self:GetBasicType() end

		metatables[name] = META
	end

	for name, meta in pairs(metatables) do
		for other_name in pairs(metatables) do
			meta["Is" .. ffibuild.ChangeCase(other_name, "foo_bar", "FooBar")] = function(self) return self.MetaName == name end
		end
	end

	function ffibuild.CreateType(declaration, type, ...)
		return setmetatable(metatables[type]:Create(declaration, ...), metatables[type])
	end

	do -- type
		local TYPE = metatables.type

		local flags = {
			const = true,
			volatile = true,
			struct = true,
			enum = true,
			union = true,
			unsigned = true,
			signed = true,
			pointer = true,
		}

		function TYPE:Create(declaration)
			local tree = {}

			local node = tree

			local prev_token
			local prev_node

			for token in (declaration:reverse() .. " "):gmatch("(%S+) ") do
				token = token:reverse()

				if token == "*" then
					token = "pointer"
				end

				if flags[token] then
					node[token] = true
				else
					node.type = token
				end

				if token == "struct" or token == "union" or token == "enum" then
					prev_node[prev_token] = nil
					node.type = token .. " " .. prev_token
				end

				if token == "pointer" then
					prev_node = node
					node.to = {}
					node = node.to
				end

				prev_token = token
				prev_node = node
			end

			return {
				last_node = node,
				tree = tree,
			}
		end

		local flags = {
			"signed",
			"unsigned",
			"const",
			"volatile",
		}

		function TYPE:GetDeclaration(meta_data)
			local node = self:GetPrimitive(meta_data).tree

			local declaration = {}

			while true do
				if node.type then
					table.insert(declaration, node.type:reverse())
				end

				for _, flag in ipairs(flags) do
					if node[flag] then
						table.insert(declaration, flag:reverse())
					end
				end

				if node.pointer then
					table.insert(declaration, "*")
					node = node.to
				else
					break
				end
			end

			return table.concat(declaration, " "):reverse()
		end

		function TYPE:GetBasicType()
			return self.last_node.type
		end

		function TYPE:GetSubType()
			if self.last_node.struct then return "struct" end
			if self.last_node.enum then return "enum" end
			if self.last_node.union then return "union" end

			return self:GetBasicType()
		end

		function TYPE:GetCopy()
			return ffibuild.CreateType(self:GetDeclaration(), "type")
		end

		function TYPE:GetPrimitive(meta_data)
			if not meta_data or not meta_data.typedefs[self:GetBasicType()] then return self end

			local copy = self:GetCopy()
			local type = copy

			for i = 1, 10 do
				type = meta_data.typedefs[type:GetBasicType()]

				if not type then break end

				local type = type:GetCopy()

				if getmetatable(type) ~= getmetatable(copy) then
					for k,v in pairs(copy) do copy[k] = nil end
					for k,v in pairs(type) do copy[k] = v end

					setmetatable(copy, getmetatable(type))

					return copy:GetPrimitive(meta_data)
				elseif type.tree then
					copy.last_node.type = nil
					for k, v in pairs(type.tree) do
						copy.last_node[k] = v
					end
					if type.last_node ~= type.tree then
						copy.last_node = type.last_node
					end
				else
					break
				end
			end

			return copy
		end

		function TYPE:MakePrimitive(meta_data, out)
			local new_type = self:GetPrimitive(meta_data)

			if new_type and new_type ~= self then
				for k,v in pairs(self) do self[k] = nil end
				for k,v in pairs(new_type) do self[k] = v end

				if getmetatable(new_type) ~= getmetatable(self) then
					setmetatable(self, getmetatable(new_type))
					self:MakePrimitive(meta_data, out)
				end

				if out then
					for i, v in ipairs(out) do
						if v:GetBasicType() == self:GetBasicType() then
							return
						end
					end
					table.insert(out, 1, self)
				end
			end
		end
	end

	do -- function
		local FUNCTION = metatables["function"]

		local basic_types = {
			["char"] = true,
			["signed char"] = true,
			["unsigned char"] = true,
			["short"] = true,
			["short int"] = true,
			["signed short"] = true,
			["signed short int"] = true,
			["unsigned short"] = true,
			["unsigned short int "] = true,
			["int"] = true,
			["signed"] = true,
			["signed int"] = true,
			["unsigned"] = true,
			["unsigned int"] = true,
			["long"] = true,
			["long int"] = true,
			["signed long"] = true,
			["signed long int"] = true,
			["unsigned long"] = true,
			["unsigned long int"] = true,
			["long long"] = true,
			["long long int"] = true,
			["signed long long"] = true,
			["signed long long int"] = true,
			["unsigned long long"] = true,
			["unsigned long long int"] = true,
			["float"] = true,
			["double"] = true,
			["long double"] = true,
		}

		local function explode(str, split)

			local temp = {}
			str = str:gsub("(%b())", function(str) table.insert(temp, str) return "___TEMP___" end)

			local out = {}

			for val in (str .. split):gmatch("(.-)"..split) do
				if val:find("___TEMP___", nil, true) then val = val:gsub("___TEMP___", function() return table.remove(temp, 1) end) end
				table.insert(out, val)
			end

			return out
		end

		function FUNCTION:Create(declaration, is_callback)
			local return_line, func_name, arg_line, func_type

			if is_callback then
				return_line, func_type, func_name, arg_line = declaration:match("^(.-)%( (%*.*) ([%a%d_]+) %) (%b())$")
			else
				return_line, func_name, arg_line = declaration:match("(.-) ([%a%d_]+) (%b())")
			end

			arg_line = arg_line:sub(3, -3)

			local arguments

			if #arg_line > 0 then
				arguments = explode(arg_line, " , ")
				for i, arg in ipairs(arguments) do
					local type

					if arg == "..." then
						type = ffibuild.CreateType(arg, "var_arg")
					elseif basic_types[arg] then
						type = ffibuild.CreateType(arg, "type")
					elseif arg:find("%b() %b()") then
						type = ffibuild.CreateType(arg, "function", true)
					else
						local declaration, name = arg:match("^([%a%d%s_%*]-) ([%a%d_]-)$")

						if not declaration then
							declaration = arg
							name = "unknown"
						end

						type = ffibuild.CreateType(declaration, "type")
						type.name = name
					end

					arguments[i] = type
				end
			end

			return {
				name = func_name,
				arguments = arguments,
				return_type = ffibuild.CreateType(return_line, "type"),
				callback = is_callback,
				func_type = func_type,
			}
		end

		function FUNCTION:GetCopy()
			-- TODO: this doesn't work because i :Create does not handle callbacks that return callbacks
			--return ffibuild.CreateType(self:GetDeclaration(), "function", self.callback)
			local copy = {}

			for k,v in pairs(self) do
				if k == "return_type" then
					copy[k] = v:GetCopy()
				elseif k == "arguments" then
					local new_arguments = {}
					for i,v in ipairs(v) do
						new_arguments[i] = v:GetCopy()
					end
					copy[k] = new_arguments
				else
					copy[k] = v
				end
			end

			return setmetatable(copy, FUNCTION)
		end

		function FUNCTION:GetBasicType()
			return self.callback and "callback" or "function"
		end

		function FUNCTION:GetSubType()
			return self:GetBasicType()
		end

		function FUNCTION:GetDeclaration(meta_data, as_callback)

			local arg_line = {}

			if self.arguments then
				for i, arg in ipairs(self.arguments) do
					arg_line[i] = arg:GetDeclaration(meta_data)
				end
			end

			arg_line = "( " .. table.concat(arg_line, " , ") .. " )"

			if self.return_type.callback then
				local ret, urn = self.return_type:GetDeclaration():match("^(.-%( %*) (.+)")
				local res = ret .. " " .. self.name .. " " .. arg_line .. " " .. urn
				if res:find("PurpleFilterAccountFunc") then print(res) end
				return res
			end

			if self.callback or as_callback == true then
				return self.return_type:GetDeclaration(meta_data) .. " ( " .. (self.func_type or "*") .. " ) " .. arg_line
			else
				return self.return_type:GetDeclaration(meta_data)  .. " " .. self.name .. " " .. arg_line
			end
		end

		function FUNCTION:MakePrimitive(...)
			self.return_type:MakePrimitive(...)
			if self.arguments then
				for i, arg in ipairs(self.arguments) do
					arg:MakePrimitive(...)
				end
			end
		end

		local keywords = {
			["and"] = true,
			["break"] = true,
			["do"] = true,
			["else"] = true,
			["elseif"] = true,
			["end"] = true,
			["false"] = true,
			["for"] = true,
			["function"] = true,
			["if"] = true,
			["in"] = true,
			["local"] = true,
			["nil"] = true,
			["not"] = true,
			["or"] = true,
			["repeat"] = true,
			["return"] = true,
			["then"] = true,
			["true"] = true,
			["until"] = true,
			["while"] = true,
		}

		function FUNCTION:GetParameters(meta, check)
			if not self.arguments then return "", "" end

			local done = {}

			local parameters = {}
			local call = {}

			for i, arg in ipairs(self.arguments) do
				local name = arg.name or "_" .. i

				if keywords[name] then
					name = name .. "_"
				end

				if meta and i == 1 then
					name = "self"
				end

				do -- fixes argument names that are the same
					if done[name] then
						name = name .. done[name]
					end

					done[name] = (done[name] or 0) + 1
				end

				table.insert(parameters, name)

				local res = check(arg, name)

				if res then
					table.insert(call, res)
				end
			end

			return table.concat(parameters, ", "), table.concat(call, ", ")
		end
	end

	do -- var arg
		local VARARG = metatables.var_arg

		function VARARG:Create()
			return {}
		end

		function VARARG:GetDeclaration()
			return "..."
		end

		function VARARG:GetCopy()
			return VARARG:Create()
		end

		function VARARG:GetBasicType()
			return "var_arg"
		end
	end
end

function ffibuild.GetMetaData(header)

	local out = {
		functions = {},
		structs = {},
		unions = {},
		typedefs = {},
		variables = {},
		enums = {},
		global_enums = {},
	}

	do -- cleanup header
		-- this assumes the header has been preprocessed with gcc -E -P
		header = " " .. header

		-- remove comments
		header = header:gsub("/%*.-%*/", "")

		-- remove things like #pragma
		header = header:gsub("#.-\n", "")

		 -- normalize everything to have equal spacing even between punctation
		header = header:gsub("([*%(%){}&%[%],;])", " %1 ")
		header = header:gsub("%s+", " ")

		-- insert a newline after ;
		header = header:gsub(";", ";\n")

		-- this will explode structs and and whatnot so make sure we remove newlines inside {} and ()
		header = header:gsub("%b{}", function(s) return s:gsub("%s+", " ") end)
		header = header:gsub("%b()", function(s) return s:gsub("%s+", " ") end)

		-- remove compiler __attribute__
		header = header:gsub("__%a-__ %b() ", "")

		-- remove __extension__
		header = header:gsub("__extension__ ", "")

		-- remove __restrict
		header = header:gsub("__restrict__ ", "")
		header = header:gsub("__restrict", "")

		-- remove volatile
		header = header:gsub(" volatile ", " ")

		-- remove inline functions
		header = header:gsub(" static __inline.-%b().-%b{}", "")
		header = header:gsub(" static inline.-%b().-%b{}", "")

		-- int foo(void); >> int foo();
		header = header:gsub(" %( void %) ", " ( ) ")

		-- foo bar[] > foo *bar
		header = header:gsub(" ([%a%d_]+) %b[] ", " * %1 ")
	end

	for line in header:gmatch(" (.-);\n") do

		if line:find("^typedef") then
			if line:find("%b() %b()") and not line:find("%b{}") then
				local type = ffibuild.CreateType(line:match("^typedef (.+) $"), "function", true)
				out.typedefs[type.name] = type
				line = nil
			else
				local content, alias = line:match("^typedef (.+) ([%a%d_]+)")

				if content:find("%b{}") then
					if content:find("^[%a%d_]+ [%a%d_]+ ({.+})") then
						line = content
					else
						line = content:gsub("^([%a%d_]+)", function(start) return start .. " " .. alias end)
					end

					local content = line:match("^([%a%d_]+ [%a%d_]+)")
					out.typedefs[alias] = ffibuild.CreateType(content, "type")

				elseif line:find("%b()") then
					content = line:match("^typedef (.+) $")
					alias = content:match(".- ([%a%d_]+) %b()")

					out.typedefs[alias] = ffibuild.CreateType(content, "function")
				else
					if line:find("^struct") or line:find("^union") then
						line = content
					else
						line = nil
					end

					out.typedefs[alias] = ffibuild.CreateType(content, "type")
				end
			end
		end

		if line then
			if line:find("^[%a%d%s_]+.-[%a%d_]+ %b() $") then
				-- uh
				if line:find("^extern") then
					line = line:sub(#"extern"+2)
				end

				local type = ffibuild.CreateType(line, "function")

				out.functions[type.name] = type

			elseif line:find("^enum") then
				local tag, content = line:match("(enum [%a%d_]+) ({.+})")

				if tag then
					out.enums[tag] = content
				else
					-- no type name = global enum
					table.insert(out.global_enums, line:match("enum ({.+})"))
				end
			elseif line:find("^struct") or line:find("^union") then
				local keyword = line:match("^([%a%d_]+)")
				local out = keyword == "struct" and out.structs or out.unions

				local tag, content = line:match("("..keyword.." [%a%d_]+) ({.+)") -- some structs can end with "*"

				if not tag then
					-- just a forward declaration or an opaque struct
					tag = line:match("("..keyword.." [%a%d_]+)")
					content = "{}"
				end

				out[tag] = content
			elseif line:find("^extern") then
				if line:find("%b() %b()") then
					local type = ffibuild.CreateType(line:match("extern (.+) $"), "function", true)
					out.variables[type.name] = type
				else
					local tag, name = line:match("^extern (.+) ([%a%d_]+) $")

					out.variables[name] = ffibuild.CreateType(tag, "type")
				end
			elseif line:find("^static") then

			else
				error(line, "?")
			end
		end
	end

	do -- global enums
		local enums

		if #out.global_enums > 0 then
			enums = "enum {\n"
			for i, v in ipairs(out.global_enums) do
				enums = enums .. v:match("{( .+ )}")
				if i ~= #out.global_enums then
					enums = enums .. ","
				end
				enums = enums .. "\n"
			end
			enums = enums  .. "};\n"
		end

		out.global_enums = enums
	end

	return out
end

return ffibuild