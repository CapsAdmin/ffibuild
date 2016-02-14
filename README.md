## simple example:

```lua
local header = ffibuild.BuildCHeader([[
	#define PURPLE_PLUGINS
	#include <libpurple/purple.h>
]], "$(pkg-config purple --cflags)")

local meta_data = ffibuild.GetMetaData(header)

local code = "local CLIB = ffi.load('purple')\n"

for func_name, func_type in pairs(meta_data.functions) do
	local friendly_name = ffibuild.ChangeCase(func_name, "foo_bar", "fooBar")
	code = code .. ffibuild.BuildLuaFunction(friendly_name, func_name, func_type) .. "\n"
end

io.write(code)
```

This creates a bunch of globals from this_casing to thisCasing based header input.




## ffi.GetMetaData(header)
ffi.GetMetaData returns a table structured like so

```lua
{
	functions = {[function], ...},
	structs = {struct _PurpleAccount = [struct], struct _PurpleBuddy = [struct]},
	unions = {union _PurpleAccount = [union], ...},
	typedefs = {gboolean = [type], gint = [type], ...},
	variables = {[variable], ...},
	enums = {[enums], [enums], [enums]},
	global_enums = {[enums], [enums], [enums]},
} = ffibuild.GetMetaData(header)
```

Where [???] represents a type object.

## types
```lua
string = type:GetDeclaration() -- Gets the declaration for the type such as "const char *", "void (*)(int, char)", "enums {FOO=1,BAR=2}", etc
string = type:GetBasicType(meta_data) -- Gets the basic type such as if type:GetDeclaration() would return "const char *" type:GetbasicType() would return "char"
[type] = type:GetPrimitive(meta_data) -- Attempts to get the primitive type using meta_data. It returns itself otherwise.
```

## functions
```lua
func_type:GetDeclaration(as_callback) -- gets the function declaration or as a callback if requested. A function cold also be a callback intitially and so GetDeclaration would return that by default.

if func_type.arguments then
	for arg_pos, type in ipairs(func_type.arguments) do
		-- see type section above
		type.name -- the name of this argument if any
	end
end

func_type.return_type -- the return argument type
```

## trimming the header and evaluating types

In the first example you would get glib functions exported as well since purple uses them internally. This is generally not wanted but there are tools in ffibuild to handle this.

You can use `local top_header, bottom_header = ffibuild.SplitHeader(header, "_Purple")` which would find the first instance of _Purple and split the header by closest statement where `top_header` in this case is linux and glib internal functions and `bottom_header` is libpurple only.

You can then call `local meta_data = ffibuild.GetMetaData(bottom_header)` to get libpurple specific meta data only and then evaluate types with `local meta_data_internal = ffibuild.GetMetaData(top_header)` to get the actual type.

Now it's possible to build your own header using `local header = ffibuild.BuildMinimalHeader(meta_data_internal, <check_function, check_enum, empty_structs>)` or `type:GetPrimitive(meta_data)` along with `type:GetDeclaration()`

## todo
Use mingw or visual studio on windows somehow.
Structs sort of work but the way I figure out what structs to include based on function arguments don't. However empty structs will work fine in the meantime.