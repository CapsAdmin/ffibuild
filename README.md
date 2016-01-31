## simple example:

```lua
local header = ffibuild.GetHeader([[
	#define PURPLE_PLUGINS
	#include <libpurple/purple.h>
]], "$(pkg-config purple --cflags)")

local meta_data = ffibuild.GetMetaData(header)

local code = "local CLIB = ffi.load('purple')\n"

for func_name, func_type in pairs(meta_data.functions) do
	local friendly_name = ffibuild.ChangeCase(func_name, "foo_bar", "fooBar")
	code = code .. ffibuild.BuildFunction(friendly_name, func_name, func_type) .. "\n"
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
	typedefs = {gboolean = {[gint], [int]}, gint = {[int]}, ...},
	variables = {[variable], ...},
	enums = {PurpleStatusType = "enum PurpleStatusType{...}", ...},
	global_enums = "enum {...}",
} = ffibuild.GetMetaData(header)
```

Where [???] represents a type object.

## types
```lua
string = type:GetDeclaration() -- Gets the string declaration of the type as a string. such as "gint**"
[type] = type:GetEvaluated(meta_data) -- Gets the evaluated type using meta_data to look it up. Otherwise it returns nil.
nil = type:Evaluate(meta_data) -- Evaluates itself to its original type using meta_data to look it up.
```

## functions
```lua
func_type:GetDeclaration(as_callback) -- gets the function declaration or as a callback if requested. A function cold also be a callback intitially and so GetDeclaration would return that by default.
func_type.callback -- if this is a callback or not

if func_type.arguments then
	for _, type in ipairs(func_type.arguments) do
		-- see type section above
		type.name -- the name of this argument if any
	end
end

func_type.return_type -- the return argument type
```

## trimming the header and evaluating types

In the first example you would get glib functions exported as well since purple uses them internally. This is generally not wanted but there are tools in ffibuild to handle this.

You can use `local top_header, bottom_header = ffibuild.SplitHeader(header, "_Purple")` which would find the first instance of _Purple and  split the header by closest statement where "top_header" is linux and glib internal functions and bottom is libpurple only.

You can then call `local meta_data = ffibuild.GetMetaData(bottom_header)` to get libpurple specific info only and then evaluate types with `local meta_data_internal = ffibuild.GetMetaData(top_header)`

Now it's possible to build your own header using meta_data and type:GetDeclaration()

## todo
Use mingw or visual studio on windows somehow.
Structs and enums are not objects with content info but instead just strings. In the meantime it's possible to generate empty struct types.
