ffibuild is a utility for generating C bindings for luajit using ffi and wrap them safe lua code. Look in examples/*/build.lua for usage.

This is Linux only at the moment but it should in theory work fine on windows with mingw or something.
Structs and enums are just strings at the moment but I will make it possible to get information about them in the future. In the meantime it's possible to generate empty struct types.

A very simple example:

```
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

ffi.GetMetaData returns a table like so

```
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


A function type is structured like this:
```
func_type:GetDeclaration(as_callback) -- gets the function declaration

if func_type.arguments then
	for _, type in ipairs(func_type.arguments) do
		type:GetDeclaration() -- gets the string declaration of the type: gint**
		type:GetEvaluated(meta_data) -- gets the evaluated type
		type:GetEvaluated(meta_data):GetDeclaration(): gboolean* >> int*
		type:Evaluate(meta_data) -- collapses the type to its original type. this is very useful to avoid ffi header conflicts and reduce the size of the header
		type.name -- the name of this argument if any
	end
end

func_type.return_type -- see argument type
```

In the first example you would get glib functions as well since purple uses them externally. This is generally not wanted but there are tools in ffibuild to handle this.

You can use `local top, bottom = ffibuild.SplitHeader(header, "_Purple"})` which would find the first instance of _Purple and legally (syntax wise) split the header there where top would be linux and glib internal functions and bottom would be libpurple only.
You can then call `ffibuild.GetMetaData(bottom)` to get libpurple specific info only and then evaluate the metadata types with `ffibuild.GetMetaData(top)` and then build your own header using :GetDeclaration()
