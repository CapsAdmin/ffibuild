local ffi = require("ffi")
ffi.cdef([[]])
local CLIB = ffi.load(_G.FFI_LIB or "sndfile")
local library = {}
library = {
}
library.e = {
}
library.clib = CLIB
return library
