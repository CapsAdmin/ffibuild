local ffi = require("ffi")
ffi.cdef([[struct graphene_simd4x4f_t {float x;float y;float z;float w;};
float(graphene_simd4f_max)(const float,const float);
_Bool(graphene_simd4f_cmp_neq)(const float,const float);
_Bool(graphene_simd4f_cmp_eq)(const float,const float);
float(graphene_simd4f_merge_low)(const float,const float);
float(graphene_simd4f_splat)(float);
float(graphene_simd4f_get_y)(const float);
float(graphene_simd4f_sqrt)(const float);
float(graphene_simd4f_reciprocal)(const float);
float(graphene_simd4f_dot3_scalar)(const float,const float);
float(graphene_simd4f_init_2f)(const float*);
float(graphene_simd4f_mul)(const float,const float);
float(graphene_simd4f_cross3)(const float,const float);
_Bool(graphene_simd4f_cmp_lt)(const float,const float);
float(graphene_simd4f_div)(const float,const float);
float(graphene_simd4f_dot3)(const float,const float);
float(graphene_simd4f_neg)(const float);
float(graphene_simd4f_flip_sign_1010)(const float);
_Bool(graphene_simd4f_cmp_ge)(const float,const float);
_Bool(graphene_simd4f_cmp_le)(const float,const float);
void(graphene_simd4x4f_transpose_in_place)(struct graphene_simd4x4f_t*);
float(graphene_simd4f_flip_sign_0101)(const float);
float(graphene_simd4f_merge_w)(const float,float);
float(graphene_simd4f_zero_zw)(const float);
float(graphene_simd4f_zero_w)(const float);
float(graphene_simd4f_shuffle_yzwx)(const float);
float(graphene_simd4f_rsqrt)(const float);
float(graphene_simd4f_shuffle_zwxy)(const float);
float(graphene_simd4f_get)(const float,unsigned int);
float(graphene_simd4f_shuffle_wxyz)(const float);
float(graphene_simd4f_sub)(const float,const float);
float(graphene_simd4f_get_w)(const float);
float(graphene_simd4f_add)(const float,const float);
float(graphene_simd4f_splat_z)(const float);
float(graphene_simd4f_splat_y)(const float);
_Bool(graphene_simd4f_cmp_gt)(const float,const float);
float(graphene_simd4f_get_z)(const float);
void(graphene_simd4f_dup_2f)(const float,float*);
void(graphene_simd4f_dup_3f)(const float,float*);
void(graphene_simd4f_dup_4f)(const float,float*);
float(graphene_simd4f_init_4f)(const float*);
float(graphene_simd4f_init_zero)();
float(graphene_simd4f_init)(float,float,float,float);
float(graphene_simd4f_merge_high)(const float,const float);
float(graphene_simd4f_init_3f)(const float*);
float(graphene_simd4f_splat_w)(const float);
float(graphene_simd4f_min)(const float,const float);
float(graphene_simd4f_get_x)(const float);
float(graphene_simd4f_splat_x)(const float);
]])
local CLIB = ffi.load(_G.FFI_LIB or "graphene")
local library = {}


--====helper metatables====
	local metatables = {}
	local object_cache = {}

	local function wrap_pointer(ptr, meta_name)
		-- TODO
		-- you should be able to use cdata as key and it would use the address
		-- but apparently that doesn't work
		local id = tostring(ptr)

		if not object_cache[meta_name] then
			object_cache[meta_name] = setmetatable({}, {__mode = "v"})
		end

		if not object_cache[meta_name][id] then
			object_cache[meta_name][id] = setmetatable({ptr = ptr}, metatables[meta_name])
		end

		return object_cache[meta_name][id]
	end
--====helper metatables====

library = {
	Simd4fMax = CLIB.graphene_simd4f_max,
	Simd4fCmpNeq = CLIB.graphene_simd4f_cmp_neq,
	Simd4fCmpEq = CLIB.graphene_simd4f_cmp_eq,
	Simd4fMergeLow = CLIB.graphene_simd4f_merge_low,
	Simd4fSplat = CLIB.graphene_simd4f_splat,
	Simd4fGetY = CLIB.graphene_simd4f_get_y,
	Simd4fSqrt = CLIB.graphene_simd4f_sqrt,
	Simd4fReciprocal = CLIB.graphene_simd4f_reciprocal,
	Simd4fDot3Scalar = CLIB.graphene_simd4f_dot3_scalar,
	Simd4fInit_2f = CLIB.graphene_simd4f_init_2f,
	Simd4fMul = CLIB.graphene_simd4f_mul,
	Simd4fCross3 = CLIB.graphene_simd4f_cross3,
	Simd4fCmpLt = CLIB.graphene_simd4f_cmp_lt,
	Simd4fDiv = CLIB.graphene_simd4f_div,
	Simd4fDot3 = CLIB.graphene_simd4f_dot3,
	Simd4fNeg = CLIB.graphene_simd4f_neg,
	Simd4fFlipSign_1010 = CLIB.graphene_simd4f_flip_sign_1010,
	Simd4fCmpGe = CLIB.graphene_simd4f_cmp_ge,
	Simd4fCmpLe = CLIB.graphene_simd4f_cmp_le,
	Simd4x4fTransposeInPlace = CLIB.graphene_simd4x4f_transpose_in_place,
	Simd4fFlipSign_0101 = CLIB.graphene_simd4f_flip_sign_0101,
	Simd4fMergeW = CLIB.graphene_simd4f_merge_w,
	Simd4fZeroZw = CLIB.graphene_simd4f_zero_zw,
	Simd4fZeroW = CLIB.graphene_simd4f_zero_w,
	Simd4fShuffleYzwx = CLIB.graphene_simd4f_shuffle_yzwx,
	Simd4fRsqrt = CLIB.graphene_simd4f_rsqrt,
	Simd4fShuffleZwxy = CLIB.graphene_simd4f_shuffle_zwxy,
	Simd4fGet = CLIB.graphene_simd4f_get,
	Simd4fShuffleWxyz = CLIB.graphene_simd4f_shuffle_wxyz,
	Simd4fSub = CLIB.graphene_simd4f_sub,
	Simd4fGetW = CLIB.graphene_simd4f_get_w,
	Simd4fAdd = CLIB.graphene_simd4f_add,
	Simd4fSplatZ = CLIB.graphene_simd4f_splat_z,
	Simd4fSplatY = CLIB.graphene_simd4f_splat_y,
	Simd4fCmpGt = CLIB.graphene_simd4f_cmp_gt,
	Simd4fGetZ = CLIB.graphene_simd4f_get_z,
	Simd4fDup_2f = CLIB.graphene_simd4f_dup_2f,
	Simd4fDup_3f = CLIB.graphene_simd4f_dup_3f,
	Simd4fDup_4f = CLIB.graphene_simd4f_dup_4f,
	Simd4fInit_4f = CLIB.graphene_simd4f_init_4f,
	Simd4fInitZero = CLIB.graphene_simd4f_init_zero,
	Simd4fInit = CLIB.graphene_simd4f_init,
	Simd4fMergeHigh = CLIB.graphene_simd4f_merge_high,
	Simd4fInit_3f = CLIB.graphene_simd4f_init_3f,
	Simd4fSplatW = CLIB.graphene_simd4f_splat_w,
	Simd4fMin = CLIB.graphene_simd4f_min,
	Simd4fGetX = CLIB.graphene_simd4f_get_x,
	Simd4fSplatX = CLIB.graphene_simd4f_splat_x,
}
library.e = {
}
library.clib = CLIB
return library
