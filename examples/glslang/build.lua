local ffibuild = dofile("../../ffibuild.lua")

local header = ffibuild.BuildCHeader([[

#ifdef _WIN32
#define C_DECL __cdecl
//#ifdef SH_EXPORTING
//    #define SH_IMPORT_EXPORT __declspec(dllexport)
//#else
//    #define SH_IMPORT_EXPORT __declspec(dllimport)
//#endif
#define SH_IMPORT_EXPORT
#else
#define SH_IMPORT_EXPORT
#ifndef __fastcall
#define __fastcall
#endif
#define C_DECL
#endif


struct TLimits {
    int nonInductiveForLoops;
    int whileLoops;
    int doWhileLoops;
    int generalUniformIndexing;
    int generalAttributeMatrixVectorIndexing;
    int generalVaryingIndexing;
    int generalSamplerIndexing;
    int generalVariableIndexing;
    int generalConstantMatrixVectorIndexing;
};

struct TBuiltInResource {
    int maxLights;
    int maxClipPlanes;
    int maxTextureUnits;
    int maxTextureCoords;
    int maxVertexAttribs;
    int maxVertexUniformComponents;
    int maxVaryingFloats;
    int maxVertexTextureImageUnits;
    int maxCombinedTextureImageUnits;
    int maxTextureImageUnits;
    int maxFragmentUniformComponents;
    int maxDrawBuffers;
    int maxVertexUniformVectors;
    int maxVaryingVectors;
    int maxFragmentUniformVectors;
    int maxVertexOutputVectors;
    int maxFragmentInputVectors;
    int minProgramTexelOffset;
    int maxProgramTexelOffset;
    int maxClipDistances;
    int maxComputeWorkGroupCountX;
    int maxComputeWorkGroupCountY;
    int maxComputeWorkGroupCountZ;
    int maxComputeWorkGroupSizeX;
    int maxComputeWorkGroupSizeY;
    int maxComputeWorkGroupSizeZ;
    int maxComputeUniformComponents;
    int maxComputeTextureImageUnits;
    int maxComputeImageUniforms;
    int maxComputeAtomicCounters;
    int maxComputeAtomicCounterBuffers;
    int maxVaryingComponents;
    int maxVertexOutputComponents;
    int maxGeometryInputComponents;
    int maxGeometryOutputComponents;
    int maxFragmentInputComponents;
    int maxImageUnits;
    int maxCombinedImageUnitsAndFragmentOutputs;
    int maxCombinedShaderOutputResources;
    int maxImageSamples;
    int maxVertexImageUniforms;
    int maxTessControlImageUniforms;
    int maxTessEvaluationImageUniforms;
    int maxGeometryImageUniforms;
    int maxFragmentImageUniforms;
    int maxCombinedImageUniforms;
    int maxGeometryTextureImageUnits;
    int maxGeometryOutputVertices;
    int maxGeometryTotalOutputComponents;
    int maxGeometryUniformComponents;
    int maxGeometryVaryingComponents;
    int maxTessControlInputComponents;
    int maxTessControlOutputComponents;
    int maxTessControlTextureImageUnits;
    int maxTessControlUniformComponents;
    int maxTessControlTotalOutputComponents;
    int maxTessEvaluationInputComponents;
    int maxTessEvaluationOutputComponents;
    int maxTessEvaluationTextureImageUnits;
    int maxTessEvaluationUniformComponents;
    int maxTessPatchComponents;
    int maxPatchVertices;
    int maxTessGenLevel;
    int maxViewports;
    int maxVertexAtomicCounters;
    int maxTessControlAtomicCounters;
    int maxTessEvaluationAtomicCounters;
    int maxGeometryAtomicCounters;
    int maxFragmentAtomicCounters;
    int maxCombinedAtomicCounters;
    int maxAtomicCounterBindings;
    int maxVertexAtomicCounterBuffers;
    int maxTessControlAtomicCounterBuffers;
    int maxTessEvaluationAtomicCounterBuffers;
    int maxGeometryAtomicCounterBuffers;
    int maxFragmentAtomicCounterBuffers;
    int maxCombinedAtomicCounterBuffers;
    int maxAtomicCounterBufferSize;
    int maxTransformFeedbackBuffers;
    int maxTransformFeedbackInterleavedComponents;
    int maxCullDistances;
    int maxCombinedClipAndCullDistances;
    int maxSamples;

    struct TLimits limits;
};



//
// Driver must call this first, once, before doing any other
// compiler/linker operations.
//
// (Call once per process, not once per thread.)
//
SH_IMPORT_EXPORT int ShInitialize();

//
// Driver should call this at process shutdown.
//
SH_IMPORT_EXPORT int __fastcall ShFinalize();

//
// Types of languages the compiler can consume.
//
typedef enum {
    EShLangVertex,
    EShLangTessControl,
    EShLangTessEvaluation,
    EShLangGeometry,
    EShLangFragment,
    EShLangCompute,
    EShLangCount,
} EShLanguage;

typedef enum {
    EShLangVertexMask         = (1 << EShLangVertex),
    EShLangTessControlMask    = (1 << EShLangTessControl),
    EShLangTessEvaluationMask = (1 << EShLangTessEvaluation),
    EShLangGeometryMask       = (1 << EShLangGeometry),
    EShLangFragmentMask       = (1 << EShLangFragment),
    EShLangComputeMask        = (1 << EShLangCompute),
} EShLanguageMask;

const char* StageName(EShLanguage);

//
// Types of output the linker will create.
//
typedef enum {
    EShExVertexFragment,
    EShExFragment
} EShExecutable;

//
// Optimization level for the compiler.
//
typedef enum {
    EShOptNoGeneration,
    EShOptNone,
    EShOptSimple,       // Optimizations that can be done quickly
    EShOptFull,         // Optimizations that will take more time
} EShOptimizationLevel;

//
// Message choices for what errors and warnings are given.
//
typedef enum {
    EShMsgDefault          = 0,         // default is to give all required errors and extra warnings
    EShMsgRelaxedErrors    = (1 << 0),  // be liberal in accepting input
    EShMsgSuppressWarnings = (1 << 1),  // suppress all warnings, except those required by the specification
    EShMsgAST              = (1 << 2),  // print the AST intermediate representation
    EShMsgSpvRules         = (1 << 3),  // issue messages for SPIR-V generation
    EShMsgVulkanRules      = (1 << 4),  // issue messages for Vulkan-requirements of GLSL for SPIR-V
    EShMsgOnlyPreprocessor = (1 << 5),  // only print out errors produced by the preprocessor
} EShMessages;

//
// Build a table for bindings.  This can be used for locating
// attributes, uniforms, globals, etc., as needed.
//
typedef struct {
    const char* name;
    int binding;
} ShBinding;

typedef struct {
    int numBindings;
    ShBinding* bindings;  // array of bindings
} ShBindingTable;

typedef void* ShHandle;

SH_IMPORT_EXPORT ShHandle ShConstructCompiler(const EShLanguage, int debugOptions);  // one per shader
SH_IMPORT_EXPORT ShHandle ShConstructLinker(const EShExecutable, int debugOptions);  // one per shader pair
SH_IMPORT_EXPORT ShHandle ShConstructUniformMap();                 // one per uniform namespace (currently entire program object)
SH_IMPORT_EXPORT void ShDestruct(ShHandle);
SH_IMPORT_EXPORT int ShCompile(
    const ShHandle,
    const char* const * shaderStrings,
    const int numStrings,
    const int* lengths,
    const EShOptimizationLevel,
    const struct TBuiltInResource *resources,
    int debugOptions,
    int defaultVersion,            // use 100 for ES environment, overridden by #version in shader
    int forwardCompatible,      // give errors for use of deprecated features
    EShMessages messages // warnings and errors
);

SH_IMPORT_EXPORT int ShLink(
    const ShHandle,               // linker object
    const ShHandle * h,           // compiler objects to link together
    const int numHandles,
    ShHandle uniformMap,          // updated with new uniforms
    short int** uniformsAccessed,  // returned with indexes of uniforms accessed
    int* numUniformsAccessed);

SH_IMPORT_EXPORT int ShLinkExt(const ShHandle, const ShHandle *h, const int numHandles);
SH_IMPORT_EXPORT void ShSetEncryptionMethod(ShHandle);
SH_IMPORT_EXPORT const char* ShGetInfoLog(const ShHandle);
SH_IMPORT_EXPORT const void* ShGetExecutable(const ShHandle);
SH_IMPORT_EXPORT int ShSetVirtualAttributeBindings(const ShHandle, const ShBindingTable*);   // to detect user aliasing
SH_IMPORT_EXPORT int ShSetFixedAttributeBindings(const ShHandle, const ShBindingTable*);     // to force any physical mappings
SH_IMPORT_EXPORT int ShGetPhysicalAttributeBindings(const ShHandle, const ShBindingTable**); // for all attributes
SH_IMPORT_EXPORT int ShExcludeAttributes(const ShHandle, int *attributes, int count);
SH_IMPORT_EXPORT int ShGetUniformLocation(const ShHandle uniformMap, const char* name);

]])

local meta_data = ffibuild.GetMetaData(header)
local header = meta_data:BuildMinimalHeader(function(name) return name:find("^Sh") end, function(name) return name:find("^ESh") end, true, true)

local lua = ffibuild.BuildGenericLua(header, "glslang", "safe_clib_index")

do
	lua = lua .. "library = {\n"

	for func_name, func_type in pairs(meta_data.functions) do
		if func_name:find("^Sh") then
			local friendly_name = func_name:match("Sh(.+)")
			lua = lua .. "\t" .. ffibuild.BuildLuaFunction(friendly_name, func_type.name, func_type, nil, nil, meta_data, false, "SAFE_INDEX(CLIB)") .. ",\n"
		end
	end

	lua = lua .. "}\n"
end

do -- enums
	lua = lua .. "library.e = {\n"
	for basic_type, type in pairs(meta_data.enums) do
		for i, enum in ipairs(type.enums) do
			lua =  lua .. "\t" .. enum.key:match("^ESh(.+)") .. " = ffi.cast(\""..basic_type.."\", \""..enum.key.."\"),\n"
		end
	end
	lua = lua .. "}\n"
end


lua = lua .. [[
library.default_limits = {
	maxLights = 32,
	maxClipPlanes = 6,
	maxTextureUnits = 32,
	maxTextureCoords = 32,
	maxVertexAttribs = 64,
	maxVertexUniformComponents = 4096,
	maxVaryingFloats = 64,
	maxVertexTextureImageUnits = 32,
	maxCombinedTextureImageUnits = 80,
	maxTextureImageUnits = 32,
	maxFragmentUniformComponents = 4096,
	maxDrawBuffers = 32,
	maxVertexUniformVectors = 128,
	maxVaryingVectors = 8,
	maxFragmentUniformVectors = 16,
	maxVertexOutputVectors = 16,
	maxFragmentInputVectors = 15,
	minProgramTexelOffset = -8,
	maxProgramTexelOffset = 7,
	maxClipDistances = 8,
	maxComputeWorkGroupCountX = 65535,
	maxComputeWorkGroupCountY = 65535,
	maxComputeWorkGroupCountZ = 65535,
	maxComputeWorkGroupSizeX = 1024,
	maxComputeWorkGroupSizeY = 1024,
	maxComputeWorkGroupSizeZ = 64,
	maxComputeUniformComponents = 1024,
	maxComputeTextureImageUnits = 16,
	maxComputeImageUniforms = 8,
	maxComputeAtomicCounters = 8,
	maxComputeAtomicCounterBuffers = 1,
	maxVaryingComponents = 60,
	maxVertexOutputComponents = 64,
	maxGeometryInputComponents = 64,
	maxGeometryOutputComponents = 128,
	maxFragmentInputComponents = 128,
	maxImageUnits = 8,
	maxCombinedImageUnitsAndFragmentOutputs = 8,
	maxCombinedShaderOutputResources = 8,
	maxImageSamples = 0,
	maxVertexImageUniforms = 0,
	maxTessControlImageUniforms = 0,
	maxTessEvaluationImageUniforms = 0,
	maxGeometryImageUniforms = 0,
	maxFragmentImageUniforms = 8,
	maxCombinedImageUniforms = 8,
	maxGeometryTextureImageUnits = 16,
	maxGeometryOutputVertices = 256,
	maxGeometryTotalOutputComponents = 1024,
	maxGeometryUniformComponents = 1024,
	maxGeometryVaryingComponents = 64,
	maxTessControlInputComponents = 128,
	maxTessControlOutputComponents = 128,
	maxTessControlTextureImageUnits = 16,
	maxTessControlUniformComponents = 1024,
	maxTessControlTotalOutputComponents = 4096,
	maxTessEvaluationInputComponents = 128,
	maxTessEvaluationOutputComponents = 128,
	maxTessEvaluationTextureImageUnits = 16,
	maxTessEvaluationUniformComponents = 1024,
	maxTessPatchComponents = 120,
	maxPatchVertices = 32,
	maxTessGenLevel = 64,
	maxViewports = 16,
	maxVertexAtomicCounters = 0,
	maxTessControlAtomicCounters = 0,
	maxTessEvaluationAtomicCounters = 0,
	maxGeometryAtomicCounters = 0,
	maxFragmentAtomicCounters = 8,
	maxCombinedAtomicCounters = 8,
	maxAtomicCounterBindings = 1,
	maxVertexAtomicCounterBuffers = 0,
	maxTessControlAtomicCounterBuffers = 0,
	maxTessEvaluationAtomicCounterBuffers = 0,
	maxGeometryAtomicCounterBuffers = 0,
	maxFragmentAtomicCounterBuffers = 1,
	maxCombinedAtomicCounterBuffers = 1,
	maxAtomicCounterBufferSize = 16384,
	maxTransformFeedbackBuffers = 4,
	maxTransformFeedbackInterleavedComponents = 64,
	maxCullDistances = 8,
	maxCombinedClipAndCullDistances = 8,
	maxSamples = 4,
	limits = {
		nonInductiveForLoops = 1,
		whileLoops = 1,
		doWhileLoops = 1,
		generalUniformIndexing = 1,
		generalAttributeMatrixVectorIndexing = 1,
		generalVaryingIndexing = 1,
		generalSamplerIndexing = 1,
		generalVariableIndexing = 1,
		generalConstantMatrixVectorIndexing = 1,
	}
}

function library.CreateDefaultResourceLimits()
	return ffi.new("struct TBuiltInResource", library.default_limits)
end

function library.TranslateVulkanShaderStageEnum(vk, e)
	if e == vk.e.SHADER_STAGE_VERTEX_BIT then
		return vk.e.LangVertex
	elseif e == vk.e.SHADER_STAGE_TESSELLATION_CONTROL_BIT then
		return vk.e.LangTessControl
	elseif e == vk.e.SHADER_STAGE_TESSELLATION_EVALUATION_BIT then
		return vk.e.LangTessEvaluation
	elseif e == vk.e.SHADER_STAGE_GEOMETRY_BIT then
		return vk.e.LangGeometry
	elseif e == vk.e.SHADER_STAGE_FRAGMENT_BIT then
		return vk.e.LangFragment
	elseif e == vk.e.SHADER_STAGE_COMPUTE_BIT then
		return vk.e.LangCompute
	end
end
]]

lua = lua .. "return library\n"

ffibuild.OutputAndValidate(lua, header)