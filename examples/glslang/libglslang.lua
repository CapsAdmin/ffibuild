local ffi = require("ffi")
ffi.cdef([[typedef enum EShMessages{EShMsgDefault=0,EShMsgRelaxedErrors=1,EShMsgSuppressWarnings=2,EShMsgAST=4,EShMsgSpvRules=8,EShMsgVulkanRules=16,EShMsgOnlyPreprocessor=32};
typedef enum EShLanguage{EShLangVertex=0,EShLangTessControl=1,EShLangTessEvaluation=2,EShLangGeometry=3,EShLangFragment=4,EShLangCompute=5,EShLangCount=6};
typedef enum EShOptimizationLevel{EShOptNoGeneration=0,EShOptNone=1,EShOptSimple=2,EShOptFull=3};
typedef enum EShExecutable{EShExVertexFragment=0,EShExFragment=1};
typedef enum EShLanguageMask{EShLangVertexMask=1,EShLangTessControlMask=2,EShLangTessEvaluationMask=4,EShLangGeometryMask=8,EShLangFragmentMask=16,EShLangComputeMask=32};
struct TLimits {int nonInductiveForLoops;int whileLoops;int doWhileLoops;int generalUniformIndexing;int generalAttributeMatrixVectorIndexing;int generalVaryingIndexing;int generalSamplerIndexing;int generalVariableIndexing;int generalConstantMatrixVectorIndexing;};
struct TBuiltInResource {int maxLights;int maxClipPlanes;int maxTextureUnits;int maxTextureCoords;int maxVertexAttribs;int maxVertexUniformComponents;int maxVaryingFloats;int maxVertexTextureImageUnits;int maxCombinedTextureImageUnits;int maxTextureImageUnits;int maxFragmentUniformComponents;int maxDrawBuffers;int maxVertexUniformVectors;int maxVaryingVectors;int maxFragmentUniformVectors;int maxVertexOutputVectors;int maxFragmentInputVectors;int minProgramTexelOffset;int maxProgramTexelOffset;int maxClipDistances;int maxComputeWorkGroupCountX;int maxComputeWorkGroupCountY;int maxComputeWorkGroupCountZ;int maxComputeWorkGroupSizeX;int maxComputeWorkGroupSizeY;int maxComputeWorkGroupSizeZ;int maxComputeUniformComponents;int maxComputeTextureImageUnits;int maxComputeImageUniforms;int maxComputeAtomicCounters;int maxComputeAtomicCounterBuffers;int maxVaryingComponents;int maxVertexOutputComponents;int maxGeometryInputComponents;int maxGeometryOutputComponents;int maxFragmentInputComponents;int maxImageUnits;int maxCombinedImageUnitsAndFragmentOutputs;int maxCombinedShaderOutputResources;int maxImageSamples;int maxVertexImageUniforms;int maxTessControlImageUniforms;int maxTessEvaluationImageUniforms;int maxGeometryImageUniforms;int maxFragmentImageUniforms;int maxCombinedImageUniforms;int maxGeometryTextureImageUnits;int maxGeometryOutputVertices;int maxGeometryTotalOutputComponents;int maxGeometryUniformComponents;int maxGeometryVaryingComponents;int maxTessControlInputComponents;int maxTessControlOutputComponents;int maxTessControlTextureImageUnits;int maxTessControlUniformComponents;int maxTessControlTotalOutputComponents;int maxTessEvaluationInputComponents;int maxTessEvaluationOutputComponents;int maxTessEvaluationTextureImageUnits;int maxTessEvaluationUniformComponents;int maxTessPatchComponents;int maxPatchVertices;int maxTessGenLevel;int maxViewports;int maxVertexAtomicCounters;int maxTessControlAtomicCounters;int maxTessEvaluationAtomicCounters;int maxGeometryAtomicCounters;int maxFragmentAtomicCounters;int maxCombinedAtomicCounters;int maxAtomicCounterBindings;int maxVertexAtomicCounterBuffers;int maxTessControlAtomicCounterBuffers;int maxTessEvaluationAtomicCounterBuffers;int maxGeometryAtomicCounterBuffers;int maxFragmentAtomicCounterBuffers;int maxCombinedAtomicCounterBuffers;int maxAtomicCounterBufferSize;int maxTransformFeedbackBuffers;int maxTransformFeedbackInterleavedComponents;int maxCullDistances;int maxCombinedClipAndCullDistances;int maxSamples;struct TLimits limits;};
struct ShBinding {const char*name;int binding;};
struct ShBindingTable {int numBindings;struct ShBinding*bindings;};
void ShSetEncryptionMethod(void*);
int ShSetVirtualAttributeBindings(void*const,const struct ShBindingTable*);
int ShGetPhysicalAttributeBindings(void*const,const struct ShBindingTable**);
int ShGetUniformLocation(void*const,const char*);
int ShCompile(void*const,const char*const*,const int,const int*,const enum EShOptimizationLevel,const struct TBuiltInResource*,int,int,int,enum EShMessages);
const char*ShGetInfoLog(void*const);
int ShSetFixedAttributeBindings(void*const,const struct ShBindingTable*);
void*ShConstructUniformMap();
int ShExcludeAttributes(void*const,int*,int);
void*ShConstructCompiler(const enum EShLanguage,int);
int ShLink(void*const,void*const*,const int,void*,short**,int*);
void ShDestruct(void*);
void*ShConstructLinker(const enum EShExecutable,int);
int ShFinalize();
int ShLinkExt(void*const,void*const*,const int);
int ShInitialize();
const void*ShGetExecutable(void*const);
]])
local CLIB = ffi.load(_G.FFI_LIB or "glslang")
local library = {}


--====helper safe_clib_index====
		function SAFE_INDEX(clib)
			return setmetatable({}, {__index = function(_, k)
				local ok, val = pcall(function() return clib[k] end)
				if ok then
					return val
				end
			end})
		end
	
--====helper safe_clib_index====

library = {
	SetEncryptionMethod = SAFE_INDEX(CLIB).ShSetEncryptionMethod,
	SetVirtualAttributeBindings = SAFE_INDEX(CLIB).ShSetVirtualAttributeBindings,
	GetPhysicalAttributeBindings = SAFE_INDEX(CLIB).ShGetPhysicalAttributeBindings,
	GetUniformLocation = SAFE_INDEX(CLIB).ShGetUniformLocation,
	Compile = SAFE_INDEX(CLIB).ShCompile,
	GetInfoLog = SAFE_INDEX(CLIB).ShGetInfoLog,
	SetFixedAttributeBindings = SAFE_INDEX(CLIB).ShSetFixedAttributeBindings,
	ConstructUniformMap = SAFE_INDEX(CLIB).ShConstructUniformMap,
	ExcludeAttributes = SAFE_INDEX(CLIB).ShExcludeAttributes,
	ConstructCompiler = SAFE_INDEX(CLIB).ShConstructCompiler,
	Link = SAFE_INDEX(CLIB).ShLink,
	Destruct = SAFE_INDEX(CLIB).ShDestruct,
	ConstructLinker = SAFE_INDEX(CLIB).ShConstructLinker,
	Finalize = SAFE_INDEX(CLIB).ShFinalize,
	LinkExt = SAFE_INDEX(CLIB).ShLinkExt,
	Initialize = SAFE_INDEX(CLIB).ShInitialize,
	GetExecutable = SAFE_INDEX(CLIB).ShGetExecutable,
}
library.e = {
	OptNoGeneration = ffi.cast("enum EShOptimizationLevel", "EShOptNoGeneration"),
	OptNone = ffi.cast("enum EShOptimizationLevel", "EShOptNone"),
	OptSimple = ffi.cast("enum EShOptimizationLevel", "EShOptSimple"),
	OptFull = ffi.cast("enum EShOptimizationLevel", "EShOptFull"),
	LangVertex = ffi.cast("enum EShLanguage", "EShLangVertex"),
	LangTessControl = ffi.cast("enum EShLanguage", "EShLangTessControl"),
	LangTessEvaluation = ffi.cast("enum EShLanguage", "EShLangTessEvaluation"),
	LangGeometry = ffi.cast("enum EShLanguage", "EShLangGeometry"),
	LangFragment = ffi.cast("enum EShLanguage", "EShLangFragment"),
	LangCompute = ffi.cast("enum EShLanguage", "EShLangCompute"),
	LangCount = ffi.cast("enum EShLanguage", "EShLangCount"),
	LangVertexMask = ffi.cast("enum EShLanguageMask", "EShLangVertexMask"),
	LangTessControlMask = ffi.cast("enum EShLanguageMask", "EShLangTessControlMask"),
	LangTessEvaluationMask = ffi.cast("enum EShLanguageMask", "EShLangTessEvaluationMask"),
	LangGeometryMask = ffi.cast("enum EShLanguageMask", "EShLangGeometryMask"),
	LangFragmentMask = ffi.cast("enum EShLanguageMask", "EShLangFragmentMask"),
	LangComputeMask = ffi.cast("enum EShLanguageMask", "EShLangComputeMask"),
	ExVertexFragment = ffi.cast("enum EShExecutable", "EShExVertexFragment"),
	ExFragment = ffi.cast("enum EShExecutable", "EShExFragment"),
	MsgDefault = ffi.cast("enum EShMessages", "EShMsgDefault"),
	MsgRelaxedErrors = ffi.cast("enum EShMessages", "EShMsgRelaxedErrors"),
	MsgSuppressWarnings = ffi.cast("enum EShMessages", "EShMsgSuppressWarnings"),
	MsgAST = ffi.cast("enum EShMessages", "EShMsgAST"),
	MsgSpvRules = ffi.cast("enum EShMessages", "EShMsgSpvRules"),
	MsgVulkanRules = ffi.cast("enum EShMessages", "EShMsgVulkanRules"),
	MsgOnlyPreprocessor = ffi.cast("enum EShMessages", "EShMsgOnlyPreprocessor"),
}
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
return library
