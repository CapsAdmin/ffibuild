local ffi = require("ffi")
ffi.cdef([[typedef enum aiTextureType{aiTextureType_NONE=0,aiTextureType_DIFFUSE=1,aiTextureType_SPECULAR=2,aiTextureType_AMBIENT=3,aiTextureType_EMISSIVE=4,aiTextureType_HEIGHT=5,aiTextureType_NORMALS=6,aiTextureType_SHININESS=7,aiTextureType_OPACITY=8,aiTextureType_DISPLACEMENT=9,aiTextureType_LIGHTMAP=10,aiTextureType_REFLECTION=11,aiTextureType_UNKNOWN=12};
typedef enum aiDefaultLogStream{aiDefaultLogStream_FILE=1,aiDefaultLogStream_STDOUT=2,aiDefaultLogStream_STDERR=4,aiDefaultLogStream_DEBUGGER=8};
typedef enum aiTextureMapping{aiTextureMapping_UV=0,aiTextureMapping_SPHERE=1,aiTextureMapping_CYLINDER=2,aiTextureMapping_BOX=3,aiTextureMapping_PLANE=4,aiTextureMapping_OTHER=5};
typedef enum aiReturn{aiReturn_SUCCESS=0,aiReturn_FAILURE=-1,aiReturn_OUTOFMEMORY=-3};
typedef enum aiTextureOp{aiTextureOp_Multiply=0,aiTextureOp_Add=1,aiTextureOp_Subtract=2,aiTextureOp_Divide=3,aiTextureOp_SmoothAdd=4,aiTextureOp_SignedAdd=5};
typedef enum aiPropertyTypeInfo{aiPTI_Float=1,aiPTI_String=3,aiPTI_Integer=4,aiPTI_Buffer=5};
typedef enum aiTextureMapMode{aiTextureMapMode_Wrap=0,aiTextureMapMode_Clamp=1,aiTextureMapMode_Decal=3,aiTextureMapMode_Mirror=2};
struct aiVector3D {union {struct {float x;float y;float z;};float v[3];};};
struct aiVector2D {union {struct {float x;float y;};float v[2];};};
struct aiColor4D {union {struct {float r;float g;float b;float a;};float c[4];};};
struct aiMatrix3x3 {union {struct {float a1;float a2;float a3;float b1;float b2;float b3;float c1;float c2;float c3;};float m[3][3];float mData[9];};};
struct aiMatrix4x4 {union {struct {float a1;float a2;float a3;float a4;float b1;float b2;float b3;float b4;float c1;float c2;float c3;float c4;float d1;float d2;float d3;float d4;};float m[4][4];float mData[16];};};
struct aiQuaternion {float w;float x;float y;float z;};
struct aiString {unsigned long length;char data[1024];};
struct aiMemoryInfo {unsigned int textures;unsigned int materials;unsigned int meshes;unsigned int nodes;unsigned int animations;unsigned int cameras;unsigned int lights;unsigned int total;};
struct aiUVTransform {struct aiVector2D mTranslation;struct aiVector2D mScaling;float mRotation;};
struct aiMaterialProperty {struct aiString mKey;unsigned int mSemantic;unsigned int mIndex;unsigned int mDataLength;enum aiPropertyTypeInfo mType;char*mData;};
struct aiMaterial {struct aiMaterialProperty**mProperties;unsigned int mNumProperties;unsigned int mNumAllocated;};
struct aiImporterDesc {const char*mName;const char*mAuthor;const char*mMaintainer;const char*mComments;unsigned int mFlags;unsigned int mMinMajor;unsigned int mMinMinor;unsigned int mMaxMajor;unsigned int mMaxMinor;const char*mFileExtensions;};
struct aiLogStream {void(*callback)(const char*,char*);char*user;};
struct aiPropertyStore {char sentinel;};
struct aiScene {};
struct aiFileIO {};
struct aiExportFormatDesc {const char*id;const char*description;const char*fileExtension;};
struct aiExportDataBlob {unsigned long size;void*data;struct aiString name;struct aiExportDataBlob*next;};
void(aiIdentityMatrix3)(struct aiMatrix3x3*);
unsigned long(aiGetImportFormatCount)();
void(aiReleaseImport)(const struct aiScene*);
void(aiCopyScene)(const struct aiScene*,struct aiScene**);
enum aiReturn(aiGetMaterialProperty)(const struct aiMaterial*,const char*,unsigned int,unsigned int,const struct aiMaterialProperty**);
void(aiSetImportPropertyFloat)(struct aiPropertyStore*,const char*,float);
enum aiReturn(aiExportSceneEx)(const struct aiScene*,const char*,const char*,struct aiFileIO*,unsigned int);
enum aiReturn(aiGetMaterialUVTransform)(const struct aiMaterial*,const char*,unsigned int,unsigned int,struct aiUVTransform*);
int(aiIsExtensionSupported)(const char*);
void(aiAttachLogStream)(const struct aiLogStream*);
void(aiTransposeMatrix3)(struct aiMatrix3x3*);
void(aiReleasePropertyStore)(struct aiPropertyStore*);
const char*(aiGetLegalString)();
void(aiSetImportPropertyString)(struct aiPropertyStore*,const char*,const struct aiString*);
unsigned int(aiGetCompileFlags)();
enum aiReturn(aiGetMaterialTexture)(const struct aiMaterial*,enum aiTextureType,unsigned int,struct aiString*,enum aiTextureMapping*,unsigned int*,float*,enum aiTextureOp*,enum aiTextureMapMode*,unsigned int*);
void(aiDecomposeMatrix)(const struct aiMatrix4x4*,struct aiVector3D*,struct aiQuaternion*,struct aiVector3D*);
void(aiTransformVecByMatrix4)(struct aiVector3D*,const struct aiMatrix4x4*);
const struct aiScene*(aiImportFileExWithProperties)(const char*,unsigned int,struct aiFileIO*,const struct aiPropertyStore*);
enum aiReturn(aiExportScene)(const struct aiScene*,const char*,const char*,unsigned int);
const struct aiScene*(aiApplyPostProcessing)(const struct aiScene*,unsigned int);
const struct aiImporterDesc*(aiGetImporterDesc)(const char*);
const struct aiExportDataBlob*(aiExportSceneToBlob)(const struct aiScene*,const char*,unsigned int);
const struct aiScene*(aiImportFileFromMemory)(const char*,unsigned int,unsigned int,const char*);
void(aiEnableVerboseLogging)(int);
void(aiTransposeMatrix4)(struct aiMatrix4x4*);
unsigned int(aiGetVersionMinor)();
struct aiLogStream(aiGetPredefinedLogStream)(enum aiDefaultLogStream,const char*);
enum aiReturn(aiGetMaterialString)(const struct aiMaterial*,const char*,unsigned int,unsigned int,struct aiString*);
enum aiReturn(aiGetMaterialFloatArray)(const struct aiMaterial*,const char*,unsigned int,unsigned int,float*,unsigned int*);
const struct aiImporterDesc*(aiGetImportFormatDescription)(unsigned long);
const struct aiExportFormatDesc*(aiGetExportFormatDescription)(unsigned long);
void(aiGetMemoryRequirements)(const struct aiScene*,struct aiMemoryInfo*);
unsigned int(aiGetVersionRevision)();
void(aiFreeScene)(const struct aiScene*);
const struct aiScene*(aiImportFile)(const char*,unsigned int);
void(aiIdentityMatrix4)(struct aiMatrix4x4*);
enum aiReturn(aiGetMaterialColor)(const struct aiMaterial*,const char*,unsigned int,unsigned int,struct aiColor4D*);
unsigned int(aiGetVersionMajor)();
void(aiReleaseExportBlob)(const struct aiExportDataBlob*);
void(aiReleaseExportFormatDescription)(const struct aiExportFormatDesc*);
unsigned long(aiGetExportFormatCount)();
void(aiMultiplyMatrix3)(struct aiMatrix3x3*,const struct aiMatrix3x3*);
void(aiMultiplyMatrix4)(struct aiMatrix4x4*,const struct aiMatrix4x4*);
void(aiTransformVecByMatrix3)(struct aiVector3D*,const struct aiMatrix3x3*);
void(aiCreateQuaternionFromMatrix)(struct aiQuaternion*,const struct aiMatrix3x3*);
void(aiSetImportPropertyMatrix)(struct aiPropertyStore*,const char*,const struct aiMatrix4x4*);
void(aiGetExtensionList)(struct aiString*);
void(aiDetachAllLogStreams)();
enum aiReturn(aiDetachLogStream)(const struct aiLogStream*);
struct aiPropertyStore*(aiCreatePropertyStore)();
unsigned int(aiGetMaterialTextureCount)(const struct aiMaterial*,enum aiTextureType);
void(aiSetImportPropertyInteger)(struct aiPropertyStore*,const char*,int);
const char*(aiGetErrorString)();
const struct aiScene*(aiImportFileFromMemoryWithProperties)(const char*,unsigned int,unsigned int,const char*,const struct aiPropertyStore*);
enum aiReturn(aiGetMaterialIntegerArray)(const struct aiMaterial*,const char*,unsigned int,unsigned int,int*,unsigned int*);
const struct aiScene*(aiImportFileEx)(const char*,unsigned int,struct aiFileIO*);
]])
local CLIB = ffi.load(_G.FFI_LIB or "assimp")
local library = {}
library = {
	IdentityMatrix3 = CLIB.aiIdentityMatrix3,
	GetImportFormatCount = CLIB.aiGetImportFormatCount,
	ReleaseImport = CLIB.aiReleaseImport,
	CopyScene = CLIB.aiCopyScene,
	GetMaterialProperty = CLIB.aiGetMaterialProperty,
	SetImportPropertyFloat = CLIB.aiSetImportPropertyFloat,
	ExportSceneEx = CLIB.aiExportSceneEx,
	GetMaterialUVTransform = CLIB.aiGetMaterialUVTransform,
	IsExtensionSupported = CLIB.aiIsExtensionSupported,
	AttachLogStream = CLIB.aiAttachLogStream,
	TransposeMatrix3 = CLIB.aiTransposeMatrix3,
	ReleasePropertyStore = CLIB.aiReleasePropertyStore,
	GetLegalString = CLIB.aiGetLegalString,
	SetImportPropertyString = CLIB.aiSetImportPropertyString,
	GetCompileFlags = CLIB.aiGetCompileFlags,
	GetMaterialTexture = CLIB.aiGetMaterialTexture,
	DecomposeMatrix = CLIB.aiDecomposeMatrix,
	TransformVecByMatrix4 = CLIB.aiTransformVecByMatrix4,
	ImportFileExWithProperties = CLIB.aiImportFileExWithProperties,
	ExportScene = CLIB.aiExportScene,
	ApplyPostProcessing = CLIB.aiApplyPostProcessing,
	GetImporterDesc = CLIB.aiGetImporterDesc,
	ExportSceneToBlob = CLIB.aiExportSceneToBlob,
	ImportFileFromMemory = CLIB.aiImportFileFromMemory,
	EnableVerboseLogging = CLIB.aiEnableVerboseLogging,
	TransposeMatrix4 = CLIB.aiTransposeMatrix4,
	GetVersionMinor = CLIB.aiGetVersionMinor,
	GetPredefinedLogStream = CLIB.aiGetPredefinedLogStream,
	GetMaterialString = CLIB.aiGetMaterialString,
	GetMaterialFloatArray = CLIB.aiGetMaterialFloatArray,
	GetImportFormatDescription = CLIB.aiGetImportFormatDescription,
	GetExportFormatDescription = CLIB.aiGetExportFormatDescription,
	GetMemoryRequirements = CLIB.aiGetMemoryRequirements,
	GetVersionRevision = CLIB.aiGetVersionRevision,
	FreeScene = CLIB.aiFreeScene,
	ImportFile = CLIB.aiImportFile,
	IdentityMatrix4 = CLIB.aiIdentityMatrix4,
	GetMaterialColor = CLIB.aiGetMaterialColor,
	GetVersionMajor = CLIB.aiGetVersionMajor,
	ReleaseExportBlob = CLIB.aiReleaseExportBlob,
	ReleaseExportFormatDescription = CLIB.aiReleaseExportFormatDescription,
	GetExportFormatCount = CLIB.aiGetExportFormatCount,
	MultiplyMatrix3 = CLIB.aiMultiplyMatrix3,
	MultiplyMatrix4 = CLIB.aiMultiplyMatrix4,
	TransformVecByMatrix3 = CLIB.aiTransformVecByMatrix3,
	CreateQuaternionFromMatrix = CLIB.aiCreateQuaternionFromMatrix,
	SetImportPropertyMatrix = CLIB.aiSetImportPropertyMatrix,
	GetExtensionList = CLIB.aiGetExtensionList,
	DetachAllLogStreams = CLIB.aiDetachAllLogStreams,
	DetachLogStream = CLIB.aiDetachLogStream,
	CreatePropertyStore = CLIB.aiCreatePropertyStore,
	GetMaterialTextureCount = CLIB.aiGetMaterialTextureCount,
	SetImportPropertyInteger = CLIB.aiSetImportPropertyInteger,
	GetErrorString = CLIB.aiGetErrorString,
	ImportFileFromMemoryWithProperties = CLIB.aiImportFileFromMemoryWithProperties,
	GetMaterialIntegerArray = CLIB.aiGetMaterialIntegerArray,
	ImportFileEx = CLIB.aiImportFileEx,
}
library.e = {
	NONE = ffi.cast("enum aiTextureType", "aiTextureType_NONE"),
	DIFFUSE = ffi.cast("enum aiTextureType", "aiTextureType_DIFFUSE"),
	SPECULAR = ffi.cast("enum aiTextureType", "aiTextureType_SPECULAR"),
	AMBIENT = ffi.cast("enum aiTextureType", "aiTextureType_AMBIENT"),
	EMISSIVE = ffi.cast("enum aiTextureType", "aiTextureType_EMISSIVE"),
	HEIGHT = ffi.cast("enum aiTextureType", "aiTextureType_HEIGHT"),
	NORMALS = ffi.cast("enum aiTextureType", "aiTextureType_NORMALS"),
	SHININESS = ffi.cast("enum aiTextureType", "aiTextureType_SHININESS"),
	OPACITY = ffi.cast("enum aiTextureType", "aiTextureType_OPACITY"),
	DISPLACEMENT = ffi.cast("enum aiTextureType", "aiTextureType_DISPLACEMENT"),
	LIGHTMAP = ffi.cast("enum aiTextureType", "aiTextureType_LIGHTMAP"),
	REFLECTION = ffi.cast("enum aiTextureType", "aiTextureType_REFLECTION"),
	UNKNOWN = ffi.cast("enum aiTextureType", "aiTextureType_UNKNOWN"),
	SET = ffi.cast("enum aiOrigin", "aiOrigin_SET"),
	CUR = ffi.cast("enum aiOrigin", "aiOrigin_CUR"),
	END = ffi.cast("enum aiOrigin", "aiOrigin_END"),
	UV = ffi.cast("enum aiTextureMapping", "aiTextureMapping_UV"),
	SPHERE = ffi.cast("enum aiTextureMapping", "aiTextureMapping_SPHERE"),
	CYLINDER = ffi.cast("enum aiTextureMapping", "aiTextureMapping_CYLINDER"),
	BOX = ffi.cast("enum aiTextureMapping", "aiTextureMapping_BOX"),
	PLANE = ffi.cast("enum aiTextureMapping", "aiTextureMapping_PLANE"),
	OTHER = ffi.cast("enum aiTextureMapping", "aiTextureMapping_OTHER"),
	Flat = ffi.cast("enum aiShadingMode", "aiShadingMode_Flat"),
	Gouraud = ffi.cast("enum aiShadingMode", "aiShadingMode_Gouraud"),
	Phong = ffi.cast("enum aiShadingMode", "aiShadingMode_Phong"),
	Blinn = ffi.cast("enum aiShadingMode", "aiShadingMode_Blinn"),
	Toon = ffi.cast("enum aiShadingMode", "aiShadingMode_Toon"),
	OrenNayar = ffi.cast("enum aiShadingMode", "aiShadingMode_OrenNayar"),
	Minnaert = ffi.cast("enum aiShadingMode", "aiShadingMode_Minnaert"),
	CookTorrance = ffi.cast("enum aiShadingMode", "aiShadingMode_CookTorrance"),
	NoShading = ffi.cast("enum aiShadingMode", "aiShadingMode_NoShading"),
	Fresnel = ffi.cast("enum aiShadingMode", "aiShadingMode_Fresnel"),
	Multiply = ffi.cast("enum aiTextureOp", "aiTextureOp_Multiply"),
	Add = ffi.cast("enum aiTextureOp", "aiTextureOp_Add"),
	Subtract = ffi.cast("enum aiTextureOp", "aiTextureOp_Subtract"),
	Divide = ffi.cast("enum aiTextureOp", "aiTextureOp_Divide"),
	SmoothAdd = ffi.cast("enum aiTextureOp", "aiTextureOp_SmoothAdd"),
	SignedAdd = ffi.cast("enum aiTextureOp", "aiTextureOp_SignedAdd"),
	Invert = ffi.cast("enum aiTextureFlags", "aiTextureFlags_Invert"),
	UseAlpha = ffi.cast("enum aiTextureFlags", "aiTextureFlags_UseAlpha"),
	IgnoreAlpha = ffi.cast("enum aiTextureFlags", "aiTextureFlags_IgnoreAlpha"),
	DEFAULT = ffi.cast("enum aiAnimBehaviour", "aiAnimBehaviour_DEFAULT"),
	CONSTANT = ffi.cast("enum aiAnimBehaviour", "aiAnimBehaviour_CONSTANT"),
	LINEAR = ffi.cast("enum aiAnimBehaviour", "aiAnimBehaviour_LINEAR"),
	REPEAT = ffi.cast("enum aiAnimBehaviour", "aiAnimBehaviour_REPEAT"),
	POINT = ffi.cast("enum aiPrimitiveType", "aiPrimitiveType_POINT"),
	LINE = ffi.cast("enum aiPrimitiveType", "aiPrimitiveType_LINE"),
	TRIANGLE = ffi.cast("enum aiPrimitiveType", "aiPrimitiveType_TRIANGLE"),
	POLYGON = ffi.cast("enum aiPrimitiveType", "aiPrimitiveType_POLYGON"),
	Default = ffi.cast("enum aiBlendMode", "aiBlendMode_Default"),
	Additive = ffi.cast("enum aiBlendMode", "aiBlendMode_Additive"),
	SupportTextFlavour = ffi.cast("enum aiImporterFlags", "aiImporterFlags_SupportTextFlavour"),
	SupportBinaryFlavour = ffi.cast("enum aiImporterFlags", "aiImporterFlags_SupportBinaryFlavour"),
	SupportCompressedFlavour = ffi.cast("enum aiImporterFlags", "aiImporterFlags_SupportCompressedFlavour"),
	LimitedSupport = ffi.cast("enum aiImporterFlags", "aiImporterFlags_LimitedSupport"),
	Experimental = ffi.cast("enum aiImporterFlags", "aiImporterFlags_Experimental"),
	UNDEFINED = ffi.cast("enum aiLightSourceType", "aiLightSource_UNDEFINED"),
	DIRECTIONAL = ffi.cast("enum aiLightSourceType", "aiLightSource_DIRECTIONAL"),
	POINT = ffi.cast("enum aiLightSourceType", "aiLightSource_POINT"),
	SPOT = ffi.cast("enum aiLightSourceType", "aiLightSource_SPOT"),
	AMBIENT = ffi.cast("enum aiLightSourceType", "aiLightSource_AMBIENT"),
	Wrap = ffi.cast("enum aiTextureMapMode", "aiTextureMapMode_Wrap"),
	Clamp = ffi.cast("enum aiTextureMapMode", "aiTextureMapMode_Clamp"),
	Decal = ffi.cast("enum aiTextureMapMode", "aiTextureMapMode_Decal"),
	Mirror = ffi.cast("enum aiTextureMapMode", "aiTextureMapMode_Mirror"),
	Float = ffi.cast("enum aiPropertyTypeInfo", "aiPTI_Float"),
	String = ffi.cast("enum aiPropertyTypeInfo", "aiPTI_String"),
	Integer = ffi.cast("enum aiPropertyTypeInfo", "aiPTI_Integer"),
	Buffer = ffi.cast("enum aiPropertyTypeInfo", "aiPTI_Buffer"),
	NORMALS = ffi.cast("enum aiComponent", "aiComponent_NORMALS"),
	TANGENTS_AND_BITANGENTS = ffi.cast("enum aiComponent", "aiComponent_TANGENTS_AND_BITANGENTS"),
	COLORS = ffi.cast("enum aiComponent", "aiComponent_COLORS"),
	TEXCOORDS = ffi.cast("enum aiComponent", "aiComponent_TEXCOORDS"),
	BONEWEIGHTS = ffi.cast("enum aiComponent", "aiComponent_BONEWEIGHTS"),
	ANIMATIONS = ffi.cast("enum aiComponent", "aiComponent_ANIMATIONS"),
	TEXTURES = ffi.cast("enum aiComponent", "aiComponent_TEXTURES"),
	LIGHTS = ffi.cast("enum aiComponent", "aiComponent_LIGHTS"),
	CAMERAS = ffi.cast("enum aiComponent", "aiComponent_CAMERAS"),
	MESHES = ffi.cast("enum aiComponent", "aiComponent_MESHES"),
	MATERIALS = ffi.cast("enum aiComponent", "aiComponent_MATERIALS"),
	FILE = ffi.cast("enum aiDefaultLogStream", "aiDefaultLogStream_FILE"),
	STDOUT = ffi.cast("enum aiDefaultLogStream", "aiDefaultLogStream_STDOUT"),
	STDERR = ffi.cast("enum aiDefaultLogStream", "aiDefaultLogStream_STDERR"),
	DEBUGGER = ffi.cast("enum aiDefaultLogStream", "aiDefaultLogStream_DEBUGGER"),
	CalcTangentSpace = ffi.cast("enum aiPostProcessSteps", "aiProcess_CalcTangentSpace"),
	JoinIdenticalVertices = ffi.cast("enum aiPostProcessSteps", "aiProcess_JoinIdenticalVertices"),
	MakeLeftHanded = ffi.cast("enum aiPostProcessSteps", "aiProcess_MakeLeftHanded"),
	Triangulate = ffi.cast("enum aiPostProcessSteps", "aiProcess_Triangulate"),
	RemoveComponent = ffi.cast("enum aiPostProcessSteps", "aiProcess_RemoveComponent"),
	GenNormals = ffi.cast("enum aiPostProcessSteps", "aiProcess_GenNormals"),
	GenSmoothNormals = ffi.cast("enum aiPostProcessSteps", "aiProcess_GenSmoothNormals"),
	SplitLargeMeshes = ffi.cast("enum aiPostProcessSteps", "aiProcess_SplitLargeMeshes"),
	PreTransformVertices = ffi.cast("enum aiPostProcessSteps", "aiProcess_PreTransformVertices"),
	LimitBoneWeights = ffi.cast("enum aiPostProcessSteps", "aiProcess_LimitBoneWeights"),
	ValidateDataStructure = ffi.cast("enum aiPostProcessSteps", "aiProcess_ValidateDataStructure"),
	ImproveCacheLocality = ffi.cast("enum aiPostProcessSteps", "aiProcess_ImproveCacheLocality"),
	RemoveRedundantMaterials = ffi.cast("enum aiPostProcessSteps", "aiProcess_RemoveRedundantMaterials"),
	FixInfacingNormals = ffi.cast("enum aiPostProcessSteps", "aiProcess_FixInfacingNormals"),
	SortByPType = ffi.cast("enum aiPostProcessSteps", "aiProcess_SortByPType"),
	FindDegenerates = ffi.cast("enum aiPostProcessSteps", "aiProcess_FindDegenerates"),
	FindInvalidData = ffi.cast("enum aiPostProcessSteps", "aiProcess_FindInvalidData"),
	GenUVCoords = ffi.cast("enum aiPostProcessSteps", "aiProcess_GenUVCoords"),
	TransformUVCoords = ffi.cast("enum aiPostProcessSteps", "aiProcess_TransformUVCoords"),
	FindInstances = ffi.cast("enum aiPostProcessSteps", "aiProcess_FindInstances"),
	OptimizeMeshes = ffi.cast("enum aiPostProcessSteps", "aiProcess_OptimizeMeshes"),
	OptimizeGraph = ffi.cast("enum aiPostProcessSteps", "aiProcess_OptimizeGraph"),
	FlipUVs = ffi.cast("enum aiPostProcessSteps", "aiProcess_FlipUVs"),
	FlipWindingOrder = ffi.cast("enum aiPostProcessSteps", "aiProcess_FlipWindingOrder"),
	SplitByBoneCount = ffi.cast("enum aiPostProcessSteps", "aiProcess_SplitByBoneCount"),
	Debone = ffi.cast("enum aiPostProcessSteps", "aiProcess_Debone"),
	SUCCESS = ffi.cast("enum aiReturn", "aiReturn_SUCCESS"),
	FAILURE = ffi.cast("enum aiReturn", "aiReturn_FAILURE"),
	OUTOFMEMORY = ffi.cast("enum aiReturn", "aiReturn_OUTOFMEMORY"),
}
return library
