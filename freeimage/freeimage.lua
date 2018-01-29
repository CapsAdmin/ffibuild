local ffi = require("ffi")
ffi.cdef([[typedef enum FREE_IMAGE_FORMAT{FIF_UNKNOWN=-1,FIF_BMP=0,FIF_ICO=1,FIF_JPEG=2,FIF_JNG=3,FIF_KOALA=4,FIF_LBM=5,FIF_IFF=5,FIF_MNG=6,FIF_PBM=7,FIF_PBMRAW=8,FIF_PCD=9,FIF_PCX=10,FIF_PGM=11,FIF_PGMRAW=12,FIF_PNG=13,FIF_PPM=14,FIF_PPMRAW=15,FIF_RAS=16,FIF_TARGA=17,FIF_TIFF=18,FIF_WBMP=19,FIF_PSD=20,FIF_CUT=21,FIF_XBM=22,FIF_XPM=23,FIF_DDS=24,FIF_GIF=25,FIF_HDR=26,FIF_FAXG3=27,FIF_SGI=28,FIF_EXR=29,FIF_J2K=30,FIF_JP2=31,FIF_PFM=32,FIF_PICT=33,FIF_RAW=34,FIF_WEBP=35,FIF_JXR=36};
typedef enum FREE_IMAGE_MDTYPE{FIDT_NOTYPE=0,FIDT_BYTE=1,FIDT_ASCII=2,FIDT_SHORT=3,FIDT_LONG=4,FIDT_RATIONAL=5,FIDT_SBYTE=6,FIDT_UNDEFINED=7,FIDT_SSHORT=8,FIDT_SLONG=9,FIDT_SRATIONAL=10,FIDT_FLOAT=11,FIDT_DOUBLE=12,FIDT_IFD=13,FIDT_PALETTE=14,FIDT_LONG8=16,FIDT_SLONG8=17,FIDT_IFD8=18};
typedef enum FREE_IMAGE_DITHER{FID_FS=0,FID_BAYER4x4=1,FID_BAYER8x8=2,FID_CLUSTER6x6=3,FID_CLUSTER8x8=4,FID_CLUSTER16x16=5,FID_BAYER16x16=6};
typedef enum FREE_IMAGE_MDMODEL{FIMD_NODATA=-1,FIMD_COMMENTS=0,FIMD_EXIF_MAIN=1,FIMD_EXIF_EXIF=2,FIMD_EXIF_GPS=3,FIMD_EXIF_MAKERNOTE=4,FIMD_EXIF_INTEROP=5,FIMD_IPTC=6,FIMD_XMP=7,FIMD_GEOTIFF=8,FIMD_ANIMATION=9,FIMD_CUSTOM=10,FIMD_EXIF_RAW=11};
typedef enum FREE_IMAGE_TYPE{FIT_UNKNOWN=0,FIT_BITMAP=1,FIT_UINT16=2,FIT_INT16=3,FIT_UINT32=4,FIT_INT32=5,FIT_FLOAT=6,FIT_DOUBLE=7,FIT_COMPLEX=8,FIT_RGB16=9,FIT_RGBA16=10,FIT_RGBF=11,FIT_RGBAF=12};
typedef enum FREE_IMAGE_COLOR_TYPE{FIC_MINISWHITE=0,FIC_MINISBLACK=1,FIC_RGB=2,FIC_PALETTE=3,FIC_RGBALPHA=4,FIC_CMYK=5};
typedef enum FREE_IMAGE_COLOR_CHANNEL{FICC_RGB=0,FICC_RED=1,FICC_GREEN=2,FICC_BLUE=3,FICC_ALPHA=4,FICC_BLACK=5,FICC_REAL=6,FICC_IMAG=7,FICC_MAG=8,FICC_PHASE=9};
typedef enum FREE_IMAGE_JPEG_OPERATION{FIJPEG_OP_NONE=0,FIJPEG_OP_FLIP_H=1,FIJPEG_OP_FLIP_V=2,FIJPEG_OP_TRANSPOSE=3,FIJPEG_OP_TRANSVERSE=4,FIJPEG_OP_ROTATE_90=5,FIJPEG_OP_ROTATE_180=6,FIJPEG_OP_ROTATE_270=7};
typedef enum FREE_IMAGE_FILTER{FILTER_BOX=0,FILTER_BICUBIC=1,FILTER_BILINEAR=2,FILTER_BSPLINE=3,FILTER_CATMULLROM=4,FILTER_LANCZOS3=5};
typedef enum FREE_IMAGE_TMO{FITMO_DRAGO03=0,FITMO_REINHARD05=1,FITMO_FATTAL02=2};
typedef enum FREE_IMAGE_QUANTIZE{FIQ_WUQUANT=0,FIQ_NNQUANT=1,FIQ_LFPQUANT=2};
struct FIBITMAP {void*data;};
struct FIMULTIBITMAP {void*data;};
struct tagRGBQUAD {unsigned char rgbBlue;unsigned char rgbGreen;unsigned char rgbRed;unsigned char rgbReserved;};
struct tagBITMAPINFOHEADER {unsigned int biSize;signed int biWidth;signed int biHeight;unsigned short biPlanes;unsigned short biBitCount;unsigned int biCompression;unsigned int biSizeImage;signed int biXPelsPerMeter;signed int biYPelsPerMeter;unsigned int biClrUsed;unsigned int biClrImportant;};
struct tagBITMAPINFO {struct tagBITMAPINFOHEADER bmiHeader;struct tagRGBQUAD bmiColors[1];};
struct FIICCPROFILE {unsigned short flags;unsigned int size;void*data;};
struct FIMETADATA {void*data;};
struct FITAG {void*data;};
struct FreeImageIO {unsigned int(*read_proc)(void*,unsigned int,unsigned int,void*);unsigned int(*write_proc)(void*,unsigned int,unsigned int,void*);int(*seek_proc)(void*,long,int);long(*tell_proc)(void*);};
struct FIMEMORY {void*data;};
struct tagBITMAPINFO*(FreeImage_GetInfo)(struct FIBITMAP*);
void(FreeImage_ConvertLine8To4)(unsigned char*,unsigned char*,int,struct tagRGBQUAD*);
long(FreeImage_TellMemory)(struct FIMEMORY*);
int(FreeImage_GetFIFCount)();
struct FIBITMAP*(FreeImage_ConvertToUINT16)(struct FIBITMAP*);
unsigned int(FreeImage_WriteMemory)(const void*,unsigned int,unsigned int,struct FIMEMORY*);
struct FIBITMAP*(FreeImage_RotateEx)(struct FIBITMAP*,double,double,double,double,double,signed int);
signed int(FreeImage_IsLittleEndian)();
signed int(FreeImage_Invert)(struct FIBITMAP*);
signed int(FreeImage_SeekMemory)(struct FIMEMORY*,long,int);
struct FIBITMAP*(FreeImage_ToneMapping)(struct FIBITMAP*,enum FREE_IMAGE_TMO,double,double);
struct FIBITMAP*(FreeImage_ConvertTo4Bits)(struct FIBITMAP*);
struct FIMULTIBITMAP*(FreeImage_OpenMultiBitmap)(enum FREE_IMAGE_FORMAT,const char*,signed int,signed int,signed int,int);
void(FreeImage_ConvertLine1To24)(unsigned char*,unsigned char*,int,struct tagRGBQUAD*);
signed int(FreeImage_SaveU)(enum FREE_IMAGE_FORMAT,struct FIBITMAP*,const int*,int);
struct FIBITMAP*(FreeImage_CreateView)(struct FIBITMAP*,unsigned int,unsigned int,unsigned int,unsigned int);
signed int(FreeImage_GetLockedPageNumbers)(struct FIMULTIBITMAP*,int*,int*);
const char*(FreeImage_GetFIFDescription)(enum FREE_IMAGE_FORMAT);
unsigned short(FreeImage_GetTagID)(struct FITAG*);
signed int(FreeImage_FlipVertical)(struct FIBITMAP*);
unsigned int(FreeImage_GetBPP)(struct FIBITMAP*);
const char*(FreeImage_TagToString)(enum FREE_IMAGE_MDMODEL,struct FITAG*,char*);
signed int(FreeImage_SetTagCount)(struct FITAG*,unsigned int);
signed int(FreeImage_PreMultiplyWithAlpha)(struct FIBITMAP*);
enum FREE_IMAGE_COLOR_TYPE(FreeImage_GetColorType)(struct FIBITMAP*);
struct FIBITMAP*(FreeImage_Copy)(struct FIBITMAP*,int,int,int,int);
struct FIBITMAP*(FreeImage_TmoReinhard05)(struct FIBITMAP*,double,double);
struct FIBITMAP*(FreeImage_GetThumbnail)(struct FIBITMAP*);
const char*(FreeImage_GetCopyrightMessage)();
void(FreeImage_SetDotsPerMeterX)(struct FIBITMAP*,unsigned int);
signed int(FreeImage_SetTagKey)(struct FITAG*,const char*);
struct FIBITMAP*(FreeImage_ConvertFromRawBits)(unsigned char*,int,int,int,unsigned int,unsigned int,unsigned int,unsigned int,signed int);
unsigned int(FreeImage_ZLibCompress)(unsigned char*,unsigned int,unsigned char*,unsigned int);
signed int(FreeImage_GetMetadata)(enum FREE_IMAGE_MDMODEL,struct FIBITMAP*,const char*,struct FITAG**);
signed int(FreeImage_IsTransparent)(struct FIBITMAP*);
void(FreeImage_ConvertLine16_565_To16_555)(unsigned char*,unsigned char*,int);
void(FreeImage_Initialise)(signed int);
signed int(FreeImage_SetTagLength)(struct FITAG*,unsigned int);
void(FreeImage_ConvertLine32To8)(unsigned char*,unsigned char*,int);
signed int(FreeImage_FIFSupportsNoPixels)(enum FREE_IMAGE_FORMAT);
int(FreeImage_GetTransparentIndex)(struct FIBITMAP*);
const void*(FreeImage_GetTagValue)(struct FITAG*);
enum FREE_IMAGE_FORMAT(FreeImage_GetFileTypeU)(const int*,int);
unsigned char*(FreeImage_GetScanLine)(struct FIBITMAP*,int);
struct FIBITMAP*(FreeImage_TmoDrago03)(struct FIBITMAP*,double,double);
unsigned int(FreeImage_ZLibUncompress)(unsigned char*,unsigned int,unsigned char*,unsigned int);
signed int(FreeImage_JPEGTransformFromHandle)(struct FreeImageIO*,void*,struct FreeImageIO*,void*,enum FREE_IMAGE_JPEG_OPERATION,int*,int*,int*,int*,signed int);
signed int(FreeImage_AdjustCurve)(struct FIBITMAP*,unsigned char*,enum FREE_IMAGE_COLOR_CHANNEL);
struct FIICCPROFILE*(FreeImage_GetICCProfile)(struct FIBITMAP*);
void(FreeImage_OutputMessageProc)(int,const char*,...);
struct FIMULTIBITMAP*(FreeImage_LoadMultiBitmapFromMemory)(enum FREE_IMAGE_FORMAT,struct FIMEMORY*,int);
const char*(FreeImage_GetTagKey)(struct FITAG*);
signed int(FreeImage_SetThumbnail)(struct FIBITMAP*,struct FIBITMAP*);
void(FreeImage_SetTransparent)(struct FIBITMAP*,signed int);
unsigned int(FreeImage_GetWidth)(struct FIBITMAP*);
signed int(FreeImage_SetTagType)(struct FITAG*,enum FREE_IMAGE_MDTYPE);
struct FIBITMAP*(FreeImage_ConvertFromRawBitsEx)(signed int,unsigned char*,enum FREE_IMAGE_TYPE,int,int,int,unsigned int,unsigned int,unsigned int,unsigned int,signed int);
signed int(FreeImage_AdjustGamma)(struct FIBITMAP*,double);
void(FreeImage_AppendPage)(struct FIMULTIBITMAP*,struct FIBITMAP*);
void(FreeImage_DeleteTag)(struct FITAG*);
signed int(FreeImage_FlipHorizontal)(struct FIBITMAP*);
unsigned int(FreeImage_GetColorsUsed)(struct FIBITMAP*);
struct FIBITMAP*(FreeImage_LockPage)(struct FIMULTIBITMAP*,int);
struct FIBITMAP*(FreeImage_ConvertToType)(struct FIBITMAP*,enum FREE_IMAGE_TYPE,signed int);
void(FreeImage_DeletePage)(struct FIMULTIBITMAP*,int);
const char*(FreeImage_GetVersion)();
signed int(FreeImage_SetChannel)(struct FIBITMAP*,struct FIBITMAP*,enum FREE_IMAGE_COLOR_CHANNEL);
signed int(FreeImage_HasPixels)(struct FIBITMAP*);
signed int(FreeImage_SetPixelColor)(struct FIBITMAP*,unsigned int,unsigned int,struct tagRGBQUAD*);
unsigned int(FreeImage_GetMetadataCount)(enum FREE_IMAGE_MDMODEL,struct FIBITMAP*);
struct FIBITMAP*(FreeImage_AllocateExT)(enum FREE_IMAGE_TYPE,int,int,int,const void*,int,const struct tagRGBQUAD*,unsigned int,unsigned int,unsigned int);
struct FIBITMAP*(FreeImage_AllocateEx)(int,int,int,const struct tagRGBQUAD*,int,const struct tagRGBQUAD*,unsigned int,unsigned int,unsigned int);
struct FIBITMAP*(FreeImage_EnlargeCanvas)(struct FIBITMAP*,int,int,int,int,const void*,int);
signed int(FreeImage_FillBackground)(struct FIBITMAP*,const void*,int);
void(FreeImage_ConvertLine16To4_555)(unsigned char*,unsigned char*,int);
struct FIBITMAP*(FreeImage_ConvertToGreyscale)(struct FIBITMAP*);
signed int(FreeImage_SetComplexChannel)(struct FIBITMAP*,struct FIBITMAP*,enum FREE_IMAGE_COLOR_CHANNEL);
unsigned char*(FreeImage_GetTransparencyTable)(struct FIBITMAP*);
struct FIBITMAP*(FreeImage_GetComplexChannel)(struct FIBITMAP*,enum FREE_IMAGE_COLOR_CHANNEL);
void(FreeImage_ConvertLine32To24)(unsigned char*,unsigned char*,int);
signed int(FreeImage_FIFSupportsWriting)(enum FREE_IMAGE_FORMAT);
signed int(FreeImage_GetBackgroundColor)(struct FIBITMAP*,struct tagRGBQUAD*);
void(FreeImage_ConvertLine24To32)(unsigned char*,unsigned char*,int);
struct FIBITMAP*(FreeImage_Dither)(struct FIBITMAP*,enum FREE_IMAGE_DITHER);
void(FreeImage_SetDotsPerMeterY)(struct FIBITMAP*,unsigned int);
unsigned int(FreeImage_SwapColors)(struct FIBITMAP*,struct tagRGBQUAD*,struct tagRGBQUAD*,signed int);
struct FIBITMAP*(FreeImage_TmoReinhard05Ex)(struct FIBITMAP*,double,double,double,double);
struct FIBITMAP*(FreeImage_Load)(enum FREE_IMAGE_FORMAT,const char*,int);
struct tagRGBQUAD*(FreeImage_GetPalette)(struct FIBITMAP*);
unsigned int(FreeImage_ApplyColorMapping)(struct FIBITMAP*,struct tagRGBQUAD*,struct tagRGBQUAD*,unsigned int,signed int,signed int);
signed int(FreeImage_AdjustColors)(struct FIBITMAP*,double,double,double,signed int);
int(FreeImage_GetAdjustColorsLookupTable)(unsigned char*,double,double,double,signed int);
signed int(FreeImage_GetHistogram)(struct FIBITMAP*,unsigned int*,enum FREE_IMAGE_COLOR_CHANNEL);
signed int(FreeImage_AdjustContrast)(struct FIBITMAP*,double);
void(FreeImage_ConvertLine1To4)(unsigned char*,unsigned char*,int);
signed int(FreeImage_AdjustBrightness)(struct FIBITMAP*,double);
struct FIBITMAP*(FreeImage_RescaleRect)(struct FIBITMAP*,int,int,int,int,int,int,enum FREE_IMAGE_FILTER,unsigned int);
signed int(FreeImage_SetTagDescription)(struct FITAG*,const char*);
struct FIBITMAP*(FreeImage_Rescale)(struct FIBITMAP*,int,int,enum FREE_IMAGE_FILTER);
signed int(FreeImage_JPEGTransformCombinedFromMemory)(struct FIMEMORY*,struct FIMEMORY*,enum FREE_IMAGE_JPEG_OPERATION,int*,int*,int*,int*,signed int);
struct FIBITMAP*(FreeImage_LoadFromHandle)(enum FREE_IMAGE_FORMAT,struct FreeImageIO*,void*,int);
unsigned int(FreeImage_GetTagLength)(struct FITAG*);
struct FIBITMAP*(FreeImage_Rotate)(struct FIBITMAP*,double,const void*);
struct FIBITMAP*(FreeImage_ConvertToRGBA16)(struct FIBITMAP*);
signed int(FreeImage_SetPixelIndex)(struct FIBITMAP*,unsigned int,unsigned int,unsigned char*);
signed int(FreeImage_JPEGTransformCombined)(const char*,const char*,enum FREE_IMAGE_JPEG_OPERATION,int*,int*,int*,int*,signed int);
void(FreeImage_ConvertLine24To16_565)(unsigned char*,unsigned char*,int);
signed int(FreeImage_JPEGCrop)(const char*,const char*,int,int,int,int);
signed int(FreeImage_JPEGTransformU)(const int*,const int*,enum FREE_IMAGE_JPEG_OPERATION,signed int);
struct FIBITMAP*(FreeImage_ConvertToFloat)(struct FIBITMAP*);
signed int(FreeImage_JPEGTransform)(const char*,const char*,enum FREE_IMAGE_JPEG_OPERATION,signed int);
unsigned int(FreeImage_GetDIBSize)(struct FIBITMAP*);
struct FIBITMAP*(FreeImage_ConvertToRGBAF)(struct FIBITMAP*);
struct FIBITMAP*(FreeImage_MultigridPoissonSolver)(struct FIBITMAP*,int);
void(FreeImage_ConvertLine4To8)(unsigned char*,unsigned char*,int);
void(FreeImage_DestroyICCProfile)(struct FIBITMAP*);
void(FreeImage_ConvertLine24To8)(unsigned char*,unsigned char*,int);
void(FreeImage_ConvertLine16To32_565)(unsigned char*,unsigned char*,int);
signed int(FreeImage_SetMetadata)(enum FREE_IMAGE_MDMODEL,struct FIBITMAP*,const char*,struct FITAG*);
void(FreeImage_ConvertLine4To24)(unsigned char*,unsigned char*,int,struct tagRGBQUAD*);
signed int(FreeImage_FindNextMetadata)(struct FIMETADATA*,struct FITAG**);
signed int(FreeImage_FIFSupportsICCProfiles)(enum FREE_IMAGE_FORMAT);
unsigned int(FreeImage_ReadMemory)(void*,unsigned int,unsigned int,struct FIMEMORY*);
void(FreeImage_UnlockPage)(struct FIMULTIBITMAP*,struct FIBITMAP*,signed int);
signed int(FreeImage_SetTagValue)(struct FITAG*,const void*);
enum FREE_IMAGE_FORMAT(FreeImage_GetFIFFromFormat)(const char*);
signed int(FreeImage_SetTagID)(struct FITAG*,unsigned short);
struct FIBITMAP*(FreeImage_MakeThumbnail)(struct FIBITMAP*,int,signed int);
struct FIBITMAP*(FreeImage_TmoFattal02)(struct FIBITMAP*,double,double);
unsigned int(FreeImage_GetTagCount)(struct FITAG*);
struct FIMEMORY*(FreeImage_OpenMemory)(unsigned char*,unsigned int);
enum FREE_IMAGE_MDTYPE(FreeImage_GetTagType)(struct FITAG*);
const char*(FreeImage_GetTagDescription)(struct FITAG*);
signed int(FreeImage_SaveToMemory)(enum FREE_IMAGE_FORMAT,struct FIBITMAP*,struct FIMEMORY*,int);
struct FIBITMAP*(FreeImage_Clone)(struct FIBITMAP*);
unsigned int(FreeImage_ZLibCRC32)(unsigned int,unsigned char*,unsigned int);
unsigned int(FreeImage_ZLibGZip)(unsigned char*,unsigned int,unsigned char*,unsigned int);
struct FIBITMAP*(FreeImage_ConvertToStandardType)(struct FIBITMAP*,signed int);
struct FIBITMAP*(FreeImage_ConvertToRGB16)(struct FIBITMAP*);
signed int(FreeImage_CloneMetadata)(struct FIBITMAP*,struct FIBITMAP*);
struct FIBITMAP*(FreeImage_ConvertToRGBF)(struct FIBITMAP*);
signed int(FreeImage_SetBackgroundColor)(struct FIBITMAP*,struct tagRGBQUAD*);
void(FreeImage_ConvertToRawBits)(unsigned char*,struct FIBITMAP*,int,unsigned int,unsigned int,unsigned int,unsigned int,signed int);
void(FreeImage_CloseMemory)(struct FIMEMORY*);
signed int(FreeImage_HasBackgroundColor)(struct FIBITMAP*);
struct FIBITMAP*(FreeImage_Threshold)(struct FIBITMAP*,unsigned char);
struct FIBITMAP*(FreeImage_ConvertTo24Bits)(struct FIBITMAP*);
enum FREE_IMAGE_TYPE(FreeImage_GetImageType)(struct FIBITMAP*);
unsigned int(FreeImage_GetLine)(struct FIBITMAP*);
struct FIBITMAP*(FreeImage_ColorQuantize)(struct FIBITMAP*,enum FREE_IMAGE_QUANTIZE);
enum FREE_IMAGE_FORMAT(FreeImage_GetFileTypeFromHandle)(struct FreeImageIO*,void*,int);
struct FIBITMAP*(FreeImage_ConvertTo32Bits)(struct FIBITMAP*);
struct FIICCPROFILE*(FreeImage_CreateICCProfile)(struct FIBITMAP*,void*,long);
signed int(FreeImage_Paste)(struct FIBITMAP*,struct FIBITMAP*,int,int,int);
int(FreeImage_IsPluginEnabled)(enum FREE_IMAGE_FORMAT);
struct FIBITMAP*(FreeImage_ConvertTo8Bits)(struct FIBITMAP*);
unsigned int(FreeImage_ApplyPaletteIndexMapping)(struct FIBITMAP*,unsigned char*,unsigned char*,unsigned int,signed int);
signed int(FreeImage_SetMetadataKeyValue)(enum FREE_IMAGE_MDMODEL,struct FIBITMAP*,const char*,const char*);
void(FreeImage_ConvertLine16To32_555)(unsigned char*,unsigned char*,int);
void(FreeImage_ConvertLine8To32)(unsigned char*,unsigned char*,int,struct tagRGBQUAD*);
unsigned int(FreeImage_GetGreenMask)(struct FIBITMAP*);
void(FreeImage_ConvertLine4To32)(unsigned char*,unsigned char*,int,struct tagRGBQUAD*);
signed int(FreeImage_FIFSupportsExportType)(enum FREE_IMAGE_FORMAT,enum FREE_IMAGE_TYPE);
struct FIBITMAP*(FreeImage_GetChannel)(struct FIBITMAP*,enum FREE_IMAGE_COLOR_CHANNEL);
void(FreeImage_InsertPage)(struct FIMULTIBITMAP*,int,struct FIBITMAP*);
void(FreeImage_ConvertLine16To24_565)(unsigned char*,unsigned char*,int);
void(FreeImage_ConvertLine16To24_555)(unsigned char*,unsigned char*,int);
unsigned int(FreeImage_GetTransparencyCount)(struct FIBITMAP*);
unsigned int(FreeImage_GetRedMask)(struct FIBITMAP*);
void(FreeImage_ConvertLine8To24)(unsigned char*,unsigned char*,int,struct tagRGBQUAD*);
void(FreeImage_FindCloseMetadata)(struct FIMETADATA*);
void(FreeImage_ConvertLine32To16_565)(unsigned char*,unsigned char*,int);
signed int(FreeImage_JPEGCropU)(const int*,const int*,int,int,int,int);
void(FreeImage_ConvertLine16_555_To16_565)(unsigned char*,unsigned char*,int);
const char*(FreeImage_GetFIFExtensionList)(enum FREE_IMAGE_FORMAT);
void(FreeImage_ConvertLine32To4)(unsigned char*,unsigned char*,int);
signed int(FreeImage_Save)(enum FREE_IMAGE_FORMAT,struct FIBITMAP*,const char*,int);
void(FreeImage_ConvertLine4To16_555)(unsigned char*,unsigned char*,int,struct tagRGBQUAD*);
signed int(FreeImage_FIFSupportsExportBPP)(enum FREE_IMAGE_FORMAT,int);
void(FreeImage_SetOutputMessageStdCall)(void(*omf)(enum FREE_IMAGE_FORMAT,const char*));
signed int(FreeImage_JPEGTransformCombinedU)(const int*,const int*,enum FREE_IMAGE_JPEG_OPERATION,int*,int*,int*,int*,signed int);
void(FreeImage_ConvertLine16To4_565)(unsigned char*,unsigned char*,int);
struct tagBITMAPINFOHEADER*(FreeImage_GetInfoHeader)(struct FIBITMAP*);
unsigned int(FreeImage_SwapPaletteIndices)(struct FIBITMAP*,unsigned char*,unsigned char*);
enum FREE_IMAGE_FORMAT(FreeImage_GetFIFFromFilenameU)(const int*);
void(FreeImage_DeInitialise)();
unsigned int(FreeImage_GetPitch)(struct FIBITMAP*);
enum FREE_IMAGE_FORMAT(FreeImage_RegisterExternalPlugin)(const char*,const char*,const char*,const char*,const char*);
unsigned int(FreeImage_GetHeight)(struct FIBITMAP*);
unsigned int(FreeImage_ZLibGUnzip)(unsigned char*,unsigned int,unsigned char*,unsigned int);
enum FREE_IMAGE_FORMAT(FreeImage_GetFileType)(const char*,int);
struct FIBITMAP*(FreeImage_ColorQuantizeEx)(struct FIBITMAP*,enum FREE_IMAGE_QUANTIZE,int,int,struct tagRGBQUAD*);
struct FIMETADATA*(FreeImage_FindFirstMetadata)(enum FREE_IMAGE_MDMODEL,struct FIBITMAP*,struct FITAG**);
void(FreeImage_ConvertLine1To8)(unsigned char*,unsigned char*,int);
struct FIBITMAP*(FreeImage_Composite)(struct FIBITMAP*,signed int,struct tagRGBQUAD*,struct FIBITMAP*);
unsigned char*(FreeImage_GetBits)(struct FIBITMAP*);
signed int(FreeImage_GetPixelIndex)(struct FIBITMAP*,unsigned int,unsigned int,unsigned char*);
unsigned int(FreeImage_GetMemorySize)(struct FIBITMAP*);
void(FreeImage_ConvertLine32To16_555)(unsigned char*,unsigned char*,int);
unsigned int(FreeImage_GetDotsPerMeterY)(struct FIBITMAP*);
void(FreeImage_ConvertLine24To4)(unsigned char*,unsigned char*,int);
void(FreeImage_ConvertLine1To32)(unsigned char*,unsigned char*,int,struct tagRGBQUAD*);
enum FREE_IMAGE_FORMAT(FreeImage_GetFIFFromMime)(const char*);
struct FITAG*(FreeImage_CreateTag)();
struct FIBITMAP*(FreeImage_LoadFromMemory)(enum FREE_IMAGE_FORMAT,struct FIMEMORY*,int);
enum FREE_IMAGE_FORMAT(FreeImage_GetFileTypeFromMemory)(struct FIMEMORY*,int);
void(FreeImage_Unload)(struct FIBITMAP*);
struct FIBITMAP*(FreeImage_ConvertTo16Bits565)(struct FIBITMAP*);
void(FreeImage_SetTransparentIndex)(struct FIBITMAP*,int);
enum FREE_IMAGE_FORMAT(FreeImage_RegisterLocalPlugin)(void(*proc_address)(struct Plugin*,int),const char*,const char*,const char*,const char*);
signed int(FreeImage_SaveMultiBitmapToHandle)(enum FREE_IMAGE_FORMAT,struct FIMULTIBITMAP*,struct FreeImageIO*,void*,int);
struct FIBITMAP*(FreeImage_LoadU)(enum FREE_IMAGE_FORMAT,const int*,int);
signed int(FreeImage_SaveToHandle)(enum FREE_IMAGE_FORMAT,struct FIBITMAP*,struct FreeImageIO*,void*,int);
signed int(FreeImage_CloseMultiBitmap)(struct FIMULTIBITMAP*,int);
struct FIBITMAP*(FreeImage_Allocate)(int,int,int,unsigned int,unsigned int,unsigned int);
struct FITAG*(FreeImage_CloneTag)(struct FITAG*);
signed int(FreeImage_AcquireMemory)(struct FIMEMORY*,unsigned char**,unsigned int*);
struct FIMULTIBITMAP*(FreeImage_OpenMultiBitmapFromHandle)(enum FREE_IMAGE_FORMAT,struct FreeImageIO*,void*,int);
void(FreeImage_ConvertLine24To16_555)(unsigned char*,unsigned char*,int);
void(FreeImage_SetOutputMessage)(void(*omf)(enum FREE_IMAGE_FORMAT,const char*));
const char*(FreeImage_GetFIFRegExpr)(enum FREE_IMAGE_FORMAT);
unsigned int(FreeImage_GetDotsPerMeterX)(struct FIBITMAP*);
void(FreeImage_ConvertLine8To16_555)(unsigned char*,unsigned char*,int,struct tagRGBQUAD*);
signed int(FreeImage_MovePage)(struct FIMULTIBITMAP*,int,int);
const char*(FreeImage_GetFormatFromFIF)(enum FREE_IMAGE_FORMAT);
struct FIBITMAP*(FreeImage_RotateClassic)(struct FIBITMAP*,double);
signed int(FreeImage_LookupX11Color)(const char*,unsigned char*,unsigned char*,unsigned char*);
enum FREE_IMAGE_FORMAT(FreeImage_GetFIFFromFilename)(const char*);
int(FreeImage_SetPluginEnabled)(enum FREE_IMAGE_FORMAT,signed int);
int(FreeImage_GetPageCount)(struct FIMULTIBITMAP*);
signed int(FreeImage_GetPixelColor)(struct FIBITMAP*,unsigned int,unsigned int,struct tagRGBQUAD*);
signed int(FreeImage_FIFSupportsReading)(enum FREE_IMAGE_FORMAT);
const char*(FreeImage_GetFIFMimeType)(enum FREE_IMAGE_FORMAT);
signed int(FreeImage_LookupSVGColor)(const char*,unsigned char*,unsigned char*,unsigned char*);
unsigned int(FreeImage_GetBlueMask)(struct FIBITMAP*);
void(FreeImage_SetTransparencyTable)(struct FIBITMAP*,unsigned char*,int);
struct FIBITMAP*(FreeImage_AllocateT)(enum FREE_IMAGE_TYPE,int,int,int,unsigned int,unsigned int,unsigned int);
signed int(FreeImage_SaveMultiBitmapToMemory)(enum FREE_IMAGE_FORMAT,struct FIMULTIBITMAP*,struct FIMEMORY*,int);
void(FreeImage_ConvertLine8To16_565)(unsigned char*,unsigned char*,int,struct tagRGBQUAD*);
struct FIBITMAP*(FreeImage_ConvertTo16Bits555)(struct FIBITMAP*);
void(FreeImage_ConvertLine16To8_555)(unsigned char*,unsigned char*,int);
void(FreeImage_ConvertLine16To8_565)(unsigned char*,unsigned char*,int);
void(FreeImage_ConvertLine1To16_555)(unsigned char*,unsigned char*,int,struct tagRGBQUAD*);
void(FreeImage_ConvertLine1To16_565)(unsigned char*,unsigned char*,int,struct tagRGBQUAD*);
void(FreeImage_ConvertLine4To16_565)(unsigned char*,unsigned char*,int,struct tagRGBQUAD*);
]])
local CLIB = ffi.load(_G.FFI_LIB or "freeimage")
local library = {}
library = {
	GetInfo = CLIB.FreeImage_GetInfo,
	ConvertLine8To4 = CLIB.FreeImage_ConvertLine8To4,
	TellMemory = CLIB.FreeImage_TellMemory,
	GetFIFCount = CLIB.FreeImage_GetFIFCount,
	ConvertToUINT16 = CLIB.FreeImage_ConvertToUINT16,
	WriteMemory = CLIB.FreeImage_WriteMemory,
	RotateEx = CLIB.FreeImage_RotateEx,
	IsLittleEndian = CLIB.FreeImage_IsLittleEndian,
	Invert = CLIB.FreeImage_Invert,
	SeekMemory = CLIB.FreeImage_SeekMemory,
	ToneMapping = CLIB.FreeImage_ToneMapping,
	ConvertTo4Bits = CLIB.FreeImage_ConvertTo4Bits,
	OpenMultiBitmap = CLIB.FreeImage_OpenMultiBitmap,
	ConvertLine1To24 = CLIB.FreeImage_ConvertLine1To24,
	SaveU = CLIB.FreeImage_SaveU,
	CreateView = CLIB.FreeImage_CreateView,
	GetLockedPageNumbers = CLIB.FreeImage_GetLockedPageNumbers,
	GetFIFDescription = CLIB.FreeImage_GetFIFDescription,
	GetTagID = CLIB.FreeImage_GetTagID,
	FlipVertical = CLIB.FreeImage_FlipVertical,
	GetBPP = CLIB.FreeImage_GetBPP,
	TagToString = CLIB.FreeImage_TagToString,
	SetTagCount = CLIB.FreeImage_SetTagCount,
	PreMultiplyWithAlpha = CLIB.FreeImage_PreMultiplyWithAlpha,
	GetColorType = CLIB.FreeImage_GetColorType,
	Copy = CLIB.FreeImage_Copy,
	TmoReinhard05 = CLIB.FreeImage_TmoReinhard05,
	GetThumbnail = CLIB.FreeImage_GetThumbnail,
	GetCopyrightMessage = CLIB.FreeImage_GetCopyrightMessage,
	SetDotsPerMeterX = CLIB.FreeImage_SetDotsPerMeterX,
	SetTagKey = CLIB.FreeImage_SetTagKey,
	ConvertFromRawBits = CLIB.FreeImage_ConvertFromRawBits,
	ZLibCompress = CLIB.FreeImage_ZLibCompress,
	GetMetadata = CLIB.FreeImage_GetMetadata,
	IsTransparent = CLIB.FreeImage_IsTransparent,
	ConvertLine16_565_To16_555 = CLIB.FreeImage_ConvertLine16_565_To16_555,
	Initialise = CLIB.FreeImage_Initialise,
	SetTagLength = CLIB.FreeImage_SetTagLength,
	ConvertLine32To8 = CLIB.FreeImage_ConvertLine32To8,
	FIFSupportsNoPixels = CLIB.FreeImage_FIFSupportsNoPixels,
	GetTransparentIndex = CLIB.FreeImage_GetTransparentIndex,
	GetTagValue = CLIB.FreeImage_GetTagValue,
	GetFileTypeU = CLIB.FreeImage_GetFileTypeU,
	GetScanLine = CLIB.FreeImage_GetScanLine,
	TmoDrago03 = CLIB.FreeImage_TmoDrago03,
	ZLibUncompress = CLIB.FreeImage_ZLibUncompress,
	JPEGTransformFromHandle = CLIB.FreeImage_JPEGTransformFromHandle,
	AdjustCurve = CLIB.FreeImage_AdjustCurve,
	GetICCProfile = CLIB.FreeImage_GetICCProfile,
	OutputMessageProc = CLIB.FreeImage_OutputMessageProc,
	LoadMultiBitmapFromMemory = CLIB.FreeImage_LoadMultiBitmapFromMemory,
	GetTagKey = CLIB.FreeImage_GetTagKey,
	SetThumbnail = CLIB.FreeImage_SetThumbnail,
	SetTransparent = CLIB.FreeImage_SetTransparent,
	GetWidth = CLIB.FreeImage_GetWidth,
	SetTagType = CLIB.FreeImage_SetTagType,
	ConvertFromRawBitsEx = CLIB.FreeImage_ConvertFromRawBitsEx,
	AdjustGamma = CLIB.FreeImage_AdjustGamma,
	AppendPage = CLIB.FreeImage_AppendPage,
	DeleteTag = CLIB.FreeImage_DeleteTag,
	FlipHorizontal = CLIB.FreeImage_FlipHorizontal,
	GetColorsUsed = CLIB.FreeImage_GetColorsUsed,
	LockPage = CLIB.FreeImage_LockPage,
	ConvertToType = CLIB.FreeImage_ConvertToType,
	DeletePage = CLIB.FreeImage_DeletePage,
	GetVersion = CLIB.FreeImage_GetVersion,
	SetChannel = CLIB.FreeImage_SetChannel,
	HasPixels = CLIB.FreeImage_HasPixels,
	SetPixelColor = CLIB.FreeImage_SetPixelColor,
	GetMetadataCount = CLIB.FreeImage_GetMetadataCount,
	AllocateExT = CLIB.FreeImage_AllocateExT,
	AllocateEx = CLIB.FreeImage_AllocateEx,
	EnlargeCanvas = CLIB.FreeImage_EnlargeCanvas,
	FillBackground = CLIB.FreeImage_FillBackground,
	ConvertLine16To4_555 = CLIB.FreeImage_ConvertLine16To4_555,
	ConvertToGreyscale = CLIB.FreeImage_ConvertToGreyscale,
	SetComplexChannel = CLIB.FreeImage_SetComplexChannel,
	GetTransparencyTable = CLIB.FreeImage_GetTransparencyTable,
	GetComplexChannel = CLIB.FreeImage_GetComplexChannel,
	ConvertLine32To24 = CLIB.FreeImage_ConvertLine32To24,
	FIFSupportsWriting = CLIB.FreeImage_FIFSupportsWriting,
	GetBackgroundColor = CLIB.FreeImage_GetBackgroundColor,
	ConvertLine24To32 = CLIB.FreeImage_ConvertLine24To32,
	Dither = CLIB.FreeImage_Dither,
	SetDotsPerMeterY = CLIB.FreeImage_SetDotsPerMeterY,
	SwapColors = CLIB.FreeImage_SwapColors,
	TmoReinhard05Ex = CLIB.FreeImage_TmoReinhard05Ex,
	Load = CLIB.FreeImage_Load,
	GetPalette = CLIB.FreeImage_GetPalette,
	ApplyColorMapping = CLIB.FreeImage_ApplyColorMapping,
	AdjustColors = CLIB.FreeImage_AdjustColors,
	GetAdjustColorsLookupTable = CLIB.FreeImage_GetAdjustColorsLookupTable,
	GetHistogram = CLIB.FreeImage_GetHistogram,
	AdjustContrast = CLIB.FreeImage_AdjustContrast,
	ConvertLine1To4 = CLIB.FreeImage_ConvertLine1To4,
	AdjustBrightness = CLIB.FreeImage_AdjustBrightness,
	RescaleRect = CLIB.FreeImage_RescaleRect,
	SetTagDescription = CLIB.FreeImage_SetTagDescription,
	Rescale = CLIB.FreeImage_Rescale,
	JPEGTransformCombinedFromMemory = CLIB.FreeImage_JPEGTransformCombinedFromMemory,
	LoadFromHandle = CLIB.FreeImage_LoadFromHandle,
	GetTagLength = CLIB.FreeImage_GetTagLength,
	Rotate = CLIB.FreeImage_Rotate,
	ConvertToRGBA16 = CLIB.FreeImage_ConvertToRGBA16,
	SetPixelIndex = CLIB.FreeImage_SetPixelIndex,
	JPEGTransformCombined = CLIB.FreeImage_JPEGTransformCombined,
	ConvertLine24To16_565 = CLIB.FreeImage_ConvertLine24To16_565,
	JPEGCrop = CLIB.FreeImage_JPEGCrop,
	JPEGTransformU = CLIB.FreeImage_JPEGTransformU,
	ConvertToFloat = CLIB.FreeImage_ConvertToFloat,
	JPEGTransform = CLIB.FreeImage_JPEGTransform,
	GetDIBSize = CLIB.FreeImage_GetDIBSize,
	ConvertToRGBAF = CLIB.FreeImage_ConvertToRGBAF,
	MultigridPoissonSolver = CLIB.FreeImage_MultigridPoissonSolver,
	ConvertLine4To8 = CLIB.FreeImage_ConvertLine4To8,
	DestroyICCProfile = CLIB.FreeImage_DestroyICCProfile,
	ConvertLine24To8 = CLIB.FreeImage_ConvertLine24To8,
	ConvertLine16To32_565 = CLIB.FreeImage_ConvertLine16To32_565,
	SetMetadata = CLIB.FreeImage_SetMetadata,
	ConvertLine4To24 = CLIB.FreeImage_ConvertLine4To24,
	FindNextMetadata = CLIB.FreeImage_FindNextMetadata,
	FIFSupportsICCProfiles = CLIB.FreeImage_FIFSupportsICCProfiles,
	ReadMemory = CLIB.FreeImage_ReadMemory,
	UnlockPage = CLIB.FreeImage_UnlockPage,
	SetTagValue = CLIB.FreeImage_SetTagValue,
	GetFIFFromFormat = CLIB.FreeImage_GetFIFFromFormat,
	SetTagID = CLIB.FreeImage_SetTagID,
	MakeThumbnail = CLIB.FreeImage_MakeThumbnail,
	TmoFattal02 = CLIB.FreeImage_TmoFattal02,
	GetTagCount = CLIB.FreeImage_GetTagCount,
	OpenMemory = CLIB.FreeImage_OpenMemory,
	GetTagType = CLIB.FreeImage_GetTagType,
	GetTagDescription = CLIB.FreeImage_GetTagDescription,
	SaveToMemory = CLIB.FreeImage_SaveToMemory,
	Clone = CLIB.FreeImage_Clone,
	ZLibCRC32 = CLIB.FreeImage_ZLibCRC32,
	ZLibGZip = CLIB.FreeImage_ZLibGZip,
	ConvertToStandardType = CLIB.FreeImage_ConvertToStandardType,
	ConvertToRGB16 = CLIB.FreeImage_ConvertToRGB16,
	CloneMetadata = CLIB.FreeImage_CloneMetadata,
	ConvertToRGBF = CLIB.FreeImage_ConvertToRGBF,
	SetBackgroundColor = CLIB.FreeImage_SetBackgroundColor,
	ConvertToRawBits = CLIB.FreeImage_ConvertToRawBits,
	CloseMemory = CLIB.FreeImage_CloseMemory,
	HasBackgroundColor = CLIB.FreeImage_HasBackgroundColor,
	Threshold = CLIB.FreeImage_Threshold,
	ConvertTo24Bits = CLIB.FreeImage_ConvertTo24Bits,
	GetImageType = CLIB.FreeImage_GetImageType,
	GetLine = CLIB.FreeImage_GetLine,
	ColorQuantize = CLIB.FreeImage_ColorQuantize,
	GetFileTypeFromHandle = CLIB.FreeImage_GetFileTypeFromHandle,
	ConvertTo32Bits = CLIB.FreeImage_ConvertTo32Bits,
	CreateICCProfile = CLIB.FreeImage_CreateICCProfile,
	Paste = CLIB.FreeImage_Paste,
	IsPluginEnabled = CLIB.FreeImage_IsPluginEnabled,
	ConvertTo8Bits = CLIB.FreeImage_ConvertTo8Bits,
	ApplyPaletteIndexMapping = CLIB.FreeImage_ApplyPaletteIndexMapping,
	SetMetadataKeyValue = CLIB.FreeImage_SetMetadataKeyValue,
	ConvertLine16To32_555 = CLIB.FreeImage_ConvertLine16To32_555,
	ConvertLine8To32 = CLIB.FreeImage_ConvertLine8To32,
	GetGreenMask = CLIB.FreeImage_GetGreenMask,
	ConvertLine4To32 = CLIB.FreeImage_ConvertLine4To32,
	FIFSupportsExportType = CLIB.FreeImage_FIFSupportsExportType,
	GetChannel = CLIB.FreeImage_GetChannel,
	InsertPage = CLIB.FreeImage_InsertPage,
	ConvertLine16To24_565 = CLIB.FreeImage_ConvertLine16To24_565,
	ConvertLine16To24_555 = CLIB.FreeImage_ConvertLine16To24_555,
	GetTransparencyCount = CLIB.FreeImage_GetTransparencyCount,
	GetRedMask = CLIB.FreeImage_GetRedMask,
	ConvertLine8To24 = CLIB.FreeImage_ConvertLine8To24,
	FindCloseMetadata = CLIB.FreeImage_FindCloseMetadata,
	ConvertLine32To16_565 = CLIB.FreeImage_ConvertLine32To16_565,
	JPEGCropU = CLIB.FreeImage_JPEGCropU,
	ConvertLine16_555_To16_565 = CLIB.FreeImage_ConvertLine16_555_To16_565,
	GetFIFExtensionList = CLIB.FreeImage_GetFIFExtensionList,
	ConvertLine32To4 = CLIB.FreeImage_ConvertLine32To4,
	Save = CLIB.FreeImage_Save,
	ConvertLine4To16_555 = CLIB.FreeImage_ConvertLine4To16_555,
	FIFSupportsExportBPP = CLIB.FreeImage_FIFSupportsExportBPP,
	SetOutputMessageStdCall = CLIB.FreeImage_SetOutputMessageStdCall,
	JPEGTransformCombinedU = CLIB.FreeImage_JPEGTransformCombinedU,
	ConvertLine16To4_565 = CLIB.FreeImage_ConvertLine16To4_565,
	GetInfoHeader = CLIB.FreeImage_GetInfoHeader,
	SwapPaletteIndices = CLIB.FreeImage_SwapPaletteIndices,
	GetFIFFromFilenameU = CLIB.FreeImage_GetFIFFromFilenameU,
	DeInitialise = CLIB.FreeImage_DeInitialise,
	GetPitch = CLIB.FreeImage_GetPitch,
	GetHeight = CLIB.FreeImage_GetHeight,
	ZLibGUnzip = CLIB.FreeImage_ZLibGUnzip,
	GetFileType = CLIB.FreeImage_GetFileType,
	ColorQuantizeEx = CLIB.FreeImage_ColorQuantizeEx,
	FindFirstMetadata = CLIB.FreeImage_FindFirstMetadata,
	ConvertLine1To8 = CLIB.FreeImage_ConvertLine1To8,
	Composite = CLIB.FreeImage_Composite,
	GetBits = CLIB.FreeImage_GetBits,
	GetPixelIndex = CLIB.FreeImage_GetPixelIndex,
	GetMemorySize = CLIB.FreeImage_GetMemorySize,
	ConvertLine32To16_555 = CLIB.FreeImage_ConvertLine32To16_555,
	GetDotsPerMeterY = CLIB.FreeImage_GetDotsPerMeterY,
	ConvertLine24To4 = CLIB.FreeImage_ConvertLine24To4,
	ConvertLine1To32 = CLIB.FreeImage_ConvertLine1To32,
	GetFIFFromMime = CLIB.FreeImage_GetFIFFromMime,
	CreateTag = CLIB.FreeImage_CreateTag,
	LoadFromMemory = CLIB.FreeImage_LoadFromMemory,
	GetFileTypeFromMemory = CLIB.FreeImage_GetFileTypeFromMemory,
	Unload = CLIB.FreeImage_Unload,
	ConvertTo16Bits565 = CLIB.FreeImage_ConvertTo16Bits565,
	SetTransparentIndex = CLIB.FreeImage_SetTransparentIndex,
	RegisterLocalPlugin = CLIB.FreeImage_RegisterLocalPlugin,
	SaveMultiBitmapToHandle = CLIB.FreeImage_SaveMultiBitmapToHandle,
	LoadU = CLIB.FreeImage_LoadU,
	SaveToHandle = CLIB.FreeImage_SaveToHandle,
	CloseMultiBitmap = CLIB.FreeImage_CloseMultiBitmap,
	Allocate = CLIB.FreeImage_Allocate,
	CloneTag = CLIB.FreeImage_CloneTag,
	AcquireMemory = CLIB.FreeImage_AcquireMemory,
	OpenMultiBitmapFromHandle = CLIB.FreeImage_OpenMultiBitmapFromHandle,
	ConvertLine24To16_555 = CLIB.FreeImage_ConvertLine24To16_555,
	SetOutputMessage = CLIB.FreeImage_SetOutputMessage,
	GetFIFRegExpr = CLIB.FreeImage_GetFIFRegExpr,
	GetDotsPerMeterX = CLIB.FreeImage_GetDotsPerMeterX,
	ConvertLine8To16_555 = CLIB.FreeImage_ConvertLine8To16_555,
	MovePage = CLIB.FreeImage_MovePage,
	GetFormatFromFIF = CLIB.FreeImage_GetFormatFromFIF,
	RotateClassic = CLIB.FreeImage_RotateClassic,
	LookupX11Color = CLIB.FreeImage_LookupX11Color,
	GetFIFFromFilename = CLIB.FreeImage_GetFIFFromFilename,
	SetPluginEnabled = CLIB.FreeImage_SetPluginEnabled,
	GetPageCount = CLIB.FreeImage_GetPageCount,
	GetPixelColor = CLIB.FreeImage_GetPixelColor,
	FIFSupportsReading = CLIB.FreeImage_FIFSupportsReading,
	GetFIFMimeType = CLIB.FreeImage_GetFIFMimeType,
	LookupSVGColor = CLIB.FreeImage_LookupSVGColor,
	GetBlueMask = CLIB.FreeImage_GetBlueMask,
	SetTransparencyTable = CLIB.FreeImage_SetTransparencyTable,
	AllocateT = CLIB.FreeImage_AllocateT,
	SaveMultiBitmapToMemory = CLIB.FreeImage_SaveMultiBitmapToMemory,
	ConvertLine8To16_565 = CLIB.FreeImage_ConvertLine8To16_565,
	ConvertTo16Bits555 = CLIB.FreeImage_ConvertTo16Bits555,
	ConvertLine16To8_555 = CLIB.FreeImage_ConvertLine16To8_555,
	ConvertLine16To8_565 = CLIB.FreeImage_ConvertLine16To8_565,
	ConvertLine1To16_555 = CLIB.FreeImage_ConvertLine1To16_555,
	ConvertLine1To16_565 = CLIB.FreeImage_ConvertLine1To16_565,
	ConvertLine4To16_565 = CLIB.FreeImage_ConvertLine4To16_565,
}
library.e = {
	FORMAT_UNKNOWN = ffi.cast("enum FREE_IMAGE_FORMAT", "FIF_UNKNOWN"),
	FORMAT_BMP = ffi.cast("enum FREE_IMAGE_FORMAT", "FIF_BMP"),
	FORMAT_ICO = ffi.cast("enum FREE_IMAGE_FORMAT", "FIF_ICO"),
	FORMAT_JPEG = ffi.cast("enum FREE_IMAGE_FORMAT", "FIF_JPEG"),
	FORMAT_JNG = ffi.cast("enum FREE_IMAGE_FORMAT", "FIF_JNG"),
	FORMAT_KOALA = ffi.cast("enum FREE_IMAGE_FORMAT", "FIF_KOALA"),
	FORMAT_LBM = ffi.cast("enum FREE_IMAGE_FORMAT", "FIF_LBM"),
	FORMAT_IFF = ffi.cast("enum FREE_IMAGE_FORMAT", "FIF_IFF"),
	FORMAT_MNG = ffi.cast("enum FREE_IMAGE_FORMAT", "FIF_MNG"),
	FORMAT_PBM = ffi.cast("enum FREE_IMAGE_FORMAT", "FIF_PBM"),
	FORMAT_PBMRAW = ffi.cast("enum FREE_IMAGE_FORMAT", "FIF_PBMRAW"),
	FORMAT_PCD = ffi.cast("enum FREE_IMAGE_FORMAT", "FIF_PCD"),
	FORMAT_PCX = ffi.cast("enum FREE_IMAGE_FORMAT", "FIF_PCX"),
	FORMAT_PGM = ffi.cast("enum FREE_IMAGE_FORMAT", "FIF_PGM"),
	FORMAT_PGMRAW = ffi.cast("enum FREE_IMAGE_FORMAT", "FIF_PGMRAW"),
	FORMAT_PNG = ffi.cast("enum FREE_IMAGE_FORMAT", "FIF_PNG"),
	FORMAT_PPM = ffi.cast("enum FREE_IMAGE_FORMAT", "FIF_PPM"),
	FORMAT_PPMRAW = ffi.cast("enum FREE_IMAGE_FORMAT", "FIF_PPMRAW"),
	FORMAT_RAS = ffi.cast("enum FREE_IMAGE_FORMAT", "FIF_RAS"),
	FORMAT_TARGA = ffi.cast("enum FREE_IMAGE_FORMAT", "FIF_TARGA"),
	FORMAT_TIFF = ffi.cast("enum FREE_IMAGE_FORMAT", "FIF_TIFF"),
	FORMAT_WBMP = ffi.cast("enum FREE_IMAGE_FORMAT", "FIF_WBMP"),
	FORMAT_PSD = ffi.cast("enum FREE_IMAGE_FORMAT", "FIF_PSD"),
	FORMAT_CUT = ffi.cast("enum FREE_IMAGE_FORMAT", "FIF_CUT"),
	FORMAT_XBM = ffi.cast("enum FREE_IMAGE_FORMAT", "FIF_XBM"),
	FORMAT_XPM = ffi.cast("enum FREE_IMAGE_FORMAT", "FIF_XPM"),
	FORMAT_DDS = ffi.cast("enum FREE_IMAGE_FORMAT", "FIF_DDS"),
	FORMAT_GIF = ffi.cast("enum FREE_IMAGE_FORMAT", "FIF_GIF"),
	FORMAT_HDR = ffi.cast("enum FREE_IMAGE_FORMAT", "FIF_HDR"),
	FORMAT_FAXG3 = ffi.cast("enum FREE_IMAGE_FORMAT", "FIF_FAXG3"),
	FORMAT_SGI = ffi.cast("enum FREE_IMAGE_FORMAT", "FIF_SGI"),
	FORMAT_EXR = ffi.cast("enum FREE_IMAGE_FORMAT", "FIF_EXR"),
	FORMAT_J2K = ffi.cast("enum FREE_IMAGE_FORMAT", "FIF_J2K"),
	FORMAT_JP2 = ffi.cast("enum FREE_IMAGE_FORMAT", "FIF_JP2"),
	FORMAT_PFM = ffi.cast("enum FREE_IMAGE_FORMAT", "FIF_PFM"),
	FORMAT_PICT = ffi.cast("enum FREE_IMAGE_FORMAT", "FIF_PICT"),
	FORMAT_RAW = ffi.cast("enum FREE_IMAGE_FORMAT", "FIF_RAW"),
	FORMAT_WEBP = ffi.cast("enum FREE_IMAGE_FORMAT", "FIF_WEBP"),
	FORMAT_JXR = ffi.cast("enum FREE_IMAGE_FORMAT", "FIF_JXR"),
	METADATA_TYPE_NOTYPE = ffi.cast("enum FREE_IMAGE_MDTYPE", "FIDT_NOTYPE"),
	METADATA_TYPE_BYTE = ffi.cast("enum FREE_IMAGE_MDTYPE", "FIDT_BYTE"),
	METADATA_TYPE_ASCII = ffi.cast("enum FREE_IMAGE_MDTYPE", "FIDT_ASCII"),
	METADATA_TYPE_SHORT = ffi.cast("enum FREE_IMAGE_MDTYPE", "FIDT_SHORT"),
	METADATA_TYPE_LONG = ffi.cast("enum FREE_IMAGE_MDTYPE", "FIDT_LONG"),
	METADATA_TYPE_RATIONAL = ffi.cast("enum FREE_IMAGE_MDTYPE", "FIDT_RATIONAL"),
	METADATA_TYPE_SBYTE = ffi.cast("enum FREE_IMAGE_MDTYPE", "FIDT_SBYTE"),
	METADATA_TYPE_UNDEFINED = ffi.cast("enum FREE_IMAGE_MDTYPE", "FIDT_UNDEFINED"),
	METADATA_TYPE_SSHORT = ffi.cast("enum FREE_IMAGE_MDTYPE", "FIDT_SSHORT"),
	METADATA_TYPE_SLONG = ffi.cast("enum FREE_IMAGE_MDTYPE", "FIDT_SLONG"),
	METADATA_TYPE_SRATIONAL = ffi.cast("enum FREE_IMAGE_MDTYPE", "FIDT_SRATIONAL"),
	METADATA_TYPE_FLOAT = ffi.cast("enum FREE_IMAGE_MDTYPE", "FIDT_FLOAT"),
	METADATA_TYPE_DOUBLE = ffi.cast("enum FREE_IMAGE_MDTYPE", "FIDT_DOUBLE"),
	METADATA_TYPE_IFD = ffi.cast("enum FREE_IMAGE_MDTYPE", "FIDT_IFD"),
	METADATA_TYPE_PALETTE = ffi.cast("enum FREE_IMAGE_MDTYPE", "FIDT_PALETTE"),
	METADATA_TYPE_LONG8 = ffi.cast("enum FREE_IMAGE_MDTYPE", "FIDT_LONG8"),
	METADATA_TYPE_SLONG8 = ffi.cast("enum FREE_IMAGE_MDTYPE", "FIDT_SLONG8"),
	METADATA_TYPE_IFD8 = ffi.cast("enum FREE_IMAGE_MDTYPE", "FIDT_IFD8"),
	DITHER_FS = ffi.cast("enum FREE_IMAGE_DITHER", "FID_FS"),
	DITHER_BAYER4x4 = ffi.cast("enum FREE_IMAGE_DITHER", "FID_BAYER4x4"),
	DITHER_BAYER8x8 = ffi.cast("enum FREE_IMAGE_DITHER", "FID_BAYER8x8"),
	DITHER_CLUSTER6x6 = ffi.cast("enum FREE_IMAGE_DITHER", "FID_CLUSTER6x6"),
	DITHER_CLUSTER8x8 = ffi.cast("enum FREE_IMAGE_DITHER", "FID_CLUSTER8x8"),
	DITHER_CLUSTER16x16 = ffi.cast("enum FREE_IMAGE_DITHER", "FID_CLUSTER16x16"),
	DITHER_BAYER16x16 = ffi.cast("enum FREE_IMAGE_DITHER", "FID_BAYER16x16"),
	METADATA_NODATA = ffi.cast("enum FREE_IMAGE_MDMODEL", "FIMD_NODATA"),
	METADATA_COMMENTS = ffi.cast("enum FREE_IMAGE_MDMODEL", "FIMD_COMMENTS"),
	METADATA_EXIF_MAIN = ffi.cast("enum FREE_IMAGE_MDMODEL", "FIMD_EXIF_MAIN"),
	METADATA_EXIF_EXIF = ffi.cast("enum FREE_IMAGE_MDMODEL", "FIMD_EXIF_EXIF"),
	METADATA_EXIF_GPS = ffi.cast("enum FREE_IMAGE_MDMODEL", "FIMD_EXIF_GPS"),
	METADATA_EXIF_MAKERNOTE = ffi.cast("enum FREE_IMAGE_MDMODEL", "FIMD_EXIF_MAKERNOTE"),
	METADATA_EXIF_INTEROP = ffi.cast("enum FREE_IMAGE_MDMODEL", "FIMD_EXIF_INTEROP"),
	METADATA_IPTC = ffi.cast("enum FREE_IMAGE_MDMODEL", "FIMD_IPTC"),
	METADATA_XMP = ffi.cast("enum FREE_IMAGE_MDMODEL", "FIMD_XMP"),
	METADATA_GEOTIFF = ffi.cast("enum FREE_IMAGE_MDMODEL", "FIMD_GEOTIFF"),
	METADATA_ANIMATION = ffi.cast("enum FREE_IMAGE_MDMODEL", "FIMD_ANIMATION"),
	METADATA_CUSTOM = ffi.cast("enum FREE_IMAGE_MDMODEL", "FIMD_CUSTOM"),
	METADATA_EXIF_RAW = ffi.cast("enum FREE_IMAGE_MDMODEL", "FIMD_EXIF_RAW"),
	IMAGE_TYPE_UNKNOWN = ffi.cast("enum FREE_IMAGE_TYPE", "FIT_UNKNOWN"),
	IMAGE_TYPE_BITMAP = ffi.cast("enum FREE_IMAGE_TYPE", "FIT_BITMAP"),
	IMAGE_TYPE_UINT16 = ffi.cast("enum FREE_IMAGE_TYPE", "FIT_UINT16"),
	IMAGE_TYPE_INT16 = ffi.cast("enum FREE_IMAGE_TYPE", "FIT_INT16"),
	IMAGE_TYPE_UINT32 = ffi.cast("enum FREE_IMAGE_TYPE", "FIT_UINT32"),
	IMAGE_TYPE_INT32 = ffi.cast("enum FREE_IMAGE_TYPE", "FIT_INT32"),
	IMAGE_TYPE_FLOAT = ffi.cast("enum FREE_IMAGE_TYPE", "FIT_FLOAT"),
	IMAGE_TYPE_DOUBLE = ffi.cast("enum FREE_IMAGE_TYPE", "FIT_DOUBLE"),
	IMAGE_TYPE_COMPLEX = ffi.cast("enum FREE_IMAGE_TYPE", "FIT_COMPLEX"),
	IMAGE_TYPE_RGB16 = ffi.cast("enum FREE_IMAGE_TYPE", "FIT_RGB16"),
	IMAGE_TYPE_RGBA16 = ffi.cast("enum FREE_IMAGE_TYPE", "FIT_RGBA16"),
	IMAGE_TYPE_RGBF = ffi.cast("enum FREE_IMAGE_TYPE", "FIT_RGBF"),
	IMAGE_TYPE_RGBAF = ffi.cast("enum FREE_IMAGE_TYPE", "FIT_RGBAF"),
	COLOR_TYPE_MINISWHITE = ffi.cast("enum FREE_IMAGE_COLOR_TYPE", "FIC_MINISWHITE"),
	COLOR_TYPE_MINISBLACK = ffi.cast("enum FREE_IMAGE_COLOR_TYPE", "FIC_MINISBLACK"),
	COLOR_TYPE_RGB = ffi.cast("enum FREE_IMAGE_COLOR_TYPE", "FIC_RGB"),
	COLOR_TYPE_PALETTE = ffi.cast("enum FREE_IMAGE_COLOR_TYPE", "FIC_PALETTE"),
	COLOR_TYPE_RGBALPHA = ffi.cast("enum FREE_IMAGE_COLOR_TYPE", "FIC_RGBALPHA"),
	COLOR_TYPE_CMYK = ffi.cast("enum FREE_IMAGE_COLOR_TYPE", "FIC_CMYK"),
	COLOR_CHANNEL_RGB = ffi.cast("enum FREE_IMAGE_COLOR_CHANNEL", "FICC_RGB"),
	COLOR_CHANNEL_RED = ffi.cast("enum FREE_IMAGE_COLOR_CHANNEL", "FICC_RED"),
	COLOR_CHANNEL_GREEN = ffi.cast("enum FREE_IMAGE_COLOR_CHANNEL", "FICC_GREEN"),
	COLOR_CHANNEL_BLUE = ffi.cast("enum FREE_IMAGE_COLOR_CHANNEL", "FICC_BLUE"),
	COLOR_CHANNEL_ALPHA = ffi.cast("enum FREE_IMAGE_COLOR_CHANNEL", "FICC_ALPHA"),
	COLOR_CHANNEL_BLACK = ffi.cast("enum FREE_IMAGE_COLOR_CHANNEL", "FICC_BLACK"),
	COLOR_CHANNEL_REAL = ffi.cast("enum FREE_IMAGE_COLOR_CHANNEL", "FICC_REAL"),
	COLOR_CHANNEL_IMAG = ffi.cast("enum FREE_IMAGE_COLOR_CHANNEL", "FICC_IMAG"),
	COLOR_CHANNEL_MAG = ffi.cast("enum FREE_IMAGE_COLOR_CHANNEL", "FICC_MAG"),
	COLOR_CHANNEL_PHASE = ffi.cast("enum FREE_IMAGE_COLOR_CHANNEL", "FICC_PHASE"),
	JPEG_OPERATION_NONE = ffi.cast("enum FREE_IMAGE_JPEG_OPERATION", "FIJPEG_OP_NONE"),
	JPEG_OPERATION_FLIP_H = ffi.cast("enum FREE_IMAGE_JPEG_OPERATION", "FIJPEG_OP_FLIP_H"),
	JPEG_OPERATION_FLIP_V = ffi.cast("enum FREE_IMAGE_JPEG_OPERATION", "FIJPEG_OP_FLIP_V"),
	JPEG_OPERATION_TRANSPOSE = ffi.cast("enum FREE_IMAGE_JPEG_OPERATION", "FIJPEG_OP_TRANSPOSE"),
	JPEG_OPERATION_TRANSVERSE = ffi.cast("enum FREE_IMAGE_JPEG_OPERATION", "FIJPEG_OP_TRANSVERSE"),
	JPEG_OPERATION_ROTATE_90 = ffi.cast("enum FREE_IMAGE_JPEG_OPERATION", "FIJPEG_OP_ROTATE_90"),
	JPEG_OPERATION_ROTATE_180 = ffi.cast("enum FREE_IMAGE_JPEG_OPERATION", "FIJPEG_OP_ROTATE_180"),
	JPEG_OPERATION_ROTATE_270 = ffi.cast("enum FREE_IMAGE_JPEG_OPERATION", "FIJPEG_OP_ROTATE_270"),
	IMAGE_FILTER_BOX = ffi.cast("enum FREE_IMAGE_FILTER", "FILTER_BOX"),
	IMAGE_FILTER_BICUBIC = ffi.cast("enum FREE_IMAGE_FILTER", "FILTER_BICUBIC"),
	IMAGE_FILTER_BILINEAR = ffi.cast("enum FREE_IMAGE_FILTER", "FILTER_BILINEAR"),
	IMAGE_FILTER_BSPLINE = ffi.cast("enum FREE_IMAGE_FILTER", "FILTER_BSPLINE"),
	IMAGE_FILTER_CATMULLROM = ffi.cast("enum FREE_IMAGE_FILTER", "FILTER_CATMULLROM"),
	IMAGE_FILTER_LANCZOS3 = ffi.cast("enum FREE_IMAGE_FILTER", "FILTER_LANCZOS3"),
	TONEMAP_OPERATOR_DRAGO03 = ffi.cast("enum FREE_IMAGE_TMO", "FITMO_DRAGO03"),
	TONEMAP_OPERATOR_REINHARD05 = ffi.cast("enum FREE_IMAGE_TMO", "FITMO_REINHARD05"),
	TONEMAP_OPERATOR_FATTAL02 = ffi.cast("enum FREE_IMAGE_TMO", "FITMO_FATTAL02"),
	QUANTIZE_WUQUANT = ffi.cast("enum FREE_IMAGE_QUANTIZE", "FIQ_WUQUANT"),
	QUANTIZE_NNQUANT = ffi.cast("enum FREE_IMAGE_QUANTIZE", "FIQ_NNQUANT"),
	QUANTIZE_LFPQUANT = ffi.cast("enum FREE_IMAGE_QUANTIZE", "FIQ_LFPQUANT"),
}
do
	do
		local function pow2ceil(n)
			return 2 ^ math.ceil(math.log(n) / math.log(2))
		end

		local function create_mip_map(bitmap, w, h, div)
			local width = pow2ceil(w)
			local height = pow2ceil(h)

			local size = width > height and width or height

			size = size / (2 ^ div)

			local new_bitmap = ffi.gc(library.Rescale(bitmap, size, size, library.e.IMAGE_FILTER_BILINEAR), library.Unload)

			return {
				data = library.GetBits(new_bitmap),
				size = library.GetMemorySize(new_bitmap),
				width = size,
				height = size,
				new_bitmap = new_bitmap,
			}
		end

		function library.LoadImageMipMaps(file_name, flags, format)
			local file = io.open(file_name, "rb")
			local data = file:read("*all")
			file:close()

			local buffer = ffi.cast("unsigned char *", data)

			local stream = library.OpenMemory(buffer, #data)
			local type = format or library.GetFileTypeFromMemory(stream, #data)

			local temp = library.LoadFromMemory(type, stream, flags or 0)
			local bitmap = library.ConvertTo32Bits(temp)


			local width = library.GetWidth(bitmap)
			local height = library.GetHeight(bitmap)

			local images = {}

			for level = 0, math.floor(math.log(math.max(width, height)) / math.log(2)) do
				images[level] = create_mip_map(bitmap, width, height, level)
			end

			library.Unload(bitmap)
			library.Unload(temp)

			library.CloseMemory(stream)

			return images
		end
	end

	function library.LoadImage(data)
		local stream_buffer = ffi.cast("unsigned char *", data)
		local stream = library.OpenMemory(stream_buffer, #data)

		local type = library.GetFileTypeFromMemory(stream, #data)

		if type == library.e.FORMAT_UNKNOWN or type > library.e.FORMAT_RAW then -- huh...
			library.CloseMemory(stream)
			error("unknown format", 2)
		end

		local bitmap = library.LoadFromMemory(type, stream, 0)

		local image_type = library.GetImageType(bitmap)
		local color_type = library.GetColorType(bitmap)

		stream_buffer = nil

		local format = "bgra"
		local type = "unsigned_byte"

		if color_type == library.e.COLOR_TYPE_RGBALPHA then
			format = "bgra"
		elseif color_type == library.e.COLOR_TYPE_RGB then
			format = "bgr"
		elseif color_type == library.e.COLOR_TYPE_MINISBLACK or color_type == library.e.COLOR_TYPE_MINISWHITE then
			format = "r"
		else
			bitmap = library.ConvertTo32Bits(bitmap)

			format = "bgra"
			wlog("unhandled freeimage color type: %s\nconverting to 8bit rgba", color_type)
		end

		ffi.gc(bitmap, library.Unload)

		if image_type == library.e.IMAGE_TYPE_BITMAP then
			type = "unsigned_byte"
		elseif image_type == library.e.IMAGE_TYPE_RGBF then
			type = "float"
			format = "rgb"
		elseif image_type == library.e.IMAGE_TYPE_RGBAF then
			type = "float"
			format = "rgba"
		else
			wlog("unhandled freeimage format type: %s", image_type)
		end

		-- the image type of some png images are RGB but bpp is actuall 32bit (RGBA)
		local bpp = library.GetBPP(bitmap)

		if bpp == 32 then
			format = "bgra"
		end

		local ret = {
			buffer = library.GetBits(bitmap),
			width = library.GetWidth(bitmap),
			height = library.GetHeight(bitmap),
			format = format,
			type = type,
		}

		library.CloseMemory(stream)

		return ret
	end

	function library.LoadMultiPageImage(data, flags)
		local buffer = ffi.cast("unsigned char *", data)

		local stream = library.OpenMemory(buffer, #data)
		local type = library.GetFileTypeFromMemory(stream, #data)

		local temp = library.LoadMultiBitmapFromMemory(type, stream, flags or 0)
		local count = library.GetPageCount(temp)

		local out = {}

		for page = 0, count - 1 do
			local temp = library.LockPage(temp, page)
			local bitmap = library.ConvertTo32Bits(temp)

			local tag = ffi.new("struct FITAG *[1]")
			library.GetMetadata(library.e.METADATA_ANIMATION, bitmap, "FrameLeft", tag)
			local x = tonumber(ffi.cast("int", library.GetTagValue(tag[0])))

			library.GetMetadata(library.e.METADATA_ANIMATION, bitmap, "FrameTop", tag)
			local y = tonumber(ffi.cast("int", library.GetTagValue(tag[0])))

			library.GetMetadata(library.e.METADATA_ANIMATION, bitmap, "FrameTime", tag)
			local ms = tonumber(ffi.cast("int", library.GetTagValue(tag[0]))) / 1000

			library.DeleteTag(tag[0])

			local data = library.GetBits(bitmap)
			local width = library.GetWidth(bitmap)
			local height = library.GetHeight(bitmap)

			ffi.gc(bitmap, library.Unload)

			table.insert(out, {w = width, h = height, x = x, y = y, ms = ms, data = data})
		end

		library.CloseMultiBitmap(temp, 0)

		return out
	end

	function library.ImageToBuffer(data, format, force_32bit)
		format = format or "png"

		local bitmap = library.ConvertFromRawBits(data.buffer, data.width, data.height, data.width * #data.format, #data.format * 8, 0,0,0,0)
		local temp
		if force_32bit then
			temp = bitmap
			bitmap = library.ConvertTo32Bits(temp)
		end

		local mem = library.OpenMemory(nil, 0)
		library.SaveToMemory(library.e["FORMAT_" .. format:upper()], bitmap, mem, 0)
		local size = library.TellMemory(mem)
		local buffer_box = ffi.new("uint8_t *[1]")
		local size_box = ffi.new("unsigned int[1]")
		local out_buffer = ffi.new("uint8_t[?]", size)
		buffer_box[0] = out_buffer
		size_box[0] = size
		library.AcquireMemory(mem, buffer_box, size_box)

		library.Unload(bitmap)
		if temp then library.Unload(temp) end
		library.CloseMemory(mem)

		return buffer_box[0], size_box[0]
	end
end
library.clib = CLIB
return library
