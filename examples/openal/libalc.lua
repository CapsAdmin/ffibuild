local ffi = require("ffi")
ffi.cdef([[struct ALCdevice_struct {};
struct ALCcontext_struct {};
void(alcDestroyContext)(struct ALCcontext_struct*);
void(alcCaptureStop)(struct ALCdevice_struct*);
void(alcGetIntegerv)(struct ALCdevice_struct*,int,int,int*);
const char*(alcGetString)(struct ALCdevice_struct*,int);
struct ALCcontext_struct*(alcGetCurrentContext)();
struct ALCcontext_struct*(alcGetThreadContext)();
int(alcGetError)(struct ALCdevice_struct*);
char(alcResetDeviceSOFT)(struct ALCdevice_struct*,const int*);
void(alcRenderSamplesSOFT)(struct ALCdevice_struct*,void*,int);
void(alcDeviceResumeSOFT)(struct ALCdevice_struct*);
void*(alcGetProcAddress)(struct ALCdevice_struct*,const char*);
struct ALCcontext_struct*(alcCreateContext)(struct ALCdevice_struct*,const int*);
const char*(alcGetStringiSOFT)(struct ALCdevice_struct*,int,int);
void(alcDevicePauseSOFT)(struct ALCdevice_struct*);
char(alcMakeContextCurrent)(struct ALCcontext_struct*);
struct ALCdevice_struct*(alcGetContextsDevice)(struct ALCcontext_struct*);
int(alcGetEnumValue)(struct ALCdevice_struct*,const char*);
char(alcIsRenderFormatSupportedSOFT)(struct ALCdevice_struct*,int,int,int);
void(alcCaptureStart)(struct ALCdevice_struct*);
char(alcSetThreadContext)(struct ALCcontext_struct*);
char(alcCaptureCloseDevice)(struct ALCdevice_struct*);
struct ALCdevice_struct*(alcCaptureOpenDevice)(const char*,unsigned int,int,int);
struct ALCdevice_struct*(alcOpenDevice)(const char*);
char(alcIsExtensionPresent)(struct ALCdevice_struct*,const char*);
void(alcCaptureSamples)(struct ALCdevice_struct*,void*,int);
void(alcProcessContext)(struct ALCcontext_struct*);
char(alcCloseDevice)(struct ALCdevice_struct*);
struct ALCdevice_struct*(alcLoopbackOpenDeviceSOFT)(const char*);
void(alcSuspendContext)(struct ALCcontext_struct*);
]])
local CLIB = ffi.load(_G.FFI_LIB or "openal")
local library = {}
library = {
	DestroyContext = CLIB.alcDestroyContext,
	CaptureStop = CLIB.alcCaptureStop,
	GetIntegerv = CLIB.alcGetIntegerv,
	GetString = CLIB.alcGetString,
	GetCurrentContext = CLIB.alcGetCurrentContext,
	GetThreadContext = CLIB.alcGetThreadContext,
	GetError = CLIB.alcGetError,
	ResetDeviceSOFT = CLIB.alcResetDeviceSOFT,
	RenderSamplesSOFT = CLIB.alcRenderSamplesSOFT,
	DeviceResumeSOFT = CLIB.alcDeviceResumeSOFT,
	GetProcAddress = CLIB.alcGetProcAddress,
	CreateContext = CLIB.alcCreateContext,
	GetStringiSOFT = CLIB.alcGetStringiSOFT,
	DevicePauseSOFT = CLIB.alcDevicePauseSOFT,
	MakeContextCurrent = CLIB.alcMakeContextCurrent,
	GetContextsDevice = CLIB.alcGetContextsDevice,
	GetEnumValue = CLIB.alcGetEnumValue,
	IsRenderFormatSupportedSOFT = CLIB.alcIsRenderFormatSupportedSOFT,
	CaptureStart = CLIB.alcCaptureStart,
	SetThreadContext = CLIB.alcSetThreadContext,
	CaptureCloseDevice = CLIB.alcCaptureCloseDevice,
	CaptureOpenDevice = CLIB.alcCaptureOpenDevice,
	OpenDevice = CLIB.alcOpenDevice,
	IsExtensionPresent = CLIB.alcIsExtensionPresent,
	CaptureSamples = CLIB.alcCaptureSamples,
	ProcessContext = CLIB.alcProcessContext,
	CloseDevice = CLIB.alcCloseDevice,
	LoopbackOpenDeviceSOFT = CLIB.alcLoopbackOpenDeviceSOFT,
	SuspendContext = CLIB.alcSuspendContext,
}
library.e = {
	API = __declspecdllimport,
	API = extern,
	APIENTRY = __cdecl,
	INVALID = 0,
	VERSION_0_1 = 1,
	FALSE = 0,
	TRUE = 1,
	FREQUENCY = 0x1007,
	REFRESH = 0x1008,
	SYNC = 0x1009,
	MONO_SOURCES = 0x1010,
	STEREO_SOURCES = 0x1011,
	NO_ERROR = 0,
	INVALID_DEVICE = 0xA001,
	INVALID_CONTEXT = 0xA002,
	INVALID_ENUM = 0xA003,
	INVALID_VALUE = 0xA004,
	OUT_OF_MEMORY = 0xA005,
	MAJOR_VERSION = 0x1000,
	MINOR_VERSION = 0x1001,
	ATTRIBUTES_SIZE = 0x1002,
	ALL_ATTRIBUTES = 0x1003,
	DEFAULT_DEVICE_SPECIFIER = 0x1004,
	DEVICE_SPECIFIER = 0x1005,
	EXTENSIONS = 0x1006,
	EXT_CAPTURE = 1,
	CAPTURE_DEVICE_SPECIFIER = 0x310,
	CAPTURE_DEFAULT_DEVICE_SPECIFIER = 0x311,
	CAPTURE_SAMPLES = 0x312,
	ENUMERATE_ALL_EXT = 1,
	DEFAULT_ALL_DEVICES_SPECIFIER = 0x1012,
	ALL_DEVICES_SPECIFIER = 0x1013,
	LOKI_audio_channel = 1,
	CHAN_MAIN_LOKI = 0x500001,
	CHAN_PCM_LOKI = 0x500002,
	CHAN_CD_LOKI = 0x500003,
	EXT_EFX = 1,
	EXT_disconnect = 1,
	CONNECTED = 0x313,
	EXT_thread_local_context = 1,
	EXT_DEDICATED = 1,
	SOFT_loopback = 1,
	FORMAT_CHANNELS_SOFT = 0x1990,
	FORMAT_TYPE_SOFT = 0x1991,
	BYTE_SOFT = 0x1400,
	UNSIGNED_BYTE_SOFT = 0x1401,
	SHORT_SOFT = 0x1402,
	UNSIGNED_SHORT_SOFT = 0x1403,
	INT_SOFT = 0x1404,
	UNSIGNED_INT_SOFT = 0x1405,
	FLOAT_SOFT = 0x1406,
	MONO_SOFT = 0x1500,
	STEREO_SOFT = 0x1501,
	QUAD_SOFT = 0x1503,
	_5POINT1_SOFT = 0x1504,
	_6POINT1_SOFT = 0x1505,
	_7POINT1_SOFT = 0x1506,
	EXT_DEFAULT_FILTER_ORDER = 1,
	DEFAULT_FILTER_ORDER = 0x1100,
	SOFT_pause_device = 1,
	SOFT_HRTF = 1,
	HRTF_SOFT = 0x1992,
	DONT_CARE_SOFT = 0x0002,
	HRTF_STATUS_SOFT = 0x1993,
	HRTF_DISABLED_SOFT = 0x0000,
	HRTF_ENABLED_SOFT = 0x0001,
	HRTF_DENIED_SOFT = 0x0002,
	HRTF_REQUIRED_SOFT = 0x0003,
	HRTF_HEADPHONES_DETECTED_SOFT = 0x0004,
	HRTF_UNSUPPORTED_FORMAT_SOFT = 0x0005,
	NUM_HRTF_SPECIFIERS_SOFT = 0x1994,
	HRTF_SPECIFIER_SOFT = 0x1995,
	HRTF_ID_SOFT = 0x1996,
	EXT_EFX_NAME = "ALC_EXT_EFX",
	EFX_MAJOR_VERSION = 0x20001,
	EFX_MINOR_VERSION = 0x20002,
	MAX_AUXILIARY_SENDS = 0x20003,
}
library.clib = CLIB
return library
