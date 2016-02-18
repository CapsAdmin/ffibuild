local ffi = require("ffi")
ffi.cdef([[typedef enum GLFWenum{GLFW_VERSION_MAJOR=3,GLFW_VERSION_MINOR=2,GLFW_VERSION_REVISION=0,GLFW_TRUE=1,GLFW_FALSE=0,GLFW_RELEASE=0,GLFW_PRESS=1,GLFW_REPEAT=2,GLFW_KEY_UNKNOWN=-1,GLFW_KEY_SPACE=32,GLFW_KEY_APOSTROPHE=39,GLFW_KEY_COMMA=44,GLFW_KEY_MINUS=45,GLFW_KEY_PERIOD=46,GLFW_KEY_SLASH=47,GLFW_KEY_0=48,GLFW_KEY_1=49,GLFW_KEY_2=50,GLFW_KEY_3=51,GLFW_KEY_4=52,GLFW_KEY_5=53,GLFW_KEY_6=54,GLFW_KEY_7=55,GLFW_KEY_8=56,GLFW_KEY_9=57,GLFW_KEY_SEMICOLON=59,GLFW_KEY_EQUAL=61,GLFW_KEY_A=65,GLFW_KEY_B=66,GLFW_KEY_C=67,GLFW_KEY_D=68,GLFW_KEY_E=69,GLFW_KEY_F=70,GLFW_KEY_G=71,GLFW_KEY_H=72,GLFW_KEY_I=73,GLFW_KEY_J=74,GLFW_KEY_K=75,GLFW_KEY_L=76,GLFW_KEY_M=77,GLFW_KEY_N=78,GLFW_KEY_O=79,GLFW_KEY_P=80,GLFW_KEY_Q=81,GLFW_KEY_R=82,GLFW_KEY_S=83,GLFW_KEY_T=84,GLFW_KEY_U=85,GLFW_KEY_V=86,GLFW_KEY_W=87,GLFW_KEY_X=88,GLFW_KEY_Y=89,GLFW_KEY_Z=90,GLFW_KEY_LEFT_BRACKET=91,GLFW_KEY_BACKSLASH=92,GLFW_KEY_RIGHT_BRACKET=93,GLFW_KEY_GRAVE_ACCENT=96,GLFW_KEY_WORLD_1=161,GLFW_KEY_WORLD_2=162,GLFW_KEY_ESCAPE=256,GLFW_KEY_ENTER=257,GLFW_KEY_TAB=258,GLFW_KEY_BACKSPACE=259,GLFW_KEY_INSERT=260,GLFW_KEY_DELETE=261,GLFW_KEY_RIGHT=262,GLFW_KEY_LEFT=263,GLFW_KEY_DOWN=264,GLFW_KEY_UP=265,GLFW_KEY_PAGE_UP=266,GLFW_KEY_PAGE_DOWN=267,GLFW_KEY_HOME=268,GLFW_KEY_END=269,GLFW_KEY_CAPS_LOCK=280,GLFW_KEY_SCROLL_LOCK=281,GLFW_KEY_NUM_LOCK=282,GLFW_KEY_PRINT_SCREEN=283,GLFW_KEY_PAUSE=284,GLFW_KEY_F1=290,GLFW_KEY_F2=291,GLFW_KEY_F3=292,GLFW_KEY_F4=293,GLFW_KEY_F5=294,GLFW_KEY_F6=295,GLFW_KEY_F7=296,GLFW_KEY_F8=297,GLFW_KEY_F9=298,GLFW_KEY_F10=299,GLFW_KEY_F11=300,GLFW_KEY_F12=301,GLFW_KEY_F13=302,GLFW_KEY_F14=303,GLFW_KEY_F15=304,GLFW_KEY_F16=305,GLFW_KEY_F17=306,GLFW_KEY_F18=307,GLFW_KEY_F19=308,GLFW_KEY_F20=309,GLFW_KEY_F21=310,GLFW_KEY_F22=311,GLFW_KEY_F23=312,GLFW_KEY_F24=313,GLFW_KEY_F25=314,GLFW_KEY_KP_0=320,GLFW_KEY_KP_1=321,GLFW_KEY_KP_2=322,GLFW_KEY_KP_3=323,GLFW_KEY_KP_4=324,GLFW_KEY_KP_5=325,GLFW_KEY_KP_6=326,GLFW_KEY_KP_7=327,GLFW_KEY_KP_8=328,GLFW_KEY_KP_9=329,GLFW_KEY_KP_DECIMAL=330,GLFW_KEY_KP_DIVIDE=331,GLFW_KEY_KP_MULTIPLY=332,GLFW_KEY_KP_SUBTRACT=333,GLFW_KEY_KP_ADD=334,GLFW_KEY_KP_ENTER=335,GLFW_KEY_KP_EQUAL=336,GLFW_KEY_LEFT_SHIFT=340,GLFW_KEY_LEFT_CONTROL=341,GLFW_KEY_LEFT_ALT=342,GLFW_KEY_LEFT_SUPER=343,GLFW_KEY_RIGHT_SHIFT=344,GLFW_KEY_RIGHT_CONTROL=345,GLFW_KEY_RIGHT_ALT=346,GLFW_KEY_RIGHT_SUPER=347,GLFW_KEY_MENU=348,GLFW_KEY_LAST=348,GLFW_MOD_SHIFT=1,GLFW_MOD_CONTROL=2,GLFW_MOD_ALT=4,GLFW_MOD_SUPER=8,GLFW_MOUSE_BUTTON_1=0,GLFW_MOUSE_BUTTON_2=1,GLFW_MOUSE_BUTTON_3=2,GLFW_MOUSE_BUTTON_4=3,GLFW_MOUSE_BUTTON_5=4,GLFW_MOUSE_BUTTON_6=5,GLFW_MOUSE_BUTTON_7=6,GLFW_MOUSE_BUTTON_8=7,GLFW_MOUSE_BUTTON_LAST=7,GLFW_MOUSE_BUTTON_LEFT=0,GLFW_MOUSE_BUTTON_RIGHT=1,GLFW_MOUSE_BUTTON_MIDDLE=2,GLFW_JOYSTICK_1=0,GLFW_JOYSTICK_2=1,GLFW_JOYSTICK_3=2,GLFW_JOYSTICK_4=3,GLFW_JOYSTICK_5=4,GLFW_JOYSTICK_6=5,GLFW_JOYSTICK_7=6,GLFW_JOYSTICK_8=7,GLFW_JOYSTICK_9=8,GLFW_JOYSTICK_10=9,GLFW_JOYSTICK_11=10,GLFW_JOYSTICK_12=11,GLFW_JOYSTICK_13=12,GLFW_JOYSTICK_14=13,GLFW_JOYSTICK_15=14,GLFW_JOYSTICK_16=15,GLFW_JOYSTICK_LAST=15,GLFW_NOT_INITIALIZED=65537,GLFW_NO_CURRENT_CONTEXT=65538,GLFW_INVALID_ENUM=65539,GLFW_INVALID_VALUE=65540,GLFW_OUT_OF_MEMORY=65541,GLFW_API_UNAVAILABLE=65542,GLFW_VERSION_UNAVAILABLE=65543,GLFW_PLATFORM_ERROR=65544,GLFW_FORMAT_UNAVAILABLE=65545,GLFW_NO_WINDOW_CONTEXT=65546,GLFW_FOCUSED=131073,GLFW_ICONIFIED=131074,GLFW_RESIZABLE=131075,GLFW_VISIBLE=131076,GLFW_DECORATED=131077,GLFW_AUTO_ICONIFY=131078,GLFW_FLOATING=131079,GLFW_RED_BITS=135169,GLFW_GREEN_BITS=135170,GLFW_BLUE_BITS=135171,GLFW_ALPHA_BITS=135172,GLFW_DEPTH_BITS=135173,GLFW_STENCIL_BITS=135174,GLFW_ACCUM_RED_BITS=135175,GLFW_ACCUM_GREEN_BITS=135176,GLFW_ACCUM_BLUE_BITS=135177,GLFW_ACCUM_ALPHA_BITS=135178,GLFW_AUX_BUFFERS=135179,GLFW_STEREO=135180,GLFW_SAMPLES=135181,GLFW_SRGB_CAPABLE=135182,GLFW_REFRESH_RATE=135183,GLFW_DOUBLEBUFFER=135184,GLFW_CLIENT_API=139265,GLFW_CONTEXT_VERSION_MAJOR=139266,GLFW_CONTEXT_VERSION_MINOR=139267,GLFW_CONTEXT_REVISION=139268,GLFW_CONTEXT_ROBUSTNESS=139269,GLFW_OPENGL_FORWARD_COMPAT=139270,GLFW_OPENGL_DEBUG_CONTEXT=139271,GLFW_OPENGL_PROFILE=139272,GLFW_CONTEXT_RELEASE_BEHAVIOR=139273,GLFW_CONTEXT_NO_ERROR=139274,GLFW_NO_API=0,GLFW_OPENGL_API=196609,GLFW_OPENGL_ES_API=196610,GLFW_NO_ROBUSTNESS=0,GLFW_NO_RESET_NOTIFICATION=200705,GLFW_LOSE_CONTEXT_ON_RESET=200706,GLFW_OPENGL_ANY_PROFILE=0,GLFW_OPENGL_CORE_PROFILE=204801,GLFW_OPENGL_COMPAT_PROFILE=204802,GLFW_CURSOR=208897,GLFW_STICKY_KEYS=208898,GLFW_STICKY_MOUSE_BUTTONS=208899,GLFW_CURSOR_NORMAL=212993,GLFW_CURSOR_HIDDEN=212994,GLFW_CURSOR_DISABLED=212995,GLFW_ANY_RELEASE_BEHAVIOR=0,GLFW_RELEASE_BEHAVIOR_FLUSH=217089,GLFW_RELEASE_BEHAVIOR_NONE=217090,GLFW_ARROW_CURSOR=221185,GLFW_IBEAM_CURSOR=221186,GLFW_CROSSHAIR_CURSOR=221187,GLFW_HAND_CURSOR=221188,GLFW_HRESIZE_CURSOR=221189,GLFW_VRESIZE_CURSOR=221190,GLFW_CONNECTED=262145,GLFW_DISCONNECTED=262146,GLFW_DONT_CARE=-1};
struct GLFWmonitor {};
struct GLFWwindow {};
struct GLFWcursor {};
struct GLFWvidmode {int width;int height;int redBits;int greenBits;int blueBits;int refreshRate;};
struct GLFWgammaramp {unsigned short*red;unsigned short*green;unsigned short*blue;unsigned int size;};
struct GLFWimage {int width;int height;unsigned char*pixels;};
void glfwRestoreWindow(struct GLFWwindow*);
void glfwDestroyCursor(struct GLFWcursor*);
const char*glfwGetVersionString();
void*glfwGetWindowUserPointer(struct GLFWwindow*);
void glfwDefaultWindowHints();
void glfwSetWindowPos(struct GLFWwindow*,int,int);
void(*glfwSetWindowFocusCallback(struct GLFWwindow*,void(*)(struct GLFWwindow*,int)))(struct GLFWwindow*,int);
void(*glfwSetWindowRefreshCallback(struct GLFWwindow*,void(*)(struct GLFWwindow*)))(struct GLFWwindow*);
void glfwSetWindowShouldClose(struct GLFWwindow*,int);
void glfwDestroyWindow(struct GLFWwindow*);
void glfwSetCursorPos(struct GLFWwindow*,double,double);
void glfwGetCursorPos(struct GLFWwindow*,double*,double*);
void(*glfwSetCharCallback(struct GLFWwindow*,void(*)(struct GLFWwindow*,unsigned int)))(struct GLFWwindow*,unsigned int);
int glfwGetPhysicalDevicePresentationSupport(void*,void*,unsigned int);
void(*glfwSetErrorCallback(void(*)(int,const char*)))(int,const char*);
int glfwWindowShouldClose(struct GLFWwindow*);
void(*glfwSetWindowCloseCallback(struct GLFWwindow*,void(*)(struct GLFWwindow*)))(struct GLFWwindow*);
void glfwHideWindow(struct GLFWwindow*);
void glfwTerminate();
const char*glfwGetClipboardString(struct GLFWwindow*);
void(*glfwSetWindowIconifyCallback(struct GLFWwindow*,void(*)(struct GLFWwindow*,int)))(struct GLFWwindow*,int);
void glfwGetVersion(int*,int*,int*);
void(*glfwSetMonitorCallback(void(*)(struct GLFWmonitor*,int)))(struct GLFWmonitor*,int);
void(*glfwSetKeyCallback(struct GLFWwindow*,void(*)(struct GLFWwindow*,int,int,int,int)))(struct GLFWwindow*,int,int,int,int);
void glfwSetGammaRamp(struct GLFWmonitor*,const struct GLFWgammaramp*);
void glfwSetWindowAspectRatio(struct GLFWwindow*,int,int);
const float*glfwGetJoystickAxes(int,int*);
int glfwGetWindowAttrib(struct GLFWwindow*,int);
struct GLFWwindow*glfwCreateWindow(int,int,const char*,struct GLFWmonitor*,struct GLFWwindow*);
void(*glfwSetCursorEnterCallback(struct GLFWwindow*,void(*)(struct GLFWwindow*,int)))(struct GLFWwindow*,int);
void glfwSetCursor(struct GLFWwindow*,struct GLFWcursor*);
void(*glfwSetFramebufferSizeCallback(struct GLFWwindow*,void(*)(struct GLFWwindow*,int,int)))(struct GLFWwindow*,int,int);
void glfwPostEmptyEvent();
const char*glfwGetMonitorName(struct GLFWmonitor*);
const struct GLFWvidmode*glfwGetVideoMode(struct GLFWmonitor*);
void glfwSetWindowTitle(struct GLFWwindow*,const char*);
void glfwSetInputMode(struct GLFWwindow*,int,int);
int glfwJoystickPresent(int);
void glfwGetMonitorPos(struct GLFWmonitor*,int*,int*);
void glfwShowWindow(struct GLFWwindow*);
const struct GLFWvidmode*glfwGetVideoModes(struct GLFWmonitor*,int*);
void(*glfwSetMouseButtonCallback(struct GLFWwindow*,void(*)(struct GLFWwindow*,int,int,int)))(struct GLFWwindow*,int,int,int);
void(*glfwGetProcAddress(const char*))();
void glfwSetWindowSize(struct GLFWwindow*,int,int);
void(*glfwSetDropCallback(struct GLFWwindow*,void(*)(struct GLFWwindow*,int,const char**)))(struct GLFWwindow*,int,const char**);
struct GLFWmonitor*glfwGetWindowMonitor(struct GLFWwindow*);
double glfwGetTime();
void glfwGetWindowFrameSize(struct GLFWwindow*,int*,int*,int*,int*);
const char*glfwGetJoystickName(int);
void(*glfwSetWindowPosCallback(struct GLFWwindow*,void(*)(struct GLFWwindow*,int,int)))(struct GLFWwindow*,int,int);
unsigned int glfwCreateWindowSurface(void*,struct GLFWwindow*,const void*,void*);
void(*glfwSetWindowSizeCallback(struct GLFWwindow*,void(*)(struct GLFWwindow*,int,int)))(struct GLFWwindow*,int,int);
void*glfwGetInstanceProcAddress(void*,const char*);
const char**glfwGetRequiredInstanceExtensions(int*);
int glfwVulkanSupported();
void glfwSwapInterval(int);
void glfwSwapBuffers(struct GLFWwindow*);
struct GLFWwindow*glfwGetCurrentContext();
void glfwMakeContextCurrent(struct GLFWwindow*);
void glfwSetTime(double);
const unsigned char*glfwGetJoystickButtons(int,int*);
struct GLFWcursor*glfwCreateCursor(const struct GLFWimage*,int,int);
int glfwGetKey(struct GLFWwindow*,int);
const char*glfwGetKeyName(int,int);
void glfwSetWindowUserPointer(struct GLFWwindow*,void*);
void glfwSetWindowSizeLimits(struct GLFWwindow*,int,int,int,int);
void glfwGetWindowPos(struct GLFWwindow*,int*,int*);
void glfwWaitEvents();
int glfwExtensionSupported(const char*);
int glfwGetMouseButton(struct GLFWwindow*,int);
struct GLFWmonitor**glfwGetMonitors(int*);
void glfwSetGamma(struct GLFWmonitor*,float);
int glfwInit();
struct GLFWcursor*glfwCreateStandardCursor(int);
void glfwPollEvents();
void glfwGetMonitorPhysicalSize(struct GLFWmonitor*,int*,int*);
void(*glfwSetCharModsCallback(struct GLFWwindow*,void(*)(struct GLFWwindow*,unsigned int,int)))(struct GLFWwindow*,unsigned int,int);
void glfwSetClipboardString(struct GLFWwindow*,const char*);
void glfwIconifyWindow(struct GLFWwindow*);
void(*glfwSetCursorPosCallback(struct GLFWwindow*,void(*)(struct GLFWwindow*,double,double)))(struct GLFWwindow*,double,double);
void(*glfwSetScrollCallback(struct GLFWwindow*,void(*)(struct GLFWwindow*,double,double)))(struct GLFWwindow*,double,double);
struct GLFWmonitor*glfwGetPrimaryMonitor();
const struct GLFWgammaramp*glfwGetGammaRamp(struct GLFWmonitor*);
void glfwWindowHint(int,int);
void glfwGetWindowSize(struct GLFWwindow*,int*,int*);
void glfwGetFramebufferSize(struct GLFWwindow*,int*,int*);
int glfwGetInputMode(struct GLFWwindow*,int);
]])
local CLIB = ffi.load(_G.FFI_LIB or "glfw3")
local library = {}
library = {
	RestoreWindow = CLIB.glfwRestoreWindow,
	DestroyCursor = CLIB.glfwDestroyCursor,
	GetVersionString = CLIB.glfwGetVersionString,
	GetWindowUserPointer = CLIB.glfwGetWindowUserPointer,
	DefaultWindowHints = CLIB.glfwDefaultWindowHints,
	SetWindowPos = CLIB.glfwSetWindowPos,
	SetWindowFocusCallback = CLIB.glfwSetWindowFocusCallback,
	SetWindowRefreshCallback = CLIB.glfwSetWindowRefreshCallback,
	SetWindowShouldClose = CLIB.glfwSetWindowShouldClose,
	DestroyWindow = CLIB.glfwDestroyWindow,
	SetCursorPos = CLIB.glfwSetCursorPos,
	GetCursorPos = CLIB.glfwGetCursorPos,
	SetCharCallback = CLIB.glfwSetCharCallback,
	GetPhysicalDevicePresentationSupport = CLIB.glfwGetPhysicalDevicePresentationSupport,
	SetErrorCallback = CLIB.glfwSetErrorCallback,
	WindowShouldClose = CLIB.glfwWindowShouldClose,
	SetWindowCloseCallback = CLIB.glfwSetWindowCloseCallback,
	HideWindow = CLIB.glfwHideWindow,
	Terminate = CLIB.glfwTerminate,
	GetClipboardString = CLIB.glfwGetClipboardString,
	SetWindowIconifyCallback = CLIB.glfwSetWindowIconifyCallback,
	GetVersion = CLIB.glfwGetVersion,
	SetMonitorCallback = CLIB.glfwSetMonitorCallback,
	SetKeyCallback = CLIB.glfwSetKeyCallback,
	SetGammaRamp = CLIB.glfwSetGammaRamp,
	SetWindowAspectRatio = CLIB.glfwSetWindowAspectRatio,
	GetJoystickAxes = CLIB.glfwGetJoystickAxes,
	GetWindowAttrib = CLIB.glfwGetWindowAttrib,
	CreateWindow = CLIB.glfwCreateWindow,
	SetCursorEnterCallback = CLIB.glfwSetCursorEnterCallback,
	SetCursor = CLIB.glfwSetCursor,
	SetFramebufferSizeCallback = CLIB.glfwSetFramebufferSizeCallback,
	PostEmptyEvent = CLIB.glfwPostEmptyEvent,
	GetMonitorName = CLIB.glfwGetMonitorName,
	GetVideoMode = CLIB.glfwGetVideoMode,
	SetWindowTitle = CLIB.glfwSetWindowTitle,
	SetInputMode = CLIB.glfwSetInputMode,
	JoystickPresent = CLIB.glfwJoystickPresent,
	GetMonitorPos = CLIB.glfwGetMonitorPos,
	ShowWindow = CLIB.glfwShowWindow,
	GetVideoModes = CLIB.glfwGetVideoModes,
	SetMouseButtonCallback = CLIB.glfwSetMouseButtonCallback,
	GetProcAddress = CLIB.glfwGetProcAddress,
	SetWindowSize = CLIB.glfwSetWindowSize,
	SetDropCallback = CLIB.glfwSetDropCallback,
	GetWindowMonitor = CLIB.glfwGetWindowMonitor,
	GetTime = CLIB.glfwGetTime,
	GetWindowFrameSize = CLIB.glfwGetWindowFrameSize,
	GetJoystickName = CLIB.glfwGetJoystickName,
	SetWindowPosCallback = CLIB.glfwSetWindowPosCallback,
	CreateWindowSurface = CLIB.glfwCreateWindowSurface,
	SetWindowSizeCallback = CLIB.glfwSetWindowSizeCallback,
	GetInstanceProcAddress = CLIB.glfwGetInstanceProcAddress,
	GetRequiredInstanceExtensions = CLIB.glfwGetRequiredInstanceExtensions,
	VulkanSupported = CLIB.glfwVulkanSupported,
	SwapInterval = CLIB.glfwSwapInterval,
	SwapBuffers = CLIB.glfwSwapBuffers,
	GetCurrentContext = CLIB.glfwGetCurrentContext,
	MakeContextCurrent = CLIB.glfwMakeContextCurrent,
	SetTime = CLIB.glfwSetTime,
	GetJoystickButtons = CLIB.glfwGetJoystickButtons,
	CreateCursor = CLIB.glfwCreateCursor,
	GetKey = CLIB.glfwGetKey,
	GetKeyName = CLIB.glfwGetKeyName,
	SetWindowUserPointer = CLIB.glfwSetWindowUserPointer,
	SetWindowSizeLimits = CLIB.glfwSetWindowSizeLimits,
	GetWindowPos = CLIB.glfwGetWindowPos,
	WaitEvents = CLIB.glfwWaitEvents,
	ExtensionSupported = CLIB.glfwExtensionSupported,
	GetMouseButton = CLIB.glfwGetMouseButton,
	GetMonitors = CLIB.glfwGetMonitors,
	SetGamma = CLIB.glfwSetGamma,
	Init = CLIB.glfwInit,
	CreateStandardCursor = CLIB.glfwCreateStandardCursor,
	PollEvents = CLIB.glfwPollEvents,
	GetMonitorPhysicalSize = CLIB.glfwGetMonitorPhysicalSize,
	SetCharModsCallback = CLIB.glfwSetCharModsCallback,
	SetClipboardString = CLIB.glfwSetClipboardString,
	IconifyWindow = CLIB.glfwIconifyWindow,
	SetCursorPosCallback = CLIB.glfwSetCursorPosCallback,
	SetScrollCallback = CLIB.glfwSetScrollCallback,
	GetPrimaryMonitor = CLIB.glfwGetPrimaryMonitor,
	GetGammaRamp = CLIB.glfwGetGammaRamp,
	WindowHint = CLIB.glfwWindowHint,
	GetWindowSize = CLIB.glfwGetWindowSize,
	GetFramebufferSize = CLIB.glfwGetFramebufferSize,
	GetInputMode = CLIB.glfwGetInputMode,
}
return library
