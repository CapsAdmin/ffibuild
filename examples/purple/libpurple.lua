local ffi = require("ffi")
ffi.cdef([[typedef enum PurpleXferStatusType{PURPLE_XFER_STATUS_UNKNOWN=0,PURPLE_XFER_STATUS_NOT_STARTED=1,PURPLE_XFER_STATUS_ACCEPTED=2,PURPLE_XFER_STATUS_STARTED=3,PURPLE_XFER_STATUS_DONE=4,PURPLE_XFER_STATUS_CANCEL_LOCAL=5,PURPLE_XFER_STATUS_CANCEL_REMOTE=6};
typedef enum PurpleMediaNetworkProtocol{PURPLE_MEDIA_NETWORK_PROTOCOL_UDP=0,PURPLE_MEDIA_NETWORK_PROTOCOL_TCP=1};
typedef enum PurpleConvUpdateType{PURPLE_CONV_UPDATE_ADD=0,PURPLE_CONV_UPDATE_REMOVE=1,PURPLE_CONV_UPDATE_ACCOUNT=2,PURPLE_CONV_UPDATE_TYPING=3,PURPLE_CONV_UPDATE_UNSEEN=4,PURPLE_CONV_UPDATE_LOGGING=5,PURPLE_CONV_UPDATE_TOPIC=6,PURPLE_CONV_ACCOUNT_ONLINE=7,PURPLE_CONV_ACCOUNT_OFFLINE=8,PURPLE_CONV_UPDATE_AWAY=9,PURPLE_CONV_UPDATE_ICON=10,PURPLE_CONV_UPDATE_TITLE=11,PURPLE_CONV_UPDATE_CHATLEFT=12,PURPLE_CONV_UPDATE_FEATURES=13};
typedef enum PurplePluginPrefType{PURPLE_PLUGIN_PREF_NONE=0,PURPLE_PLUGIN_PREF_CHOICE=1,PURPLE_PLUGIN_PREF_INFO=2,PURPLE_PLUGIN_PREF_STRING_FORMAT=3};
typedef enum PurpleConversationType{PURPLE_CONV_TYPE_UNKNOWN=0,PURPLE_CONV_TYPE_IM=1,PURPLE_CONV_TYPE_CHAT=2,PURPLE_CONV_TYPE_MISC=3,PURPLE_CONV_TYPE_ANY=4};
typedef enum PurpleRoomlistFieldType{PURPLE_ROOMLIST_FIELD_BOOL=0,PURPLE_ROOMLIST_FIELD_INT=1,PURPLE_ROOMLIST_FIELD_STRING=2};
typedef enum PurpleMediaCandidateType{PURPLE_MEDIA_CANDIDATE_TYPE_HOST=0,PURPLE_MEDIA_CANDIDATE_TYPE_SRFLX=1,PURPLE_MEDIA_CANDIDATE_TYPE_PRFLX=2,PURPLE_MEDIA_CANDIDATE_TYPE_RELAY=3,PURPLE_MEDIA_CANDIDATE_TYPE_MULTICAST=4};
typedef enum _PurplePrefType{PURPLE_PREF_NONE=0,PURPLE_PREF_BOOLEAN=1,PURPLE_PREF_INT=2,PURPLE_PREF_STRING=3,PURPLE_PREF_STRING_LIST=4,PURPLE_PREF_PATH=5,PURPLE_PREF_PATH_LIST=6};
typedef enum PurpleLogReadFlags{PURPLE_LOG_READ_NO_NEWLINE=1};
typedef enum PurplePounceOption{PURPLE_POUNCE_OPTION_NONE=0,PURPLE_POUNCE_OPTION_AWAY=1};
typedef enum PurpleMediaCaps{PURPLE_MEDIA_CAPS_NONE=0,PURPLE_MEDIA_CAPS_AUDIO=1,PURPLE_MEDIA_CAPS_AUDIO_SINGLE_DIRECTION=2,PURPLE_MEDIA_CAPS_VIDEO=4,PURPLE_MEDIA_CAPS_VIDEO_SINGLE_DIRECTION=8,PURPLE_MEDIA_CAPS_AUDIO_VIDEO=16,PURPLE_MEDIA_CAPS_MODIFY_SESSION=32,PURPLE_MEDIA_CAPS_CHANGE_DIRECTION=64};
typedef enum PurpleBlistNodeType{PURPLE_BLIST_GROUP_NODE=0,PURPLE_BLIST_CONTACT_NODE=1,PURPLE_BLIST_BUDDY_NODE=2,PURPLE_BLIST_CHAT_NODE=3,PURPLE_BLIST_OTHER_NODE=4};
typedef enum _PurpleCmdPriority{PURPLE_CMD_P_VERY_LOW=-1000,PURPLE_CMD_P_LOW=0,PURPLE_CMD_P_DEFAULT=1000,PURPLE_CMD_P_PRPL=2000,PURPLE_CMD_P_PLUGIN=3000,PURPLE_CMD_P_ALIAS=4000,PURPLE_CMD_P_HIGH=5000,PURPLE_CMD_P_VERY_HIGH=6000};
typedef enum PurpleXferType{PURPLE_XFER_UNKNOWN=0,PURPLE_XFER_SEND=1,PURPLE_XFER_RECEIVE=2};
typedef enum PurpleStringFormatType{PURPLE_STRING_FORMAT_TYPE_NONE=0,PURPLE_STRING_FORMAT_TYPE_MULTILINE=1,PURPLE_STRING_FORMAT_TYPE_HTML=2};
typedef enum _PurpleSoundEventID{PURPLE_SOUND_BUDDY_ARRIVE=0,PURPLE_SOUND_BUDDY_LEAVE=1,PURPLE_SOUND_RECEIVE=2,PURPLE_SOUND_FIRST_RECEIVE=3,PURPLE_SOUND_SEND=4,PURPLE_SOUND_CHAT_JOIN=5,PURPLE_SOUND_CHAT_LEAVE=6,PURPLE_SOUND_CHAT_YOU_SAY=7,PURPLE_SOUND_CHAT_SAY=8,PURPLE_SOUND_POUNCE_DEFAULT=9,PURPLE_SOUND_CHAT_NICK=10,PURPLE_SOUND_GOT_ATTENTION=11,PURPLE_NUM_SOUNDS=12};
typedef enum PurplePmpType{PURPLE_PMP_TYPE_UDP=0,PURPLE_PMP_TYPE_TCP=1};
typedef enum PurplePluginType{PURPLE_PLUGIN_UNKNOWN=-1,PURPLE_PLUGIN_STANDARD=0,PURPLE_PLUGIN_LOADER=1,PURPLE_PLUGIN_PROTOCOL=2};
typedef enum PurpleRequestFieldType{PURPLE_REQUEST_FIELD_NONE=0,PURPLE_REQUEST_FIELD_STRING=1,PURPLE_REQUEST_FIELD_INTEGER=2,PURPLE_REQUEST_FIELD_BOOLEAN=3,PURPLE_REQUEST_FIELD_CHOICE=4,PURPLE_REQUEST_FIELD_LIST=5,PURPLE_REQUEST_FIELD_LABEL=6,PURPLE_REQUEST_FIELD_IMAGE=7,PURPLE_REQUEST_FIELD_ACCOUNT=8};
typedef enum PurpleConnectionError{PURPLE_CONNECTION_ERROR_NETWORK_ERROR=0,PURPLE_CONNECTION_ERROR_INVALID_USERNAME=1,PURPLE_CONNECTION_ERROR_AUTHENTICATION_FAILED=2,PURPLE_CONNECTION_ERROR_AUTHENTICATION_IMPOSSIBLE=3,PURPLE_CONNECTION_ERROR_NO_SSL_SUPPORT=4,PURPLE_CONNECTION_ERROR_ENCRYPTION_ERROR=5,PURPLE_CONNECTION_ERROR_NAME_IN_USE=6,PURPLE_CONNECTION_ERROR_INVALID_SETTINGS=7,PURPLE_CONNECTION_ERROR_CERT_NOT_PROVIDED=8,PURPLE_CONNECTION_ERROR_CERT_UNTRUSTED=9,PURPLE_CONNECTION_ERROR_CERT_EXPIRED=10,PURPLE_CONNECTION_ERROR_CERT_NOT_ACTIVATED=11,PURPLE_CONNECTION_ERROR_CERT_HOSTNAME_MISMATCH=12,PURPLE_CONNECTION_ERROR_CERT_FINGERPRINT_MISMATCH=13,PURPLE_CONNECTION_ERROR_CERT_SELF_SIGNED=14,PURPLE_CONNECTION_ERROR_CERT_OTHER_ERROR=15,PURPLE_CONNECTION_ERROR_OTHER_ERROR=16};
typedef enum _PurpleCmdStatus{PURPLE_CMD_STATUS_OK=0,PURPLE_CMD_STATUS_FAILED=1,PURPLE_CMD_STATUS_NOT_FOUND=2,PURPLE_CMD_STATUS_WRONG_ARGS=3,PURPLE_CMD_STATUS_WRONG_PRPL=4,PURPLE_CMD_STATUS_WRONG_TYPE=5};
typedef enum PurpleDebugLevel{PURPLE_DEBUG_ALL=0,PURPLE_DEBUG_MISC=1,PURPLE_DEBUG_INFO=2,PURPLE_DEBUG_WARNING=3,PURPLE_DEBUG_ERROR=4,PURPLE_DEBUG_FATAL=5};
typedef enum _PurplePrivacyType{PURPLE_PRIVACY_ALLOW_ALL=1,PURPLE_PRIVACY_DENY_ALL=2,PURPLE_PRIVACY_ALLOW_USERS=3,PURPLE_PRIVACY_DENY_USERS=4,PURPLE_PRIVACY_ALLOW_BUDDYLIST=5};
typedef enum PurpleStunNatType{PURPLE_STUN_NAT_TYPE_PUBLIC_IP=0,PURPLE_STUN_NAT_TYPE_UNKNOWN_NAT=1,PURPLE_STUN_NAT_TYPE_FULL_CONE=2,PURPLE_STUN_NAT_TYPE_RESTRICTED_CONE=3,PURPLE_STUN_NAT_TYPE_PORT_RESTRICTED_CONE=4,PURPLE_STUN_NAT_TYPE_SYMMETRIC=5};
typedef enum PurpleMediaInfoType{PURPLE_MEDIA_INFO_HANGUP=0,PURPLE_MEDIA_INFO_ACCEPT=1,PURPLE_MEDIA_INFO_REJECT=2,PURPLE_MEDIA_INFO_MUTE=3,PURPLE_MEDIA_INFO_UNMUTE=4,PURPLE_MEDIA_INFO_PAUSE=5,PURPLE_MEDIA_INFO_UNPAUSE=6,PURPLE_MEDIA_INFO_HOLD=7,PURPLE_MEDIA_INFO_UNHOLD=8};
typedef enum PurplePresenceContext{PURPLE_PRESENCE_CONTEXT_UNSET=0,PURPLE_PRESENCE_CONTEXT_ACCOUNT=1,PURPLE_PRESENCE_CONTEXT_CONV=2,PURPLE_PRESENCE_CONTEXT_BUDDY=3};
typedef enum _PurpleCipherBatchMode{PURPLE_CIPHER_BATCH_MODE_ECB=0,PURPLE_CIPHER_BATCH_MODE_CBC=1};
typedef enum PurpleNotifySearchButtonType{PURPLE_NOTIFY_BUTTON_LABELED=0,PURPLE_NOTIFY_BUTTON_CONTINUE=1,PURPLE_NOTIFY_BUTTON_ADD=2,PURPLE_NOTIFY_BUTTON_INFO=3,PURPLE_NOTIFY_BUTTON_IM=4,PURPLE_NOTIFY_BUTTON_JOIN=5,PURPLE_NOTIFY_BUTTON_INVITE=6};
typedef enum PurpleNotifyMsgType{PURPLE_NOTIFY_MSG_ERROR=0,PURPLE_NOTIFY_MSG_WARNING=1,PURPLE_NOTIFY_MSG_INFO=2};
typedef enum PurpleSslErrorType{PURPLE_SSL_HANDSHAKE_FAILED=1,PURPLE_SSL_CONNECT_FAILED=2,PURPLE_SSL_CERTIFICATE_INVALID=3};
typedef enum PurpleIconScaleRules{PURPLE_ICON_SCALE_DISPLAY=1,PURPLE_ICON_SCALE_SEND=2};
typedef enum PurpleNotifyType{PURPLE_NOTIFY_MESSAGE=0,PURPLE_NOTIFY_EMAIL=1,PURPLE_NOTIFY_EMAILS=2,PURPLE_NOTIFY_FORMATTED=3,PURPLE_NOTIFY_SEARCHRESULTS=4,PURPLE_NOTIFY_USERINFO=5,PURPLE_NOTIFY_URI=6};
typedef enum PurpleCertificateVerificationStatus{PURPLE_CERTIFICATE_INVALID=0,PURPLE_CERTIFICATE_VALID=1};
typedef enum PurpleNotifyUserInfoEntryType{PURPLE_NOTIFY_USER_INFO_ENTRY_PAIR=0,PURPLE_NOTIFY_USER_INFO_ENTRY_SECTION_BREAK=1,PURPLE_NOTIFY_USER_INFO_ENTRY_SECTION_HEADER=2};
typedef enum PurpleRoomlistRoomType{PURPLE_ROOMLIST_ROOMTYPE_CATEGORY=1,PURPLE_ROOMLIST_ROOMTYPE_ROOM=2};
typedef enum PurpleTypingState{PURPLE_NOT_TYPING=0,PURPLE_TYPING=1,PURPLE_TYPED=2};
typedef enum _PurpleCmdFlag{PURPLE_CMD_FLAG_IM=1,PURPLE_CMD_FLAG_CHAT=2,PURPLE_CMD_FLAG_PRPL_ONLY=4,PURPLE_CMD_FLAG_ALLOW_WRONG_ARGS=8};
typedef enum PurpleDesktopItemType{PURPLE_DESKTOP_ITEM_TYPE_NULL=0,PURPLE_DESKTOP_ITEM_TYPE_OTHER=1,PURPLE_DESKTOP_ITEM_TYPE_APPLICATION=2,PURPLE_DESKTOP_ITEM_TYPE_LINK=3,PURPLE_DESKTOP_ITEM_TYPE_FSDEVICE=4,PURPLE_DESKTOP_ITEM_TYPE_MIME_TYPE=5,PURPLE_DESKTOP_ITEM_TYPE_DIRECTORY=6,PURPLE_DESKTOP_ITEM_TYPE_SERVICE=7,PURPLE_DESKTOP_ITEM_TYPE_SERVICE_TYPE=8};
typedef enum PurpleMessageFlags{PURPLE_MESSAGE_SEND=1,PURPLE_MESSAGE_RECV=2,PURPLE_MESSAGE_SYSTEM=4,PURPLE_MESSAGE_AUTO_RESP=8,PURPLE_MESSAGE_ACTIVE_ONLY=16,PURPLE_MESSAGE_NICK=32,PURPLE_MESSAGE_NO_LOG=64,PURPLE_MESSAGE_WHISPER=128,PURPLE_MESSAGE_ERROR=512,PURPLE_MESSAGE_DELAYED=1024,PURPLE_MESSAGE_RAW=2048,PURPLE_MESSAGE_IMAGES=4096,PURPLE_MESSAGE_NOTIFY=8192,PURPLE_MESSAGE_NO_LINKIFY=16384,PURPLE_MESSAGE_INVISIBLE=32768};
typedef enum PurpleBlistNodeFlags{PURPLE_BLIST_NODE_FLAG_NO_SAVE=1};
typedef enum PurpleType{PURPLE_TYPE_UNKNOWN=0,PURPLE_TYPE_SUBTYPE=1,PURPLE_TYPE_CHAR=2,PURPLE_TYPE_UCHAR=3,PURPLE_TYPE_BOOLEAN=4,PURPLE_TYPE_SHORT=5,PURPLE_TYPE_USHORT=6,PURPLE_TYPE_INT=7,PURPLE_TYPE_UINT=8,PURPLE_TYPE_LONG=9,PURPLE_TYPE_ULONG=10,PURPLE_TYPE_INT64=11,PURPLE_TYPE_UINT64=12,PURPLE_TYPE_STRING=13,PURPLE_TYPE_OBJECT=14,PURPLE_TYPE_POINTER=15,PURPLE_TYPE_ENUM=16,PURPLE_TYPE_BOXED=17};
typedef enum PurplePounceEvent{PURPLE_POUNCE_NONE=0,PURPLE_POUNCE_SIGNON=1,PURPLE_POUNCE_SIGNOFF=2,PURPLE_POUNCE_AWAY=4,PURPLE_POUNCE_AWAY_RETURN=8,PURPLE_POUNCE_IDLE=16,PURPLE_POUNCE_IDLE_RETURN=32,PURPLE_POUNCE_TYPING=64,PURPLE_POUNCE_TYPED=128,PURPLE_POUNCE_TYPING_STOPPED=256,PURPLE_POUNCE_MESSAGE_RECEIVED=512};
typedef enum PurpleLogType{PURPLE_LOG_IM=0,PURPLE_LOG_CHAT=1,PURPLE_LOG_SYSTEM=2};
typedef enum _XMLNodeType{XMLNODE_TYPE_TAG=0,XMLNODE_TYPE_ATTRIB=1,XMLNODE_TYPE_DATA=2};
typedef enum PurpleProxyType{PURPLE_PROXY_USE_GLOBAL=-1,PURPLE_PROXY_NONE=0,PURPLE_PROXY_HTTP=1,PURPLE_PROXY_SOCKS4=2,PURPLE_PROXY_SOCKS5=3,PURPLE_PROXY_USE_ENVVAR=4,PURPLE_PROXY_TOR=5};
typedef enum PurpleConvChatBuddyFlags{PURPLE_CBFLAGS_NONE=0,PURPLE_CBFLAGS_VOICE=1,PURPLE_CBFLAGS_HALFOP=2,PURPLE_CBFLAGS_OP=4,PURPLE_CBFLAGS_FOUNDER=8,PURPLE_CBFLAGS_TYPING=16,PURPLE_CBFLAGS_AWAY=32};
typedef enum PurpleInputCondition{PURPLE_INPUT_READ=1,PURPLE_INPUT_WRITE=2};
typedef enum PurpleRequestType{PURPLE_REQUEST_INPUT=0,PURPLE_REQUEST_CHOICE=1,PURPLE_REQUEST_ACTION=2,PURPLE_REQUEST_FIELDS=3,PURPLE_REQUEST_FILE=4,PURPLE_REQUEST_FOLDER=5};
typedef enum PurpleConnectionFlags{PURPLE_CONNECTION_HTML=1,PURPLE_CONNECTION_NO_BGCOLOR=2,PURPLE_CONNECTION_AUTO_RESP=4,PURPLE_CONNECTION_FORMATTING_WBFO=8,PURPLE_CONNECTION_NO_NEWLINES=16,PURPLE_CONNECTION_NO_FONTSIZE=32,PURPLE_CONNECTION_NO_URLDESC=64,PURPLE_CONNECTION_NO_IMAGES=128,PURPLE_CONNECTION_ALLOW_CUSTOM_SMILEY=256,PURPLE_CONNECTION_SUPPORT_MOODS=512,PURPLE_CONNECTION_SUPPORT_MOOD_MESSAGES=1024};
typedef enum PurpleMediaSessionType{PURPLE_MEDIA_NONE=0,PURPLE_MEDIA_RECV_AUDIO=1,PURPLE_MEDIA_SEND_AUDIO=2,PURPLE_MEDIA_RECV_VIDEO=4,PURPLE_MEDIA_SEND_VIDEO=8,PURPLE_MEDIA_AUDIO=3,PURPLE_MEDIA_VIDEO=12};
typedef enum PurpleConnectionState{PURPLE_DISCONNECTED=0,PURPLE_CONNECTED=1,PURPLE_CONNECTING=2};
typedef enum PurpleStunStatus{PURPLE_STUN_STATUS_UNDISCOVERED=-1,PURPLE_STUN_STATUS_UNKNOWN=0,PURPLE_STUN_STATUS_DISCOVERING=1,PURPLE_STUN_STATUS_DISCOVERED=2};
typedef enum PurpleStatusPrimitive{PURPLE_STATUS_UNSET=0,PURPLE_STATUS_OFFLINE=1,PURPLE_STATUS_AVAILABLE=2,PURPLE_STATUS_UNAVAILABLE=3,PURPLE_STATUS_INVISIBLE=4,PURPLE_STATUS_AWAY=5,PURPLE_STATUS_EXTENDED_AWAY=6,PURPLE_STATUS_MOBILE=7,PURPLE_STATUS_TUNE=8,PURPLE_STATUS_MOOD=9,PURPLE_STATUS_NUM_PRIMITIVES=10};
struct tm {int tm_sec;int tm_min;int tm_hour;int tm_mday;int tm_mon;int tm_year;int tm_wday;int tm_yday;int tm_isdst;long tm_gmtoff;const char*tm_zone;};
struct _GByteArray {unsigned char*data;unsigned int len;};
struct _GData {};
struct _GList {void*data;struct _GList*next;struct _GList*prev;};
struct _GHashTable {};
struct _GSList {void*data;struct _GSList*next;};
struct _GString {char*str;unsigned long len;unsigned long allocated_len;};
struct _GTypeClass {unsigned long g_type;};
struct _GTypeInstance {struct _GTypeClass*g_class;};
struct _GValue {unsigned long g_type;union {int v_int;unsigned int v_uint;long v_long;unsigned long v_ulong;signed long v_int64;unsigned long v_uint64;float v_float;double v_double;void*v_pointer;}data[2];};
struct _GParameter {const char*name;struct _GValue value;};
struct _GObject {struct _GTypeInstance g_type_instance;unsigned int ref_count;struct _GData*qdata;};
struct PurpleConnectionErrorInfo {enum PurpleConnectionError type;char*description;};
struct PurpleValue {enum PurpleType type;unsigned short flags;union {char char_data;unsigned char uchar_data;int boolean_data;short short_data;unsigned short ushort_data;int int_data;unsigned int uint_data;long long_data;unsigned long ulong_data;signed long int64_data;unsigned long uint64_data;char*string_data;void*object_data;void*pointer_data;int enum_data;void*boxed_data;}data;union {unsigned int subtype;char*specific_type;}u;};
struct _PurplePluginPrefFrame {};
struct _PurplePluginPref {};
struct _PurplePluginInfo {unsigned int magic;unsigned int major_version;unsigned int minor_version;enum PurplePluginType type;char*ui_requirement;unsigned long flags;struct _GList*dependencies;int priority;char*id;char*name;char*version;char*summary;char*description;char*author;char*homepage;int(*load)(struct _PurplePlugin*);int(*unload)(struct _PurplePlugin*);void(*destroy)(struct _PurplePlugin*);void*ui_info;void*extra_info;struct _PurplePluginUiInfo*prefs_info;struct _GList*(*actions)(struct _PurplePlugin*,void*);void(*_purple_reserved1)();void(*_purple_reserved2)();void(*_purple_reserved3)();void(*_purple_reserved4)();};
struct _PurplePlugin {int native_plugin;int loaded;void*handle;char*path;struct _PurplePluginInfo*info;char*error;void*ipc_data;void*extra;int unloadable;struct _GList*dependent_plugins;void(*_purple_reserved1)();void(*_purple_reserved2)();void(*_purple_reserved3)();void(*_purple_reserved4)();};
struct _PurplePluginUiInfo {struct _PurplePluginPrefFrame*(*get_plugin_pref_frame)(struct _PurplePlugin*);int page_num;struct _PurplePluginPrefFrame*frame;void(*_purple_reserved1)();void(*_purple_reserved2)();void(*_purple_reserved3)();void(*_purple_reserved4)();};
struct _PurplePluginAction {char*label;void(*callback)(struct _PurplePluginAction*);struct _PurplePlugin*plugin;void*context;void*user_data;};
struct _PurpleStatusType {};
struct _PurpleStatusAttr {};
struct _PurplePresence {};
struct _PurpleStatus {};
struct _PurpleBuddyIcon {};
struct _PurpleStoredImage {};
struct _IO_FILE {};
struct _PurpleLogLogger {char*name;char*id;void(*create)(struct _PurpleLog*);unsigned long(*write)(struct _PurpleLog*,enum PurpleMessageFlags,const char*,long,const char*);void(*finalize)(struct _PurpleLog*);struct _GList*(*list)(enum PurpleLogType,const char*,struct _PurpleAccount*);char*(*read)(struct _PurpleLog*,enum PurpleLogReadFlags*);int(*size)(struct _PurpleLog*);int(*total_size)(enum PurpleLogType,const char*,struct _PurpleAccount*);struct _GList*(*list_syslog)(struct _PurpleAccount*);void(*get_log_sets)(void(*)(struct _GHashTable*,struct _PurpleLogSet*),struct _GHashTable*);int(*remove)(struct _PurpleLog*);int(*is_deletable)(struct _PurpleLog*);void(*_purple_reserved1)();void(*_purple_reserved2)();void(*_purple_reserved3)();void(*_purple_reserved4)();};
struct _PurpleLog {enum PurpleLogType type;char*name;struct _PurpleAccount*account;struct _PurpleConversation*conv;long time;struct _PurpleLogLogger*logger;void*logger_data;struct tm*tm;};
struct _PurpleLogSet {enum PurpleLogType type;char*name;struct _PurpleAccount*account;int buddy;char*normalized_name;};
struct _PurpleConversationUiOps {void(*create_conversation)(struct _PurpleConversation*);void(*destroy_conversation)(struct _PurpleConversation*);void(*write_chat)(struct _PurpleConversation*,const char*,const char*,enum PurpleMessageFlags,long);void(*write_im)(struct _PurpleConversation*,const char*,const char*,enum PurpleMessageFlags,long);void(*write_conv)(struct _PurpleConversation*,const char*,const char*,const char*,enum PurpleMessageFlags,long);void(*chat_add_users)(struct _PurpleConversation*,struct _GList*,int);void(*chat_rename_user)(struct _PurpleConversation*,const char*,const char*,const char*);void(*chat_remove_users)(struct _PurpleConversation*,struct _GList*);void(*chat_update_user)(struct _PurpleConversation*,const char*);void(*present)(struct _PurpleConversation*);int(*has_focus)(struct _PurpleConversation*);int(*custom_smiley_add)(struct _PurpleConversation*,const char*,int);void(*custom_smiley_write)(struct _PurpleConversation*,const char*,const unsigned char*,unsigned long);void(*custom_smiley_close)(struct _PurpleConversation*,const char*);void(*send_confirm)(struct _PurpleConversation*,const char*);void(*_purple_reserved1)();void(*_purple_reserved2)();void(*_purple_reserved3)();void(*_purple_reserved4)();};
struct _PurpleConvIm {struct _PurpleConversation*conv;enum PurpleTypingState typing_state;unsigned int typing_timeout;long type_again;unsigned int send_typed_timeout;struct _PurpleBuddyIcon*icon;};
struct _PurpleConvChat {struct _PurpleConversation*conv;struct _GList*in_room;struct _GList*ignored;char*who;char*topic;int id;char*nick;int left;struct _GHashTable*users;};
struct _PurpleConvChatBuddy {char*name;char*alias;char*alias_key;int buddy;enum PurpleConvChatBuddyFlags flags;struct _GHashTable*attributes;void*ui_data;};
struct _PurpleConvMessage {char*who;char*what;enum PurpleMessageFlags flags;long when;struct _PurpleConversation*conv;char*alias;};
struct _PurpleConversation {enum PurpleConversationType type;struct _PurpleAccount*account;char*name;char*title;int logging;struct _GList*logs;union {struct _PurpleConvIm*im;struct _PurpleConvChat*chat;void*misc;}u;struct _PurpleConversationUiOps*ui_ops;void*ui_data;struct _GHashTable*data;enum PurpleConnectionFlags features;struct _GList*message_history;};
struct PurpleXferUiOps {void(*new_xfer)(struct _PurpleXfer*);void(*destroy)(struct _PurpleXfer*);void(*add_xfer)(struct _PurpleXfer*);void(*update_progress)(struct _PurpleXfer*,double);void(*cancel_local)(struct _PurpleXfer*);void(*cancel_remote)(struct _PurpleXfer*);signed long(*ui_write)(struct _PurpleXfer*,const unsigned char*,signed long);signed long(*ui_read)(struct _PurpleXfer*,unsigned char**,signed long);void(*data_not_sent)(struct _PurpleXfer*,const unsigned char*,unsigned long);void(*add_thumbnail)(struct _PurpleXfer*,const char*);};
struct _PurpleXfer {unsigned int ref;enum PurpleXferType type;struct _PurpleAccount*account;char*who;char*message;char*filename;char*local_filename;unsigned long size;struct _IO_FILE*dest_fp;char*remote_ip;int local_port;int remote_port;int fd;int watcher;unsigned long bytes_sent;unsigned long bytes_remaining;long start_time;long end_time;unsigned long current_buffer_size;enum PurpleXferStatusType status;struct {void(*init)(struct _PurpleXfer*);void(*request_denied)(struct _PurpleXfer*);void(*start)(struct _PurpleXfer*);void(*end)(struct _PurpleXfer*);void(*cancel_send)(struct _PurpleXfer*);void(*cancel_recv)(struct _PurpleXfer*);signed long(*read)(unsigned char**,struct _PurpleXfer*);signed long(*write)(const unsigned char*,unsigned long,struct _PurpleXfer*);void(*ack)(struct _PurpleXfer*,const unsigned char*,unsigned long);}ops;struct PurpleXferUiOps*ui_ops;void*ui_data;void*data;};
struct _PurpleMediaCandidate {};
struct _PurpleMediaCodec {};
struct _PurpleUtilFetchUrlData {};
struct _xmlnode {char*name;char*xmlns;enum _XMLNodeType type;char*data;unsigned long data_sz;struct _xmlnode*parent;struct _xmlnode*child;struct _xmlnode*lastchild;struct _xmlnode*next;char*prefix;struct _GHashTable*namespace_map;};
struct _PurpleNotifyUserInfoEntry {};
struct _PurpleNotifyUserInfo {};
struct PurpleNotifySearchResults {struct _GList*columns;struct _GList*rows;struct _GList*buttons;};
struct PurpleNotifySearchColumn {char*title;};
struct PurpleNotifyUiOps {void*(*notify_message)(enum PurpleNotifyMsgType,const char*,const char*,const char*);void*(*notify_email)(struct _PurpleConnection*,const char*,const char*,const char*,const char*);void*(*notify_emails)(struct _PurpleConnection*,unsigned long,int,const char**,const char**,const char**,const char**);void*(*notify_formatted)(const char*,const char*,const char*,const char*);void*(*notify_searchresults)(struct _PurpleConnection*,const char*,const char*,const char*,struct PurpleNotifySearchResults*,void*);void(*notify_searchresults_new_rows)(struct _PurpleConnection*,struct PurpleNotifySearchResults*,void*);void*(*notify_userinfo)(struct _PurpleConnection*,const char*,struct _PurpleNotifyUserInfo*);void*(*notify_uri)(const char*);void(*close_notify)(enum PurpleNotifyType,void*);void(*_purple_reserved1)();void(*_purple_reserved2)();void(*_purple_reserved3)();void(*_purple_reserved4)();};
struct _PurpleMenuAction {char*label;void(*callback)();void*data;struct _GList*children;};
struct _PurpleKeyValuePair {char*key;void*value;};
struct _PurpleMedia {};
struct _PurpleEventLoopUiOps {unsigned int(*timeout_add)(unsigned int,int(*)(void*),void*);int(*timeout_remove)(unsigned int);unsigned int(*input_add)(int,enum PurpleInputCondition,void(*)(void*,int,enum PurpleInputCondition),void*);int(*input_remove)(unsigned int);int(*input_get_error)(int,int*);unsigned int(*timeout_add_seconds)(unsigned int,int(*)(void*),void*);void(*_purple_reserved2)();void(*_purple_reserved3)();void(*_purple_reserved4)();};
struct PurpleProxyInfo {enum PurpleProxyType type;char*host;int port;char*username;char*password;};
struct _PurpleProxyConnectData {};
struct _PurpleRoomlist {struct _PurpleAccount*account;struct _GList*fields;struct _GList*rooms;int in_progress;void*ui_data;void*proto_data;unsigned int ref;};
struct _PurpleRoomlistRoom {enum PurpleRoomlistRoomType type;char*name;struct _GList*fields;struct _PurpleRoomlistRoom*parent;int expanded_once;};
struct _PurpleRoomlistField {enum PurpleRoomlistFieldType type;char*label;char*name;int hidden;};
struct _PurpleRoomlistUiOps {void(*show_with_account)(struct _PurpleAccount*);void(*create)(struct _PurpleRoomlist*);void(*set_fields)(struct _PurpleRoomlist*,struct _GList*);void(*add_room)(struct _PurpleRoomlist*,struct _PurpleRoomlistRoom*);void(*in_progress)(struct _PurpleRoomlist*,int);void(*destroy)(struct _PurpleRoomlist*);void(*_purple_reserved1)();void(*_purple_reserved2)();void(*_purple_reserved3)();void(*_purple_reserved4)();};
struct _PurpleWhiteboard {int state;struct _PurpleAccount*account;char*who;void*ui_data;void*proto_data;struct _PurpleWhiteboardPrplOps*prpl_ops;struct _GList*draw_list;};
struct _PurpleWhiteboardUiOps {void(*create)(struct _PurpleWhiteboard*);void(*destroy)(struct _PurpleWhiteboard*);void(*set_dimensions)(struct _PurpleWhiteboard*,int,int);void(*set_brush)(struct _PurpleWhiteboard*,int,int);void(*draw_point)(struct _PurpleWhiteboard*,int,int,int,int);void(*draw_line)(struct _PurpleWhiteboard*,int,int,int,int,int,int);void(*clear)(struct _PurpleWhiteboard*);void(*_purple_reserved1)();void(*_purple_reserved2)();void(*_purple_reserved3)();void(*_purple_reserved4)();};
struct _PurpleWhiteboardPrplOps {void(*start)(struct _PurpleWhiteboard*);void(*end)(struct _PurpleWhiteboard*);void(*get_dimensions)(const struct _PurpleWhiteboard*,int*,int*);void(*set_dimensions)(struct _PurpleWhiteboard*,int,int);void(*get_brush)(const struct _PurpleWhiteboard*,int*,int*);void(*set_brush)(struct _PurpleWhiteboard*,int,int);void(*send_draw_list)(struct _PurpleWhiteboard*,struct _GList*);void(*clear)(struct _PurpleWhiteboard*);void(*_purple_reserved1)();void(*_purple_reserved2)();void(*_purple_reserved3)();void(*_purple_reserved4)();};
struct _PurpleBuddyIconSpec {char*format;int min_width;int min_height;int max_width;int max_height;unsigned long max_filesize;enum PurpleIconScaleRules scale_rules;};
struct _PurpleAttentionType {const char*name;const char*incoming_description;const char*outgoing_description;const char*icon_name;const char*unlocalized_name;void*_reserved2;void*_reserved3;void*_reserved4;};
struct _PurpleBlistNode {enum PurpleBlistNodeType type;struct _PurpleBlistNode*prev;struct _PurpleBlistNode*next;struct _PurpleBlistNode*parent;struct _PurpleBlistNode*child;struct _GHashTable*settings;void*ui_data;enum PurpleBlistNodeFlags flags;};
struct _PurpleBuddy {struct _PurpleBlistNode node;char*name;char*alias;char*server_alias;void*proto_data;struct _PurpleBuddyIcon*icon;struct _PurpleAccount*account;struct _PurplePresence*presence;enum PurpleMediaCaps media_caps;};
struct _PurpleContact {struct _PurpleBlistNode node;char*alias;int totalsize;int currentsize;int online;struct _PurpleBuddy*priority;int priority_valid;};
struct _PurpleGroup {struct _PurpleBlistNode node;char*name;int totalsize;int currentsize;int online;};
struct _PurpleChat {struct _PurpleBlistNode node;char*alias;struct _GHashTable*components;struct _PurpleAccount*account;};
struct _PurpleBuddyList {struct _PurpleBlistNode*root;struct _GHashTable*buddies;void*ui_data;};
struct _PurpleBlistUiOps {void(*new_list)(struct _PurpleBuddyList*);void(*new_node)(struct _PurpleBlistNode*);void(*show)(struct _PurpleBuddyList*);void(*update)(struct _PurpleBuddyList*,struct _PurpleBlistNode*);void(*remove)(struct _PurpleBuddyList*,struct _PurpleBlistNode*);void(*destroy)(struct _PurpleBuddyList*);void(*set_visible)(struct _PurpleBuddyList*,int);void(*request_add_buddy)(struct _PurpleAccount*,const char*,const char*,const char*);void(*request_add_chat)(struct _PurpleAccount*,struct _PurpleGroup*,const char*,const char*);void(*request_add_group)();void(*save_node)(struct _PurpleBlistNode*);void(*remove_node)(struct _PurpleBlistNode*);void(*save_account)(struct _PurpleAccount*);void(*_purple_reserved1)();};
struct _PurpleCertificate {struct _PurpleCertificateScheme*scheme;void*data;};
struct _PurpleCertificatePool {char*scheme_name;char*name;char*fullname;void*data;int(*init)();void(*uninit)();int(*cert_in_pool)(const char*);struct _PurpleCertificate*(*get_cert)(const char*);int(*put_cert)(const char*,struct _PurpleCertificate*);int(*delete_cert)(const char*);struct _GList*(*get_idlist)();void(*_purple_reserved1)();void(*_purple_reserved2)();void(*_purple_reserved3)();void(*_purple_reserved4)();};
struct _PurpleCertificateScheme {char*name;char*fullname;struct _PurpleCertificate*(*import_certificate)(const char*);int(*export_certificate)(const char*,struct _PurpleCertificate*);struct _PurpleCertificate*(*copy_certificate)(struct _PurpleCertificate*);void(*destroy_certificate)(struct _PurpleCertificate*);int(*signed_by)(struct _PurpleCertificate*,struct _PurpleCertificate*);struct _GByteArray*(*get_fingerprint_sha1)(struct _PurpleCertificate*);char*(*get_unique_id)(struct _PurpleCertificate*);char*(*get_issuer_unique_id)(struct _PurpleCertificate*);char*(*get_subject_name)(struct _PurpleCertificate*);int(*check_subject_name)(struct _PurpleCertificate*,const char*);int(*get_times)(struct _PurpleCertificate*,long*,long*);struct _GSList*(*import_certificates)(const char*);int(*register_trusted_tls_cert)(struct _PurpleCertificate*,int);void(*verify_cert)(struct _PurpleCertificateVerificationRequest*,enum PurpleCertificateInvalidityFlags*);void(*_purple_reserved3)();};
struct _PurpleCertificateVerifier {char*scheme_name;char*name;void(*start_verification)(struct _PurpleCertificateVerificationRequest*);void(*destroy_request)(struct _PurpleCertificateVerificationRequest*);void(*_purple_reserved1)();void(*_purple_reserved2)();void(*_purple_reserved3)();void(*_purple_reserved4)();};
struct _PurpleCertificateVerificationRequest {struct _PurpleCertificateVerifier*verifier;struct _PurpleCertificateScheme*scheme;char*subject_name;struct _GList*cert_chain;void*data;void(*cb)(enum PurpleCertificateVerificationStatus,void*);void*cb_data;};
struct _PurpleSslConnection {char*host;int port;void*connect_cb_data;void(*connect_cb)(void*,struct _PurpleSslConnection*,enum PurpleInputCondition);void(*error_cb)(struct _PurpleSslConnection*,enum PurpleSslErrorType,void*);void*recv_cb_data;void(*recv_cb)(void*,struct _PurpleSslConnection*,enum PurpleInputCondition);int fd;unsigned int inpa;struct _PurpleProxyConnectData*connect_data;void*private_data;struct _PurpleCertificateVerifier*verifier;};
struct PurpleSslOps {int(*init)();void(*uninit)();void(*connectfunc)(struct _PurpleSslConnection*);void(*close)(struct _PurpleSslConnection*);unsigned long(*read)(struct _PurpleSslConnection*,void*,unsigned long);unsigned long(*write)(struct _PurpleSslConnection*,const void*,unsigned long);struct _GList*(*get_peer_certificates)(struct _PurpleSslConnection*);void(*_purple_reserved2)();void(*_purple_reserved3)();void(*_purple_reserved4)();};
struct PurpleConnectionUiOps {void(*connect_progress)(struct _PurpleConnection*,const char*,unsigned long,unsigned long);void(*connected)(struct _PurpleConnection*);void(*disconnected)(struct _PurpleConnection*);void(*notice)(struct _PurpleConnection*,const char*);void(*report_disconnect)(struct _PurpleConnection*,const char*);void(*network_connected)();void(*network_disconnected)();void(*report_disconnect_reason)(struct _PurpleConnection*,enum PurpleConnectionError,const char*);void(*_purple_reserved1)();void(*_purple_reserved2)();void(*_purple_reserved3)();};
struct _PurpleConnection {struct _PurplePlugin*prpl;enum PurpleConnectionFlags flags;enum PurpleConnectionState state;struct _PurpleAccount*account;char*password;int inpa;struct _GSList*buddy_chats;void*proto_data;char*display_name;unsigned int keepalive;int wants_to_die;unsigned int disconnect_timeout;long last_received;};
struct PurplePrivacyUiOps {void(*permit_added)(struct _PurpleAccount*,const char*);void(*permit_removed)(struct _PurpleAccount*,const char*);void(*deny_added)(struct _PurpleAccount*,const char*);void(*deny_removed)(struct _PurpleAccount*,const char*);void(*_purple_reserved1)();void(*_purple_reserved2)();void(*_purple_reserved3)();void(*_purple_reserved4)();};
struct _PurpleAccountUiOps {void(*notify_added)(struct _PurpleAccount*,const char*,const char*,const char*,const char*);void(*status_changed)(struct _PurpleAccount*,struct _PurpleStatus*);void(*request_add)(struct _PurpleAccount*,const char*,const char*,const char*,const char*);void*(*request_authorize)(struct _PurpleAccount*,const char*,const char*,const char*,const char*,int,void(*)(void*),void(*)(void*),void*);void(*close_account_request)(void*);void(*_purple_reserved1)();void(*_purple_reserved2)();void(*_purple_reserved3)();void(*_purple_reserved4)();};
struct _PurpleAccount {char*username;char*alias;char*password;char*user_info;char*buddy_icon_path;int remember_pass;char*protocol_id;struct _PurpleConnection*gc;int disconnecting;struct _GHashTable*settings;struct _GHashTable*ui_settings;struct PurpleProxyInfo*proxy_info;struct _GSList*permit;struct _GSList*deny;enum _PurplePrivacyType perm_deny;struct _GList*status_types;struct _PurplePresence*presence;struct _PurpleLog*system_log;void*ui_data;void(*registration_cb)(struct _PurpleAccount*,int,void*);void*registration_cb_user_data;void*priv;};
struct PurpleAccountOption {enum _PurplePrefType type;char*text;char*pref_name;union {int boolean;int integer;char*string;struct _GList*list;}default_value;int masked;};
struct PurpleAccountUserSplit {char*text;char*default_value;char field_sep;int reverse;};
struct _PurpleCipher {};
struct _PurpleCipherContext {};
struct _PurpleCipherOps {void(*set_option)(struct _PurpleCipherContext*,const char*,void*);void*(*get_option)(struct _PurpleCipherContext*,const char*);void(*init)(struct _PurpleCipherContext*,void*);void(*reset)(struct _PurpleCipherContext*,void*);void(*uninit)(struct _PurpleCipherContext*);void(*set_iv)(struct _PurpleCipherContext*,unsigned char*,unsigned long);void(*append)(struct _PurpleCipherContext*,const unsigned char*,unsigned long);int(*digest)(struct _PurpleCipherContext*,unsigned long,unsigned char,unsigned long*);int(*encrypt)(struct _PurpleCipherContext*,const unsigned char,unsigned long,unsigned char,unsigned long*);int(*decrypt)(struct _PurpleCipherContext*,const unsigned char,unsigned long,unsigned char,unsigned long*);void(*set_salt)(struct _PurpleCipherContext*,unsigned char*);unsigned long(*get_salt_size)(struct _PurpleCipherContext*);void(*set_key)(struct _PurpleCipherContext*,const unsigned char*);unsigned long(*get_key_size)(struct _PurpleCipherContext*);void(*set_batch_mode)(struct _PurpleCipherContext*,enum _PurpleCipherBatchMode);enum _PurpleCipherBatchMode(*get_batch_mode)(struct _PurpleCipherContext*);unsigned long(*get_block_size)(struct _PurpleCipherContext*);void(*set_key_with_len)(struct _PurpleCipherContext*,const unsigned char*,unsigned long);};
struct _PurpleCircBuffer {char*buffer;unsigned long growsize;unsigned long buflen;unsigned long bufused;char*inptr;char*outptr;};
struct PurpleCore {};
struct PurpleCoreUiOps {void(*ui_prefs_init)();void(*debug_ui_init)();void(*ui_init)();void(*quit)();struct _GHashTable*(*get_ui_info)();void(*_purple_reserved1)();void(*_purple_reserved2)();void(*_purple_reserved3)();};
struct PurpleDebugUiOps {void(*print)(enum PurpleDebugLevel,const char*,const char*);int(*is_enabled)(enum PurpleDebugLevel,const char*);void(*_purple_reserved1)();void(*_purple_reserved2)();void(*_purple_reserved3)();void(*_purple_reserved4)();};
struct _PurpleDesktopItem {};
struct _PurpleDnsQueryData {};
struct PurpleDnsQueryUiOps {int(*resolve_host)(struct _PurpleDnsQueryData*,void(*)(struct _PurpleDnsQueryData*,struct _GSList*),void(*)(struct _PurpleDnsQueryData*,const char*));void(*destroy)(struct _PurpleDnsQueryData*);void(*_purple_reserved1)();void(*_purple_reserved2)();void(*_purple_reserved3)();void(*_purple_reserved4)();};
struct _PurpleSrvTxtQueryData {};
struct _PurpleTxtResponse {char*content;};
struct PurpleSrvTxtQueryUiOps {int(*resolve)(struct _PurpleSrvTxtQueryData*,void(*)(struct _PurpleSrvTxtQueryData*,struct _GList*),void(*)(struct _PurpleSrvTxtQueryData*,const char*));void(*destroy)(struct _PurpleSrvTxtQueryData*);void(*_purple_reserved1)();void(*_purple_reserved2)();void(*_purple_reserved3)();void(*_purple_reserved4)();};
struct PurpleIdleUiOps {long(*get_time_idle)();void(*_purple_reserved1)();void(*_purple_reserved2)();void(*_purple_reserved3)();void(*_purple_reserved4)();};
struct _PurpleMediaManager {};
struct _PurpleMimeDocument {};
struct _PurpleMimePart {};
struct _PurpleNetworkListenData {};
struct _PurplePounce {char*ui_type;enum PurplePounceEvent events;enum PurplePounceOption options;struct _PurpleAccount*pouncer;char*pouncee;struct _GHashTable*actions;int save;void*data;};
struct PurpleRequestFields {struct _GList*groups;struct _GHashTable*fields;struct _GList*required_fields;void*ui_data;};
struct PurpleRequestFieldGroup {struct PurpleRequestFields*fields_list;char*title;struct _GList*fields;};
struct _PurpleRequestField {enum PurpleRequestFieldType type;struct PurpleRequestFieldGroup*group;char*id;char*label;char*type_hint;int visible;int required;union {struct {int multiline;int masked;int editable;char*default_value;char*value;}string;struct {int default_value;int value;}integer;struct {int default_value;int value;}boolean;struct {int default_value;int value;struct _GList*labels;}choice;struct {struct _GList*items;struct _GList*icons;struct _GHashTable*item_data;struct _GList*selected;struct _GHashTable*selected_table;int multiple_selection;}list;struct {struct _PurpleAccount*default_account;struct _PurpleAccount*account;int show_all;int(*filter_func)(struct _PurpleAccount*);}account;struct {unsigned int scale_x;unsigned int scale_y;const char*buffer;unsigned long size;}image;}u;void*ui_data;};
struct PurpleRequestUiOps {void*(*request_input)(const char*,const char*,const char*,const char*,int,int,char*,const char*,void(*)(),const char*,void(*)(),struct _PurpleAccount*,const char*,struct _PurpleConversation*,void*);void*(*request_choice)(const char*,const char*,const char*,int,const char*,void(*)(),const char*,void(*)(),struct _PurpleAccount*,const char*,struct _PurpleConversation*,void*,__builtin_va_list);void*(*request_action)(const char*,const char*,const char*,int,struct _PurpleAccount*,const char*,struct _PurpleConversation*,void*,unsigned long,__builtin_va_list);void*(*request_fields)(const char*,const char*,const char*,struct PurpleRequestFields*,const char*,void(*)(),const char*,void(*)(),struct _PurpleAccount*,const char*,struct _PurpleConversation*,void*);void*(*request_file)(const char*,const char*,int,void(*)(),void(*)(),struct _PurpleAccount*,const char*,struct _PurpleConversation*,void*);void(*close_request)(enum PurpleRequestType,void*);void*(*request_folder)(const char*,const char*,void(*)(),void(*)(),struct _PurpleAccount*,const char*,struct _PurpleConversation*,void*);void*(*request_action_with_icon)(const char*,const char*,const char*,int,struct _PurpleAccount*,const char*,struct _PurpleConversation*,const void*,unsigned long,void*,unsigned long,__builtin_va_list);void(*_purple_reserved1)();void(*_purple_reserved2)();void(*_purple_reserved3)();};
struct _PurpleSavedStatus {};
struct _PurpleSavedStatusSub {};
struct _PurpleSmiley {};
struct _PurpleSoundUiOps {void(*init)();void(*uninit)();void(*play_file)(const char*);void(*play_event)(enum _PurpleSoundEventID);void(*_purple_reserved1)();void(*_purple_reserved2)();void(*_purple_reserved3)();void(*_purple_reserved4)();};
struct _PurpleTheme {struct _GObject parent;void*priv;};
struct _PurpleSoundTheme {struct _PurpleTheme parent;void*priv;};
struct _PurpleThemeLoader {struct _GObject parent;void*priv;};
struct _PurpleStringref {};
struct _PurpleStunNatDiscovery {enum PurpleStunStatus status;enum PurpleStunNatType type;char publicip[16];char*servername;long lookup_time;};
struct _UPnPMappingAddRemove {};
int purple_log_common_sizer(struct _PurpleLog*);
void purple_status_type_add_attrs_vargs(struct _PurpleStatusType*,__builtin_va_list);
void purple_group_destroy(struct _PurpleGroup*);
void purple_xfer_prepare_thumbnail(struct _PurpleXfer*,const char*);
struct _PurpleAttentionType*purple_get_attention_type_from_code(struct _PurpleAccount*,unsigned int);
int purple_certificate_pool_delete(struct _PurpleCertificatePool*,const char*);
void purple_plugins_uninit();
void purple_idle_touch();
int purple_cipher_context_encrypt(struct _PurpleCipherContext*,const unsigned char,unsigned long,unsigned char,unsigned long*);
void purple_buddy_icons_uninit();
struct _PurpleCipher*purple_ciphers_find_cipher(const char*);
void purple_value_set_uint64(struct PurpleValue*,unsigned long);
void purple_buddy_destroy(struct _PurpleBuddy*);
void*purple_notify_email(void*,const char*,const char*,const char*,const char*,void(*)(void*),void*);
int purple_status_is_independent(const struct _PurpleStatus*);
void purple_certificate_verify_complete(struct _PurpleCertificateVerificationRequest*,enum PurpleCertificateVerificationStatus);
struct _PurpleBlistNode*purple_blist_node_get_sibling_next(struct _PurpleBlistNode*);
enum PurpleNotifyUserInfoEntryType purple_notify_user_info_entry_get_type(struct _PurpleNotifyUserInfoEntry*);
void purple_signal_disconnect(void*,const char*,void*,void(*)());
void purple_roomlist_set_fields(struct _PurpleRoomlist*,struct _GList*);
void purple_blist_node_remove_setting(struct _PurpleBlistNode*,const char*);
struct _GList*purple_notify_searchresults_row_get(struct PurpleNotifySearchResults*,unsigned int);
void purple_account_remove_buddy(struct _PurpleAccount*,struct _PurpleBuddy*,struct _PurpleGroup*);
const char*purple_version_check(unsigned int,unsigned int,unsigned int);
struct _UPnPMappingAddRemove*purple_upnp_remove_port_mapping(unsigned short,const char*,void(*)(int,void*),void*);
void purple_presence_set_idle(struct _PurplePresence*,int,long);
void purple_debug_warning(const char*,const char*,...);
void purple_blist_remove_buddy(struct _PurpleBuddy*);
void purple_upnp_cancel_port_mapping(struct _UPnPMappingAddRemove*);
const char*purple_request_field_string_get_value(const struct _PurpleRequestField*);
struct _PurpleStatusType*purple_status_get_type(const struct _PurpleStatus*);
const char*purple_account_get_alias(const struct _PurpleAccount*);
void purple_conv_send_confirm(struct _PurpleConversation*,const char*);
struct _PurpleTheme*purple_theme_manager_load_theme(const char*,const char*);
void purple_theme_manager_for_each_theme(void(*)(struct _PurpleTheme*));
void purple_theme_manager_unregister_type(struct _PurpleThemeLoader*);
void purple_theme_manager_register_type(struct _PurpleThemeLoader*);
struct _PurpleStatusAttr*purple_status_type_get_attr(const struct _PurpleStatusType*,const char*);
int purple_str_has_suffix(const char*,const char*);
void purple_theme_manager_add_theme(struct _PurpleTheme*);
struct _PurpleTheme*purple_theme_manager_find_theme(const char*,const char*);
void purple_roomlist_set_ui_ops(struct _PurpleRoomlistUiOps*);
void purple_contact_invalidate_priority_buddy(struct _PurpleContact*);
void purple_theme_manager_uninit();
unsigned long purple_theme_manager_get_type();
void purple_stun_init();
struct _GList*purple_conv_chat_set_users(struct _PurpleConvChat*,struct _GList*);
void purple_plugin_pref_set_format_type(struct _PurplePluginPref*,enum PurpleStringFormatType);
struct _PurpleStunNatDiscovery*purple_stun_discover(void(*)(struct _PurpleStunNatDiscovery*));
unsigned long purple_stringref_len(const struct _PurpleStringref*);
int purple_stringref_cmp(const struct _PurpleStringref*,const struct _PurpleStringref*);
void purple_blist_set_ui_data(void*);
const char*purple_stringref_value(const struct _PurpleStringref*);
void purple_stringref_unref(struct _PurpleStringref*);
unsigned long purple_mime_part_get_length(struct _PurpleMimePart*);
void purple_marshal_VOID__POINTER_POINTER(void(*)(),__builtin_va_list,void*,void**);
struct _PurpleStringref*purple_stringref_new_noref(const char*);
unsigned long purple_sound_theme_loader_get_type();
void purple_media_end(struct _PurpleMedia*,const char*,const char*);
const char*purple_theme_loader_get_type_string(struct _PurpleThemeLoader*);
unsigned long purple_theme_loader_get_type();
void purple_sound_theme_set_file(struct _PurpleSoundTheme*,const char*,const char*);
void purple_media_manager_set_backend_type(struct _PurpleMediaManager*,unsigned long);
void purple_pounces_register_handler(const char*,void(*)(struct _PurplePounce*,enum PurplePounceEvent,void*),void(*)(struct _PurplePounce*),void(*)(struct _PurplePounce*));
void purple_theme_set_image(struct _PurpleTheme*,const char*);
char*purple_theme_get_image_full(struct _PurpleTheme*);
const char*purple_date_format_long(const struct tm*);
const char*purple_theme_get_image(struct _PurpleTheme*);
void purple_theme_set_dir(struct _PurpleTheme*,const char*);
unsigned int purple_prefs_connect_callback(void*,const char*,void(*)(const char*,enum _PurplePrefType,const void*,void*),void*);
void purple_value_set_int(struct PurpleValue*,int);
void purple_status_attr_destroy(struct _PurpleStatusAttr*);
void purple_theme_set_author(struct _PurpleTheme*,const char*);
unsigned long purple_media_state_changed_get_type();
struct PurpleSrvTxtQueryUiOps*purple_srv_txt_query_get_ui_ops();
char*purple_markup_linkify(const char*);
unsigned long purple_xfer_get_bytes_remaining(const struct _PurpleXfer*);
void purple_theme_set_name(struct _PurpleTheme*,const char*);
unsigned long purple_theme_get_type();
unsigned int purple_timeout_add_seconds(unsigned int,int(*)(void*),void*);
void*purple_sounds_get_handle();
int purple_running_kde();
void purple_blist_update_buddy_icon(struct _PurpleBuddy*);
void purple_sound_init();
struct _PurpleSoundUiOps*purple_sound_get_ui_ops();
void purple_sound_set_ui_ops(struct _PurpleSoundUiOps*);
void purple_marshal_POINTER__POINTER(void(*)(),__builtin_va_list,void*,void**);
struct _PurpleSrvTxtQueryData*purple_srv_resolve_account(struct _PurpleAccount*,const char*,const char*,const char*,void(*)(struct _PurpleSrvResponse*,int,void*),void*);
void purple_sound_play_event(enum _PurpleSoundEventID,const struct _PurpleAccount*);
void purple_presence_add_status(struct _PurplePresence*,struct _PurpleStatus*);
void purple_plugins_unload_all();
void purple_account_set_proxy_info(struct _PurpleAccount*,struct PurpleProxyInfo*);
void*purple_notify_userinfo(struct _PurpleConnection*,const char*,struct _PurpleNotifyUserInfo*,void(*)(void*),void*);
void purple_smileys_init();
unsigned long purple_imgstore_get_size(struct _PurpleStoredImage*);
const char*purple_smileys_get_storing_dir();
struct _PurpleBlistNode*purple_blist_node_get_sibling_prev(struct _PurpleBlistNode*);
unsigned long purple_xfer_get_bytes_sent(const struct _PurpleXfer*);
struct _PurpleSmiley*purple_smileys_find_by_shortcut(const char*);
struct _GList*purple_smileys_get_all();
void purple_whiteboard_draw_list_destroy(struct _GList*);
void purple_idle_set(long);
void purple_marshal_POINTER__POINTER_POINTER(void(*)(),__builtin_va_list,void*,void**);
void purple_conv_chat_user_set_flags(struct _PurpleConvChat*,const char*,enum PurpleConvChatBuddyFlags);
const char*purple_smiley_get_extension(const struct _PurpleSmiley*);
const void*purple_smiley_get_data(const struct _PurpleSmiley*,unsigned long*);
char*purple_strreplace(const char*,const char*,const char*);
struct _PurpleRoomlistField*purple_roomlist_field_new(enum PurpleRoomlistFieldType,const char*,const char*,int);
struct _PurpleSrvTxtQueryData*purple_srv_resolve(const char*,const char*,const char*,void(*)(struct _PurpleSrvResponse*,int,void*),void*);
void purple_status_set_active_with_attrs(struct _PurpleStatus*,int,__builtin_va_list);
struct _GList*purple_log_common_lister(enum PurpleLogType,const char*,struct _PurpleAccount*,const char*,struct _PurpleLogLogger*);
char*purple_fd_get_ip(int);
void purple_request_field_bool_set_value(struct _PurpleRequestField*,int);
const char*purple_smiley_get_shortcut(const struct _PurpleSmiley*);
void purple_pounce_destroy(struct _PurplePounce*);
struct _GList*purple_log_get_system_logs(struct _PurpleAccount*);
void purple_marshal_VOID__POINTER_POINTER_POINTER_POINTER_POINTER(void(*)(),__builtin_va_list,void*,void**);
void purple_account_set_enabled(struct _PurpleAccount*,const char*,int);
void purple_whiteboard_draw_line(struct _PurpleWhiteboard*,int,int,int,int,int,int);
struct _PurpleSmiley*purple_smiley_new_from_file(const char*,const char*);
void purple_xfer_set_start_fnc(struct _PurpleXfer*,void(*)(struct _PurpleXfer*));
void purple_util_chrreplace(char*,char,char);
void purple_savedstatuses_uninit();
void purple_savedstatuses_init();
void*purple_savedstatuses_get_handle();
void purple_savedstatus_activate_for_account(const struct _PurpleSavedStatus*,struct _PurpleAccount*);
void purple_savedstatus_activate(struct _PurpleSavedStatus*);
const char*purple_savedstatus_substatus_get_message(const struct _PurpleSavedStatusSub*);
void purple_account_set_protocol_id(struct _PurpleAccount*,const char*);
struct _PurpleProxyConnectData*purple_proxy_connect_udp(void*,struct _PurpleAccount*,const char*,int,void(*)(void*,int,const char*),void*);
const struct _PurpleStatusType*purple_savedstatus_substatus_get_type(const struct _PurpleSavedStatusSub*);
struct _PurpleSavedStatusSub*purple_savedstatus_get_substatus(const struct _PurpleSavedStatus*,const struct _PurpleAccount*);
int purple_savedstatus_has_substatuses(const struct _PurpleSavedStatus*);
long purple_savedstatus_get_creation_time(const struct _PurpleSavedStatus*);
const char*purple_savedstatus_get_message(const struct _PurpleSavedStatus*);
enum PurpleStatusPrimitive purple_savedstatus_get_type(const struct _PurpleSavedStatus*);
void purple_xfer_set_filename(struct _PurpleXfer*,const char*);
const char*purple_savedstatus_get_title(const struct _PurpleSavedStatus*);
struct _PurpleSavedStatus*purple_savedstatus_find_transient_by_type_and_message(enum PurpleStatusPrimitive,const char*);
struct _PurpleSavedStatus*purple_savedstatus_find_by_creation_time(long);
int purple_certificate_pool_store(struct _PurpleCertificatePool*,const char*,struct _PurpleCertificate*);
struct _PurpleSavedStatus*purple_savedstatus_get_startup();
void purple_savedstatus_set_idleaway(int);
void purple_smiley_delete(struct _PurpleSmiley*);
void purple_account_option_add_list_item(struct PurpleAccountOption*,const char*,const char*);
struct _PurpleSavedStatus*purple_savedstatus_get_default();
struct _PurpleSavedStatus*purple_savedstatus_get_current();
void purple_signal_get_values(void*,const char*,struct PurpleValue**,int*,struct PurpleValue***);
struct _GList*purple_savedstatuses_get_popular(unsigned int);
void purple_savedstatus_delete_by_status(struct _PurpleSavedStatus*);
void purple_marshal_INT__POINTER_POINTER_POINTER_POINTER_POINTER(void(*)(),__builtin_va_list,void*,void**);
unsigned long purple_circ_buffer_get_max_read(const struct _PurpleCircBuffer*);
void purple_cipher_context_set_iv(struct _PurpleCipherContext*,unsigned char*,unsigned long);
int purple_savedstatus_delete(const char*);
void purple_savedstatus_unset_substatus(struct _PurpleSavedStatus*,const struct _PurpleAccount*);
void purple_request_field_list_set_selected(struct _PurpleRequestField*,struct _GList*);
void purple_savedstatus_set_message(struct _PurpleSavedStatus*,const char*);
void purple_savedstatus_set_type(struct _PurpleSavedStatus*,enum PurpleStatusPrimitive);
struct _PurpleStoredImage*purple_imgstore_unref(struct _PurpleStoredImage*);
struct PurpleRequestUiOps*purple_request_get_ui_ops();
void purple_conv_im_stop_send_typed_timeout(struct _PurpleConvIm*);
void purple_marshal_VOID__INT(void(*)(),__builtin_va_list,void*,void**);
void*purple_request_folder(void*,const char*,const char*,void(*)(),void(*)(),struct _PurpleAccount*,const char*,struct _PurpleConversation*,void*);
void*purple_request_file(void*,const char*,const char*,int,void(*)(),void(*)(),struct _PurpleAccount*,const char*,struct _PurpleConversation*,void*);
void purple_request_close_with_handle(void*);
void purple_request_close(enum PurpleRequestType,void*);
void*purple_request_fields(void*,const char*,const char*,const char*,struct PurpleRequestFields*,const char*,void(*)(),const char*,void(*)(),struct _PurpleAccount*,const char*,struct _PurpleConversation*,void*);
void*purple_request_action_with_icon_varg(void*,const char*,const char*,const char*,int,struct _PurpleAccount*,const char*,struct _PurpleConversation*,const void*,unsigned long,void*,unsigned long,__builtin_va_list);
void*purple_request_action_with_icon(void*,const char*,const char*,const char*,int,struct _PurpleAccount*,const char*,struct _PurpleConversation*,const void*,unsigned long,void*,unsigned long,...);
void*purple_request_action_varg(void*,const char*,const char*,const char*,int,struct _PurpleAccount*,const char*,struct _PurpleConversation*,void*,unsigned long,__builtin_va_list);
int purple_markup_find_tag(const char*,const char*,const char**,const char**,struct _GData**);
void*purple_request_action(void*,const char*,const char*,const char*,int,struct _PurpleAccount*,const char*,struct _PurpleConversation*,void*,unsigned long,...);
void*purple_request_choice_varg(void*,const char*,const char*,const char*,int,const char*,void(*)(),const char*,void(*)(),struct _PurpleAccount*,const char*,struct _PurpleConversation*,void*,__builtin_va_list);
void purple_plugin_ipc_unregister(struct _PurplePlugin*,const char*);
void*purple_request_choice(void*,const char*,const char*,const char*,int,const char*,void(*)(),const char*,void(*)(),struct _PurpleAccount*,const char*,struct _PurpleConversation*,void*,...);
void purple_conversation_autoset_title(struct _PurpleConversation*);
int(*purple_request_field_account_get_filter(const struct _PurpleRequestField*))(struct _PurpleAccount*);
void purple_cipher_context_append(struct _PurpleCipherContext*,const unsigned char*,unsigned long);
int purple_request_field_account_get_show_all(const struct _PurpleRequestField*);
char*purple_markup_strip_html(const char*);
struct _PurpleAccount*purple_request_field_account_get_value(const struct _PurpleRequestField*);
struct _PurpleAccount*purple_request_field_account_get_default_value(const struct _PurpleRequestField*);
void purple_request_field_account_set_filter(struct _PurpleRequestField*,int(*)(struct _PurpleAccount*));
void purple_request_field_account_set_value(struct _PurpleRequestField*,struct _PurpleAccount*);
void purple_request_field_account_set_default_value(struct _PurpleRequestField*,struct _PurpleAccount*);
unsigned int purple_request_field_image_get_scale_y(struct _PurpleRequestField*);
void purple_pounce_set_save(struct _PurplePounce*,int);
unsigned int purple_request_field_image_get_scale_x(struct _PurpleRequestField*);
void purple_roomlist_set_in_progress(struct _PurpleRoomlist*,int);
const char*purple_request_field_image_get_buffer(struct _PurpleRequestField*);
void purple_request_field_image_set_scale(struct _PurpleRequestField*,unsigned int,unsigned int);
struct _PurpleRequestField*purple_request_field_image_new(const char*,const char*,const char*,unsigned long);
struct _PurpleRequestField*purple_request_field_label_new(const char*,const char*);
struct _GList*purple_request_field_list_get_icons(const struct _PurpleRequestField*);
struct _GList*purple_request_field_list_get_items(const struct _PurpleRequestField*);
void purple_menu_action_free(struct _PurpleMenuAction*);
unsigned long purple_ssl_write(struct _PurpleSslConnection*,const void*,unsigned long);
struct _GList*purple_request_field_list_get_selected(const struct _PurpleRequestField*);
long purple_str_to_time(const char*,int,struct tm*,long*,const char**);
void purple_ssl_uninit();
char*purple_media_codec_to_string(const struct _PurpleMediaCodec*);
void purple_savedstatus_set_substatus(struct _PurpleSavedStatus*,const struct _PurpleAccount*,const struct _PurpleStatusType*,const char*);
void purple_request_field_list_clear_selected(struct _PurpleRequestField*);
void purple_request_field_list_add_selected(struct _PurpleRequestField*,const char*);
void purple_request_field_set_visible(struct _PurpleRequestField*,int);
void purple_request_field_list_add(struct _PurpleRequestField*,const char*,void*);
struct _PurpleStatus*purple_account_get_active_status(const struct _PurpleAccount*);
int purple_request_field_list_get_multi_select(const struct _PurpleRequestField*);
struct _PurpleLogLogger*purple_log_logger_get();
struct _PurpleRequestField*purple_request_field_list_new(const char*,const char*);
unsigned long purple_signal_connect(void*,const char*,void*,void(*)(),void*);
struct _GList*purple_conversation_get_message_history(struct _PurpleConversation*);
void purple_account_option_destroy(struct PurpleAccountOption*);
void purple_whiteboard_draw_point(struct _PurpleWhiteboard*,int,int,int,int);
int purple_request_field_choice_get_value(const struct _PurpleRequestField*);
int purple_request_field_choice_get_default_value(const struct _PurpleRequestField*);
void purple_request_field_choice_set_value(struct _PurpleRequestField*,int);
void purple_request_set_ui_ops(struct PurpleRequestUiOps*);
unsigned long purple_media_caps_get_type();
struct _PurpleRequestField*purple_request_field_choice_new(const char*,const char*,int);
void purple_marshal_VOID__POINTER_POINTER_UINT_UINT(void(*)(),__builtin_va_list,void*,void**);
void purple_notify_searchresults_new_rows(struct _PurpleConnection*,struct PurpleNotifySearchResults*,void*);
int purple_request_field_bool_get_value(const struct _PurpleRequestField*);
int purple_request_field_bool_get_default_value(const struct _PurpleRequestField*);
const char*purple_conv_chat_get_ignored_user(const struct _PurpleConvChat*,const char*);
void purple_plugin_pref_frame_add(struct _PurplePluginPrefFrame*,struct _PurplePluginPref*);
int purple_request_field_int_get_value(const struct _PurpleRequestField*);
int purple_request_field_int_get_default_value(const struct _PurpleRequestField*);
void purple_request_field_int_set_value(struct _PurpleRequestField*,int);
int purple_plugin_reload(struct _PurplePlugin*);
struct _PurpleDnsQueryData*purple_dnsquery_a_account(struct _PurpleAccount*,const char*,int,void(*)(struct _GSList*,void*,const char*),void*);
struct _PurpleRequestField*purple_request_field_int_new(const char*,const char*,int);
void purple_value_set_string(struct PurpleValue*,const char*);
int purple_log_common_total_sizer(enum PurpleLogType,const char*,struct _PurpleAccount*,const char*);
int purple_buddy_icons_node_has_custom_icon(struct _PurpleBlistNode*);
const char*purple_primitive_get_id_from_type(enum PurpleStatusPrimitive);
char*purple_str_seconds_to_string(unsigned int);
int purple_request_field_string_is_masked(const struct _PurpleRequestField*);
void purple_blist_remove_account(struct _PurpleAccount*);
int purple_request_field_string_is_multiline(const struct _PurpleRequestField*);
char*purple_markup_slice(const char*,unsigned int,unsigned int);
const char*purple_upnp_get_public_ip();
const char*purple_request_field_string_get_default_value(const struct _PurpleRequestField*);
void purple_request_field_string_set_editable(struct _PurpleRequestField*,int);
void purple_request_field_string_set_masked(struct _PurpleRequestField*,int);
void*purple_accounts_get_handle();
void purple_request_field_string_set_value(struct _PurpleRequestField*,const char*);
struct _PurpleRequestField*purple_request_field_string_new(const char*,const char*,const char*,int);
void purple_request_field_set_ui_data(struct _PurpleRequestField*,void*);
void*purple_request_field_get_ui_data(const struct _PurpleRequestField*);
const char*purple_request_field_get_type_hint(const struct _PurpleRequestField*);
int purple_smiley_set_shortcut(struct _PurpleSmiley*,const char*);
const char*purple_request_field_get_label(const struct _PurpleRequestField*);
int purple_blist_get_group_online_count(struct _PurpleGroup*);
void purple_blist_alias_chat(struct _PurpleChat*,const char*);
struct PurpleNotifySearchResults*purple_notify_searchresults_new();
struct PurpleRequestFieldGroup*purple_request_field_get_group(const struct _PurpleRequestField*);
enum PurpleRequestFieldType purple_request_field_get_type(const struct _PurpleRequestField*);
void purple_blist_rename_group(struct _PurpleGroup*,const char*);
int purple_socket_speaks_ipv4(int);
char*purple_mime_decode_field(const char*);
void purple_request_field_set_type_hint(struct _PurpleRequestField*,const char*);
struct _PurpleBuddyIcon*purple_buddy_icon_new(struct _PurpleAccount*,const char*,void*,unsigned long,const char*);
struct _PurpleRequestField*purple_request_field_new(const char*,const char*,enum PurpleRequestFieldType);
void purple_account_set_check_mail(struct _PurpleAccount*,int);
const char*purple_request_field_group_get_title(const struct PurpleRequestFieldGroup*);
void purple_request_field_group_destroy(struct PurpleRequestFieldGroup*);
struct PurpleRequestFieldGroup*purple_request_field_group_new(const char*);
struct _PurpleStatusType*purple_status_type_new_full(enum PurpleStatusPrimitive,const char*,const char*,int,int,int);
struct _PurpleAccount*purple_request_fields_get_account(const struct PurpleRequestFields*,const char*);
int purple_request_fields_get_choice(const struct PurpleRequestFields*,const char*);
int purple_request_fields_get_integer(const struct PurpleRequestFields*,const char*);
unsigned long purple_smiley_get_type();
int purple_request_fields_all_required_filled(const struct PurpleRequestFields*);
void purple_account_destroy(struct _PurpleAccount*);
int purple_request_fields_is_field_required(const struct PurpleRequestFields*,const char*);
struct _GList*purple_request_fields_get_required(const struct PurpleRequestFields*);
int purple_request_fields_exists(const struct PurpleRequestFields*,const char*);
void purple_buddy_set_protocol_data(struct _PurpleBuddy*,void*);
unsigned int purple_media_candidate_get_component_id(struct _PurpleMediaCandidate*);
int purple_log_compare(const void*,const void*);
void purple_marshal_POINTER__POINTER_INT64(void(*)(),__builtin_va_list,void*,void**);
void purple_request_fields_destroy(struct PurpleRequestFields*);
void purple_prefs_add_string_list(const char*,struct _GList*);
struct PurpleRequestFields*purple_request_fields_new();
void purple_pounces_uninit();
void purple_pounces_init();
char*purple_util_format_song_info(const char*,const char*,const char*,void*);
struct _GList*purple_pounces_get_all_for_ui(const char*);
struct _PurpleBuddy*purple_find_buddy(struct _PurpleAccount*,const char*);
struct _GList*purple_pounces_get_all();
void purple_pounces_unregister_handler(const char*);
const char*purple_sound_theme_get_file(struct _PurpleSoundTheme*,const char*);
void purple_blist_set_visible(int);
struct _PurplePounce*purple_find_pounce(const struct _PurpleAccount*,const char*,enum PurplePounceEvent);
void purple_marshal_BOOLEAN__INT_POINTER(void(*)(),__builtin_va_list,void*,void**);
void purple_pounce_execute(const struct _PurpleAccount*,const char*,enum PurplePounceEvent);
void*purple_pounce_get_data(const struct _PurplePounce*);
const char*purple_pounce_action_get_attribute(const struct _PurplePounce*,const char*,const char*);
struct _PurplePluginPref*purple_plugin_pref_new_with_label(const char*);
int purple_pounce_get_save(const struct _PurplePounce*);
int purple_account_is_connected(const struct _PurpleAccount*);
const char*purple_pounce_get_pouncee(const struct _PurplePounce*);
struct _PurpleAccount*purple_pounce_get_pouncer(const struct _PurplePounce*);
const char*purple_util_get_image_extension(const void*,unsigned long);
enum PurplePounceOption purple_pounce_get_options(const struct _PurplePounce*);
enum PurplePounceEvent purple_pounce_get_events(const struct _PurplePounce*);
void*purple_plugins_get_handle();
void purple_conv_chat_set_nick(struct _PurpleConvChat*,const char*);
void purple_pounce_action_set_attribute(struct _PurplePounce*,const char*,const char*,const char*);
void purple_pounce_action_set_enabled(struct _PurplePounce*,const char*,int);
void purple_pounce_action_register(struct _PurplePounce*,const char*);
void purple_pounce_set_pouncee(struct _PurplePounce*,const char*);
void purple_pounce_set_pouncer(struct _PurplePounce*,struct _PurpleAccount*);
void purple_xfers_uninit();
unsigned int purple_cmd_register(const char*,const char*,enum _PurpleCmdPriority,enum _PurpleCmdFlag,const char*,enum _PurpleCmdRet(*)(struct _PurpleConversation*,const char*,char**,char**,void*),const char*,void*);
void purple_signal_unregister(void*,const char*);
void purple_pounce_set_events(struct _PurplePounce*,enum PurplePounceEvent);
void purple_attention_type_set_unlocalized_name(struct _PurpleAttentionType*,const char*);
void purple_cmds_init();
void purple_pounce_destroy_all_by_buddy(struct _PurpleBuddy*);
void purple_certificate_add_ca_search_path(const char*);
int purple_media_is_initiator(struct _PurpleMedia*,const char*,const char*);
void purple_pounce_destroy_all_by_account(struct _PurpleAccount*);
void purple_smiley_set_data(struct _PurpleSmiley*,unsigned char*,unsigned long);
struct _PurplePounce*purple_pounce_new(const char*,struct _PurpleAccount*,const char*,enum PurplePounceEvent,enum PurplePounceOption);
char*purple_ntlm_gen_type3(const char*,const char*,const char*,const char*,const unsigned char*,unsigned int*);
int purple_conv_chat_is_user_ignored(const struct _PurpleConvChat*,const char*);
char*purple_ntlm_gen_type1(const char*,const char*);
void purple_network_uninit();
void purple_conv_chat_clear_users(struct _PurpleConvChat*);
void purple_network_init();
int purple_network_convert_idn_to_ascii(const char*,char**);
void purple_network_remove_port_mapping(int);
int purple_xfer_is_completed(const struct _PurpleXfer*);
void purple_imgstore_uninit();
const char*purple_network_get_turn_ip();
void purple_network_set_turn_server(const char*);
const char*purple_network_get_stun_ip();
void purple_network_set_stun_server(const char*);
void*purple_network_get_handle();
void purple_pmp_init();
int purple_network_is_available();
unsigned short purple_network_get_port_from_fd(int);
void purple_network_listen_cancel(struct _PurpleNetworkListenData*);
int purple_presence_is_idle(const struct _PurplePresence*);
int purple_account_get_silence_suppression(const struct _PurpleAccount*);
int purple_presence_is_status_primitive_active(const struct _PurplePresence*,enum PurpleStatusPrimitive);
struct _PurpleBuddyList*purple_blist_new();
void purple_account_set_username(struct _PurpleAccount*,const char*);
struct _PurpleNetworkListenData*purple_network_listen(unsigned short,int,void(*)(int,void*),void*);
const char*purple_network_get_my_ip(int);
struct _GList*purple_network_get_all_local_system_ips();
void purple_notify_searchresults_button_add_labeled(struct PurpleNotifySearchResults*,const char*,void(*)(struct _PurpleConnection*,struct _GList*,void*));
const char*purple_network_get_local_system_ip(int);
const char*purple_network_get_public_ip();
struct _PurpleStatus*purple_presence_get_active_status(const struct _PurplePresence*);
int purple_savedstatus_is_transient(const struct _PurpleSavedStatus*);
int purple_core_ensure_single_instance();
int purple_pmp_create_map(enum PurplePmpType,unsigned short,unsigned short,int);
char*purple_pmp_get_public_ip();
void purple_network_force_online();
void purple_mime_part_set_data(struct _PurpleMimePart*,const char*);
struct _PurpleStringref*purple_stringref_ref(struct _PurpleStringref*);
struct PurpleProxyInfo*purple_global_proxy_get_info();
void purple_mime_part_set_field(struct _PurpleMimePart*,const char*,const char*);
const char*purple_mime_part_get_field(struct _PurpleMimePart*,const char*);
struct _PurpleMimePart*purple_mime_part_new(struct _PurpleMimeDocument*);
void purple_log_uninit();
struct _GList*purple_mime_document_get_parts(struct _PurpleMimeDocument*);
struct _PurpleStoredImage*purple_buddy_icons_node_find_custom_icon(struct _PurpleBlistNode*);
void purple_conversation_write(struct _PurpleConversation*,const char*,const char*,enum PurpleMessageFlags,long);
void purple_xfer_error(enum PurpleXferType,struct _PurpleAccount*,const char*,const char*);
void purple_mime_document_set_field(struct _PurpleMimeDocument*,const char*,const char*);
const char*purple_mime_document_get_field(struct _PurpleMimeDocument*,const char*);
struct _GList*purple_mime_document_get_fields(struct _PurpleMimeDocument*);
const char*purple_account_user_split_get_text(const struct PurpleAccountUserSplit*);
void purple_xfer_add(struct _PurpleXfer*);
void purple_whiteboard_set_prpl_ops(struct _PurpleWhiteboard*,struct _PurpleWhiteboardPrplOps*);
struct _PurpleMimeDocument*purple_mime_document_parsen(const char*,unsigned long);
int purple_certificate_unregister_verifier(struct _PurpleCertificateVerifier*);
struct _PurpleSrvTxtQueryData*purple_txt_resolve(const char*,const char*,void(*)(struct _GList*,void*),void*);
void purple_account_add_buddies(struct _PurpleAccount*,struct _GList*);
int purple_privacy_deny_add(struct _PurpleAccount*,const char*,int);
void purple_mime_document_free(struct _PurpleMimeDocument*);
struct _PurpleMimeDocument*purple_mime_document_new();
unsigned long purple_media_manager_get_backend_type(struct _PurpleMediaManager*);
char*purple_sound_theme_get_file_full(struct _PurpleSoundTheme*,const char*);
enum PurpleMediaCaps purple_media_manager_get_ui_caps(struct _PurpleMediaManager*);
void purple_media_manager_set_ui_caps(struct _PurpleMediaManager*,enum PurpleMediaCaps);
void purple_media_manager_remove_output_windows(struct _PurpleMediaManager*,struct _PurpleMedia*,const char*,const char*);
int purple_media_manager_remove_output_window(struct _PurpleMediaManager*,unsigned long);
const char*purple_attention_type_get_name(const struct _PurpleAttentionType*);
void purple_certificate_uninit();
int purple_media_manager_create_output_window(struct _PurpleMediaManager*,struct _PurpleMedia*,const char*,const char*);
void purple_media_manager_remove_media(struct _PurpleMediaManager*,struct _PurpleMedia*);
struct PurpleValue*purple_value_dup(const struct PurpleValue*);
struct _GList*purple_media_manager_get_media(struct _PurpleMediaManager*);
const char*purple_imgstore_get_extension(struct _PurpleStoredImage*);
struct _PurpleMedia*purple_media_manager_create_media(struct _PurpleMediaManager*,struct _PurpleAccount*,const char*,const char*,int);
struct _PurpleMediaManager*purple_media_manager_get();
unsigned long purple_media_manager_get_type();
void purple_idle_uninit();
const char*purple_strcasestr(const char*,const char*);
struct PurpleIdleUiOps*purple_idle_get_ui_ops();
char*purple_str_size_to_units(unsigned long);
char*purple_srv_txt_query_get_query(struct _PurpleSrvTxtQueryData*);
void purple_theme_set_description(struct _PurpleTheme*,const char*);
void purple_srv_txt_query_set_ui_ops(struct PurpleSrvTxtQueryUiOps*);
void purple_srv_txt_query_destroy(struct _PurpleSrvTxtQueryData*);
void purple_plugin_pref_frame_destroy(struct _PurplePluginPrefFrame*);
void purple_conversation_set_logging(struct _PurpleConversation*,int);
void purple_txt_cancel(struct _PurpleSrvTxtQueryData*);
struct _PurpleMimeDocument*purple_mime_document_parse(const char*);
struct _PurpleSrvTxtQueryData*purple_txt_resolve_account(struct _PurpleAccount*,const char*,const char*,void(*)(struct _GList*,void*),void*);
void purple_srv_cancel(struct _PurpleSrvTxtQueryData*);
void purple_dnsquery_uninit();
void purple_dnsquery_init();
unsigned short purple_dnsquery_get_port(struct _PurpleDnsQueryData*);
char*purple_dnsquery_get_host(struct _PurpleDnsQueryData*);
struct PurpleDnsQueryUiOps*purple_dnsquery_get_ui_ops();
void purple_dnsquery_set_ui_ops(struct PurpleDnsQueryUiOps*);
struct _PurpleDnsQueryData*purple_dnsquery_a(const char*,int,void(*)(struct _GSList*,void*,const char*),void*);
void purple_desktop_item_unref(struct _PurpleDesktopItem*);
struct _PurpleDesktopItem*purple_desktop_item_copy(const struct _PurpleDesktopItem*);
const char*purple_desktop_item_get_string(const struct _PurpleDesktopItem*,const char*);
struct _PurpleDesktopItem*purple_desktop_item_new_from_file(const char*);
unsigned long purple_desktop_item_get_type();
void purple_debug_init();
void purple_notify_user_info_add_section_header(struct _PurpleNotifyUserInfo*,const char*);
struct _PurpleLog*purple_log_new(enum PurpleLogType,const char*,struct _PurpleAccount*,struct _PurpleConversation*,long,const struct tm*);
void purple_plugin_pref_set_bounds(struct _PurplePluginPref*,int,int);
struct PurpleDebugUiOps*purple_debug_get_ui_ops();
void purple_debug_set_ui_ops(struct PurpleDebugUiOps*);
struct _PurpleRoomlist*purple_roomlist_get_list(struct _PurpleConnection*);
int purple_debug_is_unsafe();
void purple_debug_set_unsafe(int);
void purple_plugins_register_load_notify_cb(void(*)(struct _PurplePlugin*,void*),void*);
void purple_debug_set_verbose(int);
int purple_debug_is_enabled();
void purple_debug_set_enabled(int);
void purple_debug_fatal(const char*,const char*,...);
void purple_debug_error(const char*,const char*,...);
void purple_debug_info(const char*,const char*,...);
void purple_debug_misc(const char*,const char*,...);
void purple_plugin_pref_set_label(struct _PurplePluginPref*,const char*);
struct _GHashTable*purple_core_get_ui_info();
int purple_core_migrate();
struct PurpleCoreUiOps*purple_core_get_ui_ops();
void purple_core_set_ui_ops(struct PurpleCoreUiOps*);
const char*purple_plugin_get_name(const struct _PurplePlugin*);
const char*purple_core_get_ui();
const char*purple_core_get_version();
int purple_core_quit_cb(void*);
void purple_core_quit();
void purple_cmds_uninit();
int purple_conv_chat_has_left(struct _PurpleConvChat*);
void*purple_cmds_get_handle();
void purple_account_request_change_password(struct _PurpleAccount*);
struct _PurpleConvIm*purple_conversation_get_im_data(const struct _PurpleConversation*);
struct _PurplePluginPref*purple_plugin_pref_new_with_name_and_label(const char*,const char*);
int purple_input_remove(unsigned int);
unsigned short purple_value_get_ushort(const struct PurpleValue*);
void purple_circ_buffer_append(struct _PurpleCircBuffer*,const void*,unsigned long);
void purple_circ_buffer_destroy(struct _PurpleCircBuffer*);
struct _PurpleBlistNode*purple_blist_node_get_first_child(struct _PurpleBlistNode*);
char*purple_cipher_http_digest_calculate_response(const char*,const char*,const char*,const char*,const char*,const char*,const char*,const char*,const char*);
char*purple_unescape_html(const char*);
char*purple_cipher_http_digest_calculate_session_key(const char*,const char*,const char*,const char*,const char*,const char*);
void*purple_cipher_context_get_data(struct _PurpleCipherContext*);
void purple_prefs_disconnect_by_handle(void*);
void purple_imgstore_unref_by_id(int);
void purple_conv_im_update_typing(struct _PurpleConvIm*);
void purple_cipher_context_set_key_with_len(struct _PurpleCipherContext*,const unsigned char*,unsigned long);
struct _PurpleBlistUiOps*purple_blist_get_ui_ops();
enum _PurpleCipherBatchMode purple_cipher_context_get_batch_mode(struct _PurpleCipherContext*);
void purple_cipher_context_set_batch_mode(struct _PurpleCipherContext*,enum _PurpleCipherBatchMode);
struct _IO_FILE*purple_mkstemp(char**,int);
unsigned long purple_cipher_context_get_key_size(struct _PurpleCipherContext*);
void purple_cipher_context_set_key(struct _PurpleCipherContext*,const unsigned char*);
unsigned long purple_cipher_context_get_salt_size(struct _PurpleCipherContext*);
void purple_cipher_context_set_salt(struct _PurpleCipherContext*,unsigned char*);
int purple_cipher_context_decrypt(struct _PurpleCipherContext*,const unsigned char,unsigned long,unsigned char,unsigned long*);
int purple_cipher_context_digest_to_str(struct _PurpleCipherContext*,unsigned long,char,unsigned long*);
int purple_cipher_context_digest(struct _PurpleCipherContext*,unsigned long,unsigned char,unsigned long*);
char*purple_log_read(struct _PurpleLog*,enum PurpleLogReadFlags*);
struct _PurpleCipherContext*purple_cipher_context_new_by_name(const char*,void*);
struct _PurpleMediaCodec*purple_media_codec_new(int,const char*,enum PurpleMediaSessionType,unsigned int);
void*purple_cipher_context_get_option(struct _PurpleCipherContext*,const char*);
void purple_cipher_context_set_option(struct _PurpleCipherContext*,const char*,void*);
void purple_ciphers_uninit();
void purple_ciphers_init();
struct _PurpleMediaCodec*purple_media_codec_copy(struct _PurpleMediaCodec*);
void*purple_ciphers_get_handle();
struct _GList*purple_ciphers_get_ciphers();
int purple_ciphers_unregister_cipher(struct _PurpleCipher*);
struct _PurpleCipher*purple_ciphers_register_cipher(const char*,struct _PurpleCipherOps*);
void purple_status_destroy(struct _PurpleStatus*);
void purple_xfer_set_read_fnc(struct _PurpleXfer*,signed long(*)(unsigned char**,struct _PurpleXfer*));
enum PurpleDesktopItemType purple_desktop_item_get_entry_type(const struct _PurpleDesktopItem*);
struct _GList*purple_prefs_get_path_list(const char*);
struct _PurpleBuddyIcon*purple_buddy_icon_ref(struct _PurpleBuddyIcon*);
struct _GList*purple_plugin_pref_frame_get_prefs(struct _PurplePluginPrefFrame*);
void purple_prpl_got_user_status(struct _PurpleAccount*,const char*,const char*,...);
int purple_utf8_has_word(const char*,const char*);
void purple_value_set_pointer(struct PurpleValue*,void*);
void purple_notify_user_info_entry_set_label(struct _PurpleNotifyUserInfoEntry*,const char*);
long purple_presence_get_login_time(const struct _PurplePresence*);
struct _PurplePresence*purple_presence_new_for_conv(struct _PurpleConversation*);
void purple_account_set_privacy_type(struct _PurpleAccount*,enum _PurplePrivacyType);
void purple_request_field_destroy(struct _PurpleRequestField*);
void purple_accounts_restore_current_statuses();
void purple_connection_notice(struct _PurpleConnection*,const char*);
struct _PurpleCertificate*purple_certificate_import(struct _PurpleCertificateScheme*,const char*);
const char*purple_account_option_get_default_list_value(const struct PurpleAccountOption*);
void purple_notify_close_with_handle(void*);
void purple_signals_unregister_by_instance(void*);
struct _PurpleRequestField*purple_request_field_bool_new(const char*,const char*,int);
void purple_plugin_pref_set_masked(struct _PurplePluginPref*,int);
int purple_markup_extract_info_field(const char*,int,struct _PurpleNotifyUserInfo*,const char*,int,const char*,char,const char*,const char*,int,const char*,char*(*)(const char*,unsigned long));
int purple_roomlist_get_in_progress(struct _PurpleRoomlist*);
int purple_running_gnome();
const char*purple_imgstore_get_filename(const struct _PurpleStoredImage*);
void purple_util_set_user_dir(const char*);
void purple_account_user_split_set_reverse(struct PurpleAccountUserSplit*,int);
char purple_account_user_split_get_separator(const struct PurpleAccountUserSplit*);
enum PurpleType purple_value_get_type(const struct PurpleValue*);
void purple_value_set_ulong(struct PurpleValue*,unsigned long);
void purple_privacy_deny(struct _PurpleAccount*,const char*,int,int);
void purple_mime_document_write(struct _PurpleMimeDocument*,struct _GString*);
void purple_blist_rename_buddy(struct _PurpleBuddy*,const char*);
void purple_idle_init();
struct PurpleAccountUserSplit*purple_account_user_split_new(const char*,const char*,char);
struct _GList*purple_account_option_get_list(const struct PurpleAccountOption*);
int purple_account_option_get_masked(const struct PurpleAccountOption*);
const char*purple_account_option_get_default_string(const struct PurpleAccountOption*);
int purple_account_option_get_default_int(const struct PurpleAccountOption*);
void purple_accounts_reorder(struct _PurpleAccount*,int);
int purple_account_option_get_default_bool(const struct PurpleAccountOption*);
void purple_proxy_info_set_password(struct PurpleProxyInfo*,const char*);
struct _PurpleUtilFetchUrlData*purple_util_fetch_url_request_len_with_account(struct _PurpleAccount*,const char*,int,const char*,int,const char*,int,signed long,void(*)(struct _PurpleUtilFetchUrlData*,void*,const char*,unsigned long,const char*),void*);
enum _PurplePrefType purple_account_option_get_type(const struct PurpleAccountOption*);
const char*purple_group_get_name(struct _PurpleGroup*);
struct _PurplePresence*purple_buddy_get_presence(const struct _PurpleBuddy*);
struct _PurpleSavedStatus*purple_savedstatus_get_idleaway();
void purple_account_option_set_list(struct PurpleAccountOption*,struct _GList*);
struct _PurpleUtilFetchUrlData*purple_util_fetch_url_request(const char*,int,const char*,int,const char*,int,void(*)(struct _PurpleUtilFetchUrlData*,void*,const char*,unsigned long,const char*),void*);
void purple_account_option_set_default_string(struct PurpleAccountOption*,const char*);
void purple_account_option_set_default_int(struct PurpleAccountOption*,int);
struct _PurpleMenuAction*purple_menu_action_new(const char*,void(*)(),void*,struct _GList*);
struct PurpleAccountOption*purple_account_option_list_new(const char*,const char*,struct _GList*);
struct PurpleAccountOption*purple_account_option_int_new(const char*,const char*,int);
struct PurpleAccountOption*purple_account_option_bool_new(const char*,const char*,int);
struct PurpleAccountOption*purple_account_option_new(enum _PurplePrefType,const char*,const char*);
void purple_accounts_uninit();
char*purple_str_binary_to_ascii(const unsigned char*,unsigned int);
void purple_accounts_init();
struct _PurpleAccountUiOps*purple_accounts_get_ui_ops();
const char*purple_attention_type_get_outgoing_desc(const struct _PurpleAttentionType*);
void purple_connection_error(struct _PurpleConnection*,const char*);
void purple_prefs_set_path(const char*,const char*);
struct _GList*purple_accounts_get_all();
void purple_accounts_remove(struct _PurpleAccount*);
void purple_account_clear_current_error(struct _PurpleAccount*);
int purple_account_supports_offline_message(struct _PurpleAccount*,struct _PurpleBuddy*);
void purple_account_change_password(struct _PurpleAccount*,const char*,const char*);
void purple_account_option_set_default_bool(struct PurpleAccountOption*,int);
void purple_account_remove_buddies(struct _PurpleAccount*,struct _GList*,struct _GList*);
void purple_account_add_buddies_with_invite(struct _PurpleAccount*,struct _GList*,const char*);
struct PurpleNotifySearchColumn*purple_notify_searchresults_column_new(const char*);
void purple_value_set_enum(struct PurpleValue*,int);
void purple_account_add_buddy(struct _PurpleAccount*,struct _PurpleBuddy*);
void purple_account_destroy_log(struct _PurpleAccount*);
struct _PurpleLog*purple_account_get_log(struct _PurpleAccount*,int);
int purple_account_get_ui_bool(const struct _PurpleAccount*,const char*,const char*,int);
const char*purple_account_get_ui_string(const struct _PurpleAccount*,const char*,const char*,const char*);
int purple_media_set_remote_codecs(struct _PurpleMedia*,const char*,const char*,struct _GList*);
int purple_account_get_ui_int(const struct _PurpleAccount*,const char*,const char*,int);
int purple_account_get_bool(const struct _PurpleAccount*,const char*,int);
const char*purple_account_get_string(const struct _PurpleAccount*,const char*,const char*);
void purple_status_set_attr_boolean(struct _PurpleStatus*,const char*,int);
int purple_account_get_int(const struct _PurpleAccount*,const char*,int);
void purple_xfer_set_local_filename(struct _PurpleXfer*,const char*);
int purple_account_is_status_active(const struct _PurpleAccount*,const char*);
unsigned long purple_media_info_type_get_type();
int purple_conv_present_error(const char*,struct _PurpleAccount*,const char*);
void purple_roomlist_ref(struct _PurpleRoomlist*);
struct _PurpleStatus*purple_account_get_status(const struct _PurpleAccount*,const char*);
void*purple_request_field_list_get_data(const struct _PurpleRequestField*,const char*);
enum _PurplePrivacyType purple_account_get_privacy_type(const struct _PurpleAccount*);
struct PurpleProxyInfo*purple_account_get_proxy_info(const struct _PurpleAccount*);
int purple_account_get_enabled(const struct _PurpleAccount*,const char*);
int purple_account_get_check_mail(const struct _PurpleAccount*);
int purple_account_get_remember_password(const struct _PurpleAccount*);
const char*purple_account_get_name_for_display(const struct _PurpleAccount*);
struct _PurpleConnection*purple_account_get_connection(const struct _PurpleAccount*);
struct _PurpleStoredImage*purple_buddy_icons_find_account_icon(struct _PurpleAccount*);
void purple_prefs_add_int(const char*,int);
void purple_prefs_destroy();
unsigned long purple_media_set_output_window(struct _PurpleMedia*,const char*,const char*,unsigned long);
const char*purple_account_get_buddy_icon_path(const struct _PurpleAccount*);
const char*purple_account_get_user_info(const struct _PurpleAccount*);
void purple_upnp_init();
const char*purple_account_get_password(const struct _PurpleAccount*);
const char*purple_account_get_username(const struct _PurpleAccount*);
int purple_log_is_deletable(struct _PurpleLog*);
int purple_account_is_disconnected(const struct _PurpleAccount*);
void purple_log_common_writer(struct _PurpleLog*,const char*);
void purple_plugins_register_unload_notify_cb(void(*)(struct _PurplePlugin*,void*),void*);
int purple_conversation_do_command(struct _PurpleConversation*,const char*,const char*,char**);
int purple_account_is_connecting(const struct _PurpleAccount*);
void purple_account_set_ui_bool(struct _PurpleAccount*,const char*,const char*,int);
int purple_log_get_activity_score(enum PurpleLogType,const char*,struct _PurpleAccount*);
void purple_account_disconnect(struct _PurpleAccount*);
void purple_account_set_ui_string(struct _PurpleAccount*,const char*,const char*,const char*);
void purple_account_set_ui_int(struct _PurpleAccount*,const char*,const char*,int);
void purple_account_set_string(struct _PurpleAccount*,const char*,const char*);
char*purple_util_get_image_filename(const void*,unsigned long);
void purple_account_remove_setting(struct _PurpleAccount*,const char*);
void purple_account_clear_settings(struct _PurpleAccount*);
void purple_account_set_silence_suppression(struct _PurpleAccount*,int);
void purple_account_get_public_alias(struct _PurpleAccount*,void(*)(struct _PurpleAccount*,const char*),void(*)(struct _PurpleAccount*,const char*));
int purple_log_get_total_size(enum PurpleLogType,const char*,struct _PurpleAccount*);
void purple_account_set_status_list(struct _PurpleAccount*,const char*,int,struct _GList*);
struct _PurpleBuddy*purple_buddy_new(struct _PurpleAccount*,const char*,const char*);
void purple_ssl_input_add(struct _PurpleSslConnection*,void(*)(void*,struct _PurpleSslConnection*,enum PurpleInputCondition),void*);
void purple_account_set_status_types(struct _PurpleAccount*,struct _GList*);
unsigned long purple_media_get_type();
struct _GList*purple_request_field_group_get_fields(const struct PurpleRequestFieldGroup*);
struct _PurpleLogLogger*purple_log_logger_new(const char*,const char*,int,...);
void purple_account_set_remember_password(struct _PurpleAccount*,int);
void purple_account_set_connection(struct _PurpleAccount*,struct _PurpleConnection*);
void purple_account_set_buddy_icon_path(struct _PurpleAccount*,const char*);
void purple_account_set_user_info(struct _PurpleAccount*,const char*);
void purple_account_set_alias(struct _PurpleAccount*,const char*);
void purple_account_set_password(struct _PurpleAccount*,const char*);
void purple_account_request_change_user_info(struct _PurpleAccount*);
void purple_account_request_password(struct _PurpleAccount*,void(*)(),void(*)(),void*);
void purple_log_logger_remove(struct _PurpleLogLogger*);
void purple_account_request_close_with_account(struct _PurpleAccount*);
void*purple_account_request_authorization(struct _PurpleAccount*,const char*,const char*,const char*,const char*,int,void(*)(void*),void(*)(void*),void*);
void purple_account_request_add(struct _PurpleAccount*,const char*,const char*,const char*,const char*);
void purple_media_candidate_list_free(struct _GList*);
void purple_account_notify_added(struct _PurpleAccount*,const char*,const char*,const char*,const char*);
void purple_account_connect(struct _PurpleAccount*);
void purple_buddy_icon_update(struct _PurpleBuddyIcon*);
struct _PurpleAccount*purple_account_new(const char*,const char*);
struct _PurplePluginPrefFrame*purple_plugin_pref_frame_new();
struct PurplePrivacyUiOps*purple_privacy_get_ui_ops();
void purple_privacy_set_ui_ops(struct PurplePrivacyUiOps*);
char*purple_notify_user_info_get_text_with_newline(struct _PurpleNotifyUserInfo*,const char*);
void purple_roomlist_unref(struct _PurpleRoomlist*);
int purple_privacy_permit_remove(struct _PurpleAccount*,const char*,int);
int purple_privacy_permit_add(struct _PurpleAccount*,const char*,int);
void*purple_connections_get_handle();
void purple_conv_im_stop_typing_timeout(struct _PurpleConvIm*);
void purple_connections_init();
struct PurpleConnectionUiOps*purple_connections_get_ui_ops();
void purple_connections_set_ui_ops(struct PurpleConnectionUiOps*);
void purple_prefs_add_string(const char*,const char*);
struct _GList*purple_connections_get_connecting();
struct _GList*purple_connections_get_all();
void purple_connections_disconnect_all();
struct _PurpleWhiteboard*purple_whiteboard_create(struct _PurpleAccount*,const char*,int);
int purple_connection_error_is_fatal(enum PurpleConnectionError);
unsigned char*purple_base64_decode(const char*,unsigned long*);
struct _GHashTable*purple_chat_get_components(struct _PurpleChat*);
void purple_conversation_foreach(void(*)(struct _PurpleConversation*));
const char*purple_theme_get_author(struct _PurpleTheme*);
struct _PurpleAccount*purple_accounts_find(const char*,const char*);
void purple_presence_switch_status(struct _PurplePresence*,const char*);
void purple_connection_update_progress(struct _PurpleConnection*,const char*,unsigned long,unsigned long);
void*purple_connection_get_protocol_data(const struct _PurpleConnection*);
const char*purple_connection_get_display_name(const struct _PurpleConnection*);
const char*purple_connection_get_password(const struct _PurpleConnection*);
void purple_prpl_change_account_status(struct _PurpleAccount*,struct _PurpleStatus*,struct _PurpleStatus*);
struct _PurplePlugin*purple_connection_get_prpl(const struct _PurpleConnection*);
const char*purple_markup_unescape_entity(const char*,int*);
struct _PurpleAccount*purple_connection_get_account(const struct _PurpleConnection*);
enum PurpleConnectionState purple_connection_get_state(const struct _PurpleConnection*);
struct _PurpleRoomlist*purple_roomlist_new(struct _PurpleAccount*);
void purple_connection_set_account(struct _PurpleConnection*,struct _PurpleAccount*);
void purple_xfer_ref(struct _PurpleXfer*);
struct _GList*purple_savedstatuses_get_all();
void purple_connection_destroy(struct _PurpleConnection*);
const char*purple_xfer_get_local_filename(const struct _PurpleXfer*);
void purple_value_set_int64(struct PurpleValue*,signed long);
void purple_connection_new(struct _PurpleAccount*,int,const char*);
int purple_request_field_list_is_selected(const struct _PurpleRequestField*,const char*);
void purple_ssl_init();
struct PurpleSslOps*purple_ssl_get_ops();
void purple_ssl_set_ops(struct PurpleSslOps*);
struct _GList*purple_ssl_get_peer_certificates(struct _PurpleSslConnection*);
unsigned long purple_ssl_read(struct _PurpleSslConnection*,void*,unsigned long);
void purple_blist_uninit();
void purple_ssl_close(struct _PurpleSslConnection*);
void purple_account_set_status(struct _PurpleAccount*,const char*,int,...);
struct _PurpleSslConnection*purple_ssl_connect_with_host_fd(struct _PurpleAccount*,int,void(*)(void*,struct _PurpleSslConnection*,enum PurpleInputCondition),void(*)(struct _PurpleSslConnection*,enum PurpleSslErrorType,void*),const char*,void*);
void purple_buddy_set_icon(struct _PurpleBuddy*,struct _PurpleBuddyIcon*);
struct _PurpleSslConnection*purple_ssl_connect_with_ssl_cn(struct _PurpleAccount*,const char*,int,void(*)(void*,struct _PurpleSslConnection*,enum PurpleInputCondition),void(*)(struct _PurpleSslConnection*,enum PurpleSslErrorType,void*),const char*,void*);
struct _PurpleSslConnection*purple_ssl_connect(struct _PurpleAccount*,const char*,int,void(*)(void*,struct _PurpleSslConnection*,enum PurpleInputCondition),void(*)(struct _PurpleSslConnection*,enum PurpleSslErrorType,void*),void*);
int purple_ssl_is_supported();
void purple_account_option_set_masked(struct PurpleAccountOption*,int);
int purple_certificate_unregister_pool(struct _PurpleCertificatePool*);
char*purple_unescape_text(const char*);
void purple_xfer_conversation_write(struct _PurpleXfer*,char*,int);
void purple_conv_chat_left(struct _PurpleConvChat*);
struct _GList*purple_certificate_get_verifiers();
struct _PurpleCertificateVerifier*purple_certificate_find_verifier(const char*,const char*);
struct _GSList*purple_find_buddies(struct _PurpleAccount*,const char*);
int purple_certificate_unregister_scheme(struct _PurpleCertificateScheme*);
unsigned int purple_conv_im_get_typing_timeout(const struct _PurpleConvIm*);
struct _PurpleUtilFetchUrlData*purple_util_fetch_url_request_len(const char*,int,const char*,int,const char*,int,signed long,void(*)(struct _PurpleUtilFetchUrlData*,void*,const char*,unsigned long,const char*),void*);
int purple_certificate_register_scheme(struct _PurpleCertificateScheme*);
void purple_restore_default_signal_handlers();
void*purple_certificate_get_handle();
void purple_certificate_init();
void purple_certificate_pool_destroy_idlist(struct _GList*);
void purple_status_type_destroy(struct _PurpleStatusType*);
struct _PurpleSavedStatus*purple_savedstatus_find(const char*);
struct _PurpleCertificate*purple_certificate_pool_retrieve(struct _PurpleCertificatePool*,const char*);
int purple_certificate_pool_contains(struct _PurpleCertificatePool*,const char*);
struct _PurpleCertificateScheme*purple_certificate_pool_get_scheme(struct _PurpleCertificatePool*);
int purple_certificate_pool_usable(struct _PurpleCertificatePool*);
void purple_prefs_add_bool(const char*,int);
void*purple_log_get_handle();
int purple_status_compare(const struct _PurpleStatus*,const struct _PurpleStatus*);
int purple_certificate_check_subject_name(struct _PurpleCertificate*,const char*);
char*purple_certificate_get_subject_name(struct _PurpleCertificate*);
char*purple_certificate_get_issuer_unique_id(struct _PurpleCertificate*);
char*purple_certificate_get_unique_id(struct _PurpleCertificate*);
struct _GByteArray*purple_certificate_get_fingerprint_sha1(struct _PurpleCertificate*);
void purple_print_utf8_to_console(struct _IO_FILE*,char*);
int purple_certificate_export(const char*,struct _PurpleCertificate*);
struct _GSList*purple_certificates_import(struct _PurpleCertificateScheme*,const char*);
void purple_plugins_init();
int purple_certificate_check_signature_chain_with_failing(struct _GList*,struct _PurpleCertificate**);
int purple_certificate_signed_by(struct _PurpleCertificate*,struct _PurpleCertificate*);
int purple_status_is_online(const struct _PurpleStatus*);
const char*purple_buddy_get_alias(struct _PurpleBuddy*);
enum PurpleConvChatBuddyFlags purple_conv_chat_user_get_flags(struct _PurpleConvChat*,const char*);
struct _GList*purple_media_candidate_list_copy(struct _GList*);
struct _GList*purple_certificate_copy_list(struct _GList*);
int purple_pounce_action_is_enabled(const struct _PurplePounce*,const char*);
void purple_media_set_input_volume(struct _PurpleMedia*,const char*,double);
void purple_blist_alias_contact(struct _PurpleContact*,const char*);
void purple_status_uninit();
void purple_status_init();
void purple_marshal_BOOLEAN__POINTER_POINTER_POINTER_POINTER_POINTER_POINTER(void(*)(),__builtin_va_list,void*,void**);
void purple_value_set_ushort(struct PurpleValue*,unsigned short);
void*purple_status_get_handle();
const char*purple_conv_chat_get_nick(struct _PurpleConvChat*);
long purple_presence_get_idle_time(const struct _PurplePresence*);
void purple_blist_node_set_bool(struct _PurpleBlistNode*,const char*,int);
struct _PurpleRoomlistRoom*purple_roomlist_room_new(enum PurpleRoomlistRoomType,const char*,struct _PurpleRoomlistRoom*);
void purple_blist_request_add_group();
struct _PurpleNetworkListenData*purple_network_listen_range_family(unsigned short,unsigned short,int,int,void(*)(int,void*),void*);
struct _PurpleNetworkListenData*purple_network_listen_range(unsigned short,unsigned short,int,void(*)(int,void*),void*);
int purple_presence_is_status_active(const struct _PurplePresence*,const char*);
int purple_presence_is_online(const struct _PurplePresence*);
struct _PurplePlugin*purple_plugin_new(int,const char*);
int purple_presence_is_available(const struct _PurplePresence*);
struct _PurpleStatus*purple_presence_get_status(const struct _PurplePresence*,const char*);
struct _GList*purple_presence_get_statuses(const struct _PurplePresence*);
struct _PurpleBuddy*purple_presence_get_buddy(const struct _PurplePresence*);
const char*purple_presence_get_chat_user(const struct _PurplePresence*);
struct _PurpleConversation*purple_presence_get_conversation(const struct _PurplePresence*);
enum PurplePresenceContext purple_presence_get_context(const struct _PurplePresence*);
void purple_presence_set_login_time(struct _PurplePresence*,long);
void purple_presence_set_status_active(struct _PurplePresence*,const char*,int);
void purple_imgstore_init();
void purple_xfer_start(struct _PurpleXfer*,int,const char*,unsigned int);
void purple_plugin_disable(struct _PurplePlugin*);
struct _PurplePresence*purple_presence_new_for_buddy(struct _PurpleBuddy*);
int purple_presence_compare(const struct _PurplePresence*,const struct _PurplePresence*);
struct _PurpleBlistNode*purple_blist_get_root();
struct _PurplePresence*purple_presence_new(enum PurplePresenceContext);
struct _GList*purple_media_get_session_ids(struct _PurpleMedia*);
void purple_media_remove_output_windows(struct _PurpleMedia*);
const char*purple_status_get_attr_string(const struct _PurpleStatus*,const char*);
int purple_status_get_attr_int(const struct _PurpleStatus*,const char*);
void purple_xfer_set_message(struct _PurpleXfer*,const char*);
unsigned long purple_value_get_uint64(const struct PurpleValue*);
struct PurpleValue*purple_status_get_attr_value(const struct _PurpleStatus*,const char*);
void purple_prpl_got_account_login_time(struct _PurpleAccount*,long);
int purple_status_is_available(const struct _PurpleStatus*);
int purple_status_is_exclusive(const struct _PurpleStatus*);
unsigned char*purple_ntlm_parse_type2(const char*,unsigned int*);
struct _PurplePresence*purple_status_get_presence(const struct _PurpleStatus*);
void purple_upnp_discover(void(*)(int,void*),void*);
void purple_status_set_attr_string(struct _PurpleStatus*,const char*,const char*);
void purple_status_set_attr_int(struct _PurpleStatus*,const char*,int);
struct _PurpleStoredImage*purple_smiley_get_stored_image(const struct _PurpleSmiley*);
void purple_cipher_context_reset(struct _PurpleCipherContext*,void*);
struct _PurpleStatus*purple_status_new(struct _PurpleStatusType*,struct _PurplePresence*);
void purple_blist_update_node_icon(struct _PurpleBlistNode*);
const char*purple_status_attr_get_name(const struct _PurpleStatusAttr*);
const char*purple_status_attr_get_id(const struct _PurpleStatusAttr*);
void purple_conv_im_set_typing_state(struct _PurpleConvIm*,enum PurpleTypingState);
void purple_marshal_BOOLEAN__POINTER_POINTER_POINTER_UINT(void(*)(),__builtin_va_list,void*,void**);
struct _PurpleStatusAttr*purple_status_attr_new(const char*,const char*,struct PurpleValue*);
const struct _PurpleStatusType*purple_status_type_find_with_id(struct _GList*,const char*);
struct _GList*purple_status_type_get_attrs(const struct _PurpleStatusType*);
void purple_whiteboard_clear(struct _PurpleWhiteboard*);
void purple_notify_user_info_add_pair_plaintext(struct _PurpleNotifyUserInfo*,const char*,const char*);
int purple_timeout_remove(unsigned int);
void purple_theme_manager_remove_theme(struct _PurpleTheme*);
const char*purple_status_type_get_primary_attr(const struct _PurpleStatusType*);
struct _PurpleConversation*purple_conv_im_get_conversation(const struct _PurpleConvIm*);
int purple_status_type_is_exclusive(const struct _PurpleStatusType*);
int purple_status_type_is_independent(const struct _PurpleStatusType*);
int purple_status_type_is_saveable(const struct _PurpleStatusType*);
const char*purple_status_type_get_name(const struct _PurpleStatusType*);
int purple_util_write_data_to_file(const char*,const char*,signed long);
void purple_status_type_add_attrs(struct _PurpleStatusType*,const char*,const char*,struct PurpleValue*,...);
void purple_status_type_set_primary_attr(struct _PurpleStatusType*,const char*);
struct _GList*purple_certificate_pool_get_idlist(struct _PurpleCertificatePool*);
struct _PurpleStatusType*purple_status_type_new_with_attrs(enum PurpleStatusPrimitive,const char*,const char*,int,int,int,const char*,const char*,struct PurpleValue*,...);
struct _PurpleStatusType*purple_status_type_new(enum PurpleStatusPrimitive,const char*,const char*,int);
enum PurpleStatusPrimitive purple_primitive_get_type_from_id(const char*);
void purple_xfer_set_thumbnail(struct _PurpleXfer*,const void*,unsigned long,const char*);
const char*purple_url_encode(const char*);
void purple_notify_init();
int purple_ipv6_address_is_valid(const char*);
struct PurpleProxyInfo*purple_proxy_info_new();
unsigned long purple_cipher_context_get_block_size(struct _PurpleCipherContext*);
void purple_blist_set_ui_ops(struct _PurpleBlistUiOps*);
struct _GList*purple_blist_node_get_extended_menu(struct _PurpleBlistNode*);
void purple_notify_close(enum PurpleNotifyType,void*);
void purple_notify_user_info_prepend_section_break(struct _PurpleNotifyUserInfo*);
enum PurpleBlistNodeType purple_blist_node_get_type(struct _PurpleBlistNode*);
enum PurpleBlistNodeFlags purple_blist_node_get_flags(struct _PurpleBlistNode*);
struct _PurpleKeyValuePair*purple_media_codec_get_optional_parameter(struct _PurpleMediaCodec*,const char*,const char*);
const char*purple_blist_node_get_string(struct _PurpleBlistNode*,const char*);
const char*purple_get_host_name();
void purple_blist_node_set_string(struct _PurpleBlistNode*,const char*,const char*);
int purple_blist_node_get_int(struct _PurpleBlistNode*,const char*);
void purple_blist_node_set_int(struct _PurpleBlistNode*,const char*,int);
int purple_blist_node_get_bool(struct _PurpleBlistNode*,const char*);
void purple_blist_request_add_chat(struct _PurpleAccount*,struct _PurpleGroup*,const char*,const char*);
void purple_blist_request_add_buddy(struct _PurpleAccount*,const char*,const char*,const char*);
void purple_blist_schedule_save();
struct _PurpleGroup*purple_chat_get_group(struct _PurpleChat*);
void purple_blist_add_account(struct _PurpleAccount*);
unsigned char*purple_quotedp_decode(const char*,unsigned long*);
struct _GSList*purple_group_get_accounts(struct _PurpleGroup*);
struct _PurpleGroup*purple_buddy_get_group(struct _PurpleBuddy*);
int purple_certificate_register_pool(struct _PurpleCertificatePool*);
int purple_blist_get_group_size(struct _PurpleGroup*,int);
void*purple_notify_formatted(void*,const char*,const char*,const char*,const char*,void(*)(void*),void*);
struct _GList*purple_roomlist_get_fields(struct _PurpleRoomlist*);
struct _PurpleGroup*purple_find_group(const char*);
unsigned int purple_value_get_subtype(const struct PurpleValue*);
void purple_prefs_uninit();
struct _GList*purple_mime_part_get_fields(struct _PurpleMimePart*);
const char*purple_chat_get_name(struct _PurpleChat*);
int purple_plugin_unload(struct _PurplePlugin*);
const char*purple_buddy_get_local_alias(struct _PurpleBuddy*);
struct _PurpleConversationUiOps*purple_conversation_get_ui_ops(const struct _PurpleConversation*);
void*purple_signal_emit_return_1(void*,const char*,...);
void purple_blist_remove_group(struct _PurpleGroup*);
struct _PurpleConversation*purple_find_conversation_with_account(enum PurpleConversationType,const char*,const struct _PurpleAccount*);
struct _PurplePluginAction*purple_plugin_action_new(const char*,void(*)(struct _PurplePluginAction*));
void purple_blist_remove_contact(struct _PurpleContact*);
struct _UPnPMappingAddRemove*purple_upnp_set_port_mapping(unsigned short,const char*,void(*)(int,void*),void*);
int purple_contact_on_account(struct _PurpleContact*,struct _PurpleAccount*);
enum PurpleXferType purple_xfer_get_type(const struct _PurpleXfer*);
const char*purple_contact_get_alias(struct _PurpleContact*);
void purple_contact_set_alias(struct _PurpleContact*,const char*);
struct _PurpleBuddy*purple_contact_get_priority_buddy(struct _PurpleContact*);
void purple_blist_add_contact(struct _PurpleContact*,struct _PurpleGroup*,struct _PurpleBlistNode*);
void purple_prefs_add_none(const char*);
void purple_contact_destroy(struct _PurpleContact*);
void purple_blist_add_group(struct _PurpleGroup*,struct _PurpleBlistNode*);
struct _PurpleGroup*purple_group_new(const char*);
void purple_blist_add_buddy(struct _PurpleBuddy*,struct _PurpleContact*,struct _PurpleGroup*,struct _PurpleBlistNode*);
const char*purple_roomlist_room_get_name(struct _PurpleRoomlistRoom*);
void purple_conversation_clear_message_history(struct _PurpleConversation*);
enum PurpleMediaCaps purple_buddy_get_media_caps(const struct _PurpleBuddy*);
struct _PurpleContact*purple_buddy_get_contact(struct _PurpleBuddy*);
struct _GList*purple_request_fields_get_groups(const struct PurpleRequestFields*);
void*purple_buddy_get_protocol_data(const struct _PurpleBuddy*);
struct _PurpleBuddyIcon*purple_buddy_get_icon(const struct _PurpleBuddy*);
void purple_blist_node_set_ui_data(struct _PurpleBlistNode*,void*);
void purple_blist_add_chat(struct _PurpleChat*,struct _PurpleGroup*,struct _PurpleBlistNode*);
void purple_connection_set_state(struct _PurpleConnection*,enum PurpleConnectionState);
struct _PurpleChat*purple_chat_new(struct _PurpleAccount*,const char*,struct _GHashTable*);
void purple_request_field_set_required(struct _PurpleRequestField*,int);
void purple_conv_im_write(struct _PurpleConvIm*,const char*,const char*,enum PurpleMessageFlags,long);
void purple_blist_alias_buddy(struct _PurpleBuddy*,const char*);
void purple_sound_uninit();
struct PurpleValue*purple_status_attr_get_value(const struct _PurpleStatusAttr*);
void purple_blist_update_buddy_status(struct _PurpleBuddy*,struct _PurpleStatus*);
int purple_pounces_load();
void purple_blist_destroy();
struct _PurpleAccount*purple_conversation_get_account(const struct _PurpleConversation*);
struct _PurpleAccount*purple_buddy_get_account(const struct _PurpleBuddy*);
void*purple_blist_node_get_ui_data(const struct _PurpleBlistNode*);
struct _PurpleSmiley*purple_smileys_find_by_checksum(const char*);
struct _PurpleCircBuffer*purple_circ_buffer_new(unsigned long);
struct _PurpleBlistNode*purple_blist_node_get_parent(struct _PurpleBlistNode*);
struct _PurpleBlistNode*purple_blist_node_next(struct _PurpleBlistNode*,int);
struct _GSList*purple_blist_get_buddies();
struct _PurpleBuddyList*purple_get_blist();
const char*purple_conversation_get_name(const struct _PurpleConversation*);
struct _PurpleNetworkListenData*purple_network_listen_family(unsigned short,int,int,void(*)(int,void*),void*);
void purple_buddy_icon_get_scale_size(struct _PurpleBuddyIconSpec*,int*,int*);
struct _PurpleStatusType*purple_account_get_status_type_with_primitive(const struct _PurpleAccount*,enum PurpleStatusPrimitive);
const char*purple_plugin_pref_get_label(struct _PurplePluginPref*);
const char*purple_buddy_icons_get_cache_dir();
void purple_buddy_icons_set_cache_dir(const char*);
int purple_buddy_icons_is_caching();
void purple_buddy_icons_set_caching(int);
struct _PurpleStoredImage*purple_buddy_icons_set_custom_icon(struct _PurpleContact*,unsigned char*,unsigned long);
struct _PurpleStoredImage*purple_buddy_icons_find_custom_icon(struct _PurpleContact*);
struct _GList*purple_plugin_pref_get_choices(struct _PurplePluginPref*);
struct _PurpleStoredImage*purple_buddy_icons_node_set_custom_icon(struct _PurpleBlistNode*,unsigned char*,unsigned long);
struct _PurpleStoredImage*purple_buddy_icons_set_account_icon(struct _PurpleAccount*,unsigned char*,unsigned long);
const char*purple_account_get_protocol_name(const struct _PurpleAccount*);
void purple_plugins_save_loaded(const char*);
const char*purple_buddy_icons_get_checksum_for_user(struct _PurpleBuddy*);
unsigned int purple_xfer_get_local_port(const struct _PurpleXfer*);
struct _PurpleWhiteboard*purple_whiteboard_get_session(const struct _PurpleAccount*,const char*);
enum PurpleRoomlistRoomType purple_roomlist_room_get_type(struct _PurpleRoomlistRoom*);
struct _GList*purple_media_get_local_candidates(struct _PurpleMedia*,const char*,const char*);
const char*purple_buddy_icon_get_extension(const struct _PurpleBuddyIcon*);
const void*purple_buddy_icon_get_data(const struct _PurpleBuddyIcon*,unsigned long*);
const char*purple_buddy_icon_get_checksum(const struct _PurpleBuddyIcon*);
const char*purple_buddy_icon_get_username(const struct _PurpleBuddyIcon*);
struct _PurpleAccount*purple_buddy_icon_get_account(const struct _PurpleBuddyIcon*);
void purple_buddy_icon_set_data(struct _PurpleBuddyIcon*,unsigned char*,unsigned long,const char*);
struct _GList*purple_prefs_get_string_list(const char*);
void purple_request_field_set_label(struct _PurpleRequestField*,const char*);
int purple_prpl_initiate_media(struct _PurpleAccount*,const char*,enum PurpleMediaSessionType);
unsigned long purple_media_candidate_get_type();
void purple_prpl_got_attention_in_chat(struct _PurpleConnection*,int,const char*,unsigned int);
void purple_plugins_unregister_unload_notify_cb(void(*)(struct _PurplePlugin*,void*));
struct _GList*purple_prpl_get_statuses(struct _PurpleAccount*,struct _PurplePresence*);
void purple_prpl_got_user_status_deactive(struct _PurpleAccount*,const char*,const char*);
void purple_prpl_got_user_idle(struct _PurpleAccount*,const char*,int,long);
unsigned long purple_signal_connect_priority_vargs(void*,const char*,void*,void(*)(),void*,int);
void purple_prpl_got_account_status(struct _PurpleAccount*,const char*,...);
int purple_status_is_active(const struct _PurpleStatus*);
void purple_conv_chat_cb_destroy(struct _PurpleConvChatBuddy*);
void purple_prpl_got_account_idle(struct _PurpleAccount*,int,long);
const char*purple_attention_type_get_unlocalized_name(const struct _PurpleAttentionType*);
const char*purple_attention_type_get_icon_name(const struct _PurpleAttentionType*);
void purple_accounts_set_ui_ops(struct _PurpleAccountUiOps*);
void purple_notify_user_info_destroy(struct _PurpleNotifyUserInfo*);
const char*purple_attention_type_get_incoming_desc(const struct _PurpleAttentionType*);
unsigned long purple_media_manager_set_output_window(struct _PurpleMediaManager*,struct _PurpleMedia*,const char*,const char*,unsigned long);
void purple_attention_type_set_icon_name(struct _PurpleAttentionType*,const char*);
void purple_log_init();
void purple_plugin_pref_set_name(struct _PurplePluginPref*,const char*);
void purple_attention_type_set_outgoing_desc(struct _PurpleAttentionType*,const char*);
void purple_attention_type_set_incoming_desc(struct _PurpleAttentionType*,const char*);
void purple_attention_type_set_name(struct _PurpleAttentionType*,const char*);
long purple_xfer_get_end_time(const struct _PurpleXfer*);
void purple_whiteboard_set_brush(struct _PurpleWhiteboard*,int,int);
int purple_plugin_ipc_register(struct _PurplePlugin*,const char*,void(*)(),void(*)(void(*)(),__builtin_va_list,void*,void**),struct PurpleValue*,int,...);
void purple_log_write(struct _PurpleLog*,enum PurpleMessageFlags,const char*,long,const char*);
void purple_whiteboard_destroy(struct _PurpleWhiteboard*);
void purple_imgstore_ref_by_id(int);
struct _PurpleMediaCandidate*purple_media_candidate_copy(struct _PurpleMediaCandidate*);
int purple_plugin_is_unloadable(const struct _PurplePlugin*);
void purple_whiteboard_send_draw_list(struct _PurpleWhiteboard*,struct _GList*);
void purple_whiteboard_set_dimensions(struct _PurpleWhiteboard*,int,int);
int purple_whiteboard_get_dimensions(const struct _PurpleWhiteboard*,int*,int*);
char*purple_smiley_get_full_path(struct _PurpleSmiley*);
const char*purple_xfer_get_remote_ip(const struct _PurpleXfer*);
void purple_roomlist_room_add(struct _PurpleRoomlist*,struct _PurpleRoomlistRoom*);
void purple_whiteboard_send_brush(struct _PurpleWhiteboard*,int,int);
void purple_whiteboard_set_ui_ops(struct _PurpleWhiteboardUiOps*);
struct _PurpleRoomlistUiOps*purple_roomlist_get_ui_ops();
void purple_theme_manager_refresh();
const char*purple_roomlist_field_get_label(struct _PurpleRoomlistField*);
enum PurpleRoomlistFieldType purple_roomlist_field_get_type(struct _PurpleRoomlistField*);
struct _GList*purple_roomlist_room_get_fields(struct _PurpleRoomlistRoom*);
struct _PurpleRoomlistRoom*purple_roomlist_room_get_parent(struct _PurpleRoomlistRoom*);
const char*purple_get_tzoff_str(const struct tm*,int);
void purple_proxy_init();
void purple_roomlist_room_add_field(struct _PurpleRoomlist*,struct _PurpleRoomlistRoom*,const void*);
struct _PurpleChat*purple_blist_find_chat(struct _PurpleAccount*,const char*);
struct PurpleAccountOption*purple_account_option_string_new(const char*,const char*,const char*);
unsigned long purple_request_field_image_get_size(struct _PurpleRequestField*);
void purple_connection_set_protocol_data(struct _PurpleConnection*,void*);
void purple_roomlist_show_with_account(struct _PurpleAccount*);
void purple_proxy_connect_cancel_with_handle(void*);
void purple_proxy_connect_cancel(struct _PurpleProxyConnectData*);
struct _PurpleProxyConnectData*purple_proxy_connect_socks5(void*,struct PurpleProxyInfo*,const char*,int,void(*)(void*,int,const char*),void*);
struct _PurpleProxyConnectData*purple_proxy_connect_socks5_account(void*,struct _PurpleAccount*,struct PurpleProxyInfo*,const char*,int,void(*)(void*,int,const char*),void*);
struct _PurpleProxyConnectData*purple_proxy_connect(void*,struct _PurpleAccount*,const char*,int,void(*)(void*,int,const char*),void*);
struct PurpleProxyInfo*purple_proxy_get_setup(struct _PurpleAccount*);
void purple_proxy_uninit();
void*purple_proxy_get_handle();
void purple_global_proxy_set_info(struct PurpleProxyInfo*);
void purple_util_uninit();
const char*purple_proxy_info_get_password(const struct PurpleProxyInfo*);
const char*purple_proxy_info_get_username(const struct PurpleProxyInfo*);
int purple_proxy_info_get_port(const struct PurpleProxyInfo*);
const char*purple_proxy_info_get_host(const struct PurpleProxyInfo*);
enum PurpleProxyType purple_proxy_info_get_type(const struct PurpleProxyInfo*);
const char*purple_account_option_get_setting(const struct PurpleAccountOption*);
struct PurpleNotifyUiOps*purple_notify_get_ui_ops();
void purple_conversation_set_features(struct _PurpleConversation*,enum PurpleConnectionFlags);
void purple_proxy_info_set_host(struct PurpleProxyInfo*,const char*);
struct _PurpleEventLoopUiOps*purple_eventloop_get_ui_ops();
void purple_xfer_set_cancel_send_fnc(struct _PurpleXfer*,void(*)(struct _PurpleXfer*));
double purple_xfer_get_progress(const struct _PurpleXfer*);
void purple_roomlist_room_join(struct _PurpleRoomlist*,struct _PurpleRoomlistRoom*);
unsigned int purple_input_add(int,enum PurpleInputCondition,void(*)(void*,int,enum PurpleInputCondition),void*);
unsigned int purple_timeout_add(unsigned int,int(*)(void*),void*);
const char*purple_account_get_protocol_id(const struct _PurpleAccount*);
void purple_marshal_VOID(void(*)(),__builtin_va_list,void*,void**);
void purple_notify_user_info_prepend_section_header(struct _PurpleNotifyUserInfo*,const char*);
void purple_media_set_output_volume(struct _PurpleMedia*,const char*,const char*,double);
void purple_certificate_verify(struct _PurpleCertificateVerifier*,const char*,struct _GList*,void(*)(enum PurpleCertificateVerificationStatus,void*),void*);
int purple_media_accepted(struct _PurpleMedia*,const char*,const char*);
int purple_media_codecs_ready(struct _PurpleMedia*,const char*);
void purple_serv_got_join_chat_failed(struct _PurpleConnection*,struct _GHashTable*);
struct _GList*purple_media_get_active_remote_candidates(struct _PurpleMedia*,const char*,const char*);
struct _GList*purple_media_get_active_local_candidates(struct _PurpleMedia*,const char*,const char*);
void purple_media_add_remote_candidates(struct _PurpleMedia*,const char*,const char*,struct _GList*);
struct _GList*purple_media_get_codecs(struct _PurpleMedia*,const char*);
enum PurpleMediaSessionType purple_media_get_session_type(struct _PurpleMedia*,const char*);
int purple_media_add_stream(struct _PurpleMedia*,const char*,const char*,enum PurpleMediaSessionType,int,const char*,unsigned int,struct _GParameter*);
const char**purple_media_get_available_params(struct _PurpleMedia*);
void purple_media_set_params(struct _PurpleMedia*,unsigned int,struct _GParameter*);
struct _PurpleTheme*purple_theme_loader_build(struct _PurpleThemeLoader*,const char*);
int purple_srv_txt_query_get_type(struct _PurpleSrvTxtQueryData*);
void purple_media_set_prpl_data(struct _PurpleMedia*,void*);
void*purple_media_get_prpl_data(struct _PurpleMedia*);
struct _PurpleAccount*purple_media_get_account(struct _PurpleMedia*);
void purple_conv_im_set_icon(struct _PurpleConvIm*,struct _PurpleBuddyIcon*);
void purple_savedstatus_set_title(struct _PurpleSavedStatus*,const char*);
struct _GList*purple_media_codec_list_copy(struct _GList*);
unsigned long purple_signal_connect_vargs(void*,const char*,void*,void(*)(),void*);
void purple_blist_node_set_flags(struct _PurpleBlistNode*,enum PurpleBlistNodeFlags);
void purple_media_codec_remove_optional_parameter(struct _PurpleMediaCodec*,struct _PurpleKeyValuePair*);
int purple_socket_get_family(int);
struct _GList*purple_media_codec_get_optional_parameters(struct _PurpleMediaCodec*);
unsigned int purple_media_codec_get_channels(struct _PurpleMediaCodec*);
int purple_log_common_deleter(struct _PurpleLog*);
char*purple_media_codec_get_encoding_name(struct _PurpleMediaCodec*);
unsigned int purple_media_codec_get_id(struct _PurpleMediaCodec*);
struct _PurpleCipherContext*purple_cipher_context_new(struct _PurpleCipher*,void*);
char*purple_text_strip_mnemonic(const char*);
char*purple_uuid_random();
struct _PurpleCertificateScheme*purple_certificate_find_scheme(const char*);
void purple_request_field_bool_set_default_value(struct _PurpleRequestField*,int);
const char*purple_unescape_filename(const char*);
int purple_message_meify(char*,signed long);
int purple_utf8_strcasecmp(const char*,const char*);
const char*purple_gai_strerror(int);
char*purple_utf8_salvage(const char*);
char*purple_utf8_try_convert(const char*);
const char*purple_buddy_get_contact_alias(struct _PurpleBuddy*);
struct _GList*purple_uri_list_extract_uris(const char*);
void*purple_blist_get_handle();
int purple_ipv4_address_is_valid(const char*);
int purple_email_is_valid(const char*);
const char*purple_primitive_get_name_from_type(enum PurpleStatusPrimitive);
const char*purple_url_decode(const char*);
void purple_conv_im_start_send_typed_timeout(struct _PurpleConvIm*);
void purple_util_fetch_url_cancel(struct _PurpleUtilFetchUrlData*);
const char*purple_account_option_get_text(const struct PurpleAccountOption*);
int purple_url_parse(const char*,char**,int*,char**,char**,char**);
int purple_request_field_string_is_editable(const struct _PurpleRequestField*);
void purple_idle_set_ui_ops(struct PurpleIdleUiOps*);
void purple_account_user_split_destroy(struct PurpleAccountUserSplit*);
void purple_plugins_register_probe_notify_cb(void(*)(void*),void*);
char*purple_utf8_ncr_decode(const char*);
void purple_notify_user_info_add_section_break(struct _PurpleNotifyUserInfo*);
char*purple_utf8_ncr_encode(const char*);
void*purple_notify_uri(void*,const char*);
const char*purple_request_fields_get_string(const struct PurpleRequestFields*,const char*);
void purple_str_strip_char(char*,char);
char*purple_str_add_cr(const char*);
void purple_request_field_choice_add(struct _PurpleRequestField*,const char*);
int purple_str_has_prefix(const char*,const char*);
const char*purple_normalize_nocase(const struct _PurpleAccount*,const char*);
const char*purple_normalize(const struct _PurpleAccount*,const char*);
int purple_strequal(const char*,const char*);
void purple_serv_got_private_alias(struct _PurpleConnection*,const char*,const char*);
void purple_media_codec_add_optional_parameter(struct _PurpleMediaCodec*,const char*,const char*);
int purple_running_osx();
const char*purple_value_get_string(const struct PurpleValue*);
void purple_account_set_int(struct _PurpleAccount*,const char*,int);
void purple_markup_html_to_xhtml(const char*,char**,char**);
char*purple_util_get_image_checksum(const void*,unsigned long);
void purple_marshal_BOOLEAN__POINTER_POINTER_POINTER(void(*)(),__builtin_va_list,void*,void**);
struct _xmlnode*purple_util_read_xml_from_file(const char*,const char*);
enum PurpleStatusPrimitive purple_status_type_get_primitive(const struct _PurpleStatusType*);
const char*purple_user_dir();
const char*purple_home_dir();
int purple_markup_is_rtl(const char*);
char*purple_markup_get_css_property(const char*,const char*);
char*purple_markup_get_tag_name(const char*);
const char*purple_theme_get_description(struct _PurpleTheme*);
char*purple_markup_escape_text(const char*,signed long);
void purple_plugin_pref_add_choice(struct _PurplePluginPref*,const char*,void*);
long purple_time_build(int,int,int,int,int,int);
const char*purple_date_format_full(const struct tm*);
const char*purple_date_format_short(const struct tm*);
void purple_eventloop_set_ui_ops(struct _PurpleEventLoopUiOps*);
void purple_value_destroy(struct PurpleValue*);
const char*purple_utf8_strftime(const char*,const struct tm*);
int purple_group_on_account(struct _PurpleGroup*,struct _PurpleAccount*);
void purple_connection_ssl_error(struct _PurpleConnection*,enum PurpleSslErrorType);
unsigned char*purple_base16_decode(const char*,unsigned long*);
void purple_util_init();
void*purple_pounces_get_handle();
void purple_util_set_current_song(const char*,const char*,const char*);
void purple_account_remove_group(struct _PurpleAccount*,struct _PurpleGroup*);
void purple_conv_chat_unignore(struct _PurpleConvChat*,const char*);
void purple_blist_init();
void*purple_notify_get_handle();
void purple_proxy_info_set_port(struct PurpleProxyInfo*,int);
void purple_conv_chat_invite_user(struct _PurpleConvChat*,const char*,const char*,int);
void purple_notify_user_info_entry_set_type(struct _PurpleNotifyUserInfoEntry*,enum PurpleNotifyUserInfoEntryType);
void purple_notify_user_info_entry_set_value(struct _PurpleNotifyUserInfoEntry*,const char*);
const char*purple_notify_user_info_entry_get_value(struct _PurpleNotifyUserInfoEntry*);
void purple_account_set_bool(struct _PurpleAccount*,const char*,int);
const char*purple_notify_user_info_entry_get_label(struct _PurpleNotifyUserInfoEntry*);
struct _PurpleCertificate*purple_certificate_copy(struct _PurpleCertificate*);
struct _PurpleMediaCandidate*purple_media_candidate_new(const char*,unsigned int,enum PurpleMediaCandidateType,enum PurpleMediaNetworkProtocol,const char*,unsigned int);
void purple_conversation_present(struct _PurpleConversation*);
void purple_notify_user_info_add_pair(struct _PurpleNotifyUserInfo*,const char*,const char*);
struct _GList*purple_notify_user_info_get_entries(struct _PurpleNotifyUserInfo*);
struct _PurpleNotifyUserInfo*purple_notify_user_info_new();
void*purple_notify_emails(void*,unsigned long,int,const char**,const char**,const char**,const char**,void(*)(void*),void*);
void*purple_notify_message(void*,enum PurpleNotifyMsgType,const char*,const char*,const char*,void(*)(void*),void*);
struct _GList*purple_conversation_get_extended_menu(struct _PurpleConversation*);
void purple_cipher_context_destroy(struct _PurpleCipherContext*);
void purple_conversation_update(struct _PurpleConversation*,enum PurpleConvUpdateType);
void purple_notify_searchresults_row_add(struct PurpleNotifySearchResults*,struct _GList*);
void purple_notify_searchresults_column_add(struct PurpleNotifySearchResults*,struct PurpleNotifySearchColumn*);
const char*purple_request_field_get_id(const struct _PurpleRequestField*);
void purple_notify_searchresults_button_add(struct PurpleNotifySearchResults*,enum PurpleNotifySearchButtonType,void(*)(struct _PurpleConnection*,struct _GList*,void*));
struct _PurpleStoredImage*purple_imgstore_ref(struct _PurpleStoredImage*);
unsigned long purple_media_candidate_type_get_type();
const char*purple_plugin_get_homepage(const struct _PurplePlugin*);
void*purple_notify_searchresults(struct _PurpleConnection*,const char*,const char*,const char*,struct PurpleNotifySearchResults*,void(*)(void*),void*);
void purple_log_logger_add(struct _PurpleLogLogger*);
struct _PurpleBuddyIcon*purple_buddy_icons_find(struct _PurpleAccount*,const char*);
void purple_media_error(struct _PurpleMedia*,const char*,...);
void purple_proxy_info_destroy(struct PurpleProxyInfo*);
enum PurpleTypingState purple_conv_im_get_typing_state(const struct _PurpleConvIm*);
void purple_marshal_INT__INT_INT(void(*)(),__builtin_va_list,void*,void**);
void purple_marshal_VOID__POINTER(void(*)(),__builtin_va_list,void*,void**);
unsigned long purple_signal_register(void*,const char*,void(*)(void(*)(),__builtin_va_list,void*,void**),struct PurpleValue*,int,...);
void purple_media_codec_list_free(struct _GList*);
struct _GList*purple_plugins_get_loaded();
void purple_value_set_uint(struct PurpleValue*,unsigned int);
char*purple_media_candidate_get_username(struct _PurpleMediaCandidate*);
char*purple_notify_searchresults_column_get_title(struct PurpleNotifySearchResults*,unsigned int);
int purple_certificate_get_times(struct _PurpleCertificate*,long*,long*);
void purple_marshal_INT__POINTER_POINTER_POINTER(void(*)(),__builtin_va_list,void*,void**);
char*purple_buddy_icon_get_full_path(struct _PurpleBuddyIcon*);
struct _PurpleSslConnection*purple_ssl_connect_fd(struct _PurpleAccount*,int,void(*)(void*,struct _PurpleSslConnection*,enum PurpleInputCondition),void(*)(struct _PurpleSslConnection*,enum PurpleSslErrorType,void*),void*);
char purple_value_get_char(const struct PurpleValue*);
int purple_conversation_has_focus(struct _PurpleConversation*);
void purple_request_fields_add_group(struct PurpleRequestFields*,struct PurpleRequestFieldGroup*);
unsigned int purple_media_candidate_get_priority(struct _PurpleMediaCandidate*);
const char*purple_status_type_get_id(const struct _PurpleStatusType*);
char*purple_strcasereplace(const char*,const char*,const char*);
char*purple_base16_encode_chunked(const unsigned char*,unsigned long);
void purple_buddy_icons_init();
void purple_xfer_end(struct _PurpleXfer*);
struct _PurpleGroup*purple_contact_get_group(const struct _PurpleContact*);
void purple_privacy_allow(struct _PurpleAccount*,const char*,int,int);
void purple_media_stream_info(struct _PurpleMedia*,enum PurpleMediaInfoType,const char*,const char*,int);
struct PurpleValue*purple_value_new_outgoing(enum PurpleType,...);
void purple_conv_im_send(struct _PurpleConvIm*,const char*);
int purple_privacy_check(struct _PurpleAccount*,const char*);
struct _PurpleConvChatBuddy*purple_conv_chat_cb_new(const char*,const char*,enum PurpleConvChatBuddyFlags);
void purple_xfer_ui_ready(struct _PurpleXfer*);
void purple_theme_manager_init();
struct _PurpleContact*purple_contact_new();
void purple_request_field_group_add_field(struct PurpleRequestFieldGroup*,struct _PurpleRequestField*);
char*purple_log_get_log_dir(enum PurpleLogType,const char*,struct _PurpleAccount*);
void*purple_conversations_get_handle();
void purple_log_logger_set(struct _PurpleLogLogger*);
unsigned int purple_media_candidate_get_ttl(struct _PurpleMediaCandidate*);
void purple_prefs_rename_boolean_toggle(const char*,const char*);
int purple_log_delete(struct _PurpleLog*);
struct _PurpleAccount*purple_chat_get_account(struct _PurpleChat*);
long purple_conversation_message_get_timestamp(struct _PurpleConvMessage*);
enum PurpleMediaCandidateType purple_media_candidate_get_candidate_type(struct _PurpleMediaCandidate*);
enum PurpleMediaNetworkProtocol purple_media_candidate_get_protocol(struct _PurpleMediaCandidate*);
unsigned short purple_media_candidate_get_base_port(struct _PurpleMediaCandidate*);
char*purple_media_candidate_get_base_ip(struct _PurpleMediaCandidate*);
unsigned short purple_media_candidate_get_port(struct _PurpleMediaCandidate*);
char*purple_media_candidate_get_foundation(struct _PurpleMediaCandidate*);
void purple_certificate_destroy(struct _PurpleCertificate*);
enum PurpleMediaCaps purple_prpl_get_media_caps(struct _PurpleAccount*,const char*);
void purple_connection_error_reason(struct _PurpleConnection*,enum PurpleConnectionError,const char*);
unsigned long purple_media_session_type_get_type();
unsigned long purple_media_network_protocol_get_type();
struct _PurplePresence*purple_account_get_presence(const struct _PurpleAccount*);
char*purple_strdup_withhtml(const char*);
struct PurpleXferUiOps*purple_xfers_get_ui_ops();
void purple_marshal_VOID__POINTER_UINT(void(*)(),__builtin_va_list,void*,void**);
void purple_proxy_info_set_type(struct PurpleProxyInfo*,enum PurpleProxyType);
void purple_xfer_request_denied(struct _PurpleXfer*);
void purple_xfer_cancel_remote(struct _PurpleXfer*);
const char*purple_xfer_get_thumbnail_mimetype(const struct _PurpleXfer*);
const void*purple_xfer_get_thumbnail(const struct _PurpleXfer*,unsigned long*);
void purple_xfer_unref(struct _PurpleXfer*);
struct _PurpleCertificatePool*purple_certificate_find_pool(const char*,const char*);
void purple_xfer_set_cancel_recv_fnc(struct _PurpleXfer*,void(*)(struct _PurpleXfer*));
void purple_xfer_cancel_local(struct _PurpleXfer*);
void purple_presence_add_list(struct _PurplePresence*,struct _GList*);
signed long purple_xfer_write(struct _PurpleXfer*,const unsigned char*,unsigned long);
signed long purple_xfer_read(struct _PurpleXfer*,unsigned char**);
void purple_xfer_update_progress(struct _PurpleXfer*);
void purple_xfer_set_end_fnc(struct _PurpleXfer*,void(*)(struct _PurpleXfer*));
struct _PurpleSmiley*purple_smiley_new(struct _PurpleStoredImage*,const char*);
void purple_xfer_set_init_fnc(struct _PurpleXfer*,void(*)(struct _PurpleXfer*));
void purple_xfer_set_request_denied_fnc(struct _PurpleXfer*,void(*)(struct _PurpleXfer*));
void purple_xfer_set_write_fnc(struct _PurpleXfer*,signed long(*)(const unsigned char*,unsigned long,struct _PurpleXfer*));
struct _PurpleConversation*purple_conv_chat_get_conversation(const struct _PurpleConvChat*);
void purple_xfer_set_size(struct _PurpleXfer*,unsigned long);
struct _GList*purple_account_get_status_types(const struct _PurpleAccount*);
void purple_xfer_set_completed(struct _PurpleXfer*,int);
void purple_value_set_uchar(struct PurpleValue*,unsigned char);
struct _PurpleAttentionType*purple_attention_type_new(const char*,const char*,const char*,const char*);
long purple_xfer_get_start_time(const struct _PurpleXfer*);
unsigned int purple_xfer_get_remote_port(const struct _PurpleXfer*);
void purple_whiteboard_start(struct _PurpleWhiteboard*);
void purple_buddy_icons_set_for_user(struct _PurpleAccount*,const char*,void*,unsigned long,const char*);
void purple_marshal_VOID__POINTER_INT_INT(void(*)(),__builtin_va_list,void*,void**);
unsigned long purple_xfer_get_size(const struct _PurpleXfer*);
void purple_connection_new_unregister(struct _PurpleAccount*,const char*,void(*)(struct _PurpleAccount*,int,void*),void*);
const char*purple_xfer_get_filename(const struct _PurpleXfer*);
int purple_xfer_is_canceled(const struct _PurpleXfer*);
const char*purple_ssl_strerror(enum PurpleSslErrorType);
struct _PurpleAccount*purple_xfer_get_account(const struct _PurpleXfer*);
void*purple_xfers_get_handle();
void purple_xfer_request_accepted(struct _PurpleXfer*,const char*);
struct _GList*purple_xfers_get_all();
struct _PurpleXfer*purple_xfer_new(struct _PurpleAccount*,enum PurpleXferType,const char*);
void purple_conversations_init();
const char*purple_conv_chat_cb_get_name(struct _PurpleConvChatBuddy*);
struct _PurpleConvChatBuddy*purple_conv_chat_cb_find(struct _PurpleConvChat*,const char*);
void purple_notify_set_ui_ops(struct PurpleNotifyUiOps*);
struct _PurplePluginPref*purple_plugin_pref_new_with_name(const char*);
int purple_certificate_register_verifier(struct _PurpleCertificateVerifier*);
struct _PurpleConversation*purple_find_chat(const struct _PurpleConnection*,int);
struct _PurplePresence*purple_presence_new_for_account(struct _PurpleAccount*);
void purple_certificate_destroy_list(struct _GList*);
int purple_conv_chat_find_user(struct _PurpleConvChat*,const char*);
void purple_conv_chat_remove_users(struct _PurpleConvChat*,struct _GList*,const char*);
void purple_conv_chat_remove_user(struct _PurpleConvChat*,const char*,const char*);
void purple_prefs_set_int(const char*,int);
void purple_conv_chat_add_users(struct _PurpleConvChat*,struct _GList*,struct _GList*,struct _GList*,int);
void purple_conv_chat_add_user(struct _PurpleConvChat*,const char*,const char*,enum PurpleConvChatBuddyFlags,int);
void purple_conv_chat_send_with_flags(struct _PurpleConvChat*,const char*,enum PurpleMessageFlags);
void purple_conv_chat_write(struct _PurpleConvChat*,const char*,const char*,enum PurpleMessageFlags,long);
void purple_value_set_boxed(struct PurpleValue*,void*);
int purple_conv_chat_get_id(const struct _PurpleConvChat*);
void purple_conv_chat_set_id(struct _PurpleConvChat*,int);
const char*purple_conv_chat_get_topic(const struct _PurpleConvChat*);
const char*purple_status_get_id(const struct _PurpleStatus*);
struct _GList*purple_conv_chat_get_ignored(const struct _PurpleConvChat*);
struct _GList*purple_conv_chat_set_ignored(struct _PurpleConvChat*,struct _GList*);
void purple_conv_chat_ignore(struct _PurpleConvChat*,const char*);
struct _GList*purple_conv_chat_get_users(const struct _PurpleConvChat*);
void purple_xfer_set_bytes_sent(struct _PurpleXfer*,unsigned long);
void purple_conv_custom_smiley_close(struct _PurpleConversation*,const char*);
void purple_prefs_add_path(const char*,const char*);
void purple_value_set_boolean(struct PurpleValue*,int);
void purple_conv_im_send_with_flags(struct _PurpleConvIm*,const char*,enum PurpleMessageFlags);
struct _PurpleStatusType*purple_account_get_status_type(const struct _PurpleAccount*,const char*);
void purple_blist_server_alias_buddy(struct _PurpleBuddy*,const char*);
void purple_plugin_destroy(struct _PurplePlugin*);
void purple_cipher_context_set_data(struct _PurpleCipherContext*,void*);
unsigned int purple_conv_im_get_send_typed_timeout(const struct _PurpleConvIm*);
void purple_request_field_choice_set_default_value(struct _PurpleRequestField*,int);
long purple_conv_im_get_type_again(const struct _PurpleConvIm*);
void purple_conv_im_start_typing_timeout(struct _PurpleConvIm*,int);
struct _PurplePlugin*purple_plugins_find_with_filename(const char*);
struct _PurpleBuddyIcon*purple_conv_im_get_icon(const struct _PurpleConvIm*);
enum PurpleMessageFlags purple_conversation_message_get_flags(struct _PurpleConvMessage*);
const char*purple_conversation_message_get_message(struct _PurpleConvMessage*);
const char*purple_conversation_message_get_sender(struct _PurpleConvMessage*);
enum PurpleConnectionFlags purple_conversation_get_features(struct _PurpleConversation*);
struct _GList*purple_get_chats();
void*purple_prefs_get_handle();
struct _GList*purple_get_conversations();
void*purple_conversation_get_data(struct _PurpleConversation*,const char*);
void purple_conversation_set_data(struct _PurpleConversation*,const char*,void*);
void purple_conversation_close_logs(struct _PurpleConversation*);
void purple_txt_response_destroy(struct _PurpleTxtResponse*);
void purple_marshal_INT__INT(void(*)(),__builtin_va_list,void*,void**);
void purple_conv_chat_cb_set_attribute(struct _PurpleConvChat*,struct _PurpleConvChatBuddy*,const char*,const char*);
struct _GList*purple_conv_chat_cb_get_attribute_keys(struct _PurpleConvChatBuddy*);
const char*purple_conv_chat_cb_get_attribute(struct _PurpleConvChatBuddy*,const char*);
void purple_set_blist(struct _PurpleBuddyList*);
void purple_conversation_set_name(struct _PurpleConversation*,const char*);
const char*purple_conversation_get_title(const struct _PurpleConversation*);
struct _PurpleConnection*purple_conversation_get_gc(const struct _PurpleConversation*);
void purple_blist_show();
void purple_conversation_set_account(struct _PurpleConversation*,struct _PurpleAccount*);
struct _PurplePlugin*purple_plugin_probe(const char*);
int purple_prefs_exists(const char*);
const char*purple_buddy_get_server_alias(struct _PurpleBuddy*);
void purple_conversations_set_ui_ops(struct _PurpleConversationUiOps*);
void purple_conversation_set_ui_ops(struct _PurpleConversation*,struct _PurpleConversationUiOps*);
enum PurpleConversationType purple_conversation_get_type(const struct _PurpleConversation*);
int purple_imgstore_add_with_id(void*,unsigned long,const char*);
struct _PurplePlugin*purple_plugins_find_with_name(const char*);
int purple_media_candidates_prepared(struct _PurpleMedia*,const char*,const char*);
void*purple_plugin_ipc_call(struct _PurplePlugin*,const char*,int*,...);
int purple_whiteboard_get_brush(const struct _PurpleWhiteboard*,int*,int*);
void purple_log_free(struct _PurpleLog*);
void purple_status_type_add_attr(struct _PurpleStatusType*,const char*,const char*,struct PurpleValue*);
int purple_status_type_is_available(const struct _PurpleStatusType*);
struct _GList*purple_certificate_get_schemes();
char*purple_certificate_pool_mkpath(struct _PurpleCertificatePool*,const char*);
unsigned long purple_media_codec_get_type();
void purple_request_field_list_set_multi_select(struct _PurpleRequestField*,int);
void purple_conv_chat_set_topic(struct _PurpleConvChat*,const char*,const char*);
void purple_log_logger_free(struct _PurpleLogLogger*);
int purple_program_is_valid(const char*);
int purple_log_set_compare(const void*,const void*);
int purple_log_get_size(struct _PurpleLog*);
int purple_request_field_is_visible(const struct _PurpleRequestField*);
struct _GHashTable*purple_log_get_log_sets();
struct _GList*purple_log_get_logs(enum PurpleLogType,const char*,struct _PurpleAccount*);
int purple_util_write_data_to_file_absolute(const char*,const char*,signed long);
void purple_request_field_int_set_default_value(struct _PurpleRequestField*,int);
struct _PurplePlugin*purple_plugins_find_with_basename(const char*);
char*purple_utf8_strip_unprintables(const char*);
void purple_roomlist_expand_category(struct _PurpleRoomlist*,struct _PurpleRoomlistRoom*);
void purple_marshal_BOOLEAN__POINTER_BOOLEAN(void(*)(),__builtin_va_list,void*,void**);
void purple_plugins_add_search_path(const char*);
void purple_account_unregister(struct _PurpleAccount*,void(*)(struct _PurpleAccount*,int,void*),void*);
char*purple_base16_encode(const unsigned char*,unsigned long);
void purple_got_protocol_handler_uri(const char*);
void purple_presence_destroy(struct _PurplePresence*);
struct _PurpleAccount*purple_presence_get_account(const struct _PurplePresence*);
const char*purple_plugin_get_id(const struct _PurplePlugin*);
const char*purple_plugin_get_version(const struct _PurplePlugin*);
struct _GList*purple_cmd_list(struct _PurpleConversation*);
int purple_circ_buffer_mark_read(struct _PurpleCircBuffer*,unsigned long);
void purple_conversation_set_title(struct _PurpleConversation*,const char*);
void purple_notify_user_info_prepend_pair(struct _PurpleNotifyUserInfo*,const char*,const char*);
int purple_plugin_register(struct _PurplePlugin*);
void purple_plugins_destroy_all();
void purple_dnsquery_destroy(struct _PurpleDnsQueryData*);
void*purple_imgstore_get_handle();
void purple_conversation_destroy(struct _PurpleConversation*);
struct _PurpleStoredImage*purple_imgstore_new_from_file(const char*);
struct _PurpleStoredImage*purple_imgstore_add(void*,unsigned long,const char*);
void purple_conv_custom_smiley_write(struct _PurpleConversation*,const char*,const unsigned char*,unsigned long);
void purple_blist_remove_chat(struct _PurpleChat*);
void purple_pounce_set_data(struct _PurplePounce*,void*);
void purple_plugin_pref_set_type(struct _PurplePluginPref*,enum PurplePluginPrefType);
struct _GList*purple_plugins_get_protocols();
struct _PurpleConversation*purple_conversation_new(enum PurpleConversationType,struct _PurpleAccount*,const char*);
int purple_debug_is_verbose();
void purple_plugins_unregister_probe_notify_cb(void(*)(void*));
int purple_plugins_enabled();
void purple_plugins_probe(const char*);
void purple_plugins_load_saved(const char*);
void purple_plugins_unload(enum PurplePluginType);
void purple_value_set_short(struct PurpleValue*,short);
short purple_value_get_short(const struct PurpleValue*);
void purple_connections_uninit();
void purple_plugin_ipc_unregister_all(struct _PurplePlugin*);
int purple_value_get_boolean(const struct PurpleValue*);
const char*purple_plugin_get_author(const struct _PurplePlugin*);
const char*purple_plugin_get_summary(const struct _PurplePlugin*);
const char*purple_buddy_get_local_buddy_alias(struct _PurpleBuddy*);
void purple_signals_init();
enum PurpleStringFormatType purple_plugin_pref_get_format_type(struct _PurplePluginPref*);
int purple_plugin_pref_get_masked(struct _PurplePluginPref*);
unsigned int purple_plugin_pref_get_max_length(struct _PurplePluginPref*);
void purple_plugin_pref_set_max_length(struct _PurplePluginPref*,unsigned int);
int purple_buddy_icons_has_custom_icon(struct _PurpleContact*);
enum PurplePluginPrefType purple_plugin_pref_get_type(struct _PurplePluginPref*);
struct _GList*purple_plugins_get_all();
void purple_plugin_pref_get_bounds(struct _PurplePluginPref*,int*,int*);
void*purple_buddy_icons_get_handle();
void purple_debug(enum PurpleDebugLevel,const char*,const char*,...);
const char*purple_plugin_pref_get_name(struct _PurplePluginPref*);
void purple_plugin_pref_destroy(struct _PurplePluginPref*);
void purple_privacy_init();
void purple_prefs_update_old();
int purple_prefs_load();
void purple_prefs_trigger_callback(const char*);
const char*purple_theme_get_dir(struct _PurpleTheme*);
const char*purple_buddy_get_alias_only(struct _PurpleBuddy*);
unsigned int purple_cipher_get_capabilities(struct _PurpleCipher*);
const char*purple_prefs_get_path(const char*);
struct _PurpleBuddyIcon*purple_buddy_icon_unref(struct _PurpleBuddyIcon*);
const char*purple_prefs_get_string(const char*);
int purple_prefs_get_int(const char*);
int purple_prefs_get_bool(const char*);
enum _PurplePrefType purple_prefs_get_type(const char*);
void purple_prefs_set_path_list(const char*,struct _GList*);
struct _GList*purple_accounts_get_all_active();
void purple_prefs_set_string_list(const char*,struct _GList*);
void purple_prefs_set_string(const char*,const char*);
void purple_conv_chat_rename_user(struct _PurpleConvChat*,const char*,const char*);
void purple_prefs_set_bool(const char*,int);
void purple_prefs_set_generic(const char*,void*);
void purple_prefs_rename(const char*,const char*);
void purple_prefs_add_path_list(const char*,struct _GList*);
void purple_prefs_init();
struct _GList*purple_get_ims();
void purple_marshal_POINTER__POINTER_INT64_BOOLEAN(void(*)(),__builtin_va_list,void*,void**);
void purple_marshal_POINTER__POINTER_INT(void(*)(),__builtin_va_list,void*,void**);
void purple_marshal_BOOLEAN__POINTER_POINTER_POINTER_POINTER(void(*)(),__builtin_va_list,void*,void**);
void purple_marshal_BOOLEAN__POINTER_POINTER_UINT(void(*)(),__builtin_va_list,void*,void**);
void purple_marshal_BOOLEAN__POINTER_POINTER(void(*)(),__builtin_va_list,void*,void**);
void purple_marshal_INT__POINTER_POINTER(void(*)(),__builtin_va_list,void*,void**);
void purple_conv_chat_cb_set_attributes(struct _PurpleConvChat*,struct _PurpleConvChatBuddy*,struct _GList*,struct _GList*);
void purple_marshal_VOID__POINTER_POINTER_POINTER_UINT_UINT(void(*)(),__builtin_va_list,void*,void**);
void purple_xfers_init();
void purple_marshal_VOID__POINTER_POINTER_POINTER_UINT(void(*)(),__builtin_va_list,void*,void**);
void purple_marshal_VOID__POINTER_POINTER_POINTER(void(*)(),__builtin_va_list,void*,void**);
struct _PurpleStringref*purple_stringref_printf(const char*,...);
void purple_xfers_set_ui_ops(struct PurpleXferUiOps*);
void purple_marshal_VOID__INT_INT(void(*)(),__builtin_va_list,void*,void**);
int purple_plugin_load(struct _PurplePlugin*);
struct _GList*purple_prefs_get_children_names(const char*);
void purple_signal_emit_vargs(void*,const char*,__builtin_va_list);
void purple_signal_emit(void*,const char*,...);
struct _GList*purple_request_field_choice_get_labels(const struct _PurpleRequestField*);
unsigned long purple_signal_connect_priority(void*,const char*,void*,void(*)(),void*,int);
void*purple_value_get_boxed(const struct PurpleValue*);
int purple_value_get_enum(const struct PurpleValue*);
void*purple_value_get_object(const struct PurpleValue*);
void purple_log_set_free(struct _PurpleLogSet*);
int purple_status_get_attr_boolean(const struct _PurpleStatus*,const char*);
signed long purple_value_get_int64(const struct PurpleValue*);
unsigned long purple_value_get_ulong(const struct PurpleValue*);
long purple_value_get_long(const struct PurpleValue*);
unsigned int purple_value_get_uint(const struct PurpleValue*);
int purple_value_get_int(const struct PurpleValue*);
void purple_cmd_unregister(unsigned int);
struct _GList*purple_plugins_get_search_paths();
unsigned char purple_value_get_uchar(const struct PurpleValue*);
void purple_account_add_buddy_with_invite(struct _PurpleAccount*,struct _PurpleBuddy*,const char*);
void purple_value_set_object(struct PurpleValue*,void*);
void purple_value_set_long(struct PurpleValue*,long);
void purple_value_set_char(struct PurpleValue*,char);
int purple_value_is_outgoing(const struct PurpleValue*);
const char*purple_account_user_split_get_default_value(const struct PurpleAccountUserSplit*);
struct _GList*purple_media_manager_get_media_by_account(struct _PurpleMediaManager*,struct _PurpleAccount*);
struct PurpleValue*purple_value_new(enum PurpleType,...);
void*purple_request_input(void*,const char*,const char*,const char*,const char*,int,int,char*,const char*,void(*)(),const char*,void(*)(),struct _PurpleAccount*,const char*,struct _PurpleConversation*,void*);
struct _PurpleConvChat*purple_conversation_get_chat_data(const struct _PurpleConversation*);
struct _PurpleNotifyUserInfoEntry*purple_notify_user_info_entry_new(const char*,const char*);
void purple_sound_play_file(const char*,const struct _PurpleAccount*);
unsigned int purple_notify_searchresults_get_rows_count(struct PurpleNotifySearchResults*);
struct _PurpleSavedStatus*purple_savedstatus_new(const char*,enum PurpleStatusPrimitive);
int purple_privacy_deny_remove(struct _PurpleAccount*,const char*,int);
enum PurpleXferStatusType purple_xfer_get_status(const struct _PurpleXfer*);
int purple_account_user_split_get_reverse(const struct PurpleAccountUserSplit*);
void purple_chat_destroy(struct _PurpleChat*);
int purple_plugin_is_loaded(const struct _PurplePlugin*);
const char*purple_buddy_get_name(const struct _PurpleBuddy*);
long purple_buddy_icons_get_account_icon_timestamp(struct _PurpleAccount*);
void purple_request_field_account_set_show_all(struct _PurpleRequestField*,int);
const unsigned char*purple_network_ip_atoi(const char*);
char*purple_base64_encode(const unsigned char*,unsigned long);
void purple_whiteboard_send_clear(struct _PurpleWhiteboard*);
void purple_marshal_POINTER__POINTER_INT_BOOLEAN(void(*)(),__builtin_va_list,void*,void**);
void purple_conv_im_set_type_again(struct _PurpleConvIm*,unsigned int);
struct _GList*purple_uri_list_extract_filenames(const char*);
int purple_certificate_check_signature_chain(struct _GList*);
struct _PurpleRequestField*purple_request_field_account_new(const char*,const char*,struct _PurpleAccount*);
void purple_blist_load();
const char*purple_theme_get_type_string(struct _PurpleTheme*);
void purple_conversations_uninit();
void purple_prpl_got_attention(struct _PurpleConnection*,const char*,unsigned int);
void purple_buddy_set_media_caps(struct _PurpleBuddy*,enum PurpleMediaCaps);
const char*purple_value_get_specific_type(const struct PurpleValue*);
int purple_build_dir(const char*,int);
int purple_request_field_is_required(const struct _PurpleRequestField*);
void purple_accounts_add(struct _PurpleAccount*);
void purple_prpl_got_account_actions(struct _PurpleAccount*);
int purple_conversation_is_logging(const struct _PurpleConversation*);
void purple_marshal_VOID__POINTER_POINTER_POINTER_POINTER(void(*)(),__builtin_va_list,void*,void**);
int purple_pmp_destroy_map(enum PurplePmpType,unsigned short);
struct _GList*purple_cmd_help(struct _PurpleConversation*,const char*);
void purple_xfer_prpl_ready(struct _PurpleXfer*);
int purple_conv_custom_smiley_add(struct _PurpleConversation*,const char*,const char*,const char*,int);
const char*purple_escape_filename(const char*);
void purple_status_set_active_with_attrs_list(struct _PurpleStatus*,int,struct _GList*);
const char*purple_status_get_name(const struct _PurpleStatus*);
struct _PurpleStoredImage*purple_imgstore_find_by_id(int);
void purple_pounce_set_options(struct _PurplePounce*,enum PurplePounceOption);
void purple_notify_searchresults_free(struct PurpleNotifySearchResults*);
void purple_account_register(struct _PurpleAccount*);
const void*purple_imgstore_get_data(struct _PurpleStoredImage*);
struct _PurpleBuddy*purple_find_buddy_in_group(struct _PurpleAccount*,const char*,struct _PurpleGroup*);
void purple_xfer_set_ack_fnc(struct _PurpleXfer*,void(*)(struct _PurpleXfer*,const unsigned char*,unsigned long));
int purple_status_type_is_user_settable(const struct _PurpleStatusType*);
void purple_prefs_remove(const char*);
struct _PurpleStringref*purple_stringref_new(const char*);
const char*purple_smiley_get_checksum(const struct _PurpleSmiley*);
unsigned int purple_media_codec_get_clock_rate(struct _PurpleMediaCodec*);
void purple_certificate_display_x509(struct _PurpleCertificate*);
void purple_status_set_active(struct _PurpleStatus*,int);
char*purple_media_candidate_get_password(struct _PurpleMediaCandidate*);
void purple_proxy_info_set_username(struct PurpleProxyInfo*,const char*);
void purple_signals_uninit();
const char*purple_time_format(const struct tm*);
void purple_xfer_request(struct _PurpleXfer*);
void purple_notify_user_info_remove_entry(struct _PurpleNotifyUserInfo*,struct _PurpleNotifyUserInfoEntry*);
void purple_conv_chat_send(struct _PurpleConvChat*,const char*);
void purple_plugins_unregister_load_notify_cb(void(*)(struct _PurplePlugin*,void*));
struct _GList*purple_certificate_get_pools();
int purple_request_fields_get_bool(const struct PurpleRequestFields*,const char*);
void*purple_value_get_pointer(const struct PurpleValue*);
void purple_request_field_string_set_default_value(struct _PurpleRequestField*,const char*);
int purple_savedstatus_is_idleaway();
int purple_roomlist_field_get_hidden(struct _PurpleRoomlistField*);
const char*purple_xfer_get_remote_user(const struct _PurpleXfer*);
void purple_plugin_action_free(struct _PurplePluginAction*);
void purple_notify_uninit();
int purple_media_param_is_supported(struct _PurpleMedia*,const char*);
struct _PurplePluginPref*purple_plugin_pref_new();
void purple_account_request_close(void*);
const char*purple_txt_response_get_content(struct _PurpleTxtResponse*);
char*purple_mime_part_get_field_decoded(struct _PurpleMimePart*,const char*);
const char*purple_cipher_get_name(struct _PurpleCipher*);
const struct PurpleConnectionErrorInfo*purple_account_get_current_error(struct _PurpleAccount*);
void purple_roomlist_cancel_get_list(struct _PurpleRoomlist*);
int purple_log_common_is_deletable(struct _PurpleLog*);
void purple_prpl_got_user_login_time(struct _PurpleAccount*,const char*,long);
struct _PurpleMediaManager*purple_media_get_manager(struct _PurpleMedia*);
struct _GList*purple_log_logger_get_options();
struct _PurplePlugin*purple_plugins_find_with_id(const char*);
const char*purple_mime_part_get_data(struct _PurpleMimePart*);
struct _PurpleStoredImage*purple_buddy_icons_node_set_custom_icon_from_file(struct _PurpleBlistNode*,const char*);
int purple_core_init(const char*);
enum _PurpleCmdStatus purple_cmd_do_command(struct _PurpleConversation*,const char*,const char*,char**);
int purple_media_set_send_codec(struct _PurpleMedia*,const char*,struct _PurpleMediaCodec*);
int purple_input_get_error(int,int*);
void purple_smileys_uninit();
unsigned long purple_sound_theme_get_type();
void purple_request_field_list_add_icon(struct _PurpleRequestField*,const char*,const char*,void*);
char*purple_media_candidate_get_ip(struct _PurpleMediaCandidate*);
void purple_accounts_delete(struct _PurpleAccount*);
void purple_signals_disconnect_by_handle(void*);
void purple_marshal_VOID__POINTER_INT_POINTER(void(*)(),__builtin_va_list,void*,void**);
const char*purple_theme_get_name(struct _PurpleTheme*);
void purple_marshal_VOID__POINTER_POINTER_UINT(void(*)(),__builtin_va_list,void*,void**);
void purple_blist_merge_contact(struct _PurpleContact*,struct _PurpleBlistNode*);
void purple_mime_part_get_data_decoded(struct _PurpleMimePart*,unsigned char**,unsigned long*);
void purple_account_set_public_alias(struct _PurpleAccount*,const char*,void(*)(struct _PurpleAccount*,const char*),void(*)(struct _PurpleAccount*,const char*));
const char*purple_plugin_get_description(const struct _PurplePlugin*);
void purple_prpl_send_attention(struct _PurpleConnection*,const char*,unsigned int);
void purple_connection_set_display_name(struct _PurpleConnection*,const char*);
void purple_marshal_BOOLEAN__POINTER_POINTER_POINTER_POINTER_POINTER(void(*)(),__builtin_va_list,void*,void**);
void purple_notify_user_info_remove_last_item(struct _PurpleNotifyUserInfo*);
int purple_ip_address_is_valid(const char*);
void purple_marshal_BOOLEAN__POINTER(void(*)(),__builtin_va_list,void*,void**);
void purple_marshal_VOID__POINTER_POINTER_POINTER_POINTER_UINT(void(*)(),__builtin_va_list,void*,void**);
struct _PurplePlugin*purple_find_prpl(const char*);
struct PurpleCore*purple_get_core();
void purple_prefs_disconnect_callback(unsigned int);
struct _PurpleRequestField*purple_request_fields_get_field(const struct PurpleRequestFields*,const char*);
void purple_account_set_register_callback(struct _PurpleAccount*,void(*)(struct _PurpleAccount*,int,void*),void*);
void purple_network_set_public_ip(const char*);
void*purple_signal_emit_vargs_return_1(void*,const char*,__builtin_va_list);
struct PurpleXferUiOps*purple_xfer_get_ui_ops(const struct _PurpleXfer*);
int purple_cipher_digest_region(const char*,const unsigned char*,unsigned long,unsigned long,unsigned char,unsigned long*);
unsigned int purple_notify_searchresults_get_columns_count(struct PurpleNotifySearchResults*);
void purple_prpl_got_media_caps(struct _PurpleAccount*,const char*);
void purple_marshal_BOOLEAN__POINTER_POINTER_POINTER_POINTER_UINT(void(*)(),__builtin_va_list,void*,void**);
int purple_plugin_ipc_get_params(struct _PurplePlugin*,const char*,struct PurpleValue**,int*,struct PurpleValue***);
void*purple_blist_get_ui_data();
void purple_network_listen_map_external(int);
]])
local CLIB = ffi.load(_G.FFI_LIB or "purple")
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



--====helper chars_to_string====
	local function chars_to_string(ctyp)
		if ctyp ~= nil then
			return ffi.string(ctyp)
		end
		return ""
	end
--====helper chars_to_string====

local function glist_to_table(l, meta_name)
	if not metatables[meta_name] then
		error((meta_name or "nil") .. " is not a valid meta type", 3)
	end

	local out = {}

	for i = 1, math.huge do
		out[i] = ffi.cast(metatables[meta_name].ctype, l.data)
		out[i] = wrap_pointer(out[i], meta_name)
		l = l.next
		if l == nil then
			break
		end
	end

	return out
end

ffi.cdef("struct _GList * g_list_insert(struct _GList *, void *, int);\nstruct _GList * g_list_alloc();")

local function table_to_glist(tbl)
	local list = CLIB.g_list_alloc()
	for i, v in ipairs(tbl) do
		CLIB.g_list_insert(list, v.ptr, -1)
	end
	return list
end

ffi.cdef([==[
	void *malloc(size_t size);
	void free(void *ptr);
]==])

local function replace_buffer(buffer, size, data)
	ffi.C.free(buffer[0])
	local chr = ffi.C.malloc(size)
	ffi.copy(chr, data)
	buffer[0] = chr
end

local function create_boxed_table(p, wrap_in, wrap_out)
	return setmetatable({p = p}, {__newindex = function(s, k, v) wrap_in(s.p, v) end, __index = function(s, k) return wrap_out(s.p) end})
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleBuddyIcon"),
		Ref = function(self) local v = CLIB.purple_buddy_icon_ref(self.ptr) v = wrap_pointer(v, "BuddyIcon") return v end,
		GetData = function(self, len) local v = CLIB.purple_buddy_icon_get_data(self.ptr, len)  return v end,
		GetAccount = function(self) local v = CLIB.purple_buddy_icon_get_account(self.ptr) v = wrap_pointer(v, "Account") return v end,
		GetFullPath = function(self) local v = CLIB.purple_buddy_icon_get_full_path(self.ptr) v = chars_to_string(v) return v end,
		GetUsername = function(self) local v = CLIB.purple_buddy_icon_get_username(self.ptr) v = chars_to_string(v) return v end,
		GetChecksum = function(self) local v = CLIB.purple_buddy_icon_get_checksum(self.ptr) v = chars_to_string(v) return v end,
		GetExtension = function(self) local v = CLIB.purple_buddy_icon_get_extension(self.ptr) v = chars_to_string(v) return v end,
		SetData = function(self, data, len, checksum) local v = CLIB.purple_buddy_icon_set_data(self.ptr, data, len, checksum)  return v end,
		Update = function(self) local v = CLIB.purple_buddy_icon_update(self.ptr)  return v end,
		Unref = function(self) local v = CLIB.purple_buddy_icon_unref(self.ptr) v = wrap_pointer(v, "BuddyIcon") return v end,
	}
	META.__index = META
	metatables.BuddyIcon = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleCertificatePool"),
		Mkpath = function(self, id) local v = CLIB.purple_certificate_pool_mkpath(self.ptr, id) v = chars_to_string(v) return v end,
		Delete = function(self, id) local v = CLIB.purple_certificate_pool_delete(self.ptr, id)  return v end,
		Usable = function(self) local v = CLIB.purple_certificate_pool_usable(self.ptr)  return v end,
		Contains = function(self, id) local v = CLIB.purple_certificate_pool_contains(self.ptr, id)  return v end,
		GetScheme = function(self) local v = CLIB.purple_certificate_pool_get_scheme(self.ptr)  return v end,
		Retrieve = function(self, id) local v = CLIB.purple_certificate_pool_retrieve(self.ptr, id) v = wrap_pointer(v, "Certificate") return v end,
		Store = function(self, id, crt) local v = CLIB.purple_certificate_pool_store(self.ptr, id, crt.ptr)  return v end,
		GetIdlist = function(selfcast_type) local v = CLIB.purple_certificate_pool_get_idlist(self.ptr) v = glist_to_table(v, cast_type) return v end,
	}
	META.__index = META
	metatables.CertificatePool = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleConvMessage"),
		GetFlags = function(self) local v = CLIB.purple_conversation_message_get_flags(self.ptr)  return v end,
		GetTimestamp = function(self) local v = CLIB.purple_conversation_message_get_timestamp(self.ptr)  return v end,
		GetMessage = function(self) local v = CLIB.purple_conversation_message_get_message(self.ptr) v = chars_to_string(v) return v end,
		GetSender = function(self) local v = CLIB.purple_conversation_message_get_sender(self.ptr) v = chars_to_string(v) return v end,
	}
	META.__index = META
	metatables.ConvMessage = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleCipher"),
		GetName = function(self) local v = CLIB.purple_cipher_get_name(self.ptr) v = chars_to_string(v) return v end,
		GetCapabilities = function(self) local v = CLIB.purple_cipher_get_capabilities(self.ptr)  return v end,
		ContextNew = function(self, extra) local v = CLIB.purple_cipher_context_new(self.ptr, extra) v = wrap_pointer(v, "CipherContext") return v end,
	}
	META.__index = META
	metatables.Cipher = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleStringref"),
		Value = function(self) local v = CLIB.purple_stringref_value(self.ptr) v = chars_to_string(v) return v end,
		Ref = function(self) local v = CLIB.purple_stringref_ref(self.ptr) v = wrap_pointer(v, "Stringref") return v end,
		Cmp = function(self, s2) local v = CLIB.purple_stringref_cmp(self.ptr, s2.ptr)  return v end,
		Len = function(self) local v = CLIB.purple_stringref_len(self.ptr)  return v end,
		Unref = function(self) local v = CLIB.purple_stringref_unref(self.ptr)  return v end,
	}
	META.__index = META
	metatables.Stringref = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleConvIm"),
		StopTypingTimeout = function(self) local v = CLIB.purple_conv_im_stop_typing_timeout(self.ptr)  return v end,
		GetSendTypedTimeout = function(self) local v = CLIB.purple_conv_im_get_send_typed_timeout(self.ptr)  return v end,
		GetTypeAgain = function(self) local v = CLIB.purple_conv_im_get_type_again(self.ptr)  return v end,
		GetTypingTimeout = function(self) local v = CLIB.purple_conv_im_get_typing_timeout(self.ptr)  return v end,
		SendWithFlags = function(self, message, flags) local v = CLIB.purple_conv_im_send_with_flags(self.ptr, message, flags)  return v end,
		Write = function(self, who, message, flags, mtime) local v = CLIB.purple_conv_im_write(self.ptr, who, message, flags, mtime)  return v end,
		GetConversation = function(self) local v = CLIB.purple_conv_im_get_conversation(self.ptr) v = wrap_pointer(v, "Conversation") return v end,
		GetIcon = function(self) local v = CLIB.purple_conv_im_get_icon(self.ptr) v = wrap_pointer(v, "BuddyIcon") return v end,
		UpdateTyping = function(self) local v = CLIB.purple_conv_im_update_typing(self.ptr)  return v end,
		StopSendTypedTimeout = function(self) local v = CLIB.purple_conv_im_stop_send_typed_timeout(self.ptr)  return v end,
		StartTypingTimeout = function(self, timeout) local v = CLIB.purple_conv_im_start_typing_timeout(self.ptr, timeout)  return v end,
		SetIcon = function(self, icon) local v = CLIB.purple_conv_im_set_icon(self.ptr, icon.ptr)  return v end,
		StartSendTypedTimeout = function(self) local v = CLIB.purple_conv_im_start_send_typed_timeout(self.ptr)  return v end,
		SetTypeAgain = function(self, val) local v = CLIB.purple_conv_im_set_type_again(self.ptr, val)  return v end,
		Send = function(self, message) local v = CLIB.purple_conv_im_send(self.ptr, message)  return v end,
		SetTypingState = function(self, state) local v = CLIB.purple_conv_im_set_typing_state(self.ptr, state)  return v end,
		GetTypingState = function(self) local v = CLIB.purple_conv_im_get_typing_state(self.ptr)  return v end,
	}
	META.__index = META
	metatables.ConvIm = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleMediaCodec"),
		GetClockRate = function(self) local v = CLIB.purple_media_codec_get_clock_rate(self.ptr)  return v end,
		GetChannels = function(self) local v = CLIB.purple_media_codec_get_channels(self.ptr)  return v end,
		GetOptionalParameter = function(self, name, value) local v = CLIB.purple_media_codec_get_optional_parameter(self.ptr, name, value)  return v end,
		GetEncodingName = function(self) local v = CLIB.purple_media_codec_get_encoding_name(self.ptr) v = chars_to_string(v) return v end,
		RemoveOptionalParameter = function(self, param) local v = CLIB.purple_media_codec_remove_optional_parameter(self.ptr, param)  return v end,
		AddOptionalParameter = function(self, name, value) local v = CLIB.purple_media_codec_add_optional_parameter(self.ptr, name, value)  return v end,
		GetOptionalParameters = function(selfcast_type) local v = CLIB.purple_media_codec_get_optional_parameters(self.ptr) v = glist_to_table(v, cast_type) return v end,
		Copy = function(self) local v = CLIB.purple_media_codec_copy(self.ptr) v = wrap_pointer(v, "MediaCodec") return v end,
		ToString = function(self) local v = CLIB.purple_media_codec_to_string(self.ptr) v = chars_to_string(v) return v end,
		GetId = function(self) local v = CLIB.purple_media_codec_get_id(self.ptr)  return v end,
	}
	META.__index = META
	metatables.MediaCodec = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurplePluginAction"),
		Free = function(self) local v = CLIB.purple_plugin_action_free(self.ptr)  return v end,
	}
	META.__index = META
	metatables.PluginAction = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleSoundTheme"),
		SetFile = function(self, event, filename) local v = CLIB.purple_sound_theme_set_file(self.ptr, event, filename)  return v end,
		GetFileFull = function(self, event) local v = CLIB.purple_sound_theme_get_file_full(self.ptr, event) v = chars_to_string(v) return v end,
		GetFile = function(self, event) local v = CLIB.purple_sound_theme_get_file(self.ptr, event) v = chars_to_string(v) return v end,
	}
	META.__index = META
	metatables.SoundTheme = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleConvChat"),
		Unignore = function(self, name) local v = CLIB.purple_conv_chat_unignore(self.ptr, name)  return v end,
		GetId = function(self) local v = CLIB.purple_conv_chat_get_id(self.ptr)  return v end,
		GetIgnored = function(selfcast_type) local v = CLIB.purple_conv_chat_get_ignored(self.ptr) v = glist_to_table(v, cast_type) return v end,
		GetNick = function(self) local v = CLIB.purple_conv_chat_get_nick(self.ptr) v = chars_to_string(v) return v end,
		GetUsers = function(selfcast_type) local v = CLIB.purple_conv_chat_get_users(self.ptr) v = glist_to_table(v, cast_type) return v end,
		UserGetFlags = function(self, user) local v = CLIB.purple_conv_chat_user_get_flags(self.ptr, user)  return v end,
		RenameUser = function(self, old_user, new_user) local v = CLIB.purple_conv_chat_rename_user(self.ptr, old_user, new_user)  return v end,
		Left = function(self) local v = CLIB.purple_conv_chat_left(self.ptr)  return v end,
		AddUsers = function(self, users, extra_msgs, flags, new_arrivals) local v = CLIB.purple_conv_chat_add_users(self.ptr, table_to_glist(users), table_to_glist(extra_msgs), table_to_glist(flags), new_arrivals)  return v end,
		GetConversation = function(self) local v = CLIB.purple_conv_chat_get_conversation(self.ptr) v = wrap_pointer(v, "Conversation") return v end,
		GetTopic = function(self) local v = CLIB.purple_conv_chat_get_topic(self.ptr) v = chars_to_string(v) return v end,
		SetIgnored = function(self, ignored, cast_type) local v = CLIB.purple_conv_chat_set_ignored(self.ptr, table_to_glist(ignored)) v = glist_to_table(v, cast_type) return v end,
		UserSetFlags = function(self, user, flags) local v = CLIB.purple_conv_chat_user_set_flags(self.ptr, user, flags)  return v end,
		Send = function(self, message) local v = CLIB.purple_conv_chat_send(self.ptr, message)  return v end,
		HasLeft = function(self) local v = CLIB.purple_conv_chat_has_left(self.ptr)  return v end,
		IsUserIgnored = function(self, user) local v = CLIB.purple_conv_chat_is_user_ignored(self.ptr, user)  return v end,
		SetId = function(self, id) local v = CLIB.purple_conv_chat_set_id(self.ptr, id)  return v end,
		RemoveUsers = function(self, users, reason) local v = CLIB.purple_conv_chat_remove_users(self.ptr, table_to_glist(users), reason)  return v end,
		CbSetAttributes = function(self, cb, keys, values) local v = CLIB.purple_conv_chat_cb_set_attributes(self.ptr, cb, table_to_glist(keys), table_to_glist(values))  return v end,
		ClearUsers = function(self) local v = CLIB.purple_conv_chat_clear_users(self.ptr)  return v end,
		AddUser = function(self, user, extra_msg, flags, new_arrival) local v = CLIB.purple_conv_chat_add_user(self.ptr, user, extra_msg, flags, new_arrival)  return v end,
		RemoveUser = function(self, user, reason) local v = CLIB.purple_conv_chat_remove_user(self.ptr, user, reason)  return v end,
		SendWithFlags = function(self, message, flags) local v = CLIB.purple_conv_chat_send_with_flags(self.ptr, message, flags)  return v end,
		Write = function(self, who, message, flags, mtime) local v = CLIB.purple_conv_chat_write(self.ptr, who, message, flags, mtime)  return v end,
		CbFind = function(self, name) local v = CLIB.purple_conv_chat_cb_find(self.ptr, name)  return v end,
		GetIgnoredUser = function(self, user) local v = CLIB.purple_conv_chat_get_ignored_user(self.ptr, user) v = chars_to_string(v) return v end,
		Ignore = function(self, name) local v = CLIB.purple_conv_chat_ignore(self.ptr, name)  return v end,
		SetUsers = function(self, users, cast_type) local v = CLIB.purple_conv_chat_set_users(self.ptr, table_to_glist(users)) v = glist_to_table(v, cast_type) return v end,
		SetTopic = function(self, who, topic) local v = CLIB.purple_conv_chat_set_topic(self.ptr, who, topic)  return v end,
		SetNick = function(self, nick) local v = CLIB.purple_conv_chat_set_nick(self.ptr, nick)  return v end,
		FindUser = function(self, user) local v = CLIB.purple_conv_chat_find_user(self.ptr, user)  return v end,
		CbSetAttribute = function(self, cb, key, value) local v = CLIB.purple_conv_chat_cb_set_attribute(self.ptr, cb, key, value)  return v end,
		InviteUser = function(self, user, message, confirm) local v = CLIB.purple_conv_chat_invite_user(self.ptr, user, message, confirm)  return v end,
	}
	META.__index = META
	metatables.ConvChat = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurplePluginPref"),
		GetChoices = function(selfcast_type) local v = CLIB.purple_plugin_pref_get_choices(self.ptr) v = glist_to_table(v, cast_type) return v end,
		SetBounds = function(self, min, max) local v = CLIB.purple_plugin_pref_set_bounds(self.ptr, min, max)  return v end,
		GetType = function(self) local v = CLIB.purple_plugin_pref_get_type(self.ptr)  return v end,
		GetBounds = function(self, min, max) local v = CLIB.purple_plugin_pref_get_bounds(self.ptr, min, max)  return v end,
		SetFormatType = function(self, format) local v = CLIB.purple_plugin_pref_set_format_type(self.ptr, format)  return v end,
		SetMaxLength = function(self, max_length) local v = CLIB.purple_plugin_pref_set_max_length(self.ptr, max_length)  return v end,
		GetFormatType = function(self) local v = CLIB.purple_plugin_pref_get_format_type(self.ptr)  return v end,
		SetLabel = function(self, label) local v = CLIB.purple_plugin_pref_set_label(self.ptr, label)  return v end,
		Destroy = function(self) local v = CLIB.purple_plugin_pref_destroy(self.ptr)  return v end,
		GetName = function(self) local v = CLIB.purple_plugin_pref_get_name(self.ptr) v = chars_to_string(v) return v end,
		SetMasked = function(self, mask) local v = CLIB.purple_plugin_pref_set_masked(self.ptr, mask)  return v end,
		GetLabel = function(self) local v = CLIB.purple_plugin_pref_get_label(self.ptr) v = chars_to_string(v) return v end,
		GetMasked = function(self) local v = CLIB.purple_plugin_pref_get_masked(self.ptr)  return v end,
		SetType = function(self, type) local v = CLIB.purple_plugin_pref_set_type(self.ptr, type)  return v end,
		GetMaxLength = function(self) local v = CLIB.purple_plugin_pref_get_max_length(self.ptr)  return v end,
		SetName = function(self, name) local v = CLIB.purple_plugin_pref_set_name(self.ptr, name)  return v end,
		AddChoice = function(self, label, choice) local v = CLIB.purple_plugin_pref_add_choice(self.ptr, label, choice)  return v end,
	}
	META.__index = META
	metatables.PluginPref = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleBlistNode"),
		GetBool = function(self, key) local v = CLIB.purple_blist_node_get_bool(self.ptr, key)  return v end,
		SetUiData = function(self, ui_data) local v = CLIB.purple_blist_node_set_ui_data(self.ptr, ui_data)  return v end,
		GetType = function(self) local v = CLIB.purple_blist_node_get_type(self.ptr)  return v end,
		GetSiblingNext = function(self) local v = CLIB.purple_blist_node_get_sibling_next(self.ptr) v = wrap_pointer(v, "BlistNode") return v end,
		GetString = function(self, key) local v = CLIB.purple_blist_node_get_string(self.ptr, key) v = chars_to_string(v) return v end,
		GetFirstChild = function(self) local v = CLIB.purple_blist_node_get_first_child(self.ptr) v = wrap_pointer(v, "BlistNode") return v end,
		GetSiblingPrev = function(self) local v = CLIB.purple_blist_node_get_sibling_prev(self.ptr) v = wrap_pointer(v, "BlistNode") return v end,
		GetFlags = function(self) local v = CLIB.purple_blist_node_get_flags(self.ptr)  return v end,
		SetString = function(self, key, value) local v = CLIB.purple_blist_node_set_string(self.ptr, key, value)  return v end,
		SetBool = function(self, key, value) local v = CLIB.purple_blist_node_set_bool(self.ptr, key, value)  return v end,
		GetInt = function(self, key) local v = CLIB.purple_blist_node_get_int(self.ptr, key)  return v end,
		GetUiData = function(self) local v = CLIB.purple_blist_node_get_ui_data(self.ptr)  return v end,
		GetExtendedMenu = function(selfcast_type) local v = CLIB.purple_blist_node_get_extended_menu(self.ptr) v = glist_to_table(v, cast_type) return v end,
		SetFlags = function(self, flags) local v = CLIB.purple_blist_node_set_flags(self.ptr, flags)  return v end,
		Next = function(self, offline) local v = CLIB.purple_blist_node_next(self.ptr, offline) v = wrap_pointer(v, "BlistNode") return v end,
		GetParent = function(self) local v = CLIB.purple_blist_node_get_parent(self.ptr) v = wrap_pointer(v, "BlistNode") return v end,
		RemoveSetting = function(self, key) local v = CLIB.purple_blist_node_remove_setting(self.ptr, key)  return v end,
		SetInt = function(self, key, value) local v = CLIB.purple_blist_node_set_int(self.ptr, key, value)  return v end,
	}
	META.__index = META
	metatables.BlistNode = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleRoomlist"),
		GetFields = function(selfcast_type) local v = CLIB.purple_roomlist_get_fields(self.ptr) v = glist_to_table(v, cast_type) return v end,
		RoomAddField = function(self, room, field) local v = CLIB.purple_roomlist_room_add_field(self.ptr, room.ptr, field)  return v end,
		Ref = function(self) local v = CLIB.purple_roomlist_ref(self.ptr)  return v end,
		CancelGetList = function(self) local v = CLIB.purple_roomlist_cancel_get_list(self.ptr)  return v end,
		GetInProgress = function(self) local v = CLIB.purple_roomlist_get_in_progress(self.ptr)  return v end,
		RoomAdd = function(self, room) local v = CLIB.purple_roomlist_room_add(self.ptr, room.ptr)  return v end,
		SetInProgress = function(self, in_progress) local v = CLIB.purple_roomlist_set_in_progress(self.ptr, in_progress)  return v end,
		SetFields = function(self, fields) local v = CLIB.purple_roomlist_set_fields(self.ptr, table_to_glist(fields))  return v end,
		ExpandCategory = function(self, category) local v = CLIB.purple_roomlist_expand_category(self.ptr, category.ptr)  return v end,
		RoomJoin = function(self, room) local v = CLIB.purple_roomlist_room_join(self.ptr, room.ptr)  return v end,
		Unref = function(self) local v = CLIB.purple_roomlist_unref(self.ptr)  return v end,
	}
	META.__index = META
	metatables.Roomlist = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleRoomlistRoom"),
		GetFields = function(selfcast_type) local v = CLIB.purple_roomlist_room_get_fields(self.ptr) v = glist_to_table(v, cast_type) return v end,
		GetParent = function(self) local v = CLIB.purple_roomlist_room_get_parent(self.ptr) v = wrap_pointer(v, "RoomlistRoom") return v end,
		GetName = function(self) local v = CLIB.purple_roomlist_room_get_name(self.ptr) v = chars_to_string(v) return v end,
		GetType = function(self) local v = CLIB.purple_roomlist_room_get_type(self.ptr)  return v end,
	}
	META.__index = META
	metatables.RoomlistRoom = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleRequestField"),
		ListGetItems = function(selfcast_type) local v = CLIB.purple_request_field_list_get_items(self.ptr) v = glist_to_table(v, cast_type) return v end,
		ImageGetScaleY = function(self) local v = CLIB.purple_request_field_image_get_scale_y(self.ptr)  return v end,
		BoolGetValue = function(self) local v = CLIB.purple_request_field_bool_get_value(self.ptr)  return v end,
		ChoiceSetDefaultValue = function(self, default_value) local v = CLIB.purple_request_field_choice_set_default_value(self.ptr, default_value)  return v end,
		GetType = function(self) local v = CLIB.purple_request_field_get_type(self.ptr)  return v end,
		StringGetDefaultValue = function(self) local v = CLIB.purple_request_field_string_get_default_value(self.ptr) v = chars_to_string(v) return v end,
		GetGroup = function(self) local v = CLIB.purple_request_field_get_group(self.ptr) v = wrap_pointer(v, "RequestFieldGroup") return v end,
		ListAdd = function(self, item, data) local v = CLIB.purple_request_field_list_add(self.ptr, item, data)  return v end,
		IsRequired = function(self) local v = CLIB.purple_request_field_is_required(self.ptr)  return v end,
		ChoiceAdd = function(self, label) local v = CLIB.purple_request_field_choice_add(self.ptr, label)  return v end,
		StringSetValue = function(self, value) local v = CLIB.purple_request_field_string_set_value(self.ptr, value)  return v end,
		StringGetValue = function(self) local v = CLIB.purple_request_field_string_get_value(self.ptr) v = chars_to_string(v) return v end,
		ImageSetScale = function(self, x, y) local v = CLIB.purple_request_field_image_set_scale(self.ptr, x, y)  return v end,
		BoolSetDefaultValue = function(self, default_value) local v = CLIB.purple_request_field_bool_set_default_value(self.ptr, default_value)  return v end,
		AccountGetValue = function(self) local v = CLIB.purple_request_field_account_get_value(self.ptr) v = wrap_pointer(v, "Account") return v end,
		ListSetMultiSelect = function(self, multi_select) local v = CLIB.purple_request_field_list_set_multi_select(self.ptr, multi_select)  return v end,
		AccountGetShowAll = function(self) local v = CLIB.purple_request_field_account_get_show_all(self.ptr)  return v end,
		AccountGetDefaultValue = function(self) local v = CLIB.purple_request_field_account_get_default_value(self.ptr) v = wrap_pointer(v, "Account") return v end,
		ListAddIcon = function(self, item, icon_path, data) local v = CLIB.purple_request_field_list_add_icon(self.ptr, item, icon_path, data)  return v end,
		AccountGetFilter = function(self) local v = CLIB.purple_request_field_account_get_filter(self.ptr)  return v end,
		SetLabel = function(self, label) local v = CLIB.purple_request_field_set_label(self.ptr, label)  return v end,
		ListAddSelected = function(self, item) local v = CLIB.purple_request_field_list_add_selected(self.ptr, item)  return v end,
		StringSetEditable = function(self, editable) local v = CLIB.purple_request_field_string_set_editable(self.ptr, editable)  return v end,
		SetRequired = function(self, required) local v = CLIB.purple_request_field_set_required(self.ptr, required)  return v end,
		GetLabel = function(self) local v = CLIB.purple_request_field_get_label(self.ptr) v = chars_to_string(v) return v end,
		ListGetIcons = function(selfcast_type) local v = CLIB.purple_request_field_list_get_icons(self.ptr) v = glist_to_table(v, cast_type) return v end,
		ChoiceGetDefaultValue = function(self) local v = CLIB.purple_request_field_choice_get_default_value(self.ptr)  return v end,
		ListGetData = function(self, text) local v = CLIB.purple_request_field_list_get_data(self.ptr, text)  return v end,
		ChoiceSetValue = function(self, value) local v = CLIB.purple_request_field_choice_set_value(self.ptr, value)  return v end,
		GetId = function(self) local v = CLIB.purple_request_field_get_id(self.ptr) v = chars_to_string(v) return v end,
		AccountSetValue = function(self, value) local v = CLIB.purple_request_field_account_set_value(self.ptr, value.ptr)  return v end,
		ListClearSelected = function(self) local v = CLIB.purple_request_field_list_clear_selected(self.ptr)  return v end,
		SetVisible = function(self, visible) local v = CLIB.purple_request_field_set_visible(self.ptr, visible)  return v end,
		IsVisible = function(self) local v = CLIB.purple_request_field_is_visible(self.ptr)  return v end,
		StringIsEditable = function(self) local v = CLIB.purple_request_field_string_is_editable(self.ptr)  return v end,
		SetUiData = function(self, ui_data) local v = CLIB.purple_request_field_set_ui_data(self.ptr, ui_data)  return v end,
		AccountSetDefaultValue = function(self, default_value) local v = CLIB.purple_request_field_account_set_default_value(self.ptr, default_value.ptr)  return v end,
		StringIsMasked = function(self) local v = CLIB.purple_request_field_string_is_masked(self.ptr)  return v end,
		ListSetSelected = function(self, items) local v = CLIB.purple_request_field_list_set_selected(self.ptr, table_to_glist(items))  return v end,
		IntSetValue = function(self, value) local v = CLIB.purple_request_field_int_set_value(self.ptr, value)  return v end,
		ImageGetBuffer = function(self) local v = CLIB.purple_request_field_image_get_buffer(self.ptr) v = chars_to_string(v) return v end,
		IntGetDefaultValue = function(self) local v = CLIB.purple_request_field_int_get_default_value(self.ptr)  return v end,
		AccountSetFilter = function(self, filter_func) local v = CLIB.purple_request_field_account_set_filter(self.ptr, filter_func)  return v end,
		ChoiceGetValue = function(self) local v = CLIB.purple_request_field_choice_get_value(self.ptr)  return v end,
		GetUiData = function(self) local v = CLIB.purple_request_field_get_ui_data(self.ptr)  return v end,
		StringIsMultiline = function(self) local v = CLIB.purple_request_field_string_is_multiline(self.ptr)  return v end,
		ImageGetSize = function(self) local v = CLIB.purple_request_field_image_get_size(self.ptr)  return v end,
		ListGetMultiSelect = function(self) local v = CLIB.purple_request_field_list_get_multi_select(self.ptr)  return v end,
		Destroy = function(self) local v = CLIB.purple_request_field_destroy(self.ptr)  return v end,
		StringSetDefaultValue = function(self, default_value) local v = CLIB.purple_request_field_string_set_default_value(self.ptr, default_value)  return v end,
		SetTypeHint = function(self, type_hint) local v = CLIB.purple_request_field_set_type_hint(self.ptr, type_hint)  return v end,
		IntGetValue = function(self) local v = CLIB.purple_request_field_int_get_value(self.ptr)  return v end,
		ChoiceGetLabels = function(selfcast_type) local v = CLIB.purple_request_field_choice_get_labels(self.ptr) v = glist_to_table(v, cast_type) return v end,
		AccountSetShowAll = function(self, show_all) local v = CLIB.purple_request_field_account_set_show_all(self.ptr, show_all)  return v end,
		BoolSetValue = function(self, value) local v = CLIB.purple_request_field_bool_set_value(self.ptr, value)  return v end,
		ImageGetScaleX = function(self) local v = CLIB.purple_request_field_image_get_scale_x(self.ptr)  return v end,
		ListGetSelected = function(selfcast_type) local v = CLIB.purple_request_field_list_get_selected(self.ptr) v = glist_to_table(v, cast_type) return v end,
		ListIsSelected = function(self, item) local v = CLIB.purple_request_field_list_is_selected(self.ptr, item)  return v end,
		BoolGetDefaultValue = function(self) local v = CLIB.purple_request_field_bool_get_default_value(self.ptr)  return v end,
		StringSetMasked = function(self, masked) local v = CLIB.purple_request_field_string_set_masked(self.ptr, masked)  return v end,
		GetTypeHint = function(self) local v = CLIB.purple_request_field_get_type_hint(self.ptr) v = chars_to_string(v) return v end,
		IntSetDefaultValue = function(self, default_value) local v = CLIB.purple_request_field_int_set_default_value(self.ptr, default_value)  return v end,
	}
	META.__index = META
	metatables.RequestField = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleCipherContext"),
		GetBatchMode = function(self) local v = CLIB.purple_cipher_context_get_batch_mode(self.ptr)  return v end,
		SetKeyWithLen = function(self, key, len) local v = CLIB.purple_cipher_context_set_key_with_len(self.ptr, key, len)  return v end,
		Reset = function(self, extra) local v = CLIB.purple_cipher_context_reset(self.ptr, extra)  return v end,
		SetKey = function(self, key) local v = CLIB.purple_cipher_context_set_key(self.ptr, key)  return v end,
		GetSaltSize = function(self) local v = CLIB.purple_cipher_context_get_salt_size(self.ptr)  return v end,
		DigestToStr = function(self, in_len, unknown_3, out_len) local v = CLIB.purple_cipher_context_digest_to_str(self.ptr, in_len, unknown_3, out_len)  return v end,
		GetKeySize = function(self) local v = CLIB.purple_cipher_context_get_key_size(self.ptr)  return v end,
		SetIv = function(self, iv, len) local v = CLIB.purple_cipher_context_set_iv(self.ptr, iv, len)  return v end,
		Append = function(self, data, len) local v = CLIB.purple_cipher_context_append(self.ptr, data, len)  return v end,
		Digest = function(self, in_len, unknown_3, out_len) local v = CLIB.purple_cipher_context_digest(self.ptr, in_len, unknown_3, out_len)  return v end,
		SetData = function(self, data) local v = CLIB.purple_cipher_context_set_data(self.ptr, data)  return v end,
		Destroy = function(self) local v = CLIB.purple_cipher_context_destroy(self.ptr)  return v end,
		Decrypt = function(self, unknown_2, len, unknown_4, outlen) local v = CLIB.purple_cipher_context_decrypt(self.ptr, unknown_2, len, unknown_4, outlen)  return v end,
		SetSalt = function(self, salt) local v = CLIB.purple_cipher_context_set_salt(self.ptr, salt)  return v end,
		GetBlockSize = function(self) local v = CLIB.purple_cipher_context_get_block_size(self.ptr)  return v end,
		SetOption = function(self, name, value) local v = CLIB.purple_cipher_context_set_option(self.ptr, name, value)  return v end,
		GetOption = function(self, name) local v = CLIB.purple_cipher_context_get_option(self.ptr, name)  return v end,
		GetData = function(self) local v = CLIB.purple_cipher_context_get_data(self.ptr)  return v end,
		Encrypt = function(self, unknown_2, len, unknown_4, outlen) local v = CLIB.purple_cipher_context_encrypt(self.ptr, unknown_2, len, unknown_4, outlen)  return v end,
		SetBatchMode = function(self, mode) local v = CLIB.purple_cipher_context_set_batch_mode(self.ptr, mode)  return v end,
	}
	META.__index = META
	metatables.CipherContext = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleSrvTxtQueryData"),
		GetQuery = function(self) local v = CLIB.purple_srv_txt_query_get_query(self.ptr) v = chars_to_string(v) return v end,
		Destroy = function(self) local v = CLIB.purple_srv_txt_query_destroy(self.ptr)  return v end,
		GetType = function(self) local v = CLIB.purple_srv_txt_query_get_type(self.ptr)  return v end,
	}
	META.__index = META
	metatables.SrvQueryData = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleMimeDocument"),
		GetParts = function(selfcast_type) local v = CLIB.purple_mime_document_get_parts(self.ptr) v = glist_to_table(v, cast_type) return v end,
		Write = function(self, str) local v = CLIB.purple_mime_document_write(self.ptr, str)  return v end,
		Free = function(self) local v = CLIB.purple_mime_document_free(self.ptr)  return v end,
		GetField = function(self, field) local v = CLIB.purple_mime_document_get_field(self.ptr, field) v = chars_to_string(v) return v end,
		SetField = function(self, field, value) local v = CLIB.purple_mime_document_set_field(self.ptr, field, value)  return v end,
		GetFields = function(selfcast_type) local v = CLIB.purple_mime_document_get_fields(self.ptr) v = glist_to_table(v, cast_type) return v end,
	}
	META.__index = META
	metatables.MimeDocument = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleCertificate"),
		GetIssuerUniqueId = function(self) local v = CLIB.purple_certificate_get_issuer_unique_id(self.ptr) v = chars_to_string(v) return v end,
		GetTimes = function(self, activation, expiration) local v = CLIB.purple_certificate_get_times(self.ptr, activation, expiration)  return v end,
		DisplayX509 = function(self) local v = CLIB.purple_certificate_display_x509(self.ptr)  return v end,
		Destroy = function(self) local v = CLIB.purple_certificate_destroy(self.ptr)  return v end,
		SignedBy = function(self, issuer) local v = CLIB.purple_certificate_signed_by(self.ptr, issuer.ptr)  return v end,
		GetFingerprintSha1 = function(self) local v = CLIB.purple_certificate_get_fingerprint_sha1(self.ptr)  return v end,
		CheckSubjectName = function(self, name) local v = CLIB.purple_certificate_check_subject_name(self.ptr, name)  return v end,
		GetUniqueId = function(self) local v = CLIB.purple_certificate_get_unique_id(self.ptr) v = chars_to_string(v) return v end,
		Copy = function(self) local v = CLIB.purple_certificate_copy(self.ptr) v = wrap_pointer(v, "Certificate") return v end,
		GetSubjectName = function(self) local v = CLIB.purple_certificate_get_subject_name(self.ptr) v = chars_to_string(v) return v end,
	}
	META.__index = META
	metatables.Certificate = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleMediaCandidate"),
		GetBasePort = function(self) local v = CLIB.purple_media_candidate_get_base_port(self.ptr)  return v end,
		GetProtocol = function(self) local v = CLIB.purple_media_candidate_get_protocol(self.ptr)  return v end,
		GetFoundation = function(self) local v = CLIB.purple_media_candidate_get_foundation(self.ptr) v = chars_to_string(v) return v end,
		Copy = function(self) local v = CLIB.purple_media_candidate_copy(self.ptr) v = wrap_pointer(v, "MediaCandidate") return v end,
		GetPriority = function(self) local v = CLIB.purple_media_candidate_get_priority(self.ptr)  return v end,
		GetTtl = function(self) local v = CLIB.purple_media_candidate_get_ttl(self.ptr)  return v end,
		GetIp = function(self) local v = CLIB.purple_media_candidate_get_ip(self.ptr) v = chars_to_string(v) return v end,
		GetUsername = function(self) local v = CLIB.purple_media_candidate_get_username(self.ptr) v = chars_to_string(v) return v end,
		GetBaseIp = function(self) local v = CLIB.purple_media_candidate_get_base_ip(self.ptr) v = chars_to_string(v) return v end,
		GetComponentId = function(self) local v = CLIB.purple_media_candidate_get_component_id(self.ptr)  return v end,
		GetCandidateType = function(self) local v = CLIB.purple_media_candidate_get_candidate_type(self.ptr)  return v end,
		GetPassword = function(self) local v = CLIB.purple_media_candidate_get_password(self.ptr) v = chars_to_string(v) return v end,
		GetPort = function(self) local v = CLIB.purple_media_candidate_get_port(self.ptr)  return v end,
	}
	META.__index = META
	metatables.MediaCandidate = META
end
do
	local META = {
		ctype = ffi.typeof("struct PurpleRequestFields"),
		GetBool = function(self, id) local v = CLIB.purple_request_fields_get_bool(self.ptr, id)  return v end,
		GetGroups = function(selfcast_type) local v = CLIB.purple_request_fields_get_groups(self.ptr) v = glist_to_table(v, cast_type) return v end,
		GetField = function(self, id) local v = CLIB.purple_request_fields_get_field(self.ptr, id) v = wrap_pointer(v, "RequestField") return v end,
		GetRequired = function(selfcast_type) local v = CLIB.purple_request_fields_get_required(self.ptr) v = glist_to_table(v, cast_type) return v end,
		Destroy = function(self) local v = CLIB.purple_request_fields_destroy(self.ptr)  return v end,
		GetAccount = function(self, id) local v = CLIB.purple_request_fields_get_account(self.ptr, id) v = wrap_pointer(v, "Account") return v end,
		GetInteger = function(self, id) local v = CLIB.purple_request_fields_get_integer(self.ptr, id)  return v end,
		GetString = function(self, id) local v = CLIB.purple_request_fields_get_string(self.ptr, id) v = chars_to_string(v) return v end,
		AddGroup = function(self, group) local v = CLIB.purple_request_fields_add_group(self.ptr, group.ptr)  return v end,
		GetChoice = function(self, id) local v = CLIB.purple_request_fields_get_choice(self.ptr, id)  return v end,
		IsFieldRequired = function(self, id) local v = CLIB.purple_request_fields_is_field_required(self.ptr, id)  return v end,
		AllRequiredFilled = function(self) local v = CLIB.purple_request_fields_all_required_filled(self.ptr)  return v end,
		Exists = function(self, id) local v = CLIB.purple_request_fields_exists(self.ptr, id)  return v end,
	}
	META.__index = META
	metatables.RequestFields = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleAccount"),
		SetProxyInfo = function(self, info) local v = CLIB.purple_account_set_proxy_info(self.ptr, info.ptr)  return v end,
		SetUiInt = function(self, ui, name, value) local v = CLIB.purple_account_set_ui_int(self.ptr, ui, name, value)  return v end,
		GetUiInt = function(self, ui, name, default_value) local v = CLIB.purple_account_get_ui_int(self.ptr, ui, name, default_value)  return v end,
		GetUiBool = function(self, ui, name, default_value) local v = CLIB.purple_account_get_ui_bool(self.ptr, ui, name, default_value)  return v end,
		AddBuddy = function(self, buddy) local v = CLIB.purple_account_add_buddy(self.ptr, buddy.ptr)  return v end,
		GetUiString = function(self, ui, name, default_value) local v = CLIB.purple_account_get_ui_string(self.ptr, ui, name, default_value) v = chars_to_string(v) return v end,
		SetRegisterCallback = function(self, cb, user_data) local v = CLIB.purple_account_set_register_callback(self.ptr, cb, user_data)  return v end,
		SetConnection = function(self, gc) local v = CLIB.purple_account_set_connection(self.ptr, gc.ptr)  return v end,
		ChangePassword = function(self, orig_pw, new_pw) local v = CLIB.purple_account_change_password(self.ptr, orig_pw, new_pw)  return v end,
		GetEnabled = function(self, ui) local v = CLIB.purple_account_get_enabled(self.ptr, ui)  return v end,
		GetBuddyIconPath = function(self) local v = CLIB.purple_account_get_buddy_icon_path(self.ptr) v = chars_to_string(v) return v end,
		SetBool = function(self, name, value) local v = CLIB.purple_account_set_bool(self.ptr, name, value)  return v end,
		IsStatusActive = function(self, status_id) local v = CLIB.purple_account_is_status_active(self.ptr, status_id)  return v end,
		GetPassword = function(self) local v = CLIB.purple_account_get_password(self.ptr) v = chars_to_string(v) return v end,
		SetEnabled = function(self, ui, value) local v = CLIB.purple_account_set_enabled(self.ptr, ui, value)  return v end,
		SetUserInfo = function(self, user_info) local v = CLIB.purple_account_set_user_info(self.ptr, user_info)  return v end,
		NotifyAdded = function(self, remote_user, id, alias, message) local v = CLIB.purple_account_notify_added(self.ptr, remote_user, id, alias, message)  return v end,
		GetPrivacyType = function(self) local v = CLIB.purple_account_get_privacy_type(self.ptr)  return v end,
		Register = function(self) local v = CLIB.purple_account_register(self.ptr)  return v end,
		SetStatusList = function(self, status_id, active, attrs) local v = CLIB.purple_account_set_status_list(self.ptr, status_id, active, table_to_glist(attrs))  return v end,
		Destroy = function(self) local v = CLIB.purple_account_destroy(self.ptr)  return v end,
		GetStatusTypes = function(selfcast_type) local v = CLIB.purple_account_get_status_types(self.ptr) v = glist_to_table(v, cast_type) return v end,
		SetUiString = function(self, ui, name, value) local v = CLIB.purple_account_set_ui_string(self.ptr, ui, name, value)  return v end,
		SetStatus = function(self, status_id, active, _4) local v = CLIB.purple_account_set_status(self.ptr, status_id, active, _4)  return v end,
		GetCurrentError = function(self) local v = CLIB.purple_account_get_current_error(self.ptr)  return v end,
		IsConnected = function(self) local v = CLIB.purple_account_is_connected(self.ptr)  return v end,
		GetConnection = function(self) local v = CLIB.purple_account_get_connection(self.ptr) v = wrap_pointer(v, "Connection") return v end,
		RequestChangePassword = function(self) local v = CLIB.purple_account_request_change_password(self.ptr)  return v end,
		SetUsername = function(self, username) local v = CLIB.purple_account_set_username(self.ptr, username)  return v end,
		DestroyLog = function(self) local v = CLIB.purple_account_destroy_log(self.ptr)  return v end,
		SetProtocolId = function(self, protocol_id) local v = CLIB.purple_account_set_protocol_id(self.ptr, protocol_id)  return v end,
		ClearSettings = function(self) local v = CLIB.purple_account_clear_settings(self.ptr)  return v end,
		GetUsername = function(self) local v = CLIB.purple_account_get_username(self.ptr) v = chars_to_string(v) return v end,
		IsConnecting = function(self) local v = CLIB.purple_account_is_connecting(self.ptr)  return v end,
		SetBuddyIconPath = function(self, path) local v = CLIB.purple_account_set_buddy_icon_path(self.ptr, path)  return v end,
		Disconnect = function(self) local v = CLIB.purple_account_disconnect(self.ptr)  return v end,
		RemoveGroup = function(self, group) local v = CLIB.purple_account_remove_group(self.ptr, group.ptr)  return v end,
		RequestPassword = function(self, ok_cb, cancel_cb, user_data) local v = CLIB.purple_account_request_password(self.ptr, ok_cb, cancel_cb, user_data)  return v end,
		GetStatus = function(self, status_id) local v = CLIB.purple_account_get_status(self.ptr, status_id) v = wrap_pointer(v, "Status") return v end,
		GetBool = function(self, name, default_value) local v = CLIB.purple_account_get_bool(self.ptr, name, default_value)  return v end,
		GetNameForDisplay = function(self) local v = CLIB.purple_account_get_name_for_display(self.ptr) v = chars_to_string(v) return v end,
		SetPassword = function(self, password) local v = CLIB.purple_account_set_password(self.ptr, password)  return v end,
		RemoveBuddies = function(self, buddies, groups) local v = CLIB.purple_account_remove_buddies(self.ptr, table_to_glist(buddies), table_to_glist(groups))  return v end,
		GetProtocolName = function(self) local v = CLIB.purple_account_get_protocol_name(self.ptr) v = chars_to_string(v) return v end,
		GetCheckMail = function(self) local v = CLIB.purple_account_get_check_mail(self.ptr)  return v end,
		RemoveBuddy = function(self, buddy, group) local v = CLIB.purple_account_remove_buddy(self.ptr, buddy.ptr, group.ptr)  return v end,
		GetPublicAlias = function(self, success_cb, failure_cb) local v = CLIB.purple_account_get_public_alias(self.ptr, success_cb, failure_cb)  return v end,
		AddBuddies = function(self, buddies) local v = CLIB.purple_account_add_buddies(self.ptr, table_to_glist(buddies))  return v end,
		GetAlias = function(self) local v = CLIB.purple_account_get_alias(self.ptr) v = chars_to_string(v) return v end,
		SetRememberPassword = function(self, value) local v = CLIB.purple_account_set_remember_password(self.ptr, value)  return v end,
		SetPrivacyType = function(self, privacy_type) local v = CLIB.purple_account_set_privacy_type(self.ptr, privacy_type)  return v end,
		SetString = function(self, name, value) local v = CLIB.purple_account_set_string(self.ptr, name, value)  return v end,
		SetCheckMail = function(self, value) local v = CLIB.purple_account_set_check_mail(self.ptr, value)  return v end,
		GetPresence = function(self) local v = CLIB.purple_account_get_presence(self.ptr) v = wrap_pointer(v, "Presence") return v end,
		GetStatusTypeWithPrimitive = function(self, primitive) local v = CLIB.purple_account_get_status_type_with_primitive(self.ptr, primitive) v = wrap_pointer(v, "StatusType") return v end,
		Unregister = function(self, cb, user_data) local v = CLIB.purple_account_unregister(self.ptr, cb, user_data)  return v end,
		SetSilenceSuppression = function(self, value) local v = CLIB.purple_account_set_silence_suppression(self.ptr, value)  return v end,
		GetRememberPassword = function(self) local v = CLIB.purple_account_get_remember_password(self.ptr)  return v end,
		SetInt = function(self, name, value) local v = CLIB.purple_account_set_int(self.ptr, name, value)  return v end,
		GetInt = function(self, name, default_value) local v = CLIB.purple_account_get_int(self.ptr, name, default_value)  return v end,
		SetUiBool = function(self, ui, name, value) local v = CLIB.purple_account_set_ui_bool(self.ptr, ui, name, value)  return v end,
		GetActiveStatus = function(self) local v = CLIB.purple_account_get_active_status(self.ptr) v = wrap_pointer(v, "Status") return v end,
		RemoveSetting = function(self, setting) local v = CLIB.purple_account_remove_setting(self.ptr, setting)  return v end,
		AddBuddyWithInvite = function(self, buddy, message) local v = CLIB.purple_account_add_buddy_with_invite(self.ptr, buddy.ptr, message)  return v end,
		GetUserInfo = function(self) local v = CLIB.purple_account_get_user_info(self.ptr) v = chars_to_string(v) return v end,
		SetStatusTypes = function(self, status_types) local v = CLIB.purple_account_set_status_types(self.ptr, table_to_glist(status_types))  return v end,
		SetAlias = function(self, alias) local v = CLIB.purple_account_set_alias(self.ptr, alias)  return v end,
		AddBuddiesWithInvite = function(self, buddies, message) local v = CLIB.purple_account_add_buddies_with_invite(self.ptr, table_to_glist(buddies), message)  return v end,
		SupportsOfflineMessage = function(self, buddy) local v = CLIB.purple_account_supports_offline_message(self.ptr, buddy.ptr)  return v end,
		GetProxyInfo = function(self) local v = CLIB.purple_account_get_proxy_info(self.ptr) v = wrap_pointer(v, "ProxyInfo") return v end,
		GetSilenceSuppression = function(self) local v = CLIB.purple_account_get_silence_suppression(self.ptr)  return v end,
		SetPublicAlias = function(self, alias, success_cb, failure_cb) local v = CLIB.purple_account_set_public_alias(self.ptr, alias, success_cb, failure_cb)  return v end,
		GetString = function(self, name, default_value) local v = CLIB.purple_account_get_string(self.ptr, name, default_value) v = chars_to_string(v) return v end,
		GetLog = function(self, create) local v = CLIB.purple_account_get_log(self.ptr, create) v = wrap_pointer(v, "Log") return v end,
		Connect = function(self) local v = CLIB.purple_account_connect(self.ptr)  return v end,
		RequestChangeUserInfo = function(self) local v = CLIB.purple_account_request_change_user_info(self.ptr)  return v end,
		GetProtocolId = function(self) local v = CLIB.purple_account_get_protocol_id(self.ptr) v = chars_to_string(v) return v end,
		ClearCurrentError = function(self) local v = CLIB.purple_account_clear_current_error(self.ptr)  return v end,
		GetStatusType = function(self, id) local v = CLIB.purple_account_get_status_type(self.ptr, id) v = wrap_pointer(v, "StatusType") return v end,
		RequestAuthorization = function(self, remote_user, id, alias, message, on_list, auth_cb, deny_cb, user_data) local v = CLIB.purple_account_request_authorization(self.ptr, remote_user, id, alias, message, on_list, auth_cb, deny_cb, user_data)  return v end,
		RequestCloseWithAccount = function(self) local v = CLIB.purple_account_request_close_with_account(self.ptr)  return v end,
		IsDisconnected = function(self) local v = CLIB.purple_account_is_disconnected(self.ptr)  return v end,
		RequestAdd = function(self, remote_user, id, alias, message) local v = CLIB.purple_account_request_add(self.ptr, remote_user, id, alias, message)  return v end,
	}
	META.__index = META
	metatables.Account = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleConversation"),
		GetGc = function(self) local v = CLIB.purple_conversation_get_gc(self.ptr) v = wrap_pointer(v, "Connection") return v end,
		GetType = function(self) local v = CLIB.purple_conversation_get_type(self.ptr)  return v end,
		SendConfirm = function(self, message) local v = CLIB.purple_conv_send_confirm(self.ptr, message)  return v end,
		SetAccount = function(self, account) local v = CLIB.purple_conversation_set_account(self.ptr, account.ptr)  return v end,
		DoCommand = function(self, cmdline, markup, error) local v = CLIB.purple_conversation_do_command(self.ptr, cmdline, markup, error)  return v end,
		Update = function(self, type) local v = CLIB.purple_conversation_update(self.ptr, type)  return v end,
		GetChatData = function(self) local v = CLIB.purple_conversation_get_chat_data(self.ptr) v = wrap_pointer(v, "ConvChat") return v end,
		GetMessageHistory = function(selfcast_type) local v = CLIB.purple_conversation_get_message_history(self.ptr) v = glist_to_table(v, cast_type) return v end,
		CustomSmileyWrite = function(self, smile, data, size) local v = CLIB.purple_conv_custom_smiley_write(self.ptr, smile, data, size)  return v end,
		SetUiOps = function(self, ops) local v = CLIB.purple_conversation_set_ui_ops(self.ptr, ops)  return v end,
		GetName = function(self) local v = CLIB.purple_conversation_get_name(self.ptr) v = chars_to_string(v) return v end,
		ClearMessageHistory = function(self) local v = CLIB.purple_conversation_clear_message_history(self.ptr)  return v end,
		GetFeatures = function(self) local v = CLIB.purple_conversation_get_features(self.ptr)  return v end,
		AutosetTitle = function(self) local v = CLIB.purple_conversation_autoset_title(self.ptr)  return v end,
		HasFocus = function(self) local v = CLIB.purple_conversation_has_focus(self.ptr)  return v end,
		CloseLogs = function(self) local v = CLIB.purple_conversation_close_logs(self.ptr)  return v end,
		CustomSmileyAdd = function(self, smile, cksum_type, chksum, remote) local v = CLIB.purple_conv_custom_smiley_add(self.ptr, smile, cksum_type, chksum, remote)  return v end,
		GetUiOps = function(self) local v = CLIB.purple_conversation_get_ui_ops(self.ptr)  return v end,
		GetExtendedMenu = function(selfcast_type) local v = CLIB.purple_conversation_get_extended_menu(self.ptr) v = glist_to_table(v, cast_type) return v end,
		GetTitle = function(self) local v = CLIB.purple_conversation_get_title(self.ptr) v = chars_to_string(v) return v end,
		Write = function(self, who, message, flags, mtime) local v = CLIB.purple_conversation_write(self.ptr, who, message, flags, mtime)  return v end,
		IsLogging = function(self) local v = CLIB.purple_conversation_is_logging(self.ptr)  return v end,
		GetImData = function(self) local v = CLIB.purple_conversation_get_im_data(self.ptr) v = wrap_pointer(v, "ConvIm") return v end,
		GetData = function(self, key) local v = CLIB.purple_conversation_get_data(self.ptr, key)  return v end,
		SetFeatures = function(self, features) local v = CLIB.purple_conversation_set_features(self.ptr, features)  return v end,
		Present = function(self) local v = CLIB.purple_conversation_present(self.ptr)  return v end,
		SetName = function(self, name) local v = CLIB.purple_conversation_set_name(self.ptr, name)  return v end,
		SetLogging = function(self, log) local v = CLIB.purple_conversation_set_logging(self.ptr, log)  return v end,
		Destroy = function(self) local v = CLIB.purple_conversation_destroy(self.ptr)  return v end,
		GetAccount = function(self) local v = CLIB.purple_conversation_get_account(self.ptr) v = wrap_pointer(v, "Account") return v end,
		CustomSmileyClose = function(self, smile) local v = CLIB.purple_conv_custom_smiley_close(self.ptr, smile)  return v end,
		SetData = function(self, key, data) local v = CLIB.purple_conversation_set_data(self.ptr, key, data)  return v end,
		SetTitle = function(self, title) local v = CLIB.purple_conversation_set_title(self.ptr, title)  return v end,
	}
	META.__index = META
	metatables.Conversation = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleRoomlistField"),
		GetLabel = function(self) local v = CLIB.purple_roomlist_field_get_label(self.ptr) v = chars_to_string(v) return v end,
		GetHidden = function(self) local v = CLIB.purple_roomlist_field_get_hidden(self.ptr)  return v end,
		GetType = function(self) local v = CLIB.purple_roomlist_field_get_type(self.ptr)  return v end,
	}
	META.__index = META
	metatables.RoomlistField = META
end
do
	local META = {
		ctype = ffi.typeof("struct PurpleRequestFieldGroup"),
		GetFields = function(selfcast_type) local v = CLIB.purple_request_field_group_get_fields(self.ptr) v = glist_to_table(v, cast_type) return v end,
		AddField = function(self, field) local v = CLIB.purple_request_field_group_add_field(self.ptr, field.ptr)  return v end,
		GetTitle = function(self) local v = CLIB.purple_request_field_group_get_title(self.ptr) v = chars_to_string(v) return v end,
		Destroy = function(self) local v = CLIB.purple_request_field_group_destroy(self.ptr)  return v end,
	}
	META.__index = META
	metatables.RequestFieldGroup = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleStoredImage"),
		GetSize = function(self) local v = CLIB.purple_imgstore_get_size(self.ptr)  return v end,
		GetExtension = function(self) local v = CLIB.purple_imgstore_get_extension(self.ptr) v = chars_to_string(v) return v end,
		Ref = function(self) local v = CLIB.purple_imgstore_ref(self.ptr) v = wrap_pointer(v, "StoredImage") return v end,
		GetFilename = function(self) local v = CLIB.purple_imgstore_get_filename(self.ptr) v = chars_to_string(v) return v end,
		GetData = function(self) local v = CLIB.purple_imgstore_get_data(self.ptr)  return v end,
		Unref = function(self) local v = CLIB.purple_imgstore_unref(self.ptr) v = wrap_pointer(v, "StoredImage") return v end,
	}
	META.__index = META
	metatables.StoredImage = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleMedia"),
		GetPrplData = function(self) local v = CLIB.purple_media_get_prpl_data(self.ptr)  return v end,
		SetOutputWindow = function(self, session_id, participant, window_id) local v = CLIB.purple_media_set_output_window(self.ptr, session_id, participant, window_id)  return v end,
		GetAvailableParams = function(self) local v = CLIB.purple_media_get_available_params(self.ptr)  return v end,
		SetPrplData = function(self, prpl_data) local v = CLIB.purple_media_set_prpl_data(self.ptr, prpl_data)  return v end,
		SetSendCodec = function(self, sess_id, codec) local v = CLIB.purple_media_set_send_codec(self.ptr, sess_id, codec.ptr)  return v end,
		CandidatesPrepared = function(self, session_id, participant) local v = CLIB.purple_media_candidates_prepared(self.ptr, session_id, participant)  return v end,
		AddStream = function(self, sess_id, who, type, initiator, transmitter, num_params, params) local v = CLIB.purple_media_add_stream(self.ptr, sess_id, who, type, initiator, transmitter, num_params, params)  return v end,
		GetActiveRemoteCandidates = function(self, sess_id, participant, cast_type) local v = CLIB.purple_media_get_active_remote_candidates(self.ptr, sess_id, participant) v = glist_to_table(v, cast_type) return v end,
		GetLocalCandidates = function(self, sess_id, participant, cast_type) local v = CLIB.purple_media_get_local_candidates(self.ptr, sess_id, participant) v = glist_to_table(v, cast_type) return v end,
		GetSessionIds = function(selfcast_type) local v = CLIB.purple_media_get_session_ids(self.ptr) v = glist_to_table(v, cast_type) return v end,
		Accepted = function(self, sess_id, participant) local v = CLIB.purple_media_accepted(self.ptr, sess_id, participant)  return v end,
		CodecsReady = function(self, sess_id) local v = CLIB.purple_media_codecs_ready(self.ptr, sess_id)  return v end,
		SetOutputVolume = function(self, session_id, participant, level) local v = CLIB.purple_media_set_output_volume(self.ptr, session_id, participant, level)  return v end,
		ParamIsSupported = function(self, param) local v = CLIB.purple_media_param_is_supported(self.ptr, param)  return v end,
		AddRemoteCandidates = function(self, sess_id, participant, remote_candidates) local v = CLIB.purple_media_add_remote_candidates(self.ptr, sess_id, participant, table_to_glist(remote_candidates))  return v end,
		RemoveOutputWindows = function(self) local v = CLIB.purple_media_remove_output_windows(self.ptr)  return v end,
		GetSessionType = function(self, sess_id) local v = CLIB.purple_media_get_session_type(self.ptr, sess_id)  return v end,
		GetCodecs = function(self, sess_id, cast_type) local v = CLIB.purple_media_get_codecs(self.ptr, sess_id) v = glist_to_table(v, cast_type) return v end,
		Error = function(self, error, _3) local v = CLIB.purple_media_error(self.ptr, error, _3)  return v end,
		SetParams = function(self, num_params, params) local v = CLIB.purple_media_set_params(self.ptr, num_params, params)  return v end,
		GetAccount = function(self) local v = CLIB.purple_media_get_account(self.ptr) v = wrap_pointer(v, "Account") return v end,
		SetInputVolume = function(self, session_id, level) local v = CLIB.purple_media_set_input_volume(self.ptr, session_id, level)  return v end,
		SetRemoteCodecs = function(self, sess_id, participant, codecs) local v = CLIB.purple_media_set_remote_codecs(self.ptr, sess_id, participant, table_to_glist(codecs))  return v end,
		GetActiveLocalCandidates = function(self, sess_id, participant, cast_type) local v = CLIB.purple_media_get_active_local_candidates(self.ptr, sess_id, participant) v = glist_to_table(v, cast_type) return v end,
		StreamInfo = function(self, type, session_id, participant, local_) local v = CLIB.purple_media_stream_info(self.ptr, type, session_id, participant, local_)  return v end,
		GetManager = function(self) local v = CLIB.purple_media_get_manager(self.ptr) v = wrap_pointer(v, "MediaManager") return v end,
		End = function(self, session_id, participant) local v = CLIB.purple_media_end(self.ptr, session_id, participant)  return v end,
		IsInitiator = function(self, sess_id, participant) local v = CLIB.purple_media_is_initiator(self.ptr, sess_id, participant)  return v end,
	}
	META.__index = META
	metatables.Media = META
end
do
	local META = {
		ctype = ffi.typeof("struct PurpleAccountOption"),
		GetSetting = function(self) local v = CLIB.purple_account_option_get_setting(self.ptr) v = chars_to_string(v) return v end,
		GetDefaultString = function(self) local v = CLIB.purple_account_option_get_default_string(self.ptr) v = chars_to_string(v) return v end,
		GetDefaultBool = function(self) local v = CLIB.purple_account_option_get_default_bool(self.ptr)  return v end,
		Destroy = function(self) local v = CLIB.purple_account_option_destroy(self.ptr)  return v end,
		AddListItem = function(self, key, value) local v = CLIB.purple_account_option_add_list_item(self.ptr, key, value)  return v end,
		SetDefaultBool = function(self, value) local v = CLIB.purple_account_option_set_default_bool(self.ptr, value)  return v end,
		GetList = function(selfcast_type) local v = CLIB.purple_account_option_get_list(self.ptr) v = glist_to_table(v, cast_type) return v end,
		GetType = function(self) local v = CLIB.purple_account_option_get_type(self.ptr)  return v end,
		GetMasked = function(self) local v = CLIB.purple_account_option_get_masked(self.ptr)  return v end,
		SetMasked = function(self, masked) local v = CLIB.purple_account_option_set_masked(self.ptr, masked)  return v end,
		SetDefaultInt = function(self, value) local v = CLIB.purple_account_option_set_default_int(self.ptr, value)  return v end,
		GetDefaultInt = function(self) local v = CLIB.purple_account_option_get_default_int(self.ptr)  return v end,
		GetText = function(self) local v = CLIB.purple_account_option_get_text(self.ptr) v = chars_to_string(v) return v end,
		GetDefaultListValue = function(self) local v = CLIB.purple_account_option_get_default_list_value(self.ptr) v = chars_to_string(v) return v end,
		SetList = function(self, values) local v = CLIB.purple_account_option_set_list(self.ptr, table_to_glist(values))  return v end,
		SetDefaultString = function(self, value) local v = CLIB.purple_account_option_set_default_string(self.ptr, value)  return v end,
	}
	META.__index = META
	metatables.AccountOption = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleNotifyUserInfo"),
		AddSectionHeader = function(self, label) local v = CLIB.purple_notify_user_info_add_section_header(self.ptr, label)  return v end,
		RemoveEntry = function(self, user_info_entry) local v = CLIB.purple_notify_user_info_remove_entry(self.ptr, user_info_entry.ptr)  return v end,
		GetTextWithNewline = function(self, newline) local v = CLIB.purple_notify_user_info_get_text_with_newline(self.ptr, newline) v = chars_to_string(v) return v end,
		PrependSectionBreak = function(self) local v = CLIB.purple_notify_user_info_prepend_section_break(self.ptr)  return v end,
		AddPairPlaintext = function(self, label, value) local v = CLIB.purple_notify_user_info_add_pair_plaintext(self.ptr, label, value)  return v end,
		Destroy = function(self) local v = CLIB.purple_notify_user_info_destroy(self.ptr)  return v end,
		RemoveLastItem = function(self) local v = CLIB.purple_notify_user_info_remove_last_item(self.ptr)  return v end,
		AddPair = function(self, label, value) local v = CLIB.purple_notify_user_info_add_pair(self.ptr, label, value)  return v end,
		GetEntries = function(selfcast_type) local v = CLIB.purple_notify_user_info_get_entries(self.ptr) v = glist_to_table(v, cast_type) return v end,
		PrependSectionHeader = function(self, label) local v = CLIB.purple_notify_user_info_prepend_section_header(self.ptr, label)  return v end,
		AddSectionBreak = function(self) local v = CLIB.purple_notify_user_info_add_section_break(self.ptr)  return v end,
		PrependPair = function(self, label, value) local v = CLIB.purple_notify_user_info_prepend_pair(self.ptr, label, value)  return v end,
	}
	META.__index = META
	metatables.NotifyUserInfo = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleLog"),
		CommonWriter = function(self, ext) local v = CLIB.purple_log_common_writer(self.ptr, ext)  return v end,
		CommonSizer = function(self) local v = CLIB.purple_log_common_sizer(self.ptr)  return v end,
		Delete = function(self) local v = CLIB.purple_log_delete(self.ptr)  return v end,
		GetSize = function(self) local v = CLIB.purple_log_get_size(self.ptr)  return v end,
		Write = function(self, type, from, time, message) local v = CLIB.purple_log_write(self.ptr, type, from, time, message)  return v end,
		Read = function(self, flags) local v = CLIB.purple_log_read(self.ptr, flags) v = chars_to_string(v) return v end,
		CommonDeleter = function(self) local v = CLIB.purple_log_common_deleter(self.ptr)  return v end,
		CommonIsDeletable = function(self) local v = CLIB.purple_log_common_is_deletable(self.ptr)  return v end,
		IsDeletable = function(self) local v = CLIB.purple_log_is_deletable(self.ptr)  return v end,
		Free = function(self) local v = CLIB.purple_log_free(self.ptr)  return v end,
	}
	META.__index = META
	metatables.Log = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleSslConnection"),
		InputAdd = function(self, func, data) local v = CLIB.purple_ssl_input_add(self.ptr, func, data)  return v end,
		Write = function(self, buffer, len) local v = CLIB.purple_ssl_write(self.ptr, buffer, len)  return v end,
		Read = function(self, buffer, len) local v = CLIB.purple_ssl_read(self.ptr, buffer, len)  return v end,
		Close = function(self) local v = CLIB.purple_ssl_close(self.ptr)  return v end,
		GetPeerCertificates = function(selfcast_type) local v = CLIB.purple_ssl_get_peer_certificates(self.ptr) v = glist_to_table(v, cast_type) return v end,
	}
	META.__index = META
	metatables.SslConnection = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleWhiteboard"),
		DrawPoint = function(self, x, y, color, size) local v = CLIB.purple_whiteboard_draw_point(self.ptr, x, y, color, size)  return v end,
		SetDimensions = function(self, width, height) local v = CLIB.purple_whiteboard_set_dimensions(self.ptr, width, height)  return v end,
		SendDrawList = function(self, list) local v = CLIB.purple_whiteboard_send_draw_list(self.ptr, table_to_glist(list))  return v end,
		Destroy = function(self) local v = CLIB.purple_whiteboard_destroy(self.ptr)  return v end,
		GetDimensions = function(self, width, height) local v = CLIB.purple_whiteboard_get_dimensions(self.ptr, width, height)  return v end,
		SetPrplOps = function(self, ops) local v = CLIB.purple_whiteboard_set_prpl_ops(self.ptr, ops)  return v end,
		Start = function(self) local v = CLIB.purple_whiteboard_start(self.ptr)  return v end,
		SendClear = function(self) local v = CLIB.purple_whiteboard_send_clear(self.ptr)  return v end,
		SendBrush = function(self, size, color) local v = CLIB.purple_whiteboard_send_brush(self.ptr, size, color)  return v end,
		SetBrush = function(self, size, color) local v = CLIB.purple_whiteboard_set_brush(self.ptr, size, color)  return v end,
		GetBrush = function(self, size, color) local v = CLIB.purple_whiteboard_get_brush(self.ptr, size, color)  return v end,
		DrawLine = function(self, x1, y1, x2, y2, color, size) local v = CLIB.purple_whiteboard_draw_line(self.ptr, x1, y1, x2, y2, color, size)  return v end,
		Clear = function(self) local v = CLIB.purple_whiteboard_clear(self.ptr)  return v end,
	}
	META.__index = META
	metatables.Whiteboard = META
end
do
	local META = {
		ctype = ffi.typeof("struct PurpleAccountUserSplit"),
		GetSeparator = function(self) local v = CLIB.purple_account_user_split_get_separator(self.ptr)  return v end,
		SetReverse = function(self, reverse) local v = CLIB.purple_account_user_split_set_reverse(self.ptr, reverse)  return v end,
		GetText = function(self) local v = CLIB.purple_account_user_split_get_text(self.ptr) v = chars_to_string(v) return v end,
		GetDefaultValue = function(self) local v = CLIB.purple_account_user_split_get_default_value(self.ptr) v = chars_to_string(v) return v end,
		Destroy = function(self) local v = CLIB.purple_account_user_split_destroy(self.ptr)  return v end,
		GetReverse = function(self) local v = CLIB.purple_account_user_split_get_reverse(self.ptr)  return v end,
	}
	META.__index = META
	metatables.AccountUserSplit = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleSavedStatus"),
		GetTitle = function(self) local v = CLIB.purple_savedstatus_get_title(self.ptr) v = chars_to_string(v) return v end,
		GetMessage = function(self) local v = CLIB.purple_savedstatus_get_message(self.ptr) v = chars_to_string(v) return v end,
		DeleteByStatus = function(self) local v = CLIB.purple_savedstatus_delete_by_status(self.ptr)  return v end,
		SetSubstatus = function(self, account, type, message) local v = CLIB.purple_savedstatus_set_substatus(self.ptr, account.ptr, type.ptr, message)  return v end,
		GetCreationTime = function(self) local v = CLIB.purple_savedstatus_get_creation_time(self.ptr)  return v end,
		HasSubstatuses = function(self) local v = CLIB.purple_savedstatus_has_substatuses(self.ptr)  return v end,
		SetType = function(self, type) local v = CLIB.purple_savedstatus_set_type(self.ptr, type)  return v end,
		GetSubstatus = function(self, account) local v = CLIB.purple_savedstatus_get_substatus(self.ptr, account.ptr)  return v end,
		GetType = function(self) local v = CLIB.purple_savedstatus_get_type(self.ptr)  return v end,
		UnsetSubstatus = function(self, account) local v = CLIB.purple_savedstatus_unset_substatus(self.ptr, account.ptr)  return v end,
		ActivateForAccount = function(self, account) local v = CLIB.purple_savedstatus_activate_for_account(self.ptr, account.ptr)  return v end,
		SetTitle = function(self, title) local v = CLIB.purple_savedstatus_set_title(self.ptr, title)  return v end,
		SetMessage = function(self, message) local v = CLIB.purple_savedstatus_set_message(self.ptr, message)  return v end,
		Activate = function(self) local v = CLIB.purple_savedstatus_activate(self.ptr)  return v end,
		IsTransient = function(self) local v = CLIB.purple_savedstatus_is_transient(self.ptr)  return v end,
	}
	META.__index = META
	metatables.SavedStatus = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleSmiley"),
		GetStoredImage = function(self) local v = CLIB.purple_smiley_get_stored_image(self.ptr) v = wrap_pointer(v, "StoredImage") return v end,
		SetShortcut = function(self, shortcut) local v = CLIB.purple_smiley_set_shortcut(self.ptr, shortcut)  return v end,
		GetData = function(self, len) local v = CLIB.purple_smiley_get_data(self.ptr, len)  return v end,
		Delete = function(self) local v = CLIB.purple_smiley_delete(self.ptr)  return v end,
		GetFullPath = function(self) local v = CLIB.purple_smiley_get_full_path(self.ptr) v = chars_to_string(v) return v end,
		GetChecksum = function(self) local v = CLIB.purple_smiley_get_checksum(self.ptr) v = chars_to_string(v) return v end,
		GetExtension = function(self) local v = CLIB.purple_smiley_get_extension(self.ptr) v = chars_to_string(v) return v end,
		GetShortcut = function(self) local v = CLIB.purple_smiley_get_shortcut(self.ptr) v = chars_to_string(v) return v end,
		SetData = function(self, smiley_data, smiley_data_len) local v = CLIB.purple_smiley_set_data(self.ptr, smiley_data, smiley_data_len)  return v end,
	}
	META.__index = META
	metatables.Smiley = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleStatus"),
		IsExclusive = function(self) local v = CLIB.purple_status_is_exclusive(self.ptr)  return v end,
		GetId = function(self) local v = CLIB.purple_status_get_id(self.ptr) v = chars_to_string(v) return v end,
		GetPresence = function(self) local v = CLIB.purple_status_get_presence(self.ptr) v = wrap_pointer(v, "Presence") return v end,
		SetActiveWithAttrsList = function(self, active, attrs) local v = CLIB.purple_status_set_active_with_attrs_list(self.ptr, active, table_to_glist(attrs))  return v end,
		IsAvailable = function(self) local v = CLIB.purple_status_is_available(self.ptr)  return v end,
		GetType = function(self) local v = CLIB.purple_status_get_type(self.ptr) v = wrap_pointer(v, "StatusType") return v end,
		GetAttrInt = function(self, id) local v = CLIB.purple_status_get_attr_int(self.ptr, id)  return v end,
		SetAttrBoolean = function(self, id, value) local v = CLIB.purple_status_set_attr_boolean(self.ptr, id, value)  return v end,
		IsOnline = function(self) local v = CLIB.purple_status_is_online(self.ptr)  return v end,
		SetActiveWithAttrs = function(self, active, args) local v = CLIB.purple_status_set_active_with_attrs(self.ptr, active, args)  return v end,
		GetAttrValue = function(self, id) local v = CLIB.purple_status_get_attr_value(self.ptr, id) v = wrap_pointer(v, "Value") return v end,
		GetAttrBoolean = function(self, id) local v = CLIB.purple_status_get_attr_boolean(self.ptr, id)  return v end,
		IsIndependent = function(self) local v = CLIB.purple_status_is_independent(self.ptr)  return v end,
		SetAttrString = function(self, id, value) local v = CLIB.purple_status_set_attr_string(self.ptr, id, value)  return v end,
		Destroy = function(self) local v = CLIB.purple_status_destroy(self.ptr)  return v end,
		GetName = function(self) local v = CLIB.purple_status_get_name(self.ptr) v = chars_to_string(v) return v end,
		IsActive = function(self) local v = CLIB.purple_status_is_active(self.ptr)  return v end,
		SetActive = function(self, active) local v = CLIB.purple_status_set_active(self.ptr, active)  return v end,
		GetAttrString = function(self, id) local v = CLIB.purple_status_get_attr_string(self.ptr, id) v = chars_to_string(v) return v end,
		SetAttrInt = function(self, id, value) local v = CLIB.purple_status_set_attr_int(self.ptr, id, value)  return v end,
		Compare = function(self, status2) local v = CLIB.purple_status_compare(self.ptr, status2.ptr)  return v end,
	}
	META.__index = META
	metatables.Status = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurplePluginPrefFrame"),
		Add = function(self, pref) local v = CLIB.purple_plugin_pref_frame_add(self.ptr, pref.ptr)  return v end,
		GetPrefs = function(selfcast_type) local v = CLIB.purple_plugin_pref_frame_get_prefs(self.ptr) v = glist_to_table(v, cast_type) return v end,
		Destroy = function(self) local v = CLIB.purple_plugin_pref_frame_destroy(self.ptr)  return v end,
	}
	META.__index = META
	metatables.PluginPrefFrame = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurplePresence"),
		GetStatus = function(self, status_id) local v = CLIB.purple_presence_get_status(self.ptr, status_id) v = wrap_pointer(v, "Status") return v end,
		IsStatusActive = function(self, status_id) local v = CLIB.purple_presence_is_status_active(self.ptr, status_id)  return v end,
		IsAvailable = function(self) local v = CLIB.purple_presence_is_available(self.ptr)  return v end,
		GetContext = function(self) local v = CLIB.purple_presence_get_context(self.ptr)  return v end,
		SetLoginTime = function(self, login_time) local v = CLIB.purple_presence_set_login_time(self.ptr, login_time)  return v end,
		AddList = function(self, source_list) local v = CLIB.purple_presence_add_list(self.ptr, table_to_glist(source_list))  return v end,
		GetActiveStatus = function(self) local v = CLIB.purple_presence_get_active_status(self.ptr) v = wrap_pointer(v, "Status") return v end,
		SwitchStatus = function(self, status_id) local v = CLIB.purple_presence_switch_status(self.ptr, status_id)  return v end,
		IsOnline = function(self) local v = CLIB.purple_presence_is_online(self.ptr)  return v end,
		IsStatusPrimitiveActive = function(self, primitive) local v = CLIB.purple_presence_is_status_primitive_active(self.ptr, primitive)  return v end,
		GetIdleTime = function(self) local v = CLIB.purple_presence_get_idle_time(self.ptr)  return v end,
		GetConversation = function(self) local v = CLIB.purple_presence_get_conversation(self.ptr) v = wrap_pointer(v, "Conversation") return v end,
		GetBuddy = function(self) local v = CLIB.purple_presence_get_buddy(self.ptr) v = wrap_pointer(v, "Buddy") return v end,
		SetIdle = function(self, idle, idle_time) local v = CLIB.purple_presence_set_idle(self.ptr, idle, idle_time)  return v end,
		Destroy = function(self) local v = CLIB.purple_presence_destroy(self.ptr)  return v end,
		GetAccount = function(self) local v = CLIB.purple_presence_get_account(self.ptr) v = wrap_pointer(v, "Account") return v end,
		IsIdle = function(self) local v = CLIB.purple_presence_is_idle(self.ptr)  return v end,
		GetStatuses = function(selfcast_type) local v = CLIB.purple_presence_get_statuses(self.ptr) v = glist_to_table(v, cast_type) return v end,
		GetLoginTime = function(self) local v = CLIB.purple_presence_get_login_time(self.ptr)  return v end,
		SetStatusActive = function(self, status_id, active) local v = CLIB.purple_presence_set_status_active(self.ptr, status_id, active)  return v end,
		Compare = function(self, presence2) local v = CLIB.purple_presence_compare(self.ptr, presence2.ptr)  return v end,
		AddStatus = function(self, status) local v = CLIB.purple_presence_add_status(self.ptr, status.ptr)  return v end,
		GetChatUser = function(self) local v = CLIB.purple_presence_get_chat_user(self.ptr) v = chars_to_string(v) return v end,
	}
	META.__index = META
	metatables.Presence = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleMenuAction"),
		Free = function(self) local v = CLIB.purple_menu_action_free(self.ptr)  return v end,
	}
	META.__index = META
	metatables.MenuAction = META
end
do
	local META = {
		ctype = ffi.typeof("struct PurpleValue"),
		GetObject = function(self) local v = CLIB.purple_value_get_object(self.ptr)  return v end,
		GetBoolean = function(self) local v = CLIB.purple_value_get_boolean(self.ptr)  return v end,
		SetUshort = function(self, data) local v = CLIB.purple_value_set_ushort(self.ptr, data)  return v end,
		GetSpecificType = function(self) local v = CLIB.purple_value_get_specific_type(self.ptr) v = chars_to_string(v) return v end,
		GetType = function(self) local v = CLIB.purple_value_get_type(self.ptr)  return v end,
		GetInt64 = function(self) local v = CLIB.purple_value_get_int64(self.ptr)  return v end,
		SetLong = function(self, data) local v = CLIB.purple_value_set_long(self.ptr, data)  return v end,
		SetChar = function(self, data) local v = CLIB.purple_value_set_char(self.ptr, data)  return v end,
		SetPointer = function(self, data) local v = CLIB.purple_value_set_pointer(self.ptr, data)  return v end,
		Dup = function(self) local v = CLIB.purple_value_dup(self.ptr) v = wrap_pointer(v, "Value") return v end,
		SetString = function(self, data) local v = CLIB.purple_value_set_string(self.ptr, data)  return v end,
		GetEnum = function(self) local v = CLIB.purple_value_get_enum(self.ptr)  return v end,
		GetPointer = function(self) local v = CLIB.purple_value_get_pointer(self.ptr)  return v end,
		SetUint64 = function(self, data) local v = CLIB.purple_value_set_uint64(self.ptr, data)  return v end,
		SetBoxed = function(self, data) local v = CLIB.purple_value_set_boxed(self.ptr, data)  return v end,
		GetUshort = function(self) local v = CLIB.purple_value_get_ushort(self.ptr)  return v end,
		SetInt64 = function(self, data) local v = CLIB.purple_value_set_int64(self.ptr, data)  return v end,
		SetInt = function(self, data) local v = CLIB.purple_value_set_int(self.ptr, data)  return v end,
		SetUchar = function(self, data) local v = CLIB.purple_value_set_uchar(self.ptr, data)  return v end,
		GetString = function(self) local v = CLIB.purple_value_get_string(self.ptr) v = chars_to_string(v) return v end,
		GetShort = function(self) local v = CLIB.purple_value_get_short(self.ptr)  return v end,
		GetChar = function(self) local v = CLIB.purple_value_get_char(self.ptr)  return v end,
		IsOutgoing = function(self) local v = CLIB.purple_value_is_outgoing(self.ptr)  return v end,
		GetUint = function(self) local v = CLIB.purple_value_get_uint(self.ptr)  return v end,
		SetUint = function(self, data) local v = CLIB.purple_value_set_uint(self.ptr, data)  return v end,
		GetUint64 = function(self) local v = CLIB.purple_value_get_uint64(self.ptr)  return v end,
		SetBoolean = function(self, data) local v = CLIB.purple_value_set_boolean(self.ptr, data)  return v end,
		GetSubtype = function(self) local v = CLIB.purple_value_get_subtype(self.ptr)  return v end,
		GetBoxed = function(self) local v = CLIB.purple_value_get_boxed(self.ptr)  return v end,
		Destroy = function(self) local v = CLIB.purple_value_destroy(self.ptr)  return v end,
		GetUlong = function(self) local v = CLIB.purple_value_get_ulong(self.ptr)  return v end,
		SetEnum = function(self, data) local v = CLIB.purple_value_set_enum(self.ptr, data)  return v end,
		GetInt = function(self) local v = CLIB.purple_value_get_int(self.ptr)  return v end,
		SetShort = function(self, data) local v = CLIB.purple_value_set_short(self.ptr, data)  return v end,
		SetObject = function(self, data) local v = CLIB.purple_value_set_object(self.ptr, data)  return v end,
		GetUchar = function(self) local v = CLIB.purple_value_get_uchar(self.ptr)  return v end,
		GetLong = function(self) local v = CLIB.purple_value_get_long(self.ptr)  return v end,
		SetUlong = function(self, data) local v = CLIB.purple_value_set_ulong(self.ptr, data)  return v end,
	}
	META.__index = META
	metatables.Value = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleCircBuffer"),
		GetMaxRead = function(self) local v = CLIB.purple_circ_buffer_get_max_read(self.ptr)  return v end,
		Append = function(self, src, len) local v = CLIB.purple_circ_buffer_append(self.ptr, src, len)  return v end,
		MarkRead = function(self, len) local v = CLIB.purple_circ_buffer_mark_read(self.ptr, len)  return v end,
		Destroy = function(self) local v = CLIB.purple_circ_buffer_destroy(self.ptr)  return v end,
	}
	META.__index = META
	metatables.CircBuffer = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurplePlugin"),
		Disable = function(self) local v = CLIB.purple_plugin_disable(self.ptr)  return v end,
		Reload = function(self) local v = CLIB.purple_plugin_reload(self.ptr)  return v end,
		IpcCall = function(self, command, ok, _4) local v = CLIB.purple_plugin_ipc_call(self.ptr, command, ok, _4)  return v end,
		IpcRegister = function(self, command, func, marshal, ret_value, num_params, _7) local v = CLIB.purple_plugin_ipc_register(self.ptr, command, func, marshal, ret_value.ptr, num_params, _7)  return v end,
		GetVersion = function(self) local v = CLIB.purple_plugin_get_version(self.ptr) v = chars_to_string(v) return v end,
		IpcUnregisterAll = function(self) local v = CLIB.purple_plugin_ipc_unregister_all(self.ptr)  return v end,
		IsUnloadable = function(self) local v = CLIB.purple_plugin_is_unloadable(self.ptr)  return v end,
		Register = function(self) local v = CLIB.purple_plugin_register(self.ptr)  return v end,
		IpcGetParams = function(self, command, ret_value, num_params, params) local v = CLIB.purple_plugin_ipc_get_params(self.ptr, command, ret_value.ptr, num_params, params.ptr)  return v end,
		Load = function(self) local v = CLIB.purple_plugin_load(self.ptr)  return v end,
		Unload = function(self) local v = CLIB.purple_plugin_unload(self.ptr)  return v end,
		GetHomepage = function(self) local v = CLIB.purple_plugin_get_homepage(self.ptr) v = chars_to_string(v) return v end,
		Destroy = function(self) local v = CLIB.purple_plugin_destroy(self.ptr)  return v end,
		GetSummary = function(self) local v = CLIB.purple_plugin_get_summary(self.ptr) v = chars_to_string(v) return v end,
		GetName = function(self) local v = CLIB.purple_plugin_get_name(self.ptr) v = chars_to_string(v) return v end,
		GetAuthor = function(self) local v = CLIB.purple_plugin_get_author(self.ptr) v = chars_to_string(v) return v end,
		GetDescription = function(self) local v = CLIB.purple_plugin_get_description(self.ptr) v = chars_to_string(v) return v end,
		IpcUnregister = function(self, command) local v = CLIB.purple_plugin_ipc_unregister(self.ptr, command)  return v end,
		IsLoaded = function(self) local v = CLIB.purple_plugin_is_loaded(self.ptr)  return v end,
		GetId = function(self) local v = CLIB.purple_plugin_get_id(self.ptr) v = chars_to_string(v) return v end,
	}
	META.__index = META
	metatables.Plugin = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleTxtResponse"),
		Destroy = function(self) local v = CLIB.purple_txt_response_destroy(self.ptr)  return v end,
		GetContent = function(self) local v = CLIB.purple_txt_response_get_content(self.ptr) v = chars_to_string(v) return v end,
	}
	META.__index = META
	metatables.TxtResponse = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleStatusType"),
		AddAttrsVargs = function(self, args) local v = CLIB.purple_status_type_add_attrs_vargs(self.ptr, args)  return v end,
		AddAttr = function(self, id, name, value) local v = CLIB.purple_status_type_add_attr(self.ptr, id, name, value.ptr)  return v end,
		GetPrimitive = function(self) local v = CLIB.purple_status_type_get_primitive(self.ptr)  return v end,
		AddAttrs = function(self, id, name, value, _5) local v = CLIB.purple_status_type_add_attrs(self.ptr, id, name, value.ptr, _5)  return v end,
		IsAvailable = function(self) local v = CLIB.purple_status_type_is_available(self.ptr)  return v end,
		GetAttrs = function(selfcast_type) local v = CLIB.purple_status_type_get_attrs(self.ptr) v = glist_to_table(v, cast_type) return v end,
		SetPrimaryAttr = function(self, attr_id) local v = CLIB.purple_status_type_set_primary_attr(self.ptr, attr_id)  return v end,
		Destroy = function(self) local v = CLIB.purple_status_type_destroy(self.ptr)  return v end,
		GetAttr = function(self, id) local v = CLIB.purple_status_type_get_attr(self.ptr, id) v = wrap_pointer(v, "StatusAttr") return v end,
		GetName = function(self) local v = CLIB.purple_status_type_get_name(self.ptr) v = chars_to_string(v) return v end,
		GetPrimaryAttr = function(self) local v = CLIB.purple_status_type_get_primary_attr(self.ptr) v = chars_to_string(v) return v end,
		IsExclusive = function(self) local v = CLIB.purple_status_type_is_exclusive(self.ptr)  return v end,
		IsIndependent = function(self) local v = CLIB.purple_status_type_is_independent(self.ptr)  return v end,
		IsSaveable = function(self) local v = CLIB.purple_status_type_is_saveable(self.ptr)  return v end,
		IsUserSettable = function(self) local v = CLIB.purple_status_type_is_user_settable(self.ptr)  return v end,
		GetId = function(self) local v = CLIB.purple_status_type_get_id(self.ptr) v = chars_to_string(v) return v end,
	}
	META.__index = META
	metatables.StatusType = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleDesktopItem"),
		GetEntryType = function(self) local v = CLIB.purple_desktop_item_get_entry_type(self.ptr)  return v end,
		Unref = function(self) local v = CLIB.purple_desktop_item_unref(self.ptr)  return v end,
		GetString = function(self, attr) local v = CLIB.purple_desktop_item_get_string(self.ptr, attr) v = chars_to_string(v) return v end,
		Copy = function(self) local v = CLIB.purple_desktop_item_copy(self.ptr) v = wrap_pointer(v, "DesktopItem") return v end,
	}
	META.__index = META
	metatables.DesktopItem = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleNotifyUserInfoEntry"),
		SetValue = function(self, value) local v = CLIB.purple_notify_user_info_entry_set_value(self.ptr, value)  return v end,
		GetLabel = function(self) local v = CLIB.purple_notify_user_info_entry_get_label(self.ptr) v = chars_to_string(v) return v end,
		GetValue = function(self) local v = CLIB.purple_notify_user_info_entry_get_value(self.ptr) v = chars_to_string(v) return v end,
		SetType = function(self, type) local v = CLIB.purple_notify_user_info_entry_set_type(self.ptr, type)  return v end,
		SetLabel = function(self, label) local v = CLIB.purple_notify_user_info_entry_set_label(self.ptr, label)  return v end,
		GetType = function(self) local v = CLIB.purple_notify_user_info_entry_get_type(self.ptr)  return v end,
	}
	META.__index = META
	metatables.NotifyUserInfoEntry = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleLogLogger"),
		Free = function(self) local v = CLIB.purple_log_logger_free(self.ptr)  return v end,
		Add = function(self) local v = CLIB.purple_log_logger_add(self.ptr)  return v end,
		Remove = function(self) local v = CLIB.purple_log_logger_remove(self.ptr)  return v end,
		Set = function(self) local v = CLIB.purple_log_logger_set(self.ptr)  return v end,
	}
	META.__index = META
	metatables.LogLogger = META
end
do
	local META = {
		ctype = ffi.typeof("struct PurpleProxyInfo"),
		GetPassword = function(self) local v = CLIB.purple_proxy_info_get_password(self.ptr) v = chars_to_string(v) return v end,
		SetUsername = function(self, username) local v = CLIB.purple_proxy_info_set_username(self.ptr, username)  return v end,
		SetType = function(self, type) local v = CLIB.purple_proxy_info_set_type(self.ptr, type)  return v end,
		SetPassword = function(self, password) local v = CLIB.purple_proxy_info_set_password(self.ptr, password)  return v end,
		SetHost = function(self, host) local v = CLIB.purple_proxy_info_set_host(self.ptr, host)  return v end,
		GetUsername = function(self) local v = CLIB.purple_proxy_info_get_username(self.ptr) v = chars_to_string(v) return v end,
		GetPort = function(self) local v = CLIB.purple_proxy_info_get_port(self.ptr)  return v end,
		Destroy = function(self) local v = CLIB.purple_proxy_info_destroy(self.ptr)  return v end,
		GetType = function(self) local v = CLIB.purple_proxy_info_get_type(self.ptr)  return v end,
		SetPort = function(self, port) local v = CLIB.purple_proxy_info_set_port(self.ptr, port)  return v end,
		GetHost = function(self) local v = CLIB.purple_proxy_info_get_host(self.ptr) v = chars_to_string(v) return v end,
	}
	META.__index = META
	metatables.ProxyInfo = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleXfer"),
		GetStatus = function(self) local v = CLIB.purple_xfer_get_status(self.ptr)  return v end,
		SetInitFnc = function(self, fnc) local v = CLIB.purple_xfer_set_init_fnc(self.ptr, fnc)  return v end,
		GetFilename = function(self) local v = CLIB.purple_xfer_get_filename(self.ptr) v = chars_to_string(v) return v end,
		PrplReady = function(self) local v = CLIB.purple_xfer_prpl_ready(self.ptr)  return v end,
		GetType = function(self) local v = CLIB.purple_xfer_get_type(self.ptr)  return v end,
		SetSize = function(self, size) local v = CLIB.purple_xfer_set_size(self.ptr, size)  return v end,
		RequestDenied = function(self) local v = CLIB.purple_xfer_request_denied(self.ptr)  return v end,
		UpdateProgress = function(self) local v = CLIB.purple_xfer_update_progress(self.ptr)  return v end,
		Add = function(self) local v = CLIB.purple_xfer_add(self.ptr)  return v end,
		SetLocalFilename = function(self, filename) local v = CLIB.purple_xfer_set_local_filename(self.ptr, filename)  return v end,
		GetRemoteIp = function(self) local v = CLIB.purple_xfer_get_remote_ip(self.ptr) v = chars_to_string(v) return v end,
		SetReadFnc = function(self, fnc) local v = CLIB.purple_xfer_set_read_fnc(self.ptr, fnc)  return v end,
		SetEndFnc = function(self, fnc) local v = CLIB.purple_xfer_set_end_fnc(self.ptr, fnc)  return v end,
		GetAccount = function(self) local v = CLIB.purple_xfer_get_account(self.ptr) v = wrap_pointer(v, "Account") return v end,
		CancelLocal = function(self) local v = CLIB.purple_xfer_cancel_local(self.ptr)  return v end,
		End = function(self) local v = CLIB.purple_xfer_end(self.ptr)  return v end,
		Read = function(self, buffer) local v = CLIB.purple_xfer_read(self.ptr, buffer)  return v end,
		GetBytesRemaining = function(self) local v = CLIB.purple_xfer_get_bytes_remaining(self.ptr)  return v end,
		CancelRemote = function(self) local v = CLIB.purple_xfer_cancel_remote(self.ptr)  return v end,
		IsCanceled = function(self) local v = CLIB.purple_xfer_is_canceled(self.ptr)  return v end,
		GetThumbnailMimetype = function(self) local v = CLIB.purple_xfer_get_thumbnail_mimetype(self.ptr) v = chars_to_string(v) return v end,
		SetMessage = function(self, message) local v = CLIB.purple_xfer_set_message(self.ptr, message)  return v end,
		SetCancelSendFnc = function(self, fnc) local v = CLIB.purple_xfer_set_cancel_send_fnc(self.ptr, fnc)  return v end,
		IsCompleted = function(self) local v = CLIB.purple_xfer_is_completed(self.ptr)  return v end,
		Start = function(self, fd, ip, port) local v = CLIB.purple_xfer_start(self.ptr, fd, ip, port)  return v end,
		GetThumbnail = function(self, len) local v = CLIB.purple_xfer_get_thumbnail(self.ptr, len)  return v end,
		GetLocalFilename = function(self) local v = CLIB.purple_xfer_get_local_filename(self.ptr) v = chars_to_string(v) return v end,
		GetBytesSent = function(self) local v = CLIB.purple_xfer_get_bytes_sent(self.ptr)  return v end,
		ConversationWrite = function(self, message, is_error) local v = CLIB.purple_xfer_conversation_write(self.ptr, message, is_error)  return v end,
		UiReady = function(self) local v = CLIB.purple_xfer_ui_ready(self.ptr)  return v end,
		GetSize = function(self) local v = CLIB.purple_xfer_get_size(self.ptr)  return v end,
		SetCompleted = function(self, completed) local v = CLIB.purple_xfer_set_completed(self.ptr, completed)  return v end,
		SetRequestDeniedFnc = function(self, fnc) local v = CLIB.purple_xfer_set_request_denied_fnc(self.ptr, fnc)  return v end,
		GetEndTime = function(self) local v = CLIB.purple_xfer_get_end_time(self.ptr)  return v end,
		SetCancelRecvFnc = function(self, fnc) local v = CLIB.purple_xfer_set_cancel_recv_fnc(self.ptr, fnc)  return v end,
		RequestAccepted = function(self, filename) local v = CLIB.purple_xfer_request_accepted(self.ptr, filename)  return v end,
		Unref = function(self) local v = CLIB.purple_xfer_unref(self.ptr)  return v end,
		SetFilename = function(self, filename) local v = CLIB.purple_xfer_set_filename(self.ptr, filename)  return v end,
		GetStartTime = function(self) local v = CLIB.purple_xfer_get_start_time(self.ptr)  return v end,
		Write = function(self, buffer, size) local v = CLIB.purple_xfer_write(self.ptr, buffer, size)  return v end,
		SetWriteFnc = function(self, fnc) local v = CLIB.purple_xfer_set_write_fnc(self.ptr, fnc)  return v end,
		Ref = function(self) local v = CLIB.purple_xfer_ref(self.ptr)  return v end,
		SetStartFnc = function(self, fnc) local v = CLIB.purple_xfer_set_start_fnc(self.ptr, fnc)  return v end,
		GetUiOps = function(self) local v = CLIB.purple_xfer_get_ui_ops(self.ptr)  return v end,
		SetThumbnail = function(self, thumbnail, size, mimetype) local v = CLIB.purple_xfer_set_thumbnail(self.ptr, thumbnail, size, mimetype)  return v end,
		GetRemoteUser = function(self) local v = CLIB.purple_xfer_get_remote_user(self.ptr) v = chars_to_string(v) return v end,
		SetAckFnc = function(self, fnc) local v = CLIB.purple_xfer_set_ack_fnc(self.ptr, fnc)  return v end,
		SetBytesSent = function(self, bytes_sent) local v = CLIB.purple_xfer_set_bytes_sent(self.ptr, bytes_sent)  return v end,
		GetRemotePort = function(self) local v = CLIB.purple_xfer_get_remote_port(self.ptr)  return v end,
		GetProgress = function(self) local v = CLIB.purple_xfer_get_progress(self.ptr)  return v end,
		GetLocalPort = function(self) local v = CLIB.purple_xfer_get_local_port(self.ptr)  return v end,
		PrepareThumbnail = function(self, formats) local v = CLIB.purple_xfer_prepare_thumbnail(self.ptr, formats)  return v end,
		Request = function(self) local v = CLIB.purple_xfer_request(self.ptr)  return v end,
	}
	META.__index = META
	metatables.Xfer = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleStatusAttr"),
		GetValue = function(self) local v = CLIB.purple_status_attr_get_value(self.ptr) v = wrap_pointer(v, "Value") return v end,
		GetId = function(self) local v = CLIB.purple_status_attr_get_id(self.ptr) v = chars_to_string(v) return v end,
		GetName = function(self) local v = CLIB.purple_status_attr_get_name(self.ptr) v = chars_to_string(v) return v end,
		Destroy = function(self) local v = CLIB.purple_status_attr_destroy(self.ptr)  return v end,
	}
	META.__index = META
	metatables.StatusAttr = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleContact"),
		GetGroup = function(self) local v = CLIB.purple_contact_get_group(self.ptr) v = wrap_pointer(v, "Group") return v end,
		GetPriorityBuddy = function(self) local v = CLIB.purple_contact_get_priority_buddy(self.ptr) v = wrap_pointer(v, "Buddy") return v end,
		SetAlias = function(self, alias) local v = CLIB.purple_contact_set_alias(self.ptr, alias)  return v end,
		OnAccount = function(self, account) local v = CLIB.purple_contact_on_account(self.ptr, account.ptr)  return v end,
		InvalidatePriorityBuddy = function(self) local v = CLIB.purple_contact_invalidate_priority_buddy(self.ptr)  return v end,
		GetAlias = function(self) local v = CLIB.purple_contact_get_alias(self.ptr) v = chars_to_string(v) return v end,
		Destroy = function(self) local v = CLIB.purple_contact_destroy(self.ptr)  return v end,
	}
	META.__index = META
	metatables.Contact = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurplePounce"),
		ActionRegister = function(self, name) local v = CLIB.purple_pounce_action_register(self.ptr, name)  return v end,
		GetEvents = function(self) local v = CLIB.purple_pounce_get_events(self.ptr)  return v end,
		SetSave = function(self, save) local v = CLIB.purple_pounce_set_save(self.ptr, save)  return v end,
		SetPouncer = function(self, pouncer) local v = CLIB.purple_pounce_set_pouncer(self.ptr, pouncer.ptr)  return v end,
		SetPouncee = function(self, pouncee) local v = CLIB.purple_pounce_set_pouncee(self.ptr, pouncee)  return v end,
		SetEvents = function(self, events) local v = CLIB.purple_pounce_set_events(self.ptr, events)  return v end,
		GetPouncer = function(self) local v = CLIB.purple_pounce_get_pouncer(self.ptr) v = wrap_pointer(v, "Account") return v end,
		SetOptions = function(self, options) local v = CLIB.purple_pounce_set_options(self.ptr, options)  return v end,
		ActionGetAttribute = function(self, action, attr) local v = CLIB.purple_pounce_action_get_attribute(self.ptr, action, attr) v = chars_to_string(v) return v end,
		ActionIsEnabled = function(self, action) local v = CLIB.purple_pounce_action_is_enabled(self.ptr, action)  return v end,
		SetData = function(self, data) local v = CLIB.purple_pounce_set_data(self.ptr, data)  return v end,
		Destroy = function(self) local v = CLIB.purple_pounce_destroy(self.ptr)  return v end,
		ActionSetEnabled = function(self, action, enabled) local v = CLIB.purple_pounce_action_set_enabled(self.ptr, action, enabled)  return v end,
		GetSave = function(self) local v = CLIB.purple_pounce_get_save(self.ptr)  return v end,
		GetPouncee = function(self) local v = CLIB.purple_pounce_get_pouncee(self.ptr) v = chars_to_string(v) return v end,
		GetOptions = function(self) local v = CLIB.purple_pounce_get_options(self.ptr)  return v end,
		GetData = function(self) local v = CLIB.purple_pounce_get_data(self.ptr)  return v end,
		ActionSetAttribute = function(self, action, attr, value) local v = CLIB.purple_pounce_action_set_attribute(self.ptr, action, attr, value)  return v end,
	}
	META.__index = META
	metatables.Pounce = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleConnection"),
		SslError = function(self, ssl_error) local v = CLIB.purple_connection_ssl_error(self.ptr, ssl_error)  return v end,
		SetState = function(self, state) local v = CLIB.purple_connection_set_state(self.ptr, state)  return v end,
		GetPrpl = function(self) local v = CLIB.purple_connection_get_prpl(self.ptr) v = wrap_pointer(v, "Plugin") return v end,
		SetProtocolData = function(self, proto_data) local v = CLIB.purple_connection_set_protocol_data(self.ptr, proto_data)  return v end,
		GetPassword = function(self) local v = CLIB.purple_connection_get_password(self.ptr) v = chars_to_string(v) return v end,
		GetState = function(self) local v = CLIB.purple_connection_get_state(self.ptr)  return v end,
		GetAccount = function(self) local v = CLIB.purple_connection_get_account(self.ptr) v = wrap_pointer(v, "Account") return v end,
		Destroy = function(self) local v = CLIB.purple_connection_destroy(self.ptr)  return v end,
		ErrorReason = function(self, reason, description) local v = CLIB.purple_connection_error_reason(self.ptr, reason, description)  return v end,
		GetProtocolData = function(self) local v = CLIB.purple_connection_get_protocol_data(self.ptr)  return v end,
		GetDisplayName = function(self) local v = CLIB.purple_connection_get_display_name(self.ptr) v = chars_to_string(v) return v end,
		SetAccount = function(self, account) local v = CLIB.purple_connection_set_account(self.ptr, account.ptr)  return v end,
		UpdateProgress = function(self, text, step, count) local v = CLIB.purple_connection_update_progress(self.ptr, text, step, count)  return v end,
		Notice = function(self, text) local v = CLIB.purple_connection_notice(self.ptr, text)  return v end,
		Error = function(self, reason) local v = CLIB.purple_connection_error(self.ptr, reason)  return v end,
		SetDisplayName = function(self, name) local v = CLIB.purple_connection_set_display_name(self.ptr, name)  return v end,
	}
	META.__index = META
	metatables.Connection = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleMediaManager"),
		GetBackendType = function(self) local v = CLIB.purple_media_manager_get_backend_type(self.ptr)  return v end,
		SetOutputWindow = function(self, media, session_id, participant, window_id) local v = CLIB.purple_media_manager_set_output_window(self.ptr, media.ptr, session_id, participant, window_id)  return v end,
		CreateOutputWindow = function(self, media, session_id, participant) local v = CLIB.purple_media_manager_create_output_window(self.ptr, media.ptr, session_id, participant)  return v end,
		GetMedia = function(selfcast_type) local v = CLIB.purple_media_manager_get_media(self.ptr) v = glist_to_table(v, cast_type) return v end,
		SetBackendType = function(self, backend_type) local v = CLIB.purple_media_manager_set_backend_type(self.ptr, backend_type)  return v end,
		RemoveOutputWindow = function(self, output_window_id) local v = CLIB.purple_media_manager_remove_output_window(self.ptr, output_window_id)  return v end,
		GetUiCaps = function(self) local v = CLIB.purple_media_manager_get_ui_caps(self.ptr)  return v end,
		GetMediaByAccount = function(self, account, cast_type) local v = CLIB.purple_media_manager_get_media_by_account(self.ptr, account.ptr) v = glist_to_table(v, cast_type) return v end,
		RemoveOutputWindows = function(self, media, session_id, participant) local v = CLIB.purple_media_manager_remove_output_windows(self.ptr, media.ptr, session_id, participant)  return v end,
		CreateMedia = function(self, account, conference_type, remote_user, initiator) local v = CLIB.purple_media_manager_create_media(self.ptr, account.ptr, conference_type, remote_user, initiator) v = wrap_pointer(v, "Media") return v end,
		RemoveMedia = function(self, media) local v = CLIB.purple_media_manager_remove_media(self.ptr, media.ptr)  return v end,
		SetUiCaps = function(self, caps) local v = CLIB.purple_media_manager_set_ui_caps(self.ptr, caps)  return v end,
	}
	META.__index = META
	metatables.MediaManager = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleBuddy"),
		GetPresence = function(self) local v = CLIB.purple_buddy_get_presence(self.ptr) v = wrap_pointer(v, "Presence") return v end,
		GetContactAlias = function(self) local v = CLIB.purple_buddy_get_contact_alias(self.ptr) v = chars_to_string(v) return v end,
		GetAlias = function(self) local v = CLIB.purple_buddy_get_alias(self.ptr) v = chars_to_string(v) return v end,
		SetProtocolData = function(self, data) local v = CLIB.purple_buddy_set_protocol_data(self.ptr, data)  return v end,
		GetServerAlias = function(self) local v = CLIB.purple_buddy_get_server_alias(self.ptr) v = chars_to_string(v) return v end,
		GetAliasOnly = function(self) local v = CLIB.purple_buddy_get_alias_only(self.ptr) v = chars_to_string(v) return v end,
		GetGroup = function(self) local v = CLIB.purple_buddy_get_group(self.ptr) v = wrap_pointer(v, "Group") return v end,
		GetLocalBuddyAlias = function(self) local v = CLIB.purple_buddy_get_local_buddy_alias(self.ptr) v = chars_to_string(v) return v end,
		IconsGetChecksumForUser = function(self) local v = CLIB.purple_buddy_icons_get_checksum_for_user(self.ptr) v = chars_to_string(v) return v end,
		SetMediaCaps = function(self, media_caps) local v = CLIB.purple_buddy_set_media_caps(self.ptr, media_caps)  return v end,
		Destroy = function(self) local v = CLIB.purple_buddy_destroy(self.ptr)  return v end,
		GetLocalAlias = function(self) local v = CLIB.purple_buddy_get_local_alias(self.ptr) v = chars_to_string(v) return v end,
		GetProtocolData = function(self) local v = CLIB.purple_buddy_get_protocol_data(self.ptr)  return v end,
		GetIcon = function(self) local v = CLIB.purple_buddy_get_icon(self.ptr) v = wrap_pointer(v, "BuddyIcon") return v end,
		SetIcon = function(self, icon) local v = CLIB.purple_buddy_set_icon(self.ptr, icon.ptr)  return v end,
		GetContact = function(self) local v = CLIB.purple_buddy_get_contact(self.ptr) v = wrap_pointer(v, "Contact") return v end,
		GetName = function(self) local v = CLIB.purple_buddy_get_name(self.ptr) v = chars_to_string(v) return v end,
		GetMediaCaps = function(self) local v = CLIB.purple_buddy_get_media_caps(self.ptr)  return v end,
		GetAccount = function(self) local v = CLIB.purple_buddy_get_account(self.ptr) v = wrap_pointer(v, "Account") return v end,
	}
	META.__index = META
	metatables.Buddy = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleLogSet"),
		Free = function(self) local v = CLIB.purple_log_set_free(self.ptr)  return v end,
	}
	META.__index = META
	metatables.LogSet = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleMimePart"),
		GetFields = function(selfcast_type) local v = CLIB.purple_mime_part_get_fields(self.ptr) v = glist_to_table(v, cast_type) return v end,
		GetDataDecoded = function(self, data, len) local v = CLIB.purple_mime_part_get_data_decoded(self.ptr, data, len)  return v end,
		GetFieldDecoded = function(self, field) local v = CLIB.purple_mime_part_get_field_decoded(self.ptr, field) v = chars_to_string(v) return v end,
		GetField = function(self, field) local v = CLIB.purple_mime_part_get_field(self.ptr, field) v = chars_to_string(v) return v end,
		GetLength = function(self) local v = CLIB.purple_mime_part_get_length(self.ptr)  return v end,
		SetData = function(self, data) local v = CLIB.purple_mime_part_set_data(self.ptr, data)  return v end,
		GetData = function(self) local v = CLIB.purple_mime_part_get_data(self.ptr) v = chars_to_string(v) return v end,
		SetField = function(self, field, value) local v = CLIB.purple_mime_part_set_field(self.ptr, field, value)  return v end,
	}
	META.__index = META
	metatables.MimePart = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleAttentionType"),
		GetIconName = function(self) local v = CLIB.purple_attention_type_get_icon_name(self.ptr) v = chars_to_string(v) return v end,
		GetName = function(self) local v = CLIB.purple_attention_type_get_name(self.ptr) v = chars_to_string(v) return v end,
		GetIncomingDesc = function(self) local v = CLIB.purple_attention_type_get_incoming_desc(self.ptr) v = chars_to_string(v) return v end,
		SetName = function(self, name) local v = CLIB.purple_attention_type_set_name(self.ptr, name)  return v end,
		SetUnlocalizedName = function(self, ulname) local v = CLIB.purple_attention_type_set_unlocalized_name(self.ptr, ulname)  return v end,
		SetOutgoingDesc = function(self, desc) local v = CLIB.purple_attention_type_set_outgoing_desc(self.ptr, desc)  return v end,
		SetIncomingDesc = function(self, desc) local v = CLIB.purple_attention_type_set_incoming_desc(self.ptr, desc)  return v end,
		GetOutgoingDesc = function(self) local v = CLIB.purple_attention_type_get_outgoing_desc(self.ptr) v = chars_to_string(v) return v end,
		GetUnlocalizedName = function(self) local v = CLIB.purple_attention_type_get_unlocalized_name(self.ptr) v = chars_to_string(v) return v end,
		SetIconName = function(self, name) local v = CLIB.purple_attention_type_set_icon_name(self.ptr, name)  return v end,
	}
	META.__index = META
	metatables.AttentionType = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleGroup"),
		OnAccount = function(self, account) local v = CLIB.purple_group_on_account(self.ptr, account.ptr)  return v end,
		GetName = function(self) local v = CLIB.purple_group_get_name(self.ptr) v = chars_to_string(v) return v end,
		GetAccounts = function(self) local v = CLIB.purple_group_get_accounts(self.ptr)  return v end,
		Destroy = function(self) local v = CLIB.purple_group_destroy(self.ptr)  return v end,
	}
	META.__index = META
	metatables.Group = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleChat"),
		GetGroup = function(self) local v = CLIB.purple_chat_get_group(self.ptr) v = wrap_pointer(v, "Group") return v end,
		GetName = function(self) local v = CLIB.purple_chat_get_name(self.ptr) v = chars_to_string(v) return v end,
		GetComponents = function(self) local v = CLIB.purple_chat_get_components(self.ptr)  return v end,
		GetAccount = function(self) local v = CLIB.purple_chat_get_account(self.ptr) v = wrap_pointer(v, "Account") return v end,
		Destroy = function(self) local v = CLIB.purple_chat_destroy(self.ptr)  return v end,
	}
	META.__index = META
	metatables.Chat = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleTheme"),
		GetDir = function(self) local v = CLIB.purple_theme_get_dir(self.ptr) v = chars_to_string(v) return v end,
		SetDescription = function(self, description) local v = CLIB.purple_theme_set_description(self.ptr, description)  return v end,
		SetAuthor = function(self, author) local v = CLIB.purple_theme_set_author(self.ptr, author)  return v end,
		ManagerRemoveTheme = function(self) local v = CLIB.purple_theme_manager_remove_theme(self.ptr)  return v end,
		SetImage = function(self, img) local v = CLIB.purple_theme_set_image(self.ptr, img)  return v end,
		ManagerAddTheme = function(self) local v = CLIB.purple_theme_manager_add_theme(self.ptr)  return v end,
		GetAuthor = function(self) local v = CLIB.purple_theme_get_author(self.ptr) v = chars_to_string(v) return v end,
		GetName = function(self) local v = CLIB.purple_theme_get_name(self.ptr) v = chars_to_string(v) return v end,
		GetImage = function(self) local v = CLIB.purple_theme_get_image(self.ptr) v = chars_to_string(v) return v end,
		SetName = function(self, name) local v = CLIB.purple_theme_set_name(self.ptr, name)  return v end,
		GetDescription = function(self) local v = CLIB.purple_theme_get_description(self.ptr) v = chars_to_string(v) return v end,
		GetTypeString = function(self) local v = CLIB.purple_theme_get_type_string(self.ptr) v = chars_to_string(v) return v end,
		GetImageFull = function(self) local v = CLIB.purple_theme_get_image_full(self.ptr) v = chars_to_string(v) return v end,
		SetDir = function(self, dir) local v = CLIB.purple_theme_set_dir(self.ptr, dir)  return v end,
	}
	META.__index = META
	metatables.Theme = META
end
do
	local META = {
		ctype = ffi.typeof("struct _PurpleThemeLoader"),
		GetTypeString = function(self) local v = CLIB.purple_theme_loader_get_type_string(self.ptr) v = chars_to_string(v) return v end,
		Build = function(self, dir) local v = CLIB.purple_theme_loader_build(self.ptr, dir) v = wrap_pointer(v, "Theme") return v end,
	}
	META.__index = META
	metatables.ThemeLoader = META
end
library.accounts = {
	Init = function() local v = CLIB.purple_accounts_init()  return v end,
	Add = function(account) local v = CLIB.purple_accounts_add(account.ptr)  return v end,
	Find = function(name, protocol) local v = CLIB.purple_accounts_find(name, protocol) v = wrap_pointer(v, "Account") return v end,
	GetAllActive = function(cast_type) local v = CLIB.purple_accounts_get_all_active() v = glist_to_table(v, cast_type) return v end,
	Delete = function(account) local v = CLIB.purple_accounts_delete(account.ptr)  return v end,
	Reorder = function(account, new_index) local v = CLIB.purple_accounts_reorder(account.ptr, new_index)  return v end,
	SetUiOps = function(ops) local v = CLIB.purple_accounts_set_ui_ops(ops)  return v end,
	Remove = function(account) local v = CLIB.purple_accounts_remove(account.ptr)  return v end,
	GetHandle = function() local v = CLIB.purple_accounts_get_handle()  return v end,
	GetAll = function(cast_type) local v = CLIB.purple_accounts_get_all() v = glist_to_table(v, cast_type) return v end,
	RestoreCurrentStatuses = function() local v = CLIB.purple_accounts_restore_current_statuses()  return v end,
	Uninit = function() local v = CLIB.purple_accounts_uninit()  return v end,
	GetUiOps = function() local v = CLIB.purple_accounts_get_ui_ops()  return v end,
}
library.buddy = {
	IconsFindCustomIcon = function(contact) local v = CLIB.purple_buddy_icons_find_custom_icon(contact.ptr) v = wrap_pointer(v, "StoredImage") return v end,
	New = function(account, name, alias) local v = CLIB.purple_buddy_new(account.ptr, name, alias) v = wrap_pointer(v, "Buddy") return v end,
	IconsUninit = function() local v = CLIB.purple_buddy_icons_uninit()  return v end,
	IconGetScaleSize = function(spec, width, height) local v = CLIB.purple_buddy_icon_get_scale_size(spec, width, height)  return v end,
	IconsSetForUser = function(account, username, icon_data, icon_len, checksum) local v = CLIB.purple_buddy_icons_set_for_user(account.ptr, username, icon_data, icon_len, checksum)  return v end,
	IconsSetCaching = function(caching) local v = CLIB.purple_buddy_icons_set_caching(caching)  return v end,
	IconsSetCustomIcon = function(contact, icon_data, icon_len) local v = CLIB.purple_buddy_icons_set_custom_icon(contact.ptr, icon_data, icon_len) v = wrap_pointer(v, "StoredImage") return v end,
	IconsFindAccountIcon = function(account) local v = CLIB.purple_buddy_icons_find_account_icon(account.ptr) v = wrap_pointer(v, "StoredImage") return v end,
	IconsIsCaching = function() local v = CLIB.purple_buddy_icons_is_caching()  return v end,
	IconsSetCacheDir = function(cache_dir) local v = CLIB.purple_buddy_icons_set_cache_dir(cache_dir)  return v end,
	IconsNodeFindCustomIcon = function(node) local v = CLIB.purple_buddy_icons_node_find_custom_icon(node.ptr) v = wrap_pointer(v, "StoredImage") return v end,
	IconsGetHandle = function() local v = CLIB.purple_buddy_icons_get_handle()  return v end,
	IconNew = function(account, username, icon_data, icon_len, checksum) local v = CLIB.purple_buddy_icon_new(account.ptr, username, icon_data, icon_len, checksum) v = wrap_pointer(v, "BuddyIcon") return v end,
	IconsGetAccountIconTimestamp = function(account) local v = CLIB.purple_buddy_icons_get_account_icon_timestamp(account.ptr)  return v end,
	IconsNodeSetCustomIcon = function(node, icon_data, icon_len) local v = CLIB.purple_buddy_icons_node_set_custom_icon(node.ptr, icon_data, icon_len) v = wrap_pointer(v, "StoredImage") return v end,
	IconsNodeHasCustomIcon = function(node) local v = CLIB.purple_buddy_icons_node_has_custom_icon(node.ptr)  return v end,
	IconsNodeSetCustomIconFromFile = function(node, filename) local v = CLIB.purple_buddy_icons_node_set_custom_icon_from_file(node.ptr, filename) v = wrap_pointer(v, "StoredImage") return v end,
	IconsSetAccountIcon = function(account, icon_data, icon_len) local v = CLIB.purple_buddy_icons_set_account_icon(account.ptr, icon_data, icon_len) v = wrap_pointer(v, "StoredImage") return v end,
	IconsHasCustomIcon = function(contact) local v = CLIB.purple_buddy_icons_has_custom_icon(contact.ptr)  return v end,
	IconsInit = function() local v = CLIB.purple_buddy_icons_init()  return v end,
	IconsGetCacheDir = function() local v = CLIB.purple_buddy_icons_get_cache_dir() v = chars_to_string(v) return v end,
	IconsFind = function(account, username) local v = CLIB.purple_buddy_icons_find(account.ptr, username) v = wrap_pointer(v, "BuddyIcon") return v end,
}
library.version = {
	Check = function(required_major, required_minor, required_micro) local v = CLIB.purple_version_check(required_major, required_minor, required_micro) v = chars_to_string(v) return v end,
}
library.cmds = {
	Init = function() local v = CLIB.purple_cmds_init()  return v end,
	Uninit = function() local v = CLIB.purple_cmds_uninit()  return v end,
	GetHandle = function() local v = CLIB.purple_cmds_get_handle()  return v end,
}
library.txt = {
	ResolveAccount = function(account, owner, domain, cb, extradata) local v = CLIB.purple_txt_resolve_account(account.ptr, owner, domain, cb, extradata) v = wrap_pointer(v, "SrvQueryData") return v end,
	Resolve = function(owner, domain, cb, extradata) local v = CLIB.purple_txt_resolve(owner, domain, cb, extradata) v = wrap_pointer(v, "SrvQueryData") return v end,
	Cancel = function(query_data) local v = CLIB.purple_txt_cancel(query_data.ptr)  return v end,
}
library.ipv4 = {
	AddressIsValid = function(ip) local v = CLIB.purple_ipv4_address_is_valid(ip)  return v end,
}
library.cmd = {
	List = function(convcast_type) local v = CLIB.purple_cmd_list(conv.ptr) v = glist_to_table(v, cast_type) return v end,
	Register = function(cmd, args, p, f, prpl_id, func, helpstr, data) local v = CLIB.purple_cmd_register(cmd, args, p, f, prpl_id, func, helpstr, data)  return v end,
	DoCommand = function(conv, cmdline, markup, errormsg) local v = CLIB.purple_cmd_do_command(conv.ptr, cmdline, markup, errormsg)  return v end,
	Unregister = function(id) local v = CLIB.purple_cmd_unregister(id)  return v end,
	Help = function(conv, cmd, cast_type) local v = CLIB.purple_cmd_help(conv.ptr, cmd) v = glist_to_table(v, cast_type) return v end,
}
library.xfer = {
	New = function(account, type, who) local v = CLIB.purple_xfer_new(account.ptr, type, who) v = wrap_pointer(v, "Xfer") return v end,
	Error = function(type, account, who, msg) local v = CLIB.purple_xfer_error(type, account.ptr, who, msg)  return v end,
}
library.connections = {
	SetUiOps = function(ops) local v = CLIB.purple_connections_set_ui_ops(ops)  return v end,
	Init = function() local v = CLIB.purple_connections_init()  return v end,
	GetConnecting = function(cast_type) local v = CLIB.purple_connections_get_connecting() v = glist_to_table(v, cast_type) return v end,
	GetAll = function(cast_type) local v = CLIB.purple_connections_get_all() v = glist_to_table(v, cast_type) return v end,
	GetHandle = function() local v = CLIB.purple_connections_get_handle()  return v end,
	DisconnectAll = function() local v = CLIB.purple_connections_disconnect_all()  return v end,
	Uninit = function() local v = CLIB.purple_connections_uninit()  return v end,
	GetUiOps = function() local v = CLIB.purple_connections_get_ui_ops()  return v end,
}
library.print = {
	Utf8ToConsole = function(filestream, message) local v = CLIB.purple_print_utf8_to_console(filestream, message)  return v end,
}
library.cipher = {
	DigestRegion = function(name, data, data_len, in_len, unknown_5, out_len) local v = CLIB.purple_cipher_digest_region(name, data, data_len, in_len, unknown_5, out_len)  return v end,
	HttpDigestCalculateSessionKey = function(algorithm, username, realm, password, nonce, client_nonce) local v = CLIB.purple_cipher_http_digest_calculate_session_key(algorithm, username, realm, password, nonce, client_nonce) v = chars_to_string(v) return v end,
	ContextNewByName = function(name, extra) local v = CLIB.purple_cipher_context_new_by_name(name, extra) v = wrap_pointer(v, "CipherContext") return v end,
	HttpDigestCalculateResponse = function(algorithm, method, digest_uri, qop, entity, nonce, nonce_count, client_nonce, session_key) local v = CLIB.purple_cipher_http_digest_calculate_response(algorithm, method, digest_uri, qop, entity, nonce, nonce_count, client_nonce, session_key) v = chars_to_string(v) return v end,
}
library.ciphers = {
	FindCipher = function(name) local v = CLIB.purple_ciphers_find_cipher(name) v = wrap_pointer(v, "Cipher") return v end,
	UnregisterCipher = function(cipher) local v = CLIB.purple_ciphers_unregister_cipher(cipher.ptr)  return v end,
	RegisterCipher = function(name, ops) local v = CLIB.purple_ciphers_register_cipher(name, ops) v = wrap_pointer(v, "Cipher") return v end,
	GetHandle = function() local v = CLIB.purple_ciphers_get_handle()  return v end,
	GetCiphers = function(cast_type) local v = CLIB.purple_ciphers_get_ciphers() v = glist_to_table(v, cast_type) return v end,
	Uninit = function() local v = CLIB.purple_ciphers_uninit()  return v end,
	Init = function() local v = CLIB.purple_ciphers_init()  return v end,
}
library.uri = {
	ListExtractFilenames = function(uri_listcast_type) local v = CLIB.purple_uri_list_extract_filenames(uri_list) v = glist_to_table(v, cast_type) return v end,
	ListExtractUris = function(uri_listcast_type) local v = CLIB.purple_uri_list_extract_uris(uri_list) v = glist_to_table(v, cast_type) return v end,
}
library.whiteboard = {
	SetUiOps = function(ops) local v = CLIB.purple_whiteboard_set_ui_ops(ops)  return v end,
	Create = function(account, who, state) local v = CLIB.purple_whiteboard_create(account.ptr, who, state) v = wrap_pointer(v, "Whiteboard") return v end,
	DrawListDestroy = function(draw_list) local v = CLIB.purple_whiteboard_draw_list_destroy(table_to_glist(draw_list))  return v end,
	GetSession = function(account, who) local v = CLIB.purple_whiteboard_get_session(account.ptr, who) v = wrap_pointer(v, "Whiteboard") return v end,
}
library.conversations = {
	GetHandle = function() local v = CLIB.purple_conversations_get_handle()  return v end,
	SetUiOps = function(ops) local v = CLIB.purple_conversations_set_ui_ops(ops)  return v end,
	Uninit = function() local v = CLIB.purple_conversations_uninit()  return v end,
	Init = function() local v = CLIB.purple_conversations_init()  return v end,
}
library.normalize = {
	Nocase = function(account, str) local v = CLIB.purple_normalize_nocase(account.ptr, str) v = chars_to_string(v) return v end,
}
library.input = {
	GetError = function(fd, error) local v = CLIB.purple_input_get_error(fd, error)  return v end,
	Add = function(fd, cond, func, user_data) local v = CLIB.purple_input_add(fd, cond, func, user_data)  return v end,
	Remove = function(handle) local v = CLIB.purple_input_remove(handle)  return v end,
}
library.find = {
	Prpl = function(id) local v = CLIB.purple_find_prpl(id) v = wrap_pointer(v, "Plugin") return v end,
	Pounce = function(pouncer, pouncee, events) local v = CLIB.purple_find_pounce(pouncer.ptr, pouncee, events) v = wrap_pointer(v, "Pounce") return v end,
	Buddies = function(account, name) local v = CLIB.purple_find_buddies(account.ptr, name)  return v end,
	BuddyInGroup = function(account, name, group) local v = CLIB.purple_find_buddy_in_group(account.ptr, name, group.ptr) v = wrap_pointer(v, "Buddy") return v end,
	Chat = function(gc, id) local v = CLIB.purple_find_chat(gc.ptr, id) v = wrap_pointer(v, "Conversation") return v end,
	ConversationWithAccount = function(type, name, account) local v = CLIB.purple_find_conversation_with_account(type, name, account.ptr) v = wrap_pointer(v, "Conversation") return v end,
	Buddy = function(account, name) local v = CLIB.purple_find_buddy(account.ptr, name) v = wrap_pointer(v, "Buddy") return v end,
	Group = function(name) local v = CLIB.purple_find_group(name) v = wrap_pointer(v, "Group") return v end,
}
library.privacy = {
	Init = function() local v = CLIB.purple_privacy_init()  return v end,
	DenyRemove = function(account, name, local_only) local v = CLIB.purple_privacy_deny_remove(account.ptr, name, local_only)  return v end,
	Allow = function(account, who, local_, restore) local v = CLIB.purple_privacy_allow(account.ptr, who, local_, restore)  return v end,
	Check = function(account, who) local v = CLIB.purple_privacy_check(account.ptr, who)  return v end,
	Deny = function(account, who, local_, restore) local v = CLIB.purple_privacy_deny(account.ptr, who, local_, restore)  return v end,
	DenyAdd = function(account, name, local_only) local v = CLIB.purple_privacy_deny_add(account.ptr, name, local_only)  return v end,
	PermitRemove = function(account, name, local_only) local v = CLIB.purple_privacy_permit_remove(account.ptr, name, local_only)  return v end,
	GetUiOps = function() local v = CLIB.purple_privacy_get_ui_ops()  return v end,
	SetUiOps = function(ops) local v = CLIB.purple_privacy_set_ui_ops(ops)  return v end,
	PermitAdd = function(account, name, local_only) local v = CLIB.purple_privacy_permit_add(account.ptr, name, local_only)  return v end,
}
library.log = {
	LoggerGet = function() local v = CLIB.purple_log_logger_get() v = wrap_pointer(v, "LogLogger") return v end,
	LoggerGetOptions = function(cast_type) local v = CLIB.purple_log_logger_get_options() v = glist_to_table(v, cast_type) return v end,
	GetTotalSize = function(type, name, account) local v = CLIB.purple_log_get_total_size(type, name, account.ptr)  return v end,
	GetActivityScore = function(type, name, account) local v = CLIB.purple_log_get_activity_score(type, name, account.ptr)  return v end,
	GetLogDir = function(type, name, account) local v = CLIB.purple_log_get_log_dir(type, name, account.ptr) v = chars_to_string(v) return v end,
	SetCompare = function(y, z) local v = CLIB.purple_log_set_compare(y, z)  return v end,
	Init = function() local v = CLIB.purple_log_init()  return v end,
	GetLogSets = function() local v = CLIB.purple_log_get_log_sets()  return v end,
	CommonTotalSizer = function(type, name, account, ext) local v = CLIB.purple_log_common_total_sizer(type, name, account.ptr, ext)  return v end,
	GetLogs = function(type, name, account, cast_type) local v = CLIB.purple_log_get_logs(type, name, account.ptr) v = glist_to_table(v, cast_type) return v end,
	GetSystemLogs = function(accountcast_type) local v = CLIB.purple_log_get_system_logs(account.ptr) v = glist_to_table(v, cast_type) return v end,
	CommonLister = function(type, name, account, ext, logger, cast_type) local v = CLIB.purple_log_common_lister(type, name, account.ptr, ext, logger.ptr) v = glist_to_table(v, cast_type) return v end,
	New = function(type, name, account, conv, time, tm) local v = CLIB.purple_log_new(type, name, account.ptr, conv.ptr, time, tm) v = wrap_pointer(v, "Log") return v end,
	GetHandle = function() local v = CLIB.purple_log_get_handle()  return v end,
	LoggerNew = function(id, name, functions, _4) local v = CLIB.purple_log_logger_new(id, name, functions, _4) v = wrap_pointer(v, "LogLogger") return v end,
	Uninit = function() local v = CLIB.purple_log_uninit()  return v end,
	Compare = function(y, z) local v = CLIB.purple_log_compare(y, z)  return v end,
}
library.serv = {
	GotJoinChatFailed = function(gc, data) local v = CLIB.purple_serv_got_join_chat_failed(gc.ptr, data)  return v end,
	GotPrivateAlias = function(gc, who, alias) local v = CLIB.purple_serv_got_private_alias(gc.ptr, who, alias)  return v end,
}
library.build = {
	Dir = function(path, mode) local v = CLIB.purple_build_dir(path, mode)  return v end,
}
library.sounds = {
	GetHandle = function() local v = CLIB.purple_sounds_get_handle()  return v end,
}
library.escape = {
	Filename = function(str) local v = CLIB.purple_escape_filename(str) v = chars_to_string(v) return v end,
}
library.ip = {
	AddressIsValid = function(ip) local v = CLIB.purple_ip_address_is_valid(ip)  return v end,
}
library.got = {
	ProtocolHandlerUri = function(uri) local v = CLIB.purple_got_protocol_handler_uri(uri)  return v end,
}
library.roomlist = {
	SetUiOps = function(ops) local v = CLIB.purple_roomlist_set_ui_ops(ops)  return v end,
	New = function(account) local v = CLIB.purple_roomlist_new(account.ptr) v = wrap_pointer(v, "Roomlist") return v end,
	RoomNew = function(type, name, parent) local v = CLIB.purple_roomlist_room_new(type, name, parent.ptr) v = wrap_pointer(v, "RoomlistRoom") return v end,
	FieldNew = function(type, label, name, hidden) local v = CLIB.purple_roomlist_field_new(type, label, name, hidden) v = wrap_pointer(v, "RoomlistField") return v end,
	ShowWithAccount = function(account) local v = CLIB.purple_roomlist_show_with_account(account.ptr)  return v end,
	GetList = function(gc) local v = CLIB.purple_roomlist_get_list(gc.ptr) v = wrap_pointer(v, "Roomlist") return v end,
	GetUiOps = function() local v = CLIB.purple_roomlist_get_ui_ops()  return v end,
}
library.set = {
	Blist = function(blist) local v = CLIB.purple_set_blist(blist)  return v end,
}
library.certificates = {
	Import = function(scheme, filename) local v = CLIB.purple_certificates_import(scheme, filename)  return v end,
}
library.blist = {
	RenameGroup = function(group, name) local v = CLIB.purple_blist_rename_group(group.ptr, name)  return v end,
	AliasContact = function(contact, alias) local v = CLIB.purple_blist_alias_contact(contact.ptr, alias)  return v end,
	MergeContact = function(source, node) local v = CLIB.purple_blist_merge_contact(source.ptr, node.ptr)  return v end,
	UpdateNodeIcon = function(node) local v = CLIB.purple_blist_update_node_icon(node.ptr)  return v end,
	AddAccount = function(account) local v = CLIB.purple_blist_add_account(account.ptr)  return v end,
	Show = function() local v = CLIB.purple_blist_show()  return v end,
	GetGroupSize = function(group, offline) local v = CLIB.purple_blist_get_group_size(group.ptr, offline)  return v end,
	ScheduleSave = function() local v = CLIB.purple_blist_schedule_save()  return v end,
	RemoveContact = function(contact) local v = CLIB.purple_blist_remove_contact(contact.ptr)  return v end,
	RemoveAccount = function(account) local v = CLIB.purple_blist_remove_account(account.ptr)  return v end,
	RemoveGroup = function(group) local v = CLIB.purple_blist_remove_group(group.ptr)  return v end,
	AddContact = function(contact, group, node) local v = CLIB.purple_blist_add_contact(contact.ptr, group.ptr, node.ptr)  return v end,
	UpdateBuddyStatus = function(buddy, old_status) local v = CLIB.purple_blist_update_buddy_status(buddy.ptr, old_status.ptr)  return v end,
	FindChat = function(account, name) local v = CLIB.purple_blist_find_chat(account.ptr, name) v = wrap_pointer(v, "Chat") return v end,
	SetUiOps = function(ops) local v = CLIB.purple_blist_set_ui_ops(ops)  return v end,
	AddChat = function(chat, group, node) local v = CLIB.purple_blist_add_chat(chat.ptr, group.ptr, node.ptr)  return v end,
	RemoveChat = function(chat) local v = CLIB.purple_blist_remove_chat(chat.ptr)  return v end,
	SetVisible = function(show) local v = CLIB.purple_blist_set_visible(show)  return v end,
	GetBuddies = function() local v = CLIB.purple_blist_get_buddies()  return v end,
	UpdateBuddyIcon = function(buddy) local v = CLIB.purple_blist_update_buddy_icon(buddy.ptr)  return v end,
	SetUiData = function(ui_data) local v = CLIB.purple_blist_set_ui_data(ui_data)  return v end,
	GetUiOps = function() local v = CLIB.purple_blist_get_ui_ops()  return v end,
	RequestAddGroup = function() local v = CLIB.purple_blist_request_add_group()  return v end,
	Load = function() local v = CLIB.purple_blist_load()  return v end,
	ServerAliasBuddy = function(buddy, alias) local v = CLIB.purple_blist_server_alias_buddy(buddy.ptr, alias)  return v end,
	RenameBuddy = function(buddy, name) local v = CLIB.purple_blist_rename_buddy(buddy.ptr, name)  return v end,
	RemoveBuddy = function(buddy) local v = CLIB.purple_blist_remove_buddy(buddy.ptr)  return v end,
	GetGroupOnlineCount = function(group) local v = CLIB.purple_blist_get_group_online_count(group.ptr)  return v end,
	Init = function() local v = CLIB.purple_blist_init()  return v end,
	AddBuddy = function(buddy, contact, group, node) local v = CLIB.purple_blist_add_buddy(buddy.ptr, contact.ptr, group.ptr, node.ptr)  return v end,
	AliasChat = function(chat, alias) local v = CLIB.purple_blist_alias_chat(chat.ptr, alias)  return v end,
	Uninit = function() local v = CLIB.purple_blist_uninit()  return v end,
	Destroy = function() local v = CLIB.purple_blist_destroy()  return v end,
	New = function() local v = CLIB.purple_blist_new()  return v end,
	GetUiData = function() local v = CLIB.purple_blist_get_ui_data()  return v end,
	GetRoot = function() local v = CLIB.purple_blist_get_root() v = wrap_pointer(v, "BlistNode") return v end,
	AddGroup = function(group, node) local v = CLIB.purple_blist_add_group(group.ptr, node.ptr)  return v end,
	GetHandle = function() local v = CLIB.purple_blist_get_handle()  return v end,
	RequestAddChat = function(account, group, alias, name) local v = CLIB.purple_blist_request_add_chat(account.ptr, group.ptr, alias, name)  return v end,
	RequestAddBuddy = function(account, username, group, alias) local v = CLIB.purple_blist_request_add_buddy(account.ptr, username, group, alias)  return v end,
	AliasBuddy = function(buddy, alias) local v = CLIB.purple_blist_alias_buddy(buddy.ptr, alias)  return v end,
}
library.global = {
	ProxySetInfo = function(info) local v = CLIB.purple_global_proxy_set_info(info.ptr)  return v end,
	ProxyGetInfo = function() local v = CLIB.purple_global_proxy_get_info() v = wrap_pointer(v, "ProxyInfo") return v end,
}
library.conv = {
	ChatCbGetName = function(cb) local v = CLIB.purple_conv_chat_cb_get_name(cb) v = chars_to_string(v) return v end,
	ChatCbGetAttribute = function(cb, key) local v = CLIB.purple_conv_chat_cb_get_attribute(cb, key) v = chars_to_string(v) return v end,
	PresentError = function(who, account, what) local v = CLIB.purple_conv_present_error(who, account.ptr, what)  return v end,
	ChatCbDestroy = function(cb) local v = CLIB.purple_conv_chat_cb_destroy(cb)  return v end,
	ChatCbGetAttributeKeys = function(cbcast_type) local v = CLIB.purple_conv_chat_cb_get_attribute_keys(cb) v = glist_to_table(v, cast_type) return v end,
	ChatCbNew = function(name, alias, flags) local v = CLIB.purple_conv_chat_cb_new(name, alias, flags)  return v end,
}
library.dnsquery = {
	Init = function() local v = CLIB.purple_dnsquery_init()  return v end,
	A = function(hostname, port, callback, data) local v = CLIB.purple_dnsquery_a(hostname, port, callback, data)  return v end,
	Destroy = function(query_data) local v = CLIB.purple_dnsquery_destroy(query_data)  return v end,
	SetUiOps = function(ops) local v = CLIB.purple_dnsquery_set_ui_ops(ops)  return v end,
	AAccount = function(account, hostname, port, callback, data) local v = CLIB.purple_dnsquery_a_account(account.ptr, hostname, port, callback, data)  return v end,
	GetPort = function(query_data) local v = CLIB.purple_dnsquery_get_port(query_data)  return v end,
	GetUiOps = function() local v = CLIB.purple_dnsquery_get_ui_ops()  return v end,
	Uninit = function() local v = CLIB.purple_dnsquery_uninit()  return v end,
	GetHost = function(query_data) local v = CLIB.purple_dnsquery_get_host(query_data) v = chars_to_string(v) return v end,
}
library.value = {
	New = function(type, _2) local v = CLIB.purple_value_new(type, _2) v = wrap_pointer(v, "Value") return v end,
	NewOutgoing = function(type, _2) local v = CLIB.purple_value_new_outgoing(type, _2) v = wrap_pointer(v, "Value") return v end,
}
library.status = {
	Init = function() local v = CLIB.purple_status_init()  return v end,
	TypeFindWithId = function(status_types, id) local v = CLIB.purple_status_type_find_with_id(table_to_glist(status_types), id) v = wrap_pointer(v, "StatusType") return v end,
	New = function(status_type, presence) local v = CLIB.purple_status_new(status_type.ptr, presence.ptr) v = wrap_pointer(v, "Status") return v end,
	TypeNewFull = function(primitive, id, name, saveable, user_settable, independent) local v = CLIB.purple_status_type_new_full(primitive, id, name, saveable, user_settable, independent) v = wrap_pointer(v, "StatusType") return v end,
	TypeNewWithAttrs = function(primitive, id, name, saveable, user_settable, independent, attr_id, attr_name, attr_value, _10) local v = CLIB.purple_status_type_new_with_attrs(primitive, id, name, saveable, user_settable, independent, attr_id, attr_name, attr_value.ptr, _10) v = wrap_pointer(v, "StatusType") return v end,
	AttrNew = function(id, name, value_type) local v = CLIB.purple_status_attr_new(id, name, value_type.ptr) v = wrap_pointer(v, "StatusAttr") return v end,
	TypeNew = function(primitive, id, name, user_settable) local v = CLIB.purple_status_type_new(primitive, id, name, user_settable) v = wrap_pointer(v, "StatusType") return v end,
	GetHandle = function() local v = CLIB.purple_status_get_handle()  return v end,
	Uninit = function() local v = CLIB.purple_status_uninit()  return v end,
}
library.chat = {
	New = function(account, alias, components) local v = CLIB.purple_chat_new(account.ptr, alias, components) v = wrap_pointer(v, "Chat") return v end,
}
library.message = {
	Meify = function(message, len) local v = CLIB.purple_message_meify(message, len)  return v end,
}
library.marshal = {
	_POINTER__POINTER = function(cb, args, data, return_val) local v = CLIB.purple_marshal_POINTER__POINTER(cb, args, data, return_val)  return v end,
	_VOID__POINTER_POINTER_POINTER = function(cb, args, data, return_val) local v = CLIB.purple_marshal_VOID__POINTER_POINTER_POINTER(cb, args, data, return_val)  return v end,
	_BOOLEAN__POINTER_POINTER_POINTER_POINTER_POINTER = function(cb, args, data, return_val) local v = CLIB.purple_marshal_BOOLEAN__POINTER_POINTER_POINTER_POINTER_POINTER(cb, args, data, return_val)  return v end,
	_VOID__POINTER_POINTER = function(cb, args, data, return_val) local v = CLIB.purple_marshal_VOID__POINTER_POINTER(cb, args, data, return_val)  return v end,
	_VOID__POINTER_POINTER_POINTER_POINTER = function(cb, args, data, return_val) local v = CLIB.purple_marshal_VOID__POINTER_POINTER_POINTER_POINTER(cb, args, data, return_val)  return v end,
	_BOOLEAN__POINTER_POINTER_UINT = function(cb, args, data, return_val) local v = CLIB.purple_marshal_BOOLEAN__POINTER_POINTER_UINT(cb, args, data, return_val)  return v end,
	_VOID__POINTER_POINTER_UINT_UINT = function(cb, args, data, return_val) local v = CLIB.purple_marshal_VOID__POINTER_POINTER_UINT_UINT(cb, args, data, return_val)  return v end,
	_VOID__POINTER_POINTER_POINTER_POINTER_UINT = function(cb, args, data, return_val) local v = CLIB.purple_marshal_VOID__POINTER_POINTER_POINTER_POINTER_UINT(cb, args, data, return_val)  return v end,
	_VOID__POINTER_POINTER_POINTER_POINTER_POINTER = function(cb, args, data, return_val) local v = CLIB.purple_marshal_VOID__POINTER_POINTER_POINTER_POINTER_POINTER(cb, args, data, return_val)  return v end,
	_BOOLEAN__POINTER_POINTER = function(cb, args, data, return_val) local v = CLIB.purple_marshal_BOOLEAN__POINTER_POINTER(cb, args, data, return_val)  return v end,
	_POINTER__POINTER_INT_BOOLEAN = function(cb, args, data, return_val) local v = CLIB.purple_marshal_POINTER__POINTER_INT_BOOLEAN(cb, args, data, return_val)  return v end,
	_VOID__POINTER_POINTER_POINTER_UINT = function(cb, args, data, return_val) local v = CLIB.purple_marshal_VOID__POINTER_POINTER_POINTER_UINT(cb, args, data, return_val)  return v end,
	_VOID__POINTER_UINT = function(cb, args, data, return_val) local v = CLIB.purple_marshal_VOID__POINTER_UINT(cb, args, data, return_val)  return v end,
	_INT__POINTER_POINTER_POINTER = function(cb, args, data, return_val) local v = CLIB.purple_marshal_INT__POINTER_POINTER_POINTER(cb, args, data, return_val)  return v end,
	_POINTER__POINTER_INT = function(cb, args, data, return_val) local v = CLIB.purple_marshal_POINTER__POINTER_INT(cb, args, data, return_val)  return v end,
	_INT__INT = function(cb, args, data, return_val) local v = CLIB.purple_marshal_INT__INT(cb, args, data, return_val)  return v end,
	_POINTER__POINTER_INT64_BOOLEAN = function(cb, args, data, return_val) local v = CLIB.purple_marshal_POINTER__POINTER_INT64_BOOLEAN(cb, args, data, return_val)  return v end,
	_POINTER__POINTER_INT64 = function(cb, args, data, return_val) local v = CLIB.purple_marshal_POINTER__POINTER_INT64(cb, args, data, return_val)  return v end,
	_POINTER__POINTER_POINTER = function(cb, args, data, return_val) local v = CLIB.purple_marshal_POINTER__POINTER_POINTER(cb, args, data, return_val)  return v end,
	_BOOLEAN__POINTER_POINTER_POINTER_POINTER_UINT = function(cb, args, data, return_val) local v = CLIB.purple_marshal_BOOLEAN__POINTER_POINTER_POINTER_POINTER_UINT(cb, args, data, return_val)  return v end,
	_VOID__INT = function(cb, args, data, return_val) local v = CLIB.purple_marshal_VOID__INT(cb, args, data, return_val)  return v end,
	_BOOLEAN__POINTER = function(cb, args, data, return_val) local v = CLIB.purple_marshal_BOOLEAN__POINTER(cb, args, data, return_val)  return v end,
	_VOID__POINTER_POINTER_UINT = function(cb, args, data, return_val) local v = CLIB.purple_marshal_VOID__POINTER_POINTER_UINT(cb, args, data, return_val)  return v end,
	_VOID__POINTER_INT_POINTER = function(cb, args, data, return_val) local v = CLIB.purple_marshal_VOID__POINTER_INT_POINTER(cb, args, data, return_val)  return v end,
	_INT__INT_INT = function(cb, args, data, return_val) local v = CLIB.purple_marshal_INT__INT_INT(cb, args, data, return_val)  return v end,
	_BOOLEAN__POINTER_POINTER_POINTER_UINT = function(cb, args, data, return_val) local v = CLIB.purple_marshal_BOOLEAN__POINTER_POINTER_POINTER_UINT(cb, args, data, return_val)  return v end,
	_BOOLEAN__POINTER_POINTER_POINTER = function(cb, args, data, return_val) local v = CLIB.purple_marshal_BOOLEAN__POINTER_POINTER_POINTER(cb, args, data, return_val)  return v end,
	_INT__POINTER_POINTER = function(cb, args, data, return_val) local v = CLIB.purple_marshal_INT__POINTER_POINTER(cb, args, data, return_val)  return v end,
	_VOID__POINTER_POINTER_POINTER_UINT_UINT = function(cb, args, data, return_val) local v = CLIB.purple_marshal_VOID__POINTER_POINTER_POINTER_UINT_UINT(cb, args, data, return_val)  return v end,
	_INT__POINTER_POINTER_POINTER_POINTER_POINTER = function(cb, args, data, return_val) local v = CLIB.purple_marshal_INT__POINTER_POINTER_POINTER_POINTER_POINTER(cb, args, data, return_val)  return v end,
	_VOID__POINTER_INT_INT = function(cb, args, data, return_val) local v = CLIB.purple_marshal_VOID__POINTER_INT_INT(cb, args, data, return_val)  return v end,
	_VOID = function(cb, args, data, return_val) local v = CLIB.purple_marshal_VOID(cb, args, data, return_val)  return v end,
	_BOOLEAN__INT_POINTER = function(cb, args, data, return_val) local v = CLIB.purple_marshal_BOOLEAN__INT_POINTER(cb, args, data, return_val)  return v end,
	_BOOLEAN__POINTER_POINTER_POINTER_POINTER_POINTER_POINTER = function(cb, args, data, return_val) local v = CLIB.purple_marshal_BOOLEAN__POINTER_POINTER_POINTER_POINTER_POINTER_POINTER(cb, args, data, return_val)  return v end,
	_VOID__POINTER = function(cb, args, data, return_val) local v = CLIB.purple_marshal_VOID__POINTER(cb, args, data, return_val)  return v end,
	_BOOLEAN__POINTER_BOOLEAN = function(cb, args, data, return_val) local v = CLIB.purple_marshal_BOOLEAN__POINTER_BOOLEAN(cb, args, data, return_val)  return v end,
	_VOID__INT_INT = function(cb, args, data, return_val) local v = CLIB.purple_marshal_VOID__INT_INT(cb, args, data, return_val)  return v end,
	_BOOLEAN__POINTER_POINTER_POINTER_POINTER = function(cb, args, data, return_val) local v = CLIB.purple_marshal_BOOLEAN__POINTER_POINTER_POINTER_POINTER(cb, args, data, return_val)  return v end,
}
library.base16 = {
	EncodeChunked = function(data, len) local v = CLIB.purple_base16_encode_chunked(data, len) v = chars_to_string(v) return v end,
	Encode = function(data, len) local v = CLIB.purple_base16_encode(data, len) v = chars_to_string(v) return v end,
	Decode = function(str, ret_len) local v = CLIB.purple_base16_decode(str, ret_len)  return v end,
}
library.ssl = {
	Init = function() local v = CLIB.purple_ssl_init()  return v end,
	ConnectWithHostFd = function(account, fd, func, error_func, host, data) local v = CLIB.purple_ssl_connect_with_host_fd(account.ptr, fd, func, error_func, host, data) v = wrap_pointer(v, "SslConnection") return v end,
	SetOps = function(ops) local v = CLIB.purple_ssl_set_ops(ops)  return v end,
	ConnectFd = function(account, fd, func, error_func, data) local v = CLIB.purple_ssl_connect_fd(account.ptr, fd, func, error_func, data) v = wrap_pointer(v, "SslConnection") return v end,
	Connect = function(account, host, port, func, error_func, data) local v = CLIB.purple_ssl_connect(account.ptr, host, port, func, error_func, data) v = wrap_pointer(v, "SslConnection") return v end,
	IsSupported = function() local v = CLIB.purple_ssl_is_supported()  return v end,
	ConnectWithSslCn = function(account, host, port, func, error_func, ssl_host, data) local v = CLIB.purple_ssl_connect_with_ssl_cn(account.ptr, host, port, func, error_func, ssl_host, data) v = wrap_pointer(v, "SslConnection") return v end,
	Strerror = function(error) local v = CLIB.purple_ssl_strerror(error) v = chars_to_string(v) return v end,
	Uninit = function() local v = CLIB.purple_ssl_uninit()  return v end,
	GetOps = function() local v = CLIB.purple_ssl_get_ops()  return v end,
}
library.upnp = {
	RemovePortMapping = function(portmap, protocol, cb, cb_data) local v = CLIB.purple_upnp_remove_port_mapping(portmap, protocol, cb, cb_data)  return v end,
	SetPortMapping = function(portmap, protocol, cb, cb_data) local v = CLIB.purple_upnp_set_port_mapping(portmap, protocol, cb, cb_data)  return v end,
	Discover = function(cb, cb_data) local v = CLIB.purple_upnp_discover(cb, cb_data)  return v end,
	CancelPortMapping = function(mapping_data) local v = CLIB.purple_upnp_cancel_port_mapping(mapping_data)  return v end,
	Init = function() local v = CLIB.purple_upnp_init()  return v end,
	GetPublicIp = function() local v = CLIB.purple_upnp_get_public_ip() v = chars_to_string(v) return v end,
}
library.imgstore = {
	Init = function() local v = CLIB.purple_imgstore_init()  return v end,
	FindById = function(id) local v = CLIB.purple_imgstore_find_by_id(id) v = wrap_pointer(v, "StoredImage") return v end,
	UnrefById = function(id) local v = CLIB.purple_imgstore_unref_by_id(id)  return v end,
	NewFromFile = function(path) local v = CLIB.purple_imgstore_new_from_file(path) v = wrap_pointer(v, "StoredImage") return v end,
	RefById = function(id) local v = CLIB.purple_imgstore_ref_by_id(id)  return v end,
	GetHandle = function() local v = CLIB.purple_imgstore_get_handle()  return v end,
	Add = function(data, size, filename) local v = CLIB.purple_imgstore_add(data, size, filename) v = wrap_pointer(v, "StoredImage") return v end,
	Uninit = function() local v = CLIB.purple_imgstore_uninit()  return v end,
	AddWithId = function(data, size, filename) local v = CLIB.purple_imgstore_add_with_id(data, size, filename)  return v end,
}
library.pmp = {
	Init = function() local v = CLIB.purple_pmp_init()  return v end,
	CreateMap = function(type, privateport, publicport, lifetime) local v = CLIB.purple_pmp_create_map(type, privateport, publicport, lifetime)  return v end,
	DestroyMap = function(type, privateport) local v = CLIB.purple_pmp_destroy_map(type, privateport)  return v end,
	GetPublicIp = function() local v = CLIB.purple_pmp_get_public_ip() v = chars_to_string(v) return v end,
}
library.time = {
	Format = function(tm) local v = CLIB.purple_time_format(tm) v = chars_to_string(v) return v end,
	Build = function(year, month, day, hour, min, sec) local v = CLIB.purple_time_build(year, month, day, hour, min, sec)  return v end,
}
library.util = {
	FetchUrlRequest = function(url, full, user_agent, http11, request, include_headers, callback, data) local v = CLIB.purple_util_fetch_url_request(url, full, user_agent, http11, request, include_headers, callback, data)  return v end,
	Chrreplace = function(string, delimiter, replacement) local v = CLIB.purple_util_chrreplace(string, delimiter, replacement)  return v end,
	GetImageExtension = function(data, len) local v = CLIB.purple_util_get_image_extension(data, len) v = chars_to_string(v) return v end,
	WriteDataToFileAbsolute = function(filename_full, data, size) local v = CLIB.purple_util_write_data_to_file_absolute(filename_full, data, size)  return v end,
	GetImageFilename = function(image_data, image_len) local v = CLIB.purple_util_get_image_filename(image_data, image_len) v = chars_to_string(v) return v end,
	SetUserDir = function(dir) local v = CLIB.purple_util_set_user_dir(dir)  return v end,
	SetCurrentSong = function(title, artist, album) local v = CLIB.purple_util_set_current_song(title, artist, album)  return v end,
	Init = function() local v = CLIB.purple_util_init()  return v end,
	FetchUrlRequestLen = function(url, full, user_agent, http11, request, include_headers, max_len, callback, data) local v = CLIB.purple_util_fetch_url_request_len(url, full, user_agent, http11, request, include_headers, max_len, callback, data)  return v end,
	FormatSongInfo = function(title, artist, album, unused) local v = CLIB.purple_util_format_song_info(title, artist, album, unused) v = chars_to_string(v) return v end,
	ReadXmlFromFile = function(filename, description) local v = CLIB.purple_util_read_xml_from_file(filename, description)  return v end,
	GetImageChecksum = function(image_data, image_len) local v = CLIB.purple_util_get_image_checksum(image_data, image_len) v = chars_to_string(v) return v end,
	FetchUrlRequestLenWithAccount = function(account, url, full, user_agent, http11, request, include_headers, max_len, callback, data) local v = CLIB.purple_util_fetch_url_request_len_with_account(account.ptr, url, full, user_agent, http11, request, include_headers, max_len, callback, data)  return v end,
	FetchUrlCancel = function(url_data) local v = CLIB.purple_util_fetch_url_cancel(url_data)  return v end,
	Uninit = function() local v = CLIB.purple_util_uninit()  return v end,
	WriteDataToFile = function(filename, data, size) local v = CLIB.purple_util_write_data_to_file(filename, data, size)  return v end,
}
library.socket = {
	GetFamily = function(fd) local v = CLIB.purple_socket_get_family(fd)  return v end,
	SpeaksIpv4 = function(fd) local v = CLIB.purple_socket_speaks_ipv4(fd)  return v end,
}
library.plugin = {
	ActionNew = function(label, callback) local v = CLIB.purple_plugin_action_new(label, callback) v = wrap_pointer(v, "PluginAction") return v end,
	PrefNewWithLabel = function(label) local v = CLIB.purple_plugin_pref_new_with_label(label) v = wrap_pointer(v, "PluginPref") return v end,
	New = function(native, path) local v = CLIB.purple_plugin_new(native, path) v = wrap_pointer(v, "Plugin") return v end,
	PrefNewWithName = function(name) local v = CLIB.purple_plugin_pref_new_with_name(name) v = wrap_pointer(v, "PluginPref") return v end,
	PrefNew = function() local v = CLIB.purple_plugin_pref_new() v = wrap_pointer(v, "PluginPref") return v end,
	Probe = function(filename) local v = CLIB.purple_plugin_probe(filename) v = wrap_pointer(v, "Plugin") return v end,
	PrefNewWithNameAndLabel = function(name, label) local v = CLIB.purple_plugin_pref_new_with_name_and_label(name, label) v = wrap_pointer(v, "PluginPref") return v end,
	PrefFrameNew = function() local v = CLIB.purple_plugin_pref_frame_new() v = wrap_pointer(v, "PluginPrefFrame") return v end,
}
library.home = {
	Dir = function() local v = CLIB.purple_home_dir() v = chars_to_string(v) return v end,
}
library.desktop = {
	ItemNewFromFile = function(filename) local v = CLIB.purple_desktop_item_new_from_file(filename) v = wrap_pointer(v, "DesktopItem") return v end,
	ItemGetType = function() local v = CLIB.purple_desktop_item_get_type()  return v end,
}
library.core = {
	Init = function(ui) local v = CLIB.purple_core_init(ui)  return v end,
	GetUiInfo = function() local v = CLIB.purple_core_get_ui_info()  return v end,
	EnsureSingleInstance = function() local v = CLIB.purple_core_ensure_single_instance()  return v end,
	GetVersion = function() local v = CLIB.purple_core_get_version() v = chars_to_string(v) return v end,
	Migrate = function() local v = CLIB.purple_core_migrate()  return v end,
	SetUiOps = function(ops) local v = CLIB.purple_core_set_ui_ops(ops)  return v end,
	QuitCb = function(unused) local v = CLIB.purple_core_quit_cb(unused)  return v end,
	GetUi = function() local v = CLIB.purple_core_get_ui() v = chars_to_string(v) return v end,
	GetUiOps = function() local v = CLIB.purple_core_get_ui_ops()  return v end,
	Quit = function() local v = CLIB.purple_core_quit()  return v end,
}
library.utf8 = {
	NcrEncode = function(in_) local v = CLIB.purple_utf8_ncr_encode(in_) v = chars_to_string(v) return v end,
	HasWord = function(haystack, needle) local v = CLIB.purple_utf8_has_word(haystack, needle)  return v end,
	NcrDecode = function(in_) local v = CLIB.purple_utf8_ncr_decode(in_) v = chars_to_string(v) return v end,
	TryConvert = function(str) local v = CLIB.purple_utf8_try_convert(str) v = chars_to_string(v) return v end,
	StripUnprintables = function(str) local v = CLIB.purple_utf8_strip_unprintables(str) v = chars_to_string(v) return v end,
	Strftime = function(format, tm) local v = CLIB.purple_utf8_strftime(format, tm) v = chars_to_string(v) return v end,
	Strcasecmp = function(a, b) local v = CLIB.purple_utf8_strcasecmp(a, b)  return v end,
	Salvage = function(str) local v = CLIB.purple_utf8_salvage(str) v = chars_to_string(v) return v end,
}
library.network = {
	RemovePortMapping = function(fd) local v = CLIB.purple_network_remove_port_mapping(fd)  return v end,
	ListenRangeFamily = function(start, end_, socket_family, socket_type, cb, cb_data) local v = CLIB.purple_network_listen_range_family(start, end_, socket_family, socket_type, cb, cb_data)  return v end,
	IsAvailable = function() local v = CLIB.purple_network_is_available()  return v end,
	GetStunIp = function() local v = CLIB.purple_network_get_stun_ip() v = chars_to_string(v) return v end,
	ListenCancel = function(listen_data) local v = CLIB.purple_network_listen_cancel(listen_data)  return v end,
	ListenRange = function(start, end_, socket_type, cb, cb_data) local v = CLIB.purple_network_listen_range(start, end_, socket_type, cb, cb_data)  return v end,
	GetTurnIp = function() local v = CLIB.purple_network_get_turn_ip() v = chars_to_string(v) return v end,
	SetStunServer = function(stun_server) local v = CLIB.purple_network_set_stun_server(stun_server)  return v end,
	Listen = function(port, socket_type, cb, cb_data) local v = CLIB.purple_network_listen(port, socket_type, cb, cb_data)  return v end,
	Init = function() local v = CLIB.purple_network_init()  return v end,
	ListenMapExternal = function(map_external) local v = CLIB.purple_network_listen_map_external(map_external)  return v end,
	ConvertIdnToAscii = function(in_, out) local v = CLIB.purple_network_convert_idn_to_ascii(in_, out)  return v end,
	IpAtoi = function(ip) local v = CLIB.purple_network_ip_atoi(ip)  return v end,
	GetMyIp = function(fd) local v = CLIB.purple_network_get_my_ip(fd) v = chars_to_string(v) return v end,
	ListenFamily = function(port, socket_family, socket_type, cb, cb_data) local v = CLIB.purple_network_listen_family(port, socket_family, socket_type, cb, cb_data)  return v end,
	GetPublicIp = function() local v = CLIB.purple_network_get_public_ip() v = chars_to_string(v) return v end,
	ForceOnline = function() local v = CLIB.purple_network_force_online()  return v end,
	GetAllLocalSystemIps = function(cast_type) local v = CLIB.purple_network_get_all_local_system_ips() v = glist_to_table(v, cast_type) return v end,
	GetPortFromFd = function(fd) local v = CLIB.purple_network_get_port_from_fd(fd)  return v end,
	SetPublicIp = function(ip) local v = CLIB.purple_network_set_public_ip(ip)  return v end,
	GetHandle = function() local v = CLIB.purple_network_get_handle()  return v end,
	SetTurnServer = function(turn_server) local v = CLIB.purple_network_set_turn_server(turn_server)  return v end,
	Uninit = function() local v = CLIB.purple_network_uninit()  return v end,
	GetLocalSystemIp = function(fd) local v = CLIB.purple_network_get_local_system_ip(fd) v = chars_to_string(v) return v end,
}
library.gai = {
	Strerror = function(errnum) local v = CLIB.purple_gai_strerror(errnum) v = chars_to_string(v) return v end,
}
library.markup = {
	UnescapeEntity = function(text, length) local v = CLIB.purple_markup_unescape_entity(text, length) v = chars_to_string(v) return v end,
	FindTag = function(needle, haystack, start, end_, attributes) local v = CLIB.purple_markup_find_tag(needle, haystack, start, end_, attributes)  return v end,
	GetCssProperty = function(style, opt) local v = CLIB.purple_markup_get_css_property(style, opt) v = chars_to_string(v) return v end,
	IsRtl = function(html) local v = CLIB.purple_markup_is_rtl(html)  return v end,
	Linkify = function(str) local v = CLIB.purple_markup_linkify(str) v = chars_to_string(v) return v end,
	HtmlToXhtml = function(html, dest_xhtml, dest_plain) local v = CLIB.purple_markup_html_to_xhtml(html, dest_xhtml, dest_plain)  return v end,
	Slice = function(str, x, y) local v = CLIB.purple_markup_slice(str, x, y) v = chars_to_string(v) return v end,
	ExtractInfoField = function(str, len, user_info, start_token, skip, end_token, check_value, no_value_token, display_name, is_link, link_prefix, format_cb) local v = CLIB.purple_markup_extract_info_field(str, len, user_info.ptr, start_token, skip, end_token, check_value, no_value_token, display_name, is_link, link_prefix, format_cb)  return v end,
	EscapeText = function(text, length) local v = CLIB.purple_markup_escape_text(text, length) v = chars_to_string(v) return v end,
	GetTagName = function(tag) local v = CLIB.purple_markup_get_tag_name(tag) v = chars_to_string(v) return v end,
	StripHtml = function(str) local v = CLIB.purple_markup_strip_html(str) v = chars_to_string(v) return v end,
}
library.uuid = {
	Random = function() local v = CLIB.purple_uuid_random() v = chars_to_string(v) return v end,
}
library.text = {
	StripMnemonic = function(in_) local v = CLIB.purple_text_strip_mnemonic(in_) v = chars_to_string(v) return v end,
}
library.mime = {
	DecodeField = function(str) local v = CLIB.purple_mime_decode_field(str) v = chars_to_string(v) return v end,
	DocumentParsen = function(buf, len) local v = CLIB.purple_mime_document_parsen(buf, len) v = wrap_pointer(v, "MimeDocument") return v end,
	DocumentNew = function() local v = CLIB.purple_mime_document_new() v = wrap_pointer(v, "MimeDocument") return v end,
	DocumentParse = function(buf) local v = CLIB.purple_mime_document_parse(buf) v = wrap_pointer(v, "MimeDocument") return v end,
	PartNew = function(doc) local v = CLIB.purple_mime_part_new(doc.ptr) v = wrap_pointer(v, "MimePart") return v end,
}
library.notify = {
	SearchresultsColumnAdd = function(results, column) local v = CLIB.purple_notify_searchresults_column_add(results, column)  return v end,
	Emails = function(handle, count, detailed, subjects, froms, tos, urls, cb, user_data) local v = CLIB.purple_notify_emails(handle, count, detailed, subjects, froms, tos, urls, cb, user_data)  return v end,
	SearchresultsGetColumnsCount = function(results) local v = CLIB.purple_notify_searchresults_get_columns_count(results)  return v end,
	SearchresultsRowGet = function(results, row_id, cast_type) local v = CLIB.purple_notify_searchresults_row_get(results, row_id) v = glist_to_table(v, cast_type) return v end,
	SearchresultsNew = function() local v = CLIB.purple_notify_searchresults_new()  return v end,
	Formatted = function(handle, title, primary, secondary, text, cb, user_data) local v = CLIB.purple_notify_formatted(handle, title, primary, secondary, text, cb, user_data)  return v end,
	SearchresultsGetRowsCount = function(results) local v = CLIB.purple_notify_searchresults_get_rows_count(results)  return v end,
	SearchresultsButtonAdd = function(results, type, cb) local v = CLIB.purple_notify_searchresults_button_add(results, type, cb)  return v end,
	GetUiOps = function() local v = CLIB.purple_notify_get_ui_ops()  return v end,
	Close = function(type, ui_handle) local v = CLIB.purple_notify_close(type, ui_handle)  return v end,
	CloseWithHandle = function(handle) local v = CLIB.purple_notify_close_with_handle(handle)  return v end,
	SearchresultsColumnGetTitle = function(results, column_id) local v = CLIB.purple_notify_searchresults_column_get_title(results, column_id) v = chars_to_string(v) return v end,
	Init = function() local v = CLIB.purple_notify_init()  return v end,
	Userinfo = function(gc, who, user_info, cb, user_data) local v = CLIB.purple_notify_userinfo(gc.ptr, who, user_info.ptr, cb, user_data)  return v end,
	UserInfoEntryNew = function(label, value) local v = CLIB.purple_notify_user_info_entry_new(label, value) v = wrap_pointer(v, "NotifyUserInfoEntry") return v end,
	SearchresultsRowAdd = function(results, row) local v = CLIB.purple_notify_searchresults_row_add(results, table_to_glist(row))  return v end,
	Message = function(handle, type, title, primary, secondary, cb, user_data) local v = CLIB.purple_notify_message(handle, type, title, primary, secondary, cb, user_data)  return v end,
	SearchresultsFree = function(results) local v = CLIB.purple_notify_searchresults_free(results)  return v end,
	UserInfoNew = function() local v = CLIB.purple_notify_user_info_new() v = wrap_pointer(v, "NotifyUserInfo") return v end,
	SearchresultsButtonAddLabeled = function(results, label, cb) local v = CLIB.purple_notify_searchresults_button_add_labeled(results, label, cb)  return v end,
	SetUiOps = function(ops) local v = CLIB.purple_notify_set_ui_ops(ops)  return v end,
	SearchresultsColumnNew = function(title) local v = CLIB.purple_notify_searchresults_column_new(title)  return v end,
	Uri = function(handle, uri) local v = CLIB.purple_notify_uri(handle, uri)  return v end,
	Searchresults = function(gc, title, primary, secondary, results, cb, user_data) local v = CLIB.purple_notify_searchresults(gc.ptr, title, primary, secondary, results, cb, user_data)  return v end,
	GetHandle = function() local v = CLIB.purple_notify_get_handle()  return v end,
	Email = function(handle, subject, from, to, url, cb, user_data) local v = CLIB.purple_notify_email(handle, subject, from, to, url, cb, user_data)  return v end,
	Uninit = function() local v = CLIB.purple_notify_uninit()  return v end,
	SearchresultsNewRows = function(gc, results, data) local v = CLIB.purple_notify_searchresults_new_rows(gc.ptr, results, data)  return v end,
}
library.theme = {
	ManagerRegisterType = function(loader) local v = CLIB.purple_theme_manager_register_type(loader.ptr)  return v end,
	ManagerRefresh = function() local v = CLIB.purple_theme_manager_refresh()  return v end,
	GetType = function() local v = CLIB.purple_theme_get_type()  return v end,
	ManagerFindTheme = function(name, type) local v = CLIB.purple_theme_manager_find_theme(name, type) v = wrap_pointer(v, "Theme") return v end,
	ManagerInit = function() local v = CLIB.purple_theme_manager_init()  return v end,
	ManagerForEachTheme = function(func) local v = CLIB.purple_theme_manager_for_each_theme(func)  return v end,
	ManagerUnregisterType = function(loader) local v = CLIB.purple_theme_manager_unregister_type(loader.ptr)  return v end,
	ManagerGetType = function() local v = CLIB.purple_theme_manager_get_type()  return v end,
	LoaderGetType = function() local v = CLIB.purple_theme_loader_get_type()  return v end,
	ManagerUninit = function() local v = CLIB.purple_theme_manager_uninit()  return v end,
	ManagerLoadTheme = function(theme_dir, type) local v = CLIB.purple_theme_manager_load_theme(theme_dir, type) v = wrap_pointer(v, "Theme") return v end,
}
library.eventloop = {
	GetUiOps = function() local v = CLIB.purple_eventloop_get_ui_ops()  return v end,
	SetUiOps = function(ops) local v = CLIB.purple_eventloop_set_ui_ops(ops)  return v end,
}
library.circ = {
	BufferNew = function(growsize) local v = CLIB.purple_circ_buffer_new(growsize) v = wrap_pointer(v, "CircBuffer") return v end,
}
library.group = {
	New = function(name) local v = CLIB.purple_group_new(name) v = wrap_pointer(v, "Group") return v end,
}
library.quotedp = {
	Decode = function(str, ret_len) local v = CLIB.purple_quotedp_decode(str, ret_len)  return v end,
}
library.ipv6 = {
	AddressIsValid = function(ip) local v = CLIB.purple_ipv6_address_is_valid(ip)  return v end,
}
library.url = {
	Parse = function(url, ret_host, ret_port, ret_path, ret_user, ret_passwd) local v = CLIB.purple_url_parse(url, ret_host, ret_port, ret_path, ret_user, ret_passwd)  return v end,
	Encode = function(str) local v = CLIB.purple_url_encode(str) v = chars_to_string(v) return v end,
	Decode = function(str) local v = CLIB.purple_url_decode(str) v = chars_to_string(v) return v end,
}
library.str = {
	StripChar = function(str, thechar) local v = CLIB.purple_str_strip_char(str, thechar)  return v end,
	HasPrefix = function(s, p) local v = CLIB.purple_str_has_prefix(s, p)  return v end,
	HasSuffix = function(s, x) local v = CLIB.purple_str_has_suffix(s, x)  return v end,
	ToTime = function(timestamp, utc, tm, tz_off, rest) local v = CLIB.purple_str_to_time(timestamp, utc, tm, tz_off, rest)  return v end,
	BinaryToAscii = function(binary, len) local v = CLIB.purple_str_binary_to_ascii(binary, len) v = chars_to_string(v) return v end,
	SizeToUnits = function(size) local v = CLIB.purple_str_size_to_units(size) v = chars_to_string(v) return v end,
	AddCr = function(str) local v = CLIB.purple_str_add_cr(str) v = chars_to_string(v) return v end,
	SecondsToString = function(sec) local v = CLIB.purple_str_seconds_to_string(sec) v = chars_to_string(v) return v end,
}
library.attention = {
	TypeNew = function(ulname, name, inc_desc, out_desc) local v = CLIB.purple_attention_type_new(ulname, name, inc_desc, out_desc) v = wrap_pointer(v, "AttentionType") return v end,
}
library.program = {
	IsValid = function(program) local v = CLIB.purple_program_is_valid(program)  return v end,
}
library.conversation = {
	New = function(type, account, name) local v = CLIB.purple_conversation_new(type, account.ptr, name) v = wrap_pointer(v, "Conversation") return v end,
	Foreach = function(func) local v = CLIB.purple_conversation_foreach(func)  return v end,
}
library.base64 = {
	Encode = function(data, len) local v = CLIB.purple_base64_encode(data, len) v = chars_to_string(v) return v end,
	Decode = function(str, ret_len) local v = CLIB.purple_base64_decode(str, ret_len)  return v end,
}
library.connection = {
	NewUnregister = function(account, password, cb, user_data) local v = CLIB.purple_connection_new_unregister(account.ptr, password, cb, user_data)  return v end,
	ErrorIsFatal = function(reason) local v = CLIB.purple_connection_error_is_fatal(reason)  return v end,
	New = function(account, regist, password) local v = CLIB.purple_connection_new(account.ptr, regist, password)  return v end,
}
library.pounce = {
	DestroyAllByBuddy = function(buddy) local v = CLIB.purple_pounce_destroy_all_by_buddy(buddy.ptr)  return v end,
	Execute = function(pouncer, pouncee, events) local v = CLIB.purple_pounce_execute(pouncer.ptr, pouncee, events)  return v end,
	New = function(ui_type, pouncer, pouncee, event, option) local v = CLIB.purple_pounce_new(ui_type, pouncer.ptr, pouncee, event, option) v = wrap_pointer(v, "Pounce") return v end,
	DestroyAllByAccount = function(account) local v = CLIB.purple_pounce_destroy_all_by_account(account.ptr)  return v end,
}
library.contact = {
	New = function() local v = CLIB.purple_contact_new() v = wrap_pointer(v, "Contact") return v end,
}
library.menu = {
	ActionNew = function(label, callback, data, children) local v = CLIB.purple_menu_action_new(label, callback, data, table_to_glist(children)) v = wrap_pointer(v, "MenuAction") return v end,
}
library.fd = {
	GetIp = function(fd) local v = CLIB.purple_fd_get_ip(fd) v = chars_to_string(v) return v end,
}
library.smileys = {
	Init = function() local v = CLIB.purple_smileys_init()  return v end,
	FindByChecksum = function(checksum) local v = CLIB.purple_smileys_find_by_checksum(checksum) v = wrap_pointer(v, "Smiley") return v end,
	GetAll = function(cast_type) local v = CLIB.purple_smileys_get_all() v = glist_to_table(v, cast_type) return v end,
	GetStoringDir = function() local v = CLIB.purple_smileys_get_storing_dir() v = chars_to_string(v) return v end,
	Uninit = function() local v = CLIB.purple_smileys_uninit()  return v end,
	FindByShortcut = function(shortcut) local v = CLIB.purple_smileys_find_by_shortcut(shortcut) v = wrap_pointer(v, "Smiley") return v end,
}
library.stun = {
	Init = function() local v = CLIB.purple_stun_init()  return v end,
	Discover = function(cb) local v = CLIB.purple_stun_discover(cb)  return v end,
}
library.ntlm = {
	GenType1 = function(hostname, domain) local v = CLIB.purple_ntlm_gen_type1(hostname, domain) v = chars_to_string(v) return v end,
	ParseType2 = function(type2, flags) local v = CLIB.purple_ntlm_parse_type2(type2, flags)  return v end,
	GenType3 = function(username, passw, hostname, domain, nonce, flags) local v = CLIB.purple_ntlm_gen_type3(username, passw, hostname, domain, nonce, flags) v = chars_to_string(v) return v end,
}
library.restore = {
	DefaultSignalHandlers = function() local v = CLIB.purple_restore_default_signal_handlers()  return v end,
}
library.prefs = {
	GetBool = function(name) local v = CLIB.purple_prefs_get_bool(name)  return v end,
	TriggerCallback = function(name) local v = CLIB.purple_prefs_trigger_callback(name)  return v end,
	GetPathList = function(namecast_type) local v = CLIB.purple_prefs_get_path_list(name) v = glist_to_table(v, cast_type) return v end,
	GetInt = function(name) local v = CLIB.purple_prefs_get_int(name)  return v end,
	UpdateOld = function() local v = CLIB.purple_prefs_update_old()  return v end,
	SetPath = function(name, value) local v = CLIB.purple_prefs_set_path(name, value)  return v end,
	AddString = function(name, value) local v = CLIB.purple_prefs_add_string(name, value)  return v end,
	Init = function() local v = CLIB.purple_prefs_init()  return v end,
	Load = function() local v = CLIB.purple_prefs_load()  return v end,
	SetGeneric = function(name, value) local v = CLIB.purple_prefs_set_generic(name, value)  return v end,
	Rename = function(oldname, newname) local v = CLIB.purple_prefs_rename(oldname, newname)  return v end,
	AddPath = function(name, value) local v = CLIB.purple_prefs_add_path(name, value)  return v end,
	SetString = function(name, value) local v = CLIB.purple_prefs_set_string(name, value)  return v end,
	AddNone = function(name) local v = CLIB.purple_prefs_add_none(name)  return v end,
	SetPathList = function(name, value) local v = CLIB.purple_prefs_set_path_list(name, table_to_glist(value))  return v end,
	AddInt = function(name, value) local v = CLIB.purple_prefs_add_int(name, value)  return v end,
	RenameBooleanToggle = function(oldname, newname) local v = CLIB.purple_prefs_rename_boolean_toggle(oldname, newname)  return v end,
	SetStringList = function(name, value) local v = CLIB.purple_prefs_set_string_list(name, table_to_glist(value))  return v end,
	Exists = function(name) local v = CLIB.purple_prefs_exists(name)  return v end,
	DisconnectByHandle = function(handle) local v = CLIB.purple_prefs_disconnect_by_handle(handle)  return v end,
	AddPathList = function(name, value) local v = CLIB.purple_prefs_add_path_list(name, table_to_glist(value))  return v end,
	SetInt = function(name, value) local v = CLIB.purple_prefs_set_int(name, value)  return v end,
	GetChildrenNames = function(namecast_type) local v = CLIB.purple_prefs_get_children_names(name) v = glist_to_table(v, cast_type) return v end,
	ConnectCallback = function(handle, name, cb, data) local v = CLIB.purple_prefs_connect_callback(handle, name, cb, data)  return v end,
	GetString = function(name) local v = CLIB.purple_prefs_get_string(name) v = chars_to_string(v) return v end,
	AddStringList = function(name, value) local v = CLIB.purple_prefs_add_string_list(name, table_to_glist(value))  return v end,
	AddBool = function(name, value) local v = CLIB.purple_prefs_add_bool(name, value)  return v end,
	GetStringList = function(namecast_type) local v = CLIB.purple_prefs_get_string_list(name) v = glist_to_table(v, cast_type) return v end,
	Destroy = function() local v = CLIB.purple_prefs_destroy()  return v end,
	DisconnectCallback = function(callback_id) local v = CLIB.purple_prefs_disconnect_callback(callback_id)  return v end,
	Remove = function(name) local v = CLIB.purple_prefs_remove(name)  return v end,
	GetPath = function(name) local v = CLIB.purple_prefs_get_path(name) v = chars_to_string(v) return v end,
	GetHandle = function() local v = CLIB.purple_prefs_get_handle()  return v end,
	GetType = function(name) local v = CLIB.purple_prefs_get_type(name)  return v end,
	Uninit = function() local v = CLIB.purple_prefs_uninit()  return v end,
	SetBool = function(name, value) local v = CLIB.purple_prefs_set_bool(name, value)  return v end,
}
library.signals = {
	Init = function() local v = CLIB.purple_signals_init()  return v end,
	Uninit = function() local v = CLIB.purple_signals_uninit()  return v end,
	UnregisterByInstance = function(instance) local v = CLIB.purple_signals_unregister_by_instance(instance)  return v end,
	DisconnectByHandle = function(handle) local v = CLIB.purple_signals_disconnect_by_handle(handle)  return v end,
}
library.account = {
	RequestClose = function(ui_handle) local v = CLIB.purple_account_request_close(ui_handle)  return v end,
	OptionStringNew = function(text, pref_name, default_value) local v = CLIB.purple_account_option_string_new(text, pref_name, default_value) v = wrap_pointer(v, "AccountOption") return v end,
	OptionIntNew = function(text, pref_name, default_value) local v = CLIB.purple_account_option_int_new(text, pref_name, default_value) v = wrap_pointer(v, "AccountOption") return v end,
	New = function(username, protocol_id) local v = CLIB.purple_account_new(username, protocol_id) v = wrap_pointer(v, "Account") return v end,
	UserSplitNew = function(text, default_value, sep) local v = CLIB.purple_account_user_split_new(text, default_value, sep) v = wrap_pointer(v, "AccountUserSplit") return v end,
	OptionListNew = function(text, pref_name, list) local v = CLIB.purple_account_option_list_new(text, pref_name, table_to_glist(list)) v = wrap_pointer(v, "AccountOption") return v end,
	OptionNew = function(type, text, pref_name) local v = CLIB.purple_account_option_new(type, text, pref_name) v = wrap_pointer(v, "AccountOption") return v end,
	OptionBoolNew = function(text, pref_name, default_value) local v = CLIB.purple_account_option_bool_new(text, pref_name, default_value) v = wrap_pointer(v, "AccountOption") return v end,
}
library.signal = {
	EmitVargs = function(instance, signal, args) local v = CLIB.purple_signal_emit_vargs(instance, signal, args)  return v end,
	Unregister = function(instance, signal) local v = CLIB.purple_signal_unregister(instance, signal)  return v end,
	GetValues = function(instance, signal, ret_value, num_values, values) local v = CLIB.purple_signal_get_values(instance, signal, ret_value.ptr, num_values, values.ptr)  return v end,
	Emit = function(instance, signal, _3) local v = CLIB.purple_signal_emit(instance, signal, _3)  return v end,
	Connect = function(instance, signal, handle, func, data) local v = CLIB.purple_signal_connect(instance, signal, handle, func, data)  return v end,
	EmitVargsReturn_1 = function(instance, signal, args) local v = CLIB.purple_signal_emit_vargs_return_1(instance, signal, args)  return v end,
	ConnectVargs = function(instance, signal, handle, func, data) local v = CLIB.purple_signal_connect_vargs(instance, signal, handle, func, data)  return v end,
	ConnectPriority = function(instance, signal, handle, func, data, priority) local v = CLIB.purple_signal_connect_priority(instance, signal, handle, func, data, priority)  return v end,
	Register = function(instance, signal, marshal, ret_value, num_values, _6) local v = CLIB.purple_signal_register(instance, signal, marshal, ret_value.ptr, num_values, _6)  return v end,
	Disconnect = function(instance, signal, handle, func) local v = CLIB.purple_signal_disconnect(instance, signal, handle, func)  return v end,
	ConnectPriorityVargs = function(instance, signal, handle, func, data, priority) local v = CLIB.purple_signal_connect_priority_vargs(instance, signal, handle, func, data, priority)  return v end,
	EmitReturn_1 = function(instance, signal, _3) local v = CLIB.purple_signal_emit_return_1(instance, signal, _3)  return v end,
}
library.srv = {
	Cancel = function(query_data) local v = CLIB.purple_srv_cancel(query_data.ptr)  return v end,
	ResolveAccount = function(account, protocol, transport, domain, cb, extradata) local v = CLIB.purple_srv_resolve_account(account.ptr, protocol, transport, domain, cb, extradata) v = wrap_pointer(v, "SrvQueryData") return v end,
	TxtQuerySetUiOps = function(ops) local v = CLIB.purple_srv_txt_query_set_ui_ops(ops)  return v end,
	TxtQueryGetUiOps = function() local v = CLIB.purple_srv_txt_query_get_ui_ops()  return v end,
	Resolve = function(protocol, transport, domain, cb, extradata) local v = CLIB.purple_srv_resolve(protocol, transport, domain, cb, extradata) v = wrap_pointer(v, "SrvQueryData") return v end,
}
library.stringref = {
	NewNoref = function(value) local v = CLIB.purple_stringref_new_noref(value) v = wrap_pointer(v, "Stringref") return v end,
	Printf = function(format, _2) local v = CLIB.purple_stringref_printf(format, _2) v = wrap_pointer(v, "Stringref") return v end,
	New = function(value) local v = CLIB.purple_stringref_new(value) v = wrap_pointer(v, "Stringref") return v end,
}
library.pounces = {
	Init = function() local v = CLIB.purple_pounces_init()  return v end,
	Load = function() local v = CLIB.purple_pounces_load()  return v end,
	GetHandle = function() local v = CLIB.purple_pounces_get_handle()  return v end,
	GetAllForUi = function(uicast_type) local v = CLIB.purple_pounces_get_all_for_ui(ui) v = glist_to_table(v, cast_type) return v end,
	GetAll = function(cast_type) local v = CLIB.purple_pounces_get_all() v = glist_to_table(v, cast_type) return v end,
	UnregisterHandler = function(ui) local v = CLIB.purple_pounces_unregister_handler(ui)  return v end,
	Uninit = function() local v = CLIB.purple_pounces_uninit()  return v end,
	RegisterHandler = function(ui, cb, new_pounce, free_pounce) local v = CLIB.purple_pounces_register_handler(ui, cb, new_pounce, free_pounce)  return v end,
}
library.media = {
	CandidateListCopy = function(candidatescast_type) local v = CLIB.purple_media_candidate_list_copy(table_to_glist(candidates)) v = glist_to_table(v, cast_type) return v end,
	CodecListFree = function(codecs) local v = CLIB.purple_media_codec_list_free(table_to_glist(codecs))  return v end,
	CodecGetType = function() local v = CLIB.purple_media_codec_get_type()  return v end,
	CandidateNew = function(foundation, component_id, type, proto, ip, port) local v = CLIB.purple_media_candidate_new(foundation, component_id, type, proto, ip, port) v = wrap_pointer(v, "MediaCandidate") return v end,
	ManagerGet = function() local v = CLIB.purple_media_manager_get() v = wrap_pointer(v, "MediaManager") return v end,
	InfoTypeGetType = function() local v = CLIB.purple_media_info_type_get_type()  return v end,
	SessionTypeGetType = function() local v = CLIB.purple_media_session_type_get_type()  return v end,
	CandidateGetType = function() local v = CLIB.purple_media_candidate_get_type()  return v end,
	NetworkProtocolGetType = function() local v = CLIB.purple_media_network_protocol_get_type()  return v end,
	CandidateTypeGetType = function() local v = CLIB.purple_media_candidate_type_get_type()  return v end,
	CandidateListFree = function(candidates) local v = CLIB.purple_media_candidate_list_free(table_to_glist(candidates))  return v end,
	CodecNew = function(id, encoding_name, media_type, clock_rate) local v = CLIB.purple_media_codec_new(id, encoding_name, media_type, clock_rate) v = wrap_pointer(v, "MediaCodec") return v end,
	CapsGetType = function() local v = CLIB.purple_media_caps_get_type()  return v end,
	ManagerGetType = function() local v = CLIB.purple_media_manager_get_type()  return v end,
	StateChangedGetType = function() local v = CLIB.purple_media_state_changed_get_type()  return v end,
	CodecListCopy = function(codecscast_type) local v = CLIB.purple_media_codec_list_copy(table_to_glist(codecs)) v = glist_to_table(v, cast_type) return v end,
	GetType = function() local v = CLIB.purple_media_get_type()  return v end,
}
library.presence = {
	NewForBuddy = function(buddy) local v = CLIB.purple_presence_new_for_buddy(buddy.ptr) v = wrap_pointer(v, "Presence") return v end,
	New = function(context) local v = CLIB.purple_presence_new(context) v = wrap_pointer(v, "Presence") return v end,
	NewForConv = function(conv) local v = CLIB.purple_presence_new_for_conv(conv.ptr) v = wrap_pointer(v, "Presence") return v end,
	NewForAccount = function(account) local v = CLIB.purple_presence_new_for_account(account.ptr) v = wrap_pointer(v, "Presence") return v end,
}
library.strdup = {
	Withhtml = function(src) local v = CLIB.purple_strdup_withhtml(src) v = chars_to_string(v) return v end,
}
library.prpl = {
	GotAccountStatus = function(account, status_id, _3) local v = CLIB.purple_prpl_got_account_status(account.ptr, status_id, _3)  return v end,
	GotUserLoginTime = function(account, name, login_time) local v = CLIB.purple_prpl_got_user_login_time(account.ptr, name, login_time)  return v end,
	GotMediaCaps = function(account, who) local v = CLIB.purple_prpl_got_media_caps(account.ptr, who)  return v end,
	GotUserStatus = function(account, name, status_id, _4) local v = CLIB.purple_prpl_got_user_status(account.ptr, name, status_id, _4)  return v end,
	GotAttention = function(gc, who, type_code) local v = CLIB.purple_prpl_got_attention(gc.ptr, who, type_code)  return v end,
	GotAccountIdle = function(account, idle, idle_time) local v = CLIB.purple_prpl_got_account_idle(account.ptr, idle, idle_time)  return v end,
	InitiateMedia = function(account, who, type) local v = CLIB.purple_prpl_initiate_media(account.ptr, who, type)  return v end,
	GetStatuses = function(account, presence, cast_type) local v = CLIB.purple_prpl_get_statuses(account.ptr, presence.ptr) v = glist_to_table(v, cast_type) return v end,
	GotAccountActions = function(account) local v = CLIB.purple_prpl_got_account_actions(account.ptr)  return v end,
	GotAttentionInChat = function(gc, id, who, type_code) local v = CLIB.purple_prpl_got_attention_in_chat(gc.ptr, id, who, type_code)  return v end,
	GotUserIdle = function(account, name, idle, idle_time) local v = CLIB.purple_prpl_got_user_idle(account.ptr, name, idle, idle_time)  return v end,
	SendAttention = function(gc, who, type_code) local v = CLIB.purple_prpl_send_attention(gc.ptr, who, type_code)  return v end,
	GotUserStatusDeactive = function(account, name, status_id) local v = CLIB.purple_prpl_got_user_status_deactive(account.ptr, name, status_id)  return v end,
	GotAccountLoginTime = function(account, login_time) local v = CLIB.purple_prpl_got_account_login_time(account.ptr, login_time)  return v end,
	GetMediaCaps = function(account, who) local v = CLIB.purple_prpl_get_media_caps(account.ptr, who)  return v end,
	ChangeAccountStatus = function(account, old_status, new_status) local v = CLIB.purple_prpl_change_account_status(account.ptr, old_status.ptr, new_status.ptr)  return v end,
}
library.debug = {
	Init = function() local v = CLIB.purple_debug_init()  return v end,
	Misc = function(category, format, _3) local v = CLIB.purple_debug_misc(category, format, _3)  return v end,
	SetVerbose = function(verbose) local v = CLIB.purple_debug_set_verbose(verbose)  return v end,
	IsUnsafe = function() local v = CLIB.purple_debug_is_unsafe()  return v end,
	IsVerbose = function() local v = CLIB.purple_debug_is_verbose()  return v end,
	Info = function(category, format, _3) local v = CLIB.purple_debug_info(category, format, _3)  return v end,
	Warning = function(category, format, _3) local v = CLIB.purple_debug_warning(category, format, _3)  return v end,
	IsEnabled = function() local v = CLIB.purple_debug_is_enabled()  return v end,
	SetUnsafe = function(unsafe) local v = CLIB.purple_debug_set_unsafe(unsafe)  return v end,
	Error = function(category, format, _3) local v = CLIB.purple_debug_error(category, format, _3)  return v end,
	GetUiOps = function() local v = CLIB.purple_debug_get_ui_ops()  return v end,
	SetUiOps = function(ops) local v = CLIB.purple_debug_set_ui_ops(ops)  return v end,
	SetEnabled = function(enabled) local v = CLIB.purple_debug_set_enabled(enabled)  return v end,
	Fatal = function(category, format, _3) local v = CLIB.purple_debug_fatal(category, format, _3)  return v end,
}
library.certificate = {
	Import = function(scheme, filename) local v = CLIB.purple_certificate_import(scheme, filename) v = wrap_pointer(v, "Certificate") return v end,
	Export = function(filename, crt) local v = CLIB.purple_certificate_export(filename, crt.ptr)  return v end,
	GetVerifiers = function(cast_type) local v = CLIB.purple_certificate_get_verifiers() v = glist_to_table(v, cast_type) return v end,
	AddCaSearchPath = function(path) local v = CLIB.purple_certificate_add_ca_search_path(path)  return v end,
	FindPool = function(scheme_name, pool_name) local v = CLIB.purple_certificate_find_pool(scheme_name, pool_name) v = wrap_pointer(v, "CertificatePool") return v end,
	RegisterScheme = function(scheme) local v = CLIB.purple_certificate_register_scheme(scheme)  return v end,
	CheckSignatureChainWithFailing = function(chain, failing) local v = CLIB.purple_certificate_check_signature_chain_with_failing(table_to_glist(chain), failing.ptr)  return v end,
	GetSchemes = function(cast_type) local v = CLIB.purple_certificate_get_schemes() v = glist_to_table(v, cast_type) return v end,
	GetPools = function(cast_type) local v = CLIB.purple_certificate_get_pools() v = glist_to_table(v, cast_type) return v end,
	Init = function() local v = CLIB.purple_certificate_init()  return v end,
	RegisterPool = function(pool) local v = CLIB.purple_certificate_register_pool(pool.ptr)  return v end,
	DestroyList = function(crt_list) local v = CLIB.purple_certificate_destroy_list(table_to_glist(crt_list))  return v end,
	PoolDestroyIdlist = function(idlist) local v = CLIB.purple_certificate_pool_destroy_idlist(table_to_glist(idlist))  return v end,
	RegisterVerifier = function(vr) local v = CLIB.purple_certificate_register_verifier(vr)  return v end,
	UnregisterScheme = function(scheme) local v = CLIB.purple_certificate_unregister_scheme(scheme)  return v end,
	FindScheme = function(name) local v = CLIB.purple_certificate_find_scheme(name)  return v end,
	UnregisterVerifier = function(vr) local v = CLIB.purple_certificate_unregister_verifier(vr)  return v end,
	CopyList = function(crt_listcast_type) local v = CLIB.purple_certificate_copy_list(table_to_glist(crt_list)) v = glist_to_table(v, cast_type) return v end,
	Verify = function(verifier, subject_name, cert_chain, cb, cb_data) local v = CLIB.purple_certificate_verify(verifier, subject_name, table_to_glist(cert_chain), cb, cb_data)  return v end,
	CheckSignatureChain = function(chain) local v = CLIB.purple_certificate_check_signature_chain(table_to_glist(chain))  return v end,
	VerifyComplete = function(vrq, st) local v = CLIB.purple_certificate_verify_complete(vrq, st)  return v end,
	GetHandle = function() local v = CLIB.purple_certificate_get_handle()  return v end,
	UnregisterPool = function(pool) local v = CLIB.purple_certificate_unregister_pool(pool.ptr)  return v end,
	Uninit = function() local v = CLIB.purple_certificate_uninit()  return v end,
	FindVerifier = function(scheme_name, ver_name) local v = CLIB.purple_certificate_find_verifier(scheme_name, ver_name)  return v end,
}
library.unescape = {
	Filename = function(str) local v = CLIB.purple_unescape_filename(str) v = chars_to_string(v) return v end,
	Html = function(html) local v = CLIB.purple_unescape_html(html) v = chars_to_string(v) return v end,
	Text = function(text) local v = CLIB.purple_unescape_text(text) v = chars_to_string(v) return v end,
}
library.sound = {
	Init = function() local v = CLIB.purple_sound_init()  return v end,
	ThemeGetType = function() local v = CLIB.purple_sound_theme_get_type()  return v end,
	PlayEvent = function(event, account) local v = CLIB.purple_sound_play_event(event, account.ptr)  return v end,
	PlayFile = function(filename, account) local v = CLIB.purple_sound_play_file(filename, account.ptr)  return v end,
	GetUiOps = function() local v = CLIB.purple_sound_get_ui_ops()  return v end,
	SetUiOps = function(ops) local v = CLIB.purple_sound_set_ui_ops(ops)  return v end,
	Uninit = function() local v = CLIB.purple_sound_uninit()  return v end,
	ThemeLoaderGetType = function() local v = CLIB.purple_sound_theme_loader_get_type()  return v end,
}
library.savedstatuses = {
	Init = function() local v = CLIB.purple_savedstatuses_init()  return v end,
	GetHandle = function() local v = CLIB.purple_savedstatuses_get_handle()  return v end,
	GetAll = function(cast_type) local v = CLIB.purple_savedstatuses_get_all() v = glist_to_table(v, cast_type) return v end,
	Uninit = function() local v = CLIB.purple_savedstatuses_uninit()  return v end,
	GetPopular = function(how_manycast_type) local v = CLIB.purple_savedstatuses_get_popular(how_many) v = glist_to_table(v, cast_type) return v end,
}
library.plugins = {
	RegisterLoadNotifyCb = function(func, data) local v = CLIB.purple_plugins_register_load_notify_cb(func, data)  return v end,
	GetSearchPaths = function(cast_type) local v = CLIB.purple_plugins_get_search_paths() v = glist_to_table(v, cast_type) return v end,
	Probe = function(ext) local v = CLIB.purple_plugins_probe(ext)  return v end,
	RegisterProbeNotifyCb = function(func, data) local v = CLIB.purple_plugins_register_probe_notify_cb(func, data)  return v end,
	FindWithBasename = function(basename) local v = CLIB.purple_plugins_find_with_basename(basename) v = wrap_pointer(v, "Plugin") return v end,
	UnloadAll = function() local v = CLIB.purple_plugins_unload_all()  return v end,
	FindWithId = function(id) local v = CLIB.purple_plugins_find_with_id(id) v = wrap_pointer(v, "Plugin") return v end,
	UnregisterLoadNotifyCb = function(func) local v = CLIB.purple_plugins_unregister_load_notify_cb(func)  return v end,
	FindWithName = function(name) local v = CLIB.purple_plugins_find_with_name(name) v = wrap_pointer(v, "Plugin") return v end,
	Init = function() local v = CLIB.purple_plugins_init()  return v end,
	GetAll = function(cast_type) local v = CLIB.purple_plugins_get_all() v = glist_to_table(v, cast_type) return v end,
	FindWithFilename = function(filename) local v = CLIB.purple_plugins_find_with_filename(filename) v = wrap_pointer(v, "Plugin") return v end,
	DestroyAll = function() local v = CLIB.purple_plugins_destroy_all()  return v end,
	Unload = function(type) local v = CLIB.purple_plugins_unload(type)  return v end,
	SaveLoaded = function(key) local v = CLIB.purple_plugins_save_loaded(key)  return v end,
	GetLoaded = function(cast_type) local v = CLIB.purple_plugins_get_loaded() v = glist_to_table(v, cast_type) return v end,
	Enabled = function() local v = CLIB.purple_plugins_enabled()  return v end,
	LoadSaved = function(key) local v = CLIB.purple_plugins_load_saved(key)  return v end,
	RegisterUnloadNotifyCb = function(func, data) local v = CLIB.purple_plugins_register_unload_notify_cb(func, data)  return v end,
	UnregisterUnloadNotifyCb = function(func) local v = CLIB.purple_plugins_unregister_unload_notify_cb(func)  return v end,
	UnregisterProbeNotifyCb = function(func) local v = CLIB.purple_plugins_unregister_probe_notify_cb(func)  return v end,
	GetHandle = function() local v = CLIB.purple_plugins_get_handle()  return v end,
	GetProtocols = function(cast_type) local v = CLIB.purple_plugins_get_protocols() v = glist_to_table(v, cast_type) return v end,
	Uninit = function() local v = CLIB.purple_plugins_uninit()  return v end,
	AddSearchPath = function(path) local v = CLIB.purple_plugins_add_search_path(path)  return v end,
}
library.primitive = {
	GetTypeFromId = function(id) local v = CLIB.purple_primitive_get_type_from_id(id)  return v end,
	GetIdFromType = function(type) local v = CLIB.purple_primitive_get_id_from_type(type) v = chars_to_string(v) return v end,
	GetNameFromType = function(type) local v = CLIB.purple_primitive_get_name_from_type(type) v = chars_to_string(v) return v end,
}
library.xfers = {
	Init = function() local v = CLIB.purple_xfers_init()  return v end,
	SetUiOps = function(ops) local v = CLIB.purple_xfers_set_ui_ops(ops)  return v end,
	GetUiOps = function() local v = CLIB.purple_xfers_get_ui_ops()  return v end,
	GetHandle = function() local v = CLIB.purple_xfers_get_handle()  return v end,
	Uninit = function() local v = CLIB.purple_xfers_uninit()  return v end,
	GetAll = function(cast_type) local v = CLIB.purple_xfers_get_all() v = glist_to_table(v, cast_type) return v end,
}
library.user = {
	Dir = function() local v = CLIB.purple_user_dir() v = chars_to_string(v) return v end,
}
library.email = {
	IsValid = function(address) local v = CLIB.purple_email_is_valid(address)  return v end,
}
library.timeout = {
	AddSeconds = function(interval, function_, data) local v = CLIB.purple_timeout_add_seconds(interval, function_, data)  return v end,
	Remove = function(handle) local v = CLIB.purple_timeout_remove(handle)  return v end,
	Add = function(interval, function_, data) local v = CLIB.purple_timeout_add(interval, function_, data)  return v end,
}
library.proxy = {
	Init = function() local v = CLIB.purple_proxy_init()  return v end,
	ConnectSocks5Account = function(handle, account, gpi, host, port, connect_cb, data) local v = CLIB.purple_proxy_connect_socks5_account(handle, account.ptr, gpi.ptr, host, port, connect_cb, data)  return v end,
	ConnectSocks5 = function(handle, gpi, host, port, connect_cb, data) local v = CLIB.purple_proxy_connect_socks5(handle, gpi.ptr, host, port, connect_cb, data)  return v end,
	Connect = function(handle, account, host, port, connect_cb, data) local v = CLIB.purple_proxy_connect(handle, account.ptr, host, port, connect_cb, data)  return v end,
	GetHandle = function() local v = CLIB.purple_proxy_get_handle()  return v end,
	ConnectCancelWithHandle = function(handle) local v = CLIB.purple_proxy_connect_cancel_with_handle(handle)  return v end,
	InfoNew = function() local v = CLIB.purple_proxy_info_new() v = wrap_pointer(v, "ProxyInfo") return v end,
	ConnectUdp = function(handle, account, host, port, connect_cb, data) local v = CLIB.purple_proxy_connect_udp(handle, account.ptr, host, port, connect_cb, data)  return v end,
	GetSetup = function(account) local v = CLIB.purple_proxy_get_setup(account.ptr) v = wrap_pointer(v, "ProxyInfo") return v end,
	Uninit = function() local v = CLIB.purple_proxy_uninit()  return v end,
	ConnectCancel = function(connect_data) local v = CLIB.purple_proxy_connect_cancel(connect_data)  return v end,
}
library.purple = {
	Strequal = function(left, right) local v = CLIB.purple_strequal(left, right)  return v end,
	Strreplace = function(string, delimiter, replacement) local v = CLIB.purple_strreplace(string, delimiter, replacement) v = chars_to_string(v) return v end,
	Mkstemp = function(path, binary) local v = CLIB.purple_mkstemp(path, binary)  return v end,
	Debug = function(level, category, format, _4) local v = CLIB.purple_debug(level, category, format, _4)  return v end,
	Strcasestr = function(haystack, needle) local v = CLIB.purple_strcasestr(haystack, needle) v = chars_to_string(v) return v end,
	Strcasereplace = function(string, delimiter, replacement) local v = CLIB.purple_strcasereplace(string, delimiter, replacement) v = chars_to_string(v) return v end,
	Normalize = function(account, str) local v = CLIB.purple_normalize(account.ptr, str) v = chars_to_string(v) return v end,
}
library.request = {
	Input = function(handle, title, primary, secondary, default_value, multiline, masked, hint, ok_text, ok_cb, cancel_text, cancel_cb, account, who, conv, user_data) local v = CLIB.purple_request_input(handle, title, primary, secondary, default_value, multiline, masked, hint, ok_text, ok_cb, cancel_text, cancel_cb, account.ptr, who, conv.ptr, user_data)  return v end,
	FieldLabelNew = function(id, text) local v = CLIB.purple_request_field_label_new(id, text) v = wrap_pointer(v, "RequestField") return v end,
	Choice = function(handle, title, primary, secondary, default_value, ok_text, ok_cb, cancel_text, cancel_cb, account, who, conv, user_data, _14) local v = CLIB.purple_request_choice(handle, title, primary, secondary, default_value, ok_text, ok_cb, cancel_text, cancel_cb, account.ptr, who, conv.ptr, user_data, _14)  return v end,
	FieldAccountNew = function(id, text, account) local v = CLIB.purple_request_field_account_new(id, text, account.ptr) v = wrap_pointer(v, "RequestField") return v end,
	ActionVarg = function(handle, title, primary, secondary, default_action, account, who, conv, user_data, action_count, actions) local v = CLIB.purple_request_action_varg(handle, title, primary, secondary, default_action, account.ptr, who, conv.ptr, user_data, action_count, actions)  return v end,
	GetUiOps = function() local v = CLIB.purple_request_get_ui_ops()  return v end,
	Action = function(handle, title, primary, secondary, default_action, account, who, conv, user_data, action_count, _11) local v = CLIB.purple_request_action(handle, title, primary, secondary, default_action, account.ptr, who, conv.ptr, user_data, action_count, _11)  return v end,
	FieldBoolNew = function(id, text, default_value) local v = CLIB.purple_request_field_bool_new(id, text, default_value) v = wrap_pointer(v, "RequestField") return v end,
	FieldsNew = function() local v = CLIB.purple_request_fields_new() v = wrap_pointer(v, "RequestFields") return v end,
	Folder = function(handle, title, dirname, ok_cb, cancel_cb, account, who, conv, user_data) local v = CLIB.purple_request_folder(handle, title, dirname, ok_cb, cancel_cb, account.ptr, who, conv.ptr, user_data)  return v end,
	FieldIntNew = function(id, text, default_value) local v = CLIB.purple_request_field_int_new(id, text, default_value) v = wrap_pointer(v, "RequestField") return v end,
	FieldGroupNew = function(title) local v = CLIB.purple_request_field_group_new(title) v = wrap_pointer(v, "RequestFieldGroup") return v end,
	FieldNew = function(id, text, type) local v = CLIB.purple_request_field_new(id, text, type) v = wrap_pointer(v, "RequestField") return v end,
	FieldStringNew = function(id, text, default_value, multiline) local v = CLIB.purple_request_field_string_new(id, text, default_value, multiline) v = wrap_pointer(v, "RequestField") return v end,
	FieldListNew = function(id, text) local v = CLIB.purple_request_field_list_new(id, text) v = wrap_pointer(v, "RequestField") return v end,
	FieldChoiceNew = function(id, text, default_value) local v = CLIB.purple_request_field_choice_new(id, text, default_value) v = wrap_pointer(v, "RequestField") return v end,
	ChoiceVarg = function(handle, title, primary, secondary, default_value, ok_text, ok_cb, cancel_text, cancel_cb, account, who, conv, user_data, choices) local v = CLIB.purple_request_choice_varg(handle, title, primary, secondary, default_value, ok_text, ok_cb, cancel_text, cancel_cb, account.ptr, who, conv.ptr, user_data, choices)  return v end,
	SetUiOps = function(ops) local v = CLIB.purple_request_set_ui_ops(ops)  return v end,
	ActionWithIconVarg = function(handle, title, primary, secondary, default_action, account, who, conv, icon_data, icon_size, user_data, action_count, actions) local v = CLIB.purple_request_action_with_icon_varg(handle, title, primary, secondary, default_action, account.ptr, who, conv.ptr, icon_data, icon_size, user_data, action_count, actions)  return v end,
	Fields = function(handle, title, primary, secondary, fields, ok_text, ok_cb, cancel_text, cancel_cb, account, who, conv, user_data) local v = CLIB.purple_request_fields(handle, title, primary, secondary, fields.ptr, ok_text, ok_cb, cancel_text, cancel_cb, account.ptr, who, conv.ptr, user_data)  return v end,
	CloseWithHandle = function(handle) local v = CLIB.purple_request_close_with_handle(handle)  return v end,
	File = function(handle, title, filename, savedialog, ok_cb, cancel_cb, account, who, conv, user_data) local v = CLIB.purple_request_file(handle, title, filename, savedialog, ok_cb, cancel_cb, account.ptr, who, conv.ptr, user_data)  return v end,
	Close = function(type, uihandle) local v = CLIB.purple_request_close(type, uihandle)  return v end,
	FieldImageNew = function(id, text, buf, size) local v = CLIB.purple_request_field_image_new(id, text, buf, size) v = wrap_pointer(v, "RequestField") return v end,
	ActionWithIcon = function(handle, title, primary, secondary, default_action, account, who, conv, icon_data, icon_size, user_data, action_count, _13) local v = CLIB.purple_request_action_with_icon(handle, title, primary, secondary, default_action, account.ptr, who, conv.ptr, icon_data, icon_size, user_data, action_count, _13)  return v end,
}
library.date = {
	FormatLong = function(tm) local v = CLIB.purple_date_format_long(tm) v = chars_to_string(v) return v end,
	FormatFull = function(tm) local v = CLIB.purple_date_format_full(tm) v = chars_to_string(v) return v end,
	FormatShort = function(tm) local v = CLIB.purple_date_format_short(tm) v = chars_to_string(v) return v end,
}
library.savedstatus = {
	GetDefault = function() local v = CLIB.purple_savedstatus_get_default() v = wrap_pointer(v, "SavedStatus") return v end,
	New = function(title, type) local v = CLIB.purple_savedstatus_new(title, type) v = wrap_pointer(v, "SavedStatus") return v end,
	GetStartup = function() local v = CLIB.purple_savedstatus_get_startup() v = wrap_pointer(v, "SavedStatus") return v end,
	FindByCreationTime = function(creation_time) local v = CLIB.purple_savedstatus_find_by_creation_time(creation_time) v = wrap_pointer(v, "SavedStatus") return v end,
	Delete = function(title) local v = CLIB.purple_savedstatus_delete(title)  return v end,
	SubstatusGetType = function(substatus) local v = CLIB.purple_savedstatus_substatus_get_type(substatus) v = wrap_pointer(v, "StatusType") return v end,
	GetCurrent = function() local v = CLIB.purple_savedstatus_get_current() v = wrap_pointer(v, "SavedStatus") return v end,
	FindTransientByTypeAndMessage = function(type, message) local v = CLIB.purple_savedstatus_find_transient_by_type_and_message(type, message) v = wrap_pointer(v, "SavedStatus") return v end,
	Find = function(title) local v = CLIB.purple_savedstatus_find(title) v = wrap_pointer(v, "SavedStatus") return v end,
	SubstatusGetMessage = function(substatus) local v = CLIB.purple_savedstatus_substatus_get_message(substatus) v = chars_to_string(v) return v end,
	GetIdleaway = function() local v = CLIB.purple_savedstatus_get_idleaway() v = wrap_pointer(v, "SavedStatus") return v end,
	IsIdleaway = function() local v = CLIB.purple_savedstatus_is_idleaway()  return v end,
	SetIdleaway = function(idleaway) local v = CLIB.purple_savedstatus_set_idleaway(idleaway)  return v end,
}
library.smiley = {
	New = function(img, shortcut) local v = CLIB.purple_smiley_new(img.ptr, shortcut) v = wrap_pointer(v, "Smiley") return v end,
	NewFromFile = function(shortcut, filepath) local v = CLIB.purple_smiley_new_from_file(shortcut, filepath) v = wrap_pointer(v, "Smiley") return v end,
	GetType = function() local v = CLIB.purple_smiley_get_type()  return v end,
}
library.idle = {
	Touch = function() local v = CLIB.purple_idle_touch()  return v end,
	Set = function(time) local v = CLIB.purple_idle_set(time)  return v end,
	GetUiOps = function() local v = CLIB.purple_idle_get_ui_ops()  return v end,
	SetUiOps = function(ops) local v = CLIB.purple_idle_set_ui_ops(ops)  return v end,
	Uninit = function() local v = CLIB.purple_idle_uninit()  return v end,
	Init = function() local v = CLIB.purple_idle_init()  return v end,
}
library.running = {
	Osx = function() local v = CLIB.purple_running_osx()  return v end,
	Kde = function() local v = CLIB.purple_running_kde()  return v end,
	Gnome = function() local v = CLIB.purple_running_gnome()  return v end,
}
library.get = {
	Conversations = function(cast_type) local v = CLIB.purple_get_conversations() v = glist_to_table(v, cast_type) return v end,
	HostName = function() local v = CLIB.purple_get_host_name() v = chars_to_string(v) return v end,
	Chats = function(cast_type) local v = CLIB.purple_get_chats() v = glist_to_table(v, cast_type) return v end,
	Core = function() local v = CLIB.purple_get_core()  return v end,
	Blist = function() local v = CLIB.purple_get_blist()  return v end,
	Ims = function(cast_type) local v = CLIB.purple_get_ims() v = glist_to_table(v, cast_type) return v end,
	TzoffStr = function(tm, iso) local v = CLIB.purple_get_tzoff_str(tm, iso) v = chars_to_string(v) return v end,
	AttentionTypeFromCode = function(account, type_code) local v = CLIB.purple_get_attention_type_from_code(account.ptr, type_code) v = wrap_pointer(v, "AttentionType") return v end,
}
local callbacks = {}
callbacks["received-chat-msg"] = {
	wrap = function(callback, _1, _2, _3, _4, _5) local ret = callback(wrap_pointer(_1, "Account"), chars_to_string(_2), chars_to_string(_3), wrap_pointer(_4, "Conversation"), _5 --[[unsigned int]] ) return ret end,
	definition = "void ( * ) ( struct _PurpleAccount * , const char * , const char * , struct _PurpleConversation * , unsigned int )",
}
callbacks["buddy-signed-on"] = {
	wrap = function(callback, _1) local ret = callback(wrap_pointer(_1, "Buddy")) return ret end,
	definition = "void ( * ) ( struct _PurpleBuddy * )",
}
callbacks["blocked-im-msg"] = {
	wrap = function(callback, _1, _2, _3, _4, _5) local ret = callback(wrap_pointer(_1, "Account"), chars_to_string(_2), chars_to_string(_3), _4 --[[unsigned int]] , _5 --[[unsigned int]] ) return ret end,
	definition = "void ( * ) ( struct _PurpleAccount * , const char * , const char * , unsigned int , unsigned int )",
}
callbacks["chat-buddy-left"] = {
	wrap = function(callback, _1, _2, _3) local ret = callback(wrap_pointer(_1, "Conversation"), chars_to_string(_2), chars_to_string(_3)) return ret end,
	definition = "void ( * ) ( struct _PurpleConversation * , const char * , const char * )",
}
callbacks["dbus-introspect"] = {
	wrap = function(callback, _1) local ret = callback(_1 --[[void * *]] ) return ret end,
	definition = "void ( * ) ( void * * )",
}
callbacks["writing-chat-msg"] = {
	wrap = function(callback, _1, _2, _3, _4, _5) local ret = callback(wrap_pointer(_1, "Account"), chars_to_string(_2), create_boxed_table(_3, function(p, v) replace_buffer(p, #v, v) end, function(p) return ffi.string(p[0]) end), wrap_pointer(_4, "Conversation"), _5 --[[unsigned int]] )  if ret == nil then return ffi.new("bool") end return ret end,
	definition = "bool ( * ) ( struct _PurpleAccount * , const char * , char * * , struct _PurpleConversation * , unsigned int )",
}
callbacks["file-recv-request"] = {
	wrap = function(callback, _1) local ret = callback(wrap_pointer(_1, "Xfer")) return ret end,
	definition = "void ( * ) ( struct _PurpleXfer * )",
}
callbacks["plugin-unload"] = {
	wrap = function(callback, _1) local ret = callback(wrap_pointer(_1, "Plugin")) return ret end,
	definition = "void ( * ) ( struct _PurplePlugin * )",
}
callbacks["displaying-im-msg"] = {
	wrap = function(callback, _1, _2, _3, _4, _5) local ret = callback(wrap_pointer(_1, "Account"), chars_to_string(_2), create_boxed_table(_3, function(p, v) replace_buffer(p, #v, v) end, function(p) return ffi.string(p[0]) end), wrap_pointer(_4, "Conversation"), _5 --[[int]] )  if ret == nil then return ffi.new("bool") end return ret end,
	definition = "bool ( * ) ( struct _PurpleAccount * , const char * , char * * , struct _PurpleConversation * , int )",
}
callbacks["playing-sound-event"] = {
	wrap = function(callback, _1, _2) local ret = callback(_1 --[[int]] , wrap_pointer(_2, "Account"))  if ret == nil then return ffi.new("bool") end return ret end,
	definition = "bool ( * ) ( int , struct _PurpleAccount * )",
}
callbacks["cleared-message-history"] = {
	wrap = function(callback, _1) local ret = callback(wrap_pointer(_1, "Conversation")) return ret end,
	definition = "void ( * ) ( struct _PurpleConversation * )",
}
callbacks["savedstatus-deleted"] = {
	wrap = function(callback, _1) local ret = callback(wrap_pointer(_1, "SavedStatus")) return ret end,
	definition = "void ( * ) ( struct _PurpleSavedStatus * )",
}
callbacks["buddy-typing"] = {
	wrap = function(callback, _1, _2) local ret = callback(wrap_pointer(_1, "Account"), chars_to_string(_2)) return ret end,
	definition = "void ( * ) ( struct _PurpleAccount * , const char * )",
}
callbacks["displaying-email-notification"] = {
	wrap = function(callback, _1, _2, _3, _4) local ret = callback(chars_to_string(_1), chars_to_string(_2), chars_to_string(_3), chars_to_string(_4)) return ret end,
	definition = "void ( * ) ( const char * , const char * , const char * , const char * )",
}
callbacks["file-recv-complete"] = {
	wrap = function(callback, _1) local ret = callback(wrap_pointer(_1, "Xfer")) return ret end,
	definition = "void ( * ) ( struct _PurpleXfer * )",
}
callbacks["buddy-idle-changed"] = {
	wrap = function(callback, _1, _2, _3) local ret = callback(wrap_pointer(_1, "Buddy"), _2 --[[int]] , _3 --[[int]] ) return ret end,
	definition = "void ( * ) ( struct _PurpleBuddy * , int , int )",
}
callbacks["chat-invited"] = {
	wrap = function(callback, _1, _2, _3, _4, _5) local ret = callback(wrap_pointer(_1, "Account"), chars_to_string(_2), chars_to_string(_3), chars_to_string(_4), _5 --[[void *]] )  if ret == nil then return ffi.new("int") end return ret end,
	definition = "int ( * ) ( struct _PurpleAccount * , const char * , const char * , const char * , void * )",
}
callbacks["cipher-added"] = {
	wrap = function(callback, _1) local ret = callback(wrap_pointer(_1, "Cipher")) return ret end,
	definition = "void ( * ) ( struct _PurpleCipher * )",
}
callbacks["chat-invited-user"] = {
	wrap = function(callback, _1, _2, _3) local ret = callback(wrap_pointer(_1, "Conversation"), chars_to_string(_2), chars_to_string(_3)) return ret end,
	definition = "void ( * ) ( struct _PurpleConversation * , const char * , const char * )",
}
callbacks["plugin-load"] = {
	wrap = function(callback, _1) local ret = callback(wrap_pointer(_1, "Plugin")) return ret end,
	definition = "void ( * ) ( struct _PurplePlugin * )",
}
callbacks["sending-im-msg"] = {
	wrap = function(callback, _1, _2, _3) local ret = callback(wrap_pointer(_1, "Account"), chars_to_string(_2), create_boxed_table(_3, function(p, v) replace_buffer(p, #v, v) end, function(p) return ffi.string(p[0]) end)) return ret end,
	definition = "void ( * ) ( struct _PurpleAccount * , const char * , char * * )",
}
callbacks["blist-node-extended-menu"] = {
	wrap = function(callback, _1, _2) local ret = callback(wrap_pointer(_1, "BlistNode"), _2 --[[struct _GList * *]] ) return ret end,
	definition = "void ( * ) ( struct _PurpleBlistNode * , struct _GList * * )",
}
callbacks["writing-im-msg"] = {
	wrap = function(callback, _1, _2, _3, _4, _5) local ret = callback(wrap_pointer(_1, "Account"), chars_to_string(_2), create_boxed_table(_3, function(p, v) replace_buffer(p, #v, v) end, function(p) return ffi.string(p[0]) end), wrap_pointer(_4, "Conversation"), _5 --[[unsigned int]] )  if ret == nil then return ffi.new("bool") end return ret end,
	definition = "bool ( * ) ( struct _PurpleAccount * , const char * , char * * , struct _PurpleConversation * , unsigned int )",
}
callbacks["chat-inviting-user"] = {
	wrap = function(callback, _1, _2, _3) local ret = callback(wrap_pointer(_1, "Conversation"), chars_to_string(_2), create_boxed_table(_3, function(p, v) replace_buffer(p, #v, v) end, function(p) return ffi.string(p[0]) end)) return ret end,
	definition = "void ( * ) ( struct _PurpleConversation * , const char * , char * * )",
}
callbacks["buddy-typed"] = {
	wrap = function(callback, _1, _2) local ret = callback(wrap_pointer(_1, "Account"), chars_to_string(_2)) return ret end,
	definition = "void ( * ) ( struct _PurpleAccount * , const char * )",
}
callbacks["buddy-icon-changed"] = {
	wrap = function(callback, _1) local ret = callback(wrap_pointer(_1, "Buddy")) return ret end,
	definition = "void ( * ) ( struct _PurpleBuddy * )",
}
callbacks["quitting"] = {
	wrap = function(callback) local ret = callback() return ret end,
	definition = "void ( * ) (  )",
}
callbacks["buddy-privacy-changed"] = {
	wrap = function(callback, _1) local ret = callback(wrap_pointer(_1, "Buddy")) return ret end,
	definition = "void ( * ) ( struct _PurpleBuddy * )",
}
callbacks["wrote-chat-msg"] = {
	wrap = function(callback, _1, _2, _3, _4, _5) local ret = callback(wrap_pointer(_1, "Account"), chars_to_string(_2), chars_to_string(_3), wrap_pointer(_4, "Conversation"), _5 --[[unsigned int]] ) return ret end,
	definition = "void ( * ) ( struct _PurpleAccount * , const char * , const char * , struct _PurpleConversation * , unsigned int )",
}
callbacks["buddy-typing-stopped"] = {
	wrap = function(callback, _1, _2) local ret = callback(wrap_pointer(_1, "Account"), chars_to_string(_2)) return ret end,
	definition = "void ( * ) ( struct _PurpleAccount * , const char * )",
}
callbacks["cmd-added"] = {
	wrap = function(callback, _1, _2, _3) local ret = callback(chars_to_string(_1), _2 --[[int]] , _3 --[[int]] ) return ret end,
	definition = "void ( * ) ( const char * , int , int )",
}
callbacks["dbus-method-called"] = {
	wrap = function(callback, _1, _2) local ret = callback(_1 --[[void *]] , _2 --[[void *]] )  if ret == nil then return ffi.new("bool") end return ret end,
	definition = "bool ( * ) ( void * , void * )",
}
callbacks["file-send-complete"] = {
	wrap = function(callback, _1) local ret = callback(wrap_pointer(_1, "Xfer")) return ret end,
	definition = "void ( * ) ( struct _PurpleXfer * )",
}
callbacks["file-recv-accept"] = {
	wrap = function(callback, _1) local ret = callback(wrap_pointer(_1, "Xfer")) return ret end,
	definition = "void ( * ) ( struct _PurpleXfer * )",
}
callbacks["file-send-start"] = {
	wrap = function(callback, _1) local ret = callback(wrap_pointer(_1, "Xfer")) return ret end,
	definition = "void ( * ) ( struct _PurpleXfer * )",
}
callbacks["log-timestamp"] = {
	wrap = function(callback, _1, _2, _3, _4) local ret = callback(wrap_pointer(_1, "Log"), _2 --[[int]] , _3 --[[int]] , _4 --[[bool]] )  if ret == nil then return ffi.new("char") end return ret end,
	definition = "const char * ( * ) ( struct _PurpleLog * , int , int , bool )",
}
callbacks["buddy-status-changed"] = {
	wrap = function(callback, _1, _2, _3) local ret = callback(wrap_pointer(_1, "Buddy"), wrap_pointer(_2, "Status"), wrap_pointer(_3, "Status")) return ret end,
	definition = "void ( * ) ( struct _PurpleBuddy * , struct _PurpleStatus * , struct _PurpleStatus * )",
}
callbacks["receiving-im-msg"] = {
	wrap = function(callback, _1, _2, _3, _4, _5) local ret = callback(wrap_pointer(_1, "Account"), create_boxed_table(_2, function(p, v) replace_buffer(p, #v, v) end, function(p) return ffi.string(p[0]) end), create_boxed_table(_3, function(p, v) replace_buffer(p, #v, v) end, function(p) return ffi.string(p[0]) end), wrap_pointer(_4, "Conversation"), _5 --[[unsigned int *]] )  if ret == nil then return ffi.new("bool") end return ret end,
	definition = "bool ( * ) ( struct _PurpleAccount * , char * * , char * * , struct _PurpleConversation * , unsigned int * )",
}
callbacks["chat-topic-changed"] = {
	wrap = function(callback, _1, _2, _3) local ret = callback(wrap_pointer(_1, "Conversation"), chars_to_string(_2), chars_to_string(_3)) return ret end,
	definition = "void ( * ) ( struct _PurpleConversation * , const char * , const char * )",
}
callbacks["certificate-stored"] = {
	wrap = function(callback, _1, _2) local ret = callback(wrap_pointer(_1, "CertificatePool"), chars_to_string(_2)) return ret end,
	definition = "void ( * ) ( struct _PurpleCertificatePool * , const char * )",
}
callbacks["sent-im-msg"] = {
	wrap = function(callback, _1, _2, _3) local ret = callback(wrap_pointer(_1, "Account"), chars_to_string(_2), chars_to_string(_3)) return ret end,
	definition = "void ( * ) ( struct _PurpleAccount * , const char * , const char * )",
}
callbacks["conversation-timestamp"] = {
	wrap = function(callback, _1, _2, _3, _4) local ret = callback(wrap_pointer(_1, "Conversation"), _2 --[[int]] , _3 --[[int]] , _4 --[[bool]] )  if ret == nil then return ffi.new("char") end return ret end,
	definition = "const char * ( * ) ( struct _PurpleConversation * , int , int , bool )",
}
callbacks["update-idle"] = {
	wrap = function(callback) local ret = callback() return ret end,
	definition = "void ( * ) (  )",
}
callbacks["deleting-conversation"] = {
	wrap = function(callback, _1) local ret = callback(wrap_pointer(_1, "Conversation")) return ret end,
	definition = "void ( * ) ( struct _PurpleConversation * )",
}
callbacks["chat-nick-clicked"] = {
	wrap = function(callback, _1, _2, _3) local ret = callback(wrap_pointer(_1, "Conversation"), chars_to_string(_2), _3 --[[unsigned int]] )  if ret == nil then return ffi.new("bool") end return ret end,
	definition = "bool ( * ) ( struct _PurpleConversation * , const char * , unsigned int )",
}
callbacks["chat-nick-autocomplete"] = {
	wrap = function(callback, _1) local ret = callback(wrap_pointer(_1, "Conversation"))  if ret == nil then return ffi.new("bool") end return ret end,
	definition = "bool ( * ) ( struct _PurpleConversation * )",
}
callbacks["chat-buddy-flags"] = {
	wrap = function(callback, _1, _2, _3, _4) local ret = callback(wrap_pointer(_1, "Conversation"), chars_to_string(_2), _3 --[[unsigned int]] , _4 --[[unsigned int]] ) return ret end,
	definition = "void ( * ) ( struct _PurpleConversation * , const char * , unsigned int , unsigned int )",
}
callbacks["buddy-removed"] = {
	wrap = function(callback, _1) local ret = callback(wrap_pointer(_1, "Buddy")) return ret end,
	definition = "void ( * ) ( struct _PurpleBuddy * )",
}
callbacks["conversation-switched"] = {
	wrap = function(callback, _1) local ret = callback(wrap_pointer(_1, "Conversation")) return ret end,
	definition = "void ( * ) ( struct _PurpleConversation * )",
}
callbacks["displayed-chat-msg"] = {
	wrap = function(callback, _1, _2, _3, _4, _5) local ret = callback(wrap_pointer(_1, "Account"), chars_to_string(_2), chars_to_string(_3), wrap_pointer(_4, "Conversation"), _5 --[[int]] ) return ret end,
	definition = "void ( * ) ( struct _PurpleAccount * , const char * , const char * , struct _PurpleConversation * , int )",
}
callbacks["sent-chat-msg"] = {
	wrap = function(callback, _1, _2, _3) local ret = callback(wrap_pointer(_1, "Account"), chars_to_string(_2), _3 --[[unsigned int]] ) return ret end,
	definition = "void ( * ) ( struct _PurpleAccount * , const char * , unsigned int )",
}
callbacks["savedstatus-modified"] = {
	wrap = function(callback, _1) local ret = callback(wrap_pointer(_1, "SavedStatus")) return ret end,
	definition = "void ( * ) ( struct _PurpleSavedStatus * )",
}
callbacks["chat-invite-blocked"] = {
	wrap = function(callback, _1, _2, _3, _4, _5) local ret = callback(wrap_pointer(_1, "Account"), chars_to_string(_2), chars_to_string(_3), chars_to_string(_4), _5 --[[struct _GHashTable *]] ) return ret end,
	definition = "void ( * ) ( struct _PurpleAccount * , const char * , const char * , const char * , struct _GHashTable * )",
}
callbacks["displayed-im-msg"] = {
	wrap = function(callback, _1, _2, _3, _4, _5) local ret = callback(wrap_pointer(_1, "Account"), chars_to_string(_2), chars_to_string(_3), wrap_pointer(_4, "Conversation"), _5 --[[int]] ) return ret end,
	definition = "void ( * ) ( struct _PurpleAccount * , const char * , const char * , struct _PurpleConversation * , int )",
}
callbacks["blist-node-aliased"] = {
	wrap = function(callback, _1, _2) local ret = callback(wrap_pointer(_1, "BlistNode"), chars_to_string(_2)) return ret end,
	definition = "void ( * ) ( struct _PurpleBlistNode * , const char * )",
}
callbacks["chat-join-failed"] = {
	wrap = function(callback, _1, _2) local ret = callback(wrap_pointer(_1, "Connection"), _2 --[[void *]] ) return ret end,
	definition = "void ( * ) ( struct _PurpleConnection * , void * )",
}
callbacks["displaying-chat-msg"] = {
	wrap = function(callback, _1, _2, _3, _4, _5) local ret = callback(wrap_pointer(_1, "Account"), chars_to_string(_2), create_boxed_table(_3, function(p, v) replace_buffer(p, #v, v) end, function(p) return ffi.string(p[0]) end), wrap_pointer(_4, "Conversation"), _5 --[[int]] )  if ret == nil then return ffi.new("bool") end return ret end,
	definition = "bool ( * ) ( struct _PurpleAccount * , const char * , char * * , struct _PurpleConversation * , int )",
}
callbacks["drawing-buddy"] = {
	wrap = function(callback, _1) local ret = callback(wrap_pointer(_1, "Buddy"))  if ret == nil then return ffi.new("char") end return ret end,
	definition = "const char * ( * ) ( struct _PurpleBuddy * )",
}
callbacks["drawing-tooltip"] = {
	wrap = function(callback, _1, _2, _3) local ret = callback(wrap_pointer(_1, "BlistNode"), _2 --[[struct _GString *]] , _3 --[[bool]] ) return ret end,
	definition = "void ( * ) ( struct _PurpleBlistNode * , struct _GString * , bool )",
}
callbacks["gtkblist-hiding"] = {
	wrap = function(callback, _1) local ret = callback(_1 --[[struct _PurpleBuddyList *]] ) return ret end,
	definition = "void ( * ) ( struct _PurpleBuddyList * )",
}
callbacks["displaying-emails-notification"] = {
	wrap = function(callback, _1, _2, _3, _4, _5) local ret = callback(_1 --[[void *]] , _2 --[[void *]] , _3 --[[void *]] , _4 --[[void *]] , _5 --[[unsigned int]] ) return ret end,
	definition = "void ( * ) ( void * , void * , void * , void * , unsigned int )",
}
callbacks["wrote-im-msg"] = {
	wrap = function(callback, _1, _2, _3, _4, _5) local ret = callback(wrap_pointer(_1, "Account"), chars_to_string(_2), chars_to_string(_3), wrap_pointer(_4, "Conversation"), _5 --[[unsigned int]] ) return ret end,
	definition = "void ( * ) ( struct _PurpleAccount * , const char * , const char * , struct _PurpleConversation * , unsigned int )",
}
callbacks["gtkblist-unhiding"] = {
	wrap = function(callback, _1) local ret = callback(_1 --[[struct _PurpleBuddyList *]] ) return ret end,
	definition = "void ( * ) ( struct _PurpleBuddyList * )",
}
callbacks["image-deleting"] = {
	wrap = function(callback, _1) local ret = callback(wrap_pointer(_1, "StoredImage")) return ret end,
	definition = "void ( * ) ( struct _PurpleStoredImage * )",
}
callbacks["file-recv-start"] = {
	wrap = function(callback, _1) local ret = callback(wrap_pointer(_1, "Xfer")) return ret end,
	definition = "void ( * ) ( struct _PurpleXfer * )",
}
callbacks["account-modified"] = {
	wrap = function(callback, _1) local ret = callback(wrap_pointer(_1, "Account")) return ret end,
	definition = "void ( * ) ( struct _PurpleAccount * )",
}
callbacks["chat-buddy-joining"] = {
	wrap = function(callback, _1, _2, _3) local ret = callback(wrap_pointer(_1, "Conversation"), chars_to_string(_2), _3 --[[unsigned int]] )  if ret == nil then return ffi.new("bool") end return ret end,
	definition = "bool ( * ) ( struct _PurpleConversation * , const char * , unsigned int )",
}
callbacks["savedstatus-added"] = {
	wrap = function(callback, _1) local ret = callback(wrap_pointer(_1, "SavedStatus")) return ret end,
	definition = "void ( * ) ( struct _PurpleSavedStatus * )",
}
callbacks["savedstatus-changed"] = {
	wrap = function(callback, _1, _2) local ret = callback(wrap_pointer(_1, "SavedStatus"), wrap_pointer(_2, "SavedStatus")) return ret end,
	definition = "void ( * ) ( struct _PurpleSavedStatus * , struct _PurpleSavedStatus * )",
}
callbacks["cipher-removed"] = {
	wrap = function(callback, _1) local ret = callback(wrap_pointer(_1, "Cipher")) return ret end,
	definition = "void ( * ) ( struct _PurpleCipher * )",
}
callbacks["conversation-extended-menu"] = {
	wrap = function(callback, _1, _2) local ret = callback(wrap_pointer(_1, "Conversation"), _2 --[[struct _GList * *]] ) return ret end,
	definition = "void ( * ) ( struct _PurpleConversation * , struct _GList * * )",
}
callbacks["deleting-chat-buddy"] = {
	wrap = function(callback, _1) local ret = callback(_1 --[[struct _PurpleConvChatBuddy *]] ) return ret end,
	definition = "void ( * ) ( struct _PurpleConvChatBuddy * )",
}
callbacks["displaying-userinfo"] = {
	wrap = function(callback, _1, _2, _3) local ret = callback(wrap_pointer(_1, "Account"), chars_to_string(_2), wrap_pointer(_3, "NotifyUserInfo")) return ret end,
	definition = "void ( * ) ( struct _PurpleAccount * , const char * , struct _PurpleNotifyUserInfo * )",
}
callbacks["gtkblist-created"] = {
	wrap = function(callback, _1) local ret = callback(_1 --[[struct _PurpleBuddyList *]] ) return ret end,
	definition = "void ( * ) ( struct _PurpleBuddyList * )",
}
callbacks["cmd-removed"] = {
	wrap = function(callback, _1) local ret = callback(chars_to_string(_1)) return ret end,
	definition = "void ( * ) ( const char * )",
}
callbacks["buddy-added"] = {
	wrap = function(callback, _1) local ret = callback(wrap_pointer(_1, "Buddy")) return ret end,
	definition = "void ( * ) ( struct _PurpleBuddy * )",
}
callbacks["file-send-cancel"] = {
	wrap = function(callback, _1) local ret = callback(wrap_pointer(_1, "Xfer")) return ret end,
	definition = "void ( * ) ( struct _PurpleXfer * )",
}
callbacks["conversation-updated"] = {
	wrap = function(callback, _1, _2) local ret = callback(wrap_pointer(_1, "Conversation"), _2 --[[unsigned int]] ) return ret end,
	definition = "void ( * ) ( struct _PurpleConversation * , unsigned int )",
}
callbacks["received-im-msg"] = {
	wrap = function(callback, _1, _2, _3, _4, _5) local ret = callback(wrap_pointer(_1, "Account"), chars_to_string(_2), chars_to_string(_3), wrap_pointer(_4, "Conversation"), _5 --[[unsigned int]] ) return ret end,
	definition = "void ( * ) ( struct _PurpleAccount * , const char * , const char * , struct _PurpleConversation * , unsigned int )",
}
callbacks["chat-joined"] = {
	wrap = function(callback, _1) local ret = callback(wrap_pointer(_1, "Conversation")) return ret end,
	definition = "void ( * ) ( struct _PurpleConversation * )",
}
callbacks["buddy-caps-changed"] = {
	wrap = function(callback, _1, _2, _3) local ret = callback(wrap_pointer(_1, "Buddy"), _2 --[[int]] , _3 --[[int]] ) return ret end,
	definition = "void ( * ) ( struct _PurpleBuddy * , int , int )",
}
callbacks["buddy-signed-off"] = {
	wrap = function(callback, _1) local ret = callback(wrap_pointer(_1, "Buddy")) return ret end,
	definition = "void ( * ) ( struct _PurpleBuddy * )",
}
callbacks["certificate-deleted"] = {
	wrap = function(callback, _1, _2) local ret = callback(wrap_pointer(_1, "CertificatePool"), chars_to_string(_2)) return ret end,
	definition = "void ( * ) ( struct _PurpleCertificatePool * , const char * )",
}
callbacks["chat-buddy-leaving"] = {
	wrap = function(callback, _1, _2, _3) local ret = callback(wrap_pointer(_1, "Conversation"), chars_to_string(_2), chars_to_string(_3))  if ret == nil then return ffi.new("bool") end return ret end,
	definition = "bool ( * ) ( struct _PurpleConversation * , const char * , const char * )",
}
callbacks["receiving-chat-msg"] = {
	wrap = function(callback, _1, _2, _3, _4, _5) local ret = callback(wrap_pointer(_1, "Account"), create_boxed_table(_2, function(p, v) replace_buffer(p, #v, v) end, function(p) return ffi.string(p[0]) end), create_boxed_table(_3, function(p, v) replace_buffer(p, #v, v) end, function(p) return ffi.string(p[0]) end), wrap_pointer(_4, "Conversation"), _5 --[[unsigned int *]] )  if ret == nil then return ffi.new("bool") end return ret end,
	definition = "bool ( * ) ( struct _PurpleAccount * , char * * , char * * , struct _PurpleConversation * , unsigned int * )",
}
callbacks["file-recv-cancel"] = {
	wrap = function(callback, _1) local ret = callback(wrap_pointer(_1, "Xfer")) return ret end,
	definition = "void ( * ) ( struct _PurpleXfer * )",
}
callbacks["sending-chat-msg"] = {
	wrap = function(callback, _1, _2, _3) local ret = callback(wrap_pointer(_1, "Account"), create_boxed_table(_2, function(p, v) replace_buffer(p, #v, v) end, function(p) return ffi.string(p[0]) end), _3 --[[unsigned int]] ) return ret end,
	definition = "void ( * ) ( struct _PurpleAccount * , char * * , unsigned int )",
}
callbacks["sent-attention"] = {
	wrap = function(callback, _1, _2, _3, _4) local ret = callback(wrap_pointer(_1, "Account"), chars_to_string(_2), wrap_pointer(_3, "Conversation"), _4 --[[unsigned int]] ) return ret end,
	definition = "void ( * ) ( struct _PurpleAccount * , const char * , struct _PurpleConversation * , unsigned int )",
}
callbacks["blist-node-added"] = {
	wrap = function(callback, _1) local ret = callback(wrap_pointer(_1, "BlistNode")) return ret end,
	definition = "void ( * ) ( struct _PurpleBlistNode * )",
}
callbacks["file-send-accept"] = {
	wrap = function(callback, _1) local ret = callback(wrap_pointer(_1, "Xfer")) return ret end,
	definition = "void ( * ) ( struct _PurpleXfer * )",
}
callbacks["uri-handler"] = {
	wrap = function(callback, _1, _2, _3) local ret = callback(chars_to_string(_1), chars_to_string(_2), _3 --[[struct _GHashTable *]] )  if ret == nil then return ffi.new("bool") end return ret end,
	definition = "bool ( * ) ( const char * , const char * , struct _GHashTable * )",
}
callbacks["conversation-created"] = {
	wrap = function(callback, _1) local ret = callback(wrap_pointer(_1, "Conversation")) return ret end,
	definition = "void ( * ) ( struct _PurpleConversation * )",
}
callbacks["chat-buddy-joined"] = {
	wrap = function(callback, _1, _2, _3, _4) local ret = callback(wrap_pointer(_1, "Conversation"), chars_to_string(_2), _3 --[[unsigned int]] , _4 --[[bool]] ) return ret end,
	definition = "void ( * ) ( struct _PurpleConversation * , const char * , unsigned int , bool )",
}
callbacks["blist-node-removed"] = {
	wrap = function(callback, _1) local ret = callback(wrap_pointer(_1, "BlistNode")) return ret end,
	definition = "void ( * ) ( struct _PurpleBlistNode * )",
}
callbacks["chat-left"] = {
	wrap = function(callback, _1) local ret = callback(wrap_pointer(_1, "Conversation")) return ret end,
	definition = "void ( * ) ( struct _PurpleConversation * )",
}
callbacks["got-attention"] = {
	wrap = function(callback, _1, _2, _3, _4) local ret = callback(wrap_pointer(_1, "Account"), chars_to_string(_2), wrap_pointer(_3, "Conversation"), _4 --[[unsigned int]] ) return ret end,
	definition = "void ( * ) ( struct _PurpleAccount * , const char * , struct _PurpleConversation * , unsigned int )",
}
callbacks["buddy-got-login-time"] = {
	wrap = function(callback, _1) local ret = callback(wrap_pointer(_1, "Buddy")) return ret end,
	definition = "void ( * ) ( struct _PurpleBuddy * )",
}
library.signal.Connect_real = library.signal.Connect

function library.signal.Connect(signal, callback)
	if not library.handles then error("unable to find handles (use purple.set_handles(plugin_handle, conversation_handle))", 2) end

	local info = callbacks[signal]

	if not info then error(signal .. " is not a valid signal!") end

	return library.signal.Connect_real(library.handles.conversation, signal, library.handles.plugin, ffi.cast("void(*)()", ffi.cast(info.definition, function(...)
		local ok, ret = pcall(info.wrap, callback, ...)

		if not ok then
			library.notify.Message(library.handles.plugin, "notify_msg_info", "lua error", signal, ret, nil, nil)
			return default_value
		end

		return ret
	end)), nil)
end

function library.set_handles(plugin, conv)
	library.handles = {
		plugin = ffi.cast("struct _PurplePlugin *", plugin),
		conversation = conv,
	}
end

return library
