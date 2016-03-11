local ffi = require("ffi")
ffi.cdef([[enum{SF_FORMAT_WAV=0,SF_FORMAT_AIFF=1,SF_FORMAT_AU=2,SF_FORMAT_RAW=3,SF_FORMAT_PAF=4,SF_FORMAT_SVX=5,SF_FORMAT_NIST=6,SF_FORMAT_VOC=7,SF_FORMAT_IRCAM=8,SF_FORMAT_W64=9,SF_FORMAT_MAT4=10,SF_FORMAT_MAT5=11,SF_FORMAT_PVF=12,SF_FORMAT_XI=13,SF_FORMAT_HTK=14,SF_FORMAT_SDS=15,SF_FORMAT_AVR=16,SF_FORMAT_WAVEX=17,SF_FORMAT_SD2=18,SF_FORMAT_FLAC=19,SF_FORMAT_CAF=20,SF_FORMAT_WVE=21,SF_FORMAT_OGG=22,SF_FORMAT_MPC2K=23,SF_FORMAT_RF64=24,SF_FORMAT_PCM_S8=25,SF_FORMAT_PCM_16=26,SF_FORMAT_PCM_24=27,SF_FORMAT_PCM_32=28,SF_FORMAT_PCM_U8=29,SF_FORMAT_FLOAT=30,SF_FORMAT_DOUBLE=31,SF_FORMAT_ULAW=32,SF_FORMAT_ALAW=33,SF_FORMAT_IMA_ADPCM=34,SF_FORMAT_MS_ADPCM=35,SF_FORMAT_GSM610=36,SF_FORMAT_VOX_ADPCM=37,SF_FORMAT_G721_32=38,SF_FORMAT_G723_24=39,SF_FORMAT_G723_40=40,SF_FORMAT_DWVW_12=41,SF_FORMAT_DWVW_16=42,SF_FORMAT_DWVW_24=43,SF_FORMAT_DWVW_N=44,SF_FORMAT_DPCM_8=45,SF_FORMAT_DPCM_16=46,SF_FORMAT_VORBIS=47,SF_FORMAT_ALAC_16=48,SF_FORMAT_ALAC_20=49,SF_FORMAT_ALAC_24=50,SF_FORMAT_ALAC_32=51,SF_ENDIAN_FILE=52,SF_ENDIAN_LITTLE=53,SF_ENDIAN_BIG=54,SF_ENDIAN_CPU=55,SF_FORMAT_SUBMASK=56,SF_FORMAT_TYPEMASK=57,SF_FORMAT_ENDMASK=58,
SFC_GET_LIB_VERSION=0,SFC_GET_LOG_INFO=1,SFC_GET_CURRENT_SF_INFO=2,SFC_GET_NORM_DOUBLE=3,SFC_GET_NORM_FLOAT=4,SFC_SET_NORM_DOUBLE=5,SFC_SET_NORM_FLOAT=6,SFC_SET_SCALE_FLOAT_INT_READ=7,SFC_SET_SCALE_INT_FLOAT_WRITE=8,SFC_GET_SIMPLE_FORMAT_COUNT=9,SFC_GET_SIMPLE_FORMAT=10,SFC_GET_FORMAT_INFO=11,SFC_GET_FORMAT_MAJOR_COUNT=12,SFC_GET_FORMAT_MAJOR=13,SFC_GET_FORMAT_SUBTYPE_COUNT=14,SFC_GET_FORMAT_SUBTYPE=15,SFC_CALC_SIGNAL_MAX=16,SFC_CALC_NORM_SIGNAL_MAX=17,SFC_CALC_MAX_ALL_CHANNELS=18,SFC_CALC_NORM_MAX_ALL_CHANNELS=19,SFC_GET_SIGNAL_MAX=20,SFC_GET_MAX_ALL_CHANNELS=21,SFC_SET_ADD_PEAK_CHUNK=22,SFC_SET_ADD_HEADER_PAD_CHUNK=23,SFC_UPDATE_HEADER_NOW=24,SFC_SET_UPDATE_HEADER_AUTO=25,SFC_FILE_TRUNCATE=26,SFC_SET_RAW_START_OFFSET=27,SFC_SET_DITHER_ON_WRITE=28,SFC_SET_DITHER_ON_READ=29,SFC_GET_DITHER_INFO_COUNT=30,SFC_GET_DITHER_INFO=31,SFC_GET_EMBED_FILE_INFO=32,SFC_SET_CLIPPING=33,SFC_GET_CLIPPING=34,SFC_GET_INSTRUMENT=35,SFC_SET_INSTRUMENT=36,SFC_GET_LOOP_INFO=37,SFC_GET_BROADCAST_INFO=38,SFC_SET_BROADCAST_INFO=39,SFC_GET_CHANNEL_MAP_INFO=40,SFC_SET_CHANNEL_MAP_INFO=41,SFC_RAW_DATA_NEEDS_ENDSWAP=42,SFC_WAVEX_SET_AMBISONIC=43,SFC_WAVEX_GET_AMBISONIC=44,SFC_RF64_AUTO_DOWNGRADE=45,SFC_SET_VBR_ENCODING_QUALITY=46,SFC_SET_COMPRESSION_LEVEL=47,SFC_SET_CART_INFO=48,SFC_GET_CART_INFO=49,SFC_TEST_IEEE_FLOAT_REPLACE=50,SFC_SET_ADD_DITHER_ON_WRITE=51,SFC_SET_ADD_DITHER_ON_READ=52,
SF_STR_TITLE=0,SF_STR_COPYRIGHT=1,SF_STR_SOFTWARE=2,SF_STR_ARTIST=3,SF_STR_COMMENT=4,SF_STR_DATE=5,SF_STR_ALBUM=6,SF_STR_LICENSE=7,SF_STR_TRACKNUMBER=8,SF_STR_GENRE=9,
SF_FALSE=0,SF_TRUE=1,SFM_READ=2,SFM_WRITE=3,SFM_RDWR=4,SF_AMBISONIC_NONE=5,SF_AMBISONIC_B_FORMAT=6,
SF_ERR_NO_ERROR=0,SF_ERR_UNRECOGNISED_FORMAT=1,SF_ERR_SYSTEM=2,SF_ERR_MALFORMED_FILE=3,SF_ERR_UNSUPPORTED_ENCODING=4,
SF_CHANNEL_MAP_INVALID=0,SF_CHANNEL_MAP_MONO=1,SF_CHANNEL_MAP_LEFT=2,SF_CHANNEL_MAP_RIGHT=3,SF_CHANNEL_MAP_CENTER=4,SF_CHANNEL_MAP_FRONT_LEFT=5,SF_CHANNEL_MAP_FRONT_RIGHT=6,SF_CHANNEL_MAP_FRONT_CENTER=7,SF_CHANNEL_MAP_REAR_CENTER=8,SF_CHANNEL_MAP_REAR_LEFT=9,SF_CHANNEL_MAP_REAR_RIGHT=10,SF_CHANNEL_MAP_LFE=11,SF_CHANNEL_MAP_FRONT_LEFT_OF_CENTER=12,SF_CHANNEL_MAP_FRONT_RIGHT_OF_CENTER=13,SF_CHANNEL_MAP_SIDE_LEFT=14,SF_CHANNEL_MAP_SIDE_RIGHT=15,SF_CHANNEL_MAP_TOP_CENTER=16,SF_CHANNEL_MAP_TOP_FRONT_LEFT=17,SF_CHANNEL_MAP_TOP_FRONT_RIGHT=18,SF_CHANNEL_MAP_TOP_FRONT_CENTER=19,SF_CHANNEL_MAP_TOP_REAR_LEFT=20,SF_CHANNEL_MAP_TOP_REAR_RIGHT=21,SF_CHANNEL_MAP_TOP_REAR_CENTER=22,SF_CHANNEL_MAP_AMBISONIC_B_W=23,SF_CHANNEL_MAP_AMBISONIC_B_X=24,SF_CHANNEL_MAP_AMBISONIC_B_Y=25,SF_CHANNEL_MAP_AMBISONIC_B_Z=26,SF_CHANNEL_MAP_MAX=27,
SFD_DEFAULT_LEVEL=0,SFD_CUSTOM_LEVEL=1,SFD_NO_DITHER=2,SFD_WHITE=3,SFD_TRIANGULAR_PDF=4,
SF_LOOP_NONE=0,SF_LOOP_FORWARD=1,SF_LOOP_BACKWARD=2,SF_LOOP_ALTERNATING=3,
SF_SEEK_SET=0,SF_SEEK_CUR=1,SF_SEEK_END=2,};struct SNDFILE_tag {};
struct SF_INFO {long frames;int samplerate;int channels;int format;int sections;int seekable;};
struct SF_VIRTUAL_IO {long(*get_filelen)(void*);long(*seek)(long,int,void*);long(*read)(void*,long,void*);long(*write)(const void*,long,void*);long(*tell)(void*);};
struct SF_CHUNK_INFO {char id[64];unsigned int id_size;unsigned int datalen;void*data;};
struct SF_CHUNK_ITERATOR {};
long(sf_readf_int)(struct SNDFILE_tag*,int*,long);
struct SNDFILE_tag*(sf_open_virtual)(struct SF_VIRTUAL_IO*,int,struct SF_INFO*,void*);
int(sf_get_chunk_data)(const struct SF_CHUNK_ITERATOR*,struct SF_CHUNK_INFO*);
int(sf_format_check)(const struct SF_INFO*);
long(sf_readf_float)(struct SNDFILE_tag*,float*,long);
struct SF_CHUNK_ITERATOR*(sf_next_chunk_iterator)(struct SF_CHUNK_ITERATOR*);
struct SNDFILE_tag*(sf_open)(const char*,int,struct SF_INFO*);
long(sf_writef_double)(struct SNDFILE_tag*,const double*,long);
long(sf_seek)(struct SNDFILE_tag*,long,int);
long(sf_writef_short)(struct SNDFILE_tag*,const short*,long);
int(sf_set_chunk)(struct SNDFILE_tag*,const struct SF_CHUNK_INFO*);
long(sf_readf_double)(struct SNDFILE_tag*,double*,long);
long(sf_write_float)(struct SNDFILE_tag*,const float*,long);
long(sf_read_short)(struct SNDFILE_tag*,short*,long);
const char*(sf_version_string)();
int(sf_set_string)(struct SNDFILE_tag*,int,const char*);
int(sf_error_str)(struct SNDFILE_tag*,char*,unsigned long);
const char*(sf_strerror)(struct SNDFILE_tag*);
long(sf_writef_float)(struct SNDFILE_tag*,const float*,long);
const char*(sf_error_number)(int);
const char*(sf_get_string)(struct SNDFILE_tag*,int);
long(sf_writef_int)(struct SNDFILE_tag*,const int*,long);
long(sf_write_raw)(struct SNDFILE_tag*,const void*,long);
long(sf_readf_short)(struct SNDFILE_tag*,short*,long);
struct SF_CHUNK_ITERATOR*(sf_get_chunk_iterator)(struct SNDFILE_tag*,const struct SF_CHUNK_INFO*);
long(sf_read_raw)(struct SNDFILE_tag*,void*,long);
int(sf_get_chunk_size)(const struct SF_CHUNK_ITERATOR*,struct SF_CHUNK_INFO*);
void(sf_write_sync)(struct SNDFILE_tag*);
int(sf_close)(struct SNDFILE_tag*);
long(sf_write_double)(struct SNDFILE_tag*,const double*,long);
long(sf_read_double)(struct SNDFILE_tag*,double*,long);
long(sf_read_float)(struct SNDFILE_tag*,float*,long);
long(sf_write_int)(struct SNDFILE_tag*,const int*,long);
long(sf_read_int)(struct SNDFILE_tag*,int*,long);
long(sf_write_short)(struct SNDFILE_tag*,const short*,long);
int(sf_command)(struct SNDFILE_tag*,int,void*,int);
int(sf_perror)(struct SNDFILE_tag*);
int(sf_error)(struct SNDFILE_tag*);
struct SNDFILE_tag*(sf_open_fd)(int,int,struct SF_INFO*,int);
int(sf_current_byterate)(struct SNDFILE_tag*);
]])
local CLIB = ffi.load(_G.FFI_LIB or "sndfile")
local library = {}
library = {
	readf_int = CLIB.sf_readf_int,
	open_virtual = CLIB.sf_open_virtual,
	get_chunk_data = CLIB.sf_get_chunk_data,
	format_check = CLIB.sf_format_check,
	readf_float = CLIB.sf_readf_float,
	next_chunk_iterator = CLIB.sf_next_chunk_iterator,
	open = CLIB.sf_open,
	writef_double = CLIB.sf_writef_double,
	seek = CLIB.sf_seek,
	writef_short = CLIB.sf_writef_short,
	set_chunk = CLIB.sf_set_chunk,
	readf_double = CLIB.sf_readf_double,
	write_float = CLIB.sf_write_float,
	read_short = CLIB.sf_read_short,
	version_string = CLIB.sf_version_string,
	set_string = CLIB.sf_set_string,
	error_str = CLIB.sf_error_str,
	strerror = CLIB.sf_strerror,
	writef_float = CLIB.sf_writef_float,
	error_number = CLIB.sf_error_number,
	get_string = CLIB.sf_get_string,
	writef_int = CLIB.sf_writef_int,
	write_raw = CLIB.sf_write_raw,
	readf_short = CLIB.sf_readf_short,
	get_chunk_iterator = CLIB.sf_get_chunk_iterator,
	read_raw = CLIB.sf_read_raw,
	get_chunk_size = CLIB.sf_get_chunk_size,
	write_sync = CLIB.sf_write_sync,
	close = CLIB.sf_close,
	write_double = CLIB.sf_write_double,
	read_double = CLIB.sf_read_double,
	read_float = CLIB.sf_read_float,
	write_int = CLIB.sf_write_int,
	read_int = CLIB.sf_read_int,
	write_short = CLIB.sf_write_short,
	command = CLIB.sf_command,
	perror = CLIB.sf_perror,
	error = CLIB.sf_error,
	open_fd = CLIB.sf_open_fd,
	current_byterate = CLIB.sf_current_byterate,
}
library.e = {
	FORMAT_WAV = 0,
	FORMAT_AIFF = 1,
	FORMAT_AU = 2,
	FORMAT_RAW = 3,
	FORMAT_PAF = 4,
	FORMAT_SVX = 5,
	FORMAT_NIST = 6,
	FORMAT_VOC = 7,
	FORMAT_IRCAM = 8,
	FORMAT_W64 = 9,
	FORMAT_MAT4 = 10,
	FORMAT_MAT5 = 11,
	FORMAT_PVF = 12,
	FORMAT_XI = 13,
	FORMAT_HTK = 14,
	FORMAT_SDS = 15,
	FORMAT_AVR = 16,
	FORMAT_WAVEX = 17,
	FORMAT_SD2 = 18,
	FORMAT_FLAC = 19,
	FORMAT_CAF = 20,
	FORMAT_WVE = 21,
	FORMAT_OGG = 22,
	FORMAT_MPC2K = 23,
	FORMAT_RF64 = 24,
	FORMAT_PCM_S8 = 25,
	FORMAT_PCM_16 = 26,
	FORMAT_PCM_24 = 27,
	FORMAT_PCM_32 = 28,
	FORMAT_PCM_U8 = 29,
	FORMAT_FLOAT = 30,
	FORMAT_DOUBLE = 31,
	FORMAT_ULAW = 32,
	FORMAT_ALAW = 33,
	FORMAT_IMA_ADPCM = 34,
	FORMAT_MS_ADPCM = 35,
	FORMAT_GSM610 = 36,
	FORMAT_VOX_ADPCM = 37,
	FORMAT_G721_32 = 38,
	FORMAT_G723_24 = 39,
	FORMAT_G723_40 = 40,
	FORMAT_DWVW_12 = 41,
	FORMAT_DWVW_16 = 42,
	FORMAT_DWVW_24 = 43,
	FORMAT_DWVW_N = 44,
	FORMAT_DPCM_8 = 45,
	FORMAT_DPCM_16 = 46,
	FORMAT_VORBIS = 47,
	FORMAT_ALAC_16 = 48,
	FORMAT_ALAC_20 = 49,
	FORMAT_ALAC_24 = 50,
	FORMAT_ALAC_32 = 51,
	ENDIAN_FILE = 52,
	ENDIAN_LITTLE = 53,
	ENDIAN_BIG = 54,
	ENDIAN_CPU = 55,
	FORMAT_SUBMASK = 56,
	FORMAT_TYPEMASK = 57,
	FORMAT_ENDMASK = 58,
	STR_TITLE = 0,
	STR_COPYRIGHT = 1,
	STR_SOFTWARE = 2,
	STR_ARTIST = 3,
	STR_COMMENT = 4,
	STR_DATE = 5,
	STR_ALBUM = 6,
	STR_LICENSE = 7,
	STR_TRACKNUMBER = 8,
	STR_GENRE = 9,
	FALSE = 0,
	TRUE = 1,
	AMBISONIC_NONE = 5,
	AMBISONIC_B_FORMAT = 6,
	ERR_NO_ERROR = 0,
	ERR_UNRECOGNISED_FORMAT = 1,
	ERR_SYSTEM = 2,
	ERR_MALFORMED_FILE = 3,
	ERR_UNSUPPORTED_ENCODING = 4,
	CHANNEL_MAP_INVALID = 0,
	CHANNEL_MAP_MONO = 1,
	CHANNEL_MAP_LEFT = 2,
	CHANNEL_MAP_RIGHT = 3,
	CHANNEL_MAP_CENTER = 4,
	CHANNEL_MAP_FRONT_LEFT = 5,
	CHANNEL_MAP_FRONT_RIGHT = 6,
	CHANNEL_MAP_FRONT_CENTER = 7,
	CHANNEL_MAP_REAR_CENTER = 8,
	CHANNEL_MAP_REAR_LEFT = 9,
	CHANNEL_MAP_REAR_RIGHT = 10,
	CHANNEL_MAP_LFE = 11,
	CHANNEL_MAP_FRONT_LEFT_OF_CENTER = 12,
	CHANNEL_MAP_FRONT_RIGHT_OF_CENTER = 13,
	CHANNEL_MAP_SIDE_LEFT = 14,
	CHANNEL_MAP_SIDE_RIGHT = 15,
	CHANNEL_MAP_TOP_CENTER = 16,
	CHANNEL_MAP_TOP_FRONT_LEFT = 17,
	CHANNEL_MAP_TOP_FRONT_RIGHT = 18,
	CHANNEL_MAP_TOP_FRONT_CENTER = 19,
	CHANNEL_MAP_TOP_REAR_LEFT = 20,
	CHANNEL_MAP_TOP_REAR_RIGHT = 21,
	CHANNEL_MAP_TOP_REAR_CENTER = 22,
	CHANNEL_MAP_AMBISONIC_B_W = 23,
	CHANNEL_MAP_AMBISONIC_B_X = 24,
	CHANNEL_MAP_AMBISONIC_B_Y = 25,
	CHANNEL_MAP_AMBISONIC_B_Z = 26,
	CHANNEL_MAP_MAX = 27,
	LOOP_NONE = 0,
	LOOP_FORWARD = 1,
	LOOP_BACKWARD = 2,
	LOOP_ALTERNATING = 3,
	SEEK_SET = 0,
	SEEK_CUR = 1,
	SEEK_END = 2,
}
library.clib = CLIB
return library
