#define PLUGIN_NAME "goura"
#define PLUGIN_VERSION "0.1.0"
#define PLUGIN_SUMMARY "lua for pidgin"
#define PLUGIN_DESCRIPTION "place scripts in ~.purple/"PLUGIN_NAME"/"
#define PLUGIN_WEBSITE "https://github.com/CapsAdmin/"PLUGIN_NAME
#define PLUGIN_AUTHOR "CapsAdmin"

#define PLUGIN_DIR ".purple/plugins/"PLUGIN_NAME
#define PLUGIN_MAIN_LUA "main.lua"

#define PLUGIN_ID PLUGIN_NAME"-boopboop"

//Purple plugin
#define PURPLE_PLUGINS
#include <libpurple/purple.h>

//Lua Libraries
#include <lua.h>
#include <lauxlib.h>
#include <lualib.h>

lua_State *L = NULL;

void error(PurplePlugin *plugin, const char *type, const char *reason)
{
	purple_notify_warning(plugin, PLUGIN_NAME, "compile error", lua_tostring(L, -1));
	purple_debug_warning(PLUGIN_NAME " lua error", "%s: %s", type, reason);
}

static gboolean plugin_load(PurplePlugin *plugin)
{
	L = luaL_newstate();
	luaL_openlibs(L);

	char lua_path[512];
	sprintf(lua_path, "%s/%s/%s", g_get_home_dir(), PLUGIN_DIR, PLUGIN_MAIN_LUA);

	if (luaL_loadfile(L, lua_path))
	{
		error(plugin, "compile error", lua_tostring(L, -1));

		lua_close(L);
		return FALSE;
	}

	void *conv = purple_conversations_get_handle();

	lua_pushstring(L, lua_path);
	lua_pushlightuserdata(L, plugin);
	lua_pushlightuserdata(L, conv);

	if (lua_pcall(L, 3, 0, 0))
	{
		error(plugin, "runtime error", lua_tostring(L, -1));

		lua_close(L);
		return FALSE;
	}

	return TRUE;
}

static gboolean plugin_unload(PurplePlugin *plugin)
{
    lua_close(L);
    L = NULL;

	return TRUE;
}

static PurplePluginInfo info = {
    PURPLE_PLUGIN_MAGIC,
    PURPLE_MAJOR_VERSION,
    PURPLE_MINOR_VERSION,
    PURPLE_PLUGIN_STANDARD,
    NULL,
    0,
    NULL,
    PURPLE_PRIORITY_DEFAULT,
    PLUGIN_ID,
    PLUGIN_NAME,
    PLUGIN_VERSION,
    PLUGIN_SUMMARY,
	PLUGIN_DESCRIPTION,
    PLUGIN_AUTHOR,
    PLUGIN_WEBSITE,
    plugin_load,
    plugin_unload,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL
};

static void init_plugin(PurplePlugin * plugin) {}

PURPLE_INIT_PLUGIN(autorespond, init_plugin, info)
