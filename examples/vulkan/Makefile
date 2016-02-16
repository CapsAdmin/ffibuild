OUTPUT_NAME = libvulkan.lua

LUA_DIR = ../../LuaJIT
LUA_BIN = src/luajit
LUA_CFLAGS = -I$(LUA_DIR)/src
LUA_LIBS = -L$(LUA_DIR)/src -lluajit
LUA_BUILD = build.lua

all: $(OUTPUT_NAME)

$(OUTPUT_NAME): $(LUA_DIR)
	./$(LUA_DIR)/$(LUA_BIN) $(LUA_BUILD) > $(OUTPUT_NAME)

$(LUA_DIR):
	cd ../../ && make

clean:
	rm -rf $(OUTPUT_NAME)