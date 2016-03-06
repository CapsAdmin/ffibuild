LUA_DIR = ../../LuaJIT/
LUA_BIN = $(LUA_DIR)src/luajit

all: $(LUA_DIR)
	export LD_LIBRARY_PATH=".:$LD_LIBRARY_PATH" && ./$(LUA_BIN) build.lua
	
$(LUA_DIR):
	cd ../../ && make LuaJIT
	
clean:
	rm -f lib*.lua
	rm -rf repo
	rm -f lib*.so
