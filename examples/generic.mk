LUA_DIR = ../../LuaJIT/
LUA_BIN = $(LUA_DIR)src/luajit

all: $(LUA_DIR)
	./$(LUA_BIN) build.lua
	
$(LUA_DIR):
	cd ../../ && make LuaJIT
	
clean:
	rm -f lib*.lua
	rm -rf repo
	rm -f lib*.so
