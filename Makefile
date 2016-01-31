LUA_URL = https://github.com/LuaJIT/LuaJIT
LUA_BRANCH = v2.1
LUA_DIR = LuaJIT

all:
	git clone $(LUA_URL)
	cd $(LUA_DIR); git checkout $(LUA_BRANCH)
	cd $(LUA_DIR); export CFLAGS=-fPIC; make

clean:
	rm -rf $(LUA_DIR)