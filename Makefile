LUA_URL = https://github.com/LuaJIT/LuaJIT
LUA_BRANCH = v2.1
LUA_DIR = LuaJIT

all: examples

examples: $(LUA_DIR)
	cd examples/glfw && make
	cd examples/sdl && make
	cd examples/purple && make
	cd examples/vulkan && make

$(LUA_DIR):
	git clone $(LUA_URL)
	cd $(LUA_DIR); git checkout $(LUA_BRANCH)
	cd $(LUA_DIR); export CFLAGS=-fPIC; make

clean:
	rm -rf $(LUA_DIR)
	cd examples/glfw && make clean
	cd examples/sdl && make clean
	cd examples/purple && make clean
	cd examples/vulkan && make clean