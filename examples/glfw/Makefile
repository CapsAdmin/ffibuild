NAME = glfw
CLONE = git clone https://github.com/glfw/glfw $(NAME)
BUILD = cd $(NAME) && cmake -DBUILD_SHARED_LIBS=1 . && make
COPY = cp $(NAME)/src/libglfw.so.3.2 libglfw.so

LUA_DIR = ../../LuaJIT
LUA_BIN = src/luajit
LUA_CFLAGS = -I$(LUA_DIR)/src
LUA_LIBS = -L$(LUA_DIR)/src -lluajit
LUA_BUILD = build.lua

LUA_FILE = lib$(NAME).lua

all: $(LUA_FILE)

$(LUA_FILE): $(NAME)
	./$(LUA_DIR)/$(LUA_BIN) $(LUA_BUILD) > $(LUA_FILE)
	
$(NAME): $(LUA_DIR)
	$(CLONE)
	$(BUILD)
	$(COPY)

$(LUA_DIR): $(NAME)
	cd ../../ && make
	
clean:
	rm -f $(LUA_FILE) libglfw.so
	rm -rf $(NAME)