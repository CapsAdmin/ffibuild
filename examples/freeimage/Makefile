NAME = freeimage
CLONE = wget -qO- -O $(NAME).tar.gz http://freeimage.cvs.sourceforge.net/viewvc/freeimage/?view=tar && tar xf $(NAME).tar.gz && rm $(NAME).tar.gz
BUILD = cd $(NAME)/FreeImage && make
COPY = cp $(NAME)/FreeImage/libfreeimage-3.18.0.so lib$(NAME).so

LUA_DIR = ../../LuaJIT
LUA_BIN = src/luajit
LUA_CFLAGS = -I$(LUA_DIR)/src
LUA_LIBS = -L$(LUA_DIR)/src -lluajit
LUA_BUILD = build.lua

LUA_FILE = lib$(NAME).lua

all: $(LUA_FILE)

$(LUA_FILE): $(NAME)
	./$(LUA_DIR)/$(LUA_BIN) $(LUA_BUILD) > $(LUA_FILE)
	
$(NAME)/: $(LUA_DIR)
	$(CLONE)
	$(BUILD)
	$(COPY)

$(LUA_DIR): $(NAME)
	cd ../../ && make
	
clean:
	rm -f $(LUA_FILE) lib$(NAME).so
	rm -rf $(NAME)
