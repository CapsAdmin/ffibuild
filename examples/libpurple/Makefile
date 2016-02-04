PLUGIN_NAME = goura
PLUGIN_DIR = $(HOME)/.purple/plugins
PLUGIN_HEADER = libpurple.lua

LUA_DIR = ../../LuaJIT
LUA_BIN = src/luajit
LUA_CFLAGS = -I$(LUA_DIR)/src
LUA_LIBS = -L$(LUA_DIR)/src -lluajit
LUA_BUILD = build.lua

CC := gcc
PURPLE_CFLAGS = $(shell pkg-config purple --cflags)
PURPLE_LIBS = $(shell pkg-config purple --libs)

all: $(PLUGIN_HEADER)

install: $(PLUGIN_HEADER)
	mkdir -p $(PLUGIN_DIR)
	cp $(PLUGIN_NAME).so $(PLUGIN_DIR)

	mkdir -p $(PLUGIN_DIR)/$(PLUGIN_NAME)
	cp *.lua $(PLUGIN_DIR)/$(PLUGIN_NAME)

$(PLUGIN_HEADER): $(PLUGIN_NAME).so
	./$(LUA_DIR)/$(LUA_BIN) $(LUA_BUILD) > $(PLUGIN_HEADER)

$(PLUGIN_NAME).so: $(PLUGIN_NAME).o
	$(CC) -shared $(PLUGIN_NAME).o -o $(PLUGIN_NAME).so -Wl,--whole-archive $(LUA_LIBS) -Wl,--no-whole-archive $(PURPLE_LIBS)

$(PLUGIN_NAME).o: $(LUA_DIR)
	$(CC) $(CFLAGS) -fPIC -c $(PLUGIN_NAME).c -o $(PLUGIN_NAME).o $(LUA_CFLAGS) $(PURPLE_CFLAGS) -DHAVE_CONFIG_H

$(LUA_DIR):
	cd ../../ && make

clean:
	rm -rf *.o *.c~ *.h~ *.so *.la .libs $(PLUGIN_HEADER)

uninstall:
	rm -rf $(PLUGIN_DIR)/$(PLUGIN_NAME)
	rm -rf $(PLUGIN_DIR)/$(PLUGIN_NAME).so

run:
	make install && pidgin