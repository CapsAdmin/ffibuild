LUA_URL = https://github.com/LuaJIT/LuaJIT
LUA_BRANCH = v2.1
LUA_DIR = LuaJIT

SUBDIRS := $(wildcard ./examples/*)

all: $(LUA_DIR)
	for dir in $(SUBDIRS); do \
		$(MAKE) -C $$dir; \
	done

$(LUA_DIR):
	git clone $(LUA_URL)
	cd $(LUA_DIR); git checkout $(LUA_BRANCH)
	cd $(LUA_DIR); export CFLAGS=-fPIC; make

clean:
	rm -rf $(LUA_DIR)
	-for dir in $(SUBDIRS); do \
		$(MAKE) clean -C $$dir; \
	done
