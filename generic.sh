#!/bin/sh

if [ "$1" = "clean" ]; then
	rm -rf repo/ 2> /dev/null
	mkdir ../tmp 2> /dev/null
	mv build.lua ../tmp/ 2> /dev/null
	mv make.sh ../tmp/ 2> /dev/null
	mv .gitignore ../tmp/ 2> /dev/null
	mv readme.md ../tmp/ 2> /dev/null
	rm -rf ./* 2> /dev/null
	mv ../tmp/* . 2> /dev/null
	mv ../tmp/.gitignore . 2> /dev/null
	rm -rf ../tmp 2> /dev/null
	exit
fi

LUA_DIR=../luajit/
LUA_BIN="${LUA_DIR}repo/src/luajit"
echo $LUA_BIN

if [ ! -f $LUA_BIN ]; then
	pushd $LUA_DIR
	make
	popd
fi

export LD_LIBRARY_PATH="." 
$LUA_BIN build.lua "$*"
