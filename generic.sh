#!/bin/sh

if [ "$1" = "clean" ]; then
	rm -rf repo/
	mkdir ../tmp
	mv build.lua ../tmp/
	mv build.sh ../tmp/
	mv .gitignore ../tmp/
	mv readme.md ../tmp/
	rm -rf ./*
	mv ../tmp/* .
	mv ../tmp/.gitignore .
	rm -rf ../tmp
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
