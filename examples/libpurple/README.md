Adds a plugin called goura to pidgin and exposes the entire libpurple api to lua. Note that in the upcoming pidgin 3 there's going to be an offical lua api using https://bitbucket.org/gplugin/main

When the plugin is enabled it adds a lua command !l and !print to chat and exposes the global library purple. If you want your friends to execute code you need to make a group called lua and add them there.
This is just something I personally wanted and serves as example code because I don't know what else to do with this. Modify main.lua to do what you want.
