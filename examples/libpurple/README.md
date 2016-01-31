Adds a plugin called goura to pidgin and exposes the entire libpurple api to lua. Note that in the upcoming pidgin 3 there's going to be an offical lua api using https://bitbucket.org/gplugin/main

When the plugin is enabled it adds a command "!l" to chat and exposes _G.purple. If you trust a friend to execute code on your computer with !l you can make a group called lua and add them there.

This is just something I personally wanted and so the !l command serves more as example usage. I don't know what else to do with this. Modify main.lua to do what you want.
