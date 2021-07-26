# Patches applied by platformio

See this for more info: https://docs.platformio.org/en/latest/projectconf/advanced_scripting.html?highlight=patch%20platform#override-package-files

```
diff .pio\libdeps\i2cHelper-esp32thing_plus\Ethernet\src\Ethernet.h patches\Ethernet.h > patches/fix-esp32-issue-2704.patch    
```

```
git diff --no-index .pio\libdeps\i2cHelper-esp32thing_plus\Ethernet\src\Ethernet.h patches\Ethernet.h > patches/fix-esp32-issue-2704.patch    
```