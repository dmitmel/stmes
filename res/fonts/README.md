The `xorg` directory contains rendered versions of (some) public domain fonts taken from the X.org project: <https://gitlab.freedesktop.org/xorg/font/misc-misc>. The `5x8` font in this directory is an edited version of the X.org font with the same dimensions, mixed with `5x7`.

Some useful font-related links:

- Other fonts:
  - <https://terminus-font.sourceforge.net> - this one was used previously, but the smallest size it comes in is 6x12
  - <https://github.com/olikraus/u8g2/wiki/fntlistall> - an insanely large collection of bitmap fonts
  - <https://github.com/wezm/profont> - looks really nice, some small details were taken from here
  - <https://docs.rs/embedded-graphics/latest/embedded_graphics/mono_font/index.html> - this gave me the idea of using the Xorg fonts
    - <https://github.com/embedded-graphics/embedded-graphics/blob/embedded-graphics-v0.8.0/tools/bdf-to-mono/src/lib.rs>
  - <https://docs.rs/embedded-graphics/latest/embedded_graphics/mono_font/iso_8859_5/index.html> - the cyrillic character subsets
- The PSF format:
  - <https://www.win.tue.nl/~aeb/linux/kbd/font-formats-1.html>
  - <https://en.wikipedia.org/wiki/PC_Screen_Font>
  - <https://wiki.osdev.org/PC_Screen_Font>
- The PCF format:
  - <https://fontforge.org/docs/techref/pcf-format.html>
