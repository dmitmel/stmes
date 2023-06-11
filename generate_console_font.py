# Converts a Terminus font into C code for our own use. Requires files
# downloaded from <https://terminus-font.sourceforge.net> to function.
#
# Some notes on the PSF font format:
# <https://www.win.tue.nl/~aeb/linux/kbd/font-formats-1.html>
# <https://en.wikipedia.org/wiki/PC_Screen_Font>
# <https://wiki.osdev.org/PC_Screen_Font>

import os.path
import sys

terminus_dir = os.path.join(os.path.dirname(__file__), "terminus-font-4.49.1")
src_dir = os.path.join(os.path.dirname(__file__), "src", "stmes", "video")
sys.path.append(os.path.join(terminus_dir, "bin"))

import bdfexp
import fnio

ifs = fnio.InputFileStream(os.path.join(terminus_dir, "ter-u12n.bdf"))
font = ifs.process(bdfexp.Font.read)

font_chars = [char for char in font.chars if 0x20 <= char.code <= 0x7e]

with open(os.path.join(src_dir, "console_font.h"), "w") as file:
  print("#pragma once", file=file)
  print("", file=file)
  print("#ifdef __cplusplus", file=file)
  print('extern "C" {', file=file)
  print("#endif", file=file)
  print("", file=file)
  print("#define CONSOLE_FONT_WIDTH {}".format(font.chars[0].bbx.width), file=file)
  print("#define CONSOLE_FONT_HEIGHT {}".format(font.chars[0].bbx.height), file=file)
  print("#define CONSOLE_FONT_CHARACTERS {}".format(len(font_chars)), file=file)
  print("#define CONSOLE_FONT_BYTES_PER_ROW ((CONSOLE_FONT_WIDTH + 7) / 8)", file=file)
  print(
    "#define CONSOLE_FONT_BYTES_PER_CHAR (CONSOLE_FONT_BYTES_PER_ROW * CONSOLE_FONT_HEIGHT)",
    file=file
  )
  print("", file=file)
  print("extern const unsigned char CONSOLE_FONT_DATA[];", file=file)
  print("", file=file)
  print("#ifdef __cplusplus", file=file)
  print("}", file=file)
  print("#endif", file=file)

with open(os.path.join(src_dir, "console_font.c"), "w") as file:
  print("// This file was generated from the files of the Terminus font, licensed", file=file)
  print("// under the SIL Open Font License v1.1 <https://scripts.sil.org/OFL>", file=file)
  print('#include "console_font.h"', file=file)
  print("", file=file)
  print("const unsigned char CONSOLE_FONT_DATA[] = {", file=file)
  # for char_idx, char in enumerate(font_chars):
  #   if char_idx > 0:
  #     print("", file=file)
  #   print("  /* glyph {!r} */".format(chr(char.code)), file=file)
  #   bytes_per_row = (char.bbx.width + 7) // 8
  #   for y in range(char.bbx.height):
  #     print("  ", end="", file=file)
  #     for byte in char.data[bytes_per_row * y:bytes_per_row * (y + 1)]:
  #       print("0x{:02x}, ".format(byte), end="", file=file)
  #     print("/* ", end="", file=file)
  #     for x in range(char.bbx.width):
  #       byte = char.data[bytes_per_row * y + x // 8]
  #       print("#" if byte & (0x80 >> (x % 8)) != 0 else ".", end="", file=file)
  #     print(" */", end="", file=file)
  #     print("", file=file)
  font_pixels = [[] for _ in range(font.chars[0].bbx.height)]
  for char in font_chars:
    bytes_per_row = (char.bbx.width + 7) // 8
    for y in range(char.bbx.height):
      font_pixels[y].extend(char.data[bytes_per_row * y:bytes_per_row * (y + 1)])
  for pixels_row in font_pixels:
    print("  ", end="", file=file)
    for byte_idx, byte in enumerate(pixels_row):
      if byte_idx > 0:
        print(" ", end="", file=file)
      print("0x{:02x},".format(byte), end="", file=file)
    print("", file=file)
  print("};", file=file)
