from pathlib import Path

from PIL import Image

project_dir = Path(__file__).resolve().parent
src_dir = project_dir / "src"
res_dir = project_dir / "res"


def compile_font(char_width: int, char_height: int, image_path: Path, output_path: Path) -> None:
  image_path_rel = image_path.relative_to(project_dir)
  img = Image.open(image_path).convert("L")
  font_chars = (img.width // char_width) * (img.height // char_height)
  bytes_per_row = -(char_width // -8)

  with open(output_path.with_suffix(".h"), "w") as file:
    print(f"// This file was generated from {image_path_rel}", file=file)
    print("#pragma once", file=file)
    print("", file=file)
    print("#ifdef __cplusplus", file=file)
    print('extern "C" {', file=file)
    print("#endif", file=file)
    print("", file=file)
    print(f"#define CONSOLE_FONT_WIDTH {char_width}", file=file)
    print(f"#define CONSOLE_FONT_HEIGHT {char_height}", file=file)
    print(f"#define CONSOLE_FONT_CHARACTERS {font_chars}", file=file)
    print(f"#define CONSOLE_FONT_BYTES_PER_ROW {bytes_per_row}", file=file)
    print(f"#define CONSOLE_FONT_BYTES_PER_CHAR {bytes_per_row * char_height}", file=file)
    print("", file=file)
    print("extern const unsigned char CONSOLE_FONT_DATA[];", file=file)
    print("", file=file)
    print("#ifdef __cplusplus", file=file)
    print("}", file=file)
    print("#endif", file=file)

  with open(output_path.with_suffix(".c"), "w") as file:
    print(f"// This file was generated from {image_path_rel}", file=file)
    print('#include "console_font.h"', file=file)
    print("", file=file)
    print("const unsigned char CONSOLE_FONT_DATA[] = {", file=file)
    for y in range(char_height):
      print("  ", end="", file=file)
      for char in range(font_chars):
        chars_per_row = img.width // char_width
        char_x = (char % chars_per_row) * char_width
        char_y = (char // chars_per_row) * char_height
        byte = 0
        for x in range(char_width):
          if img.getpixel((char_x + x, char_y + y)) >= 0x80:
            byte |= 1 << (7 - x)
        if char > 0:
          print(" ", end="", file=file)
        print("0x{:02x},".format(byte), end="", file=file)
      print("", file=file)
    print("};", file=file)


compile_font(5, 8, res_dir / "fonts" / "5x8.png", src_dir / "stmes" / "video" / "console_font")
