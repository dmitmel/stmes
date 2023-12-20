import contextlib
import re
import struct
import sys
from pathlib import Path
from typing import IO, Any, TypedDict

from PIL import Image

project_dir = Path(__file__).resolve().parent
terminus_dir = project_dir / "terminus-font-4.49.1"
xorg_fonts_dir = project_dir / "misc-misc-font-misc-misc-1.1.3"
res_dir = project_dir / "res"

sys.path.append(str(terminus_dir / "bin"))
import bdfexp
import fnio


def struct_read(file: IO[bytes], fmt: str) -> tuple[Any, ...]:
  return struct.unpack(fmt, file.read(struct.calcsize(fmt)))


class ParsedFont(TypedDict):
  char_width: int
  char_height: int
  chars: list[bytes]


def parse_bdf_font(path: str) -> ParsedFont:
  with contextlib.closing(fnio.InputFileStream(path)) as ifs:
    font = ifs.process(bdfexp.Font.read)

  return {
    "char_width": font.chars[0].bbx.width,
    "char_height": font.chars[0].bbx.height,
    "chars": [char.data for char in font.chars if 0x20 <= char.code <= 0x7e],
  }


def parse_psf_font(path: str) -> ParsedFont:
  with open(path, "rb") as f:

    def parse_psf_header() -> tuple[int, int, int]:
      magic1 = f.read(2)
      if magic1 == b"\x36\x04":
        flags, glyph_height = struct_read(f, "<BB")
        glyphs = 512 if flags & 1 != 0 else 256
        glyph_width = 8
        return glyphs, glyph_width, glyph_height

      magic2 = f.read(2)
      if magic1 + magic2 == b"\x72\xb5\x4a\x86":
        version, header_size, flags, glyphs, bytes_per_char, glyph_height, glyph_width = struct_read(
          f, "<IIIIIII"
        )
        return glyphs, glyph_width, glyph_height

      raise Exception("unknown format")

    glyphs_num, glyph_width, glyph_height = parse_psf_header()
    glyphs: list[bytes] = []
    for _ in range(glyphs_num):
      glyphs.append(f.read(-(glyph_width // -8) * glyph_height))

    return {
      "char_width": glyph_width,
      "char_height": glyph_height,
      "chars": glyphs,
    }


def render_font(data: ParsedFont) -> Image.Image:
  char_width = data["char_width"]
  char_height = data["char_height"]
  font_chars = data["chars"]

  chars_per_row = 16
  img = Image.new(
    "L", (char_width * chars_per_row, -(len(font_chars) // -chars_per_row) * char_height)
  )

  for i, char in enumerate(font_chars):
    char_x, char_y = (i % chars_per_row) * char_width, (i // chars_per_row) * char_height
    bytes_per_row = -(char_width // -8)
    for y in range(char_height):
      for x in range(char_width):
        pixel = char[bytes_per_row * y + x // 8] & (0x80 >> (x % 8)) != 0
        black, white = 0x00, 0xFF
        # if x < 5 and y < 8:
        #   black, white = 0x33, 0xAA
        img.putpixel((char_x + x, char_y + y), white if pixel else black)
        # print("#" if pixel else " ", end="")
      # print()

  return img


for font_path in sorted(
  (x for x in xorg_fonts_dir.iterdir() if x.suffix == ".bdf"),
  key=lambda x: [int(x) for x in re.split(r"\D+", x.name) if x != ""] + [x],
):
  font_img = render_font(parse_bdf_font(str(font_path)))
  font_img.save(res_dir / "fonts" / "xorg" / (font_path.stem + ".png"))
