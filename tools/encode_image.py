import struct

from PIL import Image

img = Image.open("penguins.png")
img = Image.alpha_composite(Image.new("RGBA", img.size, (0, 0, 0)), img)

img = img.quantize()
palette = img.getpalette("RGB")
assert palette is not None

encoded = []
img_data = list(img.getdata())
for y in range(img.height):
  row = img_data[y * img.width:(y + 1) * img.width]
  # for i in range(0, img.width, 2):
  #   r1, g1, b1 = row[i]
  #   r2, g2, b2 = row[i + 1]
  #   encoded.append((r1 & 0xF0) | (g1 >> 4))
  #   encoded.append((b1 & 0xF0) | (r2 >> 4))
  #   encoded.append((g2 & 0xF0) | (b2 >> 4))
  encoded.extend(row)

with open("doge.bin", "wb") as out:
  bin_data = struct.pack(
    f"<HHHI{len(palette)}B{len(encoded)}B", img.width, img.height, len(palette), len(encoded),
    *palette, *encoded
  )
  out.write(bin_data)
  out.flush()
