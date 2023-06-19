import struct

from PIL import Image

FRAME_COUNT = 2155
COLOR_BITS = 3
PALETTE_LEN = 1 << COLOR_BITS
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240
FILE_SIZE_LIMIT = 4 * 1024 * 1024 * 1024


def rle_encode(row: list[int]):
  row_len = len(row)
  value = 0
  i = 0
  while i < row_len:
    value = row[i]
    j = i + 1
    max_repeats = min(row_len, i + (0x100 >> COLOR_BITS))
    while j < max_repeats and row[j] == value:
      j += 1
    repeats = j - i
    yield ((repeats - 1) << COLOR_BITS) | value
    i = j


frame_idx = 0
batch_idx = 0
while frame_idx < FRAME_COUNT:
  encoded = []
  encoded_frames = []
  frame_lengths = []

  batch_frame_idx = 0
  prev_frame_rows = []

  pack_format = lambda: f"<HHIII{len(frame_lengths)}H{len(encoded_frames)}B{len(encoded)}B"

  prev_row = None

  while frame_idx < FRAME_COUNT and struct.calcsize(pack_format()) <= FILE_SIZE_LIMIT:
    frame_idx += 1
    batch_frame_idx += 1
    img_path = f"bebop/frames/{frame_idx:04d}.png"
    print(frame_idx, batch_idx, batch_frame_idx)

    img = Image.open(img_path).resize((IMAGE_WIDTH, IMAGE_HEIGHT), Image.LANCZOS)
    img = img.quantize(colors=PALETTE_LEN, method=Image.Quantize.FASTOCTREE)
    width, height = img.size

    while len(prev_frame_rows) < height:
      prev_frame_rows.append([])
    frame_start = len(encoded_frames)

    palette = img.getpalette("RGB")
    assert palette is not None
    palette = palette[:PALETTE_LEN * 3]
    encoded_frames.extend(palette)
    img_data = list(img.getdata())

    FLAG_REPEAT_PREVIOUS = 0x01
    FLAG_DUPLICATE_ROW = 0x02
    FLAG_NO_COMMON_PIXELS = 0x04
    FLAG_INCREMENT_Y = 0x08
    FLAG_COMPACT_ROW_LENGTH = 0x10

    prev_delta = None
    prev_delta_idx = 0
    prev_delta_repeats_idx = 0

    prev_row_y = -1
    for y in range(height):
      row = list(rle_encode(img_data[y * width:(y + 1) * width]))
      if len(row) >= 0x100:
        raise AssertionError()

      prev_frame_row = prev_frame_rows[y]
      if prev_frame_row == row:
        continue
      prev_frame_rows[y] = row

      common = 0
      for a, b in zip(prev_frame_row, row):
        if a != b:
          break
        common += 1
      row = row[common:]

      flags = 0
      flags |= FLAG_DUPLICATE_ROW if row == prev_row else 0
      flags |= FLAG_NO_COMMON_PIXELS if common == 0 else 0
      flags |= FLAG_INCREMENT_Y if y == prev_row_y + 1 else 0
      flags |= FLAG_COMPACT_ROW_LENGTH | (len(row) - 1) << 5 if 1 <= len(row) <= 8 else 0

      delta = []
      delta.append(flags)
      if not flags & FLAG_INCREMENT_Y:
        delta.extend(struct.pack("<H", y))
      if not flags & FLAG_NO_COMMON_PIXELS:
        delta.append(common)
      if not flags & FLAG_COMPACT_ROW_LENGTH:
        delta.append(len(row))

      if prev_delta == delta:
        flush_delta = False
        if prev_delta_repeats_idx is None:
          encoded_frames[prev_delta_idx] |= FLAG_REPEAT_PREVIOUS
          prev_delta_repeats_idx = len(encoded_frames)
          encoded_frames.append(0)
        elif encoded_frames[prev_delta_repeats_idx] >= 0xff:
          flush_delta = True
        else:
          encoded_frames[prev_delta_repeats_idx] += 1
      else:
        flush_delta = True

      if flush_delta:
        prev_delta = delta
        prev_delta_idx = len(encoded_frames)
        prev_delta_repeats_idx = None
        encoded_frames.extend(delta)

      if not flags & FLAG_DUPLICATE_ROW:
        encoded.extend(row)

      prev_row = row
      prev_row_y = y

    frame_len = len(encoded_frames) - frame_start
    if frame_len >= 0x10000:
      raise AssertionError()
    frame_lengths.append(frame_len)

  batch_idx += 1
  with open(f"bebop/encoded/{batch_idx:04d}.bin", "wb") as out:
    bin_data = struct.pack(
      pack_format(),
      IMAGE_WIDTH,
      IMAGE_HEIGHT,
      len(frame_lengths),
      len(encoded_frames),
      len(encoded),
      *frame_lengths,
      *encoded_frames,
      *encoded,
    )
    out.write(bin_data)
    out.flush()
