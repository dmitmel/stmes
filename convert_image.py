import struct

from PIL import Image


def rle_encode(row: list[int]):
  row_len = len(row)
  value = 0
  i = 0
  while i < row_len:
    value = row[i]
    j = i + 1
    max_repeats = min(row_len, i + 0x80)
    while j < max_repeats and row[j] == value:
      j += 1
    repeats = j - i
    yield ((repeats - 1) << 1) | (1 if value else 0)
    i = j


FRAME_COUNT = 6572
IMAGE_WIDTH = 240
IMAGE_HEIGHT = 180
FILE_SIZE_LIMIT = 480 * 1024
WHITE_THRESHOLD = 200

frame_idx = 0
batch_idx = 0
while frame_idx < FRAME_COUNT:
  encoded = []
  encoded_frames = []
  frame_offsets = []

  batch_frame_idx = 0
  prev_frame_rows = []

  pack_format = lambda: f"<HHIIII{len(frame_offsets)}I{len(encoded_frames)}B{len(encoded)}B"

  duplicate_rows = {}
  next_row_offset = 0
  prev_row_offset = 0

  while frame_idx < FRAME_COUNT and struct.calcsize(pack_format()) <= FILE_SIZE_LIMIT:
    frame_idx += 1
    batch_frame_idx += 1
    img_path = f"apple/frames/{frame_idx:04d}.png"
    print(frame_idx, batch_idx, batch_frame_idx)

    img = Image.open(img_path).resize((IMAGE_WIDTH, IMAGE_HEIGHT), Image.LANCZOS)
    img = img.convert("L").point(lambda x: 255 if x > WHITE_THRESHOLD else 0, mode="1")
    width, height = img.size
    img_data = list(img.getdata(0))

    while len(prev_frame_rows) < height:
      prev_frame_rows.append([])
    frame_offsets.append(len(encoded_frames))

    FLAG_CONTIGUOUS_OFFSET = 0x01
    FLAG_UNCHANGED_OFFSET = 0x02
    FLAG_NO_COMMON_PIXELS = 0x04
    FLAG_INCREMENT_Y = 0x08
    FLAG_COMPACT_ROW_LENGTH = 0x10

    prev_row_y = -1
    for y in range(height):
      row = tuple(rle_encode(img_data[y * width:(y + 1) * width]))
      if len(row) >= 0x100:
        raise AssertionError()

      prev_row = prev_frame_rows[y]
      if prev_row == row:
        continue
      prev_frame_rows[y] = row

      common = 0
      offset = duplicate_rows.get(row, None)
      if offset is None:
        for a, b in zip(prev_row, row):
          if a != b:
            break
          common += 1
        row = row[common:]
        offset = len(encoded)
        encoded.extend(row)
        duplicate_rows[row] = offset

      flags = 0
      flags |= FLAG_CONTIGUOUS_OFFSET if offset == next_row_offset else 0
      flags |= FLAG_UNCHANGED_OFFSET if offset == prev_row_offset else 0
      flags |= FLAG_NO_COMMON_PIXELS if common == 0 else 0
      flags |= FLAG_INCREMENT_Y if y == prev_row_y + 1 else 0
      flags |= FLAG_COMPACT_ROW_LENGTH | (len(row) - 1) << 5 if 1 <= len(row) <= 8 else 0

      encoded_frames.append(flags)
      if not flags & FLAG_INCREMENT_Y:
        encoded_frames.extend(struct.pack("<H", y))
      if not flags & FLAG_NO_COMMON_PIXELS:
        encoded_frames.append(common)
      if not flags & FLAG_COMPACT_ROW_LENGTH:
        encoded_frames.append(len(row))
      if not flags & FLAG_CONTIGUOUS_OFFSET and not flags & FLAG_UNCHANGED_OFFSET:
        encoded_frames.extend(struct.pack("<I", offset))

      next_row_offset = offset + len(row)
      prev_row_offset = offset
      prev_row_y = y

  frame_offsets.append(len(encoded_frames))

  batch_idx += 1
  with open(f"apple/encoded/{batch_idx:04d}.c", "w") as out:
    bin_data = struct.pack(
      pack_format(),
      IMAGE_WIDTH,
      IMAGE_HEIGHT,
      batch_frame_idx,
      len(frame_offsets),
      len(encoded_frames),
      len(encoded),
      *frame_offsets,
      *encoded_frames,
      *encoded,
    )
    out.write('#include "stmes/video_data.h"\n')
    out.write("const usize video_bin_data_len = {};\n".format(len(bin_data)))
    out.write("const u8 video_bin_data[] = {\n")
    PAGE_SIZE = 32
    for i in range(0, len(bin_data), PAGE_SIZE):
      out.write(",".join(map(str, bin_data[i:i + PAGE_SIZE])))
      if i + PAGE_SIZE < len(bin_data):
        out.write(",")
      out.write("\n")
    out.write("};\n")
    out.flush()
