// TODO comments

#include <list>
#include <memory>
#include <string>
#include <unistd.h>

#define HAVE_STDINT_H 1
#define HAVE_INTTYPES_H 1
#include <plugin-api.h>

#define PACKAGE
#define PACKAGE_VERSION
#include <bfd.h>

// TODO check status

static const struct {
  ld_plugin_tag num;
  const char* name;
} LD_PLUGIN_TAG_NAMES[] = {
  { LDPT_NULL, "LDPT_NULL" },
  { LDPT_NULL, "LDPT_NULL" },
  { LDPT_API_VERSION, "LDPT_API_VERSION" },
  { LDPT_GOLD_VERSION, "LDPT_GOLD_VERSION" },
  { LDPT_LINKER_OUTPUT, "LDPT_LINKER_OUTPUT" },
  { LDPT_OPTION, "LDPT_OPTION" },
  { LDPT_REGISTER_CLAIM_FILE_HOOK, "LDPT_REGISTER_CLAIM_FILE_HOOK" },
  { LDPT_REGISTER_ALL_SYMBOLS_READ_HOOK, "LDPT_REGISTER_ALL_SYMBOLS_READ_HOOK" },
  { LDPT_REGISTER_CLEANUP_HOOK, "LDPT_REGISTER_CLEANUP_HOOK" },
  { LDPT_ADD_SYMBOLS, "LDPT_ADD_SYMBOLS" },
  { LDPT_GET_SYMBOLS, "LDPT_GET_SYMBOLS" },
  { LDPT_ADD_INPUT_FILE, "LDPT_ADD_INPUT_FILE" },
  { LDPT_MESSAGE, "LDPT_MESSAGE" },
  { LDPT_GET_INPUT_FILE, "LDPT_GET_INPUT_FILE" },
  { LDPT_RELEASE_INPUT_FILE, "LDPT_RELEASE_INPUT_FILE" },
  { LDPT_ADD_INPUT_LIBRARY, "LDPT_ADD_INPUT_LIBRARY" },
  { LDPT_OUTPUT_NAME, "LDPT_OUTPUT_NAME" },
  { LDPT_SET_EXTRA_LIBRARY_PATH, "LDPT_SET_EXTRA_LIBRARY_PATH" },
  { LDPT_GNU_LD_VERSION, "LDPT_GNU_LD_VERSION" },
  { LDPT_GET_VIEW, "LDPT_GET_VIEW" },
  { LDPT_GET_INPUT_SECTION_COUNT, "LDPT_GET_INPUT_SECTION_COUNT" },
  { LDPT_GET_INPUT_SECTION_TYPE, "LDPT_GET_INPUT_SECTION_TYPE" },
  { LDPT_GET_INPUT_SECTION_NAME, "LDPT_GET_INPUT_SECTION_NAME" },
  { LDPT_GET_INPUT_SECTION_CONTENTS, "LDPT_GET_INPUT_SECTION_CONTENTS" },
  { LDPT_UPDATE_SECTION_ORDER, "LDPT_UPDATE_SECTION_ORDER" },
  { LDPT_ALLOW_SECTION_ORDERING, "LDPT_ALLOW_SECTION_ORDERING" },
  { LDPT_GET_SYMBOLS_V2, "LDPT_GET_SYMBOLS_V2" },
  { LDPT_ALLOW_UNIQUE_SEGMENT_FOR_SECTIONS, "LDPT_ALLOW_UNIQUE_SEGMENT_FOR_SECTIONS" },
  { LDPT_UNIQUE_SEGMENT_FOR_SECTIONS, "LDPT_UNIQUE_SEGMENT_FOR_SECTIONS" },
  { LDPT_GET_SYMBOLS_V3, "LDPT_GET_SYMBOLS_V3" },
  { LDPT_GET_INPUT_SECTION_ALIGNMENT, "LDPT_GET_INPUT_SECTION_ALIGNMENT" },
  { LDPT_GET_INPUT_SECTION_SIZE, "LDPT_GET_INPUT_SECTION_SIZE" },
  { LDPT_REGISTER_NEW_INPUT_HOOK, "LDPT_REGISTER_NEW_INPUT_HOOK" },
  { LDPT_GET_WRAP_SYMBOLS, "LDPT_GET_WRAP_SYMBOLS" },
  { LDPT_ADD_SYMBOLS_V2, "LDPT_ADD_SYMBOLS_V2" },
  { LDPT_GET_API_VERSION, "LDPT_GET_API_VERSION" },
  { LDPT_REGISTER_CLAIM_FILE_HOOK_V2, "LDPT_REGISTER_CLAIM_FILE_HOOK_V2" },
};

static struct {
  ld_plugin_message message;
  ld_plugin_register_claim_file register_claim_file;
  ld_plugin_register_all_symbols_read register_all_symbols_read;
  ld_plugin_register_cleanup register_cleanup;
  ld_plugin_get_view get_view;
} api = {};

#define LOG(...) (api.message(LDPL_INFO, __VA_ARGS__))

// struct LinkedFile {
//   std::string name;
//   int fd;
//   off_t offset;
//   off_t size;
//   void* handle;
//   LinkedFile() {}
//   explicit LinkedFile(const ld_plugin_input_file& raw)
//   : name(raw.name), fd(raw.fd), offset(raw.offset), size(raw.filesize), handle(raw.handle) {}
// };

// static std::list<LinkedFile> linked_files;

extern "C" ld_plugin_status onload(ld_plugin_tv* tv);
static ld_plugin_status claim_file_hook(const ld_plugin_input_file* file, int* claimed);
static ld_plugin_status all_symbols_read_hook();
static ld_plugin_status cleanup_hook();

ld_plugin_status onload(ld_plugin_tv* tv) {
  ld_plugin_tv* first_tv = tv;

  for (; tv->tv_tag != LDPT_NULL; tv++) {
    switch (tv->tv_tag) {
      case LDPT_MESSAGE: api.message = tv->tv_u.tv_message; break;
      case LDPT_REGISTER_CLAIM_FILE_HOOK:
        api.register_claim_file = tv->tv_u.tv_register_claim_file;
        break;
      case LDPT_REGISTER_ALL_SYMBOLS_READ_HOOK:
        api.register_all_symbols_read = tv->tv_u.tv_register_all_symbols_read;
        break;
      case LDPT_REGISTER_CLEANUP_HOOK: api.register_cleanup = tv->tv_u.tv_register_cleanup; break;
      case LDPT_GET_VIEW: api.get_view = tv->tv_u.tv_get_view; break;
      default: break;
    }
  }

  LOG("hi!");

  for (tv = first_tv; tv->tv_tag != 0; tv++) {
    for (const auto& tag : LD_PLUGIN_TAG_NAMES) {
      if (tag.num == tv->tv_tag) {
        switch (tv->tv_tag) {
          case LDPT_NULL:
          case LDPT_API_VERSION:
          case LDPT_GNU_LD_VERSION:
          case LDPT_LINKER_OUTPUT: LOG("%s(%d)", tag.name, tv->tv_u.tv_val); break;
          case LDPT_OPTION:
          case LDPT_OUTPUT_NAME: LOG("%s(%s)", tag.name, tv->tv_u.tv_string); break;
          default: LOG("%s(%p)", tag.name, (void*)tv->tv_u.tv_string); break;
        }
        break;
      }
    }
  }

  api.register_cleanup(&cleanup_hook);
  api.register_all_symbols_read(&all_symbols_read_hook);
  api.register_claim_file(&claim_file_hook);

  return LDPS_OK;
}

ld_plugin_status claim_file_hook(const ld_plugin_input_file* file, int* claimed) {
  *claimed = false;
  // linked_files.push_back(LinkedFile(*file));
  // LOG("%s %d %d", file->name, file->offset, *claimed);
  LOG("%s", file->name);
  std::unique_ptr<char[]> alloced_contents = nullptr;
  const void* view = nullptr;
  if (api.get_view) {
    api.get_view(file->handle, &view);
  } else {
    alloced_contents.reset(new char[file->filesize]);
    pread(file->fd, alloced_contents.get(), file->filesize, file->offset);
    view = alloced_contents.get();
  }
  return LDPS_OK;
}

ld_plugin_status all_symbols_read_hook() {
  // for (const auto& file : linked_files) {
  //   LOG("%s %p", file.name.c_str(), file.handle);
  //   // api.get_symbols(file.handle, 0, nullptr);
  // }
  return LDPS_OK;
}

ld_plugin_status cleanup_hook() {
  // linked_files.clear();
  return LDPS_OK;
}
