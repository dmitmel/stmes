// A failed attempt at utilizing a plugin for the GCC's ld linker to inject the
// symbol table as a loadable section into the binary. GCC uses this plugin
// system for implementing LTO, but the problem is that this *was* the original
// reason for adding a plugin system to ld and gold in the first place, and its
// API wasn't really designed for anything else. Essentially, the API allows a
// plugin to claim an object file for itself if it chooses so, in which case
// the responsibility for processing this object now lies fully on the plugin,
// at this point it should at least submit a list of all symbols it sees in the
// object. After all object files have been collected, the plugin will be
// called once again, in practice to do the LTO magic, which will result in
// compiling a bunch of new object files, which the plugin may then add to the
// link. That's it, the problem is that the plugins can only tap into the
// linkage process at a very early stage, waaaay before the linker-performed
// optimizations such as garbage collection, while what I require is the final
// symbol table.
//
// I have also considered constructing an improvised symbol table by
// repurposing some mechanism by which the unwind table entries are tied to
// function symbols, such that at first they are constructed for every
// function, but after the unused symbols are discarded their corresponding
// unwind entries are discarded as well, but unfortunately the `.ARM.exidx` and
// `.ARM.extab` sections involve special-case handling in the linker (see
// `binutils/ld/elf32-arm.c`). There is also no way to create a table
// referencing all symbols in advance by some sort of weak references, so that
// if they are not used in other parts of the program they'd be discarded.
//
// In any case, the linker plugin interface doesn't provide adequate
// functionality for what I want, but I guess some resources I have found in
// the meantime might be useful:
//
// <https://gcc.gnu.org/wiki/whopr/driver> - the best documentation on the
// plugin API I managed to find.
// <https://github.com/rui314/mold/issues/181> - an interesting discussion on
// the nature of the API.
//
// The actual implementation of the plugin interface in different linkers:
// <https://github.com/bminor/binutils-gdb/blob/binutils-2_41-release/ld/plugin.c>
// <https://github.com/bminor/binutils-gdb/blob/binutils-2_41-release/gold/plugin.cc>
// <https://github.com/bminor/binutils-gdb/blob/binutils-2_41-release/include/plugin-api.h>
//
// Various linker plugins that can be used as a reference for things:
// <https://github.com/bminor/binutils-gdb/blob/binutils-2_41-release/ld/libdep_plugin.c>
// <https://github.com/gcc-mirror/gcc/blob/d74cceb6d40a20f848aa78ab1ee9dd46c09f994a/lto-plugin/lto-plugin.c>
// <https://github.com/llvm/llvm-project/blob/7f69c8b3a6c02ea32fefb16c2016dfa1ba994858/llvm/tools/gold/gold-plugin.cpp>
// <https://github.com/adobe-flash/crossbridge/blob/8edfb144bfe52bee6799ad14929e3e00cb4f2226/gold-plugins/multiplug.cpp>
// <https://github.com/adobe-flash/crossbridge/blob/8edfb144bfe52bee6799ad14929e3e00cb4f2226/gold-plugins/makeswf.cpp>
// <https://github.com/ispras/libosuction/blob/b94e1f45bc7126ad6de29a433ea2a2a75dc08739/ld-plug/plug-priv.c>
// <https://github.com/open64-compiler/open64/blob/7e934651214049860141269bbadb457056745d51/osprey/be/ld_plugin/ld_plugin.cxx>
//
// There is also this dude who is super enthusiastic about linker plugins and
// has written some interesting but REALLY hacky applications with them, plus a
// library to simplify the work:
// <https://github.com/stephenrkell/elftin/blob/9f07a775ed68e1c1f20aa9c40440d5ccae180490/base-ldplugin/base-ldplugin.cpp>
// <https://github.com/stephenrkell/elftin/blob/9f07a775ed68e1c1f20aa9c40440d5ccae180490/include/elftin/ldplugins/base-ldplugin.hh>
// <https://github.com/stephenrkell/liballocs/blob/70ea21b7f5ebd324f00802b5c0d1a20e23a5e966/tools/gold-plugin.cpp>
//
// Oh, by the way, the object files passed to the plugin callbacks are
// generally opaque structs and the linker doesn't expose any interfaces to
// inspect the contents of the objects, so a complete implementation of my
// concept would also depend on an external library for reading ELF files! Oh
// and also once a file has been claimed, the other plugins will not see it, so
// it's really hard for plugins to cooperate outside the intended "transform
// the compiler-specific IR into machine code" use case of LTO.
//
// Some resources on using the BFD library, an internal component of binutils:
// <https://en.wikipedia.org/wiki/Binary_File_Descriptor_library#BFD_Library_Usage>
// <https://binutils.sourceware.narkive.com/irKsyWQL/using-bfd-to-get-debugging-information-from-elf-files>
// <https://stackoverflow.com/questions/22158487/number-of-sections-in-an-object-file-using-bfd-data-structure>
// <https://ftp.gnu.org/old-gnu/Manuals/bfd-2.9.1/html_mono/bfd.html>
// <https://sourceware.org/binutils/docs-2.38/bfd.pdf>
// <https://sourceware.org/binutils/docs/bfd/How-It-Works.html>
// <https://ftp.gnu.org/old-gnu/Manuals/ld/html_chapter/ld_5.html>
//
// Also a couple of links on writing plugins for GCC itself:
// <https://codesynthesis.com/~boris/blog/2010/05/03/parsing-cxx-with-gcc-plugin-part-1/>
// <https://gcc.gnu.org/onlinedocs/gccint/Plugins.html>

// TODO: Perhaps this system can transform GCC's Thread-Local-Storage emulation entries into proper TLS sections?

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

static const char* const LD_PLUGIN_TAG_NAMES[] = {
  "LDPT_NULL",
  "LDPT_API_VERSION",
  "LDPT_GOLD_VERSION",
  "LDPT_LINKER_OUTPUT",
  "LDPT_OPTION",
  "LDPT_REGISTER_CLAIM_FILE_HOOK",
  "LDPT_REGISTER_ALL_SYMBOLS_READ_HOOK",
  "LDPT_REGISTER_CLEANUP_HOOK",
  "LDPT_ADD_SYMBOLS",
  "LDPT_GET_SYMBOLS",
  "LDPT_ADD_INPUT_FILE",
  "LDPT_MESSAGE",
  "LDPT_GET_INPUT_FILE",
  "LDPT_RELEASE_INPUT_FILE",
  "LDPT_ADD_INPUT_LIBRARY",
  "LDPT_OUTPUT_NAME",
  "LDPT_SET_EXTRA_LIBRARY_PATH",
  "LDPT_GNU_LD_VERSION",
  "LDPT_GET_VIEW",
  "LDPT_GET_INPUT_SECTION_COUNT",
  "LDPT_GET_INPUT_SECTION_TYPE",
  "LDPT_GET_INPUT_SECTION_NAME",
  "LDPT_GET_INPUT_SECTION_CONTENTS",
  "LDPT_UPDATE_SECTION_ORDER",
  "LDPT_ALLOW_SECTION_ORDERING",
  "LDPT_GET_SYMBOLS_V2",
  "LDPT_ALLOW_UNIQUE_SEGMENT_FOR_SECTIONS",
  "LDPT_UNIQUE_SEGMENT_FOR_SECTIONS",
  "LDPT_GET_SYMBOLS_V3",
  "LDPT_GET_INPUT_SECTION_ALIGNMENT",
  "LDPT_GET_INPUT_SECTION_SIZE",
  "LDPT_REGISTER_NEW_INPUT_HOOK",
  "LDPT_GET_WRAP_SYMBOLS",
  "LDPT_ADD_SYMBOLS_V2",
  "LDPT_GET_API_VERSION",
  "LDPT_REGISTER_CLAIM_FILE_HOOK_V2",
};

static struct {
  ld_plugin_message message;
  ld_plugin_register_claim_file register_claim_file;
  ld_plugin_register_all_symbols_read register_all_symbols_read;
  ld_plugin_register_cleanup register_cleanup;
  ld_plugin_get_view get_view;
} api = {};

#define LOG(...) (api.message(LDPL_INFO, __VA_ARGS__))

struct LinkedFile {
  std::string name;
  int fd;
  off_t offset;
  off_t size;
  void* handle;
  LinkedFile() {}
  explicit LinkedFile(const ld_plugin_input_file& raw)
  : name(raw.name), fd(raw.fd), offset(raw.offset), size(raw.filesize), handle(raw.handle) {}
};

static std::list<LinkedFile> linked_files;

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
    const char* name = LD_PLUGIN_TAG_NAMES[tv->tv_tag];
    switch (tv->tv_tag) {
      case LDPT_NULL:
      case LDPT_API_VERSION:
      case LDPT_GNU_LD_VERSION:
      case LDPT_LINKER_OUTPUT: LOG("%s(%d)", name, tv->tv_u.tv_val); break;
      case LDPT_OPTION:
      case LDPT_OUTPUT_NAME: LOG("%s(%s)", name, tv->tv_u.tv_string); break;
      default: LOG("%s(%p)", name, (void*)tv->tv_u.tv_string); break;
    }
  }

  api.register_cleanup(&cleanup_hook);
  api.register_all_symbols_read(&all_symbols_read_hook);
  api.register_claim_file(&claim_file_hook);

  return LDPS_OK;
}

ld_plugin_status claim_file_hook(const ld_plugin_input_file* file, int* claimed) {
  *claimed = false;
  linked_files.push_back(LinkedFile(*file));
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
  for (const auto& file : linked_files) {
    LOG("%s %p %d", file.name.c_str(), file.handle, file.offset);
    // api.get_symbols(file.handle, 0, nullptr);
  }
  return LDPS_OK;
}

ld_plugin_status cleanup_hook() {
  linked_files.clear();
  return LDPS_OK;
}
