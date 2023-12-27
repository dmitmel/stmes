# The tools need to be compiled for the host machine using the host C compiler,
# but unfortunately CMake doesn't support mixing multiple different compilers
# in the same project. To remedy this I use an improvised wrapper reminiscent
# of ExternalProject combined with FetchContent: like FetchContent, the
# configuration and build steps of the subproject are performed together with
# the configuration and building of the main project (respectively), but that
# is done through a recursive invocation of CMake and Make, akin to
# ExternalProject, instead of the subproject being added with
# add_subdirectory(), for the ability to mix compilers and toolchains.
function(configure_tools_subproject_wrapper)
  set(src_dir   "${CMAKE_CURRENT_SOURCE_DIR}/tools")
  set(build_dir "${CMAKE_CURRENT_BINARY_DIR}/tools")

  get_property(is_multi_config GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)

  set(configure_cmd "${CMAKE_COMMAND}" "-S${src_dir}" "-B${build_dir}")

  # The logic for passing the generator options was taken from
  # <https://gitlab.kitware.com/cmake/cmake/-/blob/v3.28.1/Modules/ExternalProject.cmake#L3740-3809>
  # <https://gitlab.kitware.com/cmake/cmake/-/blob/v3.28.1/Modules/FetchContent.cmake#L1585-1605>
  list(APPEND configure_cmd "-G${CMAKE_GENERATOR}")
  if(CMAKE_GENERATOR_PLATFORM)
    list(APPEND configure_cmd "-A${CMAKE_GENERATOR_PLATFORM}")
  endif()
  if(CMAKE_GENERATOR_TOOLSET)
    list(APPEND configure_cmd "-T${CMAKE_GENERATOR_TOOLSET}")
  endif()
  if(CMAKE_GENERATOR_INSTANCE)
    list(APPEND configure_cmd "-DCMAKE_GENERATOR_INSTANCE:INTERNAL=${CMAKE_GENERATOR_INSTANCE}")
  endif()
  if(CMAKE_MAKE_PROGRAM)
    list(APPEND configure_cmd "-DCMAKE_MAKE_PROGRAM:FILEPATH=${CMAKE_MAKE_PROGRAM}")
  endif()

  # Pass through other relevant variables as well.
  if(NOT is_multi_config)
    list(APPEND configure_cmd "-DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}")
  endif()
  list(APPEND configure_cmd "-DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=${CMAKE_EXPORT_COMPILE_COMMANDS}")

  # A workaround for weird legacy CMake behavior, see:
  # <https://gitlab.kitware.com/cmake/cmake/-/issues/16356>
  # <https://gitlab.kitware.com/cmake/cmake/-/issues/21378>
  # <https://gitlab.kitware.com/cmake/cmake/-/merge_requests/7108>
  # <https://cmake.org/cmake/help/latest/policy/CMP0132.html>
  # The code responsible for this:
  # <https://gitlab.kitware.com/cmake/cmake/blob/v3.28.1/Source/cmGlobalGenerator.cxx#L793-809>
  list(PREPEND configure_cmd "${CMAKE_COMMAND}" -E env --unset=CC --unset=CXX --unset=ASM)

  message(STATUS "Configuring tools subproject")
  execute_process(
    COMMAND ${configure_cmd}
    RESULT_VARIABLE cmd_result
  )
  if(cmd_result)
    message(FATAL_ERROR "Configuration of the tools subproject failed: ${cmd_result}")
  endif()

  # This was shamelessly stolen, errrrm, borrowed from
  # <https://gitlab.kitware.com/cmake/cmake/-/blob/v3.28.1/Modules/ExternalProject.cmake#L1928-2019>
  if(CMAKE_GENERATOR MATCHES "Make")
    # Make is a special case because it supports recursive invocation by design.
    set(build_cmd "$(MAKE)" -C "${build_dir}")
  else()
    set(build_cmd "${CMAKE_COMMAND}" --build "${build_dir}")
    if(is_multi_config)
      list(APPEND build_cmd --config $<CONFIG>)
    endif()
  endif()

  # This target will always be considered out-of-date and will always be built
  # (exactly what we need).
  add_custom_target(tools
    COMMAND ${build_cmd}
    COMMENT "Building tools subproject"
  )
endfunction()
