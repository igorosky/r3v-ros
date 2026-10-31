# Builds the r3v Rust crates (git submodules) with cargo and exposes them as
# imported shared libraries.
include(ExternalProject)

find_program(CARGO_EXECUTABLE cargo
  HINTS "$ENV{CARGO_HOME}/bin" "$ENV{HOME}/.cargo/bin"
  REQUIRED)

set(R3V_CARGO_TARGET_DIR "${CMAKE_BINARY_DIR}/cargo" CACHE PATH
  "cargo target directory of the r3v crates (persist it to skip Rust rebuilds)")

# r3v_add_rust_library(<name> <crate dir> <library name> <features>)
# Defines the imported target r3v::<name> and the cargo build target
# <name>_cargo. <features> is a comma separated cargo feature list.
function(r3v_add_rust_library name crate_dir lib_name features)
  if(NOT EXISTS "${crate_dir}/Cargo.toml")
    message(FATAL_ERROR
      "${crate_dir} is empty. Run: git submodule update --init --recursive")
  endif()

  set(lib_path
    "${R3V_CARGO_TARGET_DIR}/release/${CMAKE_SHARED_LIBRARY_PREFIX}${lib_name}${CMAKE_SHARED_LIBRARY_SUFFIX}")

  ExternalProject_Add(${name}_cargo
    SOURCE_DIR "${crate_dir}"
    CONFIGURE_COMMAND ""
    # MAKEFLAGS/MFLAGS would point cargo at a make jobserver it cannot reach.
    BUILD_COMMAND
      "${CMAKE_COMMAND}" -E env --unset=MAKEFLAGS --unset=MFLAGS
      "${CARGO_EXECUTABLE}" build --release --locked
      --features "${features}"
      --manifest-path "${crate_dir}/Cargo.toml"
      --target-dir "${R3V_CARGO_TARGET_DIR}"
    BUILD_ALWAYS ON
    BUILD_BYPRODUCTS "${lib_path}"
    INSTALL_COMMAND ""
    USES_TERMINAL_BUILD ON)

  add_library(r3v::${name} SHARED IMPORTED GLOBAL)
  set_target_properties(r3v::${name} PROPERTIES
    IMPORTED_LOCATION "${lib_path}"
    # Rust cdylibs have no SONAME; without this the build path would end up
    # in DT_NEEDED of everything linking them.
    IMPORTED_NO_SONAME TRUE
    INTERFACE_INCLUDE_DIRECTORIES "${crate_dir}/bindings")
  add_dependencies(r3v::${name} ${name}_cargo)

  set(${name}_LIBRARY "${lib_path}" PARENT_SCOPE)
endfunction()
