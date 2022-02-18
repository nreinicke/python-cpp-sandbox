cmake_minimum_required(VERSION 3.14)

include(cmake/prelude.cmake)

project(
    sandbox
    VERSION 0.1.0
    DESCRIPTION "sandbox for testing c++"
    HOMEPAGE_URL "https://example.com/"
    LANGUAGES CXX
)

docs_early_return()

include(cmake/project-is-top-level.cmake)
include(cmake/variables.cmake)

# ----- Find pybind11 and python -----
find_package(Python COMPONENTS Interpreter Development REQUIRED)
add_subdirectory(extern/pybind11)

# ---- Declare library ----

add_library(
    sandbox_lib OBJECT
    source/lib.cpp
)
set_property(TARGET sandbox_lib PROPERTY POSITION_INDEPENDENT_CODE ON)

target_include_directories(
    sandbox_lib ${warning_guard}
    PUBLIC
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/source>"
)

target_compile_features(sandbox_lib PUBLIC cxx_std_17)

# ---- Declare executable ----

add_executable(sandbox_exe source/main.cpp)
add_executable(sandbox::exe ALIAS sandbox_exe)

set_target_properties(
    sandbox_exe PROPERTIES
    OUTPUT_NAME sandbox
    EXPORT_NAME exe
)

target_compile_features(sandbox_exe PRIVATE cxx_std_17)

target_link_libraries(sandbox_exe PRIVATE sandbox_lib)
target_link_libraries(sandbox_exe PRIVATE pybind11::embed)


if(SKBUILD)
  # Scikit-Build does not add your site-packages to the search path
  # automatically, so we need to add it _or_ the pybind11 specific directory
  # here.
  execute_process(
    COMMAND "${PYTHON_EXECUTABLE}" -c
            "import pybind11; print(pybind11.get_cmake_dir())"
    OUTPUT_VARIABLE _tmp_dir
    OUTPUT_STRIP_TRAILING_WHITESPACE COMMAND_ECHO STDOUT)
  list(APPEND CMAKE_PREFIX_PATH "${_tmp_dir}")

  # Now we can find pybind11
  find_package(pybind11 CONFIG REQUIRED)

  pybind11_add_module(_core MODULE source/py_bindings.cpp)

  target_link_libraries(_core PRIVATE sandbox_lib)

  target_compile_definitions(_core PRIVATE VERSION_INFO=${PROJECT_VERSION})

  install(TARGETS _core DESTINATION .)
endif()


# ---- Install rules ----

if(NOT CMAKE_SKIP_INSTALL_RULES)
  include(cmake/install-rules.cmake)
endif()

# ---- Developer mode ----

if(NOT sandbox_DEVELOPER_MODE)
  return()
elseif(NOT PROJECT_IS_TOP_LEVEL)
  message(
      AUTHOR_WARNING
      "Developer mode is intended for developers of sandbox"
  )
endif()

include(cmake/dev-mode.cmake)
