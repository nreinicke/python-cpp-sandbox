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

# ---- Declare library ----

add_library(
    sandbox_lib OBJECT
    source/lib.cpp
)

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
