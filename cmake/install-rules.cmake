if(PROJECT_IS_TOP_LEVEL)
  set(CMAKE_INSTALL_INCLUDEDIR include/sandbox CACHE PATH "")
endif()

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

# find_package(<package>) call for consumers to find this project
set(package sandbox)

install(
    TARGETS sandbox_exe
    EXPORT sandboxTargets
    RUNTIME COMPONENT sandbox_Runtime
)

write_basic_package_version_file(
    "${package}ConfigVersion.cmake"
    COMPATIBILITY SameMajorVersion
)

# Allow package maintainers to freely override the path for the configs
set(
    sandbox_INSTALL_CMAKEDIR "${CMAKE_INSTALL_DATADIR}/${package}"
    CACHE PATH "CMake package config location relative to the install prefix"
)
mark_as_advanced(sandbox_INSTALL_CMAKEDIR)

install(
    FILES cmake/install-config.cmake
    DESTINATION "${sandbox_INSTALL_CMAKEDIR}"
    RENAME "${package}Config.cmake"
    COMPONENT sandbox_Development
)

install(
    FILES "${PROJECT_BINARY_DIR}/${package}ConfigVersion.cmake"
    DESTINATION "${sandbox_INSTALL_CMAKEDIR}"
    COMPONENT sandbox_Development
)

install(
    EXPORT sandboxTargets
    NAMESPACE sandbox::
    DESTINATION "${sandbox_INSTALL_CMAKEDIR}"
    COMPONENT sandbox_Development
)

if(PROJECT_IS_TOP_LEVEL)
  include(CPack)
endif()
