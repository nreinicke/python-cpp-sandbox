# Parent project does not export its library target, so this CML implicitly
# depends on being added from it, i.e. the testing is done only from the build
# tree and is not feasible from an install location

project(sandboxTests LANGUAGES CXX)

add_executable(sandbox_test source/sandbox_test.cpp)
target_link_libraries(sandbox_test PRIVATE sandbox_lib)
target_compile_features(sandbox_test PRIVATE cxx_std_17)

add_test(NAME sandbox_test COMMAND sandbox_test)

add_folders(Test)
