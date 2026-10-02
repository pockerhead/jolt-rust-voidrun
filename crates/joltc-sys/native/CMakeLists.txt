# Native part of joltc-sys: builds joltc and Jolt as static libraries from the
# vendored submodules, compiles the C++ layout checks and installs a minimal
# prefix (lib/ with both archives, include/joltc.h) for build.rs to link.

# Jolt 5.6's Build/CMakeLists.txt requires 3.20.
cmake_minimum_required(VERSION 3.20)
project(joltc_sys_native C CXX)

# joltc and Jolt set the C++ standard only in their own directory scope; the
# layout checks in this scope need C++17 as well.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

foreach(required_dir JOLTC_SOURCE_DIR JOLT_PHYSICS_ROOT)
    if (NOT DEFINED ${required_dir})
        message(FATAL_ERROR "${required_dir} is not set. build.rs passes it; run `git submodule update --init` if the sources are missing.")
    endif()
endforeach()
if (NOT EXISTS "${JOLTC_SOURCE_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "joltc not found at ${JOLTC_SOURCE_DIR}. Run `git submodule update --init`.")
endif()
if (NOT EXISTS "${JOLT_PHYSICS_ROOT}/Build/CMakeLists.txt")
    message(FATAL_ERROR "Jolt Physics not found at ${JOLT_PHYSICS_ROOT}. Run `git submodule update --init`.")
endif()

# As a subproject joltc builds a static library without samples, tests or
# install rules, and takes Jolt from JOLT_PHYSICS_ROOT instead of downloading it.
add_subdirectory(${JOLTC_SOURCE_DIR} joltc)

if (DOUBLE_PRECISION)
    set(JOLTC_TARGET joltc_double)
else()
    set(JOLTC_TARGET joltc)
endif()

# Compile-time checks that joltc's C mirrors match Jolt's C++ types. Linking the
# targets brings in Jolt's public definitions (precision, object layer bits,
# instruction set) and joltc's include directory. Part of ALL, so a failing
# static_assert fails the build.
add_library(joltc_layout_checks OBJECT layout_checks.cpp)
target_link_libraries(joltc_layout_checks PRIVATE ${JOLTC_TARGET} Jolt)

install(TARGETS ${JOLTC_TARGET} Jolt ARCHIVE DESTINATION lib)
install(FILES ${JOLTC_SOURCE_DIR}/include/joltc.h DESTINATION include)
