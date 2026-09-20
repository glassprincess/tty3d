cmake_minimum_required(VERSION 3.15)
project(tty3d CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(tty3d main.cpp)

if(MSVC)
  target_compile_options(tty3d PRIVATE /W4 /O2)
else()
  target_compile_options(tty3d PRIVATE -O3 -Wall -Wextra -Wpedantic -Wshadow)
  # Static MinGW exe = zero DLL dependencies (matches "Zero Dependencies" goal).
  if(MINGW)
    target_link_options(tty3d PRIVATE -static)
  endif()
endif()

# Address+UB sanitizer build: cmake -B build-asan -DSANITIZE=ON
option(SANITIZE "Build with AddressSanitizer + UBSanitizer" OFF)
if(SANITIZE AND NOT MSVC)
  target_compile_options(tty3d PRIVATE -O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(tty3d PRIVATE -fsanitize=address,undefined)
endif()

# Selftest as ctest: ./tty3d --selftest + headless bench smoke test
enable_testing()
add_test(NAME selftest COMMAND tty3d --selftest)
add_test(NAME bench-knot COMMAND tty3d --bench 5 --size 80x24 -m knot)
