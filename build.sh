#!/bin/sh
# Build tty3d on Linux / macOS / MinGW-shell. Zero dependencies.
set -eu
cd "$(dirname "$0")"
CXX="${CXX:-g++}"
if ! command -v "$CXX" >/dev/null 2>&1; then CXX="clang++"; fi
"$CXX" -O3 -Wall -Wextra -Wpedantic -Wshadow -std=c++11 main.cpp -o tty3d
echo "[build] OK: ./tty3d"
./tty3d --selftest
