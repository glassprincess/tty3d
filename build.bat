@echo off
REM Build tty3d on Windows: MinGW (g++) or MSVC (cl) or CMake. Pick first available.
setlocal
where g++ >nul 2>nul
if %ERRORLEVEL%==0 (
  echo [build] using g++ (MinGW)
  g++ -O3 -Wall -Wextra -Wpedantic -Wshadow -std=c++11 -static main.cpp -o tty3d.exe
  if %ERRORLEVEL%==0 echo [build] OK: tty3d.exe (static, no DLLs) & exit /b 0
  exit /b 1
)
where cl >nul 2>nul
if %ERRORLEVEL%==0 (
  echo [build] using MSVC cl
  cl /O2 /EHsc /std:c++14 main.cpp /Fe:tty3d.exe
  if %ERRORLEVEL%==0 echo [build] OK: tty3d.exe & exit /b 0
  exit /b 1
)
where cmake >nul 2>nul
if %ERRORLEVEL%==0 (
  echo [build] using CMake
  cmake -B build -DCMAKE_BUILD_TYPE=Release
  cmake --build build --config Release
  copy /Y build\Release\tty3d.exe tty3d.exe >nul 2>nul
  copy /Y build\tty3d.exe tty3d.exe >nul 2>nul
  if exist tty3d.exe echo [build] OK: tty3d.exe & exit /b 0
)
echo [build] ERROR: no compiler found. Install MinGW: winget install -e --id MinGW.MinGW-w64
echo [build] or VS Build Tools: winget install -e --id Microsoft.VisualStudio.2022.BuildTools
exit /b 1
