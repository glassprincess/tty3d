# tty3d — software 3D renderer for your terminal

Renders 3D models as ASCII art, right in the terminal. No OpenGL, no Vulkan, no GUI libraries — just C++, CPU math, and a byte stream to `stdout`. One file: `main.cpp`, standard library only.

> Fork of **tri3d**, renamed to `tty3d` so it doesn't squat on the original name.
> This fork adds: Windows support, input logging (`--log` / `--debug`), non-Latin keyboard layouts, CMake + build scripts.

## Build

```sh
./build.sh                                # Linux / macOS
g++ -O3 -Wall -Wextra -std=c++11 main.cpp -o tty3d   # by hand
cmake -B build && cmake --build build     # or CMake
```

Windows 10+ (use Windows Terminal):

```bat
build.bat                                 @rem tries g++, then cl, then CMake
g++ -O3 -std=c++11 -static main.cpp -o tty3d.exe     @rem MinGW, by hand
```

No compiler? `winget install MinGW.MinGW-w64` or grab VS Build Tools.

## Run

```sh
./tty3d                        # spinning donut
./tty3d --color amber -m knot
./tty3d models/icosahedron.obj
./tty3d a.obj b.obj            # press n to flip through models
```

Built in: `donut`, `cube`, `sphere`, `cone`, `knot`. Your own files: plain Wavefront `.obj` (`v` + `f` lines, polygons get fan-triangulated). Models are auto-centered and scaled to fit.

### Controls

| What | How |
|---|---|
| rotate | drag with left mouse button, or arrow keys |
| zoom | mouse wheel, `+` / `-` |
| auto-rotate | `space` |
| next model | `n` |
| color: off → amber → cyan → green → magenta | `c` |
| flat / smooth shading | `s` |
| orbiting light | `l` |
| overlay on/off | `h` |
| reset view | `r` |
| quit | `q`, `Esc`, `Ctrl+C` |

Letter keys also work on Russian layout (`й=q`, `с=c`, `ы=s`, `т=n`, `р=h`, `д=l`, `к=k`).

```
-m, --model NAME   start model (built-in or loaded file name)
    --color NAME   off | amber | cyan | green | magenta
    --smooth       smooth (interpolated) normals
    --no-cull      two-sided lighting, no back-face culling
    --fps N        0..1000, default 60 (0 = uncapped)
    --aspect X     cell height/width, 0.5..4, default 2
    --no-hud       no overlay
    --bench N      headless: render N frames, print the last one
    --size WxH     size for --bench, default 100x40
    --log FILE     log raw input bytes, parsed events, actions
    --debug        HUD line showing the last input event live
    --selftest     built-in tests
```

Model inside-out or missing faces? It probably has flipped winding — try `--no-cull`.

### Binds not working? Check the log

```sh
./tty3d --log input.log --debug
# click around, press keys, quit with q, then read input.log
```

`raw (...)` = what your terminal actually sent, `event:` = what the parser made of it, `=> ...` = what the program did. No `raw` while dragging = your terminal doesn't do mouse tracking (needs SGR `1002`+`1006`, e.g. Windows Terminal / xterm).

## How it works

Four stages, in order, all in `main.cpp`:

1. **Math** — hand-rolled `Vec3`/`Vec4`/`Mat4`/`Quat`. Model matrix (center, scale, quaternion rotation), `lookAt` view, perspective projection. Mouse rotation accumulates in a quaternion, so no gimbal lock.
2. **Rasterizer** — near-plane clipping (Sutherland–Hodgman), barycentric fill, Z-buffer storing `1/w`. Back-face culling in view space.
3. **Shading** — Lambert `N·L` + 0.10 ambient mapped onto `" .:-=+*#%@"`, optional 24-bit color. Smooth mode interpolates area-weighted vertex normals, perspective-correct.
4. **Output** — double-buffered diff: only changed cells go out, one `write()` per frame. Raw mode, alternate screen, hidden cursor, SGR mouse. Terminal state is restored on every exit path.

## Tests

```sh
./tty3d --selftest                     # 76 checks, runs anywhere
g++ -O1 -g -fsanitize=address,undefined -o tty3d_asan main.cpp && ./tty3d_asan --selftest
python3 tests/pty_test.py ./tty3d      # Linux only: real pty, keys/mouse/resize
```

Bench your machine: `./tty3d --bench 300 --size 200x50 -m knot` (timing goes to stderr).

## Limits

No textures, no `.mtl`, no shadows. Tweak `RAMP` in `main.cpp` if you don't like the gradient. Over SSH the bottleneck is the pipe, not the renderer — run without color to send fewer bytes.
