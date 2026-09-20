#!/usr/bin/env python3
"""Drive ./tty3d inside a real pseudo-terminal: sends keys, SGR mouse events and a resize,
replays the escape sequences on a tiny terminal emulator and checks the result."""
import os, pty, sys, time, fcntl, termios, struct, select, re, signal
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from termemu import Emu

BIN = sys.argv[1] if len(sys.argv) > 1 else "./tty3d"
ARGS = sys.argv[2:] or ["--color", "amber", "-m", "knot"]

def set_size(fd, rows, cols):
    fcntl.ioctl(fd, termios.TIOCSWINSZ, struct.pack("HHHH", rows, cols, 0, 0))

pid, fd = pty.fork()
if pid == 0:
    os.environ["TERM"] = "xterm-256color"
    os.execv(BIN, [BIN] + ARGS)
set_size(fd, 30, 100)
slave_path = "/dev/pts/%d" % struct.unpack("I", fcntl.ioctl(fd, 0x80045430, b"\0\0\0\0"))[0]
slave = os.open(slave_path, os.O_RDWR | os.O_NOCTTY)
before = termios.tcgetattr(slave)
emu = Emu(30, 100)
raw = b""

def pump(seconds):
    global raw
    end = time.time() + seconds
    while time.time() < end:
        r, _, _ = select.select([fd], [], [], 0.02)
        if r:
            try: d = os.read(fd, 1 << 16)
            except OSError: return False
            if not d: return False
            raw += d; emu.feed(d)
    return True

def send(b): os.write(fd, b)

pump(1.0)
during = termios.tcgetattr(slave)
send(b"\x1b[<0;40;10M")                       # left press
for k in range(1, 11):
    send(b"\x1b[<32;%d;%dM" % (40 + 2 * k, 10 + k)); pump(0.02)
send(b"\x1b[<0;60;20m")                       # release
send(b"\x1b[<64;50;15M\x1b[<64;50;15M\x1b[<65;50;15M")   # wheel
send(b"\x1b[A\x1b[B\x1b[C\x1b[D")             # arrows
send(b"c"); pump(0.3); send(b"s"); send(b"l"); send(b"n"); pump(0.3); send(b"n"); send(b" "); pump(0.3)
set_size(fd, 24, 80); emu.resize(24, 80); pump(0.5)   # resize -> app must redraw everything
pump(0.5)
screen_before_quit = emu.text()
send(b"q")
ok = True
status = None
end = time.time() + 3
while time.time() < end:
    pump(0.05)
    p, st = os.waitpid(pid, os.WNOHANG)
    if p:
        status = st; break
pump(0.1)
after = termios.tcgetattr(slave)

def check(cond, msg):
    global ok
    print(("PASS " if cond else "FAIL ") + msg)
    ok = ok and cond

check(status is not None and os.WIFEXITED(status) and os.WEXITSTATUS(status) == 0, "exits with code 0 after 'q'")
check(raw.startswith(b"\x1b[0m\x1b[?1049h"), "enters the alternate screen first")
check(b"\x1b[?1002h" in raw and b"\x1b[?1006h" in raw, "enables SGR mouse tracking")
check(raw.rstrip().endswith(b"\x1b[?1049l"), "leaves the alternate screen last")
check(b"\x1b[?25h" in raw and b"\x1b[?7h" in raw and b"\x1b[?1006l" in raw, "cursor, autowrap and mouse are restored")
check(not (during[3] & (termios.ECHO | termios.ICANON | termios.ISIG)), "terminal was in raw mode while running")
check(after == before, "terminal attributes fully restored after exit")
check(not emu.alt and not emu.cursor_hidden and emu.autowrap and not emu.mouse, "emulator state clean after exit")
hud = screen_before_quit[0]
print("HUD line :", hud.strip())
print("last line:", screen_before_quit[-1].strip())
m = re.search(r"FPS: ([0-9.]+)", hud)
check(m is not None, "HUD shows FPS")
if m: print("reported FPS:", m.group(1))
check("polygons:" in hud, "HUD shows polygon count")
body = "\n".join(screen_before_quit[1:-1])
check(sum(ch in ".:-=+*#%@" for ch in body) > 100, "model is drawn after resize to 80x24")
check(len(screen_before_quit) == 24 and all(len(r) == 80 for r in screen_before_quit), "emulated screen is 80x24 after resize")
print("\n".join(screen_before_quit))
print("total bytes written by the program:", len(raw))
sys.exit(0 if ok else 1)
