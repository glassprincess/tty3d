#!/usr/bin/env python3
"""Every way of leaving tty3d must restore the terminal: q, Esc, Ctrl+C, SIGTERM, SIGHUP, closed stdin."""
import os, pty, sys, time, fcntl, termios, struct, select, signal, re
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from termemu import Emu

BIN = sys.argv[1] if len(sys.argv) > 1 else "./tty3d"
ok = True
def check(c, m):
    global ok
    print(("PASS " if c else "FAIL ") + m); ok = ok and c

def run(name, action, extra=(), wait=0.6):
    pid, fd = pty.fork()
    if pid == 0:
        os.environ["TERM"] = "xterm"
        os.execv(BIN, [BIN, "--color", "cyan"] + list(extra))
    fcntl.ioctl(fd, termios.TIOCSWINSZ, struct.pack("HHHH", 24, 80, 0, 0))
    slave = os.open("/dev/pts/%d" % struct.unpack("I", fcntl.ioctl(fd, 0x80045430, b"\0\0\0\0"))[0], os.O_RDWR | os.O_NOCTTY)
    before = termios.tcgetattr(slave)
    raw = b""
    emu = Emu(24, 80)
    def pump(t):
        nonlocal raw
        end = time.time() + t
        while time.time() < end:
            r, _, _ = select.select([fd], [], [], 0.02)
            if r:
                try: d = os.read(fd, 65536)
                except OSError: return
                if not d: return
                raw += d; emu.feed(d)
    pump(wait)
    action(pid, fd)
    st = None
    end = time.time() + 3
    while time.time() < end:
        pump(0.05)
        p, s = os.waitpid(pid, os.WNOHANG)
        if p: st = s; break
    pump(0.1)
    after = termios.tcgetattr(slave)
    check(st is not None and os.WIFEXITED(st) and os.WEXITSTATUS(st) == 0, name + ": clean exit code 0")
    check(after == before, name + ": termios restored")
    check(raw.rstrip().endswith(b"\x1b[?1049l"), name + ": alternate screen left")
    return raw, emu

run("q", lambda pid, fd: os.write(fd, b"q"))
run("Esc", lambda pid, fd: os.write(fd, b"\x1b"))
run("Ctrl+C", lambda pid, fd: os.write(fd, b"\x03"))
run("SIGTERM", lambda pid, fd: os.kill(pid, signal.SIGTERM))
run("SIGHUP", lambda pid, fd: os.kill(pid, signal.SIGHUP))
raw, _ = run("colors", lambda pid, fd: os.write(fd, b"q"))
check(b"\x1b[38;2;" in raw, "TrueColor sequences are emitted with --color")

# throttling: --fps 30
raw, emu = run("fps30", lambda pid, fd: os.write(fd, b"q"), extra=["--fps", "30"], wait=1.5)
# the emulator is reset by the terminal-leave sequence only logically; the grid still holds the last frame
hud = emu.text()[0]
m = re.search(r"FPS: ([0-9.]+)", hud)
print("HUD with --fps 30:", hud.strip())
check(m is not None and 29.0 <= float(m.group(1)) <= 31.0, "--fps 30 is honoured")
sys.exit(0 if ok else 1)
