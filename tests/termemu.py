import re


class Emu:
    def __init__(self, rows, cols):
        self.resize(rows, cols)
        self.alt = False; self.autowrap = True; self.mouse = set(); self.cursor_hidden = False
    def resize(self, rows, cols):
        self.rows, self.cols = rows, cols
        self.grid = [[(" ", None)] * cols for _ in range(rows)]
        self.r = self.c = 0; self.fg = None
    def feed(self, data):
        s = data.decode("latin1"); i = 0
        while i < len(s):
            ch = s[i]
            if ch == "\x1b":
                m = re.compile(r"\x1b\[([0-9;?<]*)([A-Za-z])").match(s, i)
                if not m:
                    if len(s) - i < 32: break
                    i += 1; continue
                params, fin = m.group(1), m.group(2); i = m.end()
                if fin == "H":
                    p = [int(x) if x else 1 for x in params.split(";")] if params else [1, 1]
                    self.r, self.c = p[0] - 1, (p[1] - 1 if len(p) > 1 else 0)
                elif fin == "J" and params == "2":
                    self.grid = [[(" ", None)] * self.cols for _ in range(self.rows)]
                elif fin == "m":
                    if params in ("0", "39", ""): self.fg = None
                    elif params.startswith("38;2;"): self.fg = tuple(int(x) for x in params.split(";")[2:5])
                elif fin in "hl" and params.startswith("?"):
                    on = fin == "h"
                    for code in params[1:].split(";"):
                        if code == "1049": self.alt = on
                        elif code == "7": self.autowrap = on
                        elif code == "25": self.cursor_hidden = not on
                        elif code in ("1002", "1003", "1006"):
                            (self.mouse.add if on else self.mouse.discard)(code)
            else:
                if 0 <= self.r < self.rows and 0 <= self.c < self.cols:
                    self.grid[self.r][self.c] = (ch, self.fg)
                if self.c < self.cols - 1: self.c += 1
                elif self.autowrap: self.c = 0; self.r += 1
                i += 1
        self.leftover = s[i:]
    def text(self):
        return ["".join(c for c, _ in row) for row in self.grid]
