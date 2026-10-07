"""Extract [SHOT-BEGIN]..[SHOT-END] blocks from a serial log and write PNGs (pure Python)."""
import base64, struct, sys, zlib, re

def write_png(path, w, h, rgb):
    raw = b"".join(b"\x00" + rgb[y * w * 3:(y + 1) * w * 3] for y in range(h))
    def chunk(t, d):
        c = struct.pack(">I", len(d)) + t + d
        return c + struct.pack(">I", zlib.crc32(t + d) & 0xFFFFFFFF)
    png = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b"")
    open(path, "wb").write(png)

log = open(sys.argv[1], encoding="utf-8", errors="ignore").read()
prefix = sys.argv[2]
blocks = re.findall(r"\[SHOT-BEGIN (\d+)x(\d+) RGB565LE\]\s*(.*?)\[SHOT-END\]", log, re.S)
for n, (w, h, body) in enumerate(blocks):
    w, h = int(w), int(h)
    parts = []
    for l in body.splitlines():
        l = l.split("[", 1)[0].strip()  # other tasks may append log text to a line
        if l and re.fullmatch(r"[A-Za-z0-9+/=]+", l):
            parts.append(l)
    data = base64.b64decode("".join(parts))
    if len(data) < w * h * 2:
        print(f"block {n}: short ({len(data)} bytes)"); continue
    rgb = bytearray(w * h * 3)
    for i in range(w * h):
        v = data[2 * i] | (data[2 * i + 1] << 8)
        r, g, b = (v >> 11) & 31, (v >> 5) & 63, v & 31
        rgb[3 * i], rgb[3 * i + 1], rgb[3 * i + 2] = r * 255 // 31, g * 255 // 63, b * 255 // 31
    out = f"{prefix}{n}.png"
    write_png(out, w, h, bytes(rgb))
    print("wrote", out)
