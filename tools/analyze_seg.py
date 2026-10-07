"""Split a serial log into segments at each [CMD] line (bw/foff/rssi changes) and score reception per setting."""
import json, re, sys
from collections import defaultdict

KEYS = ("bw", "freq", "rssi")
lines = open(sys.argv[1], encoding="utf-8", errors="ignore").read().splitlines()
skip_first = len(sys.argv) > 2 and sys.argv[2] == "--skip-first"
segs, cur = [], None
for l in lines:
    m = re.search(r"\[CMD\] (bw|freq|rssi)[^\n]*", l)
    if m:
        label = re.sub(r"\s+", " ", m.group(0)[6:]).strip()
        cur = {"label": label, "acc": 0, "rej": 0, "ids": set(), "rssi": []}
        segs.append(cur)
        continue
    if cur is None:
        continue
    if l.startswith("[RF-REJ]"):
        cur["rej"] += 1
    elif l.startswith("[RF] {"):
        try:
            d = json.loads(l[5:])
        except Exception:
            continue
        cur["acc"] += 1
        cur["ids"].add((d.get("protocol"), d.get("id")))
        if "rssi" in d:
            cur["rssi"].append(d["rssi"])
if skip_first and segs:
    segs = segs[1:]

agg = defaultdict(lambda: {"n": 0, "acc": 0, "ids": 0, "weak": 0, "min": 0, "rej": 0})
print("segments:")
for s in segs:
    weak = sum(1 for r in s["rssi"] if r <= -95)
    mn = min(s["rssi"]) if s["rssi"] else 0
    print(f"  {s['label']:<34} acc {s['acc']:>3}  unique {len(s['ids']):>3}  weak {weak:>3}  weakest {mn:>4}  rej {s['rej']}")
    a = agg[s["label"]]
    a["n"] += 1; a["acc"] += s["acc"]; a["ids"] += len(s["ids"]); a["weak"] += weak; a["rej"] += s["rej"]
    a["min"] = min(a["min"], mn)
print("\nper setting (averages per segment):")
for k, a in sorted(agg.items(), key=lambda kv: -kv[1]["weak"] / max(kv[1]["acc"], 1)):
    n = max(a["n"], 1)
    frac = 100.0 * a['weak'] / max(a['acc'], 1)
    print(f"  {k:<34} segs {a['n']}  acc/seg {a['acc']/n:6.1f}  weak/seg {a['weak']/n:5.1f}  weak% {frac:5.1f}  weakest {a['min']}")
