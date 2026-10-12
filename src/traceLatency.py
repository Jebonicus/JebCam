import re
import pandas as pd
import matplotlib.pyplot as plt

# 🏴‍☠️ Regex to catch each log line’s treasures
pattern = re.compile(
    r"element-latency, element-id=\(string\)(?P<id>\S+), "
    r"element=\(string\)(?P<element>\S+), "
    r"src=\(string\)\S+, "
    r"time=\(guint64\)(?P<time>\d+), "
    r"ts=\(guint64\)(?P<ts>\d+)"
)

records = []
with open("traces.log") as f:
    for line in f:
        m = pattern.search(line)
        if m:
            d = m.groupdict()
            d["time_ms"] = int(d["time"]) / 1e6  # convert ns → ms
            d["ts_ms"] = int(d["ts"]) / 1e6
            records.append(d)

df = pd.DataFrame(records)

if df.empty:
    raise SystemExit("No element-latency entries found, matey!")

# ⚓ Average latency per element
summary = (
    df.groupby("element")["time_ms"]
    .agg(["count", "mean", "min", "max"])
    .sort_values("mean", ascending=False)
)
print(summary.round(3))

# ☠️ Optional: visualize latency by element
plt.figure(figsize=(8,6))
plt.barh(summary.index, summary["mean"])
plt.xlabel("Average Latency (ms)")
plt.ylabel("Element")
plt.title("Average GStreamer Element Latency")
plt.gca().invert_yaxis()
plt.tight_layout()
plt.show()
