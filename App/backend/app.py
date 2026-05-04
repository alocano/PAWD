"""
PAWD Backend - Flask.

Changes from original:
- Multi-file support: /api/files lists available .txt data files,
  /api/raw-data?file=<name> selects which file to parse (defaults to mydata.txt).
- Duplicate tap filtering: taps with the same tap_num are deduplicated,
  keeping the first occurrence.
- Time normalization: tap and rotation times are shifted so the earliest
  event starts at t=0, matching the rotation waveform which already starts
  at 0.
- ADC values are now passed through on each tap entry (already parsed;
  exposed explicitly in the response for the frontend to display).
"""

import os
import re
from flask import Flask, jsonify, send_from_directory, request

_frontend = os.path.join(os.path.dirname(__file__), "..", "frontend")
app = Flask(__name__, static_folder=_frontend, static_url_path="")

# Directory where data files live (same folder as this module)
DATA_DIR = os.path.dirname(__file__)
DEFAULT_FILE = "mydata.txt"


def _parse_data_file(filepath):
    """
    Parse a PAWD data file and return structured JSON-ready dicts.

    Changes vs original:
    - Duplicate taps (same tap_num) are dropped — only the first occurrence
      is kept. This handles the case where hardware emits the same tap event
      twice.
    - Tap times are shifted by the minimum tap time so the first tap lands
      at t=0 (consistent with the rotation waveform origin).
    - ADC values are included explicitly in each tap entry.
    """
    sample_pattern = re.compile(r"\[SAMPLE\]\s+#(\d+)/(\d+)\s+(-?[\d.]+)\s+dps")
    rot_pattern    = re.compile(r"\[ROT\]\s+#(\d+)\s+T:([\d.]+)s\s+A:(-?[\d.]+)")
    tap_pattern    = re.compile(r"\[TAP\]\s+#(\d+)\s+T:([\d.]+)\s*(ms|s)?\s+Adc:([\d.]+)", re.IGNORECASE)

    blocks = []
    current_samples = []
    current_total = None
    taps_raw = []

    with open(filepath, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            ms = sample_pattern.match(line)
            mr = rot_pattern.match(line)
            mt = tap_pattern.match(line)
            if ms:
                idx   = int(ms.group(1))
                total = int(ms.group(2))
                dps   = float(ms.group(3))
                current_total = total
                current_samples.append((idx, dps))
            elif mr:
                blocks.append({
                    "rot_num":    int(mr.group(1)),
                    "duration":   float(mr.group(2)),
                    "amplitude":  float(mr.group(3)),
                    "samples":    current_samples,
                    "total_count": current_total or len(current_samples),
                })
                current_samples = []
                current_total = None
            elif mt:
                tap_time  = float(mt.group(2))
                time_unit = (mt.group(3) or "s").lower()
                if time_unit == "ms":
                    tap_time /= 1000.0
                taps_raw.append({
                    "tap_num": int(mt.group(1)),
                    "t":       tap_time,
                    "adc":     float(mt.group(4)),
                })

    # --- Build rotation/sample timeline ---
    samples_out   = []
    rotations_out = []
    cursor = 0.0

    for block in blocks:
        T        = block["duration"]
        n        = block["total_count"]
        interval = T / n if n > 0 else T

        for idx_1based, dps in block["samples"]:
            t = round(cursor + (idx_1based - 0.5) * interval, 4)
            samples_out.append({"t": t, "dps": dps})
        cursor += T

        rotations_out.append({
            "t":        round(cursor, 4),
            "rot_num":  block["rot_num"],
            "amplitude": block["amplitude"],
            "duration": T,
        })

    # --- Deduplicate taps: keep only the first occurrence of each tap_num ---
    seen_tap_nums = set()
    taps_deduped = []
    for tap in sorted(taps_raw, key=lambda x: (x["t"], x["tap_num"])):
        if tap["tap_num"] not in seen_tap_nums:
            seen_tap_nums.add(tap["tap_num"])
            taps_deduped.append(tap)

    taps_out = taps_deduped

    # Some logs label tap times with "s" but values are actually milliseconds.
    if taps_out and max(item["t"] for item in taps_out) > 300:
        for item in taps_out:
            item["t"] = round(item["t"] / 1000.0, 4)

    # --- Normalize tap times so the first tap starts at t=0 ---
    if taps_out:
        t_min = min(item["t"] for item in taps_out)
        if t_min > 0:
            for item in taps_out:
                item["t"] = round(item["t"] - t_min, 4)

    # Re-sort after normalization
    taps_out.sort(key=lambda x: x["t"])

    max_times = []
    if samples_out:
        max_times.append(max(item["t"] for item in samples_out))
    if rotations_out:
        max_times.append(max(item["t"] for item in rotations_out))
    if taps_out:
        max_times.append(max(item["t"] for item in taps_out))
    duration_seconds = round(max(max_times), 4) if max_times else 0.0

    return {
        "samples":          samples_out,
        "rotations":        rotations_out,
        "taps":             taps_out,
        "duration_seconds": duration_seconds,
    }


@app.route("/api/files", methods=["GET"])
def list_files():
    """
    Return a list of available .txt data files in the DATA_DIR.
    The frontend uses this to populate the file selector dropdown.
    Response: { "files": ["mydata.txt", "patient2.txt", ...] }
    """
    try:
        files = sorted(
            f for f in os.listdir(DATA_DIR)
            if f.endswith(".txt") and os.path.isfile(os.path.join(DATA_DIR, f))
        )
    except OSError:
        files = []
    return jsonify({"files": files})


@app.route("/api/raw-data", methods=["GET"])
def raw_data():
    """
    Parse and return sensor data for the requested file.

    Query parameter:
        file  (optional) — filename (basename only, no path traversal).
              Defaults to DEFAULT_FILE ("mydata.txt").

    Returns the same shape as before, with `adc` now explicit on tap entries:
    {
      "samples":   [{ "t", "dps" }, ...],
      "rotations": [{ "t", "rot_num", "amplitude", "duration" }, ...],
      "taps":      [{ "t", "tap_num", "adc" }, ...],
      "duration_seconds": 0.0,
      "file": "mydata.txt"
    }
    """
    requested = request.args.get("file", DEFAULT_FILE)

    # Security: strip any directory components so callers cannot path-traverse.
    requested = os.path.basename(requested)
    if not requested.endswith(".txt"):
        return jsonify({"error": "Only .txt files are supported."}), 400

    filepath = os.path.join(DATA_DIR, requested)
    if not os.path.exists(filepath):
        return jsonify({"error": f"Data file not found: {requested}"}), 404

    result = _parse_data_file(filepath)
    result["file"] = requested
    return jsonify(result)


@app.route("/")
def index():
    return send_from_directory(app.static_folder, "index.html")


if __name__ == "__main__":
    app.run(debug=True, port=5000)