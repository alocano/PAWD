"""
PAWD Backend - Flask.

Serves the frontend and exposes GET /api/raw-data, which parses mydata.txt
(produced by the serial receiver) for the live gyroscope chart.
"""

import os
import re
from flask import Flask, jsonify, send_from_directory

_frontend = os.path.join(os.path.dirname(__file__), "..", "frontend")
app = Flask(__name__, static_folder=_frontend, static_url_path="")

# Raw sensor data file (same folder as this module, or path from receive.py --outfile)
RAW_DATA_FILE = os.path.join(os.path.dirname(__file__), "mydata.txt")


@app.route("/api/raw-data", methods=["GET"])
def raw_data():
    """
    Parses the raw IMU text file and returns JSON ready for Chart.js.

    Each rotation block consists of N [SAMPLE] lines followed by one [ROT]
    summary line that gives the total duration T of that rotation.

    Returns:
    {
      "samples": [{ "t", "dps" }, ...],
            "rotations": [{ "t", "rot_num", "amplitude", "duration" }, ...],
      "taps": [{ "t", "tap_num", "adc" }, ...],
      "duration_seconds": 0.0
    }
    """
    if not os.path.exists(RAW_DATA_FILE):
        return jsonify({"error": f"Data file not found: {RAW_DATA_FILE}"}), 404

    sample_pattern = re.compile(r"\[SAMPLE\]\s+#(\d+)/(\d+)\s+(-?[\d.]+)\s+dps")
    rot_pattern = re.compile(r"\[ROT\]\s+#(\d+)\s+T:([\d.]+)s\s+A:(-?[\d.]+)")
    tap_pattern = re.compile(r"\[TAP\]\s+#(\d+)\s+T:([\d.]+)\s*(ms|s)?\s+Adc:([\d.]+)", re.IGNORECASE)

    blocks = []
    current_samples = []
    current_total = None
    taps = []

    with open(RAW_DATA_FILE, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            ms = sample_pattern.match(line)
            mr = rot_pattern.match(line)
            mt = tap_pattern.match(line)
            if ms:
                idx = int(ms.group(1))
                total = int(ms.group(2))
                dps = float(ms.group(3))
                current_total = total
                current_samples.append((idx, dps))
            elif mr:
                blocks.append({
                    "rot_num": int(mr.group(1)),
                    "duration": float(mr.group(2)),
                    "amplitude": float(mr.group(3)),
                    "samples": current_samples,
                    "total_count": current_total or len(current_samples),
                })
                current_samples = []
                current_total = None
            elif mt:
                tap_time = float(mt.group(2))
                time_unit = (mt.group(3) or "s").lower()
                if time_unit == "ms":
                    tap_time /= 1000.0
                taps.append({
                    "tap_num": int(mt.group(1)),
                    "t": tap_time,
                    "adc": float(mt.group(4)),
                })

    samples_out = []
    rotations_out = []
    cursor = 0.0

    for block in blocks:
        T = block["duration"]
        n = block["total_count"]
        interval = T / n if n > 0 else T

        for idx_1based, dps in block["samples"]:
            t = round(cursor + (idx_1based - 0.5) * interval, 4)
            samples_out.append({"t": t, "dps": dps})
        cursor += T

        rotations_out.append({
            "t": round(cursor, 4),
            "rot_num": block["rot_num"],
            "amplitude": block["amplitude"],
            "duration": T,
        })

    taps_out = sorted(taps, key=lambda item: (item["t"], item["tap_num"]))

    # Some logs label tap times with "s" but values are actually milliseconds.
    if taps_out and max(item["t"] for item in taps_out) > 300:
        for item in taps_out:
            item["t"] = round(item["t"] / 1000.0, 4)

    max_times = []
    if samples_out:
        max_times.append(max(item["t"] for item in samples_out))
    if rotations_out:
        max_times.append(max(item["t"] for item in rotations_out))
    if taps_out:
        max_times.append(max(item["t"] for item in taps_out))
    duration_seconds = round(max(max_times), 4) if max_times else 0.0

    return jsonify({
        "samples": samples_out,
        "rotations": rotations_out,
        "taps": taps_out,
        "duration_seconds": duration_seconds,
    })


@app.route("/")
def index():
    return send_from_directory(app.static_folder, "index.html")


if __name__ == "__main__":
    app.run(debug=True, port=5000)
