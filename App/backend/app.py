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
      "rotations": [{ "t", "rot_num", "amplitude", "duration" }, ...]
    }
    """
    if not os.path.exists(RAW_DATA_FILE):
        return jsonify({"error": f"Data file not found: {RAW_DATA_FILE}"}), 404

    sample_pattern = re.compile(r"\[SAMPLE\]\s+#(\d+)/(\d+)\s+(-?[\d.]+)\s+dps")
    rot_pattern = re.compile(r"\[ROT\]\s+#(\d+)\s+T:([\d.]+)s\s+A:(-?[\d.]+)")

    blocks = []
    current_samples = []
    current_total = None

    with open(RAW_DATA_FILE, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            ms = sample_pattern.match(line)
            mr = rot_pattern.match(line)
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

    return jsonify({"samples": samples_out, "rotations": rotations_out})


@app.route("/")
def index():
    return send_from_directory(app.static_folder, "index.html")


if __name__ == "__main__":
    app.run(debug=True, port=5000)
