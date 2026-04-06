"""
PAWD Backend - Flask + SQLite.

Responsibilities:
  1. Serve the frontend static files.
  2. Manage patient records 
  3. Store and retrieve test results
  4. Accept sensor data POSTed by receive.py and insert into SQLite.

Data flow:
  ESP32 --> USB Serial --> receive.py --> POST /api/ingest --> SQLite
  Browser --> GET /api/patients/<id>/tests --> charts + table
"""

import os
import re
import sqlite3
from datetime import datetime, timezone
from flask import Flask, request, jsonify, send_from_directory
 
_frontend = os.path.join(os.path.dirname(__file__), "..", "frontend")
app = Flask(__name__, static_folder=_frontend, static_url_path="")
DB_PATH = os.path.join(os.path.dirname(__file__), "pawd.db")
 
 
# ---------------------------------------------------------------------------
# Database
# ---------------------------------------------------------------------------
 
def get_db():
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    return conn
 
 
def init_db():
    conn = get_db()
    conn.executescript("""
        CREATE TABLE IF NOT EXISTS patients (
            id         INTEGER PRIMARY KEY AUTOINCREMENT,
            patient_id TEXT UNIQUE NOT NULL,
            age        INTEGER
        );
        CREATE TABLE IF NOT EXISTS test_results (
            id               INTEGER PRIMARY KEY AUTOINCREMENT,
            patient_id       TEXT NOT NULL,
            test_type        TEXT NOT NULL,
            updrs_score      REAL NOT NULL,
            duration_seconds REAL NOT NULL,
            test_date        TEXT NOT NULL,
            test_time        TEXT NOT NULL DEFAULT '',
            FOREIGN KEY (patient_id) REFERENCES patients(patient_id)
        );
    """)
    # Migrations for existing databases that have the old columns
    for sql in [
        "ALTER TABLE test_results ADD COLUMN test_time TEXT NOT NULL DEFAULT ''",
    ]:
        try:
            conn.execute(sql)
            conn.commit()
        except Exception:
            pass
    conn.close()
 
 
# ---------------------------------------------------------------------------
# Shared ingest logic
# ---------------------------------------------------------------------------
 
# Matches lines like:  12345: TAP,2,14.3  or  67890: PRONATION,1,10.5
LINE_PATTERN = re.compile(
    r'^\d+:\s*(TAP|PRONATION),\s*(\d+(?:\.\d+)?),\s*(\d+(?:\.\d+)?)$',
    re.IGNORECASE
)
 
 
def parse_and_insert(conn, patient_id, content, date_str, time_str):
    """Parse raw ESP32 file content and insert valid rows into test_results.
 
    Returns (inserted, skipped) counts.
    """
    inserted = 0
    skipped  = 0
 
    for raw_line in content.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        m = LINE_PATTERN.match(line)
        if not m:
            skipped += 1
            continue
 
        keyword   = m.group(1).upper()
        test_type = "finger_taps" if keyword == "TAP" else "pronation_supination"
        score     = max(0, min(4, round(float(m.group(2)))))
        duration  = float(m.group(3))
 
        conn.execute(
            """INSERT INTO test_results
                   (patient_id, test_type, updrs_score, duration_seconds, test_date, test_time)
               VALUES (?, ?, ?, ?, ?, ?)""",
            (patient_id, test_type, score, duration, date_str, time_str),
        )
        inserted += 1
 
    conn.commit()
    return inserted, skipped
 
 
# ---------------------------------------------------------------------------
# Patients
# ---------------------------------------------------------------------------
 
@app.route("/api/patients", methods=["GET"])
def list_patients():
    conn = get_db()
    rows = conn.execute(
        "SELECT id, patient_id, age FROM patients ORDER BY patient_id"
    ).fetchall()
    conn.close()
    return jsonify([dict(r) for r in rows])
 
 
@app.route("/api/patients", methods=["POST"])
def create_patient():
    """Body: { "patient_id": str, "age": int|null }"""
    data       = request.get_json() or {}
    patient_id = (data.get("patient_id") or "").strip() or None
    age        = data.get("age")
 
    if not patient_id:
        return jsonify({"error": "patient_id required"}), 400
 
    conn = get_db()
    try:
        conn.execute(
            "INSERT INTO patients (patient_id, age) VALUES (?, ?)",
            (patient_id, age),
        )
        conn.commit()
        row = conn.execute(
            "SELECT id, patient_id, age FROM patients WHERE patient_id = ?",
            (patient_id,),
        ).fetchone()
        out = dict(row)
    except sqlite3.IntegrityError:
        conn.close()
        return jsonify({"error": "patient_id already exists"}), 400
    conn.close()
    return jsonify(out), 201
 
 
# ---------------------------------------------------------------------------
# Test results
# ---------------------------------------------------------------------------
 
@app.route("/api/patients/<patient_id>/tests", methods=["GET"])
def get_patient_tests(patient_id):
    conn = get_db()
    rows = conn.execute(
        """SELECT id, patient_id, test_type, updrs_score,
                  duration_seconds, test_date, test_time
           FROM test_results
           WHERE patient_id = ?
           ORDER BY test_date ASC, test_time ASC, id ASC""",
        (patient_id,),
    ).fetchall()
    conn.close()
    return jsonify([dict(r) for r in rows])
 
 
@app.route("/api/patients/<patient_id>/tests", methods=["POST"])
def add_test_result(patient_id):
    """Body: { test_type, updrs_score, duration_seconds, test_date?, test_time? }"""
    data             = request.get_json() or {}
    test_type        = (data.get("test_type") or "").strip()
    updrs_score      = data.get("updrs_score", 0)
    duration_seconds = data.get("duration_seconds", 0)
    now              = datetime.now(timezone.utc)
    test_date        = (data.get("test_date") or now.isoformat()[:10]).strip()
    test_time        = (data.get("test_time") or now.strftime("%H:%M:%S")).strip()
 
    if test_type not in ("finger_taps", "pronation_supination"):
        return jsonify({"error": "test_type must be finger_taps or pronation_supination"}), 400
 
    updrs_int = max(0, min(4, round(float(updrs_score))))
 
    conn = get_db()
    conn.execute(
        """INSERT INTO test_results
               (patient_id, test_type, updrs_score, duration_seconds, test_date, test_time)
           VALUES (?, ?, ?, ?, ?, ?)""",
        (patient_id, test_type, updrs_int, float(duration_seconds), test_date, test_time),
    )
    conn.commit()
    row = conn.execute(
        """SELECT id, patient_id, test_type, updrs_score,
                  duration_seconds, test_date, test_time
           FROM test_results ORDER BY id DESC LIMIT 1"""
    ).fetchone()
    conn.close()
    return jsonify(dict(row)), 201
 
 
# ---------------------------------------------------------------------------
# Hardware ingest — JSON body (used by receive.py serial bridge)
# ---------------------------------------------------------------------------
 
@app.route("/api/ingest", methods=["POST"])
def ingest():
    """
    Called by receive.py after each serial file transfer.
    Body: { "patient_id": str, "content": "<raw file text>" }
 
    Each line format from the ESP32 (written by sensor firmware to /data.txt):
        <millis>: TAP,<score>,<duration>
        <millis>: PRONATION,<score>,<duration>
    e.g.  "12345: TAP,2,14.3"
    """
    data       = request.get_json() or {}
    patient_id = (data.get("patient_id") or "").strip()
    content    = data.get("content", "")
 
    if not patient_id:
        return jsonify({"error": "patient_id required"}), 400
    if not content:
        return jsonify({"error": "content required"}), 400
 
    now      = datetime.now(timezone.utc)
    date_str = now.isoformat()[:10]
    time_str = now.strftime("%H:%M:%S")
 
    conn = get_db()
    inserted, skipped = parse_and_insert(conn, patient_id, content, date_str, time_str)
    conn.close()
    return jsonify({"inserted": inserted, "skipped": skipped})
 
 
# ---------------------------------------------------------------------------
# Serve frontend
# ---------------------------------------------------------------------------
 
@app.route("/")
def index():
    return send_from_directory(app.static_folder, "index.html")
 
 
if __name__ == "__main__":
    init_db()
    # use_reloader=False prevents the Werkzeug file-watcher thread from raising
    # an exception on shutdown (a known issue on Windows with Python 3.12+).
    app.run(debug=True, port=5000, use_reloader=False)