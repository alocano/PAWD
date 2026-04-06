(function () {
  const API = "/api";

  let chartSingle    = null;
  let chartFinger    = null;
  let chartPronation = null;

  const patientSelect         = document.getElementById("patientSelect");
  const newPatientId          = document.getElementById("newPatientId");
  const newPatientAge         = document.getElementById("newPatientAge");
  const createPatientBtn      = document.getElementById("createPatient");
  const resultsPlaceholder    = document.getElementById("resultsPlaceholder");
  const resultsContent        = document.getElementById("resultsContent");
  const resultsTableBody      = document.getElementById("resultsTableBody");
  const resultsChart          = document.getElementById("resultsChart");
  const chartTitle            = document.getElementById("chartTitle");
  const chartMode             = document.getElementById("chartMode");
  const chartSingleWrap       = document.getElementById("chartSingleWrap");
  const chartBothWrap         = document.getElementById("chartBothWrap");
  const resultsChartFinger    = document.getElementById("resultsChartFinger");
  const resultsChartPronation = document.getElementById("resultsChartPronation");

  // -------------------------------------------------------------------------
  // Patients
  // -------------------------------------------------------------------------

  function getSelectedPatientId() {
    return patientSelect.value || null;
  }

  function loadPatients() {
    fetch(API + "/patients")
      .then((r) => r.json())
      .then((list) => {
        const current = patientSelect.value;
        patientSelect.innerHTML = '<option value="">-- Select patient --</option>';
        list.forEach((p) => {
          const opt = document.createElement("option");
          opt.value = p.patient_id;
          opt.textContent = p.patient_id + (p.age != null ? " (age " + p.age + ")" : "");
          patientSelect.appendChild(opt);
        });
        if (current) patientSelect.value = current;
      })
      .catch((err) => console.error("Load patients failed", err));
  }

  createPatientBtn.addEventListener("click", function () {
    const patientId = (newPatientId.value || "").trim();
    if (!patientId) { alert("Enter a patient ID."); return; }
    fetch(API + "/patients", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        patient_id: patientId,
        age: newPatientAge.value ? parseInt(newPatientAge.value, 10) : null,
      }),
    })
      .then((r) => { if (!r.ok) return r.json().then((d) => Promise.reject(d)); return r.json(); })
      .then(() => {
        newPatientId.value  = "";
        newPatientAge.value = "";
        loadPatients();
        setTimeout(() => { patientSelect.value = patientId; refreshResults(); }, 100);
      })
      .catch((err) => alert(err.error || "Failed to create patient"));
  });

  // -------------------------------------------------------------------------
  // Assessment state
  //
  // State machine per test:
  //   idle → countdown (Start pressed) → running → idle (Stop / auto-stop)
  //
  // UPDRS score is always 0 (placeholder) until hardware sync via receive.py.
  // Auto-stop triggers after INACTIVITY_TIMEOUT_S seconds with no hardware
  // movement signal. Once hardware integration is complete, the 10-rep
  // completion signal from the ESP32 will also trigger an auto-stop.
  //
  // Countdown: 3-second visual countdown before timing begins.
  // -------------------------------------------------------------------------

  const COUNTDOWN_SECONDS    = 3;
  const INACTIVITY_TIMEOUT_S = 60;   // auto-stop if no hardware signal for this long

  const assessments = {
    finger_taps: {
      startBtn:    document.getElementById("startFinger"),
      stopBtn:     document.getElementById("stopFinger"),
      statusEl:    document.getElementById("statusFinger"),
      countdownEl: document.getElementById("countdownFinger"),
      label:       "Finger Tapping",
    },
    pronation_supination: {
      startBtn:    document.getElementById("startPronation"),
      stopBtn:     document.getElementById("stopPronation"),
      statusEl:    document.getElementById("statusPronation"),
      countdownEl: document.getElementById("countdownPronation"),
      label:       "Pronation–Supination",
    },
  };

  // Per-test timer handles
  const state = {
    finger_taps:          { phase: "idle", tickTimer: null, autoStopTimer: null, startTime: null, countdownTimer: null },
    pronation_supination: { phase: "idle", tickTimer: null, autoStopTimer: null, startTime: null, countdownTimer: null },
  };

  // ---- status helpers ----

  function showStatus(testType, message, style) {
    const el = assessments[testType].statusEl;
    el.textContent = message;
    el.className   = "assessment-status" + (style ? " " + style : "");
    el.hidden      = false;
  }

  function hideStatus(testType) {
    const el = assessments[testType].statusEl;
    el.hidden      = true;
    el.textContent = "";
    el.className   = "assessment-status";
  }

  function showCountdown(testType, n) {
    const el = assessments[testType].countdownEl;
    el.textContent = n > 0 ? n : "";
    el.hidden      = n <= 0;
  }

  function hideCountdown(testType) {
    const el = assessments[testType].countdownEl;
    el.hidden      = true;
    el.textContent = "";
  }

  // ---- button visibility ----

  function setRunning(testType, running) {
    const { startBtn, stopBtn } = assessments[testType];
    const other = testType === "finger_taps" ? "pronation_supination" : "finger_taps";
    if (running) {
      startBtn.disabled = true;
      stopBtn.disabled  = false;
      stopBtn.hidden    = false;
      assessments[other].startBtn.disabled = true;
    } else {
      startBtn.disabled = false;
      stopBtn.disabled  = true;
      stopBtn.hidden    = true;
      assessments[other].startBtn.disabled = false;
    }
  }

  // ---- timer helpers ----

  function clearAllTimers(testType) {
    const s = state[testType];
    if (s.tickTimer)      { clearInterval(s.tickTimer);   s.tickTimer      = null; }
    if (s.autoStopTimer)  { clearTimeout(s.autoStopTimer); s.autoStopTimer = null; }
    if (s.countdownTimer) { clearInterval(s.countdownTimer); s.countdownTimer = null; }
  }

  // ---- countdown then run ----

  function startCountdown(testType) {
    const s = state[testType];
    s.phase = "countdown";
    let count = COUNTDOWN_SECONDS;
    showCountdown(testType, count);
    hideStatus(testType);

    s.countdownTimer = setInterval(() => {
      count--;
      if (count > 0) {
        showCountdown(testType, count);
      } else {
        clearInterval(s.countdownTimer);
        s.countdownTimer = null;
        hideCountdown(testType);
        beginTest(testType);
      }
    }, 1000);
  }

  function beginTest(testType) {
    const s       = state[testType];
    s.phase       = "running";
    s.startTime   = Date.now();

    // Live elapsed tick
    s.tickTimer = setInterval(() => {
      const elapsed = Math.round((Date.now() - s.startTime) / 1000);
      showStatus(testType, `Running… ${elapsed}s elapsed`);
    }, 1000);
    showStatus(testType, "Running… 0s elapsed");

    // Inactivity auto-stop
    s.autoStopTimer = setTimeout(() => {
      finaliseTest(testType, "auto");
    }, INACTIVITY_TIMEOUT_S * 1000);
  }

  // ---- stop / finalise ----

  function finaliseTest(testType, reason) {
    const s   = state[testType];
    const pid = getSelectedPatientId();

    clearAllTimers(testType);
    hideCountdown(testType);
    s.phase = "idle";
    setRunning(testType, false);

    if (!pid) {
      showStatus(testType, "Error: no patient selected.", "status-err");
      return;
    }

    if (reason === "countdown_cancelled") {
      hideStatus(testType);
      return;
    }

    const durationMs = Date.now() - s.startTime;
    const duration   = parseFloat((durationMs / 1000).toFixed(2));

    if (duration < 0.5) {
      showStatus(testType, "Test too short (< 0.5s) — not saved. Please retry.", "status-warn");
      return;
    }

    const autoMsg = reason === "auto"
      ? " (auto-stopped after inactivity)"
      : "";

    showStatus(testType, "Saving…");

    const now = new Date();
    fetch(`${API}/patients/${encodeURIComponent(pid)}/tests`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        test_type:        testType,
        updrs_score:      0,        // Placeholder — real score set via hardware ingest
        duration_seconds: duration,
        test_date:        now.toISOString().slice(0, 10),
        test_time:        now.toTimeString().slice(0, 8),
      }),
    })
      .then((r) => { if (!r.ok) return r.json().then((d) => Promise.reject(d)); return r.json(); })
      .then(() => {
        showStatus(testType, `Saved — ${duration}s${autoMsg}. UPDRS score pending hardware sync.`, "status-ok");
        // Clear the saved confirmation after 6 seconds so the box goes away
        setTimeout(() => hideStatus(testType), 6000);
        refreshResults();
      })
      .catch((err) => {
        showStatus(testType, "Error saving: " + (err.error || "network error"), "status-err");
      });
  }

  // ---- button handlers ----

  function handleStart(testType) {
    if (!getSelectedPatientId()) {
      alert("Select or create a patient before starting a test.");
      return;
    }
    setRunning(testType, true);
    startCountdown(testType);
  }

  function handleStop(testType) {
    const s = state[testType];
    if (s.phase === "countdown") {
      // Cancelled during countdown — abort cleanly
      clearAllTimers(testType);
      hideCountdown(testType);
      s.phase = "idle";
      setRunning(testType, false);
      hideStatus(testType);
      return;
    }
    finaliseTest(testType, "manual");
  }

  Object.keys(assessments).forEach((testType) => {
    assessments[testType].startBtn.addEventListener("click", () => handleStart(testType));
    assessments[testType].stopBtn .addEventListener("click", () => handleStop(testType));
  });

  // -------------------------------------------------------------------------
  // Sortable table
  // -------------------------------------------------------------------------

  // sortState tracks the current column key and direction for the results table
  const sortState = { key: "test_date", dir: 1 };   // dir: 1 = asc, -1 = desc

  // Map th data-sort values to comparator functions
  const comparators = {
    test_type:        (a, b) => (a.test_type        || "").localeCompare(b.test_type        || ""),
    updrs_score:      (a, b) => Number(a.updrs_score)      - Number(b.updrs_score),
    duration_seconds: (a, b) => Number(a.duration_seconds) - Number(b.duration_seconds),
    test_date:        (a, b) => {
      const dtA = (a.test_date || "") + "T" + (a.test_time || "00:00:00");
      const dtB = (b.test_date || "") + "T" + (b.test_time || "00:00:00");
      return dtA < dtB ? -1 : dtA > dtB ? 1 : (a.id || 0) - (b.id || 0);
    },
    test_time: (a, b) => {
      const dtA = (a.test_date || "") + "T" + (a.test_time || "00:00:00");
      const dtB = (b.test_date || "") + "T" + (b.test_time || "00:00:00");
      return dtA < dtB ? -1 : dtA > dtB ? 1 : (a.id || 0) - (b.id || 0);
    },
  };

  // Called by refreshResults with the current raw tests array.
  // Also called directly by th click handlers with the cached lastTests.
  let lastTests = [];

  function renderTable(tests) {
    const sorted = tests.slice().sort((a, b) => {
      const fn = comparators[sortState.key] || comparators.test_date;
      return fn(a, b) * sortState.dir;
    });

    // Update header indicators
    document.querySelectorAll("#resultsTable th[data-sort]").forEach((th) => {
      const key = th.dataset.sort;
      th.classList.toggle("sort-active", key === sortState.key);
      if (key === sortState.key) {
        th.dataset.sortDir = sortState.dir === 1 ? "asc" : "desc";
      } else {
        delete th.dataset.sortDir;
      }
    });

    resultsTableBody.innerHTML = "";
    sorted.forEach((t) => {
      const tr = document.createElement("tr");
      tr.innerHTML =
        "<td>" + (tableLabels[t.test_type] || t.test_type) + "</td>" +
        "<td>" + Math.round(Number(t.updrs_score))          + "</td>" +
        "<td>" + Number(t.duration_seconds).toFixed(1)      + "</td>" +
        "<td>" + t.test_date                                + "</td>" +
        "<td>" + (t.test_time || "")                        + "</td>";
      resultsTableBody.appendChild(tr);
    });
  }

  // Wire th click handlers once DOM is ready
  document.querySelectorAll("#resultsTable th[data-sort]").forEach((th) => {
    th.style.cursor = "pointer";
    th.addEventListener("click", () => {
      const key = th.dataset.sort;
      if (sortState.key === key) {
        sortState.dir *= -1;   // Toggle direction
      } else {
        sortState.key = key;
        sortState.dir = 1;
      }
      renderTable(lastTests);
    });
  });

  // -------------------------------------------------------------------------
  // Chart helpers
  // -------------------------------------------------------------------------

  const tableLabels = {
    finger_taps:          "Finger Taps",
    pronation_supination: "Pronation-Supination",
  };

  function buildSeries(tests, testType) {
    const relevant = tests
      .filter((t) => t.test_type === testType)
      .slice()
      .sort((a, b) => {
        const dtA = (a.test_date || "") + "T" + (a.test_time || "00:00:00");
        const dtB = (b.test_date || "") + "T" + (b.test_time || "00:00:00");
        if (dtA < dtB) return -1;
        if (dtA > dtB) return 1;
        return (a.id || 0) - (b.id || 0);
      });

    const dateCount = {};
    const entries = relevant.map((t) => {
      const d = t.test_date || "?";
      dateCount[d] = (dateCount[d] || 0) + 1;
      let shortDate = d;
      try { shortDate = new Date(d + "T12:00:00").toLocaleDateString("en-GB", { day: "numeric", month: "short" }); } catch (e) {}
      return { ...t, _shortDate: shortDate, _dateKey: d };
    });

    const dateSeen = {};
    const labeled = entries.map((t) => {
      dateSeen[t._dateKey] = (dateSeen[t._dateKey] || 0) + 1;
      const label = dateCount[t._dateKey] > 1
        ? `${t._shortDate} #${dateSeen[t._dateKey]}`
        : t._shortDate;
      return { ...t, _label: label };
    });

    return {
      xLabels:      labeled.map((t) => t._label),
      scoreData:    labeled.map((t) => Math.round(Number(t.updrs_score))),
      durationData: labeled.map((t) => Number(t.duration_seconds).toFixed(1)),
    };
  }

  function baseChartOptions(durationData) {
    return {
      responsive:          true,
      maintainAspectRatio: false,
      layout: { padding: { bottom: 8 } },
      plugins: {
        tooltip: {
          callbacks: {
            afterLabel: (ctx) => {
              const dur = durationData && durationData[ctx.dataIndex];
              return dur != null ? `Duration: ${dur}s` : "";
            },
          },
        },
      },
      scales: {
        y: {
          min:   0,
          max:   4,
          ticks: { stepSize: 1 },
          title: { display: true, text: "UPDRS score (0–4)" },
        },
        x: {
          title: { display: true, text: "Test session" },
          ticks: { maxRotation: 30, autoSkip: true, maxTicksLimit: 10 },
        },
      },
    };
  }

  function renderChart(canvasEl, title, series) {
    if (!canvasEl) return null;
    const ctx = canvasEl.getContext("2d");
    return new Chart(ctx, {
      type: "line",
      data: {
        labels: series.xLabels,
        datasets: [{
          label:                "UPDRS score",
          data:                 series.scoreData,
          borderColor:          "rgb(54, 162, 235)",
          backgroundColor:      "rgba(54, 162, 235, 0.15)",
          pointBackgroundColor: "rgb(54, 162, 235)",
          fill:                 false,
          tension:              0.2,
          pointRadius:          6,
          pointHoverRadius:     8,
          pointBorderWidth:     2,
        }],
      },
      options: {
        ...baseChartOptions(series.durationData),
        plugins: {
          ...baseChartOptions(series.durationData).plugins,
          title:  { display: true, text: title, font: { size: 14, weight: "bold" } },
          legend: { display: false },
        },
      },
    });
  }

  function destroyChart(c) { if (c) c.destroy(); return null; }

  function renderCharts(tests) {
    const mode = (chartMode && chartMode.value) || "finger_taps";

    chartSingle    = destroyChart(chartSingle);
    chartFinger    = destroyChart(chartFinger);
    chartPronation = destroyChart(chartPronation);

    if (mode === "both") {
      if (chartSingleWrap) chartSingleWrap.hidden = true;
      if (chartBothWrap)   chartBothWrap.hidden   = false;
      if (chartTitle)      chartTitle.textContent  = "Severity over time – both tests";
      chartFinger    = renderChart(resultsChartFinger,    "Finger Taps",          buildSeries(tests, "finger_taps"));
      chartPronation = renderChart(resultsChartPronation, "Pronation–Supination", buildSeries(tests, "pronation_supination"));
      return;
    }

    if (chartSingleWrap) chartSingleWrap.hidden = false;
    if (chartBothWrap)   chartBothWrap.hidden   = true;

    const title = tableLabels[mode] ? `${tableLabels[mode]} – severity over time` : "Severity over time";
    if (chartTitle) chartTitle.textContent = title;
    chartSingle = renderChart(resultsChart, title, buildSeries(tests, mode));
  }

  // -------------------------------------------------------------------------
  // Results refresh
  // -------------------------------------------------------------------------

  function refreshResults() {
    const pid = getSelectedPatientId();
    if (!pid) {
      resultsPlaceholder.hidden = false;
      resultsContent.hidden     = true;
      return;
    }
    fetch(`${API}/patients/${encodeURIComponent(pid)}/tests`)
      .then((r) => r.json())
      .then((tests) => {
        if (tests.length === 0) {
          resultsPlaceholder.hidden = false;
          resultsContent.hidden     = true;
          lastTests = [];
          return;
        }
        resultsPlaceholder.hidden = true;
        resultsContent.hidden     = false;
        lastTests = tests;
        renderTable(tests);
        renderCharts(tests);
      })
      .catch((err) => console.error("Load results failed", err));
  }

  if (chartMode) chartMode.addEventListener("change", refreshResults);
  patientSelect.addEventListener("change", refreshResults);
  loadPatients();
  refreshResults();
})();