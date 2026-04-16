(function () {
  const API = "/api";

  let chartSingle    = null;
  let chartFinger    = null;
  let chartPronation = null;
  let chartRaw       = null;

  // -------------------------------------------------------------------------
  // Raw sensor chart
  // -------------------------------------------------------------------------

  const rawPlaceholder = document.getElementById("rawPlaceholder");
  const rawChartWrap   = document.getElementById("rawChartWrap");
  const rawCanvas      = document.getElementById("rawChart");

  function loadRawChart() {
    fetch(API + "/raw-data")
      .then((r) => r.json())
      .then((data) => {
        if (data.error) {
          rawPlaceholder.textContent = "Error: " + data.error;
          return;
        }

        rawPlaceholder.hidden = true;
        rawChartWrap.hidden   = false;

        if (chartRaw) { chartRaw.destroy(); chartRaw = null; }

        const samples   = data.samples;    // [{ t, dps }, ...]
        const rotations = data.rotations;  // [{ t, rot_num, amplitude, duration }, ...]

        // Main gyroscope line — one point per sample
        const lineData = samples.map((s) => ({ x: s.t, y: s.dps }));

        // Rotation markers — scatter points sitting ON the line at y=0
        // with a tooltip showing rotation details
        const markerData = rotations.map((r) => ({ x: r.t, y: 0, meta: r }));

        const ctx = rawCanvas.getContext("2d");
        chartRaw = new Chart(ctx, {
          type: "scatter",  // scatter base lets us mix line + point datasets
          data: {
            datasets: [
              {
                // Gyroscope oscillation line
                label:           "Gyro (dps)",
                data:            lineData,
                type:            "line",
                borderColor:     "rgb(54, 162, 235)",
                backgroundColor: "rgba(54, 162, 235, 0.08)",
                pointRadius:     2,
                pointHoverRadius: 4,
                borderWidth:     1.5,
                fill:            false,
                tension:         0.3,
                showLine:        true,
                order:           2,
              },
              {
                // Rotation detected markers — large dots at y=0
                label:                "Rotation detected",
                data:                 markerData,
                type:                 "scatter",
                pointStyle:           "circle",
                pointRadius:          8,
                pointHoverRadius:     11,
                pointBorderWidth:     2,
                backgroundColor:      "rgba(255, 99, 132, 0.85)",
                borderColor:          "rgb(255, 99, 132)",
                order:                1,
              },
            ],
          },
          options: {
            responsive:          true,
            maintainAspectRatio: false,
            scales: {
              x: {
                type:  "linear",
                title: { display: true, text: "Time (s)" },
                ticks: { callback: (v) => v.toFixed(2) + "s" },
              },
              y: {
                title: { display: true, text: "Angular velocity (dps)" },
                grid:  { color: "rgba(0,0,0,0.06)" },
              },
            },
            plugins: {
              title: {
                display: true,
                text:    "Gyroscope signal – rotation markers at y=0",
                font:    { size: 14, weight: "bold" },
              },
              legend: { position: "top" },
              tooltip: {
                callbacks: {
                  // Richer tooltip for rotation markers
                  label: function (ctx) {
                    if (ctx.datasetIndex === 1) {
                      const m = ctx.raw.meta;
                      return [
                        `Rotation #${m.rot_num}`,
                        `Duration: ${m.duration}s`,
                        `Amplitude: ${m.amplitude}°`,
                        `At: ${m.t.toFixed(3)}s`,
                      ];
                    }
                    return `${ctx.parsed.y.toFixed(1)} dps at ${ctx.parsed.x.toFixed(3)}s`;
                  },
                },
              },
            },
          },
        });
      })
      .catch((err) => {
        rawPlaceholder.textContent = "Could not load sensor data.";
        console.error("Raw data load failed", err);
      });
  }

  loadRawChart();

  const patientSelect       = document.getElementById("patientSelect");
  const newPatientId        = document.getElementById("newPatientId");
  const newPatientAge       = document.getElementById("newPatientAge");
  const createPatientBtn    = document.getElementById("createPatient");
  const resultsPlaceholder  = document.getElementById("resultsPlaceholder");
  const resultsContent      = document.getElementById("resultsContent");
  const resultsTableBody    = document.getElementById("resultsTableBody");
  const resultsChart        = document.getElementById("resultsChart");
  const chartTitle          = document.getElementById("chartTitle");
  const chartMode           = document.getElementById("chartMode");
  const chartSingleWrap     = document.getElementById("chartSingleWrap");
  const chartBothWrap       = document.getElementById("chartBothWrap");
  const resultsChartFinger  = document.getElementById("resultsChartFinger");
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
        patientSelect.innerHTML = '<option value="">-- Select patient --</option>';
        list.forEach((p) => {
          const opt = document.createElement("option");
          opt.value = p.patient_id;
          opt.textContent = p.patient_id + (p.age != null ? " (age " + p.age + ")" : "");
          patientSelect.appendChild(opt);
        });
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
        patientSelect.value = patientId;
      })
      .catch((err) => alert(err.error || "Failed to create patient"));
  });

  // -------------------------------------------------------------------------
  // Simulated test buttons (right hand only)
  // -------------------------------------------------------------------------

  function addMockResult(testType) {
    const pid = getSelectedPatientId();
    if (!pid) { alert("Select or create a patient first."); return; }
    const updrs    = Math.floor(Math.random() * 5);
    const duration = (5 + Math.random() * 15).toFixed(1);
    const now      = new Date();

    fetch(`${API}/patients/${encodeURIComponent(pid)}/tests`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        test_type:        testType,
        updrs_score:      updrs,
        duration_seconds: parseFloat(duration),
        test_date:        now.toISOString().slice(0, 10),
        test_time:        now.toTimeString().slice(0, 8),
      }),
    })
      .then((r) => r.json())
      .then(() => refreshResults())
      .catch((err) => console.error("Add test failed", err));
  }

  document.getElementById("startFinger")   .addEventListener("click", () => addMockResult("finger_taps"));
  document.getElementById("startPronation").addEventListener("click", () => addMockResult("pronation_supination"));

  // -------------------------------------------------------------------------
  // Chart helpers
  // -------------------------------------------------------------------------

  const labels = {
    finger_taps:          "Finger Taps",
    pronation_supination: "Pronation-Supination",
  };

  /**
   * buildSeries(tests, testType)
   * Returns { xLabels, scoreData } — one data point per test entry,
   * sorted chronologically. Same-day entries get numbered labels.
   */
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
      try {
        shortDate = new Date(d + "T12:00:00").toLocaleDateString("en-GB", { day: "numeric", month: "short" });
      } catch (e) {}
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
      xLabels:   labeled.map((t) => t._label),
      scoreData: labeled.map((t) => Math.round(Number(t.updrs_score))),
    };
  }

  function baseChartOptions() {
    return {
      responsive:          true,
      maintainAspectRatio: false,
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
        ...baseChartOptions(),
        plugins: {
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

    const title = labels[mode] ? `${labels[mode]} – severity over time` : "Severity over time";
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
          return;
        }
        resultsPlaceholder.hidden = true;
        resultsContent.hidden     = false;

        resultsTableBody.innerHTML = "";
        tests.forEach((t) => {
          const tr = document.createElement("tr");
          tr.innerHTML =
            "<td>" + (labels[t.test_type] || t.test_type)  + "</td>" +
            "<td>" + Math.round(Number(t.updrs_score))      + "</td>" +
            "<td>" + t.duration_seconds                     + "</td>" +
            "<td>" + t.test_date                            + "</td>";
          resultsTableBody.appendChild(tr);
        });

        renderCharts(tests);
      })
      .catch((err) => console.error("Load results failed", err));
  }

  if (chartMode) chartMode.addEventListener("change", refreshResults);
  patientSelect.addEventListener("change", refreshResults);
  loadPatients();
  refreshResults();
})();