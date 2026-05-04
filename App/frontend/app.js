(function () {
  const API = "/api";

  let activeChart    = null;
  let rotationChart  = null;
  let tapChart       = null;
  let latestRawData  = null;

  const rawPlaceholder           = document.getElementById("rawPlaceholder");
  const rawContent               = document.getElementById("rawContent");
  const rawSummaryTaps           = document.getElementById("rawSummaryTaps");
  const rawSummaryTapDuration    = document.getElementById("rawSummaryTapDuration");
  const rawSummaryRotations      = document.getElementById("rawSummaryRotations");
  const rawSummaryRotationDuration = document.getElementById("rawSummaryRotationDuration");
  const testCanvas               = document.getElementById("testChart");
  const rotationCanvas           = document.getElementById("rotationChart");
  const tapCanvas                = document.getElementById("tapChart");
  const chartSingleWrap          = document.getElementById("chartSingleWrap");
  const chartBothWrap            = document.getElementById("chartBothWrap");
  const testMode                 = document.getElementById("testMode");
  const graphTitle               = document.getElementById("graphTitle");
  const graphDescription         = document.getElementById("graphDescription");
  const fileSelect               = document.getElementById("fileSelect");
  const noDataMessage            = document.getElementById("noDataMessage");
  const noDataRotation           = document.getElementById("noDataRotation");
  const noDataTap                = document.getElementById("noDataTap");

  // ── Helpers ────────────────────────────────────────────────────────────────

  function formatSeconds(value) {
    if (!Number.isFinite(value)) return "0.0 s";
    return `${value.toFixed(1)} s`;
  }

  function destroyChart(chart) {
    if (chart) chart.destroy();
    return null;
  }

  function getModeMeta(mode) {
    if (!mode) {
      return {
        title: "Select a test",
        description: "Choose Finger Taps, Pronation-Supination, or Both to display the graph.",
      };
    }
    if (mode === "finger_taps") {
      return {
        title: "Finger taps",
        description: "Each dot marks a finger tap detected by the sensor.                 ",
      };
    }
    if (mode === "both") {
      return {
        title: "Finger taps and Pronation-Supination Tests",
        description: "Displays separate Pronation-Supination and Finger Taps graphs.",
      };
    }
    return {
      title: "Pronation-Supination",
      description: "Gyroscope trace with dot markers for each full rotation detected.",
    };
  }

  function toFiniteNumber(value) {
    const num = Number(value);
    return Number.isFinite(num) ? num : null;
  }

  /**
   * When multiple events land on the exact same timestamp, spread their
   * y-positions slightly so they don't overlap on the scatter chart.
   */
  function spreadDuplicateMarkers(items, getX, spreadStep) {
    const groups = new Map();
    items.forEach((item) => {
      const x = toFiniteNumber(getX(item));
      if (x == null) return;
      const key = x.toFixed(4);
      if (!groups.has(key)) groups.set(key, []);
      groups.get(key).push({ x, meta: item });
    });

    const out = [];
    groups.forEach((groupItems) => {
      const total = groupItems.length;
      groupItems.forEach((entry, idx) => {
        const offset = total === 1 ? 0 : (idx - (total - 1) / 2) * spreadStep;
        out.push({ x: entry.x, y: Number(offset.toFixed(4)), meta: entry.meta });
      });
    });

    out.sort((a, b) => a.x - b.x);
    return out;
  }

  function computeTapDurationSeconds(taps) {
    const times = taps
      .map((tap) => toFiniteNumber(tap.t))
      .filter((v) => v != null)
      .sort((a, b) => a - b);

    if (times.length === 0) return 0;
    if (times.length === 1) return times[0];
    return Math.max(0, times[times.length - 1] - times[0]);
  }

  function computeRotationDurationSeconds(samples, rotations) {
    const sampleMax = samples
      .map((s) => toFiniteNumber(s.t))
      .filter((v) => v != null)
      .reduce((max, v) => Math.max(max, v), 0);

    const rotationMax = rotations
      .map((r) => toFiniteNumber(r.t))
      .filter((v) => v != null)
      .reduce((max, v) => Math.max(max, v), 0);

    return Math.max(sampleMax, rotationMax);
  }

  // ── Dataset builders ───────────────────────────────────────────────────────

  function buildPronationDatasets(data) {
    const samples   = Array.isArray(data.samples)   ? data.samples   : [];
    const rotations = Array.isArray(data.rotations) ? data.rotations : [];
    const datasets  = [];

    if (samples.length > 0) {
      datasets.push({
        kind:            "gyro",
        label:           "Gyro trace",
        data:            samples
          .map((s) => ({ x: toFiniteNumber(s.t), y: toFiniteNumber(s.dps) }))
          .filter((s) => s.x != null && s.y != null),
        type:            "line",
        borderColor:     "rgb(54, 162, 235)",
        backgroundColor: "rgba(54, 162, 235, 0.08)",
        pointRadius:     0,
        borderWidth:     2,
        fill:            false,
        tension:         0.25,
        showLine:        true,
        order:           3,
      });
    }

    if (rotations.length > 0) {
      datasets.push({
        kind:             "rotation",
        label:            "Full Rotation detected",
        data:             spreadDuplicateMarkers(rotations, (r) => r.t, 0.12),
        type:             "scatter",
        pointStyle:       "circle",
        pointRadius:      6,
        pointHoverRadius: 9,
        pointBorderWidth: 2,
        backgroundColor:  "rgba(239, 68, 68, 0.9)",
        borderColor:      "rgb(153, 27, 27)",
        order:            1,
      });
    }

    return datasets;
  }

  function buildTapDatasets(data) {
    const taps = Array.isArray(data.taps) ? data.taps : [];
    if (taps.length === 0) return [];

    return [{
      kind:             "tap",
      label:            "Tap Detected",
      data:             spreadDuplicateMarkers(taps, (tap) => tap.t, 0.14),
      type:             "scatter",
      pointStyle:       "circle",
      pointRadius:      6,
      pointHoverRadius: 9,
      pointBorderWidth: 2,
      backgroundColor:  "rgba(16, 185, 129, 0.9)",
      borderColor:      "rgb(4, 120, 87)",
      order:            2,
    }];
  }

  // ── Chart options ──────────────────────────────────────────────────────────

  /**
   * Build Chart.js options.
   *
   * Changes vs original:
   * - xMin is forced to 0 so the axis always starts from 0 s (items 5 & 6).
   * - Tap tooltips now include the ADC value and "since last tap" (item 7).
   * - Rotation tooltips unchanged.
   */
  function chartOptions(isTapOnly, allTapsSorted) {
    return {
      responsive:          true,
      maintainAspectRatio: false,
      animation:           false,
      scales: {
        x: {
          type:  "linear",
          min:   0,           // always start x-axis at 0 s
          title: { display: true, text: "Time (s)", font: { size: 14 } },
          ticks: {
            callback: (value) => `${Number(value).toFixed(1)}s`,
            font:     { size: 13 },
          },
          grid: { color: "rgba(0,0,0,0.06)" },
        },
        y: isTapOnly
          ? {
              min:     -1,
              max:     1,
              display: false,
              grid:    { display: false },
            }
          : {
              title: { display: true, text: "Angular velocity (dps)", font: { size: 14 } },
              ticks: { font: { size: 13 } },
              grid:  { color: "rgba(0,0,0,0.06)" },
            },
      },
      plugins: {
        title:  { display: false },
        legend: { position: "top", labels: { font: { size: 14 } } },
        tooltip: {
          callbacks: {
            label(ctx) {
              const kind = ctx.dataset.kind;

              if (kind === "rotation") {
                const marker = ctx.raw.meta;
                return [
                  `Rotation #${marker.rot_num}`,
                  `Duration: ${formatSeconds(marker.duration)}`,
                  `Time: ${formatSeconds(marker.t)}`,
                ];
              }

              if (kind === "tap") {
                const tap   = ctx.raw.meta;
                const lines = [
                  `Tap #${tap.tap_num}`,
                  `Time: ${formatSeconds(tap.t)}`,
                  `ADC: ${tap.adc}`,
                ];

                // "Since last tap" — find the previous tap in the sorted list
                if (allTapsSorted && allTapsSorted.length > 1) {
                  const idx = allTapsSorted.findIndex((t) => t.tap_num === tap.tap_num);
                  if (idx > 0) {
                    const prev  = allTapsSorted[idx - 1];
                    const delta = tap.t - prev.t;
                    if (Number.isFinite(delta) && delta >= 0) {
                      lines.push(`Since last tap: ${formatSeconds(delta)}`);
                    }
                  }
                }

                return lines;
              }

              // Gyro trace
              return `${ctx.parsed.y.toFixed(1)} dps at ${formatSeconds(ctx.parsed.x)}`;
            },
          },
        },
      },
    };
  }

  // ── No-data helpers ────────────────────────────────────────────────────────

  function showNoData(msgEl, canvasEl) {
    if (msgEl)    msgEl.hidden    = false;
    if (canvasEl) canvasEl.hidden = true;
  }

  function hideNoData(msgEl, canvasEl) {
    if (msgEl)    msgEl.hidden    = true;
    if (canvasEl) canvasEl.hidden = false;
  }

  function clearCanvas(canvas) {
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    ctx.clearRect(0, 0, canvas.width, canvas.height);
  }

  // ── Chart visibility ───────────────────────────────────────────────────────

  function setChartVisibility(mode) {
    if (!chartSingleWrap || !chartBothWrap) return;
    if (mode === "both") {
      chartSingleWrap.hidden = true;
      chartBothWrap.hidden   = false;
      return;
    }
    if (mode === "finger_taps" || mode === "pronation_supination") {
      chartSingleWrap.hidden = false;
      chartBothWrap.hidden   = true;
      return;
    }
    chartSingleWrap.hidden = true;
    chartBothWrap.hidden   = true;
  }

  // ── Main render ────────────────────────────────────────────────────────────

  function renderActiveChart(data) {
    const mode     = (testMode && testMode.value) || "";
    const modeMeta = getModeMeta(mode);

    if (graphTitle)       graphTitle.textContent       = modeMeta.title;
    if (graphDescription) graphDescription.textContent = modeMeta.description;

    setChartVisibility(mode);

    activeChart   = destroyChart(activeChart);
    rotationChart = destroyChart(rotationChart);
    tapChart      = destroyChart(tapChart);

    // Hide all no-data messages initially
    if (noDataMessage)  noDataMessage.hidden  = true;
    if (noDataRotation) noDataRotation.hidden = true;
    if (noDataTap)      noDataTap.hidden      = true;

    if (!mode) {
      clearCanvas(testCanvas);
      clearCanvas(rotationCanvas);
      clearCanvas(tapCanvas);
      return;
    }

    // Pre-sort taps by time for "since last tap" calculation
    const allTapsSorted = Array.isArray(data.taps)
      ? [...data.taps].sort((a, b) => a.t - b.t)
      : [];

    if (mode === "both") {
      if (!rotationCanvas || !tapCanvas) return;

      const pronationDatasets = buildPronationDatasets(data);
      if (pronationDatasets.length > 0) {
        hideNoData(noDataRotation, rotationCanvas);
        rotationChart = new Chart(rotationCanvas.getContext("2d"), {
          type:    "scatter",
          data:    { datasets: pronationDatasets },
          options: chartOptions(false, allTapsSorted),
        });
      } else {
        // Show "no data" overlay inside the rotation chart frame
        showNoData(noDataRotation, rotationCanvas);
      }

      const tapDatasets = buildTapDatasets(data);
      if (tapDatasets.length > 0) {
        hideNoData(noDataTap, tapCanvas);
        tapChart = new Chart(tapCanvas.getContext("2d"), {
          type:    "scatter",
          data:    { datasets: tapDatasets },
          options: chartOptions(true, allTapsSorted),
        });
      } else {
        showNoData(noDataTap, tapCanvas);
      }
      return;
    }

    if (!testCanvas) return;

    const datasets   = mode === "finger_taps"
      ? buildTapDatasets(data)
      : buildPronationDatasets(data);
    const isTapOnly  = mode === "finger_taps";

    if (datasets.length === 0) {
      // Show "no data" overlay centred in the single chart frame
      showNoData(noDataMessage, testCanvas);
      // Also make the wrapper visible so the overlay has somewhere to live
      chartSingleWrap.hidden = false;
      return;
    }

    hideNoData(noDataMessage, testCanvas);
    activeChart = new Chart(testCanvas.getContext("2d"), {
      type:    "scatter",
      data:    { datasets },
      options: chartOptions(isTapOnly, allTapsSorted),
    });
  }

  // ── Data loading ───────────────────────────────────────────────────────────

  /**
   * Load sensor data for the currently selected file.
   * Passes ?file=<name> to the backend so it knows which .txt to parse.
   */
  function loadRawChart() {
    const selectedFile = fileSelect ? fileSelect.value : "";
    const url          = selectedFile
      ? `${API}/raw-data?file=${encodeURIComponent(selectedFile)}`
      : `${API}/raw-data`;

    rawPlaceholder.hidden = false;
    rawContent.hidden     = true;
    rawPlaceholder.textContent = "Loading sensor data...";

    fetch(url)
      .then((r) => {
        if (!r.ok) throw new Error(`HTTP ${r.status}`);
        return r.json();
      })
      .then((data) => {
        if (!data || typeof data !== "object") {
          throw new Error("Unexpected API response");
        }
        if (data.error) {
          rawPlaceholder.textContent = "Error: " + data.error;
          return;
        }

        const hasRawData =
          (data.samples   && data.samples.length   > 0) ||
          (data.rotations && data.rotations.length > 0) ||
          (data.taps      && data.taps.length      > 0);

        if (!hasRawData) {
          rawPlaceholder.textContent = "No raw sensor data found in this file.";
          return;
        }

        rawPlaceholder.hidden = true;
        rawContent.hidden     = false;

        const taps      = Array.isArray(data.taps)      ? data.taps      : [];
        const samples   = Array.isArray(data.samples)   ? data.samples   : [];
        const rotations = Array.isArray(data.rotations) ? data.rotations : [];

        rawSummaryTaps.textContent = String(taps.length);
        if (rawSummaryTapDuration) {
          rawSummaryTapDuration.textContent = formatSeconds(computeTapDurationSeconds(taps));
        }
        rawSummaryRotations.textContent = String(rotations.length);
        if (rawSummaryRotationDuration) {
          rawSummaryRotationDuration.textContent =
            formatSeconds(computeRotationDurationSeconds(samples, rotations));
        }

        latestRawData = data;
        renderActiveChart(data);
      })
      .catch((err) => {
        rawPlaceholder.textContent = "Could not load sensor data.";
        console.error("Raw data load failed", err);
      });
  }

  /**
   * Fetch the list of available .txt files from /api/files and populate
   * the file selector <select>.  Once loaded, trigger an initial data load
   * for the first file in the list (or mydata.txt if present).
   */
  function loadFileList() {
    fetch(`${API}/files`)
      .then((r) => {
        if (!r.ok) throw new Error(`HTTP ${r.status}`);
        return r.json();
      })
      .then((data) => {
        const files = Array.isArray(data.files) ? data.files : [];

        if (!fileSelect) return;

        fileSelect.innerHTML = "";

        if (files.length === 0) {
          const opt    = document.createElement("option");
          opt.value    = "";
          opt.textContent = "No data files found";
          fileSelect.appendChild(opt);
          rawPlaceholder.textContent = "No data files found in the backend directory.";
          return;
        }

        files.forEach((filename) => {
          const opt    = document.createElement("option");
          opt.value    = filename;
          // Strip the .txt extension for a cleaner label
          opt.textContent = filename.replace(/\.txt$/i, "");
          fileSelect.appendChild(opt);
        });

        // Prefer mydata.txt as the default selection if it exists
        const preferred = files.find((f) => f === "mydata.txt") || files[0];
        fileSelect.value = preferred;

        loadRawChart();
      })
      .catch((err) => {
        console.error("Could not load file list", err);
        // Fall back to loading the default file
        loadRawChart();
      });
  }

  // ── Event listeners ────────────────────────────────────────────────────────

  if (testMode) {
    testMode.addEventListener("change", function () {
      if (latestRawData) renderActiveChart(latestRawData);
    });
  }

  if (fileSelect) {
    fileSelect.addEventListener("change", function () {
      // Destroy existing charts before reloading so canvas contexts are clean
      activeChart   = destroyChart(activeChart);
      rotationChart = destroyChart(rotationChart);
      tapChart      = destroyChart(tapChart);
      latestRawData = null;
      loadRawChart();
    });
  }

  // ── Boot ───────────────────────────────────────────────────────────────────

  loadFileList();
})();