(function () {
  const API = "/api";

  let activeChart    = null;
  let rotationChart  = null;
  let tapChart       = null;
  let latestRawData  = null;

  // ── Raw-data DOM refs ───────────────────────────────────────────────────────
  const rawPlaceholder             = document.getElementById("rawPlaceholder");
  const rawContent                 = document.getElementById("rawContent");
  const rawSummaryTaps             = document.getElementById("rawSummaryTaps");
  const rawSummaryTapDuration      = document.getElementById("rawSummaryTapDuration");
  const rawSummaryRotations        = document.getElementById("rawSummaryRotations");
  const rawSummaryRotationDuration = document.getElementById("rawSummaryRotationDuration");
  const testCanvas                 = document.getElementById("testChart");
  const rotationCanvas             = document.getElementById("rotationChart");
  const tapCanvas                  = document.getElementById("tapChart");
  const chartSingleWrap            = document.getElementById("chartSingleWrap");
  const chartBothWrap              = document.getElementById("chartBothWrap");
  const testMode                   = document.getElementById("testMode");
  const graphTitle                 = document.getElementById("graphTitle");
  const graphDescription           = document.getElementById("graphDescription");
  const fileSelect                 = document.getElementById("fileSelect");
  const noDataMessage              = document.getElementById("noDataMessage");
  const noDataRotation             = document.getElementById("noDataRotation");
  const noDataTap                  = document.getElementById("noDataTap");

  // ── Analysis DOM refs ───────────────────────────────────────────────────────
  const analysisPlaceholder  = document.getElementById("analysisPlaceholder");
  const analysisContent      = document.getElementById("analysisContent");

  // Score badges
  const tappingScoreBadge    = document.getElementById("tappingScoreBadge");
  const tappingScoreLabel    = document.getElementById("tappingScoreLabel");
  const prosupScoreBadge     = document.getElementById("prosupScoreBadge");
  const prosupScoreLabel     = document.getElementById("prosupScoreLabel");

  // Tapping feature cells
  const featTapRate          = document.getElementById("featTapRate");
  const featMeanITI          = document.getElementById("featMeanITI");
  const featCvITI            = document.getElementById("featCvITI");
  const featNormAmp          = document.getElementById("featNormAmp");
  const featHesitationCount  = document.getElementById("featHesitationCount");

  // Pro-sup feature cells
  const featCycleFreq        = document.getElementById("featCycleFreq");
  const featMeanPAV          = document.getElementById("featMeanPAV");
  const featCvCycle          = document.getElementById("featCvCycle");
  const featArrestCount      = document.getElementById("featArrestCount");

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
        description: "Each dot marks a finger tap detected by the sensor.",
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

  // ── Score color + label helpers ────────────────────────────────────────────

  /**
   * Returns the CSS color for a given MDS-UPDRS score (0–4).
   * 0 = green (normal) → 4 = red (severe)
   */
  function scoreColor(score) {
    const palette = ["#10b981", "#84cc16", "#f59e0b", "#f97316", "#ef4444"];
    return palette[Math.min(Math.max(score, 0), 4)];
  }

  /**
   * Returns the clinical severity label for a given score (0–4).
   */
  function scoreLabel(score) {
    const labels = ["Normal", "Mild", "Moderate", "Mod-Severe", "Severe"];
    return labels[Math.min(Math.max(score, 0), 4)];
  }

  /**
   * Applies score, color, and label to a badge + label element pair.
   */
  function applyScoreBadge(badgeEl, labelEl, score) {
    if (!badgeEl || !labelEl || score == null) return;
    badgeEl.textContent        = score;
    badgeEl.style.background   = scoreColor(score);
    labelEl.textContent        = scoreLabel(score);
    labelEl.style.color        = scoreColor(score);
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

  function chartOptions(isTapOnly, allTapsSorted) {
    return {
      responsive:          true,
      maintainAspectRatio: false,
      animation:           false,
      scales: {
        x: {
          type:  "linear",
          min:   0,
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

    if (noDataMessage)  noDataMessage.hidden  = true;
    if (noDataRotation) noDataRotation.hidden = true;
    if (noDataTap)      noDataTap.hidden      = true;

    if (!mode) {
      clearCanvas(testCanvas);
      clearCanvas(rotationCanvas);
      clearCanvas(tapCanvas);
      return;
    }

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
      showNoData(noDataMessage, testCanvas);
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

  // ── Raw data loading ───────────────────────────────────────────────────────

  function loadRawChart() {
    const selectedFile = fileSelect ? fileSelect.value : "";
    const url          = selectedFile
      ? `${API}/raw-data?file=${encodeURIComponent(selectedFile)}`
      : `${API}/raw-data`;

    rawPlaceholder.hidden      = false;
    rawContent.hidden          = true;
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

  // ── Features / analysis loading ────────────────────────────────────────────

  /**
   * Fetches /api/features for the currently selected file,
   * then populates both the score badges and the feature breakdown tables.
   *
   * Called whenever a new file is selected (alongside loadRawChart).
   */
  function loadFeatures() {
    const selectedFile = fileSelect ? fileSelect.value : "";
    const url          = selectedFile
      ? `${API}/features?file=${encodeURIComponent(selectedFile)}`
      : `${API}/features`;

    // Reset to loading state
    if (analysisPlaceholder) {
      analysisPlaceholder.hidden      = false;
      analysisPlaceholder.textContent = "Computing MDS-UPDRS scores...";
    }
    if (analysisContent) analysisContent.hidden = true;

    fetch(url)
      .then((r) => {
        if (!r.ok) throw new Error(`HTTP ${r.status}`);
        return r.json();
      })
      .then((data) => {
        if (!data || typeof data !== "object" || data.error) {
          if (analysisPlaceholder) {
            analysisPlaceholder.textContent =
              data?.error || "Analysis unavailable for this file.";
          }
          return;
        }

        // ── Populate Finger Tapping (3.4) ──────────────────────────────────
        const tp  = data.tapping  || {};
        const tpf = tp.features   || {};

        // Score badge
        if (typeof tp.score === "number") {
          applyScoreBadge(tappingScoreBadge, tappingScoreLabel, tp.score);
        }

        // Feature cells — convert to human-friendly units
        if (featTapRate) {
          featTapRate.textContent = tpf.tap_rate != null
            ? `${tpf.tap_rate.toFixed(2)} taps/s`
            : "—";
        }
        if (featMeanITI) {
          // mean_iti_s is in seconds — show as milliseconds for readability
          featMeanITI.textContent = tpf.mean_iti_s != null
            ? `${(tpf.mean_iti_s * 1000).toFixed(0)} ms`
            : "—";
        }
        if (featCvITI) {
          featCvITI.textContent = tpf.cv_iti != null
            ? tpf.cv_iti.toFixed(3)
            : "—";
        }
        if (featNormAmp) {
          // norm_amplitude is 0–1 — show as a percentage
          featNormAmp.textContent = tpf.norm_amplitude != null
            ? `${(tpf.norm_amplitude * 100).toFixed(1)}%`
            : "—";
        }
        if (featHesitationCount) {
          featHesitationCount.textContent = tpf.hesitation_count != null
            ? String(tpf.hesitation_count)
            : "—";
        }

        // ── Populate Pronation-Supination (3.6) ────────────────────────────
        const ps  = data.prosup  || {};
        const psf = ps.features  || {};

        // Score badge
        if (typeof ps.score === "number") {
          applyScoreBadge(prosupScoreBadge, prosupScoreLabel, ps.score);
        }

        // Feature cells
        if (featCycleFreq) {
          featCycleFreq.textContent = psf.cycle_freq_hz != null
            ? `${psf.cycle_freq_hz.toFixed(3)} Hz`
            : "—";
        }
        if (featMeanPAV) {
          featMeanPAV.textContent = psf.mean_pav_dps != null
            ? `${psf.mean_pav_dps.toFixed(1)} dps`
            : "—";
        }
        if (featCvCycle) {
          featCvCycle.textContent = psf.cv_cycle_times != null
            ? psf.cv_cycle_times.toFixed(3)
            : "—";
        }
        if (featArrestCount) {
          featArrestCount.textContent = psf.arrest_count != null
            ? String(psf.arrest_count)
            : "—";
        }

        // Reveal the analysis section
        if (analysisPlaceholder) analysisPlaceholder.hidden = true;
        if (analysisContent)     analysisContent.hidden     = false;
      })
      .catch((err) => {
        if (analysisPlaceholder) {
          analysisPlaceholder.textContent = "Could not load analysis data.";
        }
        console.error("Features load failed", err);
      });
  }

  // ── File list loading ──────────────────────────────────────────────────────

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
          const opt       = document.createElement("option");
          opt.value       = "";
          opt.textContent = "No data files found";
          fileSelect.appendChild(opt);
          rawPlaceholder.textContent = "No data files found in the backend directory.";
          return;
        }

        files.forEach((filename) => {
          const opt       = document.createElement("option");
          opt.value       = filename;
          opt.textContent = filename.replace(/\.txt$/i, "");
          fileSelect.appendChild(opt);
        });

        const preferred  = files.find((f) => f === "mydata.txt") || files[0];
        fileSelect.value = preferred;

        // Load raw chart AND features for the initial file
        loadRawChart();
        loadFeatures();
      })
      .catch((err) => {
        console.error("Could not load file list", err);
        loadRawChart();
        loadFeatures();
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
      // Destroy charts and reload everything for the new file
      activeChart   = destroyChart(activeChart);
      rotationChart = destroyChart(rotationChart);
      tapChart      = destroyChart(tapChart);
      latestRawData = null;
      loadRawChart();
      loadFeatures();   // recompute analysis for the newly selected file
    });
  }

  // ── Boot ───────────────────────────────────────────────────────────────────

  loadFileList();
})();