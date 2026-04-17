(function () {
  const API = "/api";

  let activeChart = null;
  let rotationChart = null;
  let tapChart = null;
  let latestRawData = null;

  const rawPlaceholder = document.getElementById("rawPlaceholder");
  const rawContent = document.getElementById("rawContent");
  const rawSummaryTaps = document.getElementById("rawSummaryTaps");
  const rawSummaryTapDuration = document.getElementById("rawSummaryTapDuration");
  const rawSummaryRotations = document.getElementById("rawSummaryRotations");
  const rawSummaryRotationDuration = document.getElementById("rawSummaryRotationDuration");
  const testCanvas = document.getElementById("testChart");
  const rotationCanvas = document.getElementById("rotationChart");
  const tapCanvas = document.getElementById("tapChart");
  const chartSingleWrap = document.getElementById("chartSingleWrap");
  const chartBothWrap = document.getElementById("chartBothWrap");
  const testMode = document.getElementById("testMode");
  const graphTitle = document.getElementById("graphTitle");
  const graphDescription = document.getElementById("graphDescription");

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
        description: "Each dot marks a finger tap event at y = 0 over time.",
      };
    }
    if (mode === "both") {
      return {
        title: "Both tests",
        description: "Displays separate Pronation-Supination and Finger Taps graphs.",
      };
    }
    return {
      title: "Pronation / supination",
      description: "Gyroscope trace with rotation markers aligned to the baseline for a cleaner view.",
    };
  }

  function toFiniteNumber(value) {
    const num = Number(value);
    return Number.isFinite(num) ? num : null;
  }

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
      .filter((value) => value != null)
      .sort((a, b) => a - b);

    if (times.length === 0) return 0;
    if (times.length === 1) return times[0];
    return Math.max(0, times[times.length - 1] - times[0]);
  }

  function computeRotationDurationSeconds(samples, rotations) {
    const sampleMax = samples
      .map((sample) => toFiniteNumber(sample.t))
      .filter((value) => value != null)
      .reduce((max, value) => Math.max(max, value), 0);

    const rotationMax = rotations
      .map((rotation) => toFiniteNumber(rotation.t))
      .filter((value) => value != null)
      .reduce((max, value) => Math.max(max, value), 0);

    return Math.max(sampleMax, rotationMax);
  }

  function buildPronationDatasets(data) {
    const samples = Array.isArray(data.samples) ? data.samples : [];
    const rotations = Array.isArray(data.rotations) ? data.rotations : [];

    const datasets = [];

    if (samples.length > 0) {
      datasets.push({
        kind: "gyro",
        label: "Gyro trace",
        data: samples
          .map((sample) => ({ x: toFiniteNumber(sample.t), y: toFiniteNumber(sample.dps) }))
          .filter((sample) => sample.x != null && sample.y != null),
        type: "line",
        borderColor: "rgb(54, 162, 235)",
        backgroundColor: "rgba(54, 162, 235, 0.08)",
        pointRadius: 0,
        borderWidth: 2,
        fill: false,
        tension: 0.25,
        showLine: true,
        order: 3,
      });
    }

    if (rotations.length > 0) {
      datasets.push({
        kind: "rotation",
        label: "Rotation detected",
        data: spreadDuplicateMarkers(rotations, (rotation) => rotation.t, 0.12),
        type: "scatter",
        pointStyle: "circle",
        pointRadius: 6,
        pointHoverRadius: 9,
        pointBorderWidth: 2,
        backgroundColor: "rgba(239, 68, 68, 0.9)",
        borderColor: "rgb(153, 27, 27)",
        order: 1,
      });
    }

    return datasets;
  }

  function buildTapDatasets(data) {
    const taps = Array.isArray(data.taps) ? data.taps : [];
    if (taps.length === 0) return [];

    return [{
      kind: "tap",
      label: "Tap events",
      data: spreadDuplicateMarkers(taps, (tap) => tap.t, 0.14),
      type: "scatter",
      pointStyle: "circle",
      pointRadius: 6,
      pointHoverRadius: 9,
      pointBorderWidth: 2,
      backgroundColor: "rgba(16, 185, 129, 0.9)",
      borderColor: "rgb(4, 120, 87)",
      order: 2,
    }];
  }

  function chartOptions(isTapOnly) {
    return {
      responsive: true,
      maintainAspectRatio: false,
      animation: false,
      scales: {
        x: {
          type: "linear",
          title: { display: true, text: "Time (s)" },
          ticks: { callback: (value) => `${Number(value).toFixed(1)}s` },
          grid: { color: "rgba(0,0,0,0.06)" },
        },
        y: isTapOnly
          ? {
            min: -1,
            max: 1,
            display: false,
            grid: { display: false },
          }
          : {
            title: { display: true, text: "Angular velocity (dps)" },
            grid: { color: "rgba(0,0,0,0.06)" },
          },
      },
      plugins: {
        title: {
          display: false,
        },
        legend: { position: "top" },
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
                const tap = ctx.raw.meta;
                return [
                  `Tap #${tap.tap_num}`,
                  `Time: ${formatSeconds(tap.t)}`,
                  `ADC: ${tap.adc}`,
                ];
              }
              return `${ctx.parsed.y.toFixed(1)} dps at ${formatSeconds(ctx.parsed.x)}`;
            },
          },
        },
      },
    };
  }

  function clearCanvas(canvas) {
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    ctx.clearRect(0, 0, canvas.width, canvas.height);
  }

  function setChartVisibility(mode) {
    if (!chartSingleWrap || !chartBothWrap) return;
    if (mode === "both") {
      chartSingleWrap.hidden = true;
      chartBothWrap.hidden = false;
      return;
    }
    if (mode === "finger_taps" || mode === "pronation_supination") {
      chartSingleWrap.hidden = false;
      chartBothWrap.hidden = true;
      return;
    }
    chartSingleWrap.hidden = true;
    chartBothWrap.hidden = true;
  }

  function renderActiveChart(data) {
    const mode = (testMode && testMode.value) || "";
    const modeMeta = getModeMeta(mode);

    if (graphTitle) graphTitle.textContent = modeMeta.title;
    if (graphDescription) graphDescription.textContent = modeMeta.description;

    setChartVisibility(mode);

    activeChart = destroyChart(activeChart);
    rotationChart = destroyChart(rotationChart);
    tapChart = destroyChart(tapChart);

    if (!mode) {
      clearCanvas(testCanvas);
      clearCanvas(rotationCanvas);
      clearCanvas(tapCanvas);
      return;
    }

    if (mode === "both") {
      if (!rotationCanvas || !tapCanvas) return;

      const pronationDatasets = buildPronationDatasets(data);
      if (pronationDatasets.length > 0) {
        rotationChart = new Chart(rotationCanvas.getContext("2d"), {
          type: "scatter",
          data: { datasets: pronationDatasets },
          options: chartOptions(false),
        });
      } else {
        clearCanvas(rotationCanvas);
      }

      const tapDatasets = buildTapDatasets(data);
      if (tapDatasets.length > 0) {
        tapChart = new Chart(tapCanvas.getContext("2d"), {
          type: "scatter",
          data: { datasets: tapDatasets },
          options: chartOptions(true),
        });
      } else {
        clearCanvas(tapCanvas);
      }
      return;
    }

    if (!testCanvas) return;
    const datasets = mode === "finger_taps" ? buildTapDatasets(data) : buildPronationDatasets(data);
    if (datasets.length === 0) {
      clearCanvas(testCanvas);
      return;
    }

    const isTapOnly = mode === "finger_taps";
    const ctx = testCanvas.getContext("2d");

    activeChart = new Chart(ctx, {
      type: "scatter",
      data: { datasets },
      options: chartOptions(isTapOnly),
    });
  }

  function loadRawChart() {
    fetch(API + "/raw-data")
      .then((r) => {
        if (!r.ok) {
          throw new Error(`HTTP ${r.status}`);
        }
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

        const hasRawData = (data.samples && data.samples.length > 0) ||
          (data.rotations && data.rotations.length > 0) ||
          (data.taps && data.taps.length > 0);

        if (!hasRawData) {
          rawPlaceholder.textContent = "No raw sensor data found in the current log.";
          return;
        }

        rawPlaceholder.hidden = true;
        rawContent.hidden = false;

        const taps = Array.isArray(data.taps) ? data.taps : [];
        const samples = Array.isArray(data.samples) ? data.samples : [];
        const rotations = Array.isArray(data.rotations) ? data.rotations : [];
        rawSummaryTaps.textContent = String(taps.length);
        if (rawSummaryTapDuration) {
          rawSummaryTapDuration.textContent = formatSeconds(computeTapDurationSeconds(taps));
        }
        rawSummaryRotations.textContent = String(rotations.length);
        if (rawSummaryRotationDuration) {
          rawSummaryRotationDuration.textContent = formatSeconds(computeRotationDurationSeconds(samples, rotations));
        }

        latestRawData = data;
        renderActiveChart(data);
      })
      .catch((err) => {
        rawPlaceholder.textContent = "Could not load sensor data.";
        console.error("Raw data load failed", err);
      });
  }

  if (testMode) {
    testMode.addEventListener("change", function () {
      if (latestRawData) renderActiveChart(latestRawData);
    });
  }

  loadRawChart();
})();