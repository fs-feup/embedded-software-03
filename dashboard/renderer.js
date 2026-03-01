// --- Variables ---
let latestData = null;
let isFrozen = false;
let currentErrors = [];
let previousErrors = [];
let faultHistoryData = [];
let lastFSM = null;
let lastFSMTime = 0;
let prevDistinctFSM = null;

// --- Receive sensor data from main process ---
window.electronAPI.onSensorData((data) => {
  latestData = data;
});

function formatTimestamp(us) {
  const ms = us / 1000;
  const date = new Date(ms);

  // Format time with milliseconds
  const hours = String(date.getHours()).padStart(2, '0');
  const minutes = String(date.getMinutes()).padStart(2, '0');
  const seconds = String(date.getSeconds()).padStart(2, '0');
  const milliseconds = String(date.getMilliseconds()).padStart(3, '0');

  return `${hours}:${minutes}:${seconds}.${milliseconds}`;
}


// --- Freeze button logic ---
const freezeBtn = document.getElementById('freezeBtn');
freezeBtn.addEventListener('click', () => {
  isFrozen = !isFrozen;
  if (isFrozen) {
    freezeBtn.classList.add('frozen');
    freezeBtn.textContent = 'Frozen';
  } else {
    freezeBtn.classList.remove('frozen');
    freezeBtn.textContent = 'Freeze';
  }
});

// --- Export button logic ---
const saveBtn = document.getElementById('saveBtn');
saveBtn.addEventListener('click', () => {
  window.electronAPI.exportData();
});
window.electronAPI.onExportDone((filename) => {
  alert(`✅ Data exported to ${filename}`);
});

// --- Auto-export faults on session close ---
window.electronAPI.onRequestFaultExport(() => {
  window.electronAPI.sendFaultData({
    cleared: faultHistoryData,
    active: currentErrors,
  });
});

// --- Helper functions ---
function createStatusDot(value, type = "boolean") {
  const span = document.createElement('span');
  span.classList.add('status-dot');

  if (type === "boolean") {
    const isOn = value === true || value === 1 || value === "1" || value === "true";
    span.classList.add(isOn ? 'status-true' : 'status-false');
  }
  else if (type === "battery") {
    const numValue = Number(value);
    switch (value) {
      case 0: span.classList.add('status-ok'); break;
      case 1: span.classList.add('status-warning'); break;
      case 2: span.classList.add('status-critical'); break;
      case 3: span.classList.add('status-failure'); break;
      default: span.classList.add('status-unknown');
    }
  }

  return span;
}


// Master's Mission enum: MANUAL=0, ACCELERATION=1, SKIDPAD=2, AUTOCROSS=3, TRACKDRIVE=4, EBS_TEST=5, INSPECTION=6, CHUCK=7
const modeNames = {
  0: 'Manual',
  1: 'Acceleration',
  2: 'Skidpad',
  3: 'Autocross',
  4: 'Trackdrive',
  5: 'EBS Test',
  6: 'Inspection',
  7: 'Chuck'
};

// Master's State enum: AS_MANUAL=0, AS_OFF=1, AS_READY=2, AS_DRIVING=3, AS_FINISHED=4, AS_EMERGENCY=5
const stateNames = {
  0: 'AS Manual',
  1: 'AS Off',
  2: 'AS Ready',
  3: 'AS Driving',
  4: 'AS Finished',
  5: 'AS Emergency'
};


function updateFSM(fsmData) {
  const prevStateEl = document.getElementById("prevStateName");
  const currStateEl = document.getElementById("stateName");
  const currStateDot = document.getElementById("currentStateDot");
  const prevDot = document.querySelector("#previousState .state-status-dot");
  const emergencyBox = document.getElementById("emergencyBox");
  const emergencyInfo = document.getElementById("emergencyInfo");

  const current = fsmData.fsm_state;
  const errorMessages = fsmData.error_messages || [];

  // --- Update prevDistinctFSM ONLY when the state *actually* changes ---
  if (lastFSM === null) {
    prevDistinctFSM = null;
  } else if (current !== lastFSM) {
    prevDistinctFSM = lastFSM;
  }

  // --- Render previous state ---
  if (prevDistinctFSM === null) {
    prevStateEl.textContent = "--";
    prevDot.className = "state-status-dot status-completed";
  } else {
    prevStateEl.textContent = stateNames[prevDistinctFSM] || prevDistinctFSM;
    prevDot.className = "state-status-dot";

    if (current === 5) {
      prevDot.classList.add("status-emergency");
    } else if (prevDistinctFSM === 5) {
      prevDot.classList.add("status-emergency");
    } else {
      prevDot.classList.add("status-completed");
    }
  }

  // --- Render current state ---
  currStateEl.textContent = stateNames[current] || current;
  currStateDot.className = "state-status-dot";
  if (current === 5) {
    currStateDot.classList.add("status-emergency");
  } else {
    currStateDot.classList.add("status-active");
  }

  // --- Emergency box and messages ---
  if (current === 5) {
    emergencyBox.classList.add("show");
    const allMessages = [...errorMessages];
    emergencyInfo.textContent = allMessages.length > 0
      ? allMessages.join('\n')
      : "Emergency state active.";
  } else {
    emergencyBox.classList.remove("show");
  }

  lastFSM = current;
}

function updateFaults(data) {
  const newErrors = [];

  // Get the current faults array safely
  if (data.fsm && Array.isArray(data.fsm.fault_messages)) {
    newErrors.push(...data.fsm.fault_messages);
  }

  const statusEl = document.getElementById('systemStatus');
  const openBtn = document.getElementById('openErrorBtn');

  // Update active faults display
  if (newErrors.length === 0) {
    statusEl.textContent = 'Status: All OK';
    statusEl.style.color = 'white';
    openBtn.classList.remove('visible');
  } else {
    const firstError = newErrors[0];
    const moreCount = newErrors.length - 1;
    statusEl.textContent = moreCount > 0
      ? `Status: ${firstError} (+${moreCount})`
      : `Status: ${firstError}`;
    statusEl.style.color = 'white';
    openBtn.classList.add('visible');
  }

  // Detect cleared faults (were in previous, not in current)
  const clearedFaults = previousErrors.filter(e => !newErrors.includes(e));

  if (clearedFaults.length > 0) {
    const timestamp = data.timestamp_us || Date.now() * 1000;
    const formattedTime = formatTimestamp(timestamp);

    clearedFaults.forEach(e => {
      faultHistoryData.push({ fault: e, time: formattedTime });
    });

    renderFaultHistory();
  }

  // Render active errors
  const errorListEl = document.getElementById('errorList');
  errorListEl.innerHTML = '';
  newErrors.forEach(e => {
    const li = document.createElement('li');
    li.textContent = e;
    errorListEl.appendChild(li);
  });

  // Save state for next comparison
  previousErrors = [...newErrors];
}
// === Error popup open/close ===
const popup = document.getElementById('errorPopup');
const openBtn = document.getElementById('openErrorBtn');
const closeBtn = document.getElementById('closePopup');

openBtn.addEventListener('click', () => {
  // Use the current active errors
  if (previousErrors.length > 0) {
    popup.classList.remove('hidden');
  }
});

closeBtn.addEventListener('click', () => {
  popup.classList.add('hidden');
});

// History tab toggle
const historyBtn = document.getElementById('historyBtn');
const activeErrorsSection = document.getElementById('activeErrorsSection');
const faultHistorySection = document.getElementById('faultHistorySection');

historyBtn.addEventListener('click', () => {
  const showingHistory = faultHistorySection.style.display === 'block';
  faultHistorySection.style.display = showingHistory ? 'none' : 'block';
  activeErrorsSection.style.display = showingHistory ? 'block' : 'none';
  historyBtn.textContent = showingHistory ? 'Show History' : 'Back to Active';
});

// Render fault history
function renderFaultHistory() {
  const historyList = document.getElementById('faultHistoryList');
  historyList.innerHTML = '';

  faultHistoryData.slice(-50).forEach(item => {
    const li = document.createElement('li');
    li.textContent = `${item.time} — ${item.fault}`;
    historyList.appendChild(li);
  });
}

// --- Update UI loop ---
setInterval(() => {
  if (!latestData || isFrozen) return;
  const data = latestData;


  const lvVoltageEl = document.getElementById('lvVoltage');
  lvVoltageEl.innerHTML = '';

  const batteryState = data.fsm?.battery_st ?? null;

  // --- Create battery status dot using helper ---
  const dot = createStatusDot(batteryState, "battery");

  // --- SoC display ---
  const socValue = data.analog?.["SoC"];
  const label = document.createElement('span');
  label.textContent = `SoC: ${socValue !== undefined ? socValue : '--'} %`;

  // --- Append both ---
  lvVoltageEl.appendChild(dot);
  lvVoltageEl.appendChild(label);


  const modeCode = data.mode;
  const modeName = modeNames[modeCode] || 'Unknown';
  document.getElementById('modeDisplay').textContent = `Mode: ${modeName}`;


  // --- Analog Sensors ---
  const analogContainer = document.getElementById('analogData');
  analogContainer.innerHTML = '';
  if (data.analog) {
    for (const [key, value] of Object.entries(data.analog)) {
      if (key === 'SoC') continue;
      const col = document.createElement('div');
      col.className = 'col-md-6';
      col.innerHTML = `
        <div class="border rounded p-2 analog-sensor-item">
          <strong>${key}:</strong>
          <span class="fs-5 text-primary analog-value">${typeof value === 'number' ? value.toFixed(2) : value}</span>
        </div>`;
      analogContainer.appendChild(col);
    }
  }

  // --- Digital Inputs ---
  const digitalContainer = document.getElementById('digitalData');
  digitalContainer.innerHTML = '';
  if (data.digital) {
    for (const [key, value] of Object.entries(data.digital)) {
      const col = document.createElement('div');
      col.className = 'col-md-6';

      const card = document.createElement('div');
      card.className = 'border rounded p-2 text-center';
      card.innerHTML = '';

      card.appendChild(createStatusDot(value));
      const label = document.createElement('strong');
      label.textContent = key;
      card.appendChild(label);

      col.appendChild(card);
      digitalContainer.appendChild(col);
    }
  }

  // --- Actuators ---
  const actuatorContainer = document.getElementById('actuatorData');
  actuatorContainer.innerHTML = '';
  if (data.actuators) {
    for (const [key, value] of Object.entries(data.actuators)) {
      const col = document.createElement('div');
      col.className = 'col-md-6';

      const card = document.createElement('div');
      card.className = 'border rounded p-2 text-center';
      card.innerHTML = '';

      card.appendChild(createStatusDot(value));
      const label = document.createElement('strong');
      label.textContent = key;
      card.appendChild(label);

      col.appendChild(card);
      actuatorContainer.appendChild(col);
    }
  }

  if (data.fsm) {
    updateFSM(data.fsm);
  }
  updateFaults(data);

}, 500);

// ========== uPlot Charts ==========

const analogSection = document.querySelector('.div3');
const analogDetail = document.getElementById('analogDetail');
const chartsContainer = document.getElementById('chartsContainer');
const backBtn = document.getElementById('backBtn');
const viewChartsBtn = document.getElementById('viewChartsBtn');

const mainButtonsContainer = document.getElementById('mainButtonsContainer');

// Store rolling history and uPlot instances
const analogHistory = {};
const charts = {};
const MAX_POINTS = 20; // 500ms * 20 = ~10 seconds
let timeCounter = 0;

// uPlot common options
function createPlotOptions(sensor) {
  return {
    width: 430,
    height: 200,
    series: [
      {
        label: "Time (s)"
      },
      {
        stroke: "#cc8282",
        width: 2,
        points: { show: false }
      }
    ],
    scales: {
      x: {
        time: false,
      },
      y: {
        auto: true,
      }
    },
    axes: [
      {
        stroke: "#ccc",
        grid: { stroke: "#333", width: 1 }
      },
      {
        stroke: "#ccc",
        grid: { stroke: "#333", width: 1 }
      }
    ],
    legend: {
      show: false
    },
    cursor: {
      drag: { x: false, y: false }
    }
  };
}

// Show detail view via button
viewChartsBtn.addEventListener('click', () => {
  // Ensure we have some data before showing charts
  if (Object.keys(analogHistory).length === 0) {
    alert('No analog sensor data available yet. Please wait for data to be received.');
    return;
  }

  window.electronAPI.send('toggle-fullscreen', true);

  document.querySelector('.parent').style.opacity = '0';

  if (mainButtonsContainer) {
    mainButtonsContainer.style.display = 'none';
  }

  setTimeout(() => {
    document.querySelector('.parent').style.display = 'none';
    analogDetail.classList.remove('hidden');
    analogDetail.style.opacity = '1';
    renderCharts();
  }, 300);
});

// Back button - destroy all charts
backBtn.addEventListener('click', () => {
  analogDetail.style.opacity = '0';

  if (mainButtonsContainer) {
    mainButtonsContainer.style.display = 'block';
  }
  setTimeout(() => {
    // Destroy all uPlot instances
    Object.keys(charts).forEach(sensor => {
      if (charts[sensor]) {
        charts[sensor].destroy();
        delete charts[sensor];
      }
    });

    analogDetail.classList.add('hidden');
    document.querySelector('.parent').style.display = 'grid';
    document.querySelector('.parent').style.opacity = '1';
  }, 300);
});

function renderCharts() {
  // Destroy existing charts
  Object.keys(charts).forEach(sensor => {
    if (charts[sensor]) {
      charts[sensor].destroy();
      delete charts[sensor];
    }
  });

  chartsContainer.innerHTML = '';

  // Check if uPlot is loaded
  if (typeof uPlot === 'undefined') {
    chartsContainer.innerHTML = '<p style="color: red; text-align: center;">Error: uPlot library not loaded</p>';
    console.error('uPlot is not defined');
    return;
  }

  const sensorNames = Object.keys(analogHistory);
  console.log('Rendering charts for sensors:', sensorNames);

  sensorNames.forEach((sensor) => {
    const card = document.createElement('div');
    card.className = 'chart-card';

    const title = document.createElement('h5');
    title.textContent = sensor;

    const chartDiv = document.createElement('div');
    chartDiv.className = 'chart-container';
    chartDiv.id = `chart-${sensor}`;

    card.appendChild(title);
    card.appendChild(chartDiv);
    chartsContainer.appendChild(card);

    // Create time series data
    const dataPoints = analogHistory[sensor] || [];
    console.log(`Sensor ${sensor} has ${dataPoints.length} data points`);

    // Ensure we have at least 2 points for uPlot
    if (dataPoints.length < 2) {
      dataPoints.push(0, 0);
    }

    const times = Array.from({ length: dataPoints.length }, (_, i) => i * 0.5);
    const data = [times, dataPoints];

    try {
      // Create uPlot instance
      const opts = createPlotOptions(sensor);
      charts[sensor] = new uPlot(opts, data, chartDiv);
      console.log(`Chart created for ${sensor}`);
    } catch (error) {
      console.error(`Error creating chart for ${sensor}:`, error);
      chartDiv.innerHTML = `<p style="color: red;">Error: ${error.message}</p>`;
    }
  });

  if (sensorNames.length === 0) {
    chartsContainer.innerHTML = '<p style="color: #ccc; text-align: center; font-size: 1.5rem;">No sensor data available yet.</p>';
  }
}

// Update analog history and charts
setInterval(() => {
  if (!latestData || isFrozen) return;

  const analog = latestData.analog || {};

  // Update history for ALL sensors
  for (const [sensor, value] of Object.entries(analog)) {
    if (sensor === 'SoC') continue;

    if (!analogHistory[sensor]) {
      analogHistory[sensor] = [];
      console.log(`Initialized history for sensor: ${sensor}`);
    }
    analogHistory[sensor].push(typeof value === 'number' ? value : 0);
    if (analogHistory[sensor].length > MAX_POINTS) {
      analogHistory[sensor].shift();
    }
  }

  // Update charts if visible
  if (!analogDetail.classList.contains('hidden')) {
    Object.entries(charts).forEach(([sensor, chart]) => {
      if (analogHistory[sensor] && chart) {
        const times = Array.from({ length: analogHistory[sensor].length }, (_, i) => i * 0.5);
        const values = analogHistory[sensor];
        try {
          chart.setData([times, values]);
        } catch (error) {
          console.error(`Error updating chart ${sensor}:`, error);
        }
      }
    });
  }
}, 500);

// ========== NAVIGATION TABS ==========

const navTabs = document.querySelectorAll('.nav-tab');
const pages = {
  'bms': document.getElementById('page-bms'),
  'state-machine': document.getElementById('page-state-machine'),
};

navTabs.forEach(tab => {
  tab.addEventListener('click', () => {
    navTabs.forEach(t => t.classList.remove('active'));
    tab.classList.add('active');

    const activePage = tab.dataset.page;
    Object.entries(pages).forEach(([name, el]) => {
      el.style.display = name === activePage ? '' : 'none';
    });
  });
});

