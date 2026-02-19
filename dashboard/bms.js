// ========== BMS PAGE ==========

// --- BMS View Toggle (Heatmap / Table) ---
document.querySelectorAll('.bms-view-toggle').forEach(toggleGroup => {
  const buttons = toggleGroup.querySelectorAll('.bms-toggle-btn');
  buttons.forEach(btn => {
    btn.addEventListener('click', () => {
      buttons.forEach(b => b.classList.remove('active'));
      btn.classList.add('active');

      const targetId = btn.dataset.target;
      const section = toggleGroup.closest('.bms-section');
      section.querySelectorAll('.bms-heatmap-container, .bms-table-container').forEach(el => {
        el.style.display = 'none';
      });
      document.getElementById(targetId).style.display = '';
    });
  });
});

// --- BMS Tooltip ---
const bmsTooltip = document.getElementById('bmsTooltip');

function showBmsTooltip(e, text) {
  bmsTooltip.textContent = text;
  bmsTooltip.style.display = 'block';
  bmsTooltip.style.left = (e.pageX + 12) + 'px';
  bmsTooltip.style.top = (e.pageY - 28) + 'px';
}

function hideBmsTooltip() {
  bmsTooltip.style.display = 'none';
}

// --- Color mapping ---
function voltageColor(v) {
  const min = 2.5, max = 4.2;
  if (v <= min || v >= max) return 'hsl(0, 70%, 40%)';   // red (hard jump at extremes)

  // quadratic curve: stays yellow aggressively, only goes green near 4.2V
  const t = Math.pow((v - min) / (max - min), 2);
  const hue = Math.round(60 + t * 60); // 60° yellow → 120° green
  return `hsl(${hue}, 75%, 38%)`;
}

function tempColor(t) {
  const min = -30, max = 55;
  if (t <= min || t >= max) return 'hsl(0, 70%, 40%)';   // red (hard jump at extremes)

  // blue→green→red with green at midpoint (~12.5°C)
  const norm = (t - min) / (max - min); // 0 at -30°C, 1 at 55°C
  const hue = norm < 0.5
    ? 240 - norm * 2 * 120   // 240° blue → 120° green
    : 120 - (norm - 0.5) * 2 * 120; // 120° green → 0° red
  return `hsl(${Math.round(hue)}, 70%, 38%)`;
}

// --- Render heatmap grid ---
function renderHeatmap(containerId, values, perRow, stacks, colorFn, unit) {
  const container = document.getElementById(containerId);
  container.innerHTML = '';

  for (let s = 0; s < stacks; s++) {
    const rowLabel = document.createElement('div');
    rowLabel.className = 'heatmap-row-label';
    rowLabel.textContent = `S${s + 1}`;
    container.appendChild(rowLabel);

    const row = document.createElement('div');
    row.className = 'heatmap-row';
    row.style.gridTemplateColumns = `repeat(${perRow}, 1fr)`;

    for (let c = 0; c < perRow; c++) {
      const idx = s * perRow + c;
      const val = values[idx];
      const cell = document.createElement('div');
      cell.className = 'heatmap-cell';
      cell.style.backgroundColor = colorFn(val);

      cell.addEventListener('mouseenter', (e) => {
        showBmsTooltip(e, `S${s + 1}C${c + 1}: ${val.toFixed(2)}${unit}`);
      });
      cell.addEventListener('mousemove', (e) => {
        bmsTooltip.style.left = (e.pageX + 12) + 'px';
        bmsTooltip.style.top = (e.pageY - 28) + 'px';
      });
      cell.addEventListener('mouseleave', hideBmsTooltip);

      row.appendChild(cell);
    }

    container.appendChild(row);
  }
}

// --- Render table view ---
function renderTable(containerId, values, perRow, stacks, unit, precision) {
  const container = document.getElementById(containerId);
  container.innerHTML = '';

  const table = document.createElement('table');
  table.className = 'bms-data-table';

  // Header row
  const thead = document.createElement('thead');
  const headerRow = document.createElement('tr');
  headerRow.innerHTML = '<th>Stack</th>';
  for (let c = 0; c < perRow; c++) {
    headerRow.innerHTML += `<th>${c + 1}</th>`;
  }
  thead.appendChild(headerRow);
  table.appendChild(thead);

  // Data rows
  const tbody = document.createElement('tbody');
  for (let s = 0; s < stacks; s++) {
    const tr = document.createElement('tr');
    tr.innerHTML = `<td class="bms-table-stack-label">S${s + 1}</td>`;
    for (let c = 0; c < perRow; c++) {
      const idx = s * perRow + c;
      const val = values[idx];
      const td = document.createElement('td');
      td.textContent = val.toFixed(precision);
      td.style.color = val !== undefined ? '#eee' : '#666';
      tr.appendChild(td);
    }
    tbody.appendChild(tr);
  }
  table.appendChild(tbody);
  container.appendChild(table);
}

// --- Render flags list ---
let lastRenderedFlagCount = 0;
let lastVoltageKey = null;
let lastTempKey = null;

function renderBmsFlags(flags) {
  const container = document.getElementById('bmsFlagsList');

  if (!flags || flags.length === 0) {
    if (lastRenderedFlagCount !== 0) {
      container.innerHTML = '<p class="bms-flags-empty">No flags reported.</p>';
      lastRenderedFlagCount = 0;
    }
    return;
  }

  if (flags.length === lastRenderedFlagCount) return;
  lastRenderedFlagCount = flags.length;

  container.innerHTML = '';
  flags.forEach(f => {
    const entry = document.createElement('div');
    entry.className = 'bms-flag-entry';

    const ts = new Date(f.timestamp);
    const timeStr = `${String(ts.getHours()).padStart(2, '0')}:${String(ts.getMinutes()).padStart(2, '0')}:${String(ts.getSeconds()).padStart(2, '0')}.`+  `${String(ts.getMilliseconds()).padStart(3, '0')}`;

    const codeBadge = document.createElement('span');
    codeBadge.className = 'bms-flag-code';
    codeBadge.textContent = f.code;

    const timeSpan = document.createElement('span');
    timeSpan.className = 'bms-flag-time';
    timeSpan.textContent = timeStr;

    const msgSpan = document.createElement('span');
    msgSpan.className = 'bms-flag-msg';
    msgSpan.textContent = f.message;

    entry.appendChild(timeSpan);
    entry.appendChild(codeBadge);
    entry.appendChild(msgSpan);
    container.appendChild(entry);
  });
}

// --- Update BMS summary stats ---
function updateBmsSummary(bms) {
  const hasVoltages = Array.isArray(bms.cell_voltages) && bms.cell_voltages.length > 0;
  const hasTemps = Array.isArray(bms.temperatures) && bms.temperatures.length > 0;

  // --- Voltages ---
  if (hasVoltages) {
    const voltages = bms.cell_voltages;
    const vMin = Math.min(...voltages);
    const vMax = Math.max(...voltages);
    const vAvg = voltages.reduce((a, b) => a + b, 0) / voltages.length;

    const vMinEl = document.getElementById('bmsVmin');
    vMinEl.textContent = `${vMin.toFixed(3)} V`;
    vMinEl.style.color = vMin < 2.8 ? '#a72f3b' : vMin < 3.0 ? '#f7c948' : '#eee';

    const vMaxEl = document.getElementById('bmsVmax');
    vMaxEl.textContent = `${vMax.toFixed(3)} V`;
    vMaxEl.style.color = vMax > 4.0 ? '#a72f3b' : vMax > 3.9 ? '#f7c948' : '#eee';

    document.getElementById('bmsVavg').textContent = `${vAvg.toFixed(3)} V`;
  } else {
    document.getElementById('bmsVmin').textContent = '-- V';
    document.getElementById('bmsVmin').style.color = '#666';
    document.getElementById('bmsVmax').textContent = '-- V';
    document.getElementById('bmsVmax').style.color = '#666';
    document.getElementById('bmsVavg').textContent = '-- V';
  }

  // --- Temperatures ---
  if (hasTemps) {
    const temps = bms.temperatures;
    const tMin = Math.min(...temps);
    const tMax = Math.max(...temps);
    const tAvg = temps.reduce((a, b) => a + b, 0) / temps.length;

    const tMinEl = document.getElementById('bmsTmin');
    tMinEl.textContent = `${tMin.toFixed(1)} C`;
    tMinEl.style.color = tMin < -10 ? '#a72f3b' : tMin < 0 ? '#f7c948' : '#eee';

    const tMaxEl = document.getElementById('bmsTmax');
    tMaxEl.textContent = `${tMax.toFixed(1)} C`;
    tMaxEl.style.color = tMax > 55 ? '#a72f3b' : tMax > 45 ? '#f7c948' : '#eee';

    document.getElementById('bmsTavg').textContent = `${tAvg.toFixed(1)} C`;
  } else if (bms.therm_min_valid || bms.cells_global_min_valid) {
    // Real mode: use CAN summary data
    const tMin = bms.therm_min ?? bms.cells_global_min ?? 0;
    const tMax = bms.therm_max ?? bms.cells_global_max ?? 0;
    const tAvg = bms.therm_avg ?? Math.round((tMin + tMax) / 2);

    const tMinEl = document.getElementById('bmsTmin');
    tMinEl.textContent = `${tMin} C`;
    tMinEl.style.color = tMin < -10 ? '#a72f3b' : tMin < 0 ? '#f7c948' : '#eee';

    const tMaxEl = document.getElementById('bmsTmax');
    tMaxEl.textContent = `${tMax} C`;
    tMaxEl.style.color = tMax > 55 ? '#a72f3b' : tMax > 45 ? '#f7c948' : '#eee';

    document.getElementById('bmsTavg').textContent = `${tAvg} C`;
  } else {
    document.getElementById('bmsTmin').textContent = '-- C';
    document.getElementById('bmsTmin').style.color = '#666';
    document.getElementById('bmsTmax').textContent = '-- C';
    document.getElementById('bmsTmax').style.color = '#666';
    document.getElementById('bmsTavg').textContent = '-- C';
  }

  // --- Pack current ---
  const currentEl = document.getElementById('bmsCurrent');
  if (bms.pack_current != null) {
    currentEl.textContent = `${bms.pack_current.toFixed(1)} A`;
    if (Math.abs(bms.pack_current) > 90) {
      currentEl.style.color = '#a72f3b';
    } else if (Math.abs(bms.pack_current) > 70) {
      currentEl.style.color = '#f7c948';
    } else {
      currentEl.style.color = '#eee';
    }
  } else {
    currentEl.textContent = '-- A';
    currentEl.style.color = '#666';
  }
}

// --- Show "no data" placeholder in a container ---
function showNoData(containerId, message) {
  const container = document.getElementById(containerId);
  if (container.dataset.nodata) return; // already showing
  container.innerHTML = `<p style="color:#666;text-align:center;padding:2em 0;">${message}</p>`;
  container.dataset.nodata = '1';
}

function clearNoData(containerId) {
  const container = document.getElementById(containerId);
  if (container.dataset.nodata) {
    container.innerHTML = '';
    delete container.dataset.nodata;
  }
}

// --- BMS Update Loop ---
setInterval(() => {
  if (!latestData) return;

  const bms = latestData.bms;
  if (!bms) return;

  // Always update flags even when frozen
  renderBmsFlags(bms.flags);

  if (isFrozen) return;

  // Update summary stats
  updateBmsSummary(bms);

  const hasVoltages = Array.isArray(bms.cell_voltages) && bms.cell_voltages.length > 0;
  const hasTemps = Array.isArray(bms.temperatures) && bms.temperatures.length > 0;

  // Update voltage heatmap/table (only if data changed)
  if (hasVoltages) {
    clearNoData('voltageHeatmap');
    clearNoData('voltageTable');

    const voltageKey = bms.cell_voltages.join(',');
    if (voltageKey !== lastVoltageKey) {
      lastVoltageKey = voltageKey;
      const voltageHeatmap = document.getElementById('voltageHeatmap');
      if (voltageHeatmap.style.display !== 'none') {
        renderHeatmap('voltageHeatmap', bms.cell_voltages, bms.cells_per_stack, bms.stacks, voltageColor, 'V');
      }
      const voltageTable = document.getElementById('voltageTable');
      if (voltageTable.style.display !== 'none') {
        renderTable('voltageTable', bms.cell_voltages, bms.cells_per_stack, bms.stacks, 'V', 3);
      }
    }
  } else {
    showNoData('voltageHeatmap', 'Cell voltages not available from CAN bus');
    showNoData('voltageTable', 'Cell voltages not available from CAN bus');
  }

  // Update temperature heatmap/table (only if data changed)
  if (hasTemps) {
    clearNoData('tempHeatmap');
    clearNoData('tempTable');

    const tempKey = bms.temperatures.join(',');
    if (tempKey !== lastTempKey) {
      lastTempKey = tempKey;
      const tempHeatmap = document.getElementById('tempHeatmap');
      if (tempHeatmap.style.display !== 'none') {
        renderHeatmap('tempHeatmap', bms.temperatures, bms.ntcs_per_stack, bms.stacks, tempColor, 'C');
      }
      const tempTable = document.getElementById('tempTable');
      if (tempTable.style.display !== 'none') {
        renderTable('tempTable', bms.temperatures, bms.ntcs_per_stack, bms.stacks, 'C', 1);
      }
    }
  } else {
    showNoData('tempHeatmap', 'Individual temps not available — see summary above');
    showNoData('tempTable', 'Individual temps not available — see summary above');
  }

}, 500);
