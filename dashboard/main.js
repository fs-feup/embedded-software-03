const { app, BrowserWindow, ipcMain } = require('electron');
const { spawn } = require('child_process');
const path = require('path');
const fs = require('fs');

let win;
let allSnapshots = [];
let canProcess = null;

// Path to the compiled CAN listener binary (now inside the project)
const CAN_EXE = process.platform === 'win32' ? 'can_listener.exe' : 'can_listener';
const CAN_BINARY = path.join(__dirname, '..', 'can_listener', CAN_EXE);

// Auto-detect default serial device based on OS
// Override with environment variable: CAN_DEVICE=/dev/ttyACM0 npm start
const CAN_DEVICE = process.env.CAN_DEVICE || (() => {
  switch (process.platform) {
    case 'win32':  return 'COM7';                      // Windows
    case 'darwin': return '/dev/cu.usbserial-DN8FHMAG'; // macOS
    default:       return '/dev/ttyUSB0';               // Linux
  }
})();

// Toggle mock mode manually (set to true to test UI without CAN adapter)
const useMock = false;

// Transform flat C++ snapshot into the nested format the renderer expects
function transformSnapshot(flat) {
  // Build fault messages from dead signals
  const faults = [];
  if (flat.bms_dead) faults.push('BMS Dead');
  if (flat.steer_dead) faults.push('Steering Dead');
  if (flat.pc_dead) faults.push('PC Dead');
  if (flat.inversor_dead) faults.push('Inversor Dead');
  if (flat.res_dead) faults.push('RES Dead');

  const errors = [];
  if (flat.emergency_signal) errors.push('Emergency signal active');

  return {
    timestamp_us: flat.t_us,
    mode: flat.master_autonomous_mission ?? flat.mission ?? 0,
    digital: {
      "ASMS": !!flat.asms_on,
      "AS ATS": !!flat.asats_pressed,
      "ATS Btn": !!flat.ats_pressed,
      "TSMS SDC": !!flat.tsms_sdc_closed,
      "Master SDC": !!flat.master_sdc_closed,
      "TS On": !!flat.ts_on,
      "WD Ready": !!flat.wd_ready,
      "Emergency": !!flat.emergency_signal,
      "EBS Engage": !!flat.ebs_engage,
      "EBS Release": !!flat.ebs_release,
      "Bamocar TS": !!flat.bamocar_ts_on,
      "Master ASMS": !!flat.master_asms_on,
    },
    analog: {
      "Hydraulic Front (bar)": flat.hydraulic_front_bar ?? 0,
      "Hydraulic Rear (bar)": flat.hydraulic_rear_bar ?? 0,
      "Brake Pressure": flat.dash_brake_pressure ?? 0,
      "APPS Higher": flat.apps_higher ?? 0,
      "APPS Lower": flat.apps_lower ?? 0,
      "RPM FR": flat.rpm_fr_raw ?? 0,
      "RPM FL": flat.rpm_fl_raw ?? 0,
      "DC Voltage": flat.dc_voltage ?? 0,
      "Motor Speed": flat.bamocar_speed ?? 0,
      "Motor Current": flat.bamocar_motor_current ?? 0,
      "SoC": flat.master_soc ?? 0,
    },
    valid: {
      "ASMS": !!flat.asms_on_valid,
      "AS ATS": !!flat.asats_pressed_valid,
      "ATS Btn": !!flat.ats_pressed_valid,
      "TSMS SDC": !!flat.tsms_sdc_closed_valid,
      "Master SDC": !!flat.master_sdc_closed_valid,
      "TS On": !!flat.ts_on_valid,
      "WD Ready": !!flat.wd_ready_valid,
      "Emergency": !!flat.emergency_signal_valid,
      "EBS Engage": !!flat.ebs_engage_valid,
      "EBS Release": !!flat.ebs_release_valid,
      "Bamocar TS": !!flat.bamocar_ts_on_valid,
      "Master ASMS": !!flat.master_asms_on_valid,
      "Hydraulic Front (bar)": !!flat.hydraulic_front_valid,
      "Hydraulic Rear (bar)": !!flat.hydraulic_rear_valid,
      "Brake Pressure": !!flat.dash_brake_pressure_valid,
      "APPS Higher": !!flat.apps_higher_valid,
      "APPS Lower": !!flat.apps_lower_valid,
      "RPM FR": !!flat.rpm_fr_valid,
      "RPM FL": !!flat.rpm_fl_valid,
      "DC Voltage": !!flat.dc_voltage_valid,
      "Motor Speed": !!flat.bamocar_speed_valid,
      "Motor Current": !!flat.bamocar_motor_current_valid,
      "SoC": !!flat.master_soc_valid,
      "mode": !!flat.mission_valid || !!flat.master_autonomous_mission_valid,
    },
    actuators: {},
    fsm: {
      fsm_state: flat.master_as_state ?? flat.state ?? 0,
      time_in_state_ms: 0,
      checkup_state: flat.state_checkup ?? 0,
      ebs_state: 0,
      error_messages: errors,
      fault_messages: faults,
      battery_st: 0,
    },
    bms: {
      therm_min: flat.therm_min ?? null,
      therm_min_valid: !!flat.therm_min_valid,
      therm_max: flat.therm_max ?? null,
      therm_max_valid: !!flat.therm_max_valid,
      therm_avg: flat.therm_avg ?? null,
      therm_avg_valid: !!flat.therm_avg_valid,
      cells_global_min: flat.cells_global_min ?? null,
      cells_global_min_valid: !!flat.cells_global_min_valid,
      cells_global_max: flat.cells_global_max ?? null,
      cells_global_max_valid: !!flat.cells_global_max_valid,
      cell_voltages: null,
      temperatures: flat.cells_all_temps?.flat() ?? null,
      pack_current: null,
      flags: [],
      stacks: 6,
      cells_per_stack: 24,
      ntcs_per_stack: 24,
    },
  };
}

function createWindow() {
  win = new BrowserWindow({
    width: 1200,
    height: 910,
    resizable: true,
    webPreferences: {
      preload: path.join(__dirname, 'preload.js'),
      contextIsolation: true,
      nodeIntegration: false,
    },
  });

  win.loadFile('index.html');
}

app.whenReady().then(() => {
  createWindow();

  if (useMock) {
    console.log('MOCK MODE ENABLED');

    const fsmStates = [0, 1, 2, 3, 4, 5];
    let lastFSMState = fsmStates[0];
    let lastStateChange = Date.now();

    // BMS mock state (persists between calls for realistic drift)
    const BMS_STACKS = 6;
    const BMS_CELLS_PER_STACK = 24;
    const BMS_NTCS_PER_STACK = 24;
    const BMS_TOTAL_CELLS = BMS_STACKS * BMS_CELLS_PER_STACK;  // 144
    const BMS_TOTAL_TEMPS = BMS_STACKS * BMS_NTCS_PER_STACK;   // 144

    // Initialize base voltages and temps with slight per-cell variation
    const bmsCellVoltages = Array.from({ length: BMS_TOTAL_CELLS }, () =>
      3.5 + (Math.random() - 0.5) * 0.3
    );
    const bmsCellTemps = Array.from({ length: BMS_TOTAL_TEMPS }, () =>
      28 + (Math.random() - 0.5) * 10
    );
    let bmsCurrentPhase = 0;
    let bmsFlagAccum = [];

    const BMS_FLAG_TYPES = [
      { code: 'OV', message: 'Overvoltage detected' },
      { code: 'UV', message: 'Undervoltage detected' },
      { code: 'OT', message: 'Overtemperature detected' },
      { code: 'UT', message: 'Undertemperature detected' },
      { code: 'OC', message: 'Overcurrent detected' },
      { code: 'COM', message: 'Communication lost with stack' },
      { code: 'BAL', message: 'Cell balancing active' },
      { code: 'ISO', message: 'Isolation fault' },
    ];

    function generateBmsMockData(now) {
      // Drift voltages slightly each tick
      for (let i = 0; i < BMS_TOTAL_CELLS; i++) {
        bmsCellVoltages[i] += (Math.random() - 0.5) * 0.02;
        bmsCellVoltages[i] = Math.max(2.4, Math.min(4.3, bmsCellVoltages[i]));
      }

      // Occasional outlier cell
      if (Math.random() < 0.05) {
        const idx = Math.floor(Math.random() * BMS_TOTAL_CELLS);
        bmsCellVoltages[idx] = Math.random() < 0.5
          ? 2.5 + Math.random() * 0.3   // low
          : 4.0 + Math.random() * 0.2;  // high
      }

      // Drift temps slightly
      for (let i = 0; i < BMS_TOTAL_TEMPS; i++) {
        bmsCellTemps[i] += (Math.random() - 0.5) * 0.5;
        bmsCellTemps[i] = Math.max(-35, Math.min(65, bmsCellTemps[i]));
      }

      // Occasional hot spot
      if (Math.random() < 0.03) {
        const idx = Math.floor(Math.random() * BMS_TOTAL_TEMPS);
        bmsCellTemps[idx] = 45 + Math.random() * 15;
      }

      // Slowly varying sinusoidal current
      bmsCurrentPhase += 0.05;
      const packCurrent = 40 * Math.sin(bmsCurrentPhase) + (Math.random() - 0.5) * 5;

      // Occasional new flags
      const newFlags = [];
      if (Math.random() < 0.08) {
        const ft = BMS_FLAG_TYPES[Math.floor(Math.random() * BMS_FLAG_TYPES.length)];
        const stack = Math.floor(Math.random() * BMS_STACKS);
        newFlags.push({
          code: ft.code,
          message: `${ft.message} (Stack ${stack + 1})`,
          timestamp: now,
        });
      }
      bmsFlagAccum = [...newFlags, ...bmsFlagAccum].slice(0, 100);

      return {
        cell_voltages: [...bmsCellVoltages],
        temperatures: [...bmsCellTemps],
        pack_current: packCurrent,
        flags: bmsFlagAccum,
        stacks: BMS_STACKS,
        cells_per_stack: BMS_CELLS_PER_STACK,
        ntcs_per_stack: BMS_NTCS_PER_STACK,
      };
    }

    function generateMockData() {
      const now = Date.now();
      if (Math.random() < 0.3) {
        lastFSMState = fsmStates[Math.floor(Math.random() * fsmStates.length)];
        lastStateChange = now;
      }

      return {
        timestamp_us: now * 1000,
        mode: Math.floor(Math.random() * 8),
        digital: {
          "ASMS": Math.random() > 0.5,
          "AS ATS": Math.random() > 0.5,
          "ATS Btn": Math.random() > 0.5,
          "TSMS SDC": Math.random() > 0.5,
          "Master SDC": Math.random() > 0.5,
          "TS On": Math.random() > 0.5,
          "WD Ready": Math.random() > 0.5,
          "Emergency": Math.random() > 0.9,
          "EBS Engage": Math.random() > 0.5,
          "EBS Release": Math.random() > 0.5,
          "Bamocar TS": Math.random() > 0.5,
          "Master ASMS": Math.random() > 0.5,
        },
        analog: {
          "Hydraulic Front (bar)": Math.random() * 10,
          "Hydraulic Rear (bar)": Math.random() * 10,
          "Brake Pressure": Math.floor(Math.random() * 1024),
          "APPS Higher": Math.floor(Math.random() * 4096),
          "APPS Lower": Math.floor(Math.random() * 4096),
          "RPM FR": Math.random() * 2000,
          "RPM FL": Math.random() * 2000,
          "DC Voltage": Math.floor(Math.random() * 600),
          "Motor Speed": Math.floor(Math.random() * 5000),
          "Motor Current": Math.floor(Math.random() * 200),
          "SoC": Math.floor(Math.random() * 100),
        },
        valid: {
          "ASMS": true, "AS ATS": true, "ATS Btn": true,
          "TSMS SDC": true, "Master SDC": true, "TS On": true,
          "WD Ready": true, "Emergency": true, "EBS Engage": true,
          "EBS Release": true, "Bamocar TS": true, "Master ASMS": true,
          "Hydraulic Front (bar)": true, "Hydraulic Rear (bar)": true,
          "Brake Pressure": true, "APPS Higher": true, "APPS Lower": true,
          "RPM FR": true, "RPM FL": true, "DC Voltage": true,
          "Motor Speed": true, "Motor Current": true, "SoC": true,
          "mode": true,
        },
        actuators: {},
        fsm: {
          fsm_state: lastFSMState,
          time_in_state_ms: now - lastStateChange,
          checkup_state: Math.floor(Math.random() * 12),
          ebs_state: Math.floor(Math.random() * 7),
          error_messages: lastFSMState === 5 ? ['Emergency signal active'] : [],
          fault_messages: Math.random() > 0.7 ? ['Steering timeout', 'PC timeout'] : [],
          battery_st: Math.floor(Math.random() * 4),
        },
        bms: generateBmsMockData(now),
      };
    }

    setInterval(() => {
      const mockData = generateMockData();
      allSnapshots.push(mockData);

      if (win && !win.isDestroyed() && win.webContents) {
        win.webContents.send('sensor-data', mockData);
      }
    }, 1000);
  }
  else {
    // Spawn the C++ CAN listener as a child process
    console.log('Spawning CAN listener:', CAN_BINARY, CAN_DEVICE);
    canProcess = spawn(CAN_BINARY, [CAN_DEVICE]);

    let buffer = '';

    canProcess.stdout.on('data', (chunk) => {
      buffer += chunk.toString();

      // Process complete lines
      let newlineIdx;
      while ((newlineIdx = buffer.indexOf('\n')) !== -1) {
        const line = buffer.slice(0, newlineIdx).trim();
        buffer = buffer.slice(newlineIdx + 1);

        if (!line.startsWith('{')) continue;

        try {
          const flat = JSON.parse(line);
          const data = transformSnapshot(flat);
          allSnapshots.push(data);

          if (win && !win.isDestroyed() && win.webContents) {
            win.webContents.send('sensor-data', data);
          }
        } catch (err) {
          console.error('Failed to parse CAN JSON:', line);
        }
      }
    });

    canProcess.stderr.on('data', (chunk) => {
      console.error('CAN listener:', chunk.toString());
    });

    canProcess.on('close', (code) => {
      console.log('CAN listener exited with code', code);
      canProcess = null;
    });
  }

  ipcMain.on('toggle-fullscreen', (events, isChartsView) => {
    if (win) {
      if (isChartsView){
        win.maximize();
      } else {
        win.unmaximize();
      }
    }
  });


  // --- Export CSV handler ---
  ipcMain.on('export-data', () => {
    if (!allSnapshots.length) {
      if (win && win.webContents) win.webContents.send('export-done', null);
      return;
    }

    const filename = path.join(__dirname, 'snapshots.csv');

    const flatten = (obj, prefix = '') =>
      Object.entries(obj).reduce((acc, [key, val]) => {
        const newKey = prefix ? `${prefix}_${key}` : key;
        if (typeof val === 'object' && val !== null)
          Object.assign(acc, flatten(val, newKey));
        else acc[newKey] = val;
        return acc;
      }, {});

    const flat = allSnapshots.map(o => flatten(o));
    const headers = Object.keys(flat[0]);
    const rows = flat.map(obj => headers.map(h => JSON.stringify(obj[h] ?? '')).join(','));
    const csv = [headers.join(','), ...rows].join(',\n');

    fs.writeFileSync(filename, csv);
    console.log(`Data saved to ${filename}`);
    if (win && win.webContents) win.webContents.send('export-done', filename);
  });

  // --- Auto-export fault log on session close ---
  let faultsExported = false;

  win.on('close', (event) => {
    if (!faultsExported) {
      event.preventDefault();
      win.webContents.send('request-fault-export');
    }
  });

  ipcMain.on('fault-export-data', (_event, { cleared = [], active = [] }) => {
    try {
      const logsDir = path.join(__dirname, 'logs');
      fs.mkdirSync(logsDir, { recursive: true });

      const now = new Date();
      const stamp = now.toISOString().replace(/[:.]/g, '-').slice(0, 19);
      const filename = path.join(logsDir, `faults_${stamp}.csv`);

      const rows = [
        'timestamp,fault',
        ...cleared.map(({ time, fault }) => `"${time}","${fault}"`),
        ...active.map(fault => `"${now.toTimeString().slice(0, 12)}","${fault}"`),
      ];

      if (rows.length > 1) {
        fs.writeFileSync(filename, rows.join('\n'));
        console.log(`Fault log saved to ${filename}`);
      }
    } catch (err) {
      console.error('Failed to save fault log:', err);
    } finally {
      faultsExported = true;
      win.close();
    }
  });

});
