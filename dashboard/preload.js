const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('electronAPI', {
  // Receive sensor data from main process
  onSensorData: (callback) => {
    ipcRenderer.on('sensor-data', (event, data) => callback(data));
  },

  send: (channel, data) => {
    ipcRenderer.send(channel, data);
  },

  // Request data export
  exportData: () => {
    ipcRenderer.send('export-data');
  },

  // Receive export confirmation
  onExportDone: (callback) => {
    ipcRenderer.on('export-done', (event, filename) => callback(filename));
  },

  // Main requests fault data before closing
  onRequestFaultExport: (callback) => {
    ipcRenderer.on('request-fault-export', () => callback());
  },

  // Renderer sends fault data back to main
  sendFaultData: (data) => {
    ipcRenderer.send('fault-export-data', data);
  }
});
