# Foxglove CAN Bridge

Bridges CAN data from can_listener to Foxglove Studio.

## Setup

```bash
pip install -r requirements.txt
```

## Usage

Connect CANdapter to PC (check COM port):

```bash
python foxglove_can_bridge.py COM3
```

In Foxglove Studio:
- Open connection -> Foxglove WebSocket
- Connect to: `ws://localhost:9000`

## Topics

- `/vehicle/cells/temperature` - Cell temps
- `/vehicle/bms_custom` - BMS errors and current
- `/vehicle/inverter_voltage` - DC bus voltage
- `/can/raw` - Raw JSON

## Architecture

```
CANdapter -> can_listener -> foxglove_can_bridge -> ws://localhost:9000 -> Foxglove
```
