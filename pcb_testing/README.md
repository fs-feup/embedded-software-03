# PCB Testing Suite

Test firmware for Dashboard and VCU PCBs.

## Quick Start

Upload firmware:
```bash
cd pcb_testing/teensy_dash_test  # or teensy_vcu_test
pio run -t upload
pio device monitor  # 115200 baud
```

Run tests:
- Press `0` for menu
- Press `i` (Dashboard) or `m` (VCU) for full test
- Or select individual tests

## Tests

### Dashboard PCB
- Digital I/O (LEDs, buzzer, switches)
- Analog sensors (APPS, brake, rotary)
- Wheel encoders
- I2C bus scan, IMU (MPU6050)
- Power rail voltages
- PWM outputs
- ADC calibration
- CAN TX/RX/stress/bidirectional
- Interrupt timing
- Continuous monitor

### VCU PCB
- Digital I/O (ASSI LEDs, EBS valves, SDC, watchdog)
- Analog sensors (brake 0-95 bar, SOC)
- Wheel speed sensors
- EBS pressure sensors, SDC logic
- I2C bus scan, current sensor, temp sensors
- Power rail voltages
- PWM outputs
- ADC calibration
- CAN TX/RX/stress/bidirectional
- Interrupt timing
- Continuous monitor
- EBS valve actuation (with safety confirmation)

## CAN Communication Test

Setup:
```
Dashboard ---- CAN H ---- 120Ω ---- VCU
          ---- CAN L --------------
          ---- GND ----------------
```

Test:
1. VCU: Press `g` (listen)
2. Dashboard: Press `c` (transmit)

Reverse:
1. Dashboard: Press `d` (listen)
2. VCU: Press `f` (transmit)

## Hardware Setup

Dashboard:
- 2x Pots: Pins 20, 22 (APPS)
- 1x Pot: Pin 19 (Brake)
- 5x Buttons: Pins 4, 5, 7, 18, 21
- LEDs: Pins 14-17
- CAN transceiver
- MPU6050 on I2C (optional)

VCU:
- 1x Pot: Pin 38 (Brake 0-95 bar)
- 1x Pot: Pin 24 (SOC)
- 8x Buttons (safety switches)
- 2x LEDs: Pins 25, 12 (ASSI)
- CAN transceiver

## Coverage

Both boards:
- Basic I/O (digital/analog)
- CAN (TX/RX/stress/bidirectional)
- I2C bus scan
- Encoders/WSS
- PWM outputs
- ADC noise analysis
- Interrupt timing
- Power rails

Dashboard specific:
- APPS dual sensor plausibility
- IMU accelerometer
- SPI interface

VCU specific:
- Hydraulic pressure 0-95 bar
- EBS sensor thresholds
- SDC logic
- Watchdog pulse
- Current sensor (I2C)
- Temperature sensors

## Pin Definitions

Pins match:
- Dashboard: `teensy_dash/include/io_settings.hpp`
- VCU: `master/include/embedded/hardwareSettings.hpp`

If PCB pins differ, update `namespace pins` in each `main.cpp`.
 