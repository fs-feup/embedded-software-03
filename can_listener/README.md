# CAN Listener

Reads CAN data from car/handcart and outputs JSON to stdout.

## Executables

- `can_listener_handcart.exe` - 125k baud (S4) for handcart
- `can_listener_accumulator.exe` - 800k baud (S7) for car

## Usage

Connect CAN adapter to PC, then:
```cmd
can_listener_handcart.exe COM7
```

Used by `foxglove_bridge` to stream data to Foxglove Studio.

## Rebuilding

Edit `can_car_listener.hpp` line 29:
- `S4` = 125k for handcart
- `S7` = 800k for accumulator

Then run `build_vs.bat` (requires VS 2022).
