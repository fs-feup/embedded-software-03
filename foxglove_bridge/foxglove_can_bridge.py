#!/usr/bin/env python3
# Foxglove CAN Bridge - connects can_listener to Foxglove Studio via WebSocket
# Usage: python foxglove_can_bridge.py COM3

import asyncio
import json
import subprocess
import sys
import time
from typing import Any

from foxglove_websocket import run_cancellable
from foxglove_websocket.server import FoxgloveServer, FoxgloveServerListener
from foxglove_websocket.types import ChannelId

# JSON schemas for Foxglove
CELLS_TEMPS_SCHEMA = json.dumps({
    "type": "object",
    "properties": {
        "min_temp": {"type": "number"},
        "max_temp": {"type": "number"},
        "avg_temp": {"type": "number"},
        "all_cells": {"type": "array", "items": {"type": "number"}}
    }
})

BMS_ERRORS_SCHEMA = json.dumps({
    "type": "object",
    "properties": {
        "bms_current": {"type": "number"},
        "discharge_limit_enforcement_fault": {"type": "boolean"},
        "charger_safety_relay_fault": {"type": "boolean"},
        "internal_hardware_fault": {"type": "boolean"},
        "internal_heatsink_thermistor_fault": {"type": "boolean"},
        "internal_software_fault": {"type": "boolean"},
        "highest_cell_voltage_too_high_fault": {"type": "boolean"},
        "lowest_cell_voltage_too_low_fault": {"type": "boolean"},
        "pack_too_hot_fault": {"type": "boolean"},
        "internal_communication_fault": {"type": "boolean"},
        "cell_balancing_stuck_off_fault": {"type": "boolean"},
        "weak_cell_fault": {"type": "boolean"},
        "low_cell_voltage_fault": {"type": "boolean"},
        "open_wiring_fault": {"type": "boolean"},
        "current_sensor_fault": {"type": "boolean"},
        "highest_cell_voltage_over_5v_fault": {"type": "boolean"},
        "cell_asic_fault": {"type": "boolean"},
        "weak_pack_fault": {"type": "boolean"},
        "fan_monitor_fault": {"type": "boolean"},
        "thermistor_fault": {"type": "boolean"},
        "external_communication_fault": {"type": "boolean"},
        "redundant_power_supply_fault": {"type": "boolean"},
        "high_voltage_isolation_fault": {"type": "boolean"},
        "input_power_supply_fault": {"type": "boolean"},
        "charge_limit_enforcement_fault": {"type": "boolean"}
    }
})

INVERTER_VOLTAGE_SCHEMA = json.dumps({
    "type": "object",
    "properties": {
        "data": {"type": "number"}
    }
})

# Generic schema for raw CAN data
RAW_CAN_SCHEMA = json.dumps({
    "type": "object",
    "additionalProperties": True
})


class Listener(FoxgloveServerListener):
    def on_subscribe(self, server: FoxgloveServer, channel_id: ChannelId) -> None:
        print(f"Client subscribed to channel {channel_id}")

    def on_unsubscribe(self, server: FoxgloveServer, channel_id: ChannelId) -> None:
        print(f"Client unsubscribed from channel {channel_id}")


def parse_can_to_cells_temps(data: dict) -> dict:
    # Flatten 6 boards × 18 NTCs into single array
    all_cells = []
    cells_all_temps = data.get("cells_all_temps", [])

    for board in cells_all_temps:
        for i, temp in enumerate(board):
            if i < 18 and temp is not None:  # Only first 18 are real
                all_cells.append(temp)
            elif i < 18:
                all_cells.append(0)

    return {
        "min_temp": data.get("therm_min", data.get("cells_global_min", 0)),
        "max_temp": data.get("therm_max", data.get("cells_global_max", 0)),
        "avg_temp": data.get("therm_avg", 0),
        "all_cells": all_cells
    }


def parse_can_to_bms_errors(data: dict) -> dict:
    # Bamocar error bitmap
    error_bitmap = data.get("bamocar_error_bitmap", 0)

    return {
        "bms_current": data.get("bamocar_motor_current", 0),
        "discharge_limit_enforcement_fault": False,
        "charger_safety_relay_fault": False,
        "internal_hardware_fault": bool(error_bitmap & 0x0001),
        "internal_heatsink_thermistor_fault": bool(error_bitmap & 0x0002),
        "internal_software_fault": bool(error_bitmap & 0x0004),
        "highest_cell_voltage_too_high_fault": False,
        "lowest_cell_voltage_too_low_fault": False,
        "pack_too_hot_fault": data.get("therm_max", 0) > 55,
        "internal_communication_fault": data.get("bms_dead", False),
        "cell_balancing_stuck_off_fault": False,
        "weak_cell_fault": False,
        "low_cell_voltage_fault": False,
        "open_wiring_fault": False,
        "current_sensor_fault": False,
        "highest_cell_voltage_over_5v_fault": False,
        "cell_asic_fault": False,
        "weak_pack_fault": False,
        "fan_monitor_fault": False,
        "thermistor_fault": not data.get("therm_min_valid", True),
        "external_communication_fault": False,
        "redundant_power_supply_fault": False,
        "high_voltage_isolation_fault": False,
        "input_power_supply_fault": False,
        "charge_limit_enforcement_fault": False
    }


def parse_can_to_inverter_voltage(data: dict) -> dict:
    return {
        "data": data.get("dc_voltage", 0)
    }


async def main(com_port: str):
    print(f"Foxglove CAN Bridge - port 9000")
    print(f"CAN port: {com_port}")
    print(f"Connect Foxglove to: ws://localhost:9000\n")

    async with FoxgloveServer("0.0.0.0", 9000, "CAN Bridge") as server:
        server.set_listener(Listener())

        # Register channels
        cells_temp_chan = await server.add_channel({
            "topic": "/vehicle/cells/temperature",
            "encoding": "json",
            "schemaName": "CellsTemps",
            "schema": CELLS_TEMPS_SCHEMA,
        })

        bms_errors_chan = await server.add_channel({
            "topic": "/vehicle/bms_custom",
            "encoding": "json",
            "schemaName": "BmsErrors",
            "schema": BMS_ERRORS_SCHEMA,
        })

        inverter_chan = await server.add_channel({
            "topic": "/vehicle/inverter_voltage",
            "encoding": "json",
            "schemaName": "InverterVoltage",
            "schema": INVERTER_VOLTAGE_SCHEMA,
        })

        raw_can_chan = await server.add_channel({
            "topic": "/can/raw",
            "encoding": "json",
            "schemaName": "RawCAN",
            "schema": RAW_CAN_SCHEMA,
        })

        print("Channels registered. Starting can_listener...\n")

        # Start can_listener subprocess
        # handcart (125k) or accumulator (800k)
        can_listener_path = sys.path[0] + "/../can_listener/can_listener_handcart.exe"

        process = await asyncio.create_subprocess_exec(
            can_listener_path, com_port,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE
        )

        print(f"can_listener PID: {process.pid}")
        print("Broadcasting to Foxglove (printing every 50th msg)\n")

        msg_count = 0

        try:
            while True:
                line = await process.stdout.readline()
                if not line:
                    break

                try:
                    data = json.loads(line.decode('utf-8').strip())
                    now_ns = time.time_ns()

                    msg_count += 1
                    if msg_count % 50 == 1:
                        cells_valid = sum(data.get("cells_board_valid", []))
                        print(f"[{msg_count}] Received data - cells_global_min: {data.get('cells_global_min', 'N/A')}, "
                              f"cells_global_max: {data.get('cells_global_max', 'N/A')}, "
                              f"boards_valid: {cells_valid}/6")

                    # Publish to all channels
                    await server.send_message(
                        cells_temp_chan,
                        now_ns,
                        json.dumps(parse_can_to_cells_temps(data)).encode()
                    )

                    await server.send_message(
                        bms_errors_chan,
                        now_ns,
                        json.dumps(parse_can_to_bms_errors(data)).encode()
                    )

                    await server.send_message(
                        inverter_chan,
                        now_ns,
                        json.dumps(parse_can_to_inverter_voltage(data)).encode()
                    )

                    # Also publish raw data for debugging
                    await server.send_message(
                        raw_can_chan,
                        now_ns,
                        line.strip()
                    )

                except json.JSONDecodeError as e:
                    print(f"JSON error: {e}")
                    print(f"Raw: {line[:200]}")
                    continue

        except asyncio.CancelledError:
            print("\nShutting down...")
            process.terminate()
            await process.wait()
            raise


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python foxglove_can_bridge.py <COM_PORT>")
        print("Example: python foxglove_can_bridge.py COM3")
        sys.exit(1)

    com_port = sys.argv[1]
    run_cancellable(main(com_port))
