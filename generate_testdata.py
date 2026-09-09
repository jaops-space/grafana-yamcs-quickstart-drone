#!/usr/bin/env python3
"""Extend the original quickstart CCSDS recording with coherent drone telemetry."""

from __future__ import annotations

import argparse
import math
import struct
from pathlib import Path


LEGACY_PACKET_SIZE = 123
CCSDS_HEADER_SIZE = 6
DEFAULT_INPUT = Path("testdata.ccsds")


def fixed_string(value: str, size: int = 16) -> bytes:
    encoded = value.encode("utf-8")[: size - 1]
    return encoded + bytes(size - len(encoded))


def flight_state(second: int) -> tuple[int, str, bool, float, float]:
    """Return mode, phase label, armed state, altitude and forward speed."""
    t = second % 300
    if t < 30:
        return 0, "STANDBY", False, 0.0, 0.0
    if t < 60:
        progress = (t - 30) / 30
        return 2, "TAKEOFF", True, 40.0 * progress, 3.0 * progress
    if t < 180:
        return 3, "SURVEY", True, 40.0 + 2.0 * math.sin(t / 12), 12.0
    if t < 220:
        progress = (t - 180) / 40
        return 4, "RETURN", True, 40.0 - 5.0 * progress, 14.0
    if t < 250:
        progress = (t - 220) / 30
        return 5, "LANDING", True, 35.0 * (1.0 - progress), 4.0 * (1.0 - progress)
    return 6, "CHARGING", False, 0.0, 0.0


def drone_extension(second: int) -> bytes:
    cycle_t = second % 300
    mode, phase, armed, altitude, speed = flight_state(second)

    in_flight = armed and mode not in (0, 1)
    throttle = 0 if not in_flight else (68 if mode == 2 else 52 if mode == 3 else 58 if mode == 4 else 38)
    battery_soc = max(12.0, 96.0 - cycle_t * 0.29) if cycle_t < 250 else 20.0 + (cycle_t - 250) * 1.45
    pack_voltage = 26.0 + battery_soc * 0.074 - (1.1 if in_flight else 0.0)
    pack_current = (4.5 + throttle * 0.72) if in_flight else (-18.0 if mode == 6 else 1.2)
    battery_temp = 25.0 + (pack_current if pack_current > 0 else 0) * 0.16

    # A deliberate, physically related motor-3 bearing event near the end of survey.
    motor3_fault = 165 <= cycle_t < 205
    failsafe = 190 <= cycle_t < 205
    if failsafe:
        mode = 4
        phase = "FAILSAFE_RETURN"

    payload = bytearray()
    payload.extend(struct.pack(">IBBB16s", second, mode, int(armed), int(failsafe), fixed_string("JAOPS-DRONE-01")))
    voltage_adc = round(pack_voltage / 0.01)
    current_adc = round(pack_current / 0.01)
    temperature_adc = round(battery_temp / 0.1)
    payload.extend(
        struct.pack(
            ">HHhhBhh",
            voltage_adc,
            voltage_adc,
            current_adc,
            current_adc,
            round(battery_soc),
            temperature_adc,
            temperature_adc,
        )
    )
    cell_base = pack_voltage / 8
    cell_offsets = (-0.018, 0.006, 0.012, 0.0, -0.008, 0.009, -0.004, 0.003)
    payload.extend(struct.pack(">8H", *(round((cell_base + d) / 0.001) for d in cell_offsets)))

    for motor in range(4):
        motor_throttle = throttle + (motor - 1) if in_flight else 0
        rpm = max(0, round(motor_throttle * 122 + 35 * math.sin(second / 3 + motor)))
        current = max(0.0, pack_current / 4 + 0.15 * math.sin(second / 4 + motor)) if in_flight else 0.0
        temperature = 27.0 + current * 1.45 + 1.5 * math.sin(second / 20 + motor)
        status = 1 if in_flight else 0
        if motor == 2 and motor3_fault:
            rpm = round(rpm * 0.72)
            current += 8.0
            temperature += 24.0
            status = 2 if not failsafe else 3
        payload.extend(
            struct.pack(
                ">HHhBB",
                rpm,
                round(current / 0.01),
                round(temperature / 0.1),
                max(0, min(100, motor_throttle)),
                status,
            )
        )

    roll = 3.0 * math.sin(second / 8) if in_flight else 0.0
    pitch = 4.0 * math.sin(second / 11) if in_flight else 0.0
    yaw = (second * 1.7) % 360
    payload.extend(struct.pack(">6f", roll, pitch, yaw, roll / 4, pitch / 4, 1.7 if in_flight else 0.0))

    latitude = 48.8566 + (0.0012 * math.sin(second / 35) if in_flight else 0.0)
    longitude = 2.3522 + (0.0018 * math.cos(second / 35) if in_flight else 0.0)
    payload.extend(struct.pack(">4f", latitude, longitude, altitude, speed))

    rail_voltages = (pack_voltage, 12.0 - (0.15 if in_flight else 0), 5.0 - (0.04 if in_flight else 0))
    branch_currents = (
        max(0.0, pack_current * 0.88),
        1.8 if in_flight else 0.5,
        0.9 if in_flight else 0.3,
        0.6,
    )
    payload.extend(struct.pack(">3H", *(round(v / 0.001) for v in rail_voltages)))
    payload.extend(struct.pack(">4H", *(round(v / 0.001) for v in branch_currents)))
    payload.extend(struct.pack(">4B", int(armed), 1, 1, 1))
    payload.extend(struct.pack(">4B", 2 if failsafe else 0, 1 if motor3_fault else 0, 0, 0))

    vibration = []
    for motor in range(4):
        base = 0.08 + (throttle / 100) * 0.42
        if motor == 2 and motor3_fault:
            base += 1.1
        vibration.extend(base * scale for scale in (1.0, 0.55, 0.28))
    payload.extend(struct.pack(">12H", *(round(v / 0.001) for v in vibration)))
    payload.extend(fixed_string(phase))
    payload.extend(struct.pack(">HBf", 0, 0, 40.0))
    return bytes(payload)


DRONE_EXTENSION_SIZE = len(drone_extension(0))


def iter_legacy_packets(data: bytes):
    offset = 0
    while offset + CCSDS_HEADER_SIZE <= len(data):
        packet_length = struct.unpack_from(">H", data, offset + 4)[0] + 7
        if packet_length < LEGACY_PACKET_SIZE or offset + packet_length > len(data):
            raise ValueError(f"invalid CCSDS packet at byte {offset}")
        # Idempotently retain only the original quickstart packet prefix.
        yield bytearray(data[offset : offset + LEGACY_PACKET_SIZE])
        offset += packet_length
    if offset != len(data):
        raise ValueError(f"trailing data at byte {offset}")


def generate(source: Path, output: Path) -> tuple[int, int]:
    packets = []
    for second, packet in enumerate(iter_legacy_packets(source.read_bytes())):
        extension = drone_extension(second)
        pack_voltage = struct.unpack_from(">H", extension, 25)[0] * 0.01
        battery_temperature = struct.unpack_from(">h", extension, 34)[0] * 0.1
        # Keep the original quickstart battery channels as two parallel 8S
        # modules feeding the drone bus instead of unrelated legacy values.
        struct.pack_into(
            ">4f",
            packet,
            58,
            pack_voltage + 0.03,
            pack_voltage - 0.02,
            battery_temperature + 0.4,
            battery_temperature - 0.3,
        )
        packet.extend(extension)
        struct.pack_into(">H", packet, 4, len(packet) - 7)
        packets.append(packet)
    output.write_bytes(b"".join(packets))
    return len(packets), len(packets[0]) if packets else 0


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--output", type=Path, default=DEFAULT_INPUT)
    args = parser.parse_args()
    count, packet_size = generate(args.input, args.output)
    print(f"wrote {count} packets of {packet_size} bytes ({DRONE_EXTENSION_SIZE}-byte drone extension)")


if __name__ == "__main__":
    main()
