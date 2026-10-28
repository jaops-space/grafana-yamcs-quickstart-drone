#!/usr/bin/env python3
"""Extend the original quickstart CCSDS recording with coherent drone telemetry.

One mission repeats every 300 s. Every value is derived from a single flight
model so the telemetry stays physically consistent:

    0-25 s    STANDBY   disarmed on the pad, motors stopped
    25-32 s   ARMED     armed on the pad, props at idle
    32-60 s   TAKEOFF   vertical climb to 40 m
    60-180 s  SURVEY    1.5 laps of an 80 m circle that starts at home
    180-220 s RETURN    straight line back to home, 40 m -> 35 m
    220-250 s LANDING   vertical descent, touchdown at 250 s
    250-300 s CHARGING  disarmed, battery recharges back to the start level

A motor 3 bearing event shows the alarm chain: DEGRADED from 160 s, FAULT and
failsafe (motor power overcurrent) from 190 s to 205 s during the return.
"""

from __future__ import annotations

import argparse
import math
import struct
from pathlib import Path


LEGACY_PACKET_SIZE = 123
CCSDS_HEADER_SIZE = 6
DEFAULT_INPUT = Path("testdata.ccsds")

CYCLE = 300
HOME_LAT, HOME_LON = 48.8566, 2.3522
CRUISE_ALT = 40.0
CIRCLE_R = 80.0           # survey circle passes through home, centre 80 m north
SOC_START = 96.0
AMBIENT_C = 27.0
G = 9.81

STANDBY, ARMED, TAKEOFF, SURVEY, RETURN, LANDING, CHARGING = 0, 1, 2, 3, 4, 5, 6


def fixed_string(value: str, size: int = 16) -> bytes:
    encoded = value.encode("utf-8")[: size - 1]
    return encoded + bytes(size - len(encoded))


def smoothstep(x: float) -> float:
    x = max(0.0, min(1.0, x))
    return x * x * (3 - 2 * x)


def phase(t: float) -> tuple[int, str]:
    for end, mode, label in ((25, STANDBY, "STANDBY"), (32, ARMED, "ARMED"), (60, TAKEOFF, "TAKEOFF"),
                             (180, SURVEY, "SURVEY"), (220, RETURN, "RETURN"), (250, LANDING, "LANDING")):
        if t < end:
            return mode, label
    return CHARGING, "CHARGING"


def position(t: float) -> tuple[float, float, float]:
    """North and east offset from home in metres, and altitude in metres."""
    mode, _ = phase(t)
    if mode in (STANDBY, ARMED, CHARGING):
        return 0.0, 0.0, 0.0
    if mode == TAKEOFF:
        return 0.0, 0.0, CRUISE_ALT * smoothstep((t - 32) / 28)
    if mode == SURVEY:
        a = math.pi + 3 * math.pi * (t - 60) / 120          # home -> 1.5 laps -> far side
        return CIRCLE_R + CIRCLE_R * math.cos(a), CIRCLE_R * math.sin(a), CRUISE_ALT + 1.5 * math.sin((t - 60) / 9)
    if mode == RETURN:
        k = smoothstep((t - 180) / 40)
        return 2 * CIRCLE_R * (1 - k), 0.0, CRUISE_ALT - 5.0 * k
    return 0.0, 0.0, 35.0 * (1 - smoothstep((t - 220) / 30))   # LANDING


def kinematics(t: float) -> dict:
    """Velocity, heading, bank and pitch from the position track."""
    dt = 0.5
    n0, e0, a0 = position(t - dt)
    n1, e1, a1 = position(t + dt)
    vn, ve, vz = (n1 - n0) / (2 * dt), (e1 - e0) / (2 * dt), (a1 - a0) / (2 * dt)
    speed = math.hypot(vn, ve)
    mode, _ = phase(t)
    if speed > 0.5:
        heading = math.degrees(math.atan2(ve, vn)) % 360
    else:
        heading = 0.0       # hover, takeoff, landing and parked: nose north
    # Banked turn: tan(roll) = v * omega / g, omega = v / R on the survey circle.
    roll = math.degrees(math.atan(speed * speed / (CIRCLE_R * G))) if mode == SURVEY else 0.0
    # Multirotors tilt nose-down to fly forward (negative pitch = nose down).
    pitch = -1.1 * speed if speed > 0.5 else 0.0
    airborne = position(t)[2] > 0.05 or mode in (TAKEOFF, LANDING)
    if airborne:
        roll += 0.6 * math.sin(t / 2.3)       # gust buffeting
        pitch += 0.5 * math.sin(t / 3.1)
    return {"speed": speed, "climb": vz, "heading": heading, "roll": roll, "pitch": pitch, "airborne": airborne}


def motor3_state(t: float) -> int:
    """0 stopped, 1 running, 2 degraded, 3 fault for motor 3 when spinning."""
    if 190 <= t < 205:
        return 3
    if 160 <= t < 220:
        return 2
    return 1


def throttle_at(t: float) -> float:
    mode, _ = phase(t)
    if mode in (STANDBY, CHARGING):
        return 0.0
    if mode == ARMED:
        return 8.0                                          # props idle on the pad
    k = kinematics(t)
    return max(10.0, min(95.0, 48.0 + 6.0 * k["climb"] + 0.9 * k["speed"]))


def simulate() -> list[dict]:
    """Integrate battery state of charge and temperatures second by second."""
    soc = SOC_START
    motor_temp = [AMBIENT_C] * 4
    battery_temp = AMBIENT_C
    states = []
    soc_at_touchdown = None
    for t in range(CYCLE):
        mode, label = phase(t)
        thr = throttle_at(t)
        flying = mode not in (STANDBY, ARMED, CHARGING)
        failsafe = 190 <= t < 205
        motor_current = []
        for m in range(4):
            spinning = thr > 0
            cur = (thr * 0.22 + 0.15 * math.sin(t / 4 + m)) if spinning else 0.0
            if m == 2 and spinning and motor3_state(t) >= 2:
                cur += 6.0 if motor3_state(t) == 2 else 9.5
            motor_current.append(max(0.0, cur))
        avionics = 1.6 if mode != CHARGING else 0.6
        pack_current = -18.0 if mode == CHARGING else sum(motor_current) + avionics

        if mode == CHARGING:
            if soc_at_touchdown is None:
                soc_at_touchdown = soc
            soc = soc_at_touchdown + (SOC_START - soc_at_touchdown) * (t - 249) / (CYCLE - 250)
        else:
            soc -= pack_current * 0.0048          # ~5.8 Ah pack, % per second

        for m in range(4):
            heat = motor_current[m] * 1.9
            motor_temp[m] += (AMBIENT_C + heat - motor_temp[m]) / 25.0   # first-order thermal lag
        battery_temp += (AMBIENT_C + max(pack_current, 0) * 0.16 - battery_temp) / 60.0

        states.append({"t": t, "mode": mode if not failsafe else RETURN,
                       "label": "FAILSAFE_RETURN" if failsafe else label,
                       "armed": mode not in (STANDBY, CHARGING), "flying": flying, "failsafe": failsafe,
                       "throttle": thr, "motor_current": motor_current, "motor_temp": list(motor_temp),
                       "pack_current": pack_current, "soc": soc, "battery_temp": battery_temp})
    return states


STATES = simulate()


def drone_extension(second: int) -> bytes:
    s = STATES[second % CYCLE]
    t = s["t"]
    n, e, altitude = position(t)
    k = kinematics(t)
    soc = max(0.0, min(100.0, s["soc"]))
    pack_current = s["pack_current"]
    pack_voltage = 25.6 + soc * 0.078 - (0.012 * pack_current if pack_current > 0 else -0.25)

    payload = bytearray()
    payload.extend(struct.pack(">IBBB16s", second, s["mode"], int(s["armed"]), int(s["failsafe"]),
                               fixed_string("JAOPS-DRONE-01")))
    voltage_adc = round(pack_voltage / 0.01)
    current_adc = round(pack_current / 0.01)
    temperature_adc = round(s["battery_temp"] / 0.1)
    payload.extend(struct.pack(">HHhhBhh", voltage_adc, voltage_adc, current_adc, current_adc,
                               round(soc), temperature_adc, temperature_adc))
    cell_base = pack_voltage / 8
    cell_offsets = (-0.018, 0.006, 0.012, 0.0, -0.008, 0.009, -0.004, 0.003)
    payload.extend(struct.pack(">8H", *(round((cell_base + d) / 0.001) for d in cell_offsets)))

    for m in range(4):
        thr = s["throttle"] + (m - 1.5) * 0.8 if s["throttle"] > 0 else 0.0
        rpm = round(thr * 120 + 30 * math.sin(second / 3 + m)) if thr > 0 else 0
        status = 1 if thr > 0 else 0
        if m == 2 and thr > 0:
            status = motor3_state(t)
            if status >= 2:
                rpm = round(rpm * (0.82 if status == 2 else 0.7))
        payload.extend(struct.pack(">HHhBB", max(0, rpm), round(s["motor_current"][m] / 0.01),
                                   round(s["motor_temp"][m] / 0.1), round(max(0, min(100, thr))), status))

    yaw_rate = 0.0
    if t >= 1:
        yaw_rate = ((k["heading"] - kinematics(t - 1)["heading"] + 180) % 360) - 180
    payload.extend(struct.pack(">6f", k["roll"], k["pitch"], k["heading"],
                               k["roll"] - kinematics(t - 1)["roll"], k["pitch"] - kinematics(t - 1)["pitch"],
                               yaw_rate))

    latitude = HOME_LAT + n / 111_320.0
    longitude = HOME_LON + e / (111_320.0 * math.cos(math.radians(HOME_LAT)))
    payload.extend(struct.pack(">4f", latitude, longitude, altitude, k["speed"]))

    flying = s["flying"]
    rail_voltages = (pack_voltage, 12.0 - (0.15 if flying else 0), 5.0 - (0.04 if flying else 0))
    propulsion = max(0.0, sum(s["motor_current"]))
    branch_currents = (propulsion, 1.8 if s["armed"] else 0.4, 0.9 if s["armed"] else 0.3, 0.6)
    payload.extend(struct.pack(">3H", *(round(v / 0.001) for v in rail_voltages)))
    payload.extend(struct.pack(">4H", *(round(min(v, 65.0) / 0.001) for v in branch_currents)))
    payload.extend(struct.pack(">4B", int(s["armed"]), 1, 1, 1))
    motor_fault = 1 if (s["throttle"] > 0 and motor3_state(t) == 3) else 0      # OVERCURRENT
    payload.extend(struct.pack(">4B", 0, motor_fault, 0, 0))

    vibration = []
    for m in range(4):
        base = 0.05 + (s["throttle"] / 100) * 0.42 if s["throttle"] > 0 else 0.02
        if m == 2 and s["throttle"] > 0 and motor3_state(t) >= 2:
            base += 0.6 if motor3_state(t) == 2 else 1.1
        vibration.extend(base * scale for scale in (1.0, 0.55, 0.28))
    payload.extend(struct.pack(">12H", *(round(v / 0.001) for v in vibration)))
    payload.extend(fixed_string(s["label"]))
    payload.extend(struct.pack(">HBf", 0, 0, CRUISE_ALT))
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
        packet.extend(extension)
        struct.pack_into(">H", packet, 4, len(packet) - 7)
        packets.append(packet)
    output.write_bytes(b"".join(packets))
    return len(packets), len(packets[0]) if packets else 0


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--output", type=Path, default=DEFAULT_INPUT)
    args = parser.parse_args()
    count, packet_size = generate(args.input, args.output)
    print(f"wrote {count} packets of {packet_size} bytes ({DRONE_EXTENSION_SIZE}-byte drone extension)")


if __name__ == "__main__":
    main()
