#!/usr/bin/env python3
"""Extend the original quickstart CCSDS recording with drone telemetry.

The drone part comes from drone_model, the same state machine and physics the
simulator runs live, stepped once per second without commands. The simulator
replaces this part on every packet it sends, so the recording mainly keeps the
archive and offline tools consistent with live behaviour.
"""

from __future__ import annotations

import argparse
import struct
from pathlib import Path

from drone_model import EXTENSION_SIZE, DroneModel

LEGACY_PACKET_SIZE = 123
CCSDS_HEADER_SIZE = 6
DEFAULT_INPUT = Path("testdata.ccsds")
DRONE_EXTENSION_SIZE = EXTENSION_SIZE


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
    drone = DroneModel()
    for packet in iter_legacy_packets(source.read_bytes()):
        drone.step(1.0)
        packet.extend(drone.pack())
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
