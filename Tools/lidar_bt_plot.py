#!/usr/bin/env python3
"""Realtime plotter for the car Bluetooth RPLIDAR point stream.

Install once:
    python -m pip install pyserial matplotlib

Run:
    python Tools/lidar_bt_plot.py

Firmware frame format:
    A5 5A C1 count seq16_le drops32_le points... checksum

Each point is 6 bytes:
    angle_cdeg16_le, distance_mm16_le, quality8, flags8

The checksum is XOR over bytes from frame type through the last payload byte.
"""

from __future__ import annotations

import argparse
import math
import struct
import time
from dataclasses import dataclass
from typing import Iterable

import matplotlib.pyplot as plt
import serial
from matplotlib.animation import FuncAnimation


SYNC = b"\xA5\x5A"
FRAME_TYPE_POINTS = 0xC1
HEADER_BYTES = 10
POINT_BYTES = 6
MAX_POINTS_PER_FRAME = 16
FLAG_SCAN_START = 0x01


@dataclass
class LidarPoint:
    angle_cdeg: int
    distance_mm: int
    quality: int
    flags: int


@dataclass
class LidarFrame:
    sequence: int
    drops: int
    points: list[LidarPoint]


def checksum(frame_without_checksum: bytes) -> int:
    value = 0
    for byte in frame_without_checksum[2:]:
        value ^= byte
    return value


def parse_frames(buffer: bytearray) -> Iterable[LidarFrame]:
    while True:
        sync_index = buffer.find(SYNC)
        if sync_index < 0:
            del buffer[:-1]
            return

        if sync_index > 0:
            del buffer[:sync_index]

        if len(buffer) < HEADER_BYTES:
            return

        if buffer[2] != FRAME_TYPE_POINTS:
            del buffer[0]
            continue

        count = buffer[3]
        if count == 0 or count > MAX_POINTS_PER_FRAME:
            del buffer[0]
            continue

        frame_len = HEADER_BYTES + count * POINT_BYTES + 1
        if len(buffer) < frame_len:
            return

        packet = bytes(buffer[:frame_len])
        del buffer[:frame_len]

        if checksum(packet[:-1]) != packet[-1]:
            continue

        sequence = packet[4] | (packet[5] << 8)
        drops = struct.unpack_from("<I", packet, 6)[0]
        points: list[LidarPoint] = []

        offset = HEADER_BYTES
        for _ in range(count):
            angle_cdeg, distance_mm, quality, flags = struct.unpack_from("<HHBB", packet, offset)
            points.append(LidarPoint(angle_cdeg, distance_mm, quality, flags))
            offset += POINT_BYTES

        yield LidarFrame(sequence, drops, points)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Plot RPLIDAR C1 points forwarded over Bluetooth.")
    parser.add_argument("--port", default="COM4", help="Serial port, default: COM4.")
    parser.add_argument("--baud", type=int, default=115200, help="Bluetooth UART baud rate.")
    parser.add_argument("--max-range", type=int, default=6000, help="Polar plot radius in millimeters.")
    parser.add_argument("--quality-min", type=int, default=0, help="Ignore points below this quality.")
    parser.add_argument("--keep", type=int, default=900, help="Fallback rolling point count before scan-start is seen.")
    parser.add_argument("--interval", type=int, default=40, help="Plot refresh interval in milliseconds.")
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()
    ser = serial.Serial(args.port, args.baud, timeout=0)

    rx_buffer = bytearray()
    current_scan: list[LidarPoint] = []
    last_scan: list[LidarPoint] = []
    last_status_time = time.monotonic()
    last_sequence = None
    last_drops = 0
    parsed_frames = 0
    parsed_points = 0

    fig = plt.figure("RPLIDAR C1 Bluetooth Plot")
    ax = fig.add_subplot(111, projection="polar")
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)
    ax.set_rlim(0, args.max_range)
    ax.grid(True)
    scatter = ax.scatter([], [], s=5, c=[], cmap="viridis", vmin=0, vmax=63)
    title = ax.set_title("waiting for lidar frames")

    def ingest() -> None:
        nonlocal current_scan, last_scan, last_status_time, last_sequence
        nonlocal last_drops, parsed_frames, parsed_points

        data = ser.read(8192)
        if data:
            rx_buffer.extend(data)

        for frame in parse_frames(rx_buffer):
            parsed_frames += 1
            parsed_points += len(frame.points)
            last_sequence = frame.sequence
            last_drops = frame.drops

            for point in frame.points:
                if point.flags & FLAG_SCAN_START:
                    if current_scan:
                        last_scan = current_scan
                    current_scan = []

                if point.distance_mm > 0 and point.quality >= args.quality_min:
                    current_scan.append(point)

            if not last_scan and len(current_scan) > args.keep:
                current_scan = current_scan[-args.keep :]

        now = time.monotonic()
        if now - last_status_time >= 1.0:
            print(
                f"frames={parsed_frames} points={parsed_points} "
                f"seq={last_sequence} firmware_drops={last_drops} buffered={len(rx_buffer)}"
            )
            last_status_time = now

    def update(_frame_index: int):
        ingest()
        points = current_scan or last_scan

        if points:
            theta = [math.radians(point.angle_cdeg / 100.0) for point in points]
            radius = [point.distance_mm for point in points]
            quality = [point.quality for point in points]
        else:
            theta = []
            radius = []
            quality = []

        scatter.set_offsets(list(zip(theta, radius)))
        scatter.set_array(quality)
        title.set_text(
            f"points={len(points)} seq={last_sequence} firmware_drops={last_drops}"
        )
        return scatter, title

    print(f"Listening on {args.port} @ {args.baud}. Close the plot window to stop.")
    FuncAnimation(fig, update, interval=args.interval, blit=False, cache_frame_data=False)
    plt.show()
    ser.close()


if __name__ == "__main__":
    main()
