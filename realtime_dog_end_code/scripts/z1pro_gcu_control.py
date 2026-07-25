#!/usr/bin/env python3
import argparse
import socket
import struct
import sys
import time


HEADER_TX = b"\xA8\xE5"
HEADER_RX = b"\x8A\x5E"
VERSION = 0x02


def crc16_ccitt_nibble(data: bytes) -> int:
    table = [
        0x0000, 0x1021, 0x2042, 0x3063,
        0x4084, 0x50A5, 0x60C6, 0x70E7,
        0x8108, 0x9129, 0xA14A, 0xB16B,
        0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    ]
    crc = 0
    for byte in data:
        idx = (crc >> 12) ^ (byte >> 4)
        crc = ((crc << 4) & 0xFFFF) ^ table[idx & 0x0F]
        idx = (crc >> 12) ^ (byte & 0x0F)
        crc = ((crc << 4) & 0xFFFF) ^ table[idx & 0x0F]
    return crc & 0xFFFF


def s16_scaled_deg(value: float) -> int:
    scaled = int(round(value * 100.0))
    return max(-18000, min(18000, scaled))


def s16_scaled_rate(value: float) -> int:
    scaled = int(round(value * 10.0))
    return max(-1500, min(1500, scaled))


def build_packet(command: int, params: bytes = b"", roll: int = 0, pitch: int = 0, yaw: int = 0,
                 control_valid: bool = False, request_subframe: int = 0x01) -> bytes:
    main = bytearray(32)
    struct.pack_into("<hhh", main, 0, roll, pitch, yaw)
    if control_valid:
        main[6] |= 0x04
    main[25] = request_subframe & 0xFF

    sub = bytearray(32)
    payload = bytearray()
    total_len = 72 + len(params)
    payload += HEADER_TX
    payload += struct.pack("<H", total_len)
    payload += bytes([VERSION])
    payload += main
    payload += sub
    payload += bytes([command & 0xFF])
    payload += params
    crc = crc16_ccitt_nibble(bytes(payload))
    payload += bytes([(crc >> 8) & 0xFF, crc & 0xFF])
    return bytes(payload)


def parse_response(data: bytes) -> str:
    if not data:
        return "no response"
    head = data[:2]
    if head != HEADER_RX:
        return f"unexpected header {data[:16].hex(' ')}"
    length = struct.unpack_from("<H", data, 2)[0] if len(data) >= 4 else None
    version = data[4] if len(data) >= 5 else None
    command = data[69] if len(data) >= 70 else None
    status = data[70] if len(data) >= 71 else None
    mode = data[5] if len(data) >= 6 else None
    angles = None
    abs_angles = None
    rates = None
    if len(data) >= 18:
        rel_x, rel_y, rel_z = struct.unpack_from("<hhh", data, 12)
        angles = (rel_x / 100.0, rel_y / 100.0, rel_z / 100.0)
    if len(data) >= 24:
        abs_pitch = struct.unpack_from("<h", data, 20)[0] / 100.0
        abs_yaw = struct.unpack_from("<H", data, 22)[0] / 100.0
        abs_angles = (abs_pitch, abs_yaw)
    if len(data) >= 30:
        rate_x, rate_y, rate_z = struct.unpack_from("<hhh", data, 24)
        rates = (rate_x / 10.0, rate_y / 10.0, rate_z / 10.0)
    return (
        f"len={length} version={version} mode={mode} command={command} status={status} "
        f"rel_xyz_deg={angles} abs_pitch_yaw_deg={abs_angles} rate_xyz_dps={rates} "
        f"raw={data[:96].hex(' ')}"
    )


def send_tcp(host: str, port: int, packet: bytes, timeout: float) -> bytes:
    with socket.create_connection((host, port), timeout=timeout) as sock:
        sock.settimeout(timeout)
        sock.sendall(packet)
        try:
            return sock.recv(4096)
        except socket.timeout:
            return b""


def command_packet(args: argparse.Namespace) -> bytes:
    if args.command == "probe":
        return build_packet(0x00)
    if args.command == "home":
        return build_packet(0x03)
    if args.command == "angle":
        return build_packet(
            0x10,
            roll=s16_scaled_deg(args.roll),
            pitch=s16_scaled_deg(args.pitch),
            yaw=s16_scaled_deg(args.yaw),
            control_valid=True,
        )
    if args.command == "euler":
        return build_packet(
            0x14,
            roll=s16_scaled_deg(args.roll),
            pitch=s16_scaled_deg(args.pitch),
            yaw=s16_scaled_deg(args.yaw),
            control_valid=True,
        )
    if args.command == "rate":
        return build_packet(
            0x00,
            roll=s16_scaled_rate(args.roll),
            pitch=s16_scaled_rate(args.pitch),
            yaw=s16_scaled_rate(args.yaw),
            control_valid=True,
        )
    if args.command == "photo":
        return build_packet(0x20, b"\x01")
    if args.command == "record-toggle":
        return build_packet(0x21, b"\x01")
    if args.command == "focus":
        return build_packet(0x26, b"\x01")
    if args.command == "zoom-in":
        return build_packet(0x22, b"\x01")
    if args.command == "zoom-out":
        return build_packet(0x23, b"\x01")
    if args.command == "zoom-stop":
        return build_packet(0x24, b"\x01")
    if args.command == "zoom-set":
        zoom = int(round(args.zoom * 10.0))
        zoom = max(10, min(600, zoom))
        return build_packet(0x25, b"\x01" + struct.pack("<h", -zoom))
    if args.command == "track-exit":
        return build_packet(0x17, b"\x00\x01\x00\x00\x00\x00\x00\x00\x00\x00")
    raise ValueError(args.command)


def main() -> int:
    parser = argparse.ArgumentParser(description="Z-1Pro/Xianfei GCU private protocol client")
    parser.add_argument("command", choices=["probe", "home", "angle", "euler", "rate", "photo", "record-toggle", "focus", "zoom-in", "zoom-out", "zoom-stop", "zoom-set", "track-exit"])
    parser.add_argument("--host", default="192.168.144.108")
    parser.add_argument("--port", type=int, default=2332)
    parser.add_argument("--timeout", type=float, default=1.5)
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--interval", type=float, default=0.05)
    parser.add_argument("--roll", type=float, default=0.0)
    parser.add_argument("--pitch", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--zoom", type=float, default=1.0)
    parser.add_argument("--print-hex", action="store_true")
    args = parser.parse_args()

    packet = command_packet(args)
    if args.print_hex:
        print(packet.hex(" "))

    last_response = b""
    for index in range(max(1, args.repeat)):
        last_response = send_tcp(args.host, args.port, packet, args.timeout)
        print(f"send[{index + 1}] {args.command}: {parse_response(last_response)}")
        if index + 1 < args.repeat:
            time.sleep(args.interval)
    return 0


if __name__ == "__main__":
    sys.exit(main())