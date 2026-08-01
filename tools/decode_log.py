#!/usr/bin/env python3
"""
decode_log.py -- decodes CANTX hex-dump lines produced by canSend()
(src/can_bus.cpp), and plots pedal position / drive state / torque over time.

Deliberately a separate, independent reimplementation of the message
dictionary in include/can_dictionary.h, not a shared library -- a real CAN
log analysis tool never gets to link against firmware headers.

Usage:
    python decode_log.py capture.txt          # decode a saved log file
    python decode_log.py --port COM5          # decode live from serial

To capture a log file:
    pio device monitor > capture.txt
"""

import struct
import argparse


CAN_ID_DIM_PEDAL     = 0x100
CAN_ID_DIM_IMU       = 0x101
CAN_ID_DIM_HEARTBEAT = 0x102
CAN_ID_VCU_STATUS    = 0x200
CAN_ID_VCU_FAULT     = 0x201

OFF_COUNTER  = 6
OFF_CHECKSUM = 7

OFF_PEDAL_POT_A = 0
OFF_PEDAL_POT_B = 2
OFF_PEDAL_FLAGS = 4

OFF_IMU_ACCEL_X = 0
OFF_IMU_ACCEL_Y = 2
OFF_IMU_ACCEL_Z = 4

OFF_HEARTBEAT_UPTIME_MS = 0
OFF_HEARTBEAT_DIM_STATE = 4

OFF_STATUS_DRIVE_STATE   = 0
OFF_STATUS_TORQUE_PCT    = 1
OFF_STATUS_ACTIVE_FAULTS = 2
OFF_STATUS_PEDAL_PCT     = 4

OFF_FAULT_CODE           = 0
OFF_FAULT_FLAGS          = 1
OFF_FAULT_LATCHED_FAULTS = 2
OFF_FAULT_NVS_WRITES     = 4


def verify_checksum(data: bytes) -> bool:
    return data[OFF_CHECKSUM] == (sum(data[0:7]) & 0xFF)


def decode_pedal(data: bytes) -> dict:
    pot_a = struct.unpack('<H', data[OFF_PEDAL_POT_A:OFF_PEDAL_POT_A + 2])[0]
    pot_b = struct.unpack('<H', data[OFF_PEDAL_POT_B:OFF_PEDAL_POT_B + 2])[0]
    return {
        "pot_a_raw": pot_a,
        "pot_b_raw": pot_b,
        "button_flags": data[OFF_PEDAL_FLAGS],
    }


def decode_imu(data: bytes) -> dict:
    # '<h' lowercase = signed 16-bit; acceleration can be negative.
    ax = struct.unpack('<h', data[OFF_IMU_ACCEL_X:OFF_IMU_ACCEL_X + 2])[0]
    ay = struct.unpack('<h', data[OFF_IMU_ACCEL_Y:OFF_IMU_ACCEL_Y + 2])[0]
    az = struct.unpack('<h', data[OFF_IMU_ACCEL_Z:OFF_IMU_ACCEL_Z + 2])[0]
    return {
        "accel_x_g": ax / 1000.0,
        "accel_y_g": ay / 1000.0,
        "accel_z_g": az / 1000.0,
    }


def decode_heartbeat(data: bytes) -> dict:
    uptime = struct.unpack(
        '<I', data[OFF_HEARTBEAT_UPTIME_MS:OFF_HEARTBEAT_UPTIME_MS + 4]
    )[0]
    return {
        "uptime_ms": uptime,
        "dim_state": data[OFF_HEARTBEAT_DIM_STATE],
    }


def decode_status(data: bytes) -> dict:
    active_faults = struct.unpack(
        '<H', data[OFF_STATUS_ACTIVE_FAULTS:OFF_STATUS_ACTIVE_FAULTS + 2]
    )[0]
    return {
        "drive_state": data[OFF_STATUS_DRIVE_STATE],
        "torque_pct": data[OFF_STATUS_TORQUE_PCT],
        "active_faults": active_faults,
        "pedal_pct": data[OFF_STATUS_PEDAL_PCT],
    }


def decode_fault(data: bytes) -> dict:
    latched_faults = struct.unpack(
        '<H', data[OFF_FAULT_LATCHED_FAULTS:OFF_FAULT_LATCHED_FAULTS + 2]
    )[0]
    return {
        "fault_code": data[OFF_FAULT_CODE],
        "flags": data[OFF_FAULT_FLAGS],
        "latched_faults": latched_faults,
        "nvs_write_count": data[OFF_FAULT_NVS_WRITES],
    }


DECODERS = {
    CAN_ID_DIM_PEDAL: decode_pedal,
    CAN_ID_DIM_IMU: decode_imu,
    CAN_ID_DIM_HEARTBEAT: decode_heartbeat,
    CAN_ID_VCU_STATUS: decode_status,
    CAN_ID_VCU_FAULT: decode_fault,
}

DRIVE_STATE_NAMES = {0: "BOOT", 1: "IDLE", 2: "READY", 3: "DRIVE", 4: "FAULT"}

FAULT_CODE_NAMES = {
    0: "NONE", 1: "PEDAL_IMPLAUSIBLE", 2: "IMU_IMPLAUSIBLE",
    3: "PEDAL_TIMEOUT", 4: "IMU_TIMEOUT", 5: "DIM_TIMEOUT",
}

FAULT_BIT_NAMES = [
    (0x01, "PEDAL_IMPLAUSIBLE"),
    (0x02, "IMU_IMPLAUSIBLE"),
    (0x04, "PEDAL_TIMEOUT"),
    (0x08, "IMU_TIMEOUT"),
    (0x10, "DIM_TIMEOUT"),
]


def format_faults(bitmask: int) -> str:
    names = [name for bit, name in FAULT_BIT_NAMES if bitmask & bit]
    return "|".join(names) if names else "none"


def parse_line(line: str):
    """
    Parse one line: "CANTX,<millis>,<id_hex>,<ok>,<b0>,...,<b7>".
    Returns None (not an exception) for anything that doesn't match --
    a live/captured log has other lines mixed in (boot banners, torn
    lines from a mid-write disconnect), and skipping those is correct.
    """
    parts = line.strip().split(',')

    if len(parts) < 4 or parts[0] != "CANTX":
        return None

    try:
        t_ms = int(parts[1])
        can_id = int(parts[2], 16)
        ok = parts[3] == "1"
        data_bytes = bytes(int(b, 16) for b in parts[4:])
    except (ValueError, IndexError):
        return None

    if len(data_bytes) != 8:
        return None

    return {"t_ms": t_ms, "id": can_id, "ok": ok, "data": data_bytes}


def _iter_lines(args):
    """Yields raw text lines from either a file or a live serial port."""
    if args.port:
        import serial  # pip install pyserial
        with serial.Serial(args.port, args.baud, timeout=1) as ser:
            print(f"Reading live from {args.port} @ {args.baud} -- Ctrl+C to stop and plot")
            while True:
                raw = ser.readline()
                if raw:
                    yield raw.decode(errors="replace")
    else:
        with open(args.source, "r", errors="replace") as f:
            for raw in f:
                yield raw


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", nargs="?",
                         help="path to a captured log file (omit if using --port)")
    parser.add_argument("--port", help="serial port to read live instead of a file, e.g. COM5")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--live", action="store_true",
                         help="print each decoded VCU_STATUS/VCU_FAULT frame as it arrives")
    parser.add_argument("--live-interval-ms", type=int, default=300,
                         help="minimum time between printed --live status lines (VCU_STATUS is 20Hz, too fast to read raw)")
    args = parser.parse_args()

    if not args.source and not args.port:
        parser.error("provide either a log file path or --port")

    times = []
    pedal_pcts = []
    drive_states = []
    torque_pcts = []
    last_live_print_ms = None

    try:
        for raw_line in _iter_lines(args):
            frame = parse_line(raw_line)
            if frame is None:
                continue
            if not verify_checksum(frame["data"]):
                continue

            if frame["id"] == CAN_ID_VCU_FAULT:
                if args.live:
                    fault = decode_fault(frame["data"])
                    code_name = FAULT_CODE_NAMES.get(fault["fault_code"], "?")
                    latched = format_faults(fault["latched_faults"])
                    print(f"t={frame['t_ms']:>8} FAULT   code={code_name:<18} latched={latched}")
                continue

            if frame["id"] != CAN_ID_VCU_STATUS:
                continue

            status = decode_status(frame["data"])
            times.append(frame["t_ms"])
            pedal_pcts.append(status["pedal_pct"])
            drive_states.append(status["drive_state"])
            torque_pcts.append(status["torque_pct"])

            if args.live and (last_live_print_ms is None
                              or frame["t_ms"] - last_live_print_ms >= args.live_interval_ms):
                last_live_print_ms = frame["t_ms"]
                state_name = DRIVE_STATE_NAMES.get(status["drive_state"], "?")
                active = format_faults(status["active_faults"])
                print(f"t={frame['t_ms']:>8} state={state_name:<6} "
                      f"pedal={status['pedal_pct']:>3}% torque={status['torque_pct']:>3}% "
                      f"faults={active}")
    except KeyboardInterrupt:
        print("\nStopped -- plotting what was captured so far.")

    if not times:
        print("No VCU_STATUS frames decoded -- nothing to plot.")
        return

    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 6))
    axes[0].plot(times, pedal_pcts)
    axes[0].set_ylabel("pedal %")
    axes[1].step(times, drive_states, where="post")
    axes[1].set_ylabel("drive state")
    axes[2].plot(times, torque_pcts)
    axes[2].set_ylabel("torque %")
    axes[-1].set_xlabel("time (ms)")
    fig.suptitle("VCU_STATUS over time")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
