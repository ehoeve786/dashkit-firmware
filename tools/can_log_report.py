#!/usr/bin/env python3
"""Summarise a DashPilot CAN log (Settings > Developer > Record CAN log).

    python3 tools/can_log_report.py can-log-2026-11-15-101500.csv [more.csv ...]

The CSV has `ms,bus,id,data`. Rows with bus 0/1 are CAN frames; rows with bus
16/17 and an id of 0x1DA5xxxx are the firmware's diagnostic records (layouts in
main/can/can_diag.h). Prints, per file: the build, frame rates per bus, the
receive statistics over time, every door unlatch, and a PASS/FAIL line for the
things that must hold for vehicle controls (no fast-lane overflow, no anomalies).
"""
import csv
import struct
import sys
from collections import Counter

HELLO, RX, TX, ERR, LEGACY, MISROUTE, ROUTE, UNLATCH = (
    0x1DA50000, 0x1DA50001, 0x1DA50002, 0x1DA50003, 0x1DA50004, 0x1DA50005, 0x1DA50010, 0x1DA50020)
DOORS = ["front left", "front right", "rear left", "rear right"]


def u16x4(d):
    return struct.unpack("<4H", d)


def report(path):
    frames = {0: Counter(), 1: Counter()}
    diag = []
    last_ms = 0
    with open(path, newline="") as f:
        for row in csv.reader(f):
            if len(row) != 4 or row[0] == "ms":
                continue
            try:
                ms, bus, cid, data = int(row[0]), int(row[1]), int(row[2], 16), bytes.fromhex(row[3])
            except ValueError:
                continue
            last_ms = max(last_ms, ms)
            if bus & 0x10:
                if len(data) == 8:
                    diag.append((ms, bus & 0x0F, cid, data))
            elif bus in frames:
                frames[bus][cid] += 1

    secs = max(last_ms / 1000.0, 0.001)
    print(f"== {path}: {secs:.0f} s")
    for bus in (0, 1):
        n = sum(frames[bus].values())
        print(f"  bus {bus}: {n} frames logged ({n / secs:.0f}/s), {len(frames[bus])} ids")

    hello = [d for d in diag if d[2] == HELLO]
    stream = bool(hello) and hello[-1][3][1] == 1
    if not diag:
        print("  no diagnostic records (firmware without opcode 0x51?)")
        return True
    print(f"  firmware: {'fast lane + bulk lanes' if stream else 'single RX FIFO'}" if hello else "  no HELLO record")

    ok = True
    for bus in (0, 1):
        rx = [u16x4(d[3]) for d in diag if d[2] == RX and d[1] == bus]
        tx = [u16x4(d[3]) for d in diag if d[2] == TX and d[1] == bus]
        err = [u16x4(d[3]) for d in diag if d[2] == ERR and d[1] == bus]
        mis = [u16x4(d[3]) for d in diag if d[2] == MISROUTE and d[1] == bus]
        legacy = [struct.unpack("<2I", d[3]) for d in diag if d[2] == LEGACY and d[1] == bus]
        if rx:
            fast, bulk = [r[0] for r in rx], [r[1] for r in rx]
            fast_ovf = rx[-1][2] - rx[0][2] if len(rx) > 1 else rx[-1][2]
            print(f"  bus {bus} rx: fast {min(fast)}..{max(fast)} fps, bulk {min(bulk)}..{max(bulk)} fps, "
                  f"overflow events fast {rx[-1][2]} / bulk {rx[-1][3]} since boot")
            if rx[-1][2] != 0:
                ok = False
                print(f"    FAIL: fast lane overflowed ({rx[-1][2]} events since boot, {fast_ovf} in this log)")
        if tx:
            print(f"  bus {bus} tx: {tx[-1][0]} sent, {tx[-1][1]} refused (TX full), "
                  f"service step max {max(t[2] for t in tx)} us, busy gap max {max(t[3] for t in tx)} us")
        if err and any(err[-1]):
            ok = False
            print(f"    FAIL: anomalies on bus {bus}: head glitch {err[-1][0]}, layout fallback {err[-1][1]}, "
                  f"desync {err[-1][2]}, spi errors {err[-1][3]}")
        if mis and mis[-1][0]:
            print(f"    note: {mis[-1][0]} fast-lane frames arrived through a bulk lane on bus {bus}")
        if legacy:
            print(f"  bus {bus} rx overflow events: {legacy[-1][0]} since boot")

    route = [u16x4(d[3]) for d in diag if d[2] == ROUTE]
    if route:
        print(f"  route: seen {max(r[0] for r in route)} fps peak, firmware uses {max(r[1] for r in route)} fps, "
              f"app gets {max(r[2] for r in route)} fps, app-queue drops {route[-1][3]} since boot")

    unl = [(d[0], d[3]) for d in diag if d[2] == UNLATCH]
    print(f"  door unlatches: {len(unl)}")
    for ms, d in unl:
        door = DOORS[d[0] & 3]
        if d[0] & 0x80:
            print(f"    {ms / 1000:7.1f}s {door}: refused by the gear/speed gate")
            continue
        avg, mx = struct.unpack("<2H", d[4:8])
        print(f"    {ms / 1000:7.1f}s {door}: {d[1]} sent, {d[2]} refused (TX full), {d[3]} phase-locked, "
              f"car->ours avg {avg} us, max {mx} us")
        if d[1] == 0 or d[3] == 0:
            ok = False
            print("      FAIL: nothing sent or never phase-locked to the car's frame")
    print(f"  => {'PASS' if ok else 'FAIL'}")
    return ok


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    results = [report(p) for p in sys.argv[1:]]
    sys.exit(0 if all(results) else 1)
