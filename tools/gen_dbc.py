#!/usr/bin/env python3
"""Generate main/can/dbc/dbc_generated.{c,h} from a Tesla DBC.

cantools is used as a PARSER ONLY. Its own `generate_c_source` emits per-message
structs and float encode/decode, which is the opposite of the string/handle table
API the firmware engine wants (and it can't do Tesla checksum/counter), so we
hand-roll the emitter here.

The messages listed in dbc/used_messages.txt are emitted, each with all of its
signals. As a check, every quoted string in the firmware sources (outside the
generated files) that is a message name in the DBC must be listed there: a
message used in code but missing from the list fails the build instead of
failing on the car. Messages are sorted by id (the RX path finds a frame's
message by binary search) and g_dbc_by_name indexes them by name.

Layout assumptions:
  * Every emitted signal is little-endian (Intel). For little-endian, cantools'
    `signal.start` IS the LSB DBC start bit, so we emit it verbatim. Messages
    with a big-endian signal are left out (the engine cannot decode them), as
    are extended-id and longer-than-8-byte messages (the value cache and the
    drivers' priority filters hold classic standard frames): listing one is
    an error.
  * Checksum signal (strict): name == <Msg>Checksum, 8 bits, little-endian.
    *CRC/*Crc and the 1-bit ESP_a* checks are intentionally NOT matched.
  * Counter signal: name == <Msg>Counter (any width; DI_*Counter is 3-bit).
"""

import argparse
import os
import re
import sys

try:
    import cantools
except ImportError:
    sys.stderr.write("error: cantools not installed (pip install -r requirements.txt)\n")
    sys.exit(1)


def signal_byte_order(sig):
    # cantools uses 'little_endian' / 'big_endian'
    return getattr(sig, "byte_order", "little_endian")


def detect_checksum(msg, sig):
    return (sig.name == msg.name + "Checksum"
            and sig.length == 8
            and signal_byte_order(sig) == "little_endian")


def detect_counter(msg, sig):
    return sig.name == msg.name + "Counter"


def c_float(x):
    # Compact but lossless-enough literal.
    return repr(float(x))


def used_names(src_dir, out_c, out_h):
    """Every quoted string in the firmware sources under src_dir."""
    skip = {os.path.abspath(out_c), os.path.abspath(out_h)}
    names = set()
    for root, _, files in os.walk(src_dir):
        for f in files:
            path = os.path.abspath(os.path.join(root, f))
            if f.endswith((".c", ".h")) and path not in skip:
                with open(path, encoding="utf-8", errors="ignore") as fh:
                    names.update(re.findall(r'"([A-Za-z][A-Za-z0-9_]*)"', fh.read()))
    return names


def load_list(path):
    """Message names from used_messages.txt: 'NAME BUS' per line, # comments."""
    names = []
    with open(path) as f:
        for lineno, raw in enumerate(f, 1):
            parts = raw.split("#", 1)[0].split()
            if not parts:
                continue
            if len(parts) != 2 or parts[1] not in ("0", "1"):
                sys.stderr.write("error: %s:%d: expected 'NAME BUS' (BUS 0 or 1), got %r\n"
                                 % (path, lineno, raw.strip()))
                sys.exit(1)
            names.append(parts[0])
    return names


def generate(dbc_path, list_path, src_dir, out_c, out_h):
    # strict=False: the full Tesla DBC has a few self-inconsistent messages
    # (e.g. UTC_unixTime); don't let them block the load.
    db = cantools.database.load_file(dbc_path, strict=False)
    listed = load_list(list_path)
    known = {m.name for m in db.messages}
    missing = sorted((used_names(src_dir, out_c, out_h) & known) - set(listed))
    unknown = sorted(set(listed) - known)
    if missing or unknown:
        if missing:
            sys.stderr.write("error: firmware code uses messages missing from %s: %s\n"
                             % (list_path, ", ".join(missing)))
        if unknown:
            sys.stderr.write("error: %s lists messages not in the DBC: %s\n"
                             % (list_path, ", ".join(unknown)))
        sys.exit(1)
    names = set(listed)

    def usable(m):
        return (not m.is_extended_frame and m.length <= 8
                and all(signal_byte_order(sig) == "little_endian" for sig in m.signals))

    used = [m for m in db.messages if m.name in names]
    bad = sorted(m.name for m in used if not usable(m))
    if bad:
        sys.stderr.write("error: the firmware names messages the engine cannot handle "
                         "(big-endian, extended or > 8 bytes): %s\n" % ", ".join(bad))
        sys.exit(1)
    messages = sorted(used, key=lambda m: m.frame_id)

    # First pass: flatten signals (sorted by start bit) and assign global indices.
    flat_signals = []          # list of (msg, sig)
    sig_index = {}             # (msg.name, sig.name) -> global index
    msg_meta = []              # per message: dict with sig_first/sig_count/etc.

    warnings = []

    for msg in messages:
        sigs = sorted(msg.signals, key=lambda s: s.start)
        sig_first = len(flat_signals)
        checksum_idx = -1
        counter_idx = -1
        checksum_algo = "DBC_CKSUM_NONE"

        for sig in sigs:
            gidx = len(flat_signals)
            sig_index[(msg.name, sig.name)] = gidx
            flat_signals.append((msg, sig))

            if detect_checksum(msg, sig):
                if checksum_idx != -1:
                    warnings.append("multiple checksum candidates in %s" % msg.name)
                checksum_idx = gidx
                checksum_algo = "DBC_CKSUM_TESLA_BYTESUM"
            elif sig.name.endswith("Checksum") or sig.name.endswith("CRC") \
                    or sig.name.endswith("Crc"):
                # Ambiguous near-miss: a *Checksum that is not 8-bit/LE, or a CRC.
                warnings.append("ignoring non-standard checksum-like signal %s.%s "
                                "(len=%d)" % (msg.name, sig.name, sig.length))

            if detect_counter(msg, sig):
                counter_idx = gidx

        msg_meta.append({
            "msg": msg,
            "sig_first": sig_first,
            "sig_count": len(sigs),
            "checksum_algo": checksum_algo,
            "checksum_sig": checksum_idx,
            "counter_sig": counter_idx,
        })

    # Second pass: resolve mux selector per signal.
    def mux_info(msg, sig):
        ids = getattr(sig, "multiplexer_ids", None)
        if not ids:
            return (-1, -1)
        # Find the message's multiplexer (selector) signal.
        sel = None
        for s in msg.signals:
            if getattr(s, "is_multiplexer", False):
                sel = s
                break
        if sel is None:
            warnings.append("muxed signal %s.%s has no selector" % (msg.name, sig.name))
            return (-1, -1)
        return (sig_index[(msg.name, sel.name)], int(ids[0]))

    for w in warnings:
        sys.stderr.write("warning: %s\n" % w)

    # --- Emit header ---
    h = []
    h.append("// AUTO-GENERATED by tools/gen_dbc.py -- DO NOT EDIT.")
    h.append("// source: %s" % os.path.basename(dbc_path))
    h.append("// Committed so a clean (Python-less) checkout still builds.")
    h.append("#pragma once")
    h.append("")
    h.append("#include <stdint.h>")
    h.append("")
    h.append("typedef enum {")
    h.append("    DBC_CKSUM_NONE = 0,")
    h.append("    DBC_CKSUM_TESLA_BYTESUM = 1,")
    h.append("} dbc_checksum_algo_t;")
    h.append("")
    h.append("typedef struct {")
    h.append("    const char *name;")
    h.append("    uint16_t start_bit;   // LSB position (little-endian / Intel)")
    h.append("    uint8_t  length;      // signal width in bits")
    h.append("    uint8_t  is_signed;   // 1 = two's complement")
    h.append("    double   scale;       // phys = raw * scale + offset")
    h.append("    double   offset;")
    h.append("    int16_t  mux_sel_sig; // g_dbc_signals index of selector, or -1")
    h.append("    int32_t  mux_val;     // selector value enabling this signal, or -1")
    h.append("} dbc_signal_t;")
    h.append("")
    h.append("typedef struct {")
    h.append("    const char *name;")
    h.append("    uint32_t id;")
    h.append("    uint8_t  dlc;")
    h.append("    uint16_t sig_first;   // index of first signal in g_dbc_signals")
    h.append("    uint16_t sig_count;")
    h.append("    uint8_t  checksum_algo;")
    h.append("    int16_t  checksum_sig; // g_dbc_signals index, or -1")
    h.append("    int16_t  counter_sig;  // g_dbc_signals index, or -1")
    h.append("} dbc_message_t;")
    h.append("")
    h.append("#define DBC_MESSAGE_COUNT %d" % len(messages))
    h.append("#define DBC_SIGNAL_COUNT  %d" % len(flat_signals))
    h.append("")
    h.append("extern const dbc_message_t g_dbc_messages[DBC_MESSAGE_COUNT];")
    h.append("extern const dbc_signal_t  g_dbc_signals[DBC_SIGNAL_COUNT];")
    h.append("")
    h.append("// g_dbc_messages is sorted by id; g_dbc_by_name lists its indices in name")
    h.append("// (strcmp) order.")
    h.append("extern const uint16_t g_dbc_by_name[DBC_MESSAGE_COUNT];")
    h.append("")

    # --- Emit source ---
    c = []
    c.append("// AUTO-GENERATED by tools/gen_dbc.py -- DO NOT EDIT.")
    c.append('#include "dbc_generated.h"')
    c.append("")
    c.append("const dbc_signal_t g_dbc_signals[DBC_SIGNAL_COUNT] = {")
    cur_msg = None
    for (msg, sig) in flat_signals:
        if msg.name != cur_msg:
            c.append("    // ---- %s (0x%03X) ----" % (msg.name, msg.frame_id))
            cur_msg = msg.name
        mux_sel, mux_val = mux_info(msg, sig)
        c.append('    { "%s", %d, %d, %d, %s, %s, %d, %d },' % (
            sig.name, sig.start, sig.length,
            1 if sig.is_signed else 0,
            c_float(sig.scale), c_float(sig.offset),
            mux_sel, mux_val))
    c.append("};")
    c.append("")
    c.append("const dbc_message_t g_dbc_messages[DBC_MESSAGE_COUNT] = {")
    for m in msg_meta:
        msg = m["msg"]
        c.append('    { "%s", 0x%03X, %d, %d, %d, %s, %d, %d },' % (
            msg.name, msg.frame_id, msg.length,
            m["sig_first"], m["sig_count"],
            m["checksum_algo"], m["checksum_sig"], m["counter_sig"]))
    c.append("};")
    c.append("")

    by_name = sorted(range(len(messages)), key=lambda i: messages[i].name.encode())
    c.append("const uint16_t g_dbc_by_name[DBC_MESSAGE_COUNT] = {")
    for i in range(0, len(by_name), 16):
        c.append("    " + ", ".join("%d" % v for v in by_name[i:i + 16]) + ",")
    c.append("};")
    c.append("")

    os.makedirs(os.path.dirname(out_h), exist_ok=True)
    with open(out_h, "w") as f:
        f.write("\n".join(h))
    with open(out_c, "w") as f:
        f.write("\n".join(c))

    sys.stderr.write("gen_dbc: %d messages, %d signals -> %s\n"
                     % (len(messages), len(flat_signals), out_c))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dbc", required=True)
    ap.add_argument("--messages", required=True, help="dbc/used_messages.txt")
    ap.add_argument("--sources", required=True, help="firmware sources, checked against the list")
    ap.add_argument("--out-c", required=True)
    ap.add_argument("--out-h", required=True)
    args = ap.parse_args()
    generate(args.dbc, args.messages, args.sources, args.out_c, args.out_h)


if __name__ == "__main__":
    main()
