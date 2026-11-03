#!/usr/bin/env python3
# Receive DashKit discovery lines (UDP broadcast) and append them to a file.
# usage: discovery_rx.py [out.csv] [port]   -> ms,bus,id,data (0=chassis 1=vehicle)
# Lines can split across datagrams; a lost datagram corrupts at most its edges,
# so filter with: grep -E '^[0-9]+,[01],0x[0-9a-f]+,[0-9a-f]*$'
import socket, sys
out = sys.argv[1] if len(sys.argv) > 1 else 'dashkit_can.csv'
port = int(sys.argv[2]) if len(sys.argv) > 2 else 4242
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(('', port))
with open(out, 'ab', buffering=0) as f:
    try:
        while True:
            f.write(s.recv(2048))
    except KeyboardInterrupt:
        pass
