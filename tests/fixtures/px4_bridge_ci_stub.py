#!/usr/bin/env python3
"""Stub px4-command-bridge.py for unit-testing the JSON pipe protocol.

Protocol (matches the real px4-command-bridge.py):
  startup  → emit {"ok": true, "status": "ready"} on stdout
  per-line → read JSON command from stdin, emit JSON response on stdout
  shutdown → emit {"ok": true, "status": "shutdown"} and exit

Output uses compact separators (no spaces) so the C++ json_bool_value() parser,
which looks for "ok":true without whitespace, matches correctly.
"""
import json
import sys


def jout(obj):
    """Emit a compact JSON line (no spaces after separators) to stdout."""
    print(json.dumps(obj, separators=(',', ':')), flush=True)


# Ready signal — C++ reads this before sending any commands.
jout({"ok": True, "status": "ready"})

for raw_line in sys.stdin:
    line = raw_line.strip()
    if not line:
        continue
    try:
        req = json.loads(line)
    except json.JSONDecodeError:
        jout({"ok": False, "error": "invalid json"})
        continue

    cmd = req.get("command", "")
    if cmd == "shutdown":
        jout({"ok": True, "status": "shutdown"})
        break
    elif cmd in ("arm", "takeoff", "land", "disarm"):
        jout({"ok": True, "status": cmd})
    elif cmd == "velocity":
        # Echo clamped velocities so the test can verify bounds were applied.
        jout({"ok": True, "status": "velocity",
              "vx": req.get("vx", 0.0),
              "vy": req.get("vy", 0.0),
              "vz": req.get("vz", 0.0)})
    else:
        jout({"ok": False, "error": f"unknown command: {cmd}"})

