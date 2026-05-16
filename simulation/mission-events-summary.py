#!/usr/bin/env python3
"""Summarize Dedalus mission_events.jsonl artifacts.

This helper intentionally stays lightweight: mission_events.jsonl is line-delimited JSON,
so we can inspect live or completed run artifacts without importing Dedalus C++ bindings.
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import OrderedDict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


@dataclass
class CommandCounts:
    ok: int = 0
    failed: int = 0


@dataclass
class MissionSummary:
    valid: bool = False
    event_count: int = 0
    tick_count: int = 0
    failure_count: int = 0
    final_state: str = "unknown"
    state_path: list[str] = field(default_factory=list)
    commands: "OrderedDict[str, CommandCounts]" = field(default_factory=OrderedDict)
    failures: list[str] = field(default_factory=list)


def append_state_if_new(states: list[str], state: str | None) -> None:
    if not state:
        return
    if not states or states[-1] != state:
        states.append(state)


def command_counts(summary: MissionSummary, command: str) -> CommandCounts:
    if command not in summary.commands:
        summary.commands[command] = CommandCounts()
    return summary.commands[command]


def read_summary(path: Path) -> MissionSummary:
    summary = MissionSummary()
    with path.open("r", encoding="utf-8") as input_file:
        for line_number, raw_line in enumerate(input_file, start=1):
            line = raw_line.strip()
            if not line:
                continue
            try:
                event: dict[str, Any] = json.loads(line)
            except json.JSONDecodeError as exc:
                summary.valid = True
                summary.event_count += 1
                summary.failure_count += 1
                summary.failures.append(f"line {line_number}: invalid JSON: {exc}")
                continue

            summary.valid = True
            summary.event_count += 1
            event_name = event.get("event")
            if isinstance(event.get("tick"), int):
                summary.tick_count = max(summary.tick_count, int(event["tick"]))

            if event_name == "state_transition":
                append_state_if_new(summary.state_path, event.get("from"))
                to_state = event.get("to")
                append_state_if_new(summary.state_path, to_state)
                if to_state:
                    summary.final_state = str(to_state)
            elif event_name == "command_result":
                command = str(event.get("command", ""))
                if command:
                    counts = command_counts(summary, command)
                    if bool(event.get("success")):
                        counts.ok += 1
                    else:
                        counts.failed += 1
                        summary.failure_count += 1
                        summary.failures.append(f"{command}: {event.get('status', '')}")
            elif event_name == "command_exception":
                command = str(event.get("command", "command"))
                summary.failure_count += 1
                summary.failures.append(f"{command}: {event.get('error', '')}")
            elif event_name == "runtime_stop":
                state = event.get("state")
                if state:
                    summary.final_state = str(state)
                    append_state_if_new(summary.state_path, str(state))
                if isinstance(event.get("tick_count"), int):
                    summary.tick_count = max(summary.tick_count, int(event["tick_count"]))

    return summary


def print_summary(summary: MissionSummary) -> None:
    if not summary.valid:
        print("Mission summary: unavailable; no event records found")
        return

    print("Mission summary:")
    print(f"  final_state: {summary.final_state}")
    print(f"  ticks: {summary.tick_count}")
    print(f"  events: {summary.event_count}")
    if summary.state_path:
        print(f"  state_path: {' -> '.join(summary.state_path)}")
    if summary.commands:
        print("  commands:")
        for command, counts in summary.commands.items():
            print(f"    {command}: ok={counts.ok} failed={counts.failed}")
    print(f"  failures: {summary.failure_count}")
    for failure in summary.failures:
        print(f"    - {failure}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("events_path", type=Path, help="Path to mission_events.jsonl")
    parser.add_argument("--expect-complete", action="store_true", help="Fail unless final_state is Complete and failures is 0")
    args = parser.parse_args()

    if not args.events_path.exists():
        print(f"mission events not found: {args.events_path}", file=sys.stderr)
        return 2

    summary = read_summary(args.events_path)
    print_summary(summary)

    if args.expect_complete and (summary.final_state != "Complete" or summary.failure_count != 0):
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
