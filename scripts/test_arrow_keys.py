#!/usr/bin/env python3
"""Diagnose arrow-key escape sequences from the interactive terminal.

Usage (must run in a real terminal, not via piped stdin):
    uv run python scripts/test_arrow_keys.py

Note: ``python - <<'PY'`` fails with termios ENOTTY because heredoc redirects stdin
away from the TTY. This script opens /dev/tty directly instead.
"""

from __future__ import annotations

import re
import select
import sys
import termios
import tty

_CSI_ARROW_RE = re.compile(r"\x1b\[[0-9;]*[ ]*([CDcd])")
_SS3_ARROW_RE = re.compile(r"\x1bO([CDcd])")


def main() -> int:
    try:
        tty_fd = open("/dev/tty", "r+b", buffering=0)  # noqa: SIM115
    except OSError as exc:
        print(f"Cannot open /dev/tty: {exc}", file=sys.stderr)
        print("Run this from an interactive terminal (ssh -t, local shell, etc.).", file=sys.stderr)
        return 1

    old_term = termios.tcgetattr(tty_fd)
    pending = ""
    print("Press arrow keys. Left/right should print DETECTED lines. Press q to quit.")
    try:
        tty.setcbreak(tty_fd.fileno())
        attr = termios.tcgetattr(tty_fd)
        attr[3] &= ~termios.ECHO
        termios.tcsetattr(tty_fd, termios.TCSADRAIN, attr)
        termios.tcflush(tty_fd, termios.TCIFLUSH)

        while True:
            ready, _, _ = select.select([tty_fd], [], [], 0.1)
            if ready:
                pending += tty_fd.read(1).decode("latin-1", errors="replace")

            match = _CSI_ARROW_RE.search(pending) or _SS3_ARROW_RE.search(pending)
            if match:
                seq = pending[match.start() : match.end()]
                final = match.group(1)
                direction = "RIGHT (save)" if final in "Cc" else "LEFT (re-record)"
                print(f"DETECTED: {seq!r} -> {final!r}  [{direction}]")
                pending = pending[: match.start()] + pending[match.end() :]

            if "q" in pending or "Q" in pending:
                break
    finally:
        termios.tcsetattr(tty_fd, termios.TCSADRAIN, old_term)
        tty_fd.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
