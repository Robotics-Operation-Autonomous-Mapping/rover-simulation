#!/usr/bin/env python3
"""Compatibility entry point."""
from pathlib import Path
import runpy

if __name__ == "__main__":
    runpy.run_path(str(Path(__file__).resolve().parent / "control" / "drive_rover.py"), run_name="__main__")
