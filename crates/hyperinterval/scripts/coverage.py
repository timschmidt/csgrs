#!/usr/bin/env python3
"""Compatibility entry point for the Hyperinterval native coverage gate."""

from pathlib import Path
import runpy


if __name__ == "__main__":
    runpy.run_path(str(Path(__file__).resolve().parents[3] / "scripts/interval_coverage.py"),
                   run_name="__main__")
