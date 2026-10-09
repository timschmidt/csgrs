"""Restore the committed fixture omitted by inherited temporary snapshots."""

import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent
record = json.loads((ROOT / "manifest.json").read_text())
raw = (ROOT / "native-circle-via.kicad_pcb").read_bytes()
for name, expected in record["inputs"].items():
    assert len(raw) == expected["bytes"]
    assert hashlib.sha256(raw).hexdigest() == expected["sha256"]
    destination = Path(name)
    assert destination.parent.parent.parent.parent.parent.is_dir()
    if destination.exists():
        assert destination.read_bytes() == raw
    else:
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(raw)
print("Restored or verified the unchanged committed fixture in all three source locations.")
