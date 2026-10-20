#!/usr/bin/env python3
"""Compile/run the README in a fresh workspace; reject foreign direct backends.

This checks local source consumption. The crate is unpublished, so this does
not claim crates.io packaging or release validation. The copied workspace lock
pins available versions; Cargo removes unrelated packages in the temporary copy.
"""

import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

crate = Path(__file__).resolve().parents[1]
root = crate.parents[1]
metadata = json.loads(subprocess.check_output(
    ["cargo", "metadata", "--locked", "--no-deps", "--format-version", "1"],
    cwd=root, text=True,
))
packages = {p["name"]: p for p in metadata["packages"]}
for dependency in packages["hyperspatial"]["dependencies"]:
    if dependency["kind"] == "dev":
        continue
    package = packages.get(dependency["name"])
    if (package is None or not dependency["name"].startswith("hyper")
            or dependency["source"] is not None or not dependency.get("path")):
        raise SystemExit(f"Hyperspatial runtime dependency must be native: {dependency['name']}")
    if Path(package["manifest_path"]).parent != Path(dependency["path"]):
        raise SystemExit(f"Runtime dependency does not resolve to workspace source: {dependency['name']}")

blocks = re.findall(r"```rust\n(.*?)```", (crate / "README.md").read_text(), re.S)
if not blocks:
    raise SystemExit("README has no Rust consumer examples")
with tempfile.TemporaryDirectory(prefix="hyperspatial-consumer-") as temporary:
    consumer = Path(temporary)
    (consumer / "src/bin").mkdir(parents=True)
    (consumer / "Cargo.toml").write_text(
        '[package]\nname = "hyperspatial-source-consumer"\nversion = "0.0.0"\n'
        'edition = "2024"\n[workspace]\n[dependencies]\n'
        f'hyperspatial = {{ path = {json.dumps(str(crate))} }}\n'
        f'hyperlattice = {{ path = {json.dumps(str(root / "crates/hyperlattice"))} }}\n'
    )
    shutil.copyfile(root / "Cargo.lock", consumer / "Cargo.lock")
    environment = os.environ.copy()
    environment.setdefault("CARGO_TARGET_DIR", str(root / "target/hyperspatial-consumer"))
    for index, block in enumerate(blocks):
        body = "\n".join(line[2:] if line.startswith("# ") else line for line in block.splitlines())
        (consumer / f"src/bin/readme_{index}.rs").write_text(
            "fn main() -> Result<(), hyperspatial::Error> {\n" + body + "\n}\n"
        )
        subprocess.run(
            ["cargo", "run", "--offline", "--manifest-path", str(consumer / "Cargo.toml"),
             "--bin", f"readme_{index}"], env=environment, check=True,
        )
print(f"Native dependency boundary and {len(blocks)} independent README consumer(s) passed")
