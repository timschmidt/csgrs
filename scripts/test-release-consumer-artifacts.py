"""Check release smoke selection against a reused Cargo target directory."""

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile


selector = Path(__file__).with_name("release-consumer-artifacts.py").resolve()
cargo = ["cargo", *filter(None, sys.argv[1:])]
with tempfile.TemporaryDirectory(prefix="release-smoke-selection.") as temporary:
    root = Path(temporary)
    manifest = root / "Cargo.toml"
    manifest.write_text('[package]\nname="smoke"\nversion="0.1.0"\nedition="2024"\n[workspace]\n')
    sources = root / "src/bin"
    sources.mkdir(parents=True)
    for name in ("quickstart_retired", "quickstart_current", "user_path"):
        (sources / (name + ".rs")).write_text(f'fn main() {{ println!("{name}"); }}\n')
    env = dict(os.environ, CARGO_TARGET_DIR=str(root / "target with spaces"), CARGO_NET_OFFLINE="true")
    build = cargo + ["build", "--bins", "--message-format=json-render-diagnostics"]
    first = subprocess.check_output(build, cwd=root, env=env)
    retired = next(Path(m["executable"]) for m in map(json.loads, first.splitlines())
                   if m["reason"] == "compiler-artifact" and m["target"]["name"] == "quickstart_retired")
    (sources / "quickstart_retired.rs").unlink()
    current = subprocess.check_output(build, cwd=root, env=env)
    assert retired.is_file()
    names = ["user_path", "quickstart_current"]
    select = [sys.executable, selector, manifest, *names]
    selected = subprocess.check_output(select, input=current).split(b"\0")
    assert selected.pop() == b""
    assert [subprocess.check_output([path], text=True).strip() for path in selected] == names

    # A stale executable and an identically named target from a different
    # manifest cannot substitute for a missing artifact of this consumer.
    foreign = next(m for m in map(json.loads, first.splitlines())
                   if m["reason"] == "compiler-artifact" and m["target"]["name"] == "quickstart_retired")
    foreign["manifest_path"] = str(root / "other/Cargo.toml")
    result = subprocess.run(select + ["quickstart_retired"], input=current + json.dumps(foreign).encode(),
                            capture_output=True)
    assert result.returncode != 0 and result.stdout == b""
    assert b"missing executable artifacts: quickstart_retired" in result.stderr
    print("release smoke selection passed with a stale binary, a warm build and a missing artifact")
