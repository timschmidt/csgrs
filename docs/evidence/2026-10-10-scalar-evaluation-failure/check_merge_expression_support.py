"""Check the original min/or expressions with the pinned verifier, without source edits."""
import hashlib
import json
from pathlib import Path
import subprocess
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    out = E / "merge-expression-support-v1917"
    assert not out.exists()
    out.mkdir()
    source = out / "probe.rs"
    source.write_text('''use vstd::prelude::*;

verus! {
fn original_sign_selection<T>(retained: Option<T>, incoming: Option<T>) -> (result: Option<T>)
    ensures result == match retained { Some(_) => retained, None => incoming },
{
    retained.or(incoming)
}

fn original_magnitude_selection(retained: i32, incoming: i32) -> (result: i32)
    ensures result == if retained < incoming { retained } else { incoming },
{
    retained.min(incoming)
}
}

fn main() {}
''')
    tool = ROOT / "crates/hyperreal/target/verus/0.2026.09.13.671956e/verus-x86-linux/verus"
    tool_before = sha(tool)
    command = [str(tool), "--no-cheating", str(source)]
    start = time.monotonic()
    result = subprocess.run(command, cwd=out, text=True, capture_output=True)
    (out / "stdout.log").write_text(result.stdout)
    (out / "stderr.log").write_text(result.stderr)
    (out / "runner.py").write_bytes(Path(__file__).read_bytes())
    assert sha(tool) == tool_before
    record = {"command": command, "exit_code": result.returncode, "seconds": time.monotonic() - start,
        "source_sha256": sha(source), "verifier_sha256": tool_before, "verifier_unchanged": True,
        "scope": "Exploratory verification of the original generic Option::or and signed i32::min selections with the pinned vstd contracts. This is not the production merge proof, runtime validation, or acceptance of either rejected variant.",
        "artifacts_sha256": {path.name: sha(path) for path in sorted(out.iterdir()) if path.is_file()}}
    (out / "result.json").write_text(json.dumps(record, indent=2) + "\n")
    print(json.dumps({"exit_code": result.returncode, "stdout": result.stdout, "stderr": result.stderr}))


if __name__ == "__main__":
    main()
