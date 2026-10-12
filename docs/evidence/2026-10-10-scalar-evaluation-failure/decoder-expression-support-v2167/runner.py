"""Check decoder expression support independently, without changing production."""
import gzip
import hashlib
import json
from pathlib import Path
import subprocess
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
read = lambda p: json.loads(p.read_text())

PROBES = {
    "optional_magnitude": '''use vstd::prelude::*;

verus! {
fn original_optional_magnitude(value: u64) -> (result: Option<i32>)
    ensures result == if value & 16u64 != 0 {
        Some(((value >> 32u32) as u32) as i32)
    } else { None },
{
    (value & 16u64 != 0).then_some(((value >> 32u32) as u32) as i32)
}
}

fn main() {}
''',
    "masked_tag": '''use vstd::prelude::*;

verus! {
fn masked_tag(value: u64) -> (result: u64)
    ensures result == value & 3u64,
{
    assert(value & 3u64 <= 3u64) by (bit_vector);
    match value & 3u64 {
        0 => 0,
        1 => 1,
        2 => 2,
        3 => 3,
        _ => unreachable!(),
    }
}
}

fn main() {}
''',
}


def main():
    out = E / "decoder-expression-support-v2167"
    assert not out.exists()
    restoration = read(E / "scalar-retained-restoration-validation-v2165.json")
    assert restoration["complete"] and len(restoration["phases"]) == 6
    assert all(row["result"]["exit_code"] == 0 for row in restoration["phases"])
    protocol = read(E / "scalar-retained-restoration-v2159/protocol.json")
    record = E / protocol["retained_source_record"]
    assert sha(record) == restoration["source_record_sha256"] == protocol["retained_source_record_sha256"]
    sources = json.loads(gzip.decompress(record.read_bytes()))
    guard = json.loads(gzip.decompress((E / "candidate-preparation/main-hashes.json.gz").read_bytes()))
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    representation = SOURCE / "crates/hyperreal/src/computable/node/representation.rs"
    assert ".then_some(((value >> Self::MSD_SHIFT) as u32) as i32)" in representation.read_text()
    tool = ROOT / "crates/hyperreal/target/verus/0.2026.09.13.671956e/verus-x86-linux/verus"
    tool_hash = sha(tool)
    out.mkdir()
    (out / "runner.py").write_bytes(Path(__file__).read_bytes())
    for name, source in PROBES.items():
        (out / (name + ".rs")).write_text(source)
    frozen = {"source_record_sha256": sha(record), "verifier_sha256": tool_hash,
        "runner_sha256": sha(Path(__file__)), "production_input_sha256": sha(representation),
        "probe_sources_sha256": {name: sha(out / (name + ".rs")) for name in PROBES},
        "commands": {name: [str(tool), "--no-cheating", str(out / (name + ".rs"))] for name in PROBES},
        "scope": "Separate expression-support probes for bool::then_some and the masked unreachable arm. Literal 16/32 replace the unchanged production constants in the first probe; neither probe includes or proves the actual decoder. No external specification, axiom or assumed result is added."}
    (out / "protocol.json").write_text(json.dumps(frozen, indent=2) + "\n")
    rows = []
    for name, command in frozen["commands"].items():
        start = time.monotonic()
        result = subprocess.run(command, cwd=out, capture_output=True, timeout=120)
        (out / (name + ".stdout")).write_bytes(result.stdout)
        (out / (name + ".stderr")).write_bytes(result.stderr)
        rows.append({"name": name, "command": command, "exit_code": result.returncode,
                     "seconds": time.monotonic() - start})
    assert sha(tool) == tool_hash
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    summary = {"protocol_sha256": sha(out / "protocol.json"), "source_record_sha256": sha(record),
        "results": rows, "source_files_verified": len(sources), "main_files_verified": len(guard),
        "runtime_source_changed": False, "actual_decoder_verified": False,
        "scope": frozen["scope"], "artifacts_sha256": {p.name: sha(p) for p in sorted(out.iterdir()) if p.is_file()}}
    (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({"probes": [{"name": r["name"], "exit_code": r["exit_code"]} for r in rows],
                      "actual_decoder_verified": False}))


if __name__ == "__main__":
    main()
