"""Exercise the verbatim committed ray filter with independently known hits."""

import hashlib
import json
from pathlib import Path
import subprocess
import time

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / "ray-filter-overflow-baseline"
OUT.mkdir()
WORK = ROOT / "target/ray-filter-overflow"
WORK.mkdir(exist_ok=True)
name = "crates/hypercurve/src/bezier_region.rs"
source = subprocess.check_output(["git", "show", "HEAD:" + name], cwd=ROOT, text=True)
start = source.index("fn f64_box_certainly_misses_forward_ray(")
end = source.index("\n}\n", start) + 3
body = source[start:end]
assert body in (ROOT / name).read_text()
main = r'''
fn main() {
    let mut failures = 0;
    let mut cases = 0;
    let mut check = |name: String, bounds: [f64; 4], origin: [[f64; 2]; 2], direction: [[f64; 2]; 2], expected_miss: bool, witness: &str| {
        assert!(bounds.into_iter().chain(origin.into_iter().flatten()).chain(direction.into_iter().flatten()).all(f64::is_finite));
        let miss = f64_box_certainly_misses_forward_ray(bounds, origin, direction);
        cases += 1;
        failures += usize::from(miss != expected_miss);
        println!("{{\"case\":\"{name}\",\"bounds\":{bounds:?},\"origin\":{origin:?},\"direction\":{direction:?},\"expected_miss\":{expected_miss},\"actual_miss\":{miss},\"witness\":\"{witness}\",\"all_inputs_finite\":true}}");
    };
    for scale in [1e-200_f64, 1.0, 1e150, 1e154, 1e155, 1e200, 1e300] {
        for sx in [-1.0_f64, 1.0] {
            for sy in [-1.0_f64, 1.0] {
                let x = sx * scale;
                let y = sy * scale;
                check(format!("ray_hit_{scale:e}_{sx}_{sy}"), [x.min(2.0*x), x.max(2.0*x), y.min(2.0*y), y.max(2.0*y)], [[0.0; 2]; 2], [[x; 2], [y; 2]], false, "At exact parameter one the ray equals a stored box corner.");
            }
        }
    }
    for sx in [-1.0_f64, 1.0] {
        for sy in [-1.0_f64, 1.0] {
            let x = sx * 1e308;
            let y = sy * 1e308;
            check(format!("inside_large_translation_{sx}_{sy}"), [(x*0.9).min(x*1.1), (x*0.9).max(x*1.1), (y*0.9).min(y*1.1), (y*0.9).max(y*1.1)], [[x; 2], [y; 2]], [[1.0; 2], [2.0; 2]], false, "The ray origin lies strictly inside both stored box intervals.");
        }
    }
    check("ordinary_side_miss".into(), [-2.0,-1.0,1.0,2.0], [[0.0; 2]; 2], [[1.0; 2]; 2], true, "Opposite-sign coordinates put every corner strictly left of y=x.");
    check("ordinary_behind_miss".into(), [-2.0,-1.0,-2.0,-1.0], [[0.0; 2]; 2], [[1.0; 2]; 2], true, "Every corner has a negative dot product with (1,1).");
    assert_eq!(failures, 0, "{failures} of {cases} exact box/ray witnesses were contradicted");
}
'''
(OUT / "probe.rs").write_text(body + main)
commands = []
for label, command in [
    ("build", ["rustc", "--edition=2024", "-O", "-D", "warnings", str(OUT / "probe.rs"), "-o", str(WORK / "baseline")]),
    ("run", [str(WORK / "baseline")]),
]:
    start_time = time.monotonic()
    with (OUT / (label + ".log")).open("w") as log, (OUT / (label + ".jsonl")).open("w") as rows:
        result = subprocess.run(command, cwd=ROOT, stdout=rows, stderr=log, check=False)
    commands.append({"name": label, "command": command, "exit_code": result.returncode, "wall_seconds": time.monotonic()-start_time})
    if label == "build": assert result.returncode == 0
rows = [json.loads(line) for line in (OUT / "run.jsonl").read_text().splitlines()]
assert len(rows) == 34
failures = [row for row in rows if row["actual_miss"] != row["expected_miss"]]
assert failures and commands[-1]["exit_code"] == 101
record = {
    "scope": "The committed private floating-point filter is extracted verbatim and compiled independently. Degenerate finite enclosures denote exact binary64 rational values. Ray parameter one and strict origin containment prove intersection independently of this implementation. This is a helper counterexample, not yet a public region-query reproduction.",
    "head": subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
    "source_file": name, "source_body_sha256": hashlib.sha256(body.encode()).hexdigest(),
    "probe_sha256": hashlib.sha256((OUT / "probe.rs").read_bytes()).hexdigest(),
    "compiler": subprocess.check_output(["rustc", "-Vv"],text=True),
    "commands": commands, "case_count": len(rows), "failure_count": len(failures),
    "main_runtime_unchanged": True,
}
(OUT / "manifest.json").write_text(json.dumps(record, indent=2)+"\n")
(OUT / "reproduce.py").write_bytes(Path(__file__).read_bytes())
print(f"Verbatim committed filter contradicts {len(failures)} of {len(rows)} independent finite-input witnesses.")
