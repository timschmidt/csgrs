"""Require byte-identical domain observations after the predicate extraction."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parent
read = lambda path: json.loads(path.read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    protocol = read(E / "cache-state-merge-replay-protocol-v1944.json")
    out = E / "cache-state-merge-domain-query-comparison-v1982.json"
    assert not out.exists()
    counts = Counter()
    differences = []
    phases = []
    for item in protocol["caps"][:9]:
        old, new = E / item["baseline_phase"], E / item["phase"]
        result = read(new / "result.json")
        assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"]
        assert sha(new / "source-hashes.json.gz") == protocol["source_record_sha256"]
        before = [line for line in (old / "stderr.log").read_text().splitlines() if line.startswith("domain bound ")]
        after = [line for line in (new / "stderr.log").read_text().splitlines() if line.startswith("domain bound ")]
        assert len(before) == len(after) and before
        for index, (a, b) in enumerate(zip(before, after)):
            if a != b:
                differences.append({"baseline_phase": old.name, "candidate_phase": new.name,
                                    "observation": index, "before": a, "after": b})
            kind = b.split()[2]
            counts[kind] += 1
            if kind == "public":
                answer = re.search(r"answer=(Ok\(-?\d+\)|Err\(\w+\))", b)
                assert answer
                counts["public_success" if answer[1].startswith("Ok(") else "public_exhausted"] += 1
        phases.append({"baseline": old.name, "candidate": new.name, "observations": len(before),
                       "baseline_stderr_sha256": sha(old / "stderr.log"),
                       "candidate_stderr_sha256": sha(new / "stderr.log")})
    assert counts["public"] == 1584 and counts["retry"] == 36
    assert counts["unit"] + counts["lower"] + counts["required"] == 56
    assert counts["endpoint"] + counts["invalid"] + counts["upper-only"] == 48
    out.write_text(json.dumps({
        "status": "All domain observations identical" if not differences else "Differences require review",
        "source_record_sha256": protocol["source_record_sha256"], "phases": phases,
        "observed_counts": dict(counts), "differences": differences,
        "scope": "All 1584 public domain schedules, 56 private certificate queries, 48 endpoint/invalid/upper-only controls and 36 fine-retry schedules from the retained runtime. Exact diagnostic-line comparison preserves samples, typed outcomes, work accounting, cached facts and allocation observations; 144 zero-work cached retry queries remain covered by the frozen tests.",
        "limits": "Finite frozen schedules and independently checked test oracles; no whole-crate proof, aggregate byte/time guarantee or runtime admission follows.",
    }, indent=2) + "\n")
    print(json.dumps({"observations": sum(row["observations"] for row in phases),
                      "public_successes": counts["public_success"], "changed_observations": len(differences)}))
    assert not differences
    assert counts["public_success"] == 453 and counts["public_exhausted"] == 1131


if __name__ == "__main__":
    main()
