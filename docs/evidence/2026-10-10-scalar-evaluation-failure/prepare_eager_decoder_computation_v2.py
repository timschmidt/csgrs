"""Preserve the v2278 reader failure and support both historical summary schemas."""
import ast
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    original = E / "qualify_eager_decoder_computation.py"
    protocol = E / "eager-bound-decoder-computation-protocol-v2278.json"
    failure = E / "eager-bound-decoder-computation-reader-failure-v2278.json"
    target = E / "qualify_eager_decoder_computation_v2.py"
    assert sha(original) == read(protocol)["runner_sha256"]
    assert not failure.exists() and not target.exists()
    assert not (E / "eager-bound-decoder-computation-review-v2278.json").exists()
    text = original.read_text()
    before = '        assert summary["binary_sha256"] == old_hashes[family]\n'
    after = '''        paired_summary = read(E / historical["original_run"]["phase"] / "summary.json")
        provenance = paired_summary["provenance"][family]
        assert provenance["original_summary_sha256"]["candidate"] == sha(phase / "summary.json")
        assert provenance["binary_sha256_at_paired_run"]["candidate"] == old_hashes[family]
        if "binary_sha256" in summary:
            assert summary["binary_sha256"] == old_hashes[family]
'''
    assert text.count(before) == 1
    text = text.replace(before, after)
    text = text.replace("computation-protocol-v2278.json", "computation-protocol-v2279.json")
    text = text.replace("computation-review-v2278.json", "computation-review-v2279.json")
    before = '        "status": "Prospective qualification method; no replay or new cost workload started",\n'
    after = before + '''        "predecessor_protocol": "eager-bound-decoder-computation-protocol-v2278.json",
        "predecessor_protocol_sha256": sha(E / "eager-bound-decoder-computation-protocol-v2278.json"),
        "reader_failure": "eager-bound-decoder-computation-reader-failure-v2278.json",
        "reader_failure_sha256": sha(E / "eager-bound-decoder-computation-reader-failure-v2278.json"),
        "schema_correction": "Use the preserved paired provenance binding each original summary hash to its executable hash; additionally check direct summary hashes where present. No fixture, cap, historical result, static condition or qualification criterion changes.",
'''
    assert text.count(before) == 1
    text = text.replace(before, after)
    ast.parse(text)
    failure.write_text(json.dumps({
        "status": "v2278 review exited one before producing any qualification",
        "runner": original.name, "runner_sha256": sha(original),
        "protocol": protocol.name, "protocol_sha256": sha(protocol),
        "command": ["python3", str(original.relative_to(E.parents[2])), "review"],
        "exit_code": 1, "exception": "KeyError: 'binary_sha256'",
        "location": "qualify_eager_decoder_computation.py line 94, standalone inverse_trig summary",
        "reason": "Four older family summaries lack a direct binary_sha256 field. Their already retained paired provenance stores the summary hash and executable hash together.",
        "new_timing_processes": 0, "source_changed": False, "qualification_created": False,
        "successor_runner": target.name,
    }, indent=2) + "\n")
    target.write_text(text)
    print(json.dumps({"preserved_failure": failure.name, "successor": target.name, "criteria_unchanged": True}))


if __name__ == "__main__":
    main()
