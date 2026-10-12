"""Compare deterministic replay receipts without discarding work or fact changes."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys

sys.set_int_max_str_digits(20000)
E = Path(__file__).resolve().parent
read = lambda path: json.loads(path.read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def describe(value):
    encoded = json.dumps(value, sort_keys=True)
    if len(encoded) > 512:
        return {"json_bytes": len(encoded.encode()), "sha256": hashlib.sha256(encoded.encode()).hexdigest()}
    return value


def main():
    protocol_path = E / "cache-state-merge-replay-protocol-v1944.json"
    protocol = read(protocol_path)
    validation = read(E / "cache-state-merge-replay-validation-v1997.json")
    assert validation["complete"] and validation["source_record_sha256"] == protocol["source_record_sha256"]
    assert validation["protocol_sha256"] == sha(protocol_path)
    out = E / "cache-state-merge-deterministic-replay-comparison-v1998.json"
    assert not out.exists()
    pairs = [(item["baseline_phase"], item["phase"]) for item in protocol["numerical_replays"]]
    pairs += [("typed-api-prototype-v1833-domain-predicate-public-proof-reuse", protocol["public_replays"][0]["phase"]),
              ("typed-api-prototype-v1834-domain-predicate-composed-proof-reuse", protocol["public_replays"][1]["phase"])]
    mappings = [{old: f"<replay-{index}>" for index, (old, new) in enumerate(pairs)},
                {new: f"<replay-{index}>" for index, (old, new) in enumerate(pairs)}]
    differences = []
    omitted_times = Counter()
    compared_cases = Counter()

    def command_path(value, side):
        if isinstance(value, str):
            for phase, marker in mappings[side].items():
                value = value.replace(phase, marker)
        return value

    def normalized(value, side, path=()):
        if isinstance(value, dict):
            answer = {}
            for key, item in value.items():
                if key in {"wall_seconds", "seconds", "elapsed_ns"}:
                    omitted_times[key] += 1
                else:
                    answer[key] = normalized(item, side, path + (key,))
            return answer
        if isinstance(value, list):
            if path and path[-1] == "command":
                return [command_path(item, side) for item in value]
            return [normalized(item, side, path + (str(index),)) for index, item in enumerate(value)]
        return value

    def compare(before, after, context, path=()):
        if before == after:
            return
        if isinstance(before, dict) and isinstance(after, dict) and before.keys() == after.keys():
            for key in before:
                compare(before[key], after[key], context, path + (key,))
        elif isinstance(before, list) and isinstance(after, list) and len(before) == len(after):
            for index, (a, b) in enumerate(zip(before, after)):
                compare(a, b, context, path + (str(index),))
        else:
            differences.append({**context, "path": ".".join(path),
                                "before": describe(before), "after": describe(after)})

    def check_phase(old, new):
        summary = read(new / "summary.json")
        assert summary["source_hashes_sha256"] == protocol["source_record_sha256"] and summary["guards_unchanged"]
        assert sha(old / "reproduce.py") == sha(new / "reproduce.py")
        return summary

    for item in protocol["numerical_replays"]:
        old, new = E / item["baseline_phase"], E / item["phase"]
        summary = check_phase(old, new)
        before, after = read(old / "receipts.json"), read(new / "receipts.json")
        expected = 400 if item["expected_cases"] == 415 else item["expected_cases"]
        assert len(before) == len(after) == expected
        for a, b in zip(before, after):
            assert a["name"] == b["name"]
            compare(normalized(a, 0), normalized(b, 1), {"phase": new.name, "case": a["name"]})
        compared_cases["numerical"] += len(before)
        if expected == 400:
            previous = read(old / "summary.json")
            for key, count in [("extremes", 6), ("cache", 9)]:
                assert len(previous[key]) == len(summary[key]) == count
                compare(normalized(previous[key], 0), normalized(summary[key], 1),
                        {"phase": new.name, "case": key})
                compared_cases["numerical"] += count
    assert compared_cases["numerical"] == 4129

    for index, item in enumerate(protocol["public_replays"]):
        old = E / pairs[13 + index][0]
        new = E / item["phase"]
        summary = read(new / "summary.json")
        assert summary["guards_unchanged"] and summary["source_hashes_sha256"] == protocol["source_record_sha256"]
        assert sha(old / "probe.rs") == sha(new / "probe.rs") == item["probe_sha256"]
        before, after = read(old / "receipts.json"), read(new / "receipts.json")
        assert len(before) == len(after) == item["expected_cases"]
        for a, b in zip(before, after):
            assert a["name"] == b["name"] and a["arguments"] == b["arguments"]
            context = {"phase": new.name, "case": a["name"]}
            if index == 0 and a["arguments"][0] in {"race", "invalid"}:
                # Worker outcomes vary with scheduling. Preserve setup, oracle,
                # process limits and success; separate audits check every fact.
                for key in ["name", "arguments", "command", "complete"]:
                    compare(normalized({key: a[key]}, 0), normalized({key: b[key]}, 1), context)
                compare({k: v for k, v in a["records"][0].items() if k != "result"},
                        {k: v for k, v in b["records"][0].items() if k != "result"}, context)
                compared_cases["concurrent_setup_only"] += 1
            else:
                compare(normalized(a, 0), normalized(b, 1), context)
                compared_cases["public_deterministic"] += 1
    assert compared_cases["public_deterministic"] == 570
    assert compared_cases["concurrent_setup_only"] == 78
    out.write_text(json.dumps({
        "status": "All compared observations identical" if not differences else "Observed differences require review",
        "source_record_sha256": protocol["source_record_sha256"], "protocol_sha256": sha(protocol_path),
        "compared_cases": dict(compared_cases), "ignored_timing_fields_across_both_sides": dict(omitted_times),
        "differences": differences,
        "scope": "Compare every retained numerical replay, all composed-root schedules and public sequential/prewarmed schedules. Only measured durations and known phase names in executable paths are normalized. Samples, work counters, typed errors, certificate observations, process limits and validity receipts remain compared.",
        "limits": "For 72 mixed-thread and six invalid-thread schedules, compare setup/oracles/process contracts only. Their scheduling-dependent results require the separate monotonic-fact audit and the frozen Rust/Python oracle checks; this comparison alone does not establish concurrent correctness or runtime admission.",
    }, indent=2) + "\n")
    print(json.dumps({"cases": dict(compared_cases), "changed_fields": len(differences)}))
    assert not differences


if __name__ == "__main__":
    main()
