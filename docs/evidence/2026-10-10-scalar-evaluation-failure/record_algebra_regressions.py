#!/usr/bin/env python3
"""Summarize retained algebra/domain failures without changing their evidence."""
import hashlib
import json
from pathlib import Path
import re
import sys

sys.dont_write_bytecode = True
from parse_test_results_v2 import parse_phase

EVIDENCE = Path(__file__).resolve().parent


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    out = EVIDENCE / sys.argv[1]
    assert not out.exists()
    phases = {}
    for version in range(142, 152):
        matches = list(EVIDENCE.glob(f"typed-api-prototype-v{version}-*/result.json"))
        assert len(matches) == 1, (version, matches)
        phase = matches[0].parent
        stdout = (phase / "stdout.log").read_text()
        stderr = (phase / "stderr.log").read_text()
        result = json.loads(matches[0].read_text())
        assert result["source_unchanged"] and result["main_unchanged"]
        phases[phase.name] = {
            "result": result,
            "tests": parse_phase(phase),
            "source_record_sha256": digest(phase / "source-hashes.json.gz"),
            "stdout_sha256": digest(phase / "stdout.log"),
            "stderr_sha256": digest(phase / "stderr.log"),
            "false_fact_responses": len(re.findall(
                r"^undefined=\d+, form=\d+, fact=\d+$", stdout, re.MULTILINE)),
            "invalid_numeric_returns": len(re.findall(
                r"^undefined=\d+, form=\d+, p=-?\d+: Ok", stdout, re.MULTILINE)),
            "invalid_numeric_cache_publications": len(re.findall(
                r"^undefined=\d+, form=\d+, p=-?\d+: cached invalid result$",
                stdout, re.MULTILINE)),
            "coarse_descendant_failures": re.findall(
                r"^(?:cos|sin|deferred_cos|deferred_sin|sinc|cosc|tiny_sqrt|tiny_cube_root|ambiguous_sqrt|ambiguous_cube_root), p=.*$",
                stdout, re.MULTILINE),
            "negative_sinc_coarse_failures": re.findall(
                r"^p=\d+: Ok.*$", stdout, re.MULTILINE),
            "compile_errors": re.findall(r"^error\[.*$", stderr, re.MULTILINE),
        }
    original = phases["typed-api-prototype-v142-algebra-domain-regressions"]
    assert [original[key] for key in ["false_fact_responses", "invalid_numeric_returns",
                                     "invalid_numeric_cache_publications"]] == [265, 258, 60]
    intermediate = phases["typed-api-prototype-v143-algebra-fact-repair"]
    assert [intermediate[key] for key in ["false_fact_responses", "invalid_numeric_returns",
                                         "invalid_numeric_cache_publications"]] == [14, 102, 102]
    assert len(phases["typed-api-prototype-v145-descendant-domain-regressions"][
        "coarse_descendant_failures"]) == 10
    ambiguous = phases["typed-api-prototype-v146-ambiguous-domain-regressions"]
    assert len(ambiguous["coarse_descendant_failures"]) == 14
    assert len(ambiguous["negative_sinc_coarse_failures"]) == 3
    assert phases["typed-api-prototype-v147-real-domain-unit-tests"]["tests"][
        "top_level_failed"] == 3
    assert phases["typed-api-prototype-v148-real-domain-unit-repair"]["tests"][
        "top_level_passed"] == 910
    assert phases["typed-api-prototype-v151-domain-refinement-tests"]["tests"][
        "top_level_passed"] == 11
    data = {
        "status": "Retained failures and focused repair runs; no production admission",
        "helper_sha256": digest(Path(__file__)),
        "phases": phases,
        "counting_limits": [
            "Counts describe individual retained phases. Repeated failures across phases are not summed as distinct counterexamples.",
            "A numeric return and its root cache publication are separate observations of the same query, not two numeric queries.",
            "The v147 failures are tangent cancellation propagation, unnecessary square evaluation during domain validation, and a quantile poll-sweep ceiling. The last is a coverage-ceiling failure, not an observed wrong quantile.",
            "The quantile fixture workload was preserved; its cancellation sweep ceiling increased from 4096 to 16384 to include added validation polls.",
            "v149 and v150 retain test-helper compilation failures (missing closure integer type, then i32 instead of the Rational constructor's i64); no tests executed in those phases.",
        ],
    }
    out.write_text(json.dumps(data, indent=2) + "\n")
    print(json.dumps({"output": out.name, "phases": len(phases),
                      "original_false_facts": original["false_fact_responses"],
                      "original_invalid_numeric_returns": original["invalid_numeric_returns"]}))


if __name__ == "__main__":
    main()
