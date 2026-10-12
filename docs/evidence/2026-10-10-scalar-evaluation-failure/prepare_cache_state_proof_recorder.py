"""Prepare the current proof audit, retaining the v1836 baseline and every failed experiment."""
import difflib
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    previous = E / "record_bound_merge_proof.py"
    runner = E / "record_cache_state_merge_proof.py"
    patch = E / "cache-state-proof-recorder-v1926.patch"
    provenance = E / "cache-state-proof-recorder-preparation-v1926.json"
    assert not any(path.exists() for path in [runner, patch, provenance])
    original = previous.read_text()
    replacements = {
        "bound-merge-proof-validation-v1840.json": "cache-state-merge-proof-validation-v1926.json",
        "bound-merge-proposal-v1840.patch": "cache-state-merge-proposal-v1926.patch",
        "bound-merge-proof-ledger-v1840.json": "cache-state-merge-proof-ledger-v1926.json",
        "typed-api-prototype-v1838-bound-merge-verus": "typed-api-prototype-v1924-cache-state-merge-verus",
        "typed-api-prototype-v1839-bound-merge-acceptance": "typed-api-prototype-v1925-cache-state-merge-acceptance",
        "typed-api-prototype-v1840-bound-merge-rejections": "typed-api-prototype-v1926-cache-state-merge-rejections",
        "2cdd765b340ce6d0a331aeb25b2596639bc262bb18805cb7efeed27aa43143b9":
            "555cf4a48df68e021119c107412824b5879baf48e252c6fb1c8d2bc22baf1c59",
        'len(calls) == 250': 'len(calls) == 251',
        '== 246': '== 247',
        '("MUTATIONS", 11)': '("MUTATIONS", 12)',
        '"fresh_rejected_mutants": 246': '"fresh_rejected_mutants": 247',
        '"negative_mutations": 246': '"negative_mutations": 247',
        "The actual included BoundInfo::merge body implements the frozen typed selection.":
            "The actual included BoundCache::merge body implements the frozen total cache-state selection, including Invalid and Unknown states. The outlined helper uses the same proved body in ordinary Rust.",
        "The v1838/v1840 bound-merge proposal passes the full conditional proof target with ":
            "The v1924/v1926 cache-state merge proposal passes the full conditional proof target with ",
        "All 246 negative mutations (226 existing and 20 new), four positive guard baselines ":
            "All 247 negative mutations (226 original and 21 merge checks), four positive guard baselines ",
        "The included bound merge now passes conditional Verus and all mutation/acceptance guards. ":
            "The outlined cache-state merge now passes conditional Verus and all mutation/acceptance guards. ",
    }
    source = original
    for old, new in replacements.items():
        assert old in source, old
        source = source.replace(old, new)
    anchor = '    prior_proof = read(E / "domain-predicate-proof-validation-v1763.json")\n'
    assert source.count(anchor) == 1
    source = source.replace(anchor, anchor + '''    diagnostics = {}
    for name in ["cache-state-merge-generated-code-v1919", "inlined-cache-merge-generated-code-v1921",
                 "outlined-cache-merge-generated-code-v1923"]:
        path = E / name / "summary.json"
        item = read(path)
        assert item["timed_processes"] == 0
        assert all(sha(path.parent / artifact) == digest for artifact, digest in item["artifacts_sha256"].items())
        diagnostics[name] = {"summary_sha256": sha(path), "source_record_sha256": item["source_record_sha256"],
            "binary_bytes": item["binary_bytes"], "functions": item["functions"], "timed_processes": 0}
    final_codegen = read(E / "outlined-cache-merge-generated-code-v1923/summary.json")
    assert final_codegen["source_record_sha256"] == expected
    assert final_codegen["only_merge_outlining_attribute_changed"]
    assert final_codegen["publication_plus_selector_bytes"] == 520
    assert diagnostics["cache-state-merge-generated-code-v1919"]["functions"]["publish"]["bytes"] == 1684
    assert diagnostics["inlined-cache-merge-generated-code-v1921"]["functions"]["publish"]["bytes"] == 1684
    experiments = ["bound-merge-cost-rejection-v1857.json", "bound-merge-static-review-v1917.json",
        "cache-state-merge-baseline-v1918/baseline.json", "cache-merge-inlining-baseline-v1920/baseline.json",
        "cache-merge-outlining-baseline-v1922/baseline.json"]
    assert not read(E / experiments[0])["cost_gate_passed"]
    assert not read(E / experiments[1])["runtime_retained"]
''', 1)
    source = source.replace('        "failed_attempts": [],', '''        "failed_attempts": [],
        "prior_experiments_sha256": {name: sha(E / name) for name in experiments},
        "compile_only_diagnostics": diagnostics,
        "outlined_publication_plus_selector_bytes": 520,
        "codegen_interpretation": "The direct-match and forced-inline variants each produce 1684 publication bytes. Outlining produces 292 wrapper bytes plus 228 selector bytes, compared with 517 publication bytes in retained v1836. The probe executable is 232 bytes larger than that baseline. No timings were measured for these variants; the extra call and complete runtime behavior still require all frozen gates.",''')
    old = '''    old = ("Continue implementation-connected review of bound producers, independent real existence "
        "and atomic publication, using the retained public and composed schedules as regression "
        "controls. Conditional predicate soundness does not discharge those prerequisites.")'''
    assert source.count(old) == 1
    source = source.replace(old, '    old = read(E / "bound-merge-static-review-v1917.json")["next_priority"]', 1)
    anchor = '    p4["remaining"] += " " + update\n'
    assert source.count(anchor) == 1
    source = source.replace(anchor, '''    update += (
        " The full cache-state selection now includes Invalid inputs, with an independent "
        "negative mutation for losing incoming evidence there. Direct matching and forced "
        "inlining both generated 1684 publication bytes; outlining yields 520 combined wrapper/"
        "selector bytes versus 517 retained bytes. This is static evidence only. The earlier "
        "v1857 timing rejection remains failed, and none of the new sources has a timing pass."
    )
''' + anchor, 1)
    compile(source, str(runner), "exec")
    assert "246" not in source and "BoundInfo::merge" not in source
    runner.write_text(source)
    patch.write_text("".join(difflib.unified_diff(original.splitlines(keepends=True), source.splitlines(keepends=True),
                    fromfile=previous.name, tofile=runner.name, n=0)))
    provenance.write_text(json.dumps({"predecessor": previous.name, "predecessor_sha256": sha(previous),
        "runner": runner.name, "runner_sha256": sha(runner), "fork_patch": patch.name,
        "fork_patch_sha256": sha(patch), "status": "Prepared; the full fresh rejection suite must finish before execution",
        "scope": "Keep the original v1836 runtime baseline, source/atomic/diagnostic checks and all scope limits. Update actual proof identities and add the invalid-cache mutant, static code-generation evidence and explicit preservation of the failed v1857 timings."}, indent=2) + "\n")
    print(json.dumps({"runner": runner.name, "runner_sha256": sha(runner), "prepared_only": True}))


if __name__ == "__main__":
    main()
