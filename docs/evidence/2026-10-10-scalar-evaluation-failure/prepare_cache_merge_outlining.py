"""Freeze unsuccessful forced inlining and prepare an outlined selector diagnostic."""
import difflib
import gzip
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
read = lambda path: json.loads(path.read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    baseline_dir = E / "cache-merge-outlining-baseline-v1922"
    runner = E / "inspect_outlined_cache_merge.py"
    patch = E / "cache-merge-outlining-runner-v1923.patch"
    provenance = E / "cache-merge-outlining-preparation-v1923.json"
    assert not any(path.exists() for path in [baseline_dir, runner, patch, provenance])
    phase = E / "typed-api-prototype-v1921-inlined-cache-merge-codegen"
    result = read(phase / "result.json")
    assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"]
    codegen_path = E / "inlined-cache-merge-generated-code-v1921/summary.json"
    codegen = read(codegen_path)
    assert codegen["only_merge_inlining_attribute_changed"]
    assert codegen["functions"]["publish"]["bytes"] == 1684 and codegen["timed_processes"] == 0
    record = phase / "source-hashes.json.gz"
    assert sha(record) == codegen["source_record_sha256"]
    sources = json.loads(gzip.decompress(record.read_bytes()))
    for name, digest in sources.items():
        assert sha(SOURCE / name) == digest, name
    baseline_dir.mkdir()
    name = "crates/hyperreal/src/computable/node/bounds.rs"
    archived = baseline_dir / "inputs" / name
    archived.parent.mkdir(parents=True)
    archived.write_bytes((SOURCE / name).read_bytes())
    baseline = {
        "status": "Forced inlining leaves publication at 1684 bytes; freeze it before testing an outlined selector",
        "previous_diagnostic_source_record": str(record.relative_to(E)),
        "previous_diagnostic_source_record_sha256": sha(record),
        "source_files_verified": len(sources), "archived_sources_sha256": {name: sha(archived)},
        "previous_codegen": str(codegen_path.relative_to(E)), "previous_codegen_sha256": sha(codegen_path),
        "previous_publication_bytes": 1684, "first_rejected_publication_bytes": 860,
        "retained_publication_bytes": 517, "timed_processes": 0,
        "change_boundary": "Only change #[inline(always)] to #[inline(never)] on the pure BoundCache::merge selector. Its body, contract, all callers, mutation cases, other sources, fixtures and compiler flags remain identical.",
        "diagnostic_scope": "Include the standalone selector symbol as well as publication and compare their aggregate code size. A smaller wrapper alone is not evidence of smaller total selection code. The new call can have a runtime cost; only the unchanged full cost protocol can measure it.",
        "limits": "No new timing observations exist for the direct-match or forced-inline variants. Neither is retained. Keep the prior v1857 cost rejection, retained v1836, all previous diagnostics and full proof/runtime/cost/replay gates unchanged.",
    }
    (baseline_dir / "baseline.json").write_text(json.dumps(baseline, indent=2) + "\n")
    previous = E / "inspect_inlined_cache_merge.py"
    original = previous.read_text()
    source = original.replace("typed-api-prototype-v1921-inlined-cache-merge-codegen", "typed-api-prototype-v1923-outlined-cache-merge-codegen")
    source = source.replace("inlined-cache-merge-generated-code-v1921", "outlined-cache-merge-generated-code-v1923")
    source = source.replace("cache-merge-inlining-baseline-v1920", "cache-merge-outlining-baseline-v1922")
    source = source.replace("Inspect the one-attribute pure-merge inlining variant without timing it.",
                            "Inspect the outlined selector and its wrapper without timing them.")
    source = source.replace('assert method.count("#[inline]") == 1', 'assert method.count("#[inline(always)]") == 1')
    source = source.replace('method.replace("#[inline]", "#[inline(always)]", 1)',
                            'method.replace("#[inline(always)]", "#[inline(never)]", 1)')
    source = source.replace('"changed_since_direct_cache_match_sha256"', '"changed_since_forced_inline_match_sha256"')
    source = source.replace('"only_merge_inlining_attribute_changed"', '"only_merge_outlining_attribute_changed"')
    anchor = '    functions = {}\n'
    assert source.count(anchor) == 1
    source = source.replace(anchor, '''    # Earlier variants inline this code; zero here means no standalone symbol.
    for which in ["baseline", "candidate"]:
        reference["functions"][which]["merge_helper"] = {
            "symbol": "<hyperreal::computable::node::BoundCache>::merge", "bytes": 0}
    functions = {}
''', 1)
    source = source.replace('read(E / baseline["previous_codegen"])["functions"][label]["bytes"]',
        'read(E / baseline["previous_codegen"])["functions"].get(label, {"bytes": 0})["bytes"]')
    source = source.replace('"functions": functions, "timed_processes": 0,',
        '"functions": functions, "publication_plus_selector_bytes": functions["publish"]["bytes"] + functions["merge_helper"]["bytes"], "timed_processes": 0,')
    source = source.replace('"binary_bytes": summary["binary_bytes"], "timed_processes": 0}',
        '"binary_bytes": summary["binary_bytes"], "publication_plus_selector_bytes": summary["publication_plus_selector_bytes"], "timed_processes": 0}')
    compile(source, str(runner), "exec")
    runner.write_text(source)
    patch.write_text("".join(difflib.unified_diff(original.splitlines(keepends=True), source.splitlines(keepends=True),
                    fromfile=previous.name, tofile=runner.name, n=0)))
    provenance.write_text(json.dumps({"predecessor": previous.name, "predecessor_sha256": sha(previous),
        "runner": runner.name, "runner_sha256": sha(runner), "fork_patch": patch.name,
        "fork_patch_sha256": sha(patch), "baseline": str((baseline_dir / "baseline.json").relative_to(E)),
        "baseline_sha256": sha(baseline_dir / "baseline.json"), "timing_processes_permitted": 0,
        "scope": "Preserve all compiler and fixture settings; update phase identities and one-attribute source checks, and inspect the standalone selector so total selection code is visible."}, indent=2) + "\n")
    print(json.dumps({"baseline": baseline_dir.name, "runner": runner.name, "timed_processes": 0}))


if __name__ == "__main__":
    main()
