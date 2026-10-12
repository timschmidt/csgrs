"""Freeze the failed static hypothesis and prepare a one-attribute inlining diagnostic."""
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
    baseline_dir = E / "cache-merge-inlining-baseline-v1920"
    runner = E / "inspect_inlined_cache_merge.py"
    patch = E / "cache-merge-inlining-runner-v1921.patch"
    provenance = E / "cache-merge-inlining-preparation-v1921.json"
    assert not any(path.exists() for path in [baseline_dir, runner, patch, provenance])
    phase = E / "typed-api-prototype-v1919-cache-state-merge-codegen"
    result = read(phase / "result.json")
    assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"]
    codegen_path = E / "cache-state-merge-generated-code-v1919/summary.json"
    codegen = read(codegen_path)
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
        "status": "Direct cache-state extraction grows publication code; freeze it before testing an inlining attribute",
        "previous_diagnostic_source_record": str(record.relative_to(E)),
        "previous_diagnostic_source_record_sha256": sha(record),
        "source_files_verified": len(sources), "archived_sources_sha256": {name: sha(archived)},
        "previous_codegen": str(codegen_path.relative_to(E)), "previous_codegen_sha256": sha(codegen_path),
        "previous_publication_bytes": 1684, "first_rejected_publication_bytes": 860,
        "retained_publication_bytes": 517, "timed_processes": 0,
        "change_boundary": "Only change #[inline] to #[inline(always)] on the included pure BoundCache::merge method. Keep the match body, contract, all other source files, mutation guards, atomic wrapper, fixtures and compiler flags unchanged.",
        "limits": "The direct-match hypothesis failed to reduce generated code. No timing or fresh proof claim follows from that build. Explicit inlining is a compiler experiment; only the unchanged full proof/runtime/cost/replay gates can qualify any resulting source. Retained v1836, both static variants, the v1857 timing rejection and all older failures/baselines remain unchanged.",
    }
    (baseline_dir / "baseline.json").write_text(json.dumps(baseline, indent=2) + "\n")
    previous = E / "inspect_cache_state_codegen.py"
    original = previous.read_text()
    source = original.replace("typed-api-prototype-v1919-cache-state-merge-codegen", "typed-api-prototype-v1921-inlined-cache-merge-codegen")
    source = source.replace("cache-state-merge-generated-code-v1919", "inlined-cache-merge-generated-code-v1921")
    source = source.replace("cache-state-merge-baseline-v1918", "cache-merge-inlining-baseline-v1920")
    source = source.replace("Build the direct cache-state merge and inspect code without timing the probe.",
                            "Inspect the one-attribute pure-merge inlining variant without timing it.")
    source = source.replace('name = "crates/hyperreal/src/computable/node/representation.rs"',
                            'name = "crates/hyperreal/src/computable/node/bounds.rs"')
    source = source.replace('and len(changed) == 6', 'and len(changed) == 1')
    start = source.index('    previous = """')
    end = source.index('    out.mkdir()', start)
    source = source[:start] + '''    start = old.index("impl BoundCache {")
    end = old.index("impl BoundInfo {", start)
    method = old[start:end]
    assert method.count("#[inline]") == 1
    assert (SOURCE / name).read_text() == old[:start] + method.replace("#[inline]", "#[inline(always)]", 1) + old[end:]
''' + source[end:]
    source = source.replace('str(SOURCE / "crates/hyperreal/src/computable/node/bounds.rs")',
                            'str(SOURCE / "crates/hyperreal/src/computable/node/representation.rs")')
    source = source.replace('"changed_since_caller_only_diagnostic_sha256"', '"changed_since_direct_cache_match_sha256"')
    source = source.replace('"declared_source_set_and_publication_expression_verified"', '"only_merge_inlining_attribute_changed"')
    source = source.replace('"rejected_bytes": reference["functions"]["candidate"][label]["bytes"]}',
        '"rejected_bytes": reference["functions"]["candidate"][label]["bytes"],\n            "direct_match_bytes": read(E / baseline["previous_codegen"])["functions"][label]["bytes"]}')
    compile(source, str(runner), "exec")
    runner.write_text(source)
    patch.write_text("".join(difflib.unified_diff(original.splitlines(keepends=True), source.splitlines(keepends=True),
                    fromfile=previous.name, tofile=runner.name, n=0)))
    provenance.write_text(json.dumps({"predecessor": previous.name, "predecessor_sha256": sha(previous),
        "runner": runner.name, "runner_sha256": sha(runner), "fork_patch": patch.name,
        "fork_patch_sha256": sha(patch), "baseline": str((baseline_dir / "baseline.json").relative_to(E)),
        "baseline_sha256": sha(baseline_dir / "baseline.json"), "timing_processes_permitted": 0,
        "scope": "Identical format/build/probe/disassembly commands and compiler flags. Only phase identities and checks for the single inlining attribute differ."}, indent=2) + "\n")
    print(json.dumps({"baseline": baseline_dir.name, "runner": runner.name, "timed_processes": 0}))


if __name__ == "__main__":
    main()
