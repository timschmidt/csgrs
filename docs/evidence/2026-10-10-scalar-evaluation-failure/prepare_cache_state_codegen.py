"""Fork the compile-only code-generation diagnostic for the direct cache match."""
import difflib
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    previous = E / "inspect_flat_merge_codegen.py"
    runner = E / "inspect_cache_state_codegen.py"
    patch = E / "cache-state-codegen-runner-v1919.patch"
    record = E / "cache-state-codegen-preparation-v1919.json"
    assert not any(path.exists() for path in [runner, patch, record])
    original = previous.read_text()
    source = original.replace("typed-api-prototype-v1916-flat-merge-codegen", "typed-api-prototype-v1919-cache-state-merge-codegen")
    source = source.replace("flat-merge-generated-code-v1916", "cache-state-merge-generated-code-v1919")
    source = source.replace("flat-merge-baseline-v1915", "cache-state-merge-baseline-v1918")
    source = source.replace('baseline["rejected_source_record"]', 'baseline["previous_diagnostic_source_record"]')
    source = source.replace("Build the frozen caller variant and inspect code without timing the probe.",
                            "Build the direct cache-state merge and inspect code without timing the probe.")
    source = source.replace('assert set(changed) == {name} and len(current) == len(before) == 24746',
        'assert set(changed) == set(baseline["archived_sources_sha256"]) and len(changed) == 6\n    assert len(current) == len(before) == 24746')
    start = source.index('    previous = """')
    end = source.index('    assert old.count(previous)', start)
    source = source[:start] + '''    previous = """            let combined = match (retained, value) {
                (BoundCache::Invalid | BoundCache::Valid(BoundInfo::Unknown), incoming) => incoming,
                (BoundCache::Valid(bound), incoming) => bound.merge(incoming),
            };
"""
    replacement = """            let combined = retained.merge(value);
"""
''' + source[end:]
    source = source.replace('["rustfmt", "--edition", "2024", "--check", str(SOURCE / name)]',
        '["rustfmt", "--edition", "2024", "--check", str(SOURCE / name), str(SOURCE / "crates/hyperreal/src/computable/node/bounds.rs")]')
    source = source.replace('"changed_since_rejected_proposal_sha256"', '"changed_since_caller_only_diagnostic_sha256"')
    source = source.replace('"single_selection_expression_change_verified"', '"declared_source_set_and_publication_expression_verified"')
    compile(source, str(runner), "exec")
    assert "flat-merge" not in source and "v1916" not in source
    runner.write_text(source)
    patch.write_text("".join(difflib.unified_diff(original.splitlines(keepends=True), source.splitlines(keepends=True),
                    fromfile=previous.name, tofile=runner.name, n=0)))
    record.write_text(json.dumps({"predecessor": previous.name, "predecessor_sha256": sha(previous),
        "runner": runner.name, "runner_sha256": sha(runner), "fork_patch": patch.name,
        "fork_patch_sha256": sha(patch), "timing_processes_permitted": 0,
        "scope": "Preserve all original build/probe/disassembly commands. Update phase identities and source checks for the six declared files, require the exact publication-call replacement, and include bounds.rs in rustfmt checks. The probe is compiled but never executed."}, indent=2) + "\n")
    print(json.dumps({"runner": runner.name, "runner_sha256": sha(runner), "timed_processes": 0}))


if __name__ == "__main__":
    main()
