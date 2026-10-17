"""Summarize terminal validation results without broadening their claims."""
import json
import re
from pathlib import Path

W = Path(__file__).resolve().parent


def read(p):
    return json.loads(Path(p).read_text())


def summaries(path):
    return [
        {"passed": int(p), "failed": int(f), "ignored": int(i), "measured": int(m), "filtered_out": int(x)}
        for p, f, i, m, x in re.findall(
            r"test result: \w+\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out;",
            Path(path).read_text(),
        )
    ]


results = {}
for name in ["default-corpus-v1", "native-build-v1", "native-corpus-v1", "curve-all-v1",
             "storage-validation-v1", "native-build-v2", "native-corpus-v2"]:
    result = read(W / name / "result.json")
    assert result["complete"], name
    commands = read(W / name / "commands.json")
    test_summaries = []
    ignored = []
    for command in commands:
        output = W / name / (command["name"] + ".stdout")
        test_summaries.extend(summaries(output))
        ignored.extend(line for line in output.read_text().splitlines() if " ... ignored" in line)
    results[name] = {
        "result": "docs/evidence/2026-10-17-circle-support-corpus/" + name + "/result.json",
        "command_failures": result["command_failures"],
        "commands": [
            {k: row[k] for k in ["name", "exit_code", "elapsed_seconds", "external_stop_reason"] if k in row}
            for row in commands
        ],
        "test_summaries": test_summaries,
        "existing_ignored_test_lines": ignored,
        "test_totals": {key: sum(row[key] for row in test_summaries)
                        for key in ["passed", "failed", "ignored", "measured", "filtered_out"]},
        "scope_complete": not result["command_failures"],
    }
assert read(W / "curve-all-v1/result.json")["cargo_target_closed"]
assert read(W / "storage-validation-v1/result.json")["command_failures"] == []
repair = read(W / "storage-preparation-v1/result.json")["implementation_sha256"]
native = read(W / "native-corpus-v2/result.json")
decision = {
    "phase": "Qualification decision before checkpoint application; retention-v1.json records the applied test-only repair.",
    "main_parent_commit": "0ed5a970cf0d9204e9f4c5c73e93b16a6a13bc89",
    "status": "qualified two-file test-storage repair; cache proposal remains unadmitted with continued corpus results recorded",
    "cache_status": "unadmitted: continued curve and native corpus results recorded; remaining affected-crate, binding and matched-cost qualification is open",
    "progress_classification": "progress",
    "owning_layer": "Hypercurve intersection batch cache; public native consumers checked without changing their represented geometry.",
    "properties": ["P4", "P17", "P18", "P19", "P20", "P27", "P35", "P36", "P37", "P38", "P39"],
    "results": results,
    "implementation_sha256": repair,
    "test_storage_repair_qualified": True,
    "production_prefixes_unchanged": True,
    "final_native_library_tests_passed": native["passed_library_tests"],
    "unchanged_native_consumer_and_doc_tests_reused": native["reused_passed_consumer_and_doc_tests"],
    "source_changes_applied": False,
    "cache_index_admitted": False,
    "performance_claimed": False,
    "full_plan_complete": False,
    "prior_decisions_preserved": [
        "The original 35-file native bundle remains rejected; its independently admitted components stay admitted.",
        "The E36 general exact KiCad track integration remains rejected by its unchanged workload gates.",
        "The E39 temporary-storage test repair remains admitted, and the full project report byte comparison remains valid for the same production candidate.",
    ],
    "limits": [
        "Only the independently qualified two-file test-storage repair is eligible for application. The cache proposal remains isolated; neither it nor the rejected exact-track integration is admitted.",
        "Full CSGRS and Hypercircuit affected-configuration runs, actual C/C++/Python/WASM consumers and final matched-cost samples remain outstanding for this proposal.",
        "The reused library was built with --no-default-features; the crate declares an empty default feature set. This minimal-feature library result is not a separate default-feature Cargo invocation or an integration/doc pass. See feature-scope-v1.json.",
        "Any failed or externally stopped command is retained in the result table and is not converted into a pass.",
        "The /tmp per-user quota was still full at the start of this continuation. New output uses guarded repository storage; retained build artifacts were not removed or reused for writes.",
        "The 1,800-second/8-GiB external containment and 768-MiB repository reserve are harness limits, not production work, heap or cancellation guarantees.",
        "No performance comparison is inferred from these functional runs. Curve functional suites overlapped independent compilation; both native runs waited for their preceding task-owned CPU-heavy checks to finish.",
        "The inherited audit still has 27,093 missing paths. New paths are pinned separately; no whole-history preservation claim is made.",
        "All other properties, future-work, coverage, directed campaigns, proofs, bindings, distribution and release obligations remain open according to their existing records.",
    ],
    "next": [
        "Investigate any recorded corpus failure without weakening policy, fixture resolution, assertions or validation.",
        "Complete remaining affected CSGRS and Hypercircuit configurations and real binding consumers in available, guarded storage; resolve the pending larger-build storage constraint.",
        "Complete matched cold/warm cost samples, preserving exact rows, certainty, report bytes and unfavorable cases, before deciding whether to admit the cache change.",
        "Continue ordinary strict-mode workload completion and the full property plan; the rejected native-ingress integration still needs its own acceptance evidence.",
    ],
}
(W / "decision-v1.json").write_text(json.dumps(decision, indent=2) + "\n")
rows = []
for name, result in results.items():
    counts = result["test_totals"]
    failures = ", ".join(result["command_failures"]) or "none"
    rows.append(
        f"| {name} | {counts['passed']} | {counts['failed']} | {counts['ignored']} | {counts['filtered_out']} | {failures} |"
    )
text = """# Continued curve and native cache qualification

This continuation made progress by executing the acceptance corpus and qualifying
the remaining five test-fixture storage corrections in two files. The index
remains isolated and unadmitted; production implementation is unchanged.
The exact command exits, terminal results and raw output are retained beside this
record. Updating this record does not itself establish implementation acceptance.

| Scope | Passed tests | Failed tests | Existing ignored tests | Filtered out | Failed commands |
| --- | ---: | ---: | ---: | ---: | --- |
""" + "\n".join(rows) + """

The minimal-feature Hypercurve library executable is reused only after source,
feature and binary checks. Its build used --no-default-features; the manifest
declares an empty default feature set. The earlier default-feature shorthand is
clarified in feature-scope-v1.json, without editing frozen protocols. Both
existing ignored tests remain unchanged, including E38's diagnostic cost test.
A fresh runtime source mirror preserves any generated failure artifacts without
modifying the closed E38 source snapshot. No separate default-feature Cargo
invocation or integration/documentation pass is claimed.

The first native library test harness is compiled with the current, independently
admitted temporary-storage repair and the verified E38 candidate dependencies.
Production source, fixtures and policies are otherwise identical to that
candidate. All 1,559 native library tests are selected, including the original
full-board, copper-neck and Gerber gates. Unchanged integration executables and
their compiled CLI path are hash-checked, and documentation tests are executed.
The first run reports five additional hardcoded-/tmp fixture failures. Each
reproduces with the committed-main harness and writable selected storage, without
the index. The repair changes only five test path constructors and removes one
unused test import. Fixture bytes, assertions and production prefixes are
unchanged. All 39 app tests and the Arrow test pass; all-feature/all-target
HyperDRC Clippy is strict, and selected temporary storage is cleaned.

The repaired indexed harness replays the entire native library after the
all-feature curve run and Clippy finish, preserving all three workload gates.
Its unchanged consumer executables and documentation checks are reused from the
first run only after source, production-prefix, dependency and artifact checks.
The original failures remain in the table and raw logs. A successful replay
does not erase them or imply admission of the cache.

The all-feature Hypercurve build uses a fresh Cargo target in repository storage.
Its full library, integration and documentation run keeps the original fixtures,
policies and generated-case settings. Strict Clippy covers all features and
targets. The target is closed after these commands; it may subsequently be read
but never reopened for writes.

The independent all-feature build may overlap the minimal-feature functional corpus.
Both finish before native timing gates begin; the all-feature tests and Clippy
wait for the first native corpus to finish. The independent test-repair builds
may overlap curve functional tests. The second native run waits for all those
checks to become terminal. No speedup is inferred from these runs.
Each executed command has 1,800-second external containment and an 8-GiB process
address-space limit. Repository free space is monitored with a 768-MiB reserve.
These limits do not establish internal work, heap or cancellation bounds.

The original 35-file disposition and the E36 rejection remain unchanged.
The complete E39 CPArti and HVP109A report-byte comparison remains evidence for
the same production proposal, not a claim about the rejected exact-track model.
The inherited 27,093 missing physical paths remain disclosed; this checkpoint
individually pins its own new physical paths.

The cache still needs the remaining affected CSGRS/Hypercircuit configurations,
actual binding consumers and final matched-cost evidence. Default-feature
integration/doc coverage is not inferred from the library run. Any failures in
this checkpoint need investigation without weakening the contract. All other
property-plan and release obligations remain active.
"""
(W / "progress-v1.md").write_text(text)
print(json.dumps({name: result["test_totals"] | {"command_failures": result["command_failures"]}
                  for name, result in results.items()}))
