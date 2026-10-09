"""Assemble the scoped retention record from preserved raw evidence."""
import hashlib
import json
import statistics
from pathlib import Path

record = Path(__file__).resolve().parent
root = record.parents[2]


def read(name):
    return json.loads((record / name).read_text())


def info(path):
    data = path.read_bytes()
    return dict(bytes=len(data), sha256=hashlib.sha256(data).hexdigest())


validation = read("validation/status.json")
assert len(validation) == 6 and all(step["exit_code"] == 0 for step in validation)
frozen = read("validation/sources.json")
assert frozen["unchanged"]
assert all(info(root / path)["sha256"] == digest for path, digest in frozen["source_hashes"].items())
cost = read("cost/manifest.json")
before, after = (cost["versions"][version] for version in ["baseline", "candidate"])
recovered = [row["name"] for row in cost["deltas"] if row["before"]["status"] == "blocked"]
preserved = [row["name"] for row in cost["deltas"] if row["before"]["status"] == "decided"]
assert len(recovered) == 7 and len(preserved) == 5
rss = {v: statistics.median(o["peak_rss_kib"] for o in cost["versions"][v]["observations"])
       for v in ["baseline", "candidate"]}
result = dict(
    base_revision=read("cost-inputs.json")["base_revision"],
    status="kept-scoped-progress",
    scope="Repair the existing hypercurve_ui restricted-fragment output failures and retain "
          "exact in-memory sources behind editable previews. No kernel or CI Rust changes; "
          "no property, target, or crate future-work completion claim.",
    contract=[
        "If exact boundary paths exist but their geometry cannot supply editable primitives, "
        "use the existing family-preserving finite curve projection for the preview.",
        "Keep the original CurveRegion2 and each available CurvePath2. Compare editor snapshots "
        "before source reuse. Direct control, closure, loop-role, or source replacement invalidates reuse.",
        "Display segmentation carries the same exact source. A projected boundary with no exact "
        "path returns an error if extracted for topology. Native line/arc adapters reject retained "
        "higher-order paths instead of substituting display chords.",
        "The existing finite JSON/URL authoring format remains a documented lossy boundary. "
        "Exact sources are serde-skipped in-memory state. Actual edits author new geometry.",
    ],
    baseline=dict(record="baseline.json", exit_code=101, passed=1, failed=2,
                  unchanged_rust_sources=245, fixture="shared-blocker.json"),
    initial_focused=dict(record="initial-focused.json", passed=3,
                         sources="initial-focused-sources.tar.gz"),
    corrected_test_assertion=dict(record="initial-regression-assertion/manifest.json",
        reason="The initial stale-source regression incorrectly assumed an open CurvePath2 "
               "cannot be constructed. The final assertion checks the edited endpoint and "
               "open traversal, while region closure must fail. Four other regressions passed."),
    harness_build_failures="probe-build-failures/manifest.json",
    validation=dict(steps=validation, test_count=42, new_regression_count=5,
                    sources=len(frozen["source_hashes"]), sources_unchanged=True),
    regressions=[
        "A rectangle extending 2^-60 beyond x=1 retains an interior point at 1+2^-61 "
        "through display segmentation, path extraction, region reconstruction and a second Boolean.",
        "Control-handle editing replaces the source while an untouched clone preserves it.",
        "Direct disconnected and non-finite public-record edits cannot reuse a stale region/path.",
        "Moving a hole to the material bin changes exact center classification.",
        "Missing exact path provenance remains an error through display segmentation; an actual "
        "editor change authors a new path. JSON deliberately contains only finite authoring data.",
    ],
    cost=dict(record="cost/manifest.json", trials_per_adapter=5, rows_per_trial=12,
              recovered_rows=recovered, byte_identical_previews=preserved,
              candidate_exact_source_and_segmented_source_checks=60,
              identity_check_scope="Structural equality to an independently invoked exact Boolean. "
                  "A false baseline identity check alone does not prove geometric inequality; "
                  "the independent sub-f64 regression demonstrates an actual classification difference.",
              median_peak_rss_kib=rss,
              probe_binary_bytes={"baseline":before["binary"]["bytes"], "candidate":after["binary"]["bytes"]},
              interpretation="Capability retention, not a speedup. All previously successful preview "
                  "bytes are unchanged. Recovered rows have no successful matched baseline. RSS includes "
                  "untimed independent checks and is not an allocation count; probe size is not library size."),
    reused_evidence=dict(record="reused-source-audit.json", unchanged_non_demo_rust_sources=517,
        prior="../2026-10-09-hypercurve-retained-circular-area/manifest.json",
        limitation="The prior core/DRC/consumer tests, full fixtures, cost witnesses and two-board "
                   "report evidence remain historical evidence for unchanged sources. They were "
                   "not rerun here. The repository-wide pre-push matrix has not been run; no push."),
    remaining=[
        "General public exact geometry/control-point access for restricted fragments.",
        "An exact result archive and explicit provenance across JSON/shared-URL reloads.",
        "Legacy sampled/Geo adapters, display-derived orientation and broader boundary uncertainty.",
        "General integration, higher moments, selected-field and composition closure, resource bounds.",
        "All remaining scope of the 39 properties, 14 targets, and crate future-work documents.",
    ],
)
result["artifacts"] = {str(p.relative_to(record)): info(p) for p in sorted(record.rglob("*"))
                       if p.is_file() and p not in {
                           record / "manifest.json", record / "artifact-audit.json"}}
(record / "manifest.json").write_text(json.dumps(result, indent=2) + "\n")
print("manifest assembled")
