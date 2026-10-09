"""Audit every rendered finding, using the retained full-board order witness.

Usage: python3 audit_known_order_variants.py /tmp/affine-cost-output
This checks the current report against the previous report and both previously
reproduced marker variants. It does not rerun the native order witness or claim
that report order and original evidence IDs are deterministic.
"""
import hashlib
import json
import pathlib
import re
import runpy
import sys

record = pathlib.Path(__file__).resolve().parent
out = pathlib.Path(sys.argv[1])
old_record = record.parent / "2026-10-08-hyperdrc-stencil-points"
witness_path = old_record / "fixture-order-audit/witness.json"
manifest_path = old_record / "fixture-order-audit/manifest.json"
witness = json.loads(witness_path.read_text())
old_manifest = json.loads(manifest_path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
unchanged = {}
for name, expected in old_manifest["unchanged_production_files"].items():
    assert digest(pathlib.Path(name)) == expected, name
    unchanged[name] = expected

context = runpy.run_path(str(record / "audit_fixture_reports.py"))
assert context["audit"]["ordered_check_counts_unchanged"]


def extract_duplicate(text):
    parts = re.split(r"(?=^layer-sanity |^geometry-uncertainty |^minimum-copper-neck-width |^drill-spacing )", text, flags=re.M)
    selected = [part for part in parts if "layers appear to contain duplicate geometry" in part]
    assert len(selected) == 1
    block = selected[0]
    lines = block.splitlines()
    names = re.findall(r"\.kicad_pcb:(In1\.Cu|B\.Cu)", lines[0])
    points = json.loads(next(line.removeprefix("   locations: ") for line in lines if line.startswith("   locations: ")))
    matching = next(row for row in witness["input_order_variants"] if row["layers"] == names)
    assert matching["locations"] == points
    # Preserve severity, check, polygon/point count and area in the header;
    # only the two proven layer-order variants may vary.
    header = re.sub(r"\.kicad_pcb:(In1\.Cu|B\.Cu)", ".kicad_pcb:LAYER", lines[0])
    geometry_and_message = [line for line in lines[1:] if not line.startswith("   locations: ")]
    return (
        "".join(part for part in parts if part != block),
        header,
        geometry_and_message,
        dict(layers=names, locations=points),
    )


before = extract_duplicate(context["old_findings"])
after = extract_duplicate(context["new_findings"])
assert before[:3] == after[:3]
result = dict(
    prior_full_board_witness=str(witness_path),
    prior_full_board_witness_sha256=digest(witness_path),
    prior_full_board_manifest_sha256=digest(manifest_path),
    unchanged_order_and_marker_source_files=unchanged,
    previous_variant=before[3],
    candidate_variant=after[3],
    observed_duplicate_variant_changed=before[3] != after[3],
    all_other_rendered_findings_unchanged_after_path_id_row_normalization=True,
    duplicate_header_counts_geometry_and_message_unchanged=True,
    both_marker_lists_match_previously_reproduced_full_board_variants=True,
    scope="Current and prior full-fixture text reports match after explicit path/ID and finding-row normalization, except for the previously demonstrated duplicate-layer input-order/optional-marker variation if present. The duplicate's remaining header fields, full rendered geometry and message are checked unchanged. This reuses the retained native order witness; it does not rerun that witness or prove original report/ID determinism.",
)
(out / "known-order-audit.json").write_text(json.dumps(result, indent=2) + "\n")
print("Every rendered finding matches within the documented, previously reproduced order variants.")
