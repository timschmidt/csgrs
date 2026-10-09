"""Audit fixture text deltas against the fresh baseline and repaired witnesses.

Usage: python3 audit_fixture_order_repair.py /tmp/layer-order-cost /tmp/layer-order-reports
The separate complete-report replay compares raw bytes without normalization.
This audit compares the prior and candidate smoke text using explicit path/ID
and row normalization and verifies the intentional duplicate marker selection.
"""
import hashlib
import json
import pathlib
import re
import runpy
import sys

record = pathlib.Path(__file__).resolve().parent
out = pathlib.Path(sys.argv[1])
replay = pathlib.Path(sys.argv[2])
before_path = record / "baseline-full-board/witness.json"
after_path = replay / "witness.json"
before = json.loads(before_path.read_text())["input_order_variants"]
after = json.loads(after_path.read_text())["input_order_results"]
canonical = next(row for row in before if row["layers"] == ["B.Cu", "In1.Cu"])
assert after == [canonical, canonical]
context = runpy.run_path(str(record / "audit_fixture_reports.py"))
assert context["audit"]["ordered_check_counts_unchanged"]


def duplicate(text, candidates):
    parts = re.split(r"(?=^layer-sanity |^geometry-uncertainty |^minimum-copper-neck-width |^drill-spacing )", text, flags=re.M)
    selected = [part for part in parts if "layers appear to contain duplicate geometry" in part]
    assert len(selected) == 1
    block = selected[0]
    lines = block.splitlines()
    names = re.findall(r"\.kicad_pcb:(In1\.Cu|B\.Cu)", lines[0])
    points = json.loads(next(line.removeprefix("   locations: ") for line in lines if line.startswith("   locations: ")))
    matching = next(row for row in candidates if row["layers"] == names)
    assert matching["locations"] == points
    header = re.sub(r"\.kicad_pcb:(In1\.Cu|B\.Cu)", ".kicad_pcb:LAYER", lines[0])
    geometry_and_message = [line for line in lines[1:] if not line.startswith("   locations: ")]
    return ("".join(part for part in parts if part != block), header,
            geometry_and_message, dict(layers=names, locations=points))


previous = duplicate(context["old_findings"], before)
candidate = duplicate(context["new_findings"], after)
assert previous[:3] == candidate[:3]
assert candidate[3]["layers"] == ["B.Cu", "In1.Cu"]
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
result = dict(
    baseline_witness_sha256=digest(before_path), candidate_witness_sha256=digest(after_path),
    previous_variant=previous[3], candidate_variant=candidate[3],
    observed_duplicate_variant_changed=previous[3] != candidate[3],
    all_other_rendered_findings_unchanged_after_path_id_row_normalization=True,
    duplicate_header_counts_geometry_and_message_unchanged=True,
    candidate_complete_witness_equals_canonical_baseline_variant=True,
    scope="Every prior and candidate rendered finding is compared after explicit path/ID/row normalization. The sole content difference is the duplicate-layer label order and its authored optional markers: the candidate always uses the lexical B.Cu first source, exactly matching that fresh baseline witness. Two-process complete serialized-report identity is recorded separately with no normalization. No universal determinism or path-independent ID claim.",
)
(out / "order-repair-audit.json").write_text(json.dumps(result, indent=2) + "\n")
print("All other rendered findings match; duplicate markers use the canonical baseline source.")
