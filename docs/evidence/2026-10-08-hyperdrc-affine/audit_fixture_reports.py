"""Compare full fixture text reports after explicit path/ID normalization.

Usage: python3 audit_fixture_reports.py /tmp/affine-cost-output
The previous stencil record supplies the baseline log. IDs hash temporary paths,
so IDs and runtime logging are excluded; coordinates, messages, geometry, order
and finding counts remain in the comparison. A delta is recorded, not approved.
"""
import difflib
import hashlib
import json
import pathlib
import re
import sys

record = pathlib.Path(__file__).resolve().parent
out = pathlib.Path(sys.argv[1])
previous = record.parent / "2026-10-08-hyperdrc-stencil-points/cost/fixture.log"
candidate = out / "fixture.log"


def report(log):
    lines = []
    active = False
    for line in log.splitlines():
        if re.match(r"\d+ violation\(s\) found", line):
            active = True
        if line.startswith("hyperdrc: finished emit report"):
            active = False
        if active and not line.startswith("   id:"):
            lines.append(re.sub(r"/tmp/hyperdrc-fixture-smoke-\d+", "FIXTURE_ROOT", line))
    return "\n".join(lines) + "\n"


old, new = previous.read_text(), candidate.read_text()
counts = lambda log: re.findall(r"finished check ([^ ]+) \((\d+) finding\(s\)", log)
old_report, new_report = report(old), report(new)
assert old_report.strip() and new_report.strip()


def finding_multiset(text):
    summaries, findings, current = [], [], []
    for line in text.splitlines():
        if re.match(r"\d+ violation\(s\) found", line):
            if current:
                findings.append("\n".join(current))
                current = []
            summaries.append(line)
        elif re.match(r"\d+\. ", line):
            if current:
                findings.append("\n".join(current))
            current = [re.sub(r"^\d+\. ", "", line)]
        elif current:
            current.append(line)
    if current:
        findings.append("\n".join(current))
    return "\n".join(summaries + sorted(findings)) + "\n"


old_findings, new_findings = finding_multiset(old_report), finding_multiset(new_report)


def duplicate_markers(text):
    result, points, header = [], {}, None
    duplicate = False
    for line in text.splitlines():
        if line and not line.startswith(" "):
            header = line
            duplicate = False
        if "layer contains duplicate polygon island geometry" in line:
            duplicate = True
        if duplicate and line.startswith("   locations: "):
            locations = json.loads(line.removeprefix("   locations: "))
            assert header not in points
            points[header] = locations
            line = "   locations: " + json.dumps(sorted(locations))
        result.append(line)
    return "\n".join(result) + "\n", points


old_content, old_markers = duplicate_markers(old_findings)
new_content, new_markers = duplicate_markers(new_findings)
marker_changes = []
for key in sorted(old_markers.keys() | new_markers.keys()):
    before, after = old_markers.get(key), new_markers.get(key)
    if before != after:
        marker_changes.append(dict(finding=key, before=before, after=after,
            exact_coordinate_multiset_preserved=before is not None and after is not None and sorted(before) == sorted(after)))
changes = "".join(difflib.unified_diff(old_report.splitlines(True), new_report.splitlines(True), fromfile="previous", tofile="candidate"))
(out / "fixture-report.diff").write_text(changes)
finding_changes = "".join(difflib.unified_diff(old_findings.splitlines(True), new_findings.splitlines(True), fromfile="previous-findings", tofile="candidate-findings"))
(out / "fixture-findings.diff").write_text(finding_changes)
info = lambda text: dict(bytes=len(text.encode()), sha256=hashlib.sha256(text.encode()).hexdigest())
audit = dict(
    previous_log=str(previous), candidate_log=str(candidate),
    previous_log_sha256=hashlib.sha256(previous.read_bytes()).hexdigest(),
    candidate_log_sha256=hashlib.sha256(candidate.read_bytes()).hexdigest(),
    previous_check_counts=counts(old), candidate_check_counts=counts(new),
    ordered_check_counts_unchanged=counts(old) == counts(new),
    normalized_reports_unchanged=old_report == new_report,
    previous_normalized=info(old_report), candidate_normalized=info(new_report),
    diff_lines=len(changes.splitlines()),
    finding_multisets_unchanged=old_findings == new_findings,
    previous_finding_multiset=info(old_findings), candidate_finding_multiset=info(new_findings),
    finding_diff_lines=len(finding_changes.splitlines()),
    duplicate_marker_changes=marker_changes,
    all_content_preserved_except_finding_and_duplicate_marker_order=old_content == new_content,
    previous_content=info(old_content), candidate_content=info(new_content),
    normalization="Keep every rendered finding row, message, location and polygon coordinate in order; replace fixture temporary root paths with FIXTURE_ROOT, remove only ID lines (which hash temporary paths), and exclude runtime/progress logging outside rendered reports. This does not establish identity of unrendered serialized fields or original IDs.",
    finding_multiset_normalization="Additionally remove finding ordinal numbers and sort complete finding blocks. Report totals stay in report order; all coordinates, locations, messages and ordering within each finding remain unchanged. This separates input/report iteration order from content; it does not claim original report order is stable.",
    duplicate_marker_normalization="Additionally sort coordinates only in duplicate-island finding marker lists. Changed lists and their exact before/after coordinates are recorded separately. No geometric tolerance or coordinate rounding is used; every other within-finding byte remains in the comparison.",
)
(out / "fixture-report-audit.json").write_text(json.dumps(audit, indent=2) + "\n")
print(json.dumps({k: audit[k] for k in ["ordered_check_counts_unchanged", "normalized_reports_unchanged", "diff_lines", "finding_multisets_unchanged", "finding_diff_lines", "all_content_preserved_except_finding_and_duplicate_marker_order", "duplicate_marker_changes"]}, indent=2))
