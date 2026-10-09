"""Summarize the frozen validation and compare the displayed smoke reports."""
import hashlib
import json
import re
from pathlib import Path

here = Path(__file__).resolve().parent
previous = here.parent / "current-correctness-candidate"


def reports(path):
    text = path.read_text()
    return [
        re.sub(r"/tmp/hyperdrc-complex-project-kicad-smoke-suite-\d+/", "<SMOKE_INPUT>/", part).splitlines()
        for part in re.findall(
            r"hyperdrc: starting emit report\n(.*?)\nhyperdrc: finished emit report", text, re.S
        )
    ]


before = reports(previous / "full-board-smoke.log")
after = reports(here / "full-board-smoke.log")
assert len(before) == len(after) == 2
comparison = []
for name, left, right in zip(("CPArti FPGA", "HVP109A"), before, after):
    assert len(left) == len(right)
    changed = [
        dict(line=i + 1, before=a, after=b)
        for i, (a, b) in enumerate(zip(left, right)) if a != b
    ]
    ids = [r for r in changed if r["before"].startswith("   id: ") and r["after"].startswith("   id: ")]
    other = [r for r in changed if r not in ids]
    comparison.append(dict(
        board=name,
        finding_count=int(right[0].split()[0]),
        line_counts=[len(left), len(right)],
        before_sha256=hashlib.sha256("\n".join(left).encode()).hexdigest(),
        after_sha256=hashlib.sha256("\n".join(right).encode()).hexdigest(),
        changed_ids=ids, other_differences=other,
        all_other_displayed_lines_identical=not other,
    ))
(here / "smoke-report-comparison.json").write_text(json.dumps(dict(
    scope="Displayed reports only; not complete JSON, output-size, provenance, or checksum identity.",
    path_normalization=r"/tmp/hyperdrc-complex-project-kicad-smoke-suite-\d+/ -> <SMOKE_INPUT>/",
    runs=[previous.name, here.name], reports=comparison,
), indent=2) + "\n")

statuses = json.loads((here / "status.json").read_text())
assert len(statuses) == 33
assert all(r["exit_code"] == 0 for r in statuses[:-1])
assert statuses[-1]["name"] == "full-board-smoke" and statuses[-1]["exit_code"] == 101
passed = ignored = 0
for r in statuses[:-1]:
    log = (here / (r["name"] + ".log")).read_text()
    for match in re.finditer(r"test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;", log):
        assert match[2] == "0"
        passed += int(match[1])
        ignored += int(match[3])
board = (here / "full-board-smoke.log").read_text()
assert "skipping complex project fixture test" not in board
worker = float(re.search(r"smoke suite took ([0-9.]+)s of worker time", board)[1])
timing = (here / "full-board-smoke-time.log").read_text()
def value(label):
    return re.search(re.escape(label) + r":\s*([^\n]+)", timing)[1]
source = json.loads((here / "sources.json").read_text())
(here / "decision.json").write_text(json.dumps(dict(
    decision="The ownership-copy trial passes focused validation but the combined native-stroke runtime gate still fails; isolated production validation and admission remain pending.",
    production_baseline_commit=source["base_revision"],
    committed_fixes_included=["7a9771f4f", "86b6212d9"],
    passed_test_invocations=passed,
    existing_ignored=ignored,
    note="Focused filters overlap; invocations are not a count of distinct tests. This is not the complete workflow validation matrix.",
    successful_commands=len(statuses)-1,
    worker_cpu_seconds=worker,
    limit_seconds=120,
    process_user_seconds=float(value("User time (seconds)")),
    process_system_seconds=float(value("System time (seconds)")),
    wall_time=value("Elapsed (wall clock) time (h:mm:ss or m:ss)"),
    maximum_rss_kib=int(value("Maximum resident set size (kbytes)")),
    profiler_attached=False,
    frozen_files=len(source["source_hashes"]),
    source_unchanged=source["unchanged"],
    displayed_findings=[r["finding_count"] for r in comparison],
    changed_displayed_ids=sum(len(r["changed_ids"]) for r in comparison),
    all_other_displayed_lines_identical=all(r["all_other_displayed_lines_identical"] for r in comparison),
    report_limit="Compared with the preceding provisional candidate with the retained correctness fixes, not the production baseline. Production has 201 HVP findings; the native-source candidate has 197. Independent audit and full JSON replay remain pending.",
), indent=2) + "\n")
print(passed, "passing invocations;", ignored, "ignored;", worker, "worker CPU seconds")
