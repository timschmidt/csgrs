#!/usr/bin/env python3
"""Count Cargo/child runs, including multiple rustdoc batches per crate."""
import json
from pathlib import Path
import re
import sys


def parse_phase(path):
    stdout = (path / "stdout.log").read_text()
    stderr = (path / "stderr.log").read_text()
    stack, batches, child_batches = [], [], []
    for line in stdout.splitlines():
        start = re.fullmatch(r"running (\d+) tests?", line)
        if start:
            stack.append({"declared_tests": int(start[1]), "test_names": []})
        name = re.fullmatch(r"test (.+) \.\.\. (?:ok|FAILED|ignored.*)", line)
        if name and stack:
            stack[-1]["test_names"].append(name[1])
        result = re.fullmatch(
            r"test result: (\w+)\. (\d+) passed; (\d+) failed; (\d+) ignored; .*", line
        )
        if result:
            assert stack, (path, line)
            row = stack.pop() | {"status": result[1], "passed": int(result[2]),
                                 "failed": int(result[3]), "ignored": int(result[4])}
            (child_batches if stack else batches).append(row)
    assert not stack, (path, stack)
    headers = [line.strip() for line in stderr.splitlines()
               if line.lstrip().startswith(("Running ", "Doc-tests "))]
    per_crate = {}
    crate = None
    kind = None
    next_header = 0
    previous_doc_header = None
    for batch in batches:
        names = batch.pop("test_names")
        doc_names = [re.fullmatch(r"crates/([^/]+)/.+ - .+", name) for name in names]
        doc_crate = None
        if any(doc_names):
            assert all(doc_names), (path, names)
            doc_crates = {match[1] for match in doc_names}
            assert len(doc_crates) == 1, (path, doc_crates)
            doc_crate = doc_crates.pop()
        if doc_crate is not None and previous_doc_header == f"Doc-tests {doc_crate}":
            # Rustdoc emits ordinary and compile-fail tests in separate harness
            # batches under one Cargo header. Name the crate from the actual
            # rustdoc test paths instead of consuming the following header.
            header = previous_doc_header
        else:
            assert next_header < len(headers), (path, next_header, batch, names)
            header = headers[next_header]
            next_header += 1
            if header.startswith("Doc-tests "):
                assert doc_crate == header.split()[-1] or (not names and batch["declared_tests"] == 0), (path, header, names)
                previous_doc_header = header
            else:
                assert doc_crate is None, (path, header, names)
                previous_doc_header = None
        if "Running unittests src/lib.rs" in header:
            crate = re.search(r"/deps/([a-z_]+)-", header)[1]
            kind = "unit_and_integration"
        elif header.startswith("Doc-tests "):
            crate = header.split()[-1]
            kind = "doctests"
        batch["target"] = header
        if crate is not None:
            info = per_crate.setdefault(crate, {"unit_and_integration": 0, "doctests": 0,
                                                "failed": 0, "ignored": 0})
            info[kind] += batch["passed"]
            info["failed"] += batch["failed"]
            info["ignored"] += batch["ignored"]
    assert next_header == len(headers), (path, next_header, len(headers))
    for batch in child_batches:
        batch.pop("test_names")
    return {
        "top_level_batches": len(batches),
        "top_level_passed": sum(row["passed"] for row in batches),
        "top_level_failed": sum(row["failed"] for row in batches),
        "top_level_ignored": sum(row["ignored"] for row in batches),
        "child_process_batches": len(child_batches),
        "child_process_passed": sum(row["passed"] for row in child_batches),
        "child_process_failed": sum(row["failed"] for row in child_batches),
        "child_process_ignored": sum(row["ignored"] for row in child_batches),
        "per_crate": per_crate, "batches": batches, "child_batches": child_batches,
        "ignored_test_lines": re.findall(r"^test .* \.\.\. ignored.*$", stdout, re.MULTILINE),
    }


if __name__ == "__main__":
    evidence = Path(__file__).resolve().parent
    result = parse_phase(evidence / sys.argv[1])
    out = evidence / sys.argv[2]
    assert not out.exists()
    out.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({key: value for key, value in result.items()
                      if key not in ["batches", "child_batches", "ignored_test_lines"]}))
