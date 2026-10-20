#!/usr/bin/env python3
"""Generate or verify the in-tree manifest of archived evidence records.

Generated evidence (raw logs, samples, source snapshots, audit corpora) lives
in the separate evidence repository named in docs/evidence-archive.md. This
repository keeps only docs/evidence-archive.tsv: one SHA-256 digest per record.

A record digest is the SHA-256 of a sorted listing with one line per file:

    <sha256 of content>  <type>  <path relative to the record>

Type is "-" for a regular file, "x" for an executable file and "l" for a
symbolic link, whose content is its target text. Paths use "/" and sort by
their UTF-8 bytes. Third-party package archives (*.crate) are not
redistributed. Their digests are listed as "excluded" rows so they can be
fetched again from their registry.

Usage:
    evidence_archive.py manifest REPOSITORY REVISION > docs/evidence-archive.tsv
    evidence_archive.py verify REPOSITORY REVISION [MANIFEST]

REPOSITORY is a git repository (bare or not) and REVISION a commit in it. To
verify an archive, pass the evidence repository and its recorded commit.
"""

import hashlib
from pathlib import Path
import subprocess
import sys

# Evidence roots. True: each child is its own record. False: the root is one.
ROOTS = {
    "docs/evidence": True,
    "crates/comparisons/audits": True,
    "crates/comparisons/integration": False,
    "crates/hyperbrep/docs/evidence": True,
    "crates/hyperspatial/verification": True,
    "crates/hyperreal/benchmarks/checkpoints": False,
    "benchmarks/baselines": True,
}
EXCLUDED_SUFFIX = ".crate"
HEADER = "record\tfiles\tbytes\tsha256"
KINDS = {"100644": "-", "100755": "x", "120000": "l"}


def tree_entries(repository, revision):
    output = subprocess.run(
        ["git", "-C", repository, "ls-tree", "-r", "-z", "--full-tree", revision, "--", *ROOTS],
        check=True,
        capture_output=True,
    ).stdout
    for item in output.split(b"\0"):
        if not item:
            continue
        meta, path = item.split(b"\t", 1)
        mode, kind, oid = meta.decode().split()
        if kind != "blob" or mode not in KINDS:
            raise SystemExit(f"unsupported tree entry {mode} {kind} {path.decode()}")
        yield path.decode(), KINDS[mode], oid


def blob_digests(repository, oids):
    process = subprocess.Popen(
        ["git", "-C", repository, "cat-file", "--batch"],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
    )
    digests = {}
    for oid in oids:
        process.stdin.write(f"{oid}\n".encode())
        process.stdin.flush()
        header = process.stdout.readline().split()
        size = int(header[2])
        digest = hashlib.sha256()
        remaining = size
        while remaining:
            chunk = process.stdout.read(min(remaining, 1 << 20))
            digest.update(chunk)
            remaining -= len(chunk)
        process.stdout.read(1)
        digests[oid] = (digest.hexdigest(), size)
    process.stdin.close()
    if process.wait():
        raise SystemExit("git cat-file failed")
    return digests


def record_of(path):
    for root, per_child in ROOTS.items():
        if path.startswith(root + "/"):
            if not per_child:
                return root, path[len(root) + 1 :]
            child, _, rest = path[len(root) + 1 :].partition("/")
            return f"{root}/{child}", rest or child
    raise SystemExit(f"path outside evidence roots: {path}")


def manifest(repository, revision):
    entries = list(tree_entries(repository, revision))
    digests = blob_digests(repository, sorted({oid for _, _, oid in entries}))
    grouped = {}
    for path, kind, oid in entries:
        record, relative = record_of(path)
        grouped.setdefault(record, []).append((relative.encode(), relative, kind, oid))
    rows, excluded = [HEADER], []
    for record in sorted(grouped, key=str.encode):
        listing, total, count = hashlib.sha256(), 0, 0
        for _, relative, kind, oid in sorted(grouped[record]):
            digest, size = digests[oid]
            if relative.endswith(EXCLUDED_SUFFIX):
                excluded.append(f"excluded:{record}/{relative}\t1\t{size}\t{digest}")
                continue
            listing.update(f"{digest}  {kind}  {relative}\n".encode())
            total += size
            count += 1
        rows.append(f"{record}\t{count}\t{total}\t{listing.hexdigest()}")
    return rows + excluded


def verify(repository, revision, expected_path):
    def archived(rows):
        return [row for row in rows if not row.startswith("excluded:")]

    expected = archived(expected_path.read_text().splitlines())
    actual = archived(manifest(repository, revision))
    if expected == actual:
        print(f"verified {len(expected) - 1} evidence records at {revision}")
        return 0
    for row in sorted(set(expected) - set(actual)):
        print(f"expected: {row}")
    for row in sorted(set(actual) - set(expected)):
        print(f"actual:   {row}")
    return 1


def main(argv):
    if len(argv) == 4 and argv[1] == "manifest":
        print("\n".join(manifest(argv[2], argv[3])))
        return 0
    if len(argv) in (4, 5) and argv[1] == "verify":
        default = Path(__file__).resolve().parents[1] / "docs/evidence-archive.tsv"
        return verify(argv[2], argv[3], Path(argv[4]) if len(argv) == 5 else default)
    print(__doc__, file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))
