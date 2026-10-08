#!/usr/bin/env python3
"""Verify the pinned comparison import, including archive bytes and LFS payloads.

This is an integrity check, not numerical revalidation of historical claims.
Symlink targets are verified as bytes; they are never followed or executed.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import stat
import subprocess


ROOT = Path(__file__).resolve().parents[1]


def digest_file(path, algorithm="sha256", prefix=b""):
    digest = hashlib.new(algorithm)
    digest.update(prefix)
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def staged_entries(root):
    prefix = subprocess.check_output(["git", "rev-parse", "--show-prefix"], cwd=root, text=True).strip()
    records = subprocess.check_output(
        ["git", "ls-files", "--stage", "--full-name", "-z", "--", "."], cwd=root
    )
    entries = {}
    for record in records.split(b"\0"):
        if not record:
            continue
        info, path = record.split(b"\t", 1)
        mode, oid, stage = info.decode().split()
        if stage != "0":
            raise ValueError("unmerged index entry")
        entries[path.decode().removeprefix(prefix)] = (mode, oid)
    return entries


def verify(root, staged=False):
    provenance = json.loads((root / "integration/provenance.json").read_text())
    inventory = root / "integration/upstream-tree.jsonl"
    if digest_file(inventory) != provenance["inventory_sha256"]:
        raise ValueError("upstream inventory digest changed")
    adaptations = {row["path"]: row for row in provenance["adaptations"]}
    if any(path.startswith("audits/") for path in adaptations):
        raise ValueError("historical archive cannot be adapted")
    index = staged_entries(root) if staged else None
    seen = set()
    counts = {"files": 0, "symlinks": 0, "lfs_payloads": 0, "adapted_files": 0, "bytes": 0}
    failures = []
    with inventory.open() as stream:
        for line in stream:
            row = json.loads(line)
            relative = Path(row["path"])
            if relative.is_absolute() or ".." in relative.parts or row["path"] in seen:
                raise ValueError("unsafe or repeated inventory path")
            seen.add(row["path"])
            path = root / relative
            if any((root / parent).is_symlink() for parent in relative.parents):
                failures.append(row["path"] + ": symlink ancestor")
                continue
            try:
                if index is not None:
                    expected = row["git_oid"]
                    if row["path"] in adaptations:
                        expected = digest_file(path, "sha1", f"blob {path.stat().st_size}\0".encode())
                    if index.get(row["path"]) != (row["mode"], expected):
                        raise ValueError("staged blob or mode differs from verified import")
                mode = path.lstat().st_mode
                if row["mode"] == "120000":
                    if not stat.S_ISLNK(mode):
                        raise ValueError("expected symlink")
                    payload = os.fsencode(os.readlink(path))
                    digest = hashlib.sha1(f"blob {len(payload)}\0".encode() + payload).hexdigest()
                    if digest != row["git_oid"]:
                        raise ValueError("symlink target changed")
                    counts["symlinks"] += 1
                    continue
                if not stat.S_ISREG(mode):
                    raise ValueError("expected regular file")
                if bool(mode & 0o111) != (row["mode"] == "100755"):
                    raise ValueError("executable bit changed")
                size = path.stat().st_size
                if row["path"] in adaptations:
                    expected = adaptations[row["path"]]["sha256"]
                    actual = digest_file(path)
                    counts["adapted_files"] += 1
                elif "lfs" in row:
                    expected = row["lfs"]["sha256"]
                    actual = digest_file(path)
                    if size != row["lfs"]["bytes"]:
                        raise ValueError("LFS payload size changed or still a pointer")
                    counts["lfs_payloads"] += 1
                else:
                    expected = row["git_oid"]
                    actual = digest_file(path, "sha1", f"blob {size}\0".encode())
                if actual != expected:
                    raise ValueError("content digest changed")
                counts["files"] += 1
                counts["bytes"] += size
            except (OSError, ValueError) as error:
                failures.append(f"{row['path']}: {error}")
    if set(adaptations) - seen:
        raise ValueError("adaptation absent from upstream inventory")
    if failures:
        raise ValueError("\n".join(failures[:40]) + f"\n{len(failures)} integrity failures")
    return {
        "scope": "Pinned import integrity; historical numerical claims remain unverified.",
        "revision": provenance["revision"],
        "verified": counts,
        "staged_blobs_verified": staged,
        "legacy_migration_discrepancies": provenance["legacy_migration_discrepancies"],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--staged", action="store_true", help="also verify index blobs, modes, and LFS pointers")
    args = parser.parse_args()
    result = json.dumps(verify(ROOT, args.staged), indent=2) + "\n"
    print(result, end="")
    if args.output:
        args.output.write_text(result)


if __name__ == "__main__":
    main()
