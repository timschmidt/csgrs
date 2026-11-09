#!/usr/bin/env python3
"""Collect every pinned SCAD case using Manifold and the native worker.

Completion means outcome collection, not source conformance. Reference exports
are adopted models with no established error bound to the original source.
"""
import argparse
import json
import resource
from pathlib import Path
import subprocess
import sys

import native


def load_manifest(path, root, expected_count):
    manifest = json.loads(path.read_text())
    files = manifest["files"]
    if manifest["count"] != expected_count or len(files) != expected_count:
        raise ValueError("corpus count differs from the declared qualification envelope")
    selected = set()
    for item in files:
        relative = Path(item["path"])
        if relative.is_absolute() or ".." in relative.parts or relative.suffix != ".scad":
            raise ValueError("corpus paths must be relative SCAD paths inside the source root")
        source = (root / relative).resolve()
        if not source.is_relative_to(root) or item["path"] in selected:
            raise ValueError("escaped or duplicate corpus source")
        selected.add(item["path"])
        if native.digest(source) != item["sha256"]:
            raise ValueError(f"source hash changed: {item['path']}")
    discovered = {x.relative_to(root).as_posix() for x in root.rglob("*.scad") if x.is_file()}
    if selected != discovered:
        raise ValueError("the manifest must enumerate the entire source corpus without a skiplist")
    return dict(manifest, root=str(root))


def positive(value):
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return parsed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=Path(__file__).with_name("corpus.json"))
    parser.add_argument("--corpus-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--openscad", type=Path, required=True)
    parser.add_argument("--worker", type=Path, required=True)
    parser.add_argument("--build-receipt", type=Path)
    parser.add_argument("--expected-count", type=positive, default=524)
    parser.add_argument("--expected-pin", default="8e8df3ddae295e8667cbc9f6c4fbb7c88b0ec45b")
    parser.add_argument("--workers", type=positive, default=2)
    parser.add_argument("--case-seconds", type=positive, default=300,
                        help="external wall containment for each engine; not a geometry budget")
    parser.add_argument("--address-space-bytes", type=positive, default=native.COLLECTOR.ADDRESS_SPACE_BYTES,
                        help="per-process virtual-address limit inherited by descendants; not an aggregate RSS/heap bound")
    args = parser.parse_args()
    if not Path("/usr/bin/prlimit").is_file():
        raise ValueError("Linux prlimit is required for external memory containment")
    _, hard = resource.getrlimit(resource.RLIMIT_AS)
    if hard != resource.RLIM_INFINITY and args.address_space_bytes > hard:
        raise ValueError("requested address-space limit exceeds the inherited hard limit")
    root = args.corpus_root.resolve()
    manifest_path = args.manifest.resolve()
    manifest = load_manifest(manifest_path, root, args.expected_count)
    revision = subprocess.check_output(["git", "-C", str(root), "rev-parse", "HEAD"], text=True).strip()
    dirty = subprocess.check_output(["git", "-C", str(root), "status", "--porcelain"], text=True)
    if revision != args.expected_pin or manifest.get("pin") != revision or dirty:
        raise ValueError("corpus and assets must be clean at the declared source pin")
    backend, worker = args.openscad.resolve(), args.worker.resolve()
    if not backend.is_file() or not worker.is_file():
        raise ValueError("both engine executables are required")
    if args.build_receipt and not args.build_receipt.is_file():
        raise ValueError("declared build receipt does not exist")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    collector = native.COLLECTOR
    collector.MANIFEST = native.MANIFEST = manifest
    collector.MANIFEST_PATH = native.MANIFEST_PATH = manifest_path
    collector.WORKERS = native.WORKERS = args.workers
    collector.CASE_SECONDS = native.CASE_SECONDS = args.case_seconds
    collector.ADDRESS_SPACE_BYTES = args.address_space_bytes
    collector.BACKEND = backend
    collector.OUT = output / "reference"
    collector.ENVIRONMENT = {"XDG_CONFIG_HOME": str(output / "backend-config"),
                             "XDG_CACHE_HOME": str(output / "backend-cache")}
    native.WORKER = worker
    native.OUT = output / "native"
    native.REFERENCE_RESULTS = collector.OUT / "results.json"
    native.BUILD_RECEIPT = args.build_receipt.resolve() if args.build_receipt else None
    collector.save(output / "inputs.json", {
        "manifest_sha256": collector.digest(manifest_path), "source_pin": revision,
        "openscad_executable": str(backend), "openscad_executable_sha256": collector.digest(backend),
        "worker": str(worker), "worker_sha256": collector.digest(worker),
        "external_seconds_per_engine_case": args.case_seconds, "workers": args.workers,
        "external_address_space_bytes_per_process": args.address_space_bytes,
        "aggregate_heap_rss_or_total_work_bound": None,
        "qualification": "Executable hashes do not attribute linked runtime dependencies or source builds. Record the verified distribution and build provenance separately."})
    collector.main()
    if collector.STOP.is_set():
        raise RuntimeError("reference collection was cancelled; native evaluation was not started")
    native.main()
    if collector.STOP.is_set():
        raise RuntimeError("native collection was cancelled")


if __name__ == "__main__":
    try:
        main()
    except (ValueError, OSError, RuntimeError, subprocess.SubprocessError) as error:
        print(str(error), file=sys.stderr)
        raise SystemExit(1) from error
