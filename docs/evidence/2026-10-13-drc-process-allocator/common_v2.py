"""Shared integrity guards for the frozen process-allocator experiment."""
import gzip
import hashlib
import json
from pathlib import Path
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
PREP = E / 'preparation-v2'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def verify(preparation):
    assert sha(E / 'prepare_v2.py') == preparation['runner_sha256']
    for name, digest in preparation['inputs_sha256'].items():
        assert sha(E / name) == digest, name
    failure = json.loads((E / 'preparation-failure-v1.json').read_text())
    assert all(sha(E / name) == digest for name, digest in failure['preserved_inputs_sha256'].items())
    assert sources() == preparation['main_source_sha256']
    assert all(sha(ROOT / name) == digest for name, digest in preparation['pending_native_sha256'].items())
    original = json.loads(gzip.decompress((PREP / 'parent-source-hashes.json.gz').read_bytes()))
    assert sha(PREP / 'parent-source-hashes.json.gz') == preparation['parent_hashes_sha256']
    assert all(sha(Path(preparation['parent_source']) / name) == digest for name, digest in original.items())
    for name, variant in preparation['variants'].items():
        path = PREP / (name + '-source-hashes.json.gz')
        assert sha(path) == variant['source_hashes_sha256']
        entries = json.loads(gzip.decompress(path.read_bytes()))
        assert len(entries) == variant['files']
        assert all(sha(Path(variant['source']) / filename) == digest for filename, digest in entries.items())
    dependencies = PREP / 'dependency-source-hashes.json.gz'
    assert sha(dependencies) == preparation['dependency_hashes_sha256']
    assert all(sha(Path(path)) == digest for path, digest in json.loads(gzip.decompress(dependencies.read_bytes())).items())
