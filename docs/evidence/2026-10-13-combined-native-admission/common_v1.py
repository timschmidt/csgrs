"""Integrity guards for the distinct current-main/combined comparison."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
WORK = ROOT / 'target/combined-native-admission-v1'
PREP = E / 'preparation-v1'
ENV = {'CARGO_HOME': '/tmp/csgrs-properties-cargo',
       'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
       'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never'}
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2) + '\n')


def compressed(path, value):
    Path(path).write_bytes(gzip.compress((json.dumps(value, sort_keys=True) + '\n').encode(), mtime=0))


def verify(preparation, full=False):
    assert sources() == preparation['main_source_sha256']
    assert all(sha(ROOT / p) == h for p, h in preparation['pending_native_sha256'].items())
    assert all(sha(E / p) == h for p, h in preparation['inputs_sha256'].items())
    assert sha(E / 'prepare_v1.py') == preparation['runner_sha256']
    for row in preparation['historical_probes'].values():
        assert sha(ROOT / row['path']) == row['sha256']
    for label, row in preparation['variants'].items():
        path = PREP / (label + '-source-hashes.json.gz')
        assert sha(path) == row['source_hashes_sha256']
        entries = json.loads(gzip.decompress(path.read_bytes()))
        selected = entries if full else {p: h for p, h in entries.items()
                                         if p in preparation['main_source_sha256'] or p in row['delta_sha256']}
        assert all(sha(Path(row['source']) / p) == h for p, h in selected.items()), label
    parent = preparation['parent']
    assert sha(ROOT / parent['preparation']) == parent['preparation_sha256']
    if full:
        entries = json.loads(gzip.decompress((ROOT / parent['hashes']).read_bytes()))
        assert sha(ROOT / parent['hashes']) == parent['hashes_sha256']
        assert all(sha(Path(parent['source']) / p) == h for p, h in entries.items())
        dependency = ROOT / parent['dependency_hashes']
        assert sha(dependency) == parent['dependency_hashes_sha256']
        assert all(sha(p) == h for p, h in json.loads(gzip.decompress(dependency.read_bytes())).items())


def order(pair):
    return ['baseline', 'candidate'] if pair % 2 else ['candidate', 'baseline']
