"""Reconstruct the exact artifact expectations of complete or referenced audits."""
import gzip
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]

def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()

def load(path):
    path = Path(path)
    with gzip.open(path, 'rt') as stream:
        audit = json.load(stream)
    if audit.get('schema_version') != 2:
        return audit['expected_artifacts_sha256'], audit['missing_artifacts_sha256']
    parent = ROOT / audit['parent_audit']
    assert sha(parent) == audit['parent_audit_sha256']
    expected, missing = load(parent)
    added = audit['added_artifacts_sha256']
    assert expected.keys().isdisjoint(added)
    expected.update(added)
    assert len(expected) == audit['expected_artifact_paths']
    assert len(missing) == audit['missing_artifact_paths']
    return expected, missing
