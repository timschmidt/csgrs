"""Load retained expectations only; unqualified history requires explicit acknowledgement."""
import gzip
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def load(path, *, allow_unqualified=False):
    with gzip.open(path, 'rt') as stream:
        record = json.load(stream)
    version = record.get('schema_version', 1)
    if version == 1:
        return record['expected_artifacts_sha256'], record['missing_artifacts_sha256']
    if version not in [2, 3]:
        raise ValueError(f'Unknown expectations schema {version}')
    if version == 3 and not allow_unqualified:
        raise ValueError('This manifest retains failed/incomplete audit history; explicitly acknowledge its unqualified scope.')
    parent_name = record.get('parent_expectations', record.get('parent_audit'))
    parent_digest = record.get('parent_expectations_sha256', record.get('parent_audit_sha256'))
    parent = ROOT / parent_name
    assert sha(parent) == parent_digest
    expected, missing = load(parent, allow_unqualified=allow_unqualified)
    added = record['added_artifacts_sha256'] if version == 2 else record['added_physical_artifacts_sha256']
    assert expected.keys().isdisjoint(added)
    expected.update(added)
    assert len(expected) == record['expected_artifact_paths']
    return expected, missing
