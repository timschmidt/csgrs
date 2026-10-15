"""Compare the complete failed snapshot bytes on starting and candidate sources."""
import gzip
import hashlib
import io
import json
from pathlib import Path
import re
import zipfile

E = Path(__file__).resolve().parent
ROOT = E.parents[2]


def sha(data):
    return hashlib.sha256(data).hexdigest()


def failures(path):
    result = {}
    for part in path.read_text().split('---- ')[1:]:
        if ' stdout ----' not in part:
            continue
        name = part.split(' stdout ----')[0]
        rows = re.findall(r'^\s*(?:left|right): (\[[^\n]+\])', part, re.M)
        if len(rows) == 2:
            result[name] = [bytes(json.loads(row)) for row in rows]
    return result


def main():
    out = E / 'downstream-comparison-v1'
    out.mkdir(exist_ok=False)
    inputs = [E / 'downstream-v1/hypercircuit-tests.stdout',
              E / 'downstream-remainder-v2/workflow.stdout',
              E / 'downstream-baseline-v1/downstream-failures.stdout']
    candidate = failures(inputs[0]) | failures(inputs[1])
    baseline = failures(inputs[2])
    assert len(candidate) == len(baseline) == 7 and candidate == baseline
    source = ROOT / 'target/planar-operation-failures/downstream-baseline-source-v1'
    hashes = json.loads(gzip.decompress((E / 'downstream-baseline-source-v1.json.gz').read_bytes()))
    assert all(sha((source / name).read_bytes()) == digest for name, digest in hashes.items())
    rows = {}
    for name, pair in sorted(candidate.items()):
        rows[name] = {'actual_bytes': len(pair[0]), 'expected_bytes': len(pair[1]),
                      'actual_sha256': sha(pair[0]), 'expected_sha256': sha(pair[1]),
                      'baseline_candidate_actual_and_expected_identical': True}
        for label, data in zip(['actual', 'expected'], pair):
            suffix = 'zip' if name.startswith('default_manufacturing') else 'json'
            (out / f'{name}-{label}.{suffix}').write_bytes(data)
        if name.startswith('default_manufacturing'):
            left, right = [zipfile.ZipFile(io.BytesIO(data)) for data in pair]
            assert left.namelist() == right.namelist()
            changed = [path for path in left.namelist() if left.read(path) != right.read(path)]
            assert changed == ['evidence/hyperdrc.json', 'manufacturing-release.json']
            rows[name]['archive_members_changed_against_old_fixture'] = changed
    result = {
        'runner_sha256': sha(Path(__file__).read_bytes()),
        'inputs_sha256': {str(path.relative_to(ROOT)): sha(path.read_bytes()) for path in inputs},
        'baseline_sources_unchanged': len(hashes), 'failures': rows,
        'conclusion': 'All seven failing actual outputs and expected fixtures match byte-for-byte on the reconstructed starting revision and the candidate. They are pre-existing failures with the unchanged 35 pending native files. No snapshot is updated and neither broad downstream trial is relabeled as passing. Geometry/materialization/fabrication checks retain their individual results. Manufacturing archive reproducibility assertions preceding its golden-fixture comparison passed.',
    }
    (out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({'same_failures': len(rows), 'baseline_sources_unchanged': len(hashes)}))


if __name__ == '__main__':
    main()
