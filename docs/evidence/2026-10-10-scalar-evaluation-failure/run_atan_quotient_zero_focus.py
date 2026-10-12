"""Validate the quotient guard sequentially before any cost measurements."""
import gzip
import hashlib
import json
from pathlib import Path
import subprocess
import sys

from parse_test_results_v3 import parse_phase

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
results = {}


def validate(phase, *args):
    assert not (E/phase).exists()
    subprocess.run([sys.executable, str(E/'validate_candidate_v2.py'), phase, *args], cwd=ROOT, check=True)
    result = read(E/phase/'result.json')
    assert result['exit_code'] == 0 and result['source_unchanged'] and result['main_unchanged']
    assert sha(E/phase/'source-hashes.json.gz') == sha(E/'typed-api-prototype-v1542-atan-quotient-zero-build/source-hashes.json.gz')
    results[phase] = result
    return E/phase


def main():
    build = validate('typed-api-prototype-v1542-atan-quotient-zero-build',
                     'test', '--release', '-p', 'hyperreal', '--lib', '--all-features', '--no-run')
    subprocess.run([sys.executable, str(E/'validate_atan_quotient_zero.py')], cwd=ROOT, check=True)
    units = validate('typed-api-prototype-v1552-atan-quotient-zero-unit-tests',
                     'test', '--release', '-p', 'hyperreal', '--lib', '--all-features')
    summary = parse_phase(units)
    assert summary['top_level_passed'] == 1063 and summary['child_process_passed'] == 1
    assert summary['top_level_failed'] == summary['child_process_failed'] == 0
    out = E/'typed-api-prototype-v1552-test-summary.json'
    assert not out.exists(); out.write_text(json.dumps(summary, indent=2)+'\n')
    validate('typed-api-prototype-v1553-atan-quotient-zero-clippy', 'clippy', '--release',
             '-p', 'hyperreal', '-p', 'hyperlimit', '-p', 'hyperlattice', '-p', 'hypercurve', '-p', 'hyperdrc',
             '--all-targets', '--all-features', '--', '-D', 'warnings')
    validate('typed-api-prototype-v1564-atan-quotient-zero-workspace-fmt', 'fmt', '--all', '--', '--check')
    source = json.loads(gzip.decompress((build/'source-hashes.json.gz').read_bytes()))
    original = json.loads(gzip.decompress((E/'candidate-preparation/source-hashes.json.gz').read_bytes()))
    paths = sorted(p for p, h in source.items() if p.endswith('.rs') and original.get(p) != h)
    assert len(paths) == 107
    validate('typed-api-prototype-v1565-atan-quotient-zero-included-fmt', '--exec', 'rustfmt', '--edition', '2024',
             '--config-path', 'crates/.rustfmt.toml', '--config', 'skip_children=true', '--check', *paths)
    validate('typed-api-prototype-v1554-atan-quotient-zero-release-build',
             'build', '--release', '-p', 'hyperreal', '-p', 'hypercurve', '--all-features')
    out = E/'atan-quotient-zero-focus-v1565.json'
    assert not out.exists()
    out.write_text(json.dumps({'source_record_sha256': sha(build/'source-hashes.json.gz'),
        'runner_sha256': sha(Path(__file__)), 'cap_record': 'atan-quotient-zero-caps-v1551.json',
        'test_summary': 'typed-api-prototype-v1552-test-summary.json', 'included_rustfmt_files': len(paths),
        'validations': results, 'status': 'Focused checks passed; cost evidence still required'}, indent=2)+'\n')
    print('Quotient-guard focused checks completed; no overlapping workloads.', flush=True)


if __name__ == '__main__':
    main()
