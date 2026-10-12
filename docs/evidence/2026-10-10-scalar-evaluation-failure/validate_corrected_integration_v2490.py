"""Validate the final source, formatting first, with all controls retained."""
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

sys.dont_write_bytecode = True
E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / 'target/scalar-evaluation-candidate/source'
OUT = E / 'corrected-integration-validation-v2490'
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    assert not OUT.exists()
    record = E / 'dependency-integration-correction-v2489/working-source-hashes.json.gz'
    sources = json.loads(gzip.decompress(record.read_bytes()))
    spec = importlib.util.spec_from_file_location('catalog_v2490', E / 'validate_candidate_v3.py')
    catalog = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(catalog)
    original = json.loads(gzip.decompress((E / 'candidate-preparation/source-hashes.json.gz').read_bytes()))
    assert catalog.source_hashes(SOURCE, original) == sources
    included = read(E / 'typed-api-prototype-v2485-integrated-included-fmt/command.json')['command']
    commands = [
        ('typed-api-prototype-v2490-integrated-workspace-fmt', ['cargo', 'fmt', '--all', '--', '--check']),
        ('typed-api-prototype-v2491-integrated-included-fmt', included),
        ('typed-api-prototype-v2492-integrated-verus', [sys.executable, str(E / 'run_candidate_verus.py'),
          'typed-api-prototype-v2492-integrated-verus']),
        ('typed-api-prototype-v2493-integrated-acceptance', [sys.executable, str(E / 'run_integrated_proof_checks_v2482.py'),
          'typed-api-prototype-v2493-integrated-acceptance', 'test_acceptance.py']),
        ('typed-api-prototype-v2494-integrated-clippy', ['cargo', 'clippy', '--release', '-p', 'hyperreal', '-p', 'hyperlimit',
          '-p', 'hyperlattice', '-p', 'hypercurve', '-p', 'hyperdrc', '--all-targets', '--all-features', '--', '-D', 'warnings']),
        ('typed-api-prototype-v2495-integrated-scalar-tests', ['cargo', 'test', '--release', '-p', 'hyperreal', '--lib', '--all-features']),
        ('typed-api-prototype-v2496-integrated-rejections', [sys.executable, str(E / 'run_integrated_proof_checks_v2482.py'),
          'typed-api-prototype-v2496-integrated-rejections', 'test_rejections.py']),
    ]
    OUT.mkdir()
    protocol = {'runner_sha256': sha(Path(__file__)), 'source_record_sha256': sha(record),
        'correction_protocol_sha256': sha(E / 'dependency-integration-correction-v2489/protocol.json'),
        'commands': commands, 'scope': 'Sequential final-source checks: formatting before proofs, full positive target, all acceptance tests, strict Clippy for five crates, release scalar tests and the complete rejection entry point. This does not qualify runtime cost, memory, size or main/full-board admission.'}
    (OUT / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    rows = []
    for phase, command in commands:
        subprocess.run([sys.executable, str(E / 'validate_candidate_v3.py'), phase, '--exec', *command], cwd=ROOT, check=True)
        result = read(E / phase / 'result.json')
        assert sha(E / phase / 'source-hashes.json.gz') == sha(record)
        passed = result['exit_code'] == 0 and result['source_unchanged'] and result['main_unchanged']
        rows.append({'phase': phase, 'result_sha256': sha(E / phase / 'result.json'), 'result': result, 'gate_passed': passed})
        (OUT / 'result.json').write_text(json.dumps({'protocol_sha256': sha(OUT / 'protocol.json'), 'phases': rows,
            'complete': len(rows) == len(commands) and all(r['gate_passed'] for r in rows)}, indent=2) + '\n')
        print(json.dumps({'phase': phase, 'gate_passed': passed}), flush=True)
        if not passed:
            break


if __name__ == '__main__':
    main()
