"""Run formatting and ordinary Cargo checks for the integrated isolated candidate."""
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
OUT = E / 'integrated-native-validation-v2484'
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    assert not OUT.exists()
    source_record = E / 'dependency-integration-v2480/working-source-hashes.json.gz'
    source = json.loads(gzip.decompress(source_record.read_bytes()))
    spec = importlib.util.spec_from_file_location('catalog_v2484', E / 'validate_candidate_v3.py')
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    original = json.loads(gzip.decompress((E / 'candidate-preparation/source-hashes.json.gz').read_bytes()))
    assert helper.source_hashes(SOURCE, original) == source
    guards = E / 'typed-api-prototype-v2483-integrated-rejections'
    proof = E / 'typed-api-prototype-v2481-integrated-dependency-verus'
    for phase in [guards, proof, E / 'typed-api-prototype-v2482-integrated-acceptance']:
        result = read(phase / 'result.json')
        assert result['exit_code'] == 0 and result['source_unchanged'] and result['main_unchanged']
        assert sha(phase / 'source-hashes.json.gz') == sha(source_record)
    included = read(E / 'typed-api-prototype-v2437-dependency-binding-included-fmt/command.json')['command']
    included += ['crates/hyperreal/src/computable/node/approximation_bound.rs',
                 'crates/hyperreal/verification/lib.rs',
                 'crates/hyperreal/verification/approximation_bound.rs',
                 'crates/hyperreal/verification/computable_bounds.rs']
    commands = [
        ('typed-api-prototype-v2484-integrated-workspace-fmt', ['cargo', 'fmt', '--all', '--', '--check']),
        ('typed-api-prototype-v2485-integrated-included-fmt', included),
        ('typed-api-prototype-v2486-integrated-clippy',
         ['cargo', 'clippy', '--release', '-p', 'hyperreal', '-p', 'hyperlimit', '-p', 'hyperlattice',
          '-p', 'hypercurve', '-p', 'hyperdrc', '--all-targets', '--all-features', '--', '-D', 'warnings']),
        ('typed-api-prototype-v2487-integrated-scalar-tests',
         ['cargo', 'test', '--release', '-p', 'hyperreal', '--lib', '--all-features']),
    ]
    OUT.mkdir()
    protocol = {'runner_sha256': sha(Path(__file__)), 'source_record_sha256': sha(source_record),
        'proof_result_sha256': sha(proof / 'result.json'), 'guard_result_sha256': sha(guards / 'result.json'),
        'commands': commands,
        'scope': 'Sequential formatting, strict Clippy for the scalar and four consumers, and release scalar library regressions. The new local integer dependency is used in ordinary Cargo compilation. These are correctness/build checks, not matched cost, memory, size or native full-board admission.'}
    (OUT / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    rows = []
    for phase, command in commands:
        subprocess.run([sys.executable, str(E / 'validate_candidate_v3.py'), phase, '--exec', *command],
                       cwd=ROOT, check=True)
        result = read(E / phase / 'result.json')
        assert sha(E / phase / 'source-hashes.json.gz') == sha(source_record)
        passed = result['exit_code'] == 0 and result['source_unchanged'] and result['main_unchanged']
        rows.append({'phase': phase, 'result_sha256': sha(E / phase / 'result.json'),
                     'gate_passed': passed, 'result': result})
        (OUT / 'result.json').write_text(json.dumps({'protocol_sha256': sha(OUT / 'protocol.json'),
            'phases': rows, 'complete': len(rows) == len(commands) and all(r['gate_passed'] for r in rows)},
            indent=2) + '\n')
        print(json.dumps({'phase': phase, 'gate_passed': passed}), flush=True)
        if not passed:
            break


if __name__ == '__main__':
    main()
