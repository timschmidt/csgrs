"""Exercise the proposed identity checker with real pinned Cargo processes."""
import gzip
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from unittest.mock import patch

sys.dont_write_bytecode = True
E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / 'target/scalar-evaluation-candidate/source'
PREVIEW = E / 'dependency-binding-candidate-v2424'
PREPARATION = E / 'dependency-binding-fixture-preparation-v2426'
OUT = E / 'dependency-binding-cargo-v2427'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
read = lambda p: json.loads(p.read_text())
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))


def files(root):
    result = {}
    for directory, children, names in os.walk(root):
        children[:] = sorted(n for n in children if n not in {'target', '.git', '__pycache__'})
        for name in sorted(names):
            path = Path(directory) / name
            assert not path.is_symlink(), path
            result[str(path.relative_to(root))] = sha(path)
    return result


def main():
    assert not OUT.exists()
    checkpoint_path = E / 'typed-api-prototype-v2386-state.json'
    assert sha(checkpoint_path) == '6dabcb25b03a7a8769c432b926b3c4a35288ed43fd172e7a46d27503d8f68cb9'
    checkpoint = read(checkpoint_path)
    sources = snapshot(E / checkpoint['source_record'])
    guard = snapshot(E / 'candidate-preparation/main-hashes.json.gz')
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    prepared = read(PREPARATION / 'result.json')
    prep_protocol = read(PREPARATION / 'protocol.json')
    assert prepared['protocol_sha256'] == sha(PREPARATION / 'protocol.json')
    work = Path(prepared['work'])
    assert files(work) == prepared['work_files_sha256']
    assert files(PREVIEW) == prep_protocol['preview_files_sha256']
    pin = read(SOURCE / 'crates/hyperreal/verification/toolchain.json')
    spec = importlib.util.spec_from_file_location('candidate_verus_dependencies', PREVIEW / 'scripts/verus_dependencies.py')
    dependencies = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(dependencies)
    OUT.mkdir()
    fixtures = []
    for fixture in prepared['fixtures']:
        root = Path(fixture['root'])
        manifest = root / 'verification/dependencies/Cargo.toml'
        cargo = ['cargo', '+' + pin['rust_toolchain']]
        metadata = [cargo + ['metadata', '--locked', '--format-version=1', '--manifest-path', str(p)]
                    for p in [manifest, root / 'Cargo.toml']]
        build = cargo + ['build', '--locked', '--manifest-path', str(manifest), '--message-format=json']
        expected = [*metadata, build, *metadata, *metadata] if fixture['expected_acceptance'] else metadata
        fixtures.append(fixture | {'expected_commands': expected})
    protocol = {
        'status': 'Frozen before invoking the checker against real Cargo fixtures',
        'runner_sha256': sha(Path(__file__)), 'preview_files_sha256': files(PREVIEW),
        'fixture_preparation_sha256': sha(PREPARATION / 'result.json'), 'fixtures': fixtures,
        'retained_checkpoint_sha256': sha(checkpoint_path), 'source_record_sha256': checkpoint['source_record_sha256'],
        'environment': prep_protocol['environment'],
        'scope': 'Run the exact proposed prepare_dependencies/check_dependency_identity functions. Instrument subprocess.run only to capture and compare the actual command sequence; every Cargo command executes normally with the pinned compiler. The matching fixture must build and bind its real path dependency. The split fixture must reject before build. No SOURCE or retained coverage changes.',
    }
    (OUT / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    os.environ.update(prep_protocol['environment'])
    real_run = subprocess.run
    results = []
    for fixture in fixtures:
        directory = OUT / fixture['name']
        directory.mkdir()
        calls = []

        def invoke(command, *arguments, **keywords):
            index = len(calls)
            assert command == fixture['expected_commands'][index], (index, command)
            started = time.monotonic()
            result = real_run(command, *arguments, **keywords)
            stdout = result.stdout if isinstance(result.stdout, str) else ''
            stderr = result.stderr if isinstance(result.stderr, str) else ''
            (directory / f'{index:02d}.stdout.log').write_text(stdout)
            (directory / f'{index:02d}.stderr.log').write_text(stderr)
            calls.append({'command': command, 'exit_code': result.returncode, 'seconds': time.monotonic() - started,
                'stdout_sha256': sha(directory / f'{index:02d}.stdout.log'),
                'stderr_sha256': sha(directory / f'{index:02d}.stderr.log'),
                'stderr_captured_by_child_call': isinstance(result.stderr, str)})
            return result

        started = time.monotonic()
        error = None
        arguments = None
        evidence = None
        with patch.object(dependencies.subprocess, 'run', side_effect=invoke):
            try:
                arguments, evidence = dependencies.prepare_dependencies(Path(fixture['root']), pin, directory / 'proof-reports')
                dependencies.check_dependency_identity(evidence)
            except RuntimeError as failure:
                error = str(failure)
        if fixture['expected_acceptance']:
            passed = error is None and evidence is not None and len(calls) == 7
            if passed:
                local = [p for p in evidence['packages'] if p['source'] is None]
                assert len(local) == 1 and local[0]['name'] == 'num-bigint'
                assert Path(local[0]['manifest_path']) == Path(fixture['root']) / 'vendor/num-bigint/Cargo.toml'
                assert 'LICENSE-MIT' in evidence['sources'][local[0]['id']]
                assert 'Cargo.toml.orig' in evidence['sources'][local[0]['id']]
                assert any('num_bigint' in n and n.endswith('.rlib') for n in evidence['artifacts'])
        else:
            passed = error is not None and error.startswith('Proof dependency does not match production resolution:') and len(calls) == 2
            assert not (directory / 'proof-reports').exists()
        assert all(row['exit_code'] == 0 for row in calls)
        data = {'name': fixture['name'], 'gate_passed': passed, 'seconds': time.monotonic() - started,
            'expected_acceptance': fixture['expected_acceptance'], 'error': error,
            'calls': calls, 'verus_arguments': arguments,
            'dependency_report_sha256': sha(directory / 'proof-reports/dependencies.json') if evidence is not None else None}
        (directory / 'result.json').write_text(json.dumps(data, indent=2) + '\n')
        results.append(data)
        assert files(work) == prepared['work_files_sha256']
        assert files(PREVIEW) == prep_protocol['preview_files_sha256']
        assert all(sha(SOURCE / n) == h for n, h in sources.items())
        assert all(sha(ROOT / n) == h for n, h in guard.items())
        (OUT / 'result.json').write_text(json.dumps({'protocol_sha256': sha(OUT / 'protocol.json'),
            'runs': results, 'complete': len(results) == len(fixtures) and passed,
            'source_unchanged': True, 'main_unchanged': True, 'preview_unchanged': True,
            'fixtures_unchanged': True, 'retained_coverage_changed': False, 'runtime_admitted': False}, indent=2) + '\n')
        assert passed, error
        print(json.dumps({'fixture': fixture['name'], 'gate_passed': passed, 'actual_cargo_calls': len(calls), 'error': error}), flush=True)


if __name__ == '__main__':
    main()
