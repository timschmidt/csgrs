"""Independently check allocator identities, complete DRC qualification and gates."""
import gzip
import hashlib
import json
from pathlib import Path
import re

from common_v2 import E, PREP, sha, verify


def main():
    out = E / 'qualification-v3'
    destination = E / 'qualification-assessment-v3.json'
    assert not destination.exists()
    prep = json.loads((PREP / 'protocol.json').read_text())
    protocol = json.loads((out / 'protocol.json').read_text())
    result = json.loads((out / 'result.json').read_text())
    binaries = json.loads((out / 'binaries.json').read_text())
    selections = json.loads((out / 'allocator-selections.json').read_text())
    assert result['complete'] and result['sources_dependencies_main_and_binaries_unchanged']
    assert sha(E / 'qualify_v3.py') == protocol['runner_sha256']
    assert sha(Path(__file__)) == protocol['assessor_sha256']
    assert sha(PREP / 'protocol.json') == protocol['preparation_sha256']
    assert prep['variants'] == protocol['variants']
    verify(prep)
    for name, digest in prep['artifact_sha256'].items():
        assert sha(PREP / name) == digest
    for variant, identity in prep['variants'].items():
        entries = json.loads(gzip.decompress((PREP / (variant + '-source-hashes.json.gz')).read_bytes()))
        source = Path(identity['source'])
        assert {str(p.relative_to(source)) for p in source.rglob('*') if p.is_file()} == set(entries)
    shared = 'crates/hyperdrc/src/allocator_contract_tests.rs'
    assert (Path(prep['variants']['baseline']['source']) / shared).read_bytes() == (Path(prep['variants']['candidate']['source']) / shared).read_bytes()
    assert protocol['variant_order'] == ['baseline', 'candidate']
    assert protocol['inherited_allocator_settings'] == []
    assert [(g['name'], g['limit_seconds'], g['metric']) for g in protocol['gates']] == [
        ('full-board', 120, 'worker CPU'), ('copper-neck', 60, 'check body wall')]
    assert len(protocol['commands']) == 8
    assert result['phases'] == json.loads((out / 'commands.json').read_text())
    phases = {p['name']: p for p in result['phases']}
    assert len(phases) == 34
    failure = json.loads((E / 'qualification-failure-v2.json').read_text())
    assert sha(E / 'qualification-failure-v2.json') == protocol['failure_sha256']
    previous = E / 'qualification-v2'
    assert all(sha(previous / name) == digest for name, digest in failure['preserved_artifacts_sha256'].items())
    assert all(sha(E / name) == digest for name, digest in failure['preserved_runners_sha256'].items())
    prior_phases = json.loads((previous / 'commands.json').read_text())
    assert len(prior_phases) == protocol['reused_commands'] == 27
    assert result['phases'][:27] == prior_phases
    expected_order = [v + '-' + label for v in ['baseline', 'candidate'] for label in [
        *[label for label, _ in protocol['commands']],
        'test-nm-demangled', 'test-nm-raw', 'test-global-alloc',
        'cli-nm-demangled', 'cli-nm-raw', 'cli-global-alloc', 'full-board', 'copper-neck']]
    expected_order.insert(expected_order.index('candidate-cli-nm-demangled'), 'candidate-test-relocations')
    expected_order.insert(expected_order.index('candidate-full-board'), 'candidate-cli-relocations')
    assert list(phases) == expected_order
    logs = {}
    for name, phase in phases.items():
        variant = phase['variant']
        assert phase['cwd'] == prep['variants'][variant]['source']
        assert phase['extra_environment'] == {'CARGO_TARGET_DIR': prep['variants'][variant]['target']}
        assert phase['command'][:4] == ['timeout', '--signal=TERM', '--kill-after=10s', '1800']
        assert phase['exit_code'] in ([0, 101] if phase['phase'] in ['full-board', 'copper-neck'] else [0])
        for stream in ['stdout', 'stderr']:
            path = out / phase[stream + '_path']
            assert sha(path) == phase[stream + '_sha256']
            raw = gzip.decompress(path.read_bytes()) if phase['compressed'] else path.read_bytes()
            assert len(raw) == phase[stream + '_bytes']
            logs[name, stream] = raw.decode()
        configured = [command for label, command in protocol['commands'] if label == phase['phase']]
        if configured:
            assert len(configured) == 1 and phase['command'][4:] == configured[0]
    tests, gates, observed_selections = [], [], []
    for variant in ['baseline', 'candidate']:
        for label in ['allocator-contracts', 'drc-tests']:
            matches = re.findall(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;', logs[variant + '-' + label, 'stdout'])
            row = {'variant': variant, 'phase': label, 'passed': sum(int(m[0]) for m in matches),
                   'failed': sum(int(m[1]) for m in matches), 'ignored': sum(int(m[2]) for m in matches),
                   'result_groups': len(matches)}
            assert tuple(row[k] for k in ['passed', 'failed', 'ignored', 'result_groups']) == (
                (2, 0, 0, 1) if label == 'allocator-contracts' else (1646, 0, 0, 10))
            tests.append(row)
        for kind in ['test', 'cli']:
            binary = binaries[variant][kind]
            path = Path(binary['path'])
            assert sha(path) == binary['sha256'] and path.stat().st_size == binary['bytes']
            assert path.is_relative_to(Path(prep['variants'][variant]['target']))
            assert Path(binary['manifest_path']) == Path(prep['variants'][variant]['source']) / 'crates/hyperdrc/Cargo.toml'
            assert binary['target']['kind'] == (['lib'] if kind == 'test' else ['bin'])
            assert bool(binary['profile']['test']) == (kind == 'test')
            events = [json.loads(line) for line in logs[variant + '-build-' + kind, 'stdout'].splitlines() if line.startswith('{')]
            artifacts = [e for e in events if e.get('reason') == 'compiler-artifact' and e.get('executable') == str(path)]
            assert len(artifacts) == 1
            artifact = artifacts[0]
            for field in ['manifest_path', 'features', 'package_id', 'target', 'profile']:
                assert binary[field] == artifact[field]
            features = {e['target']['name']: e['features'] for e in events if e.get('reason') == 'compiler-artifact'
                        and e['target']['name'] in ['mimalloc', 'libmimalloc_sys'] and e['target']['kind'] == ['lib']}
            assert features == binary['allocator_dependency_features']
            if variant == 'candidate':
                assert features == {'mimalloc': [], 'libmimalloc_sys': []}
            prefix = variant + '-' + kind
            demangled = logs[prefix + '-nm-demangled', 'stdout']
            raw = logs[prefix + '-nm-raw', 'stdout']
            assembly = logs[prefix + '-global-alloc', 'stdout']
            addresses = [line.split()[0] for line in demangled.splitlines() if line.endswith(' __rustc::__rust_alloc')]
            assert len(addresses) == 1
            symbols = [line.split()[-1] for line in raw.splitlines() if line.split()[0] == addresses[0]]
            assert len(symbols) == 1
            for suffix, command in [
                ('nm-demangled', ['nm', '-C', '--defined-only', str(path)]),
                ('nm-raw', ['nm', '--defined-only', str(path)]),
                ('global-alloc', ['objdump', '-d', '--disassemble=' + symbols[0], str(path)]),
            ]:
                assert phases[prefix + '-' + suffix]['command'][4:] == command
            target = '__rdl_alloc' if variant == 'baseline' else 'mi_malloc_aligned'
            assert ('<' + symbols[0] + '>:') in assembly
            selected = {'variant': variant, 'kind': kind, 'symbol': symbols[0],
                'binary_sha256': binary['sha256'], 'required_target_observed': target,
                'assembly_sha256': hashlib.sha256(assembly.encode()).hexdigest()}
            if variant == 'baseline':
                assert target in assembly
            else:
                assert phases[prefix + '-relocations']['command'][4:] == ['readelf', '-rW', str(path)]
                relocation_text = logs[prefix + '-relocations', 'stdout']
                with path.open('rb') as binary_file:
                    header = binary_file.read(20)
                assert header[:7] == b'\x7fELF\x02\x01\x01' and header[16:20] == b'\x03\x00\x3e\x00'
                target_symbols = [line.split() for line in demangled.splitlines() if line.endswith(' mi_malloc_aligned')]
                assert len(target_symbols) == 1
                target_address = int(target_symbols[0][0], 16)
                slots = re.findall(r'jmp\s+\*[^\n]*#\s+([0-9a-f]+)', assembly)
                if slots:
                    assert len(slots) == 1
                    entries = [line.split() for line in relocation_text.splitlines() if line.split() and line.split()[0] == slots[0].zfill(16)]
                    assert len(entries) == 1 and len(entries[0]) == 4
                    assert entries[0][2] == 'R_X86_64_RELATIVE' and int(entries[0][3], 16) == target_address
                    selected.update(dispatch='ELF64 x86-64 relative relocation', got_slot=slots[0], target_address=format(target_address, 'x'))
                else:
                    assert target in assembly
                    selected.update(dispatch='direct named target', target_address=format(target_address, 'x'))
                selected['relocations_sha256'] = hashlib.sha256(relocation_text.encode()).hexdigest()
            observed_selections.append(selected)
        for gate in protocol['gates']:
            name = variant + '-' + gate['name']
            phase = phases[name]
            assert phase['command'][4:] == ['/usr/bin/time', '-v', binaries[variant]['test']['path'], gate['test'],
                                           '--exact', '--test-threads=1', '--nocapture']
            raw = logs[name, 'stdout'] + logs[name, 'stderr']
            assert 'skipping complex project fixture test' not in raw
            passed = phase['exit_code'] == 0
            assert ('1 passed; 0 failed' if passed else '0 passed; 1 failed') in raw
            if gate['name'] == 'full-board':
                assert raw.count('finished emit report') == 2
                assert [int(v) for v in re.findall(r'finished run \((\d+) active finding', raw)] == [12, 197]
            duration = re.search(gate['failure_pattern'], raw)
            if not passed:
                assert duration and float(duration[1]) >= gate['limit_seconds']
            gates.append(gate | {'variant': variant, 'passed': passed,
                'assertion_seconds': float(duration[1]) if duration else None,
                'process_wall_seconds': phase['seconds']})
    assert tests == result['tests'] == json.loads((out / 'tests.json').read_text())
    assert gates == result['gates'] == json.loads((out / 'gates.json').read_text())
    assert observed_selections == selections == result['allocator_selections']
    candidate_passed = all(g['passed'] for g in gates if g['variant'] == 'candidate')
    assert candidate_passed == result['candidate_original_gates_passed']
    assessment = {'assessor_sha256': sha(Path(__file__)), 'commands': len(phases), 'reused_commands': 27, 'new_commands': 7, 'test_counts': tests,
        'gates': gates, 'candidate_original_gates_passed': candidate_passed,
        'allocator_selection_verified_for_cli_and_lib_test_binaries': True,
        'candidate_allocator_dependency_features_empty': True,
        'binaries': binaries, 'shared_tests_identical': True,
        'sources_dependencies_main_and_binaries_unchanged': True,
        'artifacts_sha256': {str(p.relative_to(E)): sha(p) for p in sorted(out.iterdir()) if p.is_file()},
        'next_step': ('Run the separately frozen fixed application trial.' if candidate_passed
                      else 'Reject this candidate at the unchanged original gate; do not run the conditional cost set.'),
        'limits': [
            'Focused tests overlap the full DRC suite. Independent integration/doctest drivers use their own allocator.',
            'Passing assertions do not print the check body duration; process wall time includes setup.',
            'No complete report-byte comparison or matched application cost set is established by qualification.',
            'Other platforms, the full workflow matrix and inherited scalar/native costs remain open.',
        ], 'runtime_admitted': False, 'plan_complete': False}
    destination.write_text(json.dumps(assessment, indent=2) + '\n')
    print(json.dumps({'commands': len(phases), 'reused_commands': 27, 'new_commands': 7, 'test_counts': tests, 'gates': gates,
                      'candidate_original_gates_passed': candidate_passed, 'runtime_admitted': False}))


if __name__ == '__main__':
    main()
