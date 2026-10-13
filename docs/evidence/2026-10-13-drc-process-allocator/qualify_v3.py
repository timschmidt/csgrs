"""Qualify process allocator ownership and both original native gates serially."""
import gzip
import json
import os
from pathlib import Path
import re
import subprocess
import shutil
import time

from common_v2 import E, PREP, ROOT, sha, verify

GATES = [
    {'name': 'full-board', 'test': 'zz_complex_project_performance_tests::kicad_board_completes_smoke_check_suite',
     'limit_seconds': 120, 'metric': 'worker CPU', 'failure_pattern': r'KiCad smoke suite took ([0-9.]+)s of worker time'},
    {'name': 'copper-neck', 'test': 'zz_complex_project_performance_tests::min_copper_neck_width_completes_on_copper_layers',
     'limit_seconds': 60, 'metric': 'check body wall', 'failure_pattern': r'complex project copper neck regression fixture took ([0-9.]+)s'},
]


def main():
    prep = json.loads((PREP / 'protocol.json').read_text())
    out = E / 'qualification-v3'
    out.mkdir(exist_ok=False)
    previous = E / 'qualification-v2'
    failure = json.loads((E / 'qualification-failure-v2.json').read_text())
    for name, digest in failure['preserved_artifacts_sha256'].items():
        assert sha(previous / name) == digest
        if name != 'protocol.json':
            shutil.copyfile(previous / name, out / name)
    inherited_allocator_keys = sorted(k for k in os.environ if k.startswith(('MIMALLOC_', 'MI_'))
                                      or k in ['LD_PRELOAD', 'MALLOC_CONF', 'GLIBC_TUNABLES', 'MALLOC_ARENA_MAX'])
    assert not inherited_allocator_keys, inherited_allocator_keys
    env = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
           'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never', 'RUSTDOCFLAGS': '-D warnings'}
    skips = [arg for gate in GATES for arg in ['--skip', gate['test']]]
    commands = [
        ('format', ['cargo', 'fmt', '--all', '--', '--check']),
        ('clippy-default', ['cargo', 'clippy', '--locked', '--release', '-p', 'hyperdrc', '--all-targets', '--', '-D', 'warnings']),
        ('clippy-all', ['cargo', 'clippy', '--locked', '--release', '-p', 'hyperdrc', '--all-features', '--all-targets', '--', '-D', 'warnings']),
        ('allocator-contracts', ['cargo', 'test', '--locked', '--release', '-p', 'hyperdrc', '--all-features', '--lib', 'allocator_contract_tests', '--', '--test-threads=1']),
        ('drc-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperdrc', '--all-features', '--', '--test-threads=2', *skips]),
        ('drc-docs', ['cargo', 'doc', '--locked', '-p', 'hyperdrc', '--all-features', '--no-deps']),
        ('build-test', ['cargo', 'test', '--locked', '--release', '-p', 'hyperdrc', '--all-features', '--lib', '--no-run', '--message-format=json']),
        ('build-cli', ['cargo', 'build', '--locked', '--release', '-p', 'hyperdrc', '--all-features', '--bin', 'hyperdrc', '--message-format=json']),
    ]
    protocol = {'scope': __doc__, 'runner_sha256': sha(Path(__file__)),
                'assessor_sha256': sha(E / 'assess_qualification_v3.py'),
                'preparation_sha256': sha(PREP / 'protocol.json'),
                'failure_sha256': sha(E / 'qualification-failure-v2.json'),
                'reused_commands': 27,
                'commands': commands, 'gates': GATES, 'variants': prep['variants'],
                'variant_order': ['baseline', 'candidate'], 'environment': env,
                'compiler': subprocess.check_output(['rustc', '-vV'], text=True),
                'cc': subprocess.check_output(['cc', '--version'], text=True),
                'inherited_rustflags': os.environ.get('RUSTFLAGS'),
                'inherited_allocator_settings': inherited_allocator_keys,
                'timeout_seconds': 1800,
                'limits': 'Gate process wall durations include setup and are not the internal assertion metric. Passing assertions do not print their measured duration. DRC lib unit tests and CLI use the selected process allocator; independently linked integration/doctest drivers keep their own allocator.',
                'runtime_admitted': False}
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    phases = json.loads((previous / 'commands.json').read_text())
    tests = json.loads((previous / 'tests.json').read_text())
    binaries = json.loads((previous / 'binaries.json').read_text())
    selections = json.loads((previous / 'allocator-selections.json').read_text())
    gates = json.loads((previous / 'gates.json').read_text())
    print(json.dumps({'reusing_verified_commands': len(phases), 'new_commands_expected': 7}), flush=True)

    def guards():
        verify(prep)
        assert sha(E / 'qualification-failure-v2.json') == protocol['failure_sha256']
        assert all(sha(previous / name) == digest for name, digest in failure['preserved_artifacts_sha256'].items())
        assert all(sha(E / name) == digest for name, digest in failure['preserved_runners_sha256'].items())
        assert sha(Path(__file__)) == protocol['runner_sha256']
        assert sha(E / 'assess_qualification_v3.py') == protocol['assessor_sha256']
        assert sha(PREP / 'protocol.json') == protocol['preparation_sha256']
        assert all(sha(Path(binary['path'])) == binary['sha256'] for group in binaries.values() for binary in group.values())

    def run(variant, label, command, allow_gate_failure=False, compressed=False):
        guards()
        name = variant + '-' + label
        cwd = Path(prep['variants'][variant]['source'])
        extra = {'CARGO_TARGET_DIR': prep['variants'][variant]['target']}
        wrapped = ['timeout', '--signal=TERM', '--kill-after=10s', '1800', *command]
        prior = [row for row in phases if row['name'] == name]
        if prior:
            assert len(prior) == 1 and prior[0]['command'] == wrapped
            record = prior[0]
            raw = {}
            for stream in ['stdout', 'stderr']:
                path = out / record[stream + '_path']
                assert sha(path) == record[stream + '_sha256']
                raw[stream] = gzip.decompress(path.read_bytes()) if record['compressed'] else path.read_bytes()
            return subprocess.CompletedProcess(wrapped, record['exit_code'], raw['stdout'], raw['stderr'])
        print(json.dumps({'start': name}), flush=True)
        start = time.monotonic()
        result = subprocess.run(wrapped, cwd=cwd, env=os.environ | env | extra, capture_output=True)
        elapsed = time.monotonic() - start
        record = {'name': name, 'variant': variant, 'phase': label, 'command': wrapped,
                  'cwd': str(cwd), 'extra_environment': extra, 'exit_code': result.returncode, 'seconds': elapsed,
                  'compressed': compressed}
        for stream in ['stdout', 'stderr']:
            raw = getattr(result, stream)
            path = out / (name + '.' + stream + ('.gz' if compressed else ''))
            path.write_bytes(gzip.compress(raw, mtime=0) if compressed else raw)
            record[stream + '_path'] = path.name
            record[stream + '_sha256'] = sha(path)
            record[stream + '_bytes'] = len(raw)
        phases.append(record)
        (out / 'commands.json').write_text(json.dumps(phases, indent=2) + '\n')
        print(json.dumps({'name': name, 'exit_code': result.returncode, 'seconds': elapsed}), flush=True)
        guards()
        assert result.returncode == 0 or (allow_gate_failure and result.returncode == 101), name
        stdout = result.stdout.decode()
        if label in ['allocator-contracts', 'drc-tests']:
            counts = re.findall(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;', stdout)
            row = {'variant': variant, 'phase': label, 'passed': sum(int(r[0]) for r in counts),
                   'failed': sum(int(r[1]) for r in counts), 'ignored': sum(int(r[2]) for r in counts),
                   'result_groups': len(counts)}
            expected = (2, 0, 0, 1) if label == 'allocator-contracts' else (1646, 0, 0, 10)
            assert tuple(row[k] for k in ['passed', 'failed', 'ignored', 'result_groups']) == expected, row
            tests.append(row)
            (out / 'tests.json').write_text(json.dumps(tests, indent=2) + '\n')
        return result

    def identify(variant, kind, result):
        events = [json.loads(line) for line in result.stdout.splitlines() if line.startswith(b'{')]
        artifacts = [e for e in events if e.get('reason') == 'compiler-artifact' and e.get('executable')
                     and e['target']['name'] == 'hyperdrc' and e['target']['kind'] == (['lib'] if kind == 'test' else ['bin'])]
        assert len(artifacts) == 1
        artifact = artifacts[0]
        binary = Path(artifact['executable'])
        assert binary.is_relative_to(Path(prep['variants'][variant]['target']))
        assert Path(artifact['manifest_path']) == Path(prep['variants'][variant]['source']) / 'crates/hyperdrc/Cargo.toml'
        assert bool(artifact['profile']['test']) == (kind == 'test')
        features = {e['target']['name']: e['features'] for e in events if e.get('reason') == 'compiler-artifact'
                    and e['target']['name'] in ['mimalloc', 'libmimalloc_sys'] and e['target']['kind'] == ['lib']}
        if variant == 'candidate':
            assert features == {'mimalloc': [], 'libmimalloc_sys': []}, features
        binaries.setdefault(variant, {})[kind] = {'path': str(binary), 'sha256': sha(binary),
            'bytes': binary.stat().st_size, 'manifest_path': artifact['manifest_path'],
            'features': artifact['features'], 'package_id': artifact['package_id'],
            'target': artifact['target'], 'profile': artifact['profile'], 'allocator_dependency_features': features}
        (out / 'binaries.json').write_text(json.dumps(binaries, indent=2) + '\n')

    for variant in protocol['variant_order']:
        for label, command in commands:
            result = run(variant, label, command)
            if label in ['build-test', 'build-cli']:
                identify(variant, 'test' if label == 'build-test' else 'cli', result)
        for kind in ['test', 'cli']:
            binary = binaries[variant][kind]
            demangled = run(variant, kind + '-nm-demangled', ['nm', '-C', '--defined-only', binary['path']], compressed=True).stdout.decode()
            raw = run(variant, kind + '-nm-raw', ['nm', '--defined-only', binary['path']], compressed=True).stdout.decode()
            matches = [line.split()[0] for line in demangled.splitlines() if line.endswith(' __rustc::__rust_alloc')]
            assert len(matches) == 1
            symbols = [line.split()[-1] for line in raw.splitlines() if line.split()[0] == matches[0]]
            assert len(symbols) == 1, symbols
            assembly = run(variant, kind + '-global-alloc', ['objdump', '-d', '--disassemble=' + symbols[0], binary['path']], compressed=True).stdout.decode()
            expected = '__rdl_alloc' if variant == 'baseline' else 'mi_malloc_aligned'
            selection = {'variant': variant, 'kind': kind, 'symbol': symbols[0], 'binary_sha256': binary['sha256'],
                         'required_target_observed': expected, 'assembly_sha256': __import__('hashlib').sha256(assembly.encode()).hexdigest()}
            if variant == 'baseline':
                assert expected in assembly
            else:
                relocations = run(variant, kind + '-relocations', ['readelf', '-rW', binary['path']], compressed=True).stdout.decode()
                with Path(binary['path']).open('rb') as elf:
                    header = elf.read(20)
                assert header[:7] == b'\x7fELF\x02\x01\x01' and header[16:20] == b'\x03\x00\x3e\x00'
                target_addresses = [int(line.split()[0], 16) for line in demangled.splitlines() if line.endswith(' mi_malloc_aligned')]
                assert len(target_addresses) == 1
                slots = re.findall(r'jmp\s+\*[^\n]*#\s+([0-9a-f]+)', assembly)
                if slots:
                    assert len(slots) == 1
                    relocation = [line.split() for line in relocations.splitlines()
                                  if line.split() and line.split()[0] == slots[0].zfill(16)]
                    assert len(relocation) == 1 and len(relocation[0]) == 4
                    assert relocation[0][2] == 'R_X86_64_RELATIVE'
                    assert int(relocation[0][3], 16) == target_addresses[0]
                    selection.update(dispatch='ELF64 x86-64 relative relocation', got_slot=slots[0],
                                     target_address=format(target_addresses[0], 'x'))
                else:
                    assert expected in assembly
                    selection.update(dispatch='direct named target', target_address=format(target_addresses[0], 'x'))
                selection['relocations_sha256'] = __import__('hashlib').sha256(relocations.encode()).hexdigest()
            existing = [row for row in selections if row['variant'] == variant and row['kind'] == kind]
            if existing:
                assert existing == [selection]
            else:
                selections.append(selection)
            (out / 'allocator-selections.json').write_text(json.dumps(selections, indent=2) + '\n')
        for gate in GATES:
            result = run(variant, gate['name'], ['/usr/bin/time', '-v', binaries[variant]['test']['path'], gate['test'],
                         '--exact', '--test-threads=1', '--nocapture'], allow_gate_failure=True)
            both = (result.stdout + result.stderr).decode()
            assert 'skipping complex project fixture test' not in both
            passed = result.returncode == 0
            assert ('1 passed; 0 failed' if passed else '0 passed; 1 failed') in both
            if gate['name'] == 'full-board':
                assert both.count('finished emit report') == 2
                assert [int(v) for v in re.findall(r'finished run \((\d+) active finding', both)] == [12, 197]
            duration = re.search(gate['failure_pattern'], both)
            if not passed:
                assert duration and float(duration[1]) >= gate['limit_seconds']
            record = next(row for row in phases if row['name'] == variant + '-' + gate['name'])
            observed = gate | {'variant': variant, 'passed': passed,
                               'assertion_seconds': float(duration[1]) if duration else None,
                               'process_wall_seconds': record['seconds']}
            existing = [row for row in gates if row['variant'] == variant and row['name'] == gate['name']]
            if existing:
                assert existing == [observed]
            else:
                gates.append(observed)
            (out / 'gates.json').write_text(json.dumps(gates, indent=2) + '\n')
    guards()
    result = {'complete': True, 'phases': phases, 'tests': tests, 'gates': gates,
              'allocator_selections': selections,
              'candidate_original_gates_passed': all(g['passed'] for g in gates if g['variant'] == 'candidate'),
              'sources_dependencies_main_and_binaries_unchanged': True,
              'runtime_admitted': False, 'plan_complete': False}
    (out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({'complete': True, 'candidate_original_gates_passed': result['candidate_original_gates_passed'], 'gates': gates}), flush=True)


if __name__ == '__main__':
    main()
