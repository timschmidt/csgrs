"""Check native viability once without changing either original assertion.

This candidate-only diagnostic is not full stack qualification, paired
application timing, report-byte admission or resolution of library costs.
Passing assertions do not print their internal duration; process wall must
not be substituted for worker CPU or copper-neck check-body wall.
"""
import gzip
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import time
import zipfile

from common_v1 import E, ROOT, PREP, ENV, sha, dump, verify

GATES = [
    {'name': 'full-board', 'test': 'zz_complex_project_performance_tests::kicad_board_completes_smoke_check_suite',
     'limit_seconds': 120, 'metric': 'worker CPU', 'failure_pattern': r'KiCad smoke suite took ([0-9.]+)s of worker time'},
    {'name': 'copper-neck', 'test': 'zz_complex_project_performance_tests::min_copper_neck_width_completes_on_copper_layers',
     'limit_seconds': 60, 'metric': 'check body wall', 'failure_pattern': r'complex project copper neck regression fixture took ([0-9.]+)s'},
]


def main():
    prep = json.loads((PREP / 'protocol.json').read_text())
    verify(prep, full=True)
    library = json.loads((E / 'library-costs-v1/result.json').read_text())
    assert library['complete'] and library['all_identities_match']
    assert all(sha(E / 'library-costs-v1' / p) == h for p, h in library['artifact_sha256'].items())
    old_binaries = json.loads((E / 'library-costs-v1/binaries.json').read_text())
    old_libraries = json.loads((E / 'library-costs-v1/libraries.json').read_text())
    out = E / 'native-gates-v1'
    out.mkdir(exist_ok=False)
    source = Path(prep['variants']['candidate']['source'])
    target = Path(prep['variants']['candidate']['target'])
    inherited_allocator = sorted(k for k in os.environ if k.startswith(('MIMALLOC_', 'MI_'))
                                 or k in ['LD_PRELOAD', 'MALLOC_CONF', 'GLIBC_TUNABLES', 'MALLOC_ARENA_MAX'])
    assert not inherited_allocator, inherited_allocator
    fixtures = []
    for name, entry in [('CPArti FPGA dev board.zip', 'CPArti FPGA dev board.kicad_pcb'),
                        ('HVP109A.zip', 'HVP109A.kicad_pcb')]:
        path = source / 'crates/hyperdrc/docs' / name
        with zipfile.ZipFile(path) as archive:
            info = archive.getinfo(entry)
            assert info.file_size > 0
        fixtures.append({'path': str(path), 'sha256': sha(path), 'board_entry': entry,
                         'board_bytes': info.file_size, 'board_crc32': info.CRC})
    protocol = {'scope': __doc__, 'runner_sha256': sha(__file__),
                'scope_sha256': sha(E / 'native-gates-scope-v1.json'),
                'preparation_sha256': sha(PREP / 'protocol.json'),
                'library_result_sha256': sha(E / 'library-costs-v1/result.json'),
                'library_interpretation_sha256': sha(E / 'library-interpretation-v1.json'),
                'source': str(source), 'target': str(target),
                'environment': ENV | {'CARGO_TARGET_DIR': str(target)},
                'compiler': subprocess.check_output(['rustc', '-vV'], text=True),
                'inherited_rustflags': os.environ.get('RUSTFLAGS'), 'inherited_allocator_settings': inherited_allocator,
                'unzip': {'path': shutil.which('unzip'), 'sha256': sha(shutil.which('unzip'))},
                'fixtures': fixtures, 'gates': GATES, 'timeout_seconds': 1800,
                'single_candidate_no_paired_speedup_claim': True, 'runtime_admitted': False}
    dump(out / 'protocol.json', protocol)
    commands, gates = [], []
    binary = None

    def guard(full=False):
        verify(prep, full=full)
        assert sha(__file__) == protocol['runner_sha256']
        assert sha(E / 'native-gates-scope-v1.json') == protocol['scope_sha256']
        assert sha(PREP / 'protocol.json') == protocol['preparation_sha256']
        assert sha(E / 'library-costs-v1/result.json') == protocol['library_result_sha256']
        assert sha(E / 'library-interpretation-v1.json') == protocol['library_interpretation_sha256']
        assert all(sha(r['path']) == r['sha256'] for r in old_binaries.values())
        assert all(sha(r['path']) == r['sha256'] for group in old_libraries.values() for r in group.values())
        assert all(sha(r['path']) == r['sha256'] for r in fixtures)
        assert sha(protocol['unzip']['path']) == protocol['unzip']['sha256']
        if binary is not None:
            assert sha(binary['path']) == binary['sha256']

    def run(name, command, gate=False):
        guard()
        print(json.dumps({'start': name}), flush=True)
        started = time.monotonic()
        completed = subprocess.run(['timeout', '--signal=TERM', '--kill-after=10s', '1800', *command],
                                   cwd=source, env=os.environ | protocol['environment'], capture_output=True)
        seconds = time.monotonic() - started
        row = {'name': name, 'command': command, 'cwd': str(source), 'exit_code': completed.returncode,
               'process_wall_seconds': seconds}
        for stream in ['stdout', 'stderr']:
            data = getattr(completed, stream)
            path = out / (name + '.' + stream + '.gz')
            path.write_bytes(gzip.compress(data, mtime=0))
            row[stream + '_sha256'] = sha(path)
        commands.append(row)
        dump(out / 'commands.json', commands)
        print(json.dumps({'finished': name, 'exit_code': completed.returncode, 'seconds': seconds}), flush=True)
        guard()
        assert completed.returncode == 0 or (gate and completed.returncode == 101), (name, completed.returncode)
        return completed

    for i, fixture in enumerate(fixtures):
        run('fixture-archive-' + str(i + 1), ['unzip', '-t', fixture['path']])
    raw = run('build-test', ['cargo', 'test', '--locked', '--release', '-p', 'hyperdrc', '--all-features',
                             '--lib', '--no-run', '--message-format=json']).stdout
    events = [json.loads(line) for line in raw.splitlines() if line.startswith(b'{')]
    artifacts = [e for e in events if e.get('reason') == 'compiler-artifact' and e.get('executable')
                 and e['target']['name'] == 'hyperdrc' and e['target']['kind'] == ['lib'] and e['profile']['test']]
    assert len(artifacts) == 1
    artifact = artifacts[0]
    path = Path(artifact['executable'])
    assert path.is_relative_to(target)
    assert Path(artifact['manifest_path']) == source / 'crates/hyperdrc/Cargo.toml'
    allocator_features = {e['target']['name']: e['features'] for e in events if e.get('reason') == 'compiler-artifact'
                          and e['target']['name'] in ['mimalloc', 'libmimalloc_sys'] and e['target']['kind'] == ['lib']}
    assert allocator_features == {'mimalloc': [], 'libmimalloc_sys': []}
    binary = {'path': str(path), 'sha256': sha(path), 'bytes': path.stat().st_size,
              'manifest_path': artifact['manifest_path'], 'features': artifact['features'],
              'allocator_dependency_features': allocator_features}
    dump(out / 'binary.json', binary)
    demangled = run('nm-demangled', ['nm', '-C', '--defined-only', str(path)]).stdout.decode()
    raw = run('nm-raw', ['nm', '--defined-only', str(path)]).stdout.decode()
    addresses = [line.split()[0] for line in demangled.splitlines() if line.endswith(' __rustc::__rust_alloc')]
    assert len(addresses) == 1
    symbols = [line.split()[-1] for line in raw.splitlines() if line.split()[0] == addresses[0]]
    assert len(symbols) == 1
    assembly = run('global-alloc', ['objdump', '-d', '--disassemble=' + symbols[0], str(path)]).stdout.decode()
    relocations = run('relocations', ['readelf', '-rW', str(path)]).stdout.decode()
    with path.open('rb') as stream:
        header = stream.read(20)
    assert header[:7] == b'\x7fELF\x02\x01\x01' and header[16:20] == b'\x03\x00\x3e\x00'
    targets = [int(line.split()[0], 16) for line in demangled.splitlines() if line.endswith(' mi_malloc_aligned')]
    assert len(targets) == 1
    slots = re.findall(r'jmp\s+\*[^\n]*#\s+([0-9a-f]+)', assembly)
    if slots:
        assert len(slots) == 1
        rows = [line.split() for line in relocations.splitlines() if line.split() and line.split()[0] == slots[0].zfill(16)]
        assert len(rows) == 1 and len(rows[0]) == 4 and rows[0][2] == 'R_X86_64_RELATIVE'
        assert int(rows[0][3], 16) == targets[0]
    else:
        assert 'mi_malloc_aligned' in assembly
    selection = {'binary_sha256': binary['sha256'], 'global_symbol': symbols[0],
                 'target': 'mi_malloc_aligned', 'target_address': format(targets[0], 'x'),
                 'got_slots': slots, 'verified': True, 'scope': 'Linux ELF64 x86-64 native lib test binary'}
    dump(out / 'allocator-selection.json', selection)
    raw = run('allocator-contracts', [str(path), 'allocator_contract_tests', '--test-threads=1']).stdout.decode()
    assert re.search(r'test result: ok\. 2 passed; 0 failed; 0 ignored;', raw)
    for gate in GATES:
        completed = run(gate['name'], ['/usr/bin/time', '-v', str(path), gate['test'],
                                     '--exact', '--test-threads=1', '--nocapture'], gate=True)
        both = (completed.stdout + completed.stderr).decode()
        assert 'skipping complex project fixture' not in both
        passed = completed.returncode == 0
        assert ('1 passed; 0 failed' if passed else '0 passed; 1 failed') in both
        if gate['name'] == 'full-board':
            assert both.count('finished emit report') == 2
            assert [int(n) for n in re.findall(r'finished run \((\d+) active finding', both)] == [12, 197]
        duration = re.search(gate['failure_pattern'], both)
        if not passed:
            assert duration and float(duration[1]) >= gate['limit_seconds']
        gates.append(gate | {'passed': passed, 'assertion_seconds': float(duration[1]) if duration else None,
                             'process_wall_seconds': commands[-1]['process_wall_seconds']})
        dump(out / 'gates.json', gates)
    guard(full=True)
    result = {'complete': True, 'commands': len(commands), 'allocator_contract_tests_passed': 2,
              'allocator_linkage_verified': True, 'gates': gates,
              'candidate_original_gates_passed': all(g['passed'] for g in gates),
              'full_board_findings': [12, 197], 'complete_report_byte_identity_established': False,
              'wider_stack_qualification_complete': False, 'library_costs_resolved': False,
              'all_sources_dependencies_main_old_artifacts_and_binaries_unchanged': True,
              'artifact_sha256': {p.name: sha(p) for p in sorted(out.iterdir()) if p.is_file()},
              'runtime_admitted': False, 'plan_complete': False}
    dump(out / 'result.json', result)
    print(json.dumps({k: v for k, v in result.items() if k != 'artifact_sha256'}), flush=True)


if __name__ == '__main__':
    main()
