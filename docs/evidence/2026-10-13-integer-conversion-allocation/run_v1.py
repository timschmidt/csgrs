"""Two fixed allocation observations; no timing or runtime candidate."""
import gzip
import json
import os
from pathlib import Path
import re
import subprocess

from common_v1 import E, ROOT, HARDWARE, WORK, sha, dump, verify_prior, verify


def main():
    verify_prior(full=True)
    prior = json.loads((HARDWARE / 'run-v2/protocol.json').read_text())
    assert prior['compiler'] == subprocess.check_output(['rustc', '-vV'], text=True)
    assert prior['inherited_rustflags'] == os.environ.get('RUSTFLAGS')
    out = E / 'run-v1'
    out.mkdir(exist_ok=False)
    WORK.mkdir(exist_ok=False)
    fixture = ROOT / 'docs/evidence/2026-10-13-combined-native-admission/probes-v1/integer_probe.rs'
    source = fixture.read_text()
    body = source[source.index('fn integer('):source.index('fn fixture(')]
    values = re.findall(r'"(-?\d+)"', body)
    assert len(values) == 19
    signed = values[7:13]
    assert sum(-(1 << 63) <= int(signed[i % 6]) < (1 << 63) for i in range(1000)) == 501
    inputs = out / 'inputs.rs'
    inputs.write_text('pub const VALUES: &[&str] = &' + json.dumps(values) + ';\n'
                      'pub const SIGNED_WORD_VALUES: &[&str] = &' + json.dumps(signed) + ';\n')
    probe = out / 'probe.rs'
    probe.write_bytes((E / 'probe_v1.rs').read_bytes())
    variant = prior['variants']['main']
    libraries = prior['libraries']['main'].copy()
    deps = Path(variant['target']) / 'release/deps'
    num = list(deps.glob('libnum-*.rlib'))
    assert len(num) == 1
    libraries['num'] = {'path': str(num[0]), 'sha256': sha(num[0]), 'bytes': num[0].stat().st_size}
    constructor = 'crates/hyperreal/src/rational/arithmetic/construction.rs'
    references = [Path(v['source']) / constructor for v in prior['variants'].values()]
    assert len({sha(p) for p in references}) == 1
    references += [Path(v['source']) / 'crates/hyperreal/src/computable/node/structural_analysis.rs'
                   for v in prior['variants'].values()]
    registry = Path('/tmp/csgrs-properties-cargo/registry/src/index.crates.io-1949cf8c6b5b557f/num-bigint-0.4.6')
    references += [registry / p for p in ['src/biguint.rs', 'src/biguint/convert.rs',
                                        'src/bigint.rs', 'src/bigint/convert.rs']]
    files = [E / f for f in ['probe_v1.rs', 'common_v1.py', 'run_v1.py', 'assess_v1.py',
                             'scope-v1.json', 'source-review-v1.json']]
    files += [probe, inputs, fixture, HARDWARE / 'decision-v1.json']
    protocol = {'scope': __doc__, 'inputs_sha256': {str(p): sha(p) for p in files},
                'source_reference_sha256': {str(p): sha(p) for p in references},
                'compiler': prior['compiler'], 'inherited_rustflags': os.environ.get('RUSTFLAGS'),
                'libraries': libraries, 'dependency_directory': str(deps),
                'all_three_original_constructor_sources_identical': True,
                'original_fixture_values': values, 'original_signed_word_values': signed,
                'process_orders': ['forward', 'reverse'], 'batch_inputs': 1000,
                'cases': 19, 'environment': prior['environment'],
                'no_timings_collected': True, 'new_library_builds': 0,
                'runtime_candidate': False, 'runtime_admitted': False, 'plan_complete': False}
    dump(out / 'protocol.json', protocol)
    commands, samples = [], []
    binary = WORK / 'probe'
    binary_hash = None

    def guard(full=False):
        verify(protocol, full=full)
        if binary_hash is not None:
            assert sha(binary) == binary_hash

    def run(name, command):
        guard()
        print(json.dumps({'start': name}), flush=True)
        completed = subprocess.run(['timeout', '--signal=TERM', '--kill-after=10s', '1800', *command],
                                   cwd=ROOT, env=os.environ | prior['environment'], capture_output=True)
        row = {'name': name, 'command': command, 'exit_code': completed.returncode}
        for stream in ['stdout', 'stderr']:
            path = out / (name + '.' + stream + '.gz')
            path.write_bytes(gzip.compress(getattr(completed, stream), mtime=0))
            row[stream + '_sha256'] = sha(path)
        commands.append(row)
        dump(out / 'commands.json', commands)
        print(json.dumps({'finished': name, 'exit_code': completed.returncode}), flush=True)
        assert completed.returncode == 0, name
        guard()
        return completed.stdout

    run('format', ['rustfmt', '--edition', '2024', '--config', 'skip_children=true', '--check', str(probe)])
    flags = ['--edition=2024', '-O', '-D', 'warnings', '-L', 'dependency=' + str(deps)]
    for name, row in libraries.items():
        flags += ['--extern', name + '=' + row['path']]
    run('clippy', ['clippy-driver', *flags, '--emit=metadata', str(probe), '-o', str(WORK / 'probe.rmeta')])
    run('build', ['rustc', *flags, str(probe), '-o', str(binary)])
    binary_hash = sha(binary)
    dump(out / 'binary.json', {'path': str(binary), 'sha256': binary_hash, 'bytes': binary.stat().st_size})
    for order in protocol['process_orders']:
        raw = run('allocation-' + order, ['taskset', '-c', '0', str(binary), order])
        rows = [json.loads(line) for line in raw.splitlines() if line.startswith(b'{')]
        assert len(rows) == 74 and rows[0]['passed']
        assert sum('case' in r for r in rows) == 71 and sum('batch' in r for r in rows) == 2
        samples.append({'order': order, 'rows': rows})
        dump(out / 'samples.json', samples)
    guard(full=True)
    result = {'complete': True, 'commands': len(commands), 'fixed_processes': len(samples),
              'case_method_rows_each': 71, 'batch_rows_each': 2, 'control_rows_each': 1,
              'all_exact_case_values_verified': True, 'no_timings_collected': True,
              'artifact_sha256': {p.name: sha(p) for p in sorted(out.iterdir()) if p.is_file()},
              'all_old_sources_libraries_and_binaries_unchanged': True,
              'runtime_candidate': False, 'runtime_admitted': False, 'plan_complete': False}
    dump(out / 'result.json', result)
    print(json.dumps({k: v for k, v in result.items() if k != 'artifact_sha256'}), flush=True)


if __name__ == '__main__':
    main()
