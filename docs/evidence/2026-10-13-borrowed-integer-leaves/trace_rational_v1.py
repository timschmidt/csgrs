"""Link four diagnostic drivers and record two fixed passes over six families."""
import gzip
import json
import os
from pathlib import Path
import subprocess

from rational_trace_common_v1 import E, ROOT, TRACE_PREP, TRACE_WORK, GROUPS, ORDERS, ENV, sha, dump, verify, rows, identity


def main():
    prep = json.loads((TRACE_PREP / 'protocol.json').read_text())
    verify(prep, full=True)
    review_path = E / 'rational-trace-review-v1.json'
    review = json.loads(review_path.read_text())
    assert review['preparation_sha256'] == sha(TRACE_PREP / 'protocol.json')
    assert review['new_driver_reviewed'] and review['all_original_fixture_and_query_bytes_preserved']
    assert prep['compiler'] == subprocess.check_output(['rustc', '-vV'], text=True)
    assert prep['inherited_rustflags'] == os.environ.get('RUSTFLAGS')
    out = E / 'rational-trace-v1'
    out.mkdir(exist_ok=False)
    protocol = {'scope': __doc__, 'runner_sha256': sha(__file__),
                'assessor_sha256': sha(E / 'assess_rational_trace_v1.py'),
                'preparation_sha256': sha(TRACE_PREP / 'protocol.json'), 'review_sha256': sha(review_path),
                'orders': ORDERS, 'count': 4096, 'fixed_processes': 8, 'commands': 17,
                'trace_windows': 192, 'counter_windows': 48, 'environment': ENV,
                'libraries': prep['libraries'], 'compiler': prep['compiler'],
                'inherited_rustflags': os.environ.get('RUSTFLAGS'),
                'new_library_builds': 0, 'new_runtime_sources': 0,
                'no_timing_or_allocation_comparison': True, 'runtime_admitted': False}
    dump(out / 'protocol.json', protocol)
    commands, binaries, samples = [], {}, []

    def guard(full=False):
        verify(prep, full=full)
        assert sha(__file__) == protocol['runner_sha256']
        assert sha(review_path) == protocol['review_sha256']
        assert all(sha(r['path']) == r['sha256'] for r in binaries.values())

    def run(name, command):
        guard()
        print(json.dumps({'start': name}), flush=True)
        completed = subprocess.run(['timeout', '--signal=TERM', '--kill-after=10s', '1800', *command],
                                   cwd=ROOT, env=os.environ | ENV, capture_output=True)
        row = {'name': name, 'command': command, 'cwd': str(ROOT), 'exit_code': completed.returncode}
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

    run('format-header', ['rustfmt', '--check', '--edition', '2024', '--config-path', str(ROOT / '.rustfmt.toml'),
                          str(E / 'rational_trace_header_v1.rs')])
    for group in GROUPS:
        libraries = prep['libraries'][group]
        dependency = Path(libraries['hyperreal']['path']).parent / 'deps'
        assert dependency.is_dir()
        flags = ['--edition=2024', '-O', '-D', 'warnings', str(TRACE_PREP / 'probe.rs'),
                 '-L', 'dependency=' + str(dependency)]
        for name, row in libraries.items():
            flags += ['--extern', name + '=' + row['path']]
        if group == 'cache-coverage':
            flags += ['--cfg', 'feature="cache-events"']
        run('clippy-' + group, ['clippy-driver', *flags, '--emit=metadata',
                                '-o', str(TRACE_WORK / (group + '.rmeta'))])
        binary = TRACE_WORK / ('probe-' + group)
        run('link-' + group, ['rustc', *flags, '-o', str(binary)])
        binaries[group] = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size}
        dump(out / 'binaries.json', binaries)
    for repetition, order in enumerate(ORDERS, 1):
        for position, group in enumerate(order, 1):
            name = f'trace-{repetition}-{group}'
            raw = run(name, ['taskset', '-c', '0', binaries[group]['path'], '4096'])
            parsed = rows(raw)
            assert len(raw.splitlines()) == len(parsed) == 48
            assert identity(raw) == prep['identity']
            trace = [r for r in parsed if 'trace_phase' in r]
            assert len(trace) == 24
            assert all(r['iterations'] == 4096 for r in trace)
            assert all((isinstance(r['cache_events'], dict) and len(r['cache_events']) == 27)
                       if group == 'cache-coverage' else r['cache_events'] is None for r in trace)
            samples.append({'repetition': repetition, 'position': position, 'group': group,
                            'command_name': name, 'trace': trace, 'identity_matches_closed_trial': True})
            dump(out / 'samples.json', samples)
    assert len(commands) == 17 and len(samples) == 8
    for group in GROUPS:
        own = [s['trace'] for s in samples if s['group'] == group]
        assert len(own) == 2 and own[0] == own[1], group
    public = lambda trace: [{k: v for k, v in r.items() if k != 'cache_events'} for r in trace]
    by_group = {s['group']: s['trace'] for s in samples}
    assert public(by_group['cache-coverage']) == public(by_group['full-cache'])
    total = {name: sum(r['cache_events'][name] for r in by_group['cache-coverage'])
             for name in by_group['cache-coverage'][0]['cache_events']}
    assert all(total[k] > 0 for k in ['retained_visit', 'retained_lookup', 'store_attempt',
                                     'closed_copy_declined_opaque', 'store_published', 'retained_hit'])
    guard(full=True)
    result = {'complete': True, 'commands': len(commands), 'fixed_processes': len(samples),
              'trace_windows': 192, 'counter_windows': 48, 'all_counts_repeat_within_group': True,
              'all_original_witnesses_and_representations_preserved': True,
              'counter_instrumentation_preserves_full_cache_public_counts': True,
              'new_library_builds': 0, 'new_runtime_sources': 0,
              'all_sources_libraries_and_old_artifacts_unchanged': True,
              'artifact_sha256': {p.name: sha(p) for p in sorted(out.iterdir()) if p.is_file()},
              'no_timing_or_allocation_comparison': True, 'runtime_admitted': False, 'plan_complete': False}
    dump(out / 'result.json', result)
    print(json.dumps({k: v for k, v in result.items() if k != 'artifact_sha256'}), flush=True)


if __name__ == '__main__':
    main()
