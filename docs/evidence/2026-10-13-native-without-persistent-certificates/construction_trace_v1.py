"""Count existing arithmetic events in three unchanged release libraries.

This is a diagnostic, with no timing or allocation comparison. Fixture,
expected-answer, representation-inventory and witness code is copied exactly
from the closed quadratic probe. Only its observation wrapper is replaced.
"""
import gzip
import json
import os
from pathlib import Path
import subprocess

from common_v1 import E, ROOT, PREP, WORK, ENV, sha, dump, verify

WRAPPER = '''//! Existing public dispatch/rational counters; no elapsed-cost measurement.
use hyperreal::{Rational, Real, dispatch_trace};
use std::hint::black_box;

fn measure<T>(name: &str, count: usize, body: impl FnOnce() -> T) -> T {
    dispatch_trace::reset();
    let result = dispatch_trace::with_recording(body);
    let snapshot = dispatch_trace::take_trace();
    let stats = snapshot.rational;
    let dispatch: Vec<_> = snapshot.dispatch.into_iter().map(|entry| {
        serde_json::json!({"layer": entry.layer, "operation": entry.operation,
                           "path": entry.path, "count": entry.count})
    }).collect();
    println!("{}", serde_json::json!({
        "trace_phase": name, "iterations": count, "dispatch": dispatch,
        "rational": {
            "temporary_rationals": stats.temporary_rationals,
            "reductions": stats.reductions, "gcds": stats.gcds,
            "peak_operand_bits": stats.peak_operand_bits,
            "common_factors": {"none": stats.common_factors.none,
                "power_of_two": stats.common_factors.power_of_two,
                "small": stats.common_factors.small, "medium": stats.common_factors.medium,
                "large": stats.common_factors.large},
            "gcd_operand_widths": {"both_u64": stats.gcd_operand_widths.both_u64,
                "mixed_u64_u128": stats.gcd_operand_widths.mixed_u64_u128,
                "both_u128": stats.gcd_operand_widths.both_u128,
                "arbitrary_precision": stats.gcd_operand_widths.arbitrary_precision}
        }
    }));
    black_box(result)
}

'''


def rows(data):
    return [json.loads(line) for line in data.splitlines() if line.startswith(b'{')]


def identities(data):
    return [row for row in rows(data) if 'witness' in row or 'fixture' in row]


def flatten(value, prefix=''):
    result = {}
    for key, item in value.items():
        key = prefix + key
        if isinstance(item, dict):
            result.update(flatten(item, key + '.'))
        else:
            assert isinstance(item, int)
            result[key] = item
    return result


def main():
    prep = json.loads((PREP / 'protocol.json').read_text())
    verify(prep, full=True)
    previous = ROOT / 'docs/evidence/2026-10-13-combined-native-admission'
    full_prep = json.loads((previous / 'preparation-v1/protocol.json').read_text())
    full_variant = full_prep['variants']['candidate']
    full_hash_path = previous / 'preparation-v1/candidate-source-hashes.json.gz'
    assert sha(full_hash_path) == full_variant['source_hashes_sha256']
    full_hashes = json.loads(gzip.decompress(full_hash_path.read_bytes()))
    costs = {'main': previous / 'library-costs-v1', 'full-cache': previous / 'library-costs-v1',
             'no-persistent': E / 'library-costs-v1'}
    labels = {'main': 'baseline', 'full-cache': 'candidate', 'no-persistent': 'candidate'}
    variants = {'main': prep['variants']['baseline'], 'full-cache': full_variant,
                'no-persistent': prep['variants']['candidate']}
    libraries = {}
    inputs = {str(p): sha(p) for p in [Path(__file__), E / 'next-diagnostic-v1.json',
              PREP / 'protocol.json', previous / 'preparation-v1/protocol.json', full_hash_path,
              E / 'decision-v1.json', E / 'native-gates-assessment-v1.json']}
    old_binaries = {}
    for label, directory in costs.items():
        result = json.loads((directory / 'result.json').read_text())
        assert result['complete'] and result['all_identities_match']
        for name, digest in result['artifact_sha256'].items():
            assert sha(directory / name) == digest
            inputs[str(directory / name)] = digest
        inputs[str(directory / 'result.json')] = sha(directory / 'result.json')
        group = json.loads((directory / 'libraries.json').read_text())[labels[label]]
        libraries[label] = {name: group[name] for name in ['hyperreal', 'serde_json']}
        assert 'dispatch-trace' in libraries[label]['hyperreal']['features']
        recorded = json.loads((directory / 'protocol.json').read_text())
        assert recorded['compiler'] == subprocess.check_output(['rustc', '-vV'], text=True)
        assert recorded['inherited_rustflags'] == os.environ.get('RUSTFLAGS')
        for row in json.loads((directory / 'binaries.json').read_text()).values():
            old_binaries[row['path']] = row['sha256']
    native = E / 'native-gates-v1'
    native_result = json.loads((native / 'result.json').read_text())
    assert native_result['complete']
    for name, digest in native_result['artifact_sha256'].items():
        assert sha(native / name) == digest
        inputs[str(native / name)] = digest
    inputs[str(native / 'result.json')] = sha(native / 'result.json')
    native_binary = json.loads((native / 'binary.json').read_text())
    old_binaries[native_binary['path']] = native_binary['sha256']
    out = E / 'construction-trace-v1'
    bins = WORK / 'construction-trace-binaries-v1'
    out.mkdir(exist_ok=False)
    bins.mkdir(exist_ok=False)
    original_probe = previous / 'probes-v1/quadratic_probe.rs'
    inputs[str(original_probe)] = sha(original_probe)
    original = original_probe.read_text()
    assert original.count('fn inputs(index: usize)') == 1
    suffix = original[original.index('fn inputs(index: usize)'):]
    probe = out / 'probe.rs'
    probe.write_text(WRAPPER + suffix)
    assert probe.read_text()[len(WRAPPER):] == suffix
    inputs[str(probe)] = sha(probe)
    schedule = [(1, label) for label in ['main', 'full-cache', 'no-persistent']]
    schedule += [(2, label) for label in ['no-persistent', 'full-cache', 'main']]
    protocol = {'scope': __doc__, 'runner_sha256': sha(__file__), 'inputs_sha256': inputs,
                'variants': variants, 'libraries': libraries, 'schedule': schedule,
                'inputs_per_process': 4096, 'compiler': subprocess.check_output(['rustc', '-vV'], text=True),
                'inherited_rustflags': os.environ.get('RUSTFLAGS'),
                'environment': ENV, 'new_library_builds': 0,
                'fixture_and_witness_suffix_unchanged': True,
                'observed_windows': 'construction, first, warm, cloned for all eight original quadratic families',
                'limitations': 'No timing/allocation conclusions; public counters do not cover every certificate copy or lock.',
                'runtime_admitted': False, 'plan_complete': False}
    dump(out / 'protocol.json', protocol)
    binaries, commands, samples = {}, [], []

    def guard(full=False):
        verify(prep, full=full)
        assert all(sha(p) == digest for p, digest in inputs.items())
        assert all(sha(row['path']) == row['sha256'] for group in libraries.values() for row in group.values())
        assert all(sha(row['path']) == row['sha256'] for row in binaries.values())
        if full:
            assert all(sha(p) == digest for p, digest in old_binaries.items())
            source = Path(full_variant['source'])
            assert {str(p.relative_to(source)) for p in source.rglob('*') if p.is_file()} == set(full_hashes)
            assert all(sha(source / p) == digest for p, digest in full_hashes.items())

    def run(name, label, command):
        guard()
        print(json.dumps({'start': name}), flush=True)
        completed = subprocess.run(['timeout', '--signal=TERM', '--kill-after=10s', '300', *command],
                                   cwd=variants[label]['source'], env=os.environ | ENV, capture_output=True)
        row = {'name': name, 'variant': label, 'command': command, 'cwd': variants[label]['source'],
               'exit_code': completed.returncode}
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

    guard(full=True)
    for label in ['main', 'full-cache', 'no-persistent']:
        binary = bins / label
        command = ['rustc', '--edition=2024', '-O', '-D', 'warnings', str(probe),
                   '-L', 'dependency=' + str(Path(variants[label]['target']) / 'release/deps')]
        for name, row in libraries[label].items():
            command += ['--extern', name + '=' + row['path']]
        run('build-' + label, label, command + ['-o', str(binary)])
        binaries[label] = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size}
        dump(out / 'binaries.json', binaries)
    assert len({row['sha256'] for row in binaries.values()}) == 3
    historical = []
    for directory in [previous / 'library-costs-v1', E / 'library-costs-v1']:
        for label in ['baseline', 'candidate']:
            for pair in range(1, 6):
                path = directory / ('sample-quadratic-time-' + str(pair) + '-' + label + '.stdout.gz')
                assert str(path) in inputs
                historical.append(identities(gzip.decompress(path.read_bytes())))
    assert len(historical[0]) == 32 and all(value == historical[0] for value in historical)
    for repetition, label in schedule:
        raw = run('trace-' + str(repetition) + '-' + label, label,
                  ['taskset', '-c', '0', binaries[label]['path'], '4096'])
        assert identities(raw) == historical[0]
        trace = [r for r in rows(raw) if 'trace_phase' in r]
        assert len(trace) == 32 and all(r['iterations'] == 4096 for r in trace)
        assert len({r['trace_phase'] for r in trace}) == 32
        assert sum(r['rational']['temporary_rationals'] for r in trace) > 0
        samples.append({'variant': label, 'repetition': repetition, 'trace': trace})
        dump(out / 'samples.json', samples)
    stable = all([s['trace'] for s in samples if s['variant'] == label][0]
                 == [s['trace'] for s in samples if s['variant'] == label][1] for label in variants)
    comparisons = []
    for repetition in [1, 2]:
        for left, right in [('main', 'full-cache'), ('main', 'no-persistent'), ('no-persistent', 'full-cache')]:
            pair = {label: next(s['trace'] for s in samples if s['variant'] == label and s['repetition'] == repetition)
                    for label in [left, right]}
            for a, b in zip(pair[left], pair[right], strict=True):
                assert a['trace_phase'] == b['trace_phase']
                fa, fb = flatten(a['rational']), flatten(b['rational'])
                da, db = ({(r['layer'], r['operation'], r['path']): r['count'] for r in value['dispatch']}
                          for value in [a, b])
                comparisons.append({'repetition': repetition, 'baseline': left, 'candidate': right,
                    'phase': a['trace_phase'], 'rational_candidate_minus_baseline': {k: fb[k] - fa[k] for k in fa},
                    'dispatch_candidate_minus_baseline': [{'layer': k[0], 'operation': k[1], 'path': k[2],
                        'count_delta': db.get(k, 0) - da.get(k, 0)} for k in sorted(set(da) | set(db))],
                    'all_recorded_counters_equal': a == b})
    dump(out / 'comparisons.json', comparisons)
    guard(full=True)
    result = {'complete': True, 'commands': len(commands), 'new_library_builds': 0,
              'diagnostic_processes': len(samples), 'trace_rows': sum(len(s['trace']) for s in samples),
              'comparison_rows': len(comparisons), 'counts_repeat_exactly_within_each_variant': stable,
              'all_exact_witnesses_and_inventories_match_both_closed_trials': True,
              'all_library_sources_old_binaries_and_evidence_unchanged': True,
              'all_builds_precede_diagnostics': True, 'timing_or_allocation_measurement': False,
              'artifact_sha256': {p.name: sha(p) for p in sorted(out.iterdir()) if p.is_file()},
              'runtime_admitted': False, 'plan_complete': False}
    dump(out / 'result.json', result)
    print(json.dumps({k: v for k, v in result.items() if k != 'artifact_sha256'}), flush=True)


if __name__ == '__main__':
    main()
