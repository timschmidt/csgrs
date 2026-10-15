"""One immutable paired cost probe, with identical preserved native libraries."""
import gzip
import importlib.util
import json
import os
from pathlib import Path
import resource
import signal
import subprocess
import sys
import time

sys.dont_write_bytecode = True
E = Path(__file__).resolve().parent
ROOT = E.parents[2]
spec = importlib.util.spec_from_file_location('validation', E / 'validate_v1.py')
v = importlib.util.module_from_spec(spec)
spec.loader.exec_module(v)


def main():
    out = E / 'cost-v1'
    out.mkdir(exist_ok=False)
    target = ROOT / 'target/section-proof-reuse/cost-v1'
    target.mkdir(exist_ok=False)
    source = ROOT / 'docs/evidence/2026-10-15-slice-projection-contract/section_cost_v1.rs'
    initial = v.sources()
    baseline = json.loads((E / 'baseline-v2/protocol.json').read_text())
    candidate = json.loads((E / 'candidate-v1/protocol.json').read_text())
    assert initial == json.loads(gzip.decompress((E / 'candidate-v1/source-hashes.json.gz').read_bytes()))
    assert json.loads((E / 'candidate-v1/result.json').read_text())['passed']
    assert json.loads((E / 'candidate-v1/comparison.json').read_text())['passed']
    old_target = str(ROOT / 'target/section-proof-reuse/baseline-v2')
    new_target = str(ROOT / 'target/section-proof-reuse/candidate-v1')
    old_command = next(cmd for name, cmd in baseline['commands'] if name == 'library-build')
    new_command = next(cmd for name, cmd in candidate['commands'] if name == 'library-build')
    assert [arg.replace(old_target, new_target) for arg in old_command] == new_command
    inputs = {name: digest for name, digest in candidate['preserved_prior_executables_sha256'].items()}
    for phase in ['baseline-v2', 'candidate-v1']:
        for path in (ROOT / 'target/section-proof-reuse' / phase).iterdir():
            if path.is_file(): inputs[str(path.relative_to(ROOT))] = v.sha(path)
    for path in [Path(__file__), source, E / 'validate_v1.py', E / 'section_snapshot_v1.rs',
                 E / 'baseline-v2/protocol.json', E / 'candidate-v1/protocol.json']:
        inputs[str(path.relative_to(ROOT))] = v.sha(path)
    assert all(v.sha(ROOT / name) == digest for name, digest in inputs.items())
    commands = []
    for label, protocol in [('baseline', baseline), ('candidate', candidate)]:
        cmd = next(cmd for name, cmd in protocol['commands'] if name == 'snapshot-build').copy()
        cmd[cmd.index(str(E / 'section_snapshot_v1.rs'))] = str(source)
        cmd[cmd.index('-o') + 1] = str(target / label)
        cmd += ['--check-cfg', 'cfg(candidate_api)', '--cfg', 'candidate_api']
        commands.append([label + '-build', cmd])
    shapes = ['slice_middle', 'slice_bottom', 'slice_top', 'flatten']
    for shape in shapes:
        for label in ['baseline', 'candidate']:
            commands.append([f'{shape}-{label}-dump', [str(target / label), shape, 'dump', '128']])
    for sample in range(3):
        labels = ['baseline', 'candidate'] if sample % 2 == 0 else ['candidate', 'baseline']
        for shape in shapes:
            for cache in ['warm', 'varied', 'disabled']:
                for label in labels:
                    commands.append([f'{sample}-{shape}-{cache}-{label}',
                                     [str(target / label), shape, cache, '128']])
        for label in labels:
            phase = 'baseline-v2' if label == 'baseline' else 'candidate-v1'
            commands.append([f'{sample}-{label}-corpus', [str(ROOT / 'target/section-proof-reuse' / phase / 'snapshot')]])
    v.dump(out / 'protocol.json', {
        'kind': 'focused matched probe; no corpus-wide or cross-library speedup claim',
        'commands': commands, 'inputs_sha256': inputs,
        'rustc': subprocess.check_output(['rustc', '-vV'], text=True),
        'lock_sha256': v.sha(ROOT / 'Cargo.lock'),
        'build_identity': 'Both CSGRS curve-only rlibs compiled by the same recorded rustc command, except output directory, against the very same preserved native library paths and bytes from the qualified all-feature build. These native libraries have feature unification including exact JSON serialization. This is a direct-rustc paired probe, not a Cargo feature-isolation test. Separate Cargo qualification covers that boundary.',
        'source_records': ['baseline-v2', 'candidate-v1'],
        'workload': 'The unchanged prior section_cost_v1.rs: strict cubes width 2..129; input construction, section/projection, independently derived exact area and membership, all-edge square boundary proof, exact JSON materialization, checksum and drop. Each of 72 fresh-process samples has 128 warmups and 128 timed completions. Warm repeats width 2; varied cycles 128 widths under default modeling retention; disabled cycles those widths with modeling caching disabled. Scalar/native state is not globally cold. Three samples alternate baseline/candidate order. The full 43-case snapshot corpus, including original 528-triangle sphere and 12-segment README frustum, also runs in three fresh processes per revision; process wall/RSS and full output are retained separately.',
        'outputs': 'Require identical ordered exact output rows, materialized sizes and deterministic checksums across both revisions. Semantics separately established by exact square proofs and unchanged 25 regression oracles. Snapshot includes public provenance flags, native families/domains and certainty; no inherited construction or analytic-fidelity certification claimed.',
        'allocations': 'Global System allocator wrapper counts successful alloc/zeroed/realloc calls, requested bytes, live delta and peak live bytes above warmup. Not every native allocation or total RSS; instrumentation contributes time.',
        'hardware': subprocess.check_output(['lscpu'], text=True),
        'limitations': '1800-second external wall and 8 GiB per-process address space. Shared host, no CPU affinity or frequency control. No internal work/byte/cancellation bound. Results are not directly comparable to the prior differently configured build.',
        'environment': {key: os.environ[key] for key in ['RUSTFLAGS', 'RUSTC_WRAPPER', 'RUSTUP_TOOLCHAIN'] if key in os.environ},
    })
    rows = []
    for name, command in commands:
        assert v.sources() == initial and all(v.sha(ROOT / p) == inputs[p] for p in [str(Path(__file__).relative_to(ROOT)), str(source.relative_to(ROOT))])
        started = time.monotonic()
        with (out / (name + '.stdout')).open('wb') as stdout, (out / (name + '.stderr')).open('wb') as stderr:
            child = subprocess.Popen(['/usr/bin/time', '-v', '-o', str(out / (name + '.resources'))] + command,
                                     cwd=ROOT, stdout=stdout, stderr=stderr, start_new_session=True,
                                     preexec_fn=lambda: resource.setrlimit(resource.RLIMIT_AS, (8 * 1024**3, 8 * 1024**3)))
            try:
                code = child.wait(timeout=1800)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
                code = 124
        elapsed = time.monotonic() - started
        row = {'name': name, 'exit_code': code, 'seconds': elapsed,
               'sources_unchanged': v.sources() == initial,
               'streams_sha256': {suffix: v.sha(out / (name + '.' + suffix)) for suffix in ['stdout', 'stderr', 'resources']}}
        if code == 0 and not name.endswith(('-build', '-dump', '-corpus')):
            row['sample'] = json.loads((out / (name + '.stdout')).read_text())
            assert row['sample']['completed'] == row['sample']['iterations'] == 128
        rows.append(row)
        v.dump(out / 'result.json', {'commands': rows, 'passed': len(rows) == len(commands) and all(r['exit_code'] == 0 and r['sources_unchanged'] for r in rows)})
        print(json.dumps({'name': name, 'exit_code': code}), flush=True)
        assert row['sources_unchanged']
        if code: raise SystemExit(code)
    comparisons = {}
    for shape in shapes:
        left, right = [out / f'{shape}-{label}-dump.stdout' for label in ['baseline', 'candidate']]
        assert len(json.loads(left.read_text())) == len(json.loads(right.read_text())) == 128
        comparisons[shape] = {'rows': 128, 'identical': left.read_bytes() == right.read_bytes(),
                              'baseline_sha256': v.sha(left), 'candidate_sha256': v.sha(right),
                              'bytes': [left.stat().st_size, right.stat().st_size]}
    for row in rows:
        if row['name'].endswith('-corpus'):
            comparisons[row['name']] = {'identical': (out / (row['name'] + '.stdout')).read_bytes() == (E / 'baseline-v2/snapshot.stdout').read_bytes()}
    deterministic = {}
    for row in rows:
        if 'sample' in row:
            sample = row['sample']
            key = (sample['shape'], sample['cache'])
            facts = (sample['checksum'], sample['output_bytes'])
            assert deterministic.setdefault(key, facts) == facts, row['name']
    v.dump(out / 'exact-rows.json', comparisons)
    assert all(row['identical'] for row in comparisons.values())
    assert all(v.sha(ROOT / name) == digest for name, digest in inputs.items())
    paths = [target / 'baseline', target / 'candidate']
    paths += [ROOT / 'target/section-proof-reuse' / phase / 'libcsgrs.rlib' for phase in ['baseline-v2', 'candidate-v1']]
    v.dump(out / 'artifacts.json', {str(p.relative_to(ROOT)): {'sha256': v.sha(p), 'bytes': p.stat().st_size} for p in paths})


if __name__ == '__main__':
    main()
