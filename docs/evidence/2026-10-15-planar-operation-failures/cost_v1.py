"""Run one immutable, focused planar-operation cost probe after qualification."""
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
    target = ROOT / 'target/planar-operation-failures/cost-v1'
    target.mkdir(exist_ok=False)
    source = E / 'planar_cost_v1.rs'
    baseline = ROOT / 'target/sampled-profile-failures/native-v1/release'
    candidate = ROOT / 'target/planar-operation-failures/native-v2/release'
    initial = v.sources()
    assert initial == json.loads(gzip.decompress((E / 'native-v2/source-hashes.json.gz').read_bytes()))
    assert json.loads((E / 'native-v2/result.json').read_text())['passed']
    inputs = {str(source): v.sha(source), str(Path(__file__)): v.sha(Path(__file__))}
    identity = E / 'cost-build-identity-v1.json'
    inputs[str(identity)] = v.sha(identity)
    for row in json.loads(identity.read_text())['compilation_units']:
        assert v.sha(ROOT / row['path']) == row['sha256']
        inputs[str(ROOT / row['path'])] = row['sha256']
    for directory in [baseline, candidate]:
        for path in (directory / 'deps').glob('*.rlib'):
            inputs[str(path)] = v.sha(path)
        for path in (directory / 'build').rglob('*.a'):
            inputs[str(path)] = v.sha(path)
    native_paths = sorted({str(path) for directory in [baseline, candidate]
                           for pattern in ['*/out', '*/out/lib']
                           for path in (directory / 'build').glob(pattern) if path.is_dir()})
    commands = []
    for label, directory in [('baseline', baseline), ('candidate', candidate)]:
        command = ['rustc', '--edition=2024', '-C', 'opt-level=3', '-D', 'warnings', str(source),
                   '--check-cfg', 'cfg(candidate_api)',
                   '--extern', 'csgrs=' + str(directory / 'deps/libcsgrs.rlib'),
                   '--extern', 'hyperlattice=' + str(directory / 'deps/libhyperlattice-64c0cb923a8aa061.rlib'),
                   '-L', 'dependency=' + str(directory / 'deps'), '-o', str(target / label)]
        if label == 'candidate': command += ['--cfg', 'candidate_api']
        for native in native_paths: command += ['-L', 'native=' + native]
        commands.append([label + '-build', command])
    shapes = ['translate', 'scale', 'rotate', 'matrix', 'query', 'adapter_translate', 'adapter_scale', 'adapter_rotate']
    for shape in shapes:
        for label in ['baseline', 'candidate']:
            commands.append([f'{shape}-{label}-dump', [str(target / label), shape, 'dump', '128']])
    for sample in range(3):
        labels = ['baseline', 'candidate'] if sample % 2 == 0 else ['candidate', 'baseline']
        for shape in shapes:
            for cache in ['warm', 'varied']:
                for label in labels:
                    commands.append([f'{sample}-{shape}-{cache}-{label}',
                                     [str(target / label), shape, cache, '128']])
    protocol = {
        'kind': 'focused cost probe; not a supported-corpus performance or optimization claim',
        'prior_trial': None,
        'started_unix_time': time.time(),
        'commands': commands, 'inputs_sha256': inputs,
        'rustc': subprocess.check_output(['rustc', '-vV'], text=True),
        'lock_sha256': v.sha(ROOT / 'Cargo.lock'),
        'baseline_source_record': str(ROOT / 'docs/evidence/2026-10-15-sampled-profile-failures/native-v1/protocol.json'),
        'candidate_source_record': str(E / 'native-v2/protocol.json'),
        'features': 'csgrs all-features; preserved baseline and final native release rlibs',
        'workload': 'Eight strict successful paths on rectangles of integer width 2..129 and height 3: native translation, reflection/scaling, quarter-turn, affine shear matrix, inside/boundary/outside queries, and scalar-adapter translation/scaling/rotation. Both revisions execute the same source except Result unwrapping for the API migration. Timing includes input construction, operation, independent exact coordinate and area proofs, boundary materialization, exact-expression serialization, checksum and drop. Each sample has 128 warmup and 128 timed operations. Warm repeats width 2; varied cycles 128 widths. Three fresh-process samples per revision/case with alternating order. Varied does not mean globally cold scalar caches.',
        'certainty': 'Every measured completion uses strict APIs with exact rational inputs and independent proofs of area and all four output vertices. Queries require the independently known inside, boundary and outside facts. No errors or undecided results contribute timing. All 128 exact serialized output variants per case must agree byte-for-byte. Corrected nonunit/projective/out-of-plane inputs are regressions, not matched performance workloads. This is operation-local evidence, not end-to-end provenance or analytic fidelity.',
        'allocations': 'System allocator wrapper counts successful alloc/alloc_zeroed/realloc calls, requested bytes, live-byte delta and peak live bytes above post-warmup state. These are Rust global-allocation counts, not every native allocation or RSS. Instrumentation contributes time.',
        'hardware': subprocess.check_output(['lscpu'], text=True),
        'limits': '1800s external wall and 8 GiB per-process address space. Shared host; no CPU pinning/frequency control. No internal work/cancellation bound implied.',
        'environment': {key: os.environ[key] for key in ['RUSTFLAGS', 'RUSTC_WRAPPER', 'RUSTUP_TOOLCHAIN'] if key in os.environ},
    }
    v.dump(out / 'protocol.json', protocol)
    rows = []
    for name, command in commands:
        assert v.sources() == initial
        assert v.sha(source) == inputs[str(source)]
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
        row = {'name': name, 'exit_code': code, 'seconds': time.monotonic() - started,
               'sources_unchanged': v.sources() == initial,
               'streams_sha256': {suffix: v.sha(out / (name + '.' + suffix)) for suffix in ['stdout', 'stderr', 'resources']}}
        if code == 0 and not name.endswith(('-build', '-dump')):
            row['sample'] = json.loads((out / (name + '.stdout')).read_text())
            assert row['sample']['completed'] == row['sample']['iterations'] == 128
        rows.append(row)
        v.dump(out / 'result.json', {'commands': rows, 'passed': len(rows) == len(commands) and all(r['exit_code'] == 0 and r['sources_unchanged'] for r in rows)})
        print(json.dumps({'name': name, 'exit_code': code}), flush=True)
        assert row['sources_unchanged']
        if code:
            raise SystemExit(code)
    row_comparisons = {}
    for shape in shapes:
        left, right = [out / f'{shape}-{label}-dump.stdout' for label in ['baseline', 'candidate']]
        assert len(json.loads(left.read_text())) == len(json.loads(right.read_text())) == 128
        assert left.read_bytes() == right.read_bytes(), shape
        row_comparisons[shape] = {'rows': 128, 'bytes': left.stat().st_size, 'sha256': v.sha(left)}
    paired = {}
    for row in rows:
        if 'sample' not in row: continue
        key, label = row['name'].rsplit('-', 1)
        paired.setdefault(key, {})[label] = row['sample']
    assert all(pair['baseline']['checksum'] == pair['candidate']['checksum'] and
               pair['baseline']['output_bytes'] == pair['candidate']['output_bytes']
               for pair in paired.values())
    v.dump(out / 'exact-rows.json', row_comparisons)
    assert all(v.sha(Path(name)) == digest for name, digest in inputs.items())
    v.dump(out / 'artifacts.json', {str(path.relative_to(ROOT)): {'sha256': v.sha(path), 'bytes': path.stat().st_size}
                                  for path in [target / 'baseline', target / 'candidate',
                                               baseline / 'deps/libcsgrs.rlib', candidate / 'deps/libcsgrs.rlib']
                                  if path.is_relative_to(ROOT)})
    v.dump(out / 'library_sizes.json', {str(path): path.stat().st_size for path in [baseline / 'deps/libcsgrs.rlib', candidate / 'deps/libcsgrs.rlib']})


if __name__ == '__main__':
    main()
