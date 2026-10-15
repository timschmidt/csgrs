"""Run one immutable, focused constructor cost probe after qualification."""
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
    target = ROOT / 'target/rounded-rectangle-fidelity/cost-v1'
    target.mkdir(exist_ok=False)
    source = E / 'rounded_cost_v1.rs'
    candidate = ROOT / 'target/rounded-rectangle-fidelity/native-v1/release'
    initial = v.sources()
    assert initial == json.loads(gzip.decompress((E / 'native-v1/source-hashes.json.gz').read_bytes()))
    assert json.loads((E / 'native-v1/result.json').read_text())['passed']
    inputs = {str(source): v.sha(source), str(Path(__file__)): v.sha(Path(__file__))}
    for path in (candidate / 'deps').glob('*.rlib'):
        inputs[str(path)] = v.sha(path)
    native_paths = sorted({str(path) for pattern in ['*/out', '*/out/lib']
                           for path in (candidate / 'build').glob(pattern) if path.is_dir()})
    command = ['rustc', '--edition=2024', '-C', 'opt-level=3', '-D', 'warnings', str(source),
               '--extern', 'csgrs=' + str(candidate / 'deps/libcsgrs.rlib'),
               '-L', 'dependency=' + str(candidate / 'deps'), '-o', str(target / 'candidate')]
    for native in native_paths:
        command += ['-L', 'native=' + native]
    commands = [['candidate-build', command]]
    for sample in range(3):
        for shape in ['octagon', 'benchmark', 'readme']:
            for cache in ['warm', 'miss']:
                commands.append([f'{sample}-{shape}-{cache}',
                                 [str(target / 'candidate'), shape, cache, '128']])
    protocol = {
        'kind': 'focused cost probe; not a supported-corpus performance or optimization claim',
        'started_unix_time': time.time(),
        'commands': commands, 'inputs_sha256': inputs,
        'rustc': subprocess.check_output(['rustc', '-vV'], text=True),
        'lock_sha256': v.sha(ROOT / 'Cargo.lock'),
        'candidate_source_record': str(E / 'native-v1/protocol.json'),
        'features': 'csgrs all-features; preserved release rlib from native-v1 renderer build',
        'workload': 'Corrected rounded_rectangle_with_context under STRICT. Octagon (4,2,1/2,n=1), original benchmark (12,8,2,n=8), original README binary64 literal ingress (2.4,1.5,.28,n=12). Warm repeats base width; misses cycle 128 integer width offsets to exceed the 64-entry ring FIFO. Scalar caches are not globally cold. 128 warmup and 128 measured construction+boundary materialization+oracle+drop operations; three fresh-process samples per case.',
        'certainty': 'Each result must report Certified. Every operation checks all eight exact tangents, one material boundary/no holes, and expected sample count. The one-chord fixture additionally checks independent exact area. Full semantic qualification is recorded in native-v1. No analytic-curve fidelity or prior incorrect-geometry speed comparison.',
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
        if code == 0 and not name.endswith('-build'):
            row['sample'] = json.loads((out / (name + '.stdout')).read_text())
            assert row['sample']['completed'] == row['sample']['iterations'] == 128
        rows.append(row)
        v.dump(out / 'result.json', {'commands': rows, 'passed': len(rows) == len(commands) and all(r['exit_code'] == 0 and r['sources_unchanged'] for r in rows)})
        print(json.dumps({'name': name, 'exit_code': code}), flush=True)
        assert row['sources_unchanged']
        if code:
            raise SystemExit(code)
    assert all(v.sha(Path(name)) == digest for name, digest in inputs.items())
    v.dump(out / 'artifacts.json', {str(path.relative_to(ROOT)): {'sha256': v.sha(path), 'bytes': path.stat().st_size}
                                  for path in [target / 'candidate', candidate / 'deps/libcsgrs.rlib']
                                  if path.is_relative_to(ROOT)})
    v.dump(out / 'library_sizes.json', {str(path): path.stat().st_size for path in [candidate / 'deps/libcsgrs.rlib']})


if __name__ == '__main__':
    main()
