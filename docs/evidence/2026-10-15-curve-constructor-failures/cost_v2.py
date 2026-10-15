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
    out = E / 'cost-v2'
    out.mkdir(exist_ok=False)
    target = ROOT / 'target/curve-constructor-failures/cost-v2'
    target.mkdir(exist_ok=False)
    source = E / 'constructor_cost_v2.rs'
    baseline = Path('/tmp/csgrs-curve-constructor-failures/baseline-v1/release')
    candidate = ROOT / 'target/curve-constructor-failures/native-v1/release'
    canonical = Path('/tmp/csgrs-curve-constructor-failures/candidate-v2/release')
    initial = v.sources()
    assert initial == json.loads(gzip.decompress((E / 'native-v1/source-hashes.json.gz').read_bytes()))
    assert json.loads((E / 'native-v1/result.json').read_text())['passed']
    inputs = {str(source): v.sha(source), str(Path(__file__)): v.sha(Path(__file__))}
    # The cleanup manifest identifies byte-identical dependencies retained in
    # canonical. The original baseline csgrs rlib itself remains untouched.
    for directory in [baseline, canonical, candidate]:
        for path in (directory / 'deps').glob('*.rlib'):
            inputs[str(path)] = v.sha(path)
    native_paths = sorted({str(path) for directory in [baseline, canonical, candidate]
                           for pattern in ['*/out', '*/out/lib']
                           for path in (directory / 'build').glob(pattern) if path.is_dir()})
    commands = []
    for label, directory in [('baseline', baseline), ('candidate', candidate)]:
        command = ['rustc', '--edition=2024', '-C', 'opt-level=3', '-D', 'warnings', str(source),
                   '--extern', 'csgrs=' + str(directory / 'deps/libcsgrs.rlib'),
                   '-o', str(target / label)]
        for dependency in [directory / 'deps', canonical / 'deps']:
            command += ['-L', 'dependency=' + str(dependency)]
        for native in native_paths:
            command += ['-L', 'native=' + native]
        commands.append([label + '-build', command])
    # Alternate which revision runs first; each case starts in a fresh process.
    for sample in range(5):
        labels = ['baseline', 'candidate'] if sample % 2 == 0 else ['candidate', 'baseline']
        for shape in ['rectangle', 'circle4', 'polygon']:
            for cache in ['warm', 'miss']:
                for label in labels:
                    commands.append([f'{sample}-{shape}-{cache}-{label}',
                                     [str(target / label), shape, cache, '1024']])
    protocol = {
        'kind': 'focused cost probe; not a supported-corpus performance or optimization claim',
        'started_unix_time': time.time(),
        'commands': commands, 'inputs_sha256': inputs,
        'rustc': subprocess.check_output(['rustc', '-vV'], text=True),
        'lock_sha256': v.sha(ROOT / 'Cargo.lock'),
        'baseline_source_record': str(E / 'baseline-v1/protocol.json'),
        'candidate_source_record': str(E / 'native-v1/protocol.json'),
        'features': 'csgrs no-default-features + curve; preserved release rlibs',
        'workload': 'Strict scalar-adapter rectangle(width,3), circle(width,4), polygon rectangle. Width=1 warm; widths 1..128 cycle for FIFO misses. 128 warmup operations, 1024 measured construction+exact-area assertion+output drop operations, five fresh-process samples per case and revision. All outputs materialized; no invalid or undecided case contributes a timing.',
        'certainty': 'Certified denotes successful strict constructors on exact inputs, with independently verified exact polygon area. It is operation-local, not an analytic-circle fidelity claim.',
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
            assert row['sample']['completed'] == row['sample']['iterations'] == 1024
        rows.append(row)
        v.dump(out / 'result.json', {'commands': rows, 'passed': len(rows) == len(commands) and all(r['exit_code'] == 0 and r['sources_unchanged'] for r in rows)})
        print(json.dumps({'name': name, 'exit_code': code}), flush=True)
        assert row['sources_unchanged']
        if code:
            raise SystemExit(code)
    assert all(v.sha(Path(name)) == digest for name, digest in inputs.items())
    v.dump(out / 'artifacts.json', {str(path.relative_to(ROOT)): {'sha256': v.sha(path), 'bytes': path.stat().st_size}
                                  for path in [target / 'baseline', target / 'candidate',
                                               baseline / 'deps/libcsgrs.rlib', candidate / 'deps/libcsgrs.rlib']
                                  if path.is_relative_to(ROOT)})
    v.dump(out / 'library_sizes.json', {str(path): path.stat().st_size for path in [baseline / 'deps/libcsgrs.rlib', candidate / 'deps/libcsgrs.rlib']})


if __name__ == '__main__':
    main()
