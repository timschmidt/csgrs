"""Run one immutable, focused slice/projection cost probe after qualification."""
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
    target = ROOT / 'target/slice-projection-contract/cost-v1'
    target.mkdir(exist_ok=False)
    source = E / 'section_cost_v1.rs'
    baseline = ROOT / 'target/planar-operation-failures/cost-build-v1/release'
    candidate = ROOT / 'target/slice-projection-contract/native-v1/release'
    initial = v.sources()
    assert initial == json.loads(gzip.decompress((E / 'native-v1/source-hashes.json.gz').read_bytes()))
    assert json.loads((E / 'native-v1/result.json').read_text())['passed']
    inputs = {str(source): v.sha(source), str(Path(__file__)): v.sha(Path(__file__))}
    build_protocol = E / 'native-v1/protocol.json'
    build = json.loads(build_protocol.read_text())
    prior_protocol = ROOT / 'docs/evidence/2026-10-15-planar-operation-failures/cost-build-v1/protocol.json'
    prior_build = json.loads(prior_protocol.read_text())
    assert next(command for name, command in build['commands'] if name == 'renderer-build') == prior_build['commands'][0][1]
    inputs[str(build_protocol)] = v.sha(build_protocol)
    inputs[str(prior_protocol)] = v.sha(prior_protocol)
    identity = E / 'build-identity-v2/result.json'
    assert json.loads(identity.read_text())['passed']
    inputs[str(identity)] = v.sha(identity)
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
                   '--extern', 'hypercurve=' + str(directory / 'deps/libhypercurve-65c0df1f5cff6824.rlib'),
                   '--extern', 'hyperlattice=' + str(directory / 'deps/libhyperlattice-64c0cb923a8aa061.rlib'),
                   '-L', 'dependency=' + str(directory / 'deps'), '-o', str(target / label)]
        if label == 'candidate': command += ['--cfg', 'candidate_api']
        for native in native_paths: command += ['-L', 'native=' + native]
        commands.append([label + '-build', command])
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
    protocol = {
        'kind': 'focused cost probe; not a supported-corpus performance or optimization claim',
        'build_identity': 'build-identity-v2 confirms the same toolchain, 901 core/configuration source hashes, Cargo compilation-unit configuration and actual compiled dependency names/ABI. Fresh and seeded archive bytes and encoded crate hashes differ; the cause is not established. Each probe links against its own preserved exact dependency artifacts, with Rust type compatibility checked before timing. This rebuild variance is a limitation; no bit-identical-dependency or optimization claim is made.',
        'started_unix_time': time.time(),
        'commands': commands, 'inputs_sha256': inputs,
        'rustc': subprocess.check_output(['rustc', '-vV'], text=True),
        'lock_sha256': v.sha(ROOT / 'Cargo.lock'),
        'baseline_source_record': str(prior_protocol),
        'candidate_source_record': str(build_protocol),
        'features': 'csgrs all-features; preserved baseline and final native release rlibs',
        'workload': 'Four strict completed paths on cubes of integer width 2..129: middle, bottom and top horizontal sections, and XY projection. Both revisions execute the same source except Result unwrapping. Timing includes input construction, operation, independent exact area and membership, boundary materialization and a proof that every straight edge lies on the expected square, exact-expression serialization, checksum and drop. Each sample has 128 warmup and 128 timed operations. Warm repeats width 2; varied cycles 128 widths with default modeling-cache limits; disabled cycles those widths under CachePolicy::DISABLED. Three fresh-process samples per revision/case with alternating order. Varied and disabled do not mean globally cold scalar/kernel caches.',
        'certainty': 'Every measured completion uses strict APIs with exact rational inputs and independent semantic proofs. Errors and incorrect hollow or cancelled-shell results are excluded from matched timings and remain in the regression evidence. This correctness patch intentionally changes topology and may change traversal/subdivision/serialization. Actual exact rows and checksums are retained and compared, but cross-revision byte identity is not required or claimed. Determinism is checked within each revision across all three samples. Evidence is operation-local, not inherited provenance or analytic fidelity.',
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
        row_comparisons[shape] = {'rows': 128, 'identical_serialization': left.read_bytes() == right.read_bytes(),
                                 'baseline': {'bytes': left.stat().st_size, 'sha256': v.sha(left)},
                                 'candidate': {'bytes': right.stat().st_size, 'sha256': v.sha(right)},
                                 'semantic_equality': 'Both revisions passed the independent exact square-set oracle for all 128 inputs.'}
    deterministic = {}
    for row in rows:
        if 'sample' not in row: continue
        _, key = row['name'].split('-', 1)
        sample = row['sample']
        facts = (sample['checksum'], sample['output_bytes'])
        assert deterministic.setdefault(key, facts) == facts, key
    v.dump(out / 'exact-rows.json', row_comparisons)
    assert all(v.sha(Path(name)) == digest for name, digest in inputs.items())
    v.dump(out / 'artifacts.json', {str(path.relative_to(ROOT)): {'sha256': v.sha(path), 'bytes': path.stat().st_size}
                                  for path in [target / 'baseline', target / 'candidate',
                                               baseline / 'deps/libcsgrs.rlib', candidate / 'deps/libcsgrs.rlib']
                                  if path.is_relative_to(ROOT)})
    v.dump(out / 'library_sizes.json', {str(path): path.stat().st_size for path in [baseline / 'deps/libcsgrs.rlib', candidate / 'deps/libcsgrs.rlib']})


if __name__ == '__main__':
    main()
