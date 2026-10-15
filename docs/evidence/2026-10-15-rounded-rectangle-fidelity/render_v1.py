"""Regenerate one affected README asset with unchanged renderer functions."""
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
    out = E / 'render-v1'
    out.mkdir(exist_ok=False)
    target = ROOT / 'target/rounded-rectangle-fidelity/render-v1'
    target.mkdir(exist_ok=False)
    seed = ROOT / 'target/rounded-rectangle-fidelity/render-build-v1/release'
    initial = v.sources()
    assert initial == json.loads(gzip.decompress((E / 'render-build-v1/source-hashes.json.gz').read_bytes()))
    assert json.loads((E / 'render-build-v1/result.json').read_text())['passed']
    derivation = json.loads((E / 'render-driver-v1.json').read_text())
    source = E / 'render_driver_v1.rs'
    assert v.sha(ROOT / derivation['source']) == derivation['source_sha256']
    assert v.sha(source) == derivation['derived_sha256']
    libraries = {}
    for line in (E / 'render-build-v1/renderer-build.stdout').read_text().splitlines():
        row = json.loads(line)
        if row.get('reason') == 'compiler-artifact' and row['target']['name'] in {'csgrs', 'hypercurve', 'hyperlattice', 'image'}:
            paths = [Path(p) for p in row['filenames'] if p.endswith('.rlib')]
            assert len(paths) == 1
            libraries[row['target']['name']] = paths[0]
    assert set(libraries) == {'csgrs', 'hypercurve', 'hyperlattice', 'image'}
    inputs = {str(source): v.sha(source), str(Path(__file__)): v.sha(Path(__file__)),
              str(ROOT / derivation['source']): derivation['source_sha256']}
    for path in (seed / 'deps').glob('*.rlib'):
        inputs[str(path)] = v.sha(path)
    inputs[str(seed / 'examples/readme_renders')] = v.sha(seed / 'examples/readme_renders')
    binary = target / 'rounded-readme-render'
    command = ['rustc', '--edition=2024', '-C', 'opt-level=3', '-A', 'dead_code', str(source), '-o', str(binary),
               '-L', 'dependency=' + str(seed / 'deps')]
    for name, path in sorted(libraries.items()):
        command += ['--extern', name + '=' + str(path)]
    for pattern in ['*/out', '*/out/lib']:
        for path in sorted((seed / 'build').glob(pattern)):
            if path.is_dir():
                command += ['-L', 'native=' + str(path)]
    commands = [['build', command]]
    for label in ['first', 'repeat']:
        directory = out / label
        directory.mkdir()
        commands.append([label, ['env', 'README_RENDER_OUTPUT_DIR=' + str(directory), str(binary)]])
    v.dump(out / 'protocol.json', {
        'commands': commands, 'inputs_sha256': inputs,
        'source_snapshot': str(E / 'render-build-v1/source-hashes.json.gz'),
        'derivation': derivation,
        'rustc': subprocess.check_output(['rustc', '-vV'], text=True),
        'scope': 'Only the affected asset; original full fixture and renderer. The full renderer regression remains open and unchanged.',
        'limits': '1800 seconds external wall per command and 8 GiB address space per process.',
    })
    rows = []
    for name, command in commands:
        assert v.sources() == initial
        assert all(v.sha(Path(path)) == digest for path, digest in inputs.items())
        start = time.monotonic()
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
        row = {'name': name, 'exit_code': code, 'seconds': time.monotonic() - start,
               'sources_unchanged': v.sources() == initial,
               'streams_sha256': {suffix: v.sha(out / (name + '.' + suffix)) for suffix in ['stdout', 'stderr', 'resources']}}
        rows.append(row)
        v.dump(out / 'result.json', {'commands': rows, 'passed': len(rows) == len(commands) and all(r['exit_code'] == 0 and r['sources_unchanged'] for r in rows)})
        print(json.dumps({'name': name, 'exit_code': code}), flush=True)
        assert row['sources_unchanged']
        if code:
            raise SystemExit(code)
    images = [out / label / 'rounded_rectangle.png' for label in ['first', 'repeat']]
    assert images[0].read_bytes() == images[1].read_bytes()
    v.dump(out / 'artifacts.json', {str(p.relative_to(ROOT)): {'sha256': v.sha(p), 'bytes': p.stat().st_size}
                                  for p in [binary, *images]})
    assert all(v.sha(Path(path)) == digest for path, digest in inputs.items())


if __name__ == '__main__':
    main()
