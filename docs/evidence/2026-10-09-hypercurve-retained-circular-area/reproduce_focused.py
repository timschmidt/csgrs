"""Replay the retained-endpoint and similarity regressions with frozen sources."""
import hashlib, json, subprocess, sys, time
from pathlib import Path

out = Path(sys.argv[1]).resolve()
out.mkdir(parents=True, exist_ok=False)
def sources():
    return {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(Path('crates/hypercurve').rglob('*.rs'))
            if 'target' not in p.parts}
initial = sources()
base = ['cargo', 'test', '--release', '--locked', '-p', 'hypercurve', '--all-features']
commands = [
    ('circular', base + ['--lib', 'retained_circular_tests', '--', '--test-threads=1']),
    ('root-views', base + ['--lib', 'represented_roots::tests', '--', '--test-threads=1']),
    ('similarity', base + ['--test', 'hypercurve_curve_region_promotion',
                         'algebraic_chords_and_round_centers_survive_exact_similarities',
                         '--', '--exact', '--test-threads=1']),
]
steps = []
for name, command in commands:
    assert sources() == initial
    started = time.monotonic()
    with (out / (name + '.log')).open('w') as log:
        code = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT).returncode
    steps.append(dict(name=name, command=command, exit_code=code,
                      elapsed_seconds=time.monotonic()-started))
    result = dict(steps=steps, source_hashes=initial, source_unchanged=sources()==initial)
    (out / 'manifest.json').write_text(json.dumps(result, indent=2)+'\n')
    print(name, code, flush=True)
    assert sources() == initial
    if code: raise SystemExit(code)
