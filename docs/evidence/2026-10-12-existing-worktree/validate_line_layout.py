"""Check the original compact-layout regression and retained-line certificates."""
import difflib
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

sys.dont_write_bytecode = True
from validate import ROOT, E, sources

sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    out = E / 'line-layout-fix'
    source = ROOT / 'crates/hypercurve/src/segment.rs'
    assert not (out / 'protocol.json').exists()
    initial = sources()
    original = out / 'original-segment.rs'
    previous = json.loads((E / 'geometry-validation/protocol.json').read_text())
    assert sha(original) == previous['sources_sha256']['crates/hypercurve/src/segment.rs']
    (out / 'candidate-segment.rs').write_bytes(source.read_bytes())
    (out / 'correction.patch').write_text(''.join(difflib.unified_diff(
        original.read_text().splitlines(keepends=True), source.read_text().splitlines(keepends=True),
        fromfile='before/segment.rs', tofile='after/segment.rs')))
    commands = [
        ('fmt', ['cargo', 'fmt', '--all', '--', '--check']),
        ('fragments', ['cargo', 'test', '--locked', '--release', '-p', 'hypercurve', '--all-features', '--lib', 'fragment::tests']),
        ('segments', ['cargo', 'test', '--locked', '--release', '-p', 'hypercurve', '--all-features', '--lib', 'segment::']),
    ]
    env = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
           'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never'}
    protocol = {
        'runner_sha256': sha(Path(__file__)), 'catalog_helper_sha256': sha(E / 'validate.py'),
        'sources_sha256': initial, 'commands': commands, 'environment': env,
        'original_segment_sha256': sha(original), 'candidate_segment_sha256': sha(source),
        'failure': 'The existing native_segment_storage_remains_compact test failed its unchanged LineSeg2 <= 48 byte assertion.',
        'change': 'Store flags with a single certificate enum. Displaced lines keep one Arc; offset provenance owns its optional displacement. Fragment deltas still use fragment endpoints, while support directions retain their source certificate.',
        'limits': 'Focused correctness and layout qualification. Broad native validation and full-board runtime admission remain open; no throughput or allocation-count claim.',
    }
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    results = []
    for name, command in commands:
        assert sources() == initial
        started = time.monotonic()
        with (out / (name + '.stdout')).open('w') as stdout, (out / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(command, cwd=ROOT, env=os.environ | env,
                                  stdout=stdout, stderr=stderr).returncode
        unchanged = sources() == initial
        result = {'name': name, 'exit_code': code, 'seconds': time.monotonic() - started,
                  'source_unchanged': unchanged, 'stdout_sha256': sha(out / (name + '.stdout')),
                  'stderr_sha256': sha(out / (name + '.stderr'))}
        results.append(result)
        complete = len(results) == len(commands) and all(r['exit_code'] == 0 and r['source_unchanged'] for r in results)
        (out / 'result.json').write_text(json.dumps({'phases': results, 'complete': complete}, indent=2) + '\n')
        print(json.dumps(result), flush=True)
        assert unchanged
        if code:
            raise SystemExit(code)


if __name__ == '__main__':
    main()
