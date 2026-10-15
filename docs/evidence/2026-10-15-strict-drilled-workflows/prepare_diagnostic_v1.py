"""Prepare an isolated diagnostic, leaving production sources untouched."""
import difflib
import hashlib
import json
from pathlib import Path
import shutil

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OLD = ROOT / 'docs/evidence/2026-10-15-section-proof-reuse'
SOURCE = Path('/tmp/csgrs-strict-drilled-workflows/diagnostic-v1/hypermesh/src')
TARGET = ROOT / 'target/strict-drilled-workflows/diagnostic-v1'


def dump(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    assert not SOURCE.exists() and not (E / 'diagnostic-v1.json').exists()
    shutil.copytree(ROOT / 'crates/hypermesh/src', SOURCE)
    path = SOURCE / 'predicate.rs'
    before = path.read_text()
    start = before.index('pub(crate) fn classify_real(')
    end = before.index('\n#[cfg(test)]', start)
    function = before[start:end]
    function = function.replace('    decisions\n        .decide(', '    let result = decisions\n        .decide(', 1)
    suffix = '''        })
}
'''
    assert function.endswith(suffix)
    function = function[:-len(suffix)] + '''        });
    if result.is_err() {
        eprintln!("unresolved_scalar={}", value.to_json());
        eprintln!("unresolved_backtrace={}", std::backtrace::Backtrace::force_capture());
    }
    result
}
'''
    after = before[:start] + function + before[end:]
    path.write_text(after)
    (E / 'diagnostic-v1.patch').write_text(''.join(difflib.unified_diff(before.splitlines(True), after.splitlines(True), fromfile='crates/hypermesh/src/predicate.rs', tofile='diagnostic-v1/hypermesh/src/predicate.rs')))
    commands = json.loads((OLD / 'candidate-v1.json').read_text())
    library = next(cmd for name, cmd in commands if name == 'library-build').copy()
    native_deps = ['hyperlattice', 'hyperreal', 'hyperlimit', 'hypertri']
    native = ['rustc', '--crate-name', 'hypermesh', '--crate-type', 'rlib', '--edition=2024',
              '-C', 'opt-level=3', '-C', 'debuginfo=1', '-C', 'force-frame-pointers=yes', '-D', 'warnings',
              '--cfg', 'feature="dispatch-trace"', str(SOURCE / 'lib.rs'),
              '-L', next(arg for arg in library if arg.startswith('dependency=')),
              '-o', str(TARGET / 'libhypermesh.rlib')]
    for name in native_deps:
        native += ['--extern', next(arg for arg in library if arg.startswith(name + '='))]
    library[library.index('-o') + 1] = str(TARGET / 'libcsgrs.rlib')
    library = [arg if not arg.startswith('hypermesh=') else 'hypermesh=' + str(TARGET / 'libhypermesh.rlib') for arg in library]
    library += ['-L', 'dependency=' + str(TARGET), '-C', 'debuginfo=1', '-C', 'force-frame-pointers=yes']
    probe = next(cmd for name, cmd in json.loads((E / 'baseline-v1.json').read_text()) if name == 'probe-build').copy()
    probe[probe.index('-o') + 1] = str(TARGET / 'probe')
    probe = [arg if not arg.startswith('csgrs=') else 'csgrs=' + str(TARGET / 'libcsgrs.rlib') for arg in probe]
    probe += ['-L', 'dependency=' + str(TARGET), '-C', 'debuginfo=1', '-C', 'force-frame-pointers=yes']
    commands = [['prepare-target', ['mkdir', str(TARGET)]], ['native-build', native],
                ['library-build', library], ['probe-build', probe],
                ['drill-16-strict', [str(TARGET / 'probe'), '16', 'strict']],
                ['drill-32-strict', [str(TARGET / 'probe'), '32', 'strict']]]
    dump(E / 'diagnostic-v1.json', commands)
    dump(E / 'diagnostic-v1-seed.json', None)
    dump(E / 'diagnostic-v1-target.json', str(TARGET.relative_to(ROOT)))
    preserved = json.loads((E / 'baseline-v1-preserve.json').read_text())
    for path in [*SOURCE.rglob('*.rs'), Path(__file__), E / 'diagnostic-v1.json', E / 'diagnostic-v1.patch']:
        preserved[str(path)] = sha(path)
    dump(E / 'diagnostic-v1-preserve.json', preserved)


if __name__ == '__main__':
    main()
