#!/usr/bin/env python3
"""Reject changed certificate answers, lost uncertainty and corrupted mask bits."""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from verify_verus import ROOT, verifier  # noqa: E402
from test_bound_rejections import mutate_body  # noqa: E402

MUTATIONS = [
    ('CertifiedRealSign', 'sign', 'Some(sign)', 'None'),
    ('CertifiedRealSign', 'sign', 'Self::Unknown { .. } => None,', 'Self::Unknown { .. } => Some(RealSign::Zero),'),
    ('CertifiedRealSign', 'is_known', 'matches!(self, Self::Known { .. })', 'false'),
    ('CertifiedRealOrdering', 'ordering', 'Some(ordering)', 'None'),
    ('CertifiedRealOrdering', 'ordering', 'Self::Unknown { .. } => None,', 'Self::Unknown { .. } => Some(core::cmp::Ordering::Equal),'),
    ('CertifiedRealOrdering', 'is_known', 'matches!(self, Self::Known { .. })', 'false'),
    ('CertifiedRealEquality', 'as_bool', 'Self::Equal { .. } => Some(true),', 'Self::Equal { .. } => Some(false),'),
    ('CertifiedRealEquality', 'as_bool', 'Self::Unknown { .. } => None,', 'Self::Unknown { .. } => Some(false),'),
    ('CertifiedRealEquality', 'is_known', '!matches!(self, Self::Unknown { .. })', 'matches!(self, Self::Unknown { .. })'),
    ('SymbolicDependencyMask', 'from_bits', 'Self(bits)', 'Self(0)'),
    ('SymbolicDependencyMask', 'bits', 'self.0', 'self.0 | 1'),
    ('SymbolicDependencyMask', 'contains', '== dependency.0', '!= dependency.0'),
    ('SymbolicDependencyMask', 'is_empty', 'self.0 == 0', 'self.0 != 0'),
    ('SymbolicDependencyMask', 'union', 'self.0 | other.0', 'self.0 & other.0'),
]


def mutate_impl(source, typename, method, before, after):
    start = source.index('impl ' + typename + ' {')
    cursor = source.index('{', start) + 1
    depth = 1
    while depth:
        depth += (source[cursor] == '{') - (source[cursor] == '}')
        cursor += 1
    changed = mutate_body(source[start:cursor], method, before, after)
    return source[:start] + changed + source[cursor:]


def main():
    verus = verifier()
    with tempfile.TemporaryDirectory(prefix='hyperreal-structural-rejections-') as temporary:
        root = Path(temporary)
        path = root / 'structural.rs'
        shutil.copy2(ROOT / 'src/structural.rs', path)
        original = path.read_text()
        (root / 'lib.rs').write_text('#![feature(proc_macro_hygiene)]\nmod structural;\n')
        command = [str(verus), '--edition=2024', '--crate-type=lib', '--no-cheating', str(root / 'lib.rs')]
        baseline = subprocess.run(command, capture_output=True, text=True)
        if baseline.returncode:
            raise RuntimeError('Unmutated structural proofs failed:\n' + baseline.stdout + baseline.stderr)
        changes = [(typename + '::' + method, mutate_impl(original, typename, method, before, after))
                   for typename, method, before, after in MUTATIONS]
        for name, expression in [('NONE', '0'), ('PI', '1 << 0'), ('EXP', '1 << 1'),
                                 ('SQRT', '1 << 2'), ('LOG', '1 << 3'), ('TRIG', '1 << 4'), ('OPAQUE', '1 << 15')]:
            before = f'pub const {name}: Self = Self({expression});'
            assert original.count(before) == 1
            after = f'pub const {name}: Self = Self({1 if name == "NONE" else 0});'
            changes.append(('SymbolicDependencyMask::' + name, original.replace(before, after)))
        for name, changed in changes:
            path.write_text(changed)
            result = subprocess.run(command, capture_output=True, text=True)
            path.write_text(original)
            if result.returncode == 0 or 'postcondition not satisfied' not in result.stderr:
                raise RuntimeError(f'Expected structural contract rejection for {name}:\n'
                                   + result.stdout + result.stderr)
            print('Rejected incorrect public carrier: ' + name, flush=True)


if __name__ == '__main__':
    main()
