"""Fork the frozen integer-leaf candidate for an owned coefficient merge trial."""
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
BASE = ROOT / 'target/small-integer-certificates/source'
WORK = ROOT / 'target/laurent-coefficient-merge'
SOURCE = WORK / 'source'
PARENT = ROOT / 'docs/evidence/2026-10-12-small-integer-certificates'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def main():
    original = json.loads(gzip.decompress((PARENT / 'baseline-source-hashes.json.gz').read_bytes()))
    delta = json.loads((PARENT / 'validation-v1/protocol.json').read_text())['source_changes_sha256']
    baseline = original | delta
    guard = json.loads((PARENT / 'preparation.json').read_text())['main_source_sha256']
    assert sources() == guard
    assert all(sha(BASE / name) == expected for name, expected in baseline.items())
    WORK.mkdir(exist_ok=False)
    with (E / 'copy.log').open('w') as log:
        subprocess.run(['cp', '-a', '--reflink=auto', str(BASE), str(SOURCE)],
                       stdout=log, stderr=subprocess.STDOUT, check=True)
    prefix = 'crates/hyperreal/src/computable/node/'
    path = SOURCE / (prefix + 'algebra.rs')
    text = path.read_text()
    before = '''    fn add(mut self, other: Self) -> Option<Self> {
        self.rational = self.rational + other.rational;
        for (radicand, coefficient) in other.radicals {
            self.insert_radical(radicand, coefficient)?;
        }
        for (atom, coefficient) in other.atoms {
            self.insert_atom(atom, coefficient);
        }
        Some(self)
    }'''
    after = '''    fn add_in_place(&mut self, other: Self) -> Option<()> {
        self.rational = &self.rational + other.rational;
        for (radicand, coefficient) in other.radicals {
            self.insert_radical(radicand, coefficient)?;
        }
        for (atom, coefficient) in other.atoms {
            self.insert_atom(atom, coefficient);
        }
        Some(())
    }'''
    assert text.count(before) == 1
    text = text.replace(before, after)
    before = '''                let sum = self.terms[index].1.clone().add(coefficient)?;
                if sum.is_zero() {
                    self.terms.remove(index);
                } else {
                    self.terms[index].1 = sum;
                }'''
    after = '''                // This temporary polynomial owns the coefficient vectors.
                // Every caller discards it if merging reaches a recognition
                // limit; no partially merged value is published or reused.
                self.terms[index].1.add_in_place(coefficient)?;
                if self.terms[index].1.is_zero() {
                    self.terms.remove(index);
                }'''
    assert text.count(before) == 1
    path.write_text('#[cfg(test)]\nmod laurent_merge_tests;\n\n' + text.replace(before, after))
    shutil.copyfile(E / 'laurent_merge_tests.rs', SOURCE / (prefix + 'laurent_merge_tests.rs'))
    with (E / 'prepare-format.log').open('w') as log:
        subprocess.run(['rustfmt', '--edition', '2024', str(path)], stdout=log,
                       stderr=subprocess.STDOUT, check=True)
    payload = json.dumps(baseline, sort_keys=True).encode()
    (E / 'baseline-source-hashes.json.gz').write_bytes(gzip.compress(payload, mtime=0))
    record = {
        'scope': __doc__, 'base_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'runner_sha256': sha(Path(__file__)), 'main_source_sha256': guard,
        'baseline_source_count': len(baseline), 'baseline_hashes_sha256': sha(E / 'baseline-source-hashes.json.gz'),
        'baseline': str(BASE), 'source': str(SOURCE), 'target': str(WORK / 'target'),
        'changed_paths': [prefix + 'algebra.rs', prefix + 'laurent_merge_tests.rs'],
        'contract': [
            'Only the equal-exponent coefficient merge changes. The existing owned coefficient is updated without a temporary vector clone.',
            'Preserve arithmetic and insertion order, all recognition limits, memo ownership at recursive traversal boundaries, retained publication and original retry.',
            'A failed in-place merge leaves a partial temporary polynomial, which every caller discards before fallback. No reference to this temporary is retained.',
            'No dependency, public layout, new cache, recursive prepass or native geometry change.',
            'Use three unchanged fixed scalar cost probes and keep every adverse row, allocation, retained-memory and binary-size observation.',
            'Original full-board 120-second worker and copper-neck 60-second check assertions remain unchanged.',
        ],
        'prior_rejections': ['Both direct quadratic consumers', 'Default and union SmallVec temporary normal-form layouts'],
        'runtime_admitted': False,
    }
    (E / 'preparation.json').write_text(json.dumps(record, indent=2) + '\n')
    assert sources() == guard
    print(json.dumps({'baseline_source_count': len(baseline), 'source': str(SOURCE), 'runtime_admitted': False}))


if __name__ == '__main__':
    main()
