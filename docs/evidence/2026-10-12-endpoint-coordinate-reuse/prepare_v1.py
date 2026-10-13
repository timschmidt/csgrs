"""Fork metadata-first validation for exact negative coordinate-proof reuse.

Global Point2 PartialEq and later whole endpoint queries are unchanged. The
new helper keeps the same x/y query order and temporary drop boundaries.
Only a certified NonZero coordinate can omit a same-endpoint policy retry,
and only when preview and terminal approximation are inactive. Unknown
comparisons retain the original complete retry. No owners are cached.
"""
import difflib
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
PARENT = ROOT / 'docs/evidence/2026-10-12-region-metadata-first'
BASE = ROOT / 'target/region-metadata-first/source'
WORK = ROOT / 'target/endpoint-coordinate-reuse'
SOURCE = WORK / 'source'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def main():
    parent = json.loads((PARENT / 'preparation.json').read_text())
    baseline = json.loads(gzip.decompress((PARENT / 'baseline-source-hashes.json.gz').read_bytes()))
    baseline |= parent['source_changes_sha256']
    guard = parent['main_source_sha256']
    assert sources() == guard
    assert all(sha(BASE / name) == digest for name, digest in baseline.items())
    WORK.mkdir(exist_ok=False)
    with (E / 'copy.log').open('w') as log:
        subprocess.run(['cp', '-a', '--reflink=auto', str(BASE), str(SOURCE)], stdout=log, stderr=subprocess.STDOUT, check=True)

    def replace(name, before, after):
        path = SOURCE / name
        text = path.read_text()
        assert text.count(before) == 1, name
        path.write_text(text.replace(before, after))

    point = 'crates/hypercurve/src/point.rs'
    segment = 'crates/hypercurve/src/segment.rs'
    helper = 'crates/hypercurve/src/segment/endpoint_proof_tests.rs'
    facts = 'crates/hyperreal/src/real/arithmetic/facts.rs'
    compact = 'crates/hyperreal/src/real/arithmetic/pi_laurent_compaction.rs'
    replace(point, '    /// Constructs a point from values convertible into Real coordinates.', '''    /// Retains the facts from the same lazy coordinate queries as `PartialEq`.
    ///
    /// `Some(false)` requires a nonzero coordinate difference. `None` keeps
    /// the distinction that boolean equality cannot express. An unknown x
    /// difference does not cause an additional query of y.
    pub(crate) fn coordinate_equality(&self, other: &Self) -> Option<bool> {
        if self.shares_storage(other) {
            return Some(true);
        }
        // Drop the x difference before inspecting y, as in the lazy boolean
        // operands in PartialEq. Retaining it could alter scalar sharing.
        let x = (self.x() - other.x()).zero_status();
        match x {
            ZeroKnowledge::NonZero => return Some(false),
            ZeroKnowledge::Unknown => return None,
            ZeroKnowledge::Zero => {},
        }
        let y = (self.y() - other.y()).zero_status();
        match y {
            ZeroKnowledge::Zero => Some(true),
            ZeroKnowledge::NonZero => Some(false),
            ZeroKnowledge::Unknown => None,
        }
    }

    /// Constructs a point from values convertible into Real coordinates.''')
    replace(segment, '''        if point == self.start() || point == self.end() {
            return Some(true);
        }
        let at_start = points_equal(point, self.start(), policy);
        if at_start == Some(true) {
            return Some(true);
        }
        match points_equal(point, self.end(), policy) {
            Some(true) => Some(true),
            Some(false) if at_start == Some(false) => Some(false),
            Some(false) | None => None,
        }''', '''        let initial_start = point.coordinate_equality(self.start());
        if initial_start == Some(true) {
            return Some(true);
        }
        let initial_end = point.coordinate_equality(self.end());
        if initial_end == Some(true) {
            return Some(true);
        }
        // A certified nonzero coordinate already excludes this endpoint.
        // Preview and terminal policies retain their original query sequence;
        // an inconclusive exact comparison must also keep its full retry.
        let reuse_negative = !policy.is_edge_preview() && !policy.permits_approximate_512();
        let at_start = if reuse_negative { initial_start } else { None }
            .or_else(|| points_equal(point, self.start(), policy));
        if at_start == Some(true) {
            return Some(true);
        }
        let at_end = if reuse_negative { initial_end } else { None }
            .or_else(|| points_equal(point, self.end(), policy));
        match at_end {
            Some(true) => Some(true),
            Some(false) if at_start == Some(false) => Some(false),
            Some(false) | None => None,
        }''')
    replace(segment, '#[cfg(test)]\nmod retained_delta_tests {', '#[cfg(test)]\nmod endpoint_proof_tests;\n\n#[cfg(test)]\nmod retained_delta_tests {')
    replace(facts, '''    /// means only that the bounded normal form does not cover the expression.
    pub fn exact_rational_normal_form''', '''    /// means only that the bounded normal form does not cover the expression.
    /// Successful retained certificates may strengthen later queries. Each
    /// recognition pass expands at most 512 nodes; an unsuccessful pass that
    /// reused a certificate may retry the original recognizer once. These
    /// bounds count expression expansion, not integer operand sizes.
    pub fn exact_rational_normal_form''')
    replace(facts, '''    /// original exact expression remains available unchanged.
    pub fn exact_quadratic_normal_form''', '''    /// original exact expression remains available unchanged.
    /// Recognition uses the two-pass expansion bound described by
    /// [`Self::exact_rational_normal_form`].
    pub fn exact_quadratic_normal_form''')
    replace(compact, '''    /// form, without approximation or increased recognition limits. It can keep
    /// repeated exact sums usable when their unreduced arithmetic graphs would
    /// exceed a later predicate's optional symbolic recognition budget.''', '''    /// form without approximation. Each recognition pass keeps its 512-node
    /// limit; an unsuccessful pass using retained certificates may retry the
    /// original recognizer once. Retained proofs may strengthen later queries.
    /// These limits bound expression expansion, not integer operand sizes.
    /// It can keep repeated exact sums usable when their unreduced arithmetic
    /// graphs would exceed a later predicate's optional recognition budget.''')
    (SOURCE / helper).parent.mkdir(exist_ok=True)
    shutil.copyfile(E / 'endpoint_proof_tests.rs', SOURCE / helper)
    with (E / 'prepare-format.log').open('w') as log:
        for name in [point, segment, facts, compact]:
            subprocess.run(['rustfmt', '--edition', '2024', str(SOURCE / name)], stdout=log, stderr=subprocess.STDOUT, check=True)
    changed = [point, segment, helper, facts, compact]
    current = {name: sha(SOURCE / name) for name in sorted(set(baseline) | {helper})}
    assert {name for name, digest in current.items() if baseline.get(name) != digest} == set(changed)
    with tarfile.open(E / 'candidate-delta.tar.gz', 'w:gz') as archive:
        for name in changed:
            archive.add(SOURCE / name, arcname=name)
    patch = ''.join(''.join(difflib.unified_diff(
        (BASE / name).read_text().splitlines(True) if name in baseline else [],
        (SOURCE / name).read_text().splitlines(True), 'a/' + name, 'b/' + name)) for name in changed)
    (E / 'candidate.patch').write_text(patch)
    with (E / 'baseline-source-hashes.json.gz').open('xb') as raw:
        with gzip.GzipFile(fileobj=raw, mode='wb', mtime=0, filename='') as stream:
            stream.write((json.dumps(baseline, indent=2) + '\n').encode())
    record = {
        'scope': __doc__, 'base_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'runner_sha256': sha(Path(__file__)),
        'diagnostic_assessment_sha256': sha(ROOT / 'docs/evidence/2026-10-12-endpoint-coordinate-facts/assessment-v2.json'),
        'main_source_sha256': guard, 'baseline_source': str(BASE), 'candidate_source': str(SOURCE),
        'baseline_hashes_sha256': sha(E / 'baseline-source-hashes.json.gz'),
        'source_changes_sha256': {name: current[name] for name in changed},
        'candidate_archive_sha256': sha(E / 'candidate-delta.tar.gz'), 'candidate_patch_sha256': sha(E / 'candidate.patch'),
        'copied_files': len(baseline), 'runtime_admitted': False,
    }
    assert sources() == guard
    assert all(sha(BASE / name) == digest for name, digest in baseline.items())
    (E / 'preparation.json').write_text(json.dumps(record, indent=2) + '\n')
    print(json.dumps({'copied_files': len(baseline), 'changed': changed, 'runtime_admitted': False}))


if __name__ == '__main__':
    main()
