"""Freeze instrumentation of existing retained quadratic-projection operations.

The combined endpoint/P4 parent stays unchanged. Address-only thread contexts
identify the quadratic query root. Existing cache access, copy and retry sites
are observed without new scalar queries, owners, arithmetic or cache writes.
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
PARENT = ROOT / 'docs/evidence/2026-10-13-native-tower-sign-retention'
WORK = ROOT / 'target/retained-quadratic-projection-v2'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def main():
    preparation = json.loads((PARENT / 'preparation-v1/protocol.json').read_text())
    scope = json.loads((PARENT / 'next-diagnostic-scope-v1.json').read_text())
    assessment = json.loads((PARENT / 'qualification-assessment-v2.json').read_text())
    decision = json.loads((PARENT / 'decision-v1.json').read_text())
    assert decision['decision'] == 'combined-correctness-passes-original-board-cap-still-fails'
    assert decision['qualification_assessment_sha256'] == sha(PARENT / 'qualification-assessment-v2.json')
    assert assessment['format_strict_stack_clippy_and_docs_passed']
    assert scope['parent_preparation_sha256'] == sha(PARENT / 'preparation-v1/protocol.json')
    original = Path(preparation['source'])
    expected = json.loads(gzip.decompress((PARENT / 'preparation-v1/source-hashes.json.gz').read_bytes()))
    main_guard = sources()
    pending = preparation['pending_native_sha256']
    assert len(expected) == 10131 and len(main_guard) == 1134 and len(pending) == 35
    assert main_guard == preparation['main_source_sha256']
    assert all(sha(original / p) == h for p, h in expected.items())
    assert all(sha(ROOT / p) == h for p, h in pending.items())
    WORK.mkdir(exist_ok=False)
    out = E / 'preparation-v2'
    out.mkdir(exist_ok=False)
    source = WORK / 'source'
    source.mkdir()
    for name in expected:
        destination = source / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(original / name, destination)
    helper = 'crates/hyperreal/src/retained_projection_diagnostic.rs'
    site_tests = 'crates/hyperreal/src/computable/node/retained_projection_site_tests.rs'
    lib = 'crates/hyperreal/src/lib.rs'
    algebra = 'crates/hyperreal/src/computable/node/algebra.rs'
    representation = 'crates/hyperreal/src/computable/node/representation.rs'
    structural = 'crates/hyperreal/src/computable/node/structural_analysis.rs'
    app = 'crates/hyperdrc/src/app.rs'
    for name in [helper, site_tests]:
        shutil.copyfile(E / Path(name).name, source / name)
    patches = {}

    def replace(name, before, after):
        text = (source / name).read_text()
        assert text.count(before) == 1, (name, before)
        (source / name).write_text(text.replace(before, after))
        patches.setdefault(name, []).append({'before': before, 'after': after})

    replace(lib, 'mod trace;', 'pub mod retained_projection_diagnostic;\n\nmod trace;')
    old = '''    fn pi_laurent_polynomial(&self) -> Option<PiLaurentPolynomial> {
        let guard = self
            .cell()?
            .read()
            .unwrap_or_else(|error| error.into_inner());
        Some(guard.pi_laurent_polynomial.as_deref()?.to_polynomial())
    }'''
    new = '''    fn pi_laurent_polynomial(&self) -> Option<PiLaurentPolynomial> {
        let diagnostic_address = std::ptr::from_ref(self) as usize;
        let cell = self.cell();
        crate::retained_projection_diagnostic::record_cell(diagnostic_address, cell.is_some());
        let guard = cell?.read().unwrap_or_else(|error| error.into_inner());
        let Some(polynomial) = guard.pi_laurent_polynomial.as_deref() else {
            crate::retained_projection_diagnostic::record_missing_form(diagnostic_address);
            return None;
        };
        let result = polynomial.to_polynomial();
        crate::retained_projection_diagnostic::record_copy(diagnostic_address, || {
            let (radicals, vectors) =
                polynomial
                    .terms
                    .iter()
                    .fold((0, 0), |(radicals, vectors), (_, coefficient)| {
                        let length = coefficient.radicals.len();
                        (radicals + length, vectors + usize::from(length != 0))
                    });
            (
                polynomial.terms.len(),
                polynomial
                    .terms
                    .first()
                    .map(|(power, coefficient)| (*power, coefficient.radicals.len())),
                radicals,
                vectors,
            )
        });
        Some(result)
    }'''
    replace(representation, old, new)
    original_structural = (source / structural).read_text()
    prefix = '#[cfg(test)]\nmod retained_projection_site_tests;\n\n'
    replace(structural, original_structural, prefix + original_structural)
    begin = '    pub(crate) fn exact_quadratic_surd_parts('
    text = (source / structural).read_text()
    start = text.index(begin)
    end = text.index('    #[cfg(test)]\n    pub(super) fn planning_msd(', start)
    body = text[start:end]
    instrumented = body.replace(
        '        // Project the shared bounded normal form after cancellation.',
        '        let mut projection_probe = crate::retained_projection_diagnostic::RootScope::begin(\n'
        '            std::ptr::from_ref(&self.internal.cache) as usize,\n'
        '        );\n'
        '        // Project the shared bounded normal form after cancellation.',
    ).replace(
        '        crate::trace_dispatch!("computable", "structural", "quadratic-surd");',
        '        projection_probe.success();\n'
        '        crate::trace_dispatch!("computable", "structural", "quadratic-surd");',
    )
    assert body != instrumented
    replace(structural, body, instrumented)
    replace(algebra,
            '        if reused {\n            self.pi_laurent_polynomial(budget).and_then(project)',
            '        if reused {\n'
            '            crate::retained_projection_diagnostic::record_retry(std::ptr::from_ref(\n'
            '                &self.internal.cache,\n'
            '            ) as usize);\n'
            '            self.pi_laurent_polynomial(budget).and_then(project)')
    begin = ('    pub(crate) fn complex_project_zip_kicad_board_completes_smoke_check_suite() {\n'
             '        let _geometry_guard = super::COMPLEX_PROJECT_GEOMETRY_TEST_LOCK\n'
             '            .lock()\n'
             '            .unwrap_or_else(std::sync::PoisonError::into_inner);')
    replace(app, begin, begin + '\n        let _projection_report = hyperreal::retained_projection_diagnostic::Report::begin();')
    (out / 'replacements.json').write_text(json.dumps(patches, indent=2) + '\n')
    with (out / 'format.log').open('w') as log:
        for name in [lib, algebra, representation, app]:
            subprocess.run(['rustfmt', '--edition', '2024', str(source / name)],
                           stdout=log, stderr=subprocess.STDOUT, check=True)
    # Leave the legacy included structural-analysis file outside whole-file
    # formatting. Every non-whitespace token still needs an explicit edit.
    for name, replacements in patches.items():
        recovered = ''.join((source / name).read_text().split())
        for replacement in reversed(replacements):
            after = ''.join(replacement['after'].split())
            before = ''.join(replacement['before'].split())
            assert recovered.count(after) == 1, name
            recovered = recovered.replace(after, before)
        assert recovered == ''.join((original / name).read_text().split()), name
    assert sha(source / helper) == sha(E / Path(helper).name)
    assert sha(source / site_tests) == sha(E / Path(site_tests).name)
    current = {p: sha(source / p) for p in sorted(set(expected) | {helper, site_tests})}
    changes = {p: h for p, h in current.items() if expected.get(p) != h}
    assert set(changes) == {helper, site_tests, lib, algebra, representation, structural, app}
    for name, data in [('parent-source-hashes.json.gz', expected), ('source-hashes.json.gz', current)]:
        (out / name).write_bytes(gzip.compress((json.dumps(data, sort_keys=True) + '\n').encode(), mtime=0))
    with tarfile.open(out / 'source-delta.tar.gz', 'w:gz') as archive:
        for name in changes:
            archive.add(source / name, arcname=name)
    patch = ''.join(''.join(difflib.unified_diff(
        (original / p).read_text().splitlines(True) if p in expected else [],
        (source / p).read_text().splitlines(True), 'a/' + p, 'b/' + p)) for p in changes)
    (out / 'source.patch').write_text(patch)
    protocol = {
        'scope': __doc__, 'runner_sha256': sha(Path(__file__)),
        'preparation_failure_sha256': sha(E / 'preparation-failure-v1.json'),
        'diagnostic_runner_sha256': sha(E / 'diagnose_v2.py'),
        'assessor_sha256': sha(E / 'assess_v2.py'),
        'helper_input_sha256': sha(E / Path(helper).name),
        'site_tests_input_sha256': sha(E / Path(site_tests).name),
        'base_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        'previous_goal_turn_classification': 'progress',
        'parent_preparation_sha256': sha(PARENT / 'preparation-v1/protocol.json'),
        'parent_assessment_sha256': sha(PARENT / 'qualification-assessment-v2.json'),
        'diagnostic_scope_sha256': sha(PARENT / 'next-diagnostic-scope-v1.json'),
        'parent_source': str(original), 'source': str(source),
        'target': str(WORK / 'target'),
        'parent_files': len(expected), 'diagnostic_files': len(current),
        'main_source_sha256': main_guard, 'pending_native_sha256': pending,
        'source_changes_sha256': changes,
        'parent_hashes_sha256': sha(out / 'parent-source-hashes.json.gz'),
        'source_hashes_sha256': sha(out / 'source-hashes.json.gz'),
        'archive_sha256': sha(out / 'source-delta.tar.gz'),
        'patch_sha256': sha(out / 'source.patch'),
        'replacements_sha256': sha(out / 'replacements.json'),
        'original_tokens_recovered_from_explicit_replacements': True,
        'source_review': 'The cache accessor still calls cell, takes one read guard, borrows the same stored form and copies it exactly once. Its existing output is named so completed copies can be counted; no extra scalar owner is cloned. Missing-cell and missing-form paths return at the same points. The guard still drops at function exit. Query contexts store only integer addresses and never dereference them. Original retries are observed after the first memo is dropped.',
        'runtime_admitted': False, 'plan_complete': False,
    }
    assert sources() == main_guard
    assert all(sha(ROOT / p) == h for p, h in pending.items())
    assert all(sha(original / p) == h for p, h in expected.items())
    assert all(sha(source / p) == h for p, h in current.items())
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    print(json.dumps({'source_files': len(current), 'changed_files': len(changes),
                      'original_tokens_recovered': True, 'parent_and_main_unchanged': True}))


if __name__ == '__main__':
    main()
