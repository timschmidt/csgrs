"""Prepare allocation/caller diagnostics on both frozen endpoint-trial trees.

All original mathematical queries, branch order, scalar temporaries and owner
counts remain. New scopes retain integer masks only. Probe windows cover setup,
timed bodies, postconditions and intervening work. Instrumented timings are not
additional benchmark samples. Fixed earlier five-pair measurements stay closed.
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
TRIAL = ROOT / 'docs/evidence/2026-10-12-endpoint-coordinate-reuse'
WORK = ROOT / 'target/endpoint-cost-attribution'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def replace(text, before, after):
    assert text.count(before) == 1, before[:120]
    return text.replace(before, after)


def main():
    WORK.mkdir(exist_ok=False)
    out = E / 'preparation-v1'
    out.mkdir(exist_ok=False)
    preparation = json.loads((TRIAL / 'preparation.json').read_text())
    baseline = json.loads(gzip.decompress((TRIAL / 'baseline-source-hashes.json.gz').read_bytes()))
    maps = {'baseline': baseline, 'candidate': baseline | preparation['source_changes_sha256']}
    parents = {key: Path(preparation[key + '_source']) for key in maps}
    guard = preparation['main_source_sha256']
    assert sources() == guard
    variants = {}
    for variant, expected in maps.items():
        assert all(sha(parents[variant] / name) == digest for name, digest in expected.items())
        work = WORK / variant
        work.mkdir()
        source = work / 'source'
        with (out / (variant + '-copy.log')).open('w') as log:
            subprocess.run(['cp', '-a', '--reflink=auto', str(parents[variant]), str(source)], stdout=log, stderr=subprocess.STDOUT, check=True)
        changed = []

        def edit(name, before, after):
            path = source / name
            path.write_text(replace(path.read_text(), before, after))
            if name not in changed:
                changed.append(name)

        def scope(name, signature, index, scalar=True):
            module = 'crate' if scalar else 'hyperreal'
            edit(name, signature, signature + f'\n        let _diagnostic_scope = {module}::endpoint_cost_diagnostic::enter({index});')

        lib = 'crates/hyperreal/src/lib.rs'
        helper = 'crates/hyperreal/src/endpoint_cost_diagnostic.rs'
        edit(lib, 'mod trace;', 'pub mod endpoint_cost_diagnostic;\n\nmod trace;')
        shutil.copyfile(E / 'endpoint_cost_diagnostic.rs', source / helper)
        changed.append(helper)
        point = 'crates/hypercurve/src/point.rs'
        scope(point, '    fn eq(&self, other: &Self) -> bool {', 2, False)
        if variant == 'candidate':
            scope(point, '    pub(crate) fn coordinate_equality(&self, other: &Self) -> Option<bool> {', 2, False)
        scope(point, '    pub fn distance_squared(&self, other: &Self) -> Real {', 3, False)
        segment = 'crates/hypercurve/src/segment.rs'
        scope(segment, '    pub(crate) fn contains_endpoint(&self, point: &Point2, policy: &CurveContext) -> Option<bool> {', 1, False)
        scope(segment, 'fn points_equal(left: &Point2, right: &Point2, policy: &CurveContext) -> Option<bool> {', 4, False)
        classify = 'crates/hypercurve/src/classify.rs'
        scope(classify, 'pub(crate) fn is_zero(value: &Real, policy: &CurveContext) -> Option<bool> {', 5, False)
        algebra = 'crates/hyperreal/src/computable/node/algebra.rs'
        scope(algebra, '        project: impl Fn(PiLaurentPolynomial) -> Option<T>,\n    ) -> Option<T> {', 6)
        scope(algebra, '        reused: &mut bool,\n    ) -> Option<PiLaurentPolynomial> {', 7)
        edit(algebra, '        if reused {\n            self.pi_laurent_polynomial(budget).and_then(project)',
             '        if reused {\n            crate::endpoint_cost_diagnostic::event(1);\n            self.pi_laurent_polynomial(budget).and_then(project)')
        edit(algebra, '            *reused = true;\n            return Some(polynomial);',
             '            *reused = true;\n            crate::endpoint_cost_diagnostic::event(0);\n            return Some(polynomial);')
        representation = 'crates/hyperreal/src/computable/node/representation.rs'
        scope(representation, '        exact_sign: ExactSignCache,\n    ) -> Self {', 12)
        scope(representation, '    fn cell_or_init(&self) -> &std::sync::RwLock<CachedEvaluation> {', 9)
        edit(representation, '        let allocated = Box::into_raw(Box::new(',
             '        crate::endpoint_cost_diagnostic::event(2);\n        let allocated = Box::into_raw(Box::new(')
        scope(representation, '    fn pi_laurent_polynomial(&self) -> Option<PiLaurentPolynomial> {', 11)
        scope(representation, '    fn store_pi_laurent_polynomial(&self, polynomial: &PiLaurentPolynomial) {', 10)
        edit(representation, '        if guard.pi_laurent_polynomial.is_none() {\n            guard.pi_laurent_polynomial = Some(Box::new(closed));',
             '        if guard.pi_laurent_polynomial.is_none() {\n            crate::endpoint_cost_diagnostic::event(3);\n            guard.pi_laurent_polynomial = Some(Box::new(closed));')
        ops = 'crates/hyperreal/src/rational/arithmetic/ops.rs'
        scope(ops, '    pub(crate) fn has_arithmetic_reuse_evidence(&self) -> bool {', 8)
        edit(ops, '            || self.retained_fact(RETAINED_LINEAR_REUSE_SEEN)\n        {\n            return true;',
             '            || self.retained_fact(RETAINED_LINEAR_REUSE_SEEN)\n        {\n            crate::endpoint_cost_diagnostic::event(4);\n            return true;')
        edit(ops, '        crate::trace_dispatch!("rational", "arithmetic-reuse", "first-observation");',
             '        crate::endpoint_cost_diagnostic::event(5);\n        crate::trace_dispatch!("rational", "arithmetic-reuse", "first-observation");')
        for function, index in [('retain_sum_pair', 13), ('retain_difference_pair', 13), ('retain_product_pair', 14)]:
            scope(ops, f'    fn {function}(&self, other: &Self, result: &Self) {{', index)
        scope(ops, '    fn retain_negation_pair(&self, negation: &Self) {', 15)
        with (out / (variant + '-format.log')).open('w') as log:
            for path in [lib, point, segment, classify]:
                subprocess.run(['rustfmt', '--edition', '2024', str(source / path)], stdout=log, stderr=subprocess.STDOUT, check=True)
        current = {name: sha(source / name) for name in sorted(set(expected) | {helper})}
        assert {name for name, digest in current.items() if expected.get(name) != digest} == set(changed)
        archive = out / (variant + '-instrumentation.tar.gz')
        with tarfile.open(archive, 'w:gz') as tar:
            for name in changed:
                tar.add(source / name, arcname=name)
        patch = ''.join(''.join(difflib.unified_diff(
            (parents[variant] / name).read_text().splitlines(True) if name in expected else [],
            (source / name).read_text().splitlines(True), 'a/' + name, 'b/' + name)) for name in changed)
        (out / (variant + '.patch')).write_text(patch)
        variants[variant] = {'parent_source': str(parents[variant]), 'source': str(source), 'target': str(work / 'target'),
                             'source_changes_sha256': {name: current[name] for name in changed},
                             'parent_hashes': expected, 'archive_sha256': sha(archive), 'patch_sha256': sha(out / (variant + '.patch'))}

    probes = out / 'probes'
    probes.mkdir()
    prior_probes = TRIAL / 'controls-v1/probes'
    for path in sorted(prior_probes.glob('*.rs')):
        if path.name != 'final_abort.rs':
            shutil.copyfile(path, probes / path.name)
    for name in ['trace_probe.rs', 'broad_allocation.rs']:
        shutil.copyfile(E / name, probes / name)
    prelude = 'use hyperreal::endpoint_cost_diagnostic as diagnostic;\nmod trace_probe;\n'
    arc = (probes / 'arc_cost.rs').read_text()
    # Keep the existing allocator accounting; add only nonallocating hooks.
    arc = replace(arc, '    fn allocated(bytes: usize) {', '    fn allocated(bytes: usize) {\n        hyperreal::endpoint_cost_diagnostic::allocated(bytes);')
    arc = replace(arc, '            LIVE.fetch_sub(layout.size(), Relaxed);\n            unsafe { System.dealloc(pointer, layout) };',
                  '            LIVE.fetch_sub(layout.size(), Relaxed);\n            hyperreal::endpoint_cost_diagnostic::deallocated(layout.size());\n            unsafe { System.dealloc(pointer, layout) };')
    arc = replace(arc, '                LIVE.fetch_sub(layout.size(), Relaxed);\n                allocated(size);',
                  '                LIVE.fetch_sub(layout.size(), Relaxed);\n                hyperreal::endpoint_cost_diagnostic::deallocated(layout.size());\n                allocated(size);')
    arc = replace(arc, 'fn measure<T>(name: &str, count: usize, body: impl FnOnce() -> T) -> T {',
                  'fn measure<T>(name: &str, count: usize, body: impl FnOnce() -> T) -> T {\n    let between = diagnostic::finish();\n    diagnostic::begin();')
    arc = replace(arc, '    #[cfg(not(feature = "count-alloc"))]\n    let (calls, bytes, retained, peak) = (0, 0, 0, 0);',
                  '    #[cfg(not(feature = "count-alloc"))]\n    let (calls, bytes, retained, peak) = (0, 0, 0, 0);\n    let measured = diagnostic::finish();')
    arc = replace(arc, '    black_box(result)\n}',
                  '    trace_probe::dump(name, "between", &between);\n    trace_probe::dump(name, "measured", &measured);\n    diagnostic::begin();\n    black_box(result)\n}')
    arc = replace(arc, 'fn main() {', 'fn main() {\n    diagnostic::begin();')
    index = arc.rindex('\n}')
    arc = arc[:index] + '\n    trace_probe::dump("process", "tail", &diagnostic::finish());' + arc[index:]
    # Inner module docs must remain before the inserted imports.
    first, rest = arc.split('\n', 1)
    (probes / 'arc_cost.rs').write_text(first + '\n' + prelude + rest)

    broad = (probes / 'operation_probe.rs').read_text()
    for begin, end, expected_line, check_line, label in [
        ('fn measure_region(', 'fn quadratic()', '    let expected = witness(&query(), &expected_area);', '    assert_eq!(witness(&query(), &expected_area), expected);', 'label'),
        ('fn measure_value<T>(', 'fn area_is(', '    let expected = check(&query());', '    assert_eq!(check(&query()), expected);', 'label'),
    ]:
        start = broad.index(begin)
        stop = broad.index(end, start)
        body = broad[start:stop]
        body = replace(body, expected_line, '    let between = diagnostic::finish();\n    diagnostic::begin();\n' + expected_line + '\n    let setup = diagnostic::finish();\n    diagnostic::begin();')
        body = replace(body, '    let elapsed = started.elapsed().as_nanos();', '    let elapsed = started.elapsed().as_nanos();\n    let measured = diagnostic::finish();\n    diagnostic::begin();')
        body = replace(body, check_line, check_line + '\n    let checked = diagnostic::finish();')
        println = '    println!("operation_{label}\\t{iterations}\\t{elapsed}\\t{expected}");'
        suffix = '\n'.join(f'    trace_probe::dump({label}, "{stage}", &{stage});' for stage in ['between', 'setup', 'measured', 'checked'])
        body = replace(body, println, println + '\n' + suffix + '\n    diagnostic::begin();')
        broad = broad[:start] + body + broad[stop:]
    broad = replace(broad, 'fn main() {', 'fn main() {\n    diagnostic::begin();')
    broad = replace(broad, '    pad_probe::run();\n}', '    pad_probe::run();\n    trace_probe::dump("process", "tail", &diagnostic::finish());\n}')
    (probes / 'operation_probe.rs').write_text(prelude + 'mod broad_allocation;\n' + broad)
    with (out / 'probe-format.log').open('w') as log:
        for name in ['arc_cost.rs', 'operation_probe.rs']:
            subprocess.run(['rustfmt', '--edition', '2024', str(probes / name)], stdout=log, stderr=subprocess.STDOUT, check=True)
    record = {'scope': __doc__, 'runner_sha256': sha(Path(__file__)), 'previous_goal_turn_classification': 'progress',
              'base_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
              'trial_decision_sha256': sha(TRIAL / 'decision-v1.json'), 'main_source_sha256': guard,
              'variants': variants, 'probes_sha256': {path.name: sha(path) for path in sorted(probes.glob('*.rs'))},
              'fixed_prior_cost_evidence_sha256': sha(TRIAL / 'controls-assessment-v1.json'),
              'limits': 'Integer TLS counters add fixed thread-local storage and execution overhead. Allocation rows overlap by active scope and exclude native C heaps. Node creation counts are function entries; the surrounding Arc allocation is not inside Node::new. No profiled durations support performance admission.',
              'runtime_admitted': False}
    assert sources() == guard
    for variant, expected in maps.items():
        assert all(sha(parents[variant] / name) == digest for name, digest in expected.items())
    (out / 'protocol.json').write_text(json.dumps(record, indent=2) + '\n')
    print(json.dumps({'prepared_variants': list(variants), 'probes': len(record['probes_sha256']), 'source_changes': {key: len(value['source_changes_sha256']) for key, value in variants.items()}}))


if __name__ == '__main__':
    main()
