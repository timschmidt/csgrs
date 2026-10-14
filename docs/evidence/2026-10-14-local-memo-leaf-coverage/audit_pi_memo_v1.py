"""Freeze the source audit for a distinct, Pi-only local memo prototype."""
import gzip
import json
from pathlib import Path

from leaf_memo_common_v2 import E, ROOT, PREP, dump, sha, verify


def main():
    destination = E / 'pi-memo-source-audit-v1.json'
    assert not destination.exists()
    prep = json.loads((PREP / 'protocol.json').read_text())
    verify(prep, full=True)
    assessment = json.loads((E / 'assessment-v2.json').read_text())
    interpretation = json.loads((E / 'interpretation-v1.json').read_text())
    closure = json.loads((E / 'closure-v1.json').read_text())
    assert interpretation['assessment_sha256'] == sha(E / 'assessment-v2.json')
    assert interpretation['interpreter_sha256'] == sha(E / 'interpret_v1.py')
    assert interpretation['observed_exact_leaf_kinds'] == ['Pi']
    for key, name in [('result_sha256', 'diagnostic-v2/result.json'),
                      ('assessment_sha256', 'assessment-v2.json'),
                      ('interpretation_sha256', 'interpretation-v1.json'),
                      ('failed_v1_sha256', 'failure-v1.json')]:
        assert closure[key] == sha(E / name)
    assert all(sha(E / p) == h for p, h in assessment['artifacts_sha256'].items())
    base_e = ROOT / 'docs/evidence/2026-10-14-laurent-cache-eligibility'
    baseline = json.loads((base_e / 'preparation-v1/protocol.json').read_text())['variants']['baseline']
    source = Path(baseline['source'])
    hashes = json.loads(gzip.decompress((base_e / 'preparation-v1/baseline-source-hashes.json.gz').read_bytes()))
    assert len(hashes) == 10135
    assert all(sha(source / p) == h for p, h in hashes.items())
    borrowed_e = ROOT / 'docs/evidence/2026-10-13-borrowed-integer-leaves'
    borrowed = json.loads(gzip.decompress((borrowed_e / 'preparation-v1/source-hashes.json.gz').read_bytes()))
    assert len(borrowed) == 10134 and set(borrowed) <= set(hashes)
    differences = [p for p in hashes if hashes[p] != borrowed.get(p)]
    algebra = 'crates/hyperreal/src/computable/node/algebra.rs'
    controls = 'crates/hyperreal/src/computable/node/laurent_cache_eligibility_tests.rs'
    assert set(differences) == {algebra, controls}
    parent = Path(json.loads((borrowed_e / 'preparation-v1/protocol.json').read_text())['source'])
    text = (source / algebra).read_text()
    addition = '#[cfg(test)]\nmod laurent_cache_eligibility_tests;\n\n'
    assert text.count(addition) == 1
    assert text.replace(addition, '') == (parent / algebra).read_text()
    assert 'record_cache_coverage(' not in text and 'record_cache_lookup_kind(' not in text
    assert 'record_memo_payload' not in text
    focus = []
    for row in assessment['classified_phases']:
        counts = row['memo_payloads']['Pi']['retained']
        if counts['memo_store']['events'] or counts['memo_hit']['events']:
            assert counts['memo_store']['events'] == 4096
            assert counts['memo_hit']['events'] == 12288
            focus.append({'suite': row['suite'], 'phase': row['phase'],
                          'pi_stores': 4096, 'pi_hits': 12288})
    assert len(focus) == 9
    assert all(counts['none'] == 0 for row in assessment['classified_phases']
               for modes in row['memo_payloads'].values() for events in modes.values()
               for counts in events.values())
    reviewed = [algebra, 'crates/hyperreal/src/computable/node/structural_analysis.rs',
                'crates/hyperreal/src/rational/arithmetic/construction.rs']
    evidence = [E / p for p in ['assessment-v2.json', 'interpretation-v1.json', 'closure-v1.json',
                               'failure-v1.json', 'source-review-v2.json']]
    evidence += [base_e / 'preparation-v1/protocol.json', base_e / 'preparation-v1/baseline-source-hashes.json.gz',
                 base_e / 'scalar-cost-interpretation-v1.json', borrowed_e / 'preparation-v1/protocol.json',
                 ROOT / 'docs/evidence/2026-10-14-laurent-cache-miss-classification/normalizer-work-audit-v1.json',
                 ROOT / 'docs/evidence/2026-10-13-closed-certificate-memo/preparation-v1/candidate.patch',
                 ROOT / 'docs/evidence/2026-10-13-closed-certificate-memo/decision-v1.json',
                 ROOT / 'docs/evidence/2026-10-13-retained-quadratic-projection/decision-v1.json']
    report = {
        'status': 'source-audit-complete-pi-only-memo-prototype-ready-for-protocol',
        'audit_script_sha256': sha(__file__),
        'parent_source': str(source),
        'parent_source_files': 10135,
        'parent_source_hashes_sha256': sha(base_e / 'preparation-v1/baseline-source-hashes.json.gz'),
        'parent_runtime': 'Borrowed-integer runtime plus three eligibility tests. Removing the test-only module declaration recovers the borrowed algebra byte for byte; the only other difference is that test module.',
        'parent_excludes_diagnostic_observers_and_rejected_read_guard': True,
        'source_sha256': {p: sha(source / p) for p in reviewed},
        'evidence_sha256': {str(p.relative_to(ROOT)): sha(p) for p in evidence},
        'review_scope': 'Complete normalizer and projection/retry control flow, temporary polynomial/coefficient layouts and constructors, exact_rational and from_bigint_ref, full baseline-to-classifier algebra diff, all 68 phase inventories, and relevant prior decision/mechanism sections. This is a source audit, not a formal proof or a cost measurement.',
        'observed_exact_leaf_kinds': ['Pi'],
        'observed_pi_phases': focus,
        'all_observed_pi_payloads': 'One term at pi power 1, rational coefficient 1, no radicals or opaque owners. The direct constructor and independent site control establish the coefficient/power; the workload counters establish shape and copy counts.',
        'none_payload_events_in_workloads': 0,
        'source_invariants': [
            'After the depth/work rejection, every normalizer result is Some: rational and recognized-constant branches wrap a monomial; inverse-trig branches wrap an opaque atom; the general branch replaces every declined expansion with an opaque atom.',
            'The return None statements inside the expansion closure return from that closure. expanded.or_else supplies the atom fallback. A budget rejection occurs before memo insertion. Thus a fresh query memo never receives a None payload from this producer; the zero observed None counts are corroboration, not the proof of that invariant.',
            'The projection creates an empty memo; recursive calls share it; the original retry drops the first memo before creating another. The two private normalizer signatures are the only production consumers of this memo type.',
            'A Constant(Pi) leaf constructs a closed monomial and cannot publish a persistent Laurent certificate. Its memo polynomial contains no Computable owner. Other-node payloads can own opaque references and must retain their clone and release order.',
            'Retained cache lookup precedes sharing/memo lookup, which precedes the budget check. A successful memo hit consumes no depth or work and may replay at zero budget. The shared-owner predicate also remains before memo lookup.',
        ],
        'proposed_prototype': {
            'scope': 'Only the representation of completed local memo entries, with a special tag for immutable Constant(Pi).',
            'type': 'A private enum with Pi and Polynomial(PiLaurentPolynomial) variants; retain the existing address key, Vec capacity/order and per-query lifetime.',
            'completed_result': 'Express the existing always-Some post-budget result as a polynomial directly, using the same opaque fallback. Wrap it in Some at the external normalizer return. Do not add unwrap or expect.',
            'store': 'After the unchanged first-visit computation and charging, store Pi only for Constant(Pi); store the original polynomial clone for every other kind. Keep insertion before the unchanged persistent publication attempt.',
            'hit': 'Replay the same exact pi monomial for Pi and clone the original polynomial for every other entry, at the existing memo-hit site before all budget checks.',
            'excluded_changes': 'Do not tag other exact leaves, omit memo entries, change persistent eligibility/read order, introduce a global cache or alter the polynomial/coefficient vector storage. Broader leaf coverage is not demonstrated by these workloads.',
            'layout': 'Measure the enum, complete keyed entry, polynomial, coefficient, node and persistent layouts. Equal layout and lower requested memory are not assumed from Rust enum syntax.',
        },
        'prior_experiment_distinction': [
            'The rejected closed-publication memo experiment published first and skipped local insertion after successful publication. Pi is publication-ineligible, and the proposed tag preserves every memo entry and publication order.',
            'The rejected entry eligibility predicate is absent from the selected parent. Every original cache read remains in place.',
            'Earlier inline/union/shared storage experiments remain excluded. This proposal leaves all polynomial and coefficient vectors unchanged.',
            'The root-only retained projection was deferred after coverage measurement; no runtime implementation of that shortcut was tested or rejected. This proposal does not add that shortcut.',
        ],
        'qualification_requirements': [
            'Freeze separate source, protocol, input hashes and original fixed sample schedules before execution. Keep main, all 35 pending native paths and every old source/result/binary unchanged.',
            'Run source formatting, strict all-feature/all-target library Clippy and all existing parent tests plus meaningful controls before any samples. Qualify ordinary/default behavior as appropriate to the private implementation.',
            'Check raw Pi first visit, cold zero depth/work rejection, shared zero-budget replay in retained/original modes, transition back to a sole owner, exact coefficient/power and no node-local cache allocation. Use independent expected values.',
            'Check all other leaf/constant kinds remain ordinary polynomial entries, non-Pi expressions that simplify to pi are not mislabeled, opaque owners survive replay then reclaim, and the original eight/nine-term fallback preserves previous proofs.',
            'Retain P4 sign proofs, all error/abort priorities, canonical integer behavior and all public exact witnesses. Instrumentation for work reconciliation must be a separate diagnostic source from cost binaries.',
            'Measure fixed complete rational/quadratic/integer construction/first/warm/cloned costs, all allocation fields, representations and binary sizes versus main and the unmodified borrowed runtime. Include every zero-event phase and every adverse row.',
            'Only a scalar-qualified improvement can progress to the broader library/consumer/report matrix and both original native gates: full board 120 seconds worker CPU, copper neck 60 seconds check-body wall time. Never substitute process wall for an internal duration.',
        ],
        'limits': [
            'Nine phases contain Pi copies. The remaining 59 phases still belong in validation and costs; their behavior can be affected by enum dispatch or code generation.',
            'The three opaque rational phases contain both Pi copies and opaque-owner copies. The proposed tag addresses only Pi. No event count establishes a timing share or allocation saving.',
            'Prototype preparation, compilation, layout measurement and costs have not run. No performance result or runtime admission follows from this audit.',
            'All 39 properties, 14 targets, crate future work and original native failures retain their scope.',
        ],
        'next_step': 'Freeze a Pi-only local memo prototype and its qualification/cost protocol on this parent.',
        'main_and_35_pending_native_paths_unchanged': True,
        'runtime_candidate_prepared': False,
        'runtime_admitted': False,
        'plan_complete': False,
    }
    dump(destination, report)
    print(json.dumps({'source_audit_complete': True, 'parent_files': 10135,
                      'observed_pi_phases': 9, 'runtime_candidate_prepared': False}), flush=True)


if __name__ == '__main__':
    main()
