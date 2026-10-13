"""Close the profile and specify a distinct existing-certificate coverage diagnostic."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def main():
    assert not (E / 'decision-v1.json').exists()
    a = json.loads((E / 'assessment-v1.json').read_text())
    o = json.loads((E / 'profile-v1/observations.json').read_text())
    inputs = json.loads((E / 'inputs-v1.json').read_text())
    parent = json.loads((ROOT / 'docs/evidence/2026-10-13-native-tower-sign-retention/preparation-v1/protocol.json').read_text())
    source = Path(parent['source'])
    assert sha(E / 'assess_v1.py') == a['assessor_sha256']
    assert sha(E / 'inputs-v1.json') == a['inputs_sha256']
    for name, digest in a['artifacts_sha256'].items():
        assert sha(E / name) == digest, name
    assert a['source_parent_main_and_binary_unchanged']
    assert sources() == parent['main_source_sha256']
    assert all(sha(ROOT / name) == digest for name, digest in parent['pending_native_sha256'].items())
    assert a['quality']['samples'] == 7071 and a['quality']['lost_samples'] == 0
    assert a['profiled_worker_cpu_seconds'] == 141.705158972 and not a['profiled_gate_passed']
    reviews = []

    def review(path, start, end, finding):
        text = (source / path).read_text()
        begin = text.index(start)
        body = text[begin:text.index(end, begin)]
        reviews.append({'path': path, 'source_sha256': sha(source / path), 'start': start,
                        'end_before': end, 'body_sha256': hashlib.sha256(body.encode()).hexdigest(),
                        'finding': finding})

    review('crates/hyperreal/src/computable/node/approximation_queries.rs',
           '    pub(crate) fn immediate_sign(', '    /// An approximation of this Computable',
           'Existing root-only sign inspection accepts a Valid exact-sign cache or, when that is unavailable, the known sign of an already Valid bound. It performs no graph traversal or new approximation.')
    review('crates/hyperreal/src/computable/node/structural_analysis.rs',
           '    fn exact_sign(&self)', '    fn exact_quadratic_surd_sign(',
           'The exact-sign path consults retained exact signs, separated existing approximations, inverse-trig structure and cached Unknown state before its iterative structural walk. If that walk remains inconclusive, it calls exact_quadratic_surd_sign, without consulting the root bound cache at that final call site. Cached Unknown and the direct resolver already avoid some repeated work; their removal is not proposed.')
    review('crates/hyperreal/src/computable/node/structural_analysis.rs',
           '    fn exact_quadratic_surd_sign(', '    pub(crate) fn exact_pure_quadratic_surd(',
           'The sign fallback invokes full quadratic projection, then uses exact rational coefficient signs and, for opposing signs, rational square comparison. The hypothesis concerns avoiding this entire fallback when a root sign certificate is already present, not changing multiplication formulas.')
    review('crates/hyperreal/src/computable/node/bounds.rs',
           'pub(crate) enum BoundInfo {', '#[cfg(verus_keep_ghost)]\nverus! {',
           'BoundInfo distinguishes Unknown, exact Zero and NonZero with optional sign and optional magnitude. A nonzero fact alone does not certify a sign. An inexact or missing magnitude does not by itself invalidate a stored sign.')
    review('crates/hyperreal/src/computable/node/representation.rs',
           '    fn bound(&self) -> BoundCache {', '    #[cfg(not(verus_keep_ghost))]\n    fn set_bound(',
           'The current bound and combined fact snapshot are read from existing packed atomic metadata. A diagnostic can classify these Copy values without retaining scalar/DAG owners, materializing polynomials or invoking a recognizer.')

    next_diagnostic = {
        'scope': 'Observe existing root-bound certificates at the exact-sign quadratic fallback in a separate frozen copy of the unchanged endpoint/P4 parent.',
        'rationale': 'The existing immediate-sign consumer already treats a known root-bound sign as a certificate. The exact-sign fallback does not check that cache at this site. Availability and agreement at this point have not been measured.',
        'source': str(source), 'parent_preparation_sha256': inputs['parent_preparation_sha256'],
        'site': 'Computable::exact_sign, inside the final result.is_none() branch immediately before the existing exact_quadratic_surd_sign call.',
        'observations': [
            'Read only the already-stored BoundCache at that site; classify Invalid, Valid Unknown, Valid Zero, Valid NonZero positive, Valid NonZero negative and Valid NonZero without a sign.',
            'Run the original quadratic fallback unchanged, then count its already-computed result as negative, zero, positive or None against the entry bound category.',
            'Record starts/completions so interrupted or unwound observations cannot be silently counted as completed. Preserve contradictory outcomes explicitly if any occur.'
        ],
        'instrumentation_constraints': [
            'Fixed-size integer counters and Copy metadata only. No retained addresses/owners, new scalar comparisons, cheap_bound call, cache writes, new normalization pass or arithmetic change.',
            'The diagnostic must preserve all original branches, arithmetic order, original result publication, per-pass memo handling, budgets and mandatory original retry.',
            'Counters do not bypass the fallback. This observes possible coverage; it is not a runtime optimization.',
            'Freeze the source, counters, qualification and measurement commands before execution. Use one complete-board diagnostic run and retain the original 120-second assertion. Do not extend the sample set based on outcomes.'
        ],
        'qualification_requirements': [
            'Independent counter-category controls for every bound tag and every already-computed result, including explicit mismatch and unfinished-observation accounting.',
            'Site controls demonstrating that the original fallback still executes and cache/result outcomes agree with the uninstrumented parent, including absent/unknown bounds and valid positive/negative/zero certificates.',
            'Existing P4 sign-retention, persistent-cache eight/nine-term frontier, root retry/memo lifetime, abort/no-approximation and endpoint controls, plus scalar library/docs and proportionate strict stack checks.',
            'Removing only the declared instrumentation and test additions must recover the unchanged parent source. Keep main runtime, all 35 pending native paths and original full-resolution fixtures intact.'
        ],
        'decision_rules': [
            'Zero eligible known-sign observations close this particular omission hypothesis for the measured boards.',
            'Any disagreement between an existing known sign and a definite fallback result becomes a correctness investigation before optimization.',
            'Nonzero agreeing coverage only justifies considering a separate bounded prototype; it does not predict time savings or admit a change.',
            'Any later prototype must preserve facts, certainty/escalation receipts, proof-cache behavior and P4 monotonicity, and pass fixed scalar/native cost qualification, complete report replay and both original native gates.'
        ],
        'separate_copper_neck_gate': 'Not executed by this planned coverage diagnostic; its original 60-second check-body limit remains mandatory for any retained optimization.',
        'instrumented_source_prepared': False, 'runner_prepared': False, 'executed': False,
        'runtime_admitted': False, 'plan_complete': False,
    }
    (E / 'next-diagnostic-v1.json').write_text(json.dumps(next_diagnostic, indent=2) + '\n')
    result = {
        'status': 'diagnose-existing-bound-sign-coverage-before-quadratic-fallback',
        'decision_runner_sha256': sha(Path(__file__)),
        'assessment_sha256': sha(E / 'assessment-v1.json'),
        'next_diagnostic_sha256': sha(E / 'next-diagnostic-v1.json'),
        'closed_profile': {'runs': 1, 'samples': a['quality']['samples'], 'lost_samples': 0,
                           'unknown_leaf_period_percent': a['quality']['unknown_leaf_period_percent'],
                           'worker_cpu_seconds': a['profiled_worker_cpu_seconds'],
                           'original_limit_seconds': 120, 'gate_passed': False,
                           'finding_counts': a['finding_counts']},
        'recovered_cost': {'groups': a['groups'], 'exclusive_partition': a['exclusive_recovered_stack_partition'],
                           'intersections': a['intersections'],
                           'normal_form_nearest_consumers': a['normal_form_nearest_consumers'],
                           'coefficient_multiply_nearest_consumers': a['coefficient_multiply_nearest_consumers'],
                           'normal_form_rational_symbols': o['normal_form_inclusive_rational_symbols']},
        'interpretation': [
            'Normal-form work remains prominent in this profile of the unchanged endpoint/P4 source. Rational arithmetic appears inside much of the recovered coefficient-multiplication work. The prior helper series preserved that arithmetic and remains closed.',
            'Comparison and zero testing are the largest recovered direct native consumers of normal-form stacks. The profile cannot identify redundant invocations or existing certificate availability.',
            'The 10.159% recovered original-normalizer weight is not permission to omit the mandatory original retry. Earlier frontier evidence establishes why that retry is necessary.',
            'The next question is whether an already certified root-bound sign could avoid a full sign-only quadratic normalization. This is distinct from coefficient helper boundaries, storage layouts and cache-copy/memo changes.',
            'The proposed counter diagnostic observes coverage at one existing call site. No bypass or new arithmetic is implemented or admitted.'
        ],
        'source_review': reviews,
        'earlier_evidence': inputs['related_count_evidence'],
        'next_step': next_diagnostic,
        'limits': a['limits'] + [
            'The earlier endpoint-only profile used a different source and binary. Comparing its percentages with this run does not establish a matched performance change.',
            'Zero separately recovered quadratic-sign or zero-status symbol samples do not mean those operations are free; inline frames were not expanded.',
            'The parent release full-board failure remains 133.31248412 worker-CPU seconds / 120. The profile result does not replace it.'
        ],
        'main_and_35_pending_native_paths_unchanged': True,
        'runtime_admitted': False, 'plan_complete': False,
    }
    (E / 'decision-v1.json').write_text(json.dumps(result, indent=2) + '\n')
    status_path = ROOT / 'docs/properties-status.json'
    status = json.loads(status_path.read_text())
    base = json.loads(subprocess.check_output(['git', 'show', '9025de62:docs/properties-status.json'], cwd=ROOT))
    assert {k: v for k, v in status.items() if k != 'current_work'} == {k: v for k, v in base.items() if k != 'current_work'}
    status['current_work']['native_normal_form_attribution'].update(
        status=result['status'], evidence=str((E / 'decision-v1.json').relative_to(ROOT)),
        result='Fixed unchanged-parent profile: 7071 samples, no lost samples; normal form 26.983% and coefficient multiplication 9.085% of total sampled cycle weight. The original assertion fails at 141.705158972 worker-CPU seconds / 120. These diagnostic percentages do not predict savings.',
        source_review=reviews,
        next_step='Prepare the separately frozen existing-bound-sign coverage diagnostic specified in next-diagnostic-v1.json. No instrumented source or runner has been prepared or executed; no fallback bypass is admitted.')
    status_path.write_text(json.dumps(status, indent=2) + '\n')
    print(json.dumps({'status': result['status'], 'next_diagnostic_executed': False, 'runtime_admitted': False}))


if __name__ == '__main__':
    main()
