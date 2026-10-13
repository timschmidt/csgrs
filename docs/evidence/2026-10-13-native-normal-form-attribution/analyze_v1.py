"""Attribute the single frozen native profile using predeclared symbol groups.

Recovered frame-pointer stacks have no inline expansion. Inclusive groups
overlap. Exclusive partitions describe recovered stacks, not operation counts,
elapsed time, causality or a prediction of removable work.
"""
import collections
import gzip
import hashlib
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parent
OUT = E / 'profile-v1'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
GROUPS = {
    'normal_form': ['pi_laurent_polynomial'],
    'retained_normalizer': ['pi_laurent_polynomial_retained::<true>'],
    'original_normalizer': ['pi_laurent_polynomial_retained::<false>'],
    'coefficient_multiply': ['RadicalCoefficient>::multiply'],
    'polynomial_multiply': ['PiLaurentPolynomial>::multiply'],
    'polynomial_insert': ['PiLaurentPolynomial>::insert'],
    'quadratic_projection': ['exact_quadratic_surd_parts'],
    'quadratic_sign': ['exact_quadratic_surd_sign'],
    'exact_sign': ['Computable>::exact_sign'],
    'structural_facts': ['::structural_facts'],
    'zero_status': ['::zero_status'],
    'rational_arithmetic': ['hyperreal::rational::'],
    'rational_word_constructor': ['Rational>::from_reduced_word_parts'],
    'rational_wide_gcd': ['Rational>::gcd_wide_magnitudes'],
    'approximation_cache_lookup': ['EvaluationCache>::at_precision'],
    'approximation_evaluation': ['Computable>::approx_signal'],
    'rational_interval_construction': ['Real>::interval_from_approximation'],
    'cached_rational_interval': ['::cached_rational_interval'],
    'point_equality': ['<hypercurve::point::Point2 as core::cmp::PartialEq>::eq'],
    'coordinate_equality': ['Point2>::coordinate_equality'],
    'endpoint_membership': ['CircularArc2>::contains_endpoint'],
    'curve_comparison': ['hypercurve::classify::compare_reals'],
    'curve_zero_test': ['hypercurve::classify::is_zero'],
    'curve_sign_test': ['hypercurve::classify::real_sign'],
    'regularization': ['CurveRegion2>::regularized_region_raw'],
    'topology_construction': ['CurveRegionBooleanContext>::build_split_topology'],
    'region_construction': ['CurveRegionBooleanContext>::build_boolean_region_from_topology'],
}
ALLOCATOR = re.compile(
    r'^(?:_int_malloc|_int_free|_int_realloc|unlink_chunk|malloc|cfree@|free@|'
    r'__libc_malloc|__libc_free|__libc_realloc|__rustc::__rust_alloc|'
    r'__rustc::__rust_dealloc|__rustc::__rust_realloc)'
)
CONSUMERS = ('hypercurve::', 'hyperlimit::', 'hypersolve::', 'hyperdrc::')
PROJECT = re.compile(r'^<?&?(?:hyperreal|hypercurve|hyperlimit|hypersolve|hyperlattice|hyperdrc|hypermesh|hyperpath|hypertri)::')


def main():
    assert not (OUT / 'observations.json').exists()
    manifest = json.loads((OUT / 'manifest.json').read_text())
    protocol = json.loads((OUT / 'protocol.json').read_text())
    assert manifest['complete'] and manifest['closed_profile_runs'] == 1
    assert sha(Path(manifest['binary'])) == manifest['binary_sha256']
    assert sha(Path(__file__)) == protocol['frozen_inputs_sha256']['analyze_v1.py']
    for name, digest in manifest['artifacts_sha256'].items():
        assert sha(OUT / name) == digest, name
    blocks = gzip.decompress((OUT / 'stacks.stdout.gz').read_bytes()).decode().strip().split('\n\n')
    groups = collections.Counter()
    group_samples = collections.Counter()
    symbols = collections.Counter()
    leaves = collections.Counter()
    depths = collections.Counter()
    partition = collections.Counter()
    normal_consumers = collections.Counter()
    coefficient_consumers = collections.Counter()
    normal_leaves = collections.Counter()
    coefficient_leaves = collections.Counter()
    allocator_callers = collections.Counter()
    normal_allocator_callers = collections.Counter()
    normal_rational_symbols = collections.Counter()
    coefficient_rational_symbols = collections.Counter()
    intersections = collections.Counter()
    times = []
    total = unknown = empty = 0
    for block in blocks:
        lines = block.splitlines()
        header = re.match(r'^\S+\s+\d+/\d+\s+([0-9.]+):\s+(\d+) cycles:u:', lines[0])
        assert header, lines[0]
        times.append(float(header[1]))
        period = int(header[2])
        total += period
        frames = []
        for line in lines[1:]:
            frame = re.match(r'\s+[a-f0-9]+ (.*?) \((.*)\)$', line)
            assert frame, line
            frames.append(frame[1])
        depths[len(frames)] += 1
        leaf = frames[0] if frames else '<empty stack>'
        empty += period if not frames else 0
        unknown += period if not frames or leaf == '[unknown]' else 0
        leaves[leaf] += period
        present = {name: any(needle in frame for needle in needles for frame in frames)
                   for name, needles in GROUPS.items()}
        for name, hit in present.items():
            if hit:
                groups[name] += period
                group_samples[name] += 1
        for symbol in set(frames):
            symbols[symbol] += period
            if 'hyperreal::rational::' in symbol:
                if present['normal_form']:
                    normal_rational_symbols[symbol] += period
                if present['coefficient_multiply']:
                    coefficient_rational_symbols[symbol] += period
        if present['normal_form']:
            name = ('normal_form_with_coefficient_multiply' if present['coefficient_multiply']
                    else 'normal_form_with_other_polynomial_multiply' if present['polynomial_multiply']
                    else 'normal_form_without_recovered_multiply')
        else:
            name = 'coefficient_multiply_without_normal_form' if present['coefficient_multiply'] else 'other'
        partition[name] += period
        for selected, needle, counter, selected_leaves in [
            ('normal_form', 'pi_laurent_polynomial', normal_consumers, normal_leaves),
            ('coefficient_multiply', 'RadicalCoefficient>::multiply', coefficient_consumers, coefficient_leaves),
        ]:
            if present[selected]:
                indices = [i for i, frame in enumerate(frames) if needle in frame]
                caller = next((f for f in frames[max(indices) + 1:] if any(c in f for c in CONSUMERS)),
                              '<no recovered curve/predicate/drc ancestor>')
                counter[caller] += period
                selected_leaves[leaf] += period
        if ALLOCATOR.match(leaf):
            intersections['allocator_leaf'] += period
            nearest = next((f for f in frames[1:] if PROJECT.match(f)), '<no recovered project ancestor>')
            allocator_callers[nearest] += period
            if present['normal_form']:
                intersections['allocator_leaf_with_normal_form'] += period
                normal_allocator_callers[nearest] += period
            if present['coefficient_multiply']:
                intersections['allocator_leaf_with_coefficient_multiply'] += period
        if present['normal_form'] and present['rational_arithmetic']:
            intersections['rational_arithmetic_with_normal_form'] += period
        if present['coefficient_multiply'] and present['rational_arithmetic']:
            intersections['rational_arithmetic_with_coefficient_multiply'] += period
    report = gzip.decompress((OUT / 'self-cost.stdout.gz').read_bytes()).decode()
    assert int(re.search(r'Event count \(approx\.\): (\d+)', report)[1]) == total
    lost = int(re.search(r'Total Lost Samples: (\d+)', report)[1])
    assert sum(partition.values()) == sum(leaves.values()) == total
    assert sum(normal_consumers.values()) == sum(normal_leaves.values()) == groups['normal_form']
    assert sum(coefficient_consumers.values()) == sum(coefficient_leaves.values()) == groups['coefficient_multiply']

    def rows(counter):
        return [{'symbol': key, 'period': value, 'total_profile_percent': 100 * value / total}
                for key, value in counter.most_common()]

    result = {
        'scope': __doc__, 'runner_sha256': sha(Path(__file__)),
        'profile_manifest_sha256': sha(OUT / 'manifest.json'),
        'stacks_gzip_sha256': sha(OUT / 'stacks.stdout.gz'),
        'samples': len(blocks), 'total_period': total, 'lost_samples': lost,
        'first_sample_time': min(times), 'last_sample_time': max(times),
        'empty_stack_period_percent': 100 * empty / total,
        'unknown_leaf_period_percent': 100 * unknown / total,
        'depth_sample_counts': dict(sorted(depths.items())),
        'group_definitions': GROUPS, 'allocator_leaf_pattern': ALLOCATOR.pattern,
        'project_ancestor_pattern': PROJECT.pattern, 'consumer_prefixes': CONSUMERS,
        'groups': {name: {'samples': group_samples[name], 'period': groups[name],
                          'inclusive_percent': 100 * groups[name] / total} for name in GROUPS},
        'exclusive_recovered_stack_partition': rows(partition),
        'intersections': rows(intersections),
        'inclusive_symbols': rows(symbols), 'leaf_symbols': rows(leaves),
        'normal_form_nearest_consumers': rows(normal_consumers),
        'coefficient_multiply_nearest_consumers': rows(coefficient_consumers),
        'normal_form_leaves': rows(normal_leaves),
        'coefficient_multiply_leaves': rows(coefficient_leaves),
        'allocator_leaf_nearest_project_ancestors': rows(allocator_callers),
        'normal_form_allocator_leaf_nearest_project_ancestors': rows(normal_allocator_callers),
        'normal_form_inclusive_rational_symbols': rows(normal_rational_symbols),
        'coefficient_multiply_inclusive_rational_symbols': rows(coefficient_rational_symbols),
        'limits': ['The allocator pattern is a declared subset, not a complete allocation model.',
                   'Recovered symbols and ancestors can omit inlined frames; absent symbol weight does not establish zero cost.',
                   'Percentages are sampled cycle periods, not counts, wall seconds, predicted savings or matched speedup evidence.',
                   'This single fixed profile does not identify expression identity, cache hit rates, duplicate requests or operand shapes.'],
        'runtime_admitted': False, 'plan_complete': False,
    }
    (OUT / 'observations.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({k: result[k] for k in ['samples', 'lost_samples', 'empty_stack_period_percent', 'unknown_leaf_period_percent', 'groups', 'exclusive_recovered_stack_partition']}))


if __name__ == '__main__':
    main()
