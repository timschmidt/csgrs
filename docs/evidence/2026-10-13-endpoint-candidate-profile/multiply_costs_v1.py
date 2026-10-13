"""Attribute sampled coefficient-multiplication work without guessing operand shapes."""
import collections
import gzip
import hashlib
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parent
OUT = E / 'profile-v1'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    assert not (OUT / 'multiply-costs.json').exists()
    observed = json.loads((OUT / 'observations.json').read_text())
    attribution = json.loads((OUT / 'attribution.json').read_text())
    allocator = re.compile(attribution['allocator_symbol_pattern'])
    symbol = '<hyperreal::computable::node::RadicalCoefficient>::multiply'
    blocks = gzip.decompress((OUT / 'stacks.stdout.gz').read_bytes()).decode().strip().split('\n\n')
    leaves, callers = collections.Counter(), collections.Counter()
    selected = allocated = total = selected_samples = 0
    for block in blocks:
        lines = block.splitlines()
        match = re.match(r'^\S+\s+\d+/\d+\s+[0-9.]+:\s+(\d+) cycles:u:', lines[0])
        assert match
        period = int(match[1])
        total += period
        frames = []
        for line in lines[1:]:
            match = re.match(r'\s+[a-f0-9]+ (.*?) \((.*)\)$', line)
            assert match
            frames.append(match[1])
        indices = [i for i, frame in enumerate(frames) if frame == symbol]
        if not indices:
            continue
        selected_samples += 1
        selected += period
        leaves[frames[0]] += period
        if allocator.match(frames[0]):
            allocated += period
        caller = next((frame for frame in frames[max(indices) + 1:]
                       if any(crate in frame for crate in ['hypercurve::', 'hyperlimit::', 'hypersolve::'])),
                      '<no recovered predicate or curve ancestor>')
        callers[caller] += period
    assert total == observed['total_sample_period']
    recorded = next(row for row in observed['inclusive_symbols'] if row['symbol'] == symbol)
    assert recorded['period'] == selected

    def rows(counter):
        return [{'symbol': key, 'period': value, 'total_profile_percent': 100 * value / total,
                 'selected_percent': 100 * value / selected} for key, value in counter.most_common()]

    result = {'scope': __doc__, 'runner_sha256': sha(Path(__file__)),
              'stacks_sha256': sha(OUT / 'stacks.stdout.gz'), 'symbol': symbol,
              'selected_samples': selected_samples, 'total_period': total, 'selected_period': selected,
              'inclusive_percent': 100 * selected / total,
              'allocator_leaf_subset_total_percent': 100 * allocated / total,
              'allocator_leaf_subset_of_selected_percent': 100 * allocated / selected,
              'leaf_symbols': rows(leaves), 'nearest_recovered_curve_or_predicate_ancestors': rows(callers),
              'limits': 'Inclusive sampled cycle periods are not invocation counts or predicted savings. The profile does not reveal operand shapes; a pure-rational scaling opportunity is a hypothesis requiring a separate observation.',
              'runtime_admitted': False}
    (OUT / 'multiply-costs.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({key: result[key] for key in ['selected_samples', 'inclusive_percent', 'allocator_leaf_subset_total_percent']}))


if __name__ == '__main__':
    main()
