"""Attribute a declared allocator-leaf subset and point-equality stacks."""
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
    manifest = json.loads((OUT / 'manifest.json').read_text())
    observed = json.loads((OUT / 'observations.json').read_text())
    assert sha(OUT / 'stacks.stdout.gz') == manifest['artifacts_sha256']['stacks.stdout.gz']
    allocator = re.compile(r'^(?:_int_malloc|_int_free|_int_realloc|unlink_chunk|malloc|cfree@|free@|__libc_malloc|__libc_free|__libc_realloc|__rustc::__rust_alloc|__rustc::__rust_dealloc|__rustc::__rust_realloc)')
    project = re.compile(r'^<?&?(?:hyperreal|hypercurve|hyperlimit|hypersolve|hyperlattice|hyperdrc|hypermesh|hyperpath|hypertri)::')
    point = '<hypercurve::point::Point2 as core::cmp::PartialEq>::eq'
    allocation_callers = collections.Counter()
    allocation_leaves = collections.Counter()
    point_callers = collections.Counter()
    total = allocation = allocation_normal_form = point_weight = 0
    blocks = gzip.decompress((OUT / 'stacks.stdout.gz').read_bytes()).decode().strip().split('\n\n')
    for block in blocks:
        lines = block.splitlines()
        header = re.match(r'^\S+\s+\d+/\d+\s+([0-9.]+):\s+(\d+) cycles:u:', lines[0])
        assert header, lines[0]
        period = int(header[2])
        total += period
        frames = []
        for line in lines[1:]:
            frame = re.match(r'\s+[a-f0-9]+ (.*?) \((.*)\)$', line)
            assert frame, line
            frames.append(frame[1])
        if frames and allocator.match(frames[0]):
            allocation += period
            allocation_leaves[frames[0]] += period
            if any('pi_laurent_polynomial' in frame for frame in frames):
                allocation_normal_form += period
            caller = next((frame for frame in frames[1:] if project.match(frame)), '<no recovered project-prefixed caller>')
            allocation_callers[caller] += period
        indices = [i for i, frame in enumerate(frames) if frame == point]
        if indices:
            point_weight += period
            caller = next((frame for frame in frames[max(indices) + 1:] if project.match(frame)), '<no recovered project-prefixed caller>')
            point_callers[caller] += period
    assert total == observed['total_sample_period']
    assert len(blocks) == observed['samples']

    def rows(counter, selected):
        return [{'symbol': name, 'period': value, 'total_profile_percent': 100 * value / total,
                 'selected_weight_percent': 100 * value / selected}
                for name, value in counter.most_common()]

    result = {
        'scope': __doc__, 'runner_sha256': sha(Path(__file__)),
        'stacks_sha256': sha(OUT / 'stacks.stdout.gz'), 'samples': len(blocks),
        'total_period': total, 'allocator_symbol_pattern': allocator.pattern,
        'project_symbol_pattern': project.pattern,
        'allocator_leaf_period': allocation,
        'allocator_leaf_total_percent': 100 * allocation / total,
        'allocator_leaf_with_normal_form_total_percent': 100 * allocation_normal_form / total,
        'allocator_leaves': rows(allocation_leaves, allocation),
        'allocator_nearest_recovered_project_callers': rows(allocation_callers, allocation),
        'point_equality_total_percent': 100 * point_weight / total,
        'point_equality_nearest_recovered_project_callers': rows(point_callers, point_weight),
        'limits': 'The declared symbol subset is not an exhaustive allocation model. Callers are recovered ancestors with the selected prefix, not exact allocation sites; inline frames are absent. Each selected sample is attributed once. Percentages are sampled cycle periods, not calls, bytes, wall seconds or a speedup prediction. Point and allocator categories overlap.',
        'runtime_admitted': False,
    }
    (OUT / 'attribution.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({key: result[key] for key in ['allocator_leaf_total_percent', 'allocator_leaf_with_normal_form_total_percent', 'point_equality_total_percent']}))
    for row in result['allocator_nearest_recovered_project_callers'][:12]:
        print(round(row['total_profile_percent'], 3), row['symbol'])
    print('point equality callers')
    for row in result['point_equality_nearest_recovered_project_callers'][:8]:
        print(round(row['total_profile_percent'], 3), row['symbol'])


if __name__ == '__main__':
    main()
