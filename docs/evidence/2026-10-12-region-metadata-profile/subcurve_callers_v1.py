"""Locate recovered callers above subcurve equality, including derived equality chains."""
import collections
import gzip
import hashlib
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parent
OUT = E / 'profile-v1'
manifest = json.loads((OUT / 'manifest.json').read_text())
stack_file = OUT / 'stacks.stdout.gz'
assert hashlib.sha256(stack_file.read_bytes()).hexdigest() == manifest['artifacts_sha256']['stacks.stdout.gz']
observed = json.loads((OUT / 'observations.json').read_text())
target = '<hypercurve::bezier_split::BezierSubcurve2 as core::cmp::PartialEq>::eq'
project = re.compile(r'^<?&?(?:hyperreal|hypercurve|hyperlimit|hypersolve|hyperlattice|hyperdrc)::')
counters = {name: collections.Counter() for name in ['immediate_project', 'above_equality_chain']}
total = selected = 0
for block in gzip.decompress(stack_file.read_bytes()).decode().strip().split('\n\n'):
    lines = block.splitlines()
    header = re.match(r'^\S+\s+\d+/\d+\s+([0-9.]+):\s+(\d+) cycles:u:', lines[0])
    assert header, lines[0]
    period = int(header[2])
    total += period
    frames = []
    for line in lines[1:]:
        match = re.match(r'\s+[a-f0-9]+ (.*?) \((.*)\)$', line)
        assert match, line
        frames.append(match[1])
    indices = [index for index, frame in enumerate(frames) if frame == target]
    if not indices:
        continue
    selected += period
    callers = [frame for frame in frames[max(indices) + 1:] if project.match(frame)]
    counters['immediate_project'][callers[0] if callers else '<unrecovered>'] += period
    caller = next((frame for frame in callers if 'core::cmp::PartialEq>::eq' not in frame), '<unrecovered>')
    counters['above_equality_chain'][caller] += period
assert total == observed['total_sample_period']
result = {'scope': __doc__, 'target': target, 'selected_period': selected,
          'selected_total_profile_percent': 100 * selected / total,
          'rows': {name: [{'symbol': symbol, 'period': period, 'total_profile_percent': 100 * period / total}
                          for symbol, period in values.most_common()] for name, values in counters.items()},
          'limits': 'Recovered non-inline ancestors; sampled cycle periods, not call counts or predicted savings. No runtime change is proposed by this attribution.',
          'runtime_admitted': False}
(OUT / 'subcurve-callers.json').write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps(result, indent=2))
