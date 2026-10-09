"""Locate geometry consumers in the already-recorded frame-pointer profile.

This reuses the pre-ownership-trial profile; it does not sample the current
inline-storage experiment or infer call counts from cycle samples.
"""
from collections import Counter
import gzip
import hashlib
import json
from pathlib import Path
import re

root = Path(__file__).resolve().parent
profile = root / 'current-native-frame-pointer-profile'
source = profile / 'stacks.txt.gz'
observations = json.loads((profile / 'observations.json').read_text())
with gzip.open(source, 'rt') as stream:
    blocks = stream.read().strip().split('\n\n')
total = normal_form = 0
pairs = Counter()
for block in blocks:
    lines = block.splitlines()
    match = re.match(r'^\S+\s+\d+/\d+\s+([0-9.]+):\s+(\d+) cycles:u:', lines[0])
    assert match, lines[0]
    period = int(match[2])
    total += period
    frames = []
    for line in lines[1:]:
        frame = re.match(r'\s+[a-f0-9]+ (.*?) \((.*)\)$', line)
        assert frame, line
        frames.append(frame[1])
    indices = [i for i, symbol in enumerate(frames) if 'pi_laurent_polynomial' in symbol]
    if not indices:
        continue
    normal_form += period
    outer = [symbol for symbol in frames[max(indices) + 1:]
             if any(crate in symbol for crate in ['hypercurve::', 'hyperlimit::', 'hypersolve::'])]
    if outer:
        owner = next((symbol for symbol in outer[1:]
                      if not any(crate in symbol for crate in
                                 ['hypercurve::classify::', 'hyperlimit::', 'hypersolve::classification::'])), None)
        pairs[outer[0], owner] += period
assert len(blocks) == observations['samples']
assert total == observations['total_sample_period']
assert normal_form == observations['groups']['polynomial_normal_form']['period']
result = dict(
    scope='Reanalysis of the recorded pre-ownership frame-pointer profile. Each normal-form sample contributes once to its nearest recovered curve/predicate/solver caller and the next recovered consumer outside common classification/predicate wrappers. Categories sum sampled userspace cycle periods, not call counts or ordinary release timings. A missing consumer is null.',
    source_profile='current-native-frame-pointer-profile/manifest.json',
    source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    samples=len(blocks), total_period=total, normal_form_period=normal_form,
    rows=[dict(caller=caller, consumer=consumer, period=period, total_profile_percent=100 * period / total)
          for (caller, consumer), period in pairs.most_common()],
    interpretation='Beyond temporary storage, the largest recovered consumers include bounding-box coordinate selection, circular-arc endpoint tests, materialized-subcurve equality, and parameter ordering. These identify candidate sites for inspection; they do not establish profitable shortcuts or justify changing certainty, bounds, or equality semantics.',
)
(root / 'normal-form-caller-pairs.json').write_text(json.dumps(result, indent=2) + '\n')
print('recovered consumer pairs:', len(pairs))
