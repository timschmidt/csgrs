"""Record atan evidence while preserving every property and target requirement."""
import copy
import hashlib
import json
from pathlib import Path
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
p = ROOT/'docs/properties-status.json'
assert read(E/'atan-restoration-validation-v1576.json')['source_record_sha256'] == '32b816463eacd2d2cef2f905c0455557c015117690a8a8dd635229a57726d2b2'
assert subprocess.check_output(['git', 'show', 'HEAD:docs/properties-status.json'], cwd=ROOT) == p.read_bytes()
before_bytes = p.read_bytes(); before = read(p); after = copy.deepcopy(before)
assert len(before['properties']) == 39 and len(before['targets']) == 14
p4 = next(r for r in after['properties'] if r['id'] == 'P4')
for name in ['atan-zero-term-baseline-v1515.json', 'atan-zero-term-implementation-v1540.json',
             'atan-magnitude-guard-rejection-v1541.json', 'atan-quotient-implementation-v1568.json',
             'atan-quotient-rejection-v1571.json', 'atan-source-restoration-v1572.json',
             'atan-restoration-validation-v1576.json', 'typed-api-prototype-v1576-state.json']:
    path = str(E.relative_to(ROOT)/name)
    assert path not in p4['evidence']; p4['evidence'].append(path)
update = (
    'The v1576 checkpoint restores the same retained runtime after measuring and rejecting two integral-atan '
    'zero-term shortcuts. All 320 schedules across public integer/reciprocal/opaque and private integral/deferred '
    'diagnostic routes preserve answers, allowances and cache facts; 164 independent rational enclosures and '
    'domain/precision/cancellation checks pass. The original two allocation guards fail before either repair. '
    'Million-bit direct dispatch requests 4801400 Rust bytes before repair, 16 with the magnitude guard and 24 '
    'with the quotient guard; public rational queries retain an input-sized copy. These observations establish '
    'no byte/time budget or cache contradiction. Both variants pass 1063 scalar units plus one child, all nine '
    'capped tests, strict five-crate Clippy and formatting, with all frozen test bodies unchanged. All seven cost '
    'families preserve complete outputs. The magnitude guard retains a +7.56/+9.82 percent single-root cache '
    'regression before/after fixed-CPU preparation. The quotient guard removes that large regression but retains '
    '+9.45/+11.18 percent wide-coarse integer increases in complete nine-repetition and focused 27-repetition '
    'controlled comparisons. Its wide-pi increase does not persist in the focused run. Both variants are rejected; '
    'no causal explanation is established. Every trial, failed helper assumption, original allocation failure and '
    'all nine new test bodies remain archived. Restoration reruns 1054 retained scalar units plus one child, '
    'the release build and workspace/105-input formatting. Prior v1404 broad tests, replays and conditional proofs '
    'remain evidence for identical restored bytes, not fresh runs. Continue public deferred-constructor copies, '
    'log-range fallbacks, retained quadratic towers, general cache/concurrency coverage and unchanged native/full-board '
    'admission. All 39 properties, 14 targets and crate future work remain in scope.'
)
p4['remaining'] += ' '+update
lead = 'Continue the isolated fallible scalar evaluator from typed-api-prototype-v1500-state.json before production admission. Its retained runtime is byte-identical to v1404; both new acos allocation variants were rejected.'
assert after['next_actions'][0].startswith(lead)
after['next_actions'][0] = after['next_actions'][0].replace(lead,
    'Continue the isolated fallible scalar evaluator from typed-api-prototype-v1576-state.json before production admission. Its retained runtime is byte-identical to v1404; both acos and both atan allocation variants remain rejected.', 1)
old = 'numeric atan, log-range fallbacks and retained quadratic-tower copies remain source-only candidates.'
assert old in after['next_actions'][0]
after['next_actions'][0] = after['next_actions'][0].replace(old,
    'numeric atan has now been measured; log-range fallbacks and retained quadratic-tower copies remain source-only candidates.')
after['next_actions'][0] += ' '+update
comparison = copy.deepcopy(after)
row = next(r for r in comparison['properties'] if r['id'] == 'P4')
original = next(r for r in before['properties'] if r['id'] == 'P4')
row['evidence'] = original['evidence']; row['remaining'] = original['remaining']
comparison['next_actions'][0] = before['next_actions'][0]
assert comparison == before
p.write_text(json.dumps(after, indent=2)+'\n')
out = E/'atan-ledger-update-v1576.json'; assert not out.exists()
out.write_text(json.dumps({'before_sha256': hashlib.sha256(before_bytes).hexdigest(), 'after_sha256': sha(p),
    'changed_fields': ['P4.evidence', 'P4.remaining', 'next_actions[0]'],
    'all_39_property_and_14_target_requirements_and_statuses_preserved': True,
    'other_next_actions_and_crate_future_work_preserved': True}, indent=2)+'\n')
print(out.read_text(), end='')
