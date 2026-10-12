#!/usr/bin/env python3
"""Record the rejected experiments without changing plan requirements/statuses."""
import copy
import hashlib
import json
from pathlib import Path
import subprocess

E=Path(__file__).resolve().parent
ROOT=E.parents[2]
p=ROOT/'docs/properties-status.json'
read=lambda path:json.loads(path.read_text())
sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
assert read(E/'acos-restoration-validation-v1500.json')['source_record_sha256']=='32b816463eacd2d2cef2f905c0455557c015117690a8a8dd635229a57726d2b2'
assert subprocess.check_output(['git','show','HEAD:docs/properties-status.json'],cwd=ROOT)==p.read_bytes()
before_bytes=p.read_bytes();before=read(p);after=copy.deepcopy(before)
assert len(before['properties'])==39 and len(before['targets'])==14
p4=next(r for r in after['properties'] if r['id']=='P4')
for name in ['acos-warm-baseline-v1422.json','acos-residual-implementation-v1448.json',
             'acos-residual-retention-v1486.json','acos-outlined-rejection-v1495.json',
             'acos-source-restoration-v1496.json','acos-restoration-validation-v1500.json',
             'typed-api-prototype-v1500-state.json']:
    path=str(E.relative_to(ROOT)/name)
    assert path not in p4['evidence'];p4['evidence'].append(path)
update=(
    'The v1500 checkpoint restores all v1404 source bytes after rejecting two acos half-angle allocation experiments. '
    'A public coarse warmup followed by a finer query was measured across 160 schedules, both signs, four widths '
    'through 1000001 bits, four cache states and five allowances. Domain-only warming is a separately identified '
    'private diagnostic; other warming/query paths use public APIs. All value/error results, available sign/domain '
    'certificates, exhausted-query cache retention, unlimited retries and completed zero-work reuse remain consistent. '
    'The measured million-bit fine query allocates about 11.4 MB after coarse warming; completed cache hits allocate '
    '40-48 Rust bytes. These are individual-operation allocation observations, not a cooperative-budget violation. '
    'The first direct residual and its outlined variant reduce the measured requests to about 250 KB and preserve '
    'the frozen correctness/allocation assertions, but negative completed-cache timing regresses: paired +31.65/+31.32 '
    'percent for the first variant and +30.26/+17.00 for the outlined variant, before/after fixed-CPU preparation. '
    'Both are rejected under the performance-before-memory ladder. All original and controlled trials, the two '
    'original full-size aggregate SIGXCPU failures, both failing pre-change allocation guards, corrected new-test '
    'expectation, compile failure and ten new test bodies remain archived. No older retained test changed. '
    'Restoration reruns 1054 scalar units plus one child, the release build, workspace formatting and 105 included '
    'Rust inputs. The v1404 broad tests/proofs remain prior evidence for identical source bytes, not fresh v1500 runs. '
    'Continue numeric atan, log-range fallback and retained quadratic-tower reachability measurements, general '
    'cache/kernel/concurrency coverage and the unchanged native/full-board admission gates. Further acos work '
    'must address the recorded cache-query cost before retention; no causal explanation is established.'
)
p4['remaining']+=' '+update
lead='Continue the isolated fallible scalar evaluator from typed-api-prototype-v1404-state.json before production admission.'
assert after['next_actions'][0].startswith(lead)
after['next_actions'][0]=after['next_actions'][0].replace(lead,
    'Continue the isolated fallible scalar evaluator from typed-api-prototype-v1500-state.json before production admission. Its retained runtime is byte-identical to v1404; both new acos allocation variants were rejected.',1)
old='The supplemental deferred-path review identifies cold/coarse-warmed rational acos, numeric atan, log-range fallbacks and retained quadratic-tower copies for measurement; it establishes no new failure or byte/time bound.'
assert old in after['next_actions'][0]
after['next_actions'][0]=after['next_actions'][0].replace(old,
    'The supplemental deferred-path review has now led to the measured acos schedules and rejected optimizations; numeric atan, log-range fallbacks and retained quadratic-tower copies remain source-only candidates. No new byte/time budget contract is established.')
after['next_actions'][0]+=' '+update
comparison=copy.deepcopy(after)
row=next(r for r in comparison['properties'] if r['id']=='P4')
original=next(r for r in before['properties'] if r['id']=='P4')
row['evidence']=original['evidence'];row['remaining']=original['remaining'];comparison['next_actions'][0]=before['next_actions'][0]
assert comparison==before
p.write_text(json.dumps(after,indent=2)+'\n')
record=E/'acos-ledger-update-v1500.json';assert not record.exists()
record.write_text(json.dumps({'before_sha256':hashlib.sha256(before_bytes).hexdigest(),'after_sha256':sha(p),
    'changed_fields':['P4.evidence','P4.remaining','next_actions[0]'],
    'all_39_property_and_14_target_requirements_and_statuses_preserved':True,
    'other_next_actions_and_crate_future_work_preserved':True},indent=2)+'\n')
print(record.read_text(),end='')
