#!/usr/bin/env python3
"""Correct a reporting count without rewriting executed evidence."""
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
old = E/'algebraic-preflight-baseline-v1085.json'
data = json.loads(old.read_text())
test = E/Path(data['source_record']).parent/'inputs/crates/hyperreal/src/computable/node/algebraic_preflight_tests.rs'
body = test.read_text().split('fn algebraic_preflight_public_retries_keep_completed_zero_proofs() {')[1]
assert 'for bits in [65, 4097, 65_537]' in body
assert 'for degree in [2, 3, 4]' in body
assert 'for units in [0, 8, 128]' in body
assert data['coverage']['public_retry_schedules'] == 81
data['coverage']['public_retry_schedules'] = 3*3*3
data['coverage']['public_generous_retry_allowance'] = data['coverage'].pop('public_growing_retry_allowance')
data['coverage_count_correction'] = {
    'previous_record':old.name,
    'previous_record_sha256':hashlib.sha256(old.read_bytes()).hexdigest(),
    'original_reported_schedules':81,
    'corrected_schedules':27,
    'basis':'Three stored-factor widths times three root degrees times three cold allowances gives 27 schedules. Each schedule makes three sign queries: cold, generous retry, then zero-work proof reuse. The earlier 81 count describes sign calls, not schedules.',
    'test_source_sha256':hashlib.sha256(test.read_bytes()).hexdigest(),
    'runtime_tests_fixtures_assertions_and_measurements_unchanged':True,
}
out = E/'algebraic-preflight-baseline-v1085-corrected.json'
assert not out.exists()
out.write_text(json.dumps(data,indent=2)+'\n')
print('Retained corrected coverage: 27 schedules, 81 sign queries; all original evidence preserved.')
