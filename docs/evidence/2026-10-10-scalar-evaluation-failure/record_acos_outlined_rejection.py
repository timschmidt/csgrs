#!/usr/bin/env python3
"""Preserve the outlined experiment before restoring the retained source."""
import difflib
import gzip
import hashlib
import json
from pathlib import Path
import re
import sys

sys.dont_write_bytecode = True
from parse_test_results_v3 import parse_phase

E=Path(__file__).resolve().parent
ROOT=E.parents[2]
SOURCE=ROOT/'target/scalar-evaluation-candidate/source'
read=lambda p:json.loads(p.read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
expected='2c9f69fb9f20310fb5d9b21acf98cdb5e830eea9e6b0e43975f5b85b1c5b173c'
out=E/'acos-outlined-rejection-v1495.json'
assert not out.exists()
record=E/'typed-api-prototype-v1492-acos-outlined-build/source-hashes.json.gz'
assert sha(record)==expected
source=json.loads(gzip.decompress(record.read_bytes()))
assert all(sha(SOURCE/n)==v for n,v in source.items())
baseline=read(E/'acos-warm-baseline-v1422.json')
before=json.loads(gzip.decompress((E/baseline['source_record']).read_bytes()))
kernel='crates/hyperreal/src/computable/approximation/inverse_trig.rs'
assert {n for n,v in source.items() if before.get(n)!=v}=={kernel}
original=(E/baseline['source_record']).parent/'inputs'/kernel
patch=E/'acos-outlined-runtime-diff-v1495.patch'
assert not patch.exists()
patch.write_text(''.join(difflib.unified_diff(original.read_text().splitlines(True),(SOURCE/kernel).read_text().splitlines(True),fromfile='baseline/'+kernel,tofile='outlined/'+kernel)))
passes={}
for version,suffix in [(1487,'build'),(1490,'unit-tests'),(1491,'clippy'),(1492,'build')]:
    phase=f'typed-api-prototype-v{version}-acos-outlined-{suffix}'
    result=read(E/phase/'result.json')
    assert result['exit_code']==0 and result['source_unchanged'] and result['main_unchanged']
    assert sha(E/phase/'source-hashes.json.gz')==expected
    passes[phase]=result
units=parse_phase(E/'typed-api-prototype-v1490-acos-outlined-unit-tests')
assert units['top_level_passed']==1064 and units['child_process_passed']==1
assert units['top_level_failed']==units['child_process_failed']==0
assert not any(l.startswith('warning:') for l in (E/'typed-api-prototype-v1491-acos-outlined-clippy/stderr.log').read_text().splitlines())
caps=read(E/'acos-warm-allocation-caps-v1489.json')
for phase,row in caps['tests'].items():
    assert row['result']['exit_code']==row['cap']['exit_code']==0 and not row['cap']['wall_timeout']
    assert sha(E/phase/'source-hashes.json.gz')==expected
for old,new in [(1424,1488),(1425,1489)]:
    assert (E/f'typed-api-prototype-v{old}-acos-warm-allocation-cap/stderr.log').read_bytes()==(E/f'typed-api-prototype-v{new}-acos-warm-allocation-cap/stderr.log').read_bytes()
cost=read(E/'typed-api-prototype-v1493-acos-outlined-targeted-cost/summary.json')
base_cost=read(E/'typed-api-prototype-v1418-acos-warm-baseline-cost/summary.json')
assert cost['processes']==72 and cost['guards_unchanged'] and cost['source_hashes_sha256']==expected
assert all(cost['cases'][name][key]==row[key] for name,row in base_cost['cases'].items() for key in ['iterations','checksums','reference','output_bytes'])
assert all(sha(E/'typed-api-prototype-v1418-acos-warm-baseline-cost'/n)==sha(E/'typed-api-prototype-v1493-acos-outlined-targeted-cost'/n) for n in ['probe.rs','reproduce.py'])
paired={}
for name in ['acos-outlined-targeted-cost-v1494','acos-outlined-preconditioned-cost-v1495']:
    data=read(E/name/'summary.json')
    assert data['processes']==144 and data['guards_unchanged'] and data['source_record_sha256']==expected
    assert all(sha(E/name/n)==v for n,v in data['artifacts_sha256'].items())
    paired[name]={row['arguments'][0]:row['median_percent_change'] for row in data['cases']['acos']}
assert paired['acos-outlined-targeted-cost-v1494']['cached_negative']>25
assert paired['acos-outlined-preconditioned-cost-v1495']['cached_negative']>15
out.write_text(json.dumps({
    'status':'Rejected outlined candidate; cached-query performance regresses despite allocation gains',
    'previous_retained_checkpoint':'typed-api-prototype-v1404-state.json',
    'baseline_commit':'782329f0c','first_attempt_commit':'dee893319',
    'source_record':str(record.relative_to(E)),'source_record_sha256':expected,'runtime_diff':patch.name,
    'all_ten_frozen_tests_unchanged':True,'all_prior_tests_unchanged':True,
    'passing_validations':passes,'unit_summary':units,'allocation_regressions':caps,
    'allocation_observations_identical_to_first_attempt':True,
    'targeted_cost':'typed-api-prototype-v1493-acos-outlined-targeted-cost/summary.json',
    'paired_percent_changes':paired,
    'decision':'The first candidate preserves 160 cache schedules and 1584 frozen replay receipts while reducing the million-bit Rust allocation request total from about 11.4 MB to about 250 KB. Its negative completed-cache query regresses by 31.65 percent in the first paired run and 31.32 percent with fixed-CPU preparation. Outlining the identical arithmetic preserves all ten new tests, allocation gains and 1064 passing scalar units plus one child, but the corresponding paired medians remain +30.26 and +17.00 percent. The governing ladder puts performance before memory; neither candidate is retained. No causal explanation for the cached-query timing difference is claimed.',
    'restoration_plan':'Restore all source bytes to v1404, including removing only the new unretained probe file and include. Keep every new fixture, full-resolution failed cap, allocation regression, cost trial, implementation snapshot and helper in committed evidence. Re-run the retained scalar suite after restoration and verify the full source/main guards.',
    'coverage_limits':['The outlined screen stops at its adverse targeted cost result. No ordinary-family replay, broad checkpoint, new formal proof or full-board admission is claimed for it.',
        'The proposed allocation ceiling is an optimization regression guard, not an EvaluationBudget contract. All measured value/error contracts and retained facts remain consistent.',
        'The earlier compiled-only coarse-cache test correction and failed cost-probe build remain preserved. No failing phase or unfavorable trial is discarded.',
        'All 39 properties, 14 targets and crate future work remain in scope; main runtime retains the original scalar failure and the original 49 worktree entries.']
},indent=2)+'\n')
print(json.dumps({'record':out.name,'source_sha256':expected,'paired_percent_changes':paired}))
