#!/usr/bin/env python3
"""Keep the full plan and retained runtime after two evidence-based rejections."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

E=Path(__file__).resolve().parent
ROOT=E.parents[2]
read=lambda p:json.loads(p.read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
previous=E/'typed-api-prototype-v1404-state.json'
assert sha(previous)=='fad2d16a1483824f4d6587e26a22049bfac5b715e4a2ecf9f8a74661b6f34e77'
prior=read(previous)
validation=read(E/'acos-restoration-validation-v1500.json')
assert validation['source_record_sha256']==prior['source_record_sha256']
assert read(E/'acos-ledger-update-v1500.json')['after_sha256']==sha(ROOT/'docs/properties-status.json')
metadata={
    'previous_checkpoint':previous.name,
    'status':'Both acos allocation variants rejected; retained v1404 runtime restored exactly; no runtime admission',
    'contract_decision':'p4-contract-decision-v173.json',
    'accepted_cache_rule':'Warming may strengthen Unknown into Certified and reuse higher-precision proofs at zero work; no contradiction or weakening of an available certificate.',
    'evidence_commits':['782329f0c','dee893319'],
    'retained_runtime_checkpoint':previous.name,
    'retained_runtime_source_sha256':prior['source_record_sha256'],
    'baseline_record':'acos-warm-baseline-v1422.json',
    'rejected_experiments':['acos-residual-implementation-v1448.json','acos-residual-retention-v1486.json','acos-outlined-rejection-v1495.json'],
    'restoration_record':'acos-source-restoration-v1496.json',
    'restoration_validation_record':'acos-restoration-validation-v1500.json',
    'completed_orchestrations':'acos-completed-orchestrations-v1500.json',
    'fresh_scalar_test_summary':validation['tests'],
    'fresh_included_rustfmt_files':validation['included_rust_files'],
    'restored_release_libraries_match_retained':validation['restored_libraries_match_retained'],
    'measurements':{
        'baseline_public_and_private_diagnostic_schedules':160,
        'new_test_functions_archived':10,
        'baseline_failed_allocation_guards':2,
        'first_candidate_focused_replays_with_identical_receipts':1584,
        'first_candidate_scalar_units':1064,'outlined_candidate_scalar_units':1064,
        'child_tests_per_candidate_suite':1,
        'original_first_candidate_paired_cost_processes':720,
        'controlled_first_candidate_paired_cost_processes':720,
        'outlined_paired_cost_processes':144,'controlled_outlined_paired_cost_processes':144,
        'each_controlled_cost_process_has_one_separate_fixed_cpu_prelude':True,
        'million_bit_baseline_rust_bytes_positive_negative':[11425512,11425952],
        'million_bit_candidate_rust_bytes_positive_negative':[250392,250832],
        'first_candidate_negative_cached_percent_change':[31.653991330742272,31.32],
        'outlined_negative_cached_percent_change':[30.26148884386202,16.99721252381896],
    },
    'decision':'Neither allocation optimization passes the higher-priority performance gate. Preserve every trial and fixture; restore retained source rather than admitting the unresolved cached-query regression.',
    'reused_prior_validation':{
        'checkpoint':previous.name,'checkpoint_sha256':sha(previous),
        'source_identical':True,
        'scope':'The full v1404 stack/default/debug/Clippy/format/cap/replay/proof evidence remains evidence for identical source bytes. This turn freshly reruns retained scalar units, the release build and format checks only.'
    },
    'coverage_limits':[
        'Cooperative EvaluationBudget excludes per-operation byte/time bounds. The measured rational copying is an optimization opportunity, not a demonstrated allowance or cache-value contract violation.',
        'The million-bit aggregate surveys originally reached SIGXCPU after six logged schedules. Complete uncapped baseline schedules and individual allocation guards are separately retained; no individual query timeout is inferred.',
        'Domain-only warming is an internal diagnostic. Coarse and completed numeric warming use public APIs, with setup outside allocation windows.',
        'All observed signs, domain facts, numeric values, error contracts, exhausted-query cache retention and completed zero-work reuse agree. The new test correction permits a finer cache to rescale to a different valid coarse integer.',
        'Rust counters exclude fixtures, native allocations, RSS and peak-live memory. Local timing controls do not establish the cause of cached-query regressions or whole-board performance.',
        'The rejected numerical kernels are outside the conditional Verus target. No new proof or whole-crate obligation closure is claimed.',
        'The outlined screen stops at adverse targeted costs; no ordinary-family or full replay coverage is claimed for that variant.',
        'Prepared checkpoint helpers for v1449-v1485 were never executed and refer to the rejected first variant.',
        'All prior retained tests and original full-resolution hard cases remain unchanged. All ten new test bodies and both implementations remain archived as rejected-experiment evidence.',
        'Main runtime retains the original scalar failure and all original 49 worktree entries. All 39 properties, 14 targets and crate future work remain in scope.'
    ],
    'remaining_source_review':'integer-scale-deferred-review-v1404.json',
    'remaining_requirements':prior['remaining_requirements'],
    'next_measurements':[
        'Numeric AtanDeferred paths: separate internal rational diagnostics from ordinary public AtanRational controls and public opaque/rational-offset reachability.',
        'Log-range fallback: separate necessary operand sampling from avoidable constructors and establish a public path to an explicit EvaluationBudget.',
        'Retained quadratic towers: establish public proof warming before measuring later fine-cache misses and coefficient copies.',
        'Any further acos residual rewrite must address the recorded completed-cache cost regression while preserving all frozen schedules and original workloads.'
    ]
}
path=E/'typed-api-prototype-v1500-metadata.json';assert not path.exists()
path.write_text(json.dumps(metadata,indent=2)+'\n')
phases=list(validation['phases'])
subprocess.run([sys.executable,str(E/'record_checkpoint_v4.py'),path.name,'typed-api-prototype-v1500-state.json',
    'typed-api-prototype-v1497-restored-build',*phases],cwd=ROOT,check=True)
state=read(E/'typed-api-prototype-v1500-state.json')
assert state['changed_since_previous_checkpoint']=={}
assert state['source_files']==24743 and state['archived_changed_sources_verified']==120
print(json.dumps({'checkpoint':'typed-api-prototype-v1500-state.json',
    'checkpoint_sha256':sha(E/'typed-api-prototype-v1500-state.json'),
    'artifacts':len(state['artifacts_sha256']),'source_files':state['source_files'],
    'source_unchanged_from_v1404':True}),flush=True)
