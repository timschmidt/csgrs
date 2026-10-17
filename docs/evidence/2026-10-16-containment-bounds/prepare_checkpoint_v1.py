"""Retain prior closure checks while pinning the bounds proof's exact scope."""
from pathlib import Path
W=Path(__file__).resolve().parent
s=(W.parent/'containment-winding/checkpoint_v1.py').read_text()
def change(old,new):
    global s
    assert old in s,old
    s=s.replace(old,new)
change('"""Pin the exercised winding migration and preserve inherited evidence limitations."""',
    '"""Pin the qualified bounds proof while preserving inherited evidence limitations."""')
change("E=ROOT/'docs/evidence/2026-10-16-containment-winding'", "E=ROOT/'docs/evidence/2026-10-16-containment-bounds'")
change("PARENT=ROOT/'docs/evidence/2026-10-16-containment-input-validation'", "PARENT=ROOT/'docs/evidence/2026-10-16-containment-winding'")
change('len(prior)==1000240','len(prior)==1039119')
change('containment_winding_admitted','containment_bounds_admitted')
change('candidate-preparation-v3','candidate-preparation-v2')
change('candidate-source-v3','candidate-source-v2')
change('len(final)==2829','len(final)==2830')
start=s.index("for version in ['v1','v2']:")
end=s.index("assert sha(E/'final-input-hashes-v1.json.gz')",start)
s=s[:start]+'''native_inputs=gz(E/'candidate-preparation-v1/source-hashes.json.gz')
assert native_inputs.keys()==final.keys()
assert {p for p in final if final[p]!=native_inputs[p]}=={'tests/wasm/geometry_certainty.cjs'}
assert all(sha(WORK/'candidate-source-v1'/p)==h for p,h in native_inputs.items())
assert application['native_inputs_identical_between_v1_v2']
'''+s[end:]
change("[('candidate-validation-v3','candidate_v3.py','validation_common_v3.py'),('wasm-validation-v3','wasm_v3.py','validation_common_v3.py'),('ffi-validation-v3','ffi_v1.py','validation_common_v3.py')]",
    "[('candidate-validation-v1','candidate_v1.py','validation_common_v1.py'),('wasm-validation-v2','wasm_v2.py','validation_common_v2.py')]")
change("['candidate-validation-v1','candidate-validation-v2','wasm-validation-v1','wasm-validation-v2']", "['wasm-validation-v1']")
change("assert read(E/'baseline-v6/result.json')['complete']", "assert read(E/'baseline-v1/result.json')['complete']\nassert read(E/'baseline-v1/result.json')['native_test_exit_code']==read(E/'baseline-v1/result.json')['wasm_test_exit_code']==0\nassert read(E/'wasm-validation-v1/result.json')['unexpected_exits']==['real-consumers']\nassert read(E/'supporting-fact-v1/result.json')['read_only_inputs_unchanged']")
change('wasm-validation-v3','wasm-validation-v2')
change("assert all(sha(p)==r['sha256'] for p,r in read(E/'ffi-validation-v3/result.json')['artifacts'].items())\n",'')
change('cost-v1/result.json','cost-v2/result.json')
change('cost-v1/protocol.json','cost-v2/protocol.json')
change("assert read(E/'cost-v2/result.json')['complete'] and read(E/'cost-v2/result.json')['candidate_matrix_all_certified']",
    "assert read(E/'cost-v2/result.json')['complete'] and read(E/'cost-v2/result.json')['baseline_and_candidate_all_correct_certified']\nassert read(E/'cost-v2/result.json')['all_preregistered_allocation_gates_passed']")
change('new physical nonzero-winding evidence; inherited audit remains failed/incomplete',
    'new physical certified-bounds evidence; inherited audit remains failed/incomplete')
change('including failed preparation, source copies, native/WASM targets, consumers, probes and raw observations',
    'including the initial fixed-Unknown assertion failure, both source copies, native/WASM targets, consumers, probes and raw observations')
start=s.index("current['containment_winding']=")
end=s.index('\n\nupdated=',start)
s=s[:start]+'''current['containment_bounds']={
 'status':decision['status'],'main_parent_commit':BASE,
 'contract':PREFIX+'contract-v1.md','progress':PREFIX+'progress-v1.md',
 'decision':PREFIX+'decision-v1.json','checkpoint':PREFIX+'checkpoint-v1.json',
 'artifact_expectations':PREFIX+'artifact-expectations-v1.json.gz',
 'implementation_sha256':implementation,'cost':PREFIX+'cost-v2/result.json',
 'cost_contract':PREFIX+'cost-contract-v1.json','baseline_native':decision['baseline_native'],
 'baseline_wasm':decision['baseline_wasm'],'new_native_tests':4,'winding_wasm_tests':10,
 'containment_bounds_admitted':True,'all_preregistered_allocation_gates_passed':True,
 'stronger_proof_evidence':PREFIX+'supporting-fact-v1/result.json',
 'new_certified_completion_cases_separately_demonstrated':1,
 'native_v1_reuse':decision['native_v1_reuse'],'preserved_failed_attempt':decision['preserved_failed_attempt'],
 'universal_set_membership_correctness_claimed':False,'intentional_migration':None,
 'pending_native_paths_remaining':0,'native_disposition_preserved':True,
 'published_package_pipeline_executed':False,'full_plan_complete':False,
 'limitations':decision['limitations'],'next':decision['next']}
'''+s[end:]
change('exercised nonzero-winding and cancelled-sheet composition migration; inherited history limitations preserved',
    'certified supporting-plane proof and completed-query allocation reduction; inherited history limitations preserved')
(W/'checkpoint_v1.py').write_text(s)
print('Prepared bounds checkpoint with protected-ledger and independent physical-path audit',flush=True)
