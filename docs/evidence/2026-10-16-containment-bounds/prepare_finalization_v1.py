"""Preserve the failed fixed-Unknown assertion and qualify the stronger proof."""
from pathlib import Path
W=Path(__file__).resolve().parent
def change(s,old,new):
    assert old in s,old
    return s.replace(old,new)
s=(W/'cost_v1.py').read_text()
s=change(s,"import hashlib,json,resource,statistics,subprocess,time","import gzip,hashlib,json,resource,statistics,subprocess,time")
s=change(s,"O=W/'cost-v1'","O=W/'cost-v2'")
s=change(s,"W/'wasm-validation-v1/result.json'","W/'wasm-validation-v2/result.json'")
guard='''with gzip.open(W/'candidate-preparation-v1/source-hashes.json.gz','rt') as f:native_inputs=json.load(f)
with gzip.open(W/'candidate-preparation-v2/source-hashes.json.gz','rt') as f:final_inputs=json.load(f)
assert native_inputs.keys()==final_inputs.keys()
assert {p for p in native_inputs if native_inputs[p]!=final_inputs[p]}=={'tests/wasm/geometry_certainty.cjs'}
assert all(sha(W/'candidate-source-v1'/p)==h for p,h in native_inputs.items())
assert all(sha(W/'candidate-source-v2'/p)==h for p,h in final_inputs.items())
qualified.extend([W/'candidate-preparation-v1/source-hashes.json.gz',W/'candidate-preparation-v2/source-hashes.json.gz'])
'''
s=change(s,'O.mkdir(exist_ok=False)',guard+'O.mkdir(exist_ok=False)')
s=change(s,"'baseline_and_candidate_all_correct_certified']=True","'baseline_and_candidate_all_correct_certified']=True\nresult['native_v1_reuse']='Every Rust source, manifest, dependency and native test is identical in final v2; only a WASM consumer changed and wasm-validation-v2 executed it.'")
(W/'cost_v2.py').write_text(s)
s=(W/'apply_v1.py').read_text()
s=change(s,"'wasm-validation-v1'","'wasm-validation-v2'")
s=change(s,"wasm-validation-v1/result.json","wasm-validation-v2/result.json")
s=change(s,"cost-v1/result.json","cost-v2/result.json")
s=change(s,"candidate-preparation-v1/","candidate-preparation-v2/")
s=change(s,"candidate-source-v1","candidate-source-v2")
s=change(s,"'tests/wasm/containment_winding.cjs'}","'tests/wasm/containment_winding.cjs','tests/wasm/geometry_certainty.cjs'}")
s=change(s,"folders=['prototype-v1','baseline-v1','candidate-preparation-v1','candidate-validation-v1','wasm-validation-v2','cost-v1']",
    "folders=['prototype-v1','baseline-v1','supporting-fact-v1','candidate-preparation-v1','candidate-preparation-v2','candidate-validation-v1','wasm-validation-v1','wasm-validation-v2','cost-v2']")
guard='''native=gz(WORK/'candidate-preparation-v1/source-hashes.json.gz')
assert native.keys()==qualified.keys()
assert {p for p in native if native[p]!=qualified[p]}=={'tests/wasm/geometry_certainty.cjs'}
assert all(sha(WORK/'candidate-source-v1'/p)==h for p,h in native.items())
assert read(WORK/'wasm-validation-v1/result.json')['unexpected_exits']==['real-consumers']
'''
s=change(s,'E.mkdir(exist_ok=False)',guard+'E.mkdir(exist_ok=False)')
s=change(s,"'all_final_inputs_match_qualified_sources':True,","'all_final_inputs_match_qualified_sources':True,'native_inputs_identical_between_v1_v2':True,")
s=change(s,'Applied three qualified files','Applied four qualified files')
(W/'apply_v2.py').write_text(s)
s=(W/'assess_v1.py').read_text()
s=change(s,'wasm-validation-v1','wasm-validation-v2')
s=change(s,'cost-v1/result.json','cost-v2/result.json')
s=change(s,"'new_certified_completion_cases_separately_demonstrated':0", "'new_certified_completion_cases_separately_demonstrated':1,\n    'stronger_proof_evidence':'supporting-fact-v1/result.json',\n    'preserved_failed_attempt':'wasm-validation-v1 fails an old assertion requiring Unknown; later consumer stages were not reached. Final v2 requires Certified false for the same query and adds a separate upper-face Unknown/qualified case.',\n    'native_v1_reuse':'All Rust sources, manifests, dependencies and native tests are identical in final v2; only tests/wasm/geometry_certainty.cjs changed and was requalified by wasm-validation-v2.'")
s=change(s,'applying the three paths','applying the four paths')
s=change(s,'No additional previously undecided input is credited as newly certified without separate evidence.',
    'Separately retained baseline/candidate observations show the inverse-rotated cube query at (0.5, 0, 0.5) strengthening from strict Unknown / approximate qualified false to Certified false under both policies. Its certified lower supporting plane proves non-interiority of the retained set. This does not prove fidelity to the original cube or certify prior approximate intent. The original query and geometry remain in the consumer with stronger assertions; an added upper-face query preserves strict Unknown and qualified approximate false coverage. This capability gain is not included in the cost comparison.')
s=change(s,"    'Remaining limits:','',", "    'The first WASM attempt stopped at the old fixed-Unknown assertion, so subsequent consumer and TypeScript stages did not execute in that attempt. It remains preserved. Final v2 changes only that JavaScript consumer; every native source, dependency, manifest and test is byte-identical to the complete candidate-v1 native qualification. WASM v2 uses a fresh target and executes the final consumer suite. No completed Cargo target was reopened.','',\n    'Remaining limits:','',")
(W/'assess_v2.py').write_text(s)
print('Prepared versioned cost, application and assessment runners; native code remains v1-identical',flush=True)
