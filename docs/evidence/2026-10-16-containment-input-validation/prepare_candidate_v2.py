"""Reuse native validation in the query's decision context, then freeze candidate."""
import gzip,hashlib,json,shutil,subprocess
from pathlib import Path
WORK=Path(__file__).resolve().parent
ROOT=WORK.parents[2]
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
assert json.loads((WORK/'wasm-reproduction-v1/result.json').read_text())['expected_failure_reproduced']
BASE=WORK/'baseline-source-v1';SOURCE=WORK/'candidate-source-v2';OUT=WORK/'candidate-preparation-v2'
with gzip.open(WORK/'baseline-preparation-v1/source-hashes.json.gz','rt') as f:inputs=json.load(f)
assert all(sha(BASE/p)==h for p,h in inputs.items())
SOURCE.mkdir(exist_ok=False);OUT.mkdir(exist_ok=False)
for p,h in inputs.items():
 dest=SOURCE/p;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(BASE/p,dest);assert sha(dest)==h
p=SOURCE/'crates/hypermesh/src/mesh.rs';s=p.read_text()
old='''        let decisions = DecisionContext::new(context);
        let view = self.as_ref();
        validate_non_empty_mesh_views(&[view])?;
        if !self
            .geometry_facts()
            .is_some_and(|facts| facts.valid_pwn.consume(&decisions))
        {
            validate_source_pwn(&decisions, view, 0)?;
        }
        Ok(decisions.finish(()))'''
new='''        let decisions = DecisionContext::new(context);
        self.validate_closed_pwn_decision(&decisions)?;
        Ok(decisions.finish(()))
    }

    fn validate_closed_pwn_decision(&self, decisions: &DecisionContext) -> HypermeshResult<()> {
        let view = self.as_ref();
        validate_non_empty_mesh_views(&[view])?;
        if !self
            .geometry_facts()
            .is_some_and(|facts| facts.valid_pwn.consume(decisions))
        {
            validate_source_pwn(decisions, view, 0)?;
        }
        Ok(())'''
assert s.count(old)==1;s=s.replace(old,new)
old='''    /// parity. Points on a triangle, edge, or vertex are not inside.'''
new='''    /// parity. Points on a triangle, edge, or vertex are not inside.
    ///
    /// Canonical empty geometry (both buffers empty) returns `false`. Every
    /// other input must satisfy [`Self::validate_closed_pwn`], including queries
    /// outside its bounds or on a source face. Invalid geometry returns its
    /// typed error before a containment shortcut can answer. Validation facts
    /// contribute their certainty and obey the same policy and source identity
    /// rules as explicit validation.'''
assert s.count(old)==1;s=s.replace(old,new)
start=s.index('    pub(crate) fn contains_point_decision(')
old='''        if self.triangles.is_empty() {
            return Ok(false);
        }
        let bounds ='''
new='''        if self.positions.is_empty() && self.triangles.is_empty() {
            return Ok(false);
        }
        self.validate_closed_pwn_decision(decisions)?;
        let bounds ='''
assert s[start:].count(old)==1;s=s[:start]+s[start:].replace(old,new,1);p.write_text(s)
p=SOURCE/'src/solid.rs';s=p.read_text();old='/// Exact point containment through the retained native triangle query path.'
new='''/// Exact point containment through the retained native triangle query path.
///
/// Canonical empty geometry returns `false`; every other input must be a valid
/// closed-PWN mesh. Malformed input fails before bounds or boundary shortcuts.
/// Points on source triangles are excluded by the native strict containment
/// query. This convenience entry point selects the strict predicate policy.'''
assert s.count(old)==1;s=s.replace(old,new)
old='/// Classifies a point with the selected policy and reports aggregate certainty.'
assert s.count(old)==1;s=s.replace(old,old+'''
///
/// Source validation and query decisions share the returned certainty receipt.
/// A cached approximate validation fact cannot certify a strict query.''');p.write_text(s)
p=SOURCE/'src/wasm/mesh_js.rs';s=p.read_text();old='    #[wasm_bindgen(js_name = containsPoint)]'
new='''    /// Strict containment: canonical empty geometry returns false. Malformed
    /// closed-solid input throws GeometryError with code invalid_input, even
    /// when the point is outside its bounds or on a source triangle.
'''+old
assert s.count(old)==1;s=s.replace(old,new)
old='    #[wasm_bindgen(js_name = containsPointWithContext)]'
new='''    /// Containment with the certainty of both source validation and the query.
    /// Malformed input throws GeometryError; unresolved predicates keep Unknown.
'''+old
assert s.count(old)==1;s=s.replace(old,new);p.write_text(s)
changes=['crates/hypermesh/src/mesh.rs','src/solid.rs','src/wasm/mesh_js.rs','tests/containment_inputs.rs','tests/wasm/mesh_errors.cjs']
subprocess.run(['cargo','fmt','--all'],cwd=SOURCE,check=True)
changed={p for p,h in inputs.items() if sha(SOURCE/p)!=h}
assert changed<=set(changes)
for p in changes:inputs[p]=sha(SOURCE/p)
with gzip.open(OUT/'source-hashes.json.gz','wt') as f:json.dump(inputs,f,sort_keys=True)
dump(OUT/'result.json',{'complete':True,'runner_sha256':sha(__file__),'source':str(SOURCE),'source_inputs':len(inputs),'source_hashes_sha256':sha(OUT/'source-hashes.json.gz'),'changes':{p:inputs[p] for p in changes},'baseline_contract_sha256':sha(WORK/'prototype-v1/contract.json'),'baseline_test_format_only_difference':sha(BASE/'tests/containment_inputs.rs')!=sha(SOURCE/'tests/containment_inputs.rs'),'main_sources_modified':False})
print('Frozen candidate with shared source validation and receipt propagation',flush=True)
