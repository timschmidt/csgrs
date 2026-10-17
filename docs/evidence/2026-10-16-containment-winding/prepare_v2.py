"""Freeze an exact zero-transition allowance for a single bundled radial ray."""
import gzip,hashlib,json,shutil,subprocess
from pathlib import Path
W=Path(__file__).resolve().parent; ROOT=W.parents[2]
S=W/'candidate-source-v2'; O=W/'candidate-preparation-v2'
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def dump(p,x):Path(p).write_text(json.dumps(x,indent=2)+'\n')
assert json.loads((W/'baseline-v6/result.json').read_text())['complete']
with gzip.open(W/'candidate-preparation-v1/source-hashes.json.gz','rt') as f:inputs=json.load(f)
assert all(sha(W/'candidate-source-v1'/p)==h for p,h in inputs.items())
S.mkdir(exist_ok=False);O.mkdir(exist_ok=False)
for p,h in inputs.items():
 out=S/p;out.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(W/'candidate-source-v1'/p,out);assert sha(out)==h
p=S/'tests/containment_winding.rs';shutil.copy2(W/'prototype-v4/tests/containment_winding.rs',p)
p=S/'crates/hypermesh/src/surface_arrangement.rs';s=p.read_text()
old='''        let uses = &edge_uses[edge_start..edge_end];
        if let [first, second] = uses {'''
new='''        let uses = &edge_uses[edge_start..edge_end];
        // A bundled sheet can end at an edge without bounding any volume:
        // opposite copies of one flat triangle are a closed PWN source. Uses
        // are sorted by facet, so matching endpoints prove one geometric ray.
        // Admit its cyclic zero-angle strata only when the exact aggregate
        // transition cancels in every operand. An open or cross-operand sheet
        // must still fail the ordinary radial-topology checks.
        let transition_start = uses[0].facet as usize * operand_count;
        let cancelled_bundle = uses.len() > 1
            && uses[0].facet == uses[uses.len() - 1].facet
            && _transitions[transition_start..transition_start + operand_count]
                .iter()
                .all(|&delta| delta == 0);
        if let [first, second] = uses {'''
assert s.count(old)==1;s=s.replace(old,new)
old='''                edge,
                first,
                second,''';assert s.count(old)==1;s=s.replace(old,'''                edge,
                cancelled_bundle,
                first,
                second,''')
old='''                edge,
                uses,
                &mut radial,''';assert s.count(old)==1;s=s.replace(old,'''                edge,
                cancelled_bundle,
                uses,
                &mut radial,''')
old='''    edge: [u32; 2],
    first: &EdgeUse,
    second: &EdgeUse,''';assert s.count(old)==1;s=s.replace(old,'''    edge: [u32; 2],
    cancelled_bundle: bool,
    first: &EdgeUse,
    second: &EdgeUse,''')
old='''    if !retained_separation
        && same_radial_ray''';assert s.count(old)==1;s=s.replace(old,'''    if !retained_separation
        && !cancelled_bundle
        && same_radial_ray''')
start=s.index('fn assemble_radial_ring(');end=s.index('fn try_assemble_two_face_transverse_ring(',start)
part=s[start:end].replace('    edge: [u32; 2],\n    uses:', '    edge: [u32; 2],\n    cancelled_bundle: bool,\n    uses:')
assert part.count('if distinct_rays < 2 {')==1
part=part.replace('if distinct_rays < 2 {','if distinct_rays < 2 && !cancelled_bundle {')
s=s[:start]+part+s[end:]
old='''                                assemble_radial_ring(
                                    &decisions,
                                    points,
                                    &facets,
                                    [0, 1],
                                    &uses,''';assert s.count(old)==1
s=s.replace(old,old.replace('                                    &uses,','                                    false,\n                                    &uses,'))
marker='''    #[test]
    fn open_surface_is_rejected_by_exact_radial_topology() {'''
test='''    #[test]
    fn one_ray_requires_cancellation_in_each_operand() {
        for copies in [1, 2] {
            for cross_operand in [false, true] {
                let mut polygons = Vec::new();
                for copy in 0..copies {
                    let mut first = triangle(
                        [p(0, 0, 0), p(4, 0, 0), p(0, 4, 0)],
                        0,
                        copy * 2,
                        [0, 1, 2],
                    );
                    first.delta_w = vec![1, 0];
                    let mut second = if cross_operand {
                        triangle(
                            [p(0, 4, 0), p(4, 0, 0), p(0, 0, 0)],
                            1,
                            copy * 2 + 1,
                            [2, 1, 0],
                        )
                    } else {
                        triangle(
                            [p(0, 0, 0), p(4, 0, 0), p(0, 4, 0)],
                            0,
                            copy * 2 + 1,
                            [0, 1, 2],
                        )
                    };
                    second.delta_w = if cross_operand { vec![0, 1] } else { vec![1, 0] };
                    polygons.extend([first, second]);
                }
                for policy in [
                    hyperlimit::PredicatePolicy::STRICT,
                    hyperlimit::PredicatePolicy::APPROXIMATE_512,
                ] {
                    let context = MeshContext::new(policy);
                    let decisions = DecisionContext::new(&context);
                    assert_eq!(
                        arrange_surface(&decisions, &polygons).unwrap_err(),
                        HypermeshError::SurfaceArrangementFailed {
                            reason: if copies == 1 {
                                "two-facet radial edge has one geometric ray"
                            } else {
                                "surface radial edge has fewer than two geometric rays"
                            },
                        }
                    );
                    assert_eq!(decisions.certainty(), MeshCertainty::Certified);
                }
            }
        }
    }

'''
assert s.count(marker)==1;s=s.replace(marker,test+marker);p.write_text(s)
subprocess.run(['cargo','fmt','--all'],cwd=S,check=True)
changes=list(json.loads((W/'candidate-preparation-v1/result.json').read_text())['changes'])+['crates/hypermesh/src/surface_arrangement.rs']
assert {p for p,h in inputs.items() if sha(S/p)!=h}<=set(changes)
for p in changes:inputs[p]=sha(S/p)
with gzip.open(O/'source-hashes.json.gz','wt') as f:json.dump(inputs,f,sort_keys=True)
dump(O/'result.json',{'complete':True,'base_commit':'d4eee9a675fa51d84eb0eb29503dd802bdcdbd19','runner_sha256':sha(__file__),'source':str(S),'source_inputs':len(inputs),'source_hashes_sha256':sha(O/'source-hashes.json.gz'),'changes':{p:inputs[p] for p in changes},'contract_sha256':sha(W/'prototype-v4/contract-v1.md'),'main_sources_modified':False,'prior_attempt':'candidate-source-v1 retained unchanged, flat-source control failed','arrangement_proof':'Only one canonical facet bundle and an exact zero transition for every operand admit a single-ray edge. Existing cyclic sheet strata, absolute winding, output classification, and open-source failures stay in force.'})
for name in ['validation_common','candidate','wasm']:
 p=W/(name+'_v1.py');s=p.read_text().replace('_v1','_v2').replace('-v1','-v2')
 if name=='validation_common':
  # Previously qualified tool versions remain read-only, not renamed.
  s=s.replace("OLD/'wasm-tools-v2/result.json'","OLD/'wasm-tools-v1/result.json'")
  s=s.replace("OLD/'wasm-source-v2/tests/wasm/node_modules/typescript'","OLD/'wasm-source-v1/tests/wasm/node_modules/typescript'")
 (W/(name+'_v2.py')).write_text(s)
print('Frozen candidate v2 with retained flat, mixed-set, and symbolic controls',flush=True)
