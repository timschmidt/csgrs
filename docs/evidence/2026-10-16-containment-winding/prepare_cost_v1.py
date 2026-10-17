"""Preregister retained comparable controls and the expanded completion matrix."""
import hashlib,json,shutil
from pathlib import Path
w=Path(__file__).resolve().parent
old=w.parent/'containment-input-validation/cost_v1.rs'
shutil.copy2(old,w/'control_cost_v1.rs')
allocator=old.read_text().split('fn p(')[0]
probe=(w/'prototype-v4/tests/containment_winding.rs').read_text()
helpers=probe[probe.index('#[derive(Clone, Copy)]'):probe.index('fn check_queries(')]
s=allocator.replace('//! Focused source-validation cost probe, independently checked tetrahedron queries.','//! Completed-contract cost observations for independently specified regularized sets.')+helpers+r'''

struct Case {
    name: &'static str,
    boxes: Vec<BoxInput>,
    twice: [i32;3],
    pi_translation: bool,
    flat: bool,
}
impl Case {
    fn point(&self) -> Point3 {
        let mut p = point(self.twice);
        if self.pi_translation { p.x = p.x + Real::pi(); }
        p
    }
    fn source(&self) -> TriangleMesh {
        let input = if self.flat {
            TriangleMesh::new(vec![point([0,0,0]),point([4,0,0]),point([0,4,0])],vec![Triangle::new(0,1,2),Triangle::new(2,1,0)])
        } else { mesh(&self.boxes) };
        if self.pi_translation {
            TriangleMesh::new(input.positions.iter().map(|p|Point3::new(p.x.clone()+Real::pi(),p.y.clone(),p.z.clone())).collect(),input.triangles.to_vec())
        } else { input }
    }
    fn expected(&self) -> bool { !self.flat && interior(&self.boxes,self.twice) }
}
fn cases() -> Vec<Case> {
    let mut out = Vec::new();
    let unit=box_input([0,0,0],[2,2,2],1);
    let outer=box_input([0,0,0],[3,3,3],1);
    let inner=box_input([1,1,1],[2,2,2],1);
    for (name,boxes,twice) in [
        ("simple_inside",vec![unit],[1,1,1]),
        ("simple_face",vec![unit],[0,1,1]),
        ("simple_vertex",vec![unit],[0,0,0]),
        ("simple_outside",vec![unit],[5,1,1]),
        ("nested_inside",vec![outer,inner],[3,3,3]),
        ("nested_internal_face",vec![outer,inner],[2,3,3]),
        ("nested_internal_edge",vec![outer,inner],[2,2,3]),
        ("nested_internal_vertex",vec![outer,inner],[2,2,2]),
        ("opposite_inside",vec![unit,box_input(unit.min,unit.max,-1)],[1,1,1]),
        ("opposite_face",vec![unit,box_input(unit.min,unit.max,-1)],[0,1,1]),
        ("overlap_inside",vec![unit,box_input([1,0,0],[3,2,2],1)],[3,1,1]),
        ("shared_face",vec![box_input([0,0,0],[1,2,2],1),box_input([1,0,0],[2,2,2],1)],[2,1,1]),
        ("cavity_inside",vec![outer,box_input(inner.min,inner.max,-1)],[3,3,3]),
        ("cavity_face",vec![outer,box_input(inner.min,inner.max,-1)],[2,3,3]),
        ("tangent_origin",vec![box_input([16,6,4],[17,7,5],1),box_input([-2,-2,-2],[-1,-1,-1],1)],[0,0,0]),
        ("reversed_multiplicity",vec![box_input(unit.min,unit.max,-1),box_input(unit.min,unit.max,-1)],[1,1,1]),
    ] { out.push(Case{name,boxes,twice,pi_translation:false,flat:false}); }
    for (name,twice) in [("symbolic_inside",[1,1,1]),("symbolic_face",[0,1,1]),("symbolic_vertex",[0,0,0]),("symbolic_outside",[5,1,1])] {
        out.push(Case{name,boxes:vec![unit],twice,pi_translation:true,flat:false});
    }
    out.push(Case{name:"flat_face",boxes:Vec::new(),twice:[1,1,0],pi_translation:false,flat:true});
    out
}
fn observation(mesh: &TriangleMesh, context: &GeometryContext, query: &Point3, expected: bool) -> String {
    match mesh.contains_point(context,query) {
        Ok(outcome) => {
            let correct=outcome.value==expected;
            let status=match (correct,outcome.certainty) {
                (true,GeometryCertainty::Certified)=>"certified_completion",
                (true,_)=>"qualified_approximate_completion",
                (false,_)=>"incorrect_result",
            };
            format!("{{\"status\":\"{status}\",\"value\":{},\"certainty\":\"{:?}\"}}",outcome.value,outcome.certainty)
        }
        Err(error) => {
            let status=if matches!(error,hypermesh::HypermeshError::PredicateUndecided{..}) {"undecided"} else {"typed_failure"};
            // Debug descriptions contain quotes; serialize their bytes by Rust
            // debug-string formatting, which is JSON-compatible for these errors.
            format!("{{\"status\":\"{status}\",\"error\":{:?}}}",format!("{error:?}"))
        }
    }
}
fn main() {
    let specs=cases();
    let mut rows=Vec::new();
    for (policy,context) in [("strict",GeometryContext::new(PredicatePolicy::STRICT)),("approximate",GeometryContext::new(PredicatePolicy::APPROXIMATE_512))] {
        for case in &specs {
            let expected=case.expected();
            let query=case.point();
            let mut meshes=Vec::new();
            for phase in ["cold_construct_query","warm_query"] {
                let before_live=LIVE.load(Relaxed);PEAK.store(before_live,Relaxed);
                let before_calls=CALLS.load(Relaxed);let before_bytes=BYTES.load(Relaxed);
                let start=Instant::now();let mut observations=Vec::new();
                if phase=="cold_construct_query" {
                    for _ in 0..8 { let source=case.source();observations.push(observation(&source,&context,&query,expected));meshes.push(source); }
                } else {
                    for source in &meshes { observations.push(observation(source,&context,&query,expected)); }
                }
                let elapsed=start.elapsed().as_nanos();
                let calls=CALLS.load(Relaxed)-before_calls;let bytes=BYTES.load(Relaxed)-before_bytes;
                let retained=LIVE.load(Relaxed)-before_live;let peak=PEAK.load(Relaxed)-before_live;
                rows.push(format!("{{\"case\":\"{}\",\"policy\":\"{policy}\",\"phase\":\"{phase}\",\"expected\":{expected},\"attempted\":8,\"elapsed_ns\":{elapsed},\"allocation_calls\":{calls},\"allocated_bytes\":{bytes},\"retained_requested_bytes_delta\":{retained},\"peak_extra_requested_bytes\":{peak},\"observations\":[{}]}}",case.name,observations.join(",")));
            }
            let before=LIVE.load(Relaxed);drop(meshes);let released=before-LIVE.load(Relaxed);
            rows.push(format!("{{\"case\":\"{}\",\"policy\":\"{policy}\",\"phase\":\"drop_meshes\",\"released_requested_bytes\":{released}}}",case.name));
        }
    }
    println!("[{}]",rows.join(","));
}
'''
(w/'matrix_cost_v1.rs').write_text(s)
contract={'purpose':'Correctness migration with full cost disclosure; no optimization claim.','control_source':str(old),'control_sha256':hashlib.sha256(old.read_bytes()).hexdigest(),'variants':'Qualified E31 current production Hypermesh and corrected candidate production Hypermesh, with identical Cargo dependency/feature fingerprints. Completed targets are read-only.','pairs':5,'process_order':'baseline/candidate alternating by pair, separately for unchanged control and expanded matrix','control_queries':256,'matrix_cases':21,'matrix_queries_per_process':672,'policies':['STRICT','APPROXIMATE_512'],'cache_conditions':'Each process is fresh. Cold phases construct 8 new meshes per matrix case (16 for control); warm phases reuse these mesh identities. Process scalar caches and earlier cases are not reset. No new retained cache is introduced.','timing':'Cold construction, complete query, certainty/value/error materialization and retained source vector; warm complete query and result materialization. Query and independent expected-value construction precede timing. Atomic allocator instrumentation is included in both variants.','metrics':['completion category and certainty','wall nanoseconds','allocation/reallocation calls and requested bytes','retained requested bytes','peak extra requested bytes','released requested bytes at source drop','native probe and WASM artifact sizes','process peak RSS from time -v'],'comparison':'Compare latency only when both observations complete the same expected value Certified. Keep wrong or typed-failure baselines, all individual pairs and unfavorable results; they are not faster successful completion.','limits':'Per-command external elapsed 180s and virtual-address 8GiB; no internal total-work, heap-byte, cancellation or expression-growth bound is claimed.'}
(w/'cost-contract-v1.json').write_text(json.dumps(contract,indent=2)+'\n')
print('Preregistered control and 21-case completed-contract cost matrix',flush=True)
