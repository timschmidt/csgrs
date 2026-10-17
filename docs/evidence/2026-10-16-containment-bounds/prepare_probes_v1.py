"""Preregister a certified supporting-bound shortcut against the correct E32 baseline."""
import hashlib,json,shutil
from pathlib import Path
W=Path(__file__).resolve().parent;ROOT=W.parents[2];P=W/'prototype-v1';P.mkdir(exist_ok=False)
(P/'tests/wasm').mkdir(parents=True)
(P/'contract-v1.md').write_text('''# Certified bounds before surface arrangement

Base revision: `311d04fbbe5635a29fe7e754b838cccc4b8add53`.

The owning layer is native Hypermesh containment. P4/P6/P8/P9/P10/P17/P18/
P22/P24/P25/P30/P36 apply. The public operation remains strict interior of
its regularized nonzero-winding set. Canonical empty, malformed-source,
qualified-source, symbolic-coordinate and internal-source-face behavior stays
in scope. Source validation must precede every bounds shortcut.

A finite closed PWN source has winding zero outside its position AABB. A point
on a certified supporting plane of that AABB has arbitrarily close exterior
points, hence cannot be strict interior. This proof needs source validity,
bounds and coordinate equality to be Certified. An approximate equality or
already-qualified dependency does not enable the new shortcut. Existing
outside-bounds behavior, certainty receipts and fallback algorithms remain.
No new retained cache, geometry conversion, representation or API is added.

Before implementation, rerun independent correctness controls and current
cost probes using the qualified E32 production libraries. Verify that every
current runtime input matches that snapshot; admission metadata is the only
permitted baseline difference. Reuse both existing allocation probes unchanged.
Both sides of the final comparison must complete the same Certified results.

Acceptance preserves the entire E32 strict-success corpus and adds all six
supporting planes with exact 1/10^12 offsets, retained pi translation, bounds
that contain points outside the actual tetrahedron, mutated invalid sources
and policy-qualified query coordinates. Real WASM clients exercise exact
binary64 near-boundary inputs. No epsilon, fixture reduction, new Unknown for
an old Certified result, or weaker validation is acceptable. Newly sufficient
proofs may strengthen an undecided query only with a certified non-interior
witness; such a gain is recorded separately from timing comparisons.

Use five alternating fresh-process pairs for the unchanged 21-case matrix and
tetrahedron control, with all raw observations retained. Require all candidate
queries to remain correct/Certified, a reduction of at least half the warm
allocation calls on each global-boundary matrix case, and reduced allocation
work in the tetrahedron control. Inspect all other rows and preserve adverse
samples; an isolated favorable latency is insufficient. Instrumented timing
is not an unqualified production speed claim. Preserve the geometry-output
and serialization contracts; this changes proof search, not represented sets.

Run full affected native/minimal suites, real WASM/TypeScript consumers,
Clippy, docs and formatting after the final edit. This does not complete the
full properties, historical artifact audit, release, workflow or coverage
requirements. No total-work, heap-byte, expression-growth or cancellation bound
is added; externally contain validation and report resource observations.
''')
(P/'tests/containment_bounds.rs').write_text(r'''//! Certified strict containment near all supporting planes, without tolerances.
use std::sync::Arc;
use csgrs::{GeometryCertainty, GeometryContext, Real, TriangleMesh, solid};
use csgrs::solid::SolidExt;
use hypermesh::{HypermeshError, MeshCertainty, Point3, Triangle};

fn p(x: i32, y: i32, z: i32) -> Point3 { Point3::new(x.into(), y.into(), z.into()) }
fn query(values: [Real;3]) -> Point3 { let [x,y,z]=values; Point3::new(x,y,z) }
fn tetrahedron() -> TriangleMesh {
    TriangleMesh::new(vec![p(0,0,0),p(1,0,0),p(0,1,0),p(0,0,1)],vec![Triangle::new(0,2,1),Triangle::new(0,1,3),Triangle::new(0,3,2),Triangle::new(1,2,3)])
}
fn check(mesh: &TriangleMesh, point: &Point3, expected: bool) {
    for context in [GeometryContext::STRICT,GeometryContext::APPROXIMATE_512,GeometryContext::STRICT] {
        let native=mesh.contains_point(&context.mesh_context(),point).unwrap();
        assert_eq!(native.certainty,MeshCertainty::Certified);
        assert_eq!(native.value,expected);
        let modeled=solid::contains_point_with_context(mesh,point,&context).unwrap();
        assert_eq!(modeled.certainty,GeometryCertainty::Certified);
        assert_eq!(modeled.value,expected);
    }
}

#[test]
fn all_six_supporting_planes_preserve_exact_near_boundary_membership() {
    let epsilon=(Real::one()/Real::from(1_000_000_000_000_u64)).unwrap();
    let cube=solid::cube(Real::from(2)).unwrap();
    for shift in [Real::zero(),Real::pi()] {
        let mesh=cube.translated(shift.clone(),Real::zero(),Real::zero()).unwrap();
        for axis in 0..3 {
            for upper in [false,true] {
                for sign in [-1,0,1] {
                    let mut xyz=[Real::one(),Real::one(),Real::one()];
                    let offset=Real::from(sign)*epsilon.clone();
                    xyz[axis]=if upper {Real::from(2)-offset} else {offset};
                    xyz[0]=xyz[0].clone()+shift.clone();
                    check(&mesh,&query(xyz),sign>0);
                }
            }
        }
    }
}

#[test]
fn bounding_box_interior_does_not_imply_solid_interior() {
    let mut positions=tetrahedron().positions.to_vec();
    positions.push(p(2,2,2)); // Unused positions enlarge the bound, not the set.
    let mesh=TriangleMesh::new(positions,tetrahedron().triangles.to_vec());
    let eighth=(Real::one()/Real::from(8)).unwrap();
    let half=(Real::one()/Real::from(2)).unwrap();
    check(&mesh,&Point3::new(eighth.clone(),eighth.clone(),eighth.clone()),true);
    check(&mesh,&Point3::new(half.clone(),half.clone(),half),false);
    check(&mesh,&Point3::new(Real::zero(),eighth.clone(),eighth.clone()),false);
    check(&mesh,&Point3::new(Real::from(2),eighth.clone(),eighth),false);
}

#[test]
fn warmed_bounds_never_hide_mutated_invalid_triangles() {
    let mesh=solid::cube(Real::from(2)).unwrap();
    check(&mesh,&p(0,1,1),false);
    let mut changed=mesh.clone();
    Arc::make_mut(&mut changed.triangles)[0]=Triangle::new(0,0,0);
    for axis in 0..3 {
        for value in [0,2] {
            let mut xyz=[Real::one(),Real::one(),Real::one()];xyz[axis]=Real::from(value);
            for context in [GeometryContext::STRICT,GeometryContext::APPROXIMATE_512,GeometryContext::STRICT] {
                assert_eq!(changed.contains_point(&context.mesh_context(),&query(xyz.clone())),Err(HypermeshError::DegenerateTriangle{mesh_index:0,triangle_index:0}));
            }
        }
    }
    check(&mesh,&p(1,1,1),true);
}

#[test]
fn approximate_boundary_equalities_keep_their_receipt() {
    let mesh=solid::cube(Real::from(2)).unwrap();
    let sine=Real::e().sin();let cosine=Real::e().cos();
    let zero=&sine*&sine+&cosine*&cosine-Real::one();
    // The independent trigonometric identity places both points on a face;
    // this expression deliberately requires the policy terminal today.
    for x in [zero.clone(),Real::from(2)-zero] {
        let point=Point3::new(x,Real::one(),Real::one());
        for _ in 0..2 {
            assert!(matches!(solid::contains_point(&mesh,&point),Err(HypermeshError::PredicateUndecided{..})));
            let outcome=solid::contains_point_with_context(&mesh,&point,&GeometryContext::APPROXIMATE_512).unwrap();
            assert!(!outcome.value);
            assert_eq!(outcome.certainty,GeometryCertainty::Approximate512Consumed);
        }
    }
}
''')
p=P/'tests/wasm/containment_winding.cjs';shutil.copy2(ROOT/'tests/wasm/containment_winding.cjs',p)
p.write_text(p.read_text()+r'''

test('all six supporting planes preserve nearby binary64 membership',()=>{
  const mesh=MeshJs.cube(2);
  try {
    for (let axis=0;axis<3;++axis) for (const upper of [false,true]) for (const sign of [-1,0,1]) {
      const xyz=[1,1,1],edge=upper?2:0;
      xyz[axis]=upper?2-sign*1e-12:sign*1e-12;
      assert.equal(xyz[axis]===edge,sign===0);
      const point=new Point3Js(...xyz);
      try {
        assert.equal(mesh.containsPoint(point),sign>0);
        for (const approximate of [false,true,false]) {
          const result=mesh.containsPointWithContext(point,approximate);
          try {
            assert.equal(result.status,GeometryQueryStatusJs.Decided);
            assert.equal(result.certainty,GeometryCertaintyJs.Certified);
            assert.equal(result.value,sign>0);
          } finally { result.free(); }
        }
      } finally { point.free(); }
    }
  } finally { mesh.free(); }
});
''')
for scope in ['control','matrix']:shutil.copy2(W.parent/'containment-winding'/(scope+'_cost_v1.rs'),W/(scope+'_cost_v1.rs'))
contract={'base_commit':'311d04fbbe5635a29fe7e754b838cccc4b8add53','benchmark_source':'Unchanged E32 control and 21-case matrix, now comparing two correct implementations.','probe_sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in W.glob('*_cost_v1.rs')},'pairs':5,'targeted_matrix_cases':['simple_face','simple_vertex','opposite_face','symbolic_face','symbolic_vertex','flat_face'],'required_allocation_call_reduction_on_each_warm_targeted_row':0.5,'control_requirement':'Fewer allocation calls and bytes in every cold/warm policy pair; no wrong/undecided/qualified replacement of Certified success.','all_other_rows':'Retain and inspect every raw sample and all allocation, retained/peak bytes, source-drop, process RSS, timing and artifact-size observations.','resource_limits':'Externally bounded validation; no internal total-work, heap-byte, expression-growth or cancellation claim.'}
(W/'cost-contract-v1.json').write_text(json.dumps(contract,indent=2)+'\n')
