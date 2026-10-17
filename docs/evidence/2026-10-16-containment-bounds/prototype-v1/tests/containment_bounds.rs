//! Certified strict containment near all supporting planes, without tolerances.
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
