use csgrs::adapter::TriangleMeshF64;
use csgrs::{
    GeometryCertainty, GeometryContext, Real,
    solid::{self, SolidExt},
};
use hyperlattice::Point3;

fn main() {
    let body = TriangleMeshF64::cuboid(12.0, 12.0, 4.0).unwrap();
    let drill = TriangleMeshF64::cylinder(2.0, 6.0, 16)
        .unwrap()
        .translate(6.0, 6.0, -1.0)
        .unwrap();
    assert!(body.difference(&drill).unwrap().native().is_closed_manifold());
    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
        let result = body.difference_with_context(&drill, &context).unwrap();
        assert_eq!(result.certainty, GeometryCertainty::Certified);
        let topology = result
            .value
            .native()
            .is_closed_manifold_geometry(&GeometryContext::STRICT.mesh_context())
            .unwrap();
        assert!(topology.value);
        for (coordinates, expected) in
            [([1, 1, 2], true), ([6, 6, 2], false), ([-1, 6, 2], false)]
        {
            let [x, y, z] = coordinates.map(Real::from);
            let membership = solid::contains_point_with_context(
                result.value.native(),
                &Point3::new(x, y, z),
                &GeometryContext::STRICT,
            )
            .unwrap();
            assert_eq!(membership.certainty, GeometryCertainty::Certified);
            assert_eq!(membership.value, expected);
        }
    }
    println!(
        "original 16-segment adapter drill: certified under both policies, manifold and independent memberships"
    );
    let cube = solid::cube(Real::one()).unwrap();
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let unknown_zero = &sine * &sine + &cosine * &cosine - Real::one();
    let shifted = cube
        .translated(unknown_zero, Real::zero(), Real::zero())
        .unwrap();
    let a = TriangleMeshF64::from_native(cube);
    let b = TriangleMeshF64::from_native(shifted);
    assert!(
        a.difference_with_context(&b, &GeometryContext::STRICT)
            .is_err()
    );
    let approximate = a
        .difference_with_context(&b, &GeometryContext::APPROXIMATE_512)
        .unwrap();
    assert_eq!(
        approximate.certainty,
        GeometryCertainty::Approximate512Consumed
    );
    assert!(approximate.value.native().positions.is_empty());
    assert!(approximate.value.native().triangles.is_empty());
    assert!(
        a.difference_with_context(&b, &GeometryContext::STRICT)
            .is_err()
    );
    println!(
        "adapter terminal equality: strict error, qualified empty, strict error after reuse"
    );
}
