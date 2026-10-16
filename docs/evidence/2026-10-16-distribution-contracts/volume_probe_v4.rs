use csgrs::{
    GeometryContext, Real, TriangleMesh,
    errors::ValidationError,
    solid::{self, SolidExt},
};
use hypermesh::Vector3;

trait Complete {
    fn complete(self) -> TriangleMesh;
}
impl Complete for TriangleMesh {
    fn complete(self) -> TriangleMesh {
        self
    }
}
impl Complete for Result<TriangleMesh, ValidationError> {
    fn complete(self) -> TriangleMesh {
        self.unwrap()
    }
}
fn r(n: i32) -> Real {
    n.into()
}
fn volume(mesh: &TriangleMesh, count: i32, label: &str) {
    let six = mesh.triangles.iter().fold(r(0), |sum, t| {
        let [a, b, c] = t.indices().map(|i| mesh.positions[i].to_vector());
        sum + a.dot(&b.cross(&c))
    });
    let expected = r(6 * count);
    let before = six.certified_eq_until(&expected, -1024).as_bool();
    let normal = six.exact_rational_normal_form();
    let delta = (&six - &expected).exact_rational_normal_form();
    println!(
        "{label}: equality_before={before:?} normal={normal:?} difference_normal={delta:?} equality_after={:?}",
        six.certified_eq_until(&expected, -1024).as_bool()
    );
    // Divergence theorem with F=(0,0,z): V = integral_boundary z*n_z dA.
    // The affine face average is (a.z+b.z+c.z)/3 and oriented area is cross/2.
    // This uses every oriented triangle, without global-origin cubic products.
    let local = mesh.triangles.iter().enumerate().fold(r(0), |sum, (index, t)| {
        let [a, b, c] = t.indices().map(|i| &mesh.positions[i]);
        let term = (&a.z + &b.z + &c.z) * &(b - a).cross(&(c - a)).0[2];
        let normal = term.exact_rational_normal_form();
        assert!(normal.is_some(), "{label} triangle {index}: rational={normal:?}, quadratic={:?}", term.exact_quadratic_normal_form());
        sum + Real::from(normal.unwrap())
    });
    println!("{label}: per-face divergence equality={:?} normal={:?}",
        local.certified_eq_until(&expected, -1024).as_bool(),
        local.exact_rational_normal_form());
    assert_eq!(local.certified_eq_until(&expected, -1024).as_bool(), Some(true));
    assert_eq!(local.exact_rational_normal_form(), expected.exact_rational());
}
fn main() {
    let cube = solid::cube(r(1)).unwrap();
    for (name, count, mesh) in [
        (
            "linear-8",
            8,
            solid::distribute_linear(&cube, 8, Vector3::x(), r(2)).complete(),
        ),
        (
            "grid-4x4",
            16,
            solid::distribute_grid(&cube, 4, 4, r(2), r(2)).complete(),
        ),
        (
            "arc-12",
            12,
            solid::distribute_arc(&cube, 12, r(10), r(0), r(330)).complete(),
        ),
    ] {
        volume(&mesh, count, name);
        assert!(
            mesh.is_closed_manifold_geometry(&GeometryContext::STRICT.mesh_context())
                .unwrap()
                .value
        );
        let accumulated = solid::empty().try_union(&mesh).unwrap();
        volume(&accumulated, count, &format!("{name}-after-empty-union"));
        let removed = accumulated.try_difference(&mesh).unwrap();
        assert!(removed.positions.is_empty() && removed.triangles.is_empty());
        println!("{name}: strict topology, empty union and complete self subtraction pass");
    }
}
