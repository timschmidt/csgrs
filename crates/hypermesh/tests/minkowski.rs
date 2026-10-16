use hypermesh::{
    HypermeshError, MeshCertainty, MeshContext, MinkowskiOptions, Point3, PredicatePolicy, Real,
    Triangle, TriangleMesh, minkowski_hull, minkowski_sum, minkowski_sum_with_options,
};

fn r(n: i32) -> Real {
    n.into()
}
fn p(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}
fn tetrahedron() -> TriangleMesh {
    TriangleMesh::new(
        vec![p(0, 0, 0), p(2, 0, 0), p(0, 2, 0), p(0, 0, 2)],
        vec![
            Triangle::new(0, 2, 1),
            Triangle::new(0, 1, 3),
            Triangle::new(0, 3, 2),
            Triangle::new(1, 2, 3),
        ],
    )
}
fn volume(mesh: &TriangleMesh, six_expected: i32) {
    let six = mesh.triangles.iter().fold(r(0), |s, t| {
        let [a, b, c] = t.indices().map(|i| mesh.positions[i].to_vector());
        s + a.dot(&b.cross(&c))
    });
    assert_eq!(
        six.exact_rational_normal_form(),
        r(six_expected).exact_rational_normal_form()
    );
}

#[test]
fn native_entry_points_certify_sums_and_reject_invalid_sources() {
    let tetra = tetrahedron();
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        let context = MeshContext::new(policy);
        // 2T is a tetrahedron of intercept four: volume 4³/6.
        let sum = minkowski_sum(&context, &tetra, &tetra).unwrap();
        assert_eq!(sum.certainty, MeshCertainty::Certified);
        volume(&sum.value, 64);
        sum.value.validate_closed_pwn(&context).unwrap();
        let inverted = minkowski_sum(&context, &tetra.reversed_winding(), &tetra).unwrap();
        volume(&inverted.value, 64);
        let mut unused = tetra.positions.to_vec();
        unused.push(p(50, 50, 50));
        let unused = TriangleMesh::new(unused, tetra.triangles.to_vec());
        volume(&minkowski_sum(&context, &unused, &tetra).unwrap().value, 64);
        let empty = TriangleMesh::new(vec![], vec![]);
        let invalid = TriangleMesh::new(vec![], vec![Triangle::new(0, 1, 2)]);
        assert!(matches!(
            minkowski_sum(&context, &empty, &invalid),
            Err(HypermeshError::VertexIndexOutOfBounds { .. })
        ));
        assert!(matches!(
            minkowski_sum_with_options(
                &context,
                &tetra,
                &tetra,
                &MinkowskiOptions {
                    max_pair_points: 15,
                    ..Default::default()
                }
            ),
            Err(HypermeshError::ResourceLimitExceeded { required: 16, .. })
        ));
        volume(
            &minkowski_hull(&context, &tetra.positions, &tetra.positions)
                .unwrap()
                .value,
            64,
        );
    }
}

#[test]
fn closed_zero_winding_and_lower_dimensional_boundaries_regularize_to_empty() {
    let context = MeshContext::new(PredicatePolicy::STRICT);
    let tetra = tetrahedron();
    let cancelled =
        TriangleMesh::try_concatenate(&[tetra.clone(), tetra.reversed_winding()]).unwrap();
    let flat = TriangleMesh::new(
        vec![p(0, 0, 0), p(2, 0, 0), p(0, 2, 0)],
        vec![Triangle::new(0, 1, 2), Triangle::new(0, 2, 1)],
    );
    for source in [cancelled, flat] {
        source.validate_closed_pwn(&context).unwrap();
        let sum = minkowski_sum(&context, &source, &tetra).unwrap();
        assert_eq!(sum.certainty, MeshCertainty::Certified);
        assert!(sum.value.positions.is_empty() && sum.value.triangles.is_empty());
    }
}
