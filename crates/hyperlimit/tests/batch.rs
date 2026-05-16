use hyperlimit::{
    Plane3, Point2, Point3, PredicatePolicy, classify_point_line, classify_point_line_batch,
    classify_point_oriented_plane, classify_point_oriented_plane_batch, classify_point_plane,
    classify_point_plane_batch, incircle2d, incircle2d_batch, insphere3d, insphere3d_batch,
    orient2d, orient2d_batch, orient3d, orient3d_batch,
};

type Real = hyperreal::Real;

fn real(value: f64) -> Real {
    Real::try_from(value).expect("finite test scalar")
}

fn p2(x: f64, y: f64) -> Point2 {
    Point2::new(real(x), real(y))
}

fn p3(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(real(x), real(y), real(z))
}

#[cfg(feature = "parallel")]
fn sp2(x: i128, y: i128) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

#[cfg(feature = "parallel")]
fn sp3(x: i128, y: i128, z: i128) -> Point3 {
    Point3::new(Real::from(x), Real::from(y), Real::from(z))
}

#[test]
fn sequential_batches_match_scalar_predicates() {
    let orient2_cases = vec![
        (p2(0.0, 0.0), p2(1.0, 0.0), p2(0.0, 1.0)),
        (p2(0.0, 0.0), p2(0.0, 1.0), p2(1.0, 0.0)),
    ];
    assert_eq!(
        orient2d_batch(&orient2_cases),
        orient2_cases
            .iter()
            .map(|(a, b, c)| orient2d(a, b, c))
            .collect::<Vec<_>>()
    );
    assert_eq!(
        classify_point_line_batch(&orient2_cases),
        orient2_cases
            .iter()
            .map(|(a, b, point)| classify_point_line(a, b, point))
            .collect::<Vec<_>>()
    );

    let orient3_cases = vec![
        (
            p3(0.0, 0.0, 0.0),
            p3(1.0, 0.0, 0.0),
            p3(0.0, 1.0, 0.0),
            p3(0.0, 0.0, 1.0),
        ),
        (
            p3(0.0, 0.0, 0.0),
            p3(0.0, 1.0, 0.0),
            p3(1.0, 0.0, 0.0),
            p3(0.0, 0.0, 1.0),
        ),
    ];
    assert_eq!(
        orient3d_batch(&orient3_cases),
        orient3_cases
            .iter()
            .map(|(a, b, c, d)| orient3d(a, b, c, d))
            .collect::<Vec<_>>()
    );
    assert_eq!(
        classify_point_oriented_plane_batch(&orient3_cases),
        orient3_cases
            .iter()
            .map(|(a, b, c, point)| classify_point_oriented_plane(a, b, c, point))
            .collect::<Vec<_>>()
    );

    let plane_cases = vec![
        (
            p3(0.0, 0.0, 3.0),
            Plane3::new(p3(0.0, 0.0, 1.0), real(-2.0)),
        ),
        (
            p3(0.0, 0.0, 1.0),
            Plane3::new(p3(0.0, 0.0, 1.0), real(-2.0)),
        ),
    ];
    assert_eq!(
        classify_point_plane_batch(&plane_cases),
        plane_cases
            .iter()
            .map(|(point, plane)| classify_point_plane(point, plane))
            .collect::<Vec<_>>()
    );

    let incircle_cases = vec![(p2(1.0, 0.0), p2(0.0, 1.0), p2(-1.0, 0.0), p2(0.0, 0.0))];
    assert_eq!(
        incircle2d_batch(&incircle_cases),
        incircle_cases
            .iter()
            .map(|(a, b, c, d)| incircle2d(a, b, c, d))
            .collect::<Vec<_>>()
    );

    let insphere_cases = vec![(
        p3(1.0, 0.0, 0.0),
        p3(-1.0, 0.0, 0.0),
        p3(0.0, 1.0, 0.0),
        p3(0.0, 0.0, 1.0),
        p3(0.0, 0.0, 0.0),
    )];
    assert_eq!(
        insphere3d_batch(&insphere_cases),
        insphere_cases
            .iter()
            .map(|(a, b, c, d, e)| insphere3d(a, b, c, d, e))
            .collect::<Vec<_>>()
    );
}

#[test]
fn batch_policy_is_applied_to_each_case() {
    let cases = vec![(p2(0.0, 0.0), p2(1.0, 0.0), p2(0.0, 1.0))];

    assert_eq!(
        hyperlimit::orient2d_batch_with_policy(&cases, PredicatePolicy::STRICT),
        vec![hyperlimit::orient::orient2d_with_policy(
            &cases[0].0,
            &cases[0].1,
            &cases[0].2,
            PredicatePolicy::STRICT,
        )]
    );
}

#[cfg(feature = "parallel")]
#[test]
fn parallel_batches_match_sequential_batches() {
    let orient2_cases = (0..2048)
        .map(|i| {
            let x = i as i128 - 1024;
            let eps = if i % 2 == 0 { 1 } else { -1 };
            (sp2(0, 0), sp2(2048, 2048), sp2(x, x + eps))
        })
        .collect::<Vec<_>>();
    assert_eq!(
        orient2d_batch(&orient2_cases),
        hyperlimit::orient2d_batch_parallel(&orient2_cases)
    );

    let orient3_cases = (0..2048)
        .map(|i| {
            let x = i as i128 - 1024;
            let z = if i % 2 == 0 { 1 } else { -1 };
            (sp3(0, 0, 0), sp3(1, 0, 0), sp3(0, 1, 0), sp3(x, -x, z))
        })
        .collect::<Vec<_>>();
    assert_eq!(
        orient3d_batch(&orient3_cases),
        hyperlimit::orient3d_batch_parallel(&orient3_cases)
    );
}
