use hyperlimit::{
    Plane3, Point2, Point3, PredicatePolicy, classify_point_line, classify_point_line_batch,
    classify_point_oriented_plane, classify_point_oriented_plane_batch, classify_point_plane,
    classify_point_plane_batch, incircle2d, incircle2d_batch, insphere3d, insphere3d_batch,
    orient2d, orient2d_batch, orient3d, orient3d_batch,
};

#[test]
fn sequential_batches_match_scalar_predicates() {
    let orient2_cases = vec![
        (
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 0.0),
            Point2::new(0.0, 1.0),
        ),
        (
            Point2::new(0.0, 0.0),
            Point2::new(0.0, 1.0),
            Point2::new(1.0, 0.0),
        ),
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
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 1.0),
        ),
        (
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 1.0),
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
            Point3::new(0.0, 0.0, 3.0),
            Plane3::new(Point3::new(0.0, 0.0, 1.0), -2.0),
        ),
        (
            Point3::new(0.0, 0.0, 1.0),
            Plane3::new(Point3::new(0.0, 0.0, 1.0), -2.0),
        ),
    ];
    assert_eq!(
        classify_point_plane_batch(&plane_cases),
        plane_cases
            .iter()
            .map(|(point, plane)| classify_point_plane(point, plane))
            .collect::<Vec<_>>()
    );

    let incircle_cases = vec![(
        Point2::new(1.0, 0.0),
        Point2::new(0.0, 1.0),
        Point2::new(-1.0, 0.0),
        Point2::new(0.0, 0.0),
    )];
    assert_eq!(
        incircle2d_batch(&incircle_cases),
        incircle_cases
            .iter()
            .map(|(a, b, c, d)| incircle2d(a, b, c, d))
            .collect::<Vec<_>>()
    );

    let insphere_cases = vec![(
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(-1.0, 0.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
        Point3::new(0.0, 0.0, 1.0),
        Point3::new(0.0, 0.0, 0.0),
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
    let cases = vec![(
        Point2::new(0.0, 0.0),
        Point2::new(1.0, 0.0),
        Point2::new(0.0, 1.0),
    )];

    assert_eq!(
        hyperlimit::orient2d_batch_with_policy(&cases, PredicatePolicy::APPROXIMATE),
        vec![hyperlimit::orient::orient2d_with_policy(
            &cases[0].0,
            &cases[0].1,
            &cases[0].2,
            PredicatePolicy::APPROXIMATE,
        )]
    );
}

#[cfg(feature = "parallel")]
#[test]
fn parallel_batches_match_sequential_batches() {
    let orient2_cases = (0..2048)
        .map(|i| {
            let x = -0.5 + i as f64 / 2048.0;
            (
                Point2::new(0.0, 0.0),
                Point2::new(1.0, 1.0),
                Point2::new(x, x + if i % 2 == 0 { 1.0e-12 } else { -1.0e-12 }),
            )
        })
        .collect::<Vec<_>>();
    assert_eq!(
        orient2d_batch(&orient2_cases),
        hyperlimit::orient2d_batch_parallel(&orient2_cases)
    );

    let orient3_cases = (0..2048)
        .map(|i| {
            let x = -0.5 + i as f64 / 2048.0;
            (
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(x, -x, if i % 2 == 0 { 1.0e-12 } else { -1.0e-12 }),
            )
        })
        .collect::<Vec<_>>();
    assert_eq!(
        orient3d_batch(&orient3_cases),
        hyperlimit::orient3d_batch_parallel(&orient3_cases)
    );
}
