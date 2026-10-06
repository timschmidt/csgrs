use hyperlimit::{
    Plane3, Point2, Point3, classify_circle_line2, classify_circle_line2_batch,
    classify_circle_segment2, classify_circle_segment2_batch, classify_point_line,
    classify_point_line_batch, classify_point_oriented_plane, classify_point_oriented_plane_batch,
    classify_point_plane, classify_point_plane_batch, classify_ray_triangle3_intersection,
    classify_ray_triangle3_intersection_batch, classify_segment_triangle3_intersection,
    classify_segment_triangle3_intersection_batch, classify_segment3_intersection,
    classify_segment3_intersection_batch, incircle2, incircle2_batch, insphere3, insphere3_batch,
    orient2, orient2_batch, orient3, orient3_batch,
};

const APPROX: hyperlimit::PredicatePolicy = hyperlimit::PredicatePolicy::APPROXIMATE_512;

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
        orient2_batch(&orient2_cases, APPROX),
        orient2_cases
            .iter()
            .map(|(a, b, c)| orient2(a, b, c, APPROX))
            .collect::<Vec<_>>()
    );
    assert_eq!(
        classify_point_line_batch(&orient2_cases, APPROX),
        orient2_cases
            .iter()
            .map(|(a, b, point)| classify_point_line(a, b, point, APPROX))
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
        orient3_batch(&orient3_cases, APPROX),
        orient3_cases
            .iter()
            .map(|(a, b, c, d)| orient3(a, b, c, d, APPROX))
            .collect::<Vec<_>>()
    );
    assert_eq!(
        classify_point_oriented_plane_batch(&orient3_cases, APPROX),
        orient3_cases
            .iter()
            .map(|(a, b, c, point)| classify_point_oriented_plane(a, b, c, point, APPROX))
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
        classify_point_plane_batch(&plane_cases, APPROX),
        plane_cases
            .iter()
            .map(|(point, plane)| classify_point_plane(point, plane, APPROX))
            .collect::<Vec<_>>()
    );

    let incircle_cases = vec![(p2(1.0, 0.0), p2(0.0, 1.0), p2(-1.0, 0.0), p2(0.0, 0.0))];
    assert_eq!(
        incircle2_batch(&incircle_cases, APPROX),
        incircle_cases
            .iter()
            .map(|(a, b, c, d)| incircle2(a, b, c, d, APPROX))
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
        insphere3_batch(&insphere_cases, APPROX),
        insphere_cases
            .iter()
            .map(|(a, b, c, d, e)| insphere3(a, b, c, d, e, APPROX))
            .collect::<Vec<_>>()
    );

    let segment3_cases = vec![(
        p3(0.0, 0.0, 0.0),
        p3(4.0, 0.0, 0.0),
        p3(2.0, -1.0, 0.0),
        p3(2.0, 1.0, 0.0),
    )];
    assert_eq!(
        classify_segment3_intersection_batch(&segment3_cases, APPROX),
        segment3_cases
            .iter()
            .map(|(a, b, c, d)| classify_segment3_intersection(a, b, c, d, APPROX))
            .collect::<Vec<_>>()
    );

    let segment_triangle_cases = vec![(
        p3(1.0, 1.0, -1.0),
        p3(1.0, 1.0, 1.0),
        p3(0.0, 0.0, 0.0),
        p3(4.0, 0.0, 0.0),
        p3(0.0, 4.0, 0.0),
    )];
    assert_eq!(
        classify_segment_triangle3_intersection_batch(&segment_triangle_cases, APPROX),
        segment_triangle_cases
            .iter()
            .map(|(p, q, a, b, c)| classify_segment_triangle3_intersection(p, q, a, b, c, APPROX))
            .collect::<Vec<_>>()
    );

    let ray_triangle_cases = vec![(
        p3(1.0, 1.0, -1.0),
        p3(0.0, 0.0, 1.0),
        p3(0.0, 0.0, 0.0),
        p3(4.0, 0.0, 0.0),
        p3(0.0, 4.0, 0.0),
    )];
    assert_eq!(
        classify_ray_triangle3_intersection_batch(&ray_triangle_cases, APPROX),
        ray_triangle_cases
            .iter()
            .map(|(origin, direction, a, b, c)| {
                classify_ray_triangle3_intersection(origin, direction, a, b, c, APPROX)
            })
            .collect::<Vec<_>>()
    );

    let circle_cases = vec![(p2(0.0, 0.0), real(25.0), p2(-10.0, 5.0), p2(10.0, 5.0))];
    assert_eq!(
        classify_circle_line2_batch(&circle_cases, APPROX),
        circle_cases
            .iter()
            .map(|(center, radius_squared, a, b)| {
                classify_circle_line2(center, radius_squared, a, b, APPROX)
            })
            .collect::<Vec<_>>()
    );
    assert_eq!(
        classify_circle_segment2_batch(&circle_cases, APPROX),
        circle_cases
            .iter()
            .map(|(center, radius_squared, a, b)| {
                classify_circle_segment2(center, radius_squared, a, b, APPROX)
            })
            .collect::<Vec<_>>()
    );
}

#[test]
fn batch_uses_the_strict_scalar_path_for_each_case() {
    let cases = vec![(p2(0.0, 0.0), p2(1.0, 0.0), p2(0.0, 1.0))];

    assert_eq!(
        hyperlimit::orient2_batch(&cases, APPROX),
        vec![hyperlimit::orient2(
            &cases[0].0,
            &cases[0].1,
            &cases[0].2,
            APPROX
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
        orient2_batch(&orient2_cases, APPROX),
        hyperlimit::orient2_batch_parallel(&orient2_cases, APPROX)
    );

    let orient3_cases = (0..2048)
        .map(|i| {
            let x = i as i128 - 1024;
            let z = if i % 2 == 0 { 1 } else { -1 };
            (sp3(0, 0, 0), sp3(1, 0, 0), sp3(0, 1, 0), sp3(x, -x, z))
        })
        .collect::<Vec<_>>();
    assert_eq!(
        orient3_batch(&orient3_cases, APPROX),
        hyperlimit::orient3_batch_parallel(&orient3_cases, APPROX)
    );

    let line_cases = vec![
        (sp2(0, 0), sp2(4, 0), sp2(2, 1)),
        (sp2(0, 0), sp2(4, 0), sp2(2, -1)),
        (sp2(0, 0), sp2(4, 0), sp2(2, 0)),
    ];
    assert_eq!(
        classify_point_line_batch(&line_cases, APPROX),
        hyperlimit::classify_point_line_batch_parallel(&line_cases, APPROX)
    );

    let plane_cases = vec![
        (sp3(0, 0, 3), Plane3::new(sp3(0, 0, 1), Real::from(-2_i128))),
        (sp3(0, 0, 2), Plane3::new(sp3(0, 0, 1), Real::from(-2_i128))),
        (sp3(0, 0, 1), Plane3::new(sp3(0, 0, 1), Real::from(-2_i128))),
    ];
    assert_eq!(
        classify_point_plane_batch(&plane_cases, APPROX),
        hyperlimit::classify_point_plane_batch_parallel(&plane_cases, APPROX)
    );
    assert_eq!(
        classify_point_oriented_plane_batch(&orient3_cases, APPROX),
        hyperlimit::classify_point_oriented_plane_batch_parallel(&orient3_cases, APPROX)
    );

    let incircle_cases = vec![
        (sp2(1, 0), sp2(0, 1), sp2(-1, 0), sp2(0, 0)),
        (sp2(1, 0), sp2(0, 1), sp2(-1, 0), sp2(0, -1)),
    ];
    assert_eq!(
        incircle2_batch(&incircle_cases, APPROX),
        hyperlimit::incircle2_batch_parallel(&incircle_cases, APPROX)
    );

    let insphere_cases = vec![
        (
            sp3(1, 0, 0),
            sp3(-1, 0, 0),
            sp3(0, 1, 0),
            sp3(0, 0, 1),
            sp3(0, 0, 0),
        ),
        (
            sp3(1, 0, 0),
            sp3(-1, 0, 0),
            sp3(0, 1, 0),
            sp3(0, 0, 1),
            sp3(1, 0, 0),
        ),
    ];
    assert_eq!(
        insphere3_batch(&insphere_cases, APPROX),
        hyperlimit::insphere3_batch_parallel(&insphere_cases, APPROX)
    );

    let segment3_cases = vec![
        (sp3(0, 0, 0), sp3(4, 0, 0), sp3(2, -1, 0), sp3(2, 1, 0)),
        (sp3(0, 0, 0), sp3(1, 0, 0), sp3(2, 0, 0), sp3(3, 0, 0)),
    ];
    assert_eq!(
        classify_segment3_intersection_batch(&segment3_cases, APPROX),
        hyperlimit::classify_segment3_intersection_batch_parallel(&segment3_cases, APPROX)
    );

    let triangle = (sp3(0, 0, 0), sp3(4, 0, 0), sp3(0, 4, 0));
    let segment_triangle_cases = vec![
        (
            sp3(1, 1, -1),
            sp3(1, 1, 1),
            triangle.0.clone(),
            triangle.1.clone(),
            triangle.2.clone(),
        ),
        (
            sp3(5, 5, -1),
            sp3(5, 5, 1),
            triangle.0.clone(),
            triangle.1.clone(),
            triangle.2.clone(),
        ),
    ];
    assert_eq!(
        classify_segment_triangle3_intersection_batch(&segment_triangle_cases, APPROX),
        hyperlimit::classify_segment_triangle3_intersection_batch_parallel(
            &segment_triangle_cases,
            APPROX
        )
    );

    let ray_triangle_cases = vec![
        (
            sp3(1, 1, -1),
            sp3(0, 0, 1),
            triangle.0.clone(),
            triangle.1.clone(),
            triangle.2.clone(),
        ),
        (
            sp3(1, 1, -1),
            sp3(0, 0, -1),
            triangle.0,
            triangle.1,
            triangle.2,
        ),
    ];
    assert_eq!(
        classify_ray_triangle3_intersection_batch(&ray_triangle_cases, APPROX),
        hyperlimit::classify_ray_triangle3_intersection_batch_parallel(&ray_triangle_cases, APPROX)
    );

    let circle_cases = vec![
        (sp2(0, 0), Real::from(25_i128), sp2(-10, 5), sp2(10, 5)),
        (sp2(0, 0), Real::from(25_i128), sp2(-10, 6), sp2(10, 6)),
    ];
    assert_eq!(
        classify_circle_line2_batch(&circle_cases, APPROX),
        hyperlimit::classify_circle_line2_batch_parallel(&circle_cases, APPROX)
    );
    assert_eq!(
        classify_circle_segment2_batch(&circle_cases, APPROX),
        hyperlimit::classify_circle_segment2_batch_parallel(&circle_cases, APPROX)
    );
}
