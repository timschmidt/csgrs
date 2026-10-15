use hypercurve::{
    CurveRegion2, CurveRegionLoopRole, MarchingSquaresError, MarchingSquaresGrid, Point2,
    PredicatePolicy, Real, RegionPointLocation, evaluate_under, marching_squares,
};

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
fn axes(n: usize) -> Vec<Real> {
    (0..n).map(|i| Real::from(i as u64)).collect()
}
fn extract(nx: usize, ny: usize, values: Vec<Real>) -> CurveRegion2 {
    marching_squares(&MarchingSquaresGrid {
        x: &axes(nx),
        y: &axes(ny),
        values: &values,
    })
    .unwrap()
}
fn location(region: &CurveRegion2, x: Real, y: Real) -> RegionPointLocation {
    region.classify_point(&Point2::new(x, y).into()).unwrap()
}

#[test]
fn exact_diamond_area_and_zero_vertex_regularization() {
    let mut values = vec![Real::one(); 9];
    values[4] = -Real::one();
    let region = extract(3, 3, values);
    assert_eq!(region.filled_area().unwrap(), Some(q(1, 2)));
    assert_eq!(
        location(&region, 1.into(), 1.into()),
        RegionPointLocation::Inside
    );
    assert_eq!(
        location(&region, q(1, 2), 1.into()),
        RegionPointLocation::Boundary
    );

    let values = (0..5)
        .flat_map(|y| (0..5).map(move |x| Real::from((x - 2) * (x - 2) + (y - 2) * (y - 2) - 1)))
        .collect();
    let region = extract(5, 5, values);
    assert_eq!(region.filled_area().unwrap(), Some(Real::from(2)));
    assert_eq!(
        location(&region, 2.into(), 2.into()),
        RegionPointLocation::Inside
    );
    assert_eq!(
        location(&region, 1.into(), 2.into()),
        RegionPointLocation::Boundary
    );
}

#[test]
fn native_arrangement_preserves_holes_and_disconnected_material() {
    let mut values = vec![Real::one(); 25];
    for y in 1..4 {
        for x in 1..4 {
            values[y * 5 + x] = -Real::one();
        }
    }
    values[12] = Real::one();
    let ring = extract(5, 5, values);
    // Outer 3x3 square with four corner triangles removed has area 17/2.
    // The central diamond has area 1/2, leaving exactly 8.
    assert_eq!(ring.filled_area().unwrap(), Some(Real::from(8)));
    assert_eq!(
        location(&ring, 2.into(), 2.into()),
        RegionPointLocation::Outside
    );
    assert_eq!(
        location(&ring, 1.into(), 1.into()),
        RegionPointLocation::Inside
    );
    let roles = ring.loop_roles().unwrap();
    let profiles = ring.boundary_profiles().unwrap();
    let (outer, holes) = profiles[0].linear_contours().unwrap().unwrap();
    assert_eq!(outer.segments().len(), 8);
    assert_eq!(holes.len(), 1);
    assert_eq!(holes[0].segments().len(), 4);
    assert_eq!(
        roles
            .iter()
            .filter(|&&r| r == CurveRegionLoopRole::Hole)
            .count(),
        1
    );
    assert_eq!(
        roles
            .iter()
            .filter(|&&r| r == CurveRegionLoopRole::Material)
            .count(),
        1
    );

    let mut values = vec![Real::one(); 21];
    values[8] = -Real::one();
    values[12] = -Real::one();
    let islands = extract(7, 3, values);
    assert_eq!(islands.filled_area().unwrap(), Some(Real::one()));
    assert_eq!(
        islands.loop_roles().unwrap(),
        vec![CurveRegionLoopRole::Material; 2]
    );
    assert_eq!(
        location(&islands, 3.into(), 1.into()),
        RegionPointLocation::Outside
    );
}

#[test]
fn both_diagonal_cases_obey_the_declared_center_rule() {
    for case_five in [false, true] {
        for center_inside in [false, true] {
            let mut values = vec![Real::from(3); 16];
            let negative = if center_inside {
                -Real::from(5)
            } else {
                -Real::one()
            };
            for i in if case_five { [5, 10] } else { [6, 9] } {
                values[i] = negative.clone();
            }
            let region = extract(4, 4, values);
            assert_eq!(
                location(&region, q(3, 2), q(3, 2)),
                if center_inside {
                    RegionPointLocation::Inside
                } else {
                    RegionPointLocation::Outside
                }
            );
            assert_eq!(
                region.loop_roles().unwrap().len(),
                if center_inside { 1 } else { 2 }
            );
        }
    }
}

#[test]
fn grid_domain_errors_never_publish_clipped_or_empty_geometry() {
    assert!(matches!(
        MarchingSquaresGrid::checked_sample_count([1, 3]),
        Err(MarchingSquaresError::GridTooSmall { .. })
    ));
    assert!(matches!(
        MarchingSquaresGrid::checked_sample_count([usize::MAX, 2]),
        Err(MarchingSquaresError::CapacityOverflow)
    ));
    let axis = axes(3);
    let short = vec![Real::one(); 8];
    assert!(matches!(
        marching_squares(&MarchingSquaresGrid {
            x: &axis,
            y: &axis,
            values: &short
        }),
        Err(MarchingSquaresError::SampleCountMismatch {
            expected: 9,
            actual: 8
        })
    ));
    let values = vec![Real::one(); 9];
    let reversed = vec![2.into(), 1.into(), 0.into()];
    assert!(matches!(
        marching_squares(&MarchingSquaresGrid {
            x: &reversed,
            y: &axis,
            values: &values
        }),
        Err(MarchingSquaresError::AxisNotIncreasing { .. })
    ));
    // The crossing reaches the sample boundary: no synthetic closing chord.
    let values = vec![
        Real::from(-1),
        1.into(),
        1.into(),
        (-1).into(),
        1.into(),
        1.into(),
        (-1).into(),
        1.into(),
        1.into(),
    ];
    assert!(matches!(
        marching_squares(&MarchingSquaresGrid {
            x: &axis,
            y: &axis,
            values: &values
        }),
        Err(MarchingSquaresError::BoundaryNotExterior { .. })
    ));
    assert!(extract(3, 3, vec![Real::one(); 9]).is_empty());
}

#[test]
fn actual_evidence_controls_certainty_across_policy_sequences() {
    let axis = axes(3);
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let unknown = &sine * &sine + &cosine * &cosine - Real::one();
    let mut values = vec![Real::one(); 9];
    values[0] = unknown;
    let grid = MarchingSquaresGrid {
        x: &axis,
        y: &axis,
        values: &values,
    };
    for policy in [
        PredicatePolicy::STRICT,
        PredicatePolicy::APPROXIMATE_512,
        PredicatePolicy::STRICT,
    ] {
        let result = evaluate_under(policy, || marching_squares(&grid));
        if policy == PredicatePolicy::STRICT {
            assert!(matches!(
                result.into_unverified(),
                Err(MarchingSquaresError::PredicateUndecided {
                    operation: "sample sign",
                    index: 0
                })
            ));
        } else {
            assert!(!result.is_certified());
            assert!(matches!(
                result.into_unverified(),
                Err(MarchingSquaresError::BoundaryNotExterior { sample: [0, 0] })
            ));
        }
    }
    let result = evaluate_under(PredicatePolicy::APPROXIMATE_512, || {
        extract(3, 3, vec![Real::one(); 9])
    });
    assert!(result.is_certified());
}

#[test]
fn exact_translation_beyond_float_range_preserves_area_and_location() {
    let shift = Real::from(10).powi(400.into()).unwrap();
    let axis: Vec<_> = axes(3).into_iter().map(|x| x + &shift).collect();
    let mut values = vec![Real::one(); 9];
    values[4] = -Real::one();
    let region = marching_squares(&MarchingSquaresGrid {
        x: &axis,
        y: &axis,
        values: &values,
    })
    .unwrap();
    assert_eq!(region.filled_area().unwrap(), Some(q(1, 2)));
    assert_eq!(
        location(&region, &shift + Real::one(), &shift + Real::one()),
        RegionPointLocation::Inside
    );
}

#[cfg(feature = "triangulation")]
#[test]
fn native_linear_triangulation_keeps_exact_vertices_and_hole_area() {
    let b = q(1, 3);
    let outer = hypercurve::Contour2::from_real_ring(&[
        [0.into(), 0.into()],
        [b.clone(), 0.into()],
        [b.clone(), b.clone()],
        [0.into(), b.clone()],
        [0.into(), 0.into()],
    ])
    .unwrap();
    let a = q(1, 9);
    let c = q(2, 9);
    let hole = hypercurve::Contour2::from_real_ring(&[
        [a.clone(), a.clone()],
        [a.clone(), c.clone()],
        [c.clone(), c.clone()],
        [c.clone(), a.clone()],
        [a.clone(), a.clone()],
    ])
    .unwrap();
    let result = hypercurve::triangulate_linear_contours(
        &hypertri::TriangulationContext::new(PredicatePolicy::STRICT),
        &outer,
        &[hole],
    )
    .unwrap();
    assert_eq!(
        result.certainty,
        hypertri::TriangulationCertainty::Certified
    );
    let twice_area = result.value.iter().fold(Real::zero(), |sum, [a, b, c]| {
        sum + (b.x() - a.x()) * (c.y() - a.y()) - (b.y() - a.y()) * (c.x() - a.x())
    });
    assert_eq!(twice_area, q(16, 81));
    assert!(result.value.iter().flatten().all(|p| {
        [Real::zero(), a.clone(), c.clone(), b.clone()].contains(p.x())
            && [Real::zero(), a.clone(), c.clone(), b.clone()].contains(p.y())
    }));
}
