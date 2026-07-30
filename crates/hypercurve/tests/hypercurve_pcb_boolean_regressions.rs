//! Exact Boolean regressions derived from release-scale PCB process checks.
//!
//! A solder-paste image commonly contains hundreds of disjoint curved
//! apertures, each strictly contained by one member of a much larger disjoint
//! copper image. Treating the two images as one all-to-all Boolean exposed a
//! severe performance cliff in the unified region kernel. Keep both a routine
//! reduced case and the original Easyduino-scale topology in the corpus.

use hypercurve::{BulgeVertex2, Classification, Contour2, CurvePolicy, CurveRegion2, Point2, Real};

fn point(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn subdivided_square(
    center_x: i64,
    center_y: i64,
    half_extent: i64,
    segments_per_side: i64,
) -> Contour2 {
    assert_eq!((2 * half_extent) % segments_per_side, 0);
    let step = 2 * half_extent / segments_per_side;
    let mut vertices = Vec::new();
    for index in 0..segments_per_side {
        vertices.push(BulgeVertex2::new(
            point(
                center_x - half_extent + index * step,
                center_y - half_extent,
            ),
            Real::zero(),
        ));
    }
    for index in 0..segments_per_side {
        vertices.push(BulgeVertex2::new(
            point(
                center_x + half_extent,
                center_y - half_extent + index * step,
            ),
            Real::zero(),
        ));
    }
    for index in 0..segments_per_side {
        vertices.push(BulgeVertex2::new(
            point(
                center_x + half_extent - index * step,
                center_y + half_extent,
            ),
            Real::zero(),
        ));
    }
    for index in 0..segments_per_side {
        vertices.push(BulgeVertex2::new(
            point(
                center_x - half_extent,
                center_y + half_extent - index * step,
            ),
            Real::zero(),
        ));
    }
    Contour2::from_bulge_vertices(&vertices)
        .expect("subdivided exact square forms a closed contour")
}

fn pcb_containment_fixture(
    cover_count: usize,
    subject_count: usize,
) -> (CurveRegion2, CurveRegion2) {
    assert!(subject_count <= cover_count);
    let centers = (0..cover_count).map(|index| {
        let column = i64::try_from(index % 32).expect("fixture column fits i64");
        let row = i64::try_from(index / 32).expect("fixture row fits i64");
        (column * 100, row * 100)
    });
    let cover = centers
        .clone()
        .map(|(x, y)| subdivided_square(x, y, 30, 15))
        .collect::<Vec<_>>();
    let subject = centers
        .take(subject_count)
        .map(|(x, y)| subdivided_square(x, y, 28, 14))
        .collect::<Vec<_>>();
    let policy = CurvePolicy::STRICT;
    (
        CurveRegion2::try_from_native_contours(cover, Vec::new(), &policy)
            .expect("disjoint cover components form an exact region"),
        CurveRegion2::try_from_native_contours(subject, Vec::new(), &policy)
            .expect("disjoint subject components form an exact region"),
    )
}

fn assert_exact_containment_difference_is_empty(cover_count: usize, subject_count: usize) {
    let (cover, subject) = pcb_containment_fixture(cover_count, subject_count);
    let result = subject
        .boolean_region(
            &cover,
            hypercurve::BooleanOp::Difference,
            &CurvePolicy::STRICT,
        )
        .expect("PCB containment difference must decide exactly")
        .value;
    assert!(result.is_empty());

    for point in [point(0, 0), point(100, 0)] {
        assert_eq!(
            cover
                .classify_point(&point, &CurvePolicy::STRICT)
                .expect("cover point classification must decide"),
            Classification::Decided(hypercurve::RegionPointLocation::Inside),
        );
    }
}

#[test]
fn pcb_process_image_containment_regression_decides_exactly() {
    assert_exact_containment_difference_is_empty(64, 24);
}

#[test]
#[ignore = "release-scale performance corpus; run explicitly when profiling region Booleans"]
fn easyduino_scale_process_image_containment_corpus() {
    // Matches the order of magnitude observed in the Easyduino Nano release:
    // 138 paste components against 479 front-copper components.
    assert_exact_containment_difference_is_empty(479, 138);
}

#[test]
#[ignore = "release-scale performance corpus; run explicitly when profiling region Booleans"]
fn easyduino_uno_scale_process_image_with_holes_corpus() {
    // Matches the Easyduino Uno front-copper topology that originally took
    // about a minute to classify: 1,098 material components, 455 holes, and
    // 136 paste apertures. The apertures use material components after the
    // holed prefix so their exact difference is empty.
    let centers = (0..1_098).map(|index| {
        let column = i64::from(index % 32);
        let row = i64::from(index / 32);
        (column * 100, row * 100)
    });
    let materials = centers
        .clone()
        .map(|(x, y)| subdivided_square(x, y, 30, 15))
        .collect::<Vec<_>>();
    let holes = centers
        .clone()
        .take(455)
        .map(|(x, y)| subdivided_square(x, y, 8, 8))
        .collect::<Vec<_>>();
    let subjects = centers
        .skip(455)
        .take(136)
        .map(|(x, y)| subdivided_square(x, y, 28, 14))
        .collect::<Vec<_>>();
    let policy = CurvePolicy::STRICT;
    let cover = CurveRegion2::try_from_native_contours(materials, holes, &policy)
        .expect("holed front-copper corpus forms an exact region");
    let subject = CurveRegion2::try_from_native_contours(subjects, Vec::new(), &policy)
        .expect("paste corpus forms an exact region");

    let result = subject
        .boolean_region(&cover, hypercurve::BooleanOp::Difference, &policy)
        .expect("holed PCB containment difference must decide exactly")
        .value;

    assert!(result.is_empty());
}

#[test]
fn pcb_process_image_hole_ownership_culls_sparse_materials() {
    let centers = (0..512).map(|index| {
        let column = i64::from(index % 32);
        let row = i64::from(index / 32);
        (column * 100, row * 100)
    });
    let materials = centers
        .clone()
        .map(|(x, y)| subdivided_square(x, y, 30, 15))
        .collect::<Vec<_>>();
    let holes = centers
        .take(128)
        .map(|(x, y)| subdivided_square(x, y, 10, 10))
        .collect::<Vec<_>>();
    let region = CurveRegion2::try_from_native_contours(materials, holes, &CurvePolicy::STRICT)
        .expect("sparse material and hole loops form an exact region");

    let profiles = region
        .boundary_profiles(&CurvePolicy::STRICT)
        .expect("PCB profile ownership must complete exactly");
    let Classification::Decided(profiles) = profiles else {
        panic!("PCB profile ownership must decide");
    };

    assert_eq!(profiles.len(), 512);
    assert_eq!(
        profiles
            .iter()
            .map(|profile| profile.holes().len())
            .sum::<usize>(),
        128
    );
    assert!(profiles.iter().all(|profile| profile.holes().len() <= 1));
}
