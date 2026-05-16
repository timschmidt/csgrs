use hypercurve::{
    BulgeVertex2, Classification, Contour2, CurvePolicy, Real, Region2, RegionPointLocation,
    RegionView2, UncertaintyReason,
};

fn s(value: i32) -> Real {
    value.into()
}

fn p(x: i32, y: i32) -> hypercurve::Point2 {
    hypercurve::Point2::new(s(x), s(y))
}

fn vertex(x: i32, y: i32) -> BulgeVertex2 {
    BulgeVertex2::new(p(x, y), s(0))
}

fn rectangle(xmin: i32, ymin: i32, xmax: i32, ymax: i32) -> Contour2 {
    Contour2::from_bulge_vertices(&[
        vertex(xmin, ymin),
        vertex(xmax, ymin),
        vertex(xmax, ymax),
        vertex(xmin, ymax),
    ])
    .unwrap()
}

fn policy() -> CurvePolicy {
    CurvePolicy::certified()
}

#[test]
fn empty_region_classifies_everything_outside() {
    let region = Region2::empty();

    assert!(region.is_empty());
    assert_eq!(
        region.signed_depth(&p(0, 0), &policy()),
        Classification::Decided(0)
    );
    assert_eq!(
        region.classify_point(&p(0, 0), &policy()),
        Classification::Decided(RegionPointLocation::Outside)
    );
}

#[test]
fn material_contour_classifies_inside_outside_and_boundary() {
    let region = Region2::from_material_contours(vec![rectangle(0, 0, 10, 10)]);

    assert_eq!(
        region.classify_point(&p(1, 1), &policy()),
        Classification::Decided(RegionPointLocation::Inside)
    );
    assert_eq!(
        region.classify_point(&p(11, 1), &policy()),
        Classification::Decided(RegionPointLocation::Outside)
    );
    assert_eq!(
        region.classify_point(&p(10, 5), &policy()),
        Classification::Decided(RegionPointLocation::Boundary)
    );
}

#[test]
fn region_aabb_miss_has_zero_depth_without_boundary_work() {
    let region = Region2::new(
        vec![rectangle(0, 0, 10, 10), rectangle(20, 20, 30, 30)],
        vec![rectangle(3, 3, 7, 7)],
    );

    assert_eq!(
        region.signed_depth(&p(100, 100), &policy()),
        Classification::Decided(0)
    );
    assert_eq!(
        region.classify_point(&p(100, 100), &policy()),
        Classification::Decided(RegionPointLocation::Outside)
    );
}

#[test]
fn sparse_region_classification_keeps_only_relevant_contour_depth() {
    let region = Region2::from_material_contours(vec![
        rectangle(0, 0, 4, 4),
        rectangle(20, 20, 24, 24),
        rectangle(40, 40, 44, 44),
    ]);

    assert_eq!(
        region.signed_depth(&p(21, 21), &policy()),
        Classification::Decided(1)
    );
    assert_eq!(
        region.classify_point(&p(21, 21), &policy()),
        Classification::Decided(RegionPointLocation::Inside)
    );
    assert_eq!(
        region.classify_point(&p(20, 22), &policy()),
        Classification::Decided(RegionPointLocation::Boundary)
    );
}

#[test]
fn hole_bin_subtracts_from_material_depth() {
    let region = Region2::new(vec![rectangle(0, 0, 10, 10)], vec![rectangle(3, 3, 7, 7)]);

    assert_eq!(
        region.signed_depth(&p(1, 1), &policy()),
        Classification::Decided(1)
    );
    assert_eq!(
        region.classify_point(&p(1, 1), &policy()),
        Classification::Decided(RegionPointLocation::Inside)
    );
    assert_eq!(
        region.signed_depth(&p(5, 5), &policy()),
        Classification::Decided(0)
    );
    assert_eq!(
        region.classify_point(&p(5, 5), &policy()),
        Classification::Decided(RegionPointLocation::Outside)
    );
}

#[test]
fn hole_boundary_is_explicit() {
    let region = Region2::new(vec![rectangle(0, 0, 10, 10)], vec![rectangle(3, 3, 7, 7)]);

    assert_eq!(
        region.signed_depth(&p(3, 5), &policy()),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
    assert_eq!(
        region.classify_point(&p(3, 5), &policy()),
        Classification::Decided(RegionPointLocation::Boundary)
    );
}

#[test]
fn material_island_inside_hole_adds_depth_back() {
    let region = Region2::new(
        vec![rectangle(0, 0, 10, 10), rectangle(4, 4, 6, 6)],
        vec![rectangle(2, 2, 8, 8)],
    );

    assert_eq!(
        region.signed_depth(&p(1, 1), &policy()),
        Classification::Decided(1)
    );
    assert_eq!(
        region.classify_point(&p(1, 1), &policy()),
        Classification::Decided(RegionPointLocation::Inside)
    );
    assert_eq!(
        region.signed_depth(&p(3, 3), &policy()),
        Classification::Decided(0)
    );
    assert_eq!(
        region.classify_point(&p(3, 3), &policy()),
        Classification::Decided(RegionPointLocation::Outside)
    );
    assert_eq!(
        region.signed_depth(&p(5, 5), &policy()),
        Classification::Decided(1)
    );
    assert_eq!(
        region.classify_point(&p(5, 5), &policy()),
        Classification::Decided(RegionPointLocation::Inside)
    );
}

#[test]
fn borrowed_region_view_matches_owned_region() {
    let outer = rectangle(0, 0, 10, 10);
    let island = rectangle(4, 4, 6, 6);
    let hole = rectangle(2, 2, 8, 8);
    let material = [outer.clone(), island.clone()];
    let holes = [hole.clone()];
    let view = RegionView2::new(&material, &holes);
    let owned = Region2::new(vec![outer, island], vec![hole]);

    assert_eq!(view.material_contours().len(), 2);
    assert_eq!(view.hole_contours().len(), 1);
    for point in [p(1, 1), p(3, 3), p(5, 5), p(11, 1)] {
        assert_eq!(
            view.classify_point(&point, &policy()),
            owned.classify_point(&point, &policy())
        );
        assert_eq!(
            view.signed_depth(&point, &policy()),
            owned.signed_depth(&point, &policy())
        );
    }
}

#[test]
fn prepared_region_classifier_matches_owned_region() {
    let region = Region2::new(
        vec![rectangle(0, 0, 10, 10), rectangle(4, 4, 6, 6)],
        vec![rectangle(2, 2, 8, 8)],
    );
    let policy = policy();
    let prepared = region.prepare_point_classifier(&policy);

    assert!(prepared.region_box().is_some());
    assert_eq!(prepared.material_contours().len(), 2);
    assert_eq!(prepared.hole_contours().len(), 1);

    for point in [p(1, 1), p(3, 3), p(5, 5), p(11, 1), p(100, 100)] {
        assert_eq!(
            prepared.classify_point(&point, &policy),
            region.classify_point(&point, &policy)
        );
        assert_eq!(
            prepared.signed_depth(&point, &policy),
            region.signed_depth(&point, &policy)
        );
    }
}

#[test]
fn prepared_region_view_preserves_boundary_hits() {
    let material = [rectangle(0, 0, 4, 4), rectangle(20, 20, 24, 24)];
    let holes: [Contour2; 0] = [];
    let view = RegionView2::new(&material, &holes);
    let policy = policy();
    let prepared = view.prepare_point_classifier(&policy);

    assert_eq!(
        prepared.classify_point(&p(20, 22), &policy),
        Classification::Decided(RegionPointLocation::Boundary)
    );
    assert_eq!(
        prepared.classify_point(&p(21, 21), &policy),
        Classification::Decided(RegionPointLocation::Inside)
    );
    assert_eq!(
        prepared.classify_point(&p(100, 100), &policy),
        Classification::Decided(RegionPointLocation::Outside)
    );
    assert_eq!(
        prepared.signed_depth(&p(100, 100), &policy),
        Classification::Decided(0)
    );
}
