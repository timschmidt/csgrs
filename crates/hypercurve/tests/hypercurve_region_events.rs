use hypercurve::{
    BulgeVertex2, Classification, Contour2, ContourIntersection, ContourIntersectionSet,
    ContourOperand, ContourSplitMap, CurvePolicy, Real, Region2, RegionContourIntersection,
    RegionContourKey, RegionContourRole, RegionIntersectionSet, RegionSide,
};

fn s(value: i32) -> Real {
    value.into()
}

fn p(x: i32, y: i32) -> hypercurve::Point2 {
    hypercurve::Point2::new(s(x), s(y))
}

fn vertex(x: i32, y: i32, bulge: i32) -> BulgeVertex2 {
    BulgeVertex2::new(p(x, y), s(bulge))
}

fn contour(vertices: &[BulgeVertex2]) -> Contour2 {
    Contour2::from_bulge_vertices(vertices).unwrap()
}

fn rectangle(xmin: i32, ymin: i32, xmax: i32, ymax: i32) -> Contour2 {
    contour(&[
        vertex(xmin, ymin, 0),
        vertex(xmax, ymin, 0),
        vertex(xmax, ymax, 0),
        vertex(xmin, ymax, 0),
    ])
}

fn policy() -> CurvePolicy {
    CurvePolicy::certified()
}

fn assert_topology_error<T>(result: hypercurve::CurveResult<T>) {
    match result {
        Err(hypercurve::CurveError::Topology(_)) => {}
        Ok(_) => panic!("expected topology error"),
        Err(error) => panic!("expected topology error, got {error:?}"),
    }
}

#[test]
fn region_events_keep_material_and_hole_roles() {
    let region = Region2::new(vec![rectangle(0, 0, 10, 10)], vec![rectangle(3, 3, 7, 7)]);
    let cutter = Region2::from_material_contours(vec![rectangle(5, -1, 11, 11)]);

    let events = region.intersect_region(&cutter, &policy()).unwrap();
    assert_eq!(events.len(), 2);

    let material_key = RegionContourKey::new(RegionSide::First, RegionContourRole::Material, 0);
    let hole_key = RegionContourKey::new(RegionSide::First, RegionContourRole::Hole, 0);
    let cutter_key = RegionContourKey::new(RegionSide::Second, RegionContourRole::Material, 0);

    assert!(events.pairs().iter().any(|pair| {
        pair.first == material_key && pair.second == cutter_key && pair.intersections.len() == 2
    }));
    assert!(events.pairs().iter().any(|pair| {
        pair.first == hole_key && pair.second == cutter_key && pair.intersections.len() == 2
    }));
    assert_eq!(events.pairs_for_contour(cutter_key).count(), 2);
}

#[test]
fn region_intersection_set_constructor_validates_pair_ownership() {
    let empty = RegionIntersectionSet::new(Vec::new()).unwrap();
    assert_eq!(empty.first_contour_count(), None);
    assert_eq!(empty.second_contour_count(), None);

    let first = Region2::from_material_contours(vec![rectangle(0, 0, 4, 4)]);
    let second = Region2::from_material_contours(vec![rectangle(2, -1, 6, 3)]);
    let events = first.intersect_region(&second, &policy()).unwrap();
    let pair = events.pairs()[0].clone();
    let synthetic = RegionIntersectionSet::new(vec![pair.clone()]).unwrap();
    assert_eq!(synthetic.first_contour_count(), None);
    assert_eq!(synthetic.second_contour_count(), None);

    assert_topology_error(RegionIntersectionSet::new(vec![pair.clone(), pair.clone()]));
    assert_topology_error(RegionIntersectionSet::new(vec![
        RegionContourIntersection {
            first: pair.second,
            second: pair.first,
            intersections: pair.intersections,
        },
    ]));

    assert_topology_error(RegionIntersectionSet::new(vec![
        RegionContourIntersection {
            first: pair.first,
            second: pair.second,
            intersections: ContourIntersectionSet::new(Vec::new()).unwrap(),
        },
    ]));
}

#[test]
fn region_split_rejects_events_outside_supplied_views() {
    let first = Region2::from_material_contours(vec![rectangle(0, 0, 4, 4)]);
    let second = Region2::from_material_contours(vec![rectangle(2, -1, 6, 3)]);
    let events = first.intersect_region(&second, &policy()).unwrap();
    let pair = events.pairs()[0].clone();

    let forged_key_set = RegionIntersectionSet::new(vec![RegionContourIntersection {
        first: RegionContourKey::new(RegionSide::First, RegionContourRole::Material, 1),
        second: pair.second,
        intersections: pair.intersections.clone(),
    }])
    .unwrap();
    assert_topology_error(forged_key_set.split_regions(
        &first.as_view(),
        &second.as_view(),
        &policy(),
    ));

    let mut forged_events = pair.intersections.events().to_vec();
    match &mut forged_events[0] {
        ContourIntersection::Point(point) => point.a_segment_index = 99,
        ContourIntersection::Overlap(overlap) => overlap.a_segment_index = 99,
        ContourIntersection::Uncertain(uncertain) => uncertain.a_segment_index = 99,
    }
    let forged_segment_set = RegionIntersectionSet::new(vec![RegionContourIntersection {
        first: pair.first,
        second: pair.second,
        intersections: ContourIntersectionSet::new(forged_events).unwrap(),
    }])
    .unwrap();
    assert_topology_error(forged_segment_set.split_regions(
        &first.as_view(),
        &second.as_view(),
        &policy(),
    ));
}

#[test]
fn region_view_events_match_owned_region_events() {
    let region = Region2::new(vec![rectangle(0, 0, 10, 10)], vec![rectangle(3, 3, 7, 7)]);
    let cutter = Region2::from_material_contours(vec![rectangle(5, -1, 11, 11)]);

    let owned_events = region.intersect_region(&cutter, &policy()).unwrap();
    let view_events = region
        .as_view()
        .intersect_region(&cutter.as_view(), &policy())
        .unwrap();

    assert_eq!(owned_events, view_events);
}

#[test]
fn region_pair_events_feed_split_maps_for_keyed_contours() {
    let region = Region2::new(vec![rectangle(0, 0, 10, 10)], vec![rectangle(3, 3, 7, 7)]);
    let cutter = Region2::from_material_contours(vec![rectangle(5, -1, 11, 11)]);

    let events = region.intersect_region(&cutter, &policy()).unwrap();
    let material_key = RegionContourKey::new(RegionSide::First, RegionContourRole::Material, 0);
    let material_pair = events
        .pairs_for_contour(material_key)
        .next()
        .expect("expected material pair");

    let Classification::Decided(split_map) = ContourSplitMap::from_intersections(
        region.material_contours()[0].len(),
        &material_pair.intersections,
        ContourOperand::First,
        &policy(),
    ) else {
        panic!("expected decided material split map");
    };

    assert_eq!(split_map.params_for_segment(0).unwrap().len(), 3);
    assert_eq!(split_map.params_for_segment(2).unwrap().len(), 3);
}

#[test]
fn region_event_broad_phase_skips_disjoint_contour_pairs() {
    let region = Region2::new(
        vec![rectangle(0, 0, 4, 4), rectangle(20, 20, 24, 24)],
        vec![rectangle(40, 40, 44, 44)],
    );
    let cutter = Region2::from_material_contours(vec![rectangle(2, -1, 6, 2)]);

    let events = region.intersect_region(&cutter, &policy()).unwrap();

    assert_eq!(events.first_contour_count(), Some(3));
    assert_eq!(events.second_contour_count(), Some(1));
    assert_eq!(events.candidate_pair_count(), 3);
    assert_eq!(events.skipped_aabb_pair_count(), 2);
    assert_eq!(events.tested_pair_count(), 1);
    assert_eq!(events.intersecting_pair_count(), 1);
    assert_eq!(events.len(), 1);
    assert_eq!(
        events.pairs()[0].first,
        RegionContourKey::new(RegionSide::First, RegionContourRole::Material, 0)
    );
    assert_eq!(
        events.pairs()[0].second,
        RegionContourKey::new(RegionSide::Second, RegionContourRole::Material, 0)
    );
}

#[test]
fn region_event_broad_phase_keeps_boundary_touching_contours() {
    let region = Region2::from_material_contours(vec![rectangle(0, 0, 4, 4)]);
    let cutter = Region2::from_material_contours(vec![rectangle(4, 1, 6, 3)]);

    let events = region.intersect_region(&cutter, &policy()).unwrap();

    assert_eq!(events.len(), 1);
    assert!(!events.pairs()[0].intersections.is_empty());
}

#[test]
fn prepared_region_events_match_owned_region_events() {
    let region = Region2::new(
        vec![rectangle(0, 0, 10, 10), rectangle(30, 30, 34, 34)],
        vec![rectangle(3, 3, 7, 7)],
    );
    let cutter = Region2::from_material_contours(vec![rectangle(5, -1, 11, 11)]);
    let policy = policy();
    let prepared_region = region.prepare_topology_queries(&policy);
    let prepared_cutter = cutter.prepare_topology_queries(&policy);

    assert!(prepared_region.region_box().is_some());
    assert_eq!(prepared_region.prepared_material_contours().len(), 2);
    assert_eq!(prepared_region.prepared_hole_contours().len(), 1);
    assert_eq!(
        prepared_region.prepared_material_contours()[0]
            .segment_boxes()
            .len(),
        region.material_contours()[0].segments().len()
    );

    let owned_events = region.intersect_region(&cutter, &policy).unwrap();
    let prepared_events = prepared_region
        .intersect_prepared_region(&prepared_cutter, &policy)
        .unwrap();
    let mixed_events = prepared_region
        .intersect_region(&cutter.as_view(), &policy)
        .unwrap();
    let right_prepared_events = region
        .as_view()
        .intersect_prepared_region(&prepared_cutter, &policy)
        .unwrap();

    assert_eq!(prepared_events, owned_events);
    assert_eq!(mixed_events, owned_events);
    assert_eq!(right_prepared_events, owned_events);
    assert_eq!(prepared_events.first_contour_count(), Some(3));
    assert_eq!(prepared_events.second_contour_count(), Some(1));
    assert_eq!(prepared_events.len(), 2);
}

#[test]
fn prepared_region_events_keep_boundary_touching_contours() {
    let region = Region2::from_material_contours(vec![rectangle(0, 0, 4, 4)]);
    let cutter = Region2::from_material_contours(vec![rectangle(4, 1, 6, 3)]);
    let policy = policy();
    let prepared_region = region.prepare_topology_queries(&policy);
    let prepared_cutter = cutter.prepare_topology_queries(&policy);

    let events = prepared_region
        .intersect_prepared_region(&prepared_cutter, &policy)
        .unwrap();

    assert_eq!(events.len(), 1);
    assert!(!events.pairs()[0].intersections.is_empty());
}
