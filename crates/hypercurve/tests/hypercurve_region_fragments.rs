use hypercurve::{
    BulgeVertex2, Classification, Contour2, CurveError, CurvePolicy, Real, Region2,
    RegionContourFragments, RegionContourKey, RegionContourRole, RegionFragmentBuildStage2,
    RegionFragmentSet, RegionSide, SegmentKind, SegmentKindCounts,
};

fn s(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
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
        Err(CurveError::Topology(_)) => {}
        Ok(_) => panic!("expected topology error"),
        Err(error) => panic!("expected topology error, got {error:?}"),
    }
}

#[test]
fn region_fragments_split_all_keyed_contours() {
    let region = Region2::new(vec![rectangle(0, 0, 10, 10)], vec![rectangle(3, 3, 7, 7)]);
    let cutter = Region2::from_material_contours(vec![rectangle(5, -1, 11, 11)]);
    let intersections = region.intersect_region(&cutter, &policy()).unwrap();

    let built = intersections
        .split_regions_with_report(&region.as_view(), &cutter.as_view(), &policy())
        .unwrap();
    assert!(built.report().status().is_native_exact());
    assert_eq!(
        built.report().stage(),
        RegionFragmentBuildStage2::ContourSplitting
    );
    assert_eq!(built.report().first_source_contour_count(), 2);
    assert_eq!(built.report().second_source_contour_count(), 1);
    assert_eq!(built.report().first_material_source_segment_count(), 4);
    assert_eq!(built.report().first_hole_source_segment_count(), 4);
    assert_eq!(built.report().second_material_source_segment_count(), 4);
    assert_eq!(built.report().second_hole_source_segment_count(), 0);
    assert_eq!(built.report().first_source_segment_count(), 8);
    assert_eq!(built.report().second_source_segment_count(), 4);
    assert_eq!(
        built.report().intersection_pair_count(),
        intersections.intersecting_pair_count()
    );
    assert_eq!(
        built.report().intersection_event_count(),
        intersections.event_count()
    );
    assert_eq!(
        built.report().first_event_segment_kind_counts(),
        intersections.first_event_segment_kind_counts()
    );
    assert_eq!(
        built.report().second_event_segment_kind_counts(),
        intersections.second_event_segment_kind_counts()
    );
    assert_eq!(
        built.report().first_event_segment_kind_counts(),
        SegmentKindCounts { lines: 4, arcs: 0 }
    );
    assert_eq!(
        built.report().second_event_segment_kind_counts(),
        SegmentKindCounts { lines: 4, arcs: 0 }
    );
    assert_eq!(
        built.report().candidate_pair_count(),
        intersections.candidate_pair_count()
    );
    assert_eq!(
        built.report().skipped_aabb_pair_count(),
        intersections.skipped_aabb_pair_count()
    );
    assert_eq!(
        built.report().tested_pair_count(),
        intersections.tested_pair_count()
    );
    assert_eq!(built.report().output_contour_count(), Some(3));
    assert_eq!(built.report().output_fragment_count(), Some(20));
    assert_eq!(built.report().first_output_contour_count(), Some(2));
    assert_eq!(built.report().second_output_contour_count(), Some(1));
    assert_eq!(built.report().first_output_fragment_count(), Some(12));
    assert_eq!(built.report().second_output_fragment_count(), Some(8));
    assert_eq!(
        built.report().first_material_output_fragment_count(),
        Some(6)
    );
    assert_eq!(built.report().first_hole_output_fragment_count(), Some(6));
    assert_eq!(
        built.report().second_material_output_fragment_count(),
        Some(8)
    );
    assert_eq!(built.report().second_hole_output_fragment_count(), Some(0));
    assert_eq!(built.report().contour_reports().len(), 3);
    assert_eq!(built.report().blocker(), None);

    let material_key = RegionContourKey::new(RegionSide::First, RegionContourRole::Material, 0);
    let hole_key = RegionContourKey::new(RegionSide::First, RegionContourRole::Hole, 0);
    let cutter_key = RegionContourKey::new(RegionSide::Second, RegionContourRole::Material, 0);
    let fragments = built
        .fragments()
        .expect("reported region fragments should materialize");

    assert_eq!(fragments.len(), 3);
    assert_eq!(
        fragments
            .fragments_for_contour(material_key)
            .unwrap()
            .fragments
            .len(),
        6
    );
    assert_eq!(
        fragments
            .fragments_for_contour(hole_key)
            .unwrap()
            .fragments
            .len(),
        6
    );
    assert_eq!(
        fragments
            .fragments_for_contour(cutter_key)
            .unwrap()
            .fragments
            .len(),
        8
    );
    assert_eq!(built.report().contour_reports()[0].key(), material_key);
    assert_eq!(
        built.report().contour_reports()[0].source_segment_count(),
        4
    );
    assert_eq!(
        built.report().contour_reports()[0].source_segment_kind_counts(),
        SegmentKindCounts { lines: 4, arcs: 0 }
    );
    assert_eq!(
        built.report().contour_reports()[0].contributing_pair_count(),
        intersections.pairs_for_contour(material_key).count()
    );
    assert_eq!(
        built.report().contour_reports()[0].intersection_event_count(),
        intersections
            .pairs_for_contour(material_key)
            .map(|pair| pair.intersections.events().len())
            .sum()
    );
    assert_eq!(
        built.report().contour_reports()[0].output_fragment_count(),
        6
    );
    assert_eq!(
        built.report().contour_reports()[0].output_fragment_kind_counts(),
        SegmentKindCounts { lines: 6, arcs: 0 }
    );
    assert_eq!(
        built.report().contour_reports()[0].output_fragments().len(),
        6
    );
    assert_eq!(
        built.report().contour_reports()[0].output_fragments()[0].source_segment_index(),
        0
    );
    assert_eq!(
        built.report().contour_reports()[0].output_fragments()[0].source_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        built.report().contour_reports()[0].output_fragments()[0].source_segment_start_point(),
        &p(0, 0)
    );
    assert_eq!(
        built.report().contour_reports()[0].output_fragments()[0].source_segment_end_point(),
        &p(10, 0)
    );
    assert_eq!(
        built.report().contour_reports()[0].output_fragments()[0]
            .source_range()
            .start(),
        &s(0)
    );
    assert_eq!(
        built.report().contour_reports()[0].output_fragments()[0]
            .source_range()
            .end(),
        &q(1, 2)
    );
    assert_eq!(
        built.report().contour_reports()[0].output_fragments()[0].output_fragment_index(),
        0
    );
    assert_eq!(
        built.report().contour_reports()[0].output_fragments()[0].output_fragment_kind(),
        SegmentKind::Line
    );
    assert!(
        built.report().contour_reports()[0]
            .status()
            .is_native_exact()
    );
    assert_eq!(built.report().contour_reports()[1].key(), hole_key);
    assert_eq!(
        built.report().contour_reports()[1].contributing_pair_count(),
        intersections.pairs_for_contour(hole_key).count()
    );
    assert_eq!(
        built.report().contour_reports()[1].intersection_event_count(),
        intersections
            .pairs_for_contour(hole_key)
            .map(|pair| pair.intersections.events().len())
            .sum()
    );
    assert_eq!(
        built.report().contour_reports()[1].output_fragment_count(),
        6
    );
    assert_eq!(built.report().contour_reports()[2].key(), cutter_key);
    assert_eq!(
        built.report().contour_reports()[2].contributing_pair_count(),
        intersections.pairs_for_contour(cutter_key).count()
    );
    assert_eq!(
        built.report().contour_reports()[2].intersection_event_count(),
        intersections
            .pairs_for_contour(cutter_key)
            .map(|pair| pair.intersections.events().len())
            .sum()
    );
    assert_eq!(
        built.report().contour_reports()[2].output_fragment_count(),
        8
    );
}

#[test]
fn region_fragment_set_constructor_validates_unique_contour_keys() {
    RegionFragmentSet::new(Vec::new()).unwrap();
    assert_topology_error(RegionFragmentSet::new(vec![RegionContourFragments {
        key: RegionContourKey::new(RegionSide::First, RegionContourRole::Material, 0),
        fragments: hypercurve::ContourFragmentSet::new(Vec::new()).unwrap(),
    }]));

    let first = Region2::from_material_contours(vec![rectangle(0, 0, 2, 2)]);
    let second = Region2::from_material_contours(vec![rectangle(4, 4, 6, 6)]);
    let intersections = first.intersect_region(&second, &policy()).unwrap();
    let Classification::Decided(fragments) = intersections
        .split_regions(&first.as_view(), &second.as_view(), &policy())
        .unwrap()
    else {
        panic!("expected decided disjoint fragments");
    };

    let first_key = RegionContourKey::new(RegionSide::First, RegionContourRole::Material, 0);
    let second_key = RegionContourKey::new(RegionSide::Second, RegionContourRole::Material, 0);
    let first_fragments = fragments
        .fragments_for_contour(first_key)
        .unwrap()
        .fragments
        .clone();
    let second_fragments = fragments
        .fragments_for_contour(second_key)
        .unwrap()
        .fragments
        .clone();

    RegionFragmentSet::new(vec![
        RegionContourFragments {
            key: first_key,
            fragments: first_fragments.clone(),
        },
        RegionContourFragments {
            key: second_key,
            fragments: second_fragments,
        },
    ])
    .unwrap();

    assert_topology_error(RegionFragmentSet::new(vec![
        RegionContourFragments {
            key: first_key,
            fragments: first_fragments.clone(),
        },
        RegionContourFragments {
            key: first_key,
            fragments: first_fragments,
        },
    ]));
}

#[test]
fn region_fragments_keep_disjoint_contours_unsplit() {
    let first = Region2::from_material_contours(vec![rectangle(0, 0, 2, 2)]);
    let second = Region2::from_material_contours(vec![rectangle(4, 4, 6, 6)]);
    let intersections = first.intersect_region(&second, &policy()).unwrap();
    assert!(intersections.is_empty());

    let fragments = intersections
        .split_regions(&first.as_view(), &second.as_view(), &policy())
        .unwrap();
    let Classification::Decided(fragments) = fragments else {
        panic!("expected decided disjoint fragments");
    };

    let first_key = RegionContourKey::new(RegionSide::First, RegionContourRole::Material, 0);
    let second_key = RegionContourKey::new(RegionSide::Second, RegionContourRole::Material, 0);

    assert_eq!(fragments.len(), 2);
    assert_eq!(
        fragments
            .fragments_for_contour(first_key)
            .unwrap()
            .fragments
            .len(),
        4
    );
    assert_eq!(
        fragments
            .fragments_for_contour(second_key)
            .unwrap()
            .fragments
            .len(),
        4
    );

    let built = intersections
        .split_regions_with_report(&first.as_view(), &second.as_view(), &policy())
        .unwrap();
    assert!(built.report().status().is_native_exact());
    assert_eq!(
        built.report().intersection_event_count(),
        intersections.event_count()
    );
    assert_eq!(built.report().first_material_source_segment_count(), 4);
    assert_eq!(built.report().first_hole_source_segment_count(), 0);
    assert_eq!(built.report().second_material_source_segment_count(), 4);
    assert_eq!(built.report().second_hole_source_segment_count(), 0);
    assert_eq!(
        built.report().first_material_output_fragment_count(),
        Some(4)
    );
    assert_eq!(built.report().first_hole_output_fragment_count(), Some(0));
    assert_eq!(
        built.report().second_material_output_fragment_count(),
        Some(4)
    );
    assert_eq!(built.report().second_hole_output_fragment_count(), Some(0));
    assert_eq!(built.report().contour_reports().len(), 2);
    for report in built.report().contour_reports() {
        assert_eq!(report.contributing_pair_count(), 0);
        assert_eq!(report.intersection_event_count(), 0);
        assert_eq!(report.output_fragment_count(), 4);
        assert_eq!(report.output_fragments().len(), 4);
    }
    assert_eq!(
        built.report().contour_reports()[0].output_fragments()[0].source_segment_start_point(),
        &p(0, 0)
    );
    assert_eq!(
        built.report().contour_reports()[0].output_fragments()[0].source_segment_end_point(),
        &p(2, 0)
    );
    assert_eq!(
        built.report().contour_reports()[0].output_fragments()[0]
            .source_range()
            .start(),
        &s(0)
    );
    assert_eq!(
        built.report().contour_reports()[0].output_fragments()[0]
            .source_range()
            .end(),
        &s(1)
    );
}

#[test]
fn region_fragments_preserve_same_circle_arc_overlap_events() {
    let first = Region2::from_material_contours(vec![contour(&[vertex(0, 0, 1), vertex(2, 0, 1)])]);
    let second =
        Region2::from_material_contours(vec![contour(&[vertex(0, 0, 1), vertex(2, 0, 1)])]);

    let intersections = first.intersect_region(&second, &policy()).unwrap();
    let Classification::Decided(fragments) = intersections
        .split_regions(&first.as_view(), &second.as_view(), &policy())
        .unwrap()
    else {
        panic!("expected decided same-circle arc overlap fragments");
    };

    let first_key = RegionContourKey::new(RegionSide::First, RegionContourRole::Material, 0);
    let second_key = RegionContourKey::new(RegionSide::Second, RegionContourRole::Material, 0);
    assert_eq!(
        fragments
            .fragments_for_contour(first_key)
            .unwrap()
            .fragments
            .len(),
        2
    );
    assert_eq!(
        fragments
            .fragments_for_contour(second_key)
            .unwrap()
            .fragments
            .len(),
        2
    );
}
