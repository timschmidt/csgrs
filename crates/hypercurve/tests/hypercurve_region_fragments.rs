use hypercurve::{
    BulgeVertex2, Classification, Contour2, CurvePolicy, DefaultBackend, Region2, RegionContourKey,
    RegionContourRole, RegionSide, Scalar,
};

fn s(value: i32) -> Scalar<DefaultBackend> {
    value.into()
}

fn p(x: i32, y: i32) -> hypercurve::Point2<DefaultBackend> {
    hypercurve::Point2::new(s(x), s(y))
}

fn vertex(x: i32, y: i32, bulge: i32) -> BulgeVertex2<DefaultBackend> {
    BulgeVertex2::new(p(x, y), s(bulge))
}

fn contour(vertices: &[BulgeVertex2<DefaultBackend>]) -> Contour2<DefaultBackend> {
    Contour2::from_bulge_vertices(vertices).unwrap()
}

fn rectangle(xmin: i32, ymin: i32, xmax: i32, ymax: i32) -> Contour2<DefaultBackend> {
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

#[test]
fn region_fragments_split_all_keyed_contours() {
    let region = Region2::new(vec![rectangle(0, 0, 10, 10)], vec![rectangle(3, 3, 7, 7)]);
    let cutter = Region2::from_material_contours(vec![rectangle(5, -1, 11, 11)]);
    let intersections = region.intersect_region(&cutter, &policy()).unwrap();

    let fragments = intersections
        .split_regions(&region.as_view(), &cutter.as_view(), &policy())
        .unwrap();
    let Classification::Decided(fragments) = fragments else {
        panic!("expected decided region fragments");
    };

    let material_key = RegionContourKey::new(RegionSide::First, RegionContourRole::Material, 0);
    let hole_key = RegionContourKey::new(RegionSide::First, RegionContourRole::Hole, 0);
    let cutter_key = RegionContourKey::new(RegionSide::Second, RegionContourRole::Material, 0);

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
