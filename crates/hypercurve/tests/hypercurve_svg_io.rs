#![cfg(feature = "svg")]

use hypercurve::{
    BulgeVertex2, CircularArc2, Contour2, ContourClosureStage2, CurveString2, FillRule, LineSeg2,
    Point2, Real, Region2, RegionBoundaryContourBuildStage2, RetainedImportFormat2,
    RetainedImportTopology2, RetainedTopologyStatus, Segment2, SegmentKind, SvgPathExportTarget2,
    UncertaintyReason, import_svg_contour_path_data_with_report, import_svg_path_data_with_report,
    import_svg_region_path_data_with_report, retained_svg_import_record,
};

fn s(value: i32) -> Real {
    Real::from(value)
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(s(x), s(y))
}

fn line(a: (i32, i32), b: (i32, i32)) -> Segment2 {
    Segment2::Line(LineSeg2::try_new(p(a.0, a.1), p(b.0, b.1)).unwrap())
}

fn rectangle(xmin: i32, ymin: i32, xmax: i32, ymax: i32) -> Contour2 {
    Contour2::from_bulge_vertices(&[
        BulgeVertex2::new(p(xmin, ymin), Real::zero()),
        BulgeVertex2::new(p(xmax, ymin), Real::zero()),
        BulgeVertex2::new(p(xmax, ymax), Real::zero()),
        BulgeVertex2::new(p(xmin, ymax), Real::zero()),
    ])
    .unwrap()
}

#[test]
fn curve_string_svg_export_reports_display_boundary() {
    let curve = CurveString2::try_new(vec![line((0, 0), (2, 0)), line((2, 0), (2, 1))]).unwrap();

    let exported = curve.to_svg_path_data_with_report().unwrap();

    assert_eq!(exported.path_data().unwrap(), "M 0 0 L 2 0 L 2 1");
    assert_eq!(
        exported.report().target(),
        SvgPathExportTarget2::CurveString
    );
    assert_eq!(exported.report().segment_count(), 2);
    assert_eq!(exported.report().segment_reports().len(), 2);
    assert_eq!(exported.report().segment_reports()[0].carrier_index(), 0);
    assert_eq!(exported.report().segment_reports()[0].segment_index(), 0);
    assert_eq!(
        exported.report().segment_reports()[0].segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        exported.report().segment_reports()[0].start_point(),
        &p(0, 0)
    );
    assert_eq!(exported.report().segment_reports()[0].end_point(), &p(2, 0));
    assert_eq!(
        exported.report().segment_reports()[0].status(),
        RetainedTopologyStatus::DisplayOrExport
    );
    assert_eq!(exported.report().segment_reports()[1].carrier_index(), 0);
    assert_eq!(exported.report().segment_reports()[1].segment_index(), 1);
    assert_eq!(
        exported.report().segment_reports()[1].start_point(),
        &p(2, 0)
    );
    assert_eq!(exported.report().segment_reports()[1].end_point(), &p(2, 1));
    assert_eq!(exported.report().closed_subpath_count(), 0);
    assert_eq!(
        exported.report().status(),
        RetainedTopologyStatus::DisplayOrExport
    );
    assert_eq!(exported.report().blocker(), None);
}

#[test]
fn arc_svg_export_uses_exact_radius_not_radius_squared() {
    let arc =
        Segment2::Arc(CircularArc2::try_from_center(p(2, 0), p(0, 2), p(0, 0), false).unwrap());
    let curve = CurveString2::try_new(vec![arc]).unwrap();

    let exported = curve.to_svg_path_data_with_report().unwrap();

    assert_eq!(exported.path_data().unwrap(), "M 2 0 A 2 2 0 0 0 0 2");
    assert_eq!(exported.report().segment_kind_counts().arcs, 1);
}

#[test]
fn region_svg_export_preserves_material_and_hole_counts() {
    let region = Region2::new(vec![rectangle(0, 0, 4, 4)], vec![rectangle(1, 1, 2, 2)]);

    let exported = region.to_svg_path_data_with_report().unwrap();

    assert_eq!(exported.report().target(), SvgPathExportTarget2::Region);
    assert_eq!(exported.report().material_contour_count(), 1);
    assert_eq!(exported.report().hole_contour_count(), 1);
    assert_eq!(exported.report().curve_string_count(), 2);
    assert_eq!(exported.report().closed_subpath_count(), 2);
    assert_eq!(exported.report().segment_count(), 8);
    assert_eq!(exported.report().segment_reports().len(), 8);
    assert_eq!(exported.report().segment_reports()[0].carrier_index(), 0);
    assert_eq!(exported.report().segment_reports()[0].segment_index(), 0);
    assert_eq!(
        exported.report().segment_reports()[0].start_point(),
        &p(0, 0)
    );
    assert_eq!(exported.report().segment_reports()[0].end_point(), &p(4, 0));
    assert_eq!(exported.report().segment_reports()[4].carrier_index(), 1);
    assert_eq!(exported.report().segment_reports()[4].segment_index(), 0);
    assert_eq!(
        exported.report().segment_reports()[4].start_point(),
        &p(1, 1)
    );
    assert_eq!(exported.report().segment_reports()[4].end_point(), &p(2, 1));
    assert!(exported.path_data().unwrap().contains("M 1 1"));
}

#[test]
fn svg_line_path_import_materializes_with_retained_report() {
    let imported = import_svg_path_data_with_report("M 0 0 L 1.5 0 H 2 V .5 Z", 7, 3, None);

    let curve = imported
        .curve_string()
        .expect("line path should materialize");
    assert_eq!(curve.len(), 4);
    assert_eq!(
        curve
            .to_svg_path_data_with_report()
            .unwrap()
            .path_data()
            .unwrap(),
        "M 0 0 L 1 5/10 0 L 2 0 L 2 5/10 L 0 0"
    );
    assert_eq!(imported.report().source_index(), 7);
    assert_eq!(imported.report().source_version(), 3);
    assert_eq!(imported.report().command_count(), 5);
    assert_eq!(
        imported.report().status(),
        RetainedTopologyStatus::ImportedLossy
    );
    assert_eq!(imported.report().blocker(), None);
    assert!(imported.report().lossy_boundary());
    let record = imported.report().retained_import().unwrap();
    assert_eq!(record.format(), RetainedImportFormat2::Svg);
    assert_eq!(record.source_version(), 3);
    assert!(record.topology_status().is_imported_lossy());
}

#[test]
fn svg_relative_line_path_import_materializes_exact_native_lines() {
    let imported = import_svg_path_data_with_report("m 1 1 l 2 0 h 1 v 2 z", 23, 2, None);

    let curve = imported
        .curve_string()
        .expect("relative line path should materialize");
    assert_eq!(curve.len(), 4);
    assert_eq!(
        curve
            .to_svg_path_data_with_report()
            .unwrap()
            .path_data()
            .unwrap(),
        "M 1 1 L 3 1 L 4 1 L 4 3 L 1 1"
    );
    assert_eq!(imported.report().source_index(), 23);
    assert_eq!(imported.report().source_version(), 2);
    assert_eq!(imported.report().command_count(), 5);
    assert_eq!(
        imported.report().status(),
        RetainedTopologyStatus::ImportedLossy
    );
    let record = imported.report().retained_import().unwrap();
    assert_eq!(
        record.source_topology(),
        RetainedImportTopology2::ClosedRing
    );
    assert_eq!(record.input_point_count(), 4);
    assert_eq!(record.emitted_segment_count(), 4);
}

#[test]
fn svg_circular_semicircle_arc_import_materializes_with_retained_report() {
    let imported = import_svg_path_data_with_report("M 0 0 A 1 1 0 0 0 2 0", 8, 1, None);

    let curve = imported
        .curve_string()
        .expect("circular semicircle arc path should materialize");
    assert_eq!(curve.len(), 1);
    assert_eq!(
        curve
            .to_svg_path_data_with_report()
            .unwrap()
            .path_data()
            .unwrap(),
        "M 0 0 A 1 1 0 0 0 2 0"
    );
    assert_eq!(imported.report().source_index(), 8);
    assert_eq!(imported.report().source_version(), 1);
    assert_eq!(imported.report().command_count(), 2);
    assert_eq!(
        imported.report().status(),
        RetainedTopologyStatus::ImportedLossy
    );
    assert_eq!(imported.report().blocker(), None);
    let record = imported.report().retained_import().unwrap();
    assert_eq!(record.format(), RetainedImportFormat2::Svg);
    assert_eq!(
        record.source_topology(),
        RetainedImportTopology2::OpenCurveString
    );
    assert_eq!(record.input_point_count(), 2);
    assert_eq!(record.emitted_segment_count(), 1);
}

#[test]
fn svg_relative_semicircle_arc_import_materializes_exact_native_arc() {
    let imported = import_svg_path_data_with_report("m 0 0 a 1 1 0 0 0 2 0", 24, 1, None);

    let curve = imported
        .curve_string()
        .expect("relative semicircle arc path should materialize");
    assert_eq!(curve.len(), 1);
    assert_eq!(
        curve
            .to_svg_path_data_with_report()
            .unwrap()
            .path_data()
            .unwrap(),
        "M 0 0 A 1 1 0 0 0 2 0"
    );
    assert_eq!(imported.report().source_index(), 24);
    assert_eq!(
        imported.report().status(),
        RetainedTopologyStatus::ImportedLossy
    );
    let record = imported.report().retained_import().unwrap();
    assert_eq!(
        record.source_topology(),
        RetainedImportTopology2::OpenCurveString
    );
}

#[test]
fn svg_cubic_path_import_remains_explicitly_unsupported() {
    let imported = import_svg_path_data_with_report("M 0 0 C 1 0 1 1 2 1", 8, 1, None);

    assert!(imported.curve_string().is_none());
    assert_eq!(imported.report().source_index(), 8);
    assert_eq!(imported.report().command_count(), 2);
    assert_eq!(
        imported.report().status(),
        RetainedTopologyStatus::Unsupported
    );
    assert_eq!(
        imported.report().blocker(),
        Some(UncertaintyReason::Unsupported)
    );
}

#[test]
fn svg_non_semicircle_arc_import_remains_explicitly_unsupported() {
    let imported = import_svg_path_data_with_report("M 0 0 A 2 2 0 0 0 2 0", 9, 1, None);

    assert!(imported.curve_string().is_none());
    assert_eq!(imported.report().source_index(), 9);
    assert_eq!(imported.report().command_count(), 2);
    assert_eq!(
        imported.report().status(),
        RetainedTopologyStatus::Unsupported
    );
    assert_eq!(
        imported.report().blocker(),
        Some(UncertaintyReason::Unsupported)
    );
}

#[test]
fn svg_closed_line_path_import_materializes_contour_with_closure_evidence() {
    let path_data = "M 0 0 L 2 0 L 2 1 L 0 1 Z";
    let imported =
        import_svg_contour_path_data_with_report(path_data, FillRule::EvenOdd, 12, 4, None);

    let contour = imported
        .contour()
        .expect("closed line path should materialize");
    assert_eq!(contour.len(), 4);
    assert_eq!(contour.fill_rule(), FillRule::EvenOdd);
    assert_eq!(
        imported.report().status(),
        RetainedTopologyStatus::ImportedLossy
    );
    assert!(imported.report().lossy_boundary());
    assert_eq!(imported.report().blocker(), None);
    assert_eq!(imported.report().source_index(), 12);
    assert_eq!(imported.report().source_version(), 4);
    assert_eq!(imported.report().source_tolerance(), None);
    assert_eq!(imported.report().input_byte_count(), path_data.len());
    assert_eq!(imported.report().command_count(), 5);
    let record = imported.report().retained_import().unwrap();
    assert_eq!(record.format(), RetainedImportFormat2::Svg);
    assert_eq!(record.source_index(), 12);
    assert_eq!(record.source_version(), 4);
    assert_eq!(imported.report().path_report().source_index(), 12);
    assert_eq!(
        imported.report().closure_report().unwrap().stage(),
        ContourClosureStage2::ContourMaterialization
    );
}

#[test]
fn svg_closed_semicircle_arc_import_materializes_two_segment_contour() {
    let path_data = "M 0 0 A 1 1 0 0 0 2 0 Z";
    let imported =
        import_svg_contour_path_data_with_report(path_data, FillRule::NonZero, 10, 1, None);

    assert_eq!(
        imported.report().path_report().status(),
        RetainedTopologyStatus::ImportedLossy
    );
    assert_eq!(
        imported.report().status(),
        RetainedTopologyStatus::ImportedLossy
    );
    let contour = imported
        .contour()
        .expect("closed semicircle arc path should materialize");
    assert_eq!(contour.len(), 2);
    let record = imported.report().retained_import().unwrap();
    assert_eq!(
        record.source_topology(),
        RetainedImportTopology2::ClosedContour
    );
    assert_eq!(record.input_point_count(), 2);
    assert_eq!(record.emitted_segment_count(), 2);
}

#[test]
fn svg_open_line_path_contour_import_is_explicitly_unsupported() {
    let imported = import_svg_contour_path_data_with_report(
        "M 0 0 L 2 0 L 2 1",
        FillRule::NonZero,
        13,
        0,
        None,
    );

    assert!(imported.contour().is_none());
    assert_eq!(
        imported.report().status(),
        RetainedTopologyStatus::Unsupported
    );
    assert_eq!(
        imported.report().blocker(),
        Some(UncertaintyReason::Unsupported)
    );
    assert!(imported.report().closure_report().is_none());
}

#[test]
fn svg_region_import_materializes_nested_closed_line_subpaths() {
    let imported = import_svg_region_path_data_with_report(
        "M 0 0 L 4 0 L 4 4 L 0 4 Z M 1 1 L 2 1 L 2 2 L 1 2 Z",
        FillRule::NonZero,
        21,
        6,
        None,
        &hypercurve::CurvePolicy::certified(),
    );

    let region = imported
        .region()
        .expect("nested closed subpaths should materialize");
    assert_eq!(region.material_contours().len(), 1);
    assert_eq!(region.hole_contours().len(), 1);
    assert_eq!(
        imported.report().status(),
        RetainedTopologyStatus::ImportedLossy
    );
    assert!(imported.report().lossy_boundary());
    assert_eq!(imported.report().blocker(), None);
    assert_eq!(imported.report().source_index(), 21);
    assert_eq!(imported.report().source_version(), 6);
    assert_eq!(imported.report().subpath_count(), 2);
    assert_eq!(imported.report().materialized_contour_count(), 2);
    assert_eq!(imported.report().path_reports().len(), 2);
    assert_eq!(imported.report().closure_reports().len(), 2);
    assert_eq!(
        imported.report().boundary_build_stage(),
        Some(RegionBoundaryContourBuildStage2::RoleAssignment)
    );
    assert_eq!(
        imported.report().boundary_build_status(),
        Some(RetainedTopologyStatus::NativeExact)
    );
    assert_eq!(
        imported.report().boundary_build_source_contour_count(),
        Some(2)
    );
    assert_eq!(
        imported
            .report()
            .boundary_build_validation_intersection_event_count(),
        Some(0)
    );
}

#[test]
fn svg_region_import_rejects_open_subpath_with_report() {
    let imported = import_svg_region_path_data_with_report(
        "M 0 0 L 4 0 L 4 4 Z M 10 10 L 11 10",
        FillRule::NonZero,
        22,
        0,
        None,
        &hypercurve::CurvePolicy::certified(),
    );

    assert!(imported.region().is_none());
    assert_eq!(imported.report().subpath_count(), 2);
    assert_eq!(imported.report().path_reports().len(), 2);
    assert_eq!(imported.report().closure_reports().len(), 1);
    assert_eq!(
        imported.report().status(),
        RetainedTopologyStatus::Unsupported
    );
    assert!(imported.report().lossy_boundary());
    assert_eq!(
        imported.report().blocker(),
        Some(UncertaintyReason::Unsupported)
    );
}

#[test]
fn retained_svg_import_records_are_named_boundaries() {
    let record = retained_svg_import_record(11, 5, None, 3, 2, 0).unwrap();

    assert_eq!(record.format(), RetainedImportFormat2::Svg);
    assert_eq!(record.source_index(), 11);
    assert_eq!(record.source_version(), 5);
    assert_eq!(
        record.topology_status(),
        RetainedTopologyStatus::ImportedLossy
    );
}
