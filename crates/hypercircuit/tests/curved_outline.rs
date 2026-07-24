#![cfg(feature = "interchange")]

use hypercircuit::{
    AdapterKind, BoardContour, BoardContourSegment, BoardId, BoardOutline, Circuit, CircuitId,
    Design, FabricationExportOptions, FabricationFileKind, FabricationPackage, KiCadExportOmission,
    KiCadExportOptions, KiCadImportOptions, KiCadImportReport, MaterializationOptions,
    MaterializationProjection, NegotiatedRoutePolicy, NegotiatedRouteStatus, PcbDesignRules,
    PcbLayout, PcbStackup, PlacementSolvePolicy, Real, SemanticDocument, StackupLayer,
    StackupLayerKind, TransientPolicy,
};
use hypercurve::{Classification, CurvePolicy, RegionPointLocation, UncertaintyReason};
use hyperlattice::Point2;
use hyperpath::{ArcDirection, CubicBezier, ExplicitCircularArc, LinePathSegment, TraceLayer};

#[cfg(feature = "lceda")]
use hypercircuit::{LcedaExportOmission, LcedaProExportOptions, LcedaProExportReport};

fn p(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn circuit() -> Circuit {
    Circuit::new(
        CircuitId::new("curved-outline").unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
}

fn exterior() -> BoardContour {
    BoardContour::from_segments(vec![
        LinePathSegment::new(p(0, 0), p(10, 0)).into(),
        LinePathSegment::new(p(10, 0), p(10, 10)).into(),
        ExplicitCircularArc::new(
            p(5, 10),
            Real::from(5),
            p(10, 10),
            p(0, 10),
            ArcDirection::Ccw,
        )
        .unwrap()
        .into(),
        LinePathSegment::new(p(0, 10), p(0, 0)).into(),
    ])
}

fn cubic_cutout() -> BoardContour {
    BoardContour::from_segments(vec![
        LinePathSegment::new(p(2, 2), p(4, 2)).into(),
        LinePathSegment::new(p(4, 2), p(4, 4)).into(),
        CubicBezier::new(p(4, 4), p(4, 5), p(2, 5), p(2, 4)).into(),
        LinePathSegment::new(p(2, 4), p(2, 2)).into(),
    ])
}

fn cubic_exterior() -> BoardContour {
    BoardContour::from_segments(vec![
        LinePathSegment::new(p(0, 0), p(10, 0)).into(),
        LinePathSegment::new(p(10, 0), p(10, 10)).into(),
        CubicBezier::new(p(10, 10), p(10, 12), p(0, 12), p(0, 10)).into(),
        LinePathSegment::new(p(0, 10), p(0, 0)).into(),
    ])
}

fn layout(with_cubic_cutout: bool) -> PcbLayout {
    PcbLayout {
        id: BoardId::new("curved-outline").unwrap(),
        outline: BoardOutline {
            exterior: exterior(),
            cutouts: with_cubic_cutout.then(cubic_cutout).into_iter().collect(),
        },
        stackup: PcbStackup {
            layers: vec![StackupLayer {
                name: "F.Cu".into(),
                kind: StackupLayerKind::Conductor(TraceLayer(0)),
                thickness: Real::one(),
                material: Some("copper".into()),
            }],
        },
        land_patterns: Vec::new(),
        placements: Vec::new(),
        placement_constraints: Vec::new(),
        routes: Vec::new(),
        vias: Vec::new(),
        zones: Vec::new(),
        keepouts: Vec::new(),
        rules: PcbDesignRules::default(),
    }
}

#[test]
fn mixed_curve_board_contours_round_trip_and_render_without_chord_loss() {
    let circuit = circuit();
    let layout = layout(true);
    assert!(layout.validate(&circuit).is_valid());

    let document = SemanticDocument::new(circuit.clone(), None)
        .unwrap()
        .with_pcb(layout.clone())
        .unwrap();
    let json = document.to_json_pretty().unwrap();
    assert!(json.contains("\"circular-arc\""));
    assert!(json.contains("\"cubic-bezier\""));
    assert_eq!(SemanticDocument::from_json(&json).unwrap(), document);

    let svg = layout.to_svg(&circuit, Default::default()).unwrap();
    assert!(svg.svg.contains(" A 5 5 "));
    assert!(svg.svg.contains(" C "));

    let kicad = layout
        .export_kicad(&circuit, KiCadExportOptions::default())
        .unwrap();
    assert!(kicad.board.contains("(gr_arc "));
    assert!(
        kicad
            .omissions
            .contains(&KiCadExportOmission::CubicBoardContourBezier {
                contour: "cutout[0]".into(),
                segment: 2,
            })
    );
    let imported = KiCadImportReport::from_str(
        &kicad.board,
        KiCadImportOptions::new(
            CircuitId::new("curved-import").unwrap(),
            BoardId::new("curved-import").unwrap(),
            Real::one(),
        ),
    )
    .unwrap();
    assert!(
        imported
            .layout
            .outline
            .exterior
            .segments()
            .iter()
            .any(|segment| matches!(segment, BoardContourSegment::CircularArc(_)))
    );

    let placement = layout.solve_placement(&circuit, &PlacementSolvePolicy::default());
    assert!(placement.issues.is_empty(), "{:?}", placement.issues);
    let boundary = layout.outline.boundary_geometry().unwrap();
    assert_eq!(
        boundary
            .classify_point(&p(3, 3), &CurvePolicy::certified())
            .unwrap(),
        Classification::Decided(RegionPointLocation::Outside)
    );
    assert_eq!(
        boundary
            .contains_axis_aligned_box(&p(5, 5), &p(7, 7), &CurvePolicy::certified())
            .unwrap(),
        Classification::Decided(true)
    );
    assert_eq!(
        boundary
            .contains_axis_aligned_box(&p(1, 1), &p(3, 3), &CurvePolicy::certified())
            .unwrap(),
        Classification::Decided(false)
    );
    let mut arc_layout = layout.clone();
    arc_layout.outline.cutouts.clear();
    let arc_boundary = arc_layout.outline.boundary_geometry().unwrap();
    let disc = arc_boundary
        .contains_disc(&p(5, 5), Real::one(), &CurvePolicy::certified())
        .unwrap();
    assert_eq!(disc, Classification::Decided(true), "{disc:?}");
    assert_eq!(
        arc_boundary
            .contains_disc(&p(5, 14), Real::from(2), &CurvePolicy::certified())
            .unwrap(),
        Classification::Decided(false)
    );
    assert_eq!(
        arc_boundary
            .contains_segment(&p(2, 2), &p(8, 2), Real::one(), &CurvePolicy::certified())
            .unwrap(),
        Classification::Decided(true)
    );
    assert_eq!(
        boundary
            .contains_disc(&p(6, 6), Real::one(), &CurvePolicy::certified())
            .unwrap(),
        Classification::Uncertain(UncertaintyReason::Unsupported)
    );
    assert_eq!(
        layout
            .negotiated_autoroute(&circuit, NegotiatedRoutePolicy::default())
            .unwrap()
            .status,
        NegotiatedRouteStatus::Complete
    );
}

#[test]
fn disconnected_mixed_contours_fail_structural_validation() {
    let circuit = circuit();
    let mut layout = layout(false);
    layout.outline.exterior = BoardContour::from_segments(vec![
        LinePathSegment::new(p(0, 0), p(10, 0)).into(),
        LinePathSegment::new(p(10, 1), p(0, 10)).into(),
        LinePathSegment::new(p(0, 10), p(0, 0)).into(),
    ]);
    assert!(
        layout
            .validate(&circuit)
            .issues
            .contains(&hypercircuit::LayoutValidationIssue::InvalidBoardContour)
    );
    assert!(matches!(
        SemanticDocument::new(circuit, None)
            .unwrap()
            .with_pcb(layout),
        Err(hypercircuit::SemanticInterchangeError::InvalidPcb { .. })
    ));
}

#[cfg(feature = "geometry")]
#[test]
fn arc_board_profile_materializes_exactly_and_emits_cam_arcs() {
    let circuit = circuit();
    let layout = layout(false);
    let materialized = layout
        .materialize(&circuit, MaterializationOptions::default())
        .unwrap();
    assert!(!materialized.substrate.as_curve_region().is_empty());

    #[cfg(feature = "drc")]
    {
        let handoff = hypercircuit::HyperDrcHandoff::from_materialization(&layout, &materialized);
        assert!(
            !handoff
                .board
                .board_outline
                .as_ref()
                .unwrap()
                .as_curve_region()
                .is_empty()
        );
        let readiness = handoff.run_readiness(&hypercircuit::DrcReadinessPolicy::default());
        assert!(readiness.is_release_clean(), "{:?}", readiness.violations);

        let legend = hyperdrc::PcbSketch::new(
            csgrs::sketch::Profile::polygon_points(&[
                hypercurve::Point2::new(Real::from(5), Real::from(12)),
                hypercurve::Point2::new(Real::from(6), Real::from(12)),
                hypercurve::Point2::new(Real::from(6), Real::from(13)),
                hypercurve::Point2::new(Real::from(5), Real::from(13)),
            ]),
            None,
        );
        assert!(
            hyperdrc::checks::silkscreen_board_edge_clearance(
                "legend",
                &legend,
                "board-outline",
                handoff.board.board_outline.as_ref().unwrap(),
                &hyperdrc::scalar::scalar("0.25"),
                &hyperdrc::scalar::scalar("1.0e-9"),
            )
            .is_empty()
        );
    }

    let package = FabricationPackage::from_materialization(&layout, &materialized).unwrap();
    let profile = package
        .files
        .iter()
        .find(|file| {
            file.kind == FabricationFileKind::GerberX2 && file.name.ends_with("-Profile.gbr")
        })
        .unwrap();
    let profile = std::str::from_utf8(&profile.bytes).unwrap();
    assert!(profile.contains("G75*"));
    assert!(profile.contains("G03*"));

    #[cfg(feature = "drc")]
    {
        let audit = package.audit_cam_round_trip();
        assert!(audit.is_release_clean(), "{:?}", audit.issues);
    }
}

#[cfg(feature = "geometry")]
#[test]
fn cubic_board_profile_requires_an_explicit_cam_projection_policy() {
    let circuit = circuit();
    let layout = layout(true);
    let materialized = layout
        .materialize(&circuit, MaterializationOptions::default())
        .unwrap();
    let error = FabricationPackage::from_materialization(&layout, &materialized).unwrap_err();
    assert!(
        error
            .to_string()
            .contains("explicit fabrication projection policy")
    );

    let mut projected_layout = layout;
    projected_layout.outline.exterior = cubic_exterior();
    let materialized = projected_layout
        .materialize(&circuit, MaterializationOptions::default())
        .unwrap();
    for chord_error in [0.0, f64::NAN] {
        let error = FabricationPackage::from_materialization_with_options(
            &projected_layout,
            &materialized,
            FabricationExportOptions::millimeters().with_cubic_contour_chord_error(chord_error),
        )
        .unwrap_err();
        assert!(error.to_string().contains("finite positive chord error"));
    }
    let package = FabricationPackage::from_materialization_with_options(
        &projected_layout,
        &materialized,
        FabricationExportOptions::millimeters().with_cubic_contour_chord_error(0.01),
    )
    .unwrap();
    let evidence = package
        .manifest
        .geometry_projections
        .iter()
        .filter_map(|projection| match projection {
            MaterializationProjection::CubicBezierBoardContourPolyline {
                source,
                segment,
                chord_error,
                generated_segments,
            } => Some((source, segment, chord_error, generated_segments)),
            _ => None,
        })
        .collect::<Vec<_>>();
    assert_eq!(evidence.len(), 2);
    assert!(
        evidence
            .iter()
            .any(|(source, segment, chord_error, generated)| {
                source.as_str() == "board.exterior"
                    && **segment == 2
                    && chord_error.as_str() == "0.01"
                    && **generated > 1
            })
    );
    assert!(
        evidence
            .iter()
            .any(|(source, segment, chord_error, generated)| {
                source.as_str() == "board.cutout[0]"
                    && **segment == 2
                    && chord_error.as_str() == "0.01"
                    && **generated > 1
            })
    );
    let profile = package
        .files
        .iter()
        .find(|file| {
            file.kind == FabricationFileKind::GerberX2 && file.name.ends_with("-Profile.gbr")
        })
        .unwrap();
    let profile = std::str::from_utf8(&profile.bytes).unwrap();
    assert!(profile.matches("G01*X").count() > 8);

    #[cfg(feature = "drc")]
    {
        let audit = package.audit_cam_round_trip();
        assert!(audit.is_release_clean(), "{:?}", audit.issues);

        let checked =
            Design::from_layout(CircuitId::new("curved-release").unwrap(), projected_layout)
                .finish()
                .unwrap();
        let release = checked
            .prepare_release(hypercircuit::ReleasePreparationOptions {
                fabrication: FabricationExportOptions::millimeters()
                    .with_cubic_contour_chord_error(0.01),
                ..hypercircuit::ReleasePreparationOptions::default()
            })
            .unwrap();
        assert_eq!(
            release
                .fabrication
                .manifest
                .geometry_projections
                .iter()
                .filter(|projection| matches!(
                    projection,
                    MaterializationProjection::CubicBezierBoardContourPolyline { .. }
                ))
                .count(),
            2
        );
        assert!(release.cam_round_trip.is_release_clean());
    }
}

#[cfg(feature = "lceda")]
#[test]
fn lceda_curved_contour_chords_are_explicitly_audited() {
    let circuit = circuit();
    let layout = layout(true);
    let report = LcedaProExportReport::from_design(
        &circuit,
        None,
        Some(&layout),
        LcedaProExportOptions::millimeters(),
    )
    .unwrap();
    assert!(report.omissions.iter().any(|omission| matches!(
        omission,
        LcedaExportOmission::CurvedBoardContourChord { contour, segment: 2 }
            if contour == "board.exterior"
    )));
    assert!(report.omissions.iter().any(|omission| matches!(
        omission,
        LcedaExportOmission::CurvedBoardContourChord { contour, segment: 2 }
            if contour == "board.cutout[0]"
    )));
}
