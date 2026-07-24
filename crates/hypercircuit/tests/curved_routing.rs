#![cfg(all(feature = "geometry", feature = "interchange", feature = "lceda"))]

use hypercircuit::{
    AdapterKind, BoardId, BoardOutline, Circuit, CircuitId, FabricationPackage,
    KiCadExportOmission, KiCadExportOptions, KiCadImportOptions, KiCadImportReport,
    LayoutValidationIssue, LcedaExportOmission, LcedaProExportOptions, LcedaProExportReport,
    MaterializationOptions, Net, NetId, PcbDesignRules, PcbLayout, PcbRoute, PcbStackup, Real,
    RouteId, RoutingProblemReport, RoutingSolution, SemanticDocument, StackupLayer,
    StackupLayerKind, TransientPolicy,
};
use hyperlattice::Point2;
use hyperpath::{ArcDirection, CubicBezier, ExplicitCircularArc, LinePathSegment, TraceLayer};

fn p(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn fixture() -> (Circuit, PcbLayout) {
    let net = NetId::new("RF").unwrap();
    let circuit = Circuit::new(
        CircuitId::new("curved-route").unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: net.clone(),
        is_ground: false,
    });
    let layer = TraceLayer(0);
    let arc =
        ExplicitCircularArc::new(p(5, 5), Real::from(5), p(5, 0), p(10, 5), ArcDirection::Ccw)
            .unwrap();
    let layout = PcbLayout {
        id: BoardId::new("curved-route").unwrap(),
        outline: BoardOutline {
            exterior: vec![p(-2, -2), p(17, -2), p(17, 8), p(-2, 8)].into(),
            cutouts: Vec::new(),
        },
        stackup: PcbStackup {
            layers: vec![StackupLayer {
                name: "F.Cu".into(),
                kind: StackupLayerKind::Conductor(layer),
                thickness: Real::one(),
                material: None,
            }],
        },
        land_patterns: Vec::new(),
        placements: Vec::new(),
        placement_constraints: Vec::new(),
        routes: vec![PcbRoute {
            id: RouteId::new("rf-bend").unwrap(),
            net,
            layer,
            width: Real::one(),
            segments: vec![
                LinePathSegment::new(p(0, 0), p(5, 0)).into(),
                arc.into(),
                CubicBezier::new(p(10, 5), p(11, 6), p(14, 6), p(15, 5)).into(),
            ],
        }],
        vias: Vec::new(),
        zones: Vec::new(),
        keepouts: Vec::new(),
        rules: PcbDesignRules::default(),
    };
    (circuit, layout)
}

#[test]
fn exact_mixed_line_arc_bezier_route_survives_native_workflow_boundaries() {
    let (circuit, layout) = fixture();
    assert!(layout.validate(&circuit).is_valid());

    let routing = RoutingProblemReport::from_layout(&circuit, &layout).unwrap();
    assert_eq!(routing.problem.existing.traces().len(), 1);
    assert_eq!(routing.problem.existing.arcs().len(), 1);
    assert_eq!(routing.problem.existing.beziers().len(), 1);
    let accepted =
        RoutingSolution::from_hyperpath(&routing.problem, &routing.problem.existing).unwrap();
    assert_eq!(accepted.routes.len(), 3);
    assert!(accepted.omissions.is_empty());

    let materialized = layout
        .materialize(
            &circuit,
            MaterializationOptions {
                route_bezier_chord_error: 0.05,
                ..MaterializationOptions::default()
            },
        )
        .unwrap();
    assert_eq!(materialized.copper_features.len(), 1);
    assert!(materialized.copper_layers[0].copper.is_some());
    assert!(materialized.projections.iter().any(|projection| matches!(
        projection,
        hypercircuit::MaterializationProjection::CircularRoutePolyline { source, .. }
            if source == "rf-bend"
    )));
    assert!(materialized.projections.iter().any(|projection| matches!(
        projection,
        hypercircuit::MaterializationProjection::CubicBezierRoutePolyline { source, .. }
            if source == "rf-bend"
    )));
    let fabrication = FabricationPackage::from_materialization(&layout, &materialized).unwrap();
    assert!(fabrication.verify_integrity().is_empty());
    assert_eq!(
        fabrication.manifest.geometry_projections,
        materialized.projections
    );

    let svg = layout
        .to_svg(&circuit, hypercircuit::PcbSvgOptions::default())
        .unwrap();
    assert!(svg.svg.contains(" A "));
    assert!(svg.svg.contains(" C "));
    assert!(svg.svg.contains("data-source=\"rf-bend\""));

    let document = SemanticDocument::new(circuit.clone(), None)
        .unwrap()
        .with_pcb(layout.clone())
        .unwrap();
    let json = document.to_json_pretty().unwrap();
    assert!(json.contains("\"kind\": \"circular-arc\""));
    assert!(json.contains("\"kind\": \"cubic-bezier\""));
    assert_eq!(SemanticDocument::from_json(&json).unwrap(), document);

    let kicad = layout
        .export_kicad(&circuit, KiCadExportOptions::default())
        .unwrap();
    assert!(
        !kicad
            .omissions
            .iter()
            .any(|omission| matches!(omission, KiCadExportOmission::CircularRouteArc { .. }))
    );
    assert!(kicad.omissions.iter().any(|omission| matches!(
        omission,
        KiCadExportOmission::CubicRouteBezier { route, segment: 2 }
            if route == "rf-bend"
    )));
    assert!(kicad.board.contains("(arc "));
    let imported = KiCadImportReport::from_str(
        &kicad.board,
        KiCadImportOptions::new(
            CircuitId::new("curved-import").unwrap(),
            BoardId::new("curved-import").unwrap(),
            Real::one(),
        ),
    )
    .unwrap();
    assert_eq!(imported.layout.routes.len(), 2);
    assert!(imported.layout.routes.iter().any(|route| matches!(
        route.segments.as_slice(),
        [hypercircuit::PcbRouteSegment::CircularArc(_)]
    )));

    let lceda = LcedaProExportReport::from_design(
        &circuit,
        None,
        Some(&layout),
        LcedaProExportOptions::millimeters(),
    )
    .unwrap();
    assert!(lceda.omissions.iter().any(|omission| matches!(
        omission,
        LcedaExportOmission::CircularRouteArc { route, segment: 1 }
            if route == "rf-bend"
    )));
    assert!(lceda.omissions.iter().any(|omission| matches!(
        omission,
        LcedaExportOmission::CubicRouteBezier { route, segment: 2 }
            if route == "rf-bend"
    )));
}

#[test]
fn route_width_must_be_smaller_than_an_arc_diameter() {
    let (circuit, mut layout) = fixture();
    layout.routes[0].width = Real::from(10);

    assert!(
        layout
            .validate(&circuit)
            .issues
            .iter()
            .any(|issue| matches!(
                issue,
                LayoutValidationIssue::InvalidRouteArcWidth(route) if route.as_str() == "rf-bend"
            ))
    );
}
