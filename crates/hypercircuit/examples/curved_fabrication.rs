//! Release a board with exact cubic exterior/cutout truth through an explicit
//! bounded CAM projection policy.

use hypercircuit::{
    AdapterKind, BoardContour, BoardId, BoardOutline, CircuitId, Design, FabricationExportOptions,
    MaterializationProjection, PcbDesignRules, PcbLayout, PcbStackup, Real,
    ReleasePreparationOptions, StackupLayer, StackupLayerKind, TransientPolicy,
};
use hyperlattice::Point2;
use hyperpath::{CubicBezier, LinePathSegment, TraceLayer};

fn point(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn main() {
    let exterior = BoardContour::from_segments(vec![
        LinePathSegment::new(point(0, 0), point(40, 0)).into(),
        LinePathSegment::new(point(40, 0), point(40, 25)).into(),
        CubicBezier::new(point(40, 25), point(40, 30), point(0, 30), point(0, 25)).into(),
        LinePathSegment::new(point(0, 25), point(0, 0)).into(),
    ]);
    let cutout = BoardContour::from_segments(vec![
        LinePathSegment::new(point(15, 10), point(25, 10)).into(),
        LinePathSegment::new(point(25, 10), point(25, 15)).into(),
        CubicBezier::new(point(25, 15), point(25, 18), point(15, 18), point(15, 15)).into(),
        LinePathSegment::new(point(15, 15), point(15, 10)).into(),
    ]);
    let layout = PcbLayout {
        id: BoardId::new("curved-fabrication").unwrap(),
        outline: BoardOutline {
            exterior,
            cutouts: vec![cutout],
        },
        stackup: PcbStackup {
            layers: vec![StackupLayer {
                name: "F.Cu".into(),
                kind: StackupLayerKind::Conductor(TraceLayer(0)),
                thickness: (Real::from(35) / Real::from(1000)).unwrap(),
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
    };
    let checked = Design::from_layout(CircuitId::new("curved-fabrication").unwrap(), layout)
        .transient_policy(TransientPolicy::Static)
        .adapter_policy(AdapterKind::Dc)
        .finish()
        .unwrap();
    let release = checked
        .prepare_release(ReleasePreparationOptions {
            fabrication: FabricationExportOptions::millimeters()
                .with_cubic_contour_chord_error(0.01),
            ..ReleasePreparationOptions::default()
        })
        .unwrap();

    let projections = release
        .fabrication
        .manifest
        .geometry_projections
        .iter()
        .filter(|projection| {
            matches!(
                projection,
                MaterializationProjection::CubicBezierBoardContourPolyline { .. }
            )
        })
        .count();
    assert_eq!(projections, 2);
    assert!(release.cam_round_trip.is_release_clean());
    println!(
        "{} fabrication files; {projections} explicit cubic CAM projections",
        release.fabrication.files.len()
    );
}
