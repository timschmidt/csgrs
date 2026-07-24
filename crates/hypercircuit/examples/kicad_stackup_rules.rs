//! Emit a KiCad board with exact physical stackup and custom-rule companion.

use std::path::PathBuf;

use hypercircuit::{
    AdapterKind, BoardId, BoardOutline, Circuit, CircuitId, KiCadExportOptions, Net, NetClass,
    NetClassId, NetId, PcbDesignRules, PcbLayout, PcbStackup, Real, StackupLayer, StackupLayerKind,
    TransientPolicy,
};
use hyperlattice::Point2;
use hyperpath::TraceLayer;

fn point(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let output = std::env::args()
        .nth(1)
        .ok_or("usage: kicad_stackup_rules OUTPUT.kicad_pcb")?;
    let board_path = PathBuf::from(output);
    let rule_path = board_path.with_extension("kicad_dru");
    let project_path = board_path.with_extension("kicad_pro");
    let signal = NetId::new("POWER")?;
    let circuit = Circuit::new(
        CircuitId::new("kicad-stackup-rules")?,
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: signal.clone(),
        is_ground: false,
    });
    let front = TraceLayer(0);
    let back = TraceLayer(1);
    let layout = PcbLayout {
        id: BoardId::new("kicad-stackup-rules")?,
        outline: BoardOutline {
            exterior: vec![point(0, 0), point(40, 0), point(40, 25), point(0, 25)].into(),
            cutouts: Vec::new(),
        },
        stackup: PcbStackup {
            layers: vec![
                StackupLayer {
                    name: "F.Cu".into(),
                    kind: StackupLayerKind::Conductor(front),
                    thickness: (Real::from(35) / Real::from(1_000))?,
                    material: Some("Copper".into()),
                },
                StackupLayer {
                    name: "dielectric 1".into(),
                    kind: StackupLayerKind::Dielectric,
                    thickness: (Real::from(153) / Real::from(100))?,
                    material: Some("FR4".into()),
                },
                StackupLayer {
                    name: "B.Cu".into(),
                    kind: StackupLayerKind::Conductor(back),
                    thickness: (Real::from(35) / Real::from(1_000))?,
                    material: Some("Copper".into()),
                },
            ],
        },
        land_patterns: Vec::new(),
        placements: Vec::new(),
        placement_constraints: Vec::new(),
        routes: Vec::new(),
        vias: Vec::new(),
        zones: Vec::new(),
        keepouts: Vec::new(),
        rules: PcbDesignRules {
            net_classes: vec![NetClass {
                id: NetClassId::new("power")?,
                parent: None,
                nets: vec![signal],
                min_trace_width: Some((Real::from(3) / Real::from(5))?),
                preferred_trace_width: Some((Real::from(4) / Real::from(5))?),
                min_clearance: Some((Real::one() / Real::from(4))?),
                preferred_via_land_diameter: Some((Real::from(6) / Real::from(5))?),
                preferred_via_drill_diameter: Some((Real::from(3) / Real::from(5))?),
                preferred_via_style: None,
                max_length: Some(Real::from(100)),
                max_via_count: Some(4),
                target_impedance_ohms: None,
                impedance_tolerance_ohms: None,
                requires_reference_plane: false,
            }],
            ..PcbDesignRules::default()
        },
    };
    let report = layout.export_kicad(&circuit, KiCadExportOptions::default())?;
    std::fs::write(&board_path, report.board)?;
    let rules = report
        .design_rules
        .ok_or("net-class policy produced no KiCad custom rules")?;
    let project = report
        .project
        .ok_or("net-class policy produced no KiCad project settings")?;
    std::fs::write(&rule_path, rules)?;
    std::fs::write(&project_path, project)?;
    println!(
        "{} + {} + {}: projections={}, omissions={}",
        board_path.display(),
        rule_path.display(),
        project_path.display(),
        report.numeric_projections.len(),
        report.omissions.len()
    );
    Ok(())
}
