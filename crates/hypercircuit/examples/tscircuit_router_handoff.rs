//! Export a checked board to ts-circuit `SimpleRouteJson` and restore a solver result.

use hypercircuit::{
    BoardOutline, Design, Footprint, PcbStackup, Plating, Real, RoutingProblemReport,
    TscircuitRoutingExportOptions, TscircuitRoutingImportOptions, TscircuitRoutingImportReport,
    parts,
};
use hyperlattice::Point2;
use hyperpath::TraceLayer;
use serde_json::json;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut design = Design::new(
        "tscircuit-router-handoff",
        BoardOutline::rectangle(Real::from(20), Real::from(10)),
        PcbStackup::two_layer(
            (Real::from(35) / Real::from(1_000))?,
            (Real::from(153) / Real::from(100))?,
            Some("copper".into()),
            Some("FR4".into()),
        ),
    )?;
    let signal = design.signal("SIGNAL")?;
    let ground = design.ground("GND")?;
    let footprint =
        || Footprint::two_pad_smd(Real::one(), Real::one(), Real::one(), vec![TraceLayer(0)]);
    let left = design.add(
        parts::resistor("R1", Real::from(1_000))
            .footprint(footprint())
            .at(Point2::new(Real::from(5), Real::from(5))),
    )?;
    let right = design.add(
        parts::resistor("R2", Real::from(1_000))
            .footprint(footprint())
            .at(Point2::new(Real::from(15), Real::from(5))),
    )?;
    design.connect(&signal, [left.pin("2")?, right.pin("1")?])?;
    design.connect(&ground, [left.pin("1")?, right.pin("2")?])?;
    let checked = design.finish()?;

    let handoff = RoutingProblemReport::from_layout(&checked.circuit, &checked.layout)?;
    let exported = handoff.export_tscircuit_simple_route_json(
        &checked.layout,
        TscircuitRoutingExportOptions {
            fallback_min_trace_width: Some(Real::one()),
            ..TscircuitRoutingExportOptions::default()
        },
    )?;
    let mut solver_document: serde_json::Value = serde_json::from_str(&exported.json)?;
    solver_document["traces"] = json!([
        {
            "type": "pcb_trace",
            "pcb_trace_id": "SIGNAL",
            "route": [
                {"route_type": "wire", "x": 6, "y": 5, "width": 1, "layer": "F.Cu"},
                {"route_type": "wire", "x": 14, "y": 5, "width": 1, "layer": "F.Cu"}
            ]
        },
        {
            "type": "pcb_trace",
            "pcb_trace_id": "GND",
            "route": [
                {"route_type": "wire", "x": 4, "y": 5, "width": 1, "layer": "F.Cu"},
                {"route_type": "wire", "x": 16, "y": 5, "width": 1, "layer": "F.Cu"}
            ]
        }
    ]);
    let imported = TscircuitRoutingImportReport::from_str(
        &handoff.problem,
        &serde_json::to_string(&solver_document)?,
        TscircuitRoutingImportOptions {
            via_land_diameter: Real::one(),
            via_drill_diameter: (Real::one() / Real::from(2))?,
            via_plating: Plating::Plated,
        },
    )?;
    let routed = imported.solution.append_to(&checked.layout)?;
    assert!(routed.validate(&checked.circuit).is_valid());

    println!(
        "{} connections, {} obstacles, {} projections, {} omissions -> {} routes",
        solver_document["connections"]
            .as_array()
            .map_or(0, Vec::len),
        solver_document["obstacles"].as_array().map_or(0, Vec::len),
        exported.projections.len(),
        exported.omissions.len(),
        imported.solution.routes.len(),
    );
    Ok(())
}
