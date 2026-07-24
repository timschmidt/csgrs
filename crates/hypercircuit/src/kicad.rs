//! Reviewable KiCad PCB export from retained circuit and layout intent.
//!
//! KiCad is a finite decimal interchange boundary. Every exact scalar written
//! to the file is recorded in [`KiCadNumericProjection`], and unsupported
//! semantic details are returned as explicit omissions.

use std::collections::BTreeMap;
use std::fmt::{Display, Formatter, Write};

use hyperpath::{ArcDirection, ExplicitArcSweepClass, ExplicitCircularArc, TraceLayer};
use hyperreal::Real;

use crate::{
    BoardSide, Circuit, CopperZoneConnection, CopperZoneFill, DrillShape, LandPatternGraphic,
    LandPatternGraphicPrimitive, LandPatternPad, LayerRole, NetId, PadShape, PcbLayout, Plating,
    StackupLayerKind,
};

/// Finite-decimal output policy for KiCad interchange.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct KiCadExportOptions {
    /// Digits written after the decimal point before trailing-zero removal.
    pub decimal_places: usize,
}

impl Default for KiCadExportOptions {
    fn default() -> Self {
        Self { decimal_places: 9 }
    }
}

/// Audit record for one exact scalar projected into KiCad decimal syntax.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct KiCadNumericProjection {
    /// Semantic source field.
    pub field: String,
    /// Exact-aware source spelling.
    pub source: String,
    /// Decimal token written to the board.
    pub emitted: String,
}

/// One retained net class lowered into a KiCad custom-design-rule companion.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct KiCadDesignRuleProjection {
    /// Stable hypercircuit net-class identity.
    pub net_class: String,
    /// Logical nets selected directly by the emitted KiCad condition.
    pub nets: Vec<String>,
    /// KiCad constraint tokens emitted for the class.
    pub constraints: Vec<String>,
    /// Whether inherited values were flattened into the emitted effective rule.
    pub flattened_inheritance: bool,
}

/// One retained net class projected into native KiCad project settings.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct KiCadProjectNetClassProjection {
    /// Stable hypercircuit net-class identity.
    pub net_class: String,
    /// Exact logical net names assigned in `netclass_assignments`.
    pub nets: Vec<String>,
    /// Native KiCad class fields emitted for this class.
    pub fields: Vec<String>,
    /// Whether inherited values were flattened into the emitted class.
    pub flattened_inheritance: bool,
}

/// Retained intent not represented by the current KiCad writer.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum KiCadExportOmission {
    /// Authored rounded-rectangle radius is currently emitted using KiCad's default ratio.
    RoundedRectangleRadius { land_pattern: String, pad: String },
    /// A non-centered or non-axis-aligned routed slot was projected to its cutter diameter.
    RoutedSlotProjectedToRound { land_pattern: String, pad: String },
    /// Per-pad mask or paste margin is retained but not written yet.
    PadMaskOrPasteMargin { land_pattern: String, pad: String },
    /// Keepout source geometry needs a typed KiCad keepout mapping.
    Keepouts(usize),
    /// Relational placement constraints are resolved externally but not written.
    PlacementConstraints(usize),
    /// A physical stackup layer kind was projected to KiCad's dielectric vocabulary.
    StackupLayerKindLowered { layer: String, kind: String },
    /// A physical layer name was projected to KiCad's required canonical stackup name.
    StackupLayerNameLowered { layer: String, emitted: String },
    /// Net-class inheritance was flattened into one effective custom rule.
    NetClassInheritanceLowered { net_class: String },
    /// A named via construction has no native KiCad net-class identity mapping.
    NetClassViaStylePreference {
        net_class: String,
        via_style: String,
    },
    /// Impedance/reference-plane intent remains retained outside the KiCad rule subset.
    NetClassSignalIntegrityPolicy { net_class: String },
    /// A net name could not be embedded safely in a KiCad rule expression.
    DesignRuleNetName { net_class: String, net: String },
    /// These retained rule families remain outside the current KiCad companion subset.
    AdvancedDesignRules {
        via_styles: usize,
        differential_pairs: usize,
        route_constraint_regions: usize,
        route_rule_regions: usize,
        escape_policies: usize,
        length_tuning_patterns: usize,
        phase_tuning_groups: usize,
    },
    /// Artwork omitted a stroke width, so the review writer applied its documented default.
    DefaultGraphicStroke {
        land_pattern: String,
        graphic: String,
    },
    /// Explicit via mask opening/tenting intent is retained but not written yet.
    ViaMaskIntent { via: String },
    /// Full-circle or uncertified route arc cannot become one KiCad midpoint arc.
    CircularRouteArc { route: String, segment: usize },
    /// Cubic route centerline has no native KiCad PCB track primitive.
    CubicRouteBezier { route: String, segment: usize },
    /// Cubic board edge remains retained because this writer has not certified KiCad curve syntax.
    CubicBoardContourBezier { contour: String, segment: usize },
    /// KiCad's zone record does not retain a non-four thermal spoke count.
    ZoneThermalSpokeCount { zone: String, spoke_count: u8 },
    /// Stitching generation was capped or contained an indeterminate/id-collision decision.
    ZoneStitchingIncomplete { zone: String },
    /// Generated vias were emitted, but KiCad cannot retain their declarative source policy.
    ZoneStitchingPolicyLowered { zone: String, generated_vias: usize },
}

/// KiCad board text plus an auditable finite/loss boundary.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct KiCadExportReport {
    /// Standalone `.kicad_pcb` content.
    pub board: String,
    /// Optional `.kicad_dru` companion containing retained enforceable rules.
    pub design_rules: Option<String>,
    /// Optional `.kicad_pro` companion containing native class defaults and assignments.
    pub project: Option<String>,
    /// Every exact-to-decimal scalar projection.
    pub numeric_projections: Vec<KiCadNumericProjection>,
    /// Net-class-to-custom-rule lowering evidence.
    pub design_rule_projections: Vec<KiCadDesignRuleProjection>,
    /// Net-class-to-project-settings lowering evidence.
    pub project_net_class_projections: Vec<KiCadProjectNetClassProjection>,
    /// Semantic details not represented in `board`.
    pub omissions: Vec<KiCadExportOmission>,
}

/// Failure before a structurally meaningful KiCad board can be written.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum KiCadExportError {
    /// Circuit/layout validation failed.
    InvalidDesign,
    /// An exact scalar could not be projected to a finite KiCad token.
    NonFiniteScalar(String),
    /// Formatting into the owned string failed unexpectedly.
    Formatting,
}

impl Display for KiCadExportError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidDesign => formatter.write_str("cannot export an invalid circuit layout"),
            Self::NonFiniteScalar(field) => {
                write!(formatter, "KiCad scalar projection is non-finite: {field}")
            }
            Self::Formatting => formatter.write_str("KiCad board formatting failed"),
        }
    }
}

impl std::error::Error for KiCadExportError {}

impl PcbLayout {
    /// Exports a standalone review board without claiming lossless KiCad round trip.
    pub fn export_kicad(
        &self,
        circuit: &Circuit,
        options: KiCadExportOptions,
    ) -> Result<KiCadExportReport, KiCadExportError> {
        if options.decimal_places == 0
            || !circuit.validate().is_valid()
            || !self.validate(circuit).is_valid()
        {
            return Err(KiCadExportError::InvalidDesign);
        }
        let mut emitter = Emitter::new(options);
        let net_numbers = circuit
            .nets
            .iter()
            .enumerate()
            .map(|(index, net)| (net.id.clone(), index + 1))
            .collect::<BTreeMap<_, _>>();
        let conductor_layers = conductor_layers(self);
        let thickness = self
            .stackup
            .layers
            .iter()
            .fold(Real::zero(), |sum, layer| sum + layer.thickness.clone());

        writeln!(emitter.board, "(kicad_pcb").map_err(|_| KiCadExportError::Formatting)?;
        writeln!(emitter.board, "  (version 20240108)")
            .map_err(|_| KiCadExportError::Formatting)?;
        writeln!(emitter.board, "  (generator \"hypercircuit\")")
            .map_err(|_| KiCadExportError::Formatting)?;
        let thickness = emitter.number("stackup.finished_thickness", &thickness)?;
        writeln!(emitter.board, "  (general (thickness {thickness}))")
            .map_err(|_| KiCadExportError::Formatting)?;
        writeln!(emitter.board, "  (paper \"A4\")").map_err(|_| KiCadExportError::Formatting)?;
        writeln!(emitter.board, "  (layers").map_err(|_| KiCadExportError::Formatting)?;
        for (slot, name, _) in &conductor_layers {
            writeln!(emitter.board, "    ({slot} {} signal)", quote(name))
                .map_err(|_| KiCadExportError::Formatting)?;
        }
        writeln!(emitter.board, "    (36 \"B.SilkS\" user \"b.silkscreen\")")
            .map_err(|_| KiCadExportError::Formatting)?;
        writeln!(emitter.board, "    (37 \"F.SilkS\" user \"f.silkscreen\")")
            .map_err(|_| KiCadExportError::Formatting)?;
        writeln!(emitter.board, "    (34 \"B.Paste\" user)")
            .map_err(|_| KiCadExportError::Formatting)?;
        writeln!(emitter.board, "    (35 \"F.Paste\" user)")
            .map_err(|_| KiCadExportError::Formatting)?;
        writeln!(emitter.board, "    (38 \"B.Mask\" user)")
            .map_err(|_| KiCadExportError::Formatting)?;
        writeln!(emitter.board, "    (39 \"F.Mask\" user)")
            .map_err(|_| KiCadExportError::Formatting)?;
        writeln!(emitter.board, "    (44 \"Edge.Cuts\" user)")
            .map_err(|_| KiCadExportError::Formatting)?;
        writeln!(emitter.board, "    (46 \"B.Fab\" user)")
            .map_err(|_| KiCadExportError::Formatting)?;
        writeln!(emitter.board, "    (47 \"F.Fab\" user)")
            .map_err(|_| KiCadExportError::Formatting)?;
        writeln!(emitter.board, "    (48 \"B.CrtYd\" user)")
            .map_err(|_| KiCadExportError::Formatting)?;
        writeln!(emitter.board, "    (49 \"F.CrtYd\" user)")
            .map_err(|_| KiCadExportError::Formatting)?;
        writeln!(emitter.board, "  )").map_err(|_| KiCadExportError::Formatting)?;
        emit_stackup(&mut emitter, self)?;
        emit_design_rules(&mut emitter, self)?;
        emit_project_settings(&mut emitter, self)?;
        for net in &circuit.nets {
            writeln!(
                emitter.board,
                "  (net {} {})",
                net_numbers[&net.id],
                quote(net.id.as_str())
            )
            .map_err(|_| KiCadExportError::Formatting)?;
        }

        emit_contour(&mut emitter, "outline", &self.outline.exterior)?;
        for (index, cutout) in self.outline.cutouts.iter().enumerate() {
            emit_contour(&mut emitter, &format!("cutout[{index}]"), cutout)?;
        }

        for placement in &self.placements {
            let pattern = self
                .land_patterns
                .iter()
                .find(|pattern| pattern.id == placement.land_pattern)
                .expect("validated land pattern must exist");
            let instance = circuit
                .instances
                .iter()
                .find(|instance| instance.id == placement.instance)
                .expect("validated circuit instance must exist");
            let side_layer = match placement.side {
                BoardSide::Front => conductor_layers.first(),
                BoardSide::Back => conductor_layers.last(),
            }
            .map(|(_, name, _)| name.as_str())
            .unwrap_or("F.Cu");
            let x = emitter.number(
                &format!("placement.{}.x", placement.instance.as_str()),
                &placement.position.x,
            )?;
            let y = emitter.number(
                &format!("placement.{}.y", placement.instance.as_str()),
                &placement.position.y,
            )?;
            let rotation = emitter.number(
                &format!("placement.{}.rotation", placement.instance.as_str()),
                &placement.rotation_degrees,
            )?;
            writeln!(
                emitter.board,
                "  (footprint {} (layer {}) (at {x} {y} {rotation})",
                quote(pattern.id.as_str()),
                quote(side_layer)
            )
            .map_err(|_| KiCadExportError::Formatting)?;
            writeln!(
                emitter.board,
                "    (property \"Reference\" {})",
                quote(instance.id.as_str())
            )
            .map_err(|_| KiCadExportError::Formatting)?;
            for pad in &pattern.pads {
                emit_pad(
                    &mut emitter,
                    self,
                    pattern,
                    instance,
                    pad,
                    &net_numbers,
                    &conductor_layers,
                )?;
            }
            for graphic in &pattern.graphics {
                emit_land_pattern_graphic(
                    &mut emitter,
                    pattern,
                    graphic,
                    placement.side,
                    &conductor_layers,
                )?;
            }
            for (model_index, model) in pattern.models.iter().enumerate() {
                let prefix = format!("land_pattern.{}.models[{model_index}]", pattern.id.as_str());
                let x = emitter.number(&format!("{prefix}.offset_x"), &model.transform.offset_x)?;
                let y = emitter.number(&format!("{prefix}.offset_y"), &model.transform.offset_y)?;
                let z = emitter.number(&format!("{prefix}.offset_z"), &model.transform.offset_z)?;
                let rx = emitter.number(
                    &format!("{prefix}.rotate_x_degrees"),
                    &model.transform.rotate_x_degrees,
                )?;
                let ry = emitter.number(
                    &format!("{prefix}.rotate_y_degrees"),
                    &model.transform.rotate_y_degrees,
                )?;
                let rz = emitter.number(
                    &format!("{prefix}.rotate_z_degrees"),
                    &model.transform.rotate_z_degrees,
                )?;
                let sx = emitter.number(&format!("{prefix}.scale_x"), &model.transform.scale_x)?;
                let sy = emitter.number(&format!("{prefix}.scale_y"), &model.transform.scale_y)?;
                let sz = emitter.number(&format!("{prefix}.scale_z"), &model.transform.scale_z)?;
                writeln!(
                    emitter.board,
                    "    (model {} (offset (xyz {x} {y} {z})) (scale (xyz {sx} {sy} {sz})) (rotate (xyz {rx} {ry} {rz})))",
                    quote(&model.uri)
                )
                .map_err(|_| KiCadExportError::Formatting)?;
            }
            writeln!(emitter.board, "  )").map_err(|_| KiCadExportError::Formatting)?;
        }

        for route in &self.routes {
            for (index, segment) in route.segments.iter().enumerate() {
                let prefix = format!("route.{}.segment[{index}]", route.id.as_str());
                let x0 = emitter.number(&format!("{prefix}.start.x"), &segment.start().x)?;
                let y0 = emitter.number(&format!("{prefix}.start.y"), &segment.start().y)?;
                let x1 = emitter.number(&format!("{prefix}.end.x"), &segment.end().x)?;
                let y1 = emitter.number(&format!("{prefix}.end.y"), &segment.end().y)?;
                let width =
                    emitter.number(&format!("route.{}.width", route.id.as_str()), &route.width)?;
                match segment {
                    crate::PcbRouteSegment::Line(_) => writeln!(
                        emitter.board,
                        "  (segment (start {x0} {y0}) (end {x1} {y1}) (width {width}) (layer {}) (net {}))",
                        quote(layer_name(&conductor_layers, route.layer)),
                        net_numbers[&route.net]
                    )
                    .map_err(|_| KiCadExportError::Formatting)?,
                    crate::PcbRouteSegment::CircularArc(arc) => {
                        let Some(midpoint) = arc_midpoint(arc) else {
                            emitter.omissions.push(KiCadExportOmission::CircularRouteArc {
                                route: route.id.as_str().to_owned(),
                                segment: index,
                            });
                            continue;
                        };
                        let xm = emitter.number(&format!("{prefix}.mid.x"), &midpoint.x)?;
                        let ym = emitter.number(&format!("{prefix}.mid.y"), &midpoint.y)?;
                        writeln!(
                            emitter.board,
                            "  (arc (start {x0} {y0}) (mid {xm} {ym}) (end {x1} {y1}) (width {width}) (layer {}) (net {}))",
                            quote(layer_name(&conductor_layers, route.layer)),
                            net_numbers[&route.net]
                        )
                        .map_err(|_| KiCadExportError::Formatting)?;
                    }
                    crate::PcbRouteSegment::CubicBezier(_) => {
                        emitter
                            .omissions
                            .push(KiCadExportOmission::CubicRouteBezier {
                                route: route.id.as_str().to_owned(),
                                segment: index,
                            });
                    }
                }
            }
        }
        let stitching = self.realize_stitching_vias();
        for evidence in &stitching.evidence {
            emitter
                .omissions
                .push(KiCadExportOmission::ZoneStitchingPolicyLowered {
                    zone: evidence.zone.clone(),
                    generated_vias: evidence.accepted,
                });
            if evidence.truncated
                || evidence.rejected.indeterminate != 0
                || evidence.rejected.id_collision != 0
            {
                emitter
                    .omissions
                    .push(KiCadExportOmission::ZoneStitchingIncomplete {
                        zone: evidence.zone.clone(),
                    });
            }
        }
        for via in self.vias.iter().chain(&stitching.vias) {
            let x = emitter.number(&format!("via.{}.x", via.id.as_str()), &via.center.x)?;
            let y = emitter.number(&format!("via.{}.y", via.id.as_str()), &via.center.y)?;
            let size = emitter.number(
                &format!("via.{}.land_diameter", via.id.as_str()),
                &via.land_diameter,
            )?;
            let drill = emitter.number(
                &format!("via.{}.drill_diameter", via.id.as_str()),
                &via.drill_diameter,
            )?;
            writeln!(
                emitter.board,
                "  (via (at {x} {y}) (size {size}) (drill {drill}) (layers {} {}) (net {}))",
                quote(layer_name(&conductor_layers, via.start_layer)),
                quote(layer_name(&conductor_layers, via.end_layer)),
                net_numbers[&via.net]
            )
            .map_err(|_| KiCadExportError::Formatting)?;
            if !matches!(via.mask.front, crate::ViaMaskDisposition::Unspecified)
                || !matches!(via.mask.back, crate::ViaMaskDisposition::Unspecified)
            {
                emitter.omissions.push(KiCadExportOmission::ViaMaskIntent {
                    via: via.id.as_str().to_owned(),
                });
            }
        }
        for zone in &self.zones {
            let clearance = emitter.number(
                &format!("zone.{}.clearance", zone.id.as_str()),
                &zone.clearance,
            )?;
            writeln!(
                emitter.board,
                "  (zone (net {}) (net_name {}) (layer {}) (hatch edge 0.5)",
                net_numbers[&zone.net],
                quote(zone.net.as_str()),
                quote(layer_name(&conductor_layers, zone.layer))
            )
            .map_err(|_| KiCadExportError::Formatting)?;
            writeln!(emitter.board, "    (priority {})", zone.priority)
                .map_err(|_| KiCadExportError::Formatting)?;
            match zone.connection {
                CopperZoneConnection::Solid => writeln!(
                    emitter.board,
                    "    (connect_pads yes (clearance {clearance}))"
                ),
                CopperZoneConnection::Isolated => writeln!(
                    emitter.board,
                    "    (connect_pads no (clearance {clearance}))"
                ),
                CopperZoneConnection::ThermalRelief { .. } => {
                    writeln!(emitter.board, "    (connect_pads (clearance {clearance}))")
                }
            }
            .map_err(|_| KiCadExportError::Formatting)?;
            write!(emitter.board, "    (fill yes").map_err(|_| KiCadExportError::Formatting)?;
            if let CopperZoneFill::Hatched {
                line_width,
                gap,
                angle_degrees,
            } = &zone.fill
            {
                let width = emitter.number(
                    &format!("zone.{}.hatch.line_width", zone.id.as_str()),
                    line_width,
                )?;
                let gap = emitter.number(&format!("zone.{}.hatch.gap", zone.id.as_str()), gap)?;
                let angle = emitter.number(
                    &format!("zone.{}.hatch.angle", zone.id.as_str()),
                    angle_degrees,
                )?;
                write!(
                    emitter.board,
                    " (mode hatch) (hatch_thickness {width}) (hatch_gap {gap}) (hatch_orientation {angle})"
                )
                .map_err(|_| KiCadExportError::Formatting)?;
            }
            if let CopperZoneConnection::ThermalRelief {
                air_gap,
                spoke_width,
                spoke_count,
            } = &zone.connection
            {
                let gap = emitter.number(
                    &format!("zone.{}.thermal.air_gap", zone.id.as_str()),
                    air_gap,
                )?;
                let width = emitter.number(
                    &format!("zone.{}.thermal.spoke_width", zone.id.as_str()),
                    spoke_width,
                )?;
                write!(
                    emitter.board,
                    " (thermal_gap {gap}) (thermal_bridge_width {width})"
                )
                .map_err(|_| KiCadExportError::Formatting)?;
                if *spoke_count != 4 {
                    emitter
                        .omissions
                        .push(KiCadExportOmission::ZoneThermalSpokeCount {
                            zone: zone.id.as_str().into(),
                            spoke_count: *spoke_count,
                        });
                }
            }
            match (zone.islands.remove_unconnected, &zone.islands.minimum_area) {
                (false, None) => write!(emitter.board, " (island_removal_mode 1)"),
                (true, None) => write!(emitter.board, " (island_removal_mode 0)"),
                (true, Some(minimum_area)) => {
                    let minimum_area = emitter.number(
                        &format!("zone.{}.islands.minimum_area", zone.id.as_str()),
                        minimum_area,
                    )?;
                    write!(
                        emitter.board,
                        " (island_removal_mode 2) (island_area_min {minimum_area})"
                    )
                }
                (false, Some(_)) => unreachable!("validated zone island policy"),
            }
            .map_err(|_| KiCadExportError::Formatting)?;
            writeln!(emitter.board, ")").map_err(|_| KiCadExportError::Formatting)?;
            write!(emitter.board, "    (polygon (pts").map_err(|_| KiCadExportError::Formatting)?;
            for (index, point) in zone.boundary.iter().enumerate() {
                let x = emitter.number(
                    &format!("zone.{}.point[{index}].x", zone.id.as_str()),
                    &point.x,
                )?;
                let y = emitter.number(
                    &format!("zone.{}.point[{index}].y", zone.id.as_str()),
                    &point.y,
                )?;
                write!(emitter.board, " (xy {x} {y})").map_err(|_| KiCadExportError::Formatting)?;
            }
            writeln!(emitter.board, "))").map_err(|_| KiCadExportError::Formatting)?;
            writeln!(emitter.board, "  )").map_err(|_| KiCadExportError::Formatting)?;
        }
        if !self.keepouts.is_empty() {
            emitter
                .omissions
                .push(KiCadExportOmission::Keepouts(self.keepouts.len()));
        }
        if !self.placement_constraints.is_empty() {
            emitter
                .omissions
                .push(KiCadExportOmission::PlacementConstraints(
                    self.placement_constraints.len(),
                ));
        }
        writeln!(emitter.board, ")").map_err(|_| KiCadExportError::Formatting)?;
        Ok(KiCadExportReport {
            board: emitter.board,
            design_rules: emitter.design_rules,
            project: emitter.project,
            numeric_projections: emitter.projections,
            design_rule_projections: emitter.design_rule_projections,
            project_net_class_projections: emitter.project_net_class_projections,
            omissions: emitter.omissions,
        })
    }
}

fn arc_midpoint(arc: &ExplicitCircularArc) -> Option<hyperlattice::Point2> {
    if arc.facts().sweep_class == ExplicitArcSweepClass::FullCircle
        || arc.facts().sweep_class == ExplicitArcSweepClass::Unknown
    {
        return None;
    }
    let sx = arc.start().x.clone() - arc.center().x.clone();
    let sy = arc.start().y.clone() - arc.center().y.clone();
    let (mut vx, mut vy) = if arc.facts().sweep_class == ExplicitArcSweepClass::HalfTurn {
        match arc.direction() {
            ArcDirection::Ccw => (-sy, sx),
            ArcDirection::Cw => (sy, -sx),
        }
    } else {
        (
            sx + (arc.end().x.clone() - arc.center().x.clone()),
            sy + (arc.end().y.clone() - arc.center().y.clone()),
        )
    };
    if arc.facts().sweep_class == ExplicitArcSweepClass::GreaterThanHalfTurn {
        vx = -vx;
        vy = -vy;
    }
    let length = (vx.clone() * vx.clone() + vy.clone() * vy.clone())
        .sqrt()
        .ok()?;
    let scale = (arc.radius().clone() / length).ok()?;
    Some(hyperlattice::Point2::new(
        arc.center().x.clone() + vx * scale.clone(),
        arc.center().y.clone() + vy * scale,
    ))
}

struct Emitter {
    options: KiCadExportOptions,
    board: String,
    design_rules: Option<String>,
    project: Option<String>,
    projections: Vec<KiCadNumericProjection>,
    design_rule_projections: Vec<KiCadDesignRuleProjection>,
    project_net_class_projections: Vec<KiCadProjectNetClassProjection>,
    omissions: Vec<KiCadExportOmission>,
}

impl Emitter {
    fn new(options: KiCadExportOptions) -> Self {
        Self {
            options,
            board: String::new(),
            design_rules: None,
            project: None,
            projections: Vec::new(),
            design_rule_projections: Vec::new(),
            project_net_class_projections: Vec::new(),
            omissions: Vec::new(),
        }
    }

    fn number(&mut self, field: &str, value: &Real) -> Result<String, KiCadExportError> {
        let Some(value_f64) = value.to_f64_lossy().filter(|value| value.is_finite()) else {
            return Err(KiCadExportError::NonFiniteScalar(field.to_owned()));
        };
        let mut emitted = format!("{:.*}", self.options.decimal_places, value_f64);
        if emitted.contains('.') {
            while emitted.ends_with('0') {
                emitted.pop();
            }
            if emitted.ends_with('.') {
                emitted.pop();
            }
        }
        if emitted == "-0" {
            emitted = "0".into();
        }
        self.projections.push(KiCadNumericProjection {
            field: field.to_owned(),
            source: value.to_string(),
            emitted: emitted.clone(),
        });
        Ok(emitted)
    }
}

fn emit_stackup(emitter: &mut Emitter, layout: &PcbLayout) -> Result<(), KiCadExportError> {
    let conductor_positions = layout
        .stackup
        .layers
        .iter()
        .enumerate()
        .filter_map(|(index, layer)| {
            matches!(layer.kind, StackupLayerKind::Conductor(_)).then_some(index)
        })
        .collect::<Vec<_>>();
    let first_conductor = conductor_positions.first().copied().unwrap_or(0);
    let last_conductor = conductor_positions.last().copied().unwrap_or(0);
    let mut dielectric_index = 0_usize;

    writeln!(emitter.board, "  (setup").map_err(|_| KiCadExportError::Formatting)?;
    writeln!(emitter.board, "    (stackup").map_err(|_| KiCadExportError::Formatting)?;
    for (index, layer) in layout.stackup.layers.iter().enumerate() {
        let (name, kind) = match &layer.kind {
            StackupLayerKind::Conductor(_) => (layer.name.clone(), "copper".to_owned()),
            StackupLayerKind::Dielectric => {
                dielectric_index += 1;
                let canonical = format!("dielectric {dielectric_index}");
                if layer.name != canonical {
                    emitter
                        .omissions
                        .push(KiCadExportOmission::StackupLayerNameLowered {
                            layer: layer.name.clone(),
                            emitted: canonical.clone(),
                        });
                }
                (canonical, "core".to_owned())
            }
            StackupLayerKind::SolderMask if index < first_conductor => {
                if layer.name != "F.Mask" {
                    emitter
                        .omissions
                        .push(KiCadExportOmission::StackupLayerNameLowered {
                            layer: layer.name.clone(),
                            emitted: "F.Mask".into(),
                        });
                }
                ("F.Mask".into(), "Top Solder Mask".into())
            }
            StackupLayerKind::SolderMask if index > last_conductor => {
                if layer.name != "B.Mask" {
                    emitter
                        .omissions
                        .push(KiCadExportOmission::StackupLayerNameLowered {
                            layer: layer.name.clone(),
                            emitted: "B.Mask".into(),
                        });
                }
                ("B.Mask".into(), "Bottom Solder Mask".into())
            }
            StackupLayerKind::SolderMask => {
                dielectric_index += 1;
                let canonical = format!("dielectric {dielectric_index}");
                emitter
                    .omissions
                    .push(KiCadExportOmission::StackupLayerKindLowered {
                        layer: layer.name.clone(),
                        kind: "interior solder mask".into(),
                    });
                (canonical, "core".into())
            }
            StackupLayerKind::Custom(kind) => {
                dielectric_index += 1;
                let canonical = format!("dielectric {dielectric_index}");
                emitter
                    .omissions
                    .push(KiCadExportOmission::StackupLayerKindLowered {
                        layer: layer.name.clone(),
                        kind: kind.clone(),
                    });
                (canonical, "core".into())
            }
        };
        let thickness = emitter.number(
            &format!("stackup.layer[{index}].thickness"),
            &layer.thickness,
        )?;
        write!(
            emitter.board,
            "      (layer {} (type {}) (thickness {thickness})",
            quote(&name),
            quote(&kind)
        )
        .map_err(|_| KiCadExportError::Formatting)?;
        if let Some(material) = &layer.material {
            write!(emitter.board, " (material {})", quote(material))
                .map_err(|_| KiCadExportError::Formatting)?;
        }
        writeln!(emitter.board, ")").map_err(|_| KiCadExportError::Formatting)?;
    }
    writeln!(emitter.board, "    )").map_err(|_| KiCadExportError::Formatting)?;
    writeln!(emitter.board, "    (pad_to_mask_clearance 0)")
        .map_err(|_| KiCadExportError::Formatting)?;
    writeln!(emitter.board, "  )").map_err(|_| KiCadExportError::Formatting)?;
    Ok(())
}

fn emit_design_rules(emitter: &mut Emitter, layout: &PcbLayout) -> Result<(), KiCadExportError> {
    let resolved = layout
        .rules
        .resolve_net_classes()
        .map_err(|_| KiCadExportError::InvalidDesign)?;
    let mut contents = String::from("(version 1)\n");
    for (authored, effective) in layout.rules.net_classes.iter().zip(resolved) {
        let nets = authored
            .nets
            .iter()
            .filter_map(|net| {
                if net.as_str().contains(['\'', '\n', '\r']) {
                    emitter
                        .omissions
                        .push(KiCadExportOmission::DesignRuleNetName {
                            net_class: authored.id.as_str().into(),
                            net: net.as_str().into(),
                        });
                    None
                } else {
                    Some(net.as_str().to_owned())
                }
            })
            .collect::<Vec<_>>();
        if nets.is_empty() {
            continue;
        }
        let mut constraints = Vec::new();
        if let Some(value) = &effective.min_trace_width {
            let value = emitter.number(
                &format!("net_class.{}.min_trace_width", authored.id.as_str()),
                value,
            )?;
            constraints.push(format!("track_width (min {value}mm)"));
        }
        if let Some(value) = &effective.min_clearance {
            let value = emitter.number(
                &format!("net_class.{}.min_clearance", authored.id.as_str()),
                value,
            )?;
            constraints.push(format!("clearance (min {value}mm)"));
        }
        if let Some(value) = &effective.max_length {
            let value = emitter.number(
                &format!("net_class.{}.max_length", authored.id.as_str()),
                value,
            )?;
            constraints.push(format!("length (max {value}mm)"));
        }
        if let Some(value) = effective.max_via_count {
            constraints.push(format!("via_count (max {value})"));
        }
        if constraints.is_empty() {
            continue;
        }
        let condition = nets
            .iter()
            .map(|net| format!("A.NetName == '{net}'"))
            .collect::<Vec<_>>()
            .join(" || ");
        writeln!(
            contents,
            "(rule {}\n  (condition {})",
            quote(&format!("hypercircuit.netclass.{}", authored.id.as_str())),
            quote(&condition)
        )
        .map_err(|_| KiCadExportError::Formatting)?;
        for constraint in &constraints {
            writeln!(contents, "  (constraint {constraint})")
                .map_err(|_| KiCadExportError::Formatting)?;
        }
        writeln!(contents, ")").map_err(|_| KiCadExportError::Formatting)?;
        emitter
            .design_rule_projections
            .push(KiCadDesignRuleProjection {
                net_class: authored.id.as_str().into(),
                nets,
                constraints,
                flattened_inheritance: authored.parent.is_some(),
            });
        if authored.parent.is_some() {
            emitter
                .omissions
                .push(KiCadExportOmission::NetClassInheritanceLowered {
                    net_class: authored.id.as_str().into(),
                });
        }
        if let Some(via_style) = &effective.preferred_via_style {
            emitter
                .omissions
                .push(KiCadExportOmission::NetClassViaStylePreference {
                    net_class: authored.id.as_str().into(),
                    via_style: via_style.as_str().into(),
                });
        }
        if effective.target_impedance_ohms.is_some()
            || effective.impedance_tolerance_ohms.is_some()
            || effective.requires_reference_plane
        {
            emitter
                .omissions
                .push(KiCadExportOmission::NetClassSignalIntegrityPolicy {
                    net_class: authored.id.as_str().into(),
                });
        }
    }
    let advanced = &layout.rules;
    if !advanced.via_styles.is_empty()
        || !advanced.differential_pairs.is_empty()
        || !advanced.route_constraint_regions.is_empty()
        || !advanced.route_rule_regions.is_empty()
        || !advanced.escape_policies.is_empty()
        || !advanced.length_tuning_patterns.is_empty()
        || !advanced.phase_tuning_groups.is_empty()
    {
        emitter
            .omissions
            .push(KiCadExportOmission::AdvancedDesignRules {
                via_styles: advanced.via_styles.len(),
                differential_pairs: advanced.differential_pairs.len(),
                route_constraint_regions: advanced.route_constraint_regions.len(),
                route_rule_regions: advanced.route_rule_regions.len(),
                escape_policies: advanced.escape_policies.len(),
                length_tuning_patterns: advanced.length_tuning_patterns.len(),
                phase_tuning_groups: advanced.phase_tuning_groups.len(),
            });
    }
    if !emitter.design_rule_projections.is_empty() {
        emitter.design_rules = Some(contents);
    }
    Ok(())
}

fn emit_project_settings(
    emitter: &mut Emitter,
    layout: &PcbLayout,
) -> Result<(), KiCadExportError> {
    if layout.rules.net_classes.is_empty() {
        return Ok(());
    }
    let resolved = layout
        .rules
        .resolve_net_classes()
        .map_err(|_| KiCadExportError::InvalidDesign)?;
    let mut classes = Vec::new();
    let mut assignments = serde_json::Map::new();
    for (index, (authored, effective)) in layout.rules.net_classes.iter().zip(resolved).enumerate()
    {
        let priority = if authored.id.as_str() == "Default" {
            i32::MAX
        } else {
            i32::try_from(index).map_err(|_| KiCadExportError::Formatting)?
        };
        let mut class = serde_json::Map::new();
        class.insert(
            "name".into(),
            serde_json::Value::String(authored.id.as_str().into()),
        );
        class.insert("priority".into(), serde_json::Value::from(priority));
        let mut fields = vec!["name".into(), "priority".into()];
        if let Some(value) = effective
            .preferred_trace_width
            .as_ref()
            .or(effective.min_trace_width.as_ref())
        {
            class.insert(
                "track_width".into(),
                project_number(
                    emitter,
                    &format!("project.net_class.{}.track_width", authored.id.as_str()),
                    value,
                )?,
            );
            fields.push(if effective.preferred_trace_width.is_some() {
                "track_width".into()
            } else {
                "track_width<-min_trace_width".into()
            });
        }
        if let Some(value) = &effective.min_clearance {
            class.insert(
                "clearance".into(),
                project_number(
                    emitter,
                    &format!("project.net_class.{}.clearance", authored.id.as_str()),
                    value,
                )?,
            );
            fields.push("clearance".into());
        }
        if let Some(value) = &effective.preferred_via_land_diameter {
            class.insert(
                "via_diameter".into(),
                project_number(
                    emitter,
                    &format!("project.net_class.{}.via_diameter", authored.id.as_str()),
                    value,
                )?,
            );
            fields.push("via_diameter".into());
        }
        if let Some(value) = &effective.preferred_via_drill_diameter {
            class.insert(
                "via_drill".into(),
                project_number(
                    emitter,
                    &format!("project.net_class.{}.via_drill", authored.id.as_str()),
                    value,
                )?,
            );
            fields.push("via_drill".into());
        }
        let nets = authored
            .nets
            .iter()
            .map(|net| net.as_str().to_owned())
            .collect::<Vec<_>>();
        for net in &nets {
            assignments.insert(
                net.clone(),
                serde_json::Value::Array(vec![serde_json::Value::String(
                    authored.id.as_str().into(),
                )]),
            );
        }
        classes.push(serde_json::Value::Object(class));
        emitter
            .project_net_class_projections
            .push(KiCadProjectNetClassProjection {
                net_class: authored.id.as_str().into(),
                nets,
                fields,
                flattened_inheritance: authored.parent.is_some(),
            });
    }
    let project = serde_json::json!({
        "meta": {
            "filename": format!("{}.kicad_pro", layout.id.as_str()),
            "version": 1
        },
        "net_settings": {
            "classes": classes,
            "meta": {
                "version": 4
            },
            "net_colors": null,
            "netclass_assignments": assignments,
            "netclass_patterns": []
        }
    });
    let mut contents =
        serde_json::to_string_pretty(&project).map_err(|_| KiCadExportError::Formatting)?;
    contents.push('\n');
    emitter.project = Some(contents);
    Ok(())
}

fn project_number(
    emitter: &mut Emitter,
    field: &str,
    value: &Real,
) -> Result<serde_json::Value, KiCadExportError> {
    let token = emitter.number(field, value)?;
    serde_json::from_str(&token).map_err(|_| KiCadExportError::Formatting)
}

type ConductorLayer = (u16, String, TraceLayer);

fn conductor_layers(layout: &PcbLayout) -> Vec<ConductorLayer> {
    let layers = layout
        .stackup
        .layers
        .iter()
        .filter_map(|layer| match layer.kind {
            StackupLayerKind::Conductor(index) => Some((layer.name.clone(), index)),
            _ => None,
        })
        .collect::<Vec<_>>();
    let last = layers.len().saturating_sub(1);
    layers
        .into_iter()
        .enumerate()
        .map(|(position, (name, index))| {
            let slot = if position == 0 {
                0
            } else if position == last {
                31
            } else {
                u16::try_from(position).unwrap_or(30).min(30)
            };
            (slot, name, index)
        })
        .collect()
}

fn layer_name(layers: &[ConductorLayer], index: TraceLayer) -> &str {
    layers
        .iter()
        .find(|(_, _, candidate)| *candidate == index)
        .map(|(_, name, _)| name.as_str())
        .unwrap_or("F.Cu")
}

fn emit_contour(
    emitter: &mut Emitter,
    prefix: &str,
    contour: &crate::BoardContour,
) -> Result<(), KiCadExportError> {
    for (index, segment) in contour.segments().iter().enumerate() {
        let x0 = emitter.number(&format!("{prefix}[{index}].start.x"), &segment.start().x)?;
        let y0 = emitter.number(&format!("{prefix}[{index}].start.y"), &segment.start().y)?;
        let x1 = emitter.number(&format!("{prefix}[{index}].end.x"), &segment.end().x)?;
        let y1 = emitter.number(&format!("{prefix}[{index}].end.y"), &segment.end().y)?;
        match segment {
            crate::BoardContourSegment::Line(_) => writeln!(
                emitter.board,
                "  (gr_line (start {x0} {y0}) (end {x1} {y1}) (stroke (width 0.1) (type default)) (layer \"Edge.Cuts\"))"
            )
            .map_err(|_| KiCadExportError::Formatting)?,
            crate::BoardContourSegment::CircularArc(arc) => {
                let Some(midpoint) = arc_midpoint(arc) else {
                    return Err(KiCadExportError::NonFiniteScalar(format!(
                        "{prefix}[{index}].mid"
                    )));
                };
                let xm = emitter.number(&format!("{prefix}[{index}].mid.x"), &midpoint.x)?;
                let ym = emitter.number(&format!("{prefix}[{index}].mid.y"), &midpoint.y)?;
                writeln!(
                    emitter.board,
                    "  (gr_arc (start {x0} {y0}) (mid {xm} {ym}) (end {x1} {y1}) (stroke (width 0.1) (type default)) (layer \"Edge.Cuts\"))"
                )
                .map_err(|_| KiCadExportError::Formatting)?;
            }
            crate::BoardContourSegment::CubicBezier(_) => {
                emitter
                    .omissions
                    .push(KiCadExportOmission::CubicBoardContourBezier {
                        contour: prefix.to_owned(),
                        segment: index,
                    });
                writeln!(
                    emitter.board,
                    "  (gr_line (start {x0} {y0}) (end {x1} {y1}) (stroke (width 0.1) (type default)) (layer \"Edge.Cuts\"))"
                )
                .map_err(|_| KiCadExportError::Formatting)?;
            }
        }
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn emit_pad(
    emitter: &mut Emitter,
    layout: &PcbLayout,
    pattern: &crate::LandPattern,
    instance: &crate::CircuitInstance,
    pad: &LandPatternPad,
    net_numbers: &BTreeMap<NetId, usize>,
    conductor_layers: &[ConductorLayer],
) -> Result<(), KiCadExportError> {
    let net = pattern
        .pin_map
        .iter()
        .find(|mapping| mapping.pad == pad.id)
        .and_then(|mapping| {
            instance
                .pins
                .iter()
                .find(|binding| binding.pin == mapping.pin)
        })
        .map(|binding| &binding.net);
    let x = emitter.number(
        &format!(
            "land_pattern.{}.pad.{}.x",
            pattern.id.as_str(),
            pad.id.as_str()
        ),
        &pad.center.x,
    )?;
    let y = emitter.number(
        &format!(
            "land_pattern.{}.pad.{}.y",
            pattern.id.as_str(),
            pad.id.as_str()
        ),
        &pad.center.y,
    )?;
    let rotation = emitter.number(
        &format!(
            "land_pattern.{}.pad.{}.rotation",
            pattern.id.as_str(),
            pad.id.as_str()
        ),
        &pad.rotation_degrees,
    )?;
    let (shape, width, height) = match &pad.shape {
        PadShape::Circle { diameter } => ("circle", diameter, diameter),
        PadShape::Rectangle { width, height } => ("rect", width, height),
        PadShape::RoundedRectangle { width, height, .. } => {
            emitter
                .omissions
                .push(KiCadExportOmission::RoundedRectangleRadius {
                    land_pattern: pattern.id.as_str().to_owned(),
                    pad: pad.id.as_str().to_owned(),
                });
            ("roundrect", width, height)
        }
        PadShape::Obround { width, height } => ("oval", width, height),
        PadShape::Polygon { .. } => ("custom", &Real::one(), &Real::one()),
    };
    let width = emitter.number(
        &format!(
            "land_pattern.{}.pad.{}.width",
            pattern.id.as_str(),
            pad.id.as_str()
        ),
        width,
    )?;
    let height = emitter.number(
        &format!(
            "land_pattern.{}.pad.{}.height",
            pattern.id.as_str(),
            pad.id.as_str()
        ),
        height,
    )?;
    let pad_type = match (&pad.drill, pad.plating) {
        (None, _) => "smd",
        (Some(_), Plating::NonPlated) => "np_thru_hole",
        (Some(_), _) => "thru_hole",
    };
    write!(
        emitter.board,
        "    (pad {} {pad_type} {shape} (at {x} {y} {rotation}) (size {width} {height}) (layers",
        quote(pad.id.as_str())
    )
    .map_err(|_| KiCadExportError::Formatting)?;
    if pad.drill.is_some() {
        write!(emitter.board, " \"*.Cu\" \"*.Mask\"").map_err(|_| KiCadExportError::Formatting)?;
    } else {
        for layer in &pad.copper_layers {
            write!(
                emitter.board,
                " {}",
                quote(layer_name(conductor_layers, *layer))
            )
            .map_err(|_| KiCadExportError::Formatting)?;
        }
    }
    write!(emitter.board, ")").map_err(|_| KiCadExportError::Formatting)?;
    if let Some(net) = net {
        write!(
            emitter.board,
            " (net {} {})",
            net_numbers[net],
            quote(net.as_str())
        )
        .map_err(|_| KiCadExportError::Formatting)?;
    }
    if let Some(drill) = &pad.drill {
        match drill {
            DrillShape::Round { diameter } => {
                let diameter = emitter.number(
                    &format!(
                        "land_pattern.{}.pad.{}.drill",
                        pattern.id.as_str(),
                        pad.id.as_str()
                    ),
                    diameter,
                )?;
                write!(emitter.board, " (drill {diameter})")
                    .map_err(|_| KiCadExportError::Formatting)?;
            }
            DrillShape::Slot { start, end, width } => {
                if let Some((drill_width, drill_height)) =
                    kicad_oval_slot_dimensions(start, end, width)
                {
                    let drill_width = emitter.number(
                        &format!(
                            "land_pattern.{}.pad.{}.drill.width",
                            pattern.id.as_str(),
                            pad.id.as_str()
                        ),
                        &drill_width,
                    )?;
                    let drill_height = emitter.number(
                        &format!(
                            "land_pattern.{}.pad.{}.drill.height",
                            pattern.id.as_str(),
                            pad.id.as_str()
                        ),
                        &drill_height,
                    )?;
                    write!(emitter.board, " (drill oval {drill_width} {drill_height})")
                        .map_err(|_| KiCadExportError::Formatting)?;
                } else {
                    let diameter = emitter.number(
                        &format!(
                            "land_pattern.{}.pad.{}.drill.projected_diameter",
                            pattern.id.as_str(),
                            pad.id.as_str()
                        ),
                        width,
                    )?;
                    write!(emitter.board, " (drill {diameter})")
                        .map_err(|_| KiCadExportError::Formatting)?;
                    emitter
                        .omissions
                        .push(KiCadExportOmission::RoutedSlotProjectedToRound {
                            land_pattern: pattern.id.as_str().to_owned(),
                            pad: pad.id.as_str().to_owned(),
                        });
                }
            }
        }
    }
    if shape == "roundrect" {
        write!(emitter.board, " (roundrect_rratio 0.25)")
            .map_err(|_| KiCadExportError::Formatting)?;
    }
    if let PadShape::Polygon { vertices } = &pad.shape {
        write!(emitter.board, " (primitives (gr_poly (pts")
            .map_err(|_| KiCadExportError::Formatting)?;
        for (index, point) in vertices.iter().enumerate() {
            let x = emitter.number(
                &format!(
                    "land_pattern.{}.pad.{}.vertex[{index}].x",
                    pattern.id.as_str(),
                    pad.id.as_str()
                ),
                &point.x,
            )?;
            let y = emitter.number(
                &format!(
                    "land_pattern.{}.pad.{}.vertex[{index}].y",
                    pattern.id.as_str(),
                    pad.id.as_str()
                ),
                &point.y,
            )?;
            write!(emitter.board, " (xy {x} {y})").map_err(|_| KiCadExportError::Formatting)?;
        }
        write!(emitter.board, ") (width 0) (fill yes)))")
            .map_err(|_| KiCadExportError::Formatting)?;
    }
    if pad.solder_mask_margin.is_some() || pad.paste_margin.is_some() {
        emitter
            .omissions
            .push(KiCadExportOmission::PadMaskOrPasteMargin {
                land_pattern: pattern.id.as_str().to_owned(),
                pad: pad.id.as_str().to_owned(),
            });
    }
    writeln!(emitter.board, ")").map_err(|_| KiCadExportError::Formatting)?;
    let _ = layout;
    Ok(())
}

fn kicad_oval_slot_dimensions(
    start: &hyperlattice::Point2,
    end: &hyperlattice::Point2,
    cutter_width: &Real,
) -> Option<(Real, Real)> {
    let centered = start.x == -end.x.clone() && start.y == -end.y.clone();
    if !centered {
        return None;
    }
    if start.y == Real::zero() && end.y == Real::zero() {
        let length = (end.x.clone() - start.x.clone()).abs();
        return Some((length + cutter_width.clone(), cutter_width.clone()));
    }
    if start.x == Real::zero() && end.x == Real::zero() {
        let length = (end.y.clone() - start.y.clone()).abs();
        return Some((cutter_width.clone(), length + cutter_width.clone()));
    }
    None
}

fn emit_land_pattern_graphic(
    emitter: &mut Emitter,
    pattern: &crate::LandPattern,
    graphic: &LandPatternGraphic,
    side: BoardSide,
    conductor_layers: &[ConductorLayer],
) -> Result<(), KiCadExportError> {
    let prefix = format!(
        "land_pattern.{}.graphic.{}",
        pattern.id.as_str(),
        graphic.id.as_str()
    );
    let layer = quote(&graphic_layer(&graphic.layer, side, conductor_layers));
    let stroke = graphic_stroke(emitter, pattern, graphic, &prefix)?;
    match &graphic.primitive {
        LandPatternGraphicPrimitive::Line { start, end } => {
            let x0 = emitter.number(&format!("{prefix}.start.x"), &start.x)?;
            let y0 = emitter.number(&format!("{prefix}.start.y"), &start.y)?;
            let x1 = emitter.number(&format!("{prefix}.end.x"), &end.x)?;
            let y1 = emitter.number(&format!("{prefix}.end.y"), &end.y)?;
            writeln!(
                emitter.board,
                "    (fp_line (start {x0} {y0}) (end {x1} {y1}) (stroke (width {stroke}) (type default)) (layer {layer}))"
            )
            .map_err(|_| KiCadExportError::Formatting)?;
        }
        LandPatternGraphicPrimitive::Circle { center, radius } => {
            let center_x = emitter.number(&format!("{prefix}.center.x"), &center.x)?;
            let center_y = emitter.number(&format!("{prefix}.center.y"), &center.y)?;
            let end_x = emitter.number(
                &format!("{prefix}.end.x"),
                &(center.x.clone() + radius.clone()),
            )?;
            writeln!(
                emitter.board,
                "    (fp_circle (center {center_x} {center_y}) (end {end_x} {center_y}) (stroke (width {stroke}) (type default)) (fill none) (layer {layer}))"
            )
            .map_err(|_| KiCadExportError::Formatting)?;
        }
        LandPatternGraphicPrimitive::Polygon { vertices, filled } => {
            write!(emitter.board, "    (fp_poly (pts").map_err(|_| KiCadExportError::Formatting)?;
            for (index, point) in vertices.iter().enumerate() {
                let x = emitter.number(&format!("{prefix}.vertex[{index}].x"), &point.x)?;
                let y = emitter.number(&format!("{prefix}.vertex[{index}].y"), &point.y)?;
                write!(emitter.board, " (xy {x} {y})").map_err(|_| KiCadExportError::Formatting)?;
            }
            let fill = if *filled { "solid" } else { "none" };
            writeln!(
                emitter.board,
                ") (stroke (width {stroke}) (type default)) (fill {fill}) (layer {layer}))"
            )
            .map_err(|_| KiCadExportError::Formatting)?;
        }
        LandPatternGraphicPrimitive::Text {
            text,
            position,
            height,
            rotation_degrees,
        } => {
            let x = emitter.number(&format!("{prefix}.position.x"), &position.x)?;
            let y = emitter.number(&format!("{prefix}.position.y"), &position.y)?;
            let rotation = emitter.number(&format!("{prefix}.rotation"), rotation_degrees)?;
            let height = emitter.number(&format!("{prefix}.height"), height)?;
            writeln!(
                emitter.board,
                "    (fp_text user {} (at {x} {y} {rotation}) (layer {layer}) (effects (font (size {height} {height}) (thickness {stroke}))))",
                quote(text)
            )
            .map_err(|_| KiCadExportError::Formatting)?;
        }
    }
    Ok(())
}

fn graphic_stroke(
    emitter: &mut Emitter,
    pattern: &crate::LandPattern,
    graphic: &LandPatternGraphic,
    prefix: &str,
) -> Result<String, KiCadExportError> {
    let stroke = if let Some(stroke) = &graphic.stroke_width {
        stroke.clone()
    } else {
        emitter
            .omissions
            .push(KiCadExportOmission::DefaultGraphicStroke {
                land_pattern: pattern.id.as_str().into(),
                graphic: graphic.id.as_str().into(),
            });
        (Real::from(15) / Real::from(100)).expect("100 is a nonzero exact denominator")
    };
    emitter.number(&format!("{prefix}.stroke_width"), &stroke)
}

fn graphic_layer(role: &LayerRole, side: BoardSide, conductor_layers: &[ConductorLayer]) -> String {
    let swap = |front: &str, back: &str| match side {
        BoardSide::Front => front.to_owned(),
        BoardSide::Back => back.to_owned(),
    };
    match role {
        LayerRole::Copper(layer) => layer_name(conductor_layers, *layer).into(),
        LayerRole::FrontSolderMask => swap("F.Mask", "B.Mask"),
        LayerRole::BackSolderMask => swap("B.Mask", "F.Mask"),
        LayerRole::FrontPaste => swap("F.Paste", "B.Paste"),
        LayerRole::BackPaste => swap("B.Paste", "F.Paste"),
        LayerRole::FrontSilkscreen => swap("F.SilkS", "B.SilkS"),
        LayerRole::BackSilkscreen => swap("B.SilkS", "F.SilkS"),
        LayerRole::EdgeCuts => "Edge.Cuts".into(),
        LayerRole::Fabrication => swap("F.Fab", "B.Fab"),
        LayerRole::Courtyard => swap("F.CrtYd", "B.CrtYd"),
        LayerRole::Custom(name) => name.clone(),
    }
}

fn quote(value: &str) -> String {
    format!("\"{}\"", value.replace('\\', "\\\\").replace('"', "\\\""))
}
