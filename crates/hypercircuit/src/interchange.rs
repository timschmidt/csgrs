//! Versioned semantic JSON interchange for circuit and schematic intent.

use std::fmt::{Display, Formatter};

use hyperlattice::Point2;
use hyperpath::{ArcDirection, CubicBezier, ExplicitCircularArc, LinePathSegment, TraceLayer};
use hyperreal::Real;
use serde::{Deserialize, Serialize};

use crate::{
    BoardContour, BoardContourSegment, Circuit, DesignRevision, PcbLayout, PcbRouteSegment,
    SchematicLayout,
};

/// Stable schema family emitted by this crate.
pub const SEMANTIC_SCHEMA: &str = "org.hypercircuit.semantic";

/// Latest schema version understood by this crate.
pub const SEMANTIC_SCHEMA_VERSION: u32 = 26;

/// Oldest schema revision upgraded by the built-in additive migrations.
pub const SEMANTIC_SCHEMA_MIN_MIGRATABLE_VERSION: u32 = 8;

/// One additive semantic-schema capability introduced after version 8.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SemanticMigrationStep {
    /// Version 9 added module parameters and per-subcircuit overrides.
    ModuleParameters,
    /// Version 10 added constraint regions, escape policies, and length tuning.
    AdvancedRoutingIntent,
    /// Version 11 added nonlinear diode device models.
    DiodeModels,
    /// Version 12 added named via constructions and net-class selection.
    ViaStyles,
    /// Version 13 added cycle-safe net-class inheritance.
    NetClassInheritance,
    /// Version 14 added regional width and clearance overrides.
    RegionalRouteRules,
    /// Version 15 promoted polygon point rings into exact mixed-curve board contours.
    MixedCurveBoardContours,
    /// Version 16 added exact periodic pulse-source timing.
    PeriodicSourceWaveforms,
    /// Version 17 added exact sine and exponential source parameters.
    AnalyticSourceWaveforms,
    /// Version 18 added executable MOSFET polarity and D/G/S terminal roles.
    MosfetModels,
    /// Version 19 separated preferred routing width from enforced minimum width.
    PreferredTraceWidths,
    /// Version 20 separated reusable multipart symbol definitions from placements.
    ReusableSchematicSymbols,
    /// Version 21 made external package-model format and transform explicit.
    ExternalPcb3dModels,
    /// Version 22 separated package models from optional fallback body envelopes.
    IndependentPcb3dModels,
    /// Version 23 retained exact footprint-local pad rotation.
    PadLocalRotations,
    /// Version 24 made differential-pair target impedance explicit on the pair.
    DifferentialPairImpedance,
    /// Version 25 added atomic phase-tuning groups over retained patterns.
    PhaseTuningGroups,
    /// Version 26 added bounded reduced-width/spacing differential-pair fanout.
    DifferentialPairNeckdown,
}

/// Evidence describing an automatic semantic JSON upgrade.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SemanticMigrationReport {
    /// Schema revision present in the input JSON.
    pub from_version: u32,
    /// Current schema revision assigned to the decoded document.
    pub to_version: u32,
    /// Additive capability boundaries crossed in ascending order.
    pub steps: Vec<SemanticMigrationStep>,
}

/// A versioned semantic design document.
///
/// Logical connectivity remains authoritative in [`Circuit`]. The optional
/// schematic records how that topology is presented and is validated against
/// the circuit whenever a document crosses the interchange boundary.
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct SemanticDocument {
    /// Schema family discriminator.
    pub schema: String,
    /// Exact schema revision.
    pub version: u32,
    /// Optimistic-concurrency revision for retained editor deltas.
    #[serde(default)]
    pub design_revision: DesignRevision,
    /// Authoritative circuit graph and simulation intent.
    pub circuit: Circuit,
    /// Optional circuit-bound schematic presentation.
    pub schematic: Option<SchematicLayout>,
    /// Optional circuit-bound PCB intent.
    pub pcb: Option<PcbLayout>,
}

/// Failure to construct, encode, or decode a semantic design document.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum SemanticInterchangeError {
    /// JSON syntax or data-shape failure.
    Json(String),
    /// The document belongs to another schema family or revision.
    UnsupportedSchema { schema: String, version: u32 },
    /// The retained circuit graph failed structural validation.
    InvalidCircuit { issue_count: usize },
    /// Schematic endpoints or nets disagree with the retained circuit.
    InvalidSchematic { issue_count: usize },
    /// PCB identities, references, or structural intent are inconsistent.
    InvalidPcb { issue_count: usize },
}

impl Display for SemanticInterchangeError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Json(message) => write!(formatter, "semantic JSON error: {message}"),
            Self::UnsupportedSchema { schema, version } => {
                write!(formatter, "unsupported semantic schema {schema}@{version}")
            }
            Self::InvalidCircuit { issue_count } => {
                write!(
                    formatter,
                    "semantic circuit has {issue_count} validation issue(s)"
                )
            }
            Self::InvalidSchematic { issue_count } => write!(
                formatter,
                "semantic schematic has {issue_count} validation issue(s)"
            ),
            Self::InvalidPcb { issue_count } => {
                write!(
                    formatter,
                    "semantic PCB has {issue_count} validation issue(s)"
                )
            }
        }
    }
}

impl std::error::Error for SemanticInterchangeError {}

impl SemanticDocument {
    /// Constructs and validates the current schema revision.
    pub fn new(
        circuit: Circuit,
        schematic: Option<SchematicLayout>,
    ) -> Result<Self, SemanticInterchangeError> {
        let document = Self {
            schema: SEMANTIC_SCHEMA.to_owned(),
            version: SEMANTIC_SCHEMA_VERSION,
            design_revision: DesignRevision::default(),
            circuit,
            schematic,
            pcb: None,
        };
        document.validate()?;
        Ok(document)
    }

    /// Attaches and validates retained PCB intent against the circuit graph.
    pub fn with_pcb(mut self, pcb: PcbLayout) -> Result<Self, SemanticInterchangeError> {
        self.pcb = Some(pcb);
        self.validate()?;
        Ok(self)
    }

    /// Decodes JSON, migrates supported older revisions, and validates the result.
    pub fn from_json(json: &str) -> Result<Self, SemanticInterchangeError> {
        Self::from_json_migrating(json).map(|(document, _)| document)
    }

    /// Decodes JSON and reports every additive schema boundary crossed.
    pub fn from_json_migrating(
        json: &str,
    ) -> Result<(Self, SemanticMigrationReport), SemanticInterchangeError> {
        let mut value = serde_json::from_str::<serde_json::Value>(json)
            .map_err(|error| SemanticInterchangeError::Json(error.to_string()))?;
        let schema = value
            .get("schema")
            .and_then(serde_json::Value::as_str)
            .ok_or_else(|| {
                SemanticInterchangeError::Json(
                    "semantic document schema must be a string".to_owned(),
                )
            })?
            .to_owned();
        let version = value
            .get("version")
            .and_then(serde_json::Value::as_u64)
            .and_then(|version| u32::try_from(version).ok())
            .ok_or_else(|| {
                SemanticInterchangeError::Json(
                    "semantic document version must be an unsigned 32-bit integer".to_owned(),
                )
            })?;
        if schema != SEMANTIC_SCHEMA
            || !(SEMANTIC_SCHEMA_MIN_MIGRATABLE_VERSION..=SEMANTIC_SCHEMA_VERSION)
                .contains(&version)
        {
            return Err(SemanticInterchangeError::UnsupportedSchema { schema, version });
        }

        let mut steps = Vec::new();
        if version < 9 {
            steps.push(SemanticMigrationStep::ModuleParameters);
        }
        if version < 10 {
            steps.push(SemanticMigrationStep::AdvancedRoutingIntent);
        }
        if version < 11 {
            steps.push(SemanticMigrationStep::DiodeModels);
        }
        if version < 12 {
            steps.push(SemanticMigrationStep::ViaStyles);
        }
        if version < 13 {
            steps.push(SemanticMigrationStep::NetClassInheritance);
        }
        if version < 14 {
            steps.push(SemanticMigrationStep::RegionalRouteRules);
        }
        if version < 15 {
            migrate_board_contours(&mut value)?;
            steps.push(SemanticMigrationStep::MixedCurveBoardContours);
        }
        if version < 16 {
            steps.push(SemanticMigrationStep::PeriodicSourceWaveforms);
        }
        if version < 17 {
            steps.push(SemanticMigrationStep::AnalyticSourceWaveforms);
        }
        if version < 18 {
            steps.push(SemanticMigrationStep::MosfetModels);
        }
        if version < 19 {
            steps.push(SemanticMigrationStep::PreferredTraceWidths);
        }
        if version < 20 {
            migrate_reusable_schematic_symbols(&mut value)?;
            steps.push(SemanticMigrationStep::ReusableSchematicSymbols);
        }
        if version < 21 {
            migrate_external_pcb_3d_models(&mut value)?;
            steps.push(SemanticMigrationStep::ExternalPcb3dModels);
        }
        if version < 22 {
            migrate_independent_pcb_3d_models(&mut value)?;
            steps.push(SemanticMigrationStep::IndependentPcb3dModels);
        }
        if version < 23 {
            migrate_pad_local_rotations(&mut value)?;
            steps.push(SemanticMigrationStep::PadLocalRotations);
        }
        if version < 24 {
            steps.push(SemanticMigrationStep::DifferentialPairImpedance);
        }
        if version < 25 {
            steps.push(SemanticMigrationStep::PhaseTuningGroups);
        }
        if version < 26 {
            steps.push(SemanticMigrationStep::DifferentialPairNeckdown);
        }
        value["version"] = serde_json::Value::from(SEMANTIC_SCHEMA_VERSION);
        let document = serde_json::from_value::<Self>(value)
            .map_err(|error| SemanticInterchangeError::Json(error.to_string()))?;
        document.validate()?;
        Ok((
            document,
            SemanticMigrationReport {
                from_version: version,
                to_version: SEMANTIC_SCHEMA_VERSION,
                steps,
            },
        ))
    }

    /// Encodes deterministic pretty JSON after replaying semantic validation.
    pub fn to_json_pretty(&self) -> Result<String, SemanticInterchangeError> {
        self.validate()?;
        serde_json::to_string_pretty(self)
            .map_err(|error| SemanticInterchangeError::Json(error.to_string()))
    }

    /// Replays the schema, circuit, and schematic consistency gates.
    pub fn validate(&self) -> Result<(), SemanticInterchangeError> {
        if self.schema != SEMANTIC_SCHEMA || self.version != SEMANTIC_SCHEMA_VERSION {
            return Err(SemanticInterchangeError::UnsupportedSchema {
                schema: self.schema.clone(),
                version: self.version,
            });
        }
        let circuit = self.circuit.validate();
        if !circuit.is_valid() {
            return Err(SemanticInterchangeError::InvalidCircuit {
                issue_count: circuit.issues.len(),
            });
        }
        if let Some(schematic) = &self.schematic {
            let report = schematic.validate(&self.circuit);
            if !report.is_valid() {
                return Err(SemanticInterchangeError::InvalidSchematic {
                    issue_count: report.issues.len(),
                });
            }
        }
        if let Some(pcb) = &self.pcb {
            let report = pcb.validate(&self.circuit);
            if !report.is_valid() {
                return Err(SemanticInterchangeError::InvalidPcb {
                    issue_count: report.issues.len(),
                });
            }
        }
        Ok(())
    }
}

fn migrate_reusable_schematic_symbols(
    value: &mut serde_json::Value,
) -> Result<(), SemanticInterchangeError> {
    let models_by_instance = value
        .get("circuit")
        .and_then(|circuit| circuit.get("instances"))
        .and_then(serde_json::Value::as_array)
        .into_iter()
        .flatten()
        .filter_map(|instance| {
            Some((
                instance.get("id")?.as_str()?.to_owned(),
                instance.get("model")?.as_str()?.to_owned(),
            ))
        })
        .collect::<std::collections::BTreeMap<_, _>>();
    let Some(schematic) = value
        .get_mut("schematic")
        .and_then(serde_json::Value::as_object_mut)
    else {
        return Ok(());
    };
    if schematic.contains_key("symbol_definitions") {
        return Ok(());
    }
    let symbols = schematic
        .get_mut("symbols")
        .and_then(serde_json::Value::as_array_mut)
        .ok_or_else(|| {
            SemanticInterchangeError::Json("legacy schematic symbols must be an array".into())
        })?;
    let mut definitions = Vec::with_capacity(symbols.len());
    for symbol in symbols {
        let object = symbol.as_object_mut().ok_or_else(|| {
            SemanticInterchangeError::Json("legacy schematic symbol must be an object".into())
        })?;
        let symbol_id = object
            .get("id")
            .and_then(serde_json::Value::as_str)
            .ok_or_else(|| {
                SemanticInterchangeError::Json("legacy schematic symbol id must be a string".into())
            })?
            .to_owned();
        let instance = object
            .get("instance")
            .and_then(serde_json::Value::as_str)
            .ok_or_else(|| {
                SemanticInterchangeError::Json(
                    "legacy schematic symbol instance must be a string".into(),
                )
            })?;
        let model = models_by_instance.get(instance).ok_or_else(|| {
            SemanticInterchangeError::Json(format!(
                "legacy schematic symbol {symbol_id} references absent instance {instance}"
            ))
        })?;
        let definition = format!("legacy:{symbol_id}");
        let unit = object
            .get("unit")
            .cloned()
            .unwrap_or_else(|| serde_json::Value::from(1));
        let body_width = object.remove("body_width").ok_or_else(|| {
            SemanticInterchangeError::Json(format!(
                "legacy schematic symbol {symbol_id} has no body_width"
            ))
        })?;
        let body_height = object.remove("body_height").ok_or_else(|| {
            SemanticInterchangeError::Json(format!(
                "legacy schematic symbol {symbol_id} has no body_height"
            ))
        })?;
        let pins = object.remove("pins").ok_or_else(|| {
            SemanticInterchangeError::Json(format!(
                "legacy schematic symbol {symbol_id} has no pins"
            ))
        })?;
        definitions.push(serde_json::json!({
            "id": definition,
            "model": model,
            "name": symbol_id,
            "units": [{
                "unit": unit,
                "body_width": body_width,
                "body_height": body_height,
                "pins": pins,
                "graphics": [],
            }],
        }));
        object.insert("definition".into(), serde_json::Value::String(definition));
    }
    schematic.insert(
        "symbol_definitions".into(),
        serde_json::Value::Array(definitions),
    );
    Ok(())
}

fn migrate_external_pcb_3d_models(
    value: &mut serde_json::Value,
) -> Result<(), SemanticInterchangeError> {
    let Some(patterns) = value
        .get_mut("pcb")
        .and_then(serde_json::Value::as_object_mut)
        .and_then(|pcb| pcb.get_mut("land_patterns"))
        .and_then(serde_json::Value::as_array_mut)
    else {
        return Ok(());
    };
    let zero = serde_json::to_value(Real::zero())
        .map_err(|error| SemanticInterchangeError::Json(error.to_string()))?;
    let one = serde_json::to_value(Real::one())
        .map_err(|error| SemanticInterchangeError::Json(error.to_string()))?;
    for pattern in patterns {
        let Some(body) = pattern
            .as_object_mut()
            .and_then(|pattern| pattern.get_mut("body"))
            .and_then(serde_json::Value::as_object_mut)
        else {
            continue;
        };
        if body.contains_key("model") {
            continue;
        }
        let legacy = body
            .remove("model_handle")
            .unwrap_or(serde_json::Value::Null);
        let model = match legacy {
            serde_json::Value::Null => serde_json::Value::Null,
            serde_json::Value::String(uri) => {
                let lower = uri.to_ascii_lowercase();
                let format = if lower.ends_with(".obj") {
                    "WavefrontObj"
                } else if lower.ends_with(".step") || lower.ends_with(".stp") {
                    "Step"
                } else if lower.ends_with(".wrl") || lower.ends_with(".vrml") {
                    "Vrml"
                } else if lower.ends_with(".gltf") || lower.ends_with(".glb") {
                    "Gltf"
                } else {
                    "Unspecified"
                };
                let offset_z = body
                    .get("standoff")
                    .cloned()
                    .unwrap_or_else(|| zero.clone());
                serde_json::json!({
                    "uri": uri,
                    "format": format,
                    "transform": {
                        "offset_x": zero.clone(),
                        "offset_y": zero.clone(),
                        "offset_z": offset_z,
                        "rotate_x_degrees": zero.clone(),
                        "rotate_y_degrees": zero.clone(),
                        "rotate_z_degrees": zero.clone(),
                        "scale_x": one.clone(),
                        "scale_y": one.clone(),
                        "scale_z": one.clone(),
                    }
                })
            }
            _ => {
                return Err(SemanticInterchangeError::Json(
                    "legacy land-pattern body model_handle must be a string or null".into(),
                ));
            }
        };
        body.insert("model".into(), model);
    }
    Ok(())
}

fn migrate_independent_pcb_3d_models(
    value: &mut serde_json::Value,
) -> Result<(), SemanticInterchangeError> {
    let Some(patterns) = value
        .get_mut("pcb")
        .and_then(serde_json::Value::as_object_mut)
        .and_then(|pcb| pcb.get_mut("land_patterns"))
        .and_then(serde_json::Value::as_array_mut)
    else {
        return Ok(());
    };
    for pattern in patterns {
        let object = pattern.as_object_mut().ok_or_else(|| {
            SemanticInterchangeError::Json("land pattern must be an object".into())
        })?;
        if object.contains_key("models") {
            continue;
        }
        let model = object
            .get_mut("body")
            .and_then(serde_json::Value::as_object_mut)
            .and_then(|body| body.remove("model"))
            .unwrap_or(serde_json::Value::Null);
        object.insert(
            "models".into(),
            serde_json::Value::Array(match model {
                serde_json::Value::Null => Vec::new(),
                model => vec![model],
            }),
        );
    }
    Ok(())
}

fn migrate_pad_local_rotations(
    value: &mut serde_json::Value,
) -> Result<(), SemanticInterchangeError> {
    let Some(patterns) = value
        .get_mut("pcb")
        .and_then(serde_json::Value::as_object_mut)
        .and_then(|pcb| pcb.get_mut("land_patterns"))
        .and_then(serde_json::Value::as_array_mut)
    else {
        return Ok(());
    };
    let zero = serde_json::to_value(Real::zero())
        .map_err(|error| SemanticInterchangeError::Json(error.to_string()))?;
    for pattern in patterns {
        let pads = pattern
            .as_object_mut()
            .and_then(|pattern| pattern.get_mut("pads"))
            .and_then(serde_json::Value::as_array_mut)
            .ok_or_else(|| {
                SemanticInterchangeError::Json("land-pattern pads must be an array".into())
            })?;
        for pad in pads {
            let pad = pad.as_object_mut().ok_or_else(|| {
                SemanticInterchangeError::Json("land-pattern pad must be an object".into())
            })?;
            pad.entry("rotation_degrees")
                .or_insert_with(|| zero.clone());
        }
    }
    Ok(())
}

fn migrate_board_contours(value: &mut serde_json::Value) -> Result<(), SemanticInterchangeError> {
    let Some(outline) = value
        .get_mut("pcb")
        .and_then(serde_json::Value::as_object_mut)
        .and_then(|pcb| pcb.get_mut("outline"))
        .and_then(serde_json::Value::as_object_mut)
    else {
        return Ok(());
    };
    if let Some(exterior) = outline.get_mut("exterior") {
        *exterior = migrate_point_ring(exterior)?;
    }
    if let Some(cutouts) = outline
        .get_mut("cutouts")
        .and_then(serde_json::Value::as_array_mut)
    {
        for cutout in cutouts {
            *cutout = migrate_point_ring(cutout)?;
        }
    }
    Ok(())
}

fn migrate_point_ring(
    ring: &serde_json::Value,
) -> Result<serde_json::Value, SemanticInterchangeError> {
    let points = ring.as_array().ok_or_else(|| {
        SemanticInterchangeError::Json("legacy board contour must be a point array".into())
    })?;
    if points.is_empty() {
        return Ok(serde_json::Value::Array(Vec::new()));
    }
    let segments = points
        .iter()
        .enumerate()
        .map(|(index, start)| {
            serde_json::json!({
                "kind": "line",
                "start": start,
                "end": &points[(index + 1) % points.len()],
            })
        })
        .collect();
    Ok(serde_json::Value::Array(segments))
}

#[derive(Deserialize, Serialize)]
struct ExactPoint {
    x: Real,
    y: Real,
}

impl From<&Point2> for ExactPoint {
    fn from(point: &Point2) -> Self {
        Self {
            x: point.x.clone(),
            y: point.y.clone(),
        }
    }
}

impl From<ExactPoint> for Point2 {
    fn from(point: ExactPoint) -> Self {
        Self::new(point.x, point.y)
    }
}

pub(crate) mod point {
    use super::{Deserialize, ExactPoint, Point2, Serialize};

    pub fn serialize<S>(point: &Point2, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        ExactPoint::from(point).serialize(serializer)
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<Point2, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        ExactPoint::deserialize(deserializer).map(Into::into)
    }
}

pub(crate) mod points {
    use super::{Deserialize, ExactPoint, Point2, Serialize};

    pub fn serialize<S>(points: &[Point2], serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        points
            .iter()
            .map(ExactPoint::from)
            .collect::<Vec<_>>()
            .serialize(serializer)
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<Vec<Point2>, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        Vec::<ExactPoint>::deserialize(deserializer)
            .map(|points| points.into_iter().map(Into::into).collect())
    }
}

pub(crate) mod trace_layer {
    use super::TraceLayer;

    pub fn serialize<S>(layer: &TraceLayer, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        serializer.serialize_u16(layer.0)
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<TraceLayer, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        <u16 as serde::Deserialize>::deserialize(deserializer).map(TraceLayer)
    }
}

#[derive(Deserialize, Serialize)]
#[serde(tag = "kind", rename_all = "kebab-case")]
enum ExactBoardContourSegment {
    Line {
        start: ExactPoint,
        end: ExactPoint,
    },
    CircularArc {
        center: ExactPoint,
        radius: Real,
        start: ExactPoint,
        end: ExactPoint,
        counter_clockwise: bool,
    },
    CubicBezier {
        start: ExactPoint,
        control0: ExactPoint,
        control1: ExactPoint,
        end: ExactPoint,
    },
}

fn encode_board_contour(contour: &BoardContour) -> Vec<ExactBoardContourSegment> {
    contour
        .segments()
        .iter()
        .map(|segment| match segment {
            BoardContourSegment::Line(segment) => ExactBoardContourSegment::Line {
                start: ExactPoint::from(segment.start()),
                end: ExactPoint::from(segment.end()),
            },
            BoardContourSegment::CircularArc(arc) => ExactBoardContourSegment::CircularArc {
                center: ExactPoint::from(arc.center()),
                radius: arc.radius().clone(),
                start: ExactPoint::from(arc.start()),
                end: ExactPoint::from(arc.end()),
                counter_clockwise: arc.direction() == ArcDirection::Ccw,
            },
            BoardContourSegment::CubicBezier(bezier) => ExactBoardContourSegment::CubicBezier {
                start: ExactPoint::from(bezier.start()),
                control0: ExactPoint::from(bezier.control0()),
                control1: ExactPoint::from(bezier.control1()),
                end: ExactPoint::from(bezier.end()),
            },
        })
        .collect()
}

fn decode_board_contour<E: serde::de::Error>(
    segments: Vec<ExactBoardContourSegment>,
) -> Result<BoardContour, E> {
    segments
        .into_iter()
        .map(|segment| match segment {
            ExactBoardContourSegment::Line { start, end } => Ok(BoardContourSegment::Line(
                LinePathSegment::new(start.into(), end.into()),
            )),
            ExactBoardContourSegment::CircularArc {
                center,
                radius,
                start,
                end,
                counter_clockwise,
            } => ExplicitCircularArc::new(
                center.into(),
                radius,
                start.into(),
                end.into(),
                if counter_clockwise {
                    ArcDirection::Ccw
                } else {
                    ArcDirection::Cw
                },
            )
            .map(BoardContourSegment::CircularArc)
            .map_err(|error| E::custom(format!("{error:?}"))),
            ExactBoardContourSegment::CubicBezier {
                start,
                control0,
                control1,
                end,
            } => Ok(BoardContourSegment::CubicBezier(CubicBezier::new(
                start.into(),
                control0.into(),
                control1.into(),
                end.into(),
            ))),
        })
        .collect::<Result<Vec<_>, _>>()
        .map(BoardContour::from_segments)
}

pub(crate) mod board_contour {
    use super::{
        BoardContour, Deserialize, ExactBoardContourSegment, Serialize, decode_board_contour,
        encode_board_contour,
    };

    pub fn serialize<S>(contour: &BoardContour, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        encode_board_contour(contour).serialize(serializer)
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<BoardContour, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        decode_board_contour(Vec::<ExactBoardContourSegment>::deserialize(deserializer)?)
    }
}

pub(crate) mod board_contours {
    use super::{
        BoardContour, Deserialize, ExactBoardContourSegment, Serialize, decode_board_contour,
        encode_board_contour,
    };

    pub fn serialize<S>(contours: &[BoardContour], serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        contours
            .iter()
            .map(encode_board_contour)
            .collect::<Vec<_>>()
            .serialize(serializer)
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<Vec<BoardContour>, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        Vec::<Vec<ExactBoardContourSegment>>::deserialize(deserializer)?
            .into_iter()
            .map(decode_board_contour)
            .collect()
    }
}

pub(crate) mod trace_layers {
    use super::TraceLayer;
    use serde::{Deserialize, Serialize};

    pub fn serialize<S>(layers: &[TraceLayer], serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        layers
            .iter()
            .map(|layer| layer.0)
            .collect::<Vec<_>>()
            .serialize(serializer)
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<Vec<TraceLayer>, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        Vec::<u16>::deserialize(deserializer)
            .map(|layers| layers.into_iter().map(TraceLayer).collect())
    }
}

#[derive(Deserialize, Serialize)]
#[serde(tag = "kind", rename_all = "kebab-case")]
enum ExactRouteSegment {
    Line {
        start: ExactPoint,
        end: ExactPoint,
    },
    CircularArc {
        center: ExactPoint,
        radius: Real,
        start: ExactPoint,
        end: ExactPoint,
        counter_clockwise: bool,
    },
    CubicBezier {
        start: ExactPoint,
        control0: ExactPoint,
        control1: ExactPoint,
        end: ExactPoint,
    },
}

pub(crate) mod route_segments {
    use super::{
        ArcDirection, CubicBezier, Deserialize, ExactPoint, ExactRouteSegment, ExplicitCircularArc,
        LinePathSegment, PcbRouteSegment, Serialize,
    };

    pub fn serialize<S>(segments: &[PcbRouteSegment], serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        segments
            .iter()
            .map(|segment| match segment {
                PcbRouteSegment::Line(segment) => ExactRouteSegment::Line {
                    start: ExactPoint::from(segment.start()),
                    end: ExactPoint::from(segment.end()),
                },
                PcbRouteSegment::CircularArc(arc) => ExactRouteSegment::CircularArc {
                    center: ExactPoint::from(arc.center()),
                    radius: arc.radius().clone(),
                    start: ExactPoint::from(arc.start()),
                    end: ExactPoint::from(arc.end()),
                    counter_clockwise: arc.direction() == ArcDirection::Ccw,
                },
                PcbRouteSegment::CubicBezier(bezier) => ExactRouteSegment::CubicBezier {
                    start: ExactPoint::from(bezier.start()),
                    control0: ExactPoint::from(bezier.control0()),
                    control1: ExactPoint::from(bezier.control1()),
                    end: ExactPoint::from(bezier.end()),
                },
            })
            .collect::<Vec<_>>()
            .serialize(serializer)
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<Vec<PcbRouteSegment>, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        Vec::<ExactRouteSegment>::deserialize(deserializer)?
            .into_iter()
            .map(|segment| match segment {
                ExactRouteSegment::Line { start, end } => Ok(PcbRouteSegment::Line(
                    LinePathSegment::new(start.into(), end.into()),
                )),
                ExactRouteSegment::CircularArc {
                    center,
                    radius,
                    start,
                    end,
                    counter_clockwise,
                } => ExplicitCircularArc::new(
                    center.into(),
                    radius,
                    start.into(),
                    end.into(),
                    if counter_clockwise {
                        ArcDirection::Ccw
                    } else {
                        ArcDirection::Cw
                    },
                )
                .map(PcbRouteSegment::CircularArc)
                .map_err(|error| serde::de::Error::custom(format!("{error:?}"))),
                ExactRouteSegment::CubicBezier {
                    start,
                    control0,
                    control1,
                    end,
                } => Ok(PcbRouteSegment::CubicBezier(CubicBezier::new(
                    start.into(),
                    control0.into(),
                    control1.into(),
                    end.into(),
                ))),
            })
            .collect()
    }
}
