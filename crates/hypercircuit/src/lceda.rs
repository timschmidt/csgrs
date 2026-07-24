//! Audited EasyEDA/LCEDA Pro (`.epro2`) project interchange.
//!
//! LCEDA Pro is a finite, external interchange boundary. The exporter retains
//! logical identities and authored drawing/layout geometry, records every
//! exact-to-decimal projection, and reports semantic details for which the
//! current record vocabulary has no faithful representation.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::{Display, Formatter};
use std::io::{self, Write};
use std::str::FromStr;

use hyperlattice::Point2;
use hyperpath::TraceLayer;
use hyperreal::{Real, RealSign};
use serde_json::{Value, json};

use crate::{
    BoardContour, BoardContourSegment, BoardSide, Circuit, CopperZoneConnection, CopperZoneFill,
    DrillShape, LandPattern, LandPatternGraphicPrimitive, LandPatternPad, LayerRole, PadShape,
    PcbLayout, PcbRouteSegment, Plating, SchematicEndpoint, SchematicLayout, SchematicPoint,
    StackupLayerKind,
};

/// Schema identifying the auditable hypercircuit export report.
pub const LCEDA_PRO_EXPORT_SCHEMA: &str = "hypercircuit/lceda-pro-export";
/// Version of the report and mapping policy.
pub const LCEDA_PRO_EXPORT_VERSION: u32 = 4;

/// Physical unit used by authored PCB coordinates.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LcedaSourceLengthUnit {
    /// Millimetres.
    Millimeter,
    /// Thousandths of an inch.
    Mil,
    /// Inches.
    Inch,
}

impl LcedaSourceLengthUnit {
    fn mil_factor(self) -> Real {
        match self {
            Self::Millimeter => {
                (Real::from(5000) / Real::from(127)).expect("127 is a nonzero exact denominator")
            }
            Self::Mil => Real::one(),
            Self::Inch => Real::from(1000),
        }
    }
}

/// LCEDA Pro output and finite-decimal policy.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct LcedaProExportOptions {
    /// Unit in which physical PCB dimensions were authored.
    pub source_length_unit: LcedaSourceLengthUnit,
    /// Digits written after the decimal point before zero trimming.
    pub decimal_places: usize,
}

impl LcedaProExportOptions {
    /// Conventional millimetre-authored board policy.
    pub const fn millimeters() -> Self {
        Self {
            source_length_unit: LcedaSourceLengthUnit::Millimeter,
            decimal_places: 6,
        }
    }
}

/// One exact scalar projected into LCEDA numeric syntax.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct LcedaNumericProjection {
    /// Stable semantic field path.
    pub field: String,
    /// Exact-aware source spelling.
    pub source: String,
    /// Unit of the emitted token.
    pub emitted_unit: &'static str,
    /// Decimal token placed in the record stream.
    pub emitted: String,
}

/// Retained intent not faithfully represented by this mapping version.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum LcedaExportOmission {
    /// The public Pro specification is incomplete and announces an incompatible V3 format.
    EditorConformanceReview,
    /// LCEDA receives physical layers but not all material-library semantics.
    DetailedStackup,
    /// Relational constraints are already reflected only when placements were resolved.
    PlacementConstraints(usize),
    /// Typed keepout scopes do not have a proven Pro record mapping yet.
    Keepouts(usize),
    /// The arbitrary pad polygon was represented by its bounding rectangle.
    PolygonPadApproximation { land_pattern: String, pad: String },
    /// The routed slot was represented by its overall rectangular hole envelope.
    RoutedSlotApproximation { land_pattern: String, pad: String },
    /// Source-specific/custom artwork layer was placed on the document layer.
    CustomArtworkLayer {
        land_pattern: String,
        graphic: String,
    },
    /// A mask or paste expansion remains retained only in hypercircuit.
    PadProcessMargin { land_pattern: String, pad: String },
    /// A non-plated or unspecified via was emitted, but editor behavior must be reviewed.
    ViaPlatingReview { via: String },
    /// Explicit via mask opening/tenting intent remains retained only in hypercircuit.
    ViaMaskIntent { via: String },
    /// Exact circular route arc has no proven LCEDA Pro record mapping yet.
    CircularRouteArc { route: String, segment: usize },
    /// Exact cubic route Bezier has no proven LCEDA Pro record mapping yet.
    CubicRouteBezier { route: String, segment: usize },
    /// A curved board edge was represented by its endpoint chord.
    CurvedBoardContourChord { contour: String, segment: usize },
    /// Zone policy is retained in extension fields pending native-editor conformance.
    ZonePolicyExtension { zone: String },
    /// Sheet parentage is retained as metadata, not native hierarchical sheet symbols.
    SchematicHierarchyMetadata,
    /// Reusable multipart symbol graphics remain retained in HyperCircuit.
    SchematicSymbolLibrary {
        /// Number of reusable definitions.
        definitions: usize,
        /// Number of independently placeable units.
        units: usize,
        /// Number of exact graphic primitives.
        graphics: usize,
    },
}

/// Deterministic archive plus its full loss/projection audit.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct LcedaProExportReport {
    /// Report schema.
    pub schema: &'static str,
    /// Mapping version.
    pub version: u32,
    /// Complete deterministic `.epro2` ZIP bytes.
    pub archive: Vec<u8>,
    /// Exact `project2.json` archive member.
    pub project_json: String,
    /// Exact `.epru` JSON-record stream archive member.
    pub record_stream: String,
    /// Archive member names in deterministic order.
    pub files: Vec<String>,
    /// Every exact-to-finite conversion.
    pub numeric_projections: Vec<LcedaNumericProjection>,
    /// Every known semantic approximation or omission.
    pub omissions: Vec<LcedaExportOmission>,
}

/// Failure before a structurally meaningful project can be produced.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum LcedaExportError {
    /// Circuit, schematic, or PCB validation failed.
    InvalidDesign,
    /// Decimal precision must be nonzero and bounded.
    InvalidOptions,
    /// An exact scalar did not project to a finite number.
    NonFiniteScalar(String),
    /// JSON serialization failed.
    Json,
    /// ZIP size or I/O limits were exceeded.
    Archive(String),
}

impl Display for LcedaExportError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidDesign => formatter.write_str("cannot export an invalid LCEDA design"),
            Self::InvalidOptions => formatter.write_str("invalid LCEDA export options"),
            Self::NonFiniteScalar(field) => {
                write!(formatter, "LCEDA scalar projection is non-finite: {field}")
            }
            Self::Json => formatter.write_str("LCEDA JSON serialization failed"),
            Self::Archive(message) => write!(formatter, "LCEDA archive failed: {message}"),
        }
    }
}

impl std::error::Error for LcedaExportError {}

/// One finite LCEDA token restored to an exact source-unit scalar.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct LcedaNumericImport {
    /// Stable semantic destination field.
    pub field: String,
    /// Decimal token read from the archive.
    pub source: String,
    /// Unit carried by the archive token.
    pub source_unit: &'static str,
    /// Exact source-unit value restored to HyperCircuit.
    pub restored: String,
}

/// Supported-subset import fact that remains baseline-owned or unrecognized.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum LcedaImportOmission {
    /// Circuit connectivity remains authoritative in the caller-supplied baseline.
    BaselineCircuitTruth,
    /// Footprint, pad, stackup, constraints, and keepout definitions remain baseline-owned.
    BaselinePhysicalDefinitions,
    /// Wire endpoint kinds, page membership, ports, and reusable symbols remain baseline-owned.
    BaselineSchematicTopology,
    /// A typed pin/port endpoint remained anchored to authoritative connectivity.
    AnchoredWireEndpointPreserved {
        wire: String,
        endpoint: &'static str,
    },
    /// A curved baseline outline segment was not replaced by its exported chord.
    CurvedOutlinePreserved { contour: String, segment: usize },
    /// A PCB record was not part of the supported editable subset.
    UnsupportedPcbRecord { kind: String, id: String },
    /// A schematic record was not part of the supported presentation subset.
    UnsupportedSchematicRecord { kind: String, id: String },
}

/// Audited LCEDA Pro supported-subset import against authoritative design truth.
#[derive(Clone, Debug, PartialEq)]
pub struct LcedaProImportReport {
    /// Structurally validated PCB after applying supported editor changes.
    pub layout: PcbLayout,
    /// Number of placement records applied.
    pub placements: usize,
    /// Number of straight route segments applied.
    pub route_segments: usize,
    /// Number of via records applied.
    pub vias: usize,
    /// Number of zone records applied.
    pub zones: usize,
    /// Number of straight outline records applied.
    pub outline_segments: usize,
    /// Every decimal-to-exact source-unit reconstruction.
    pub numeric_imports: Vec<LcedaNumericImport>,
    /// Explicit baseline assumptions and unsupported records.
    pub omissions: Vec<LcedaImportOmission>,
}

/// Audited schematic-presentation import against authoritative connectivity.
#[derive(Clone, Debug, PartialEq)]
pub struct LcedaSchematicImportReport {
    /// Structurally validated schematic after applying supported editor changes.
    pub schematic: SchematicLayout,
    /// Number of symbol transforms applied.
    pub symbols: usize,
    /// Number of wire polylines applied.
    pub wires: usize,
    /// Number of label presentations applied.
    pub labels: usize,
    /// Every decimal schematic coordinate reconstructed exactly.
    pub numeric_imports: Vec<LcedaNumericImport>,
    /// Explicit baseline assumptions and unsupported records.
    pub omissions: Vec<LcedaImportOmission>,
}

/// Failure to parse or safely apply an LCEDA Pro archive.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum LcedaImportError {
    /// Caller-supplied circuit/layout truth is invalid.
    InvalidBaseline,
    /// ZIP structure is malformed, compressed, or exceeds member bounds.
    InvalidArchive,
    /// No unique `.epru` record stream exists.
    MissingRecordStream,
    /// One JSON record line is malformed.
    InvalidRecord(usize),
    /// A required supported-subset field is absent or invalid.
    InvalidField(String),
    /// A stable HyperCircuit target is absent from the baseline.
    UnknownTarget(String),
    /// Applied editor changes produce an invalid retained layout.
    InvalidResult,
}

impl Display for LcedaImportError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidBaseline => formatter.write_str("invalid LCEDA import baseline"),
            Self::InvalidArchive => formatter.write_str("invalid LCEDA Pro archive"),
            Self::MissingRecordStream => formatter.write_str("LCEDA archive has no unique .epru"),
            Self::InvalidRecord(line) => write!(formatter, "invalid LCEDA record line {line}"),
            Self::InvalidField(field) => write!(formatter, "invalid LCEDA field {field}"),
            Self::UnknownTarget(target) => write!(formatter, "unknown LCEDA target {target}"),
            Self::InvalidResult => {
                formatter.write_str("LCEDA edits produce an invalid retained design")
            }
        }
    }
}

impl std::error::Error for LcedaImportError {}

impl LcedaProExportReport {
    /// Exports one circuit with optional authored schematic and PCB views.
    pub fn from_design(
        circuit: &Circuit,
        schematic: Option<&SchematicLayout>,
        layout: Option<&PcbLayout>,
        options: LcedaProExportOptions,
    ) -> Result<Self, LcedaExportError> {
        if options.decimal_places == 0
            || options.decimal_places > 15
            || !circuit.validate().is_valid()
            || schematic.is_some_and(|drawing| !drawing.validate(circuit).is_valid())
            || layout.is_some_and(|board| !board.validate(circuit).is_valid())
        {
            return Err(
                if options.decimal_places == 0 || options.decimal_places > 15 {
                    LcedaExportError::InvalidOptions
                } else {
                    LcedaExportError::InvalidDesign
                },
            );
        }

        let mut emitter = Emitter::new(options);
        emitter
            .omissions
            .push(LcedaExportOmission::EditorConformanceReview);
        emitter.render(circuit, schematic, layout)?;
        let title = circuit.id.as_str();
        let project_json = serde_json::to_string_pretty(&json!({
            "title": title,
            "cbb_project": false,
            "editorVersion": "",
            "introduction": "",
            "description": "Generated by hypercircuit with an auditable finite/loss boundary",
            "tags": "[\"hypercircuit\"]"
        }))
        .map_err(|_| LcedaExportError::Json)?
            + "\n";
        let record_stream = emitter.writer.finish();
        let epru_name = format!("{}.epru", safe_filename(title));
        let files = vec![
            "IMAGE/".to_owned(),
            "project2.json".to_owned(),
            epru_name.clone(),
        ];
        let mut archive = ZipArchive::new();
        archive.add("IMAGE/", Vec::new());
        archive.add("project2.json", project_json.as_bytes().to_vec());
        archive.add(&epru_name, record_stream.as_bytes().to_vec());
        let archive = archive
            .finish()
            .map_err(|error| LcedaExportError::Archive(error.to_string()))?;

        Ok(Self {
            schema: LCEDA_PRO_EXPORT_SCHEMA,
            version: LCEDA_PRO_EXPORT_VERSION,
            archive,
            project_json,
            record_stream,
            files,
            numeric_projections: emitter.projector.projections,
            omissions: emitter.omissions,
        })
    }
}

impl LcedaProImportReport {
    /// Applies the supported PCB editor subset against authoritative circuit/layout truth.
    pub fn from_archive(
        circuit: &Circuit,
        baseline: &PcbLayout,
        archive: &[u8],
        source_length_unit: LcedaSourceLengthUnit,
    ) -> Result<Self, LcedaImportError> {
        if !circuit.validate().is_valid() || !baseline.validate(circuit).is_valid() {
            return Err(LcedaImportError::InvalidBaseline);
        }
        let files = stored_archive_members(archive)?;
        let streams = files
            .iter()
            .filter(|(name, _)| name.ends_with(".epru"))
            .collect::<Vec<_>>();
        if streams.len() != 1 {
            return Err(LcedaImportError::MissingRecordStream);
        }
        let stream =
            std::str::from_utf8(streams[0].1).map_err(|_| LcedaImportError::InvalidArchive)?;
        let records = parse_record_stream(stream)?;
        let mut layout = baseline.clone();
        let mut numeric_imports = Vec::new();
        let mut omissions = vec![
            LcedaImportOmission::BaselineCircuitTruth,
            LcedaImportOmission::BaselinePhysicalDefinitions,
        ];
        let expected_placements = baseline
            .placements
            .iter()
            .map(|placement| placement.instance.as_str().to_owned())
            .collect::<BTreeSet<_>>();
        let expected_route_segments = baseline
            .routes
            .iter()
            .flat_map(|route| {
                route
                    .segments
                    .iter()
                    .enumerate()
                    .filter(|(_, segment)| matches!(segment, PcbRouteSegment::Line(_)))
                    .map(|(index, _)| (route.id.as_str().to_owned(), index))
            })
            .collect::<BTreeSet<_>>();
        let expected_vias = baseline
            .vias
            .iter()
            .map(|via| via.id.as_str().to_owned())
            .collect::<BTreeSet<_>>();
        let expected_zones = baseline
            .zones
            .iter()
            .map(|zone| zone.id.as_str().to_owned())
            .collect::<BTreeSet<_>>();
        let mut expected_outline_segments = baseline
            .outline
            .exterior
            .segments()
            .iter()
            .enumerate()
            .map(|(index, _)| ("board.exterior".to_owned(), index))
            .collect::<BTreeSet<_>>();
        for (cutout_index, cutout) in baseline.outline.cutouts.iter().enumerate() {
            expected_outline_segments.extend(cutout.segments().iter().enumerate().map(
                |(segment_index, _)| (format!("board.cutout[{cutout_index}]"), segment_index),
            ));
        }
        let mut seen_placements = BTreeSet::new();
        let mut seen_route_segments = BTreeSet::new();
        let mut seen_vias = BTreeSet::new();
        let mut seen_zones = BTreeSet::new();
        let mut seen_outline_segments = BTreeSet::new();
        let mut placements = 0;
        let mut route_segments = 0;
        let mut vias = 0;
        let mut zones = 0;
        let mut outline_segments = 0;
        for record in records.iter().filter(|record| record.document == "PCB") {
            match record.kind.as_str() {
                "COMPONENT" if record.body.get("hypercircuitInstance").is_some() => {
                    let instance = string_field(&record.body, "hypercircuitInstance")?;
                    if !seen_placements.insert(instance.to_owned()) {
                        return Err(LcedaImportError::InvalidField(format!(
                            "placement.{instance}"
                        )));
                    }
                    let placement = layout
                        .placements
                        .iter_mut()
                        .find(|placement| placement.instance.as_str() == instance)
                        .ok_or_else(|| LcedaImportError::UnknownTarget(instance.into()))?;
                    placement.position = physical_point(
                        &record.body,
                        "x",
                        "y",
                        &format!("placement.{instance}.position"),
                        source_length_unit,
                        &mut numeric_imports,
                    )?;
                    placement.rotation_degrees = scalar_field(
                        &record.body,
                        "angle",
                        &format!("placement.{instance}.rotation"),
                        "degree",
                        &mut numeric_imports,
                    )?;
                    placement.side = match usize_field(&record.body, "layerId")? {
                        1 => BoardSide::Front,
                        2 => BoardSide::Back,
                        _ => {
                            return Err(LcedaImportError::InvalidField(format!(
                                "placement.{instance}.layerId"
                            )));
                        }
                    };
                    let pattern = string_field(&record.body, "hypercircuitLandPattern")?;
                    let pattern = crate::LandPatternId::new(pattern)
                        .map_err(|_| LcedaImportError::InvalidField("land pattern".into()))?;
                    if placement.land_pattern != pattern {
                        return Err(LcedaImportError::InvalidField(format!(
                            "placement.{instance}.land_pattern"
                        )));
                    }
                    placements += 1;
                }
                "TRACK" if record.body.get("hypercircuitRoute").is_some() => {
                    let id = string_field(&record.body, "hypercircuitRoute")?;
                    let index = usize_field(&record.body, "hypercircuitSegment")?;
                    if !seen_route_segments.insert((id.to_owned(), index)) {
                        return Err(LcedaImportError::InvalidField(format!(
                            "route.{id}.segment[{index}]"
                        )));
                    }
                    let route = layout
                        .routes
                        .iter_mut()
                        .find(|route| route.id.as_str() == id)
                        .ok_or_else(|| LcedaImportError::UnknownTarget(id.into()))?;
                    let start = physical_point(
                        &record.body,
                        "startX",
                        "startY",
                        &format!("route.{id}.segment[{index}].start"),
                        source_length_unit,
                        &mut numeric_imports,
                    )?;
                    let end = physical_point(
                        &record.body,
                        "endX",
                        "endY",
                        &format!("route.{id}.segment[{index}].end"),
                        source_length_unit,
                        &mut numeric_imports,
                    )?;
                    let segment = route
                        .segments
                        .get_mut(index)
                        .ok_or_else(|| LcedaImportError::UnknownTarget(format!("{id}[{index}]")))?;
                    if !matches!(segment, PcbRouteSegment::Line(_)) {
                        return Err(LcedaImportError::InvalidField(format!(
                            "route.{id}.segment[{index}]"
                        )));
                    }
                    *segment = hyperpath::LinePathSegment::new(start, end).into();
                    route.width = physical_field(
                        &record.body,
                        "width",
                        &format!("route.{id}.width"),
                        source_length_unit,
                        &mut numeric_imports,
                    )?;
                    route.layer = TraceLayer(u16_field(&record.body, "hypercircuitLayer")?);
                    let net = string_field(&record.body, "netName")?;
                    if route.net.as_str() != net {
                        return Err(LcedaImportError::InvalidField(format!("route.{id}.net")));
                    }
                    route_segments += 1;
                }
                "VIA"
                    if record
                        .body
                        .get("hypercircuitAuthored")
                        .and_then(Value::as_bool)
                        == Some(false) =>
                {
                    omissions.push(LcedaImportOmission::UnsupportedPcbRecord {
                        kind: record.kind.clone(),
                        id: record.id.clone(),
                    });
                }
                "VIA" if record.body.get("hypercircuitVia").is_some() => {
                    let id = string_field(&record.body, "hypercircuitVia")?;
                    if !seen_vias.insert(id.to_owned()) {
                        return Err(LcedaImportError::InvalidField(format!("via.{id}")));
                    }
                    let via = layout
                        .vias
                        .iter_mut()
                        .find(|via| via.id.as_str() == id)
                        .ok_or_else(|| LcedaImportError::UnknownTarget(id.into()))?;
                    via.center = physical_point(
                        &record.body,
                        "centerX",
                        "centerY",
                        &format!("via.{id}.center"),
                        source_length_unit,
                        &mut numeric_imports,
                    )?;
                    via.land_diameter = physical_field(
                        &record.body,
                        "diameter",
                        &format!("via.{id}.land_diameter"),
                        source_length_unit,
                        &mut numeric_imports,
                    )?;
                    via.drill_diameter = physical_field(
                        &record.body,
                        "holeDiameter",
                        &format!("via.{id}.drill_diameter"),
                        source_length_unit,
                        &mut numeric_imports,
                    )?;
                    via.start_layer =
                        TraceLayer(u16_field(&record.body, "hypercircuitStartLayer")?);
                    via.end_layer = TraceLayer(u16_field(&record.body, "hypercircuitEndLayer")?);
                    via.plating = if bool_field(&record.body, "plated")? {
                        Plating::Plated
                    } else {
                        Plating::NonPlated
                    };
                    if via.net.as_str() != string_field(&record.body, "netName")? {
                        return Err(LcedaImportError::InvalidField(format!("via.{id}.net")));
                    }
                    vias += 1;
                }
                "COPPERAREA" if record.body.get("hypercircuitZone").is_some() => {
                    let id = string_field(&record.body, "hypercircuitZone")?;
                    if !seen_zones.insert(id.to_owned()) {
                        return Err(LcedaImportError::InvalidField(format!("zone.{id}")));
                    }
                    let zone = layout
                        .zones
                        .iter_mut()
                        .find(|zone| zone.id.as_str() == id)
                        .ok_or_else(|| LcedaImportError::UnknownTarget(id.into()))?;
                    zone.boundary = physical_points(
                        &record.body,
                        "points",
                        &format!("zone.{id}.boundary"),
                        source_length_unit,
                        &mut numeric_imports,
                    )?;
                    zone.clearance = physical_field(
                        &record.body,
                        "clearance",
                        &format!("zone.{id}.clearance"),
                        source_length_unit,
                        &mut numeric_imports,
                    )?;
                    zone.layer = TraceLayer(u16_field(&record.body, "hypercircuitLayer")?);
                    if zone.net.as_str() != string_field(&record.body, "netName")? {
                        return Err(LcedaImportError::InvalidField(format!("zone.{id}.net")));
                    }
                    zone.priority = i32_field(&record.body, "priority")?;
                    zones += 1;
                }
                "LINE" if record.body.get("hypercircuitContour").is_some() => {
                    let contour = string_field(&record.body, "hypercircuitContour")?;
                    let index = usize_field(&record.body, "hypercircuitSegment")?;
                    if !seen_outline_segments.insert((contour.to_owned(), index)) {
                        return Err(LcedaImportError::InvalidField(format!(
                            "outline.{contour}[{index}]"
                        )));
                    }
                    let start = physical_point(
                        &record.body,
                        "startX",
                        "startY",
                        &format!("outline.{contour}[{index}].start"),
                        source_length_unit,
                        &mut numeric_imports,
                    )?;
                    let end = physical_point(
                        &record.body,
                        "endX",
                        "endY",
                        &format!("outline.{contour}[{index}].end"),
                        source_length_unit,
                        &mut numeric_imports,
                    )?;
                    let target = outline_contour_mut(&mut layout, contour)?;
                    let Some(existing) = target.segments().get(index) else {
                        return Err(LcedaImportError::UnknownTarget(format!(
                            "{contour}[{index}]"
                        )));
                    };
                    if !matches!(existing, BoardContourSegment::Line(_)) {
                        omissions.push(LcedaImportOmission::CurvedOutlinePreserved {
                            contour: contour.into(),
                            segment: index,
                        });
                        continue;
                    }
                    let mut segments = target.segments().to_vec();
                    segments[index] =
                        BoardContourSegment::Line(hyperpath::LinePathSegment::new(start, end));
                    *target = BoardContour::from_segments(segments);
                    outline_segments += 1;
                }
                "CANVAS" | "LAYER" | "NET" | "PAD_NET" | "META" => {}
                _ => omissions.push(LcedaImportOmission::UnsupportedPcbRecord {
                    kind: record.kind.clone(),
                    id: record.id.clone(),
                }),
            }
        }
        if seen_placements != expected_placements
            || seen_route_segments != expected_route_segments
            || seen_vias != expected_vias
            || seen_zones != expected_zones
            || seen_outline_segments != expected_outline_segments
            || !layout.validate(circuit).is_valid()
        {
            return Err(LcedaImportError::InvalidResult);
        }
        Ok(Self {
            layout,
            placements,
            route_segments,
            vias,
            zones,
            outline_segments,
            numeric_imports,
            omissions,
        })
    }
}

impl LcedaSchematicImportReport {
    /// Applies supported schematic presentation edits without transferring connectivity truth.
    pub fn from_archive(
        circuit: &Circuit,
        baseline: &SchematicLayout,
        archive: &[u8],
    ) -> Result<Self, LcedaImportError> {
        if !circuit.validate().is_valid() || !baseline.validate(circuit).is_valid() {
            return Err(LcedaImportError::InvalidBaseline);
        }
        let files = stored_archive_members(archive)?;
        let streams = files
            .iter()
            .filter(|(name, _)| name.ends_with(".epru"))
            .collect::<Vec<_>>();
        if streams.len() != 1 {
            return Err(LcedaImportError::MissingRecordStream);
        }
        let stream =
            std::str::from_utf8(streams[0].1).map_err(|_| LcedaImportError::InvalidArchive)?;
        let records = parse_record_stream(stream)?;
        let records = records
            .iter()
            .filter(|record| record.document == "SCH_PAGE")
            .collect::<Vec<_>>();
        let mut schematic = baseline.clone();
        let mut numeric_imports = Vec::new();
        let mut omissions = vec![LcedaImportOmission::BaselineSchematicTopology];
        let mut symbols = 0;
        let mut labels = 0;
        let mut seen_symbols = BTreeSet::new();
        let mut seen_labels = BTreeSet::new();
        let mut wire_nets = BTreeMap::<String, String>::new();
        let mut wire_lines =
            BTreeMap::<String, Vec<(usize, SchematicPoint, SchematicPoint)>>::new();
        for record in &records {
            match record.kind.as_str() {
                "COMPONENT" if record.body.get("hypercircuitSymbol").is_some() => {
                    let id = string_field(&record.body, "hypercircuitSymbol")?;
                    if !seen_symbols.insert(id.to_owned()) {
                        return Err(LcedaImportError::InvalidField(format!("symbol.{id}")));
                    }
                    let symbol = schematic
                        .symbols
                        .iter_mut()
                        .find(|symbol| symbol.id.as_str() == id)
                        .ok_or_else(|| LcedaImportError::UnknownTarget(id.into()))?;
                    symbol.position = schematic_point(
                        &record.body,
                        "x",
                        "y",
                        &format!("symbol.{id}.position"),
                        &mut numeric_imports,
                    )?;
                    let rotation = integer_field(&record.body, "rotation")?;
                    if rotation.rem_euclid(90) != 0 {
                        return Err(LcedaImportError::InvalidField(format!(
                            "symbol.{id}.rotation"
                        )));
                    }
                    symbol.quarter_turns = i8::try_from(rotation / 90).map_err(|_| {
                        LcedaImportError::InvalidField(format!("symbol.{id}.rotation"))
                    })?;
                    symbols += 1;
                }
                "NETLABEL" if record.body.get("hypercircuitLabel").is_some() => {
                    let id = string_field(&record.body, "hypercircuitLabel")?;
                    if !seen_labels.insert(id.to_owned()) {
                        return Err(LcedaImportError::InvalidField(format!("label.{id}")));
                    }
                    let label = schematic
                        .labels
                        .iter_mut()
                        .find(|label| label.id.as_str() == id)
                        .ok_or_else(|| LcedaImportError::UnknownTarget(id.into()))?;
                    let net = string_field(&record.body, "net")?;
                    if label.net.as_str() != net {
                        return Err(LcedaImportError::InvalidField(format!("label.{id}.net")));
                    }
                    label.position = schematic_point(
                        &record.body,
                        "x",
                        "y",
                        &format!("label.{id}.position"),
                        &mut numeric_imports,
                    )?;
                    label.text = string_field(&record.body, "text")?.into();
                    labels += 1;
                }
                "WIRE" if record.body.get("hypercircuitWire").is_some() => {
                    let id = string_field(&record.body, "hypercircuitWire")?;
                    let net = string_field(&record.body, "net")?;
                    if wire_nets.insert(id.into(), net.into()).is_some() {
                        return Err(LcedaImportError::InvalidField(format!("wire.{id}")));
                    }
                }
                "LINE" if record.body.get("hypercircuitWire").is_some() => {
                    let id = string_field(&record.body, "hypercircuitWire")?;
                    let index = usize_field(&record.body, "hypercircuitSegment")?;
                    let start = schematic_point(
                        &record.body,
                        "startX",
                        "startY",
                        &format!("wire.{id}.segment[{index}].start"),
                        &mut numeric_imports,
                    )?;
                    let end = schematic_point(
                        &record.body,
                        "endX",
                        "endY",
                        &format!("wire.{id}.segment[{index}].end"),
                        &mut numeric_imports,
                    )?;
                    wire_lines
                        .entry(id.into())
                        .or_default()
                        .push((index, start, end));
                }
                "CANVAS" | "META" | "ATTR" => {}
                _ => omissions.push(LcedaImportOmission::UnsupportedSchematicRecord {
                    kind: record.kind.clone(),
                    id: record.id.clone(),
                }),
            }
        }
        if seen_symbols.len() != schematic.symbols.len()
            || seen_labels.len() != schematic.labels.len()
            || wire_nets.len() != schematic.wires.len()
        {
            return Err(LcedaImportError::InvalidResult);
        }
        let mut wires = 0;
        for (id, net) in wire_nets {
            let wire = schematic
                .wires
                .iter_mut()
                .find(|wire| wire.id.as_str() == id)
                .ok_or_else(|| LcedaImportError::UnknownTarget(id.clone()))?;
            if wire.net.as_str() != net {
                return Err(LcedaImportError::InvalidField(format!("wire.{id}.net")));
            }
            let mut lines = wire_lines
                .remove(&id)
                .ok_or_else(|| LcedaImportError::InvalidField(format!("wire.{id}.segments")))?;
            lines.sort_by_key(|(index, _, _)| *index);
            if lines.is_empty()
                || lines
                    .iter()
                    .enumerate()
                    .any(|(expected, (index, _, _))| expected != *index)
                || lines.windows(2).any(|pair| pair[0].2 != pair[1].1)
            {
                return Err(LcedaImportError::InvalidField(format!(
                    "wire.{id}.segments"
                )));
            }
            let first = lines[0].1.clone();
            let last = lines.last().expect("nonempty wire line set").2.clone();
            apply_schematic_endpoint(&mut wire.from, first, &id, "from", &mut omissions);
            apply_schematic_endpoint(&mut wire.to, last, &id, "to", &mut omissions);
            wire.waypoints = lines.iter().map(|(_, _, end)| end.clone()).collect();
            wire.waypoints.pop();
            wires += 1;
        }
        if !wire_lines.is_empty()
            || wires != schematic.wires.len()
            || !schematic.validate(circuit).is_valid()
        {
            return Err(LcedaImportError::InvalidResult);
        }
        Ok(Self {
            schematic,
            symbols,
            wires,
            labels,
            numeric_imports,
            omissions,
        })
    }
}

struct ImportedRecord {
    document: String,
    kind: String,
    id: String,
    body: Value,
}

fn parse_record_stream(stream: &str) -> Result<Vec<ImportedRecord>, LcedaImportError> {
    let mut document = String::new();
    let mut records = Vec::new();
    for (index, line) in stream.lines().enumerate() {
        let line_number = index + 1;
        let (header, body) = line
            .split_once("||")
            .ok_or(LcedaImportError::InvalidRecord(line_number))?;
        let header: Value = serde_json::from_str(header)
            .map_err(|_| LcedaImportError::InvalidRecord(line_number))?;
        let body: Value = serde_json::from_str(
            body.strip_suffix('|')
                .ok_or(LcedaImportError::InvalidRecord(line_number))?,
        )
        .map_err(|_| LcedaImportError::InvalidRecord(line_number))?;
        let kind = string_field(&header, "type")
            .map_err(|_| LcedaImportError::InvalidRecord(line_number))?;
        if kind == "DOCHEAD" {
            document = string_field(&body, "docType")
                .map_err(|_| LcedaImportError::InvalidRecord(line_number))?
                .into();
            continue;
        }
        let id = header
            .get("id")
            .and_then(Value::as_str)
            .unwrap_or_default()
            .to_owned();
        records.push(ImportedRecord {
            document: document.clone(),
            kind: kind.into(),
            id,
            body,
        });
    }
    Ok(records)
}

fn stored_archive_members(archive: &[u8]) -> Result<BTreeMap<String, Vec<u8>>, LcedaImportError> {
    let mut files = BTreeMap::new();
    let mut offset = 0_usize;
    while archive.get(offset..offset + 4) == Some(&0x0403_4b50_u32.to_le_bytes()) {
        let header = archive
            .get(offset..offset + 30)
            .ok_or(LcedaImportError::InvalidArchive)?;
        let flags = u16::from_le_bytes(
            header[6..8]
                .try_into()
                .map_err(|_| LcedaImportError::InvalidArchive)?,
        );
        let compression = u16::from_le_bytes(
            header[8..10]
                .try_into()
                .map_err(|_| LcedaImportError::InvalidArchive)?,
        );
        if flags != 0 || compression != 0 {
            return Err(LcedaImportError::InvalidArchive);
        }
        let size = u32::from_le_bytes(
            header[18..22]
                .try_into()
                .map_err(|_| LcedaImportError::InvalidArchive)?,
        ) as usize;
        let name_len = u16::from_le_bytes(
            header[26..28]
                .try_into()
                .map_err(|_| LcedaImportError::InvalidArchive)?,
        ) as usize;
        let extra_len = u16::from_le_bytes(
            header[28..30]
                .try_into()
                .map_err(|_| LcedaImportError::InvalidArchive)?,
        ) as usize;
        let name_start = offset
            .checked_add(30)
            .ok_or(LcedaImportError::InvalidArchive)?;
        let content_start = name_start
            .checked_add(name_len)
            .and_then(|value| value.checked_add(extra_len))
            .ok_or(LcedaImportError::InvalidArchive)?;
        let content_end = content_start
            .checked_add(size)
            .ok_or(LcedaImportError::InvalidArchive)?;
        let name = std::str::from_utf8(
            archive
                .get(name_start..name_start + name_len)
                .ok_or(LcedaImportError::InvalidArchive)?,
        )
        .map_err(|_| LcedaImportError::InvalidArchive)?
        .to_owned();
        let contents = archive
            .get(content_start..content_end)
            .ok_or(LcedaImportError::InvalidArchive)?
            .to_vec();
        let expected_crc = u32::from_le_bytes(
            header[14..18]
                .try_into()
                .map_err(|_| LcedaImportError::InvalidArchive)?,
        );
        if crc32(&contents) != expected_crc {
            return Err(LcedaImportError::InvalidArchive);
        }
        if files.insert(name, contents).is_some() {
            return Err(LcedaImportError::InvalidArchive);
        }
        offset = content_end;
    }
    if files.is_empty() {
        return Err(LcedaImportError::InvalidArchive);
    }
    Ok(files)
}

fn string_field<'a>(object: &'a Value, field: &str) -> Result<&'a str, LcedaImportError> {
    object
        .get(field)
        .and_then(Value::as_str)
        .ok_or_else(|| LcedaImportError::InvalidField(field.into()))
}

fn bool_field(object: &Value, field: &str) -> Result<bool, LcedaImportError> {
    object
        .get(field)
        .and_then(Value::as_bool)
        .ok_or_else(|| LcedaImportError::InvalidField(field.into()))
}

fn integer_field(object: &Value, field: &str) -> Result<i64, LcedaImportError> {
    object
        .get(field)
        .and_then(Value::as_i64)
        .ok_or_else(|| LcedaImportError::InvalidField(field.into()))
}

fn usize_field(object: &Value, field: &str) -> Result<usize, LcedaImportError> {
    usize::try_from(integer_field(object, field)?)
        .map_err(|_| LcedaImportError::InvalidField(field.into()))
}

fn u16_field(object: &Value, field: &str) -> Result<u16, LcedaImportError> {
    u16::try_from(integer_field(object, field)?)
        .map_err(|_| LcedaImportError::InvalidField(field.into()))
}

fn i32_field(object: &Value, field: &str) -> Result<i32, LcedaImportError> {
    i32::try_from(integer_field(object, field)?)
        .map_err(|_| LcedaImportError::InvalidField(field.into()))
}

fn number_token(object: &Value, field: &str) -> Result<String, LcedaImportError> {
    match object.get(field) {
        Some(Value::Number(number)) => Ok(number.to_string()),
        Some(Value::String(value)) => Ok(value.clone()),
        _ => Err(LcedaImportError::InvalidField(field.into())),
    }
}

fn scalar_field(
    object: &Value,
    field: &str,
    destination: &str,
    unit: &'static str,
    imports: &mut Vec<LcedaNumericImport>,
) -> Result<Real, LcedaImportError> {
    let token = number_token(object, field)?;
    let value =
        Real::from_str(&token).map_err(|_| LcedaImportError::InvalidField(destination.into()))?;
    imports.push(LcedaNumericImport {
        field: destination.into(),
        source: token,
        source_unit: unit,
        restored: value.to_string(),
    });
    Ok(value)
}

fn schematic_point(
    object: &Value,
    x: &str,
    y: &str,
    destination: &str,
    imports: &mut Vec<LcedaNumericImport>,
) -> Result<SchematicPoint, LcedaImportError> {
    Ok(SchematicPoint::new(
        scalar_field(object, x, &format!("{destination}.x"), "schematic", imports)?,
        scalar_field(object, y, &format!("{destination}.y"), "schematic", imports)?,
    ))
}

fn apply_schematic_endpoint(
    endpoint: &mut SchematicEndpoint,
    imported: SchematicPoint,
    wire: &str,
    endpoint_name: &'static str,
    omissions: &mut Vec<LcedaImportOmission>,
) {
    if let SchematicEndpoint::Junction(point) = endpoint {
        *point = imported;
    } else {
        omissions.push(LcedaImportOmission::AnchoredWireEndpointPreserved {
            wire: wire.into(),
            endpoint: endpoint_name,
        });
    }
}

fn physical_field(
    object: &Value,
    field: &str,
    destination: &str,
    source_length_unit: LcedaSourceLengthUnit,
    imports: &mut Vec<LcedaNumericImport>,
) -> Result<Real, LcedaImportError> {
    let token = number_token(object, field)?;
    let mil =
        Real::from_str(&token).map_err(|_| LcedaImportError::InvalidField(destination.into()))?;
    let value = (mil / source_length_unit.mil_factor())
        .map_err(|_| LcedaImportError::InvalidField(destination.into()))?;
    imports.push(LcedaNumericImport {
        field: destination.into(),
        source: token,
        source_unit: "mil",
        restored: value.to_string(),
    });
    Ok(value)
}

fn physical_point(
    object: &Value,
    x: &str,
    y: &str,
    destination: &str,
    source_length_unit: LcedaSourceLengthUnit,
    imports: &mut Vec<LcedaNumericImport>,
) -> Result<Point2, LcedaImportError> {
    Ok(Point2::new(
        physical_field(
            object,
            x,
            &format!("{destination}.x"),
            source_length_unit,
            imports,
        )?,
        physical_field(
            object,
            y,
            &format!("{destination}.y"),
            source_length_unit,
            imports,
        )?,
    ))
}

fn physical_points(
    object: &Value,
    field: &str,
    destination: &str,
    source_length_unit: LcedaSourceLengthUnit,
    imports: &mut Vec<LcedaNumericImport>,
) -> Result<Vec<Point2>, LcedaImportError> {
    object
        .get(field)
        .and_then(Value::as_array)
        .ok_or_else(|| LcedaImportError::InvalidField(destination.into()))?
        .iter()
        .enumerate()
        .map(|(index, point)| {
            let pair = point
                .as_array()
                .filter(|pair| pair.len() == 2)
                .ok_or_else(|| LcedaImportError::InvalidField(format!("{destination}[{index}]")))?;
            let wrapper = json!({"x":pair[0],"y":pair[1]});
            physical_point(
                &wrapper,
                "x",
                "y",
                &format!("{destination}[{index}]"),
                source_length_unit,
                imports,
            )
        })
        .collect()
}

fn outline_contour_mut<'a>(
    layout: &'a mut PcbLayout,
    contour: &str,
) -> Result<&'a mut BoardContour, LcedaImportError> {
    if contour == "board.exterior" {
        return Ok(&mut layout.outline.exterior);
    }
    let index = contour
        .strip_prefix("board.cutout[")
        .and_then(|value| value.strip_suffix(']'))
        .and_then(|value| value.parse::<usize>().ok())
        .ok_or_else(|| LcedaImportError::InvalidField("outline contour".into()))?;
    layout
        .outline
        .cutouts
        .get_mut(index)
        .ok_or_else(|| LcedaImportError::UnknownTarget(contour.into()))
}

struct Emitter {
    writer: RecordWriter,
    projector: Projector,
    omissions: Vec<LcedaExportOmission>,
}

impl Emitter {
    fn new(options: LcedaProExportOptions) -> Self {
        Self {
            writer: RecordWriter::new(),
            projector: Projector::new(options),
            omissions: Vec::new(),
        }
    }

    fn render(
        &mut self,
        circuit: &Circuit,
        schematic: Option<&SchematicLayout>,
        layout: Option<&PcbLayout>,
    ) -> Result<(), LcedaExportError> {
        if let Some(board) = layout {
            for pattern in &board.land_patterns {
                self.render_footprint(pattern)?;
            }
        }
        self.render_symbols_and_devices(circuit);
        if let Some(drawing) = schematic {
            self.render_schematic(circuit, drawing)?;
        }
        if let Some(board) = layout {
            self.render_pcb(circuit, board)?;
        }
        self.writer.dochead("CONFIG", "CONFIG");
        self.writer
            .record("META", "META", json!({"defaultSheet": ""}));
        self.writer.dochead("FONT", "FONT");
        self.writer.record(
            "FONT",
            "default2",
            json!({"fontFamily":"default2","source":"system"}),
        );
        Ok(())
    }

    fn render_footprint(&mut self, pattern: &LandPattern) -> Result<(), LcedaExportError> {
        let pattern_name = pattern.id.as_str();
        self.writer.dochead(
            "FOOTPRINT",
            &stable_uuid(&format!("footprint:{pattern_name}")),
        );
        self.canvas("mil");
        self.standard_layers();
        for (index, pad) in pattern.pads.iter().enumerate() {
            self.render_pad(pattern, pad, index)?;
        }
        for (index, graphic) in pattern.graphics.iter().enumerate() {
            let layer_id = layer_role_id(&graphic.layer);
            if matches!(graphic.layer, LayerRole::Custom(_)) {
                self.omissions
                    .push(LcedaExportOmission::CustomArtworkLayer {
                        land_pattern: pattern_name.to_owned(),
                        graphic: graphic.id.as_str().to_owned(),
                    });
            }
            let prefix = format!("footprint.{pattern_name}.graphic.{}", graphic.id.as_str());
            let width = match &graphic.stroke_width {
                Some(value) => self.projector.physical(&format!("{prefix}.width"), value)?,
                None => "5".to_owned(),
            };
            match &graphic.primitive {
                LandPatternGraphicPrimitive::Line { start, end } => {
                    let (sx, sy) = self
                        .projector
                        .physical_point(&format!("{prefix}.start"), start)?;
                    let (ex, ey) = self
                        .projector
                        .physical_point(&format!("{prefix}.end"), end)?;
                    self.writer.record("LINE", &stable_uuid(&prefix), json!({
                        "groupId":0,"netName":"","layerId":layer_id,
                        "startX":number(&sx),"startY":number(&sy),"endX":number(&ex),
                        "endY":number(&ey),"width":number(&width),"locked":false,"zIndex":300+index
                    }));
                }
                LandPatternGraphicPrimitive::Circle { center, radius } => {
                    let (x, y) = self
                        .projector
                        .physical_point(&format!("{prefix}.center"), center)?;
                    let radius = self
                        .projector
                        .physical(&format!("{prefix}.radius"), radius)?;
                    self.writer.record("CIRCLE", &stable_uuid(&prefix), json!({
                        "layerId":layer_id,"centerX":number(&x),"centerY":number(&y),
                        "radius":number(&radius),"width":number(&width),"fillColor":"none","zIndex":300+index
                    }));
                }
                LandPatternGraphicPrimitive::Polygon { vertices, filled } => {
                    let points = self
                        .projector
                        .physical_points(&format!("{prefix}.vertices"), vertices)?;
                    self.writer.record(
                        "POLYGON",
                        &stable_uuid(&prefix),
                        json!({
                            "layerId":layer_id,"points":points,"width":number(&width),
                            "fill":filled,"netName":"","zIndex":300+index
                        }),
                    );
                }
                LandPatternGraphicPrimitive::Text {
                    text,
                    position,
                    height,
                    rotation_degrees,
                } => {
                    let (x, y) = self
                        .projector
                        .physical_point(&format!("{prefix}.position"), position)?;
                    let height = self
                        .projector
                        .physical(&format!("{prefix}.height"), height)?;
                    let angle = self.projector.scalar(
                        &format!("{prefix}.rotation"),
                        rotation_degrees,
                        "degree",
                    )?;
                    self.writer.record("STRING", &stable_uuid(&prefix), json!({
                        "layerId":layer_id,"x":number(&x),"y":number(&y),"text":text,
                        "fontFamily":"default","fontSize":number(&height),"angle":number(&angle),
                        "origin":"CENTER_MIDDLE","zIndex":300+index
                    }));
                }
            }
        }
        self.writer.record("META", "META", json!({
            "title":pattern_name,"description":"Generated from retained hypercircuit land-pattern intent",
            "tags":["hypercircuit"]
        }));
        Ok(())
    }

    fn render_pad(
        &mut self,
        pattern: &LandPattern,
        pad: &LandPatternPad,
        index: usize,
    ) -> Result<(), LcedaExportError> {
        let pattern_name = pattern.id.as_str();
        let pad_name = pad.id.as_str();
        let prefix = format!("footprint.{pattern_name}.pad.{pad_name}");
        let (x, y) = self
            .projector
            .physical_point(&format!("{prefix}.center"), &pad.center)?;
        let shape = self.pad_shape(pattern_name, pad_name, &prefix, &pad.shape)?;
        let hole = match &pad.drill {
            None => Value::Null,
            Some(DrillShape::Round { diameter }) => {
                let diameter = self
                    .projector
                    .physical(&format!("{prefix}.drill"), diameter)?;
                json!({"holeType":"ROUND","width":number(&diameter),"height":number(&diameter)})
            }
            Some(DrillShape::Slot { start, end, width }) => {
                self.omissions
                    .push(LcedaExportOmission::RoutedSlotApproximation {
                        land_pattern: pattern_name.to_owned(),
                        pad: pad_name.to_owned(),
                    });
                let dx = end.x.clone() - start.x.clone();
                let dy = end.y.clone() - start.y.clone();
                let span_x = dx.abs() + width.clone();
                let span_y = dy.abs() + width.clone();
                let width = self
                    .projector
                    .physical(&format!("{prefix}.slot_width"), &span_x)?;
                let height = self
                    .projector
                    .physical(&format!("{prefix}.slot_height"), &span_y)?;
                json!({"holeType":"SLOT","width":number(&width),"height":number(&height)})
            }
        };
        if pad.solder_mask_margin.is_some() || pad.paste_margin.is_some() {
            self.omissions.push(LcedaExportOmission::PadProcessMargin {
                land_pattern: pattern_name.to_owned(),
                pad: pad_name.to_owned(),
            });
        }
        let layer_id = pad_layer_id(pad);
        let rotation = self.projector.scalar(
            &format!("{prefix}.rotation"),
            &pad.rotation_degrees,
            "degree",
        )?;
        self.writer.record(
            "PAD",
            &format!("pad_{}", sanitize(pad_name)),
            json!({
                "groupId":0,"netName":"","layerId":layer_id,"num":pad_name,
                "centerX":number(&x),"centerY":number(&y),"padAngle":0,"hole":hole,
                "defaultPad":shape,"specialPad":[],"padOffsetX":0,"padOffsetY":0,
                "relativeAngle":number(&rotation),"plated":matches!(pad.plating, Plating::Plated),
                "padType":"NORMAL","locked":false,"zIndex":100+index,"padLen":0
            }),
        );
        Ok(())
    }

    fn pad_shape(
        &mut self,
        pattern: &str,
        pad: &str,
        prefix: &str,
        shape: &PadShape,
    ) -> Result<Value, LcedaExportError> {
        let dimensions = match shape {
            PadShape::Circle { diameter } => {
                let d = self
                    .projector
                    .physical(&format!("{prefix}.diameter"), diameter)?;
                return Ok(json!({"padType":"ELLIPSE","width":number(&d),"height":number(&d)}));
            }
            PadShape::Rectangle { width, height } => (width.clone(), height.clone(), None, "RECT"),
            PadShape::RoundedRectangle {
                width,
                height,
                corner_radius,
            } => (
                width.clone(),
                height.clone(),
                Some(corner_radius.clone()),
                "RECT",
            ),
            PadShape::Obround { width, height } => (width.clone(), height.clone(), None, "OVAL"),
            PadShape::Polygon { vertices } => {
                self.omissions
                    .push(LcedaExportOmission::PolygonPadApproximation {
                        land_pattern: pattern.to_owned(),
                        pad: pad.to_owned(),
                    });
                let (width, height) = polygon_span(prefix, vertices)?;
                (width, height, None, "RECT")
            }
        };
        let width = self
            .projector
            .physical(&format!("{prefix}.width"), &dimensions.0)?;
        let height = self
            .projector
            .physical(&format!("{prefix}.height"), &dimensions.1)?;
        let radius = dimensions
            .2
            .as_ref()
            .map(|value| self.projector.physical(&format!("{prefix}.radius"), value))
            .transpose()?;
        Ok(
            json!({"padType":dimensions.3,"width":number(&width),"height":number(&height),
            "radius":radius.map_or(Value::from(0), |value| number(&value))}),
        )
    }

    fn render_symbols_and_devices(&mut self, circuit: &Circuit) {
        for instance in &circuit.instances {
            let name = instance.id.as_str();
            let symbol_uuid = stable_uuid(&format!("symbol:{name}"));
            self.writer.dochead("SYMBOL", &symbol_uuid);
            self.writer
                .record("META", "META", json!({"title":name,"prefix":name}));
            self.writer
                .dochead("DEVICE", &stable_uuid(&format!("device:{name}")));
            self.writer.record("META", "META", json!({
                "title":name,"attributes":{
                    "Symbol":symbol_uuid,
                    "Name":instance.part.as_ref().map_or(instance.model.as_str(), |part| part.as_str()),
                    "Designator":name,"Add into BOM":"yes","Convert to PCB":"yes"
                }
            }));
        }
    }

    fn render_schematic(
        &mut self,
        circuit: &Circuit,
        drawing: &SchematicLayout,
    ) -> Result<(), LcedaExportError> {
        self.writer
            .dochead("SCH", &stable_uuid(&format!("sch:{}", circuit.id.as_str())));
        self.writer
            .record("META", "META", json!({"title":circuit.id.as_str()}));
        if !drawing.sheets.is_empty() {
            self.omissions
                .push(LcedaExportOmission::SchematicHierarchyMetadata);
        }
        if !drawing.symbol_definitions.is_empty() {
            self.omissions
                .push(LcedaExportOmission::SchematicSymbolLibrary {
                    definitions: drawing.symbol_definitions.len(),
                    units: drawing
                        .symbol_definitions
                        .iter()
                        .map(|definition| definition.units.len())
                        .sum(),
                    graphics: drawing
                        .symbol_definitions
                        .iter()
                        .flat_map(|definition| &definition.units)
                        .map(|unit| unit.graphics.len())
                        .sum(),
                });
        }
        let pages: Vec<(String, String, Option<String>)> = if drawing.sheets.is_empty() {
            vec![("page-1".to_owned(), circuit.id.as_str().to_owned(), None)]
        } else {
            drawing
                .sheets
                .iter()
                .map(|sheet| {
                    (
                        sheet.id.as_str().to_owned(),
                        sheet.title.clone(),
                        sheet
                            .parent
                            .as_ref()
                            .map(|parent| parent.as_str().to_owned()),
                    )
                })
                .collect()
        };
        for (page_id, title, parent) in pages {
            self.writer
                .dochead("SCH_PAGE", &stable_uuid(&format!("sch-page:{page_id}")));
            self.canvas("schematic");
            self.writer.record(
                "META",
                "META",
                json!({
                    "title":title,"sheetId":page_id,"parentSheetId":parent
                }),
            );
            let selected = |kind: &str, id: &str| -> bool {
                if drawing.sheets.is_empty() {
                    return true;
                }
                let sheet = drawing
                    .sheets
                    .iter()
                    .find(|sheet| sheet.id.as_str() == page_id);
                sheet.is_some_and(|sheet| match kind {
                    "symbol" => sheet.symbols.iter().any(|value| value.as_str() == id),
                    "wire" => sheet.wires.iter().any(|value| value.as_str() == id),
                    "label" => sheet.labels.iter().any(|value| value.as_str() == id),
                    "port" => sheet.ports.iter().any(|value| value.as_str() == id),
                    _ => false,
                })
            };
            for symbol in drawing
                .symbols
                .iter()
                .filter(|symbol| selected("symbol", symbol.id.as_str()))
            {
                let (x, y) = self.projector.schematic_point(
                    &format!("schematic.{}.symbol.{}", page_id, symbol.id.as_str()),
                    &symbol.position,
                )?;
                self.writer.record("COMPONENT", &format!("c_{}", sanitize(symbol.id.as_str())), json!({
                    "x":number(&x),"y":number(&y),"rotation":i32::from(symbol.quarter_turns)*90,
                    "partId":format!("{}.{}", symbol.instance.as_str(), symbol.unit),
                    "hypercircuitSymbol":symbol.id.as_str(),
                    "device":stable_uuid(&format!("device:{}", symbol.instance.as_str())),"zIndex":100
                }));
            }
            for wire in drawing
                .wires
                .iter()
                .filter(|wire| selected("wire", wire.id.as_str()))
            {
                let mut points = Vec::new();
                points.push(
                    endpoint_point(drawing, &wire.from).ok_or(LcedaExportError::InvalidDesign)?,
                );
                points.extend(wire.waypoints.iter().cloned());
                points.push(
                    endpoint_point(drawing, &wire.to).ok_or(LcedaExportError::InvalidDesign)?,
                );
                let points = points
                    .iter()
                    .enumerate()
                    .map(|(index, point)| {
                        self.projector.schematic_point(
                            &format!(
                                "schematic.{page_id}.wire.{}.point[{index}]",
                                wire.id.as_str()
                            ),
                            point,
                        )
                    })
                    .collect::<Result<Vec<_>, _>>()?;
                self.writer
                    .wire(wire.id.as_str(), wire.net.as_str(), &points);
            }
            for label in drawing
                .labels
                .iter()
                .filter(|label| selected("label", label.id.as_str()))
            {
                let (x, y) = self.projector.schematic_point(
                    &format!("schematic.{page_id}.label.{}", label.id.as_str()),
                    &label.position,
                )?;
                self.writer.record(
                    "NETLABEL",
                    label.id.as_str(),
                    json!({
                        "x":number(&x),"y":number(&y),"net":label.net.as_str(),"text":label.text,
                        "hypercircuitLabel":label.id.as_str()
                    }),
                );
            }
        }
        Ok(())
    }

    fn render_pcb(&mut self, circuit: &Circuit, board: &PcbLayout) -> Result<(), LcedaExportError> {
        self.writer
            .dochead("PCB", &stable_uuid(&format!("pcb:{}", board.id.as_str())));
        self.canvas("mil");
        self.standard_layers();
        let layer_ids = conductor_layer_ids(board);
        for (layer, id, name) in &layer_ids {
            self.writer.record("LAYER", &format!("layer_{}", layer.0), json!({
                "layerId":id,"layerType":"SIGNAL","layerName":name,"use":true,"show":true,"locked":false
            }));
        }
        for net in &circuit.nets {
            self.writer.record(
                "NET",
                &format!("[\"NET\",\"{}\"]", net.id.as_str()),
                json!({
                    "netType":null,"retLine":true,"differentialName":null,"isPositiveNet":false
                }),
            );
        }
        if board.stackup.layers.len() > 2 {
            self.omissions.push(LcedaExportOmission::DetailedStackup);
        }
        if !board.placement_constraints.is_empty() {
            self.omissions
                .push(LcedaExportOmission::PlacementConstraints(
                    board.placement_constraints.len(),
                ));
        }
        if !board.keepouts.is_empty() {
            self.omissions
                .push(LcedaExportOmission::Keepouts(board.keepouts.len()));
        }
        for (index, placement) in board.placements.iter().enumerate() {
            let prefix = format!("pcb.placement.{}", placement.instance.as_str());
            let (x, y) = self
                .projector
                .physical_point(&format!("{prefix}.position"), &placement.position)?;
            let angle = self.projector.scalar(
                &format!("{prefix}.rotation"),
                &placement.rotation_degrees,
                "degree",
            )?;
            let component_id = format!("pcb_{}", sanitize(placement.instance.as_str()));
            self.writer.record("COMPONENT", &component_id, json!({
                "layerId":if placement.side == BoardSide::Front {1} else {2},
                "x":number(&x),"y":number(&y),"angle":number(&angle),
                "footprint":stable_uuid(&format!("footprint:{}", placement.land_pattern.as_str())),
                "hypercircuitInstance":placement.instance.as_str(),
                "hypercircuitLandPattern":placement.land_pattern.as_str(),
                "locked":false,"zIndex":index+1
            }));
            if let (Some(instance), Some(pattern)) = (
                circuit
                    .instances
                    .iter()
                    .find(|value| value.id == placement.instance),
                board
                    .land_patterns
                    .iter()
                    .find(|value| value.id == placement.land_pattern),
            ) {
                for mapping in &pattern.pin_map {
                    if let Some(binding) =
                        instance.pins.iter().find(|value| value.pin == mapping.pin)
                    {
                        self.writer.record(
                            "PAD_NET",
                            &format!(
                                "[\"PAD_NET\",\"{component_id}\",\"{}\",\"pad_{}\"]",
                                mapping.pad.as_str(),
                                sanitize(mapping.pad.as_str())
                            ),
                            json!({"padNet":binding.net.as_str(),"padLen":0}),
                        );
                    }
                }
            }
        }
        for route in &board.routes {
            let layer_id = lookup_layer(&layer_ids, route.layer);
            let width = self.projector.physical(
                &format!("pcb.route.{}.width", route.id.as_str()),
                &route.width,
            )?;
            for (index, segment) in route.segments.iter().enumerate() {
                let segment = match segment {
                    crate::PcbRouteSegment::Line(segment) => segment,
                    crate::PcbRouteSegment::CircularArc(_) => {
                        self.omissions.push(LcedaExportOmission::CircularRouteArc {
                            route: route.id.as_str().to_owned(),
                            segment: index,
                        });
                        continue;
                    }
                    crate::PcbRouteSegment::CubicBezier(_) => {
                        self.omissions.push(LcedaExportOmission::CubicRouteBezier {
                            route: route.id.as_str().to_owned(),
                            segment: index,
                        });
                        continue;
                    }
                };
                let (sx, sy) = self.projector.physical_point(
                    &format!("pcb.route.{}.segment[{index}].start", route.id.as_str()),
                    segment.start(),
                )?;
                let (ex, ey) = self.projector.physical_point(
                    &format!("pcb.route.{}.segment[{index}].end", route.id.as_str()),
                    segment.end(),
                )?;
                self.writer.record(
                    "TRACK",
                    &format!("{}_{}", route.id.as_str(), index),
                    json!({
                        "netName":route.net.as_str(),"layerId":layer_id,"startX":number(&sx),
                        "startY":number(&sy),"endX":number(&ex),"endY":number(&ey),
                        "hypercircuitRoute":route.id.as_str(),"hypercircuitSegment":index,
                        "hypercircuitLayer":route.layer.0,
                        "width":number(&width),"locked":false,"zIndex":1000+index
                    }),
                );
            }
        }
        let stitching = board.realize_stitching_vias();
        for via in board.vias.iter().chain(&stitching.vias) {
            let authored = board.vias.iter().any(|candidate| candidate.id == via.id);
            let prefix = format!("pcb.via.{}", via.id.as_str());
            let (x, y) = self
                .projector
                .physical_point(&format!("{prefix}.center"), &via.center)?;
            let diameter = self
                .projector
                .physical(&format!("{prefix}.diameter"), &via.land_diameter)?;
            let drill = self
                .projector
                .physical(&format!("{prefix}.drill"), &via.drill_diameter)?;
            if via.plating != Plating::Plated {
                self.omissions.push(LcedaExportOmission::ViaPlatingReview {
                    via: via.id.as_str().to_owned(),
                });
            }
            if !matches!(via.mask.front, crate::ViaMaskDisposition::Unspecified)
                || !matches!(via.mask.back, crate::ViaMaskDisposition::Unspecified)
            {
                self.omissions.push(LcedaExportOmission::ViaMaskIntent {
                    via: via.id.as_str().to_owned(),
                });
            }
            self.writer.record(
                "VIA",
                via.id.as_str(),
                json!({
                    "netName":via.net.as_str(),"centerX":number(&x),"centerY":number(&y),
                    "diameter":number(&diameter),"holeDiameter":number(&drill),
                    "startLayer":lookup_layer(&layer_ids, via.start_layer),
                    "endLayer":lookup_layer(&layer_ids, via.end_layer),
                    "hypercircuitStartLayer":via.start_layer.0,
                    "hypercircuitEndLayer":via.end_layer.0,
                    "hypercircuitVia":via.id.as_str(),
                    "hypercircuitAuthored":authored,
                    "plated":matches!(via.plating, Plating::Plated),"locked":false
                }),
            );
        }
        for zone in &board.zones {
            let prefix = format!("pcb.zone.{}", zone.id.as_str());
            let points = self
                .projector
                .physical_points(&format!("{prefix}.boundary"), &zone.boundary)?;
            let clearance = self
                .projector
                .physical(&format!("{prefix}.clearance"), &zone.clearance)?;
            let fill = match &zone.fill {
                CopperZoneFill::Solid => json!({"style":"SOLID"}),
                CopperZoneFill::Hatched {
                    line_width,
                    gap,
                    angle_degrees,
                } => {
                    let width = self
                        .projector
                        .physical(&format!("{prefix}.hatch.line_width"), line_width)?;
                    let gap = self
                        .projector
                        .physical(&format!("{prefix}.hatch.gap"), gap)?;
                    let angle = self.projector.scalar(
                        &format!("{prefix}.hatch.angle"),
                        angle_degrees,
                        "degree",
                    )?;
                    json!({
                        "style":"HATCH","lineWidth":number(&width),"gap":number(&gap),
                        "angle":number(&angle)
                    })
                }
            };
            let connection = match &zone.connection {
                CopperZoneConnection::Solid => json!({"style":"SOLID"}),
                CopperZoneConnection::Isolated => json!({"style":"ISOLATED"}),
                CopperZoneConnection::ThermalRelief {
                    air_gap,
                    spoke_width,
                    spoke_count,
                } => {
                    let gap = self
                        .projector
                        .physical(&format!("{prefix}.thermal.air_gap"), air_gap)?;
                    let width = self
                        .projector
                        .physical(&format!("{prefix}.thermal.spoke_width"), spoke_width)?;
                    json!({
                        "style":"THERMAL","airGap":number(&gap),
                        "spokeWidth":number(&width),"spokeCount":spoke_count
                    })
                }
            };
            let minimum_island_area = match &zone.islands.minimum_area {
                Some(area) => Some(
                    self.projector
                        .physical_area(&format!("{prefix}.islands.minimum_area"), area)?,
                ),
                None => None,
            };
            let islands = json!({
                "removeUnconnected":zone.islands.remove_unconnected,
                "minimumArea":minimum_island_area.as_ref().map(|area| number(area))
            });
            let stitching = match &zone.stitching {
                Some(policy) => {
                    let pitch = self
                        .projector
                        .physical(&format!("{prefix}.stitching.pitch"), &policy.pitch)?;
                    let clearance = self.projector.physical(
                        &format!("{prefix}.stitching.edge_clearance"),
                        &policy.edge_clearance,
                    )?;
                    let land = self.projector.physical(
                        &format!("{prefix}.stitching.land_diameter"),
                        &policy.land_diameter,
                    )?;
                    let drill = self.projector.physical(
                        &format!("{prefix}.stitching.drill_diameter"),
                        &policy.drill_diameter,
                    )?;
                    json!({
                        "pitch":number(&pitch),"edgeClearance":number(&clearance),
                        "startLayer":policy.start_layer.0,"endLayer":policy.end_layer.0,
                        "landDiameter":number(&land),"drillDiameter":number(&drill),
                        "maximumVias":policy.maximum_vias
                    })
                }
                None => Value::Null,
            };
            self.writer.record(
                "COPPERAREA",
                zone.id.as_str(),
                json!({
                    "netName":zone.net.as_str(),"layerId":lookup_layer(&layer_ids, zone.layer),
                    "points":points,"fillStyle":fill["style"],"locked":false,
                    "hypercircuitLayer":zone.layer.0,
                    "hypercircuitZone":zone.id.as_str(),
                    "hypercircuitFill":fill,"hypercircuitConnection":connection,
                    "hypercircuitIslands":islands,"hypercircuitStitching":stitching,
                    "clearance":number(&clearance),"priority":zone.priority
                }),
            );
            self.omissions
                .push(LcedaExportOmission::ZonePolicyExtension {
                    zone: zone.id.as_str().into(),
                });
        }
        self.render_outline("board.exterior", &board.outline.exterior, false)?;
        for (index, cutout) in board.outline.cutouts.iter().enumerate() {
            self.render_outline(&format!("board.cutout[{index}]"), cutout, true)?;
        }
        self.writer.record("META", "META", json!({
            "title":board.id.as_str(),"board":stable_uuid(&format!("board:{}", board.id.as_str()))
        }));
        Ok(())
    }

    fn render_outline(
        &mut self,
        id: &str,
        contour: &crate::BoardContour,
        cutout: bool,
    ) -> Result<(), LcedaExportError> {
        for (index, segment) in contour.segments().iter().enumerate() {
            let start = segment.start();
            let end = segment.end();
            if !matches!(segment, crate::BoardContourSegment::Line(_)) {
                self.omissions
                    .push(LcedaExportOmission::CurvedBoardContourChord {
                        contour: id.to_owned(),
                        segment: index,
                    });
            }
            let (sx, sy) = self
                .projector
                .physical_point(&format!("pcb.{id}[{index}].start"), start)?;
            let (ex, ey) = self
                .projector
                .physical_point(&format!("pcb.{id}[{index}].end"), end)?;
            self.writer.record(
                "LINE",
                &format!("{}_{}", sanitize(id), index),
                json!({
                    "layerId":11,"startX":number(&sx),"startY":number(&sy),
                    "endX":number(&ex),"endY":number(&ey),"width":1,
                    "hypercircuitContour":id,"hypercircuitSegment":index,
                    "boardCutout":cutout,"locked":false
                }),
            );
        }
        Ok(())
    }

    fn canvas(&mut self, unit: &str) {
        self.writer.record(
            "CANVAS",
            "CANVAS",
            json!({
                "originX":0,"originY":0,"unit":unit,"gridXSize":10,"gridYSize":10,
                "snapXSize":1,"snapYSize":1,"gridType":"GRID"
            }),
        );
    }

    fn standard_layers(&mut self) {
        for (id, kind, name) in [
            (1, "TOP", "Top Layer"),
            (2, "BOTTOM", "Bottom Layer"),
            (3, "TOP_SILK", "Top Silkscreen Layer"),
            (4, "BOT_SILK", "Bottom Silkscreen Layer"),
            (5, "TOP_SOLDER_MASK", "Top Solder Mask Layer"),
            (6, "BOT_SOLDER_MASK", "Bottom Solder Mask Layer"),
            (7, "TOP_PASTE_MASK", "Top Paste Mask Layer"),
            (8, "BOT_PASTE_MASK", "Bottom Paste Mask Layer"),
            (11, "OUTLINE", "Board Outline Layer"),
            (12, "MULTI", "Multi-Layer"),
            (13, "DOCUMENT", "Document Layer"),
        ] {
            self.writer.record("LAYER", &format!("[\"LAYER\",{id}]"), json!({
                "layerId":id,"layerType":kind,"layerName":name,"use":true,"show":true,"locked":false
            }));
        }
    }
}

struct Projector {
    options: LcedaProExportOptions,
    projections: Vec<LcedaNumericProjection>,
}

impl Projector {
    fn new(options: LcedaProExportOptions) -> Self {
        Self {
            options,
            projections: Vec::new(),
        }
    }

    fn physical(&mut self, field: &str, value: &Real) -> Result<String, LcedaExportError> {
        let mil = value.clone() * self.options.source_length_unit.mil_factor();
        self.scalar(field, &mil, "mil")
    }

    fn physical_area(&mut self, field: &str, value: &Real) -> Result<String, LcedaExportError> {
        let factor = self.options.source_length_unit.mil_factor();
        let square_mil = value.clone() * factor.clone() * factor;
        self.scalar(field, &square_mil, "mil^2")
    }

    fn scalar(
        &mut self,
        field: &str,
        value: &Real,
        unit: &'static str,
    ) -> Result<String, LcedaExportError> {
        let Some(finite) = value.to_f64_lossy().filter(|value| value.is_finite()) else {
            return Err(LcedaExportError::NonFiniteScalar(field.to_owned()));
        };
        let mut emitted = format!("{:.*}", self.options.decimal_places, finite);
        trim_number(&mut emitted);
        self.projections.push(LcedaNumericProjection {
            field: field.to_owned(),
            source: value.to_string(),
            emitted_unit: unit,
            emitted: emitted.clone(),
        });
        Ok(emitted)
    }

    fn physical_point(
        &mut self,
        field: &str,
        point: &Point2,
    ) -> Result<(String, String), LcedaExportError> {
        Ok((
            self.physical(&format!("{field}.x"), &point.x)?,
            self.physical(&format!("{field}.y"), &point.y)?,
        ))
    }

    fn schematic_point(
        &mut self,
        field: &str,
        point: &SchematicPoint,
    ) -> Result<(String, String), LcedaExportError> {
        Ok((
            self.scalar(&format!("{field}.x"), &point.x, "schematic")?,
            self.scalar(&format!("{field}.y"), &point.y, "schematic")?,
        ))
    }

    fn physical_points(
        &mut self,
        field: &str,
        points: &[Point2],
    ) -> Result<Vec<Value>, LcedaExportError> {
        points
            .iter()
            .enumerate()
            .map(|(index, point)| {
                let (x, y) = self.physical_point(&format!("{field}[{index}]"), point)?;
                Ok(json!([number(&x), number(&y)]))
            })
            .collect()
    }
}

struct RecordWriter {
    output: String,
    document_ticket: usize,
    ticket: usize,
}

impl RecordWriter {
    fn new() -> Self {
        Self {
            output: String::new(),
            document_ticket: 1,
            ticket: 1,
        }
    }

    fn dochead(&mut self, doc_type: &str, uuid: &str) {
        let header = json!({"type":"DOCHEAD","ticket":self.document_ticket});
        let body = json!({
            "docType":doc_type,"client":"hypercircuit","uuid":uuid,"updateTime":0,
            "version":"0","editVersion":"hypercircuit-1","user":{
                "uuid":stable_uuid("hypercircuit-user"),"nickname":"hypercircuit",
                "username":"hypercircuit","avatar":""
            }
        });
        self.push(header, body);
        self.document_ticket += 1;
        self.ticket = 1;
    }

    fn record(&mut self, kind: &str, id: &str, body: Value) {
        let header = json!({"type":kind,"ticket":self.ticket,"id":id});
        self.push(header, body);
        self.ticket += 1;
    }

    fn wire(&mut self, id: &str, net: &str, points: &[(String, String)]) {
        self.record(
            "WIRE",
            id,
            json!({"locked":false,"net":net,"hypercircuitWire":id}),
        );
        for (index, segment) in points.windows(2).enumerate() {
            self.record(
                "LINE",
                &format!("{id}_line_{index}"),
                json!({
                    "lineGroup":id,"startX":number(&segment[0].0),"startY":number(&segment[0].1),
                    "endX":number(&segment[1].0),"endY":number(&segment[1].1),
                    "hypercircuitWire":id,"hypercircuitSegment":index,"fillColor":"none"
                }),
            );
        }
        self.record(
            "ATTR",
            &format!("{id}_net"),
            json!({"parentId":id,"key":"NET","value":net}),
        );
    }

    fn push(&mut self, header: Value, body: Value) {
        self.output.push_str(&header.to_string());
        self.output.push_str("||");
        self.output.push_str(&body.to_string());
        self.output.push_str("|\n");
    }

    fn finish(self) -> String {
        self.output
    }
}

fn endpoint_point(
    drawing: &SchematicLayout,
    endpoint: &SchematicEndpoint,
) -> Option<SchematicPoint> {
    match endpoint {
        SchematicEndpoint::Pin { symbol, pin } => {
            let symbol = drawing
                .symbols
                .iter()
                .find(|candidate| candidate.id == *symbol)?;
            let pin = drawing
                .symbol_unit(symbol)?
                .pins
                .iter()
                .find(|candidate| candidate.pin == *pin)?;
            Some(SchematicPoint::new(
                symbol.position.x.clone() + pin.position.x.clone(),
                symbol.position.y.clone() + pin.position.y.clone(),
            ))
        }
        SchematicEndpoint::Port(port) => drawing
            .ports
            .iter()
            .find(|candidate| candidate.port == *port)
            .map(|value| value.position.clone()),
        SchematicEndpoint::SheetPort(port) => drawing
            .sheet_ports
            .iter()
            .find(|candidate| candidate.id == *port)
            .map(|value| value.position.clone()),
        SchematicEndpoint::Junction(point) => Some(point.clone()),
    }
}

fn layer_role_id(role: &LayerRole) -> usize {
    match role {
        LayerRole::Copper(layer) => {
            if layer.0 == 0 {
                1
            } else {
                2
            }
        }
        LayerRole::FrontSolderMask => 5,
        LayerRole::BackSolderMask => 6,
        LayerRole::FrontPaste => 7,
        LayerRole::BackPaste => 8,
        LayerRole::FrontSilkscreen => 3,
        LayerRole::BackSilkscreen => 4,
        LayerRole::EdgeCuts => 11,
        LayerRole::Fabrication | LayerRole::Courtyard | LayerRole::Custom(_) => 13,
    }
}

fn pad_layer_id(pad: &LandPatternPad) -> usize {
    if pad.drill.is_some() || pad.copper_layers.len() > 1 {
        12
    } else if pad.copper_layers.first().is_some_and(|layer| layer.0 > 0) {
        2
    } else {
        1
    }
}

fn conductor_layer_ids(layout: &PcbLayout) -> Vec<(TraceLayer, usize, String)> {
    let layers = layout
        .stackup
        .layers
        .iter()
        .filter_map(|layer| match layer.kind {
            StackupLayerKind::Conductor(id) => Some((id, layer.name.clone())),
            _ => None,
        })
        .collect::<Vec<_>>();
    layers
        .iter()
        .enumerate()
        .map(|(index, (layer, name))| {
            let id = if index == 0 {
                1
            } else if index + 1 == layers.len() {
                2
            } else {
                20 + index
            };
            (*layer, id, name.clone())
        })
        .collect()
}

fn lookup_layer(layers: &[(TraceLayer, usize, String)], target: TraceLayer) -> usize {
    layers
        .iter()
        .find_map(|(layer, id, _)| (*layer == target).then_some(*id))
        .unwrap_or(13)
}

fn polygon_span(field: &str, vertices: &[Point2]) -> Result<(Real, Real), LcedaExportError> {
    let Some(first) = vertices.first() else {
        return Ok((Real::zero(), Real::zero()));
    };
    let (mut min_x, mut max_x, mut min_y, mut max_y) = (
        first.x.clone(),
        first.x.clone(),
        first.y.clone(),
        first.y.clone(),
    );
    for (index, point) in vertices[1..].iter().enumerate() {
        update_extrema(
            &format!("{field}.vertices[{}].x", index + 1),
            &point.x,
            &mut min_x,
            &mut max_x,
        )?;
        update_extrema(
            &format!("{field}.vertices[{}].y", index + 1),
            &point.y,
            &mut min_y,
            &mut max_y,
        )?;
    }
    Ok((max_x - min_x, max_y - min_y))
}

fn update_extrema(
    field: &str,
    value: &Real,
    minimum: &mut Real,
    maximum: &mut Real,
) -> Result<(), LcedaExportError> {
    match (value.clone() - minimum.clone()).refine_sign_until(-64) {
        Some(RealSign::Negative) => *minimum = value.clone(),
        Some(RealSign::Zero | RealSign::Positive) => {}
        None => return Err(LcedaExportError::NonFiniteScalar(field.to_owned())),
    }
    match (value.clone() - maximum.clone()).refine_sign_until(-64) {
        Some(RealSign::Positive) => *maximum = value.clone(),
        Some(RealSign::Zero | RealSign::Negative) => {}
        None => return Err(LcedaExportError::NonFiniteScalar(field.to_owned())),
    }
    Ok(())
}

fn number(token: &str) -> Value {
    serde_json::from_str(token).unwrap_or_else(|_| Value::String(token.to_owned()))
}

fn trim_number(value: &mut String) {
    if value.contains('.') {
        while value.ends_with('0') {
            value.pop();
        }
        if value.ends_with('.') {
            value.pop();
        }
    }
    if value == "-0" {
        *value = "0".to_owned();
    }
}

fn sanitize(value: &str) -> String {
    value
        .chars()
        .map(|ch| if ch.is_ascii_alphanumeric() { ch } else { '_' })
        .collect()
}

fn safe_filename(value: &str) -> String {
    let safe = sanitize(value);
    if safe.is_empty() {
        "hypercircuit".to_owned()
    } else {
        safe
    }
}

fn stable_uuid(seed: &str) -> String {
    let a = fnv64(seed.as_bytes(), 0xcbf2_9ce4_8422_2325);
    let b = fnv64(seed.as_bytes(), 0x8422_2325_cbf2_9ce4);
    format!("{a:016x}{b:016x}")
}

fn fnv64(bytes: &[u8], offset: u64) -> u64 {
    bytes.iter().fold(offset, |hash, byte| {
        (hash ^ u64::from(*byte)).wrapping_mul(0x1000_0000_01b3)
    })
}

struct ZipArchive {
    files: Vec<(String, Vec<u8>)>,
}

impl ZipArchive {
    fn new() -> Self {
        Self { files: Vec::new() }
    }
    fn add(&mut self, name: &str, contents: Vec<u8>) {
        self.files.push((name.to_owned(), contents));
    }

    fn finish(self) -> io::Result<Vec<u8>> {
        let mut output = Vec::new();
        let mut central = Vec::new();
        let count = u16::try_from(self.files.len()).map_err(size_error)?;
        for (name, contents) in self.files {
            let offset = u32::try_from(output.len()).map_err(size_error)?;
            let name = name.as_bytes();
            let name_len = u16::try_from(name.len()).map_err(size_error)?;
            let size = u32::try_from(contents.len()).map_err(size_error)?;
            let crc = crc32(&contents);
            zip_local_header(&mut output, crc, size, name_len);
            output.write_all(name)?;
            output.write_all(&contents)?;
            zip_central_header(&mut central, crc, size, name_len, offset);
            central.write_all(name)?;
        }
        let central_offset = u32::try_from(output.len()).map_err(size_error)?;
        let central_size = u32::try_from(central.len()).map_err(size_error)?;
        output.write_all(&central)?;
        put_u32(&mut output, 0x0605_4b50);
        put_u16(&mut output, 0);
        put_u16(&mut output, 0);
        put_u16(&mut output, count);
        put_u16(&mut output, count);
        put_u32(&mut output, central_size);
        put_u32(&mut output, central_offset);
        put_u16(&mut output, 0);
        Ok(output)
    }
}

fn zip_local_header(out: &mut Vec<u8>, crc: u32, size: u32, name_len: u16) {
    put_u32(out, 0x0403_4b50);
    put_u16(out, 20);
    put_u16(out, 0);
    put_u16(out, 0);
    put_u16(out, 0);
    put_u16(out, 0x0021);
    put_u32(out, crc);
    put_u32(out, size);
    put_u32(out, size);
    put_u16(out, name_len);
    put_u16(out, 0);
}

fn zip_central_header(out: &mut Vec<u8>, crc: u32, size: u32, name_len: u16, offset: u32) {
    put_u32(out, 0x0201_4b50);
    put_u16(out, 20);
    put_u16(out, 20);
    put_u16(out, 0);
    put_u16(out, 0);
    put_u16(out, 0);
    put_u16(out, 0x0021);
    put_u32(out, crc);
    put_u32(out, size);
    put_u32(out, size);
    put_u16(out, name_len);
    put_u16(out, 0);
    put_u16(out, 0);
    put_u16(out, 0);
    put_u16(out, 0);
    put_u32(out, 0);
    put_u32(out, offset);
}

fn size_error<T>(_error: T) -> io::Error {
    io::Error::new(
        io::ErrorKind::InvalidInput,
        "ZIP field exceeds format limit",
    )
}
fn put_u16(out: &mut Vec<u8>, value: u16) {
    out.extend_from_slice(&value.to_le_bytes());
}
fn put_u32(out: &mut Vec<u8>, value: u32) {
    out.extend_from_slice(&value.to_le_bytes());
}

fn crc32(bytes: &[u8]) -> u32 {
    let mut crc = 0xffff_ffff_u32;
    for byte in bytes {
        crc ^= u32::from(*byte);
        for _ in 0..8 {
            let mask = 0_u32.wrapping_sub(crc & 1);
            crc = (crc >> 1) ^ (0xedb8_8320 & mask);
        }
    }
    !crc
}
