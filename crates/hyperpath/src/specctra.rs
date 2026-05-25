//! Exact PCB route interchange records for Specctra DSN/SES-style flows.
//!
//! DSN/SES exchange is an autorouter boundary: Lee and Hightower style search
//! may produce route candidates outside the exact stack, but those candidates
//! should enter `hyperpath` as fixed-grid exact geometry before clearance
//! predicates certify topology. This module models validated route records and
//! a conservative S-expression subset: canonical `(routes ...)` records,
//! DSN/SES-style envelopes that contain route records, and multi-segment
//! `(path ...)` wires. It deliberately lowers syntax to exact fixed-grid
//! records before geometry import. The boundary follows Yap, "Towards Exact
//! Geometric Computation," *Computational Geometry* 7.1-2 (1997).

use hyperlimit::Point2;
use hyperreal::{Rational, Real};
use std::fmt::Write;

use crate::pcb::{NetId, PcbTrace, PcbViaStack, TraceLayer, ViaDrillIntent};
use crate::provenance::{PathProvenance, PathSourceFormat};
use crate::routing::{MeanderKeepout, MeanderObstacle};
use crate::segment::LinePathSegment;
use crate::specctra_syntax::{is_bare_atom, tokenize, write_atom};
use crate::swept::SweptLineSegment;

/// Exact route-level net alias retained from a Specctra DSN/SES-style file.
///
/// Real DSN/SES files carry human net names as well as router-internal
/// identifiers. This canonical subset keeps the alias table separate from
/// geometric records: `(net N NAME)` records validate that `N` lowers to the
/// exact [`NetId`] used by wires and vias while retaining `NAME` for diagnostics
/// and round-trips. The split follows Yap's exact object boundary: net labels
/// are source metadata, while numeric net ids remain the exact predicate key.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SpecctraNetAlias {
    /// Exact numeric net identifier used by route geometry.
    pub net: NetId,
    /// Source net name as one canonical non-whitespace atom.
    pub name: String,
}

/// Exact route-level layer alias retained from a Specctra DSN/SES-style file.
///
/// Layer names are source metadata, not geometric predicates. The canonical
/// `(layer N NAME)` record maps a human board-layer name onto the exact
/// [`TraceLayer`] identifier used by wires and vias. This keeps diagnostics and
/// interchange round-trips close to DSN/SES practice while preserving Yap's
/// object split: route predicates consume exact numeric layer ids.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SpecctraLayerAlias {
    /// Exact numeric layer identifier used by route geometry.
    pub layer: TraceLayer,
    /// Source layer name as one canonical non-whitespace atom.
    pub name: String,
}

/// Exact straight trace record in a Specctra DSN/SES-style route exchange.
#[derive(Clone, Debug, PartialEq)]
pub struct SpecctraTraceRecord {
    /// Net identifier.
    pub net: NetId,
    /// Copper layer identifier.
    pub layer: TraceLayer,
    /// Exact route start point.
    pub start: Point2,
    /// Exact route end point.
    pub end: Point2,
    /// Exact trace width.
    pub width: Real,
    /// Source provenance for the route token.
    pub provenance: PathProvenance,
}

/// Raw fixed-grid trace token lowered from a DSN/SES route file.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SpecctraGridTraceRecord {
    /// Net identifier.
    pub net: NetId,
    /// Copper layer identifier.
    pub layer: TraceLayer,
    /// Start X coordinate in source grid units.
    pub start_x: i64,
    /// Start Y coordinate in source grid units.
    pub start_y: i64,
    /// End X coordinate in source grid units.
    pub end_x: i64,
    /// End Y coordinate in source grid units.
    pub end_y: i64,
    /// Trace width in source grid units.
    pub width: i64,
    /// Denominator of one source unit.
    pub grid_denominator: u64,
}

/// Exact via record in a Specctra DSN/SES-style route exchange.
#[derive(Clone, Debug, PartialEq)]
pub struct SpecctraViaRecord {
    /// Net identifier.
    pub net: NetId,
    /// Inclusive first copper layer touched by this via.
    pub start_layer: TraceLayer,
    /// Inclusive final copper layer touched by this via.
    pub end_layer: TraceLayer,
    /// Exact via center.
    pub center: Point2,
    /// Exact copper land diameter.
    pub land_diameter: Real,
    /// Exact drill diameter.
    pub drill_diameter: Real,
    /// Retained drill plating intent from the route interchange boundary.
    pub drill_intent: ViaDrillIntent,
    /// Source provenance for the route token.
    pub provenance: PathProvenance,
}

/// Raw fixed-grid via token lowered from a DSN/SES route file.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SpecctraGridViaRecord {
    /// Net identifier.
    pub net: NetId,
    /// Inclusive first copper layer touched by this via.
    pub start_layer: TraceLayer,
    /// Inclusive final copper layer touched by this via.
    pub end_layer: TraceLayer,
    /// Center X coordinate in source grid units.
    pub x: i64,
    /// Center Y coordinate in source grid units.
    pub y: i64,
    /// Copper land diameter in source grid units.
    pub land_diameter: i64,
    /// Drill diameter in source grid units.
    pub drill_diameter: i64,
    /// Retained drill plating intent from the route interchange boundary.
    pub drill_intent: ViaDrillIntent,
    /// Denominator of one source unit.
    pub grid_denominator: u64,
}

/// Raw fixed-grid keepout shape lowered from a DSN/SES route file.
///
/// Keepouts are retained route-search constraints, not board/copper booleans.
/// Rectangles and circles are enough to capture common autorouter blockages
/// such as no-route channels, drills, vias, and machine exclusion discs while
/// preserving the exact fixed-grid source values.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SpecctraGridKeepoutShape {
    /// Axis-aligned rectangular keepout in source grid units.
    Rect {
        /// Minimum X coordinate in source grid units.
        min_x: i64,
        /// Minimum Y coordinate in source grid units.
        min_y: i64,
        /// Maximum X coordinate in source grid units.
        max_x: i64,
        /// Maximum Y coordinate in source grid units.
        max_y: i64,
    },
    /// Circular/disc keepout in source grid units.
    Circle {
        /// Center X coordinate in source grid units.
        x: i64,
        /// Center Y coordinate in source grid units.
        y: i64,
        /// Radius in source grid units.
        radius: i64,
    },
}

/// Raw fixed-grid keepout token lowered from a DSN/SES route file.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SpecctraGridKeepoutRecord {
    /// Optional copper layer identifier for layer-scoped route blockages.
    pub layer: Option<TraceLayer>,
    /// Retained fixed-grid shape.
    pub shape: SpecctraGridKeepoutShape,
    /// Denominator of one source unit.
    pub grid_denominator: u64,
}

/// Exact keepout shape retained from a Specctra DSN/SES-style route exchange.
#[derive(Clone, Debug, PartialEq)]
pub struct SpecctraKeepoutRecord {
    /// Optional copper layer identifier for layer-scoped route blockages.
    pub layer: Option<TraceLayer>,
    /// Exact keepout used by route placement predicates.
    pub keepout: MeanderKeepout,
    /// Source provenance for the keepout token.
    pub provenance: PathProvenance,
}

/// Exact route made of validated straight trace records.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SpecctraRoute {
    traces: Vec<PcbTrace>,
    vias: Vec<PcbViaStack>,
}

impl SpecctraRoute {
    /// Construct a route from already validated traces.
    pub fn new(traces: Vec<PcbTrace>) -> Self {
        Self {
            traces,
            vias: Vec::new(),
        }
    }

    /// Construct a route from already validated traces and vias.
    pub fn with_vias(traces: Vec<PcbTrace>, vias: Vec<PcbViaStack>) -> Self {
        Self { traces, vias }
    }

    /// Return the validated trace list.
    pub fn traces(&self) -> &[PcbTrace] {
        &self.traces
    }

    /// Return the validated via list.
    pub fn vias(&self) -> &[PcbViaStack] {
        &self.vias
    }

    /// Import exact trace records, stopping at the first invalid record.
    pub fn from_records(records: &[SpecctraTraceRecord]) -> Result<Self, SpecctraImportError> {
        records
            .iter()
            .map(import_specctra_trace_record)
            .collect::<Result<Vec<_>, _>>()
            .map(Self::new)
    }

    /// Import exact trace and via records, stopping at the first invalid record.
    pub fn from_trace_and_via_records(
        traces: &[SpecctraTraceRecord],
        vias: &[SpecctraViaRecord],
    ) -> Result<Self, SpecctraImportError> {
        let traces = traces
            .iter()
            .map(import_specctra_trace_record)
            .collect::<Result<Vec<_>, _>>()?;
        let vias = vias
            .iter()
            .map(import_specctra_via_record)
            .collect::<Result<Vec<_>, _>>()?;
        Ok(Self::with_vias(traces, vias))
    }
}

/// Errors while lowering external route records into exact trace geometry.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SpecctraImportError {
    /// Source grid denominator was zero or otherwise invalid.
    InvalidGrid,
    /// Trace width was exactly negative.
    NegativeWidth,
    /// Via land or drill diameter was exactly negative.
    NegativeDiameter,
    /// Circular keepout radius was exactly negative.
    NegativeRadius,
    /// Rectangular keepout bounds were not exactly ordered.
    InvalidKeepoutBounds,
    /// Via start layer was above its end layer.
    ReversedLayerSpan,
}

/// Errors while parsing the minimal DSN/SES-style route text form.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SpecctraParseError {
    /// Parentheses, field names, or route shape did not match the supported form.
    InvalidSyntax,
    /// Integer token could not be parsed or did not fit the target field.
    InvalidInteger,
    /// Source grid denominator was zero or otherwise invalid.
    InvalidGrid,
    /// Trace width was exactly negative after exact fixed-grid lowering.
    NegativeWidth,
    /// Via land or drill diameter was exactly negative after exact fixed-grid lowering.
    NegativeDiameter,
    /// Circular keepout radius was exactly negative after exact fixed-grid lowering.
    NegativeRadius,
    /// Rectangular keepout bounds were not exactly ordered after exact lowering.
    InvalidKeepoutBounds,
    /// Via start layer was above its end layer.
    ReversedLayerSpan,
    /// Route-level net alias was empty, malformed, or duplicated.
    InvalidNetAlias,
    /// Route-level layer alias was empty, malformed, or duplicated.
    InvalidLayerAlias,
    /// Via drill intent was not one of the canonical supported atoms.
    InvalidDrillIntent,
}

impl From<SpecctraImportError> for SpecctraParseError {
    fn from(error: SpecctraImportError) -> Self {
        match error {
            SpecctraImportError::InvalidGrid => Self::InvalidGrid,
            SpecctraImportError::NegativeWidth => Self::NegativeWidth,
            SpecctraImportError::NegativeDiameter => Self::NegativeDiameter,
            SpecctraImportError::NegativeRadius => Self::NegativeRadius,
            SpecctraImportError::InvalidKeepoutBounds => Self::InvalidKeepoutBounds,
            SpecctraImportError::ReversedLayerSpan => Self::ReversedLayerSpan,
        }
    }
}

/// Convert a fixed-grid DSN/SES token into an exact trace record.
///
/// The conversion uses `Rational::fraction` directly so source-grid values are
/// not rounded through floats. This keeps the import boundary compatible with
/// exact predicates and with future DSN/SES parsers.
pub fn specctra_grid_trace_record(
    record: SpecctraGridTraceRecord,
) -> Result<SpecctraTraceRecord, SpecctraImportError> {
    let provenance =
        PathProvenance::fixed_grid(PathSourceFormat::Specctra, record.grid_denominator)
            .ok_or(SpecctraImportError::InvalidGrid)?;
    Ok(SpecctraTraceRecord {
        net: record.net,
        layer: record.layer,
        start: Point2::new(
            grid_real(record.start_x, record.grid_denominator)?,
            grid_real(record.start_y, record.grid_denominator)?,
        ),
        end: Point2::new(
            grid_real(record.end_x, record.grid_denominator)?,
            grid_real(record.end_y, record.grid_denominator)?,
        ),
        width: grid_real(record.width, record.grid_denominator)?,
        provenance,
    })
}

/// Convert a fixed-grid DSN/SES via token into an exact via record.
///
/// This is the via analogue of [`specctra_grid_trace_record`]. It preserves
/// land and drill dimensions as exact rationals and keeps drill plating intent
/// as a discrete fabrication fact. The split follows Yap's exact
/// object/predicate model: route import preserves source intent, while
/// annular-ring, drill, layer-span, and clearance predicates certify the
/// resulting exact objects.
pub fn specctra_grid_via_record(
    record: SpecctraGridViaRecord,
) -> Result<SpecctraViaRecord, SpecctraImportError> {
    let provenance =
        PathProvenance::fixed_grid(PathSourceFormat::Specctra, record.grid_denominator)
            .ok_or(SpecctraImportError::InvalidGrid)?;
    Ok(SpecctraViaRecord {
        net: record.net,
        start_layer: record.start_layer,
        end_layer: record.end_layer,
        center: Point2::new(
            grid_real(record.x, record.grid_denominator)?,
            grid_real(record.y, record.grid_denominator)?,
        ),
        land_diameter: grid_real(record.land_diameter, record.grid_denominator)?,
        drill_diameter: grid_real(record.drill_diameter, record.grid_denominator)?,
        drill_intent: record.drill_intent,
        provenance,
    })
}

/// Convert a fixed-grid DSN/SES keepout token into an exact keepout record.
///
/// This is a retained autorouter constraint boundary. The exact keepout can
/// feed route-placement predicates such as circular meander keepouts, but it
/// does not clip board outlines or materialize copper/stock topology. That
/// preserves Yap's exact object/predicate split for DSN/SES import.
pub fn specctra_grid_keepout_record(
    record: SpecctraGridKeepoutRecord,
) -> Result<SpecctraKeepoutRecord, SpecctraImportError> {
    let provenance =
        PathProvenance::fixed_grid(PathSourceFormat::Specctra, record.grid_denominator)
            .ok_or(SpecctraImportError::InvalidGrid)?;
    let keepout = match record.shape {
        SpecctraGridKeepoutShape::Rect {
            min_x,
            min_y,
            max_x,
            max_y,
        } => {
            if min_x > max_x || min_y > max_y {
                return Err(SpecctraImportError::InvalidKeepoutBounds);
            }
            MeanderKeepout::Rectangular(MeanderObstacle {
                min: Point2::new(
                    grid_real(min_x, record.grid_denominator)?,
                    grid_real(min_y, record.grid_denominator)?,
                ),
                max: Point2::new(
                    grid_real(max_x, record.grid_denominator)?,
                    grid_real(max_y, record.grid_denominator)?,
                ),
            })
        }
        SpecctraGridKeepoutShape::Circle { x, y, radius } => {
            if radius < 0 {
                return Err(SpecctraImportError::NegativeRadius);
            }
            MeanderKeepout::Circular {
                center: Point2::new(
                    grid_real(x, record.grid_denominator)?,
                    grid_real(y, record.grid_denominator)?,
                ),
                radius: grid_real(radius, record.grid_denominator)?,
            }
        }
    };
    Ok(SpecctraKeepoutRecord {
        layer: record.layer,
        keepout,
        provenance,
    })
}

/// Lower an exact Specctra route record into a validated PCB trace.
pub fn import_specctra_trace_record(
    record: &SpecctraTraceRecord,
) -> Result<PcbTrace, SpecctraImportError> {
    let centerline = LinePathSegment::with_provenance(
        record.start.clone(),
        record.end.clone(),
        record.provenance,
    );
    let swept = SweptLineSegment::new(centerline, record.width.clone())
        .map_err(|_| SpecctraImportError::NegativeWidth)?;
    Ok(PcbTrace::new(record.net, record.layer, swept))
}

/// Lower an exact Specctra via record into a validated PCB via stack.
pub fn import_specctra_via_record(
    record: &SpecctraViaRecord,
) -> Result<PcbViaStack, SpecctraImportError> {
    PcbViaStack::with_drill_intent(
        record.net,
        record.start_layer,
        record.end_layer,
        record.center.clone(),
        record.land_diameter.clone(),
        record.drill_diameter.clone(),
        record.drill_intent,
    )
    .map_err(|message| match message {
        "via start layer must not be above end layer" => SpecctraImportError::ReversedLayerSpan,
        "pad diameter must be nonnegative" | "via drill diameter must be nonnegative" => {
            SpecctraImportError::NegativeDiameter
        }
        _ => SpecctraImportError::NegativeDiameter,
    })
}

/// Export a validated PCB trace into an exact Specctra-style route record.
pub fn export_specctra_trace_record(trace: &PcbTrace) -> SpecctraTraceRecord {
    SpecctraTraceRecord {
        net: trace.net(),
        layer: trace.layer(),
        start: trace.swept().centerline().start().clone(),
        end: trace.swept().centerline().end().clone(),
        width: trace.swept().width().clone(),
        provenance: trace.provenance(),
    }
}

/// Export a validated PCB via into an exact Specctra-style via record.
pub fn export_specctra_via_record(via: &PcbViaStack) -> Option<SpecctraViaRecord> {
    Some(SpecctraViaRecord {
        net: via.net(),
        start_layer: via.start_layer(),
        end_layer: via.end_layer(),
        center: via.center().clone(),
        land_diameter: via.land_diameter().clone(),
        drill_diameter: via.drill_diameter()?.clone(),
        drill_intent: via.drill_intent(),
        provenance: PathProvenance::native(),
    })
}

/// Lower an exact Specctra keepout record into a retained route keepout.
pub fn import_specctra_keepout_record(record: &SpecctraKeepoutRecord) -> MeanderKeepout {
    record.keepout.clone()
}

/// Serialize fixed-grid route records into a small DSN/SES-style S-expression.
///
/// This is intentionally a canonical subset rather than a complete Specctra
/// writer. It preserves integer grid tokens exactly, providing a stable
/// fixture/export boundary for Lee/Hightower-style autorouter candidates before
/// they are lowered into exact `PcbTrace` geometry.
pub fn serialize_specctra_grid_trace_records(records: &[SpecctraGridTraceRecord]) -> String {
    let mut output = String::from("(routes");
    for record in records {
        write_wire_record(&mut output, record);
    }
    output.push(')');
    output
}

/// Serialize fixed-grid via records into the canonical route subset.
pub fn serialize_specctra_grid_via_records(records: &[SpecctraGridViaRecord]) -> String {
    let mut output = String::from("(routes");
    for record in records {
        write_via_record(&mut output, record);
    }
    output.push(')');
    output
}

/// Serialize fixed-grid keepout records into the canonical route subset.
pub fn serialize_specctra_grid_keepout_records(records: &[SpecctraGridKeepoutRecord]) -> String {
    let mut output = String::from("(routes");
    for record in records {
        write_keepout_record(&mut output, record);
    }
    output.push(')');
    output
}

/// Parsed canonical fixed-grid Specctra route tokens.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct SpecctraGridRouteRecords {
    /// Route-level net aliases retained for diagnostics and round-trips.
    pub net_aliases: Vec<SpecctraNetAlias>,
    /// Route-level layer aliases retained for diagnostics and round-trips.
    pub layer_aliases: Vec<SpecctraLayerAlias>,
    /// Fixed-grid wire records.
    pub traces: Vec<SpecctraGridTraceRecord>,
    /// Fixed-grid via records.
    pub vias: Vec<SpecctraGridViaRecord>,
    /// Fixed-grid retained route keepouts.
    pub keepouts: Vec<SpecctraGridKeepoutRecord>,
}

impl SpecctraGridRouteRecords {
    /// Return whether the document contained no retained geometry or aliases.
    pub fn is_empty(&self) -> bool {
        self.net_aliases.is_empty()
            && self.layer_aliases.is_empty()
            && self.traces.is_empty()
            && self.vias.is_empty()
            && self.keepouts.is_empty()
    }

    fn extend(&mut self, other: Self) -> Result<(), SpecctraParseError> {
        for alias in other.net_aliases {
            self.push_net_alias(alias)?;
        }
        for alias in other.layer_aliases {
            self.push_layer_alias(alias)?;
        }
        self.traces.extend(other.traces);
        self.vias.extend(other.vias);
        self.keepouts.extend(other.keepouts);
        Ok(())
    }

    fn push_net_alias(&mut self, alias: SpecctraNetAlias) -> Result<(), SpecctraParseError> {
        if self
            .net_aliases
            .iter()
            .any(|existing| existing.net == alias.net || existing.name == alias.name)
        {
            return Err(SpecctraParseError::InvalidNetAlias);
        }
        self.net_aliases.push(alias);
        Ok(())
    }

    fn push_layer_alias(&mut self, alias: SpecctraLayerAlias) -> Result<(), SpecctraParseError> {
        if self
            .layer_aliases
            .iter()
            .any(|existing| existing.layer == alias.layer || existing.name == alias.name)
        {
            return Err(SpecctraParseError::InvalidLayerAlias);
        }
        self.layer_aliases.push(alias);
        Ok(())
    }
}

/// Serialize mixed fixed-grid wire/via records into one canonical route form.
///
/// This is still a deliberately small DSN/SES subset, but the mixed serializer
/// gives tests and autorouter fixtures a single canonical text boundary for
/// typed trace and via records before exact import and predicate validation.
pub fn serialize_specctra_grid_route_records(records: &SpecctraGridRouteRecords) -> String {
    let mut output = String::from("(routes");
    for alias in &records.net_aliases {
        write_net_alias(&mut output, alias);
    }
    for alias in &records.layer_aliases {
        write_layer_alias(&mut output, alias);
    }
    for record in &records.traces {
        write_wire_record(&mut output, record);
    }
    for record in &records.vias {
        write_via_record(&mut output, record);
    }
    for record in &records.keepouts {
        write_keepout_record(&mut output, record);
    }
    output.push(')');
    output
}

/// Parse the canonical fixed-grid DSN/SES-style route subset.
///
/// Supported records have the form
/// `(routes (wire (net N) (layer L) (start X Y) (end X Y) (width W) (grid D)))`.
/// DSN/SES route envelopes such as `(session "name" (routes ...))` are also
/// accepted, and `(wire (net N) (path L W X0 Y0 X1 Y1 ... Xn Yn) (grid D))`
/// lowers a retained polyline into one exact trace record per consecutive
/// point pair. This keeps autorouter path output as source-grid integers
/// rather than sampled or rounded path geometry.
pub fn parse_specctra_grid_trace_records(
    input: &str,
) -> Result<Vec<SpecctraGridTraceRecord>, SpecctraParseError> {
    Ok(parse_specctra_grid_route_records(input)?.traces)
}

/// Parse canonical fixed-grid wire and via records.
///
/// Supported route-level net aliases have the form `(net N NAME)`.
/// Supported route-level layer aliases have the form `(layer N NAME)`.
/// Supported via records have the form
/// `(via (net N) (layers A B) (at X Y) (land D) (drill H) (intent plated) (grid G))`.
/// Supported keepout records have the form
/// `(keepout (layer L) (rect X0 Y0 X1 Y1) (grid G))` or
/// `(keepout (circle X Y R) (grid G))`; the layer is optional.
/// This remains intentionally narrower than full DSN/SES, but it gives
/// autorouter fixtures an exact typed boundary for layer transitions and drill
/// fabrication predicates, plus route-search keepouts, instead of leaving
/// geometry constraints as unvalidated text.
pub fn parse_specctra_grid_route_records(
    input: &str,
) -> Result<SpecctraGridRouteRecords, SpecctraParseError> {
    let tokens = tokenize(input)?;
    let mut parser = Parser::new(&tokens);
    let records = parser.parse_document()?;
    if parser.peek().is_some() {
        return Err(SpecctraParseError::InvalidSyntax);
    }
    Ok(records)
}

/// Parse and lower the canonical fixed-grid route subset into validated traces.
pub fn import_specctra_text_route(input: &str) -> Result<SpecctraRoute, SpecctraParseError> {
    let records = parse_specctra_grid_route_records(input)?;
    let exact_traces = records
        .traces
        .into_iter()
        .map(specctra_grid_trace_record)
        .collect::<Result<Vec<_>, _>>()?;
    let exact_vias = records
        .vias
        .into_iter()
        .map(specctra_grid_via_record)
        .collect::<Result<Vec<_>, _>>()?;
    SpecctraRoute::from_trace_and_via_records(&exact_traces, &exact_vias).map_err(Into::into)
}

fn grid_real(value: i64, denominator: u64) -> Result<Real, SpecctraImportError> {
    Rational::fraction(value, denominator)
        .map(Real::new)
        .map_err(|_| SpecctraImportError::InvalidGrid)
}

fn write_net_alias(output: &mut String, alias: &SpecctraNetAlias) {
    write!(output, " (net {} ", alias.net.0).expect("writing to a String cannot fail");
    write_atom(output, &alias.name);
    output.push(')');
}

fn write_layer_alias(output: &mut String, alias: &SpecctraLayerAlias) {
    write!(output, " (layer {} ", alias.layer.0).expect("writing to a String cannot fail");
    write_atom(output, &alias.name);
    output.push(')');
}

fn write_wire_record(output: &mut String, record: &SpecctraGridTraceRecord) {
    write!(
        output,
        " (wire (net {}) (layer {}) (start {} {}) (end {} {}) (width {}) (grid {}))",
        record.net.0,
        record.layer.0,
        record.start_x,
        record.start_y,
        record.end_x,
        record.end_y,
        record.width,
        record.grid_denominator
    )
    .expect("writing to a String cannot fail");
}

fn write_via_record(output: &mut String, record: &SpecctraGridViaRecord) {
    write!(
        output,
        " (via (net {}) (layers {} {}) (at {} {}) (land {}) (drill {}) (intent {}) (grid {}))",
        record.net.0,
        record.start_layer.0,
        record.end_layer.0,
        record.x,
        record.y,
        record.land_diameter,
        record.drill_diameter,
        drill_intent_atom(record.drill_intent),
        record.grid_denominator
    )
    .expect("writing to a String cannot fail");
}

fn write_keepout_record(output: &mut String, record: &SpecctraGridKeepoutRecord) {
    output.push_str(" (keepout");
    if let Some(layer) = record.layer {
        write!(output, " (layer {})", layer.0).expect("writing to a String cannot fail");
    }
    match record.shape {
        SpecctraGridKeepoutShape::Rect {
            min_x,
            min_y,
            max_x,
            max_y,
        } => write!(output, " (rect {min_x} {min_y} {max_x} {max_y})")
            .expect("writing to a String cannot fail"),
        SpecctraGridKeepoutShape::Circle { x, y, radius } => {
            write!(output, " (circle {x} {y} {radius})").expect("writing to a String cannot fail");
        }
    }
    write!(output, " (grid {}))", record.grid_denominator)
        .expect("writing to a String cannot fail");
}

fn drill_intent_atom(intent: ViaDrillIntent) -> &'static str {
    match intent {
        ViaDrillIntent::Unspecified => "unspecified",
        ViaDrillIntent::Plated => "plated",
        ViaDrillIntent::NonPlated => "nonplated",
    }
}

struct Parser<'a> {
    tokens: &'a [String],
    index: usize,
}

impl<'a> Parser<'a> {
    fn new(tokens: &'a [String]) -> Self {
        Self { tokens, index: 0 }
    }

    fn parse_document(&mut self) -> Result<SpecctraGridRouteRecords, SpecctraParseError> {
        self.expect("(")?;
        let root = self.next()?;
        let records = if root == "routes" {
            self.parse_routes_body()?
        } else {
            self.parse_envelope_body()?
        };
        self.expect(")")?;
        if records.is_empty() {
            return Err(SpecctraParseError::InvalidSyntax);
        }
        Ok(records)
    }

    fn parse_routes_body(&mut self) -> Result<SpecctraGridRouteRecords, SpecctraParseError> {
        let mut records = SpecctraGridRouteRecords::default();
        while self.peek() == Some("(") {
            match self.peek_record_kind()? {
                "net" => {
                    let alias = self.parse_net_alias()?;
                    records.push_net_alias(alias)?;
                }
                "layer" => {
                    let alias = self.parse_layer_alias()?;
                    records.push_layer_alias(alias)?;
                }
                "wire" => records.traces.extend(self.parse_wire()?),
                "via" => records.vias.push(self.parse_via()?),
                "keepout" => records.keepouts.push(self.parse_keepout()?),
                _ => return Err(SpecctraParseError::InvalidSyntax),
            }
        }
        Ok(records)
    }

    fn parse_envelope_body(&mut self) -> Result<SpecctraGridRouteRecords, SpecctraParseError> {
        let mut records = SpecctraGridRouteRecords::default();
        while self.peek().is_some() && self.peek() != Some(")") {
            if self.peek() == Some("(") {
                if self.peek_record_kind()? == "routes" {
                    self.expect("(")?;
                    self.expect("routes")?;
                    let nested = self.parse_routes_body()?;
                    self.expect(")")?;
                    records.extend(nested)?;
                } else {
                    let nested = self.parse_unknown_group_for_routes()?;
                    records.extend(nested)?;
                }
            } else {
                self.next()?;
            }
        }
        Ok(records)
    }

    fn parse_unknown_group_for_routes(
        &mut self,
    ) -> Result<SpecctraGridRouteRecords, SpecctraParseError> {
        self.expect("(")?;
        self.next()?;
        let mut records = SpecctraGridRouteRecords::default();
        while self.peek().is_some() && self.peek() != Some(")") {
            if self.peek() == Some("(") {
                if self.peek_record_kind()? == "routes" {
                    self.expect("(")?;
                    self.expect("routes")?;
                    let nested = self.parse_routes_body()?;
                    self.expect(")")?;
                    records.extend(nested)?;
                } else {
                    let nested = self.parse_unknown_group_for_routes()?;
                    records.extend(nested)?;
                }
            } else {
                self.next()?;
            }
        }
        self.expect(")")?;
        Ok(records)
    }

    fn peek(&self) -> Option<&str> {
        self.tokens.get(self.index).map(String::as_str)
    }

    fn next(&mut self) -> Result<&'a str, SpecctraParseError> {
        let token = self
            .tokens
            .get(self.index)
            .ok_or(SpecctraParseError::InvalidSyntax)?;
        self.index += 1;
        Ok(token)
    }

    fn expect(&mut self, expected: &str) -> Result<(), SpecctraParseError> {
        if self.next()? == expected {
            Ok(())
        } else {
            Err(SpecctraParseError::InvalidSyntax)
        }
    }

    fn peek_record_kind(&self) -> Result<&str, SpecctraParseError> {
        if self.peek() != Some("(") {
            return Err(SpecctraParseError::InvalidSyntax);
        }
        self.tokens
            .get(self.index + 1)
            .map(String::as_str)
            .ok_or(SpecctraParseError::InvalidSyntax)
    }

    fn parse_net_alias(&mut self) -> Result<SpecctraNetAlias, SpecctraParseError> {
        self.expect("(")?;
        self.expect("net")?;
        let net = self.parse_u32()?;
        let name = self.next()?.to_owned();
        self.expect(")")?;
        if !is_valid_alias_name(&name) {
            return Err(SpecctraParseError::InvalidNetAlias);
        }
        Ok(SpecctraNetAlias {
            net: NetId(net),
            name,
        })
    }

    fn parse_layer_alias(&mut self) -> Result<SpecctraLayerAlias, SpecctraParseError> {
        self.expect("(")?;
        self.expect("layer")?;
        let layer = self.parse_u16()?;
        let name = self.next()?.to_owned();
        self.expect(")")?;
        if !is_valid_alias_name(&name) {
            return Err(SpecctraParseError::InvalidLayerAlias);
        }
        Ok(SpecctraLayerAlias {
            layer: TraceLayer(layer),
            name,
        })
    }

    fn parse_wire(&mut self) -> Result<Vec<SpecctraGridTraceRecord>, SpecctraParseError> {
        self.expect("(")?;
        self.expect("wire")?;
        let mut net = None;
        let mut layer = None;
        let mut start = None;
        let mut end = None;
        let mut width = None;
        let mut grid_denominator = None;
        let mut path = None;
        while self.peek() == Some("(") {
            match self
                .peek_field_name()
                .ok_or(SpecctraParseError::InvalidSyntax)?
            {
                "net" => set_once(&mut net, self.parse_u32_field("net")?)?,
                "layer" => set_once(&mut layer, self.parse_u16_field("layer")?)?,
                "start" => set_once(&mut start, self.parse_i64_pair_field("start")?)?,
                "end" => set_once(&mut end, self.parse_i64_pair_field("end")?)?,
                "width" => set_once(&mut width, self.parse_i64_field("width")?)?,
                "grid" => set_once(&mut grid_denominator, self.parse_u64_field("grid")?)?,
                "path" => set_once(&mut path, self.parse_path_field()?)?,
                _ => return Err(SpecctraParseError::InvalidSyntax),
            }
        }
        self.expect(")")?;
        let net = net.ok_or(SpecctraParseError::InvalidSyntax)?;
        let grid_denominator = grid_denominator.ok_or(SpecctraParseError::InvalidSyntax)?;
        if grid_denominator == 0 {
            return Err(SpecctraParseError::InvalidGrid);
        }
        if let Some(path) = path {
            if layer.is_some() || start.is_some() || end.is_some() || width.is_some() {
                return Err(SpecctraParseError::InvalidSyntax);
            }
            return path.into_records(NetId(net), grid_denominator);
        }
        let (start_x, start_y) = start.ok_or(SpecctraParseError::InvalidSyntax)?;
        let (end_x, end_y) = end.ok_or(SpecctraParseError::InvalidSyntax)?;
        Ok(vec![SpecctraGridTraceRecord {
            net: NetId(net),
            layer: TraceLayer(layer.ok_or(SpecctraParseError::InvalidSyntax)?),
            start_x,
            start_y,
            end_x,
            end_y,
            width: width.ok_or(SpecctraParseError::InvalidSyntax)?,
            grid_denominator,
        }])
    }

    fn parse_via(&mut self) -> Result<SpecctraGridViaRecord, SpecctraParseError> {
        self.expect("(")?;
        self.expect("via")?;
        let net = self.parse_u32_field("net")?;
        let (start_layer, end_layer) = self.parse_u16_pair_field("layers")?;
        let (x, y) = self.parse_i64_pair_field("at")?;
        let land_diameter = self.parse_i64_field("land")?;
        let drill_diameter = self.parse_i64_field("drill")?;
        let drill_intent = if self.peek_field_name() == Some("intent") {
            self.parse_drill_intent_field()?
        } else {
            ViaDrillIntent::Unspecified
        };
        let grid_denominator = self.parse_u64_field("grid")?;
        self.expect(")")?;
        if grid_denominator == 0 {
            return Err(SpecctraParseError::InvalidGrid);
        }
        Ok(SpecctraGridViaRecord {
            net: NetId(net),
            start_layer: TraceLayer(start_layer),
            end_layer: TraceLayer(end_layer),
            x,
            y,
            land_diameter,
            drill_diameter,
            drill_intent,
            grid_denominator,
        })
    }

    fn parse_keepout(&mut self) -> Result<SpecctraGridKeepoutRecord, SpecctraParseError> {
        self.expect("(")?;
        self.expect("keepout")?;
        let mut layer = None;
        let mut shape = None;
        let mut grid_denominator = None;
        while self.peek() == Some("(") {
            match self
                .peek_field_name()
                .ok_or(SpecctraParseError::InvalidSyntax)?
            {
                "layer" => set_once(&mut layer, TraceLayer(self.parse_u16_field("layer")?))?,
                "rect" => set_once(&mut shape, self.parse_keepout_rect_field()?)?,
                "circle" => set_once(&mut shape, self.parse_keepout_circle_field()?)?,
                "grid" => set_once(&mut grid_denominator, self.parse_u64_field("grid")?)?,
                _ => return Err(SpecctraParseError::InvalidSyntax),
            }
        }
        self.expect(")")?;
        let grid_denominator = grid_denominator.ok_or(SpecctraParseError::InvalidSyntax)?;
        if grid_denominator == 0 {
            return Err(SpecctraParseError::InvalidGrid);
        }
        let record = SpecctraGridKeepoutRecord {
            layer,
            shape: shape.ok_or(SpecctraParseError::InvalidSyntax)?,
            grid_denominator,
        };
        specctra_grid_keepout_record(record)?;
        Ok(record)
    }

    fn peek_field_name(&self) -> Option<&str> {
        if self.peek() == Some("(") {
            self.tokens.get(self.index + 1).map(String::as_str)
        } else {
            None
        }
    }

    fn parse_drill_intent_field(&mut self) -> Result<ViaDrillIntent, SpecctraParseError> {
        self.expect("(")?;
        self.expect("intent")?;
        let intent = match self.next()? {
            "unspecified" => ViaDrillIntent::Unspecified,
            "plated" => ViaDrillIntent::Plated,
            "nonplated" => ViaDrillIntent::NonPlated,
            _ => return Err(SpecctraParseError::InvalidDrillIntent),
        };
        self.expect(")")?;
        Ok(intent)
    }

    fn parse_path_field(&mut self) -> Result<SpecctraPathWire, SpecctraParseError> {
        self.expect("(")?;
        self.expect("path")?;
        let layer = self.parse_u16()?;
        let width = self.parse_i64()?;
        let mut points = Vec::new();
        while self.peek().is_some() && self.peek() != Some(")") {
            let x = self.parse_i64()?;
            if self.peek() == Some(")") {
                return Err(SpecctraParseError::InvalidSyntax);
            }
            let y = self.parse_i64()?;
            points.push((x, y));
        }
        self.expect(")")?;
        if points.len() < 2 {
            return Err(SpecctraParseError::InvalidSyntax);
        }
        Ok(SpecctraPathWire {
            layer: TraceLayer(layer),
            width,
            points,
        })
    }

    fn parse_keepout_rect_field(&mut self) -> Result<SpecctraGridKeepoutShape, SpecctraParseError> {
        self.expect("(")?;
        self.expect("rect")?;
        let min_x = self.parse_i64()?;
        let min_y = self.parse_i64()?;
        let max_x = self.parse_i64()?;
        let max_y = self.parse_i64()?;
        self.expect(")")?;
        Ok(SpecctraGridKeepoutShape::Rect {
            min_x,
            min_y,
            max_x,
            max_y,
        })
    }

    fn parse_keepout_circle_field(
        &mut self,
    ) -> Result<SpecctraGridKeepoutShape, SpecctraParseError> {
        self.expect("(")?;
        self.expect("circle")?;
        let x = self.parse_i64()?;
        let y = self.parse_i64()?;
        let radius = self.parse_i64()?;
        self.expect(")")?;
        Ok(SpecctraGridKeepoutShape::Circle { x, y, radius })
    }

    fn parse_i64_field(&mut self, name: &str) -> Result<i64, SpecctraParseError> {
        self.expect("(")?;
        self.expect(name)?;
        let value = self.parse_i64()?;
        self.expect(")")?;
        Ok(value)
    }

    fn parse_u32_field(&mut self, name: &str) -> Result<u32, SpecctraParseError> {
        let value = self.parse_u64_field(name)?;
        u32::try_from(value).map_err(|_| SpecctraParseError::InvalidInteger)
    }

    fn parse_u32(&mut self) -> Result<u32, SpecctraParseError> {
        let value = self.parse_u64()?;
        u32::try_from(value).map_err(|_| SpecctraParseError::InvalidInteger)
    }

    fn parse_u16_field(&mut self, name: &str) -> Result<u16, SpecctraParseError> {
        let value = self.parse_u64_field(name)?;
        u16::try_from(value).map_err(|_| SpecctraParseError::InvalidInteger)
    }

    fn parse_u16(&mut self) -> Result<u16, SpecctraParseError> {
        let value = self.parse_u64()?;
        u16::try_from(value).map_err(|_| SpecctraParseError::InvalidInteger)
    }

    fn parse_u64_field(&mut self, name: &str) -> Result<u64, SpecctraParseError> {
        self.expect("(")?;
        self.expect(name)?;
        let value = self.parse_u64()?;
        self.expect(")")?;
        Ok(value)
    }

    fn parse_i64_pair_field(&mut self, name: &str) -> Result<(i64, i64), SpecctraParseError> {
        self.expect("(")?;
        self.expect(name)?;
        let first = self.parse_i64()?;
        let second = self.parse_i64()?;
        self.expect(")")?;
        Ok((first, second))
    }

    fn parse_u16_pair_field(&mut self, name: &str) -> Result<(u16, u16), SpecctraParseError> {
        self.expect("(")?;
        self.expect(name)?;
        let first = self.parse_u64()?;
        let second = self.parse_u64()?;
        self.expect(")")?;
        Ok((
            u16::try_from(first).map_err(|_| SpecctraParseError::InvalidInteger)?,
            u16::try_from(second).map_err(|_| SpecctraParseError::InvalidInteger)?,
        ))
    }

    fn parse_i64(&mut self) -> Result<i64, SpecctraParseError> {
        self.next()?
            .parse()
            .map_err(|_| SpecctraParseError::InvalidInteger)
    }

    fn parse_u64(&mut self) -> Result<u64, SpecctraParseError> {
        self.next()?
            .parse()
            .map_err(|_| SpecctraParseError::InvalidInteger)
    }
}

struct SpecctraPathWire {
    layer: TraceLayer,
    width: i64,
    points: Vec<(i64, i64)>,
}

impl SpecctraPathWire {
    fn into_records(
        self,
        net: NetId,
        grid_denominator: u64,
    ) -> Result<Vec<SpecctraGridTraceRecord>, SpecctraParseError> {
        Ok(self
            .points
            .windows(2)
            .map(|pair| SpecctraGridTraceRecord {
                net,
                layer: self.layer,
                start_x: pair[0].0,
                start_y: pair[0].1,
                end_x: pair[1].0,
                end_y: pair[1].1,
                width: self.width,
                grid_denominator,
            })
            .collect())
    }
}

fn set_once<T>(slot: &mut Option<T>, value: T) -> Result<(), SpecctraParseError> {
    if slot.replace(value).is_some() {
        return Err(SpecctraParseError::InvalidSyntax);
    }
    Ok(())
}

fn is_valid_alias_name(name: &str) -> bool {
    !name.is_empty()
        && (is_bare_atom(name)
            || !name
                .chars()
                .any(|character| character == '(' || character == ')'))
}
