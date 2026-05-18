//! Exact PCB route interchange records for Specctra DSN/SES-style flows.
//!
//! DSN/SES exchange is an autorouter boundary: Lee and Hightower style search
//! may produce route candidates outside the exact stack, but those candidates
//! should enter `hyperpath` as fixed-grid exact geometry before clearance
//! predicates certify topology. This module intentionally models validated
//! route records rather than a complete S-expression parser; a parser can
//! lower tokens into these exact records without changing predicate code. The
//! boundary follows Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7.1-2 (1997).

use hyperlimit::Point2;
use hyperreal::{Rational, Real};
use std::fmt::Write;

use crate::pcb::{NetId, PcbTrace, PcbViaStack, TraceLayer};
use crate::provenance::{PathProvenance, PathSourceFormat};
use crate::segment::LinePathSegment;
use crate::swept::SweptLineSegment;

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
    /// Denominator of one source unit.
    pub grid_denominator: u64,
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
    /// Via start layer was above its end layer.
    ReversedLayerSpan,
}

impl From<SpecctraImportError> for SpecctraParseError {
    fn from(error: SpecctraImportError) -> Self {
        match error {
            SpecctraImportError::InvalidGrid => Self::InvalidGrid,
            SpecctraImportError::NegativeWidth => Self::NegativeWidth,
            SpecctraImportError::NegativeDiameter => Self::NegativeDiameter,
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
/// land and drill dimensions as exact rationals so annular-ring, drill, layer
/// span, and clearance predicates can validate the imported object without
/// reconstructing intent from rounded coordinates.
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
    PcbViaStack::with_drill(
        record.net,
        record.start_layer,
        record.end_layer,
        record.center.clone(),
        record.land_diameter.clone(),
        record.drill_diameter.clone(),
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
        provenance: PathProvenance::native(),
    })
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

/// Parsed canonical fixed-grid Specctra route tokens.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct SpecctraGridRouteRecords {
    /// Fixed-grid wire records.
    pub traces: Vec<SpecctraGridTraceRecord>,
    /// Fixed-grid via records.
    pub vias: Vec<SpecctraGridViaRecord>,
}

/// Serialize mixed fixed-grid wire/via records into one canonical route form.
///
/// This is still a deliberately small DSN/SES subset, but the mixed serializer
/// gives tests and autorouter fixtures a single canonical text boundary for
/// typed trace and via records before exact import and predicate validation.
pub fn serialize_specctra_grid_route_records(records: &SpecctraGridRouteRecords) -> String {
    let mut output = String::from("(routes");
    for record in &records.traces {
        write_wire_record(&mut output, record);
    }
    for record in &records.vias {
        write_via_record(&mut output, record);
    }
    output.push(')');
    output
}

/// Parse the canonical fixed-grid DSN/SES-style route subset.
///
/// Supported records have the form
/// `(routes (wire (net N) (layer L) (start X Y) (end X Y) (width W) (grid D)))`.
/// A full Specctra parser can lower richer syntax into `SpecctraGridTraceRecord`
/// later; this subset exists so exact route import/export can be tested and
/// benchmarked without adding a lossy or permissive text boundary.
pub fn parse_specctra_grid_trace_records(
    input: &str,
) -> Result<Vec<SpecctraGridTraceRecord>, SpecctraParseError> {
    Ok(parse_specctra_grid_route_records(input)?.traces)
}

/// Parse canonical fixed-grid wire and via records.
///
/// Supported via records have the form
/// `(via (net N) (layers A B) (at X Y) (land D) (drill H) (grid G))`.
/// This remains intentionally narrower than full DSN/SES, but it gives
/// autorouter fixtures an exact typed boundary for layer transitions and drill
/// fabrication predicates instead of leaving vias as unvalidated text.
pub fn parse_specctra_grid_route_records(
    input: &str,
) -> Result<SpecctraGridRouteRecords, SpecctraParseError> {
    let tokens = tokenize(input);
    let mut parser = Parser::new(&tokens);
    parser.expect("(")?;
    parser.expect("routes")?;
    let mut records = SpecctraGridRouteRecords::default();
    while parser.peek() == Some("(") {
        match parser.peek_record_kind()? {
            "wire" => records.traces.push(parser.parse_wire()?),
            "via" => records.vias.push(parser.parse_via()?),
            _ => return Err(SpecctraParseError::InvalidSyntax),
        }
    }
    parser.expect(")")?;
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
        " (via (net {}) (layers {} {}) (at {} {}) (land {}) (drill {}) (grid {}))",
        record.net.0,
        record.start_layer.0,
        record.end_layer.0,
        record.x,
        record.y,
        record.land_diameter,
        record.drill_diameter,
        record.grid_denominator
    )
    .expect("writing to a String cannot fail");
}

fn tokenize(input: &str) -> Vec<String> {
    let mut tokens = Vec::new();
    let mut current = String::new();
    for character in input.chars() {
        match character {
            '(' | ')' => {
                if !current.is_empty() {
                    tokens.push(std::mem::take(&mut current));
                }
                tokens.push(character.to_string());
            }
            c if c.is_whitespace() => {
                if !current.is_empty() {
                    tokens.push(std::mem::take(&mut current));
                }
            }
            c => current.push(c),
        }
    }
    if !current.is_empty() {
        tokens.push(current);
    }
    tokens
}

struct Parser<'a> {
    tokens: &'a [String],
    index: usize,
}

impl<'a> Parser<'a> {
    fn new(tokens: &'a [String]) -> Self {
        Self { tokens, index: 0 }
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

    fn parse_wire(&mut self) -> Result<SpecctraGridTraceRecord, SpecctraParseError> {
        self.expect("(")?;
        self.expect("wire")?;
        let net = self.parse_u32_field("net")?;
        let layer = self.parse_u16_field("layer")?;
        let (start_x, start_y) = self.parse_i64_pair_field("start")?;
        let (end_x, end_y) = self.parse_i64_pair_field("end")?;
        let width = self.parse_i64_field("width")?;
        let grid_denominator = self.parse_u64_field("grid")?;
        self.expect(")")?;
        if grid_denominator == 0 {
            return Err(SpecctraParseError::InvalidGrid);
        }
        Ok(SpecctraGridTraceRecord {
            net: NetId(net),
            layer: TraceLayer(layer),
            start_x,
            start_y,
            end_x,
            end_y,
            width,
            grid_denominator,
        })
    }

    fn parse_via(&mut self) -> Result<SpecctraGridViaRecord, SpecctraParseError> {
        self.expect("(")?;
        self.expect("via")?;
        let net = self.parse_u32_field("net")?;
        let (start_layer, end_layer) = self.parse_u16_pair_field("layers")?;
        let (x, y) = self.parse_i64_pair_field("at")?;
        let land_diameter = self.parse_i64_field("land")?;
        let drill_diameter = self.parse_i64_field("drill")?;
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
            grid_denominator,
        })
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

    fn parse_u16_field(&mut self, name: &str) -> Result<u16, SpecctraParseError> {
        let value = self.parse_u64_field(name)?;
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
