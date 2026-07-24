//! Native KiCad schematic interchange for circuit-owned drawing intent.
//!
//! The KiCad file remains a presentation boundary. Import reconstructs a
//! [`SchematicLayout`] against a caller-supplied authoritative [`Circuit`];
//! it never infers or replaces circuit topology from drawing geometry.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::{Display, Formatter, Write};
use std::path::Path;
use std::str::FromStr;

use hyperreal::{Real, RealSign};

use crate::sexp::{self, Sexp};
use crate::{
    Circuit, CircuitInstanceId, NetId, PinElectricalKind, PinRef, PortDirection, PortId,
    SchematicEndpoint, SchematicGraphic, SchematicGraphicFill, SchematicLabel, SchematicLabelId,
    SchematicLayout, SchematicPinPlacement, SchematicPinSide, SchematicPoint,
    SchematicPortPlacement, SchematicSheet, SchematicSheetId, SchematicSheetLink,
    SchematicSheetLinkId, SchematicSheetPort, SchematicSheetPortId, SchematicSymbol,
    SchematicSymbolDefinition, SchematicSymbolDefinitionId, SchematicSymbolId, SchematicSymbolUnit,
    SchematicWire, SchematicWireId,
};

const KICAD_SCHEMATIC_VERSION: &str = "20250114";
const SYMBOL_ID_PROPERTY: &str = "HyperCircuit Symbol";
const INSTANCE_ID_PROPERTY: &str = "HyperCircuit Instance";
const DEFINITION_ID_PROPERTY: &str = "HyperCircuit Definition";
const DEFINITION_MODEL_PROPERTY: &str = "HyperCircuit Definition Model";
const DEFINITION_NAME_PROPERTY: &str = "HyperCircuit Definition Name";
const SHEET_ID_COMMENT_PREFIX: &str = "hypercircuit-sheet:";
const CHILD_SHEET_ID_PROPERTY: &str = "HyperCircuit Sheet";
const SHEET_LINK_PROPERTY_PREFIX: &str = "HyperCircuit Link ";

/// Finite-decimal and project-name policy for KiCad schematic export.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct KiCadSchematicExportOptions {
    /// Digits written after the decimal point for exact schematic coordinates.
    pub decimal_places: usize,
    /// KiCad project identity used in symbol instance paths.
    pub project_name: String,
}

impl Default for KiCadSchematicExportOptions {
    fn default() -> Self {
        Self {
            decimal_places: 6,
            project_name: "hypercircuit".into(),
        }
    }
}

/// One exact scalar projected to KiCad's finite-decimal syntax.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct KiCadSchematicNumericProjection {
    /// Semantic field being projected.
    pub field: String,
    /// Exact source expression.
    pub source: String,
    /// Decimal token written to KiCad.
    pub emitted: String,
}

/// Stable relation between one circuit-owned symbol and its KiCad identities.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct KiCadSchematicSymbolProjection {
    /// Circuit-owned drawing identity retained as a hidden symbol property.
    pub symbol: SchematicSymbolId,
    /// Embedded KiCad library identifier.
    pub library_id: String,
    /// Deterministic native KiCad symbol UUID.
    pub uuid: String,
}

/// One circuit-owned polyline lowered to one or more native two-point wires.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct KiCadSchematicWireProjection {
    /// Circuit-owned wire identity.
    pub wire: SchematicWireId,
    /// Ordered native KiCad segment UUIDs.
    pub native_uuids: Vec<String>,
}

/// Explicit loss at the schematic interchange boundary.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum KiCadSchematicExportOmission {
    /// KiCad local labels cannot separate displayed text from electrical net name.
    LabelTextProjected {
        /// Circuit-owned label identity.
        label: SchematicLabelId,
        /// Authored display text retained by hypercircuit.
        authored: String,
        /// Native KiCad electrical label text.
        emitted: String,
    },
    /// KiCad requires one shared sheet-pin/hierarchical-label name.
    SheetPortNameProjected {
        /// Parent-page boundary port.
        parent_port: SchematicSheetPortId,
        /// Child-page boundary port.
        child_port: SchematicSheetPortId,
        /// Parent-authored display name retained only in metadata.
        parent_name: String,
        /// Native shared name taken from the child port.
        emitted: String,
    },
}

/// Native `.kicad_sch` text plus replayable projection evidence.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct KiCadSchematicExportReport {
    /// Standalone KiCad 9 schematic file content.
    pub schematic: String,
    /// Every exact-to-decimal boundary crossing.
    pub numeric_projections: Vec<KiCadSchematicNumericProjection>,
    /// Stable circuit-symbol to native-symbol mapping.
    pub symbols: Vec<KiCadSchematicSymbolProjection>,
    /// Stable circuit-wire to native-segment mapping.
    pub wires: Vec<KiCadSchematicWireProjection>,
    /// Number of native connectivity labels generated for junction-only wires.
    pub generated_connectivity_labels: usize,
    /// Retained presentation facts not represented exactly by KiCad.
    pub omissions: Vec<KiCadSchematicExportOmission>,
}

/// Filename and finite projection policy for a native multi-file schematic.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct KiCadSchematicBookExportOptions {
    /// Scalar and project settings shared by every file.
    pub schematic: KiCadSchematicExportOptions,
    /// Basename of the root schematic file.
    pub root_filename: String,
}

impl Default for KiCadSchematicBookExportOptions {
    fn default() -> Self {
        Self {
            schematic: KiCadSchematicExportOptions::default(),
            root_filename: "root.kicad_sch".into(),
        }
    }
}

/// Stable file/path identity for one explicit schematic page.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct KiCadSchematicSheetProjection {
    /// Circuit-owned page identity.
    pub sheet: SchematicSheetId,
    /// Native file basename.
    pub filename: String,
    /// File-header UUID.
    pub file_uuid: String,
    /// Native instance path used by symbols and nested sheets.
    pub instance_path: String,
    /// Deterministic one-based page number.
    pub page: usize,
}

/// One generated page in a native KiCad hierarchy package.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct KiCadSchematicBookFile {
    /// Page/file/path identity.
    pub projection: KiCadSchematicSheetProjection,
    /// Native `.kicad_sch` contents.
    pub schematic: String,
    /// Exact-to-decimal boundaries in this file.
    pub numeric_projections: Vec<KiCadSchematicNumericProjection>,
    /// Symbol mappings emitted in this file.
    pub symbols: Vec<KiCadSchematicSymbolProjection>,
    /// Wire-to-native-segment mappings emitted in this file.
    pub wires: Vec<KiCadSchematicWireProjection>,
    /// Connectivity-only labels generated in this file.
    pub generated_connectivity_labels: usize,
    /// File-local representational losses.
    pub omissions: Vec<KiCadSchematicExportOmission>,
}

/// Complete native KiCad hierarchy package.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct KiCadSchematicBookExportReport {
    /// Basename KiCad should open as the hierarchy root.
    pub root_filename: String,
    /// Root-first files; every child filename is unique.
    pub files: Vec<KiCadSchematicBookFile>,
}

/// Failure before a structurally meaningful KiCad schematic can be emitted.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum KiCadSchematicExportError {
    /// Circuit or schematic validation failed.
    InvalidDesign,
    /// Native hierarchical files require a multi-file export package.
    HierarchyRequiresMultiFileExport,
    /// A book export was requested for an implicit flat schematic.
    BookRequiresExplicitHierarchy,
    /// A package filename was unsafe or lacked the native extension.
    InvalidBookFilename(String),
    /// A retained boundary port does not participate in a parent/child link.
    UnlinkedSheetPort(SchematicSheetPortId),
    /// Validated hierarchy could not be lowered to one internally consistent package.
    InvalidBookState(String),
    /// A validated wire endpoint could not be resolved to a drawing point.
    UnresolvedWireEndpoint {
        /// Circuit-owned wire identity.
        wire: SchematicWireId,
        /// Endpoint position in the wire record.
        endpoint: &'static str,
    },
    /// Native simplification collapsed a wire to fewer than two distinct points.
    DegenerateWire(SchematicWireId),
    /// Export options are empty or outside the supported finite projection bound.
    InvalidOptions,
    /// An exact scalar had no finite projection.
    NonFiniteScalar(String),
    /// Internal text formatting failed.
    Formatting,
}

impl Display for KiCadSchematicExportError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidDesign => formatter.write_str("cannot export an invalid schematic"),
            Self::HierarchyRequiresMultiFileExport => {
                formatter.write_str("KiCad hierarchy requires a multi-file schematic export")
            }
            Self::BookRequiresExplicitHierarchy => {
                formatter.write_str("KiCad book export requires explicit schematic sheets")
            }
            Self::InvalidBookFilename(filename) => {
                write!(
                    formatter,
                    "invalid KiCad schematic book filename {filename:?}"
                )
            }
            Self::UnlinkedSheetPort(port) => {
                write!(
                    formatter,
                    "schematic sheet port {} is not linked",
                    port.as_str()
                )
            }
            Self::InvalidBookState(message) => {
                write!(formatter, "invalid KiCad schematic book state: {message}")
            }
            Self::UnresolvedWireEndpoint { wire, endpoint } => write!(
                formatter,
                "cannot resolve {endpoint} endpoint of schematic wire {}",
                wire.as_str()
            ),
            Self::DegenerateWire(wire) => write!(
                formatter,
                "schematic wire {} collapses to fewer than two points",
                wire.as_str()
            ),
            Self::InvalidOptions => formatter.write_str("invalid KiCad schematic export options"),
            Self::NonFiniteScalar(field) => {
                write!(formatter, "KiCad schematic scalar is non-finite: {field}")
            }
            Self::Formatting => formatter.write_str("failed to format KiCad schematic"),
        }
    }
}

impl std::error::Error for KiCadSchematicExportError {}

impl SchematicLayout {
    /// Exports this flat circuit-bound drawing as a native KiCad 9 schematic.
    pub fn export_kicad_schematic(
        &self,
        circuit: &Circuit,
        options: KiCadSchematicExportOptions,
    ) -> Result<KiCadSchematicExportReport, KiCadSchematicExportError> {
        if !circuit.validate().is_valid() || !self.validate(circuit).is_valid() {
            return Err(KiCadSchematicExportError::InvalidDesign);
        }
        if !self.sheets.is_empty() || !self.sheet_ports.is_empty() || !self.sheet_links.is_empty() {
            return Err(KiCadSchematicExportError::HierarchyRequiresMultiFileExport);
        }
        if options.project_name.trim().is_empty() || options.decimal_places > 12 {
            return Err(KiCadSchematicExportError::InvalidOptions);
        }
        let mut emitter = SchematicEmitter::new(options);
        emitter.emit(circuit, self)?;
        Ok(KiCadSchematicExportReport {
            schematic: emitter.output,
            numeric_projections: emitter.numeric_projections,
            symbols: emitter.symbols,
            wires: emitter.wires,
            generated_connectivity_labels: emitter.generated_connectivity_labels,
            omissions: emitter.omissions,
        })
    }

    /// Exports explicit sheet hierarchy as one native KiCad file per page.
    pub fn export_kicad_schematic_book(
        &self,
        circuit: &Circuit,
        options: KiCadSchematicBookExportOptions,
    ) -> Result<KiCadSchematicBookExportReport, KiCadSchematicExportError> {
        if !circuit.validate().is_valid() || !self.validate(circuit).is_valid() {
            return Err(KiCadSchematicExportError::InvalidDesign);
        }
        if self.sheets.is_empty() {
            return Err(KiCadSchematicExportError::BookRequiresExplicitHierarchy);
        }
        if options.schematic.project_name.trim().is_empty() || options.schematic.decimal_places > 12
        {
            return Err(KiCadSchematicExportError::InvalidOptions);
        }
        if !valid_kicad_filename(&options.root_filename) {
            return Err(KiCadSchematicExportError::InvalidBookFilename(
                options.root_filename,
            ));
        }
        for port in &self.sheet_ports {
            if !self
                .sheet_links
                .iter()
                .any(|link| link.parent_port == port.id || link.child_port == port.id)
            {
                return Err(KiCadSchematicExportError::UnlinkedSheetPort(
                    port.id.clone(),
                ));
            }
        }
        BookEmitter::new(self, circuit, options).emit()
    }
}

struct SchematicEmitter {
    options: KiCadSchematicExportOptions,
    output: String,
    numeric_projections: Vec<KiCadSchematicNumericProjection>,
    symbols: Vec<KiCadSchematicSymbolProjection>,
    wires: Vec<KiCadSchematicWireProjection>,
    generated_connectivity_labels: usize,
    omissions: Vec<KiCadSchematicExportOmission>,
}

impl SchematicEmitter {
    fn new(options: KiCadSchematicExportOptions) -> Self {
        Self {
            options,
            output: String::new(),
            numeric_projections: Vec::new(),
            symbols: Vec::new(),
            wires: Vec::new(),
            generated_connectivity_labels: 0,
            omissions: Vec::new(),
        }
    }

    fn emit(
        &mut self,
        circuit: &Circuit,
        layout: &SchematicLayout,
    ) -> Result<(), KiCadSchematicExportError> {
        let root_uuid = stable_uuid(&format!("sheet:{}", circuit.id.as_str()));
        writeln!(self.output, "(kicad_sch").map_err(formatting)?;
        writeln!(self.output, "  (version {KICAD_SCHEMATIC_VERSION})").map_err(formatting)?;
        writeln!(self.output, "  (generator \"hypercircuit\")").map_err(formatting)?;
        writeln!(self.output, "  (generator_version \"0.3\")").map_err(formatting)?;
        writeln!(self.output, "  (uuid \"{root_uuid}\")").map_err(formatting)?;
        writeln!(self.output, "  (paper \"A4\")").map_err(formatting)?;
        writeln!(self.output, "  (lib_symbols").map_err(formatting)?;
        for definition in &layout.symbol_definitions {
            self.emit_library_symbol(circuit, definition)?;
        }
        writeln!(self.output, "  )").map_err(formatting)?;

        for wire in &layout.wires {
            self.emit_wire(layout, wire)?;
        }
        for label in &layout.labels {
            self.emit_label(label)?;
        }
        for port in &layout.ports {
            self.emit_port(circuit, port)?;
        }
        for (index, symbol) in layout.symbols.iter().enumerate() {
            self.emit_symbol_instance(circuit, layout, symbol, index, &format!("/{root_uuid}"))?;
        }
        writeln!(self.output, "  (sheet_instances").map_err(formatting)?;
        writeln!(self.output, "    (path \"/\" (page \"1\"))").map_err(formatting)?;
        writeln!(self.output, "  )").map_err(formatting)?;
        writeln!(self.output, "  (embedded_fonts no)").map_err(formatting)?;
        writeln!(self.output, ")").map_err(formatting)?;
        Ok(())
    }

    fn emit_library_symbol(
        &mut self,
        circuit: &Circuit,
        definition: &SchematicSymbolDefinition,
    ) -> Result<(), KiCadSchematicExportError> {
        let model = circuit
            .device_models
            .iter()
            .find(|candidate| candidate.id == definition.model)
            .ok_or(KiCadSchematicExportError::InvalidDesign)?;
        let library_id = library_id(&definition.id);
        writeln!(self.output, "    (symbol {}", quoted(&library_id)).map_err(formatting)?;
        writeln!(self.output, "      (pin_names (offset 0))").map_err(formatting)?;
        writeln!(self.output, "      (exclude_from_sim no)").map_err(formatting)?;
        writeln!(self.output, "      (in_bom yes)").map_err(formatting)?;
        writeln!(self.output, "      (on_board yes)").map_err(formatting)?;
        self.library_property("Reference", "U", "0", "-2.54", false)?;
        self.library_property("Value", model.id.as_str(), "0", "0", false)?;
        self.library_property("Footprint", "", "0", "0", true)?;
        self.library_property("Datasheet", "", "0", "0", true)?;
        self.library_property("Description", "Generated by HyperCircuit", "0", "0", true)?;
        self.library_property(
            DEFINITION_ID_PROPERTY,
            definition.id.as_str(),
            "0",
            "0",
            true,
        )?;
        self.library_property(
            DEFINITION_MODEL_PROPERTY,
            definition.model.as_str(),
            "0",
            "0",
            true,
        )?;
        self.library_property(DEFINITION_NAME_PROPERTY, &definition.name, "0", "0", true)?;
        for unit in &definition.units {
            let width = self.scalar(
                &format!(
                    "symbol_definitions[{}].units[{}].body_width",
                    definition.id.as_str(),
                    unit.unit
                ),
                &unit.body_width,
            )?;
            let height = self.scalar(
                &format!(
                    "symbol_definitions[{}].units[{}].body_height",
                    definition.id.as_str(),
                    unit.unit
                ),
                &unit.body_height,
            )?;
            self.library_property(
                &unit_body_property(unit.unit, "Width"),
                &width,
                "0",
                "0",
                true,
            )?;
            self.library_property(
                &unit_body_property(unit.unit, "Height"),
                &height,
                "0",
                "0",
                true,
            )?;
            self.library_property(
                &format!("HyperCircuit Unit {} Graphics", unit.unit),
                &unit
                    .graphics
                    .iter()
                    .map(graphic_kind)
                    .collect::<Vec<_>>()
                    .join(","),
                "0",
                "0",
                true,
            )?;
        }
        for unit in &definition.units {
            let unit_name = format!("{}_{}_1", kicad_symbol_name(&definition.id), unit.unit);
            writeln!(self.output, "      (symbol {}", quoted(&unit_name)).map_err(formatting)?;
            for (index, graphic) in unit.graphics.iter().enumerate() {
                self.emit_library_graphic(definition, unit, index, graphic)?;
            }
            for pin in &unit.pins {
                let pin_definition = model
                    .pins
                    .iter()
                    .find(|candidate| candidate.pin == pin.pin)
                    .ok_or(KiCadSchematicExportError::InvalidDesign)?;
                let x = self.scalar(
                    &format!(
                        "symbol_definitions[{}].units[{}].pins[{}].x",
                        definition.id.as_str(),
                        unit.unit,
                        pin.pin.as_str()
                    ),
                    &pin.position.x,
                )?;
                let y = self.scalar(
                    &format!(
                        "symbol_definitions[{}].units[{}].pins[{}].y",
                        definition.id.as_str(),
                        unit.unit,
                        pin.pin.as_str()
                    ),
                    &pin.position.y,
                )?;
                let angle = pin_angle(pin.side);
                let length = pin_lead_length(unit, pin);
                let length = self.scalar(
                    &format!(
                        "symbol_definitions[{}].units[{}].pins[{}].length",
                        definition.id.as_str(),
                        unit.unit,
                        pin.pin.as_str()
                    ),
                    &length,
                )?;
                writeln!(
                    self.output,
                    "        (pin {} line",
                    kicad_pin_kind(pin_definition.kind)
                )
                .map_err(formatting)?;
                writeln!(self.output, "          (at {x} {y} {angle})").map_err(formatting)?;
                writeln!(self.output, "          (length {length})").map_err(formatting)?;
                self.pin_text("name", pin.pin.as_str())?;
                self.pin_text("number", pin.pin.as_str())?;
                writeln!(self.output, "        )").map_err(formatting)?;
            }
            writeln!(self.output, "      )").map_err(formatting)?;
        }
        writeln!(self.output, "      (embedded_fonts no)").map_err(formatting)?;
        writeln!(self.output, "    )").map_err(formatting)?;
        Ok(())
    }

    fn emit_library_graphic(
        &mut self,
        definition: &SchematicSymbolDefinition,
        unit: &SchematicSymbolUnit,
        index: usize,
        graphic: &SchematicGraphic,
    ) -> Result<(), KiCadSchematicExportError> {
        let field = format!(
            "symbol_definitions[{}].units[{}].graphics[{index}]",
            definition.id.as_str(),
            unit.unit
        );
        match graphic {
            SchematicGraphic::Line {
                from,
                to,
                stroke_width,
            } => {
                writeln!(self.output, "        (polyline").map_err(formatting)?;
                self.emit_graphic_points(&field, [from, to])?;
                self.emit_graphic_stroke_fill(&field, stroke_width, SchematicGraphicFill::None)?;
                writeln!(self.output, "        )").map_err(formatting)
            }
            SchematicGraphic::Rectangle {
                start,
                end,
                stroke_width,
                fill,
            } => {
                let (start_x, start_y) = self.point(&format!("{field}.start"), start)?;
                let (end_x, end_y) = self.point(&format!("{field}.end"), end)?;
                writeln!(self.output, "        (rectangle").map_err(formatting)?;
                writeln!(self.output, "          (start {start_x} {start_y})")
                    .map_err(formatting)?;
                writeln!(self.output, "          (end {end_x} {end_y})").map_err(formatting)?;
                self.emit_graphic_stroke_fill(&field, stroke_width, *fill)?;
                writeln!(self.output, "        )").map_err(formatting)
            }
            SchematicGraphic::Circle {
                center,
                radius,
                stroke_width,
                fill,
            } => {
                let (x, y) = self.point(&format!("{field}.center"), center)?;
                let radius = self.scalar(&format!("{field}.radius"), radius)?;
                writeln!(
                    self.output,
                    "        (circle (center {x} {y}) (radius {radius})"
                )
                .map_err(formatting)?;
                self.emit_graphic_stroke_fill(&field, stroke_width, *fill)?;
                writeln!(self.output, "        )").map_err(formatting)
            }
            SchematicGraphic::Arc {
                start,
                mid,
                end,
                stroke_width,
            } => {
                writeln!(self.output, "        (arc").map_err(formatting)?;
                for (name, point) in [("start", start), ("mid", mid), ("end", end)] {
                    let (x, y) = self.point(&format!("{field}.{name}"), point)?;
                    writeln!(self.output, "          ({name} {x} {y})").map_err(formatting)?;
                }
                self.emit_graphic_stroke_fill(&field, stroke_width, SchematicGraphicFill::None)?;
                writeln!(self.output, "        )").map_err(formatting)
            }
            SchematicGraphic::Polyline {
                points,
                closed,
                stroke_width,
                fill,
            } => {
                writeln!(self.output, "        (polyline").map_err(formatting)?;
                let mut emitted = points.iter().collect::<Vec<_>>();
                if *closed {
                    emitted.push(&points[0]);
                }
                self.emit_graphic_points(&field, emitted)?;
                self.emit_graphic_stroke_fill(&field, stroke_width, *fill)?;
                writeln!(self.output, "        )").map_err(formatting)
            }
            SchematicGraphic::Text {
                position,
                text,
                size,
                quarter_turns,
            } => {
                let (x, y) = self.point(&format!("{field}.position"), position)?;
                let size = self.scalar(&format!("{field}.size"), size)?;
                let angle = i32::from(quarter_turns.rem_euclid(4)) * 900;
                writeln!(self.output, "        (text {}", quoted(text)).map_err(formatting)?;
                writeln!(self.output, "          (at {x} {y} {angle})").map_err(formatting)?;
                writeln!(
                    self.output,
                    "          (effects (font (size {size} {size})))"
                )
                .map_err(formatting)?;
                writeln!(self.output, "        )").map_err(formatting)
            }
        }
    }

    fn emit_graphic_points<'a>(
        &mut self,
        field: &str,
        points: impl IntoIterator<Item = &'a SchematicPoint>,
    ) -> Result<(), KiCadSchematicExportError> {
        writeln!(self.output, "          (pts").map_err(formatting)?;
        for (index, point) in points.into_iter().enumerate() {
            let (x, y) = self.point(&format!("{field}.points[{index}]"), point)?;
            writeln!(self.output, "            (xy {x} {y})").map_err(formatting)?;
        }
        writeln!(self.output, "          )").map_err(formatting)
    }

    fn emit_graphic_stroke_fill(
        &mut self,
        field: &str,
        stroke_width: &Real,
        fill: SchematicGraphicFill,
    ) -> Result<(), KiCadSchematicExportError> {
        let width = self.scalar(&format!("{field}.stroke_width"), stroke_width)?;
        writeln!(
            self.output,
            "          (stroke (width {width}) (type default))"
        )
        .map_err(formatting)?;
        writeln!(
            self.output,
            "          (fill (type {}))",
            match fill {
                SchematicGraphicFill::None => "none",
                SchematicGraphicFill::Background => "background",
                SchematicGraphicFill::Foreground => "outline",
            }
        )
        .map_err(formatting)
    }

    fn library_property(
        &mut self,
        name: &str,
        value: &str,
        x: &str,
        y: &str,
        hidden: bool,
    ) -> Result<(), KiCadSchematicExportError> {
        writeln!(
            self.output,
            "      (property {} {}",
            quoted(name),
            quoted(value)
        )
        .map_err(formatting)?;
        writeln!(self.output, "        (at {x} {y} 0)").map_err(formatting)?;
        write!(self.output, "        (effects (font (size 1.27 1.27))").map_err(formatting)?;
        if hidden {
            write!(self.output, " (hide yes)").map_err(formatting)?;
        }
        writeln!(self.output, ")").map_err(formatting)?;
        writeln!(self.output, "      )").map_err(formatting)
    }

    fn pin_text(&mut self, token: &str, value: &str) -> Result<(), KiCadSchematicExportError> {
        writeln!(
            self.output,
            "          ({token} {} (effects (font (size 1.0 1.0))))",
            quoted(value)
        )
        .map_err(formatting)
    }

    fn emit_wire(
        &mut self,
        layout: &SchematicLayout,
        wire: &SchematicWire,
    ) -> Result<(), KiCadSchematicExportError> {
        let mut points = Vec::with_capacity(wire.waypoints.len() + 2);
        points.push(endpoint_point(layout, &wire.from).ok_or_else(|| {
            KiCadSchematicExportError::UnresolvedWireEndpoint {
                wire: wire.id.clone(),
                endpoint: "from",
            }
        })?);
        points.extend(wire.waypoints.iter().cloned());
        points.push(endpoint_point(layout, &wire.to).ok_or_else(|| {
            KiCadSchematicExportError::UnresolvedWireEndpoint {
                wire: wire.id.clone(),
                endpoint: "to",
            }
        })?);
        points = simplify_polyline(points);
        if points.len() < 2 {
            return Err(KiCadSchematicExportError::DegenerateWire(wire.id.clone()));
        }
        let mut native_uuids = Vec::with_capacity(points.len() - 1);
        for (segment, pair) in points.windows(2).enumerate() {
            let wire_uuid = stable_uuid(&format!("wire:{}:{segment}", wire.id.as_str()));
            native_uuids.push(wire_uuid.clone());
            writeln!(self.output, "  (wire").map_err(formatting)?;
            writeln!(self.output, "    (pts").map_err(formatting)?;
            for (endpoint, point) in pair.iter().enumerate() {
                let (x, y) = self.point(
                    &format!(
                        "wires[{}].segments[{segment}].points[{endpoint}]",
                        wire.id.as_str()
                    ),
                    point,
                )?;
                writeln!(self.output, "      (xy {x} {y})").map_err(formatting)?;
            }
            writeln!(self.output, "    )").map_err(formatting)?;
            writeln!(self.output, "    (stroke (width 0) (type solid))").map_err(formatting)?;
            writeln!(self.output, "    (uuid \"{wire_uuid}\")").map_err(formatting)?;
            writeln!(self.output, "  )").map_err(formatting)?;
            let has_from_evidence =
                segment == 0 && !matches!(wire.from, SchematicEndpoint::Junction(_));
            let has_to_evidence =
                segment + 2 == points.len() && !matches!(wire.to, SchematicEndpoint::Junction(_));
            if !has_from_evidence && !has_to_evidence {
                let synthetic_uuid = stable_uuid(&format!("synthetic-net-label:{wire_uuid}"));
                self.emit_native_label(
                    wire.net.as_str(),
                    &pair[0],
                    &synthetic_uuid,
                    &format!(
                        "wires[{}].segments[{segment}].connectivity_label",
                        wire.id.as_str()
                    ),
                )?;
                self.generated_connectivity_labels += 1;
            }
        }
        self.wires.push(KiCadSchematicWireProjection {
            wire: wire.id.clone(),
            native_uuids,
        });
        Ok(())
    }

    fn emit_label(&mut self, label: &SchematicLabel) -> Result<(), KiCadSchematicExportError> {
        if label.text != label.net.as_str() {
            self.omissions
                .push(KiCadSchematicExportOmission::LabelTextProjected {
                    label: label.id.clone(),
                    authored: label.text.clone(),
                    emitted: label.net.as_str().to_owned(),
                });
        }
        self.emit_native_label(
            label.net.as_str(),
            &label.position,
            &stable_uuid(&format!("label:{}", label.id.as_str())),
            &format!("labels[{}]", label.id.as_str()),
        )
    }

    fn emit_native_label(
        &mut self,
        text: &str,
        position: &SchematicPoint,
        uuid: &str,
        field: &str,
    ) -> Result<(), KiCadSchematicExportError> {
        let (x, y) = self.point(field, position)?;
        writeln!(self.output, "  (label {}", quoted(text)).map_err(formatting)?;
        writeln!(self.output, "    (at {x} {y} 0)").map_err(formatting)?;
        writeln!(
            self.output,
            "    (effects (font (size 1.27 1.27)) (justify left bottom))"
        )
        .map_err(formatting)?;
        writeln!(self.output, "    (uuid \"{uuid}\")").map_err(formatting)?;
        writeln!(self.output, "  )").map_err(formatting)
    }

    fn emit_port(
        &mut self,
        circuit: &Circuit,
        placement: &SchematicPortPlacement,
    ) -> Result<(), KiCadSchematicExportError> {
        let port = circuit
            .ports
            .iter()
            .find(|candidate| candidate.id == placement.port)
            .ok_or(KiCadSchematicExportError::InvalidDesign)?;
        let (x, y) = self.point(
            &format!("ports[{}].position", port.id.as_str()),
            &placement.position,
        )?;
        let shape = port_shape(port.direction);
        writeln!(
            self.output,
            "  (hierarchical_label {}",
            quoted(port.id.as_str())
        )
        .map_err(formatting)?;
        writeln!(self.output, "    (shape {shape})").map_err(formatting)?;
        writeln!(self.output, "    (at {x} {y} 0)").map_err(formatting)?;
        writeln!(
            self.output,
            "    (effects (font (size 1.27 1.27)) (justify left))"
        )
        .map_err(formatting)?;
        writeln!(
            self.output,
            "    (uuid \"{}\")",
            stable_uuid(&format!("port:{}", port.id.as_str()))
        )
        .map_err(formatting)?;
        writeln!(self.output, "  )").map_err(formatting)
    }

    fn emit_symbol_instance(
        &mut self,
        circuit: &Circuit,
        layout: &SchematicLayout,
        symbol: &SchematicSymbol,
        index: usize,
        instance_path: &str,
    ) -> Result<(), KiCadSchematicExportError> {
        let instance = circuit
            .instances
            .iter()
            .find(|candidate| candidate.id == symbol.instance)
            .ok_or(KiCadSchematicExportError::InvalidDesign)?;
        let library_id = library_id(&symbol.definition);
        let unit = layout
            .symbol_unit(symbol)
            .ok_or(KiCadSchematicExportError::InvalidDesign)?;
        let uuid = stable_uuid(&format!("symbol:{}", symbol.id.as_str()));
        let (x, y) = self.point(
            &format!("symbols[{}].position", symbol.id.as_str()),
            &symbol.position,
        )?;
        let angle = symbol.quarter_turns.rem_euclid(4) * 90;
        writeln!(self.output, "  (symbol").map_err(formatting)?;
        writeln!(self.output, "    (lib_id {})", quoted(&library_id)).map_err(formatting)?;
        writeln!(self.output, "    (at {x} {y} {angle})").map_err(formatting)?;
        writeln!(self.output, "    (unit {})", symbol.unit).map_err(formatting)?;
        writeln!(self.output, "    (exclude_from_sim no)").map_err(formatting)?;
        writeln!(self.output, "    (in_bom yes)").map_err(formatting)?;
        writeln!(self.output, "    (on_board yes)").map_err(formatting)?;
        writeln!(self.output, "    (dnp no)").map_err(formatting)?;
        writeln!(self.output, "    (uuid \"{uuid}\")").map_err(formatting)?;
        self.instance_property("Reference", &format!("U{}", index + 1), &x, &y, false)?;
        self.instance_property("Value", instance.model.as_str(), &x, &y, false)?;
        self.instance_property("Footprint", "", &x, &y, true)?;
        self.instance_property("Datasheet", "", &x, &y, true)?;
        self.instance_property("Description", "Generated by HyperCircuit", &x, &y, true)?;
        self.instance_property(SYMBOL_ID_PROPERTY, symbol.id.as_str(), &x, &y, true)?;
        self.instance_property(INSTANCE_ID_PROPERTY, symbol.instance.as_str(), &x, &y, true)?;
        for pin in &unit.pins {
            writeln!(self.output, "    (pin {}", quoted(pin.pin.as_str())).map_err(formatting)?;
            writeln!(
                self.output,
                "      (uuid \"{}\")",
                stable_uuid(&format!(
                    "symbol-pin:{}:{}",
                    symbol.id.as_str(),
                    pin.pin.as_str()
                ))
            )
            .map_err(formatting)?;
            writeln!(self.output, "    )").map_err(formatting)?;
        }
        writeln!(self.output, "    (instances").map_err(formatting)?;
        writeln!(
            self.output,
            "      (project {}",
            quoted(&self.options.project_name)
        )
        .map_err(formatting)?;
        writeln!(self.output, "        (path {}", quoted(instance_path)).map_err(formatting)?;
        writeln!(
            self.output,
            "          (reference {})",
            quoted(&format!("U{}", index + 1))
        )
        .map_err(formatting)?;
        writeln!(self.output, "          (unit {})", symbol.unit).map_err(formatting)?;
        writeln!(self.output, "        )").map_err(formatting)?;
        writeln!(self.output, "      )").map_err(formatting)?;
        writeln!(self.output, "    )").map_err(formatting)?;
        writeln!(self.output, "  )").map_err(formatting)?;
        self.symbols.push(KiCadSchematicSymbolProjection {
            symbol: symbol.id.clone(),
            library_id,
            uuid,
        });
        Ok(())
    }

    fn instance_property(
        &mut self,
        name: &str,
        value: &str,
        x: &str,
        y: &str,
        hidden: bool,
    ) -> Result<(), KiCadSchematicExportError> {
        writeln!(
            self.output,
            "    (property {} {}",
            quoted(name),
            quoted(value)
        )
        .map_err(formatting)?;
        writeln!(self.output, "      (at {x} {y} 0)").map_err(formatting)?;
        write!(self.output, "      (effects (font (size 1.27 1.27))").map_err(formatting)?;
        if hidden {
            write!(self.output, " (hide yes)").map_err(formatting)?;
        }
        writeln!(self.output, ")").map_err(formatting)?;
        writeln!(self.output, "    )").map_err(formatting)
    }

    fn point(
        &mut self,
        field: &str,
        point: &SchematicPoint,
    ) -> Result<(String, String), KiCadSchematicExportError> {
        Ok((
            self.scalar(&format!("{field}.x"), &point.x)?,
            self.scalar(&format!("{field}.y"), &point.y)?,
        ))
    }

    fn scalar(&mut self, field: &str, value: &Real) -> Result<String, KiCadSchematicExportError> {
        let Some(finite) = value.to_f64_lossy().filter(|value| value.is_finite()) else {
            return Err(KiCadSchematicExportError::NonFiniteScalar(field.into()));
        };
        let emitted = format!("{:.*}", self.options.decimal_places, finite);
        self.numeric_projections
            .push(KiCadSchematicNumericProjection {
                field: field.into(),
                source: value.to_string(),
                emitted: emitted.clone(),
            });
        Ok(emitted)
    }
}

struct BookEmitter<'a> {
    layout: &'a SchematicLayout,
    circuit: &'a Circuit,
    options: KiCadSchematicBookExportOptions,
    projections: Vec<KiCadSchematicSheetProjection>,
}

impl<'a> BookEmitter<'a> {
    fn new(
        layout: &'a SchematicLayout,
        circuit: &'a Circuit,
        options: KiCadSchematicBookExportOptions,
    ) -> Self {
        Self {
            layout,
            circuit,
            options,
            projections: Vec::new(),
        }
    }

    fn emit(mut self) -> Result<KiCadSchematicBookExportReport, KiCadSchematicExportError> {
        let ordered = ordered_sheets(&self.layout.sheets);
        let root = ordered
            .first()
            .ok_or(KiCadSchematicExportError::BookRequiresExplicitHierarchy)?;
        let root_uuid = stable_uuid(&format!("schematic-file:{}", root.id.as_str()));
        for (index, sheet) in ordered.iter().enumerate() {
            let page = index + 1;
            let filename = if page == 1 {
                self.options.root_filename.clone()
            } else {
                format!(
                    "sheet-{page}-{}.kicad_sch",
                    filename_slug(sheet.id.as_str())
                )
            };
            let instance_path = if sheet.parent.is_none() {
                format!("/{root_uuid}")
            } else {
                let parent_id = sheet.parent.as_ref().ok_or_else(|| {
                    KiCadSchematicExportError::InvalidBookState(format!(
                        "sheet {} lost its parent identity",
                        sheet.id.as_str()
                    ))
                })?;
                let parent = self
                    .projections
                    .iter()
                    .find(|projection| &projection.sheet == parent_id)
                    .ok_or_else(|| {
                        KiCadSchematicExportError::InvalidBookState(format!(
                            "parent {} was not ordered before child {}",
                            parent_id.as_str(),
                            sheet.id.as_str()
                        ))
                    })?;
                format!(
                    "{}/{}",
                    parent.instance_path,
                    stable_uuid(&format!("sheet-object:{}", sheet.id.as_str()))
                )
            };
            self.projections.push(KiCadSchematicSheetProjection {
                sheet: sheet.id.clone(),
                filename,
                file_uuid: stable_uuid(&format!("schematic-file:{}", sheet.id.as_str())),
                instance_path,
                page,
            });
        }

        let mut files = Vec::with_capacity(ordered.len());
        for sheet in ordered {
            files.push(self.emit_file(sheet)?);
        }
        Ok(KiCadSchematicBookExportReport {
            root_filename: self.options.root_filename,
            files,
        })
    }

    fn emit_file(
        &self,
        sheet: &SchematicSheet,
    ) -> Result<KiCadSchematicBookFile, KiCadSchematicExportError> {
        let projection = self
            .projections
            .iter()
            .find(|projection| projection.sheet == sheet.id)
            .ok_or_else(|| {
                KiCadSchematicExportError::InvalidBookState(format!(
                    "sheet {} has no file projection",
                    sheet.id.as_str()
                ))
            })?
            .clone();
        let mut emitter = SchematicEmitter::new(self.options.schematic.clone());
        writeln!(emitter.output, "(kicad_sch").map_err(formatting)?;
        writeln!(emitter.output, "  (version {KICAD_SCHEMATIC_VERSION})").map_err(formatting)?;
        writeln!(emitter.output, "  (generator \"hypercircuit\")").map_err(formatting)?;
        writeln!(emitter.output, "  (generator_version \"0.3\")").map_err(formatting)?;
        writeln!(emitter.output, "  (uuid \"{}\")", projection.file_uuid).map_err(formatting)?;
        writeln!(emitter.output, "  (paper \"A4\")").map_err(formatting)?;
        writeln!(emitter.output, "  (title_block").map_err(formatting)?;
        writeln!(emitter.output, "    (title {})", quoted(&sheet.title)).map_err(formatting)?;
        writeln!(
            emitter.output,
            "    (comment 1 {})",
            quoted(&format!("{SHEET_ID_COMMENT_PREFIX}{}", sheet.id.as_str()))
        )
        .map_err(formatting)?;
        writeln!(emitter.output, "  )").map_err(formatting)?;

        let symbols = sheet
            .symbols
            .iter()
            .map(|id| {
                self.layout
                    .symbols
                    .iter()
                    .find(|symbol| &symbol.id == id)
                    .ok_or_else(|| {
                        KiCadSchematicExportError::InvalidBookState(format!(
                            "sheet {} references missing symbol {}",
                            sheet.id.as_str(),
                            id.as_str()
                        ))
                    })
            })
            .collect::<Result<Vec<_>, _>>()?;
        writeln!(emitter.output, "  (lib_symbols").map_err(formatting)?;
        let definition_ids = symbols
            .iter()
            .map(|symbol| symbol.definition.clone())
            .collect::<BTreeSet<_>>();
        for definition in self
            .layout
            .symbol_definitions
            .iter()
            .filter(|definition| definition_ids.contains(&definition.id))
        {
            emitter
                .emit_library_symbol(self.circuit, definition)
                .map_err(|error| {
                    KiCadSchematicExportError::InvalidBookState(format!(
                        "sheet {} library definition {}: {error}",
                        sheet.id.as_str(),
                        definition.id.as_str()
                    ))
                })?;
        }
        writeln!(emitter.output, "  )").map_err(formatting)?;

        for id in &sheet.wires {
            let wire = self
                .layout
                .wires
                .iter()
                .find(|wire| &wire.id == id)
                .ok_or_else(|| {
                    KiCadSchematicExportError::InvalidBookState(format!(
                        "sheet {} references missing wire {}",
                        sheet.id.as_str(),
                        id.as_str()
                    ))
                })?;
            emitter.emit_wire(self.layout, wire).map_err(|error| {
                KiCadSchematicExportError::InvalidBookState(format!(
                    "sheet {} wire {}: {error}",
                    sheet.id.as_str(),
                    wire.id.as_str()
                ))
            })?;
        }
        for id in &sheet.labels {
            let label = self
                .layout
                .labels
                .iter()
                .find(|label| &label.id == id)
                .ok_or_else(|| {
                    KiCadSchematicExportError::InvalidBookState(format!(
                        "sheet {} references missing label {}",
                        sheet.id.as_str(),
                        id.as_str()
                    ))
                })?;
            emitter.emit_label(label).map_err(|error| {
                KiCadSchematicExportError::InvalidBookState(format!(
                    "sheet {} label {}: {error}",
                    sheet.id.as_str(),
                    label.id.as_str()
                ))
            })?;
        }
        for id in &sheet.ports {
            let port = self
                .layout
                .ports
                .iter()
                .find(|port| &port.port == id)
                .ok_or_else(|| {
                    KiCadSchematicExportError::InvalidBookState(format!(
                        "sheet {} references missing circuit port {}",
                        sheet.id.as_str(),
                        id.as_str()
                    ))
                })?;
            emitter.emit_port(self.circuit, port).map_err(|error| {
                KiCadSchematicExportError::InvalidBookState(format!(
                    "sheet {} circuit port {}: {error}",
                    sheet.id.as_str(),
                    port.port.as_str()
                ))
            })?;
        }
        for port in self.child_side_ports(&sheet.id) {
            emitter
                .emit_sheet_hierarchical_label(port)
                .map_err(|error| {
                    KiCadSchematicExportError::InvalidBookState(format!(
                        "sheet {} boundary label {}: {error}",
                        sheet.id.as_str(),
                        port.id.as_str()
                    ))
                })?;
        }
        for (index, symbol) in symbols.iter().enumerate() {
            emitter
                .emit_symbol_instance(
                    self.circuit,
                    self.layout,
                    symbol,
                    index,
                    &projection.instance_path,
                )
                .map_err(|error| {
                    KiCadSchematicExportError::InvalidBookState(format!(
                        "sheet {} symbol instance {}: {error}",
                        sheet.id.as_str(),
                        symbol.id.as_str()
                    ))
                })?;
        }
        for child in self
            .layout
            .sheets
            .iter()
            .filter(|candidate| candidate.parent.as_ref() == Some(&sheet.id))
        {
            self.emit_child_sheet(&mut emitter, sheet, child, &projection)
                .map_err(|error| {
                    KiCadSchematicExportError::InvalidBookState(format!(
                        "sheet {} child {}: {error}",
                        sheet.id.as_str(),
                        child.id.as_str()
                    ))
                })?;
        }
        if sheet.parent.is_none() {
            writeln!(emitter.output, "  (sheet_instances").map_err(formatting)?;
            writeln!(emitter.output, "    (path \"/\" (page \"1\"))").map_err(formatting)?;
            writeln!(emitter.output, "  )").map_err(formatting)?;
        }
        writeln!(emitter.output, "  (embedded_fonts no)").map_err(formatting)?;
        writeln!(emitter.output, ")").map_err(formatting)?;
        Ok(KiCadSchematicBookFile {
            projection,
            schematic: emitter.output,
            numeric_projections: emitter.numeric_projections,
            symbols: emitter.symbols,
            wires: emitter.wires,
            generated_connectivity_labels: emitter.generated_connectivity_labels,
            omissions: emitter.omissions,
        })
    }

    fn child_side_ports(&self, sheet: &SchematicSheetId) -> Vec<&SchematicSheetPort> {
        self.layout
            .sheet_links
            .iter()
            .filter_map(|link| {
                let port = self
                    .layout
                    .sheet_ports
                    .iter()
                    .find(|port| port.id == link.child_port)?;
                (&port.sheet == sheet).then_some(port)
            })
            .collect()
    }

    fn emit_child_sheet(
        &self,
        emitter: &mut SchematicEmitter,
        parent_sheet: &SchematicSheet,
        child_sheet: &SchematicSheet,
        parent_projection: &KiCadSchematicSheetProjection,
    ) -> Result<(), KiCadSchematicExportError> {
        let child_projection = self
            .projections
            .iter()
            .find(|projection| projection.sheet == child_sheet.id)
            .ok_or_else(|| {
                KiCadSchematicExportError::InvalidBookState(format!(
                    "child sheet {} has no file projection",
                    child_sheet.id.as_str()
                ))
            })?;
        let links = self
            .layout
            .sheet_links
            .iter()
            .filter_map(|link| {
                let parent_port = self
                    .layout
                    .sheet_ports
                    .iter()
                    .find(|port| port.id == link.parent_port)?;
                let child_port = self
                    .layout
                    .sheet_ports
                    .iter()
                    .find(|port| port.id == link.child_port)?;
                (parent_port.sheet == parent_sheet.id && child_port.sheet == child_sheet.id)
                    .then_some((link, parent_port, child_port))
            })
            .collect::<Vec<_>>();
        let (origin, width, height) = generated_sheet_box(
            &links
                .iter()
                .map(|(_, parent, _)| parent.position.clone())
                .collect::<Vec<_>>(),
        )?;
        let (x, y) = emitter.point(
            &format!("sheets[{}].box.origin", child_sheet.id.as_str()),
            &origin,
        )?;
        let width = emitter.scalar(
            &format!("sheets[{}].box.width", child_sheet.id.as_str()),
            &width,
        )?;
        let height = emitter.scalar(
            &format!("sheets[{}].box.height", child_sheet.id.as_str()),
            &height,
        )?;
        writeln!(emitter.output, "  (sheet").map_err(formatting)?;
        writeln!(emitter.output, "    (at {x} {y})").map_err(formatting)?;
        writeln!(emitter.output, "    (size {width} {height})").map_err(formatting)?;
        writeln!(emitter.output, "    (exclude_from_sim no)").map_err(formatting)?;
        writeln!(emitter.output, "    (in_bom yes)").map_err(formatting)?;
        writeln!(emitter.output, "    (on_board yes)").map_err(formatting)?;
        writeln!(emitter.output, "    (dnp no)").map_err(formatting)?;
        writeln!(emitter.output, "    (stroke (width 0.1524) (type solid))").map_err(formatting)?;
        writeln!(emitter.output, "    (fill (color 0 0 0 0.0000))").map_err(formatting)?;
        writeln!(
            emitter.output,
            "    (uuid \"{}\")",
            stable_uuid(&format!("sheet-object:{}", child_sheet.id.as_str()))
        )
        .map_err(formatting)?;
        emitter.instance_property("Sheetname", &child_sheet.title, &x, &y, false)?;
        emitter.instance_property("Sheetfile", &child_projection.filename, &x, &y, false)?;
        emitter.instance_property(
            CHILD_SHEET_ID_PROPERTY,
            child_sheet.id.as_str(),
            &x,
            &y,
            true,
        )?;
        for (index, (link, parent, child)) in links.iter().enumerate() {
            if parent.name != child.name {
                emitter
                    .omissions
                    .push(KiCadSchematicExportOmission::SheetPortNameProjected {
                        parent_port: parent.id.clone(),
                        child_port: child.id.clone(),
                        parent_name: parent.name.clone(),
                        emitted: child.name.clone(),
                    });
            }
            emitter.instance_property(
                &format!("{SHEET_LINK_PROPERTY_PREFIX}{}", index + 1),
                &encode_fields(&[
                    link.id.as_str(),
                    parent.id.as_str(),
                    child.id.as_str(),
                    &parent.name,
                    &child.name,
                    parent.net.as_str(),
                ]),
                &x,
                &y,
                true,
            )?;
            let (pin_x, pin_y) = emitter.point(
                &format!(
                    "sheets[{}].pins[{}]",
                    child_sheet.id.as_str(),
                    parent.id.as_str()
                ),
                &parent.position,
            )?;
            writeln!(emitter.output, "    (pin {} passive", quoted(&child.name))
                .map_err(formatting)?;
            writeln!(emitter.output, "      (at {pin_x} {pin_y} 0)").map_err(formatting)?;
            writeln!(
                emitter.output,
                "      (uuid \"{}\")",
                stable_uuid(&format!("sheet-pin:{}", parent.id.as_str()))
            )
            .map_err(formatting)?;
            writeln!(
                emitter.output,
                "      (effects (font (size 1.27 1.27)) (justify right))"
            )
            .map_err(formatting)?;
            writeln!(emitter.output, "    )").map_err(formatting)?;
        }
        writeln!(emitter.output, "    (instances").map_err(formatting)?;
        writeln!(
            emitter.output,
            "      (project {}",
            quoted(&self.options.schematic.project_name)
        )
        .map_err(formatting)?;
        writeln!(
            emitter.output,
            "        (path {} (page {}))",
            quoted(&parent_projection.instance_path),
            quoted(&child_projection.page.to_string())
        )
        .map_err(formatting)?;
        writeln!(emitter.output, "      )").map_err(formatting)?;
        writeln!(emitter.output, "    )").map_err(formatting)?;
        writeln!(emitter.output, "  )").map_err(formatting)
    }
}

impl SchematicEmitter {
    fn emit_sheet_hierarchical_label(
        &mut self,
        port: &SchematicSheetPort,
    ) -> Result<(), KiCadSchematicExportError> {
        let (x, y) = self.point(
            &format!("sheet_ports[{}].position", port.id.as_str()),
            &port.position,
        )?;
        writeln!(self.output, "  (hierarchical_label {}", quoted(&port.name))
            .map_err(formatting)?;
        writeln!(self.output, "    (shape passive)").map_err(formatting)?;
        writeln!(self.output, "    (at {x} {y} 180)").map_err(formatting)?;
        writeln!(
            self.output,
            "    (effects (font (size 1.27 1.27)) (justify right))"
        )
        .map_err(formatting)?;
        writeln!(
            self.output,
            "    (uuid \"{}\")",
            stable_uuid(&format!("sheet-label:{}", port.id.as_str()))
        )
        .map_err(formatting)?;
        writeln!(self.output, "  )").map_err(formatting)
    }
}

/// One finite KiCad coordinate imported as an exact decimal.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct KiCadSchematicNumericImport {
    /// Imported field.
    pub field: String,
    /// Native decimal token.
    pub token: String,
}

/// Explicit assumptions or identity changes made during schematic import.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum KiCadSchematicImportOmission {
    /// KiCad wire UUIDs became new circuit-owned drawing identities.
    GeneratedWireIdentities { count: usize },
    /// KiCad label UUIDs became new circuit-owned drawing identities.
    GeneratedLabelIdentities { count: usize },
    /// Supplied files not reachable from the root Sheetfile graph were ignored.
    UnreferencedFiles { filenames: Vec<String> },
}

/// Reconstructed circuit-bound drawing and its import audit.
#[derive(Clone, Debug, PartialEq)]
pub struct KiCadSchematicImportReport {
    /// Editable schematic presentation validated against the supplied circuit.
    pub layout: SchematicLayout,
    /// Every finite decimal imported as exact rational data.
    pub numeric_imports: Vec<KiCadSchematicNumericImport>,
    /// Identity changes that KiCad cannot natively retain.
    pub omissions: Vec<KiCadSchematicImportOmission>,
}

/// One file identity accepted while reconstructing a native hierarchy.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct KiCadSchematicSheetImport {
    /// Reconstructed circuit-owned sheet identity.
    pub sheet: SchematicSheetId,
    /// Native file basename.
    pub filename: String,
    /// Native file-header UUID.
    pub file_uuid: String,
}

/// Multi-file hierarchy reconstructed against authoritative circuit truth.
#[derive(Clone, Debug, PartialEq)]
pub struct KiCadSchematicBookImportReport {
    /// Complete editable hierarchical presentation.
    pub layout: SchematicLayout,
    /// Root-first file identities.
    pub files: Vec<KiCadSchematicSheetImport>,
    /// Every native decimal imported as exact rational data.
    pub numeric_imports: Vec<KiCadSchematicNumericImport>,
    /// Audited identity changes and unreferenced inputs.
    pub omissions: Vec<KiCadSchematicImportOmission>,
}

/// Failure to reconstruct the supported native KiCad schematic subset.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum KiCadSchematicImportError {
    /// S-expression structure was malformed or outside the supported subset.
    Parse(String),
    /// A native numeric token was invalid.
    InvalidNumber { field: String, token: String },
    /// A generated symbol lacks required circuit identity metadata.
    MissingSymbolMetadata(String),
    /// A retained or generated identity was invalid.
    InvalidIdentifier(String),
    /// A wire had no authoritative circuit-net evidence.
    UnresolvedWireNet(String),
    /// Native geometry produced a drawing that disagrees with the circuit.
    InvalidImportedLayout { issue_count: usize },
    /// File input failed.
    Io(String),
    /// A Sheetfile reference was absent from the supplied package.
    MissingBookFile(String),
    /// Two files claimed the same retained sheet identity.
    DuplicateBookSheet(SchematicSheetId),
    /// Native Sheetfile references contain a cycle.
    BookFileCycle(String),
    /// Hidden hierarchy metadata was missing or malformed.
    InvalidBookMetadata(String),
}

impl Display for KiCadSchematicImportError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Parse(message) => write!(formatter, "KiCad schematic parse error: {message}"),
            Self::InvalidNumber { field, token } => {
                write!(
                    formatter,
                    "invalid KiCad schematic number {token:?} for {field}"
                )
            }
            Self::MissingSymbolMetadata(symbol) => {
                write!(
                    formatter,
                    "KiCad symbol {symbol} lacks HyperCircuit identity metadata"
                )
            }
            Self::InvalidIdentifier(value) => {
                write!(formatter, "invalid imported schematic identity {value:?}")
            }
            Self::UnresolvedWireNet(wire) => {
                write!(
                    formatter,
                    "cannot resolve authoritative net for KiCad wire {wire}"
                )
            }
            Self::InvalidImportedLayout { issue_count } => write!(
                formatter,
                "imported KiCad schematic has {issue_count} validation issue(s)"
            ),
            Self::Io(message) => write!(formatter, "failed to read KiCad schematic: {message}"),
            Self::MissingBookFile(filename) => {
                write!(
                    formatter,
                    "KiCad schematic book is missing file {filename:?}"
                )
            }
            Self::DuplicateBookSheet(sheet) => write!(
                formatter,
                "multiple KiCad files claim schematic sheet {}",
                sheet.as_str()
            ),
            Self::BookFileCycle(filename) => {
                write!(formatter, "KiCad Sheetfile cycle reaches {filename:?}")
            }
            Self::InvalidBookMetadata(message) => {
                write!(
                    formatter,
                    "invalid KiCad schematic book metadata: {message}"
                )
            }
        }
    }
}

impl std::error::Error for KiCadSchematicImportError {}

impl KiCadSchematicImportReport {
    /// Imports the supported flat KiCad schematic subset against circuit truth.
    pub fn from_str(source: &str, circuit: &Circuit) -> Result<Self, KiCadSchematicImportError> {
        let root = sexp::parse(source).map_err(KiCadSchematicImportError::Parse)?;
        if root.list_name() != Some("kicad_sch") {
            return Err(KiCadSchematicImportError::Parse(
                "root expression is not kicad_sch".into(),
            ));
        }
        SchematicImporter::new(&root, circuit).import()
    }

    /// Imports a native `.kicad_sch` file against circuit truth.
    pub fn from_path(path: &Path, circuit: &Circuit) -> Result<Self, KiCadSchematicImportError> {
        let source = std::fs::read_to_string(path)
            .map_err(|error| KiCadSchematicImportError::Io(error.to_string()))?;
        Self::from_str(&source, circuit)
    }
}

impl KiCadSchematicBookImportReport {
    /// Reconstructs a native multi-file schematic package against circuit truth.
    pub fn from_files(
        root_filename: &str,
        files: &BTreeMap<String, String>,
        circuit: &Circuit,
    ) -> Result<Self, KiCadSchematicImportError> {
        if !valid_kicad_filename(root_filename) {
            return Err(KiCadSchematicImportError::InvalidBookMetadata(format!(
                "invalid root filename {root_filename:?}"
            )));
        }
        let mut roots = BTreeMap::new();
        for (filename, source) in files {
            if !valid_kicad_filename(filename) {
                return Err(KiCadSchematicImportError::InvalidBookMetadata(format!(
                    "invalid package filename {filename:?}"
                )));
            }
            let root = sexp::parse(source).map_err(|error| {
                KiCadSchematicImportError::Parse(format!("{filename}: {error}"))
            })?;
            if root.list_name() != Some("kicad_sch") {
                return Err(KiCadSchematicImportError::Parse(format!(
                    "{filename}: root expression is not kicad_sch"
                )));
            }
            roots.insert(filename.clone(), root);
        }
        if !roots.contains_key(root_filename) {
            return Err(KiCadSchematicImportError::MissingBookFile(
                root_filename.into(),
            ));
        }
        BookImporter::new(roots, circuit).import(root_filename)
    }
}

#[derive(Clone)]
struct NativeLabel {
    uuid: String,
    text: String,
    position: SchematicPoint,
}

struct SchematicImporter<'a> {
    root: &'a Sexp,
    circuit: &'a Circuit,
    numeric_imports: Vec<KiCadSchematicNumericImport>,
}

impl<'a> SchematicImporter<'a> {
    fn new(root: &'a Sexp, circuit: &'a Circuit) -> Self {
        Self {
            root,
            circuit,
            numeric_imports: Vec::new(),
        }
    }

    fn import(mut self) -> Result<KiCadSchematicImportReport, KiCadSchematicImportError> {
        if !self.circuit.validate().is_valid() {
            return Err(KiCadSchematicImportError::InvalidImportedLayout { issue_count: 1 });
        }
        let libraries = self.library_symbols()?;
        let (symbol_definitions, symbols) = self.import_symbols(&libraries)?;
        let ports = self.import_ports_excluding(&BTreeSet::new())?;
        let native_labels = self.import_native_labels()?;
        let wire_nodes = self.root.named_children("wire").collect::<Vec<_>>();
        let synthetic = wire_nodes
            .iter()
            .filter_map(|wire| wire.named_child("uuid")?.atom_at(1))
            .map(|uuid| stable_uuid(&format!("synthetic-net-label:{uuid}")))
            .collect::<BTreeSet<_>>();
        let labels = native_labels
            .iter()
            .filter(|label| !synthetic.contains(&label.uuid))
            .map(|label| self.semantic_label(label))
            .collect::<Result<Vec<_>, _>>()?;
        let mut layout = SchematicLayout {
            symbol_definitions,
            symbols,
            ports,
            labels,
            ..SchematicLayout::default()
        };
        layout.wires =
            coalesce_native_wires(self.import_wires(&layout, &native_labels, &wire_nodes)?);
        let validation = layout.validate(self.circuit);
        if !validation.is_valid() {
            return Err(KiCadSchematicImportError::InvalidImportedLayout {
                issue_count: validation.issues.len(),
            });
        }
        let mut omissions = Vec::new();
        if !layout.wires.is_empty() {
            omissions.push(KiCadSchematicImportOmission::GeneratedWireIdentities {
                count: layout.wires.len(),
            });
        }
        if !layout.labels.is_empty() {
            omissions.push(KiCadSchematicImportOmission::GeneratedLabelIdentities {
                count: layout.labels.len(),
            });
        }
        Ok(KiCadSchematicImportReport {
            layout,
            numeric_imports: self.numeric_imports,
            omissions,
        })
    }

    fn library_symbols(&self) -> Result<BTreeMap<String, &'a Sexp>, KiCadSchematicImportError> {
        let libraries = self
            .root
            .named_child("lib_symbols")
            .ok_or_else(|| KiCadSchematicImportError::Parse("missing lib_symbols".into()))?;
        let mut result = BTreeMap::new();
        for symbol in libraries.named_children("symbol") {
            let id = symbol.atom_at(1).ok_or_else(|| {
                KiCadSchematicImportError::Parse("library symbol has no id".into())
            })?;
            result.insert(id.to_owned(), symbol);
        }
        Ok(result)
    }

    fn import_symbols(
        &mut self,
        libraries: &BTreeMap<String, &'a Sexp>,
    ) -> Result<(Vec<SchematicSymbolDefinition>, Vec<SchematicSymbol>), KiCadSchematicImportError>
    {
        let mut definitions = BTreeMap::new();
        let mut symbols = Vec::new();
        for node in self.root.named_children("symbol") {
            let library_id = required_child_atom(node, "lib_id", 1)?;
            let library = libraries.get(library_id).ok_or_else(|| {
                KiCadSchematicImportError::Parse(format!(
                    "symbol references absent library id {library_id}"
                ))
            })?;
            let native_uuid = required_child_atom(node, "uuid", 1)?;
            let symbol_id = property(node, SYMBOL_ID_PROPERTY)
                .ok_or_else(|| KiCadSchematicImportError::MissingSymbolMetadata(native_uuid.into()))
                .and_then(|value| {
                    SchematicSymbolId::new(value)
                        .map_err(|_| KiCadSchematicImportError::InvalidIdentifier(value.into()))
                })?;
            let instance = property(node, INSTANCE_ID_PROPERTY)
                .ok_or_else(|| KiCadSchematicImportError::MissingSymbolMetadata(native_uuid.into()))
                .and_then(|value| {
                    CircuitInstanceId::new(value)
                        .map_err(|_| KiCadSchematicImportError::InvalidIdentifier(value.into()))
                })?;
            let position = self.parse_at(node, &format!("symbols[{native_uuid}].position"))?;
            let at = node
                .named_child("at")
                .ok_or_else(|| KiCadSchematicImportError::Parse("symbol missing at".into()))?;
            let angle = at.atom_at(3).unwrap_or("0").parse::<i32>().map_err(|_| {
                KiCadSchematicImportError::InvalidNumber {
                    field: format!("symbols[{native_uuid}].rotation"),
                    token: at.atom_at(3).unwrap_or("").into(),
                }
            })?;
            if angle.rem_euclid(90) != 0 {
                return Err(KiCadSchematicImportError::Parse(format!(
                    "symbol {native_uuid} rotation is not a quarter turn"
                )));
            }
            let unit = node
                .named_child("unit")
                .and_then(|child| child.atom_at(1))
                .unwrap_or("1")
                .parse::<u16>()
                .map_err(|_| {
                    KiCadSchematicImportError::Parse(format!(
                        "symbol {native_uuid} has invalid unit"
                    ))
                })?;
            let definition_id = property(library, DEFINITION_ID_PROPERTY)
                .ok_or_else(|| KiCadSchematicImportError::MissingSymbolMetadata(native_uuid.into()))
                .and_then(|value| {
                    SchematicSymbolDefinitionId::new(value)
                        .map_err(|_| KiCadSchematicImportError::InvalidIdentifier(value.into()))
                })?;
            if !definitions.contains_key(&definition_id) {
                let definition = self.import_library_definition(library, native_uuid)?;
                definitions.insert(definition.id.clone(), definition);
            }
            symbols.push(SchematicSymbol {
                id: symbol_id,
                instance,
                definition: definition_id,
                unit,
                position,
                quarter_turns: i8::try_from(angle.div_euclid(90).rem_euclid(4))
                    .expect("quarter turn is in range"),
            });
        }
        Ok((definitions.into_values().collect(), symbols))
    }

    fn import_library_definition(
        &mut self,
        library: &Sexp,
        native_uuid: &str,
    ) -> Result<SchematicSymbolDefinition, KiCadSchematicImportError> {
        let id_text = property(library, DEFINITION_ID_PROPERTY)
            .ok_or_else(|| KiCadSchematicImportError::MissingSymbolMetadata(native_uuid.into()))?;
        let id = SchematicSymbolDefinitionId::new(id_text)
            .map_err(|_| KiCadSchematicImportError::InvalidIdentifier(id_text.into()))?;
        let model_text = property(library, DEFINITION_MODEL_PROPERTY)
            .ok_or_else(|| KiCadSchematicImportError::MissingSymbolMetadata(native_uuid.into()))?;
        let model = crate::DeviceModelId::new(model_text)
            .map_err(|_| KiCadSchematicImportError::InvalidIdentifier(model_text.into()))?;
        let name = property(library, DEFINITION_NAME_PROPERTY)
            .ok_or_else(|| KiCadSchematicImportError::MissingSymbolMetadata(native_uuid.into()))?
            .to_owned();
        let mut units = Vec::new();
        for node in library.named_children("symbol") {
            let Some(unit) = library_symbol_unit(node) else {
                continue;
            };
            let field = format!("symbol_definitions[{}].units[{unit}]", id.as_str());
            let body_width = self.property_number(
                library,
                &unit_body_property(unit, "Width"),
                &format!("{field}.body_width"),
            )?;
            let body_height = self.property_number(
                library,
                &unit_body_property(unit, "Height"),
                &format!("{field}.body_height"),
            )?;
            let pins = self.library_pins(node, native_uuid)?;
            let graphics = self.library_graphics(library, node, unit, &field)?;
            units.push(SchematicSymbolUnit {
                unit,
                body_width,
                body_height,
                pins,
                graphics,
            });
        }
        units.sort_by_key(|unit| unit.unit);
        Ok(SchematicSymbolDefinition {
            id,
            model,
            name,
            units,
        })
    }

    fn library_pins(
        &mut self,
        unit: &Sexp,
        native_uuid: &str,
    ) -> Result<Vec<SchematicPinPlacement>, KiCadSchematicImportError> {
        let mut pins = Vec::new();
        for pin in unit.named_children("pin") {
            let number = pin
                .named_child("number")
                .and_then(|number| number.atom_at(1))
                .ok_or_else(|| {
                    KiCadSchematicImportError::Parse(format!(
                        "symbol {native_uuid} library pin has no number"
                    ))
                })?;
            let position = self.parse_at(pin, &format!("symbols[{native_uuid}].pins[{number}]"))?;
            let angle = pin
                .named_child("at")
                .and_then(|at| at.atom_at(3))
                .unwrap_or("0")
                .parse::<i32>()
                .map_err(|_| {
                    KiCadSchematicImportError::Parse(format!(
                        "symbol {native_uuid} pin {number} has invalid angle"
                    ))
                })?;
            let side = match angle.rem_euclid(360) {
                0 => SchematicPinSide::Left,
                90 => SchematicPinSide::Top,
                180 => SchematicPinSide::Right,
                270 => SchematicPinSide::Bottom,
                _ => {
                    return Err(KiCadSchematicImportError::Parse(format!(
                        "symbol {native_uuid} pin {number} is not axis-aligned"
                    )));
                }
            };
            pins.push(SchematicPinPlacement {
                pin: PinRef::new(number)
                    .map_err(|_| KiCadSchematicImportError::InvalidIdentifier(number.into()))?,
                position,
                side,
            });
        }
        Ok(pins)
    }

    fn property_number(
        &mut self,
        node: &Sexp,
        name: &str,
        field: &str,
    ) -> Result<Real, KiCadSchematicImportError> {
        self.number(property(node, name), field)
    }

    fn library_graphics(
        &mut self,
        library: &Sexp,
        unit_node: &Sexp,
        unit: u16,
        field: &str,
    ) -> Result<Vec<SchematicGraphic>, KiCadSchematicImportError> {
        let kinds = property(library, &format!("HyperCircuit Unit {unit} Graphics"))
            .unwrap_or("")
            .split(',')
            .filter(|kind| !kind.is_empty())
            .collect::<Vec<_>>();
        let nodes = unit_node
            .children()
            .iter()
            .filter(|node| {
                matches!(
                    node.list_name(),
                    Some("polyline" | "rectangle" | "circle" | "arc" | "text")
                )
            })
            .collect::<Vec<_>>();
        if kinds.len() != nodes.len() {
            return Err(KiCadSchematicImportError::Parse(format!(
                "{field} graphic metadata count does not match native graphics"
            )));
        }
        nodes
            .into_iter()
            .zip(kinds)
            .enumerate()
            .map(|(index, (node, kind))| {
                self.library_graphic(node, kind, &format!("{field}.graphics[{index}]"))
            })
            .collect()
    }

    fn library_graphic(
        &mut self,
        node: &Sexp,
        kind: &str,
        field: &str,
    ) -> Result<SchematicGraphic, KiCadSchematicImportError> {
        let stroke_width = self.number(
            node.named_child("stroke")
                .and_then(|stroke| stroke.named_child("width"))
                .and_then(|width| width.atom_at(1)),
            &format!("{field}.stroke_width"),
        );
        let fill = || -> Result<SchematicGraphicFill, KiCadSchematicImportError> {
            match node
                .named_child("fill")
                .and_then(|fill| fill.named_child("type"))
                .and_then(|kind| kind.atom_at(1))
                .unwrap_or("none")
            {
                "none" => Ok(SchematicGraphicFill::None),
                "background" => Ok(SchematicGraphicFill::Background),
                "outline" => Ok(SchematicGraphicFill::Foreground),
                value => Err(KiCadSchematicImportError::Parse(format!(
                    "{field} has unsupported fill {value}"
                ))),
            }
        };
        match kind {
            "line" => {
                let points = self.library_polyline_points(node, field)?;
                if points.len() != 2 {
                    return Err(KiCadSchematicImportError::Parse(format!(
                        "{field} line does not contain two points"
                    )));
                }
                Ok(SchematicGraphic::Line {
                    from: points[0].clone(),
                    to: points[1].clone(),
                    stroke_width: stroke_width?,
                })
            }
            "rectangle" => Ok(SchematicGraphic::Rectangle {
                start: self.parse_named_point(node, "start", &format!("{field}.start"))?,
                end: self.parse_named_point(node, "end", &format!("{field}.end"))?,
                stroke_width: stroke_width?,
                fill: fill()?,
            }),
            "circle" => Ok(SchematicGraphic::Circle {
                center: self.parse_named_point(node, "center", &format!("{field}.center"))?,
                radius: self.number(
                    node.named_child("radius")
                        .and_then(|radius| radius.atom_at(1)),
                    &format!("{field}.radius"),
                )?,
                stroke_width: stroke_width?,
                fill: fill()?,
            }),
            "arc" => Ok(SchematicGraphic::Arc {
                start: self.parse_named_point(node, "start", &format!("{field}.start"))?,
                mid: self.parse_named_point(node, "mid", &format!("{field}.mid"))?,
                end: self.parse_named_point(node, "end", &format!("{field}.end"))?,
                stroke_width: stroke_width?,
            }),
            "polyline" => {
                let mut points = self.library_polyline_points(node, field)?;
                let closed = points.len() > 2 && points.first() == points.last();
                if closed {
                    points.pop();
                }
                Ok(SchematicGraphic::Polyline {
                    points,
                    closed,
                    stroke_width: stroke_width?,
                    fill: fill()?,
                })
            }
            "text" => {
                let position = self.parse_at(node, &format!("{field}.position"))?;
                let angle = node
                    .named_child("at")
                    .and_then(|at| at.atom_at(3))
                    .unwrap_or("0")
                    .parse::<i32>()
                    .map_err(|_| {
                        KiCadSchematicImportError::Parse(format!(
                            "{field} text has invalid rotation"
                        ))
                    })?;
                if angle.rem_euclid(900) != 0 {
                    return Err(KiCadSchematicImportError::Parse(format!(
                        "{field} text is not quarter-turn aligned"
                    )));
                }
                let size_node = node
                    .named_child("effects")
                    .and_then(|effects| effects.named_child("font"))
                    .and_then(|font| font.named_child("size"))
                    .ok_or_else(|| {
                        KiCadSchematicImportError::Parse(format!("{field} text has no size"))
                    })?;
                Ok(SchematicGraphic::Text {
                    position,
                    text: node.atom_at(1).unwrap_or("").to_owned(),
                    size: self.number(size_node.atom_at(1), &format!("{field}.size"))?,
                    quarter_turns: i8::try_from(angle.div_euclid(900).rem_euclid(4))
                        .expect("quarter turn is in range"),
                })
            }
            value => Err(KiCadSchematicImportError::Parse(format!(
                "{field} has unsupported HyperCircuit graphic kind {value}"
            ))),
        }
    }

    fn library_polyline_points(
        &mut self,
        node: &Sexp,
        field: &str,
    ) -> Result<Vec<SchematicPoint>, KiCadSchematicImportError> {
        node.named_child("pts")
            .ok_or_else(|| KiCadSchematicImportError::Parse(format!("{field} has no points")))?
            .named_children("xy")
            .enumerate()
            .map(|(index, point)| self.parse_xy(point, &format!("{field}.points[{index}]")))
            .collect()
    }

    fn import_ports_excluding(
        &mut self,
        excluded_uuids: &BTreeSet<String>,
    ) -> Result<Vec<SchematicPortPlacement>, KiCadSchematicImportError> {
        self.root
            .named_children("hierarchical_label")
            .filter(|node| {
                node.named_child("uuid")
                    .and_then(|uuid| uuid.atom_at(1))
                    .is_none_or(|uuid| !excluded_uuids.contains(uuid))
            })
            .map(|node| {
                let id = node.atom_at(1).ok_or_else(|| {
                    KiCadSchematicImportError::Parse("hierarchical label has no name".into())
                })?;
                let port = PortId::new(id)
                    .map_err(|_| KiCadSchematicImportError::InvalidIdentifier(id.into()))?;
                if !self
                    .circuit
                    .ports
                    .iter()
                    .any(|candidate| candidate.id == port)
                {
                    return Err(KiCadSchematicImportError::InvalidIdentifier(id.into()));
                }
                let position = self.parse_at(node, &format!("ports[{id}].position"))?;
                Ok(SchematicPortPlacement { port, position })
            })
            .collect()
    }

    fn import_native_labels(&mut self) -> Result<Vec<NativeLabel>, KiCadSchematicImportError> {
        self.root
            .named_children("label")
            .map(|node| {
                let text = node
                    .atom_at(1)
                    .ok_or_else(|| KiCadSchematicImportError::Parse("label has no text".into()))?
                    .to_owned();
                let uuid = required_child_atom(node, "uuid", 1)?.to_owned();
                let position = self.parse_at(node, &format!("labels[{uuid}].position"))?;
                Ok(NativeLabel {
                    uuid,
                    text,
                    position,
                })
            })
            .collect()
    }

    fn semantic_label(
        &self,
        label: &NativeLabel,
    ) -> Result<SchematicLabel, KiCadSchematicImportError> {
        let net = NetId::new(&label.text)
            .map_err(|_| KiCadSchematicImportError::InvalidIdentifier(label.text.clone()))?;
        if !self
            .circuit
            .nets
            .iter()
            .any(|candidate| candidate.id == net)
        {
            return Err(KiCadSchematicImportError::InvalidIdentifier(
                label.text.clone(),
            ));
        }
        let id = SchematicLabelId::new(format!("kicad-label-{}", label.uuid))
            .map_err(|_| KiCadSchematicImportError::InvalidIdentifier(label.uuid.clone()))?;
        Ok(SchematicLabel {
            id,
            net,
            position: label.position.clone(),
            text: label.text.clone(),
        })
    }

    fn import_wires(
        &mut self,
        layout: &SchematicLayout,
        labels: &[NativeLabel],
        nodes: &[&Sexp],
    ) -> Result<Vec<SchematicWire>, KiCadSchematicImportError> {
        let mut result = Vec::new();
        for node in nodes {
            let uuid = required_child_atom(node, "uuid", 1)?;
            let points_node = node
                .named_child("pts")
                .ok_or_else(|| KiCadSchematicImportError::Parse("wire has no pts".into()))?;
            let points = points_node
                .named_children("xy")
                .enumerate()
                .map(|(index, point)| {
                    self.parse_xy(point, &format!("wires[{uuid}].points[{index}]"))
                })
                .collect::<Result<Vec<_>, _>>()?;
            if points.len() < 2 {
                return Err(KiCadSchematicImportError::Parse(format!(
                    "wire {uuid} has fewer than two points"
                )));
            }
            let from_evidence = endpoint_for(layout, self.circuit, &points[0]);
            let to_evidence = endpoint_for(
                layout,
                self.circuit,
                points.last().expect("wire has two points"),
            );
            let mut nets = Vec::new();
            if let Some((_, net)) = &from_evidence {
                nets.push(net.clone());
            }
            if let Some((_, net)) = &to_evidence {
                nets.push(net.clone());
            }
            for label in labels {
                if points.iter().any(|point| point == &label.position)
                    && let Ok(net) = NetId::new(&label.text)
                    && self
                        .circuit
                        .nets
                        .iter()
                        .any(|candidate| candidate.id == net)
                {
                    nets.push(net);
                }
            }
            nets.sort();
            nets.dedup();
            if nets.len() != 1 {
                return Err(KiCadSchematicImportError::UnresolvedWireNet(uuid.into()));
            }
            let net = nets.remove(0);
            let from = from_evidence
                .map(|evidence| evidence.0)
                .unwrap_or_else(|| SchematicEndpoint::Junction(points[0].clone()));
            let to = to_evidence.map(|evidence| evidence.0).unwrap_or_else(|| {
                SchematicEndpoint::Junction(points.last().expect("wire has two points").clone())
            });
            result.push(SchematicWire {
                id: SchematicWireId::new(format!("kicad-wire-{uuid}"))
                    .map_err(|_| KiCadSchematicImportError::InvalidIdentifier(uuid.into()))?,
                net,
                from,
                waypoints: points[1..points.len() - 1].to_vec(),
                to,
            });
        }
        Ok(result)
    }

    fn parse_at(
        &mut self,
        node: &Sexp,
        field: &str,
    ) -> Result<SchematicPoint, KiCadSchematicImportError> {
        self.parse_named_point(node, "at", field)
    }

    fn parse_named_point(
        &mut self,
        node: &Sexp,
        name: &str,
        field: &str,
    ) -> Result<SchematicPoint, KiCadSchematicImportError> {
        let point = node
            .named_child(name)
            .ok_or_else(|| KiCadSchematicImportError::Parse(format!("missing {field}")))?;
        self.parse_xy(point, field)
    }

    fn parse_xy(
        &mut self,
        node: &Sexp,
        field: &str,
    ) -> Result<SchematicPoint, KiCadSchematicImportError> {
        let x = self.number(node.atom_at(1), &format!("{field}.x"))?;
        let y = self.number(node.atom_at(2), &format!("{field}.y"))?;
        Ok(SchematicPoint::new(x, y))
    }

    fn number(
        &mut self,
        token: Option<&str>,
        field: &str,
    ) -> Result<Real, KiCadSchematicImportError> {
        let token =
            token.ok_or_else(|| KiCadSchematicImportError::Parse(format!("missing {field}")))?;
        let value =
            Real::from_str(token).map_err(|_| KiCadSchematicImportError::InvalidNumber {
                field: field.into(),
                token: token.into(),
            })?;
        self.numeric_imports.push(KiCadSchematicNumericImport {
            field: field.into(),
            token: token.into(),
        });
        Ok(value)
    }
}

struct PagePartial {
    filename: String,
    sheet: SchematicSheetId,
    symbol_definitions: Vec<SchematicSymbolDefinition>,
    symbols: Vec<SchematicSymbol>,
    ports: Vec<SchematicPortPlacement>,
    labels: Vec<SchematicLabel>,
    native_labels: Vec<NativeLabel>,
}

struct ChildDescriptor {
    filename: String,
}

struct BookImporter<'a> {
    roots: BTreeMap<String, Sexp>,
    circuit: &'a Circuit,
    visiting: BTreeSet<String>,
    visited: BTreeSet<String>,
    claimed_sheets: BTreeSet<SchematicSheetId>,
    sheets: Vec<SchematicSheet>,
    sheet_ports: Vec<SchematicSheetPort>,
    sheet_links: Vec<SchematicSheetLink>,
    files: Vec<KiCadSchematicSheetImport>,
    numeric_imports: Vec<KiCadSchematicNumericImport>,
}

impl<'a> BookImporter<'a> {
    fn new(roots: BTreeMap<String, Sexp>, circuit: &'a Circuit) -> Self {
        Self {
            roots,
            circuit,
            visiting: BTreeSet::new(),
            visited: BTreeSet::new(),
            claimed_sheets: BTreeSet::new(),
            sheets: Vec::new(),
            sheet_ports: Vec::new(),
            sheet_links: Vec::new(),
            files: Vec::new(),
            numeric_imports: Vec::new(),
        }
    }

    fn import(
        mut self,
        root_filename: &str,
    ) -> Result<KiCadSchematicBookImportReport, KiCadSchematicImportError> {
        if !self.circuit.validate().is_valid() {
            return Err(KiCadSchematicImportError::InvalidImportedLayout { issue_count: 1 });
        }
        self.discover(root_filename, None)?;

        let mut partials = Vec::with_capacity(self.files.len());
        for file in &self.files {
            let root = self
                .roots
                .get(&file.filename)
                .ok_or_else(|| KiCadSchematicImportError::MissingBookFile(file.filename.clone()))?;
            let excluded = self
                .sheet_ports
                .iter()
                .filter(|port| {
                    port.sheet == file.sheet
                        && self
                            .sheet_links
                            .iter()
                            .any(|link| link.child_port == port.id)
                })
                .map(|port| stable_uuid(&format!("sheet-label:{}", port.id.as_str())))
                .collect::<BTreeSet<_>>();
            let mut importer = SchematicImporter::new(root, self.circuit);
            let libraries = importer.library_symbols()?;
            let (symbol_definitions, symbols) = importer.import_symbols(&libraries)?;
            let ports = importer.import_ports_excluding(&excluded)?;
            let native_labels = importer.import_native_labels()?;
            let native_wires = root.named_children("wire").collect::<Vec<_>>();
            let synthetic = native_wires
                .iter()
                .filter_map(|wire| wire.named_child("uuid")?.atom_at(1))
                .map(|uuid| stable_uuid(&format!("synthetic-net-label:{uuid}")))
                .collect::<BTreeSet<_>>();
            let labels = native_labels
                .iter()
                .filter(|label| !synthetic.contains(&label.uuid))
                .map(|label| importer.semantic_label(label))
                .collect::<Result<Vec<_>, _>>()?;
            self.numeric_imports.extend(importer.numeric_imports);
            partials.push(PagePartial {
                filename: file.filename.clone(),
                sheet: file.sheet.clone(),
                symbol_definitions,
                symbols,
                ports,
                labels,
                native_labels,
            });
        }

        let mut layout = SchematicLayout {
            sheets: self.sheets,
            sheet_ports: self.sheet_ports,
            sheet_links: self.sheet_links,
            ..SchematicLayout::default()
        };
        for partial in &partials {
            for definition in &partial.symbol_definitions {
                if let Some(existing) = layout
                    .symbol_definitions
                    .iter()
                    .find(|candidate| candidate.id == definition.id)
                {
                    if existing != definition {
                        return Err(KiCadSchematicImportError::InvalidBookMetadata(format!(
                            "symbol definition {} differs between schematic pages",
                            definition.id.as_str()
                        )));
                    }
                } else {
                    layout.symbol_definitions.push(definition.clone());
                }
            }
            layout.symbols.extend(partial.symbols.iter().cloned());
            layout.ports.extend(partial.ports.iter().cloned());
            layout.labels.extend(partial.labels.iter().cloned());
        }

        for partial in partials {
            let root = self.roots.get(&partial.filename).ok_or_else(|| {
                KiCadSchematicImportError::MissingBookFile(partial.filename.clone())
            })?;
            let nodes = root.named_children("wire").collect::<Vec<_>>();
            let page_layout = SchematicLayout {
                symbol_definitions: partial.symbol_definitions.clone(),
                symbols: partial.symbols.clone(),
                ports: partial.ports.clone(),
                sheet_ports: layout
                    .sheet_ports
                    .iter()
                    .filter(|port| port.sheet == partial.sheet)
                    .cloned()
                    .collect(),
                ..SchematicLayout::default()
            };
            let mut importer = SchematicImporter::new(root, self.circuit);
            let wires = coalesce_native_wires(importer.import_wires(
                &page_layout,
                &partial.native_labels,
                &nodes,
            )?);
            self.numeric_imports.extend(importer.numeric_imports);
            let sheet = layout
                .sheets
                .iter_mut()
                .find(|sheet| sheet.id == partial.sheet)
                .ok_or_else(|| {
                    KiCadSchematicImportError::InvalidBookMetadata(format!(
                        "page {} disappeared during import",
                        partial.sheet.as_str()
                    ))
                })?;
            sheet.symbols = partial
                .symbols
                .iter()
                .map(|symbol| symbol.id.clone())
                .collect();
            sheet.ports = partial.ports.iter().map(|port| port.port.clone()).collect();
            sheet.labels = partial
                .labels
                .iter()
                .map(|label| label.id.clone())
                .collect();
            sheet.wires = wires.iter().map(|wire| wire.id.clone()).collect();
            layout.wires.extend(wires);
        }

        let validation = layout.validate(self.circuit);
        if !validation.is_valid() {
            return Err(KiCadSchematicImportError::InvalidImportedLayout {
                issue_count: validation.issues.len(),
            });
        }
        let mut omissions = Vec::new();
        if !layout.wires.is_empty() {
            omissions.push(KiCadSchematicImportOmission::GeneratedWireIdentities {
                count: layout.wires.len(),
            });
        }
        if !layout.labels.is_empty() {
            omissions.push(KiCadSchematicImportOmission::GeneratedLabelIdentities {
                count: layout.labels.len(),
            });
        }
        let unreferenced = self
            .roots
            .keys()
            .filter(|filename| !self.visited.contains(*filename))
            .cloned()
            .collect::<Vec<_>>();
        if !unreferenced.is_empty() {
            omissions.push(KiCadSchematicImportOmission::UnreferencedFiles {
                filenames: unreferenced,
            });
        }
        Ok(KiCadSchematicBookImportReport {
            layout,
            files: self.files,
            numeric_imports: self.numeric_imports,
            omissions,
        })
    }

    fn discover(
        &mut self,
        filename: &str,
        parent: Option<SchematicSheetId>,
    ) -> Result<(), KiCadSchematicImportError> {
        if self.visited.contains(filename) {
            return Ok(());
        }
        if !self.visiting.insert(filename.to_owned()) {
            return Err(KiCadSchematicImportError::BookFileCycle(filename.into()));
        }
        let root = self
            .roots
            .get(filename)
            .cloned()
            .ok_or_else(|| KiCadSchematicImportError::MissingBookFile(filename.into()))?;
        let (sheet, title, file_uuid) = file_sheet_metadata(&root, filename)?;
        if !self.claimed_sheets.insert(sheet.clone()) {
            return Err(KiCadSchematicImportError::DuplicateBookSheet(sheet));
        }
        let children = self.child_descriptors(&root, &sheet, filename)?;
        let current_sheet = sheet.clone();
        self.files.push(KiCadSchematicSheetImport {
            sheet: sheet.clone(),
            filename: filename.into(),
            file_uuid,
        });
        self.sheets.push(SchematicSheet {
            id: sheet,
            title,
            parent,
            symbols: Vec::new(),
            ports: Vec::new(),
            wires: Vec::new(),
            labels: Vec::new(),
        });
        for child in children {
            self.discover(&child.filename, Some(current_sheet.clone()))?;
        }
        self.visiting.remove(filename);
        self.visited.insert(filename.into());
        Ok(())
    }

    fn child_descriptors(
        &mut self,
        root: &Sexp,
        parent_sheet: &SchematicSheetId,
        parent_filename: &str,
    ) -> Result<Vec<ChildDescriptor>, KiCadSchematicImportError> {
        let mut children = Vec::new();
        for sheet_node in root.named_children("sheet") {
            let child_filename = property(sheet_node, "Sheetfile")
                .ok_or_else(|| {
                    KiCadSchematicImportError::InvalidBookMetadata(format!(
                        "{parent_filename}: sheet object has no Sheetfile"
                    ))
                })?
                .to_owned();
            let child_id_text = property(sheet_node, CHILD_SHEET_ID_PROPERTY).ok_or_else(|| {
                KiCadSchematicImportError::InvalidBookMetadata(format!(
                    "{parent_filename}: sheet object has no {CHILD_SHEET_ID_PROPERTY}"
                ))
            })?;
            let child_id = SchematicSheetId::new(child_id_text)
                .map_err(|_| KiCadSchematicImportError::InvalidIdentifier(child_id_text.into()))?;
            let child_root = self.roots.get(&child_filename).cloned().ok_or_else(|| {
                KiCadSchematicImportError::MissingBookFile(child_filename.clone())
            })?;
            let (claimed_child, _, _) = file_sheet_metadata(&child_root, &child_filename)?;
            if claimed_child != child_id {
                return Err(KiCadSchematicImportError::InvalidBookMetadata(format!(
                    "{parent_filename}: Sheetfile {child_filename:?} claims sheet {}, expected {}",
                    claimed_child.as_str(),
                    child_id.as_str()
                )));
            }
            let mut link_properties = sheet_node
                .named_children("property")
                .filter_map(|property| {
                    let name = property.atom_at(1)?;
                    let index = name
                        .strip_prefix(SHEET_LINK_PROPERTY_PREFIX)?
                        .parse()
                        .ok()?;
                    Some((index, property.atom_at(2).unwrap_or("").to_owned()))
                })
                .collect::<Vec<(usize, String)>>();
            link_properties.sort_by_key(|(index, _)| *index);
            let pins = sheet_node.named_children("pin").collect::<Vec<_>>();
            if link_properties.len() != pins.len() {
                return Err(KiCadSchematicImportError::InvalidBookMetadata(format!(
                    "{parent_filename}: child sheet {} has {} link records but {} pins",
                    child_id.as_str(),
                    link_properties.len(),
                    pins.len()
                )));
            }
            for ((_, encoded), pin_node) in link_properties.iter().zip(pins) {
                let fields = decode_fields(encoded)?;
                if fields.len() != 6 {
                    return Err(KiCadSchematicImportError::InvalidBookMetadata(format!(
                        "{parent_filename}: child sheet {} link record has {} fields",
                        child_id.as_str(),
                        fields.len()
                    )));
                }
                let link_id = SchematicSheetLinkId::new(&fields[0])
                    .map_err(|_| KiCadSchematicImportError::InvalidIdentifier(fields[0].clone()))?;
                let parent_port_id = SchematicSheetPortId::new(&fields[1])
                    .map_err(|_| KiCadSchematicImportError::InvalidIdentifier(fields[1].clone()))?;
                let child_port_id = SchematicSheetPortId::new(&fields[2])
                    .map_err(|_| KiCadSchematicImportError::InvalidIdentifier(fields[2].clone()))?;
                let net = NetId::new(&fields[5])
                    .map_err(|_| KiCadSchematicImportError::InvalidIdentifier(fields[5].clone()))?;
                if !self
                    .circuit
                    .nets
                    .iter()
                    .any(|candidate| candidate.id == net)
                {
                    return Err(KiCadSchematicImportError::InvalidIdentifier(
                        fields[5].clone(),
                    ));
                }
                let parent_position = book_at(
                    pin_node,
                    &format!(
                        "{parent_filename}.sheet[{}].pin[{}]",
                        child_id.as_str(),
                        parent_port_id.as_str()
                    ),
                    &mut self.numeric_imports,
                )?;
                let expected_label_uuid =
                    stable_uuid(&format!("sheet-label:{}", child_port_id.as_str()));
                let child_label = child_root
                    .named_children("hierarchical_label")
                    .find(|label| {
                        label.named_child("uuid").and_then(|uuid| uuid.atom_at(1))
                            == Some(expected_label_uuid.as_str())
                    })
                    .ok_or_else(|| {
                        KiCadSchematicImportError::InvalidBookMetadata(format!(
                            "{child_filename}: missing hierarchical label for port {}",
                            child_port_id.as_str()
                        ))
                    })?;
                let child_position = book_at(
                    child_label,
                    &format!("{child_filename}.sheet_port[{}]", child_port_id.as_str()),
                    &mut self.numeric_imports,
                )?;
                self.sheet_ports.push(SchematicSheetPort {
                    id: parent_port_id.clone(),
                    sheet: parent_sheet.clone(),
                    net: net.clone(),
                    name: fields[3].clone(),
                    position: parent_position,
                });
                self.sheet_ports.push(SchematicSheetPort {
                    id: child_port_id.clone(),
                    sheet: child_id.clone(),
                    net,
                    name: fields[4].clone(),
                    position: child_position,
                });
                self.sheet_links.push(SchematicSheetLink {
                    id: link_id,
                    parent_port: parent_port_id,
                    child_port: child_port_id,
                });
            }
            children.push(ChildDescriptor {
                filename: child_filename,
            });
        }
        Ok(children)
    }
}

fn endpoint_for(
    layout: &SchematicLayout,
    circuit: &Circuit,
    point: &SchematicPoint,
) -> Option<(SchematicEndpoint, NetId)> {
    for symbol in &layout.symbols {
        let instance = circuit
            .instances
            .iter()
            .find(|candidate| candidate.id == symbol.instance)?;
        let unit = layout.symbol_unit(symbol)?;
        for pin in &unit.pins {
            if endpoint_pin_point(symbol, pin) == *point {
                let net = instance
                    .pins
                    .iter()
                    .find(|binding| binding.pin == pin.pin)?
                    .net
                    .clone();
                return Some((
                    SchematicEndpoint::Pin {
                        symbol: symbol.id.clone(),
                        pin: pin.pin.clone(),
                    },
                    net,
                ));
            }
        }
    }
    for port in &layout.ports {
        if port.position == *point {
            let net = circuit
                .ports
                .iter()
                .find(|candidate| candidate.id == port.port)?
                .net
                .clone();
            return Some((SchematicEndpoint::Port(port.port.clone()), net));
        }
    }
    for port in &layout.sheet_ports {
        if port.position == *point {
            return Some((
                SchematicEndpoint::SheetPort(port.id.clone()),
                port.net.clone(),
            ));
        }
    }
    None
}

fn ordered_sheets(sheets: &[SchematicSheet]) -> Vec<&SchematicSheet> {
    let mut children = BTreeMap::<Option<SchematicSheetId>, Vec<&SchematicSheet>>::new();
    for sheet in sheets {
        children
            .entry(sheet.parent.clone())
            .or_default()
            .push(sheet);
    }
    for siblings in children.values_mut() {
        siblings.sort_by(|left, right| left.id.cmp(&right.id));
    }
    let mut ordered = Vec::with_capacity(sheets.len());
    let mut stack = children.remove(&None).unwrap_or_default();
    stack.reverse();
    while let Some(sheet) = stack.pop() {
        ordered.push(sheet);
        if let Some(mut descendants) = children.remove(&Some(sheet.id.clone())) {
            descendants.reverse();
            stack.extend(descendants);
        }
    }
    ordered
}

fn generated_sheet_box(
    points: &[SchematicPoint],
) -> Result<(SchematicPoint, Real, Real), KiCadSchematicExportError> {
    if points.is_empty() {
        return Ok((
            SchematicPoint::new(Real::from(20), Real::from(20)),
            Real::from(40),
            Real::from(20),
        ));
    }
    let mut min_x = points[0].x.clone();
    let mut max_x = points[0].x.clone();
    let mut min_y = points[0].y.clone();
    let mut max_y = points[0].y.clone();
    for point in &points[1..] {
        if less_than(&point.x, &min_x) {
            min_x = point.x.clone();
        }
        if less_than(&max_x, &point.x) {
            max_x = point.x.clone();
        }
        if less_than(&point.y, &min_y) {
            min_y = point.y.clone();
        }
        if less_than(&max_y, &point.y) {
            max_y = point.y.clone();
        }
    }
    let vertical_margin =
        (Real::from(127) / Real::from(50)).map_err(|_| KiCadSchematicExportError::InvalidDesign)?;
    let origin = SchematicPoint::new(
        min_x.clone() - Real::from(20),
        min_y.clone() - vertical_margin.clone(),
    );
    let width = max_x - min_x + Real::from(20);
    let height = max_y - min_y + vertical_margin.clone() + vertical_margin;
    Ok((origin, width, height))
}

fn less_than(left: &Real, right: &Real) -> bool {
    (left.clone() - right.clone()).refine_sign_until(-128) == Some(RealSign::Negative)
}

fn valid_kicad_filename(filename: &str) -> bool {
    !filename.is_empty()
        && filename.ends_with(".kicad_sch")
        && !filename.contains('/')
        && !filename.contains('\\')
        && filename != ".kicad_sch"
}

fn filename_slug(value: &str) -> String {
    let mut output = String::new();
    for character in value.chars() {
        if character.is_ascii_alphanumeric() || character == '-' || character == '_' {
            output.push(character.to_ascii_lowercase());
        } else if !output.ends_with('-') {
            output.push('-');
        }
    }
    let output = output.trim_matches('-');
    if output.is_empty() {
        "sheet".into()
    } else {
        output.into()
    }
}

fn encode_fields(fields: &[&str]) -> String {
    let mut output = String::new();
    for field in fields {
        write!(output, "{}:{field}", field.len()).expect("writing a String cannot fail");
    }
    output
}

fn decode_fields(encoded: &str) -> Result<Vec<String>, KiCadSchematicImportError> {
    let mut remaining = encoded;
    let mut fields = Vec::new();
    while !remaining.is_empty() {
        let colon = remaining.find(':').ok_or_else(|| {
            KiCadSchematicImportError::InvalidBookMetadata(
                "length-prefixed link field has no ':'".into(),
            )
        })?;
        let length = remaining[..colon].parse::<usize>().map_err(|_| {
            KiCadSchematicImportError::InvalidBookMetadata(
                "length-prefixed link field has invalid length".into(),
            )
        })?;
        remaining = &remaining[colon + 1..];
        let value = remaining.get(..length).ok_or_else(|| {
            KiCadSchematicImportError::InvalidBookMetadata(
                "length-prefixed link field is truncated".into(),
            )
        })?;
        if !remaining.is_char_boundary(length) {
            return Err(KiCadSchematicImportError::InvalidBookMetadata(
                "length-prefixed link field splits UTF-8".into(),
            ));
        }
        fields.push(value.to_owned());
        remaining = &remaining[length..];
    }
    Ok(fields)
}

fn file_sheet_metadata(
    root: &Sexp,
    filename: &str,
) -> Result<(SchematicSheetId, String, String), KiCadSchematicImportError> {
    let title_block = root.named_child("title_block").ok_or_else(|| {
        KiCadSchematicImportError::InvalidBookMetadata(format!("{filename}: missing title_block"))
    })?;
    let title = title_block
        .named_child("title")
        .and_then(|title| title.atom_at(1))
        .ok_or_else(|| {
            KiCadSchematicImportError::InvalidBookMetadata(format!(
                "{filename}: title_block has no title"
            ))
        })?
        .to_owned();
    let marker = title_block
        .named_children("comment")
        .find(|comment| comment.atom_at(1) == Some("1"))
        .and_then(|comment| comment.atom_at(2))
        .ok_or_else(|| {
            KiCadSchematicImportError::InvalidBookMetadata(format!(
                "{filename}: title_block has no sheet identity comment"
            ))
        })?;
    let id = marker
        .strip_prefix(SHEET_ID_COMMENT_PREFIX)
        .ok_or_else(|| {
            KiCadSchematicImportError::InvalidBookMetadata(format!(
                "{filename}: invalid sheet identity comment"
            ))
        })?;
    let sheet = SchematicSheetId::new(id)
        .map_err(|_| KiCadSchematicImportError::InvalidIdentifier(id.into()))?;
    let file_uuid = required_child_atom(root, "uuid", 1)?.to_owned();
    Ok((sheet, title, file_uuid))
}

fn book_at(
    node: &Sexp,
    field: &str,
    imports: &mut Vec<KiCadSchematicNumericImport>,
) -> Result<SchematicPoint, KiCadSchematicImportError> {
    let at = node
        .named_child("at")
        .ok_or_else(|| KiCadSchematicImportError::Parse(format!("missing {field}.at")))?;
    let parse = |token: Option<&str>,
                 axis: &str,
                 imports: &mut Vec<KiCadSchematicNumericImport>|
     -> Result<Real, KiCadSchematicImportError> {
        let token = token
            .ok_or_else(|| KiCadSchematicImportError::Parse(format!("missing {field}.{axis}")))?;
        let value =
            Real::from_str(token).map_err(|_| KiCadSchematicImportError::InvalidNumber {
                field: format!("{field}.{axis}"),
                token: token.into(),
            })?;
        imports.push(KiCadSchematicNumericImport {
            field: format!("{field}.{axis}"),
            token: token.into(),
        });
        Ok(value)
    };
    Ok(SchematicPoint::new(
        parse(at.atom_at(1), "x", imports)?,
        parse(at.atom_at(2), "y", imports)?,
    ))
}

fn simplify_polyline(points: Vec<SchematicPoint>) -> Vec<SchematicPoint> {
    let mut result = Vec::with_capacity(points.len());
    for point in points {
        if result.last() == Some(&point) {
            continue;
        }
        while result.len() >= 2 {
            let first = &result[result.len() - 2];
            let middle = &result[result.len() - 1];
            let first_dx = middle.x.clone() - first.x.clone();
            let first_dy = middle.y.clone() - first.y.clone();
            let second_dx = point.x.clone() - middle.x.clone();
            let second_dy = point.y.clone() - middle.y.clone();
            let cross = first_dx.clone() * second_dy.clone() - first_dy.clone() * second_dx.clone();
            let dot = first_dx * second_dx + first_dy * second_dy;
            if cross.refine_sign_until(-128) == Some(RealSign::Zero)
                && dot.refine_sign_until(-128) != Some(RealSign::Negative)
            {
                result.pop();
            } else {
                break;
            }
        }
        result.push(point);
    }
    result
}

fn coalesce_native_wires(mut wires: Vec<SchematicWire>) -> Vec<SchematicWire> {
    loop {
        let mut junctions = BTreeMap::<(NetId, String, String), Vec<(usize, bool)>>::new();
        for (index, wire) in wires.iter().enumerate() {
            for (at_start, endpoint) in [(true, &wire.from), (false, &wire.to)] {
                if let SchematicEndpoint::Junction(point) = endpoint {
                    junctions
                        .entry((wire.net.clone(), point.x.to_string(), point.y.to_string()))
                        .or_default()
                        .push((index, at_start));
                }
            }
        }
        let Some((_, pair)) = junctions.into_iter().find(|(_, occurrences)| {
            occurrences.len() == 2 && occurrences[0].0 != occurrences[1].0
        }) else {
            break;
        };
        let (first_index, first_at_start) = pair[0];
        let (second_index, second_at_start) = pair[1];
        let (low, high) = if first_index < second_index {
            (first_index, second_index)
        } else {
            (second_index, first_index)
        };
        let second = wires.remove(high);
        let first = wires.remove(low);
        let low_at_start = if first_index == low {
            first_at_start
        } else {
            second_at_start
        };
        let high_at_start = if second_index == high {
            second_at_start
        } else {
            first_at_start
        };
        let first = orient_wire(first, low_at_start);
        let second = orient_wire(second, !high_at_start);
        let shared = match &first.to {
            SchematicEndpoint::Junction(point) => point.clone(),
            _ => unreachable!("selected first wire ends at junction"),
        };
        let mut waypoints = first.waypoints;
        waypoints.push(shared);
        waypoints.extend(second.waypoints);
        wires.insert(
            low,
            SchematicWire {
                id: first.id,
                net: first.net,
                from: first.from,
                waypoints,
                to: second.to,
            },
        );
    }
    wires
}

fn orient_wire(mut wire: SchematicWire, reverse: bool) -> SchematicWire {
    if reverse {
        std::mem::swap(&mut wire.from, &mut wire.to);
        wire.waypoints.reverse();
    }
    wire
}

fn endpoint_point(
    layout: &SchematicLayout,
    endpoint: &SchematicEndpoint,
) -> Option<SchematicPoint> {
    match endpoint {
        SchematicEndpoint::Pin { symbol, pin } => {
            let symbol = layout
                .symbols
                .iter()
                .find(|candidate| &candidate.id == symbol)?;
            let pin = layout
                .symbol_unit(symbol)?
                .pins
                .iter()
                .find(|candidate| &candidate.pin == pin)?;
            Some(endpoint_pin_point(symbol, pin))
        }
        SchematicEndpoint::Port(port) => layout
            .ports
            .iter()
            .find(|candidate| &candidate.port == port)
            .map(|placement| placement.position.clone()),
        SchematicEndpoint::SheetPort(port) => layout
            .sheet_ports
            .iter()
            .find(|candidate| &candidate.id == port)
            .map(|port| port.position.clone()),
        SchematicEndpoint::Junction(point) => Some(point.clone()),
    }
}

fn endpoint_pin_point(symbol: &SchematicSymbol, pin: &SchematicPinPlacement) -> SchematicPoint {
    let (x, y) = match symbol.quarter_turns.rem_euclid(4) {
        0 => (pin.position.x.clone(), pin.position.y.clone()),
        1 => (-pin.position.y.clone(), pin.position.x.clone()),
        2 => (-pin.position.x.clone(), -pin.position.y.clone()),
        _ => (pin.position.y.clone(), -pin.position.x.clone()),
    };
    SchematicPoint::new(x + symbol.position.x.clone(), y + symbol.position.y.clone())
}

fn property<'a>(node: &'a Sexp, name: &str) -> Option<&'a str> {
    node.named_children("property")
        .find(|property| property.atom_at(1) == Some(name))
        .and_then(|property| property.atom_at(2))
}

fn required_child_atom<'a>(
    node: &'a Sexp,
    name: &str,
    index: usize,
) -> Result<&'a str, KiCadSchematicImportError> {
    node.named_child(name)
        .and_then(|child| child.atom_at(index))
        .ok_or_else(|| KiCadSchematicImportError::Parse(format!("missing {name}")))
}

fn pin_lead_length(unit: &SchematicSymbolUnit, pin: &SchematicPinPlacement) -> Real {
    let half_width = divide_two(&unit.body_width).unwrap_or_else(|_| Real::from(0));
    let half_height = divide_two(&unit.body_height).unwrap_or_else(|_| Real::from(0));
    let candidate = match pin.side {
        SchematicPinSide::Left => -half_width - pin.position.x.clone(),
        SchematicPinSide::Right => pin.position.x.clone() - half_width,
        SchematicPinSide::Top => -half_height - pin.position.y.clone(),
        SchematicPinSide::Bottom => pin.position.y.clone() - half_height,
    };
    if candidate.refine_sign_until(-128) == Some(RealSign::Positive) {
        candidate
    } else {
        Real::from(0)
    }
}

fn library_id(id: &SchematicSymbolDefinitionId) -> String {
    format!("hypercircuit:{}", kicad_symbol_name(id))
}

fn kicad_symbol_name(id: &SchematicSymbolDefinitionId) -> String {
    format!("hc_{}", stable_uuid(id.as_str()).replace('-', "_"))
}

fn unit_body_property(unit: u16, dimension: &str) -> String {
    format!("HyperCircuit Unit {unit} Body {dimension}")
}

fn graphic_kind(graphic: &SchematicGraphic) -> &'static str {
    match graphic {
        SchematicGraphic::Line { .. } => "line",
        SchematicGraphic::Rectangle { .. } => "rectangle",
        SchematicGraphic::Circle { .. } => "circle",
        SchematicGraphic::Arc { .. } => "arc",
        SchematicGraphic::Polyline { .. } => "polyline",
        SchematicGraphic::Text { .. } => "text",
    }
}

fn library_symbol_unit(node: &Sexp) -> Option<u16> {
    let name = node.atom_at(1)?;
    let mut fields = name.rsplit('_');
    let style = fields.next()?;
    let unit = fields.next()?;
    (style == "1").then(|| unit.parse().ok()).flatten()
}

fn divide_two(value: &Real) -> Result<Real, KiCadSchematicExportError> {
    (value.clone() / Real::from(2)).map_err(|_| KiCadSchematicExportError::InvalidDesign)
}

fn pin_angle(side: SchematicPinSide) -> i32 {
    match side {
        SchematicPinSide::Left => 0,
        SchematicPinSide::Right => 180,
        SchematicPinSide::Top => 90,
        SchematicPinSide::Bottom => 270,
    }
}

fn kicad_pin_kind(kind: PinElectricalKind) -> &'static str {
    match kind {
        PinElectricalKind::Input => "input",
        PinElectricalKind::Output => "output",
        PinElectricalKind::Bidirectional => "bidirectional",
        PinElectricalKind::Passive => "passive",
        PinElectricalKind::PowerInput => "power_in",
        PinElectricalKind::PowerOutput => "power_out",
        PinElectricalKind::OpenCollector => "open_collector",
        PinElectricalKind::OpenEmitter => "open_emitter",
        PinElectricalKind::NotConnected => "no_connect",
    }
}

fn port_shape(direction: PortDirection) -> &'static str {
    match direction {
        PortDirection::Input | PortDirection::PowerInput | PortDirection::Ground => "input",
        PortDirection::Output | PortDirection::PowerOutput => "output",
        PortDirection::Bidirectional => "bidirectional",
        PortDirection::Passive => "passive",
    }
}

fn quoted(value: &str) -> String {
    let mut output = String::with_capacity(value.len() + 2);
    output.push('"');
    for character in value.chars() {
        match character {
            '\\' => output.push_str("\\\\"),
            '"' => output.push_str("\\\""),
            '\n' => output.push_str("\\n"),
            '\r' => output.push_str("\\r"),
            '\t' => output.push_str("\\t"),
            other => output.push(other),
        }
    }
    output.push('"');
    output
}

fn stable_uuid(value: &str) -> String {
    fn hash(seed: u64, bytes: &[u8]) -> u64 {
        let mut value = seed;
        for byte in bytes {
            value ^= u64::from(*byte);
            value = value.wrapping_mul(0x100000001b3);
        }
        value
    }
    let first = hash(0xcbf29ce484222325, value.as_bytes());
    let second = hash(0x84222325cbf29ce4, value.as_bytes());
    let mut bytes = [0u8; 16];
    bytes[..8].copy_from_slice(&first.to_be_bytes());
    bytes[8..].copy_from_slice(&second.to_be_bytes());
    bytes[6] = (bytes[6] & 0x0f) | 0x50;
    bytes[8] = (bytes[8] & 0x3f) | 0x80;
    format!(
        "{:02x}{:02x}{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}",
        bytes[0],
        bytes[1],
        bytes[2],
        bytes[3],
        bytes[4],
        bytes[5],
        bytes[6],
        bytes[7],
        bytes[8],
        bytes[9],
        bytes[10],
        bytes[11],
        bytes[12],
        bytes[13],
        bytes[14],
        bytes[15]
    )
}

fn formatting(_: std::fmt::Error) -> KiCadSchematicExportError {
    KiCadSchematicExportError::Formatting
}
