//! Standalone KiCad symbol/footprint library import into portable part definitions.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::{Display, Formatter};
use std::path::Path;
use std::str::FromStr;

use hyperlattice::Point2;
use hyperpath::TraceLayer;
use hyperreal::Real;

use crate::sexp::{self, Sexp};
use crate::{
    DeviceModel, DeviceModelId, DeviceModelKind, DevicePin, DrillShape, LandPattern,
    LandPatternGraphic, LandPatternGraphicId, LandPatternGraphicPrimitive, LandPatternId,
    LandPatternPad, LayerRole, PackageResolutionError, PadId, PadPinMap, PadShape, PartRef,
    Pcb3dModelReference, Pcb3dModelTransform, PinElectricalKind, PinRef, Plating,
    PortablePartDefinition, SchematicGraphic, SchematicGraphicFill, SchematicPinPlacement,
    SchematicPinSide, SchematicPoint, SchematicSymbolDefinition, SchematicSymbolDefinitionId,
    SchematicSymbolUnit,
};

/// Explicit identities and projection policy for one native library import.
#[derive(Clone, Debug, PartialEq)]
pub struct KiCadPartLibraryImportOptions {
    /// Package-local portable export name.
    pub export_name: String,
    /// Optional external procurement/library identity.
    pub part: Option<PartRef>,
    /// Stable retained electrical-model identity.
    pub model_id: DeviceModelId,
    /// Executable or custom model family assigned to imported pins.
    pub model_kind: DeviceModelKind,
    /// Stable retained symbol-definition identity.
    pub symbol_definition_id: SchematicSymbolDefinitionId,
    /// Stable retained land-pattern identity.
    pub land_pattern_id: LandPatternId,
    /// Exact positive stroke substituted for native zero/default symbol strokes.
    pub default_symbol_stroke_width: Real,
    /// Exact positive fallback for a symbol body with no extent on one axis.
    pub minimum_symbol_body_extent: Real,
    /// Native copper-layer name to retained routing-layer map.
    pub copper_layers: BTreeMap<String, TraceLayer>,
}

impl KiCadPartLibraryImportOptions {
    /// Creates a conventional front/back library import policy.
    pub fn new(
        export_name: impl Into<String>,
        model_id: DeviceModelId,
        model_kind: DeviceModelKind,
        symbol_definition_id: SchematicSymbolDefinitionId,
        land_pattern_id: LandPatternId,
    ) -> Self {
        Self {
            export_name: export_name.into(),
            part: None,
            model_id,
            model_kind,
            symbol_definition_id,
            land_pattern_id,
            default_symbol_stroke_width: (Real::one() / Real::from(4)).expect("four is nonzero"),
            minimum_symbol_body_extent: Real::one(),
            copper_layers: BTreeMap::from([
                ("F.Cu".into(), TraceLayer(0)),
                ("B.Cu".into(), TraceLayer(1)),
            ]),
        }
    }

    /// Attaches a stable external parts-library identity.
    pub fn part_ref(mut self, part: PartRef) -> Self {
        self.part = Some(part);
        self
    }

    /// Overrides the exact default symbol stroke.
    pub fn default_symbol_stroke_width(mut self, width: Real) -> Self {
        self.default_symbol_stroke_width = width;
        self
    }

    /// Overrides the exact minimum nominal body extent.
    pub fn minimum_symbol_body_extent(mut self, extent: Real) -> Self {
        self.minimum_symbol_body_extent = extent;
        self
    }

    /// Maps one canonical native copper-layer name.
    pub fn copper_layer(mut self, name: impl Into<String>, layer: TraceLayer) -> Self {
        self.copper_layers.insert(name.into(), layer);
        self
    }
}

/// One exact native decimal retained during standalone library import.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct KiCadLibraryNumericImport {
    /// Semantic destination field.
    pub field: String,
    /// Native decimal token.
    pub source: String,
    /// Exact retained value.
    pub exact: String,
}

/// Native library intent that required projection or remains unsupported.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum KiCadLibraryImportOmission {
    /// Symbol inheritance was resolved into a standalone retained definition.
    SymbolInheritanceFlattened { child: String, parent: String },
    /// Alternate De Morgan/body styles were not selected.
    AlternateSymbolBodyStyle { unit: u16, style: u16 },
    /// A common unit-zero graphic was copied into every concrete unit.
    CommonSymbolGraphicsReplicated { units: usize, graphics: usize },
    /// A native symbol graphic is outside the retained primitive subset.
    UnsupportedSymbolGraphic { unit: u16, primitive: String },
    /// Native zero/default stroke used the caller's explicit exact fallback.
    DefaultSymbolStroke { unit: u16, graphic: usize },
    /// A zero-axis symbol extent used the caller's explicit exact fallback.
    MinimumSymbolBodyExtent { unit: u16, axis: String },
    /// One pin number used conflicting native electrical kinds across units.
    ConflictingPinElectricalKind { pin: String },
    /// Pin graphical shape is presentation not represented by retained ERC kind.
    PinGraphicShape { pin: String, shape: String },
    /// A native pad shape was conservatively represented by its size box.
    UnsupportedPadShape { pad: String, shape: String },
    /// A native copper layer had no caller-supplied retained mapping.
    UnsupportedCopperLayer { pad: String, layer: String },
    /// Native paste-ratio policy has no per-pad retained equivalent.
    PadPasteRatio { pad: String },
    /// A footprint graphic is outside the retained primitive subset.
    UnsupportedFootprintGraphic { primitive: String, layer: String },
    /// A native layer was retained as a named custom artwork role.
    CustomArtworkLayer { layer: String },
}

/// Portable part plus exact-import and loss evidence.
#[derive(Clone, Debug, PartialEq)]
pub struct KiCadPartLibraryImportReport {
    /// Unified retained part ready for `PartLibraryArtifact` or `Design::import_part`.
    pub part: PortablePartDefinition,
    /// Native symbol properties such as reference, value, footprint, and description.
    pub symbol_properties: BTreeMap<String, String>,
    /// Exact decimal audit in deterministic parse order.
    pub numeric_imports: Vec<KiCadLibraryNumericImport>,
    /// Every flattened, projected, or unsupported source fact.
    pub omissions: Vec<KiCadLibraryImportOmission>,
}

/// Failure to import a standalone KiCad library pair.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum KiCadLibraryImportError {
    /// Native S-expression syntax or structure was malformed.
    Parse(String),
    /// Requested top-level symbol was absent.
    MissingSymbol(String),
    /// Symbol inheritance references an absent or cyclic parent.
    InvalidSymbolInheritance(String),
    /// A required decimal was absent or invalid.
    InvalidNumber { field: String, token: String },
    /// A native identity could not become a stable retained id.
    InvalidIdentifier(String),
    /// Pin direction was not axis-aligned.
    UnsupportedPinOrientation { pin: String, degrees: String },
    /// Footprint exposes a numbered pad absent from the symbol.
    FootprintPinAbsentFromSymbol(String),
    /// Caller projection policy was nonpositive or incomplete.
    InvalidOptions(String),
    /// Resulting portable part failed authoritative validation.
    InvalidPortablePart(String),
    /// Source file could not be read.
    Io(String),
}

impl Display for KiCadLibraryImportError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Parse(message) => write!(formatter, "KiCad library parse error: {message}"),
            Self::MissingSymbol(symbol) => write!(formatter, "KiCad symbol {symbol} is absent"),
            Self::InvalidSymbolInheritance(symbol) => {
                write!(formatter, "invalid KiCad symbol inheritance for {symbol}")
            }
            Self::InvalidNumber { field, token } => {
                write!(formatter, "invalid KiCad number {token:?} for {field}")
            }
            Self::InvalidIdentifier(value) => {
                write!(formatter, "invalid imported KiCad identifier {value:?}")
            }
            Self::UnsupportedPinOrientation { pin, degrees } => write!(
                formatter,
                "KiCad pin {pin} has unsupported orientation {degrees}"
            ),
            Self::FootprintPinAbsentFromSymbol(pin) => {
                write!(formatter, "KiCad footprint pad {pin} has no symbol pin")
            }
            Self::InvalidOptions(message) => write!(formatter, "invalid import options: {message}"),
            Self::InvalidPortablePart(message) => {
                write!(formatter, "invalid imported portable part: {message}")
            }
            Self::Io(message) => write!(formatter, "failed to read KiCad library: {message}"),
        }
    }
}

impl std::error::Error for KiCadLibraryImportError {}

impl KiCadPartLibraryImportReport {
    /// Imports one named symbol and one standalone footprint from native text.
    pub fn from_str(
        symbol_library: &str,
        symbol_name: &str,
        footprint: &str,
        options: KiCadPartLibraryImportOptions,
    ) -> Result<Self, KiCadLibraryImportError> {
        validate_options(&options)?;
        let symbol_root = sexp::parse(symbol_library).map_err(KiCadLibraryImportError::Parse)?;
        if symbol_root.list_name() != Some("kicad_symbol_lib") {
            return Err(KiCadLibraryImportError::Parse(
                "expected kicad_symbol_lib root".into(),
            ));
        }
        let footprint_root = sexp::parse(footprint).map_err(KiCadLibraryImportError::Parse)?;
        if !matches!(footprint_root.list_name(), Some("footprint" | "module")) {
            return Err(KiCadLibraryImportError::Parse(
                "expected footprint root".into(),
            ));
        }
        let mut importer = LibraryImporter {
            options,
            numeric_imports: Vec::new(),
            omissions: Vec::new(),
        };
        let (model, symbol, properties) = importer.import_symbol(&symbol_root, symbol_name)?;
        let land_pattern = importer.import_footprint(&footprint_root, &model)?;
        let part = PortablePartDefinition {
            name: importer.options.export_name.clone(),
            part: importer.options.part.clone(),
            model,
            symbol: Some(symbol),
            land_pattern: Some(land_pattern),
        };
        part.validate().map_err(|error| {
            KiCadLibraryImportError::InvalidPortablePart(package_error_text(error))
        })?;
        Ok(Self {
            part,
            symbol_properties: properties,
            numeric_imports: importer.numeric_imports,
            omissions: importer.omissions,
        })
    }

    /// Reads and imports one named symbol plus one `.kicad_mod` file.
    pub fn from_paths(
        symbol_library: impl AsRef<Path>,
        symbol_name: &str,
        footprint: impl AsRef<Path>,
        options: KiCadPartLibraryImportOptions,
    ) -> Result<Self, KiCadLibraryImportError> {
        let symbols = std::fs::read_to_string(symbol_library)
            .map_err(|error| KiCadLibraryImportError::Io(error.to_string()))?;
        let footprint = std::fs::read_to_string(footprint)
            .map_err(|error| KiCadLibraryImportError::Io(error.to_string()))?;
        Self::from_str(&symbols, symbol_name, &footprint, options)
    }
}

struct LibraryImporter {
    options: KiCadPartLibraryImportOptions,
    numeric_imports: Vec<KiCadLibraryNumericImport>,
    omissions: Vec<KiCadLibraryImportOmission>,
}

impl LibraryImporter {
    fn import_symbol(
        &mut self,
        root: &Sexp,
        symbol_name: &str,
    ) -> Result<
        (
            DeviceModel,
            SchematicSymbolDefinition,
            BTreeMap<String, String>,
        ),
        KiCadLibraryImportError,
    > {
        let symbols = root
            .named_children("symbol")
            .filter_map(|symbol| Some((symbol.atom_at(1)?.to_owned(), symbol)))
            .collect::<BTreeMap<_, _>>();
        let requested = symbols
            .get(symbol_name)
            .copied()
            .ok_or_else(|| KiCadLibraryImportError::MissingSymbol(symbol_name.into()))?;
        let mut chain = Vec::new();
        let mut current_name = symbol_name.to_owned();
        let mut seen = BTreeSet::new();
        loop {
            if !seen.insert(current_name.clone()) {
                return Err(KiCadLibraryImportError::InvalidSymbolInheritance(
                    symbol_name.into(),
                ));
            }
            let current = symbols.get(&current_name).copied().ok_or_else(|| {
                KiCadLibraryImportError::InvalidSymbolInheritance(symbol_name.into())
            })?;
            chain.push((current_name.clone(), current));
            let Some(parent) = current
                .named_child("extends")
                .and_then(|extends| extends.atom_at(1))
            else {
                break;
            };
            self.omissions
                .push(KiCadLibraryImportOmission::SymbolInheritanceFlattened {
                    child: current_name,
                    parent: parent.into(),
                });
            current_name = parent.into();
        }
        chain.reverse();
        let mut properties = BTreeMap::new();
        for (_, symbol) in &chain {
            for property in symbol.named_children("property") {
                if let (Some(name), Some(value)) = (property.atom_at(1), property.atom_at(2)) {
                    properties.insert(name.into(), value.into());
                }
            }
        }
        let base = chain
            .iter()
            .map(|(_, symbol)| *symbol)
            .find(|symbol| symbol.named_children("symbol").next().is_some())
            .unwrap_or(requested);
        let nested = base.named_children("symbol").collect::<Vec<_>>();
        let mut unit_numbers = nested
            .iter()
            .filter_map(|node| unit_identity(node))
            .filter_map(|(unit, style)| {
                if style > 1 {
                    self.omissions
                        .push(KiCadLibraryImportOmission::AlternateSymbolBodyStyle { unit, style });
                    None
                } else {
                    (unit != 0).then_some(unit)
                }
            })
            .collect::<BTreeSet<_>>();
        if unit_numbers.is_empty() {
            unit_numbers.insert(1);
        }
        let common_nodes = nested
            .iter()
            .copied()
            .filter(|node| unit_identity(node).is_some_and(|(unit, style)| unit == 0 && style <= 1))
            .collect::<Vec<_>>();
        let common_graphic_count = common_nodes
            .iter()
            .map(|node| symbol_graphic_nodes(node).count())
            .sum();
        if !common_nodes.is_empty() && unit_numbers.len() > 1 && common_graphic_count > 0 {
            self.omissions
                .push(KiCadLibraryImportOmission::CommonSymbolGraphicsReplicated {
                    units: unit_numbers.len(),
                    graphics: common_graphic_count,
                });
        }
        let mut pin_kinds = BTreeMap::<PinRef, PinElectricalKind>::new();
        let mut units = Vec::new();
        for unit_number in unit_numbers {
            let mut sources = vec![base];
            sources.extend(common_nodes.clone());
            sources.extend(nested.iter().copied().filter(|node| {
                unit_identity(node).is_some_and(|(unit, style)| unit == unit_number && style <= 1)
            }));
            let mut pins = Vec::new();
            let mut seen_pins = BTreeSet::new();
            let mut graphics = Vec::new();
            for source in sources {
                for pin in source.named_children("pin") {
                    let (placement, kind) = self.symbol_pin(pin, unit_number)?;
                    if seen_pins.insert(placement.pin.clone()) {
                        match pin_kinds.get(&placement.pin) {
                            Some(existing) if *existing != kind => {
                                self.omissions.push(
                                    KiCadLibraryImportOmission::ConflictingPinElectricalKind {
                                        pin: placement.pin.as_str().into(),
                                    },
                                );
                                pin_kinds.insert(
                                    placement.pin.clone(),
                                    PinElectricalKind::Bidirectional,
                                );
                            }
                            None => {
                                pin_kinds.insert(placement.pin.clone(), kind);
                            }
                            _ => {}
                        }
                        pins.push(placement);
                    }
                }
                for node in symbol_graphic_nodes(source) {
                    let index = graphics.len();
                    if let Some(graphic) = self.symbol_graphic(node, unit_number, index)? {
                        graphics.push(graphic);
                    }
                }
            }
            let (body_width, body_height) = self.symbol_extents(unit_number, &pins, &graphics)?;
            units.push(SchematicSymbolUnit {
                unit: unit_number,
                body_width,
                body_height,
                pins,
                graphics,
            });
        }
        let pins = pin_kinds
            .into_iter()
            .map(|(pin, kind)| DevicePin {
                optional: kind == PinElectricalKind::NotConnected,
                pin,
                kind,
            })
            .collect();
        Ok((
            DeviceModel {
                id: self.options.model_id.clone(),
                kind: self.options.model_kind.clone(),
                pins,
                parameters: Vec::new(),
            },
            SchematicSymbolDefinition {
                id: self.options.symbol_definition_id.clone(),
                model: self.options.model_id.clone(),
                name: properties
                    .get("Value")
                    .cloned()
                    .unwrap_or_else(|| symbol_name.into()),
                units,
            },
            properties,
        ))
    }

    fn symbol_pin(
        &mut self,
        pin: &Sexp,
        unit: u16,
    ) -> Result<(SchematicPinPlacement, PinElectricalKind), KiCadLibraryImportError> {
        let number = pin
            .named_child("number")
            .and_then(|number| number.atom_at(1))
            .ok_or_else(|| KiCadLibraryImportError::Parse("symbol pin has no number".into()))?;
        let pin_ref = PinRef::new(number)
            .map_err(|_| KiCadLibraryImportError::InvalidIdentifier(number.into()))?;
        let position = self.schematic_at(pin, &format!("symbol.units[{unit}].pins[{number}]"))?;
        let angle = pin
            .named_child("at")
            .and_then(|at| at.atom_at(3))
            .unwrap_or("0");
        let angle_value = angle.parse::<i32>().map_err(|_| {
            KiCadLibraryImportError::UnsupportedPinOrientation {
                pin: number.into(),
                degrees: angle.into(),
            }
        })?;
        let side = match angle_value.rem_euclid(360) {
            0 => SchematicPinSide::Left,
            90 => SchematicPinSide::Top,
            180 => SchematicPinSide::Right,
            270 => SchematicPinSide::Bottom,
            _ => {
                return Err(KiCadLibraryImportError::UnsupportedPinOrientation {
                    pin: number.into(),
                    degrees: angle.into(),
                });
            }
        };
        let kind = match pin.atom_at(1).unwrap_or("passive") {
            "input" => PinElectricalKind::Input,
            "output" | "tri_state" => PinElectricalKind::Output,
            "bidirectional" => PinElectricalKind::Bidirectional,
            "power_in" => PinElectricalKind::PowerInput,
            "power_out" => PinElectricalKind::PowerOutput,
            "open_collector" => PinElectricalKind::OpenCollector,
            "open_emitter" => PinElectricalKind::OpenEmitter,
            "no_connect" => PinElectricalKind::NotConnected,
            _ => PinElectricalKind::Passive,
        };
        if let Some(shape) = pin.atom_at(2).filter(|shape| *shape != "line") {
            self.omissions
                .push(KiCadLibraryImportOmission::PinGraphicShape {
                    pin: number.into(),
                    shape: shape.into(),
                });
        }
        Ok((
            SchematicPinPlacement {
                pin: pin_ref,
                position,
                side,
            },
            kind,
        ))
    }

    fn symbol_graphic(
        &mut self,
        node: &Sexp,
        unit: u16,
        index: usize,
    ) -> Result<Option<SchematicGraphic>, KiCadLibraryImportError> {
        let field = format!("symbol.units[{unit}].graphics[{index}]");
        let (stroke, defaulted) = self.symbol_stroke(node, &field)?;
        if defaulted {
            self.omissions
                .push(KiCadLibraryImportOmission::DefaultSymbolStroke {
                    unit,
                    graphic: index,
                });
        }
        let fill = self.symbol_fill(node)?;
        Ok(match node.list_name() {
            Some("rectangle") => Some(SchematicGraphic::Rectangle {
                start: self.schematic_point(node, "start", &format!("{field}.start"))?,
                end: self.schematic_point(node, "end", &format!("{field}.end"))?,
                stroke_width: stroke,
                fill,
            }),
            Some("circle") => Some(SchematicGraphic::Circle {
                center: self.schematic_point(node, "center", &format!("{field}.center"))?,
                radius: self.number_child(node, "radius", &format!("{field}.radius"))?,
                stroke_width: stroke,
                fill,
            }),
            Some("arc") => Some(SchematicGraphic::Arc {
                start: self.schematic_point(node, "start", &format!("{field}.start"))?,
                mid: self.schematic_point(node, "mid", &format!("{field}.mid"))?,
                end: self.schematic_point(node, "end", &format!("{field}.end"))?,
                stroke_width: stroke,
            }),
            Some("polyline") => {
                let mut points = self.schematic_points(node, &field)?;
                let closed = points.len() > 2 && points.first() == points.last();
                if closed {
                    points.pop();
                }
                Some(SchematicGraphic::Polyline {
                    points,
                    closed,
                    stroke_width: stroke,
                    fill,
                })
            }
            Some("text") => {
                let position = self.schematic_at(node, &format!("{field}.position"))?;
                let size = node
                    .named_child("effects")
                    .and_then(|effects| effects.named_child("font"))
                    .and_then(|font| font.named_child("size"))
                    .and_then(|size| size.atom_at(1))
                    .ok_or_else(|| KiCadLibraryImportError::Parse("text has no size".into()))
                    .and_then(|token| self.number(token, &format!("{field}.size")))?;
                let angle = node
                    .named_child("at")
                    .and_then(|at| at.atom_at(3))
                    .unwrap_or("0")
                    .parse::<i32>()
                    .unwrap_or(0);
                Some(SchematicGraphic::Text {
                    position,
                    text: node.atom_at(1).unwrap_or("").into(),
                    size,
                    quarter_turns: i8::try_from(angle.div_euclid(900).rem_euclid(4))
                        .expect("quarter turn is bounded"),
                })
            }
            Some(primitive) => {
                self.omissions
                    .push(KiCadLibraryImportOmission::UnsupportedSymbolGraphic {
                        unit,
                        primitive: primitive.into(),
                    });
                None
            }
            None => None,
        })
    }

    fn symbol_extents(
        &mut self,
        unit: u16,
        pins: &[SchematicPinPlacement],
        graphics: &[SchematicGraphic],
    ) -> Result<(Real, Real), KiCadLibraryImportError> {
        let mut points = pins
            .iter()
            .map(|pin| pin.position.clone())
            .collect::<Vec<_>>();
        for graphic in graphics {
            match graphic {
                SchematicGraphic::Line { from, to, .. } => {
                    points.extend([from.clone(), to.clone()]);
                }
                SchematicGraphic::Rectangle { start, end, .. }
                | SchematicGraphic::Arc { start, end, .. } => {
                    points.extend([start.clone(), end.clone()]);
                    if let SchematicGraphic::Arc { mid, .. } = graphic {
                        points.push(mid.clone());
                    }
                }
                SchematicGraphic::Circle { center, radius, .. } => {
                    points.push(SchematicPoint::new(
                        center.x.clone() - radius.clone(),
                        center.y.clone() - radius.clone(),
                    ));
                    points.push(SchematicPoint::new(
                        center.x.clone() + radius.clone(),
                        center.y.clone() + radius.clone(),
                    ));
                }
                SchematicGraphic::Polyline {
                    points: vertices, ..
                } => points.extend(vertices.iter().cloned()),
                SchematicGraphic::Text { position, .. } => points.push(position.clone()),
            }
        }
        let Some(first) = points.first() else {
            return Ok((
                self.options.minimum_symbol_body_extent.clone(),
                self.options.minimum_symbol_body_extent.clone(),
            ));
        };
        let mut min_x = first.x.clone();
        let mut max_x = first.x.clone();
        let mut min_y = first.y.clone();
        let mut max_y = first.y.clone();
        for point in points.iter().skip(1) {
            min_x = min_real(&min_x, &point.x);
            max_x = max_real(&max_x, &point.x);
            min_y = min_real(&min_y, &point.y);
            max_y = max_real(&max_y, &point.y);
        }
        let mut width = max_x - min_x;
        let mut height = max_y - min_y;
        if width <= Real::zero() {
            width = self.options.minimum_symbol_body_extent.clone();
            self.omissions
                .push(KiCadLibraryImportOmission::MinimumSymbolBodyExtent {
                    unit,
                    axis: "x".into(),
                });
        }
        if height <= Real::zero() {
            height = self.options.minimum_symbol_body_extent.clone();
            self.omissions
                .push(KiCadLibraryImportOmission::MinimumSymbolBodyExtent {
                    unit,
                    axis: "y".into(),
                });
        }
        Ok((width, height))
    }

    fn import_footprint(
        &mut self,
        root: &Sexp,
        model: &DeviceModel,
    ) -> Result<LandPattern, KiCadLibraryImportError> {
        let model_pins = model
            .pins
            .iter()
            .map(|pin| pin.pin.clone())
            .collect::<BTreeSet<_>>();
        let mut pads = Vec::new();
        let mut mappings = Vec::new();
        let mut used_pad_ids = BTreeSet::new();
        for (index, pad) in root.named_children("pad").enumerate() {
            let number = pad.atom_at(1).unwrap_or("");
            let base = if number.is_empty() {
                format!("mechanical-{index}")
            } else {
                number.into()
            };
            let pad_name = unique_name(&base, &mut used_pad_ids, index);
            let pad_id = PadId::new(&pad_name)
                .map_err(|_| KiCadLibraryImportError::InvalidIdentifier(pad_name.clone()))?;
            let center = pad
                .named_child("at")
                .map(|at| self.point(at, &format!("footprint.pads[{index}].at")))
                .transpose()?
                .unwrap_or_else(Point2::origin);
            let rotation_degrees = pad
                .named_child("at")
                .and_then(|at| at.atom_at(3))
                .map(|token| self.number(token, &format!("footprint.pads[{index}].rotation")))
                .transpose()?
                .unwrap_or_else(Real::zero);
            let size = pad
                .named_child("size")
                .ok_or_else(|| KiCadLibraryImportError::Parse("pad has no size".into()))?;
            let width = self.number_at(size, 1, &format!("footprint.pads[{index}].width"))?;
            let height = self.number_at(size, 2, &format!("footprint.pads[{index}].height"))?;
            let shape_name = pad.atom_at(3).unwrap_or("rect");
            let shape = match shape_name {
                "circle" => PadShape::Circle {
                    diameter: max_real(&width, &height),
                },
                "rect" => PadShape::Rectangle {
                    width: width.clone(),
                    height: height.clone(),
                },
                "oval" => PadShape::Obround {
                    width: width.clone(),
                    height: height.clone(),
                },
                "roundrect" => {
                    let ratio = pad
                        .named_child("roundrect_rratio")
                        .and_then(|ratio| ratio.atom_at(1))
                        .map(|token| {
                            self.number(token, &format!("footprint.pads[{index}].roundrect"))
                        })
                        .transpose()?
                        .unwrap_or_else(|| (Real::one() / Real::from(4)).expect("four is nonzero"));
                    PadShape::RoundedRectangle {
                        width: width.clone(),
                        height: height.clone(),
                        corner_radius: min_real(&width, &height) * ratio,
                    }
                }
                shape => {
                    self.omissions
                        .push(KiCadLibraryImportOmission::UnsupportedPadShape {
                            pad: number.into(),
                            shape: shape.into(),
                        });
                    PadShape::Rectangle {
                        width: width.clone(),
                        height: height.clone(),
                    }
                }
            };
            let copper_layers = self.pad_copper_layers(pad, number)?;
            let drill = self.pad_drill(pad, index)?;
            let plating = match (drill.is_some(), pad.atom_at(2)) {
                (true, Some("np_thru_hole")) => Plating::NonPlated,
                (true, _) => Plating::Plated,
                _ => Plating::Unspecified,
            };
            let solder_mask_margin = self.optional_number_child(
                pad,
                "solder_mask_margin",
                &format!("footprint.pads[{index}].solder_mask_margin"),
            )?;
            let paste_margin = self.optional_number_child(
                pad,
                "solder_paste_margin",
                &format!("footprint.pads[{index}].solder_paste_margin"),
            )?;
            if pad.named_child("solder_paste_margin_ratio").is_some() {
                self.omissions
                    .push(KiCadLibraryImportOmission::PadPasteRatio { pad: number.into() });
            }
            pads.push(LandPatternPad {
                id: pad_id.clone(),
                center,
                rotation_degrees,
                copper_layers,
                shape,
                drill,
                plating,
                solder_mask_margin,
                paste_margin,
            });
            if !number.is_empty() {
                let pin = PinRef::new(number)
                    .map_err(|_| KiCadLibraryImportError::InvalidIdentifier(number.into()))?;
                if !model_pins.contains(&pin) {
                    return Err(KiCadLibraryImportError::FootprintPinAbsentFromSymbol(
                        number.into(),
                    ));
                }
                mappings.push(PadPinMap { pin, pad: pad_id });
            }
        }
        let mut graphics = Vec::new();
        for node in root.children().iter().filter(|node| {
            matches!(
                node.list_name(),
                Some("fp_line" | "fp_rect" | "fp_circle" | "fp_poly" | "fp_text" | "fp_arc")
            )
        }) {
            if let Some(graphic) = self.footprint_graphic(node, graphics.len())? {
                graphics.push(graphic);
            }
        }
        let models = root
            .named_children("model")
            .enumerate()
            .map(|(index, model)| self.model_reference(model, &format!("model[{index}]")))
            .collect::<Result<Vec<_>, _>>()?;
        Ok(LandPattern {
            id: self.options.land_pattern_id.clone(),
            pads,
            pin_map: mappings,
            graphics,
            body: None,
            models,
        })
    }

    fn pad_copper_layers(
        &mut self,
        pad: &Sexp,
        number: &str,
    ) -> Result<Vec<TraceLayer>, KiCadLibraryImportError> {
        let names = pad
            .named_child("layers")
            .map(|layers| {
                layers
                    .children()
                    .iter()
                    .skip(1)
                    .filter_map(Sexp::as_atom)
                    .collect::<Vec<_>>()
            })
            .unwrap_or_default();
        let mut result = BTreeSet::new();
        for name in names {
            if matches!(name, "*.Cu" | "F&B.Cu") {
                result.extend(self.options.copper_layers.values().copied());
            } else if name.ends_with(".Cu") {
                if let Some(layer) = self.options.copper_layers.get(name) {
                    result.insert(*layer);
                } else {
                    self.omissions
                        .push(KiCadLibraryImportOmission::UnsupportedCopperLayer {
                            pad: number.into(),
                            layer: name.into(),
                        });
                }
            }
        }
        Ok(result.into_iter().collect())
    }

    fn pad_drill(
        &mut self,
        pad: &Sexp,
        index: usize,
    ) -> Result<Option<DrillShape>, KiCadLibraryImportError> {
        let Some(drill) = pad.named_child("drill") else {
            return Ok(None);
        };
        if drill.atom_at(1) == Some("oval") {
            let width = self.number_at(drill, 2, &format!("footprint.pads[{index}].drill.x"))?;
            let height = self.number_at(drill, 3, &format!("footprint.pads[{index}].drill.y"))?;
            let half_length = ((max_real(&width, &height) - min_real(&width, &height))
                / Real::from(2))
            .map_err(|_| KiCadLibraryImportError::Parse("invalid slot drill".into()))?;
            let cutter = min_real(&width, &height);
            let (start, end) = if width >= height {
                (
                    Point2::new(-half_length.clone(), Real::zero()),
                    Point2::new(half_length, Real::zero()),
                )
            } else {
                (
                    Point2::new(Real::zero(), -half_length.clone()),
                    Point2::new(Real::zero(), half_length),
                )
            };
            Ok(Some(DrillShape::Slot {
                start,
                end,
                width: cutter,
            }))
        } else {
            Ok(Some(DrillShape::Round {
                diameter: self.number_at(drill, 1, &format!("footprint.pads[{index}].drill"))?,
            }))
        }
    }

    fn footprint_graphic(
        &mut self,
        node: &Sexp,
        index: usize,
    ) -> Result<Option<LandPatternGraphic>, KiCadLibraryImportError> {
        let layer_name = node
            .named_child("layer")
            .and_then(|layer| layer.atom_at(1))
            .unwrap_or("User.Drawings");
        let layer = layer_role(layer_name);
        if matches!(layer, LayerRole::Custom(_)) {
            self.omissions
                .push(KiCadLibraryImportOmission::CustomArtworkLayer {
                    layer: layer_name.into(),
                });
        }
        let stroke_width = node
            .named_child("stroke")
            .and_then(|stroke| stroke.named_child("width"))
            .and_then(|width| width.atom_at(1))
            .map(|token| self.number(token, &format!("footprint.graphics[{index}].width")))
            .transpose()?;
        let primitive = match node.list_name() {
            Some("fp_line") => LandPatternGraphicPrimitive::Line {
                start: self.point_child(node, "start", &format!("footprint.graphics[{index}]"))?,
                end: self.point_child(node, "end", &format!("footprint.graphics[{index}]"))?,
            },
            Some("fp_rect") => {
                let start =
                    self.point_child(node, "start", &format!("footprint.graphics[{index}]"))?;
                let end = self.point_child(node, "end", &format!("footprint.graphics[{index}]"))?;
                LandPatternGraphicPrimitive::Polygon {
                    vertices: vec![
                        start.clone(),
                        Point2::new(end.x.clone(), start.y.clone()),
                        end.clone(),
                        Point2::new(start.x, end.y),
                    ],
                    filled: node
                        .named_child("fill")
                        .and_then(|fill| fill.atom_at(1))
                        .is_some_and(|fill| fill != "no" && fill != "none"),
                }
            }
            Some("fp_circle") => {
                let center =
                    self.point_child(node, "center", &format!("footprint.graphics[{index}]"))?;
                let end = self.point_child(node, "end", &format!("footprint.graphics[{index}]"))?;
                let dx = end.x - center.x.clone();
                let dy = end.y - center.y.clone();
                let radius_squared = dx.clone() * dx + dy.clone() * dy;
                let radius = radius_squared.sqrt().map_err(|_| {
                    KiCadLibraryImportError::Parse("circle radius is not exact".into())
                })?;
                LandPatternGraphicPrimitive::Circle { center, radius }
            }
            Some("fp_poly") => LandPatternGraphicPrimitive::Polygon {
                vertices: self.points(node, &format!("footprint.graphics[{index}]"))?,
                filled: node
                    .named_child("fill")
                    .and_then(|fill| fill.atom_at(1))
                    .is_some_and(|fill| fill != "no" && fill != "none"),
            },
            Some("fp_text") => {
                let position = node
                    .named_child("at")
                    .map(|at| self.point(at, &format!("footprint.graphics[{index}].at")))
                    .transpose()?
                    .unwrap_or_else(Point2::origin);
                let rotation = node
                    .named_child("at")
                    .and_then(|at| at.atom_at(3))
                    .map(|token| {
                        self.number(token, &format!("footprint.graphics[{index}].rotation"))
                    })
                    .transpose()?
                    .unwrap_or_else(Real::zero);
                let height = node
                    .named_child("effects")
                    .and_then(|effects| effects.named_child("font"))
                    .and_then(|font| font.named_child("size"))
                    .and_then(|size| size.atom_at(2).or_else(|| size.atom_at(1)))
                    .map(|token| self.number(token, &format!("footprint.graphics[{index}].height")))
                    .transpose()?
                    .unwrap_or_else(Real::one);
                LandPatternGraphicPrimitive::Text {
                    text: node.atom_at(2).unwrap_or("").into(),
                    position,
                    height,
                    rotation_degrees: rotation,
                }
            }
            Some(primitive) => {
                self.omissions
                    .push(KiCadLibraryImportOmission::UnsupportedFootprintGraphic {
                        primitive: primitive.into(),
                        layer: layer_name.into(),
                    });
                return Ok(None);
            }
            None => return Ok(None),
        };
        Ok(Some(LandPatternGraphic {
            id: LandPatternGraphicId::new(format!("kicad-graphic-{index}"))
                .expect("generated graphic id is nonempty"),
            layer,
            stroke_width,
            primitive,
        }))
    }

    fn symbol_stroke(
        &mut self,
        node: &Sexp,
        field: &str,
    ) -> Result<(Real, bool), KiCadLibraryImportError> {
        let width = node
            .named_child("stroke")
            .and_then(|stroke| stroke.named_child("width"))
            .and_then(|width| width.atom_at(1))
            .map(|token| self.number(token, &format!("{field}.stroke")))
            .transpose()?;
        match width {
            Some(width) if width > Real::zero() => Ok((width, false)),
            _ => Ok((self.options.default_symbol_stroke_width.clone(), true)),
        }
    }

    fn symbol_fill(&self, node: &Sexp) -> Result<SchematicGraphicFill, KiCadLibraryImportError> {
        match node
            .named_child("fill")
            .and_then(|fill| fill.named_child("type"))
            .and_then(|kind| kind.atom_at(1))
            .unwrap_or("none")
        {
            "none" => Ok(SchematicGraphicFill::None),
            "background" => Ok(SchematicGraphicFill::Background),
            "outline" => Ok(SchematicGraphicFill::Foreground),
            value => Err(KiCadLibraryImportError::Parse(format!(
                "unsupported symbol fill {value}"
            ))),
        }
    }

    fn schematic_at(
        &mut self,
        node: &Sexp,
        field: &str,
    ) -> Result<SchematicPoint, KiCadLibraryImportError> {
        self.schematic_point(node, "at", field)
    }

    fn schematic_point(
        &mut self,
        node: &Sexp,
        name: &str,
        field: &str,
    ) -> Result<SchematicPoint, KiCadLibraryImportError> {
        let point = node
            .named_child(name)
            .ok_or_else(|| KiCadLibraryImportError::Parse(format!("missing {field}")))?;
        Ok(SchematicPoint::new(
            self.number_at(point, 1, &format!("{field}.x"))?,
            self.number_at(point, 2, &format!("{field}.y"))?,
        ))
    }

    fn schematic_points(
        &mut self,
        node: &Sexp,
        field: &str,
    ) -> Result<Vec<SchematicPoint>, KiCadLibraryImportError> {
        let points = node
            .named_child("pts")
            .ok_or_else(|| KiCadLibraryImportError::Parse(format!("{field} has no points")))?;
        points
            .named_children("xy")
            .enumerate()
            .map(|(index, point)| {
                Ok(SchematicPoint::new(
                    self.number_at(point, 1, &format!("{field}[{index}].x"))?,
                    self.number_at(point, 2, &format!("{field}[{index}].y"))?,
                ))
            })
            .collect()
    }

    fn point_child(
        &mut self,
        node: &Sexp,
        name: &str,
        field: &str,
    ) -> Result<Point2, KiCadLibraryImportError> {
        node.named_child(name)
            .ok_or_else(|| KiCadLibraryImportError::Parse(format!("missing {field}.{name}")))
            .and_then(|point| self.point(point, &format!("{field}.{name}")))
    }

    fn point(&mut self, node: &Sexp, field: &str) -> Result<Point2, KiCadLibraryImportError> {
        Ok(Point2::new(
            self.number_at(node, 1, &format!("{field}.x"))?,
            self.number_at(node, 2, &format!("{field}.y"))?,
        ))
    }

    fn points(&mut self, node: &Sexp, field: &str) -> Result<Vec<Point2>, KiCadLibraryImportError> {
        node.named_child("pts")
            .ok_or_else(|| KiCadLibraryImportError::Parse(format!("{field} has no points")))?
            .named_children("xy")
            .enumerate()
            .map(|(index, point)| self.point(point, &format!("{field}[{index}]")))
            .collect()
    }

    fn number_child(
        &mut self,
        node: &Sexp,
        name: &str,
        field: &str,
    ) -> Result<Real, KiCadLibraryImportError> {
        node.named_child(name)
            .and_then(|child| child.atom_at(1))
            .ok_or_else(|| KiCadLibraryImportError::InvalidNumber {
                field: field.into(),
                token: String::new(),
            })
            .and_then(|token| self.number(token, field))
    }

    fn optional_number_child(
        &mut self,
        node: &Sexp,
        name: &str,
        field: &str,
    ) -> Result<Option<Real>, KiCadLibraryImportError> {
        node.named_child(name)
            .and_then(|child| child.atom_at(1))
            .map(|token| self.number(token, field))
            .transpose()
    }

    fn number_at(
        &mut self,
        node: &Sexp,
        index: usize,
        field: &str,
    ) -> Result<Real, KiCadLibraryImportError> {
        node.atom_at(index)
            .ok_or_else(|| KiCadLibraryImportError::InvalidNumber {
                field: field.into(),
                token: String::new(),
            })
            .and_then(|token| self.number(token, field))
    }

    fn number(&mut self, token: &str, field: &str) -> Result<Real, KiCadLibraryImportError> {
        let value = Real::from_str(token).map_err(|_| KiCadLibraryImportError::InvalidNumber {
            field: field.into(),
            token: token.into(),
        })?;
        self.numeric_imports.push(KiCadLibraryNumericImport {
            field: field.into(),
            source: token.into(),
            exact: value.to_string(),
        });
        Ok(value)
    }

    fn model_reference(
        &mut self,
        model: &Sexp,
        field: &str,
    ) -> Result<Pcb3dModelReference, KiCadLibraryImportError> {
        let uri = model
            .atom_at(1)
            .filter(|uri| !uri.trim().is_empty())
            .ok_or_else(|| KiCadLibraryImportError::Parse(format!("{field} has no URI")))?
            .to_owned();
        let offset = self.model_xyz(model, "offset", Real::zero(), &format!("{field}.offset"))?;
        let scale = self.model_xyz(model, "scale", Real::one(), &format!("{field}.scale"))?;
        let rotation = self.model_xyz(model, "rotate", Real::zero(), &format!("{field}.rotate"))?;
        Ok(Pcb3dModelReference {
            format: crate::layout::pcb_3d_model_format_from_uri(&uri),
            uri,
            transform: Pcb3dModelTransform {
                offset_x: offset[0].clone(),
                offset_y: offset[1].clone(),
                offset_z: offset[2].clone(),
                rotate_x_degrees: rotation[0].clone(),
                rotate_y_degrees: rotation[1].clone(),
                rotate_z_degrees: rotation[2].clone(),
                scale_x: scale[0].clone(),
                scale_y: scale[1].clone(),
                scale_z: scale[2].clone(),
            },
        })
    }

    fn model_xyz(
        &mut self,
        model: &Sexp,
        group: &str,
        default: Real,
        field: &str,
    ) -> Result<[Real; 3], KiCadLibraryImportError> {
        let Some(xyz) = model
            .named_child(group)
            .and_then(|group| group.named_child("xyz"))
        else {
            return Ok([default.clone(), default.clone(), default]);
        };
        Ok([
            self.number_at(xyz, 1, &format!("{field}.x"))?,
            self.number_at(xyz, 2, &format!("{field}.y"))?,
            self.number_at(xyz, 3, &format!("{field}.z"))?,
        ])
    }
}

fn validate_options(
    options: &KiCadPartLibraryImportOptions,
) -> Result<(), KiCadLibraryImportError> {
    if options.export_name.trim().is_empty()
        || options.default_symbol_stroke_width <= Real::zero()
        || options.minimum_symbol_body_extent <= Real::zero()
        || options.copper_layers.is_empty()
    {
        return Err(KiCadLibraryImportError::InvalidOptions(
            "name, positive stroke/body extent, and copper layers are required".into(),
        ));
    }
    Ok(())
}

fn unit_identity(node: &Sexp) -> Option<(u16, u16)> {
    let id = node.atom_at(1)?;
    let mut parts = id.rsplitn(3, '_');
    let style = parts.next()?.parse().ok()?;
    let unit = parts.next()?.parse().ok()?;
    Some((unit, style))
}

fn symbol_graphic_nodes(node: &Sexp) -> impl Iterator<Item = &Sexp> {
    node.children().iter().filter(|child| {
        matches!(
            child.list_name(),
            Some("arc" | "bezier" | "circle" | "curve" | "polyline" | "rectangle" | "text")
        )
    })
}

fn layer_role(name: &str) -> LayerRole {
    match name {
        "F.SilkS" => LayerRole::FrontSilkscreen,
        "B.SilkS" => LayerRole::BackSilkscreen,
        "F.Mask" => LayerRole::FrontSolderMask,
        "B.Mask" => LayerRole::BackSolderMask,
        "F.Paste" => LayerRole::FrontPaste,
        "B.Paste" => LayerRole::BackPaste,
        "F.Fab" | "B.Fab" => LayerRole::Fabrication,
        "F.CrtYd" | "B.CrtYd" => LayerRole::Courtyard,
        "Edge.Cuts" => LayerRole::EdgeCuts,
        other => LayerRole::Custom(other.into()),
    }
}

fn unique_name(base: &str, used: &mut BTreeSet<String>, index: usize) -> String {
    if used.insert(base.into()) {
        return base.into();
    }
    let candidate = format!("{base}#{index}");
    used.insert(candidate.clone());
    candidate
}

fn max_real(left: &Real, right: &Real) -> Real {
    if left >= right {
        left.clone()
    } else {
        right.clone()
    }
}

fn min_real(left: &Real, right: &Real) -> Real {
    if left <= right {
        left.clone()
    } else {
        right.clone()
    }
}

fn package_error_text(error: PackageResolutionError) -> String {
    error.to_string()
}
