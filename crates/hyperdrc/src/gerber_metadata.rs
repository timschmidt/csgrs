//! Lightweight Gerber image setup, file, aperture, and object metadata extraction.
//!
//! The geometry loader already parses the image commands needed by design-rule
//! checks. This module extracts a deliberately narrow subset of X2/X3 metadata
//! so package-readiness checks can understand layer intent even when filenames
//! are opaque, image units can be preserved, and aperture, component, pin, and
//! net intent can become structured parser evidence.

use std::fmt::Debug;

use crate::date::parse_iso_day;

/// Gerber file units declared by the `%MO...*%` mode command.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum GerberUnits {
    /// Metric coordinates and aperture dimensions, expressed in millimeters.
    Millimeters,
    /// Imperial coordinates and aperture dimensions, expressed in inches.
    Inches,
}

/// Gerber fixed-coordinate format declared by the `%FS...*%` command.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct GerberCoordinateFormat {
    /// Number of integer digits in X/Y coordinate values.
    pub integer_digits: u8,
    /// Number of decimal digits in X/Y coordinate values.
    pub decimal_digits: u8,
}

/// Image setup commands that affect coordinate interpretation.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct GerberImageSetup {
    /// File units declared by `%MOMM*%` or `%MOIN*%`.
    pub units: Option<GerberUnits>,
    /// Coordinate digit format declared by `%FSLAX..Y..*%`.
    pub coordinate_format: Option<GerberCoordinateFormat>,
}

/// File-level Gerber attributes used by package-readiness checks.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct GerberLayerMetadata {
    /// Value of the X2 `.Part` file attribute, for example `Single`, `Array`,
    /// `FabricationPanel`, `Coupon`, or `Other,<field>`.
    pub part: Option<String>,
    /// Value of the X2 `.FileFunction` file attribute, for example
    /// `Copper,L1,Top`.
    pub file_function: Option<String>,
    /// Value of the X2 `.FilePolarity` file attribute, for example `Positive`
    /// or `Negative`.
    pub file_polarity: Option<String>,
    /// Optional identifier from the X2 `.SameCoordinates` file attribute.
    ///
    /// `Some("")` means the attribute was present without an identifier, which
    /// is valid in Gerber X2.
    pub same_coordinates: Option<String>,
    /// Value of the X2 `.CreationDate` file attribute.
    pub creation_date: Option<String>,
    /// Value of the X2 `.GenerationSoftware` file attribute, commonly
    /// `<vendor>,<application>,<version>`.
    pub generation_software: Option<String>,
    /// Value of the X2 `.ProjectId` file attribute, commonly
    /// `<name>,<guid>,<revision>`.
    pub project_id: Option<String>,
    /// Value of the X2 `.MD5` file attribute.
    pub md5: Option<String>,
}

/// Aperture-level Gerber attribute extracted for future design-intent checks.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GerberApertureMetadata {
    /// One-based source line number where the attribute was declared.
    pub line: usize,
    /// Raw normalized `.AperFunction` value, for example `SMDPad,CuDef`.
    pub function: String,
}

/// Gerber aperture definition extracted from a `%ADD...*%` command.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GerberApertureDefinition {
    /// One-based source line number where the aperture was defined.
    pub line: usize,
    /// Aperture D-code. Gerber reserves D01-D09, so valid apertures start at D10.
    pub d_code: u32,
    /// Aperture template name such as `C`, `R`, `O`, `P`, or a macro name.
    pub template: String,
    /// Raw parameter string after the template comma, if present.
    pub parameters: Option<String>,
}

/// Gerber aperture macro definition extracted from a `%AM...*%` command.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GerberApertureMacro {
    /// One-based source line number where the macro was declared.
    pub line: usize,
    /// Macro name used later as a custom `%ADD...*%` aperture template.
    pub name: String,
    /// Raw macro body after the name separator, without the enclosing `%...%`.
    pub body: String,
    /// Number of non-empty primitive or variable-assignment statements.
    pub primitive_count: usize,
}

/// Gerber aperture usage extracted from image operation commands.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GerberApertureUse {
    /// One-based source line number where the aperture was selected or used.
    pub line: usize,
    /// Aperture D-code selected or used by the operation.
    pub d_code: u32,
    /// Whether this record is a `Dnn` selection or an operation using the current aperture.
    pub kind: GerberApertureUseKind,
}

/// How a Gerber aperture was used in the image stream.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum GerberApertureUseKind {
    /// Bare `Dnn*` command selecting the current aperture.
    Select,
    /// `D01` draw operation using the current aperture.
    Draw,
    /// `D03` flash operation using the current aperture.
    Flash,
}

/// Coordinate operation extracted from an image statement ending in `D01`, `D02`, or `D03`.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GerberCoordinateOperation {
    /// One-based source line number where the operation appeared.
    pub line: usize,
    /// Operation kind declared by the trailing D-code.
    pub kind: GerberCoordinateOperationKind,
    /// Raw X coordinate field, still in the file's fixed coordinate notation.
    pub x: Option<String>,
    /// Raw Y coordinate field, still in the file's fixed coordinate notation.
    pub y: Option<String>,
    /// Raw I arc-offset field, still in the file's fixed coordinate notation.
    pub i: Option<String>,
    /// Raw J arc-offset field, still in the file's fixed coordinate notation.
    pub j: Option<String>,
}

/// Gerber coordinate operation kind.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum GerberCoordinateOperationKind {
    /// `D01` draws using the current aperture and interpolation mode.
    Draw,
    /// `D02` moves the current point without exposure.
    Move,
    /// `D03` flashes the current aperture.
    Flash,
}

/// Gerber image polarity declared by `%LP...*%`.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum GerberImagePolarity {
    /// Dark polarity adds material to the image.
    Dark,
    /// Clear polarity subtracts material from the image.
    Clear,
}

/// Gerber image transformation extracted from `%LM...*%`, `%LR...*%`, or `%LS...*%`.
#[derive(Clone, Debug, PartialEq)]
pub struct GerberImageTransform {
    /// One-based source line number where the transformation was declared.
    pub line: usize,
    /// Transformation state after this command.
    pub kind: GerberImageTransformKind,
}

/// Stateful Gerber image transformation command.
#[derive(Clone, Debug, PartialEq)]
pub enum GerberImageTransformKind {
    /// Mirror mode declared by `%LM...*%`.
    Mirror(GerberMirrorMode),
    /// Rotation angle in degrees declared by `%LR...*%`.
    Rotation(f64),
    /// Scale factor declared by `%LS...*%`.
    Scale(f64),
}

/// Gerber mirror mode declared by `%LM...*%`.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum GerberMirrorMode {
    /// No mirroring.
    None,
    /// Mirror the X axis.
    X,
    /// Mirror the Y axis.
    Y,
    /// Mirror both axes.
    XY,
}

/// Image polarity change extracted from a `%LP...*%` command.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GerberPolarityChange {
    /// One-based source line number where the polarity was declared.
    pub line: usize,
    /// Polarity state after this command.
    pub polarity: GerberImagePolarity,
}

/// Gerber region-mode transition extracted from `G36*` and `G37*` commands.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GerberRegionEvent {
    /// One-based source line number where the transition was declared.
    pub line: usize,
    /// Region-mode transition kind.
    pub kind: GerberRegionEventKind,
}

/// State transition for Gerber region mode.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum GerberRegionEventKind {
    /// `G36*` starts a region outline.
    Start,
    /// `G37*` ends the current region outline.
    End,
}

/// Gerber step-and-repeat transition extracted from `%SR...*%` commands.
#[derive(Clone, Debug, PartialEq)]
pub struct GerberStepRepeatEvent {
    /// One-based source line number where the transition was declared.
    pub line: usize,
    /// Step-and-repeat transition kind.
    pub kind: GerberStepRepeatEventKind,
}

/// State transition for Gerber step-and-repeat mode.
#[derive(Clone, Debug, PartialEq)]
pub enum GerberStepRepeatEventKind {
    /// `%SRX...Y...I...J...*%` starts repeated plotting.
    Start {
        /// Number of repeated copies in the X direction.
        x_repeats: u32,
        /// Number of repeated copies in the Y direction.
        y_repeats: u32,
        /// X-axis step distance in the file's current unit system.
        x_step: f64,
        /// Y-axis step distance in the file's current unit system.
        y_step: f64,
    },
    /// `%SR*%` ends the current step-and-repeat block.
    End,
}

/// Gerber interpolation mode declared by `G01`, `G02`, or `G03`.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum GerberInterpolationMode {
    /// Linear interpolation for straight draws.
    Linear,
    /// Clockwise circular interpolation for arc draws.
    ClockwiseCircular,
    /// Counter-clockwise circular interpolation for arc draws.
    CounterClockwiseCircular,
}

/// Interpolation mode declaration extracted from the image stream.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GerberInterpolationEvent {
    /// One-based source line number where the mode was declared.
    pub line: usize,
    /// Active interpolation mode after this declaration.
    pub mode: GerberInterpolationMode,
}

/// Gerber arc quadrant mode declared by `G74` or `G75`.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum GerberQuadrantMode {
    /// Single-quadrant arc interpolation.
    Single,
    /// Multi-quadrant arc interpolation.
    Multi,
}

/// Quadrant mode declaration extracted from the image stream.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GerberQuadrantEvent {
    /// One-based source line number where the mode was declared.
    pub line: usize,
    /// Active quadrant mode after this declaration.
    pub mode: GerberQuadrantMode,
}

/// Object-level Gerber attribute extracted for future netlist and assembly checks.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum GerberObjectMetadata {
    /// X2/X3 `.N` object attribute naming the CAD net or nets for a conducting object.
    Net {
        /// One-based source line number where the attribute was declared.
        line: usize,
        /// Net names. A single empty string is the standard unconnected-object marker.
        nets: Vec<String>,
    },
    /// X2/X3 `.C` object attribute naming the component reference designator.
    Component {
        /// One-based source line number where the attribute was declared.
        line: usize,
        /// Component reference designator.
        refdes: String,
    },
    /// X2/X3 `.P` object attribute naming a component pin.
    Pin {
        /// One-based source line number where the attribute was declared.
        line: usize,
        /// Component reference designator.
        refdes: String,
        /// Pin number. `None` means the attribute used the standard empty pin field.
        pin: Option<String>,
    },
}

/// Gerber attribute-delete command extracted from `%TD...*%`.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GerberAttributeDelete {
    /// One-based source line number where the delete command appeared.
    pub line: usize,
    /// Attribute delete target.
    pub target: GerberAttributeDeleteTarget,
}

/// Target of a Gerber `%TD...*%` attribute-delete command.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum GerberAttributeDeleteTarget {
    /// `%TD*%` clears all currently active aperture/object attributes.
    All,
    /// `%TD.<name>*%` clears the named active aperture/object attribute.
    Named(String),
}

/// Gerber metadata extraction report with non-fatal parser diagnostics.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct GerberMetadataReport {
    /// Parsed image setup commands used to interpret coordinates.
    pub image_setup: GerberImageSetup,
    /// Parsed file-level attributes.
    pub metadata: GerberLayerMetadata,
    /// Parsed aperture-level `.AperFunction` declarations.
    pub aperture_functions: Vec<GerberApertureMetadata>,
    /// Parsed aperture definitions from `%ADD...*%` commands.
    pub aperture_definitions: Vec<GerberApertureDefinition>,
    /// Parsed aperture macro definitions from `%AM...*%` commands.
    pub aperture_macros: Vec<GerberApertureMacro>,
    /// Parsed aperture selections and operations that require a current aperture.
    pub aperture_uses: Vec<GerberApertureUse>,
    /// Parsed coordinate operations ending in `D01`, `D02`, or `D03`.
    pub coordinate_operations: Vec<GerberCoordinateOperation>,
    /// Parsed image polarity changes from `%LP...*%` commands.
    pub polarity_changes: Vec<GerberPolarityChange>,
    /// Parsed image transformations from `%LM...*%`, `%LR...*%`, and `%LS...*%`.
    pub image_transforms: Vec<GerberImageTransform>,
    /// Parsed region-mode transitions from `G36*` and `G37*` commands.
    pub region_events: Vec<GerberRegionEvent>,
    /// Parsed step-and-repeat transitions from `%SR...*%` commands.
    pub step_repeat_events: Vec<GerberStepRepeatEvent>,
    /// Parsed interpolation mode declarations from `G01`, `G02`, and `G03`.
    pub interpolation_events: Vec<GerberInterpolationEvent>,
    /// Parsed quadrant mode declarations from `G74` and `G75`.
    pub quadrant_events: Vec<GerberQuadrantEvent>,
    /// Parsed object-level `.N`, `.C`, and `.P` declarations.
    pub object_attributes: Vec<GerberObjectMetadata>,
    /// Parsed attribute-delete commands from `%TD...*%`.
    pub attribute_deletes: Vec<GerberAttributeDelete>,
    /// Non-fatal issues found while reading file-level attributes.
    pub issues: Vec<GerberMetadataIssue>,
}

/// Non-fatal Gerber metadata parser issue.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GerberMetadataIssue {
    /// One-based source line number.
    pub line: usize,
    /// Machine-readable issue kind.
    pub kind: GerberMetadataIssueKind,
}

/// Public enumeration for `GerberMetadataIssueKind`.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum GerberMetadataIssueKind {
    /// A file attribute that requires a value was present without one.
    MissingFileAttributeValue {
        /// Attribute name, for example `TF.FileFunction`.
        attribute: String,
    },
    /// A file attribute was repeated with the same value.
    DuplicateFileAttribute {
        /// Attribute name, for example `TF.FilePolarity`.
        attribute: String,
        /// Repeated attribute value.
        value: String,
    },
    /// A file attribute was repeated with a different value.
    ConflictingFileAttribute {
        /// Attribute name, for example `TF.FileFunction`.
        attribute: String,
        /// First value kept by the parser.
        first: String,
        /// Later conflicting value.
        duplicate: String,
    },
    /// A file attribute used a value outside the standard value set.
    InvalidFileAttributeValue {
        /// Attribute name, for example `TF.FilePolarity`.
        attribute: String,
        /// Attribute value kept by the parser for downstream review.
        value: String,
    },
    /// An aperture attribute that requires a value was present without one.
    MissingApertureAttributeValue {
        /// Attribute name, for example `TA.AperFunction`.
        attribute: String,
    },
    /// An aperture attribute used a value outside the standard form HyperDRC validates.
    InvalidApertureAttributeValue {
        /// Attribute name, for example `TA.AperFunction`.
        attribute: String,
        /// Attribute value kept by the parser for downstream review.
        value: String,
    },
    /// An aperture definition was present without a D-code or template.
    MissingApertureDefinitionValue {
        /// Command name, currently `ADD`.
        command: String,
    },
    /// An aperture definition used a malformed D-code, template, or parameter list.
    InvalidApertureDefinitionValue {
        /// Command name, currently `ADD`.
        command: String,
        /// Command value kept by the parser for downstream review.
        value: String,
    },
    /// An aperture D-code was repeated with the same definition.
    DuplicateApertureDefinition {
        /// Aperture D-code.
        d_code: u32,
    },
    /// An aperture D-code was repeated with a different definition.
    ConflictingApertureDefinition {
        /// Aperture D-code.
        d_code: u32,
        /// First definition kept by the parser.
        first: String,
        /// Later conflicting definition.
        duplicate: String,
    },
    /// An aperture macro was present without a name or body.
    MissingApertureMacroValue {
        /// Command name, currently `AM`.
        command: String,
    },
    /// An aperture macro used a malformed name or empty primitive list.
    InvalidApertureMacroValue {
        /// Command name, currently `AM`.
        command: String,
        /// Command value kept by the parser for downstream review.
        value: String,
    },
    /// An aperture macro name was repeated with the same body.
    DuplicateApertureMacro {
        /// Macro name.
        name: String,
    },
    /// An aperture macro name was repeated with a different body.
    ConflictingApertureMacro {
        /// Macro name.
        name: String,
        /// First macro body summary kept by the parser.
        first: String,
        /// Later conflicting macro body summary.
        duplicate: String,
    },
    /// A `Dnn` command selected an aperture that has not been defined.
    UndefinedApertureSelection {
        /// Undefined aperture D-code.
        d_code: u32,
    },
    /// A draw or flash operation was encountered before any current aperture was selected.
    MissingCurrentAperture {
        /// Operation code, usually `D01` or `D03`.
        operation: String,
    },
    /// An image polarity command that requires a value was present without one.
    MissingPolarityCommandValue {
        /// Command name, currently `LP`.
        command: String,
    },
    /// An image polarity command used a malformed polarity value.
    InvalidPolarityCommandValue {
        /// Command name, currently `LP`.
        command: String,
        /// Command value kept by the parser for downstream review.
        value: String,
    },
    /// A `G36*` region start appeared while a region was already open.
    NestedRegion {
        /// One-based line where the already-open region started.
        open_line: usize,
    },
    /// A `G37*` region end appeared while no region was open.
    UnmatchedRegionEnd,
    /// End of file was reached while a `G36*` region was still open.
    UnterminatedRegion {
        /// One-based line where the unterminated region started.
        open_line: usize,
    },
    /// A step-and-repeat command used malformed or incomplete parameters.
    InvalidStepRepeatCommandValue {
        /// Command name, currently `SR`.
        command: String,
        /// Command value kept by the parser for downstream review.
        value: String,
    },
    /// A step-and-repeat start appeared while another repeat block was already open.
    NestedStepRepeat {
        /// One-based line where the already-open repeat block started.
        open_line: usize,
    },
    /// A step-and-repeat close appeared while no repeat block was open.
    UnmatchedStepRepeatEnd,
    /// End of file was reached while a step-and-repeat block was still open.
    UnterminatedStepRepeat {
        /// One-based line where the unterminated repeat block started.
        open_line: usize,
    },
    /// An object attribute that requires a value was present without one.
    MissingObjectAttributeValue {
        /// Attribute name, for example `TO.C`.
        attribute: String,
    },
    /// An object attribute used a value outside the standard form HyperDRC validates.
    InvalidObjectAttributeValue {
        /// Attribute name, for example `TO.P`.
        attribute: String,
        /// Attribute value kept by the parser for downstream review.
        value: String,
    },
    /// An attribute-delete command used a malformed target.
    InvalidAttributeDeleteValue {
        /// Command name, currently `TD`.
        command: String,
        /// Command value kept by the parser for downstream review.
        value: String,
    },
    /// An image setup command that requires a value was present without one.
    MissingImageCommandValue {
        /// Command name, for example `MO` or `FS`.
        command: String,
    },
    /// An image setup command used a value outside the standard form HyperDRC validates.
    InvalidImageCommandValue {
        /// Command name, for example `MO` or `FS`.
        command: String,
        /// Command value kept by the parser for downstream review.
        value: String,
    },
    /// An image setup command was repeated with the same value.
    DuplicateImageCommand {
        /// Command name, for example `MO` or `FS`.
        command: String,
        /// Repeated command value.
        value: String,
    },
    /// An image setup command was repeated with a different value.
    ConflictingImageCommand {
        /// Command name, for example `MO` or `FS`.
        command: String,
        /// First value kept by the parser.
        first: String,
        /// Later conflicting value.
        duplicate: String,
    },
}

impl GerberMetadataIssue {
    /// Human-readable parser diagnostic text.
    pub fn message(&self) -> String {
        match &self.kind {
            GerberMetadataIssueKind::MissingFileAttributeValue { attribute } => {
                format!("Gerber X2 file attribute {attribute} is missing its required value")
            }
            GerberMetadataIssueKind::DuplicateFileAttribute { attribute, value } => {
                format!("Gerber X2 file attribute {attribute} is repeated with value {value:?}")
            }
            GerberMetadataIssueKind::ConflictingFileAttribute {
                attribute,
                first,
                duplicate,
            } => format!(
                "Gerber X2 file attribute {attribute} is redefined from {first:?} to {duplicate:?}; the first value is used"
            ),
            GerberMetadataIssueKind::InvalidFileAttributeValue { attribute, value } => {
                format!("Gerber X2 file attribute {attribute} has non-standard value {value:?}")
            }
            GerberMetadataIssueKind::MissingApertureAttributeValue { attribute } => {
                format!("Gerber X2 aperture attribute {attribute} is missing its required value")
            }
            GerberMetadataIssueKind::InvalidApertureAttributeValue { attribute, value } => {
                format!("Gerber X2 aperture attribute {attribute} has non-standard value {value:?}")
            }
            GerberMetadataIssueKind::MissingApertureDefinitionValue { command } => {
                format!(
                    "Gerber aperture definition command {command} is missing its required value"
                )
            }
            GerberMetadataIssueKind::InvalidApertureDefinitionValue { command, value } => {
                format!(
                    "Gerber aperture definition command {command} has non-standard value {value:?}"
                )
            }
            GerberMetadataIssueKind::DuplicateApertureDefinition { d_code } => {
                format!("Gerber aperture definition D{d_code} is repeated with the same template")
            }
            GerberMetadataIssueKind::ConflictingApertureDefinition {
                d_code,
                first,
                duplicate,
            } => format!(
                "Gerber aperture definition D{d_code} is redefined from {first:?} to {duplicate:?}; the first definition is used"
            ),
            GerberMetadataIssueKind::MissingApertureMacroValue { command } => {
                format!("Gerber aperture macro command {command} is missing its required value")
            }
            GerberMetadataIssueKind::InvalidApertureMacroValue { command, value } => {
                format!("Gerber aperture macro command {command} has non-standard value {value:?}")
            }
            GerberMetadataIssueKind::DuplicateApertureMacro { name } => {
                format!("Gerber aperture macro {name} is repeated with the same body")
            }
            GerberMetadataIssueKind::ConflictingApertureMacro {
                name,
                first,
                duplicate,
            } => format!(
                "Gerber aperture macro {name} is redefined from {first:?} to {duplicate:?}; the first definition is used"
            ),
            GerberMetadataIssueKind::UndefinedApertureSelection { d_code } => {
                format!("Gerber image selects undefined aperture D{d_code}")
            }
            GerberMetadataIssueKind::MissingCurrentAperture { operation } => {
                format!("Gerber image operation {operation} requires a current aperture")
            }
            GerberMetadataIssueKind::MissingPolarityCommandValue { command } => {
                format!("Gerber image polarity command {command} is missing its required value")
            }
            GerberMetadataIssueKind::InvalidPolarityCommandValue { command, value } => {
                format!("Gerber image polarity command {command} has non-standard value {value:?}")
            }
            GerberMetadataIssueKind::NestedRegion { open_line } => {
                format!(
                    "Gerber region starts while the region opened on line {open_line} is still active"
                )
            }
            GerberMetadataIssueKind::UnmatchedRegionEnd => {
                "Gerber region end appears without an active region".to_string()
            }
            GerberMetadataIssueKind::UnterminatedRegion { open_line } => {
                format!("Gerber region opened on line {open_line} is not closed before end of file")
            }
            GerberMetadataIssueKind::InvalidStepRepeatCommandValue { command, value } => {
                format!("Gerber step-and-repeat command {command} has non-standard value {value:?}")
            }
            GerberMetadataIssueKind::NestedStepRepeat { open_line } => {
                format!(
                    "Gerber step-and-repeat starts while the repeat block opened on line {open_line} is still active"
                )
            }
            GerberMetadataIssueKind::UnmatchedStepRepeatEnd => {
                "Gerber step-and-repeat end appears without an active repeat block".to_string()
            }
            GerberMetadataIssueKind::UnterminatedStepRepeat { open_line } => {
                format!(
                    "Gerber step-and-repeat block opened on line {open_line} is not closed before end of file"
                )
            }
            GerberMetadataIssueKind::MissingObjectAttributeValue { attribute } => {
                format!("Gerber X2 object attribute {attribute} is missing its required value")
            }
            GerberMetadataIssueKind::InvalidObjectAttributeValue { attribute, value } => {
                format!("Gerber X2 object attribute {attribute} has non-standard value {value:?}")
            }
            GerberMetadataIssueKind::InvalidAttributeDeleteValue { command, value } => {
                format!(
                    "Gerber X2 attribute-delete command {command} has non-standard value {value:?}"
                )
            }
            GerberMetadataIssueKind::MissingImageCommandValue { command } => {
                format!("Gerber image command {command} is missing its required value")
            }
            GerberMetadataIssueKind::InvalidImageCommandValue { command, value } => {
                format!("Gerber image command {command} has non-standard value {value:?}")
            }
            GerberMetadataIssueKind::DuplicateImageCommand { command, value } => {
                format!("Gerber image command {command} is repeated with value {value:?}")
            }
            GerberMetadataIssueKind::ConflictingImageCommand {
                command,
                first,
                duplicate,
            } => format!(
                "Gerber image command {command} is redefined from {first:?} to {duplicate:?}; the first value is used"
            ),
        }
    }
}

/// Extract the X2 file-level metadata that HyperDRC currently consumes.
///
/// The Ucamco Gerber Layer Format Specification, rev. 2024.05, sections 4.2.1
/// and 4.2.2 define `%MO...*%` units and `%FS...*%` coordinate format,
/// section 4.4 defines standard `%ADD...*%` aperture templates and `%AM...*%`
/// aperture macros, interpolation and quadrant `G` commands preserve line/arc
/// image semantics, region-mode commands `G36*`/`G37*` preserve filled contour
/// boundaries, step-and-repeat commands `%SR...*%` replicate image substreams,
/// image polarity and transformation commands define dark/clear material
/// semantics and mirror/rotate/scale state, sections 5.2 and 5.6 define
/// `%TF...*%` file attributes such as `.FileFunction` and
/// `.Part`, `.FilePolarity`, `.CreationDate`, `.GenerationSoftware`,
/// `.ProjectId`, and `.MD5`; section 5.6.10 defines `.AperFunction` aperture
/// intent; sections 5.6.13-5.6.15 define `.N`, `.P`, and `.C` object
/// attributes for net, component-pin, and component-refdes evidence; and the
/// `%TD...*%` command deletes active aperture/object attributes. This parser
/// also accepts the standardized
/// `G04 #@! TF...*` comment form described by the same specification for
/// legacy readers.
///
/// ```
/// use hyperdrc::gerber_metadata::parse_gerber_metadata;
///
/// let metadata = parse_gerber_metadata(
///     b"%TF.Part,Single*%\n%TF.FileFunction,Copper,L1,Top*%\n%TF.FilePolarity,Positive*%\n%TF.SameCoordinates,PX1*%\n%TF.CreationDate,2026-05-16T12:00:00Z*%\n%TF.ProjectId,Widget,550e8400-e29b-41d4-a716-446655440000,A*%\n%TF.MD5,d41d8cd98f00b204e9800998ecf8427e*%",
/// );
/// assert_eq!(metadata.part.as_deref(), Some("Single"));
/// assert_eq!(metadata.file_function.as_deref(), Some("Copper,L1,Top"));
/// assert_eq!(metadata.file_polarity.as_deref(), Some("Positive"));
/// assert_eq!(metadata.same_coordinates.as_deref(), Some("PX1"));
/// assert_eq!(metadata.creation_date.as_deref(), Some("2026-05-16T12:00:00Z"));
/// assert_eq!(
///     metadata.project_id.as_deref(),
///     Some("Widget,550e8400-e29b-41d4-a716-446655440000,A")
/// );
/// assert_eq!(
///     metadata.md5.as_deref(),
///     Some("d41d8cd98f00b204e9800998ecf8427e")
/// );
/// ```
pub fn parse_gerber_metadata(bytes: &[u8]) -> GerberLayerMetadata {
    parse_gerber_metadata_report(bytes).metadata
}

/// Extract Gerber setup metadata, X2/X3 attributes, and non-fatal parser diagnostics.
pub fn parse_gerber_metadata_report(bytes: &[u8]) -> GerberMetadataReport {
    let text = String::from_utf8_lossy(bytes);
    let mut report = GerberMetadataReport::default();
    let mut current_aperture = None;
    let mut open_region_line = None;
    let mut open_step_repeat_line = None;

    for (line_index, line) in text.lines().enumerate() {
        let line_number = line_index + 1;
        for chunk in line.split('%') {
            if capture_aperture_macro_chunk(chunk, line_number, &mut report) {
                continue;
            }
            let Some(command) = command_body(chunk) else {
                continue;
            };
            capture_attribute(
                command,
                line_number,
                &mut open_step_repeat_line,
                &mut report,
            );
        }

        for command in line.split('*') {
            let command = command.trim();
            if let Some(attribute) = command.strip_prefix("G04 #@!") {
                capture_attribute(
                    attribute.trim(),
                    line_number,
                    &mut open_step_repeat_line,
                    &mut report,
                );
            } else {
                capture_image_operation(
                    command,
                    line_number,
                    &mut current_aperture,
                    &mut open_region_line,
                    &mut report,
                );
            }
        }
    }

    if let Some(open_line) = open_region_line {
        report.issues.push(GerberMetadataIssue {
            line: open_line,
            kind: GerberMetadataIssueKind::UnterminatedRegion { open_line },
        });
    }
    if let Some(open_line) = open_step_repeat_line {
        report.issues.push(GerberMetadataIssue {
            line: open_line,
            kind: GerberMetadataIssueKind::UnterminatedStepRepeat { open_line },
        });
    }

    log::trace!(
        "gerber metadata parse: units={} coordinate_format={} part={} file_function={} file_polarity={} same_coordinates={} creation_date={} generation_software={} project_id={} md5={} aper_functions={} aperture_definitions={} aperture_macros={} aperture_uses={} coordinate_operations={} polarity_changes={} image_transforms={} region_events={} step_repeat_events={} interpolation_events={} quadrant_events={} object_attributes={} attribute_deletes={} issues={}",
        report.image_setup.units.is_some(),
        report.image_setup.coordinate_format.is_some(),
        report.metadata.part.is_some(),
        report.metadata.file_function.is_some(),
        report.metadata.file_polarity.is_some(),
        report.metadata.same_coordinates.is_some(),
        report.metadata.creation_date.is_some(),
        report.metadata.generation_software.is_some(),
        report.metadata.project_id.is_some(),
        report.metadata.md5.is_some(),
        report.aperture_functions.len(),
        report.aperture_definitions.len(),
        report.aperture_macros.len(),
        report.aperture_uses.len(),
        report.coordinate_operations.len(),
        report.polarity_changes.len(),
        report.image_transforms.len(),
        report.region_events.len(),
        report.step_repeat_events.len(),
        report.interpolation_events.len(),
        report.quadrant_events.len(),
        report.object_attributes.len(),
        report.attribute_deletes.len(),
        report.issues.len()
    );

    report
}

fn capture_image_operation(
    command: &str,
    line: usize,
    current_aperture: &mut Option<u32>,
    open_region_line: &mut Option<usize>,
    report: &mut GerberMetadataReport,
) {
    if command.is_empty() || command.starts_with('%') || command.starts_with("G04") {
        return;
    }

    if capture_image_mode_command(command, line, report) {
        return;
    }

    match command {
        "G36" => {
            // Ucamco Gerber Layer Format Specification, rev. 2024.05, defines
            // G36/G37 as the region start/end commands for filled contours.
            // Region boundaries matter for later checks because an extracted
            // polygon can otherwise lose the source evidence that it came from
            // a region rather than repeated stroked draws.
            if let Some(open_line) = *open_region_line {
                report.issues.push(GerberMetadataIssue {
                    line,
                    kind: GerberMetadataIssueKind::NestedRegion { open_line },
                });
            } else {
                *open_region_line = Some(line);
            }
            report.region_events.push(GerberRegionEvent {
                line,
                kind: GerberRegionEventKind::Start,
            });
            return;
        }
        "G37" => {
            if open_region_line.take().is_none() {
                report.issues.push(GerberMetadataIssue {
                    line,
                    kind: GerberMetadataIssueKind::UnmatchedRegionEnd,
                });
            }
            report.region_events.push(GerberRegionEvent {
                line,
                kind: GerberRegionEventKind::End,
            });
            return;
        }
        _ => {}
    }

    if let Some(d_code) = bare_d_code(command) {
        if d_code >= 10 {
            if aperture_defined(report, d_code) {
                *current_aperture = Some(d_code);
                report.aperture_uses.push(GerberApertureUse {
                    line,
                    d_code,
                    kind: GerberApertureUseKind::Select,
                });
            } else {
                report.issues.push(GerberMetadataIssue {
                    line,
                    kind: GerberMetadataIssueKind::UndefinedApertureSelection { d_code },
                });
            }
        }
        return;
    }

    let Some(operation) = trailing_operation_code(command) else {
        return;
    };
    capture_coordinate_operation(command, line, operation, report);
    let kind = match operation {
        1 => Some(GerberApertureUseKind::Draw),
        3 => Some(GerberApertureUseKind::Flash),
        _ => None,
    };
    let Some(kind) = kind else {
        return;
    };
    let Some(d_code) = *current_aperture else {
        report.issues.push(GerberMetadataIssue {
            line,
            kind: GerberMetadataIssueKind::MissingCurrentAperture {
                operation: format!("D{operation:02}"),
            },
        });
        return;
    };
    report
        .aperture_uses
        .push(GerberApertureUse { line, d_code, kind });
}

fn capture_coordinate_operation(
    command: &str,
    line: usize,
    operation: u32,
    report: &mut GerberMetadataReport,
) {
    let kind = match operation {
        1 => Some(GerberCoordinateOperationKind::Draw),
        2 => Some(GerberCoordinateOperationKind::Move),
        3 => Some(GerberCoordinateOperationKind::Flash),
        _ => None,
    };
    let Some(kind) = kind else {
        return;
    };

    // Ucamco Gerber Layer Format Specification, rev. 2024.05, defines D01,
    // D02, and D03 as draw, move, and flash operations. Storing the raw
    // coordinate fields keeps parser diagnostics explainable after the geometry
    // loader has already converted the image stream into flattened polygons.
    report
        .coordinate_operations
        .push(GerberCoordinateOperation {
            line,
            kind,
            x: coordinate_field(command, 'X'),
            y: coordinate_field(command, 'Y'),
            i: coordinate_field(command, 'I'),
            j: coordinate_field(command, 'J'),
        });
}

fn capture_image_mode_command(
    command: &str,
    line: usize,
    report: &mut GerberMetadataReport,
) -> bool {
    let Some((g_code, end_index)) = leading_g_code(command) else {
        return false;
    };

    match g_code {
        1 => {
            // Ucamco Gerber Layer Format Specification, rev. 2024.05, defines
            // G01/G02/G03 as stateful interpolation modes. Preserving explicit
            // declarations lets parser diagnostics explain whether an extracted
            // segment originated from linear or circular image semantics.
            report.interpolation_events.push(GerberInterpolationEvent {
                line,
                mode: GerberInterpolationMode::Linear,
            });
        }
        2 => {
            report.interpolation_events.push(GerberInterpolationEvent {
                line,
                mode: GerberInterpolationMode::ClockwiseCircular,
            });
        }
        3 => {
            report.interpolation_events.push(GerberInterpolationEvent {
                line,
                mode: GerberInterpolationMode::CounterClockwiseCircular,
            });
        }
        74 => {
            // Ucamco Gerber Layer Format Specification, rev. 2024.05, defines
            // G74/G75 as arc quadrant modes. The mode influences how arc center
            // offsets are interpreted before geometry flattening.
            report.quadrant_events.push(GerberQuadrantEvent {
                line,
                mode: GerberQuadrantMode::Single,
            });
        }
        75 => {
            report.quadrant_events.push(GerberQuadrantEvent {
                line,
                mode: GerberQuadrantMode::Multi,
            });
        }
        _ => return false,
    }

    command.get(end_index..).is_some_and(str::is_empty)
}

fn leading_g_code(command: &str) -> Option<(u32, usize)> {
    let rest = command.strip_prefix('G')?;
    let digit_len = rest
        .bytes()
        .take_while(|byte| byte.is_ascii_digit())
        .count();
    if digit_len == 0 {
        return None;
    }
    let digits = rest.get(..digit_len)?;
    Some((digits.parse::<u32>().ok()?, 1 + digit_len))
}

fn bare_d_code(command: &str) -> Option<u32> {
    let digits = command.strip_prefix('D')?;
    (!digits.is_empty() && digits.bytes().all(|byte| byte.is_ascii_digit()))
        .then(|| digits.parse::<u32>().ok())
        .flatten()
}

fn trailing_operation_code(command: &str) -> Option<u32> {
    let d_index = command.rfind('D')?;
    let digits = command.get(d_index + 1..)?;
    (!digits.is_empty() && digits.bytes().all(|byte| byte.is_ascii_digit()))
        .then(|| digits.parse::<u32>().ok())
        .flatten()
}

fn coordinate_field(command: &str, letter: char) -> Option<String> {
    let start = command.find(letter)? + letter.len_utf8();
    let end = command
        .get(start..)?
        .char_indices()
        .find_map(|(offset, ch)| {
            matches!(ch, 'X' | 'Y' | 'I' | 'J' | 'D' | 'G').then_some(start + offset)
        })
        .unwrap_or(command.len());
    let value = command.get(start..end)?.trim();
    (!value.is_empty()).then(|| value.to_string())
}

fn aperture_defined(report: &GerberMetadataReport, d_code: u32) -> bool {
    report
        .aperture_definitions
        .iter()
        .any(|definition| definition.d_code == d_code)
}

fn command_body(chunk: &str) -> Option<&str> {
    let body = chunk.split_once('*').map_or(chunk, |(body, _)| body).trim();
    (body.starts_with("TF.")
        || body.starts_with("TA.")
        || body.starts_with("TO.")
        || body.starts_with("MO")
        || body.starts_with("FS")
        || body.starts_with("LP")
        || body.starts_with("LM")
        || body.starts_with("LR")
        || body.starts_with("LS")
        || body.starts_with("SR")
        || body.starts_with("ADD")
        || body.starts_with("TD"))
    .then_some(body)
}

fn capture_aperture_macro_chunk(
    chunk: &str,
    line: usize,
    report: &mut GerberMetadataReport,
) -> bool {
    let chunk = chunk.trim();
    if !chunk.starts_with("AM") {
        return false;
    }
    capture_aperture_macro(command_chunk_body(chunk), line, report);
    true
}

fn command_chunk_body(chunk: &str) -> &str {
    chunk
        .strip_suffix('%')
        .unwrap_or(chunk)
        .strip_suffix('*')
        .unwrap_or(chunk)
        .trim()
}

fn capture_attribute(
    command: &str,
    line: usize,
    open_step_repeat_line: &mut Option<usize>,
    report: &mut GerberMetadataReport,
) {
    if command.starts_with("MO") {
        capture_unit_command(command, line, report);
    } else if command.starts_with("FS") {
        capture_format_command(command, line, report);
    } else if command.starts_with("LP") {
        capture_polarity_command(command, line, report);
    } else if command.starts_with("LM") {
        capture_mirror_command(command, line, report);
    } else if command.starts_with("LR") {
        capture_rotation_command(command, line, report);
    } else if command.starts_with("LS") {
        capture_scale_command(command, line, report);
    } else if command.starts_with("SR") {
        capture_step_repeat_command(command, line, open_step_repeat_line, report);
    } else if command.starts_with("ADD") {
        capture_aperture_definition(command, line, report);
    } else if command.starts_with("TF.") {
        capture_tf_attribute(command, line, report);
    } else if command.starts_with("TA.AperFunction") {
        capture_aper_function(command, line, report);
    } else if command.starts_with("TO.") {
        capture_object_attribute(command, line, report);
    } else if command.starts_with("TD") {
        capture_attribute_delete(command, line, report);
    }
}

fn capture_attribute_delete(command: &str, line: usize, report: &mut GerberMetadataReport) {
    let Some(value) = command.strip_prefix("TD") else {
        return;
    };

    let target = if value.is_empty() {
        GerberAttributeDeleteTarget::All
    } else if valid_attribute_delete_target(value) {
        GerberAttributeDeleteTarget::Named(value.to_string())
    } else {
        report.issues.push(GerberMetadataIssue {
            line,
            kind: GerberMetadataIssueKind::InvalidAttributeDeleteValue {
                command: "TD".to_string(),
                value: value.to_string(),
            },
        });
        return;
    };

    // Ucamco Gerber Layer Format Specification, rev. 2024.05, defines `%TD*%`
    // and `%TD.<attribute>*%` as deletion commands for active attributes.
    // HyperDRC preserves the command as source evidence without mutating
    // earlier parsed declarations; future object-scope checks can replay the
    // stream when they need exact attribute lifetime semantics.
    report
        .attribute_deletes
        .push(GerberAttributeDelete { line, target });
}

fn capture_aperture_macro(command: &str, line: usize, report: &mut GerberMetadataReport) {
    let Some(value) = command.strip_prefix("AM") else {
        return;
    };
    if value.is_empty() {
        report.issues.push(GerberMetadataIssue {
            line,
            kind: GerberMetadataIssueKind::MissingApertureMacroValue {
                command: "AM".to_string(),
            },
        });
        return;
    }

    let Some(macro_definition) = parse_aperture_macro_value(value, line) else {
        report.issues.push(GerberMetadataIssue {
            line,
            kind: GerberMetadataIssueKind::InvalidApertureMacroValue {
                command: "AM".to_string(),
                value: value.to_string(),
            },
        });
        return;
    };

    if let Some(existing) = report
        .aperture_macros
        .iter()
        .find(|existing| existing.name == macro_definition.name)
    {
        let kind = if existing.body == macro_definition.body {
            GerberMetadataIssueKind::DuplicateApertureMacro {
                name: macro_definition.name,
            }
        } else {
            GerberMetadataIssueKind::ConflictingApertureMacro {
                name: macro_definition.name.clone(),
                first: aperture_macro_label(existing),
                duplicate: aperture_macro_label(&macro_definition),
            }
        };
        report.issues.push(GerberMetadataIssue { line, kind });
        return;
    }

    // Ucamco Gerber Layer Format Specification, rev. 2024.05, section 4.5
    // defines aperture macros as reusable aperture templates. HyperDRC stores
    // the body as source evidence instead of expanding primitives here because
    // geometry expansion belongs to the Gerber image parser, while package
    // readiness needs to know that custom macro apertures were present.
    report.aperture_macros.push(macro_definition);
}

fn capture_step_repeat_command(
    command: &str,
    line: usize,
    open_step_repeat_line: &mut Option<usize>,
    report: &mut GerberMetadataReport,
) {
    let Some(value) = command.strip_prefix("SR") else {
        return;
    };

    if value.is_empty() {
        if open_step_repeat_line.take().is_none() {
            report.issues.push(GerberMetadataIssue {
                line,
                kind: GerberMetadataIssueKind::UnmatchedStepRepeatEnd,
            });
        }
        report.step_repeat_events.push(GerberStepRepeatEvent {
            line,
            kind: GerberStepRepeatEventKind::End,
        });
        return;
    }

    let Some((x_repeats, y_repeats, x_step, y_step)) = parse_step_repeat_value(value) else {
        report.issues.push(GerberMetadataIssue {
            line,
            kind: GerberMetadataIssueKind::InvalidStepRepeatCommandValue {
                command: "SR".to_string(),
                value: value.to_string(),
            },
        });
        return;
    };

    // Ucamco Gerber Layer Format Specification, rev. 2024.05, defines
    // `%SRX<n>Y<n>I<n>J<n>*%` as a stateful step-and-repeat block. Preserving
    // the transition is important because downstream polygon evidence has
    // already lost whether repeated copper came from explicit objects or a
    // compact repeat command in the manufacturing image.
    if let Some(open_line) = *open_step_repeat_line {
        report.issues.push(GerberMetadataIssue {
            line,
            kind: GerberMetadataIssueKind::NestedStepRepeat { open_line },
        });
    } else {
        *open_step_repeat_line = Some(line);
    }
    report.step_repeat_events.push(GerberStepRepeatEvent {
        line,
        kind: GerberStepRepeatEventKind::Start {
            x_repeats,
            y_repeats,
            x_step,
            y_step,
        },
    });
}

fn capture_unit_command(command: &str, line: usize, report: &mut GerberMetadataReport) {
    let Some(value) = command.strip_prefix("MO") else {
        return;
    };
    if value.is_empty() {
        report.issues.push(GerberMetadataIssue {
            line,
            kind: GerberMetadataIssueKind::MissingImageCommandValue {
                command: "MO".to_string(),
            },
        });
        return;
    }

    let units = match value {
        "MM" => Some(GerberUnits::Millimeters),
        "IN" => Some(GerberUnits::Inches),
        _ => None,
    };
    let Some(units) = units else {
        report.issues.push(GerberMetadataIssue {
            line,
            kind: GerberMetadataIssueKind::InvalidImageCommandValue {
                command: "MO".to_string(),
                value: value.to_string(),
            },
        });
        return;
    };
    insert_image_command(
        &mut report.image_setup.units,
        "MO",
        units,
        value.to_string(),
        line,
        &mut report.issues,
    );
}

fn capture_format_command(command: &str, line: usize, report: &mut GerberMetadataReport) {
    let Some(value) = command.strip_prefix("FS") else {
        return;
    };
    if value.is_empty() {
        report.issues.push(GerberMetadataIssue {
            line,
            kind: GerberMetadataIssueKind::MissingImageCommandValue {
                command: "FS".to_string(),
            },
        });
        return;
    }

    let Some(format) = parse_coordinate_format(value) else {
        report.issues.push(GerberMetadataIssue {
            line,
            kind: GerberMetadataIssueKind::InvalidImageCommandValue {
                command: "FS".to_string(),
                value: value.to_string(),
            },
        });
        return;
    };
    insert_image_command(
        &mut report.image_setup.coordinate_format,
        "FS",
        format,
        value.to_string(),
        line,
        &mut report.issues,
    );
}

fn capture_polarity_command(command: &str, line: usize, report: &mut GerberMetadataReport) {
    let Some(value) = command.strip_prefix("LP") else {
        return;
    };
    if value.is_empty() {
        report.issues.push(GerberMetadataIssue {
            line,
            kind: GerberMetadataIssueKind::MissingPolarityCommandValue {
                command: "LP".to_string(),
            },
        });
        return;
    }

    let polarity = match value {
        "D" => Some(GerberImagePolarity::Dark),
        "C" => Some(GerberImagePolarity::Clear),
        _ => None,
    };
    let Some(polarity) = polarity else {
        report.issues.push(GerberMetadataIssue {
            line,
            kind: GerberMetadataIssueKind::InvalidPolarityCommandValue {
                command: "LP".to_string(),
                value: value.to_string(),
            },
        });
        return;
    };

    // Ucamco Gerber Layer Format Specification, rev. 2024.05, defines image
    // polarity as a stateful dark/clear command. Preserving every transition is
    // useful because flattened geometry no longer shows whether a region came
    // from additive copper or subtractive clear polarity.
    report
        .polarity_changes
        .push(GerberPolarityChange { line, polarity });
}

fn capture_mirror_command(command: &str, line: usize, report: &mut GerberMetadataReport) {
    let Some(value) = command.strip_prefix("LM") else {
        return;
    };
    if value.is_empty() {
        report.issues.push(GerberMetadataIssue {
            line,
            kind: GerberMetadataIssueKind::MissingImageCommandValue {
                command: "LM".to_string(),
            },
        });
        return;
    }

    let mode = match value {
        "N" => Some(GerberMirrorMode::None),
        "X" => Some(GerberMirrorMode::X),
        "Y" => Some(GerberMirrorMode::Y),
        "XY" => Some(GerberMirrorMode::XY),
        _ => None,
    };
    let Some(mode) = mode else {
        report.issues.push(GerberMetadataIssue {
            line,
            kind: GerberMetadataIssueKind::InvalidImageCommandValue {
                command: "LM".to_string(),
                value: value.to_string(),
            },
        });
        return;
    };

    // Ucamco Gerber Layer Format Specification, rev. 2024.05, defines `%LM`,
    // `%LR`, and `%LS` as stateful image transformation commands. HyperDRC
    // preserves them as source evidence because downstream flattened geometry
    // generally cannot explain that an object was produced under mirror,
    // rotation, or scale state.
    report.image_transforms.push(GerberImageTransform {
        line,
        kind: GerberImageTransformKind::Mirror(mode),
    });
}

fn capture_rotation_command(command: &str, line: usize, report: &mut GerberMetadataReport) {
    let Some(value) = command.strip_prefix("LR") else {
        return;
    };
    let Some(rotation) = parse_finite_f64(value) else {
        let kind = if value.is_empty() {
            GerberMetadataIssueKind::MissingImageCommandValue {
                command: "LR".to_string(),
            }
        } else {
            GerberMetadataIssueKind::InvalidImageCommandValue {
                command: "LR".to_string(),
                value: value.to_string(),
            }
        };
        report.issues.push(GerberMetadataIssue { line, kind });
        return;
    };

    report.image_transforms.push(GerberImageTransform {
        line,
        kind: GerberImageTransformKind::Rotation(rotation),
    });
}

fn capture_scale_command(command: &str, line: usize, report: &mut GerberMetadataReport) {
    let Some(value) = command.strip_prefix("LS") else {
        return;
    };
    let Some(scale) = parse_finite_f64(value).filter(|scale| *scale > 0.0) else {
        let kind = if value.is_empty() {
            GerberMetadataIssueKind::MissingImageCommandValue {
                command: "LS".to_string(),
            }
        } else {
            GerberMetadataIssueKind::InvalidImageCommandValue {
                command: "LS".to_string(),
                value: value.to_string(),
            }
        };
        report.issues.push(GerberMetadataIssue { line, kind });
        return;
    };

    report.image_transforms.push(GerberImageTransform {
        line,
        kind: GerberImageTransformKind::Scale(scale),
    });
}

fn capture_aperture_definition(command: &str, line: usize, report: &mut GerberMetadataReport) {
    let Some(value) = command.strip_prefix("ADD") else {
        return;
    };
    if value.is_empty() {
        report.issues.push(GerberMetadataIssue {
            line,
            kind: GerberMetadataIssueKind::MissingApertureDefinitionValue {
                command: "ADD".to_string(),
            },
        });
        return;
    }

    let Some(definition) = parse_aperture_definition_value(value, line) else {
        report.issues.push(GerberMetadataIssue {
            line,
            kind: GerberMetadataIssueKind::InvalidApertureDefinitionValue {
                command: "ADD".to_string(),
                value: value.to_string(),
            },
        });
        return;
    };

    if let Some(existing) = report
        .aperture_definitions
        .iter()
        .find(|existing| existing.d_code == definition.d_code)
    {
        let kind = if same_aperture_definition(existing, &definition) {
            GerberMetadataIssueKind::DuplicateApertureDefinition {
                d_code: definition.d_code,
            }
        } else {
            GerberMetadataIssueKind::ConflictingApertureDefinition {
                d_code: definition.d_code,
                first: aperture_definition_label(existing),
                duplicate: aperture_definition_label(&definition),
            }
        };
        report.issues.push(GerberMetadataIssue { line, kind });
        return;
    }

    report.aperture_definitions.push(definition);
}

fn capture_tf_attribute(command: &str, line: usize, report: &mut GerberMetadataReport) {
    if command.starts_with("TF.Part") {
        capture_required_attribute(command, "TF.Part", line, report);
    } else if command.starts_with("TF.FileFunction") {
        capture_required_attribute(command, "TF.FileFunction", line, report);
    } else if command.starts_with("TF.FilePolarity") {
        capture_required_attribute(command, "TF.FilePolarity", line, report);
    } else if command.starts_with("TF.CreationDate") {
        capture_required_attribute(command, "TF.CreationDate", line, report);
    } else if command.starts_with("TF.GenerationSoftware") {
        capture_required_attribute(command, "TF.GenerationSoftware", line, report);
    } else if command.starts_with("TF.ProjectId") {
        capture_required_attribute(command, "TF.ProjectId", line, report);
    } else if command.starts_with("TF.MD5") {
        capture_required_attribute(command, "TF.MD5", line, report);
    } else if command.starts_with("TF.SameCoordinates")
        && let Some(value) = optional_attribute_value(command, "TF.SameCoordinates")
    {
        insert_attribute(
            &mut report.metadata.same_coordinates,
            "TF.SameCoordinates",
            value,
            line,
            &mut report.issues,
        );
    }
}

fn capture_aper_function(command: &str, line: usize, report: &mut GerberMetadataReport) {
    let Some(value) = attribute_value(command, "TA.AperFunction") else {
        report.issues.push(GerberMetadataIssue {
            line,
            kind: GerberMetadataIssueKind::MissingApertureAttributeValue {
                attribute: "TA.AperFunction".to_string(),
            },
        });
        return;
    };

    if !valid_aper_function_value(&value) {
        report.issues.push(GerberMetadataIssue {
            line,
            kind: GerberMetadataIssueKind::InvalidApertureAttributeValue {
                attribute: "TA.AperFunction".to_string(),
                value: value.clone(),
            },
        });
    }
    report.aperture_functions.push(GerberApertureMetadata {
        line,
        function: value,
    });
}

fn capture_object_attribute(command: &str, line: usize, report: &mut GerberMetadataReport) {
    if command.starts_with("TO.N") {
        capture_net_attribute(command, line, report);
    } else if command.starts_with("TO.C") {
        capture_component_attribute(command, line, report);
    } else if command.starts_with("TO.P") {
        capture_pin_attribute(command, line, report);
    }
}

fn capture_net_attribute(command: &str, line: usize, report: &mut GerberMetadataReport) {
    let Some(fields) = raw_attribute_fields(command, "TO.N") else {
        report.issues.push(GerberMetadataIssue {
            line,
            kind: GerberMetadataIssueKind::MissingObjectAttributeValue {
                attribute: "TO.N".to_string(),
            },
        });
        return;
    };

    let nets = fields
        .iter()
        .map(|field| field.trim().to_string())
        .collect::<Vec<_>>();
    if !valid_net_attribute_fields(&nets) {
        report.issues.push(GerberMetadataIssue {
            line,
            kind: GerberMetadataIssueKind::InvalidObjectAttributeValue {
                attribute: "TO.N".to_string(),
                value: fields.join(","),
            },
        });
    }
    report
        .object_attributes
        .push(GerberObjectMetadata::Net { line, nets });
}

fn capture_component_attribute(command: &str, line: usize, report: &mut GerberMetadataReport) {
    let Some(fields) = raw_attribute_fields(command, "TO.C") else {
        report.issues.push(GerberMetadataIssue {
            line,
            kind: GerberMetadataIssueKind::MissingObjectAttributeValue {
                attribute: "TO.C".to_string(),
            },
        });
        return;
    };

    let value = fields.join(",");
    let refdes = fields.first().map_or("", |field| field.trim());
    if fields.len() != 1 || refdes.is_empty() {
        report.issues.push(GerberMetadataIssue {
            line,
            kind: GerberMetadataIssueKind::InvalidObjectAttributeValue {
                attribute: "TO.C".to_string(),
                value,
            },
        });
        return;
    }
    report
        .object_attributes
        .push(GerberObjectMetadata::Component {
            line,
            refdes: refdes.to_string(),
        });
}

fn capture_pin_attribute(command: &str, line: usize, report: &mut GerberMetadataReport) {
    let Some(fields) = raw_attribute_fields(command, "TO.P") else {
        report.issues.push(GerberMetadataIssue {
            line,
            kind: GerberMetadataIssueKind::MissingObjectAttributeValue {
                attribute: "TO.P".to_string(),
            },
        });
        return;
    };

    let value = fields.join(",");
    let refdes = fields.first().map_or("", |field| field.trim());
    if !(fields.len() == 2 && !refdes.is_empty()) {
        report.issues.push(GerberMetadataIssue {
            line,
            kind: GerberMetadataIssueKind::InvalidObjectAttributeValue {
                attribute: "TO.P".to_string(),
                value,
            },
        });
        return;
    }

    let pin = fields[1].trim();
    report.object_attributes.push(GerberObjectMetadata::Pin {
        line,
        refdes: refdes.to_string(),
        pin: (!pin.is_empty()).then(|| pin.to_string()),
    });
}

fn capture_required_attribute(
    command: &str,
    attribute: &str,
    line: usize,
    report: &mut GerberMetadataReport,
) {
    let Some(value) = attribute_value(command, attribute) else {
        report.issues.push(GerberMetadataIssue {
            line,
            kind: GerberMetadataIssueKind::MissingFileAttributeValue {
                attribute: attribute.to_string(),
            },
        });
        return;
    };
    validate_standard_attribute_value(attribute, &value, line, &mut report.issues);
    let slot = match attribute {
        "TF.Part" => &mut report.metadata.part,
        "TF.FileFunction" => &mut report.metadata.file_function,
        "TF.FilePolarity" => &mut report.metadata.file_polarity,
        "TF.CreationDate" => &mut report.metadata.creation_date,
        "TF.GenerationSoftware" => &mut report.metadata.generation_software,
        "TF.ProjectId" => &mut report.metadata.project_id,
        "TF.MD5" => &mut report.metadata.md5,
        _ => return,
    };
    insert_attribute(slot, attribute, value, line, &mut report.issues);
}

fn validate_standard_attribute_value(
    attribute: &str,
    value: &str,
    line: usize,
    issues: &mut Vec<GerberMetadataIssue>,
) {
    let valid = match attribute {
        // Ucamco Gerber Layer Format Specification, rev. 2024.05, section
        // 5.6.2 defines the standard `.Part` values. `Other` must carry the
        // mandatory informal field so CAM review can understand the intent.
        "TF.Part" => valid_part_value(value),
        // Ucamco Gerber Layer Format Specification, rev. 2024.05, section
        // 5.6.3 defines `.FileFunction` as the layer-role attribute. HyperDRC
        // validates the role forms it consumes for package completeness:
        // Copper requires layer/side fields, while side-specific companion
        // layers such as Soldermask, Paste, and Legend require a Top/Bot side.
        // Less common functions are preserved without parser-level rejection
        // so future manifest checks can review them explicitly.
        "TF.FileFunction" => valid_file_function_value(value),
        // Ucamco Gerber Layer Format Specification, rev. 2024.05, section
        // 5.6.4 restricts `.FilePolarity` to Positive or Negative.
        "TF.FilePolarity" => valid_file_polarity_value(value),
        // Ucamco Gerber Layer Format Specification, rev. 2024.05, section
        // 5.6.6 defines `.CreationDate` as the file creation date and time.
        // We validate the ISO day prefix used by the manifest freshness checks
        // without attempting to interpret timezone offsets.
        "TF.CreationDate" => valid_creation_date_value(value),
        // Ucamco Gerber Layer Format Specification, rev. 2024.05, section
        // 5.6.7 gives `.GenerationSoftware` the exact
        // <vendor>,<application>,<version> syntax, which is the minimum needed
        // for release-package provenance review.
        "TF.GenerationSoftware" => valid_generation_software_value(value),
        // Ucamco Gerber Layer Format Specification, rev. 2024.05, section
        // 5.6.8 gives `.ProjectId` the <Name>,<GUID>,<Revision> syntax and
        // requires an RFC4122 version 1 or 4 GUID. We validate the canonical
        // hyphenated UUID form because that is what the standard examples use.
        "TF.ProjectId" => valid_project_id_value(value),
        // Ucamco Gerber Layer Format Specification, rev. 2024.05, section
        // 5.6.9 defines `.MD5` as a file signature/checksum. HyperDRC validates
        // the standard 128-bit hexadecimal digest syntax here; it deliberately
        // does not recompute the checksum because a new digest dependency would
        // be a separate integration decision.
        "TF.MD5" => valid_md5_value(value),
        _ => true,
    };
    if valid {
        return;
    }

    issues.push(GerberMetadataIssue {
        line,
        kind: GerberMetadataIssueKind::InvalidFileAttributeValue {
            attribute: attribute.to_string(),
            value: value.to_string(),
        },
    });
}

fn parse_coordinate_format(value: &str) -> Option<GerberCoordinateFormat> {
    // Ucamco Gerber Layer Format Specification, rev. 2024.05, section 4.2.2
    // defines the current coordinate format as FSLAX<n>6Y<n>6 with identical
    // X/Y digit pairs. Earlier decimal counts existed historically, but the
    // current parser diagnostic reports them as non-standard setup metadata so
    // release packages do not silently mix coordinate resolution conventions.
    let rest = value.strip_prefix("LAX")?;
    let (x_digits, y_digits) = rest.split_once('Y')?;
    if x_digits.len() != 2 || y_digits.len() != 2 || x_digits != y_digits {
        return None;
    }
    let mut digits = x_digits.bytes();
    let integer_digits = digits.next()?.checked_sub(b'0')?;
    let decimal_digits = digits.next()?.checked_sub(b'0')?;
    if !(1..=6).contains(&integer_digits) || decimal_digits != 6 {
        return None;
    }
    Some(GerberCoordinateFormat {
        integer_digits,
        decimal_digits,
    })
}

fn parse_step_repeat_value(value: &str) -> Option<(u32, u32, f64, f64)> {
    let fields = parse_letter_value_fields(value)?;
    let mut x_repeats = None;
    let mut y_repeats = None;
    let mut x_step = None;
    let mut y_step = None;

    for (letter, raw_value) in fields {
        match letter {
            'X' if x_repeats.is_none() => {
                x_repeats = parse_positive_u32(raw_value);
            }
            'Y' if y_repeats.is_none() => {
                y_repeats = parse_positive_u32(raw_value);
            }
            'I' if x_step.is_none() => {
                x_step = parse_non_negative_f64(raw_value);
            }
            'J' if y_step.is_none() => {
                y_step = parse_non_negative_f64(raw_value);
            }
            _ => return None,
        }
    }

    Some((x_repeats?, y_repeats?, x_step?, y_step?))
}

fn parse_letter_value_fields(value: &str) -> Option<Vec<(char, &str)>> {
    if value.is_empty() {
        return None;
    }
    let mut fields = Vec::new();
    let mut current_letter = None;
    let mut current_start = 0;

    for (index, ch) in value.char_indices() {
        if matches!(ch, 'X' | 'Y' | 'I' | 'J') {
            if let Some(letter) = current_letter {
                let raw_value = value.get(current_start..index)?;
                if raw_value.is_empty() {
                    return None;
                }
                fields.push((letter, raw_value));
            } else if index != 0 {
                return None;
            }
            current_letter = Some(ch);
            current_start = index + ch.len_utf8();
        }
    }

    let letter = current_letter?;
    let raw_value = value.get(current_start..)?;
    if raw_value.is_empty() {
        return None;
    }
    fields.push((letter, raw_value));
    Some(fields)
}

fn parse_positive_u32(value: &str) -> Option<u32> {
    (!value.is_empty() && value.bytes().all(|byte| byte.is_ascii_digit()))
        .then(|| value.parse::<u32>().ok())
        .flatten()
        .filter(|value| *value > 0)
}

fn parse_non_negative_f64(value: &str) -> Option<f64> {
    let parsed = value.parse::<f64>().ok()?;
    (parsed.is_finite() && parsed >= 0.0).then_some(parsed)
}

fn parse_finite_f64(value: &str) -> Option<f64> {
    value
        .trim()
        .parse::<f64>()
        .ok()
        .filter(|value| value.is_finite())
}

fn parse_aperture_definition_value(value: &str, line: usize) -> Option<GerberApertureDefinition> {
    let dcode_digits = value
        .chars()
        .take_while(|ch| ch.is_ascii_digit())
        .collect::<String>();
    if dcode_digits.is_empty() {
        return None;
    }
    let d_code = dcode_digits.parse::<u32>().ok()?;
    if d_code < 10 {
        return None;
    }

    let rest = &value[dcode_digits.len()..];
    let template_end = rest
        .find(',')
        .or_else(|| rest.find('X'))
        .unwrap_or(rest.len());
    let template = rest.get(..template_end)?.trim();
    if !valid_aperture_template(template) {
        return None;
    }
    let parameters = rest
        .get(template_end..)
        .and_then(|tail| tail.strip_prefix(',').or_else(|| tail.strip_prefix('X')))
        .map(str::trim)
        .filter(|tail| !tail.is_empty())
        .map(str::to_string);

    if !valid_aperture_parameters(template, parameters.as_deref()) {
        return None;
    }

    Some(GerberApertureDefinition {
        line,
        d_code,
        template: template.to_string(),
        parameters,
    })
}

fn parse_aperture_macro_value(value: &str, line: usize) -> Option<GerberApertureMacro> {
    let (name, body) = value.split_once('*')?;
    let name = name.trim();
    let body = body.trim().trim_end_matches('*').trim();
    if !valid_aperture_template(name) || body.is_empty() {
        return None;
    }
    let primitive_count = body
        .split('*')
        .map(str::trim)
        .filter(|primitive| !primitive.is_empty() && !primitive.starts_with("0 "))
        .count();
    if primitive_count == 0 || !valid_aperture_macro_body(body) {
        return None;
    }

    Some(GerberApertureMacro {
        line,
        name: name.to_string(),
        body: body.to_string(),
        primitive_count,
    })
}

fn valid_aperture_macro_body(body: &str) -> bool {
    body.split('*')
        .map(str::trim)
        .filter(|primitive| !primitive.is_empty())
        .all(|primitive| {
            primitive.starts_with('$')
                || primitive
                    .split_once(',')
                    .is_some_and(|(code, _)| valid_macro_primitive_code(code.trim()))
        })
}

fn valid_macro_primitive_code(code: &str) -> bool {
    matches!(code, "0" | "1" | "4" | "5" | "6" | "7" | "20" | "21" | "22")
}

fn valid_aperture_template(template: &str) -> bool {
    !template.is_empty()
        && template
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || byte == b'_' || byte == b'.')
}

fn valid_aperture_parameters(template: &str, parameters: Option<&str>) -> bool {
    match template {
        // Ucamco Gerber Layer Format Specification, rev. 2024.05, section 4.4
        // defines the standard C/R/O/P aperture templates and their parameter
        // counts. Macro templates are preserved without semantic validation
        // because their body can appear elsewhere in the same image stream.
        "C" => {
            let Some(values) = parse_aperture_numeric_parameters(parameters) else {
                return false;
            };
            (values.len() == 1 || values.len() == 2)
                && values[0] >= 0.0
                && optional_positive(values.get(1))
        }
        "R" | "O" => {
            let Some(values) = parse_aperture_numeric_parameters(parameters) else {
                return false;
            };
            (values.len() == 2 || values.len() == 3)
                && values[0] >= 0.0
                && values[1] >= 0.0
                && optional_positive(values.get(2))
        }
        "P" => {
            let Some(values) = parse_aperture_numeric_parameters(parameters) else {
                return false;
            };
            (values.len() == 2 || values.len() == 3 || values.len() == 4)
                && values[0] >= 0.0
                && values[1] >= 3.0
                && values[1].fract().abs() <= f64::EPSILON
                && optional_positive(values.get(3))
        }
        _ => true,
    }
}

fn parse_aperture_numeric_parameters(parameters: Option<&str>) -> Option<Vec<f64>> {
    parameters?
        .split('X')
        .map(str::trim)
        .map(|field| {
            if field.is_empty() {
                return None;
            }
            field.parse::<f64>().ok().filter(|value| value.is_finite())
        })
        .collect()
}

fn optional_positive(value: Option<&f64>) -> bool {
    value.is_none_or(|value| *value > 0.0)
}

fn same_aperture_definition(
    first: &GerberApertureDefinition,
    second: &GerberApertureDefinition,
) -> bool {
    first.template == second.template && first.parameters == second.parameters
}

fn aperture_definition_label(definition: &GerberApertureDefinition) -> String {
    match &definition.parameters {
        Some(parameters) => format!("{},{}", definition.template, parameters),
        None => definition.template.clone(),
    }
}

fn aperture_macro_label(macro_definition: &GerberApertureMacro) -> String {
    format!(
        "{} primitive(s): {}",
        macro_definition.primitive_count, macro_definition.body
    )
}

fn valid_file_polarity_value(value: &str) -> bool {
    matches!(
        value.trim().to_ascii_lowercase().as_str(),
        "positive" | "negative"
    )
}

fn valid_aper_function_value(value: &str) -> bool {
    let fields = non_empty_fields(value);
    let Some(function) = fields.first().map(|field| normalized_token(field)) else {
        return false;
    };

    match function.as_str() {
        // Ucamco Gerber Layer Format Specification, rev. 2024.05, section
        // 5.6.10 lists `.AperFunction` values. HyperDRC validates the common
        // copper-pad, drill, and material forms whose mandatory qualifiers are
        // easy to check without layer context, while preserving less common
        // future values as parser evidence.
        "smdpad" | "bgapad" => {
            fields.len() == 2 && matches!(normalized_token(fields[1]).as_str(), "cudef" | "smdef")
        }
        "otherdrill" | "other" => fields.len() > 1,
        "viadrill" => {
            fields.len() <= 2
                && fields
                    .get(1)
                    .is_none_or(|field| valid_via_protection_code(field))
        }
        "componentdrill" => {
            fields.len() <= 2
                && fields
                    .get(1)
                    .is_none_or(|field| normalized_token(field) == "pressfit")
        }
        _ => true,
    }
}

fn valid_via_protection_code(value: &str) -> bool {
    matches!(
        normalized_token(value).as_str(),
        "ia" | "ib" | "iia" | "iib" | "iiia" | "iiib" | "iva" | "ivb" | "v" | "vi" | "vii" | "none"
    )
}

fn valid_net_attribute_fields(nets: &[String]) -> bool {
    if nets.is_empty() {
        return false;
    }
    // Ucamco Gerber Layer Format Specification, rev. 2024.05, section
    // 5.6.13 defines `.N` as one or more CAD net names. The empty net name is
    // reserved for unconnected objects and `N/C` is reserved for single-pad
    // not-connected nets, so an empty field is valid only by itself.
    nets.len() == 1 || nets.iter().all(|net| !net.trim().is_empty())
}

fn valid_attribute_delete_target(value: &str) -> bool {
    let value = value.trim();
    value.starts_with('.')
        && value.len() > 1
        && value
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'.' | b'_' | b'-'))
}

fn valid_file_function_value(value: &str) -> bool {
    let fields = non_empty_fields(value);
    let Some(function) = fields.first().map(|field| normalized_token(field)) else {
        return false;
    };

    match function.as_str() {
        "copper" => {
            fields.len() >= 3
                && valid_file_function_layer(fields[1])
                && valid_file_function_side(fields[2], true)
        }
        "soldermask" | "paste" | "legend" => {
            fields.len() == 2 && valid_file_function_side(fields[1], false)
        }
        // Profile layers are enough to identify the board outline for manifest
        // readiness. Some exporters add plating/rout qualifiers, so only the
        // primary function token is enforced here.
        "profile" => true,
        _ => true,
    }
}

fn valid_file_function_layer(value: &str) -> bool {
    let value = value.trim();
    let Some(layer_number) = value.strip_prefix('L').or_else(|| value.strip_prefix('l')) else {
        return false;
    };
    !layer_number.is_empty() && layer_number.bytes().all(|byte| byte.is_ascii_digit())
}

fn valid_file_function_side(value: &str, allow_inner: bool) -> bool {
    match normalized_token(value).as_str() {
        "top" | "bot" => true,
        "inr" => allow_inner,
        _ => false,
    }
}

fn valid_part_value(value: &str) -> bool {
    let fields = value
        .split(',')
        .map(str::trim)
        .filter(|field| !field.is_empty())
        .collect::<Vec<_>>();
    let Some(kind) = fields.first() else {
        return false;
    };
    match kind.to_ascii_lowercase().as_str() {
        "single" | "array" | "fabricationpanel" | "coupon" => true,
        "other" => fields.len() > 1,
        _ => false,
    }
}

fn normalized_token(value: &str) -> String {
    value
        .trim()
        .chars()
        .filter(|ch| !ch.is_ascii_whitespace() && *ch != '-' && *ch != '_')
        .collect::<String>()
        .to_ascii_lowercase()
}

fn valid_creation_date_value(value: &str) -> bool {
    let value = value.trim();
    let Some(date) = value.get(0..10) else {
        return false;
    };
    parse_iso_day(date).is_some() && (value.len() == 10 || value.as_bytes().get(10) == Some(&b'T'))
}

fn valid_generation_software_value(value: &str) -> bool {
    non_empty_fields(value).len() == 3
}

fn valid_project_id_value(value: &str) -> bool {
    let fields = non_empty_fields(value);
    fields.len() == 3 && valid_rfc4122_v1_or_v4_guid(fields[1])
}

fn non_empty_fields(value: &str) -> Vec<&str> {
    value
        .split(',')
        .map(str::trim)
        .filter(|field| !field.is_empty())
        .collect()
}

fn valid_rfc4122_v1_or_v4_guid(value: &str) -> bool {
    let bytes = value.as_bytes();
    if bytes.len() != 36
        || bytes[8] != b'-'
        || bytes[13] != b'-'
        || bytes[18] != b'-'
        || bytes[23] != b'-'
    {
        return false;
    }
    if !bytes
        .iter()
        .enumerate()
        .filter(|(index, _)| !matches!(index, 8 | 13 | 18 | 23))
        .all(|(_, byte)| byte.is_ascii_hexdigit())
    {
        return false;
    }

    matches!(bytes[14], b'1' | b'4')
        && matches!(bytes[19].to_ascii_lowercase(), b'8' | b'9' | b'a' | b'b')
}

fn valid_md5_value(value: &str) -> bool {
    let value = value.trim();
    value.len() == 32 && value.bytes().all(|byte| byte.is_ascii_hexdigit())
}

fn insert_attribute(
    slot: &mut Option<String>,
    attribute: &str,
    value: String,
    line: usize,
    issues: &mut Vec<GerberMetadataIssue>,
) {
    let Some(first) = slot.as_ref() else {
        *slot = Some(value);
        return;
    };

    let kind = if first == &value {
        GerberMetadataIssueKind::DuplicateFileAttribute {
            attribute: attribute.to_string(),
            value,
        }
    } else {
        GerberMetadataIssueKind::ConflictingFileAttribute {
            attribute: attribute.to_string(),
            first: first.clone(),
            duplicate: value,
        }
    };
    issues.push(GerberMetadataIssue { line, kind });
}

fn insert_image_command<T: Copy + Debug + Eq>(
    slot: &mut Option<T>,
    command: &str,
    value: T,
    raw_value: String,
    line: usize,
    issues: &mut Vec<GerberMetadataIssue>,
) {
    let Some(first) = slot.as_ref() else {
        *slot = Some(value);
        return;
    };

    let kind = if first == &value {
        GerberMetadataIssueKind::DuplicateImageCommand {
            command: command.to_string(),
            value: raw_value,
        }
    } else {
        GerberMetadataIssueKind::ConflictingImageCommand {
            command: command.to_string(),
            first: format!("{first:?}"),
            duplicate: raw_value,
        }
    };
    issues.push(GerberMetadataIssue { line, kind });
}

fn attribute_value(command: &str, name: &str) -> Option<String> {
    let rest = command.strip_prefix(name)?;
    let rest = rest.strip_prefix(',')?;
    let value = rest
        .split(',')
        .map(str::trim)
        .filter(|field| !field.is_empty())
        .collect::<Vec<_>>()
        .join(",");
    (!value.is_empty()).then_some(value)
}

fn optional_attribute_value(command: &str, name: &str) -> Option<String> {
    let rest = command.strip_prefix(name)?;
    if rest.is_empty() {
        return Some(String::new());
    }
    let rest = rest.strip_prefix(',')?;
    Some(
        rest.split(',')
            .map(str::trim)
            .filter(|field| !field.is_empty())
            .collect::<Vec<_>>()
            .join(","),
    )
}

fn raw_attribute_fields(command: &str, name: &str) -> Option<Vec<String>> {
    let rest = command.strip_prefix(name)?;
    let rest = rest.strip_prefix(',')?;
    Some(
        rest.split(',')
            .map(|field| field.trim().to_string())
            .collect(),
    )
}

#[cfg(test)]
mod tests {
    use super::{
        GerberApertureUseKind, GerberAttributeDeleteTarget, GerberCoordinateFormat,
        GerberCoordinateOperationKind, GerberImagePolarity, GerberImageTransformKind,
        GerberInterpolationMode, GerberLayerMetadata, GerberMetadataIssueKind, GerberMirrorMode,
        GerberObjectMetadata, GerberQuadrantMode, GerberRegionEventKind, GerberStepRepeatEventKind,
        GerberUnits, parse_gerber_metadata, parse_gerber_metadata_report,
    };

    #[test]
    fn extracts_x2_file_function_and_file_polarity() {
        let metadata = parse_gerber_metadata(
            b"G04 header*\n%TF.FileFunction,Copper,L4,Bot,Signal*%\n%TF.FilePolarity,Positive*%\nM02*\n",
        );

        assert_eq!(
            metadata,
            GerberLayerMetadata {
                part: None,
                file_function: Some("Copper,L4,Bot,Signal".to_string()),
                file_polarity: Some("Positive".to_string()),
                same_coordinates: None,
                creation_date: None,
                generation_software: None,
                project_id: None,
                md5: None,
            }
        );
    }

    #[test]
    fn extracts_standardized_comment_attributes_for_legacy_exports() {
        let metadata = parse_gerber_metadata(
            b"G04 #@! TF.Part,Array*\nG04 #@! TF.FileFunction,Soldermask,Top*\nG04 #@! TF.FilePolarity,Negative*\nG04 #@! TF.SameCoordinates,PX1*\nG04 #@! TF.CreationDate,2026-05-16T12:00:00Z*\nG04 #@! TF.GenerationSoftware,KiCad,KiCad,9.0*\nG04 #@! TF.ProjectId,Widget,550e8400-e29b-41d4-a716-446655440000,A*\nG04 #@! TF.MD5,d41d8cd98f00b204e9800998ecf8427e*\n",
        );

        assert_eq!(metadata.part.as_deref(), Some("Array"));
        assert_eq!(metadata.file_function.as_deref(), Some("Soldermask,Top"));
        assert_eq!(metadata.file_polarity.as_deref(), Some("Negative"));
        assert_eq!(metadata.same_coordinates.as_deref(), Some("PX1"));
        assert_eq!(
            metadata.creation_date.as_deref(),
            Some("2026-05-16T12:00:00Z")
        );
        assert_eq!(
            metadata.generation_software.as_deref(),
            Some("KiCad,KiCad,9.0")
        );
        assert_eq!(
            metadata.project_id.as_deref(),
            Some("Widget,550e8400-e29b-41d4-a716-446655440000,A")
        );
        assert_eq!(
            metadata.md5.as_deref(),
            Some("d41d8cd98f00b204e9800998ecf8427e")
        );
    }

    #[test]
    fn accepts_same_coordinates_without_identifier() {
        let metadata = parse_gerber_metadata(b"%TF.SameCoordinates*%\n");

        assert_eq!(metadata.same_coordinates.as_deref(), Some(""));
    }

    #[test]
    fn keeps_first_file_attribute_when_an_invalid_redefinition_appears() {
        let metadata = parse_gerber_metadata(
            b"%TF.FileFunction,Copper,L1,Top*%\n%TF.FileFunction,Copper,L2,Inr*%\n",
        );

        assert_eq!(metadata.file_function.as_deref(), Some("Copper,L1,Top"));
    }

    #[test]
    fn ignores_missing_values_in_layer_metadata_projection() {
        let metadata = parse_gerber_metadata(b"%TF.FileFunction*%\n%TA.AperFunction,Conductor*%\n");

        assert_eq!(metadata, GerberLayerMetadata::default());
    }

    #[test]
    fn extracts_and_validates_image_setup_commands() {
        let report = parse_gerber_metadata_report(
            b"%MOMM*%\n%FSLAX46Y46*%\n%MOIN*%\n%FSLAX45Y45*%\n%MOYD*%\n%FS*%\n%FSLAX36Y46*%\n",
        );

        assert_eq!(report.image_setup.units, Some(GerberUnits::Millimeters));
        assert_eq!(
            report.image_setup.coordinate_format,
            Some(GerberCoordinateFormat {
                integer_digits: 4,
                decimal_digits: 6
            })
        );
        assert!(report.issues.iter().any(|issue| {
            issue.line == 3
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::ConflictingImageCommand { command, duplicate, .. }
                        if command == "MO" && duplicate == "IN"
                )
        }));
        assert!(report.issues.iter().any(|issue| {
            issue.line == 4
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::InvalidImageCommandValue { command, value }
                        if command == "FS" && value == "LAX45Y45"
                )
        }));
        assert!(report.issues.iter().any(|issue| {
            issue.line == 5
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::InvalidImageCommandValue { command, value }
                        if command == "MO" && value == "YD"
                )
        }));
        assert!(report.issues.iter().any(|issue| {
            issue.line == 6
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::MissingImageCommandValue { command }
                        if command == "FS"
                )
        }));
        assert!(report.issues.iter().any(|issue| {
            issue.line == 7
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::InvalidImageCommandValue { command, value }
                        if command == "FS" && value == "LAX36Y46"
                )
        }));
    }

    #[test]
    fn reports_duplicate_image_setup_commands() {
        let report =
            parse_gerber_metadata_report(b"%MOMM*%\n%MOMM*%\n%FSLAX36Y36*%\n%FSLAX36Y36*%\n");

        assert_eq!(report.issues.len(), 2);
        assert!(matches!(
            &report.issues[0].kind,
            GerberMetadataIssueKind::DuplicateImageCommand { command, value }
                if command == "MO" && value == "MM"
        ));
        assert!(matches!(
            &report.issues[1].kind,
            GerberMetadataIssueKind::DuplicateImageCommand { command, value }
                if command == "FS" && value == "LAX36Y36"
        ));
    }

    #[test]
    fn extracts_and_validates_aperture_definitions() {
        let report = parse_gerber_metadata_report(
            b"%ADD10C,0.5*%\n%ADD11R,1.0X0.5X0.2*%\n%ADD12O,1.0X0.5*%\n%ADD13P,1.2X6X45X0.2*%\n%ADD14MACRO_NAME,1X2*%\n%ADD9C,0.5*%\n%ADD15R,1.0*%\n%ADD16P,1.0X2*%\n%ADD17C,0.5X0*%\n%ADD*%\n",
        );

        assert_eq!(report.aperture_definitions.len(), 5);
        assert_eq!(report.aperture_definitions[0].d_code, 10);
        assert_eq!(report.aperture_definitions[0].template, "C");
        assert_eq!(
            report.aperture_definitions[0].parameters.as_deref(),
            Some("0.5")
        );
        assert_eq!(report.aperture_definitions[4].template, "MACRO_NAME");
        assert!(report.issues.iter().any(|issue| {
            issue.line == 6
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::InvalidApertureDefinitionValue { command, value }
                        if command == "ADD" && value == "9C,0.5"
                )
        }));
        assert!(report.issues.iter().any(|issue| {
            issue.line == 7
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::InvalidApertureDefinitionValue { command, value }
                        if command == "ADD" && value == "15R,1.0"
                )
        }));
        assert!(report.issues.iter().any(|issue| {
            issue.line == 8
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::InvalidApertureDefinitionValue { command, value }
                        if command == "ADD" && value == "16P,1.0X2"
                )
        }));
        assert!(report.issues.iter().any(|issue| {
            issue.line == 9
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::InvalidApertureDefinitionValue { command, value }
                        if command == "ADD" && value == "17C,0.5X0"
                )
        }));
        assert!(report.issues.iter().any(|issue| {
            issue.line == 10
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::MissingApertureDefinitionValue { command }
                        if command == "ADD"
                )
        }));
    }

    #[test]
    fn extracts_and_validates_aperture_macros() {
        let report = parse_gerber_metadata_report(
            b"%AMTHERM*1,1,0.5,0,0,0*20,1,0.1,0,0,1,1,0*%\n%AMTHERM*1,1,0.5,0,0,0*20,1,0.1,0,0,1,1,0*%\n%AMTHERM*1,1,0.6,0,0,0*%\n%AM*%\n%AMBAD*99,1,2*%\n",
        );

        assert_eq!(report.aperture_macros.len(), 1);
        assert_eq!(report.aperture_macros[0].name, "THERM");
        assert_eq!(report.aperture_macros[0].primitive_count, 2);
        assert!(report.issues.iter().any(|issue| {
            issue.line == 2
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::DuplicateApertureMacro { name } if name == "THERM"
                )
        }));
        assert!(report.issues.iter().any(|issue| {
            issue.line == 3
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::ConflictingApertureMacro { name, .. }
                        if name == "THERM"
                )
        }));
        assert!(report.issues.iter().any(|issue| {
            issue.line == 4
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::MissingApertureMacroValue { command }
                        if command == "AM"
                )
        }));
        assert!(report.issues.iter().any(|issue| {
            issue.line == 5
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::InvalidApertureMacroValue { command, value }
                        if command == "AM" && value == "BAD*99,1,2"
                )
        }));
    }

    #[test]
    fn reports_duplicate_and_conflicting_aperture_definitions() {
        let report =
            parse_gerber_metadata_report(b"%ADD10C,0.5*%\n%ADD10C,0.5*%\n%ADD10R,0.5X0.5*%\n");

        assert_eq!(report.aperture_definitions.len(), 1);
        assert!(matches!(
            &report.issues[0].kind,
            GerberMetadataIssueKind::DuplicateApertureDefinition { d_code }
                if *d_code == 10
        ));
        assert!(matches!(
            &report.issues[1].kind,
            GerberMetadataIssueKind::ConflictingApertureDefinition {
                d_code,
                first,
                duplicate,
            } if *d_code == 10 && first == "C,0.5" && duplicate == "R,0.5X0.5"
        ));
    }

    #[test]
    fn extracts_aperture_selections_and_operations() {
        let report = parse_gerber_metadata_report(
            b"%ADD10C,0.5*%\nD10*\nX0Y0D02*\nX10Y0D01*\nX10Y10D03*\nD11*\n",
        );

        assert_eq!(
            report
                .aperture_uses
                .iter()
                .map(|use_record| (use_record.d_code, use_record.kind))
                .collect::<Vec<_>>(),
            vec![
                (10, GerberApertureUseKind::Select),
                (10, GerberApertureUseKind::Draw),
                (10, GerberApertureUseKind::Flash),
            ]
        );
        assert_eq!(
            report
                .coordinate_operations
                .iter()
                .map(|operation| (
                    operation.line,
                    operation.kind,
                    operation.x.as_deref(),
                    operation.y.as_deref(),
                ))
                .collect::<Vec<_>>(),
            vec![
                (3, GerberCoordinateOperationKind::Move, Some("0"), Some("0")),
                (
                    4,
                    GerberCoordinateOperationKind::Draw,
                    Some("10"),
                    Some("0")
                ),
                (
                    5,
                    GerberCoordinateOperationKind::Flash,
                    Some("10"),
                    Some("10")
                ),
            ]
        );
        assert!(report.issues.iter().any(|issue| {
            issue.line == 6
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::UndefinedApertureSelection { d_code }
                        if *d_code == 11
                )
        }));
    }

    #[test]
    fn reports_draw_or_flash_before_current_aperture() {
        let report = parse_gerber_metadata_report(b"X0Y0D03*\nX1Y1D01*\n");

        assert_eq!(report.aperture_uses.len(), 0);
        assert!(report.issues.iter().any(|issue| {
            issue.line == 1
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::MissingCurrentAperture { operation }
                        if operation == "D03"
                )
        }));
        assert!(report.issues.iter().any(|issue| {
            issue.line == 2
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::MissingCurrentAperture { operation }
                        if operation == "D01"
                )
        }));
    }

    #[test]
    fn extracts_and_validates_image_polarity_commands() {
        let report = parse_gerber_metadata_report(b"%LPD*%\n%LPC*%\n%LP*%\n%LPX*%\n");

        assert_eq!(
            report
                .polarity_changes
                .iter()
                .map(|change| change.polarity)
                .collect::<Vec<_>>(),
            vec![GerberImagePolarity::Dark, GerberImagePolarity::Clear]
        );
        assert!(report.issues.iter().any(|issue| {
            issue.line == 3
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::MissingPolarityCommandValue { command }
                        if command == "LP"
                )
        }));
        assert!(report.issues.iter().any(|issue| {
            issue.line == 4
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::InvalidPolarityCommandValue { command, value }
                        if command == "LP" && value == "X"
                )
        }));
    }

    #[test]
    fn extracts_and_validates_image_transform_commands() {
        let report = parse_gerber_metadata_report(
            b"%LMN*%\n%LMX*%\n%LMY*%\n%LMXY*%\n%LR90*%\n%LS0.5*%\n%LMZ*%\n%LRnan*%\n%LS0*%\n%LM*%\n",
        );

        assert_eq!(
            report
                .image_transforms
                .iter()
                .map(|transform| (transform.line, transform.kind.clone()))
                .collect::<Vec<_>>(),
            vec![
                (1, GerberImageTransformKind::Mirror(GerberMirrorMode::None)),
                (2, GerberImageTransformKind::Mirror(GerberMirrorMode::X)),
                (3, GerberImageTransformKind::Mirror(GerberMirrorMode::Y)),
                (4, GerberImageTransformKind::Mirror(GerberMirrorMode::XY)),
                (5, GerberImageTransformKind::Rotation(90.0)),
                (6, GerberImageTransformKind::Scale(0.5)),
            ]
        );
        assert!(report.issues.iter().any(|issue| {
            issue.line == 7
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::InvalidImageCommandValue { command, value }
                        if command == "LM" && value == "Z"
                )
        }));
        assert!(report.issues.iter().any(|issue| {
            issue.line == 8
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::InvalidImageCommandValue { command, value }
                        if command == "LR" && value == "nan"
                )
        }));
        assert!(report.issues.iter().any(|issue| {
            issue.line == 9
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::InvalidImageCommandValue { command, value }
                        if command == "LS" && value == "0"
                )
        }));
        assert!(report.issues.iter().any(|issue| {
            issue.line == 10
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::MissingImageCommandValue { command }
                        if command == "LM"
                )
        }));
    }

    #[test]
    fn extracts_and_validates_region_mode_commands() {
        let report =
            parse_gerber_metadata_report(b"G36*\nX0Y0D02*\nX10Y0D01*\nG37*\nG37*\nG36*\nG36*\n");

        assert_eq!(
            report
                .region_events
                .iter()
                .map(|event| (event.line, event.kind))
                .collect::<Vec<_>>(),
            vec![
                (1, GerberRegionEventKind::Start),
                (4, GerberRegionEventKind::End),
                (5, GerberRegionEventKind::End),
                (6, GerberRegionEventKind::Start),
                (7, GerberRegionEventKind::Start),
            ]
        );
        assert!(report.issues.iter().any(|issue| {
            issue.line == 5 && matches!(&issue.kind, GerberMetadataIssueKind::UnmatchedRegionEnd)
        }));
        assert!(report.issues.iter().any(|issue| {
            issue.line == 7
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::NestedRegion { open_line } if *open_line == 6
                )
        }));
        assert!(report.issues.iter().any(|issue| {
            issue.line == 6
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::UnterminatedRegion { open_line } if *open_line == 6
                )
        }));
    }

    #[test]
    fn extracts_interpolation_and_quadrant_mode_commands() {
        let report =
            parse_gerber_metadata_report(b"G01*\nG75*\nG02X1Y1I0J1D01*\nG03X0Y0I-1J0D01*\nG74*\n");

        assert_eq!(
            report
                .interpolation_events
                .iter()
                .map(|event| (event.line, event.mode))
                .collect::<Vec<_>>(),
            vec![
                (1, GerberInterpolationMode::Linear),
                (3, GerberInterpolationMode::ClockwiseCircular),
                (4, GerberInterpolationMode::CounterClockwiseCircular),
            ]
        );
        assert_eq!(
            report
                .quadrant_events
                .iter()
                .map(|event| (event.line, event.mode))
                .collect::<Vec<_>>(),
            vec![
                (2, GerberQuadrantMode::Multi),
                (5, GerberQuadrantMode::Single),
            ]
        );
        assert!(report.issues.iter().any(|issue| {
            issue.line == 3
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::MissingCurrentAperture { operation }
                        if operation == "D01"
                )
        }));
        let clockwise_arc = report
            .coordinate_operations
            .iter()
            .find(|operation| operation.line == 3)
            .expect("clockwise arc operation should be preserved");
        assert_eq!(clockwise_arc.i.as_deref(), Some("0"));
        assert_eq!(clockwise_arc.j.as_deref(), Some("1"));
    }

    #[test]
    fn extracts_and_validates_step_repeat_commands() {
        let report = parse_gerber_metadata_report(
            b"%SRX2Y3I1.25J2.5*%\n%SR*%\n%SR*%\n%SRX0Y1I0J0*%\n%SRX2Y2I0J0*%\n%SRX3Y3I1J1*%\n",
        );

        assert_eq!(report.step_repeat_events.len(), 5);
        assert!(matches!(
            &report.step_repeat_events[0].kind,
            GerberStepRepeatEventKind::Start {
                x_repeats,
                y_repeats,
                x_step,
                y_step,
            } if *x_repeats == 2
                && *y_repeats == 3
                && (*x_step - 1.25).abs() <= f64::EPSILON
                && (*y_step - 2.5).abs() <= f64::EPSILON
        ));
        assert!(matches!(
            &report.step_repeat_events[1].kind,
            GerberStepRepeatEventKind::End
        ));
        assert!(report.issues.iter().any(|issue| {
            issue.line == 3
                && matches!(&issue.kind, GerberMetadataIssueKind::UnmatchedStepRepeatEnd)
        }));
        assert!(report.issues.iter().any(|issue| {
            issue.line == 4
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::InvalidStepRepeatCommandValue { command, value }
                        if command == "SR" && value == "X0Y1I0J0"
                )
        }));
        assert!(report.issues.iter().any(|issue| {
            issue.line == 6
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::NestedStepRepeat { open_line } if *open_line == 5
                )
        }));
        assert!(report.issues.iter().any(|issue| {
            issue.line == 5
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::UnterminatedStepRepeat { open_line }
                        if *open_line == 5
                )
        }));
    }

    #[test]
    fn extracts_and_validates_aperture_function_attributes() {
        let report = parse_gerber_metadata_report(
            b"%TA.AperFunction,Conductor*%\n%TA.AperFunction,SMDPad,CuDef*%\nG04 #@! TA.AperFunction,ViaDrill,IVa*\n%TA.AperFunction,BGAPad*%\n%TA.AperFunction,OtherDrill*%\n%TA.AperFunction*%\n",
        );

        assert_eq!(
            report
                .aperture_functions
                .iter()
                .map(|function| function.function.as_str())
                .collect::<Vec<_>>(),
            vec![
                "Conductor",
                "SMDPad,CuDef",
                "ViaDrill,IVa",
                "BGAPad",
                "OtherDrill"
            ]
        );
        assert!(report.issues.iter().any(|issue| {
            issue.line == 4
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::InvalidApertureAttributeValue { attribute, value }
                        if attribute == "TA.AperFunction" && value == "BGAPad"
                )
        }));
        assert!(report.issues.iter().any(|issue| {
            issue.line == 5
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::InvalidApertureAttributeValue { attribute, value }
                        if attribute == "TA.AperFunction" && value == "OtherDrill"
                )
        }));
        assert!(report.issues.iter().any(|issue| {
            issue.line == 6
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::MissingApertureAttributeValue { attribute }
                        if attribute == "TA.AperFunction"
                )
        }));
    }

    #[test]
    fn extracts_and_validates_object_net_component_and_pin_attributes() {
        let report = parse_gerber_metadata_report(
            b"%TO.N,GND*%\n%TO.N,*%\n%TO.N,USB_P,USB_N*%\nG04 #@! TO.C,U3*\n%TO.P,U3,7*%\n%TO.P,U4,*%\n%TO.N,GND,*%\n%TO.C,*%\n%TO.P,,1*%\n%TO.N*%\n",
        );

        assert_eq!(
            report.object_attributes,
            vec![
                GerberObjectMetadata::Net {
                    line: 1,
                    nets: vec!["GND".to_string()]
                },
                GerberObjectMetadata::Net {
                    line: 2,
                    nets: vec!["".to_string()]
                },
                GerberObjectMetadata::Net {
                    line: 3,
                    nets: vec!["USB_P".to_string(), "USB_N".to_string()]
                },
                GerberObjectMetadata::Component {
                    line: 4,
                    refdes: "U3".to_string()
                },
                GerberObjectMetadata::Pin {
                    line: 5,
                    refdes: "U3".to_string(),
                    pin: Some("7".to_string())
                },
                GerberObjectMetadata::Pin {
                    line: 6,
                    refdes: "U4".to_string(),
                    pin: None
                },
                GerberObjectMetadata::Net {
                    line: 7,
                    nets: vec!["GND".to_string(), "".to_string()]
                },
            ]
        );
        assert!(report.issues.iter().any(|issue| {
            issue.line == 7
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::InvalidObjectAttributeValue { attribute, value }
                        if attribute == "TO.N" && value == "GND,"
                )
        }));
        assert!(report.issues.iter().any(|issue| {
            issue.line == 8
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::InvalidObjectAttributeValue { attribute, value }
                        if attribute == "TO.C" && value.is_empty()
                )
        }));
        assert!(report.issues.iter().any(|issue| {
            issue.line == 9
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::InvalidObjectAttributeValue { attribute, value }
                        if attribute == "TO.P" && value == ",1"
                )
        }));
        assert!(report.issues.iter().any(|issue| {
            issue.line == 10
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::MissingObjectAttributeValue { attribute }
                        if attribute == "TO.N"
                )
        }));
    }

    #[test]
    fn extracts_and_validates_attribute_delete_commands() {
        let report =
            parse_gerber_metadata_report(b"%TD*%\n%TD.N*%\nG04 #@! TD.C\n%TDN*%\n%TD.*%\n");

        assert_eq!(
            report
                .attribute_deletes
                .iter()
                .map(|delete| (delete.line, delete.target.clone()))
                .collect::<Vec<_>>(),
            vec![
                (1, GerberAttributeDeleteTarget::All),
                (2, GerberAttributeDeleteTarget::Named(".N".to_string())),
                (3, GerberAttributeDeleteTarget::Named(".C".to_string())),
            ]
        );
        assert!(report.issues.iter().any(|issue| {
            issue.line == 4
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::InvalidAttributeDeleteValue { command, value }
                        if command == "TD" && value == "N"
                )
        }));
        assert!(report.issues.iter().any(|issue| {
            issue.line == 5
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::InvalidAttributeDeleteValue { command, value }
                        if command == "TD" && value == "."
                )
        }));
        assert!(
            report.issues.iter().any(|issue| {
                issue
                    .message()
                    .contains("attribute-delete command TD has non-standard value")
            }),
            "invalid attribute delete should have a human-readable diagnostic"
        );
    }

    #[test]
    fn reports_missing_required_attribute_values() {
        let report = parse_gerber_metadata_report(
            b"%TF.Part*%\n%TF.FileFunction*%\n%TF.FilePolarity,%\n%TF.CreationDate*%\n%TF.GenerationSoftware,%\n%TF.ProjectId,%\n%TF.MD5,%\n",
        );

        assert_eq!(report.metadata, GerberLayerMetadata::default());
        assert_eq!(report.issues.len(), 7);
        assert!(report.issues.iter().any(|issue| {
            issue.line == 1
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::MissingFileAttributeValue { attribute }
                        if attribute == "TF.Part"
                )
        }));
        assert!(report.issues.iter().any(|issue| {
            issue.line == 2
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::MissingFileAttributeValue { attribute }
                        if attribute == "TF.FileFunction"
                )
        }));
        assert!(report.issues.iter().any(|issue| {
            issue.line == 3
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::MissingFileAttributeValue { attribute }
                        if attribute == "TF.FilePolarity"
                )
        }));
        assert!(report.issues.iter().any(|issue| {
            issue.line == 4
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::MissingFileAttributeValue { attribute }
                        if attribute == "TF.CreationDate"
                )
        }));
        assert!(report.issues.iter().any(|issue| {
            issue.line == 5
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::MissingFileAttributeValue { attribute }
                        if attribute == "TF.GenerationSoftware"
                )
        }));
        assert!(report.issues.iter().any(|issue| {
            issue.line == 6
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::MissingFileAttributeValue { attribute }
                        if attribute == "TF.ProjectId"
                )
        }));
        assert!(report.issues.iter().any(|issue| {
            issue.line == 7
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::MissingFileAttributeValue { attribute }
                        if attribute == "TF.MD5"
                )
        }));
    }

    #[test]
    fn reports_duplicate_and_conflicting_redefinitions() {
        let report = parse_gerber_metadata_report(
            b"%TF.FileFunction,Copper,L1,Top*%\n%TF.FileFunction,Copper,L1,Top*%\n%TF.FileFunction,Copper,L2,Inr*%\n",
        );

        assert_eq!(
            report.metadata.file_function.as_deref(),
            Some("Copper,L1,Top")
        );
        assert_eq!(report.issues.len(), 2);
        assert!(matches!(
            &report.issues[0].kind,
            GerberMetadataIssueKind::DuplicateFileAttribute { attribute, value }
                if attribute == "TF.FileFunction" && value == "Copper,L1,Top"
        ));
        assert!(matches!(
            &report.issues[1].kind,
            GerberMetadataIssueKind::ConflictingFileAttribute {
                attribute,
                first,
                duplicate,
            } if attribute == "TF.FileFunction"
                && first == "Copper,L1,Top"
                && duplicate == "Copper,L2,Inr"
        ));
    }

    #[test]
    fn reports_non_standard_file_attributes_without_dropping_metadata() {
        let report = parse_gerber_metadata_report(
            b"%TF.Part,Other*%\n%TF.FileFunction,Copper,Top*%\n%TF.FilePolarity,Inverted*%\n%TF.CreationDate,2026-02-30T08:00:00Z*%\n%TF.GenerationSoftware,KiCad,OnlyTwoFields*%\n%TF.ProjectId,Widget,not-a-guid,A*%\n%TF.MD5,not-a-digest*%\n%TF.Part,Array*%\n",
        );

        assert_eq!(report.metadata.part.as_deref(), Some("Other"));
        assert_eq!(report.metadata.file_function.as_deref(), Some("Copper,Top"));
        assert_eq!(report.metadata.file_polarity.as_deref(), Some("Inverted"));
        assert_eq!(
            report.metadata.creation_date.as_deref(),
            Some("2026-02-30T08:00:00Z")
        );
        assert_eq!(
            report.metadata.generation_software.as_deref(),
            Some("KiCad,OnlyTwoFields")
        );
        assert_eq!(
            report.metadata.project_id.as_deref(),
            Some("Widget,not-a-guid,A")
        );
        assert_eq!(report.metadata.md5.as_deref(), Some("not-a-digest"));
        assert_eq!(report.issues.len(), 8);
        assert!(report.issues.iter().any(|issue| {
            issue.line == 1
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::InvalidFileAttributeValue { attribute, value }
                        if attribute == "TF.Part" && value == "Other"
                )
        }));
        assert!(report.issues.iter().any(|issue| {
            issue.line == 2
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::InvalidFileAttributeValue { attribute, value }
                        if attribute == "TF.FileFunction" && value == "Copper,Top"
                )
        }));
        assert!(report.issues.iter().any(|issue| {
            issue.line == 3
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::InvalidFileAttributeValue { attribute, value }
                        if attribute == "TF.FilePolarity" && value == "Inverted"
                )
        }));
        assert!(report.issues.iter().any(|issue| {
            issue.line == 4
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::InvalidFileAttributeValue { attribute, value }
                        if attribute == "TF.CreationDate" && value == "2026-02-30T08:00:00Z"
                )
        }));
        assert!(report.issues.iter().any(|issue| {
            issue.line == 5
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::InvalidFileAttributeValue { attribute, value }
                        if attribute == "TF.GenerationSoftware" && value == "KiCad,OnlyTwoFields"
                )
        }));
        assert!(report.issues.iter().any(|issue| {
            issue.line == 6
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::InvalidFileAttributeValue { attribute, value }
                        if attribute == "TF.ProjectId" && value == "Widget,not-a-guid,A"
                )
        }));
        assert!(report.issues.iter().any(|issue| {
            issue.line == 7
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::InvalidFileAttributeValue { attribute, value }
                        if attribute == "TF.MD5" && value == "not-a-digest"
                )
        }));
        assert!(report.issues.iter().any(|issue| {
            issue.line == 8
                && matches!(
                    &issue.kind,
                    GerberMetadataIssueKind::ConflictingFileAttribute { attribute, .. }
                        if attribute == "TF.Part"
                )
        }));
    }

    #[test]
    fn accepts_standard_file_function_forms_used_by_manifest_roles() {
        let report = parse_gerber_metadata_report(
            b"%TF.FileFunction,Copper,L1,Top*%\n%TF.FileFunction,Copper,L2,Inr*%\n%TF.FileFunction,Soldermask,Bot*%\n%TF.FileFunction,Paste,Top*%\n%TF.FileFunction,Legend,Bot*%\n%TF.FileFunction,Profile,NP*%\n%TF.FileFunction,Glue,Top*%\n",
        );

        assert!(report.issues.iter().all(|issue| !matches!(
            &issue.kind,
            GerberMetadataIssueKind::InvalidFileAttributeValue { attribute, .. }
                if attribute == "TF.FileFunction"
        )));
    }

    #[test]
    fn reports_file_function_values_missing_required_role_fields() {
        let report = parse_gerber_metadata_report(
            b"%TF.FileFunction,Copper,L1*%\n%TF.FileFunction,Soldermask,Inr*%\n%TF.FileFunction,Paste,Top,Extra*%\n%TF.FileFunction,Legend*%\n",
        );

        let invalid_values = report
            .issues
            .iter()
            .filter_map(|issue| match &issue.kind {
                GerberMetadataIssueKind::InvalidFileAttributeValue { attribute, value }
                    if attribute == "TF.FileFunction" =>
                {
                    Some(value.as_str())
                }
                _ => None,
            })
            .collect::<Vec<_>>();

        assert_eq!(
            invalid_values,
            vec!["Copper,L1", "Soldermask,Inr", "Paste,Top,Extra", "Legend"]
        );
    }

    #[test]
    fn accepts_standard_project_id_and_generation_software_values() {
        let report = parse_gerber_metadata_report(
            b"%TF.GenerationSoftware,Ucamco,UcamX,2017.04*%\n%TF.ProjectId,My PCB,f81d4fae-7dec-11d0-a765-00a0c91e6bf6,2*%\n%TF.ProjectId,My PCB,f81d4fae-7dec-11d0-a765-00a0c91e6bf6,2*%\n",
        );

        assert_eq!(
            report.metadata.generation_software.as_deref(),
            Some("Ucamco,UcamX,2017.04")
        );
        assert_eq!(
            report.metadata.project_id.as_deref(),
            Some("My PCB,f81d4fae-7dec-11d0-a765-00a0c91e6bf6,2")
        );
        assert!(matches!(
            &report.issues[0].kind,
            GerberMetadataIssueKind::DuplicateFileAttribute { attribute, .. }
                if attribute == "TF.ProjectId"
        ));
        assert_eq!(report.issues.len(), 1);
    }

    #[test]
    fn issue_messages_are_human_readable() {
        let report = parse_gerber_metadata_report(b"%TF.FilePolarity*%\n");

        assert!(
            report.issues[0]
                .message()
                .contains("TF.FilePolarity is missing")
        );
        let report = parse_gerber_metadata_report(b"%TF.FilePolarity,Clear*%\n");
        assert!(
            report.issues[0]
                .message()
                .contains("TF.FilePolarity has non-standard value")
        );
        let report = parse_gerber_metadata_report(b"%TA.AperFunction*%\n");
        assert!(
            report.issues[0]
                .message()
                .contains("TA.AperFunction is missing")
        );
        let report = parse_gerber_metadata_report(b"%TO.C*%\n");
        assert!(report.issues[0].message().contains("TO.C is missing"));
        let report = parse_gerber_metadata_report(b"%TO.P,,1*%\n");
        assert!(
            report.issues[0]
                .message()
                .contains("TO.P has non-standard value")
        );
        let report = parse_gerber_metadata_report(b"%MO*%\n");
        assert!(report.issues[0].message().contains("MO is missing"));
        let report = parse_gerber_metadata_report(b"%FSLAX35Y35*%\n");
        assert!(
            report.issues[0]
                .message()
                .contains("FS has non-standard value")
        );
        let report = parse_gerber_metadata_report(b"%ADD*%\n");
        assert!(report.issues[0].message().contains("ADD is missing"));
        let report = parse_gerber_metadata_report(b"%ADD9C,0.5*%\n");
        assert!(
            report.issues[0]
                .message()
                .contains("ADD has non-standard value")
        );
        let report = parse_gerber_metadata_report(b"%AM*%\n");
        assert!(report.issues[0].message().contains("AM is missing"));
        let report = parse_gerber_metadata_report(b"%AMBAD*99,1,2*%\n");
        assert!(
            report.issues[0]
                .message()
                .contains("AM has non-standard value")
        );
        let report = parse_gerber_metadata_report(
            b"%AMM*1,1,0.5,0,0,0*%\n%AMM*1,1,0.5,0,0,0*%\n%AMM*1,1,0.6,0,0,0*%\n",
        );
        assert!(
            report.issues[0]
                .message()
                .contains("is repeated with the same body")
        );
        assert!(report.issues[1].message().contains("is redefined"));
        let report = parse_gerber_metadata_report(b"D99*\n");
        assert!(
            report.issues[0]
                .message()
                .contains("selects undefined aperture")
        );
        let report = parse_gerber_metadata_report(b"X0Y0D03*\n");
        assert!(
            report.issues[0]
                .message()
                .contains("requires a current aperture")
        );
        let report = parse_gerber_metadata_report(b"%LP*%\n");
        assert!(report.issues[0].message().contains("LP is missing"));
        let report = parse_gerber_metadata_report(b"%LPX*%\n");
        assert!(
            report.issues[0]
                .message()
                .contains("LP has non-standard value")
        );
        let report = parse_gerber_metadata_report(b"G37*\n");
        assert!(
            report.issues[0]
                .message()
                .contains("without an active region")
        );
        let report = parse_gerber_metadata_report(b"G36*\nG36*\n");
        assert!(report.issues[0].message().contains("is still active"));
        assert!(
            report.issues[1]
                .message()
                .contains("is not closed before end of file")
        );
        let report = parse_gerber_metadata_report(b"%SRX0Y1I0J0*%\n");
        assert!(
            report.issues[0]
                .message()
                .contains("SR has non-standard value")
        );
        let report = parse_gerber_metadata_report(b"%SR*%\n");
        assert!(
            report.issues[0]
                .message()
                .contains("without an active repeat block")
        );
        let report = parse_gerber_metadata_report(b"%SRX1Y1I0J0*%\n%SRX1Y1I1J1*%\n");
        assert!(report.issues[0].message().contains("is still active"));
        assert!(
            report.issues[1]
                .message()
                .contains("is not closed before end of file")
        );
    }
}
