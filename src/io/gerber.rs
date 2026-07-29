//! Gerber (RS-274X) input and output for native 2D curve geometry.
//!
//! The implementation uses `gerber-types` for code generation and `gerber_parser`
//! for parsing. Gerber regions map directly to filled Hypercurve regions. Simple
//! circular, rectangular, obround, and regular-polygon flashes are imported as
//! filled shapes. Linear and circular aperture-stroked traces are imported for
//! standard apertures by constructing the swept aperture area.

use crate::curve::{self, CurveRegionExt};
use gerber_types::{
    Aperture, ApertureDefinition, AxisSelect, Circle, Command, CommentContent,
    CoordinateFormat, CoordinateMode, CoordinateNumber, CoordinateOffset, Coordinates, DCode,
    ExtendedCode, FunctionCode, GCode, GerberCode, ImageMirroring, ImageOffset, ImagePolarity,
    ImageRotation, ImageScaling, InterpolationMode, MCode, Mirroring, Operation, Polarity,
    Rectangular, Rotation, Scaling, StepAndRepeat, Unit, ZeroOmission,
};
use hypercurve::{
    Classification, Contour2, CurvePolicy, CurveRegion2, CurveString2,
    FiniteProjectionOptions, FiniteRegionProfile2,
};
use hyperlattice::Real;
use std::collections::HashMap;
use std::convert::TryFrom;
use std::fmt::Debug;
use std::io::{BufReader, Cursor};
use std::panic::{AssertUnwindSafe, catch_unwind};

use super::IoError;

/// Finite XY coordinate used only while reading or writing Gerber records.
///
/// This is deliberately a local file-format boundary type, not CAD topology.
/// Imported coordinates are promoted into `hypercurve::CurveRegion2` and
/// `CurveString2` as soon as enough structure is known; exported coordinates
/// are lossy projections from hypercurve objects. Keeping primitive numbers at
/// the format boundary follows Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7(1-2), 1997
/// (<https://doi.org/10.1016/0925-7721(95)00040-2>), and Hobby, "Practical
/// Segment Intersection with Finite Precision Output," *Computational
/// Geometry* 13(4), 1999
/// (<https://doi.org/10.1016/S0925-7721(99)00021-8>).
#[derive(Clone, Copy, Debug, PartialEq)]
struct GerberCoord<T> {
    x: T,
    y: T,
}

type Coord<T> = GerberCoord<T>;

fn real(value: f64) -> Option<Real> {
    Real::try_from(value).ok()
}

fn required_real(value: f64, field: &'static str) -> Result<Real, IoError> {
    real(value).ok_or(IoError::UnrepresentableCoordinate {
        format: "Gerber",
        field,
        target: "Real",
    })
}

fn positive(value: f64, field: &'static str) -> Result<f64, IoError> {
    if value.is_finite() && value > 0.0 {
        Ok(value)
    } else {
        Err(IoError::MalformedInput(format!(
            "Gerber {field} must be finite and positive"
        )))
    }
}

fn coord_on_circle(center: Coord<f64>, radius: f64, angle: f64) -> Coord<f64> {
    Coord {
        x: center.x + angle.cos() * radius,
        y: center.y + angle.sin() * radius,
    }
}

fn origin_circle_coord(radius: f64, angle: f64) -> Coord<f64> {
    coord_on_circle(Coord { x: 0.0, y: 0.0 }, radius, angle)
}

/// Options used when exporting a native filled region to Gerber.
#[derive(Clone, Copy, Debug)]
pub struct GerberExportOptions {
    /// Gerber file units. Defaults to millimeters.
    pub unit: Unit,
    /// Coordinate encoding used for emitted X/Y values.
    pub coordinate_format: CoordinateFormat,
}

impl Default for GerberExportOptions {
    fn default() -> Self {
        Self {
            unit: Unit::Millimeters,
            coordinate_format: CoordinateFormat::new(
                ZeroOmission::Leading,
                CoordinateMode::Absolute,
                4,
                6,
            ),
        }
    }
}

/// Parses Gerber into distinct native filled, string, and path carriers.
pub fn import_gerber(
    data: &[u8],
) -> Result<(CurveRegion2, Vec<CurveString2>, Vec<hypercurve::CurvePath2>), IoError> {
    parse_gerber(data).map(|region| (region, Vec::new(), Vec::new()))
}

/// Serializes a native filled region to Gerber.
pub fn export_gerber(region: &CurveRegion2) -> Result<Vec<u8>, IoError> {
    export_gerber_with_options(region, GerberExportOptions::default())
}

#[allow(clippy::result_large_err)]
fn parse_gerber(gerber_data: &[u8]) -> Result<CurveRegion2, IoError> {
    preflight_gerber_input(gerber_data)?;

    let reader = BufReader::new(Cursor::new(gerber_data));
    let parsed = catch_unwind(AssertUnwindSafe(|| gerber_parser::parse(reader)))
        .map_err(|_| IoError::GerberParsing("Gerber parser panicked".to_string()))?;
    let doc = parsed.map_err(|(_doc, error)| IoError::GerberParsing(format!("{error:?}")))?;

    let errors = doc.errors();
    if !errors.is_empty() {
        let message = errors
            .iter()
            .map(|error| error.to_string())
            .collect::<Vec<_>>()
            .join("; ");
        return Err(IoError::GerberParsing(message));
    }

    let units = doc.units.ok_or_else(|| {
        IoError::MalformedInput("Gerber input does not declare units".into())
    })?;
    let mut state = ImportState::new(units);
    for command in doc.commands() {
        state.apply_command(command, &doc.apertures)?;
    }

    state.finish()
}

fn preflight_gerber_input(gerber_data: &[u8]) -> Result<(), IoError> {
    std::str::from_utf8(gerber_data)
        .map_err(|error| IoError::GerberParsing(format!("Invalid UTF-8: {error}")))?;
    Ok(())
}

pub fn export_gerber_with_options(
    region: &CurveRegion2,
    options: GerberExportOptions,
) -> Result<Vec<u8>, IoError> {
    if options.coordinate_format.coordinate_mode != CoordinateMode::Absolute {
        return Err(IoError::Unsupported {
            format: "Gerber",
            detail: "incremental-coordinate export is not supported".into(),
        });
    }
    let mut commands = vec![
        FunctionCode::GCode(GCode::Comment(CommentContent::String(
            "Generated by csgrs".to_string(),
        )))
        .into(),
        ExtendedCode::Unit(options.unit).into(),
        ExtendedCode::CoordinateFormat(options.coordinate_format).into(),
        ExtendedCode::LoadPolarity(Polarity::Dark).into(),
        ExtendedCode::ApertureDefinition(ApertureDefinition::new(
            10,
            Aperture::Circle(Circle::new(0.01)),
        ))
        .into(),
        FunctionCode::DCode(DCode::SelectAperture(10)).into(),
        FunctionCode::GCode(GCode::InterpolationMode(InterpolationMode::Linear)).into(),
    ];

    // Closed-region export consumes the native hypercurve boundary directly.
    // Supported boundary input is promoted into CurveRegion2/CurveString2 when
    // curves are constructed. Gerber coordinates are serialization output,
    // while geometric classification stays with hyper geometry; that split
    // follows Yap, "Towards Exact Geometric Computation," Computational
    // Geometry 7(1-2), 1997
    // (<https://doi.org/10.1016/0925-7721(95)00040-2>), and the finite-output
    // boundary described by Hobby, "Practical Segment Intersection with
    // Finite Precision Output," Computational Geometry 13(4), 1999
    // (<https://doi.org/10.1016/S0925-7721(99)00021-8>).
    if region.is_empty() {
        return Err(IoError::Geometry {
            format: "Gerber",
            detail: "cannot export empty geometry".into(),
        });
    }
    {
        let projection_options =
            FiniteProjectionOptions::try_new(1e-3).map_err(|error| IoError::Geometry {
                format: "Gerber",
                detail: format!("invalid projection configuration: {error}"),
            })?;
        match region
            .project_to_finite_profiles(&projection_options, &CurvePolicy::certified())
            .map_err(|error| IoError::Geometry {
                format: "Gerber",
                detail: format!("native region projection failed: {error}"),
            })? {
            Classification::Decided(region_profiles) => {
                emit_region_profiles(&region_profiles, &mut commands, options)?;
            },
            Classification::Uncertain(reason) => {
                return Err(IoError::Geometry {
                    format: "Gerber",
                    detail: format!("region projection is uncertain: {reason:?}"),
                });
            },
        }
    }

    commands.push(FunctionCode::MCode(MCode::EndOfFile).into());

    let mut bytes = Vec::new();
    commands.serialize(&mut bytes)?;
    Ok(bytes)
}

struct ImportState {
    curve: CurveRegion2,
    pending_dark: Vec<CurveRegion2>,
    unit_scale: f64,
    coordinate_mode: CoordinateMode,
    source_current: Coord<f64>,
    current: Coord<f64>,
    selected_aperture: Option<i32>,
    interpolation_mode: InterpolationMode,
    polarity: Polarity,
    aperture_transform: ApertureTransform,
    image_transform: ImageTransform,
    step_repeat: Option<StepRepeatState>,
    region: Option<RegionBuilder>,
}

impl ImportState {
    fn new(unit: Unit) -> Self {
        Self {
            curve: CurveRegion2::empty(),
            pending_dark: Vec::new(),
            unit_scale: unit_scale(unit),
            coordinate_mode: CoordinateMode::Absolute,
            source_current: Coord { x: 0.0, y: 0.0 },
            current: Coord { x: 0.0, y: 0.0 },
            selected_aperture: None,
            interpolation_mode: InterpolationMode::Linear,
            polarity: Polarity::Dark,
            aperture_transform: ApertureTransform::default(),
            image_transform: ImageTransform::default(),
            step_repeat: None,
            region: None,
        }
    }

    fn finish(mut self) -> Result<CurveRegion2, IoError> {
        if self.region.is_some() {
            return Err(IoError::MalformedInput(
                "Gerber input ends inside a region".into(),
            ));
        }
        if self.step_repeat.is_some() {
            return Err(IoError::MalformedInput(
                "Gerber input ends inside a step-repeat block".into(),
            ));
        }
        self.flush_pending_dark()?;
        Ok(self.curve)
    }

    fn apply_command(
        &mut self,
        command: &Command,
        apertures: &HashMap<i32, Aperture>,
    ) -> Result<(), IoError> {
        match command {
            Command::ExtendedCode(ExtendedCode::CoordinateFormat(format)) => {
                self.coordinate_mode = format.coordinate_mode;
            },
            Command::ExtendedCode(ExtendedCode::Unit(unit)) => {
                self.unit_scale = unit_scale(*unit);
            },
            Command::FunctionCode(FunctionCode::GCode(GCode::InterpolationMode(mode))) => {
                self.interpolation_mode = *mode;
            },
            Command::ExtendedCode(ExtendedCode::LoadPolarity(polarity)) => {
                self.polarity = *polarity;
            },
            Command::ExtendedCode(ExtendedCode::LoadMirroring(mirroring)) => {
                self.aperture_transform.mirroring = *mirroring;
            },
            Command::ExtendedCode(ExtendedCode::LoadScaling(scaling)) => {
                self.aperture_transform.scaling = *scaling;
            },
            Command::ExtendedCode(ExtendedCode::LoadRotation(rotation)) => {
                self.aperture_transform.rotation = *rotation;
            },
            Command::ExtendedCode(ExtendedCode::StepAndRepeat(step_repeat)) => {
                self.apply_step_repeat(step_repeat)?;
            },
            Command::ExtendedCode(ExtendedCode::AxisSelect(axis_select)) => {
                self.image_transform.axis_select = *axis_select;
            },
            Command::ExtendedCode(ExtendedCode::MirrorImage(mirroring)) => {
                self.image_transform.mirroring = *mirroring;
            },
            Command::ExtendedCode(ExtendedCode::OffsetImage(offset)) => {
                self.image_transform.offset = *offset;
            },
            Command::ExtendedCode(ExtendedCode::ScaleImage(scaling)) => {
                self.image_transform.scaling = *scaling;
            },
            Command::ExtendedCode(ExtendedCode::RotateImage(rotation)) => {
                self.image_transform.rotation = *rotation;
            },
            Command::ExtendedCode(ExtendedCode::ImagePolarity(polarity)) => {
                if matches!(polarity, ImagePolarity::Negative) {
                    return Err(IoError::Unsupported {
                        format: "Gerber",
                        detail: "negative image polarity has an unbounded background".into(),
                    });
                }
                self.polarity = Polarity::Dark;
            },
            Command::ExtendedCode(ExtendedCode::ApertureBlock(_)) => {
                return Err(IoError::Unsupported {
                    format: "Gerber",
                    detail: "aperture blocks are not supported".into(),
                });
            },
            Command::ExtendedCode(
                ExtendedCode::ApertureDefinition(_)
                | ExtendedCode::ApertureMacro(_)
                | ExtendedCode::FileAttribute(_)
                | ExtendedCode::ObjectAttribute(_)
                | ExtendedCode::ApertureAttribute(_)
                | ExtendedCode::DeleteAttribute(_)
                | ExtendedCode::ImageName(_),
            ) => {},
            Command::FunctionCode(FunctionCode::GCode(GCode::RegionMode(enabled))) => {
                if *enabled {
                    if self.region.is_some() {
                        return Err(IoError::MalformedInput(
                            "Gerber region mode was opened while already active".into(),
                        ));
                    }
                    self.region = Some(RegionBuilder::new());
                } else {
                    let region = self.region.take().ok_or_else(|| {
                        IoError::MalformedInput(
                            "Gerber region mode was closed while not active".into(),
                        )
                    })?;
                    let curve = region.into_curve()?;
                    self.apply_polarity(curve)?;
                }
            },
            Command::FunctionCode(FunctionCode::DCode(DCode::SelectAperture(code))) => {
                self.selected_aperture = Some(*code);
            },
            Command::FunctionCode(FunctionCode::DCode(DCode::Operation(operation))) => {
                self.apply_operation(operation, apertures)?;
            },
            Command::FunctionCode(FunctionCode::GCode(GCode::Unit(unit))) => {
                self.unit_scale = unit_scale(*unit);
            },
            Command::FunctionCode(FunctionCode::GCode(GCode::CoordinateMode(mode))) => {
                self.coordinate_mode = *mode;
            },
            Command::FunctionCode(FunctionCode::GCode(
                GCode::QuadrantMode(_) | GCode::Comment(_) | GCode::SelectAperture,
            ))
            | Command::FunctionCode(FunctionCode::MCode(MCode::EndOfFile)) => {},
        }

        Ok(())
    }

    fn apply_step_repeat(&mut self, step_repeat: &StepAndRepeat) -> Result<(), IoError> {
        self.step_repeat = match *step_repeat {
            StepAndRepeat::Open {
                repeat_x,
                repeat_y,
                distance_x,
                distance_y,
            } => {
                if self.step_repeat.is_some() {
                    return Err(IoError::MalformedInput(
                        "Gerber step-repeat was opened while already active".into(),
                    ));
                }
                let distance_x = distance_x * self.unit_scale;
                let distance_y = distance_y * self.unit_scale;
                if repeat_x == 0
                    || repeat_y == 0
                    || !distance_x.is_finite()
                    || !distance_y.is_finite()
                {
                    return Err(IoError::MalformedInput(
                        "Gerber step-repeat requires positive counts and finite distances"
                            .into(),
                    ));
                }
                Some(StepRepeatState {
                    repeat_x,
                    repeat_y,
                    distance_x,
                    distance_y,
                })
            },
            StepAndRepeat::Close => {
                if self.step_repeat.is_none() {
                    return Err(IoError::MalformedInput(
                        "Gerber step-repeat was closed while not active".into(),
                    ));
                }
                None
            },
        };
        Ok(())
    }

    fn apply_polarity(&mut self, curve: CurveRegion2) -> Result<(), IoError> {
        let curves = repeat_curves(curve, self.step_repeat)?;
        for curve in curves {
            match self.polarity {
                Polarity::Dark => self.pending_dark.push(curve),
                Polarity::Clear => {
                    self.flush_pending_dark()?;
                    self.curve = self.curve.try_difference(&curve).map_err(|error| {
                        IoError::Geometry {
                            format: "Gerber",
                            detail: error.to_string(),
                        }
                    })?;
                },
            }
        }
        Ok(())
    }

    /// Merge consecutive dark objects through a balanced exact-Boolean tree.
    ///
    /// Dark Gerber exposure is a commutative union until a clear-polarity
    /// object appears. Pairwise rounds prevent a large accumulated region from
    /// being rebuilt for every flash or region while preserving command-order
    /// semantics at each clear boundary.
    fn flush_pending_dark(&mut self) -> Result<(), IoError> {
        if self.pending_dark.is_empty() {
            return Ok(());
        }
        let mut level = std::mem::take(&mut self.pending_dark);
        while level.len() > 1 {
            let mut next = Vec::with_capacity(level.len().div_ceil(2));
            let mut profiles = level.into_iter();
            while let Some(left) = profiles.next() {
                let Some(right) = profiles.next() else {
                    next.push(left);
                    break;
                };
                next.push(left.try_union(&right).map_err(|error| IoError::Geometry {
                    format: "Gerber",
                    detail: error.to_string(),
                })?);
            }
            level = next;
        }
        let dark = level
            .pop()
            .expect("nonempty pending dark level retains one profile");
        self.curve = if self.curve.is_empty() {
            dark
        } else {
            self.curve
                .try_union(&dark)
                .map_err(|error| IoError::Geometry {
                    format: "Gerber",
                    detail: error.to_string(),
                })?
        };
        Ok(())
    }

    fn apply_operation(
        &mut self,
        operation: &Operation,
        apertures: &HashMap<i32, Aperture>,
    ) -> Result<(), IoError> {
        match operation {
            Operation::Move(coords) => {
                if let Some(coords) = coords {
                    (self.source_current, self.current) = resolve_coordinates(
                        coords,
                        self.source_current,
                        self.unit_scale,
                        self.coordinate_mode,
                        self.image_transform,
                    );
                }
                if let Some(region) = &mut self.region {
                    region.move_to(self.current)?;
                }
            },
            Operation::Interpolate(coords, offset) => {
                let (source_target, target) = coords
                    .as_ref()
                    .map(|coords| {
                        resolve_coordinates(
                            coords,
                            self.source_current,
                            self.unit_scale,
                            self.coordinate_mode,
                            self.image_transform,
                        )
                    })
                    .unwrap_or((self.source_current, self.current));

                let arc = match (self.interpolation_mode, offset) {
                    (InterpolationMode::Linear, _) | (_, None) => None,
                    (
                        InterpolationMode::ClockwiseCircular
                        | InterpolationMode::CounterclockwiseCircular,
                        Some(offset),
                    ) => {
                        if !self.image_transform.preserves_circles() {
                            return Err(IoError::Unsupported {
                                format: "Gerber",
                                detail:
                                    "circular interpolation under non-uniform image scaling"
                                        .into(),
                            });
                        }
                        Some(circular_interpolation(
                            self.current,
                            target,
                            resolve_offset(offset, self.unit_scale, self.image_transform),
                            self.interpolation_mode,
                            self.image_transform.reverses_orientation(),
                        )?)
                    },
                };
                let points = arc
                    .as_ref()
                    .map_or_else(|| vec![target], |arc| arc.points.clone());

                if let Some(region) = &mut self.region {
                    for point in &points {
                        region.line_to(self.current, *point);
                        self.current = *point;
                    }
                    self.source_current = source_target;
                    return Ok(());
                }

                let Some(code) = self.selected_aperture else {
                    return Err(IoError::MalformedInput(
                        "Gerber interpolation has no selected aperture".into(),
                    ));
                };
                let Some(aperture) = apertures.get(&code) else {
                    return Err(IoError::MalformedInput(format!(
                        "Gerber interpolation references unknown aperture D{code}"
                    )));
                };

                if let Some(arc) = &arc {
                    let curve = trace_arc_to_curve(
                        aperture,
                        arc,
                        self.unit_scale,
                        self.aperture_transform,
                    )?;
                    self.apply_polarity(curve)?;
                    self.source_current = source_target;
                    self.current = target;
                    return Ok(());
                }

                let mut start = self.current;
                for point in points {
                    let curve = trace_to_curve(
                        aperture,
                        start,
                        point,
                        self.unit_scale,
                        self.aperture_transform,
                    )?;
                    self.apply_polarity(curve)?;
                    start = point;
                }
                self.source_current = source_target;
                self.current = start;
            },
            Operation::Flash(coords) => {
                if let Some(coords) = coords {
                    (self.source_current, self.current) = resolve_coordinates(
                        coords,
                        self.source_current,
                        self.unit_scale,
                        self.coordinate_mode,
                        self.image_transform,
                    );
                }

                let Some(code) = self.selected_aperture else {
                    return Err(IoError::MalformedInput(
                        "Gerber flash has no selected aperture".into(),
                    ));
                };
                let Some(aperture) = apertures.get(&code) else {
                    return Err(IoError::MalformedInput(format!(
                        "Gerber flash references unknown aperture D{code}"
                    )));
                };
                let curve = flash_to_curve(
                    aperture,
                    self.current,
                    self.unit_scale,
                    self.aperture_transform,
                )?;
                self.apply_polarity(curve)?;
            },
        }

        Ok(())
    }
}

struct RegionBuilder {
    current_ring: Vec<[f64; 2]>,
    rings: Vec<Vec<[f64; 2]>>,
}

#[derive(Clone, Copy, Debug)]
struct StepRepeatState {
    repeat_x: u32,
    repeat_y: u32,
    distance_x: f64,
    distance_y: f64,
}

#[derive(Clone, Copy, Debug)]
struct ApertureTransform {
    mirroring: Mirroring,
    scaling: Scaling,
    rotation: Rotation,
}

impl Default for ApertureTransform {
    fn default() -> Self {
        Self {
            mirroring: Mirroring::None,
            scaling: Scaling { scale: 1.0 },
            rotation: Rotation { rotation: 0.0 },
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct ImageTransform {
    axis_select: AxisSelect,
    mirroring: ImageMirroring,
    offset: ImageOffset,
    scaling: ImageScaling,
    rotation: ImageRotation,
}

#[derive(Clone, Debug)]
struct CircularInterpolation {
    center: Coord<f64>,
    radius: f64,
    start_angle: f64,
    sweep: f64,
    points: Vec<Coord<f64>>,
}

impl Default for ImageTransform {
    fn default() -> Self {
        Self {
            axis_select: AxisSelect::AXBY,
            mirroring: ImageMirroring::None,
            offset: ImageOffset::default(),
            scaling: ImageScaling::default(),
            rotation: ImageRotation::None,
        }
    }
}

impl RegionBuilder {
    const fn new() -> Self {
        Self {
            current_ring: Vec::new(),
            rings: Vec::new(),
        }
    }

    fn move_to(&mut self, point: Coord<f64>) -> Result<(), IoError> {
        self.finish_ring()?;
        self.current_ring.push(point_from_coord(point));
        Ok(())
    }

    fn line_to(&mut self, start: Coord<f64>, end: Coord<f64>) {
        if self.current_ring.is_empty() {
            self.current_ring.push(point_from_coord(start));
        }
        self.current_ring.push(point_from_coord(end));
    }

    fn finish_ring(&mut self) -> Result<(), IoError> {
        if self.current_ring.is_empty() {
            return Ok(());
        }
        if self.current_ring.len() < 3 {
            return Err(IoError::MalformedInput(
                "Gerber region contour has fewer than three points".into(),
            ));
        }

        if self.current_ring.first() != self.current_ring.last() {
            let first = self.current_ring[0];
            self.current_ring.push(first);
        }

        self.rings.push(std::mem::take(&mut self.current_ring));
        Ok(())
    }

    fn into_curve(mut self) -> Result<CurveRegion2, IoError> {
        self.finish_ring()?;

        let mut rings = self.rings;
        if rings.is_empty() {
            return Err(IoError::MalformedInput(
                "Gerber region contains no valid closed contours".into(),
            ));
        }
        let contours = rings
            .drain(..)
            .map(|ring| {
                Contour2::from_finite_ring(&ring).map_err(|error| IoError::Geometry {
                    format: "Gerber",
                    detail: format!("invalid region contour: {error}"),
                })
            })
            .collect::<Result<Vec<_>, _>>()?;
        let region = match CurveRegion2::try_from_native_boundary_contours(
            contours,
            &CurvePolicy::certified(),
        )
        .map_err(|error| IoError::Geometry {
            format: "Gerber",
            detail: error.to_string(),
        })? {
            Classification::Decided(region) => region,
            Classification::Uncertain(reason) => {
                return Err(IoError::Geometry {
                    format: "Gerber",
                    detail: format!("region contour nesting is uncertain: {reason:?}"),
                });
            },
        };
        Ok(region)
    }
}

/// Emit native hypercurve region profiles as Gerber regions grouped by
/// material-contour ownership.
///
/// Gerber regions are finite serialization records; `CurveRegion2` remains the
/// source of topology. Grouping holes by their containing material contour
/// follows the point-in-polygon classification family surveyed by Hormann and
/// Agathos, "The point in polygon problem for arbitrary polygons,"
/// *Computational Geometry* 20(3), 2001
/// (<https://doi.org/10.1016/S0925-7721(01)00012-8>), while keeping exact
/// hyper geometry internal follows Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7(1-2), 1997
/// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
fn emit_region_profiles(
    profiles: &[FiniteRegionProfile2],
    commands: &mut Vec<Command>,
    options: GerberExportOptions,
) -> Result<(), IoError> {
    for profile in profiles {
        commands.push(FunctionCode::GCode(GCode::RegionMode(true)).into());
        emit_point_ring(profile.material().points(), commands, options)?;
        for hole in profile.holes() {
            emit_point_ring(hole.points(), commands, options)?;
        }
        commands.push(FunctionCode::GCode(GCode::RegionMode(false)).into());
    }
    Ok(())
}

fn emit_point_ring(
    ring: &[[f64; 2]],
    commands: &mut Vec<Command>,
    options: GerberExportOptions,
) -> Result<(), IoError> {
    let first = ring.first().ok_or_else(|| IoError::Geometry {
        format: "Gerber",
        detail: "projected region contains an empty contour".into(),
    })?;

    let first_coordinates = gerber_coordinates(
        Coord {
            x: first[0],
            y: first[1],
        },
        options,
    )?;
    commands.push(
        FunctionCode::DCode(DCode::Operation(Operation::Move(Some(
            first_coordinates.clone(),
        ))))
        .into(),
    );

    let mut previous_coordinates = first_coordinates.clone();
    for point in ring.iter().skip(1) {
        let coordinates = gerber_coordinates(
            Coord {
                x: point[0],
                y: point[1],
            },
            options,
        )?;
        if coordinates == previous_coordinates {
            continue;
        }
        commands.push(
            FunctionCode::DCode(DCode::Operation(Operation::Interpolate(
                Some(coordinates.clone()),
                None,
            )))
            .into(),
        );
        previous_coordinates = coordinates;
    }
    if previous_coordinates != first_coordinates {
        commands.push(
            FunctionCode::DCode(DCode::Operation(Operation::Interpolate(
                Some(first_coordinates),
                None,
            )))
            .into(),
        );
    }

    Ok(())
}

fn gerber_coordinates(
    coord: Coord<f64>,
    options: GerberExportOptions,
) -> Result<Coordinates, IoError> {
    let x = export_coordinate_number(coord.x, options)?;
    let y = export_coordinate_number(coord.y, options)?;
    Ok(Coordinates::new(x, y, options.coordinate_format))
}

fn export_coordinate_number(
    value: f64,
    options: GerberExportOptions,
) -> Result<CoordinateNumber, IoError> {
    let value = match options.unit {
        Unit::Millimeters => value,
        Unit::Inches => value / 25.4,
    };
    Ok(CoordinateNumber::try_from(value)?.validate(&options.coordinate_format)?)
}

fn resolve_coordinates(
    coords: &Coordinates,
    source_current: Coord<f64>,
    unit_scale: f64,
    coordinate_mode: CoordinateMode,
    image_transform: ImageTransform,
) -> (Coord<f64>, Coord<f64>) {
    let supplied = Coord {
        x: coords.x.map(|x| f64::from(x) * unit_scale).unwrap_or(0.0),
        y: coords.y.map(|y| f64::from(y) * unit_scale).unwrap_or(0.0),
    };
    let source = match coordinate_mode {
        CoordinateMode::Absolute => Coord {
            x: coords.x.map_or(source_current.x, |_| supplied.x),
            y: coords.y.map_or(source_current.y, |_| supplied.y),
        },
        CoordinateMode::Incremental => Coord {
            x: source_current.x + supplied.x,
            y: source_current.y + supplied.y,
        },
    };
    (source, image_transform.apply_point(source, unit_scale))
}

fn resolve_offset(
    offset: &CoordinateOffset,
    unit_scale: f64,
    image_transform: ImageTransform,
) -> Coord<f64> {
    let coord = Coord {
        x: offset.x.map(|x| f64::from(x) * unit_scale).unwrap_or(0.0),
        y: offset.y.map(|y| f64::from(y) * unit_scale).unwrap_or(0.0),
    };
    image_transform.apply_vector(coord)
}

impl ImageTransform {
    fn preserves_circles(self) -> bool {
        self.scaling.a.is_finite()
            && self.scaling.b.is_finite()
            && self.scaling.a.abs() == self.scaling.b.abs()
    }

    fn reverses_orientation(self) -> bool {
        let x = self.apply_vector(Coord { x: 1.0, y: 0.0 });
        let y = self.apply_vector(Coord { x: 0.0, y: 1.0 });
        x.x * y.y - x.y * y.x < 0.0
    }

    fn apply_point(self, point: Coord<f64>, unit_scale: f64) -> Coord<f64> {
        let mut point = match self.axis_select {
            AxisSelect::AXBY => point,
            AxisSelect::AYBX => Coord {
                x: point.y,
                y: point.x,
            },
        };

        point.x *= self.scaling.a;
        point.y *= self.scaling.b;

        match self.mirroring {
            ImageMirroring::None => {},
            ImageMirroring::A => point.x = -point.x,
            ImageMirroring::B => point.y = -point.y,
            ImageMirroring::AB => {
                point.x = -point.x;
                point.y = -point.y;
            },
        }

        point = rotate_coord(point, image_rotation_degrees(self.rotation));
        point.x += self.offset.a * unit_scale;
        point.y += self.offset.b * unit_scale;
        point
    }

    fn apply_vector(self, vector: Coord<f64>) -> Coord<f64> {
        let mut vector = match self.axis_select {
            AxisSelect::AXBY => vector,
            AxisSelect::AYBX => Coord {
                x: vector.y,
                y: vector.x,
            },
        };

        vector.x *= self.scaling.a;
        vector.y *= self.scaling.b;

        match self.mirroring {
            ImageMirroring::None => {},
            ImageMirroring::A => vector.x = -vector.x,
            ImageMirroring::B => vector.y = -vector.y,
            ImageMirroring::AB => {
                vector.x = -vector.x;
                vector.y = -vector.y;
            },
        }

        rotate_coord(vector, image_rotation_degrees(self.rotation))
    }
}

fn flash_to_curve(
    aperture: &Aperture,
    center: Coord<f64>,
    unit_scale: f64,
    aperture_transform: ApertureTransform,
) -> Result<CurveRegion2, IoError> {
    let mut curve = match aperture {
        Aperture::Circle(circle) => {
            let diameter = positive(
                aperture_transform.scale_length(circle.diameter * unit_scale),
                "circle aperture diameter",
            )?;
            let mut curve =
                curve::circle(required_real(diameter * 0.5, "circle aperture radius")?, 64);
            if let Some(hole_diameter) = circle.hole_diameter {
                let hole_diameter = positive(
                    aperture_transform.scale_length(hole_diameter * unit_scale),
                    "circle aperture hole diameter",
                )?;
                if hole_diameter >= diameter {
                    return Err(IoError::MalformedInput(
                        "Gerber aperture hole must be smaller than its outer diameter".into(),
                    ));
                }
                curve = add_aperture_hole(
                    curve,
                    curve::circle(
                        required_real(hole_diameter * 0.5, "circle aperture hole radius")?,
                        64,
                    ),
                );
            }
            curve
        },
        Aperture::Rectangle(rect) => {
            aperture_rect_curve(rect, unit_scale, aperture_transform, false)?
        },
        Aperture::Obround(rect) => {
            aperture_rect_curve(rect, unit_scale, aperture_transform, true)?
        },
        Aperture::Polygon(polygon) => {
            if polygon.vertices < 3 {
                return Err(IoError::MalformedInput(
                    "Gerber polygon aperture requires at least three vertices".into(),
                ));
            }
            let diameter = positive(
                aperture_transform.scale_length(polygon.diameter * unit_scale),
                "polygon aperture diameter",
            )?;
            let rotation = polygon.rotation.unwrap_or(0.0);
            let mut curve = curve::rotated(
                &curve::regular_ngon(
                    polygon.vertices as usize,
                    required_real(diameter * 0.5, "polygon aperture radius")?,
                ),
                required_real(rotation, "polygon rotation")?,
            );
            if let Some(hole_diameter) = polygon.hole_diameter {
                let hole_diameter = positive(
                    aperture_transform.scale_length(hole_diameter * unit_scale),
                    "polygon aperture hole diameter",
                )?;
                if hole_diameter >= diameter {
                    return Err(IoError::MalformedInput(
                        "Gerber aperture hole must be smaller than its outer diameter".into(),
                    ));
                }
                curve = add_aperture_hole(
                    curve,
                    curve::circle(
                        required_real(hole_diameter * 0.5, "polygon aperture hole radius")?,
                        64,
                    ),
                );
            }
            curve
        },
        Aperture::Macro(..) => {
            return Err(IoError::Unsupported {
                format: "Gerber",
                detail: "aperture macro flashes are not supported".into(),
            });
        },
    };

    curve = apply_aperture_transform(curve, aperture_transform)?;
    Ok(curve::translated(
        &curve,
        required_real(center.x, "flash center x")?,
        required_real(center.y, "flash center y")?,
    ))
}

fn add_aperture_hole(outer: CurveRegion2, hole: CurveRegion2) -> CurveRegion2 {
    outer
        .try_difference(&hole)
        .unwrap_or_else(|_| CurveRegion2::empty())
}

impl ApertureTransform {
    fn scale_length(self, value: f64) -> f64 {
        value * self.scaling.scale
    }
}

fn aperture_rect_curve(
    rect: &Rectangular,
    unit_scale: f64,
    aperture_transform: ApertureTransform,
    rounded: bool,
) -> Result<CurveRegion2, IoError> {
    let width = positive(
        aperture_transform.scale_length(rect.x * unit_scale),
        "rectangular aperture width",
    )?;
    let height = positive(
        aperture_transform.scale_length(rect.y * unit_scale),
        "rectangular aperture height",
    )?;
    let mut curve = curve::translated(
        &if rounded {
            curve::rounded_rectangle(
                required_real(width, "rectangular aperture width")?,
                required_real(height, "rectangular aperture height")?,
                required_real(width.min(height) * 0.5, "obround aperture radius")?,
                16,
            )
        } else {
            curve::rectangle(
                required_real(width, "rectangular aperture width")?,
                required_real(height, "rectangular aperture height")?,
            )
        },
        required_real(-width * 0.5, "rectangular aperture x origin")?,
        required_real(-height * 0.5, "rectangular aperture y origin")?,
    );

    if let Some(hole_diameter) = rect.hole_diameter {
        let hole_diameter = positive(
            aperture_transform.scale_length(hole_diameter * unit_scale),
            "rectangular aperture hole diameter",
        )?;
        if hole_diameter >= width.min(height) {
            return Err(IoError::MalformedInput(
                "Gerber aperture hole must fit inside its outer aperture".into(),
            ));
        }
        curve = add_aperture_hole(
            curve,
            curve::circle(
                required_real(hole_diameter * 0.5, "rectangular aperture hole radius")?,
                64,
            ),
        );
    }

    Ok(curve)
}

fn trace_to_curve(
    aperture: &Aperture,
    start: Coord<f64>,
    end: Coord<f64>,
    unit_scale: f64,
    aperture_transform: ApertureTransform,
) -> Result<CurveRegion2, IoError> {
    if nearly_same(start, end) {
        return flash_to_curve(aperture, start, unit_scale, aperture_transform);
    }

    match aperture {
        Aperture::Circle(circle) => {
            let radius = aperture_transform.scale_length(circle.diameter * unit_scale) * 0.5;
            linear_circular_sweep_curve(start, end, radius)
        },
        Aperture::Rectangle(_) | Aperture::Obround(_) | Aperture::Polygon(_) => {
            let points = aperture_outline_points(aperture, unit_scale, aperture_transform)?;
            let swept_points = points
                .iter()
                .flat_map(|point| {
                    [
                        Coord {
                            x: point.x + start.x,
                            y: point.y + start.y,
                        },
                        Coord {
                            x: point.x + end.x,
                            y: point.y + end.y,
                        },
                    ]
                })
                .collect::<Vec<_>>();
            polygon_from_coords(convex_hull(swept_points)?)
        },
        Aperture::Macro(..) => Err(IoError::Unsupported {
            format: "Gerber",
            detail: "aperture macro strokes are not supported".into(),
        }),
    }
}

fn linear_circular_sweep_curve(
    start: Coord<f64>,
    end: Coord<f64>,
    radius: f64,
) -> Result<CurveRegion2, IoError> {
    let points = circle_points(positive(radius, "circular stroke radius")?, 64)
        .into_iter()
        .flat_map(|point| {
            [start, end].map(|center| Coord {
                x: point.x + center.x,
                y: point.y + center.y,
            })
        })
        .collect();
    polygon_from_coords(convex_hull(points)?)
}

fn trace_arc_to_curve(
    aperture: &Aperture,
    arc: &CircularInterpolation,
    unit_scale: f64,
    aperture_transform: ApertureTransform,
) -> Result<CurveRegion2, IoError> {
    match aperture {
        Aperture::Circle(circle) => {
            let radius = aperture_transform.scale_length(circle.diameter * unit_scale) * 0.5;
            circular_arc_sweep_curve(arc, radius)
        },
        Aperture::Rectangle(_) | Aperture::Obround(_) | Aperture::Polygon(_) => {
            Err(IoError::Unsupported {
                format: "Gerber",
                detail: "curved strokes with non-circular apertures are not supported".into(),
            })
        },
        Aperture::Macro(..) => Err(IoError::Unsupported {
            format: "Gerber",
            detail: "aperture macro strokes are not supported".into(),
        }),
    }
}

/// Construct the complete bounded projection of a circular Gerber interpolation
/// swept by a circular aperture.
///
/// The boundary is an annular sector with semicircular end caps. Full circles
/// become a disk or annulus. This retains the command's topology directly and
/// only samples its circular boundary at the finite file-import boundary.
fn circular_arc_sweep_curve(
    arc: &CircularInterpolation,
    aperture_radius: f64,
) -> Result<CurveRegion2, IoError> {
    if !aperture_radius.is_finite() || aperture_radius <= 0.0 {
        return Err(IoError::Geometry {
            format: "Gerber",
            detail: "circular aperture radius must be finite and positive".into(),
        });
    }
    let outer_radius = arc.radius + aperture_radius;
    let inner_radius = arc.radius - aperture_radius;
    let full_circle = (arc.sweep.abs() - std::f64::consts::TAU).abs() <= 1.0e-12;

    if full_circle {
        let outer =
            polygon_from_coords(translated_circle_points(arc.center, outer_radius, 96))?;
        if inner_radius <= 0.0 {
            return Ok(outer);
        }
        let inner =
            polygon_from_coords(translated_circle_points(arc.center, inner_radius, 96))?;
        return Ok(add_aperture_hole(outer, inner));
    }
    if inner_radius <= 0.0 {
        return Err(IoError::Unsupported {
            format: "Gerber",
            detail: "partial circular trace whose aperture reaches its interpolation center"
                .into(),
        });
    }

    let (start_angle, sweep) = if arc.sweep > 0.0 {
        (arc.start_angle, arc.sweep)
    } else {
        (arc.start_angle + arc.sweep, -arc.sweep)
    };
    let end_angle = start_angle + sweep;
    let mut boundary = arc_points(arc.center, outer_radius, start_angle, end_angle, false, 32);
    boundary.extend(
        arc_points(
            coord_on_circle(arc.center, arc.radius, end_angle),
            aperture_radius,
            end_angle,
            end_angle + std::f64::consts::PI,
            false,
            16,
        )
        .into_iter()
        .skip(1),
    );
    boundary.extend(
        arc_points(arc.center, inner_radius, end_angle, start_angle, true, 32)
            .into_iter()
            .skip(1),
    );
    boundary.extend(
        arc_points(
            coord_on_circle(arc.center, arc.radius, start_angle),
            aperture_radius,
            start_angle + std::f64::consts::PI,
            start_angle + std::f64::consts::TAU,
            false,
            16,
        )
        .into_iter()
        .skip(1),
    );
    polygon_from_coords(boundary)
}

fn aperture_outline_points(
    aperture: &Aperture,
    unit_scale: f64,
    aperture_transform: ApertureTransform,
) -> Result<Vec<Coord<f64>>, IoError> {
    let points = match aperture {
        Aperture::Circle(circle) => {
            let radius = positive(
                aperture_transform.scale_length(circle.diameter * unit_scale) * 0.5,
                "circle stroke radius",
            )?;
            circle_points(radius, 64)
        },
        Aperture::Rectangle(rect) => {
            let hw = positive(
                aperture_transform.scale_length(rect.x * unit_scale) * 0.5,
                "rectangular stroke half-width",
            )?;
            let hh = positive(
                aperture_transform.scale_length(rect.y * unit_scale) * 0.5,
                "rectangular stroke half-height",
            )?;
            vec![
                Coord { x: -hw, y: -hh },
                Coord { x: hw, y: -hh },
                Coord { x: hw, y: hh },
                Coord { x: -hw, y: hh },
            ]
        },
        Aperture::Obround(rect) => {
            let width = positive(
                aperture_transform.scale_length(rect.x * unit_scale),
                "obround stroke width",
            )?;
            let height = positive(
                aperture_transform.scale_length(rect.y * unit_scale),
                "obround stroke height",
            )?;
            rounded_rect_points(width, height, width.min(height) * 0.5, 16)
        },
        Aperture::Polygon(polygon) => {
            if polygon.vertices < 3 {
                return Err(IoError::MalformedInput(
                    "Gerber polygon aperture requires at least three vertices".into(),
                ));
            }
            let radius = positive(
                aperture_transform.scale_length(polygon.diameter * unit_scale) * 0.5,
                "polygon stroke radius",
            )?;
            let rotation = polygon.rotation.unwrap_or(0.0).to_radians();
            (0..polygon.vertices)
                .map(|i| {
                    let theta = std::f64::consts::TAU * f64::from(i)
                        / f64::from(polygon.vertices)
                        + rotation;
                    origin_circle_coord(radius, theta)
                })
                .collect()
        },
        Aperture::Macro(..) => {
            return Err(IoError::Unsupported {
                format: "Gerber",
                detail: "aperture macro strokes are not supported".into(),
            });
        },
    };

    Ok(points
        .into_iter()
        .map(|point| apply_aperture_transform_to_coord(point, aperture_transform))
        .collect())
}

fn circular_interpolation(
    start: Coord<f64>,
    end: Coord<f64>,
    offset: Coord<f64>,
    mode: InterpolationMode,
    orientation_reversed: bool,
) -> Result<CircularInterpolation, IoError> {
    let center = Coord {
        x: start.x + offset.x,
        y: start.y + offset.y,
    };
    let radius = (start.x - center.x).hypot(start.y - center.y);
    if !radius.is_finite() || radius <= 0.0 {
        return Err(IoError::MalformedInput(
            "Gerber circular interpolation requires a finite non-zero center offset".into(),
        ));
    }
    let end_radius = (end.x - center.x).hypot(end.y - center.y);
    if !end_radius.is_finite()
        || (!nearly_same(start, end)
            && (end_radius - radius).abs() > 1.0e-9 * radius.max(end_radius).max(1.0))
    {
        return Err(IoError::MalformedInput(
            "Gerber circular interpolation endpoints do not share one center radius".into(),
        ));
    }

    let start_angle = (start.y - center.y).atan2(start.x - center.x);
    let mut end_angle = (end.y - center.y).atan2(end.x - center.x);
    let clockwise = (mode == InterpolationMode::ClockwiseCircular) ^ orientation_reversed;

    if nearly_same(start, end) {
        end_angle = if clockwise {
            start_angle - std::f64::consts::TAU
        } else {
            start_angle + std::f64::consts::TAU
        };
    }

    let mut sweep = end_angle - start_angle;
    if clockwise && sweep >= 0.0 {
        sweep -= std::f64::consts::TAU;
    } else if !clockwise && sweep <= 0.0 {
        sweep += std::f64::consts::TAU;
    }
    let points = arc_points(center, radius, start_angle, end_angle, clockwise, 32)
        .into_iter()
        .skip(1)
        .collect();
    Ok(CircularInterpolation {
        center,
        radius,
        start_angle,
        sweep,
        points,
    })
}

fn arc_points(
    center: Coord<f64>,
    radius: f64,
    start_angle: f64,
    end_angle: f64,
    clockwise: bool,
    min_segments: usize,
) -> Vec<Coord<f64>> {
    let mut sweep = end_angle - start_angle;
    if clockwise && sweep >= 0.0 {
        sweep -= std::f64::consts::TAU;
    } else if !clockwise && sweep <= 0.0 {
        sweep += std::f64::consts::TAU;
    }

    let segments =
        ((sweep.abs() / (std::f64::consts::PI / 24.0)).ceil() as usize).max(min_segments);
    (0..=segments)
        .map(|i| {
            let angle = start_angle + sweep * (i as f64) / (segments as f64);
            coord_on_circle(center, radius, angle)
        })
        .collect()
}

fn circle_points(radius: f64, segments: usize) -> Vec<Coord<f64>> {
    (0..segments)
        .map(|i| {
            let theta = std::f64::consts::TAU * (i as f64) / (segments as f64);
            origin_circle_coord(radius, theta)
        })
        .collect()
}

fn translated_circle_points(
    center: Coord<f64>,
    radius: f64,
    segments: usize,
) -> Vec<Coord<f64>> {
    circle_points(radius, segments)
        .into_iter()
        .map(|point| Coord {
            x: point.x + center.x,
            y: point.y + center.y,
        })
        .collect()
}

fn rounded_rect_points(
    width: f64,
    height: f64,
    radius: f64,
    corner_segments: usize,
) -> Vec<Coord<f64>> {
    let radius = radius.min(width * 0.5).min(height * 0.5);
    if radius <= 0.0 {
        let hw = width * 0.5;
        let hh = height * 0.5;
        return vec![
            Coord { x: -hw, y: -hh },
            Coord { x: hw, y: -hh },
            Coord { x: hw, y: hh },
            Coord { x: -hw, y: hh },
        ];
    }

    let centers = [
        (width * 0.5 - radius, height * 0.5 - radius, 0.0),
        (
            -width * 0.5 + radius,
            height * 0.5 - radius,
            std::f64::consts::PI * 0.5,
        ),
        (
            -width * 0.5 + radius,
            -height * 0.5 + radius,
            std::f64::consts::PI,
        ),
        (
            width * 0.5 - radius,
            -height * 0.5 + radius,
            std::f64::consts::PI * 1.5,
        ),
    ];
    centers
        .into_iter()
        .flat_map(|(cx, cy, start)| {
            (0..=corner_segments).map(move |i| {
                let theta = start
                    + (std::f64::consts::PI * 0.5) * (i as f64) / (corner_segments as f64);
                coord_on_circle(Coord { x: cx, y: cy }, radius, theta)
            })
        })
        .collect()
}

fn polygon_from_coords(mut points: Vec<Coord<f64>>) -> Result<CurveRegion2, IoError> {
    if points.len() < 3 {
        return Err(IoError::Geometry {
            format: "Gerber",
            detail: "contour has fewer than three points".into(),
        });
    }
    if points.first() != points.last() {
        points.push(points[0]);
    }
    let points = points
        .into_iter()
        .map(|coord| [coord.x, coord.y])
        .collect::<Vec<_>>();
    let contour = Contour2::from_finite_ring(&points).map_err(|error| IoError::Geometry {
        format: "Gerber",
        detail: format!("invalid finite contour: {error}"),
    })?;
    CurveRegion2::try_from_native_material_contours(vec![contour], &CurvePolicy::certified())
        .map_err(|error| IoError::Geometry {
            format: "Gerber",
            detail: error.to_string(),
        })
}

const fn point_from_coord(coord: Coord<f64>) -> [f64; 2] {
    [coord.x, coord.y]
}

fn repeat_curves(
    curve: CurveRegion2,
    step_repeat: Option<StepRepeatState>,
) -> Result<Vec<CurveRegion2>, IoError> {
    let Some(step_repeat) = step_repeat else {
        return Ok(vec![curve]);
    };

    let repeat_x =
        usize::try_from(step_repeat.repeat_x).map_err(|_| IoError::SizeOverflow {
            format: "Gerber",
            limit: "step-repeat x count",
        })?;
    let repeat_y =
        usize::try_from(step_repeat.repeat_y).map_err(|_| IoError::SizeOverflow {
            format: "Gerber",
            limit: "step-repeat y count",
        })?;
    let count = repeat_x.checked_mul(repeat_y).ok_or(IoError::SizeOverflow {
        format: "Gerber",
        limit: "step-repeat instance count",
    })?;
    let mut curves = Vec::with_capacity(count);
    for x in 0..step_repeat.repeat_x {
        for y in 0..step_repeat.repeat_y {
            curves.push(curve::translated(
                &curve,
                real(f64::from(x) * step_repeat.distance_x).ok_or_else(|| {
                    IoError::UnrepresentableCoordinate {
                        format: "Gerber",
                        field: "step-repeat x translation",
                        target: "Real",
                    }
                })?,
                real(f64::from(y) * step_repeat.distance_y).ok_or_else(|| {
                    IoError::UnrepresentableCoordinate {
                        format: "Gerber",
                        field: "step-repeat y translation",
                        target: "Real",
                    }
                })?,
            ));
        }
    }
    Ok(curves)
}

fn apply_aperture_transform(
    curve: CurveRegion2,
    aperture_transform: ApertureTransform,
) -> Result<CurveRegion2, IoError> {
    let (sx, sy) = match aperture_transform.mirroring {
        Mirroring::None => (1.0, 1.0),
        Mirroring::X => (-1.0, 1.0),
        Mirroring::Y => (1.0, -1.0),
        Mirroring::XY => (-1.0, -1.0),
    };
    Ok(curve::rotated(
        &curve::scaled(
            &curve,
            required_real(sx, "aperture mirror x")?,
            required_real(sy, "aperture mirror y")?,
        ),
        required_real(aperture_transform.rotation.rotation, "aperture rotation")?,
    ))
}

fn apply_aperture_transform_to_coord(
    point: Coord<f64>,
    aperture_transform: ApertureTransform,
) -> Coord<f64> {
    let mut point = point;
    match aperture_transform.mirroring {
        Mirroring::None => {},
        Mirroring::X => point.x = -point.x,
        Mirroring::Y => point.y = -point.y,
        Mirroring::XY => {
            point.x = -point.x;
            point.y = -point.y;
        },
    }
    rotate_coord(point, aperture_transform.rotation.rotation)
}

fn rotate_coord(point: Coord<f64>, degrees: f64) -> Coord<f64> {
    let radians = degrees.to_radians();
    let (sin, cos) = radians.sin_cos();
    let x = point.x * cos - point.y * sin;
    let y = point.x * sin + point.y * cos;
    Coord { x, y }
}

const fn image_rotation_degrees(rotation: ImageRotation) -> f64 {
    match rotation {
        ImageRotation::None => 0.0,
        ImageRotation::CCW_90 => 90.0,
        ImageRotation::CCW_180 => 180.0,
        ImageRotation::CCW_270 => 270.0,
    }
}

fn convex_hull(mut points: Vec<Coord<f64>>) -> Result<Vec<Coord<f64>>, IoError> {
    // Andrew's monotone-chain hull keeps the finite Gerber aperture sweep
    // bounded, while turn classification is delegated to the shared hyperreal
    // orientation predicate. See Andrew, "Another Efficient Algorithm for
    // Convex Hulls in Two Dimensions," Information Processing Letters 9(5),
    // 1979, and Yap, "Towards Exact Geometric Computation," Computational
    // Geometry 7(1-2), 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    points.sort_by(|a, b| a.x.total_cmp(&b.x).then_with(|| a.y.total_cmp(&b.y)));
    points.dedup_by(|a, b| nearly_same(*a, *b));

    if points.len() <= 3 {
        return Ok(points);
    }

    let mut lower = Vec::new();
    for point in &points {
        while lower.len() >= 2
            && !is_left_turn(lower[lower.len() - 2], lower[lower.len() - 1], *point)?
        {
            lower.pop();
        }
        lower.push(*point);
    }

    let mut upper = Vec::new();
    for point in points.iter().rev() {
        while upper.len() >= 2
            && !is_left_turn(upper[upper.len() - 2], upper[upper.len() - 1], *point)?
        {
            upper.pop();
        }
        upper.push(*point);
    }

    lower.pop();
    upper.pop();
    lower.extend(upper);
    Ok(lower)
}

fn is_left_turn(origin: Coord<f64>, a: Coord<f64>, b: Coord<f64>) -> Result<bool, IoError> {
    let origin = real(origin.x)
        .zip(real(origin.y))
        .map(|(x, y)| hyperlimit::Point2::new(x, y))
        .ok_or(IoError::UnrepresentableCoordinate {
            format: "Gerber",
            field: "hull origin",
            target: "Real",
        })?;
    let a = real(a.x)
        .zip(real(a.y))
        .map(|(x, y)| hyperlimit::Point2::new(x, y))
        .ok_or(IoError::UnrepresentableCoordinate {
            format: "Gerber",
            field: "hull point",
            target: "Real",
        })?;
    let b = real(b.x)
        .zip(real(b.y))
        .map(|(x, y)| hyperlimit::Point2::new(x, y))
        .ok_or(IoError::UnrepresentableCoordinate {
            format: "Gerber",
            field: "hull point",
            target: "Real",
        })?;
    Ok(matches!(
        hyperlimit::orient2(&origin, &a, &b).value(),
        Some(hyperlimit::Sign::Positive)
    ))
}

fn nearly_same(a: Coord<f64>, b: Coord<f64>) -> bool {
    a.x == b.x && a.y == b.y
}

const fn unit_scale(unit: Unit) -> f64 {
    match unit {
        Unit::Millimeters => 1.0,
        Unit::Inches => 25.4,
    }
}
