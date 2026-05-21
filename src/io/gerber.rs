//! Gerber (RS-274X) input and output for 2D [`Profile`] geometry.
//!
//! The implementation uses `gerber-types` for code generation and `gerber_parser`
//! for parsing. Gerber regions map directly to filled `Profile` polygons. Simple
//! circular, rectangular, obround, and regular-polygon flashes are imported as
//! filled shapes. Linear and circular aperture-stroked traces are imported for
//! standard apertures by constructing the swept aperture area.

use crate::csg::CSG;
use crate::float_types::{
    PI, Real, TAU, hangle_sin_cos, hdegrees_to_radians, hreal_affine, hreal_mul, hreal_sub,
    hreal_to_f64, hxy_distance, hxy_orientation_sign, hxy_unit_direction, tolerance,
};
use crate::sketch::Profile;
use gerber_types::{
    Aperture, ApertureDefinition, AxisSelect, Circle, Command, CommentContent,
    CoordinateFormat, CoordinateMode, CoordinateNumber, CoordinateOffset, Coordinates, DCode,
    ExtendedCode, FunctionCode, GCode, GerberCode, ImageMirroring, ImageOffset, ImagePolarity,
    ImageRotation, ImageScaling, InterpolationMode, MCode, Mirroring, Operation, Polarity,
    Rectangular, Rotation, Scaling, StepAndRepeat, Unit, ZeroOmission,
};
use hypercurve::{
    Classification, Contour2, CurveString2, FiniteProjectionOptions, FiniteRegionProfile2,
    Point2, Segment2,
};
use std::collections::HashMap;
use std::convert::TryFrom;
use std::fmt::Debug;
use std::io::{BufReader, Cursor};
use std::panic::{AssertUnwindSafe, catch_unwind};

use super::IoError;

/// Finite XY coordinate used only while reading or writing Gerber records.
///
/// This is deliberately a local file-format boundary type, not CAD topology.
/// Imported coordinates are promoted into `hypercurve::Region2` and
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

fn coord_on_circle(center: Coord<Real>, radius: Real, angle: Real) -> Option<Coord<Real>> {
    let (sin, cos) = hangle_sin_cos(angle)?;
    let x = hreal_affine(center.x, cos, radius)?;
    let y = hreal_affine(center.y, sin, radius)?;
    Some(Coord { x, y })
}

fn origin_circle_coord(radius: Real, angle: Real) -> Option<Coord<Real>> {
    coord_on_circle(Coord { x: 0.0, y: 0.0 }, radius, angle)
}

/// Options used when exporting a [`Profile`] to Gerber.
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

/// Parse Gerber data into a `Profile`.
pub trait FromGerber<M>: Sized {
    fn from_gerber(gerber_data: &[u8], metadata: M) -> Result<Self, IoError>;
}

/// Serialize geometry to Gerber.
pub trait ToGerber {
    fn to_gerber(&self) -> Result<Vec<u8>, IoError>;
    fn to_gerber_with_options(&self, options: GerberExportOptions)
    -> Result<Vec<u8>, IoError>;
}

impl<M> FromGerber<M> for Profile<M>
where
    M: Clone + Debug + Send + Sync,
{
    fn from_gerber(gerber_data: &[u8], metadata: M) -> Result<Self, IoError> {
        preflight_gerber_input(gerber_data)?;

        let reader = BufReader::new(Cursor::new(gerber_data));
        let parsed = catch_unwind(AssertUnwindSafe(|| gerber_parser::parse(reader)))
            .map_err(|_| IoError::GerberParsing("Gerber parser panicked".to_string()))?;
        let doc =
            parsed.map_err(|(_doc, error)| IoError::GerberParsing(format!("{error:?}")))?;

        let errors = doc.errors();
        if !errors.is_empty() {
            let message = errors
                .iter()
                .map(|error| error.to_string())
                .collect::<Vec<_>>()
                .join("; ");
            return Err(IoError::GerberParsing(message));
        }

        let mut state =
            ImportState::<M>::new(doc.units.unwrap_or(Unit::Millimeters), metadata);
        for command in doc.commands() {
            state.apply_command(command, &doc.apertures)?;
        }

        Ok(state.sketch)
    }
}

fn preflight_gerber_input(gerber_data: &[u8]) -> Result<(), IoError> {
    let text = std::str::from_utf8(gerber_data)
        .map_err(|error| IoError::GerberParsing(format!("Invalid UTF-8: {error}")))?;

    for line in text.lines() {
        let line = line.trim();
        if line.is_empty() {
            continue;
        }

        if !line.contains('*') {
            return Err(IoError::GerberParsing(format!(
                "Gerber command is missing terminator: {line:?}"
            )));
        }
    }

    Ok(())
}

impl<M> ToGerber for Profile<M>
where
    M: Clone + Debug + Send + Sync,
{
    fn to_gerber(&self) -> Result<Vec<u8>, IoError> {
        self.to_gerber_with_options(GerberExportOptions::default())
    }

    fn to_gerber_with_options(
        &self,
        options: GerberExportOptions,
    ) -> Result<Vec<u8>, IoError> {
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
        // Supported boundary input is promoted into Region2/CurveString2 when
        // sketches are constructed. Gerber coordinates are serialization output,
        // while geometric classification stays with hyper geometry; that split
        // follows Yap, "Towards Exact Geometric Computation," Computational
        // Geometry 7(1-2), 1997
        // (<https://doi.org/10.1016/0925-7721(95)00040-2>), and the finite-output
        // boundary described by Hobby, "Practical Segment Intersection with
        // Finite Precision Output," Computational Geometry 13(4), 1999
        // (<https://doi.org/10.1016/S0925-7721(99)00021-8>).
        let has_native_region =
            !self.as_region().is_empty() && Self::region_has_nonzero_area(self.as_region());
        let has_native_wires = !self.wires().is_empty();
        if has_native_region || has_native_wires {
            let projection_options = FiniteProjectionOptions::try_new(1e-3)
                .expect("positive finite projection tolerance is valid");
            if let Classification::Decided(region_profiles) = self
                .project_region_profiles(&projection_options)
                .map_err(|error| {
                    IoError::GerberParsing(format!(
                        "native hypercurve Gerber region projection failed: {error}"
                    ))
                })?
            {
                emit_region_profiles(&region_profiles, &mut commands, options)?;
            }
            emit_native_wires(self.wires(), &mut commands, options)?;
        }

        commands.push(FunctionCode::MCode(MCode::EndOfFile).into());

        let mut bytes = Vec::new();
        commands.serialize(&mut bytes)?;
        Ok(bytes)
    }
}

struct ImportState<M> {
    sketch: Profile<M>,
    metadata: M,
    unit_scale: Real,
    current: Coord<Real>,
    selected_aperture: Option<i32>,
    interpolation_mode: InterpolationMode,
    polarity: Polarity,
    aperture_transform: ApertureTransform,
    image_transform: ImageTransform,
    step_repeat: Option<StepRepeatState>,
    region: Option<RegionBuilder>,
}

impl<M> ImportState<M>
where
    M: Clone + Debug + Send + Sync,
{
    fn new(unit: Unit, metadata: M) -> Self {
        Self {
            sketch: Profile::empty(metadata.clone()),
            metadata,
            unit_scale: unit_scale(unit),
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

    fn apply_command(
        &mut self,
        command: &Command,
        apertures: &HashMap<i32, Aperture>,
    ) -> Result<(), IoError> {
        match command {
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
                self.apply_step_repeat(step_repeat);
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
                self.polarity = match polarity {
                    ImagePolarity::Positive => Polarity::Dark,
                    ImagePolarity::Negative => Polarity::Clear,
                };
            },
            Command::FunctionCode(FunctionCode::GCode(GCode::RegionMode(enabled))) => {
                if *enabled {
                    self.region = Some(RegionBuilder::new());
                } else if let Some(region) = self.region.take() {
                    if let Some(sketch) = region.into_sketch(self.metadata.clone()) {
                        self.apply_polarity(sketch);
                    }
                }
            },
            Command::FunctionCode(FunctionCode::DCode(DCode::SelectAperture(code))) => {
                self.selected_aperture = Some(*code);
            },
            Command::FunctionCode(FunctionCode::DCode(DCode::Operation(operation))) => {
                self.apply_operation(operation, apertures)?;
            },
            _ => {},
        }

        Ok(())
    }

    fn apply_step_repeat(&mut self, step_repeat: &StepAndRepeat) {
        self.step_repeat = match *step_repeat {
            StepAndRepeat::Open {
                repeat_x,
                repeat_y,
                distance_x,
                distance_y,
            } => Some(StepRepeatState {
                repeat_x,
                repeat_y,
                distance_x: (distance_x as Real) * self.unit_scale,
                distance_y: (distance_y as Real) * self.unit_scale,
            }),
            StepAndRepeat::Close => None,
        };
    }

    fn apply_polarity(&mut self, sketch: Profile<M>) {
        let sketches = repeat_sketches(sketch, self.step_repeat);
        for sketch in sketches {
            self.sketch = match self.polarity {
                Polarity::Dark => self.sketch.union(&sketch),
                Polarity::Clear => self.sketch.difference(&sketch),
            };
        }
    }

    fn apply_operation(
        &mut self,
        operation: &Operation,
        apertures: &HashMap<i32, Aperture>,
    ) -> Result<(), IoError> {
        match operation {
            Operation::Move(coords) => {
                if let Some(coords) = coords {
                    self.current = resolve_coordinates(
                        coords,
                        self.current,
                        self.unit_scale,
                        self.image_transform,
                    );
                }
                if let Some(region) = &mut self.region {
                    region.move_to(self.current);
                }
            },
            Operation::Interpolate(coords, offset) => {
                let target = coords
                    .as_ref()
                    .map(|coords| {
                        resolve_coordinates(
                            coords,
                            self.current,
                            self.unit_scale,
                            self.image_transform,
                        )
                    })
                    .unwrap_or(self.current);

                let points = match (self.interpolation_mode, offset) {
                    (InterpolationMode::Linear, _) | (_, None) => vec![target],
                    (
                        InterpolationMode::ClockwiseCircular
                        | InterpolationMode::CounterclockwiseCircular,
                        Some(offset),
                    ) => approximate_arc(
                        self.current,
                        target,
                        resolve_offset(offset, self.unit_scale, self.image_transform),
                        self.interpolation_mode,
                    ),
                };

                if let Some(region) = &mut self.region {
                    for point in &points {
                        region.line_to(self.current, *point);
                        self.current = *point;
                    }
                    return Ok(());
                }

                let Some(code) = self.selected_aperture else {
                    self.current = target;
                    return Ok(());
                };
                let Some(aperture) = apertures.get(&code) else {
                    self.current = target;
                    return Ok(());
                };

                if points.len() > 1 {
                    let mut path = Vec::with_capacity(points.len() + 1);
                    path.push(self.current);
                    path.extend(points.iter().copied());
                    if let Some(sketch) = trace_path_to_sketch(
                        aperture,
                        &path,
                        self.metadata.clone(),
                        self.unit_scale,
                        self.aperture_transform,
                    ) {
                        self.apply_polarity(sketch);
                        self.current = target;
                        return Ok(());
                    }
                }

                let mut start = self.current;
                for point in points {
                    if let Some(sketch) = trace_to_sketch(
                        aperture,
                        start,
                        point,
                        self.metadata.clone(),
                        self.unit_scale,
                        self.aperture_transform,
                    ) {
                        self.apply_polarity(sketch);
                    }
                    start = point;
                }
                self.current = start;
            },
            Operation::Flash(coords) => {
                if let Some(coords) = coords {
                    self.current = resolve_coordinates(
                        coords,
                        self.current,
                        self.unit_scale,
                        self.image_transform,
                    );
                }

                let Some(code) = self.selected_aperture else {
                    return Ok(());
                };
                let Some(aperture) = apertures.get(&code) else {
                    return Ok(());
                };
                if let Some(sketch) = flash_to_sketch(
                    aperture,
                    self.current,
                    self.metadata.clone(),
                    self.unit_scale,
                    self.aperture_transform,
                ) {
                    self.apply_polarity(sketch);
                }
            },
        }

        Ok(())
    }
}

struct RegionBuilder {
    current_ring: Vec<[Real; 2]>,
    rings: Vec<Vec<[Real; 2]>>,
}

#[derive(Clone, Copy, Debug)]
struct StepRepeatState {
    repeat_x: u32,
    repeat_y: u32,
    distance_x: Real,
    distance_y: Real,
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
    fn new() -> Self {
        Self {
            current_ring: Vec::new(),
            rings: Vec::new(),
        }
    }

    fn move_to(&mut self, point: Coord<Real>) {
        self.finish_ring();
        self.current_ring.push(point_from_coord(point));
    }

    fn line_to(&mut self, start: Coord<Real>, end: Coord<Real>) {
        if self.current_ring.is_empty() {
            self.current_ring.push(point_from_coord(start));
        }
        self.current_ring.push(point_from_coord(end));
    }

    fn finish_ring(&mut self) {
        if self.current_ring.len() < 3 {
            self.current_ring.clear();
            return;
        }

        if self.current_ring.first() != self.current_ring.last() {
            let first = self.current_ring[0];
            self.current_ring.push(first);
        }

        self.rings.push(std::mem::take(&mut self.current_ring));
    }

    fn into_sketch<M>(mut self, metadata: M) -> Option<Profile<M>>
    where
        M: Clone + Debug + Send + Sync,
    {
        self.finish_ring();

        let mut rings = self
            .rings
            .into_iter()
            .filter(|ring| ring.len() >= 4)
            .collect::<Vec<_>>();
        if rings.is_empty() {
            return None;
        }

        let exterior = Contour2::from_finite_ring(&rings.remove(0)).ok()?;
        let holes = rings
            .iter()
            .filter_map(|ring| Contour2::from_finite_ring(ring).ok())
            .collect::<Vec<_>>();
        Some(Profile::from_region(
            hypercurve::Region2::new(vec![exterior], holes),
            metadata,
        ))
    }
}

/// Emit native hypercurve region profiles as Gerber regions grouped by
/// material-contour ownership.
///
/// Gerber regions are finite serialization records; `Region2` remains the
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
    ring: &[[Real; 2]],
    commands: &mut Vec<Command>,
    options: GerberExportOptions,
) -> Result<(), IoError> {
    let Some(first) = ring.first() else {
        return Ok(());
    };

    commands.push(
        FunctionCode::DCode(DCode::Operation(Operation::Move(Some(gerber_coordinates(
            Coord {
                x: first[0],
                y: first[1],
            },
            options,
        )?))))
        .into(),
    );

    for point in ring.iter().skip(1) {
        commands.push(
            FunctionCode::DCode(DCode::Operation(Operation::Interpolate(
                Some(gerber_coordinates(
                    Coord {
                        x: point[0],
                        y: point[1],
                    },
                    options,
                )?),
                None,
            )))
            .into(),
        );
    }

    Ok(())
}

/// Emit native open hypercurve wires as Gerber interpolation commands.
///
/// Straight segments serialize as linear interpolation and circular arcs as
/// RS-274X circular interpolation with I/J center offsets. This keeps
/// `CurveString2` as the CAD representation and converts to finite Gerber
/// coordinates only at the CAM file boundary, following Yap, "Towards Exact
/// Geometric Computation," *Computational Geometry* 7(1-2), 1997
/// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
fn emit_native_wires(
    wires: &[CurveString2],
    commands: &mut Vec<Command>,
    options: GerberExportOptions,
) -> Result<(), IoError> {
    for wire in wires {
        emit_native_wire(wire, commands, options)?;
    }
    commands
        .push(FunctionCode::GCode(GCode::InterpolationMode(InterpolationMode::Linear)).into());
    Ok(())
}

fn emit_native_wire(
    wire: &CurveString2,
    commands: &mut Vec<Command>,
    options: GerberExportOptions,
) -> Result<(), IoError> {
    let Some(first) = wire.segments().first() else {
        return Ok(());
    };
    commands.push(
        FunctionCode::DCode(DCode::Operation(Operation::Move(Some(gerber_coordinates(
            finite_coord(first.start())?,
            options,
        )?))))
        .into(),
    );

    for segment in wire.segments() {
        match segment {
            Segment2::Line(line) => {
                commands.push(
                    FunctionCode::GCode(GCode::InterpolationMode(InterpolationMode::Linear))
                        .into(),
                );
                commands.push(
                    FunctionCode::DCode(DCode::Operation(Operation::Interpolate(
                        Some(gerber_coordinates(finite_coord(line.end())?, options)?),
                        None,
                    )))
                    .into(),
                );
            },
            Segment2::Arc(arc) => {
                let start = finite_coord(arc.start())?;
                let end = finite_coord(arc.end())?;
                let center = finite_coord(arc.center())?;
                let offset = Coord {
                    x: center.x - start.x,
                    y: center.y - start.y,
                };
                let interpolation = if arc.is_clockwise() {
                    InterpolationMode::ClockwiseCircular
                } else {
                    InterpolationMode::CounterclockwiseCircular
                };
                commands
                    .push(FunctionCode::GCode(GCode::InterpolationMode(interpolation)).into());
                commands.push(
                    FunctionCode::DCode(DCode::Operation(Operation::Interpolate(
                        Some(gerber_coordinates(end, options)?),
                        Some(gerber_coordinate_offset(offset, options)?),
                    )))
                    .into(),
                );
            },
        }
    }

    Ok(())
}

fn gerber_coordinates(
    coord: Coord<Real>,
    options: GerberExportOptions,
) -> Result<Coordinates, IoError> {
    let x = export_coordinate_number(coord.x, options)?;
    let y = export_coordinate_number(coord.y, options)?;
    Ok(Coordinates::new(x, y, options.coordinate_format))
}

fn gerber_coordinate_offset(
    coord: Coord<Real>,
    options: GerberExportOptions,
) -> Result<CoordinateOffset, IoError> {
    let x = export_coordinate_number(coord.x, options)?;
    let y = export_coordinate_number(coord.y, options)?;
    Ok(CoordinateOffset::new(x, y, options.coordinate_format).validate()?)
}

fn finite_coord(point: &Point2) -> Result<Coord<Real>, IoError> {
    Ok(Coord {
        x: hreal_to_f64(point.x()).ok_or_else(|| {
            IoError::MalformedInput("non-finite hyperreal x coordinate".into())
        })?,
        y: hreal_to_f64(point.y()).ok_or_else(|| {
            IoError::MalformedInput("non-finite hyperreal y coordinate".into())
        })?,
    })
}

fn export_coordinate_number(
    value: Real,
    options: GerberExportOptions,
) -> Result<CoordinateNumber, IoError> {
    let value = match options.unit {
        Unit::Millimeters => value as f64,
        Unit::Inches => (value as f64) / 25.4,
    };
    Ok(CoordinateNumber::try_from(value)?.validate(&options.coordinate_format)?)
}

fn resolve_coordinates(
    coords: &Coordinates,
    current: Coord<Real>,
    unit_scale: Real,
    image_transform: ImageTransform,
) -> Coord<Real> {
    let coord = Coord {
        x: coords
            .x
            .map(|x| (f64::from(x) as Real) * unit_scale)
            .unwrap_or(current.x),
        y: coords
            .y
            .map(|y| (f64::from(y) as Real) * unit_scale)
            .unwrap_or(current.y),
    };
    image_transform.apply_point(coord, unit_scale)
}

fn resolve_offset(
    offset: &CoordinateOffset,
    unit_scale: Real,
    image_transform: ImageTransform,
) -> Coord<Real> {
    let coord = Coord {
        x: offset
            .x
            .map(|x| (f64::from(x) as Real) * unit_scale)
            .unwrap_or(0.0),
        y: offset
            .y
            .map(|y| (f64::from(y) as Real) * unit_scale)
            .unwrap_or(0.0),
    };
    image_transform.apply_vector(coord)
}

impl ImageTransform {
    fn apply_point(self, point: Coord<Real>, unit_scale: Real) -> Coord<Real> {
        let mut point = match self.axis_select {
            AxisSelect::AXBY => point,
            AxisSelect::AYBX => Coord {
                x: point.y,
                y: point.x,
            },
        };

        point.x *= self.scaling.a as Real;
        point.y *= self.scaling.b as Real;

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
        point.x += (self.offset.a as Real) * unit_scale;
        point.y += (self.offset.b as Real) * unit_scale;
        point
    }

    fn apply_vector(self, vector: Coord<Real>) -> Coord<Real> {
        let mut vector = match self.axis_select {
            AxisSelect::AXBY => vector,
            AxisSelect::AYBX => Coord {
                x: vector.y,
                y: vector.x,
            },
        };

        vector.x *= self.scaling.a as Real;
        vector.y *= self.scaling.b as Real;

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

fn flash_to_sketch<M>(
    aperture: &Aperture,
    center: Coord<Real>,
    metadata: M,
    unit_scale: Real,
    aperture_transform: ApertureTransform,
) -> Option<Profile<M>>
where
    M: Clone + Debug + Send + Sync,
{
    let mut sketch = match aperture {
        Aperture::Circle(circle) => {
            let radius =
                aperture_transform.scale_length((circle.diameter as Real) * unit_scale) * 0.5;
            let mut sketch = Profile::circle(radius, 64, metadata.clone());
            if let Some(hole_diameter) = circle.hole_diameter {
                sketch = sketch.difference(&Profile::circle(
                    aperture_transform.scale_length((hole_diameter as Real) * unit_scale)
                        * 0.5,
                    64,
                    metadata,
                ));
            }
            sketch
        },
        Aperture::Rectangle(rect) => {
            aperture_rect_sketch(rect, metadata, unit_scale, aperture_transform, false)
        },
        Aperture::Obround(rect) => {
            aperture_rect_sketch(rect, metadata, unit_scale, aperture_transform, true)
        },
        Aperture::Polygon(polygon) => {
            let radius =
                aperture_transform.scale_length((polygon.diameter as Real) * unit_scale) * 0.5;
            let rotation = polygon.rotation.unwrap_or(0.0) as Real;
            let mut sketch =
                Profile::regular_ngon(polygon.vertices as usize, radius, metadata.clone())
                    .rotate(0.0, 0.0, rotation);
            if let Some(hole_diameter) = polygon.hole_diameter {
                sketch = sketch.difference(&Profile::circle(
                    aperture_transform.scale_length((hole_diameter as Real) * unit_scale)
                        * 0.5,
                    64,
                    metadata,
                ));
            }
            sketch
        },
        Aperture::Macro(..) => return None,
    };

    sketch = apply_aperture_transform(sketch, aperture_transform);
    Some(sketch.translate(center.x, center.y, 0.0))
}

impl ApertureTransform {
    fn scale_length(self, value: Real) -> Real {
        value * (self.scaling.scale as Real)
    }
}

fn aperture_rect_sketch<M>(
    rect: &Rectangular,
    metadata: M,
    unit_scale: Real,
    aperture_transform: ApertureTransform,
    rounded: bool,
) -> Profile<M>
where
    M: Clone + Debug + Send + Sync,
{
    let width = aperture_transform.scale_length((rect.x as Real) * unit_scale);
    let height = aperture_transform.scale_length((rect.y as Real) * unit_scale);
    let mut sketch = if rounded {
        Profile::rounded_rectangle(
            width,
            height,
            width.min(height) * 0.5,
            16,
            metadata.clone(),
        )
    } else {
        Profile::rectangle(width, height, metadata.clone())
    }
    .translate(-width * 0.5, -height * 0.5, 0.0);

    if let Some(hole_diameter) = rect.hole_diameter {
        sketch = sketch.difference(&Profile::circle(
            aperture_transform.scale_length((hole_diameter as Real) * unit_scale) * 0.5,
            64,
            metadata,
        ));
    }

    sketch
}

fn trace_to_sketch<M>(
    aperture: &Aperture,
    start: Coord<Real>,
    end: Coord<Real>,
    metadata: M,
    unit_scale: Real,
    aperture_transform: ApertureTransform,
) -> Option<Profile<M>>
where
    M: Clone + Debug + Send + Sync,
{
    if nearly_same(start, end) {
        return flash_to_sketch(aperture, start, metadata, unit_scale, aperture_transform);
    }

    match aperture {
        Aperture::Circle(circle) => {
            let radius =
                aperture_transform.scale_length((circle.diameter as Real) * unit_scale) * 0.5;
            Some(capsule_sketch(start, end, radius, metadata))
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
            polygon_from_coords(convex_hull(swept_points), metadata)
        },
        Aperture::Macro(..) => None,
    }
}

fn trace_path_to_sketch<M>(
    aperture: &Aperture,
    path: &[Coord<Real>],
    metadata: M,
    unit_scale: Real,
    aperture_transform: ApertureTransform,
) -> Option<Profile<M>>
where
    M: Clone + Debug + Send + Sync,
{
    if path.len() < 2 {
        return None;
    }

    match aperture {
        Aperture::Circle(circle) => {
            let radius =
                aperture_transform.scale_length((circle.diameter as Real) * unit_scale) * 0.5;
            circular_swept_path_sketch(path, radius, metadata)
        },
        Aperture::Rectangle(_) | Aperture::Obround(_) | Aperture::Polygon(_) => None,
        Aperture::Macro(..) => None,
    }
}

/// Approximate an aperture stroke along an already-finite Gerber centerline as a
/// single region before it reaches hypercurve boolean composition.
///
/// Gerber import is an API boundary with decimal coordinates, so this is a
/// finite-output construction rather than internal topology. Building one
/// conservative swept envelope avoids the numerically fragile cascade of chord
/// capsule booleans while preserving the broad Gerber trace shape. The finite
/// boundary discipline follows Hobby, "Practical Segment Intersection with
/// Finite Precision Output," *Computational Geometry* 13(4), 1999
/// (<https://doi.org/10.1016/S0925-7721(99)00021-8>).
fn circular_swept_path_sketch<M>(
    path: &[Coord<Real>],
    radius: Real,
    metadata: M,
) -> Option<Profile<M>>
where
    M: Clone + Debug + Send + Sync,
{
    if radius <= tolerance() {
        return polygon_from_coords(path.to_vec(), metadata);
    }

    let points = path
        .iter()
        .flat_map(|center| {
            circle_points(radius, 16).into_iter().map(|point| Coord {
                x: center.x + point.x,
                y: center.y + point.y,
            })
        })
        .collect::<Vec<_>>();
    polygon_from_coords(convex_hull(points), metadata)
}

fn aperture_outline_points(
    aperture: &Aperture,
    unit_scale: Real,
    aperture_transform: ApertureTransform,
) -> Option<Vec<Coord<Real>>> {
    let points = match aperture {
        Aperture::Circle(circle) => {
            let radius =
                aperture_transform.scale_length((circle.diameter as Real) * unit_scale) * 0.5;
            circle_points(radius, 64)
        },
        Aperture::Rectangle(rect) => {
            let hw = aperture_transform.scale_length((rect.x as Real) * unit_scale) * 0.5;
            let hh = aperture_transform.scale_length((rect.y as Real) * unit_scale) * 0.5;
            vec![
                Coord { x: -hw, y: -hh },
                Coord { x: hw, y: -hh },
                Coord { x: hw, y: hh },
                Coord { x: -hw, y: hh },
            ]
        },
        Aperture::Obround(rect) => {
            let width = aperture_transform.scale_length((rect.x as Real) * unit_scale);
            let height = aperture_transform.scale_length((rect.y as Real) * unit_scale);
            rounded_rect_points(width, height, width.min(height) * 0.5, 16)
        },
        Aperture::Polygon(polygon) => {
            let radius =
                aperture_transform.scale_length((polygon.diameter as Real) * unit_scale) * 0.5;
            let rotation = polygon.rotation.unwrap_or(0.0) as Real;
            let Some(rotation) = hdegrees_to_radians(rotation) else {
                return None;
            };
            (0..polygon.vertices)
                .filter_map(|i| {
                    let theta = TAU * (i as Real) / (polygon.vertices as Real) + rotation;
                    origin_circle_coord(radius, theta)
                })
                .collect()
        },
        Aperture::Macro(..) => return None,
    };

    Some(
        points
            .into_iter()
            .map(|point| apply_aperture_transform_to_coord(point, aperture_transform))
            .collect(),
    )
}

fn capsule_sketch<M>(
    start: Coord<Real>,
    end: Coord<Real>,
    radius: Real,
    metadata: M,
) -> Profile<M>
where
    M: Clone + Debug + Send + Sync,
{
    let length = hxy_distance((start.x, start.y), (end.x, end.y)).unwrap_or(0.0);
    if length <= tolerance() {
        return Profile::circle(radius, 64, metadata).translate(start.x, start.y, 0.0);
    }

    let Some((ux, uy)) = hxy_unit_direction((start.x, start.y), (end.x, end.y)) else {
        return Profile::circle(radius, 64, metadata).translate(start.x, start.y, 0.0);
    };
    let nx = -uy;
    let ny = ux;
    let start_angle = ny.atan2(nx);
    let end_angle = (-ny).atan2(-nx);
    let mut points = vec![
        Coord {
            x: start.x + radius * nx,
            y: start.y + radius * ny,
        },
        Coord {
            x: end.x + radius * nx,
            y: end.y + radius * ny,
        },
    ];
    points.extend(
        arc_points(end, radius, start_angle, end_angle, true, 24)
            .into_iter()
            .skip(1),
    );
    points.push(Coord {
        x: start.x - radius * nx,
        y: start.y - radius * ny,
    });
    points.extend(
        arc_points(start, radius, end_angle, start_angle, true, 24)
            .into_iter()
            .skip(1),
    );
    polygon_from_coords(points, metadata.clone()).unwrap_or_else(|| Profile::empty(metadata))
}

fn approximate_arc(
    start: Coord<Real>,
    end: Coord<Real>,
    offset: Coord<Real>,
    mode: InterpolationMode,
) -> Vec<Coord<Real>> {
    let center = Coord {
        x: start.x + offset.x,
        y: start.y + offset.y,
    };
    let radius = hxy_distance((start.x, start.y), (center.x, center.y)).unwrap_or(0.0);
    if radius <= tolerance() {
        return vec![end];
    }

    let start_angle = (start.y - center.y).atan2(start.x - center.x);
    let mut end_angle = (end.y - center.y).atan2(end.x - center.x);
    let clockwise = mode == InterpolationMode::ClockwiseCircular;

    if nearly_same(start, end) {
        end_angle = if clockwise {
            start_angle - TAU
        } else {
            start_angle + TAU
        };
    }

    let points = arc_points(center, radius, start_angle, end_angle, clockwise, 32);
    points.into_iter().skip(1).collect()
}

fn arc_points(
    center: Coord<Real>,
    radius: Real,
    start_angle: Real,
    end_angle: Real,
    clockwise: bool,
    min_segments: usize,
) -> Vec<Coord<Real>> {
    let mut sweep = end_angle - start_angle;
    if clockwise && sweep >= 0.0 {
        sweep -= TAU;
    } else if !clockwise && sweep <= 0.0 {
        sweep += TAU;
    }

    let segments = ((sweep.abs() / (PI / 24.0)).ceil() as usize).max(min_segments);
    (0..=segments)
        .map(|i| {
            let angle = start_angle + sweep * (i as Real) / (segments as Real);
            coord_on_circle(center, radius, angle).unwrap_or(center)
        })
        .collect()
}

fn circle_points(radius: Real, segments: usize) -> Vec<Coord<Real>> {
    (0..segments)
        .map(|i| {
            let theta = TAU * (i as Real) / (segments as Real);
            origin_circle_coord(radius, theta).unwrap_or(Coord { x: 0.0, y: 0.0 })
        })
        .collect()
}

fn rounded_rect_points(
    width: Real,
    height: Real,
    radius: Real,
    corner_segments: usize,
) -> Vec<Coord<Real>> {
    let radius = radius.min(width * 0.5).min(height * 0.5);
    if radius <= tolerance() {
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
        (-width * 0.5 + radius, height * 0.5 - radius, PI * 0.5),
        (-width * 0.5 + radius, -height * 0.5 + radius, PI),
        (width * 0.5 - radius, -height * 0.5 + radius, PI * 1.5),
    ];
    centers
        .into_iter()
        .flat_map(|(cx, cy, start)| {
            (0..=corner_segments).map(move |i| {
                let theta = start + (PI * 0.5) * (i as Real) / (corner_segments as Real);
                coord_on_circle(Coord { x: cx, y: cy }, radius, theta)
                    .unwrap_or(Coord { x: cx, y: cy })
            })
        })
        .collect()
}

fn polygon_from_coords<M>(mut points: Vec<Coord<Real>>, metadata: M) -> Option<Profile<M>>
where
    M: Clone + Debug + Send + Sync,
{
    if points.len() < 3 {
        return None;
    }
    if points.first() != points.last() {
        points.push(points[0]);
    }
    let points = points
        .into_iter()
        .map(|coord| [coord.x, coord.y])
        .collect::<Vec<_>>();
    Contour2::from_finite_ring(&points)
        .ok()
        .map(|contour| Profile::from_contour(contour, metadata))
}

fn point_from_coord(coord: Coord<Real>) -> [Real; 2] {
    [coord.x, coord.y]
}

fn repeat_sketches<M>(
    sketch: Profile<M>,
    step_repeat: Option<StepRepeatState>,
) -> Vec<Profile<M>>
where
    M: Clone + Debug + Send + Sync,
{
    let Some(step_repeat) = step_repeat else {
        return vec![sketch];
    };

    let mut sketches = Vec::new();
    for x in 0..step_repeat.repeat_x {
        for y in 0..step_repeat.repeat_y {
            sketches.push(sketch.translate(
                (x as Real) * step_repeat.distance_x,
                (y as Real) * step_repeat.distance_y,
                0.0,
            ));
        }
    }
    sketches
}

fn apply_aperture_transform<M>(
    sketch: Profile<M>,
    aperture_transform: ApertureTransform,
) -> Profile<M>
where
    M: Clone + Debug + Send + Sync,
{
    let (sx, sy) = match aperture_transform.mirroring {
        Mirroring::None => (1.0, 1.0),
        Mirroring::X => (-1.0, 1.0),
        Mirroring::Y => (1.0, -1.0),
        Mirroring::XY => (-1.0, -1.0),
    };
    sketch
        .scale(sx, sy, 1.0)
        .rotate(0.0, 0.0, aperture_transform.rotation.rotation as Real)
}

fn apply_aperture_transform_to_coord(
    point: Coord<Real>,
    aperture_transform: ApertureTransform,
) -> Coord<Real> {
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
    rotate_coord(point, aperture_transform.rotation.rotation as Real)
}

fn rotate_coord(point: Coord<Real>, degrees: Real) -> Coord<Real> {
    let Some(radians) = hdegrees_to_radians(degrees) else {
        return point;
    };
    let Some((sin, cos)) = hangle_sin_cos(radians) else {
        return point;
    };
    let Some(x_cos) = hreal_mul(point.x, cos) else {
        return point;
    };
    let Some(y_sin) = hreal_mul(point.y, sin) else {
        return point;
    };
    let Some(x) = hreal_sub(x_cos, y_sin) else {
        return point;
    };
    let Some(x_sin) = hreal_mul(point.x, sin) else {
        return point;
    };
    let Some(y) = hreal_affine(x_sin, cos, point.y) else {
        return point;
    };
    Coord { x, y }
}

fn image_rotation_degrees(rotation: ImageRotation) -> Real {
    match rotation {
        ImageRotation::None => 0.0,
        ImageRotation::CCW_90 => 90.0,
        ImageRotation::CCW_180 => 180.0,
        ImageRotation::CCW_270 => 270.0,
    }
}

fn convex_hull(mut points: Vec<Coord<Real>>) -> Vec<Coord<Real>> {
    // Andrew's monotone-chain hull keeps the finite Gerber aperture sweep
    // bounded, while turn classification is delegated to the shared hyperreal
    // orientation predicate. See Andrew, "Another Efficient Algorithm for
    // Convex Hulls in Two Dimensions," Information Processing Letters 9(5),
    // 1979, and Yap, "Towards Exact Geometric Computation," Computational
    // Geometry 7(1-2), 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    points.sort_by(|a, b| a.x.total_cmp(&b.x).then_with(|| a.y.total_cmp(&b.y)));
    points.dedup_by(|a, b| nearly_same(*a, *b));

    if points.len() <= 3 {
        return points;
    }

    let mut lower = Vec::new();
    for point in &points {
        while lower.len() >= 2
            && !is_left_turn(lower[lower.len() - 2], lower[lower.len() - 1], *point)
        {
            lower.pop();
        }
        lower.push(*point);
    }

    let mut upper = Vec::new();
    for point in points.iter().rev() {
        while upper.len() >= 2
            && !is_left_turn(upper[upper.len() - 2], upper[upper.len() - 1], *point)
        {
            upper.pop();
        }
        upper.push(*point);
    }

    lower.pop();
    upper.pop();
    lower.extend(upper);
    lower
}

fn is_left_turn(origin: Coord<Real>, a: Coord<Real>, b: Coord<Real>) -> bool {
    matches!(
        hxy_orientation_sign((origin.x, origin.y), (a.x, a.y), (b.x, b.y)),
        Some(hyperreal::RealSign::Positive)
    )
}

fn nearly_same(a: Coord<Real>, b: Coord<Real>) -> bool {
    (a.x - b.x).abs() <= tolerance() && (a.y - b.y).abs() <= tolerance()
}

fn unit_scale(unit: Unit) -> Real {
    match unit {
        Unit::Millimeters => 1.0,
        Unit::Inches => 25.4,
    }
}

#[cfg(test)]
mod tests {
    use super::{Coord, FromGerber, ToGerber, convex_hull};
    use crate::csg::CSG;
    use crate::float_types::{PI, Real};
    use crate::sketch::Profile;

    fn assert_bounds_close(
        sketch: &Profile<()>,
        min_x: Real,
        min_y: Real,
        max_x: Real,
        max_y: Real,
        epsilon: Real,
    ) {
        let bounds = sketch.bounding_box();
        assert!((bounds.mins.x - min_x).abs() < epsilon);
        assert!((bounds.mins.y - min_y).abs() < epsilon);
        assert!((bounds.maxs.x - max_x).abs() < epsilon);
        assert!((bounds.maxs.y - max_y).abs() < epsilon);
    }

    fn native_region_area(sketch: &Profile<()>) -> Real {
        sketch
            .region_profiles()
            .iter()
            .map(|profile| profile.projected_filled_area())
            .sum()
    }

    #[test]
    fn gerber_convex_hull_uses_hyperreal_orientation_predicate() {
        let hull = convex_hull(vec![
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 1.0e-12, y: 0.0 },
            Coord { x: 1.0, y: 0.0 },
            Coord { x: 1.0, y: 1.0 },
            Coord { x: 0.0, y: 1.0 },
            Coord { x: 0.5, y: 0.5 },
        ]);

        assert_eq!(hull.len(), 4);
        assert!(
            hull.iter()
                .all(|point| point.x.is_finite() && point.y.is_finite())
        );
        assert!(hull.iter().any(|point| point.x == 0.0 && point.y == 0.0));
        assert!(hull.iter().any(|point| point.x == 1.0 && point.y == 0.0));
        assert!(hull.iter().any(|point| point.x == 1.0 && point.y == 1.0));
        assert!(hull.iter().any(|point| point.x == 0.0 && point.y == 1.0));
    }

    #[test]
    fn exports_and_imports_square_region() {
        let square = Profile::<()>::square(5.0, ());

        let gerber = square.to_gerber().unwrap();
        let gerber_text = String::from_utf8(gerber.clone()).unwrap();
        assert!(gerber_text.contains("%FSLAX46Y46*%"));
        assert!(gerber_text.contains("G36*"));
        assert!(gerber_text.contains("G37*"));

        let parsed = Profile::<()>::from_gerber(&gerber, ()).unwrap();
        assert_bounds_close(&parsed, 0.0, 0.0, 5.0, 5.0, 1.0e-9);
    }

    #[test]
    fn imports_region_as_native_hypercurve_region() {
        let gerber = b"G04 region*\n%MOMM*%\n%FSLAX46Y46*%\nG36*\nX0Y0D02*\nX4000000Y0D01*\nX4000000Y3000000D01*\nX0Y3000000D01*\nX0Y0D01*\nG37*\nM02*\n";

        let parsed = Profile::<()>::from_gerber(gerber, ()).unwrap();
        assert_eq!(parsed.material_contour_count(), 1);
        assert!(!parsed.as_region().is_empty());

        assert_bounds_close(&parsed, 0.0, 0.0, 4.0, 3.0, 1.0e-9);
    }

    #[test]
    fn imports_circle_flash() {
        let gerber = b"G04 flash*\n%MOMM*%\n%FSLAX46Y46*%\n%ADD10C,2*%\nD10*\nX3000000Y4000000D03*\nM02*\n";

        let parsed = Profile::<()>::from_gerber(gerber, ()).unwrap();
        assert_bounds_close(&parsed, 2.0, 3.0, 4.0, 5.0, 1.0e-6);
    }

    #[test]
    fn imports_circular_trace_as_capsule() {
        let gerber = b"G04 trace*\n%MOMM*%\n%FSLAX46Y46*%\n%ADD10C,1*%\nD10*\nX0Y0D02*\nX4000000Y0D01*\nM02*\n";

        let parsed = Profile::<()>::from_gerber(gerber, ()).unwrap();
        assert_bounds_close(&parsed, -0.5, -0.5, 4.5, 0.5, 1.0e-6);
    }

    #[test]
    fn imports_rectangular_trace_as_sweep() {
        let gerber = b"G04 trace*\n%MOMM*%\n%FSLAX46Y46*%\n%ADD10R,1X2*%\nD10*\nX0Y0D02*\nX4000000Y0D01*\nM02*\n";

        let parsed = Profile::<()>::from_gerber(gerber, ()).unwrap();
        assert_bounds_close(&parsed, -0.5, -1.0, 4.5, 1.0, 1.0e-6);
    }

    #[test]
    fn imports_arc_trace() {
        let gerber = b"G04 arc*\n%MOMM*%\n%FSLAX46Y46*%\n%ADD10C,0.2*%\nD10*\nX1000000Y0D02*\nG03X0Y1000000I-1000000J0D01*\nM02*\n";

        let parsed = Profile::<()>::from_gerber(gerber, ()).unwrap();
        let bounds = parsed.bounding_box();
        let (min_x, min_y, max_x, max_y) =
            (bounds.mins.x, bounds.mins.y, bounds.maxs.x, bounds.maxs.y);
        assert!(min_x >= -0.11);
        assert!(min_y >= -0.11);
        assert!((max_x - 1.1).abs() < 0.02);
        assert!((max_y - 1.1).abs() < 0.02);
    }

    #[test]
    fn imports_step_repeat() {
        let gerber = b"G04 step repeat*\n%MOMM*%\n%FSLAX46Y46*%\n%ADD10C,1*%\nD10*\n%SRX2Y2I2J3*%\nX0Y0D03*\n%SR*%\nM02*\n";

        let parsed = Profile::<()>::from_gerber(gerber, ()).unwrap();
        assert_bounds_close(&parsed, -0.5, -0.5, 2.5, 3.5, 1.0e-6);
    }

    #[test]
    fn imports_load_rotation_for_flashes() {
        let gerber = b"G04 rotated flash*\n%MOMM*%\n%FSLAX46Y46*%\n%ADD10R,1X2*%\nD10*\n%LR90*%\nX0Y0D03*\nM02*\n";

        let parsed = Profile::<()>::from_gerber(gerber, ()).unwrap();
        assert_bounds_close(&parsed, -1.0, -0.5, 1.0, 0.5, 1.0e-6);
    }

    #[test]
    fn imports_aperture_holes() {
        let gerber = b"G04 aperture hole*\n%MOMM*%\n%FSLAX46Y46*%\n%ADD10C,4X2*%\nD10*\nX0Y0D03*\nM02*\n";

        let parsed = Profile::<()>::from_gerber(gerber, ()).unwrap();
        let area = native_region_area(&parsed);
        assert!((area - (PI * 3.0)).abs() < 0.05);
    }

    #[test]
    fn imports_clear_polarity_as_difference() {
        let gerber = b"G04 clear polarity*\n%MOMM*%\n%FSLAX46Y46*%\n%LPD*%\nG36*\nX0Y0D02*\nX4000000Y0D01*\nX4000000Y4000000D01*\nX0Y4000000D01*\nX0Y0D01*\nG37*\n%ADD10R,2X2*%\nD10*\n%LPC*%\nX2000000Y2000000D03*\nM02*\n";

        let parsed = Profile::<()>::from_gerber(gerber, ()).unwrap();
        let area = native_region_area(&parsed);
        assert!((area - 12.0).abs() < 1.0e-6);
    }

    #[test]
    fn exports_empty_sketch_without_area_commands() {
        let sketch = Profile::<()>::empty(());

        let gerber = String::from_utf8(sketch.to_gerber().unwrap()).unwrap();
        assert!(gerber.ends_with("M02*\n"));
    }
}
