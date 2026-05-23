//! SVG input and output.

use crate::csg::CSG;
use crate::hyper_math::{Real, hreal_f64_gt, hreal_from_f64, hreal_to_f64};
use crate::sketch::Profile;
use hypercurve::{
    Classification, Contour2, CurvePolicy, CurveString2, FiniteProjectionOptions,
    FiniteRegionProfile2, Point2, Region2, Segment2,
};
use std::fmt::Debug;
use std::ops::Add;
use svg::node::element::path;

use super::IoError;

/// Finite SVG path command runs parsed at the file-format boundary.
///
/// The parser keeps path coordinates in a local representation until they are
/// promoted into `hypercurve::Contour2`/`CurveString2`; it does not use
/// a finite polyline backend as an intermediate CAD container. This follows Yap,
/// "Towards Exact Geometric Computation," *Computational Geometry* 7(1-2), 1997
/// (<https://doi.org/10.1016/0925-7721(95)00040-2>), with SVG path syntax from
/// the W3C SVG 1.1 path-data specification.
#[derive(Clone, Debug, PartialEq)]
struct SvgCoord {
    x: f64,
    y: f64,
}

impl SvgCoord {
    const fn zero() -> Self {
        Self { x: 0.0, y: 0.0 }
    }
}

fn finite_svg_scalar(value: f64, label: &str) -> Result<f64, IoError> {
    hreal_from_f64(value).map_err(|error| {
        IoError::MalformedInput(format!(
            "SVG {label} must be finite hyperreal boundary data: {error:?}"
        ))
    })?;
    Ok(value)
}

fn positive_svg_scalar(value: f64, label: &str) -> Result<f64, IoError> {
    finite_svg_scalar(value, label)?;
    if !hreal_f64_gt(value, 0.0) {
        return Err(IoError::MalformedInput(format!(
            "SVG {label} must be positive"
        )));
    }
    Ok(value)
}

fn nonnegative_svg_scalar(value: f64, label: &str) -> Result<f64, IoError> {
    finite_svg_scalar(value, label)?;
    if hreal_f64_gt(0.0, value) {
        return Err(IoError::MalformedInput(format!(
            "SVG {label} must be non-negative"
        )));
    }
    Ok(value)
}

fn svg_real(value: f64, label: &str) -> Result<Real, IoError> {
    hreal_from_f64(value).map_err(|error| {
        IoError::MalformedInput(format!(
            "SVG {label} must be finite hyperreal boundary data: {error:?}"
        ))
    })
}

fn finite_svg_coord(x: f64, y: f64, label: &str) -> Result<SvgCoord, IoError> {
    Ok(SvgCoord {
        x: finite_svg_scalar(x, &format!("{label}.x"))?,
        y: finite_svg_scalar(y, &format!("{label}.y"))?,
    })
}

fn validate_svg_runs(runs: &SvgPathRuns) -> Result<(), IoError> {
    for (run_index, run) in runs.runs.iter().enumerate() {
        for (point_index, point) in run.iter().enumerate() {
            finite_svg_coord(point.x, point.y, &format!("path[{run_index}][{point_index}]"))?;
        }
    }
    Ok(())
}

impl Add for SvgCoord {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
    }
}

struct SvgPathRuns {
    runs: Vec<Vec<SvgCoord>>,
}

impl SvgPathRuns {
    fn new() -> Self {
        Self { runs: Vec::new() }
    }
}

fn path_run_is_closed(run: &[SvgCoord]) -> bool {
    run.len() >= 2 && run.first() == run.last()
}

/// A helper struct to build finite path command runs from SVG Path commands.
///
/// The API aims to be compatible with the [SVG 1.1 Paths specification][svg-paths].
/// The single instance of this struct is meant to be used for building paths from a single
/// `d` attribute of an SVG `<path/>`.
///
/// In method documentation:
/// - *Current path* refers to the part of the path that was started by the most recent `M`/`m` (moveto/moveby) command
/// - *Current point* refers to the last point of the current path
/// - Method names correspond to SVG Path commands
/// - Method suffix `_to` indicates a command that uses absolute coordinates
/// - Method suffix `_by` indicates a command that uses relative coordinates
///
/// Parsed finite runs are promoted into `hypercurve::Region2` or
/// `hypercurve::CurveString2` at the Profile boundary. Keeping SVG's finite
/// interchange coordinates at the boundary while native CAD ownership lives in
/// hyper geometry follows Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7(1-2), 1997
/// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
///
/// **At the moment, curves are not supported.**
/// When support for curves is implemented, the underlying data structure may change to accommodate that.
///
/// [svg-paths]: https://www.w3.org/TR/SVG11/paths.html
struct PathBuilder {
    inner: SvgPathRuns,
}

impl From<PathBuilder> for SvgPathRuns {
    fn from(val: PathBuilder) -> Self {
        val.inner
    }
}

impl PathBuilder {
    pub fn new() -> Self {
        Self {
            inner: SvgPathRuns::new(),
        }
    }

    /// Get the current position to be used for relative moves.
    fn get_position(&self) -> SvgCoord {
        self.inner
            .runs
            .last()
            .and_then(|run| run.last())
            .cloned()
            .unwrap_or(SvgCoord::zero())
    }

    /// Get a mutable reference to the current path, or an error if no path has been started.
    ///
    /// To accommodate for the semantics of [`close`], this function will automatically start a new path
    /// if the last path has 2 or more points and is closed.
    /// For this reason, using this proxy is recommended for implementing any drawing command.
    fn get_path_mut_or_fail(&mut self) -> Result<&mut Vec<SvgCoord>, IoError> {
        let start_new_path = self
            .inner
            .runs
            .last()
            .map(|p| path_run_is_closed(p))
            .unwrap_or(false);

        if start_new_path {
            self.inner.runs.push(vec![self.get_position()]);
        }

        self.inner.runs.last_mut().ok_or_else(|| {
            IoError::MalformedPath(
                "Attempted to extend the current path, but no path was started.".to_string(),
            )
        })
    }

    /// Start a new path at `point`.
    pub fn move_to(&mut self, point: SvgCoord) {
        self.inner.runs.push(vec![point]);
    }

    /// Start a new path at `delta` relative to the last point.
    /// If and only if this is the first command, the point is treated as absolute coordinates.
    pub fn move_by(&mut self, delta: SvgCoord) {
        let position = self.get_position();
        self.inner.runs.push(vec![position + delta]);
    }

    /// Extend the current path to the `point`.
    /// Can not be the first command.
    pub fn line_to(&mut self, point: SvgCoord) -> Result<(), IoError> {
        let line = self.get_path_mut_or_fail()?;
        line.push(point);
        Ok(())
    }

    /// Extend the current path by `delta` relative to the current point.
    /// Can not be the first command.
    pub fn line_by(&mut self, delta: SvgCoord) -> Result<(), IoError> {
        let position = self.get_position();
        let line = self.get_path_mut_or_fail()?;
        line.push(position + delta);
        Ok(())
    }

    /// Extend the current path with a horizontal move to `x`.
    /// Can not be the first command.
    pub fn hline_to(&mut self, x: f64) -> Result<(), IoError> {
        let SvgCoord { y, .. } = self.get_position();
        let line = self.get_path_mut_or_fail()?;
        line.push(SvgCoord { x, y });
        Ok(())
    }

    /// Extend the current path with a horizontal move by `dx` relative to the current point.
    /// Can not be the first command.
    pub fn hline_by(&mut self, dx: f64) -> Result<(), IoError> {
        let SvgCoord { x, y } = self.get_position();
        let line = self.get_path_mut_or_fail()?;
        line.push(SvgCoord { x: x + dx, y });
        Ok(())
    }

    /// Extend the current path with a vertical move to `y`.
    /// Can not be the first command.
    pub fn vline_to(&mut self, y: f64) -> Result<(), IoError> {
        let SvgCoord { x, .. } = self.get_position();
        let line = self.get_path_mut_or_fail()?;
        line.push(SvgCoord { x, y });
        Ok(())
    }

    /// Extend the current path with a vertical move by `dy` relative to the current point.
    /// Can not be the first command.
    pub fn vline_by(&mut self, dy: f64) -> Result<(), IoError> {
        let SvgCoord { x, y } = self.get_position();
        let line = self.get_path_mut_or_fail()?;
        line.push(SvgCoord { x, y: y + dy });
        Ok(())
    }

    /// Close the current path.
    ///
    /// In SVG, closing a path using a Close command is different from closing a path using a drawing command.
    /// Specifically, [line caps are handled differently][svg-paths-close] in such cases.
    /// For the sake of simplicity, this API *does not differentiate these cases* at the moment.
    ///
    /// To follow SVG specification:
    /// - If this is followed by a moveto/moveby command, they determine the start of the new path.
    /// - If this is followed by any other command, the new path starts at the end of the last path.
    ///
    /// Can not be the first command.
    ///
    /// [svg-paths-close]: https://www.w3.org/TR/SVG11/paths.html#PathDataClosePathCommand
    pub fn close(&mut self) -> Result<(), IoError> {
        // TODO: maybe make sure there are at least 3 points?
        let line = self.get_path_mut_or_fail()?;
        if let (Some(first), Some(last)) = (line.first().cloned(), line.last().cloned()) {
            if first != last {
                line.push(first);
            }
        }
        Ok(())
    }

    /// Extend the current path with a quadratic Bézier curve from the current point using absolute coordinates.
    ///
    /// - Using the current point as the start point
    /// - Using `control` as the control point
    /// - Using `end` as the end point
    ///
    /// Can not be the first command.
    ///
    /// **Not implemented**
    pub fn quadratic_curve_to(
        &mut self,
        _control: SvgCoord,
        _end: SvgCoord,
    ) -> Result<(), IoError> {
        Err(IoError::Unimplemented(
            "quadratic curveto (absolute quadratic Bézier curve)".to_string(),
        ))
    }

    /// Extend the current path with a quadratic Bézier curve from the current point using coordinates relative
    /// to the current point.
    ///
    /// - Using the current point as the start point
    /// - Using `control` as the control point
    /// - Using `end` as the end point
    ///
    /// Can not be the first command.
    ///
    /// **Not implemented**
    pub fn quadratic_curve_by(
        &mut self,
        _control: SvgCoord,
        _end: SvgCoord,
    ) -> Result<(), IoError> {
        Err(IoError::Unimplemented(
            "quadratic curveby (relative quadratic Bézier curve)".to_string(),
        ))
    }

    /// Extend the current path with a *smooth* quadratic Bézier curve from the current point using absolute coordinates.
    ///
    /// - Using the current point as the start point
    /// - Using a reflection of `control` of the previous command relative to the current point as the control point
    ///   - If there is no previous command, or if the previous command is not one of [`quadratic_curve_to`],
    ///     [`quadratic_curve_by`], [`quadratic_smooth_curve_to`], [`quadratic_smooth_curve_by`], current point
    ///     is used as the first control point
    /// - Using `end` as the end point
    ///
    /// Can not be the first command.
    ///
    /// **Not implemented**
    pub fn quadratic_smooth_curve_to(&mut self, _end: SvgCoord) -> Result<(), IoError> {
        Err(IoError::Unimplemented(
            "quadratic smooth curveto (absolute quadratic Bézier curve with a reflected control point)".to_string()
        ))
    }

    /// Extend the current path with a *smooth* quadratic Bézier curve from the current point using coordinates relative
    /// to the current point.
    ///
    /// - Using the current point as the start point
    /// - Using a reflection of `control` of the previous command relative to the current point as the control point
    ///   - If there is no previous command, or if the previous command is not one of [`quadratic_curve_to`],
    ///     [`quadratic_curve_by`], [`quadratic_smooth_curve_to`], [`quadratic_smooth_curve_by`], current point
    ///     is used as the first control point
    /// - Using `end` as the end point
    ///
    /// Can not be the first command.
    ///
    /// **Not implemented**
    pub fn quadratic_smooth_curve_by(&mut self, _end: SvgCoord) -> Result<(), IoError> {
        Err(IoError::Unimplemented(
            "quadratic smooth curveby (relative quadratic Bézier curve with a reflected control point)".to_string()
        ))
    }

    /// Extend the current path with a cubic Bézier curve from the current point using absolute coordinates.
    ///
    /// - Using the current point as the start point
    /// - Using `control_start` as the first control point
    /// - Using `control_end` as the second control point
    /// - Using `end` as the end point
    ///
    /// Can not be the first command.
    ///
    /// **Not implemented**
    pub fn curve_to(
        &mut self,
        _control_start: SvgCoord,
        _control_end: SvgCoord,
        _end: SvgCoord,
    ) -> Result<(), IoError> {
        Err(IoError::Unimplemented(
            "curveto (absolute cubic Bézier curve)".to_string(),
        ))
    }

    /// Extend the current path with a cubic Bézier curve from the current point using coordinates relative
    /// to the current point.
    ///
    /// - Using the current point as the start point
    /// - Using `control_start` as the first control point
    /// - Using `control_end` as the second control point
    /// - Using `end` as the end point
    ///
    /// Can not be the first command.
    ///
    /// **Not implemented**
    pub fn curve_by(
        &mut self,
        _control_start: SvgCoord,
        _control_end: SvgCoord,
        _end: SvgCoord,
    ) -> Result<(), IoError> {
        Err(IoError::Unimplemented(
            "curveby (relative cubic Bézier curve)".to_string(),
        ))
    }

    /// Extend the current path with a *smooth* cubic Bézier curve from the current point using absolute coordinates.
    ///
    /// - Using the current point as the start point
    /// - Using a reflection of `control_end` of the previous command relative to the current point as the first control point
    ///   - If there is no previous command, or if the previous command is not one of [`curve_to`], [`curve_by`],
    ///     [`smooth_curve_to`], [`smooth_curve_by`], current point is used as the first control point
    /// - Using `control_end` as the second control point
    /// - Using `end` as the end point
    ///
    /// Can not be the first command.
    ///
    /// **Not implemented**
    pub fn smooth_curve_to(
        &mut self,
        _control_end: SvgCoord,
        _end: SvgCoord,
    ) -> Result<(), IoError> {
        Err(IoError::Unimplemented(
            "smooth curveto (absolute cubic Bézier curve with a reflected start control point)".to_string()
        ))
    }

    /// Extend the current path with a *smooth* cubic Bézier curve from the current point using coordinates relative
    /// to the current point.
    ///
    /// - Using the current point as the start point
    /// - Using a reflection of `control_end` of the previous command relative to the current point as the first control point
    ///   - If there is no previous command, or if the previous command is not one of [`curve_to`], [`curve_by`],
    ///     [`smooth_curve_to`], [`smooth_curve_by`], current point is used as the first control point
    /// - Using `control_end` as the second control point
    /// - Using `end` as the end point
    ///
    /// Can not be the first command.
    ///
    /// **Not implemented**
    pub fn smooth_curve_by(
        &mut self,
        _control_end: SvgCoord,
        _end: SvgCoord,
    ) -> Result<(), IoError> {
        Err(IoError::Unimplemented(
            "smooth curveby (relative cubic Bézier curve with a reflected start control point)".to_string()
        ))
    }

    /// Extend the current path with an elliptical arc from the current point using absolute coordinates.
    ///
    /// See [SVG Path Data - Elliptical Arc Curve commands][svg-arc].
    ///
    /// Can not be the first command.
    ///
    /// **Not implemented**
    ///
    /// Developers: see also [SVG Elliptical Arc Implementation Notes][svg-arc-impl-notes]
    ///
    /// [svg-arc]: https://www.w3.org/TR/SVG11/paths.html#PathDataEllipticalArcCommands
    /// [svg-arc-impl-notes]: https://www.w3.org/TR/SVG11/implnote.html#ArcImplementationNotes
    pub fn elliptical_arc_to(
        &mut self,
        _rx: f64,
        _ry: f64,
        _x_axis_rotation: f64,
        _large_arc_flag: bool,
        _sweep_flag: bool,
        _end: SvgCoord,
    ) -> Result<(), IoError> {
        Err(IoError::Unimplemented("elliptical arc to".to_string()))
    }

    /// Extend the current path with an elliptical arc from the current point using coordinates relative
    /// to the current point.
    ///
    /// See [SVG Path Data - Elliptical Arc Curve commands][svg-arc].
    ///
    /// Can not be the first command.
    ///
    /// **Not implemented**
    ///
    /// Developers: see also [SVG Elliptical Arc Implementation Notes][svg-arc-impl-notes]
    ///
    /// [svg-arc]: https://www.w3.org/TR/SVG11/paths.html#PathDataEllipticalArcCommands
    /// [svg-arc-impl-notes]: https://www.w3.org/TR/SVG11/implnote.html#ArcImplementationNotes
    pub fn elliptical_arc_by(
        &mut self,
        _rx: f64,
        _ry: f64,
        _x_axis_rotation: f64,
        _large_arc_flag: bool,
        _sweep_flag: bool,
        _end: SvgCoord,
    ) -> Result<(), IoError> {
        Err(IoError::Unimplemented("elliptical arc by".to_string()))
    }
}

#[allow(unused)]
pub trait FromSVG<M>: Sized {
    fn from_svg(doc: &str, metadata: M) -> Result<Self, IoError>;
}

impl<M> FromSVG<M> for Profile<M>
where
    M: Clone + Send + Sync + Debug,
{
    fn from_svg(doc: &str, metadata: M) -> Result<Self, IoError> {
        use svg::node::element::tag::{self, Type::*};
        use svg::parser::Event;

        macro_rules! expect_attr {
            ($attrs:expr, $attr:literal) => {
                $attrs.get($attr).ok_or_else(|| {
                    IoError::MalformedInput(format!("Missing attribute {}", $attr))
                })
            };
        }

        macro_rules! option_attr {
            ($attrs:expr, $attr:literal) => {
                $attrs.get($attr)
            };
        }

        let mut sketch_union = Profile::empty(metadata.clone());

        for event in svg::read(doc)? {
            match event {
                Event::Instruction(..)
                | Event::Declaration(..)
                | Event::Text(..)
                | Event::Comment(..)
                | Event::Tag(tag::SVG, ..)
                | Event::Tag(tag::Description, ..)
                | Event::Tag(tag::Text, ..)
                | Event::Tag(tag::Title, ..) => {},

                Event::Error(error) => {
                    return Err(error.into());
                },

                Event::Tag(tag::Group, ..) => {
                    // TODO: keep track of transforms
                    // TODO: keep track of style properties
                },

                Event::Tag(tag::Path, Empty, attrs) => {
                    let data = expect_attr!(attrs, "d")?;
                    let data = path::Data::parse(data)?;
                    let runs = svg_path_to_runs(data)?;

                    // TODO: This is tricky.
                    // Whether a <path/> contains lines or polygons really depends on the current stroke and fill,
                    // which requires keeping track of them by parsing `style=""` and other attributes,
                    // and pushing/popping the current "style context" on group entry and exit.
                    //
                    // On top of that, when a <path/> is a polygon, subpaths may define additional polygons OR
                    // holes in existing polygons (and how specifically, may depend on either their winding order
                    // or on the level of nestedness).
                    // Read more / see examples here: https://developer.mozilla.org/en-US/docs/Web/SVG/Reference/Attribute/fill-rule
                    //
                    // This is a bit advanced, so (for now) this code just assumes that:
                    // - every closed subpath is a polygon (as if with solid fill and zero stroke thickness)
                    // - every unclosed subpath is a zero-width native wire (as if with no fill)
                    //
                    // Stroke expansion still requires current stroke-width; preserving the
                    // path as `CurveString2` keeps source topology available for that later pass.

                    if let Some(sketch) = sketch_from_svg_runs(runs, metadata.clone()) {
                        sketch_union = sketch_union.union(&sketch);
                    }
                },

                Event::Tag(tag::Circle, Empty, attrs) => {
                    let cx = finite_svg_scalar(expect_attr!(attrs, "cx")?.parse()?, "cx")?;
                    let cy = finite_svg_scalar(expect_attr!(attrs, "cy")?.parse()?, "cy")?;
                    let r = positive_svg_scalar(expect_attr!(attrs, "r")?.parse()?, "r")?;

                    // TODO: add a way for the user to configure this?
                    let segments = (r.ceil() as usize).max(6);

                    let sketch = Self::circle(svg_real(r, "r")?, segments, metadata.clone())
                        .translate(svg_real(cx, "cx")?, svg_real(cy, "cy")?, Real::zero());
                    sketch_union = sketch_union.union(&sketch);
                },

                Event::Tag(tag::Rectangle, Empty, attrs) => {
                    let x = finite_svg_scalar(expect_attr!(attrs, "x")?.parse()?, "x")?;
                    let y = finite_svg_scalar(expect_attr!(attrs, "y")?.parse()?, "y")?;
                    let w =
                        positive_svg_scalar(expect_attr!(attrs, "width")?.parse()?, "width")?;
                    let h = positive_svg_scalar(
                        expect_attr!(attrs, "height")?.parse()?,
                        "height",
                    )?;
                    let rx = nonnegative_svg_scalar(
                        option_attr!(attrs, "rx").map_or(Ok(0.0), |a| a.parse())?,
                        "rx",
                    )?;
                    let ry = nonnegative_svg_scalar(
                        option_attr!(attrs, "ry").map_or(Ok(0.0), |a| a.parse())?,
                        "ry",
                    )?;

                    // TODO: support rx != ry
                    let r = (rx + ry) / 2.0;

                    // TODO: add a way for the user to configure this?
                    let segments = (r.ceil() as usize).max(6);

                    let sketch = Self::rounded_rectangle(
                        svg_real(w, "width")?,
                        svg_real(h, "height")?,
                        svg_real(r, "corner radius")?,
                        segments,
                        metadata.clone(),
                    )
                    .translate(
                        svg_real(x, "x")?,
                        svg_real(y, "y")?,
                        Real::zero(),
                    );
                    sketch_union = sketch_union.union(&sketch);
                },

                Event::Tag(tag::Ellipse, Empty, attrs) => {
                    let cx = finite_svg_scalar(expect_attr!(attrs, "cx")?.parse()?, "cx")?;
                    let cy = finite_svg_scalar(expect_attr!(attrs, "cy")?.parse()?, "cy")?;
                    let rx = positive_svg_scalar(expect_attr!(attrs, "rx")?.parse()?, "rx")?;
                    let ry = positive_svg_scalar(expect_attr!(attrs, "ry")?.parse()?, "ry")?;

                    // TODO: add a way for the user to configure this?
                    let segments = (rx.max(ry).ceil() as usize).max(6);

                    let sketch = Self::ellipse(
                        svg_real(rx * 2.0, "rx")?,
                        svg_real(ry * 2.0, "ry")?,
                        segments,
                        metadata.clone(),
                    )
                    .translate(
                        svg_real(cx, "cx")?,
                        svg_real(cy, "cy")?,
                        Real::zero(),
                    );
                    sketch_union = sketch_union.union(&sketch);
                },

                Event::Tag(tag::Line, Empty, attrs) => {
                    let x1 = finite_svg_scalar(expect_attr!(attrs, "x1")?.parse()?, "x1")?;
                    let y1 = finite_svg_scalar(expect_attr!(attrs, "y1")?.parse()?, "y1")?;
                    let x2 = finite_svg_scalar(expect_attr!(attrs, "x2")?.parse()?, "x2")?;
                    let y2 = finite_svg_scalar(expect_attr!(attrs, "y2")?.parse()?, "y2")?;

                    if let Ok(wire) =
                        CurveString2::from_finite_point_iter([[x1, y1], [x2, y2]])
                    {
                        let sketch = Self::from_wire(wire, metadata.clone());
                        sketch_union = sketch_union.union(&sketch);
                    }
                },

                Event::Tag(tag::Polygon, Empty, attrs) => {
                    let points = expect_attr!(attrs, "points")?;
                    let points = svg_points_to_coords(points)?
                        .into_iter()
                        .map(|coord| [coord.x, coord.y])
                        .collect::<Vec<_>>();
                    if let Ok(contour) = Contour2::from_finite_ring(&points) {
                        sketch_union =
                            sketch_union.union(&Self::from_contour(contour, metadata.clone()));
                    }
                },

                Event::Tag(tag::Polyline, Empty, attrs) => {
                    let points = expect_attr!(attrs, "points")?;
                    let points = svg_points_to_coords(points)?;
                    if let Ok(wire) = CurveString2::from_finite_point_iter(
                        points.into_iter().map(|coord| [coord.x, coord.y]),
                    ) {
                        let sketch = Self::from_wire(wire, metadata.clone());
                        sketch_union = sketch_union.union(&sketch);
                    }
                },

                tag => {
                    // TODO: Non-empty tags should also be supported
                    return Err(IoError::Unimplemented(format!("Parsing tag {tag:?}")));
                },
            }
        }

        Ok(sketch_union)
    }
}

/// Promote one SVG path element's finite subpath runs into native Profile topology.
///
/// Closed subpaths are passed together to [`Region2::from_boundary_contours`]
/// so nested SVG rings become material/hole roles inside hypercurve rather
/// than through per-ring `Profile` booleans. Open subpaths become
/// [`CurveString2`] wires. This keeps SVG's finite coordinates at the
/// file-format boundary while the CAD state is composed from hypercurve types,
/// following Yap, "Towards Exact Geometric Computation," *Computational
/// Geometry* 7(1-2), 1997
/// (<https://doi.org/10.1016/0925-7721(95)00040-2>). The nesting reduction is
/// the point-in-polygon problem surveyed by Hormann and Agathos, "The Point in
/// Polygon Problem for Arbitrary Polygons," *Computational Geometry* 20(3),
/// 2001 (<https://doi.org/10.1016/S0925-7721(01)00012-8>), while path syntax
/// follows the W3C SVG 1.1 path-data specification.
fn sketch_from_svg_runs<M>(runs: SvgPathRuns, metadata: M) -> Option<Profile<M>>
where
    M: Clone + Send + Sync + Debug,
{
    let mut closed_runs = Vec::new();
    let mut contours = Vec::new();
    let mut wires = Vec::new();

    for run in runs.runs {
        if path_run_is_closed(&run) {
            let points = svg_run_to_points(&run);
            if let Some(contour) = contour_from_svg_closed_run(&run) {
                contours.push(contour);
                closed_runs.push(points);
            }
        } else if let Ok(wire) = CurveString2::from_finite_point_iter(
            run.into_iter().map(|coord| [coord.x, coord.y]),
        ) {
            wires.push(wire);
        }
    }

    let region = if contours.is_empty() {
        Region2::empty()
    } else {
        match Region2::from_boundary_contours(contours, &CurvePolicy::certified()) {
            Ok(Classification::Decided(region)) => region,
            Ok(Classification::Uncertain(_)) | Err(_) => {
                let mut fallback = Profile::empty(metadata.clone());
                for points in closed_runs {
                    if let Ok(contour) = Contour2::from_finite_ring(&points) {
                        fallback =
                            fallback.union(&Profile::from_contour(contour, metadata.clone()));
                    }
                }
                fallback.as_region().clone()
            },
        }
    };

    if region.is_empty() && wires.is_empty() {
        None
    } else {
        Some(Profile::from_region_and_wires(region, wires, metadata))
    }
}

fn svg_run_to_points(run: &[SvgCoord]) -> Vec<[f64; 2]> {
    run.iter().map(|coord| [coord.x, coord.y]).collect()
}

fn contour_from_svg_closed_run(run: &[SvgCoord]) -> Option<Contour2> {
    let mut points = svg_run_to_points(run);
    if points.len() < 4 {
        return None;
    }
    if points.first() == points.last() {
        points.pop();
    }
    Contour2::from_finite_ring(&points).ok()
}

#[allow(unused)]
pub trait ToSVG {
    fn to_svg(&self) -> String;
}

impl<M: Clone + Send + Sync + Debug> ToSVG for Profile<M> {
    fn to_svg(&self) -> String {
        use svg::node::element;

        let mut g = element::Group::new();

        let make_region_path = |profile: &FiniteRegionProfile2| {
            let mut data = path::Data::new();

            let rings = std::iter::once(profile.material().points())
                .chain(profile.holes().iter().map(|hole| hole.points()));
            for ring in rings {
                let mut points = ring.iter();
                let Some(start) = points.next() else {
                    continue;
                };
                data = data.move_to((start[0], start[1]));
                for point in points {
                    data = data.line_to((point[0], point[1]));
                }
                data = data.close();
            }

            element::Path::new()
                .set("fill", "black")
                .set("fill-rule", "evenodd")
                .set("stroke", "none")
                .set("d", data)
        };

        let make_wire_path = |wire: &CurveString2| {
            let Some(data) = native_wire_path_data(wire) else {
                return None;
            };
            Some(
                element::Path::new()
                    .set("fill", "none")
                    .set("stroke", "black")
                    .set("stroke-width", 1)
                    .set("vector-effect", "non-scaling-stroke")
                    .set("d", data),
            )
        };

        let projection_options = FiniteProjectionOptions::try_new(1e-3)
            .expect("positive finite projection tolerance is valid");
        let region_profiles = match self.project_region_profiles(&projection_options) {
            Ok(Classification::Decided(profiles)) => profiles,
            Ok(Classification::Uncertain(_)) | Err(_) => Vec::new(),
        };
        let native_wires = self.wires();
        // SVG export is now backed only by native hypercurve topology. Filled
        // profiles and open wires stay as `Region2`/`CurveString2` until this
        // file-format boundary; finite path commands are emitted only after
        // hypercurve projection. Material/hole profiles are grouped before
        // serialization, using the point-in-polygon topology discussed by
        // Hormann and Agathos, "The point in polygon problem for arbitrary
        // polygons," Computational Geometry 20(3), 2001
        // (<https://doi.org/10.1016/S0925-7721(01)00012-8>). Avoiding the old
        // Avoiding the old finite fallback keeps exact CAD ownership inside hypercurve, following
        // Yap, "Towards Exact Geometric Computation," Computational Geometry
        // 7(1-2), 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>).
        for profile in &region_profiles {
            g = g.add(make_region_path(profile));
        }

        for wire in native_wires {
            if let Some(path) = make_wire_path(wire) {
                g = g.add(path);
            }
        }

        let (min_x, min_y, max_x, max_y) = self
            .native_xy_bounds()
            .and_then(|(min_x, min_y, max_x, max_y)| {
                Some((
                    hreal_to_f64(&min_x)?,
                    hreal_to_f64(&min_y)?,
                    hreal_to_f64(&max_x)?,
                    hreal_to_f64(&max_y)?,
                ))
            })
            .unwrap_or((0.0, 0.0, 1.0, 1.0));
        let doc = svg::Document::new()
            .set("viewBox", (min_x, min_y, max_x - min_x, max_y - min_y))
            .add(g);

        doc.to_string()
    }
}

fn finite_svg_point(point: &Point2) -> Option<(f64, f64)> {
    Some((hreal_to_f64(point.x())?, hreal_to_f64(point.y())?))
}

/// Project a native hypercurve wire directly to SVG path commands.
///
/// This is a file-format boundary projection only: `CurveString2` remains the
/// CAD representation and circular arcs are emitted as SVG elliptical-arc
/// commands instead of being tessellated into line strings. SVG's path grammar
/// follows the W3C SVG 1.1 path-data specification, while keeping finite
/// coordinates out at the file boundary follows Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7(1-2), 1997
/// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
fn native_wire_path_data(wire: &CurveString2) -> Option<path::Data> {
    let first = wire.segments().first()?;
    let start = finite_svg_point(first.start())?;
    let mut data = path::Data::new().move_to(start);

    for segment in wire.segments() {
        match segment {
            Segment2::Line(line) => {
                data = data.line_to(finite_svg_point(line.end())?);
            },
            Segment2::Arc(arc) => {
                let radius = hreal_to_f64(arc.radius_squared_ref())?.sqrt();
                if !hreal_f64_gt(radius, 0.0) {
                    return None;
                }
                let end = finite_svg_point(arc.end())?;
                data = data.elliptical_arc_to((
                    radius,
                    radius,
                    0.0,
                    0,
                    i32::from(arc.is_clockwise()),
                    end.0,
                    end.1,
                ));
            },
        }
    }

    Some(data)
}

fn svg_path_to_runs(path_data: path::Data) -> Result<SvgPathRuns, IoError> {
    let mut builder = PathBuilder::new();

    for cmd in path_data.iter() {
        use svg::node::element::path::{Command::*, Position::*};

        macro_rules! ensure_param_count {
            ($count:expr, $div_by:expr) => {
                if $count % $div_by != 0 {
                    return Err(IoError::MalformedPath(format!("Expected the number of parameters {} to be divisible by {} in command {cmd:?}", $count, $div_by)));
                }
            };
        }

        let param_count = match cmd {
            Move(..) | Line(..) => 2,

            HorizontalLine(..) | VerticalLine(..) => 1,

            QuadraticCurve(..) => 4,
            SmoothQuadraticCurve(..) => 2,
            CubicCurve(..) => 6,
            SmoothCubicCurve(..) => 4,
            EllipticalArc(..) => 7,

            Close => {
                builder.close()?;
                continue;
            },
        };

        match cmd {
            Move(Absolute, params) => {
                ensure_param_count!(params.len(), param_count);
                let mut coords = params.chunks(param_count);

                if let Some(&[x, y]) = coords.next() {
                    builder.move_to(SvgCoord {
                        x: f64::from(x),
                        y: f64::from(y),
                    });
                }

                // Follow-up coordinates for MoveTo are implicit LineTo
                while let Some(&[x, y]) = coords.next() {
                    builder.line_to(SvgCoord {
                        x: f64::from(x),
                        y: f64::from(y),
                    })?;
                }
            },
            Move(Relative, params) => {
                ensure_param_count!(params.len(), param_count);
                let mut coords = params.chunks(param_count);

                if let Some(&[dx, dy]) = coords.next() {
                    builder.move_by(SvgCoord {
                        x: f64::from(dx),
                        y: f64::from(dy),
                    });
                }

                // Follow-up coordinates for MoveTo are implicit LineTo
                while let Some(&[dx, dy]) = coords.next() {
                    builder.line_by(SvgCoord {
                        x: f64::from(dx),
                        y: f64::from(dy),
                    })?;
                }
            },
            Line(Absolute, params) => {
                ensure_param_count!(params.len(), param_count);
                let mut coords = params.chunks(param_count);
                while let Some(&[x, y]) = coords.next() {
                    builder.line_to(SvgCoord {
                        x: f64::from(x),
                        y: f64::from(y),
                    })?;
                }
            },
            Line(Relative, params) => {
                ensure_param_count!(params.len(), param_count);
                let mut coords = params.chunks(param_count);
                while let Some(&[dx, dy]) = coords.next() {
                    builder.line_by(SvgCoord {
                        x: f64::from(dx),
                        y: f64::from(dy),
                    })?;
                }
            },
            HorizontalLine(Absolute, params) => {
                for &x in params.iter() {
                    builder.hline_to(f64::from(x))?;
                }
            },
            HorizontalLine(Relative, params) => {
                for &dx in params.iter() {
                    builder.hline_by(f64::from(dx))?;
                }
            },
            VerticalLine(Absolute, params) => {
                for &y in params.iter() {
                    builder.vline_to(f64::from(y))?;
                }
            },
            VerticalLine(Relative, params) => {
                for &dy in params.iter() {
                    builder.vline_by(f64::from(dy))?;
                }
            },

            QuadraticCurve(Absolute, params) => {
                ensure_param_count!(params.len(), param_count);
                let mut params = params.chunks(param_count);
                while let Some(&[cx, cy, x, y]) = params.next() {
                    builder.quadratic_curve_to(
                        SvgCoord {
                            x: f64::from(cx),
                            y: f64::from(cy),
                        },
                        SvgCoord {
                            x: f64::from(x),
                            y: f64::from(y),
                        },
                    )?;
                }
            },
            QuadraticCurve(Relative, params) => {
                ensure_param_count!(params.len(), param_count);
                let mut params = params.chunks(param_count);
                while let Some(&[cx, cy, x, y]) = params.next() {
                    builder.quadratic_curve_by(
                        SvgCoord {
                            x: f64::from(cx),
                            y: f64::from(cy),
                        },
                        SvgCoord {
                            x: f64::from(x),
                            y: f64::from(y),
                        },
                    )?;
                }
            },
            SmoothQuadraticCurve(Absolute, params) => {
                ensure_param_count!(params.len(), param_count);
                let mut params = params.chunks(param_count);
                while let Some(&[x, y]) = params.next() {
                    builder.quadratic_smooth_curve_to(SvgCoord {
                        x: f64::from(x),
                        y: f64::from(y),
                    })?;
                }
            },
            SmoothQuadraticCurve(Relative, params) => {
                ensure_param_count!(params.len(), param_count);
                let mut params = params.chunks(param_count);
                while let Some(&[x, y]) = params.next() {
                    builder.quadratic_smooth_curve_by(SvgCoord {
                        x: f64::from(x),
                        y: f64::from(y),
                    })?;
                }
            },

            CubicCurve(Absolute, params) => {
                ensure_param_count!(params.len(), param_count);
                let mut params = params.chunks(param_count);
                while let Some(&[c1x, c1y, c2x, c2y, x, y]) = params.next() {
                    builder.curve_to(
                        SvgCoord {
                            x: f64::from(c1x),
                            y: f64::from(c1y),
                        },
                        SvgCoord {
                            x: f64::from(c2x),
                            y: f64::from(c2y),
                        },
                        SvgCoord {
                            x: f64::from(x),
                            y: f64::from(y),
                        },
                    )?;
                }
            },
            CubicCurve(Relative, params) => {
                ensure_param_count!(params.len(), param_count);
                let mut params = params.chunks(param_count);
                while let Some(&[c1x, c1y, c2x, c2y, x, y]) = params.next() {
                    builder.curve_by(
                        SvgCoord {
                            x: f64::from(c1x),
                            y: f64::from(c1y),
                        },
                        SvgCoord {
                            x: f64::from(c2x),
                            y: f64::from(c2y),
                        },
                        SvgCoord {
                            x: f64::from(x),
                            y: f64::from(y),
                        },
                    )?;
                }
            },
            SmoothCubicCurve(Absolute, params) => {
                ensure_param_count!(params.len(), param_count);
                let mut params = params.chunks(param_count);
                while let Some(&[c2x, c2y, x, y]) = params.next() {
                    builder.smooth_curve_to(
                        SvgCoord {
                            x: f64::from(c2x),
                            y: f64::from(c2y),
                        },
                        SvgCoord {
                            x: f64::from(x),
                            y: f64::from(y),
                        },
                    )?;
                }
            },
            SmoothCubicCurve(Relative, params) => {
                ensure_param_count!(params.len(), param_count);
                let mut params = params.chunks(param_count);
                while let Some(&[c2x, c2y, x, y]) = params.next() {
                    builder.smooth_curve_by(
                        SvgCoord {
                            x: f64::from(c2x),
                            y: f64::from(c2y),
                        },
                        SvgCoord {
                            x: f64::from(x),
                            y: f64::from(y),
                        },
                    )?;
                }
            },

            EllipticalArc(Absolute, params) => {
                ensure_param_count!(params.len(), param_count);
                let mut params = params.chunks(param_count);
                while let Some(&[rx, ry, x_rot, large_arc, sweep, x, y]) = params.next() {
                    let large_arc = large_arc == 1.0;
                    let sweep = sweep == 1.0;
                    builder.elliptical_arc_to(
                        f64::from(rx),
                        f64::from(ry),
                        f64::from(x_rot),
                        large_arc,
                        sweep,
                        SvgCoord {
                            x: f64::from(x),
                            y: f64::from(y),
                        },
                    )?;
                }
            },
            EllipticalArc(Relative, params) => {
                ensure_param_count!(params.len(), param_count);
                let mut params = params.chunks(param_count);
                while let Some(&[rx, ry, x_rot, large_arc, sweep, x, y]) = params.next() {
                    let large_arc = large_arc == 1.0;
                    let sweep = sweep == 1.0;
                    builder.elliptical_arc_by(
                        f64::from(rx),
                        f64::from(ry),
                        f64::from(x_rot),
                        large_arc,
                        sweep,
                        SvgCoord {
                            x: f64::from(x),
                            y: f64::from(y),
                        },
                    )?;
                }
            },

            Close => {
                unreachable!("Expected an early continue.");
            },
        }
    }

    let runs = builder.into();
    validate_svg_runs(&runs)?;
    Ok(runs)
}

/// Parse the SVG `<polyline/>` and `<polygon/>` [`points`][points] attribute.
///
/// The result is a plain finite coordinate run which is immediately promoted to
/// `hypercurve::Contour2` or `CurveString2` by the caller. SVG points are
/// serialization input, not an internal topology container. The syntax follows
/// the W3C SVG 1.1 points grammar, and the finite-boundary/native-kernel split
/// follows Yap, "Towards Exact Geometric Computation," *Computational Geometry*
/// 7(1-2), 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>).
///
/// [points]: https://www.w3.org/TR/SVG11/shapes.html#PointsBNF
fn svg_points_to_coords(points: &str) -> Result<Vec<SvgCoord>, IoError> {
    use nom::IResult;
    use nom::Parser;
    use nom::branch::alt;
    use nom::character::complete::{char, multispace0, multispace1};
    use nom::combinator::opt;
    use nom::multi::separated_list1;
    use nom::number::complete::float;
    use nom::sequence::{delimited, pair, separated_pair, tuple};

    fn comma_wsp(i: &str) -> IResult<&str, ()> {
        let (i, _) = alt((
            tuple((multispace1, opt(char(',')), multispace0)).map(|_| ()),
            pair(char(','), multispace0).map(|_| ()),
        ))(i)?;
        Ok((i, ()))
    }

    fn point(i: &str) -> IResult<&str, SvgCoord> {
        let (i, (x, y)) = separated_pair(float, comma_wsp, float)(i)?;
        Ok((
            i,
            SvgCoord {
                x: f64::from(x),
                y: f64::from(y),
            },
        ))
    }

    fn all_points(i: &str) -> IResult<&str, Vec<SvgCoord>> {
        delimited(multispace0, separated_list1(comma_wsp, point), multispace0)(i)
    }

    match all_points(points) {
        Ok(("", points)) => {
            for (index, point) in points.iter().enumerate() {
                finite_svg_coord(point.x, point.y, &format!("points[{index}]"))?;
            }
            Ok(points)
        },
        Ok(_) => Err(IoError::MalformedInput(format!(
            "Could not parse the list of points: {points}"
        ))),
        Err(err) => Err(IoError::MalformedInput(format!(
            "Could not parse the list of points ({err}): {points}"
        ))),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hr(value: f64) -> Real {
        Real::try_from(value).expect("finite SVG test scalar")
    }

    #[test]
    fn basic_svg_io() {
        let svg_in = r#"
<svg viewBox="0 0 100 100" xmlns="http://www.w3.org/2000/svg">
<g>
<path d="M0,0,100,0,100,100 z" fill="black" fill-rule="evenodd" stroke="none"/>
</g>
</svg>
        "#;

        let sketch = Profile::<()>::from_svg(svg_in, ()).unwrap();
        let svg_out = sketch.to_svg();
        let reparsed = Profile::<()>::from_svg(&svg_out, ()).unwrap();

        assert_eq!(
            sketch.region_profiles().len(),
            reparsed.region_profiles().len()
        );
        assert_eq!(sketch.triangulate().len(), reparsed.triangulate().len());
    }

    #[test]
    fn svg_points_parsing() {
        let expected = vec![
            SvgCoord { x: 350.0, y: 75.0 },
            SvgCoord { x: 379.0, y: 161.0 },
            SvgCoord { x: 469.0, y: 161.0 },
            SvgCoord { x: 397.0, y: 215.0 },
            SvgCoord { x: 423.0, y: 301.0 },
            SvgCoord { x: 350.0, y: 250.0 },
            SvgCoord { x: 277.0, y: 301.0 },
            SvgCoord { x: 303.0, y: 215.0 },
            SvgCoord { x: 231.0, y: 161.0 },
            SvgCoord { x: 321.0, y: 161.0 },
        ];

        let points = "
            350,75  379,161 469,161 397,215
            423,301 350,250 277,301 303,215
            231,161 321,161
        ";
        let points = svg_points_to_coords(points).unwrap();
        assert_eq!(points, expected);

        let points = "
            350 75  379 161 469 161 397 215
            423 301 350 250 277 301 303 215
            231 161 321 161
        ";
        let points = svg_points_to_coords(points).unwrap();
        assert_eq!(points, expected);

        let points = "
            350,75,379,161,469,161,397,215,
            423,301,350,250,277,301,303,215,
            231,161,321,161
        ";
        let points = svg_points_to_coords(points).unwrap();
        assert_eq!(points, expected);

        let points = "
            350 , 75 , 379 , 161 , 469 , 161 , 397 , 215 ,
            423 ,301, 350 ,250, 277 ,301, 303 ,215,
            231    161    321    161
        ";
        let points = svg_points_to_coords(points).unwrap();
        assert_eq!(points, expected);
    }

    #[test]
    fn svg_path_parsing_keeps_open_runs_as_native_wires() {
        let runs =
            svg_path_to_runs(path::Data::parse("M 1 2 l 3 4 H 10 v 6").unwrap()).unwrap();
        assert_eq!(
            runs.runs,
            vec![vec![
                SvgCoord { x: 1.0, y: 2.0 },
                SvgCoord { x: 4.0, y: 6.0 },
                SvgCoord { x: 10.0, y: 6.0 },
                SvgCoord { x: 10.0, y: 12.0 },
            ]]
        );

        let svg_in = r#"
<svg viewBox="0 0 20 20" xmlns="http://www.w3.org/2000/svg">
<path d="M 1 2 l 3 4 H 10 v 6"/>
</svg>
        "#;
        let sketch = Profile::<()>::from_svg(svg_in, ()).unwrap();
        assert_eq!(sketch.region_profiles().len(), 0);
        assert_eq!(sketch.wires().len(), 1);
    }

    #[test]
    fn svg_path_nested_closed_runs_delegate_roles_to_hypercurve_region() {
        let svg_in = r#"
<svg viewBox="0 0 20 20" xmlns="http://www.w3.org/2000/svg">
<path d="
    M 0 0 L 10 0 L 10 10 L 0 10 Z
    M 2 2 L 8 2 L 8 8 L 2 8 Z
    M 4 4 L 6 4 L 6 6 L 4 6 Z
" fill="black" fill-rule="evenodd" stroke="none"/>
</svg>
        "#;

        let sketch = Profile::<()>::from_svg(svg_in, ()).unwrap();

        assert_eq!(sketch.material_contour_count(), 2);
        assert_eq!(sketch.hole_contour_count(), 1);
        assert!(sketch.wires().is_empty());
        assert!(sketch.contains_xy(hr(1.0), hr(1.0)).unwrap());
        assert!(!sketch.contains_xy(hr(3.0), hr(3.0)).unwrap());
        assert!(sketch.contains_xy(hr(5.0), hr(5.0)).unwrap());
    }

    #[test]
    fn svg_path_mixed_closed_and_open_runs_stays_native_hyper_geometry() {
        let svg_in = r#"
<svg viewBox="0 0 20 20" xmlns="http://www.w3.org/2000/svg">
<path d="
    M 0 0 L 4 0 L 4 4 L 0 4 Z
    M 10 1 L 12 3 L 14 1
" fill="black" stroke="black"/>
</svg>
        "#;

        let sketch = Profile::<()>::from_svg(svg_in, ()).unwrap();

        assert_eq!(sketch.material_contour_count(), 1);
        assert_eq!(sketch.hole_contour_count(), 0);
        assert_eq!(sketch.wires().len(), 1);
        assert!(sketch.contains_xy(hr(1.0), hr(1.0)).unwrap());
        assert!(
            sketch
                .wire_polylines()
                .iter()
                .flatten()
                .any(|point| point[0] >= 14.0),
            "open SVG subpath should remain a native CurveString2 wire"
        );
    }
}
