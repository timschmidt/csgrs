//! SVG input and output.

use geo::{BoundingRect, Coord, CoordNum, CoordsIter, LineString, MapCoords, MultiLineString, Polygon};
use svg::node::element::path;

use crate::csg::CSG;

use super::IoError;


/// A helper struct to build [`geo::MultiLineString`] from SVG Path commands.
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
/// **At the moment, curves are not supported.**
/// When support for curves is implemented, the underlying data structure may change to accommodate that.
///
/// [svg-paths]: https://www.w3.org/TR/SVG11/paths.html
struct PathBuilder<F: CoordNum> {
    inner: MultiLineString<F>,
}

impl<F: CoordNum> Into<MultiLineString<F>> for PathBuilder<F> {
    fn into(self) -> MultiLineString<F> {
        self.inner
    }
}

impl<F: CoordNum> PathBuilder<F> {
    pub fn new() -> Self {
        Self {
            inner: MultiLineString::new(vec![]),
        }
    }

    /// Get the current position to be used for relative moves.
    fn get_pos(&self) -> Coord<F> {
        self.inner.0.last()
            .and_then(|ls| ls.0.last())
            .copied()
            .unwrap_or(Coord::zero())
    }

    /// Get a mutable reference to the current path, or an error if no path has been started.
    ///
    /// To accomodate for the semantics of [`close`], this function will automatically start a new path
    /// if the last path has 2 or more points and is closed.
    /// For this reason, using this proxy is recommended for implementing any drawing command.
    fn get_path_mut_or_fail(&mut self) -> Result<&mut LineString<F>, IoError> {
        let start_new_path = self.inner.0.last()
            .map(|p| p.coords_count() >= 2 && p.is_closed())
            .unwrap_or(false);

        if start_new_path {
            self.inner.0.push(LineString::new(vec![self.get_pos()]));
        }

        self.inner.0.last_mut()
            .ok_or_else(|| IoError::MalformedPath(format!("Attempted to extend the current path, but no path was started.")))
    }

    /// Start a new path at `point`.
    pub fn move_to(&mut self, point: Coord<F>) {
        self.inner.0.push(LineString::new(vec![point]));
    }

    /// Start a new path at `delta` relative to the last point.
    /// If and only if this is the first command, the point is treated as absolute coordinates.
    pub fn move_by(&mut self, delta: Coord<F>) {
        let pos = self.get_pos();
        self.inner.0.push(LineString::new(vec![pos + delta]));
    }

    /// Extend the current path to the `point`.
    /// Can not be the first command.
    pub fn line_to(&mut self, point: Coord<F>) -> Result<(), IoError> {
        let line = self.get_path_mut_or_fail()?;
        line.0.push(point);
        Ok(())
    }

    /// Extend the current path by `delta` relative to the current point.
    /// Can not be the first command.
    pub fn line_by(&mut self, delta: Coord<F>) -> Result<(), IoError> {
        let pos = self.get_pos();
        let line = self.get_path_mut_or_fail()?;
        line.0.push(pos + delta);
        Ok(())
    }

    /// Extend the current path with a horizontal move to `x`.
    /// Can not be the first command.
    pub fn hline_to(&mut self, x: F) -> Result<(), IoError> {
        let Coord { y, .. } = self.get_pos();
        let line = self.get_path_mut_or_fail()?;
        line.0.push(Coord { x, y });
        Ok(())
    }

    /// Extend the current path with a horizontal move by `dx` relative to the current point.
    /// Can not be the first command.
    pub fn hline_by(&mut self, dx: F) -> Result<(), IoError> {
        let Coord { x, y } = self.get_pos();
        let line = self.get_path_mut_or_fail()?;
        line.0.push(Coord { x: x + dx, y });
        Ok(())
    }

    /// Extend the current path with a vertical move to `y`.
    /// Can not be the first command.
    pub fn vline_to(&mut self, y: F) -> Result<(), IoError> {
        let Coord { x, .. } = self.get_pos();
        let line = self.get_path_mut_or_fail()?;
        line.0.push(Coord { x, y });
        Ok(())
    }

    /// Extend the current path with a vertical move by `dy` relative to the current point.
    /// Can not be the first command.
    pub fn vline_by(&mut self, dy: F) -> Result<(), IoError> {
        let Coord { x, y } = self.get_pos();
        let line = self.get_path_mut_or_fail()?;
        line.0.push(Coord { x, y: y + dy });
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
        line.close();
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
    pub fn quadratic_curve_to(&mut self, _control: Coord<F>, _end: Coord<F>) -> Result<(), IoError> {
        Err(IoError::Unimplemented(format!("quadratic curveto (absolute quadratic Bézier curve)")))
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
    pub fn quadratic_curve_by(&mut self, _control: Coord<F>, _end: Coord<F>) -> Result<(), IoError> {
        Err(IoError::Unimplemented(format!("quadratic curveby (relative quadratic Bézier curve)")))
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
    pub fn quadratic_smooth_curve_to(&mut self, _end: Coord<F>) -> Result<(), IoError> {
        Err(IoError::Unimplemented(format!("quadratic smooth curveto (absolute quadratic Bézier curve with a reflected control point)")))
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
    pub fn quadratic_smooth_curve_by(&mut self, _end: Coord<F>) -> Result<(), IoError> {
        Err(IoError::Unimplemented(format!("quadratic smooth curveby (relative quadratic Bézier curve with a reflected control point)")))
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
    pub fn curve_to(&mut self, _control_start: Coord<F>, _control_end: Coord<F>, _end: Coord<F>) -> Result<(), IoError> {
        Err(IoError::Unimplemented(format!("curveto (absolute cubic Bézier curve)")))
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
    pub fn curve_by(&mut self, _control_start: Coord<F>, _control_end: Coord<F>, _end: Coord<F>) -> Result<(), IoError> {
        Err(IoError::Unimplemented(format!("curveby (relative cubic Bézier curve)")))
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
    pub fn smooth_curve_to(&mut self, _control_end: Coord<F>, _end: Coord<F>) -> Result<(), IoError> {
        Err(IoError::Unimplemented(format!("smooth curveto (absolute cubic Bézier curve with a reflected start control point)")))
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
    pub fn smooth_curve_by(&mut self, _control_end: Coord<F>, _end: Coord<F>) -> Result<(), IoError> {
        Err(IoError::Unimplemented(format!("smooth curveby (relative cubic Bézier curve with a reflected start control point)")))
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
        _rx: F,
        _ry: F,
        _x_axis_rotation: F,
        _large_arc_flag: bool,
        _sweep_flag: bool,
        _end: Coord<F>,
    ) -> Result<(), IoError> {
        Err(IoError::Unimplemented(format!("elliptical arc to")))
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
        _rx: F,
        _ry: F,
        _x_axis_rotation: F,
        _large_arc_flag: bool,
        _sweep_flag: bool,
        _end: Coord<F>,
    ) -> Result<(), IoError> {
        Err(IoError::Unimplemented(format!("elliptical arc by")))
    }
}


pub trait FromSVG: Sized {
    fn from_svg(doc: &str) -> Result<Self, IoError>;
}

impl FromSVG for CSG<()> {
    fn from_svg(doc: &str) -> Result<Self, IoError> {
        use svg::parser::Event;
        use svg::node::element::tag::{self, Type::*};

        macro_rules! expect_attr {
            ($attrs:expr, $attr:literal) => {
                $attrs.get($attr)
                    .ok_or_else(|| IoError::MalformedInput(format!("Missing attribute {}", $attr)))
            };
        }

        macro_rules! option_attr {
            ($attrs:expr, $attr:literal) => {
                $attrs.get($attr)
            };
        }

        let mut csg_union = Self::new();

        for event in svg::read(doc)? {
            match event {
                | Event::Instruction(..)
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
                    let mls = svg_path_to_multi_line_string(data)?;

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
                    // - every unclosed subpath is a line (as if with no fill)
                    //
                    // The lines are then (for now) discarded as expanding lines requires knowing current stroke-width.

                    for ls in mls.0.into_iter() {
                        if ls.is_closed() {
                            let polygon = Polygon::new(ls, vec![]);
                            let csg = Self::from_geo(polygon.into(), None);
                            csg_union = csg_union.union(&csg);
                        }
                    }
                },

                Event::Tag(tag::Circle, Empty, attrs) => {
                    let cx = expect_attr!(attrs, "cx")?.parse()?;
                    let cy = expect_attr!(attrs, "cy")?.parse()?;
                    let r: f64 = expect_attr!(attrs, "r")?.parse()?;

                    // TODO: add a way for the user to configure this?
                    let segments = (r.ceil() as usize).max(6);

                    let csg = Self::circle(r, segments, None)
                        .translate(cx, cy, 0.0);
                    csg_union = csg_union.union(&csg);
                },

                Event::Tag(tag::Rectangle, Empty, attrs) => {
                    let x: f64 = expect_attr!(attrs, "x")?.parse()?;
                    let y: f64 = expect_attr!(attrs, "y")?.parse()?;
                    let w: f64 = expect_attr!(attrs, "width")?.parse()?;
                    let h: f64 = expect_attr!(attrs, "height")?.parse()?;
                    let rx: f64 = option_attr!(attrs, "rx").map_or(Ok(0.0), |a| a.parse())?;
                    let ry: f64 = option_attr!(attrs, "ry").map_or(Ok(0.0), |a| a.parse())?;

                    // TODO: support rx != ry
                    let r = (rx + ry) / 2.0;

                    // TODO: add a way for the user to configure this?
                    let segments = (r.ceil() as usize).max(6);

                    let csg = Self::rounded_rectangle(w, h, r, segments, None)
                            .translate(x, y, 0.0);
                    csg_union = csg_union.union(&csg);
                },

                Event::Tag(tag::Ellipse, Empty, attrs) => {
                    let cx = expect_attr!(attrs, "cx")?.parse()?;
                    let cy = expect_attr!(attrs, "cy")?.parse()?;
                    let rx: f64 = expect_attr!(attrs, "rx")?.parse()?;
                    let ry: f64 = expect_attr!(attrs, "ry")?.parse()?;

                    // TODO: add a way for the user to configure this?
                    let segments = (rx.max(ry).ceil() as usize).max(6);

                    let csg = Self::ellipse(rx * 2.0, ry * 2.0, segments, None)
                        .translate(cx, cy, 0.0);
                    csg_union = csg_union.union(&csg);
                },

                Event::Tag(tag::Line, Empty, attrs) => {
                    let _x1 = expect_attr!(attrs, "x1")?;
                    let _y1 = expect_attr!(attrs, "y1")?;
                    let _x2 = expect_attr!(attrs, "x2")?;
                    let _y2 = expect_attr!(attrs, "y2")?;

                    // TODO: This needs knowing current stroke-width
                },

                Event::Tag(tag::Polygon, Empty, attrs) => {
                    let points = expect_attr!(attrs, "points")?;
                    let polygon = Polygon::new(
                        svg_points_to_line_string(points)?,
                        vec![],
                    );
                    let csg = Self::from_geo(polygon.into(), None);
                    csg_union = csg_union.union(&csg);
                },

                Event::Tag(tag::Polyline, Empty, attrs) => {
                    let points = expect_attr!(attrs, "points")?;
                    let _ls = svg_points_to_line_string::<f64>(points)?;

                    // TODO: This needs knowing current stroke-width
                },

                tag => {
                    // TODO: Non-empty tags should also be supported

                    unimplemented!("Parsing tag {tag:?}");
                }
            }
        }

        Ok(csg_union)
    }
}


pub trait ToSVG {
    fn to_svg(&self) -> String;
}

impl<S: Clone> ToSVG for CSG<S> {
    fn to_svg(&self) -> String {
        use svg::node::element;
        use geo::Geometry::*;

        let mut g = element::Group::new();

        let make_line_string = |line_string: &geo::LineString| {
            let mut data = path::Data::new();
            let mut points = line_string.coords();

            if let Some(start) = points.next() {
                data = data.move_to(start.x_y());
            }
            for point in points {
                data = data.line_to(point.x_y());
            }

            element::Path::new()
                .set("fill", "none")
                .set("stroke", "black")
                .set("stroke-width", 1)
                .set("vector-effect", "non-scaling-stroke")
                .set("d", data)
        };

        let make_polygon = |polygon: &geo::Polygon| {
            let mut data = path::Data::new();

            let exterior = polygon.exterior();
            data = data.move_to(
                // Skip the last point because it is equal to the first one
                exterior.0[..(exterior.0.len() - 1)]
                    .into_iter()
                    .flat_map(|c| [c.x as f32, c.y as f32])
                    .collect::<Vec<f32>>()
            );

            data = data.close();

            for interior in polygon.interiors() {
                data = data.move_to(
                    // Skip the last point because it is equal to the first one
                    interior.0[..(interior.0.len() - 1)]
                        .into_iter()
                        .flat_map(|c| [c.x as f32, c.y as f32])
                        .collect::<Vec<f32>>()
                );

                data = data.close();
            }

            element::Path::new()
                .set("fill", "black")
                .set("fill-rule", "evenodd")
                .set("stroke", "none")
                .set("d", data)
        };

        let bounds = self.geometry.bounding_rect()
            .unwrap_or(geo::Rect::new(Coord { x: 0.0, y: 0.0 }, Coord { x: 1.0, y: 1.0 }));

        for geometry in self.geometry.iter() {
            match geometry {
                Line(line) => {
                    g = g.add(
                        element::Line::new()
                            .set("stroke", "black")
                            .set("stroke-width", 1)
                            .set("vector-effect", "non-scaling-stroke")
                            .set("x1", line.start.x)
                            .set("y1", line.start.y)
                            .set("x2", line.end.x)
                            .set("y2", line.end.y)
                    );
                },

                LineString(line_string) => {
                    g = g.add(make_line_string(line_string));
                },
                Polygon(polygon) => {
                    g = g.add(make_polygon(polygon));
                },
                MultiLineString(multi_line_string) => {
                    for line_string in multi_line_string {
                        g = g.add(make_line_string(line_string));
                    }
                },
                MultiPolygon(multi_polygon) => {
                    for polygon in multi_polygon {
                        g = g.add(make_polygon(polygon));
                    }
                },

                Rect(rect) => {
                    g = g.add(make_polygon(&rect.to_polygon()));
                },

                Triangle(triangle) => {
                    g = g.add(make_polygon(&triangle.to_polygon()));
                },

                GeometryCollection(_) => unimplemented!("Exporting nested geometry collections to SVG"),

                // Can't really export points to SVG
                Point(_) => {},
                MultiPoint(_) => {},
            }
        }

        let doc = svg::Document::new()
            .set("viewBox", (bounds.min().x, bounds.min().y, bounds.width(), bounds.height()))
            .add(g);

        doc.to_string()
    }
}


fn svg_path_to_multi_line_string<F: CoordNum>(path_data: path::Data) -> Result<MultiLineString<F>, IoError> {
    // `svg` crate returns `f32`, so that's what is used here.
    let mut builder = PathBuilder::<f32>::new();

    for cmd in path_data.into_iter() {
        use svg::node::element::path::{Command::*, Position::*};

        macro_rules! ensure_param_count {
            ($count:expr, $div_by:expr) => {
                if $count % $div_by != 0 {
                    return Err(IoError::MalformedPath(format!("Expected the number of parameters {} to be divisible by {} in command {cmd:?}", $count, $div_by)));
                }
            };
        }

        let param_count = match cmd {
            | Move(..)
            | Line(..) => 2,

            | HorizontalLine(..)
            | VerticalLine(..) => 1,

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
                    builder.move_to(Coord { x, y });
                }

                // Follow-up coordinates for MoveTo are implicit LineTo
                while let Some(&[x, y]) = coords.next() {
                    builder.line_to(Coord { x, y })?;
                }
            },
            Move(Relative, params) => {
                ensure_param_count!(params.len(), param_count);
                let mut coords = params.chunks(param_count);

                if let Some(&[dx, dy]) = coords.next() {
                    builder.move_by(Coord { x: dx, y: dy });
                }

                // Follow-up coordinates for MoveTo are implicit LineTo
                while let Some(&[dx, dy]) = coords.next() {
                    builder.line_by(Coord { x: dx, y: dy })?;
                }
            },
            Line(Absolute, params) => {
                ensure_param_count!(params.len(), param_count);
                let mut coords = params.chunks(param_count);
                while let Some(&[x, y]) = coords.next() {
                    builder.line_to(Coord { x, y })?;
                }
            },
            Line(Relative, params) => {
                ensure_param_count!(params.len(), param_count);
                let mut coords = params.chunks(param_count);
                while let Some(&[dx, dy]) = coords.next() {
                    builder.line_by(Coord { x: dx, y: dy })?;
                }
            },
            HorizontalLine(Absolute, params) => {
                for &x in params.into_iter() {
                    builder.hline_to(x)?;
                }
            },
            HorizontalLine(Relative, params) => {
                for &dx in params.into_iter() {
                    builder.hline_by(dx)?;
                }
            },
            VerticalLine(Absolute, params) => {
                for &y in params.into_iter() {
                    builder.vline_to(y)?;
                }
            },
            VerticalLine(Relative, params) => {
                for &dy in params.into_iter() {
                    builder.vline_by(dy)?;
                }
            },

            QuadraticCurve(Absolute, params) => {
                ensure_param_count!(params.len(), param_count);
                let mut params = params.chunks(param_count);
                while let Some(&[cx, cy, x, y]) = params.next() {
                    builder.quadratic_curve_to(Coord { x: cx, y: cy }, Coord { x, y })?;
                }
            },
            QuadraticCurve(Relative, params) => {
                ensure_param_count!(params.len(), param_count);
                let mut params = params.chunks(param_count);
                while let Some(&[cx, cy, x, y]) = params.next() {
                    builder.quadratic_curve_by(Coord { x: cx, y: cy }, Coord { x, y })?;
                }
            },
            SmoothQuadraticCurve(Absolute, params) => {
                ensure_param_count!(params.len(), param_count);
                let mut params = params.chunks(param_count);
                while let Some(&[x, y]) = params.next() {
                    builder.quadratic_smooth_curve_to(Coord { x, y })?;
                }
            },
            SmoothQuadraticCurve(Relative, params) => {
                ensure_param_count!(params.len(), param_count);
                let mut params = params.chunks(param_count);
                while let Some(&[x, y]) = params.next() {
                    builder.quadratic_smooth_curve_by(Coord { x, y })?;
                }
            },

            CubicCurve(Absolute, params) => {
                ensure_param_count!(params.len(), param_count);
                let mut params = params.chunks(param_count);
                while let Some(&[c1x, c1y, c2x, c2y, x, y]) = params.next() {
                    builder.curve_to(Coord { x: c1x, y: c1y }, Coord { x: c2x, y: c2y }, Coord { x, y })?;
                }
            },
            CubicCurve(Relative, params) => {
                ensure_param_count!(params.len(), param_count);
                let mut params = params.chunks(param_count);
                while let Some(&[c1x, c1y, c2x, c2y, x, y]) = params.next() {
                    builder.curve_by(Coord { x: c1x, y: c1y }, Coord { x: c2x, y: c2y }, Coord { x, y })?;
                }
            },
            SmoothCubicCurve(Absolute, params) => {
                ensure_param_count!(params.len(), param_count);
                let mut params = params.chunks(param_count);
                while let Some(&[c2x, c2y, x, y]) = params.next() {
                    builder.smooth_curve_to(Coord { x: c2x, y: c2y }, Coord { x, y })?;
                }
            },
            SmoothCubicCurve(Relative, params) => {
                ensure_param_count!(params.len(), param_count);
                let mut params = params.chunks(param_count);
                while let Some(&[c2x, c2y, x, y]) = params.next() {
                    builder.smooth_curve_by(Coord { x: c2x, y: c2y }, Coord { x, y })?;
                }
            },

            EllipticalArc(Absolute, params) => {
                ensure_param_count!(params.len(), param_count);
                let mut params = params.chunks(param_count);
                while let Some(&[rx, ry, x_rot, large_arc, sweep, x, y]) = params.next() {
                    let large_arc = large_arc == 1.0;
                    let sweep = sweep == 1.0;
                    builder.elliptical_arc_to(rx, ry, x_rot, large_arc, sweep, Coord { x, y })?;
                }
            },
            EllipticalArc(Relative, params) => {
                ensure_param_count!(params.len(), param_count);
                let mut params = params.chunks(param_count);
                while let Some(&[rx, ry, x_rot, large_arc, sweep, x, y]) = params.next() {
                    let large_arc = large_arc == 1.0;
                    let sweep = sweep == 1.0;
                    builder.elliptical_arc_by(rx, ry, x_rot, large_arc, sweep, Coord { x, y })?;
                }
            },

            Close => {
                unreachable!("Expected an early continue.");
            },
        }
    }

    let mls: MultiLineString<f32> = builder.into();
    let mls = mls.map_coords(|c| Coord {
        x: F::from(c.x).unwrap(),
        y: F::from(c.y).unwrap(),
    });

    Ok(mls)
}


/// Parse contents of the SVG <polyline/> and <polygon/> attribute [`points`][points] into a `LineString`.
///
/// [points]: https://www.w3.org/TR/SVG11/shapes.html#PointsBNF
fn svg_points_to_line_string<F: CoordNum>(points: &str) -> Result<LineString<F>, IoError> {
    use nom::IResult;
    use nom::Parser;
    use nom::branch::alt;
    use nom::character::complete::{char, multispace0, multispace1};
    use nom::combinator::opt;
    use nom::number::complete::float;
    use nom::sequence::{pair, tuple, delimited, preceded, separated_pair, terminated};
    use nom::multi::separated_list1;

    fn comma_wsp(i: &str) -> IResult<&str, ()> {
        let (i, _) = alt((
            tuple((
                multispace1,
                opt(char(',')),
                multispace0,
            )).map(|_| ()),
            pair(
                char(','),
                multispace0,
            ).map(|_| ()),
        ))(i)?;
        Ok((i, ()))
    }

    fn point<F: CoordNum>(i: &str) -> IResult<&str, Coord<F>> {
        let (i, (x, y)) = separated_pair(
            float,
            comma_wsp,
            float,
        )(i)?;
        Ok((i, Coord {
            x: F::from(x).unwrap(),
            y: F::from(y).unwrap(),
        }))
    }

    fn all_points<F: CoordNum>(i: &str) -> IResult<&str, Vec<Coord<F>>> {
        delimited(
            multispace0,
            separated_list1(
                comma_wsp,
                point,
            ),
            multispace0,
        )(i)
    }

    match all_points(points) {
        Ok(("", points)) => Ok(LineString::new(points)),
        Ok(_) => Err(IoError::MalformedInput(format!("Could not parse the list of points: {points}"))),
        Err(err) => Err(IoError::MalformedInput(format!("Could not parse the list of points ({err}): {points}"))),
    }
}


#[cfg(test)]
mod tests {
    use geo::line_string;

    use super::*;

    #[test]
    fn basic_svg_io() {
        let svg_in = r#"
<svg viewBox="0 0 100 100" xmlns="http://www.w3.org/2000/svg">
<g>
<path d="M0,0,100,0,100,100 z" fill="black" fill-rule="evenodd" stroke="none"/>
</g>
</svg>
        "#;

        let csg = CSG::from_svg(svg_in).unwrap();
        let svg_out = csg.to_svg();

        assert_eq!(svg_in.trim(), svg_out.trim());
    }

    #[test]
    fn svg_points_parsing() {
        let expected = line_string![
            (x: 350.0, y:  75.0),
            (x: 379.0, y: 161.0),
            (x: 469.0, y: 161.0),
            (x: 397.0, y: 215.0),
            (x: 423.0, y: 301.0),
            (x: 350.0, y: 250.0),
            (x: 277.0, y: 301.0),
            (x: 303.0, y: 215.0),
            (x: 231.0, y: 161.0),
            (x: 321.0, y: 161.0),
        ];

        let points = "
            350,75  379,161 469,161 397,215
            423,301 350,250 277,301 303,215
            231,161 321,161
        ";
        let points = svg_points_to_line_string(points).unwrap();
        assert_eq!(points, expected);

        let points = "
            350 75  379 161 469 161 397 215
            423 301 350 250 277 301 303 215
            231 161 321 161
        ";
        let points = svg_points_to_line_string(points).unwrap();
        assert_eq!(points, expected);

        let points = "
            350,75,379,161,469,161,397,215,
            423,301,350,250,277,301,303,215,
            231,161,321,161
        ";
        let points = svg_points_to_line_string(points).unwrap();
        assert_eq!(points, expected);

        let points = "
            350 , 75 , 379 , 161 , 469 , 161 , 397 , 215 ,
            423 ,301, 350 ,250, 277 ,301, 303 ,215,
            231    161    321    161
        ";
        let points = svg_points_to_line_string(points).unwrap();
        assert_eq!(points, expected);
    }
}
