//! Strict SVG document import and fallible finite SVG export.

use crate::csg::CSG;
use crate::io::{IoError, finite_f64};
use crate::sketch::Profile;
use hypercurve::{
    Classification, CurvePolicy, CurveString2, FillRule, FiniteProjectionOptions, Region2,
    import_svg_path_data_with_report, import_svg_region_path_data_with_report,
};
use hyperlattice::{Matrix4, Real};
use std::fmt::Debug;

/// Controls finite approximation at the SVG boundary.
#[derive(Clone, Copy, Debug)]
pub struct SvgOptions {
    pub curve_tolerance: f64,
    pub max_curve_segments: usize,
}

impl Default for SvgOptions {
    fn default() -> Self {
        Self {
            curve_tolerance: 0.1,
            max_curve_segments: 4096,
        }
    }
}

impl SvgOptions {
    fn validate(self) -> Result<Self, IoError> {
        if !self.curve_tolerance.is_finite()
            || self.curve_tolerance <= 0.0
            || self.max_curve_segments < 8
        {
            return Err(IoError::MalformedInput(
                "SVG projection options require a positive finite tolerance and at least 8 segments"
                    .into(),
            ));
        }
        Ok(self)
    }

    fn segments_for_radius(self, radius: f64) -> Result<usize, IoError> {
        if !radius.is_finite() || radius <= 0.0 {
            return Err(IoError::MalformedInput(
                "SVG radius must be finite and positive".into(),
            ));
        }
        let requested = (std::f64::consts::TAU * radius / self.curve_tolerance)
            .ceil()
            .max(16.0) as usize;
        if requested > self.max_curve_segments {
            return Err(IoError::SizeOverflow {
                format: "SVG",
                limit: "configured curve-segment count",
            });
        }
        Ok(requested)
    }
}

#[derive(Clone, Copy, Debug)]
struct Affine2([f64; 6]);

impl Affine2 {
    const IDENTITY: Self = Self([1.0, 0.0, 0.0, 1.0, 0.0, 0.0]);

    fn then(self, rhs: Self) -> Self {
        let [a, b, c, d, e, f] = self.0;
        let [g, h, i, j, k, l] = rhs.0;
        Self([
            a * g + c * h,
            b * g + d * h,
            a * i + c * j,
            b * i + d * j,
            a * k + c * l + e,
            b * k + d * l + f,
        ])
    }

    fn matrix(self) -> Result<Matrix4, IoError> {
        let values = self
            .0
            .map(|value| {
                Real::try_from(value).map_err(|error| {
                    IoError::MalformedInput(format!("SVG transform is not finite: {error}"))
                })
            })
            .into_iter()
            .collect::<Result<Vec<_>, _>>()?;
        Ok(Matrix4::from_row_major([
            values[0].clone(),
            values[2].clone(),
            Real::zero(),
            values[4].clone(),
            values[1].clone(),
            values[3].clone(),
            Real::zero(),
            values[5].clone(),
            Real::zero(),
            Real::zero(),
            Real::one(),
            Real::zero(),
            Real::zero(),
            Real::zero(),
            Real::zero(),
            Real::one(),
        ]))
    }
}

#[derive(Clone, Copy, Debug)]
struct StyleContext {
    transform: Affine2,
    fill: bool,
    stroke: bool,
    displayed: bool,
    visibility: bool,
    opacity: f64,
    fill_opacity: f64,
    stroke_opacity: f64,
    stroke_width: f64,
    fill_rule: FillRule,
}

impl Default for StyleContext {
    fn default() -> Self {
        Self {
            transform: Affine2::IDENTITY,
            fill: true,
            stroke: false,
            displayed: true,
            visibility: true,
            opacity: 1.0,
            fill_opacity: 1.0,
            stroke_opacity: 1.0,
            stroke_width: 1.0,
            fill_rule: FillRule::NonZero,
        }
    }
}

impl StyleContext {
    fn visible(self) -> bool {
        self.displayed && self.visibility && self.opacity > 0.0
    }

    fn fills(self) -> bool {
        self.fill && self.fill_opacity > 0.0
    }

    fn strokes(self) -> bool {
        self.stroke && self.stroke_opacity > 0.0 && self.stroke_width > 0.0
    }
}

fn parse_numbers(value: &str) -> Result<Vec<f64>, IoError> {
    value
        .replace(',', " ")
        .split_whitespace()
        .map(|part| {
            part.parse::<f64>().map_err(|error| {
                IoError::MalformedInput(format!("invalid SVG number: {error}"))
            })
        })
        .collect()
}

fn parse_transform(value: &str) -> Result<Affine2, IoError> {
    let mut remaining = value.trim();
    let mut transform = Affine2::IDENTITY;
    while !remaining.is_empty() {
        let open = remaining.find('(').ok_or_else(|| {
            IoError::MalformedInput(format!("invalid SVG transform {value:?}"))
        })?;
        let close = remaining[open + 1..]
            .find(')')
            .map(|index| index + open + 1)
            .ok_or_else(|| {
                IoError::MalformedInput(format!("invalid SVG transform {value:?}"))
            })?;
        let name = remaining[..open].trim();
        let numbers = parse_numbers(&remaining[open + 1..close])?;
        let next = match (name, numbers.as_slice()) {
            ("matrix", &[a, b, c, d, e, f]) => Affine2([a, b, c, d, e, f]),
            ("translate", &[x]) => Affine2([1.0, 0.0, 0.0, 1.0, x, 0.0]),
            ("translate", &[x, y]) => Affine2([1.0, 0.0, 0.0, 1.0, x, y]),
            ("scale", &[scale]) => Affine2([scale, 0.0, 0.0, scale, 0.0, 0.0]),
            ("scale", &[x, y]) => Affine2([x, 0.0, 0.0, y, 0.0, 0.0]),
            ("rotate", &[degrees]) => rotation(degrees),
            ("rotate", &[degrees, x, y]) => Affine2([1.0, 0.0, 0.0, 1.0, x, y])
                .then(rotation(degrees))
                .then(Affine2([1.0, 0.0, 0.0, 1.0, -x, -y])),
            ("skewX", &[degrees]) => {
                Affine2([1.0, 0.0, degrees.to_radians().tan(), 1.0, 0.0, 0.0])
            },
            ("skewY", &[degrees]) => {
                Affine2([1.0, degrees.to_radians().tan(), 0.0, 1.0, 0.0, 0.0])
            },
            _ => {
                return Err(IoError::Unsupported {
                    format: "SVG",
                    detail: format!("transform {name} with {} arguments", numbers.len()),
                });
            },
        };
        transform = transform.then(next);
        remaining = remaining[close + 1..].trim_start_matches(|character: char| {
            character.is_whitespace() || character == ','
        });
    }
    Ok(transform)
}

fn rotation(degrees: f64) -> Affine2 {
    let (sin, cos) = degrees.to_radians().sin_cos();
    Affine2([cos, sin, -sin, cos, 0.0, 0.0])
}

fn apply_style(
    mut context: StyleContext,
    attrs: &svg::node::Attributes,
) -> Result<StyleContext, IoError> {
    let mut properties = Vec::<(&str, &str)>::new();
    for name in [
        "fill",
        "stroke",
        "fill-rule",
        "display",
        "visibility",
        "opacity",
        "fill-opacity",
        "stroke-opacity",
        "stroke-width",
    ] {
        if let Some(value) = attrs.get(name) {
            properties.push((name, value));
        }
    }
    if let Some(style) = attrs.get("style") {
        for declaration in style.split(';').filter(|part| !part.trim().is_empty()) {
            let (name, value) = declaration.split_once(':').ok_or_else(|| {
                IoError::MalformedInput(format!(
                    "invalid SVG style declaration {declaration:?}"
                ))
            })?;
            properties.push((name.trim(), value.trim()));
        }
    }
    let mut local_opacity = 1.0;
    let mut local_display = true;
    for (name, value) in properties {
        match name {
            "fill" => context.fill = value != "none",
            "stroke" => context.stroke = value != "none",
            "fill-rule" => {
                context.fill_rule = match value {
                    "nonzero" => FillRule::NonZero,
                    "evenodd" => FillRule::EvenOdd,
                    _ => {
                        return Err(IoError::MalformedInput(format!(
                            "invalid SVG fill-rule {value:?}"
                        )));
                    },
                }
            },
            "display" => local_display = value != "none",
            "visibility" => context.visibility = !matches!(value, "hidden" | "collapse"),
            "opacity" => local_opacity = unit_interval(value, "opacity")?,
            "fill-opacity" => context.fill_opacity = unit_interval(value, "fill-opacity")?,
            "stroke-opacity" => {
                context.stroke_opacity = unit_interval(value, "stroke-opacity")?
            },
            "stroke-width" => {
                context.stroke_width = value.parse::<f64>()?;
                if !context.stroke_width.is_finite() || context.stroke_width < 0.0 {
                    return Err(IoError::MalformedInput(
                        "SVG stroke-width must be finite and non-negative".into(),
                    ));
                }
            },
            _ => {},
        }
    }
    context.opacity *= local_opacity;
    context.displayed &= local_display;
    if let Some(value) = attrs.get("transform") {
        context.transform = context.transform.then(parse_transform(value)?);
    }
    Ok(context)
}

fn unit_interval(value: &str, name: &str) -> Result<f64, IoError> {
    let value = value.parse::<f64>()?;
    if !value.is_finite() || !(0.0..=1.0).contains(&value) {
        return Err(IoError::MalformedInput(format!(
            "SVG {name} must be between zero and one"
        )));
    }
    Ok(value)
}

fn number(
    attrs: &svg::node::Attributes,
    name: &str,
    default: Option<f64>,
) -> Result<f64, IoError> {
    let value = match attrs.get(name) {
        Some(value) => value.parse::<f64>()?,
        None => default
            .ok_or_else(|| IoError::MalformedInput(format!("missing SVG attribute {name}")))?,
    };
    if !value.is_finite() {
        return Err(IoError::MalformedInput(format!("SVG {name} must be finite")));
    }
    Ok(value)
}

fn real(value: f64, name: &'static str) -> Result<Real, IoError> {
    Real::try_from(value)
        .map_err(|error| IoError::MalformedInput(format!("SVG {name} is invalid: {error}")))
}

fn points(value: &str) -> Result<Vec<[Real; 2]>, IoError> {
    let numbers = parse_numbers(value)?;
    if numbers.len() < 4 || numbers.len() % 2 != 0 {
        return Err(IoError::MalformedInput(
            "SVG points require coordinate pairs".into(),
        ));
    }
    numbers
        .chunks_exact(2)
        .map(|pair| Ok([real(pair[0], "point x")?, real(pair[1], "point y")?]))
        .collect()
}

fn imported_path<M: Clone + Debug + Send + Sync>(
    data: &str,
    context: StyleContext,
    source_index: u64,
    metadata: M,
) -> Result<Profile<M>, IoError> {
    let mut region = Region2::empty();
    let mut wires = Vec::new();
    if context.fills() {
        let result = import_svg_region_path_data_with_report(
            data,
            context.fill_rule,
            source_index,
            1,
            None,
            &CurvePolicy::certified(),
        );
        let blocker = result.report().blocker();
        region = result.into_region().ok_or_else(|| IoError::Geometry {
            format: "SVG",
            detail: format!("path region was not certified: {blocker:?}"),
        })?;
    }
    if context.strokes() {
        let result = import_svg_path_data_with_report(data, source_index, 1, None);
        let blocker = result.report().blocker();
        wires.push(result.into_curve_string().ok_or_else(|| IoError::Geometry {
            format: "SVG",
            detail: format!("stroked path was not retained: {blocker:?}"),
        })?);
    }
    Ok(Profile::from_region_and_wires(region, wires, metadata))
}

fn styled_shape<M: Clone + Debug + Send + Sync>(
    shape: Profile<M>,
    context: StyleContext,
) -> Profile<M> {
    if context.fills() && context.strokes() {
        let wires = shape
            .as_region()
            .material_contours()
            .iter()
            .chain(shape.as_region().hole_contours())
            .map(|contour| contour.curve_string().clone())
            .collect();
        Profile::from_region_and_wires(
            shape.as_region().clone(),
            wires,
            shape.metadata().clone(),
        )
    } else if context.strokes() {
        let wires = shape
            .as_region()
            .material_contours()
            .iter()
            .chain(shape.as_region().hole_contours())
            .map(|contour| contour.curve_string().clone())
            .collect();
        Profile::from_wires(wires, shape.metadata().clone())
    } else if context.fills() {
        shape
    } else {
        Profile::empty(shape.metadata().clone())
    }
}

pub trait FromSVG<M>: Sized {
    fn from_svg(document: &str, metadata: M) -> Result<Self, IoError>;
    fn from_svg_with_options(
        document: &str,
        metadata: M,
        options: SvgOptions,
    ) -> Result<Self, IoError>;
}

impl<M: Clone + Debug + Send + Sync> FromSVG<M> for Profile<M> {
    fn from_svg(document: &str, metadata: M) -> Result<Self, IoError> {
        Self::from_svg_with_options(document, metadata, SvgOptions::default())
    }

    fn from_svg_with_options(
        document: &str,
        metadata: M,
        options: SvgOptions,
    ) -> Result<Self, IoError> {
        use svg::node::element::tag;
        use svg::node::element::tag::Type::{Empty, End, Start};
        use svg::parser::Event;

        let options = options.validate()?;
        let mut contexts = vec![StyleContext::default()];
        let mut output = Profile::empty(metadata.clone());
        let mut source_index = 0_u64;
        for event in svg::read(document)? {
            let Event::Tag(name, kind, attrs) = event else {
                if let Event::Error(error) = event {
                    return Err(error.into());
                }
                continue;
            };
            if matches!(name, tag::Group | tag::SVG) {
                match kind {
                    Start => {
                        let parent = *contexts.last().ok_or_else(|| {
                            IoError::MalformedInput("SVG style context stack is empty".into())
                        })?;
                        contexts.push(apply_style(parent, &attrs)?);
                    },
                    End => {
                        if contexts.len() > 1 {
                            contexts.pop();
                        }
                    },
                    Empty => {},
                }
                continue;
            }
            if matches!(kind, End) || matches!(name, tag::Description | tag::Title | tag::Text)
            {
                continue;
            }
            let parent = *contexts.last().ok_or_else(|| {
                IoError::MalformedInput("SVG style context stack is empty".into())
            })?;
            let context = apply_style(parent, &attrs)?;
            if !context.visible() {
                continue;
            }
            let shape = match name {
                tag::Path => {
                    let data = attrs.get("d").ok_or_else(|| {
                        IoError::MalformedInput("missing SVG path d attribute".into())
                    })?;
                    imported_path(data, context, source_index, metadata.clone())?
                },
                tag::Circle => {
                    let radius = number(&attrs, "r", None)?;
                    let segments = options.segments_for_radius(radius)?;
                    styled_shape(
                        Profile::circle(real(radius, "radius")?, segments, metadata.clone())
                            .translate(
                                real(number(&attrs, "cx", Some(0.0))?, "cx")?,
                                real(number(&attrs, "cy", Some(0.0))?, "cy")?,
                                Real::zero(),
                            ),
                        context,
                    )
                },
                tag::Ellipse => {
                    let rx = number(&attrs, "rx", None)?;
                    let ry = number(&attrs, "ry", None)?;
                    if rx <= 0.0 || ry <= 0.0 {
                        return Err(IoError::MalformedInput(
                            "SVG ellipse radii must be positive".into(),
                        ));
                    }
                    let segments = options.segments_for_radius(rx.max(ry))?;
                    styled_shape(
                        Profile::ellipse(
                            real(2.0 * rx, "rx")?,
                            real(2.0 * ry, "ry")?,
                            segments,
                            metadata.clone(),
                        )
                        .translate(
                            real(number(&attrs, "cx", Some(0.0))?, "cx")?,
                            real(number(&attrs, "cy", Some(0.0))?, "cy")?,
                            Real::zero(),
                        ),
                        context,
                    )
                },
                tag::Rectangle => rectangle(&attrs, context, options, metadata.clone())?,
                tag::Line => {
                    let wire = CurveString2::from_real_point_iter([
                        [
                            real(number(&attrs, "x1", Some(0.0))?, "x1")?,
                            real(number(&attrs, "y1", Some(0.0))?, "y1")?,
                        ],
                        [
                            real(number(&attrs, "x2", Some(0.0))?, "x2")?,
                            real(number(&attrs, "y2", Some(0.0))?, "y2")?,
                        ],
                    ])
                    .map_err(|error| IoError::Geometry {
                        format: "SVG",
                        detail: error.to_string(),
                    })?;
                    if context.strokes() {
                        Profile::from_wires(vec![wire], metadata.clone())
                    } else {
                        Profile::empty(metadata.clone())
                    }
                },
                tag::Polygon => {
                    let polygon_points = points(attrs.get("points").ok_or_else(|| {
                        IoError::MalformedInput("missing SVG polygon points".into())
                    })?)?;
                    if polygon_points.len() < 3 {
                        return Err(IoError::MalformedInput(
                            "SVG polygon requires at least three points".into(),
                        ));
                    }
                    styled_shape(Profile::polygon(&polygon_points, metadata.clone()), context)
                },
                tag::Polyline => {
                    let polyline_points = points(attrs.get("points").ok_or_else(|| {
                        IoError::MalformedInput("missing SVG polyline points".into())
                    })?)?;
                    let wire = CurveString2::from_real_point_iter(polyline_points.clone())
                        .map_err(|error| IoError::Geometry {
                            format: "SVG",
                            detail: error.to_string(),
                        })?;
                    let region = if context.fills() {
                        Profile::polygon(&polyline_points, metadata.clone())
                            .as_region()
                            .clone()
                    } else {
                        Region2::empty()
                    };
                    let wires = if context.strokes() {
                        vec![wire]
                    } else {
                        Vec::new()
                    };
                    Profile::from_region_and_wires(region, wires, metadata.clone())
                },
                other => {
                    return Err(IoError::Unsupported {
                        format: "SVG",
                        detail: format!("element {other}"),
                    });
                },
            };
            source_index += 1;
            let transformed = shape.transform(&context.transform.matrix()?);
            output = output
                .try_union(&transformed)
                .map_err(|error| IoError::Geometry {
                    format: "SVG",
                    detail: error.to_string(),
                })?;
        }
        Ok(output)
    }
}

fn rectangle<M: Clone + Debug + Send + Sync>(
    attrs: &svg::node::Attributes,
    context: StyleContext,
    options: SvgOptions,
    metadata: M,
) -> Result<Profile<M>, IoError> {
    let x = number(attrs, "x", Some(0.0))?;
    let y = number(attrs, "y", Some(0.0))?;
    let width = number(attrs, "width", None)?;
    let height = number(attrs, "height", None)?;
    if width <= 0.0 || height <= 0.0 {
        return Err(IoError::MalformedInput(
            "SVG rectangle dimensions must be positive".into(),
        ));
    }
    let rx_attr = attrs
        .get("rx")
        .map(|value| value.parse::<f64>())
        .transpose()?;
    let ry_attr = attrs
        .get("ry")
        .map(|value| value.parse::<f64>())
        .transpose()?;
    let (rx, ry) = match (rx_attr, ry_attr) {
        (None, None) => (0.0, 0.0),
        (Some(rx), None) => (rx, rx),
        (None, Some(ry)) => (ry, ry),
        (Some(rx), Some(ry)) => (rx, ry),
    };
    if !rx.is_finite() || !ry.is_finite() || rx < 0.0 || ry < 0.0 {
        return Err(IoError::MalformedInput(
            "SVG rectangle corner radii must be finite and non-negative".into(),
        ));
    }
    let rx = rx.clamp(0.0, width / 2.0);
    let ry = ry.clamp(0.0, height / 2.0);
    let shape = if rx == 0.0 || ry == 0.0 {
        Profile::rectangle(real(width, "width")?, real(height, "height")?, metadata)
    } else {
        let segments = options.segments_for_radius(rx.max(ry))? / 4;
        let mut boundary = Vec::new();
        for (cx, cy, start) in [
            (width - rx, ry, -std::f64::consts::FRAC_PI_2),
            (width - rx, height - ry, 0.0),
            (rx, height - ry, std::f64::consts::FRAC_PI_2),
            (rx, ry, std::f64::consts::PI),
        ] {
            for index in 0..=segments {
                let angle =
                    start + std::f64::consts::FRAC_PI_2 * index as f64 / segments as f64;
                boundary.push([
                    real(cx + rx * angle.cos(), "rounded rectangle x")?,
                    real(cy + ry * angle.sin(), "rounded rectangle y")?,
                ]);
            }
        }
        Profile::polygon(&boundary, metadata)
    };
    Ok(styled_shape(
        shape.translate(real(x, "x")?, real(y, "y")?, Real::zero()),
        context,
    ))
}

pub trait ToSVG {
    fn to_svg(&self) -> Result<String, IoError>;
    fn to_svg_with_options(&self, options: SvgOptions) -> Result<String, IoError>;
}

impl<M: Clone + Debug + Send + Sync> ToSVG for Profile<M> {
    fn to_svg(&self) -> Result<String, IoError> {
        self.to_svg_with_options(SvgOptions::default())
    }

    fn to_svg_with_options(&self, options: SvgOptions) -> Result<String, IoError> {
        let options = options.validate()?;
        let projection =
            FiniteProjectionOptions::try_new(options.curve_tolerance).map_err(|error| {
                IoError::Geometry {
                    format: "SVG",
                    detail: error.to_string(),
                }
            })?;
        let profiles = match self.project_region_profiles(&projection).map_err(|error| {
            IoError::Geometry {
                format: "SVG",
                detail: error.to_string(),
            }
        })? {
            Classification::Decided(profiles) => profiles,
            Classification::Uncertain(reason) => {
                return Err(IoError::Geometry {
                    format: "SVG",
                    detail: format!("region projection is uncertain: {reason:?}"),
                });
            },
        };
        let (min_x, min_y, max_x, max_y) =
            self.native_xy_bounds().ok_or_else(|| IoError::Geometry {
                format: "SVG",
                detail: "cannot bound empty geometry".into(),
            })?;
        let bounds = [
            finite_f64(&min_x, "SVG", "minimum x")?,
            finite_f64(&min_y, "SVG", "minimum y")?,
            finite_f64(&max_x, "SVG", "maximum x")?,
            finite_f64(&max_y, "SVG", "maximum y")?,
        ];
        let mut body = String::new();
        for profile in profiles {
            body.push_str("<path fill=\"black\" fill-rule=\"evenodd\" stroke=\"none\" d=\"");
            append_ring(&mut body, profile.material().points());
            for hole in profile.holes() {
                append_ring(&mut body, hole.points());
            }
            body.push_str("\"/>\n");
        }
        for wire in self.wires() {
            let polyline = wire
                .project_to_finite_polyline(&projection)
                .map_err(|error| IoError::Geometry {
                    format: "SVG",
                    detail: error.to_string(),
                })?;
            body.push_str("<path fill=\"none\" stroke=\"black\" d=\"");
            append_open(&mut body, polyline.points());
            body.push_str("\"/>\n");
        }
        Ok(format!(
            "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"{:.17} {:.17} {:.17} {:.17}\">\n{body}</svg>",
            bounds[0],
            bounds[1],
            bounds[2] - bounds[0],
            bounds[3] - bounds[1]
        ))
    }
}

fn append_ring(output: &mut String, points: &[[f64; 2]]) {
    append_open(output, points);
    output.push_str(" Z ");
}

fn append_open(output: &mut String, points: &[[f64; 2]]) {
    if let Some(first) = points.first() {
        output.push_str(&format!("M {:.17} {:.17}", first[0], first[1]));
        for point in &points[1..] {
            output.push_str(&format!(" L {:.17} {:.17}", point[0], point[1]));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn imports_inherited_transform_and_svg_defaults() {
        let document = r#"<svg xmlns="http://www.w3.org/2000/svg"><g transform="translate(3 4)"><circle r="2"/></g></svg>"#;
        let profile = Profile::<()>::from_svg(document, ()).unwrap();
        let bounds = profile.bounding_box();
        for (actual, expected) in [
            (&bounds.mins.x, 1.0),
            (&bounds.mins.y, 2.0),
            (&bounds.maxs.x, 5.0),
            (&bounds.maxs.y, 6.0),
        ] {
            assert!((actual.to_f64_lossy().unwrap() - expected).abs() < 0.001);
        }
    }

    #[test]
    fn polyline_fill_closes_but_stroke_remains_open() {
        let document = r#"<svg xmlns="http://www.w3.org/2000/svg"><polyline points="0,0 2,0 1,1" stroke="black"/></svg>"#;
        let profile = Profile::<()>::from_svg(document, ()).unwrap();
        assert_eq!(profile.as_region().material_contours().len(), 1);
        assert_eq!(profile.wires().len(), 1);
        assert_ne!(profile.wires()[0].start(), profile.wires()[0].end());
    }

    #[test]
    fn export_reports_empty_geometry() {
        assert!(Profile::<()>::empty(()).to_svg().is_err());
    }

    #[test]
    fn rejects_unbounded_curve_sampling() {
        let document =
            r#"<svg xmlns="http://www.w3.org/2000/svg"><circle r="1000000"/></svg>"#;
        assert!(Profile::<()>::from_svg(document, ()).is_err());
    }

    #[test]
    fn rejects_invalid_shape_dimensions() {
        for document in [
            r#"<svg xmlns="http://www.w3.org/2000/svg"><ellipse rx="2" ry="-1"/></svg>"#,
            r#"<svg xmlns="http://www.w3.org/2000/svg"><rect width="2" height="2" rx="-1"/></svg>"#,
            r#"<svg xmlns="http://www.w3.org/2000/svg"><polygon points="0,0 1,1"/></svg>"#,
        ] {
            assert!(Profile::<()>::from_svg(document, ()).is_err());
        }
    }
}
