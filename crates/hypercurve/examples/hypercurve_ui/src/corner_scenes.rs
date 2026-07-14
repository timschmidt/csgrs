use egui::{CentralPanel, ScrollArea, SidePanel, Slider};
use egui_plot::{Plot, PlotPoint, Text};
use hypercurve::{
    CircularArc2, CubicBezier2, Curve2, CurveFamily2, CurvePath2, CurveRegion2, LineSeg2, Point2,
    QuadraticBezier2, RationalBezier2, RationalQuadraticBezier2, Real,
};

use crate::geometry::{Polyline, Shape};
use crate::plotting::draw_shape;
use crate::theme::Theme;

const DISPLAY_STEPS: usize = 48;
#[cfg(test)]
const ALL_FAMILIES: [CurveFamily2; 8] = [
    CurveFamily2::Line,
    CurveFamily2::CircularArc,
    CurveFamily2::QuadraticBezier,
    CurveFamily2::CubicBezier,
    CurveFamily2::RationalQuadraticBezier,
    CurveFamily2::RationalBezier,
    CurveFamily2::PolynomialBSpline,
    CurveFamily2::Nurbs,
];

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CornerOperation {
    Fillet,
    Chamfer,
}

impl CornerOperation {
    const fn heading(self) -> &'static str {
        match self {
            Self::Fillet => "Fillets",
            Self::Chamfer => "Chamfers",
        }
    }

    const fn amount_label(self) -> &'static str {
        match self {
            Self::Fillet => "Radius",
            Self::Chamfer => "Setback",
        }
    }

    const fn amount_bounds(self) -> (f64, f64) {
        match self {
            Self::Fillet => (0.25, 0.7),
            Self::Chamfer => (0.25, 1.5),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum HoleCornerKind {
    AffinePair,
    ArcThenLine,
    LineThenArc,
}

#[derive(Clone, Copy, Debug)]
struct HoleSpec {
    origin: (i32, i32),
    previous_family: CurveFamily2,
    next_family: CurveFamily2,
    top_family: CurveFamily2,
    corner_kind: HoleCornerKind,
}

const HOLE_SPECS: [HoleSpec; 14] = [
    hole(
        (-24, -11),
        CurveFamily2::Line,
        CurveFamily2::QuadraticBezier,
        CurveFamily2::CircularArc,
    ),
    hole(
        (-14, -11),
        CurveFamily2::QuadraticBezier,
        CurveFamily2::CubicBezier,
        CurveFamily2::RationalQuadraticBezier,
    ),
    hole(
        (-4, -11),
        CurveFamily2::CubicBezier,
        CurveFamily2::RationalQuadraticBezier,
        CurveFamily2::RationalBezier,
    ),
    hole(
        (6, -11),
        CurveFamily2::RationalQuadraticBezier,
        CurveFamily2::RationalBezier,
        CurveFamily2::PolynomialBSpline,
    ),
    hole(
        (16, -11),
        CurveFamily2::RationalBezier,
        CurveFamily2::PolynomialBSpline,
        CurveFamily2::Nurbs,
    ),
    hole(
        (-24, -2),
        CurveFamily2::PolynomialBSpline,
        CurveFamily2::Nurbs,
        CurveFamily2::CircularArc,
    ),
    hole(
        (-14, -2),
        CurveFamily2::Nurbs,
        CurveFamily2::Line,
        CurveFamily2::QuadraticBezier,
    ),
    hole(
        (-4, -2),
        CurveFamily2::Line,
        CurveFamily2::CubicBezier,
        CurveFamily2::CubicBezier,
    ),
    hole(
        (6, -2),
        CurveFamily2::QuadraticBezier,
        CurveFamily2::RationalBezier,
        CurveFamily2::RationalQuadraticBezier,
    ),
    hole(
        (16, -2),
        CurveFamily2::CubicBezier,
        CurveFamily2::PolynomialBSpline,
        CurveFamily2::RationalBezier,
    ),
    hole(
        (-24, 7),
        CurveFamily2::RationalQuadraticBezier,
        CurveFamily2::Nurbs,
        CurveFamily2::CircularArc,
    ),
    hole(
        (-14, 7),
        CurveFamily2::Nurbs,
        CurveFamily2::QuadraticBezier,
        CurveFamily2::PolynomialBSpline,
    ),
    HoleSpec {
        origin: (-4, 7),
        previous_family: CurveFamily2::CircularArc,
        next_family: CurveFamily2::Line,
        top_family: CurveFamily2::Nurbs,
        corner_kind: HoleCornerKind::ArcThenLine,
    },
    HoleSpec {
        origin: (6, 7),
        previous_family: CurveFamily2::Line,
        next_family: CurveFamily2::CircularArc,
        top_family: CurveFamily2::CubicBezier,
        corner_kind: HoleCornerKind::LineThenArc,
    },
];

const fn hole(
    origin: (i32, i32),
    previous_family: CurveFamily2,
    next_family: CurveFamily2,
    top_family: CurveFamily2,
) -> HoleSpec {
    HoleSpec {
        origin,
        previous_family,
        next_family,
        top_family,
        corner_kind: HoleCornerKind::AffinePair,
    }
}

struct CornerRegionResult {
    region: CurveRegion2,
    display: Shape,
}

pub struct CornerScene {
    operation: CornerOperation,
    amount: f64,
    source_region: CurveRegion2,
    source_display: Shape,
    cached_amount_bits: Option<u64>,
    result_region: Option<CurveRegion2>,
    result_display: Option<Shape>,
    last_error: Option<String>,
}

impl CornerScene {
    pub fn new(operation: CornerOperation, amount: f64) -> Self {
        assert!(amount.is_finite(), "demo corner amount must be finite");
        let (minimum, maximum) = operation.amount_bounds();
        let amount = amount.clamp(minimum, maximum);
        let source_paths = curve_region_paths().expect("demo region paths must be valid");
        let source_region = CurveRegion2::try_from_boundary_paths(&source_paths)
            .expect("demo CurveRegion2 must be valid");
        let source_display = display_region(&source_paths).expect("demo region must be drawable");
        Self {
            operation,
            amount,
            source_region,
            source_display,
            cached_amount_bits: None,
            result_region: None,
            result_display: None,
            last_error: None,
        }
    }

    #[cfg(any(target_arch = "wasm32", test))]
    pub const fn amount(&self) -> f64 {
        self.amount
    }

    pub fn apply_amount(&mut self, amount: f64) -> Result<(), String> {
        if !amount.is_finite() {
            return Err(format!("{} must be finite", self.operation.amount_label()));
        }
        let (minimum, maximum) = self.operation.amount_bounds();
        self.amount = amount.clamp(minimum, maximum);
        self.cached_amount_bits = None;
        Ok(())
    }

    pub fn ui(&mut self, ctx: &egui::Context, theme: &Theme) {
        SidePanel::right(match self.operation {
            CornerOperation::Fillet => "fillet_controls",
            CornerOperation::Chamfer => "chamfer_controls",
        })
        .default_width(230.0)
        .show(ctx, |ui| {
            ScrollArea::vertical().show(ui, |ui| {
                ui.heading(self.operation.heading());
                let (minimum, maximum) = self.operation.amount_bounds();
                let mut slider =
                    Slider::new(&mut self.amount, minimum..=maximum)
                        .text(self.operation.amount_label());
                if self.operation == CornerOperation::Fillet {
                    // The egui value is the rational circle parameter `q`; only
                    // this display boundary presents the constructed radius 2q².
                    slider = slider
                        .custom_formatter(|scale, _| format!("{:.3}", 2.0 * scale * scale))
                        .custom_parser(|text| {
                            text.parse::<f64>()
                                .ok()
                                .filter(|radius| *radius >= 0.0)
                                .map(|radius| (radius / 2.0).sqrt())
                        });
                }
                ui.add(slider);
                ui.separator();
                ui.label(format!(
                    "CurveRegion2: 1 material + {} mixed-family holes",
                    self.source_region.len().saturating_sub(1)
                ));
                ui.small(
                    "Every adjustment rebuilds all hole corners across repeated curve-family pairings.",
                );
                if let Some(result) = &self.result_region {
                    ui.small(format!("Result retains {} exact boundary loops", result.len()));
                }
                if let Some(error) = &self.last_error {
                    ui.separator();
                    ui.colored_label(theme.error, error);
                }
            });
        });

        self.refresh_result();
        CentralPanel::default().show(ctx, |ui| {
            Plot::new(match self.operation {
                CornerOperation::Fillet => "fillet_plot",
                CornerOperation::Chamfer => "chamfer_plot",
            })
            .data_aspect(1.0)
            .allow_drag(true)
            .show(ui, |plot_ui| {
                draw_shape(
                    plot_ui,
                    "source CurveRegion2",
                    &self.source_display,
                    theme.warning,
                    None,
                    None,
                );
                if let Some(result) = &self.result_display {
                    draw_shape(
                        plot_ui,
                        "result CurveRegion2",
                        result,
                        theme.result,
                        Some(translucent(theme.result, 42)),
                        None,
                    );
                }
                plot_ui.text(
                    Text::new(
                        "corner operation label",
                        PlotPoint::new(0.0, 16.3),
                        format!(
                            "{} · {} mixed-family corner interactions",
                            self.operation.heading(),
                            HOLE_SPECS.len()
                        ),
                    )
                    .color(theme.accent),
                );
            });
        });
    }

    fn refresh_result(&mut self) {
        let amount_bits = self.amount.to_bits();
        if self.cached_amount_bits == Some(amount_bits) {
            return;
        }
        self.cached_amount_bits = Some(amount_bits);
        let exact_amount = match Real::try_from(self.amount).map_err(string_error) {
            Ok(amount) => amount,
            Err(error) => {
                self.last_error = Some(error);
                return;
            }
        };
        let source_paths = match curve_region_paths() {
            Ok(paths) => paths,
            Err(error) => {
                self.last_error = Some(error);
                return;
            }
        };
        match build_corner_result(self.operation, exact_amount, &source_paths) {
            Ok(result) => {
                self.result_region = Some(result.region);
                self.result_display = Some(result.display);
                self.last_error = None;
            }
            Err(error) => {
                self.result_region = None;
                self.result_display = None;
                self.last_error = Some(error);
            }
        }
    }

    #[cfg(test)]
    fn source_region(&self) -> &CurveRegion2 {
        &self.source_region
    }
}

fn build_corner_result(
    operation: CornerOperation,
    amount: Real,
    source_paths: &[CurvePath2],
) -> Result<CornerRegionResult, String> {
    let mut result_paths = source_paths.to_vec();
    for (hole_index, spec) in HOLE_SPECS.iter().enumerate() {
        result_paths[hole_index + 1] = edit_hole(
            &source_paths[hole_index + 1],
            *spec,
            operation,
            amount.clone(),
        )?;
    }
    let region = CurveRegion2::try_from_boundary_paths(&result_paths).map_err(string_error)?;
    let display = display_region(&result_paths)?;
    Ok(CornerRegionResult { region, display })
}

fn edit_hole(
    source: &CurvePath2,
    spec: HoleSpec,
    operation: CornerOperation,
    amount: Real,
) -> Result<CurvePath2, String> {
    if operation == CornerOperation::Chamfer {
        return source
            .chamfer_vertex_by_parameters(
                1,
                unit_setback_parameter(&amount)?,
                unit_advance_parameter(&amount)?,
            )
            .map_err(string_error);
    }

    let radius = Real::from(2_i32) * (&amount * &amount);
    let (previous_parameter, next_parameter, center_x, clockwise) = match spec.corner_kind {
        HoleCornerKind::AffinePair => (
            unit_setback_parameter(&radius)?,
            unit_advance_parameter(&radius)?,
            -radius.clone(),
            false,
        ),
        HoleCornerKind::ArcThenLine => {
            let witnesses = arc_line_fillet_witnesses(&amount)?;
            (
                witnesses.arc_then_line_parameter,
                witnesses.line_advance_parameter,
                -witnesses.center_distance,
                true,
            )
        }
        HoleCornerKind::LineThenArc => {
            let witnesses = arc_line_fillet_witnesses(&amount)?;
            (
                Real::one() - &witnesses.line_advance_parameter,
                witnesses.line_then_arc_parameter,
                -witnesses.center_distance,
                false,
            )
        }
    };
    let center = local_real_point(spec.origin, center_x, radius);
    source
        .fillet_vertex_by_parameters(1, previous_parameter, next_parameter, &center, clockwise)
        .map_err(string_error)
}

fn unit_setback_parameter(amount: &Real) -> Result<Real, String> {
    let four = Real::from(4_i32);
    ((&four - amount) / four).map_err(string_error)
}

fn unit_advance_parameter(amount: &Real) -> Result<Real, String> {
    (amount / Real::from(4_i32)).map_err(string_error)
}

struct ArcLineFilletWitnesses {
    center_distance: Real,
    line_advance_parameter: Real,
    arc_then_line_parameter: Real,
    line_then_arc_parameter: Real,
}

fn arc_line_fillet_witnesses(scale: &Real) -> Result<ArcLineFilletWitnesses, String> {
    // On the fixed radius-2 source circle, q gives exact rational tangent
    // witnesses and a fillet radius of 2q². Keeping q below 3/4 keeps both
    // tangent points strictly inside the authored arc and line domains.
    let center_distance = Real::from(4_i32) * scale;
    let line_advance_parameter = scale.clone();
    let maximum_scale = rational(3, 4);
    let angular_half_tangent = ((&maximum_scale - scale) / (Real::one() + &maximum_scale * scale))
        .map_err(string_error)?;
    let arc_then_line_parameter = (&angular_half_tangent
        / (rational(3, 5) + rational(1, 5) * &angular_half_tangent))
        .map_err(string_error)?;
    Ok(ArcLineFilletWitnesses {
        center_distance,
        line_advance_parameter,
        line_then_arc_parameter: Real::one() - &arc_then_line_parameter,
        arc_then_line_parameter,
    })
}

fn curve_region_paths() -> Result<Vec<CurvePath2>, String> {
    let mut paths = Vec::with_capacity(HOLE_SPECS.len() + 1);
    paths.push(outer_boundary()?);
    for spec in HOLE_SPECS {
        paths.push(family_hole(spec)?);
    }
    Ok(paths)
}

fn outer_boundary() -> Result<CurvePath2, String> {
    let points = [
        point(-28, -14),
        point(0, -14),
        point(24, -14),
        point(24, 0),
        point(24, 15),
        point(0, 15),
        point(-28, 15),
        point(-28, 0),
    ];
    let families = [
        CurveFamily2::Line,
        CurveFamily2::QuadraticBezier,
        CurveFamily2::CubicBezier,
        CurveFamily2::RationalQuadraticBezier,
        CurveFamily2::RationalBezier,
        CurveFamily2::PolynomialBSpline,
        CurveFamily2::Nurbs,
        CurveFamily2::Line,
    ];
    let mut curves = Vec::with_capacity(points.len());
    for index in 0..points.len() {
        curves.push(affine_family_curve(
            families[index],
            points[index].clone(),
            points[(index + 1) % points.len()].clone(),
        )?);
    }
    CurvePath2::try_new(curves).map_err(string_error)
}

fn family_hole(spec: HoleSpec) -> Result<CurvePath2, String> {
    match spec.corner_kind {
        HoleCornerKind::AffinePair => affine_pair_hole(spec),
        HoleCornerKind::ArcThenLine => arc_then_line_hole(spec),
        HoleCornerKind::LineThenArc => line_then_arc_hole(spec),
    }
}

fn affine_pair_hole(spec: HoleSpec) -> Result<CurvePath2, String> {
    CurvePath2::try_new(vec![
        affine_family_curve(
            spec.previous_family,
            local_point(spec.origin, -4, 0),
            local_point(spec.origin, 0, 0),
        )?,
        affine_family_curve(
            spec.next_family,
            local_point(spec.origin, 0, 0),
            local_point(spec.origin, 0, 4),
        )?,
        representative_top_curve(spec.top_family, spec.origin)?,
        line_curve(
            local_point(spec.origin, -4, 4),
            local_point(spec.origin, -4, 0),
        )?,
    ])
    .map_err(string_error)
}

fn arc_then_line_hole(spec: HoleSpec) -> Result<CurvePath2, String> {
    let arc_start = arc_transition_point(spec.origin);
    let previous = CircularArc2::try_from_center(
        arc_start.clone(),
        local_point(spec.origin, 0, 0),
        local_point(spec.origin, 0, 2),
        false,
    )
    .map_err(string_error)?;
    CurvePath2::try_new(vec![
        Curve2::from(previous),
        line_curve(
            local_point(spec.origin, 0, 0),
            local_point(spec.origin, -4, 0),
        )?,
        line_curve(
            local_point(spec.origin, -4, 0),
            local_point(spec.origin, -4, 4),
        )?,
        representative_top_curve(spec.top_family, spec.origin)?
            .reversed()
            .map_err(string_error)?,
        line_curve(local_point(spec.origin, 0, 4), arc_start)?,
    ])
    .map_err(string_error)
}

fn line_then_arc_hole(spec: HoleSpec) -> Result<CurvePath2, String> {
    let arc_end = arc_transition_point(spec.origin);
    let next = CircularArc2::try_from_center(
        local_point(spec.origin, 0, 0),
        arc_end.clone(),
        local_point(spec.origin, 0, 2),
        true,
    )
    .map_err(string_error)?;
    CurvePath2::try_new(vec![
        line_curve(
            local_point(spec.origin, -4, 0),
            local_point(spec.origin, 0, 0),
        )?,
        Curve2::from(next),
        line_curve(arc_end, local_point(spec.origin, 0, 4))?,
        representative_top_curve(spec.top_family, spec.origin)?,
        line_curve(
            local_point(spec.origin, -4, 4),
            local_point(spec.origin, -4, 0),
        )?,
    ])
    .map_err(string_error)
}

fn arc_transition_point(origin: (i32, i32)) -> Point2 {
    local_real_point(origin, -rational(48, 25), rational(36, 25))
}

fn affine_family_curve(family: CurveFamily2, start: Point2, end: Point2) -> Result<Curve2, String> {
    let midpoint = interpolate_point(&start, &end, rational(1, 2));
    let first_third = interpolate_point(&start, &end, rational(1, 3));
    let second_third = interpolate_point(&start, &end, rational(2, 3));
    Ok(match family {
        CurveFamily2::Line => line_curve(start, end)?,
        CurveFamily2::QuadraticBezier => Curve2::from(QuadraticBezier2::new(start, midpoint, end)),
        CurveFamily2::CubicBezier => {
            Curve2::from(CubicBezier2::new(start, first_third, second_third, end))
        }
        CurveFamily2::RationalQuadraticBezier => Curve2::from(
            RationalQuadraticBezier2::try_new(
                start,
                midpoint,
                end,
                Real::one(),
                Real::one(),
                Real::one(),
            )
            .map_err(string_error)?,
        ),
        CurveFamily2::RationalBezier => Curve2::from(
            RationalBezier2::try_new(
                vec![start, first_third, second_third, end],
                vec![Real::one(); 4],
            )
            .map_err(string_error)?,
        ),
        CurveFamily2::PolynomialBSpline => {
            Curve2::try_polynomial_bspline(1, vec![start, end], linear_spline_knots(), None)
                .map_err(string_error)?
        }
        CurveFamily2::Nurbs => Curve2::try_nurbs(
            1,
            vec![start, end],
            vec![Real::one(), Real::one()],
            linear_spline_knots(),
            None,
        )
        .map_err(string_error)?,
        CurveFamily2::CircularArc => {
            return Err("a circular arc cannot carry an affine line image".into());
        }
    })
}

fn representative_top_curve(family: CurveFamily2, origin: (i32, i32)) -> Result<Curve2, String> {
    let start = local_point(origin, 0, 4);
    let end = local_point(origin, -4, 4);
    Ok(match family {
        CurveFamily2::Line => line_curve(start, end)?,
        CurveFamily2::CircularArc => Curve2::from(
            CircularArc2::from_bulge(start, end, rational(1, 3)).map_err(string_error)?,
        ),
        CurveFamily2::QuadraticBezier => Curve2::from(QuadraticBezier2::new(
            start,
            local_point(origin, -2, 6),
            end,
        )),
        CurveFamily2::CubicBezier => Curve2::from(CubicBezier2::new(
            start,
            local_point(origin, -1, 6),
            local_point(origin, -3, 3),
            end,
        )),
        CurveFamily2::RationalQuadraticBezier => Curve2::from(
            RationalQuadraticBezier2::try_unit_end_weights(
                start,
                local_point(origin, -2, 6),
                end,
                Real::from(2_i32),
            )
            .map_err(string_error)?,
        ),
        CurveFamily2::RationalBezier => Curve2::from(
            RationalBezier2::try_new(
                vec![
                    start,
                    local_point(origin, -1, 6),
                    local_point(origin, -2, 3),
                    local_point(origin, -3, 6),
                    end,
                ],
                vec![
                    Real::one(),
                    Real::from(2_i32),
                    Real::one(),
                    Real::from(3_i32),
                    Real::one(),
                ],
            )
            .map_err(string_error)?,
        ),
        CurveFamily2::PolynomialBSpline => {
            Curve2::try_polynomial_bspline(3, top_spline_points(origin), top_spline_knots(), None)
                .map_err(string_error)?
        }
        CurveFamily2::Nurbs => Curve2::try_nurbs(
            3,
            top_spline_points(origin),
            vec![
                Real::one(),
                Real::from(2_i32),
                Real::one(),
                Real::from(3_i32),
                Real::one(),
            ],
            top_spline_knots(),
            None,
        )
        .map_err(string_error)?,
    })
}

fn interpolate_point(start: &Point2, end: &Point2, parameter: Real) -> Point2 {
    let x = start.x() + &((end.x() - start.x()) * &parameter);
    let y = start.y() + &((end.y() - start.y()) * parameter);
    Point2::new(x, y)
}

fn top_spline_points(origin: (i32, i32)) -> Vec<Point2> {
    vec![
        local_point(origin, 0, 4),
        local_point(origin, -1, 6),
        local_point(origin, -2, 3),
        local_point(origin, -3, 6),
        local_point(origin, -4, 4),
    ]
}

fn top_spline_knots() -> Vec<Real> {
    vec![
        Real::zero(),
        Real::zero(),
        Real::zero(),
        Real::zero(),
        Real::one(),
        Real::from(2_i32),
        Real::from(2_i32),
        Real::from(2_i32),
        Real::from(2_i32),
    ]
}

fn linear_spline_knots() -> Vec<Real> {
    vec![Real::zero(), Real::zero(), Real::one(), Real::one()]
}

fn display_region(paths: &[CurvePath2]) -> Result<Shape, String> {
    let material = sample_path(&paths[0], DISPLAY_STEPS)?;
    let holes = paths[1..]
        .iter()
        .map(|path| sample_path(path, DISPLAY_STEPS).map(Polyline::marked_hole))
        .collect::<Result<Vec<_>, _>>()?;
    Ok(Shape {
        materials: vec![material],
        holes,
    })
}

fn sample_path(path: &CurvePath2, steps: usize) -> Result<Polyline, String> {
    let mut display = Polyline::new();
    let step_count = Real::from(i32::try_from(steps).map_err(string_error)?);
    for (curve_index, curve) in path.curves().iter().enumerate() {
        let domain = curve.parameter_domain();
        let span = domain.end() - domain.start();
        for index in 0..=steps {
            if curve_index > 0 && index == 0 {
                continue;
            }
            let fraction = (Real::from(i32::try_from(index).map_err(string_error)?) / &step_count)
                .map_err(string_error)?;
            let parameter = domain.start() + &(&span * fraction);
            let point = curve.point_at(&parameter).map_err(string_error)?;
            display.add(real_to_f64(point.x()), real_to_f64(point.y()), 0.0);
        }
    }
    display.is_closed = path.start() == path.end();
    Ok(display)
}

fn line_curve(start: Point2, end: Point2) -> Result<Curve2, String> {
    LineSeg2::try_new(start, end)
        .map(Curve2::from)
        .map_err(string_error)
}

fn point(x: i32, y: i32) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn local_point(origin: (i32, i32), x: i32, y: i32) -> Point2 {
    point(origin.0 + x, origin.1 + y)
}

fn local_real_point(origin: (i32, i32), x: Real, y: Real) -> Point2 {
    Point2::new(Real::from(origin.0) + x, Real::from(origin.1) + y)
}

fn rational(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).expect("nonzero exact denominator")
}

fn real_to_f64(value: &Real) -> f64 {
    value
        .to_f64_lossy()
        .unwrap_or_else(|| f64::from(value.clone()))
}

fn translucent(color: egui::Color32, alpha: u8) -> egui::Color32 {
    egui::Color32::from_rgba_unmultiplied(color.r(), color.g(), color.b(), alpha)
}

fn string_error(error: impl std::fmt::Display) -> String {
    error.to_string()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn corner_region_repeats_every_curve_family_across_many_complex_holes() {
        let scene = CornerScene::new(CornerOperation::Fillet, 1.0);
        let provenance = scene
            .source_region()
            .fragment_provenance()
            .expect("direct CurveRegion2 construction retains provenance");

        assert_eq!(scene.source_region().len(), HOLE_SPECS.len() + 1);
        assert_eq!(scene.source_display.materials.len(), 1);
        assert_eq!(scene.source_display.holes.len(), HOLE_SPECS.len());
        for family in ALL_FAMILIES {
            assert!(
                provenance
                    .iter()
                    .filter(|fragment| fragment.family() == family)
                    .count()
                    >= 2,
                "{family:?} should appear multiple times"
            );
        }
    }

    #[test]
    fn both_corner_tabs_materialize_every_hole_at_both_slider_extremes() {
        for operation in [CornerOperation::Fillet, CornerOperation::Chamfer] {
            let mut scene = CornerScene::new(operation, 1.0);
            let amounts = match operation {
                CornerOperation::Fillet => [0.25, 0.7],
                CornerOperation::Chamfer => [0.25, 1.5],
            };
            for amount in amounts {
                scene.apply_amount(amount).unwrap();
                scene.refresh_result();
                assert!(
                    scene.result_region.is_some(),
                    "{operation:?} at {amount} failed: {:?}",
                    scene.last_error
                );
                assert_eq!(
                    scene.result_region.as_ref().unwrap().len(),
                    HOLE_SPECS.len() + 1
                );
                assert_eq!(
                    scene.result_display.as_ref().unwrap().holes.len(),
                    HOLE_SPECS.len()
                );
            }
        }
    }
}
