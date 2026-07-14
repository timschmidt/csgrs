use egui::{CentralPanel, ScrollArea, SidePanel, Slider};
use egui_plot::{Plot, PlotPoint, Text};
use hypercurve::{
    CircularArc2, CubicBezier2, Curve2, CurveFamily2, CurvePolicy, CurveString2, LineSeg2, Point2,
    QuadraticBezier2, RationalBezier2, RationalQuadraticBezier2, Real, Segment2,
};

use crate::geometry::Polyline;
use crate::plotting::draw_polyline;
use crate::theme::Theme;

const GALLERY_STEPS: usize = 48;
const FAMILY_COLORS: [egui::Color32; 8] = [
    egui::Color32::from_rgb(70, 150, 235),
    egui::Color32::from_rgb(225, 95, 105),
    egui::Color32::from_rgb(70, 180, 100),
    egui::Color32::from_rgb(220, 160, 55),
    egui::Color32::from_rgb(155, 105, 220),
    egui::Color32::from_rgb(45, 175, 175),
    egui::Color32::from_rgb(210, 90, 175),
    egui::Color32::from_rgb(125, 150, 55),
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
}

struct CurveFamilyExample {
    curve: Curve2,
    display: Polyline,
    label_position: [f64; 2],
}

impl CurveFamilyExample {
    fn new(curve: Curve2, label_position: [f64; 2]) -> Result<Self, String> {
        let display = sample_curve(&curve, GALLERY_STEPS)?;
        Ok(Self {
            curve,
            display,
            label_position,
        })
    }
}

pub struct CornerScene {
    operation: CornerOperation,
    amount: f64,
    source: Polyline,
    examples: Vec<CurveFamilyExample>,
    cached_amount_bits: Option<u64>,
    result: Option<Polyline>,
    last_error: Option<String>,
}

impl CornerScene {
    pub fn new(operation: CornerOperation, amount: f64) -> Self {
        let source = line_corner().expect("the exact demo corner must be valid");
        let examples = curve_family_examples().expect("the exact curve gallery must be valid");
        Self {
            operation,
            amount,
            source: Polyline::from_segments(source.segments(), false),
            examples,
            cached_amount_bits: None,
            result: None,
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
        self.amount = amount.clamp(0.25, 2.5);
        self.cached_amount_bits = None;
        Ok(())
    }

    pub fn ui(&mut self, ctx: &egui::Context, theme: &Theme) {
        SidePanel::right(match self.operation {
            CornerOperation::Fillet => "fillet_controls",
            CornerOperation::Chamfer => "chamfer_controls",
        })
        .default_width(220.0)
        .show(ctx, |ui| {
            ScrollArea::vertical().show(ui, |ui| {
                ui.heading(self.operation.heading());
                ui.add(
                    Slider::new(&mut self.amount, 0.25..=2.5).text(self.operation.amount_label()),
                );
                ui.separator();
                for (index, example) in self.examples.iter().enumerate() {
                    ui.colored_label(FAMILY_COLORS[index], family_label(example.curve.family()));
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
                draw_polyline(
                    plot_ui,
                    "source corner",
                    &self.source,
                    theme.warning,
                    None,
                    Some(theme.warning),
                );
                if let Some(result) = &self.result {
                    draw_polyline(
                        plot_ui,
                        self.operation.heading(),
                        result,
                        theme.result,
                        None,
                        Some(theme.result),
                    );
                }
                plot_ui.text(
                    Text::new(
                        "corner operation label",
                        PlotPoint::new(0.0, 12.6),
                        self.operation.heading(),
                    )
                    .color(theme.accent),
                );

                for (index, example) in self.examples.iter().enumerate() {
                    let label = family_label(example.curve.family());
                    draw_polyline(
                        plot_ui,
                        label,
                        &example.display,
                        FAMILY_COLORS[index],
                        None,
                        None,
                    );
                    plot_ui.text(
                        Text::new(
                            format!("{label} label"),
                            PlotPoint::new(example.label_position[0], example.label_position[1]),
                            label,
                        )
                        .color(FAMILY_COLORS[index]),
                    );
                }
            });
        });
    }

    fn refresh_result(&mut self) {
        let amount_bits = self.amount.to_bits();
        if self.cached_amount_bits == Some(amount_bits) {
            return;
        }
        self.cached_amount_bits = Some(amount_bits);
        match build_corner_result(self.operation, self.amount) {
            Ok(result) => {
                self.result = Some(result);
                self.last_error = None;
            }
            Err(error) => {
                self.result = None;
                self.last_error = Some(error);
            }
        }
    }

    #[cfg(test)]
    fn families(&self) -> Vec<CurveFamily2> {
        self.examples
            .iter()
            .map(|example| example.curve.family())
            .collect()
    }

    #[cfg(test)]
    fn result(&mut self) -> Option<&Polyline> {
        self.refresh_result();
        self.result.as_ref()
    }
}

fn line_corner() -> Result<CurveString2, String> {
    CurveString2::try_new(vec![
        Segment2::Line(LineSeg2::try_new(point(-4, 10), point(0, 10)).map_err(string_error)?),
        Segment2::Line(LineSeg2::try_new(point(0, 10), point(0, 14)).map_err(string_error)?),
    ])
    .map_err(string_error)
}

fn build_corner_result(operation: CornerOperation, amount: f64) -> Result<Polyline, String> {
    let amount = Real::try_from(amount).map_err(string_error)?;
    let four = Real::from(4_i32);
    let previous_parameter = ((&four - &amount) / &four).map_err(string_error)?;
    let next_parameter = (&amount / &four).map_err(string_error)?;
    let source = line_corner()?;
    let policy = CurvePolicy::certified();
    let result = match operation {
        CornerOperation::Fillet => source
            .fillet_vertex_by_parameters(
                1,
                previous_parameter,
                next_parameter,
                &Point2::new(-amount.clone(), Real::from(10_i32) + amount),
                false,
                &policy,
            )
            .map_err(string_error)?
            .into_curve_string(),
        CornerOperation::Chamfer => source
            .chamfer_vertex_by_parameters(1, previous_parameter, next_parameter, &policy)
            .map_err(string_error)?
            .into_curve_string(),
    }
    .ok_or_else(|| format!("exact {} did not materialize", operation.heading()))?;
    Ok(Polyline::from_segments(result.segments(), false))
}

fn curve_family_examples() -> Result<Vec<CurveFamilyExample>, String> {
    let origins = [
        (-8, 6),
        (1, 6),
        (-8, 2),
        (1, 2),
        (-8, -2),
        (1, -2),
        (-8, -6),
        (1, -6),
    ];
    let curves = vec![
        Curve2::from(
            LineSeg2::try_new(point_at(origins[0], 0, 0), point_at(origins[0], 5, 2))
                .map_err(string_error)?,
        ),
        Curve2::from(
            CircularArc2::from_bulge(
                point_at(origins[1], 0, 0),
                point_at(origins[1], 5, 0),
                rational(1, 2),
            )
            .map_err(string_error)?,
        ),
        Curve2::from(QuadraticBezier2::new(
            point_at(origins[2], 0, 0),
            point_at(origins[2], 2, 3),
            point_at(origins[2], 5, 0),
        )),
        Curve2::from(CubicBezier2::new(
            point_at(origins[3], 0, 0),
            point_at(origins[3], 1, 3),
            point_at(origins[3], 4, -2),
            point_at(origins[3], 5, 1),
        )),
        Curve2::from(
            RationalQuadraticBezier2::try_unit_end_weights(
                point_at(origins[4], 0, 0),
                point_at(origins[4], 2, 3),
                point_at(origins[4], 5, 0),
                Real::from(2_i32),
            )
            .map_err(string_error)?,
        ),
        Curve2::from(
            RationalBezier2::try_new(
                vec![
                    point_at(origins[5], 0, 0),
                    point_at(origins[5], 1, 3),
                    point_at(origins[5], 3, -2),
                    point_at(origins[5], 4, 3),
                    point_at(origins[5], 5, 0),
                ],
                vec![
                    Real::from(1_i32),
                    Real::from(2_i32),
                    Real::from(1_i32),
                    Real::from(3_i32),
                    Real::from(1_i32),
                ],
            )
            .map_err(string_error)?,
        ),
        Curve2::try_polynomial_bspline(3, spline_points(origins[6]), spline_knots(), None)
            .map_err(string_error)?,
        Curve2::try_nurbs(
            3,
            spline_points(origins[7]),
            vec![
                Real::from(1_i32),
                Real::from(2_i32),
                Real::from(1_i32),
                Real::from(3_i32),
                Real::from(1_i32),
            ],
            spline_knots(),
            None,
        )
        .map_err(string_error)?,
    ];

    curves
        .into_iter()
        .zip(origins)
        .map(|(curve, (x, y))| {
            CurveFamilyExample::new(curve, [f64::from(x) + 2.5, f64::from(y) - 0.7])
        })
        .collect()
}

fn spline_points(origin: (i32, i32)) -> Vec<Point2> {
    vec![
        point_at(origin, 0, 0),
        point_at(origin, 1, 3),
        point_at(origin, 3, -2),
        point_at(origin, 4, 3),
        point_at(origin, 5, 0),
    ]
}

fn spline_knots() -> Vec<Real> {
    vec![
        Real::from(0_i32),
        Real::from(0_i32),
        Real::from(0_i32),
        Real::from(0_i32),
        Real::from(1_i32),
        Real::from(2_i32),
        Real::from(2_i32),
        Real::from(2_i32),
        Real::from(2_i32),
    ]
}

fn sample_curve(curve: &Curve2, steps: usize) -> Result<Polyline, String> {
    let domain = curve.parameter_domain();
    let span = domain.end() - domain.start();
    let mut display = Polyline::new();
    for index in 0..=steps {
        let fraction = Real::try_from(index as f64 / steps as f64).map_err(string_error)?;
        let parameter = domain.start() + &(&span * fraction);
        let point = curve.point_at(&parameter).map_err(string_error)?;
        display.add(real_to_f64(point.x()), real_to_f64(point.y()), 0.0);
    }
    Ok(display)
}

fn point(x: i32, y: i32) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn point_at(origin: (i32, i32), x: i32, y: i32) -> Point2 {
    point(origin.0 + x, origin.1 + y)
}

fn rational(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).expect("nonzero exact denominator")
}

fn real_to_f64(value: &Real) -> f64 {
    value
        .to_f64_lossy()
        .unwrap_or_else(|| f64::from(value.clone()))
}

fn family_label(family: CurveFamily2) -> &'static str {
    match family {
        CurveFamily2::Line => "Line",
        CurveFamily2::CircularArc => "Circular arc",
        CurveFamily2::QuadraticBezier => "Quadratic Bezier",
        CurveFamily2::CubicBezier => "Cubic Bezier",
        CurveFamily2::RationalQuadraticBezier => "Rational quadratic Bezier",
        CurveFamily2::RationalBezier => "Rational Bezier",
        CurveFamily2::PolynomialBSpline => "Polynomial B-spline",
        CurveFamily2::Nurbs => "NURBS",
    }
}

fn string_error(error: impl std::fmt::Display) -> String {
    error.to_string()
}

#[cfg(test)]
mod tests {
    use super::*;

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

    #[test]
    fn fillet_scene_materializes_and_represents_every_curve_family() {
        let mut scene = CornerScene::new(CornerOperation::Fillet, 1.0);

        assert_eq!(scene.families(), ALL_FAMILIES);
        assert!(scene.result().is_some());
    }

    #[test]
    fn chamfer_scene_materializes_and_represents_every_curve_family() {
        let mut scene = CornerScene::new(CornerOperation::Chamfer, 1.0);

        assert_eq!(scene.families(), ALL_FAMILIES);
        assert!(scene.result().is_some());
    }
}
