use egui::{CentralPanel, ScrollArea, SidePanel, Slider};
use egui_plot::{Plot, PlotPoint, Text};
use hypercurve::{
    CircularArc2, CubicBezier2, Curve2, CurveFamily2, CurveGeometry2, CurveParameterSide2,
    CurvePath2, CurveRegion2, LineSeg2, Point2, QuadraticBezier2, RationalBezier2,
    RationalQuadraticBezier2, Real, RealSign,
};

use crate::geometry::{Polyline, Shape};
use crate::plotting::draw_shape;
use crate::theme::Theme;

const DISPLAY_STEPS: usize = 48;
const FAMILY_COUNT: usize = 8;
const HOLE_COUNT: usize = 4;
#[cfg(test)]
const OUTER_CURVE_COUNT: usize = FAMILY_COUNT * 2;
const HOLE_CURVE_COUNT: usize = FAMILY_COUNT;
const EDITED_CORNER_COUNT: usize = 36;
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

#[derive(Clone, Copy, Debug)]
struct HoleTemplate {
    origin: (i32, i32),
    points: [(i32, i32); HOLE_CURVE_COUNT],
    arc_center: (i32, i32),
    families_after_arc: [CurveFamily2; FAMILY_COUNT - 1],
}

fn hole_templates() -> [HoleTemplate; HOLE_COUNT] {
    // Affine chords use integer Pythagorean directions, while each arc meets
    // its neighbors tangentially. That keeps every corner decision exact.
    [
        HoleTemplate {
            origin: (-48, -9),
            points: [
                (-13, -2),
                (-8, -7),
                (4, -7),
                (8, -4),
                (8, 8),
                (4, 11),
                (-9, 11),
                (-13, 8),
            ],
            arc_center: (-8, -2),
            families_after_arc: [
                CurveFamily2::Line,
                CurveFamily2::QuadraticBezier,
                CurveFamily2::CubicBezier,
                CurveFamily2::RationalQuadraticBezier,
                CurveFamily2::RationalBezier,
                CurveFamily2::PolynomialBSpline,
                CurveFamily2::Nurbs,
            ],
        },
        HoleTemplate {
            origin: (24, -8),
            points: [
                (-13, -4),
                (-7, -10),
                (5, -10),
                (9, -7),
                (9, 5),
                (5, 8),
                (-9, 8),
                (-13, 5),
            ],
            arc_center: (-7, -4),
            families_after_arc: [
                CurveFamily2::CubicBezier,
                CurveFamily2::Nurbs,
                CurveFamily2::Line,
                CurveFamily2::RationalBezier,
                CurveFamily2::QuadraticBezier,
                CurveFamily2::PolynomialBSpline,
                CurveFamily2::RationalQuadraticBezier,
            ],
        },
        HoleTemplate {
            origin: (-45, 43),
            points: [
                (-14, -1),
                (-10, -5),
                (8, -5),
                (11, -1),
                (7, 2),
                (11, 5),
                (-10, 5),
                (-14, 2),
            ],
            arc_center: (-10, -1),
            families_after_arc: [
                CurveFamily2::PolynomialBSpline,
                CurveFamily2::RationalQuadraticBezier,
                CurveFamily2::Nurbs,
                CurveFamily2::QuadraticBezier,
                CurveFamily2::Line,
                CurveFamily2::CubicBezier,
                CurveFamily2::RationalBezier,
            ],
        },
        HoleTemplate {
            origin: (37, 39),
            points: [
                (-10, -6),
                (-5, -11),
                (5, -11),
                (9, -8),
                (9, 4),
                (5, 7),
                (-2, 7),
                (-10, 1),
            ],
            arc_center: (-5, -6),
            families_after_arc: [
                CurveFamily2::RationalBezier,
                CurveFamily2::CubicBezier,
                CurveFamily2::PolynomialBSpline,
                CurveFamily2::Line,
                CurveFamily2::Nurbs,
                CurveFamily2::RationalQuadraticBezier,
                CurveFamily2::QuadraticBezier,
            ],
        },
    ]
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
                let mut slider = Slider::new(&mut self.amount, minimum..=maximum)
                    .text(self.operation.amount_label());
                if self.operation == CornerOperation::Fillet {
                    slider = slider
                        .custom_formatter(|q, _| format!("{:.3}", 2.0 * q * q))
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
                    "CurveRegion2: {} boundaries · 1 irregular material + {HOLE_COUNT} unique eight-family holes",
                    self.source_region.len(),
                ));
                ui.small(
                    "Every curve family appears twice on the concave outer loop and once in each hole. Every non-smooth corner is edited.",
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
                        PlotPoint::new(0.0, 125.0),
                        format!(
                            "{} · {} exact corners",
                            self.operation.heading(),
                            EDITED_CORNER_COUNT
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
    let result_paths = source_paths
        .iter()
        .enumerate()
        .map(|(boundary_index, path)| {
            edit_all_corners(path, operation, amount.clone())
                .map_err(|error| format!("boundary {boundary_index}: {error}"))
        })
        .collect::<Result<Vec<_>, _>>()?;
    let region = CurveRegion2::try_from_boundary_paths(&result_paths).map_err(string_error)?;
    let display = display_region(&result_paths)?;
    Ok(CornerRegionResult { region, display })
}

#[derive(Clone)]
struct CornerWitness {
    previous_parameter: Real,
    next_parameter: Real,
    center: Point2,
    clockwise: bool,
}

fn edit_all_corners(
    source: &CurvePath2,
    operation: CornerOperation,
    amount: Real,
) -> Result<CurvePath2, String> {
    if source.curves().len() < 2 {
        return Ok(source.clone());
    }
    let mut corner_indices = Vec::new();
    for vertex_index in 0..source.curves().len() {
        if corner_orientation(source, vertex_index)?.is_some() {
            corner_indices.push(vertex_index);
        }
    }
    let mut result = source.clone();

    // Editing in descending source order keeps every not-yet-edited vertex at
    // its original index. Recomputing each witness on the current path accounts
    // for the neighboring end trim already applied to a retained curve.
    for vertex_index in corner_indices.iter().rev() {
        if *vertex_index != 0 {
            let witness = corner_witness_for(&result, *vertex_index, operation, &amount)?;
            result = apply_corner_edit(result, *vertex_index, operation, &witness)?;
        }
    }
    if corner_indices.first() == Some(&0) {
        let seam_witness = corner_witness_for(&result, 0, operation, &amount)?;
        result = apply_corner_edit(result, 0, operation, &seam_witness)?;
    }
    Ok(result)
}

fn corner_orientation(source: &CurvePath2, vertex_index: usize) -> Result<Option<bool>, String> {
    let previous_index = if vertex_index == 0 {
        source.curves().len() - 1
    } else {
        vertex_index - 1
    };
    let previous = &source.curves()[previous_index];
    let next = &source.curves()[vertex_index];
    let (previous_dx, previous_dy) = endpoint_tangent(previous, false)?;
    let (next_dx, next_dy) = endpoint_tangent(next, true)?;
    let cross = &previous_dx * &next_dy - &previous_dy * &next_dx;
    Ok(match cross.structural_facts().sign {
        Some(RealSign::Positive) => Some(false),
        Some(RealSign::Negative) => Some(true),
        Some(RealSign::Zero) => {
            let dot = &previous_dx * &next_dx + &previous_dy * &next_dy;
            match dot.structural_facts().sign {
                Some(RealSign::Positive) => None,
                Some(RealSign::Negative) => Some(false),
                Some(RealSign::Zero) | None => {
                    return Err(format!(
                        "vertex {vertex_index} has an indeterminate endpoint tangent"
                    ));
                }
            }
        }
        None => {
            return Err(format!(
                "vertex {vertex_index} has an indeterminate turn orientation"
            ));
        }
    })
}

fn endpoint_tangent(curve: &Curve2, at_start: bool) -> Result<(Real, Real), String> {
    if let CurveGeometry2::CircularArc(arc) = curve.geometry() {
        let endpoint = if at_start { arc.start() } else { arc.end() };
        let (radius_x, radius_y) = endpoint.delta_from(arc.center());
        return Ok(if arc.is_clockwise() {
            (radius_y, -radius_x)
        } else {
            (-radius_y, radius_x)
        });
    }

    let domain = curve.parameter_domain();
    let (parameter, side) = if at_start {
        (domain.start(), CurveParameterSide2::Right)
    } else {
        (domain.end(), CurveParameterSide2::Left)
    };
    let tangent = curve
        .derivative_at_side(parameter, side)
        .map_err(string_error)?;
    Ok((tangent.dx().clone(), tangent.dy().clone()))
}

fn apply_corner_edit(
    path: CurvePath2,
    vertex_index: usize,
    operation: CornerOperation,
    witness: &CornerWitness,
) -> Result<CurvePath2, String> {
    match operation {
        CornerOperation::Fillet => path
            .fillet_vertex_by_parameters(
                vertex_index,
                witness.previous_parameter.clone(),
                witness.next_parameter.clone(),
                &witness.center,
                witness.clockwise,
            )
            .map_err(|error| format!("vertex {vertex_index}: {error}")),
        CornerOperation::Chamfer => path
            .chamfer_vertex_by_parameters(
                vertex_index,
                witness.previous_parameter.clone(),
                witness.next_parameter.clone(),
            )
            .map_err(|error| format!("vertex {vertex_index}: {error}")),
    }
}

fn corner_witness_for(
    source: &CurvePath2,
    vertex_index: usize,
    operation: CornerOperation,
    amount: &Real,
) -> Result<CornerWitness, String> {
    let amount = match operation {
        CornerOperation::Fillet => Real::from(2_i32) * (amount * amount),
        CornerOperation::Chamfer => amount.clone(),
    };
    linear_image_corner_witness(source, vertex_index, operation, &amount)
}

fn linear_image_corner_witness(
    source: &CurvePath2,
    vertex_index: usize,
    operation: CornerOperation,
    amount: &Real,
) -> Result<CornerWitness, String> {
    let previous_index = if vertex_index == 0 {
        source.curves().len() - 1
    } else {
        vertex_index - 1
    };
    let previous = &source.curves()[previous_index];
    let next = &source.curves()[vertex_index];
    let previous_length = chord_length(previous)?;
    let next_length = chord_length(next)?;
    let (previous_dx, previous_dy) = previous.end().delta_from(previous.start());
    let (next_dx, next_dy) = next.end().delta_from(next.start());
    let previous_unit_x = (&previous_dx / &previous_length).map_err(string_error)?;
    let previous_unit_y = (&previous_dy / &previous_length).map_err(string_error)?;
    let next_unit_x = (&next_dx / &next_length).map_err(string_error)?;
    let next_unit_y = (&next_dy / &next_length).map_err(string_error)?;
    let clockwise = corner_orientation(source, vertex_index)?
        .ok_or_else(|| format!("vertex {vertex_index} is smooth"))?;
    let setback = match operation {
        CornerOperation::Chamfer => amount.clone(),
        CornerOperation::Fillet => {
            // For unit tangents enclosing turn angle θ, the tangent setback is
            // r·tan(θ/2) = r·|cross|/(1 + dot).
            let dot = &previous_unit_x * &next_unit_x + &previous_unit_y * &next_unit_y;
            let cross = &previous_unit_x * &next_unit_y - &previous_unit_y * &next_unit_x;
            let positive_cross = if clockwise { -cross } else { cross };
            (amount * positive_cross / (Real::one() + dot)).map_err(string_error)?
        }
    };
    let previous_fraction = (&setback / &previous_length).map_err(string_error)?;
    let next_fraction = (&setback / &next_length).map_err(string_error)?;

    let previous_domain = previous.parameter_domain();
    let previous_span = previous_domain.end() - previous_domain.start();
    let previous_parameter = previous_domain.end() - &(&previous_span * &previous_fraction);
    let next_domain = next.parameter_domain();
    let next_span = next_domain.end() - next_domain.start();
    let next_parameter = next_domain.start() + &(&next_span * &next_fraction);

    let previous_offset_x = &previous_dx * &previous_fraction;
    let previous_offset_y = &previous_dy * &previous_fraction;
    let vertex = next.start();
    let previous_tangent = Point2::new(
        vertex.x() - previous_offset_x,
        vertex.y() - previous_offset_y,
    );
    let (normal_x, normal_y) = if clockwise {
        (previous_unit_y, -previous_unit_x)
    } else {
        (-previous_unit_y, previous_unit_x)
    };
    let center = Point2::new(
        previous_tangent.x() + &(normal_x * amount),
        previous_tangent.y() + &(normal_y * amount),
    );

    Ok(CornerWitness {
        previous_parameter,
        next_parameter,
        center,
        clockwise,
    })
}

fn chord_length(curve: &Curve2) -> Result<Real, String> {
    let (dx, dy) = curve.end().delta_from(curve.start());
    (&dx * &dx + &dy * &dy).sqrt().map_err(string_error)
}

fn curve_region_paths() -> Result<Vec<CurvePath2>, String> {
    let mut paths = Vec::with_capacity(HOLE_COUNT + 1);
    paths.push(outer_boundary()?);
    for template in hole_templates() {
        paths.push(eight_family_hole(template)?);
    }
    Ok(paths)
}

fn outer_boundary() -> Result<CurvePath2, String> {
    // The two quarter arcs have tangent affine neighbors. Every other chord
    // has an integer Pythagorean direction so all 12 corners are certifiable.
    let points = [
        point(-90, -20),
        point(-70, -40),
        point(-40, -40),
        point(0, -70),
        point(30, -30),
        point(70, 0),
        point(70, 10),
        point(100, 50),
        point(100, 80),
        point(80, 100),
        point(50, 100),
        point(10, 70),
        point(-20, 110),
        point(-60, 80),
        point(-60, 60),
        point(-90, 20),
    ];
    let families = [
        CurveFamily2::CircularArc,
        CurveFamily2::Line,
        CurveFamily2::QuadraticBezier,
        CurveFamily2::CubicBezier,
        CurveFamily2::RationalQuadraticBezier,
        CurveFamily2::RationalBezier,
        CurveFamily2::PolynomialBSpline,
        CurveFamily2::Nurbs,
        CurveFamily2::CircularArc,
        CurveFamily2::Line,
        CurveFamily2::QuadraticBezier,
        CurveFamily2::CubicBezier,
        CurveFamily2::RationalQuadraticBezier,
        CurveFamily2::RationalBezier,
        CurveFamily2::PolynomialBSpline,
        CurveFamily2::Nurbs,
    ];
    let mut curves = Vec::with_capacity(points.len());
    for index in 0..points.len() {
        let start = points[index].clone();
        let end = points[(index + 1) % points.len()].clone();
        curves.push(match index {
            0 => circular_arc_curve(start, end, point(-70, -20))?,
            8 => circular_arc_curve(start, end, point(80, 80))?,
            _ => affine_family_curve(families[index], start, end)?,
        });
    }
    CurvePath2::try_new(curves).map_err(string_error)
}

fn eight_family_hole(template: HoleTemplate) -> Result<CurvePath2, String> {
    let points = template
        .points
        .map(|(x, y)| local_point(template.origin, x, y));
    let mut curves = Vec::with_capacity(HOLE_CURVE_COUNT);
    curves.push(circular_arc_curve(
        points[0].clone(),
        points[1].clone(),
        local_point(
            template.origin,
            template.arc_center.0,
            template.arc_center.1,
        ),
    )?);
    for (index, family) in template.families_after_arc.into_iter().enumerate() {
        curves.push(affine_family_curve(
            family,
            points[index + 1].clone(),
            points[(index + 2) % points.len()].clone(),
        )?);
    }
    CurvePath2::try_new(curves).map_err(string_error)
}

fn circular_arc_curve(start: Point2, end: Point2, center: Point2) -> Result<Curve2, String> {
    CircularArc2::try_from_center(start, end, center, false)
        .map(Curve2::from)
        .map_err(string_error)
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

fn interpolate_point(start: &Point2, end: &Point2, parameter: Real) -> Point2 {
    let x = start.x() + &((end.x() - start.x()) * &parameter);
    let y = start.y() + &((end.y() - start.y()) * parameter);
    Point2::new(x, y)
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
    fn corner_region_has_an_irregular_outer_loop_and_four_unique_holes() {
        let scene = CornerScene::new(CornerOperation::Fillet, 1.0);
        let provenance = scene
            .source_region()
            .fragment_provenance()
            .expect("direct CurveRegion2 construction retains provenance");

        assert_eq!(scene.source_region().len(), HOLE_COUNT + 1);
        assert_eq!(scene.source_display.materials.len(), 1);
        assert_eq!(scene.source_display.holes.len(), HOLE_COUNT);
        for family in ALL_FAMILIES {
            assert_eq!(
                provenance
                    .iter()
                    .filter(|fragment| fragment.family() == family)
                    .count(),
                HOLE_COUNT + 2,
                "{family:?} should appear twice outside and once in every hole"
            );
        }

        let paths = curve_region_paths().unwrap();
        let outer = &paths[0];
        assert_eq!(outer.curves().len(), OUTER_CURVE_COUNT);
        for family in ALL_FAMILIES {
            assert_eq!(
                outer
                    .curves()
                    .iter()
                    .filter(|curve| curve.family() == family)
                    .count(),
                2,
                "outer loop should demonstrate {family:?} twice"
            );
        }

        let hole_family_orders = paths[1..]
            .iter()
            .map(|path| {
                assert_eq!(path.curves().len(), HOLE_CURVE_COUNT);
                for family in ALL_FAMILIES {
                    assert_eq!(
                        path.curves()
                            .iter()
                            .filter(|curve| curve.family() == family)
                            .count(),
                        1,
                        "each hole should demonstrate {family:?} exactly once"
                    );
                }
                path.curves().iter().map(Curve2::family).collect::<Vec<_>>()
            })
            .collect::<Vec<_>>();
        for first in 0..hole_family_orders.len() {
            for second in first + 1..hole_family_orders.len() {
                assert_ne!(hole_family_orders[first], hole_family_orders[second]);
            }
        }

        let templates = hole_templates();
        for first in 0..templates.len() {
            for second in first + 1..templates.len() {
                assert_ne!(templates[first].points, templates[second].points);
                assert_ne!(
                    templates[first].families_after_arc,
                    templates[second].families_after_arc
                );
            }
        }

        let outer_turns = (0..outer.curves().len())
            .filter_map(|vertex_index| corner_orientation(outer, vertex_index).unwrap())
            .collect::<Vec<_>>();
        assert!(outer_turns.contains(&false));
        assert!(outer_turns.contains(&true), "outer loop should be concave");
    }

    #[test]
    fn both_corner_tabs_materialize_every_boundary_at_both_slider_extremes() {
        for operation in [CornerOperation::Fillet, CornerOperation::Chamfer] {
            let mut scene = CornerScene::new(operation, 1.0);
            let (minimum, maximum) = operation.amount_bounds();
            for amount in [minimum, maximum] {
                scene.apply_amount(amount).unwrap();
                scene.refresh_result();
                assert!(
                    scene.result_region.is_some(),
                    "{operation:?} at {amount} failed: {:?}",
                    scene.last_error
                );
                assert_eq!(scene.result_region.as_ref().unwrap().len(), HOLE_COUNT + 1);
                assert_eq!(
                    scene.result_display.as_ref().unwrap().holes.len(),
                    HOLE_COUNT
                );
            }
        }
    }

    #[test]
    fn every_non_smooth_corner_is_edited_at_both_slider_extremes() {
        let source_paths = curve_region_paths().unwrap();
        let corner_counts = source_paths
            .iter()
            .enumerate()
            .map(|(boundary_index, path)| {
                (0..path.curves().len())
                    .filter(|index| {
                        corner_orientation(path, *index)
                            .unwrap_or_else(|error| {
                                panic!("boundary {boundary_index} vertex {index}: {error}")
                            })
                            .is_some()
                    })
                    .count()
            })
            .collect::<Vec<_>>();
        assert_eq!(corner_counts.iter().sum::<usize>(), EDITED_CORNER_COUNT);

        for operation in [CornerOperation::Fillet, CornerOperation::Chamfer] {
            let (minimum, maximum) = operation.amount_bounds();
            for slider_amount in [minimum, maximum] {
                let amount = Real::try_from(slider_amount).unwrap();
                for (boundary_index, source) in source_paths.iter().enumerate() {
                    let edited = edit_all_corners(source, operation, amount.clone())
                        .unwrap_or_else(|error| {
                            panic!(
                                "{operation:?} boundary {boundary_index} at {slider_amount} failed: {error}"
                            )
                        });
                    assert_eq!(
                        edited.curves().len(),
                        source.curves().len() + corner_counts[boundary_index],
                        "{operation:?} should add one curve at every non-smooth corner"
                    );
                }
            }
        }
    }

    #[test]
    fn only_the_intended_arc_joins_are_smooth() {
        let paths = curve_region_paths().unwrap();
        for (boundary_index, path) in paths.iter().enumerate() {
            let expected_smooth = if boundary_index == 0 {
                vec![0, 1, 8, 9]
            } else {
                vec![0, 1]
            };
            for vertex_index in 0..path.curves().len() {
                let is_smooth = corner_orientation(path, vertex_index)
                    .unwrap_or_else(|error| {
                        panic!("boundary {boundary_index} vertex {vertex_index}: {error}")
                    })
                    .is_none();
                assert_eq!(
                    is_smooth,
                    expected_smooth.contains(&vertex_index),
                    "unexpected smooth/corner classification at boundary {boundary_index} vertex {vertex_index}"
                );
            }
        }
    }
}
