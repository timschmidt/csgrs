use egui::{CentralPanel, ScrollArea, SidePanel, Slider};
use egui_plot::{Plot, PlotPoint, Text};
use hypercurve::{
    CircularArc2, CubicBezier2, Curve2, CurveFamily2, CurveParameterSide2, CurvePath2,
    CurveRegion2, LineSeg2, Point2, QuadraticBezier2, RationalBezier2, RationalQuadraticBezier2,
    Real, RealSign,
};

use crate::geometry::{Polyline, Shape};
use crate::plotting::draw_shape;
use crate::theme::Theme;

const DISPLAY_STEPS: usize = 48;
const RECTANGLE_HOLE_COUNT: usize = 12;
const ARC_HOLE_COUNT: usize = 2;
const EDITED_CORNER_COUNT: usize = 4 + (RECTANGLE_HOLE_COUNT + ARC_HOLE_COUNT) * 4;
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
enum HoleKind {
    Rectangle,
    ArcThenFamily,
    FamilyThenArc,
}

#[derive(Clone, Copy, Debug)]
struct HoleSpec {
    origin: (i32, i32),
    families: [CurveFamily2; 4],
    kind: HoleKind,
}

const HOLE_SPECS: [HoleSpec; 14] = [
    rectangle(
        (-24, -11),
        CurveFamily2::Line,
        CurveFamily2::QuadraticBezier,
        CurveFamily2::CubicBezier,
        CurveFamily2::RationalQuadraticBezier,
    ),
    rectangle(
        (-14, -11),
        CurveFamily2::QuadraticBezier,
        CurveFamily2::CubicBezier,
        CurveFamily2::RationalQuadraticBezier,
        CurveFamily2::RationalBezier,
    ),
    rectangle(
        (-4, -11),
        CurveFamily2::CubicBezier,
        CurveFamily2::RationalQuadraticBezier,
        CurveFamily2::RationalBezier,
        CurveFamily2::PolynomialBSpline,
    ),
    rectangle(
        (6, -11),
        CurveFamily2::RationalQuadraticBezier,
        CurveFamily2::RationalBezier,
        CurveFamily2::PolynomialBSpline,
        CurveFamily2::Nurbs,
    ),
    rectangle(
        (16, -11),
        CurveFamily2::RationalBezier,
        CurveFamily2::PolynomialBSpline,
        CurveFamily2::Nurbs,
        CurveFamily2::Line,
    ),
    rectangle(
        (-24, -2),
        CurveFamily2::PolynomialBSpline,
        CurveFamily2::Nurbs,
        CurveFamily2::Line,
        CurveFamily2::QuadraticBezier,
    ),
    rectangle(
        (-14, -2),
        CurveFamily2::Nurbs,
        CurveFamily2::Line,
        CurveFamily2::QuadraticBezier,
        CurveFamily2::CubicBezier,
    ),
    rectangle(
        (-4, -2),
        CurveFamily2::Line,
        CurveFamily2::CubicBezier,
        CurveFamily2::Nurbs,
        CurveFamily2::RationalBezier,
    ),
    rectangle(
        (6, -2),
        CurveFamily2::QuadraticBezier,
        CurveFamily2::RationalBezier,
        CurveFamily2::RationalQuadraticBezier,
        CurveFamily2::PolynomialBSpline,
    ),
    rectangle(
        (16, -2),
        CurveFamily2::CubicBezier,
        CurveFamily2::PolynomialBSpline,
        CurveFamily2::RationalBezier,
        CurveFamily2::Nurbs,
    ),
    rectangle(
        (-24, 7),
        CurveFamily2::RationalQuadraticBezier,
        CurveFamily2::Nurbs,
        CurveFamily2::Line,
        CurveFamily2::CubicBezier,
    ),
    rectangle(
        (-14, 7),
        CurveFamily2::Nurbs,
        CurveFamily2::QuadraticBezier,
        CurveFamily2::PolynomialBSpline,
        CurveFamily2::RationalQuadraticBezier,
    ),
    arc_hole(
        (-4, 7),
        HoleKind::ArcThenFamily,
        CurveFamily2::Nurbs,
        CurveFamily2::QuadraticBezier,
        CurveFamily2::CubicBezier,
        CurveFamily2::RationalQuadraticBezier,
    ),
    arc_hole(
        (6, 7),
        HoleKind::FamilyThenArc,
        CurveFamily2::RationalBezier,
        CurveFamily2::PolynomialBSpline,
        CurveFamily2::Line,
        CurveFamily2::CubicBezier,
    ),
];

const fn rectangle(
    origin: (i32, i32),
    bottom: CurveFamily2,
    right: CurveFamily2,
    top: CurveFamily2,
    left: CurveFamily2,
) -> HoleSpec {
    HoleSpec {
        origin,
        families: [bottom, right, top, left],
        kind: HoleKind::Rectangle,
    }
}

const fn arc_hole(
    origin: (i32, i32),
    kind: HoleKind,
    first: CurveFamily2,
    second: CurveFamily2,
    third: CurveFamily2,
    fourth: CurveFamily2,
) -> HoleSpec {
    HoleSpec {
        origin,
        families: [first, second, third, fourth],
        kind,
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
                    // The UI value is the rational arc-tangency parameter q;
                    // the visible radius is the exact construction 2q².
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
                    "CurveRegion2: {} boundaries · 1 material + {RECTANGLE_HOLE_COUNT} mixed-family polygons + {ARC_HOLE_COUNT} arc-corner holes",
                    self.source_region.len(),
                ));
                ui.small(
                    "Every adjustment edits every corner, including both circular-arc/family orders.",
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
            let kind = if boundary_index == 0 {
                HoleKind::Rectangle
            } else {
                HOLE_SPECS[boundary_index - 1].kind
            };
            edit_all_corners(path, kind, operation, amount.clone())
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
    kind: HoleKind,
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
            let witness = corner_witness_for(&result, *vertex_index, kind, operation, &amount)?;
            result = apply_corner_edit(result, *vertex_index, operation, &witness)?;
        }
    }
    if corner_indices.first() == Some(&0) {
        let seam_witness = corner_witness_for(&result, 0, kind, operation, &amount)?;
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
    let previous_tangent = previous
        .derivative_at_side(previous.parameter_domain().end(), CurveParameterSide2::Left)
        .map_err(string_error)?;
    let next_tangent = next
        .derivative_at_side(next.parameter_domain().start(), CurveParameterSide2::Right)
        .map_err(string_error)?;
    let cross =
        previous_tangent.dx() * next_tangent.dy() - previous_tangent.dy() * next_tangent.dx();
    Ok(match cross.structural_facts().sign {
        Some(RealSign::Positive) => Some(false),
        Some(RealSign::Negative) => Some(true),
        Some(RealSign::Zero) => {
            let dot = previous_tangent.dx() * next_tangent.dx()
                + previous_tangent.dy() * next_tangent.dy();
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
    kind: HoleKind,
    operation: CornerOperation,
    amount: &Real,
) -> Result<CornerWitness, String> {
    let special_vertex = match kind {
        HoleKind::ArcThenFamily => Some(1),
        HoleKind::FamilyThenArc => Some(4),
        HoleKind::Rectangle => None,
    };
    if special_vertex == Some(vertex_index) {
        return arc_family_corner_witness(source, vertex_index, kind, operation, amount);
    }
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

fn arc_family_corner_witness(
    source: &CurvePath2,
    vertex_index: usize,
    kind: HoleKind,
    operation: CornerOperation,
    amount: &Real,
) -> Result<CornerWitness, String> {
    let q = match operation {
        CornerOperation::Fillet => amount.clone(),
        CornerOperation::Chamfer => (amount / Real::from(4_i32)).map_err(string_error)?,
    };
    let line_distance = Real::from(4_i32) * &q;
    let maximum_q = rational(3, 4);
    let angular_half_tangent =
        ((&maximum_q - &q) / (Real::one() + &maximum_q * &q)).map_err(string_error)?;
    let arc_parameter = (&angular_half_tangent
        / (rational(3, 5) + rational(1, 5) * &angular_half_tangent))
        .map_err(string_error)?;
    let radius = Real::from(2_i32) * (&q * &q);
    let vertex = source.curves()[vertex_index].start();
    let center = Point2::new(vertex.x() - &line_distance, vertex.y() + radius);

    match kind {
        HoleKind::ArcThenFamily => {
            let next = &source.curves()[vertex_index];
            let next_fraction = (&line_distance / chord_length(next)?).map_err(string_error)?;
            let next_domain = next.parameter_domain();
            let next_parameter =
                next_domain.start() + &((next_domain.end() - next_domain.start()) * next_fraction);
            Ok(CornerWitness {
                previous_parameter: arc_parameter,
                next_parameter,
                center,
                clockwise: true,
            })
        }
        HoleKind::FamilyThenArc => {
            let previous = &source.curves()[vertex_index - 1];
            let previous_fraction =
                (&line_distance / chord_length(previous)?).map_err(string_error)?;
            let previous_domain = previous.parameter_domain();
            let previous_parameter = previous_domain.end()
                - &((previous_domain.end() - previous_domain.start()) * previous_fraction);
            Ok(CornerWitness {
                previous_parameter,
                next_parameter: Real::one() - arc_parameter,
                center,
                clockwise: false,
            })
        }
        HoleKind::Rectangle => Err("rectangle has no circular-arc corner witness".into()),
    }
}

fn chord_length(curve: &Curve2) -> Result<Real, String> {
    let (dx, dy) = curve.end().delta_from(curve.start());
    (&dx * &dx + &dy * &dy).sqrt().map_err(string_error)
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
    match spec.kind {
        HoleKind::Rectangle => rectangle_hole(spec),
        HoleKind::ArcThenFamily => arc_then_family_hole(spec),
        HoleKind::FamilyThenArc => arc_then_family_hole(spec)?.reversed().map_err(string_error),
    }
}

fn rectangle_hole(spec: HoleSpec) -> Result<CurvePath2, String> {
    let points = [
        local_point(spec.origin, -4, 0),
        local_point(spec.origin, 0, 0),
        local_point(spec.origin, 0, 4),
        local_point(spec.origin, -4, 4),
    ];
    let mut curves = Vec::with_capacity(points.len());
    for index in 0..points.len() {
        curves.push(affine_family_curve(
            spec.families[index],
            points[index].clone(),
            points[(index + 1) % points.len()].clone(),
        )?);
    }
    CurvePath2::try_new(curves).map_err(string_error)
}

fn arc_then_family_hole(spec: HoleSpec) -> Result<CurvePath2, String> {
    let arc_start = local_real_point(spec.origin, -rational(48, 25), rational(36, 25));
    let corner = local_point(spec.origin, 0, 0);
    let arc = CircularArc2::try_from_center(
        arc_start.clone(),
        corner.clone(),
        local_point(spec.origin, 0, 2),
        false,
    )
    .map_err(string_error)?;
    let points = [
        corner,
        local_point(spec.origin, -8, 0),
        local_real_point(spec.origin, Real::from(-8_i32), rational(39, 10)),
        local_real_point(spec.origin, -rational(18, 5), rational(36, 5)),
        arc_start,
    ];
    let mut curves = vec![Curve2::from(arc)];
    for index in 0..spec.families.len() {
        curves.push(affine_family_curve(
            spec.families[index],
            points[index].clone(),
            points[index + 1].clone(),
        )?);
    }
    CurvePath2::try_new(curves).map_err(string_error)
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

    #[test]
    fn every_family_participates_in_a_corner_and_both_operations_edit_them_all() {
        let source_paths = curve_region_paths().unwrap();
        let corner_counts = source_paths
            .iter()
            .map(|path| {
                (0..path.curves().len())
                    .filter(|index| corner_orientation(path, *index).unwrap().is_some())
                    .count()
            })
            .collect::<Vec<_>>();
        assert_eq!(corner_counts.iter().sum::<usize>(), EDITED_CORNER_COUNT);
        assert_eq!(
            (
                source_paths[RECTANGLE_HOLE_COUNT + 1].curves()[0].family(),
                source_paths[RECTANGLE_HOLE_COUNT + 1].curves()[1].family(),
            ),
            (CurveFamily2::CircularArc, CurveFamily2::Nurbs)
        );
        assert_eq!(
            (
                source_paths[RECTANGLE_HOLE_COUNT + 2].curves()[3].family(),
                source_paths[RECTANGLE_HOLE_COUNT + 2].curves()[4].family(),
            ),
            (CurveFamily2::RationalBezier, CurveFamily2::CircularArc)
        );

        for family in ALL_FAMILIES {
            assert!(source_paths.iter().any(|path| {
                (0..path.curves().len()).any(|vertex_index| {
                    if corner_orientation(path, vertex_index).unwrap().is_none() {
                        return false;
                    }
                    let previous_index = if vertex_index == 0 {
                        path.curves().len() - 1
                    } else {
                        vertex_index - 1
                    };
                    path.curves()[previous_index].family() == family
                        || path.curves()[vertex_index].family() == family
                })
            }));
        }

        for operation in [CornerOperation::Fillet, CornerOperation::Chamfer] {
            let amount = match operation {
                CornerOperation::Fillet => rational(1, 2),
                CornerOperation::Chamfer => Real::one(),
            };
            for (boundary_index, source) in source_paths.iter().enumerate() {
                let kind = if boundary_index == 0 {
                    HoleKind::Rectangle
                } else {
                    HOLE_SPECS[boundary_index - 1].kind
                };
                let edited = edit_all_corners(source, kind, operation, amount.clone()).unwrap();
                assert_eq!(
                    edited.curves().len(),
                    source.curves().len() + corner_counts[boundary_index]
                );
            }
        }
    }
}
