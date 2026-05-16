use egui::{CentralPanel, Color32, RichText, ScrollArea, SidePanel, Stroke, TopBottomPanel};
use egui_plot::{Line, Plot, PlotPoint, PlotPoints, PlotUi, Points, Text};
use hypertri::{Constraint, QualityPolicy, Triangle};

pub struct MainApp {
    active: Scene,
    polygon: PolygonScene,
    points: PointScene,
    constraints: ConstraintScene,
    compare: CompareScene,
    view: ViewOptions,
}

impl MainApp {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        cc.egui_ctx.style_mut(|style| {
            for font_id in style.text_styles.values_mut() {
                font_id.size += 1.0;
            }
        });
        Self {
            active: Scene::Polygon,
            polygon: PolygonScene::default(),
            points: PointScene::default(),
            constraints: ConstraintScene::default(),
            compare: CompareScene::default(),
            view: ViewOptions::default(),
        }
    }
}

impl eframe::App for MainApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        TopBottomPanel::top("scene_tabs").show(ctx, |ui| {
            ui.horizontal_wrapped(|ui| {
                ui.selectable_value(&mut self.active, Scene::Polygon, "Earcut Polygon");
                ui.selectable_value(&mut self.active, Scene::Points, "Delaunay Points");
                ui.selectable_value(&mut self.active, Scene::Constraints, "Constrained CDT");
                ui.selectable_value(&mut self.active, Scene::Compare, "Runtime Compare");
                ui.separator();
                ui.hyperlink_to("GitHub", "https://github.com/timschmidt/hypertri");
            });
        });

        SidePanel::right("controls")
            .default_width(260.0)
            .show(ctx, |ui| {
                ScrollArea::vertical().show(ui, |ui| {
                    self.view.ui(ui);
                    ui.separator();
                    match self.active {
                        Scene::Polygon => self.polygon.controls(ui),
                        Scene::Points => self.points.controls(ui),
                        Scene::Constraints => self.constraints.controls(ui),
                        Scene::Compare => self.compare.controls(ui),
                    }
                });
            });

        CentralPanel::default().show(ctx, |ui| {
            Plot::new("hypertri_plot")
                .data_aspect(1.0)
                .allow_drag(true)
                .allow_zoom(true)
                .show(ui, |plot_ui| match self.active {
                    Scene::Polygon => self.polygon.draw(plot_ui, &self.view),
                    Scene::Points => self.points.draw(plot_ui, &self.view),
                    Scene::Constraints => self.constraints.draw(plot_ui, &self.view),
                    Scene::Compare => self.compare.draw(plot_ui, &self.view),
                });
        });
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Scene {
    Polygon,
    Points,
    Constraints,
    Compare,
}

struct ViewOptions {
    show_input: bool,
    show_triangles: bool,
    show_vertices: bool,
    show_constraints: bool,
    show_indices: bool,
}

impl Default for ViewOptions {
    fn default() -> Self {
        Self {
            show_input: true,
            show_triangles: true,
            show_vertices: true,
            show_constraints: true,
            show_indices: false,
        }
    }
}

impl ViewOptions {
    fn ui(&mut self, ui: &mut egui::Ui) {
        ui.heading("Display");
        ui.checkbox(&mut self.show_input, "Input rings");
        ui.checkbox(&mut self.show_triangles, "Triangles");
        ui.checkbox(&mut self.show_vertices, "Vertices");
        ui.checkbox(&mut self.show_constraints, "Constraints");
        ui.checkbox(&mut self.show_indices, "Indices");
    }
}

struct PolygonScene {
    case: PolygonCase,
}

impl Default for PolygonScene {
    fn default() -> Self {
        Self {
            case: PolygonCase::Holed,
        }
    }
}

impl PolygonScene {
    fn controls(&mut self, ui: &mut egui::Ui) {
        ui.heading("Earcut Polygon");
        case_radio(ui, &mut self.case, PolygonCase::Concave, "Concave");
        case_radio(ui, &mut self.case, PolygonCase::Holed, "Holed");
        case_radio(
            ui,
            &mut self.case,
            PolygonCase::Adversarial,
            "Near collinear",
        );
        let input = self.case.input();
        let facts = polygon_facts(input.vertices.len(), input.holes.len());
        ui.separator();
        ui.label(facts);
    }

    fn draw(&self, plot_ui: &mut PlotUi<'_>, view: &ViewOptions) {
        let input = self.case.input();
        if view.show_input {
            draw_rings(plot_ui, &input.vertices, &input.holes);
        }
        match hypertri::f64::earcut(&input.vertices, &input.holes) {
            Ok(indices) => {
                if view.show_triangles {
                    draw_index_triangles(plot_ui, "earcut", &input.vertices, &indices, RESULT);
                }
                if view.show_vertices {
                    draw_points(plot_ui, "vertices", &input.vertices, VERTEX);
                }
                if view.show_indices {
                    draw_labels(plot_ui, &input.vertices);
                }
                draw_status(
                    plot_ui,
                    "earcut ok",
                    indices.len() / 3,
                    input.vertices.len(),
                );
            }
            Err(error) => draw_error(plot_ui, &format!("{error}")),
        }
    }
}

struct PointScene {
    case: PointCase,
}

impl Default for PointScene {
    fn default() -> Self {
        Self {
            case: PointCase::Cloud,
        }
    }
}

impl PointScene {
    fn controls(&mut self, ui: &mut egui::Ui) {
        ui.heading("Delaunay Points");
        case_radio(ui, &mut self.case, PointCase::Cloud, "Point cloud");
        case_radio(ui, &mut self.case, PointCase::Grid, "Perturbed grid");
        case_radio(
            ui,
            &mut self.case,
            PointCase::Degenerate,
            "Cospherical stress",
        );
    }

    fn draw(&self, plot_ui: &mut PlotUi<'_>, view: &ViewOptions) {
        let points = self.case.points();
        match hypertri::f64::delaunay(&points) {
            Ok(triangulation) => {
                if view.show_triangles {
                    draw_triangles(
                        plot_ui,
                        "delaunay",
                        &points,
                        triangulation.triangles(),
                        RESULT,
                    );
                }
                if view.show_vertices {
                    draw_points(plot_ui, "points", &points, VERTEX);
                }
                if view.show_indices {
                    draw_labels(plot_ui, &points);
                }
                draw_status(
                    plot_ui,
                    "delaunay ok",
                    triangulation.triangles().len(),
                    points.len(),
                );
            }
            Err(error) => draw_error(plot_ui, &format!("{error}")),
        }
    }
}

struct ConstraintScene {
    case: ConstraintCase,
}

impl Default for ConstraintScene {
    fn default() -> Self {
        Self {
            case: ConstraintCase::BoxWithHole,
        }
    }
}

impl ConstraintScene {
    fn controls(&mut self, ui: &mut egui::Ui) {
        ui.heading("Constrained CDT");
        case_radio(
            ui,
            &mut self.case,
            ConstraintCase::BoxWithHole,
            "Box with hole",
        );
        case_radio(
            ui,
            &mut self.case,
            ConstraintCase::Crossing,
            "Crossing constraints",
        );
        case_radio(ui, &mut self.case, ConstraintCase::OpenPslg, "Open PSLG");
    }

    fn draw(&self, plot_ui: &mut PlotUi<'_>, view: &ViewOptions) {
        let input = self.case.input();
        match hypertri::f64::constrained_delaunay(&input.points, &input.constraints) {
            Ok(triangulation) => {
                let points = approx_points(triangulation.points());
                if view.show_triangles {
                    draw_triangles(
                        plot_ui,
                        "constrained",
                        &points,
                        triangulation.triangles(),
                        RESULT,
                    );
                }
                if view.show_constraints {
                    draw_constraints(plot_ui, "constraints", &points, &input.constraints, PRIMARY);
                    draw_constraints(
                        plot_ui,
                        "protected subsegments",
                        &points,
                        triangulation.constraint_edges(),
                        WARNING,
                    );
                }
                if view.show_vertices {
                    draw_points(plot_ui, "vertices", &points, VERTEX);
                }
                if view.show_indices {
                    draw_labels(plot_ui, &points);
                }
                draw_status(
                    plot_ui,
                    "cdt ok",
                    triangulation.triangles().len(),
                    triangulation.points().len(),
                );
            }
            Err(error) => draw_error(plot_ui, &format!("{error}")),
        }
    }
}

struct CompareScene {
    quality: QualityPolicy,
}

impl Default for CompareScene {
    fn default() -> Self {
        Self {
            quality: QualityPolicy::PreserveBoundary,
        }
    }
}

impl CompareScene {
    fn controls(&mut self, ui: &mut egui::Ui) {
        ui.heading("Runtime Compare");
        ui.radio_value(
            &mut self.quality,
            QualityPolicy::PreserveBoundary,
            "Preserve boundary",
        );
        ui.radio_value(
            &mut self.quality,
            QualityPolicy::PreferDelaunay,
            "Prefer Delaunay",
        );
        ui.separator();
        ui.label("Shows earcut and runtime selected triangulations over the same holed polygon.");
    }

    fn draw(&self, plot_ui: &mut PlotUi<'_>, view: &ViewOptions) {
        let input = PolygonCase::Holed.input();
        if view.show_input {
            draw_rings(plot_ui, &input.vertices, &input.holes);
        }
        if let Ok(earcut) = hypertri::f64::earcut(&input.vertices, &input.holes) {
            draw_index_triangles(
                plot_ui,
                "earcut baseline",
                &input.vertices,
                &earcut,
                SECONDARY,
            );
        }

        let exact = lift_points(&input.vertices);
        let polygon = hypertri::PolygonInput::new(exact, input.holes.clone());
        let options = hypertri::TriangulationOptions {
            algorithm: hypertri::PolygonTriangulationAlgorithm::Auto,
            quality: self.quality,
        };
        match hypertri::triangulate_polygon(&polygon, options) {
            Ok(indices) => {
                if view.show_triangles {
                    draw_index_triangles(plot_ui, "runtime", &input.vertices, &indices, RESULT);
                }
                if view.show_vertices {
                    draw_points(plot_ui, "vertices", &input.vertices, VERTEX);
                }
                draw_status(
                    plot_ui,
                    "runtime ok",
                    indices.len() / 3,
                    input.vertices.len(),
                );
            }
            Err(error) => draw_error(plot_ui, &format!("{error}")),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum PolygonCase {
    Concave,
    Holed,
    Adversarial,
}

impl PolygonCase {
    fn input(self) -> PolygonInput {
        match self {
            Self::Concave => PolygonInput {
                vertices: vec![
                    [-5.0, -3.0],
                    [2.0, -3.5],
                    [5.0, -1.0],
                    [1.5, 0.2],
                    [4.5, 3.5],
                    [-1.0, 2.4],
                    [-4.0, 4.0],
                    [-2.5, 0.4],
                ],
                holes: vec![],
            },
            Self::Holed => PolygonInput {
                vertices: vec![
                    [-6.0, -4.0],
                    [6.0, -4.0],
                    [6.0, 4.0],
                    [-6.0, 4.0],
                    [-2.0, -1.5],
                    [-0.2, 1.8],
                    [2.2, -1.1],
                ],
                holes: vec![4],
            },
            Self::Adversarial => PolygonInput {
                vertices: vec![
                    [-6.0, -2.0],
                    [-2.0, -2.0000000001],
                    [1.0, -2.0],
                    [6.0, -2.0],
                    [5.0, 2.5],
                    [0.5, 3.0],
                    [-3.0, 2.7],
                    [-5.0, 0.1],
                ],
                holes: vec![],
            },
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum PointCase {
    Cloud,
    Grid,
    Degenerate,
}

impl PointCase {
    fn points(self) -> Vec<[f64; 2]> {
        match self {
            Self::Cloud => vec![
                [-5.4, -2.1],
                [-3.1, 1.2],
                [-2.2, -3.4],
                [-0.4, 3.3],
                [1.5, -2.7],
                [3.1, 2.8],
                [4.5, -0.4],
                [0.2, 0.0],
                [-4.8, 3.4],
                [5.2, 3.8],
            ],
            Self::Grid => (-2_i32..=2)
                .flat_map(|x| {
                    (-2_i32..=2).map(move |y| {
                        let nudge = f64::from((x * 7 + y * 11).rem_euclid(5)) * 0.07;
                        [f64::from(x) * 2.0 + nudge, f64::from(y) * 1.6 - nudge]
                    })
                })
                .collect(),
            Self::Degenerate => (0..12)
                .map(|i| {
                    let angle = f64::from(i) * std::f64::consts::TAU / 12.0;
                    [angle.cos() * 4.0, angle.sin() * 4.0]
                })
                .chain([[0.0, 0.0], [1.0, 0.0]])
                .collect(),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum ConstraintCase {
    BoxWithHole,
    Crossing,
    OpenPslg,
}

impl ConstraintCase {
    fn input(self) -> ConstraintInput {
        match self {
            Self::BoxWithHole => ConstraintInput {
                points: vec![
                    [-6.0, -4.0],
                    [6.0, -4.0],
                    [6.0, 4.0],
                    [-6.0, 4.0],
                    [-1.7, -1.4],
                    [1.8, -1.1],
                    [0.2, 1.9],
                ],
                constraints: ring_constraints(0, 4)
                    .into_iter()
                    .chain(ring_constraints(4, 3))
                    .collect(),
            },
            Self::Crossing => ConstraintInput {
                points: vec![
                    [-5.0, -3.0],
                    [5.0, 3.0],
                    [-5.0, 3.0],
                    [5.0, -3.0],
                    [0.0, 4.0],
                    [0.0, -4.0],
                ],
                constraints: vec![
                    Constraint::new(0, 1),
                    Constraint::new(2, 3),
                    Constraint::new(4, 5),
                ],
            },
            Self::OpenPslg => ConstraintInput {
                points: vec![
                    [-5.0, -3.0],
                    [-2.0, 1.0],
                    [0.5, -2.0],
                    [2.0, 2.2],
                    [5.0, -1.0],
                    [-1.0, 4.0],
                    [3.8, 4.0],
                ],
                constraints: vec![
                    Constraint::new(0, 1),
                    Constraint::new(1, 2),
                    Constraint::new(2, 3),
                    Constraint::new(3, 4),
                    Constraint::new(1, 5),
                    Constraint::new(3, 6),
                ],
            },
        }
    }
}

struct PolygonInput {
    vertices: Vec<[f64; 2]>,
    holes: Vec<usize>,
}

struct ConstraintInput {
    points: Vec<[f64; 2]>,
    constraints: Vec<Constraint>,
}

fn ring_constraints(start: usize, len: usize) -> Vec<Constraint> {
    (0..len)
        .map(|i| Constraint::new(start + i, start + ((i + 1) % len)))
        .collect()
}

fn case_radio<T: Copy + PartialEq>(ui: &mut egui::Ui, value: &mut T, case: T, label: &str) {
    ui.radio_value(value, case, label);
}

fn lift_points(points: &[[f64; 2]]) -> Vec<hypertri::ExactPoint> {
    points
        .iter()
        .map(|point| {
            hypertri::Point2::new(
                hypertri::Real::try_from(point[0]).unwrap(),
                hypertri::Real::try_from(point[1]).unwrap(),
            )
        })
        .collect()
}

fn approx_points(points: &[hypertri::ExactPoint]) -> Vec<[f64; 2]> {
    points
        .iter()
        .map(|point| {
            [
                point.x.to_f64_approx().unwrap_or(0.0),
                point.y.to_f64_approx().unwrap_or(0.0),
            ]
        })
        .collect()
}

fn draw_rings(plot_ui: &mut PlotUi<'_>, points: &[[f64; 2]], holes: &[usize]) {
    let mut starts = vec![0];
    starts.extend_from_slice(holes);
    for (ring_index, start) in starts.iter().copied().enumerate() {
        let end = starts.get(ring_index + 1).copied().unwrap_or(points.len());
        let mut ring = points[start..end].to_vec();
        if let Some(first) = ring.first().copied() {
            ring.push(first);
        }
        let color = if ring_index == 0 { PRIMARY } else { SECONDARY };
        plot_ui.line(Line::new(format!("ring {ring_index}"), PlotPoints::from(ring)).color(color));
    }
}

fn draw_index_triangles(
    plot_ui: &mut PlotUi<'_>,
    name: &str,
    points: &[[f64; 2]],
    indices: &[usize],
    color: Color32,
) {
    for (index, tri) in indices.chunks_exact(3).enumerate() {
        let triangle = [tri[0], tri[1], tri[2]];
        draw_triangle(plot_ui, format!("{name} {index}"), points, triangle, color);
    }
}

fn draw_triangles(
    plot_ui: &mut PlotUi<'_>,
    name: &str,
    points: &[[f64; 2]],
    triangles: &[Triangle],
    color: Color32,
) {
    for (index, &triangle) in triangles.iter().enumerate() {
        draw_triangle(plot_ui, format!("{name} {index}"), points, triangle, color);
    }
}

fn draw_triangle(
    plot_ui: &mut PlotUi<'_>,
    name: String,
    points: &[[f64; 2]],
    triangle: Triangle,
    color: Color32,
) {
    if triangle.iter().any(|&index| index >= points.len()) {
        return;
    }
    let mut series = triangle
        .iter()
        .map(|&index| points[index])
        .collect::<Vec<_>>();
    series.push(points[triangle[0]]);
    plot_ui.line(
        Line::new(name, PlotPoints::from(series))
            .color(color)
            .stroke(Stroke::new(1.5, color)),
    );
}

fn draw_constraints(
    plot_ui: &mut PlotUi<'_>,
    name: &str,
    points: &[[f64; 2]],
    constraints: &[Constraint],
    color: Color32,
) {
    for (index, constraint) in constraints.iter().enumerate() {
        if constraint.from >= points.len() || constraint.to >= points.len() {
            continue;
        }
        plot_ui.line(
            Line::new(
                format!("{name} {index}"),
                PlotPoints::from(vec![points[constraint.from], points[constraint.to]]),
            )
            .color(color)
            .stroke(Stroke::new(3.0, color)),
        );
    }
}

fn draw_points(plot_ui: &mut PlotUi<'_>, name: &str, points: &[[f64; 2]], color: Color32) {
    plot_ui.points(
        Points::new(name, PlotPoints::from(points.to_vec()))
            .radius(4.5)
            .color(color),
    );
}

fn draw_labels(plot_ui: &mut PlotUi<'_>, points: &[[f64; 2]]) {
    for (index, point) in points.iter().enumerate() {
        plot_ui.text(
            Text::new(
                format!("label {index}"),
                PlotPoint::new(point[0] + 0.12, point[1] + 0.12),
                RichText::new(index.to_string()).color(LABEL),
            )
            .anchor(egui::Align2::LEFT_BOTTOM),
        );
    }
}

fn draw_status(plot_ui: &mut PlotUi<'_>, label: &str, triangles: usize, vertices: usize) {
    plot_ui.text(Text::new(
        "status",
        PlotPoint::new(-7.0, 5.0),
        RichText::new(format!(
            "{label}: {triangles} triangles, {vertices} vertices"
        ))
        .color(LABEL),
    ));
}

fn draw_error(plot_ui: &mut PlotUi<'_>, error: &str) {
    plot_ui.text(Text::new(
        "error",
        PlotPoint::new(-7.0, 5.0),
        RichText::new(error).color(ERROR),
    ));
}

fn polygon_facts(vertices: usize, holes: usize) -> String {
    let rings = holes + 1;
    format!("{vertices} vertices, {rings} rings, {holes} holes")
}

const PRIMARY: Color32 = Color32::from_rgb(74, 163, 255);
const SECONDARY: Color32 = Color32::from_rgb(232, 98, 132);
const RESULT: Color32 = Color32::from_rgb(93, 202, 128);
const WARNING: Color32 = Color32::from_rgb(238, 191, 73);
const ERROR: Color32 = Color32::from_rgb(255, 93, 93);
const VERTEX: Color32 = Color32::from_rgb(247, 230, 121);
const LABEL: Color32 = Color32::from_rgb(214, 221, 232);
