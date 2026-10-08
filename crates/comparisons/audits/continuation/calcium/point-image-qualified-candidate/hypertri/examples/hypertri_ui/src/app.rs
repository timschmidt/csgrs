// Shared-state helpers are active in the browser build and compiled on the host
// so their serialization code remains testable.
#![cfg_attr(not(target_arch = "wasm32"), allow(dead_code))]

use egui::{CentralPanel, Color32, RichText, ScrollArea, SidePanel, Stroke, TopBottomPanel};
use egui_plot::{Line, Plot, PlotPoint, PlotPoints, PlotUi, Points, Text};
use hypertri::{
    Constraint, PredicatePolicy, QualityPolicy, Triangle, TriangulationContext,
};
use serde::{Deserialize, Serialize};

const APPROX: TriangulationContext =
    TriangulationContext::new(PredicatePolicy::APPROXIMATE_512);

pub struct MainApp {
    active: Scene,
    polygon: PolygonScene,
    points: PointScene,
    constraints: ConstraintScene,
    compare: CompareScene,
    view: ViewOptions,
    #[cfg(target_arch = "wasm32")]
    share_status: Option<String>,
}

impl MainApp {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        cc.egui_ctx.style_mut(|style| {
            for font_id in style.text_styles.values_mut() {
                font_id.size += 1.0;
            }
        });
        Self::load_or_default()
    }

    fn load_or_default() -> Self {
        #[cfg(target_arch = "wasm32")]
        {
            match crate::share::load_from_location::<MainAppState>() {
                Ok(Some(state)) => match Self::from_state(state) {
                    Ok(app) => return app,
                    Err(error) => log::warn!("ignoring invalid shared hypertri UI state: {error}"),
                },
                Ok(None) => {}
                Err(error) => log::warn!("ignoring invalid shared hypertri UI state: {error}"),
            }
        }

        Self::default()
    }

    fn from_state(state: MainAppState) -> Result<Self, String> {
        if state.version != 1 {
            return Err(format!("unsupported state version {}", state.version));
        }
        let mut app = Self {
            active: state.active,
            view: state.view,
            ..Self::default()
        };
        app.polygon.apply_state(state.polygon)?;
        app.points.apply_state(state.points)?;
        app.constraints.apply_state(state.constraints)?;
        app.compare.apply_state(state.compare);
        Ok(app)
    }

    fn state(&self) -> MainAppState {
        MainAppState {
            version: 1,
            active: self.active,
            polygon: self.polygon.state(),
            points: self.points.state(),
            constraints: self.constraints.state(),
            compare: self.compare.state(),
            view: self.view.clone(),
        }
    }
}

impl Default for MainApp {
    fn default() -> Self {
        Self {
            active: Scene::Polygon,
            polygon: PolygonScene::default(),
            points: PointScene::default(),
            constraints: ConstraintScene::default(),
            compare: CompareScene::default(),
            view: ViewOptions::default(),
            #[cfg(target_arch = "wasm32")]
            share_status: None,
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
                #[cfg(target_arch = "wasm32")]
                {
                    if ui
                        .button("Share")
                        .on_hover_text("Copy a URL for this demo state")
                        .clicked()
                    {
                        match crate::share::share_url(&self.state()) {
                            Ok(url) => {
                                ctx.copy_text(url);
                                self.share_status = Some("Copied share URL".to_owned());
                            }
                            Err(error) => self.share_status = Some(error),
                        }
                    }
                    if let Some(status) = &self.share_status {
                        ui.label(status);
                    }
                }
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
                .allow_drag(false)
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

#[derive(Clone, Debug, Serialize, Deserialize)]
struct MainAppState {
    version: u8,
    active: Scene,
    polygon: PolygonSceneState,
    points: PointSceneState,
    constraints: ConstraintSceneState,
    compare: CompareSceneState,
    view: ViewOptions,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
enum Scene {
    Polygon,
    Points,
    Constraints,
    Compare,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
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

#[derive(Clone, Debug, Serialize, Deserialize)]
struct PolygonSceneState {
    case: PolygonCase,
    input: PolygonInput,
}

struct PolygonScene {
    case: PolygonCase,
    input: PolygonInput,
    drag: Option<usize>,
}

impl Default for PolygonScene {
    fn default() -> Self {
        let case = PolygonCase::Holed;
        Self {
            case,
            input: case.input(),
            drag: None,
        }
    }
}

impl PolygonScene {
    fn state(&self) -> PolygonSceneState {
        PolygonSceneState {
            case: self.case,
            input: self.input.clone(),
        }
    }

    fn apply_state(&mut self, state: PolygonSceneState) -> Result<(), String> {
        validate_polygon_input(&state.input, "polygon scene")?;
        self.case = state.case;
        self.input = state.input;
        self.drag = None;
        Ok(())
    }

    fn controls(&mut self, ui: &mut egui::Ui) {
        ui.heading("Earcut Polygon");
        let mut changed = false;
        changed |= case_radio(ui, &mut self.case, PolygonCase::Concave, "Concave");
        changed |= case_radio(ui, &mut self.case, PolygonCase::Holed, "Holed");
        changed |= case_radio(
            ui,
            &mut self.case,
            PolygonCase::Adversarial,
            "Near collinear",
        );
        if changed {
            self.input = self.case.input();
            self.drag = None;
        }
        if ui.button("Reset").clicked() {
            self.input = self.case.input();
            self.drag = None;
        }
        let facts = polygon_facts(self.input.vertices.len(), self.input.holes.len());
        ui.separator();
        ui.label(facts);
    }

    fn draw(&mut self, plot_ui: &mut PlotUi<'_>, view: &ViewOptions) {
        if let Some(point) =
            handle_vertex_interaction(plot_ui, &mut self.input.vertices, &mut self.drag)
        {
            self.insert_vertex(point);
        }
        let input = &self.input;
        if view.show_input {
            draw_rings(plot_ui, &input.vertices, &input.holes);
        }
        match hypertri::f64::earcut(&APPROX, &input.vertices, &input.holes) {
            Ok(outcome) => {
                let indices = outcome.value;
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

    fn insert_vertex(&mut self, point: [f64; 2]) {
        if let Some(first_hole) = self.input.holes.first().copied() {
            self.input.vertices.insert(first_hole, point);
            for hole in &mut self.input.holes {
                *hole += 1;
            }
        } else {
            self.input.vertices.push(point);
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct PointSceneState {
    case: PointCase,
    points: Vec<[f64; 2]>,
}

struct PointScene {
    case: PointCase,
    points: Vec<[f64; 2]>,
    drag: Option<usize>,
}

impl Default for PointScene {
    fn default() -> Self {
        let case = PointCase::Cloud;
        Self {
            case,
            points: case.points(),
            drag: None,
        }
    }
}

impl PointScene {
    fn state(&self) -> PointSceneState {
        PointSceneState {
            case: self.case,
            points: self.points.clone(),
        }
    }

    fn apply_state(&mut self, state: PointSceneState) -> Result<(), String> {
        validate_points(&state.points, 3, "point scene")?;
        self.case = state.case;
        self.points = state.points;
        self.drag = None;
        Ok(())
    }

    fn controls(&mut self, ui: &mut egui::Ui) {
        ui.heading("Delaunay Points");
        let mut changed = false;
        changed |= case_radio(ui, &mut self.case, PointCase::Cloud, "Point cloud");
        changed |= case_radio(ui, &mut self.case, PointCase::Grid, "Perturbed grid");
        changed |= case_radio(
            ui,
            &mut self.case,
            PointCase::Degenerate,
            "Cospherical stress",
        );
        if changed {
            self.points = self.case.points();
            self.drag = None;
        }
        if ui.button("Reset").clicked() {
            self.points = self.case.points();
            self.drag = None;
        }
    }

    fn draw(&mut self, plot_ui: &mut PlotUi<'_>, view: &ViewOptions) {
        if let Some(point) = handle_vertex_interaction(plot_ui, &mut self.points, &mut self.drag) {
            self.points.push(point);
        }
        match hypertri::f64::delaunay(&APPROX, &self.points) {
            Ok(outcome) => {
                let triangulation = outcome.value;
                if view.show_triangles {
                    draw_triangles(
                        plot_ui,
                        "delaunay",
                        &self.points,
                        triangulation.triangles(),
                        RESULT,
                    );
                }
                if view.show_vertices {
                    draw_points(plot_ui, "points", &self.points, VERTEX);
                }
                if view.show_indices {
                    draw_labels(plot_ui, &self.points);
                }
                draw_status(
                    plot_ui,
                    "delaunay ok",
                    triangulation.triangles().len(),
                    self.points.len(),
                );
            }
            Err(error) => draw_error(plot_ui, &format!("{error}")),
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct ConstraintSceneState {
    case: ConstraintCase,
    input: ConstraintInputState,
}

struct ConstraintScene {
    case: ConstraintCase,
    input: ConstraintInput,
    drag: Option<usize>,
}

impl Default for ConstraintScene {
    fn default() -> Self {
        let case = ConstraintCase::BoxWithHole;
        Self {
            case,
            input: case.input(),
            drag: None,
        }
    }
}

impl ConstraintScene {
    fn state(&self) -> ConstraintSceneState {
        ConstraintSceneState {
            case: self.case,
            input: self.input.state(),
        }
    }

    fn apply_state(&mut self, state: ConstraintSceneState) -> Result<(), String> {
        validate_constraint_input(&state.input, "constraint scene")?;
        self.case = state.case;
        self.input = ConstraintInput::from_state(state.input);
        self.drag = None;
        Ok(())
    }

    fn controls(&mut self, ui: &mut egui::Ui) {
        ui.heading("Constrained CDT");
        let mut changed = false;
        changed |= case_radio(
            ui,
            &mut self.case,
            ConstraintCase::BoxWithHole,
            "Box with hole",
        );
        changed |= case_radio(
            ui,
            &mut self.case,
            ConstraintCase::Crossing,
            "Crossing constraints",
        );
        changed |= case_radio(ui, &mut self.case, ConstraintCase::OpenPslg, "Open PSLG");
        if changed {
            self.input = self.case.input();
            self.drag = None;
        }
        if ui.button("Reset").clicked() {
            self.input = self.case.input();
            self.drag = None;
        }
    }

    fn draw(&mut self, plot_ui: &mut PlotUi<'_>, view: &ViewOptions) {
        if let Some(point) =
            handle_vertex_interaction(plot_ui, &mut self.input.points, &mut self.drag)
        {
            self.input.points.push(point);
        }
        let input = &self.input;
        match hypertri::f64::constrained_delaunay(
            &APPROX,
            &input.points,
            &input.constraints,
        ) {
            Ok(outcome) => {
                let triangulation = outcome.value;
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

#[derive(Clone, Debug, Serialize, Deserialize)]
struct CompareSceneState {
    quality: QualityPolicyState,
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
    fn state(&self) -> CompareSceneState {
        CompareSceneState {
            quality: QualityPolicyState::from_quality(self.quality),
        }
    }

    fn apply_state(&mut self, state: CompareSceneState) {
        self.quality = state.quality.into_quality();
    }

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
        if let Ok(outcome) = hypertri::f64::earcut(&APPROX, &input.vertices, &input.holes) {
            let earcut = outcome.value;
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
        match hypertri::triangulate_polygon(&APPROX, &polygon, options) {
            Ok(outcome) => {
                let indices = outcome.value;
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

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
enum QualityPolicyState {
    PreserveBoundary,
    PreferDelaunay,
}

impl QualityPolicyState {
    fn from_quality(quality: QualityPolicy) -> Self {
        match quality {
            QualityPolicy::PreserveBoundary => Self::PreserveBoundary,
            QualityPolicy::PreferDelaunay => Self::PreferDelaunay,
        }
    }

    fn into_quality(self) -> QualityPolicy {
        match self {
            Self::PreserveBoundary => QualityPolicy::PreserveBoundary,
            Self::PreferDelaunay => QualityPolicy::PreferDelaunay,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
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

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
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

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
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

#[derive(Clone, Debug, Serialize, Deserialize)]
struct PolygonInput {
    vertices: Vec<[f64; 2]>,
    holes: Vec<usize>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct ConstraintInputState {
    points: Vec<[f64; 2]>,
    constraints: Vec<ConstraintState>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
struct ConstraintState {
    from: usize,
    to: usize,
}

struct ConstraintInput {
    points: Vec<[f64; 2]>,
    constraints: Vec<Constraint>,
}

impl ConstraintInput {
    fn state(&self) -> ConstraintInputState {
        ConstraintInputState {
            points: self.points.clone(),
            constraints: self
                .constraints
                .iter()
                .map(|constraint| ConstraintState {
                    from: constraint.from,
                    to: constraint.to,
                })
                .collect(),
        }
    }

    fn from_state(state: ConstraintInputState) -> Self {
        Self {
            points: state.points,
            constraints: state
                .constraints
                .into_iter()
                .map(|constraint| Constraint::new(constraint.from, constraint.to))
                .collect(),
        }
    }
}

const MAX_SHARED_POINTS: usize = 16_384;
const MAX_SHARED_CONSTRAINTS: usize = 32_768;

fn validate_polygon_input(input: &PolygonInput, label: &str) -> Result<(), String> {
    validate_points(&input.vertices, 3, label)?;
    if input.holes.len() >= input.vertices.len() {
        return Err(format!("{label} has too many hole starts"));
    }

    let mut previous = 0usize;
    for (index, &hole) in input.holes.iter().enumerate() {
        if hole == 0 || hole >= input.vertices.len() {
            return Err(format!(
                "{label} hole start {index} is outside the vertex range"
            ));
        }
        if hole <= previous {
            return Err(format!("{label} hole starts must be strictly increasing"));
        }
        if hole - previous < 3 {
            return Err(format!(
                "{label} ring {index} needs at least three vertices"
            ));
        }
        previous = hole;
    }

    if input.vertices.len() - previous < 3 {
        return Err(format!("{label} final ring needs at least three vertices"));
    }
    Ok(())
}

fn validate_constraint_input(input: &ConstraintInputState, label: &str) -> Result<(), String> {
    validate_points(&input.points, 2, label)?;
    if input.constraints.len() > MAX_SHARED_CONSTRAINTS {
        return Err(format!(
            "{label} has {} constraints; the shared-state limit is {MAX_SHARED_CONSTRAINTS}",
            input.constraints.len()
        ));
    }
    for (index, constraint) in input.constraints.iter().enumerate() {
        if constraint.from >= input.points.len() || constraint.to >= input.points.len() {
            return Err(format!(
                "{label} constraint {index} references a missing point"
            ));
        }
        if constraint.from == constraint.to {
            return Err(format!(
                "{label} constraint {index} has identical endpoints"
            ));
        }
    }
    Ok(())
}

fn validate_points(points: &[[f64; 2]], min_count: usize, label: &str) -> Result<(), String> {
    if points.len() < min_count {
        return Err(format!("{label} needs at least {min_count} point(s)"));
    }
    if points.len() > MAX_SHARED_POINTS {
        return Err(format!(
            "{label} has {} points; the shared-state limit is {MAX_SHARED_POINTS}",
            points.len()
        ));
    }
    for (index, point) in points.iter().enumerate() {
        if !point[0].is_finite() || !point[1].is_finite() {
            return Err(format!(
                "{label} point {index} must have finite coordinates"
            ));
        }
    }
    Ok(())
}

fn ring_constraints(start: usize, len: usize) -> Vec<Constraint> {
    (0..len)
        .map(|i| Constraint::new(start + i, start + ((i + 1) % len)))
        .collect()
}

fn case_radio<T: Copy + PartialEq>(ui: &mut egui::Ui, value: &mut T, case: T, label: &str) -> bool {
    ui.radio_value(value, case, label).changed()
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
                point.x.to_f64_lossy().unwrap_or(0.0),
                point.y.to_f64_lossy().unwrap_or(0.0),
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
            .stroke(Stroke::new(1.5_f32, color)),
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
            .stroke(Stroke::new(3.0_f32, color)),
        );
    }
}

fn draw_points(plot_ui: &mut PlotUi<'_>, name: &str, points: &[[f64; 2]], color: Color32) {
    plot_ui.points(
        Points::new(name, PlotPoints::from(points.to_vec()))
            .radius(4.5_f32)
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

fn handle_vertex_interaction(
    plot_ui: &mut PlotUi<'_>,
    points: &mut [[f64; 2]],
    drag: &mut Option<usize>,
) -> Option<[f64; 2]> {
    let response = plot_ui.response();
    let pointer_plot = plot_ui.pointer_coordinate()?;
    let pointer = [pointer_plot.x, pointer_plot.y];
    let pointer_screen = response.hover_pos()?;
    let primary_pressed = plot_ui.ctx().input(|input| input.pointer.primary_pressed());
    let primary_down = plot_ui.ctx().input(|input| input.pointer.primary_down());
    let primary_released = plot_ui
        .ctx()
        .input(|input| input.pointer.primary_released());

    if primary_released {
        *drag = None;
    }

    if primary_pressed && response.hovered() {
        *drag = nearest_vertex(plot_ui, points, pointer_screen, 12.0);
        if drag.is_none() {
            return Some(pointer);
        }
    }

    if primary_down
        && let Some(index) = *drag
        && let Some(point) = points.get_mut(index)
    {
        *point = pointer;
    }

    None
}

fn nearest_vertex(
    plot_ui: &PlotUi<'_>,
    points: &[[f64; 2]],
    pointer_screen: egui::Pos2,
    max_distance: f32,
) -> Option<usize> {
    points
        .iter()
        .enumerate()
        .filter_map(|(index, point)| {
            let screen = plot_ui.screen_from_plot(PlotPoint::new(point[0], point[1]));
            let distance = screen.distance(pointer_screen);
            (distance <= max_distance).then_some((index, distance))
        })
        .min_by(|(_, left), (_, right)| left.total_cmp(right))
        .map(|(index, _)| index)
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn app_state_round_trips_through_share_encoding() {
        let app = MainApp::default();
        let encoded = crate::share::encode_state(&app.state()).unwrap();
        let decoded = crate::share::decode_state::<MainAppState>(&encoded).unwrap();
        let restored = MainApp::from_state(decoded).unwrap();

        assert_eq!(restored.active, app.active);
        assert_eq!(restored.view.show_triangles, app.view.show_triangles);
        assert_eq!(
            restored.polygon.input.vertices.len(),
            app.polygon.input.vertices.len()
        );
        assert_eq!(restored.points.points.len(), app.points.points.len());
        assert_eq!(
            restored.constraints.input.constraints.len(),
            app.constraints.input.constraints.len()
        );
    }

    #[test]
    fn app_state_rejects_invalid_polygon_holes() {
        let mut state = MainApp::default().state();
        state.polygon.input.holes = vec![state.polygon.input.vertices.len() + 1];

        assert!(MainApp::from_state(state).is_err());
    }

    #[test]
    fn app_state_rejects_constraints_with_missing_vertices() {
        let mut state = MainApp::default().state();
        state.constraints.input.constraints[0].to = state.constraints.input.points.len();

        assert!(MainApp::from_state(state).is_err());
    }
}
