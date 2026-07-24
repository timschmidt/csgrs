use std::f64::consts::TAU;

use egui::{CentralPanel, ScrollArea, SidePanel, Slider};
use egui_plot::{Plot, PlotBounds};
use hypercurve::{
    BooleanOp, Curve2, CurvePath2, CurvePolicy, CurveRegion2, LineSeg2, Point2, Real,
};
use serde::{Deserialize, Serialize};

use crate::geometry::{BooleanMode, CurvePrimitive, Polyline, Shape, Vertex};
use crate::plotting::draw_shape;
use crate::theme::Theme;

const DEFAULT_REGIONS_PER_LAYER: usize = 2_000;
const DEFAULT_CURVES_PER_REGION: usize = 6;
const MIN_REGIONS_PER_LAYER: usize = 16;
const MAX_REGIONS_PER_LAYER: usize = 20_000;
const MIN_CURVES_PER_REGION: usize = 3;
const MAX_CURVES_PER_REGION: usize = 16;
const MAX_CURVES_PER_LAYER: usize = 160_000;
const BOOLEAN_BATCH_SIZE: usize = 8;
const GRID_SPACING: f64 = 3.0;

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct TortureSceneState {
    pub regions_per_layer: usize,
    pub curves_per_region: usize,
    pub operation: Option<BooleanMode>,
    pub seed: u64,
    #[serde(default)]
    pub segmented: bool,
}

impl Default for TortureSceneState {
    fn default() -> Self {
        Self {
            regions_per_layer: DEFAULT_REGIONS_PER_LAYER,
            curves_per_region: DEFAULT_CURVES_PER_REGION,
            operation: None,
            seed: 0x6a09_e667_f3bc_c909,
            segmented: false,
        }
    }
}

struct TorturePair {
    first: CurveRegion2,
    second: CurveRegion2,
}

pub struct TortureScene {
    requested_regions_per_layer: usize,
    requested_curves_per_region: usize,
    operation: Option<BooleanMode>,
    seed: u64,
    pairs: Vec<TorturePair>,
    first_display: Shape,
    second_display: Shape,
    result_regions: Vec<CurveRegion2>,
    result_display: Shape,
    segmented_result_display: Shape,
    segmented: bool,
    generated_curves_per_region: usize,
    evaluated_operation: Option<BooleanMode>,
    evaluated_pairs: usize,
    blocked_pairs: usize,
    fit_bounds: PlotBounds,
    fit_pending: bool,
    last_error: Option<String>,
}

impl Default for TortureScene {
    fn default() -> Self {
        Self::from_state(TortureSceneState::default())
    }
}

impl TortureScene {
    pub fn from_state(mut state: TortureSceneState) -> Self {
        state.curves_per_region = state
            .curves_per_region
            .clamp(MIN_CURVES_PER_REGION, MAX_CURVES_PER_REGION);
        state.regions_per_layer = state.regions_per_layer.clamp(
            MIN_REGIONS_PER_LAYER,
            max_regions_for_curves(state.curves_per_region),
        );
        Self {
            requested_regions_per_layer: state.regions_per_layer,
            requested_curves_per_region: state.curves_per_region,
            operation: state.operation,
            seed: state.seed,
            pairs: Vec::new(),
            first_display: Shape::default(),
            second_display: Shape::default(),
            result_regions: Vec::new(),
            result_display: Shape::default(),
            segmented_result_display: Shape::default(),
            segmented: state.segmented,
            generated_curves_per_region: 0,
            evaluated_operation: None,
            evaluated_pairs: 0,
            blocked_pairs: 0,
            fit_bounds: PlotBounds::NOTHING,
            fit_pending: true,
            last_error: None,
        }
    }

    #[cfg(any(target_arch = "wasm32", test))]
    pub const fn state(&self) -> TortureSceneState {
        TortureSceneState {
            regions_per_layer: self.requested_regions_per_layer,
            curves_per_region: self.requested_curves_per_region,
            operation: self.operation,
            seed: self.seed,
            segmented: self.segmented,
        }
    }

    pub fn ui(&mut self, ctx: &egui::Context, theme: &Theme) {
        self.ensure_generated();
        SidePanel::right("torture_controls")
            .default_width(275.0)
            .show(ctx, |ui| {
                ScrollArea::vertical().show(ui, |ui| {
                    ui.heading("CurveRegion2 Torture");
                    let curve_response = ui.add(
                        Slider::new(
                            &mut self.requested_curves_per_region,
                            MIN_CURVES_PER_REGION..=MAX_CURVES_PER_REGION,
                        )
                        .integer()
                        .text("Curves / region"),
                    );
                    if curve_response.changed() {
                        self.requested_regions_per_layer = self
                            .requested_regions_per_layer
                            .min(max_regions_for_curves(self.requested_curves_per_region));
                    }
                    ui.add(
                        Slider::new(
                            &mut self.requested_regions_per_layer,
                            MIN_REGIONS_PER_LAYER
                                ..=max_regions_for_curves(self.requested_curves_per_region),
                        )
                        .integer()
                        .logarithmic(true)
                        .text("Regions / layer"),
                    );
                    if ui.button("Re-generate").clicked() {
                        self.seed = self.seed.wrapping_add(0x9e37_79b9_7f4a_7c15);
                        self.regenerate();
                    }
                    if ui.button("Zoom to fit").clicked() {
                        self.fit_pending = true;
                    }
                    ui.separator();
                    operation_combo(ui, &mut self.operation);
                    ui.checkbox(&mut self.segmented, "Segmented");
                    ui.small("Pairwise operation: layer A[i] op layer B[i].");
                    ui.small(
                        "Centroid-grid placement keeps neighbors sparse while paired regions overlap moderately.",
                    );
                    ui.separator();
                    ui.label(format!(
                        "{} CurveRegion2s · {} authored curves",
                        self.pairs.len() * 2,
                        self.pairs.len() * 2 * self.generated_curves_per_region
                    ));
                    if self.operation.is_some() {
                        ui.label(format!(
                            "Evaluated {} / {} pairs · {} blocked",
                            self.evaluated_pairs,
                            self.pairs.len(),
                            self.blocked_pairs
                        ));
                        ui.label(format!(
                            "Buffered {} exact results · {} displayed loops",
                            self.result_regions.len(),
                            self.result_display.materials.len() + self.result_display.holes.len()
                        ));
                    }
                    if let Some(error) = &self.last_error {
                        ui.separator();
                        ui.colored_label(theme.error, error);
                    }
                });
            });

        self.advance_boolean(ctx);
        CentralPanel::default().show(ctx, |ui| {
            Plot::new("curve_region_torture_plot")
                .data_aspect(1.0)
                .show(ui, |plot_ui| {
                    if self.fit_pending && self.fit_bounds.is_valid() {
                        plot_ui.set_plot_bounds(self.fit_bounds);
                        self.fit_pending = false;
                    }
                    if self.operation.is_some() {
                        let result = if self.segmented {
                            &self.segmented_result_display
                        } else {
                            &self.result_display
                        };
                        draw_shape(
                            plot_ui,
                            "torture boolean results",
                            result,
                            theme.result,
                            Some(theme.result.gamma_multiply(0.28)),
                            None,
                        );
                    } else {
                        draw_shape(
                            plot_ui,
                            "torture layer A",
                            &self.first_display,
                            theme.primary,
                            Some(theme.primary.gamma_multiply(0.10)),
                            None,
                        );
                        draw_shape(
                            plot_ui,
                            "torture layer B",
                            &self.second_display,
                            theme.secondary,
                            Some(theme.secondary.gamma_multiply(0.10)),
                            None,
                        );
                    }
                });
        });
    }

    fn ensure_generated(&mut self) {
        if self.pairs.is_empty() {
            self.regenerate();
        }
    }

    fn regenerate(&mut self) {
        match generate_pairs(
            self.requested_regions_per_layer,
            self.requested_curves_per_region,
            self.seed,
        ) {
            Ok(generated) => {
                self.pairs = generated.pairs;
                self.first_display = Shape::from_materials(generated.first_display);
                self.second_display = Shape::from_materials(generated.second_display);
                self.generated_curves_per_region = self.requested_curves_per_region;
                self.fit_bounds = generated.fit_bounds;
                self.fit_pending = true;
                self.last_error = None;
                self.restart_boolean();
            }
            Err(error) => self.last_error = Some(error),
        }
    }

    fn restart_boolean(&mut self) {
        self.result_regions.clear();
        self.result_display = Shape::default();
        self.segmented_result_display = Shape::default();
        self.evaluated_operation = self.operation;
        self.evaluated_pairs = 0;
        self.blocked_pairs = 0;
        self.last_error = None;
    }

    fn advance_boolean(&mut self, ctx: &egui::Context) {
        if self.evaluated_operation != self.operation {
            self.restart_boolean();
        }
        let Some(operation) = self.operation else {
            return;
        };
        let end = (self.evaluated_pairs + BOOLEAN_BATCH_SIZE).min(self.pairs.len());
        for (offset, pair) in self.pairs[self.evaluated_pairs..end].iter().enumerate() {
            match boolean_pair(pair, operation) {
                Ok(Some((region, shape))) => {
                    let segmented = shape.segmented_for_display();
                    self.result_regions.push(region);
                    self.result_display.materials.extend(shape.materials);
                    self.result_display.holes.extend(shape.holes);
                    self.segmented_result_display
                        .materials
                        .extend(segmented.materials);
                    self.segmented_result_display.holes.extend(segmented.holes);
                }
                Ok(None) => self.blocked_pairs += 1,
                Err(error) => {
                    self.blocked_pairs += 1;
                    if self.last_error.is_none() {
                        self.last_error = Some(format!(
                            "first blocked pair {}: {error}",
                            self.evaluated_pairs + offset
                        ));
                    }
                }
            }
        }
        self.evaluated_pairs = end;
        if self.evaluated_pairs < self.pairs.len() {
            ctx.request_repaint();
        }
    }
}

struct GeneratedPairs {
    pairs: Vec<TorturePair>,
    first_display: Vec<Polyline>,
    second_display: Vec<Polyline>,
    fit_bounds: PlotBounds,
}

fn generate_pairs(
    count: usize,
    curves_per_region: usize,
    seed: u64,
) -> Result<GeneratedPairs, String> {
    let mut rng = FuzzRng::new(seed);
    let columns = ((count as f64 * 1.6).sqrt().ceil() as usize).max(1);
    let mut pairs = Vec::with_capacity(count);
    let mut first_display = Vec::with_capacity(count);
    let mut second_display = Vec::with_capacity(count);
    let mut min = [f64::INFINITY, f64::INFINITY];
    let mut max = [f64::NEG_INFINITY, f64::NEG_INFINITY];

    for index in 0..count {
        let column = index % columns;
        let row = index / columns;
        let base_x = column as f64 * GRID_SPACING;
        let base_y = row as f64 * GRID_SPACING;
        let first_center = [base_x + rng.signed(0.12), base_y + rng.signed(0.12)];
        let offset_angle = rng.unit() * TAU;
        let offset_distance = 0.55 + rng.unit() * 0.20;
        let second_center = [
            first_center[0] + offset_angle.cos() * offset_distance,
            first_center[1] + offset_angle.sin() * offset_distance,
        ];
        let first_radius = 0.92 + rng.unit() * 0.14;
        let second_radius = 0.92 + rng.unit() * 0.14;
        let first = fuzzed_region(
            first_center,
            first_radius,
            curves_per_region,
            rng.unit() * TAU,
            &mut rng,
        )?;
        let second = fuzzed_region(
            second_center,
            second_radius,
            curves_per_region,
            rng.unit() * TAU,
            &mut rng,
        )?;
        extend_bounds(&mut min, &mut max, &first.display);
        extend_bounds(&mut min, &mut max, &second.display);
        first_display.push(first.display);
        second_display.push(second.display);
        pairs.push(TorturePair {
            first: first.region,
            second: second.region,
        });
    }

    let padding = 1.0;
    let fit_bounds = PlotBounds::from_min_max(
        [min[0] - padding, min[1] - padding],
        [max[0] + padding, max[1] + padding],
    );
    Ok(GeneratedPairs {
        pairs,
        first_display,
        second_display,
        fit_bounds,
    })
}

struct GeneratedRegion {
    region: CurveRegion2,
    display: Polyline,
}

fn fuzzed_region(
    center: [f64; 2],
    radius: f64,
    curve_count: usize,
    rotation: f64,
    rng: &mut FuzzRng,
) -> Result<GeneratedRegion, String> {
    let vertices = (0..curve_count)
        .map(|index| {
            let angle = rotation + index as f64 * TAU / curve_count as f64;
            let local_radius = radius * (0.88 + rng.unit() * 0.18);
            Vertex::new(
                center[0] + angle.cos() * local_radius,
                center[1] + angle.sin() * local_radius,
                0.0,
            )
        })
        .collect::<Vec<_>>();
    let mut display_curves = Vec::with_capacity(curve_count);
    let mut exact_curves = Vec::with_capacity(curve_count);
    for index in 0..curve_count {
        let start = vertices[index];
        let end = vertices[(index + 1) % curve_count];
        display_curves.push(CurvePrimitive::Line { start, end });
        exact_curves.push(
            LineSeg2::try_new(exact_point(start)?, exact_point(end)?)
                .map(Curve2::from)
                .map_err(|error| error.to_string())?,
        );
    }
    let path = CurvePath2::try_new(exact_curves).map_err(|error| error.to_string())?;
    let region =
        CurveRegion2::try_from_boundary_paths(&[path]).map_err(|error| error.to_string())?;
    Ok(GeneratedRegion {
        region,
        display: Polyline::from_curve_data(display_curves, true),
    })
}

fn exact_point(vertex: Vertex) -> Result<Point2, String> {
    Ok(Point2::new(
        Real::try_from(vertex.x).map_err(|_| "failed to lift torture x coordinate")?,
        Real::try_from(vertex.y).map_err(|_| "failed to lift torture y coordinate")?,
    ))
}

fn extend_bounds(min: &mut [f64; 2], max: &mut [f64; 2], polyline: &Polyline) {
    for vertex in polyline.handles() {
        min[0] = min[0].min(vertex.x);
        min[1] = min[1].min(vertex.y);
        max[0] = max[0].max(vertex.x);
        max[1] = max[1].max(vertex.y);
    }
}

fn boolean_pair(
    pair: &TorturePair,
    operation: BooleanMode,
) -> Result<Option<(CurveRegion2, Shape)>, String> {
    let retained = pair
        .first
        .retain_boolean(&pair.second, &CurvePolicy::certified())
        .map_err(|error| error.to_string())?;
    let region = retained
        .boolean_region(match operation {
            BooleanMode::Union => BooleanOp::Union,
            BooleanMode::Intersection => BooleanOp::Intersection,
            BooleanMode::Difference => BooleanOp::Difference,
            BooleanMode::Xor => BooleanOp::Xor,
        })
        .map_err(|error| error.to_string())?;
    Shape::from_curve_region(&region).map(|display| display.map(|display| (region, display)))
}

fn operation_combo(ui: &mut egui::Ui, operation: &mut Option<BooleanMode>) {
    let selected = match operation {
        None => "Inputs",
        Some(BooleanMode::Union) => "Union",
        Some(BooleanMode::Intersection) => "Intersection",
        Some(BooleanMode::Difference) => "Difference A − B",
        Some(BooleanMode::Xor) => "Xor",
    };
    egui::ComboBox::from_id_salt("torture_boolean_operation")
        .selected_text(selected)
        .show_ui(ui, |ui| {
            ui.selectable_value(operation, None, "Inputs");
            ui.selectable_value(operation, Some(BooleanMode::Union), "Union");
            ui.selectable_value(operation, Some(BooleanMode::Intersection), "Intersection");
            ui.selectable_value(operation, Some(BooleanMode::Difference), "Difference A − B");
            ui.selectable_value(operation, Some(BooleanMode::Xor), "Xor");
        });
}

const fn max_regions_for_curves(curves_per_region: usize) -> usize {
    let by_curve_budget = MAX_CURVES_PER_LAYER / curves_per_region;
    if by_curve_budget < MAX_REGIONS_PER_LAYER {
        by_curve_budget
    } else {
        MAX_REGIONS_PER_LAYER
    }
}

struct FuzzRng(u64);

impl FuzzRng {
    fn new(seed: u64) -> Self {
        Self(seed | 1)
    }

    fn unit(&mut self) -> f64 {
        let mut value = self.0;
        value ^= value >> 12;
        value ^= value << 25;
        value ^= value >> 27;
        self.0 = value;
        let sample = value.wrapping_mul(0x2545_f491_4f6c_dd1d) >> 11;
        sample as f64 * (1.0 / ((1_u64 << 53) as f64))
    }

    fn signed(&mut self, magnitude: f64) -> f64 {
        (self.unit() * 2.0 - 1.0) * magnitude
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn generator_builds_two_requested_sparse_layers() {
        let generated = generate_pairs(12, 7, 42).unwrap();
        assert_eq!(generated.pairs.len(), 12);
        assert_eq!(generated.first_display.len(), 12);
        assert_eq!(generated.second_display.len(), 12);
        assert!(generated.fit_bounds.is_valid());
        assert!(
            generated
                .first_display
                .iter()
                .chain(generated.second_display.iter())
                .all(|polyline| polyline.curve_data.len() == 7)
        );
        for (first, second) in generated
            .first_display
            .iter()
            .zip(&generated.second_display)
        {
            let first = test_bounds(first);
            let second = test_bounds(second);
            let overlap_width = first[2].min(second[2]) - first[0].max(second[0]);
            let overlap_height = first[3].min(second[3]) - first[1].max(second[1]);
            let first_area = (first[2] - first[0]) * (first[3] - first[1]);
            let second_area = (second[2] - second[0]) * (second[3] - second[1]);
            let overlap_ratio = overlap_width * overlap_height / first_area.min(second_area);
            assert!(overlap_width > 0.0 && overlap_height > 0.0);
            assert!(
                overlap_ratio < 0.85,
                "paired bounding boxes should overlap without nearly coinciding"
            );
        }
    }

    #[test]
    fn default_torture_scene_materializes_thousands_of_regions() {
        let mut scene = TortureScene::default();
        scene.ensure_generated();
        assert_eq!(scene.pairs.len(), DEFAULT_REGIONS_PER_LAYER);
        assert_eq!(
            scene.first_display.materials.len() + scene.second_display.materials.len(),
            DEFAULT_REGIONS_PER_LAYER * 2
        );
    }

    #[test]
    fn one_fuzzed_pair_resolves_every_boolean() {
        let generated = generate_pairs(1, 6, 7).unwrap();
        for operation in [
            BooleanMode::Union,
            BooleanMode::Intersection,
            BooleanMode::Difference,
            BooleanMode::Xor,
        ] {
            assert!(
                boolean_pair(&generated.pairs[0], operation)
                    .unwrap()
                    .is_some(),
                "{operation:?} should resolve"
            );
        }
    }

    #[test]
    fn boolean_batches_append_to_the_exact_result_buffer() {
        let mut scene = TortureScene::from_state(TortureSceneState {
            regions_per_layer: MIN_REGIONS_PER_LAYER,
            curves_per_region: 5,
            operation: Some(BooleanMode::Union),
            seed: 99,
            segmented: false,
        });
        scene.ensure_generated();
        let context = egui::Context::default();
        scene.advance_boolean(&context);
        assert_eq!(scene.evaluated_pairs, BOOLEAN_BATCH_SIZE);
        assert_eq!(scene.result_regions.len(), BOOLEAN_BATCH_SIZE);
        scene.advance_boolean(&context);
        assert_eq!(scene.evaluated_pairs, MIN_REGIONS_PER_LAYER);
        assert_eq!(scene.result_regions.len(), MIN_REGIONS_PER_LAYER);
        assert_eq!(
            scene.segmented_result_display.materials.len(),
            scene.result_display.materials.len()
        );
        assert!(
            scene
                .segmented_result_display
                .materials
                .iter()
                .all(|polyline| polyline.curve_data.is_empty())
        );
        scene.advance_boolean(&context);
        assert_eq!(scene.result_regions.len(), MIN_REGIONS_PER_LAYER);
    }

    fn test_bounds(polyline: &Polyline) -> [f64; 4] {
        polyline.handles().into_iter().fold(
            [
                f64::INFINITY,
                f64::INFINITY,
                f64::NEG_INFINITY,
                f64::NEG_INFINITY,
            ],
            |bounds, vertex| {
                [
                    bounds[0].min(vertex.x),
                    bounds[1].min(vertex.y),
                    bounds[2].max(vertex.x),
                    bounds[3].max(vertex.y),
                ]
            },
        )
    }
}
