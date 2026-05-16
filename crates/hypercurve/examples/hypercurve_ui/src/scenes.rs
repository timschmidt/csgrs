use egui::{CentralPanel, ScrollArea, SidePanel, Slider, TopBottomPanel};
use egui_plot::Plot;
use hypercurve::OffsetCap;

use crate::editor::PolylineEditor;
use crate::geometry::{
    BooleanMode, Polyline, Shape, boolean_polylines, contour_intersections, contour_slices,
};
use crate::plotting::{draw_points, draw_polyline, draw_shape, find_near_vertex};
use crate::theme::Theme;

pub struct DemoScenes {
    active: usize,
    pline_boolean: PlineBooleanScene,
    pline_offset: PlineOffsetScene,
    multi_boolean: MultiPlineBooleanScene,
    multi_offset: MultiPlineOffsetScene,
}

impl Default for DemoScenes {
    fn default() -> Self {
        Self {
            active: 0,
            pline_boolean: PlineBooleanScene::default(),
            pline_offset: PlineOffsetScene::default(),
            multi_boolean: MultiPlineBooleanScene::default(),
            multi_offset: MultiPlineOffsetScene::default(),
        }
    }
}

impl DemoScenes {
    pub fn ui(&mut self, ctx: &egui::Context) {
        TopBottomPanel::top("scene_tabs").show(ctx, |ui| {
            ui.horizontal_wrapped(|ui| {
                for (index, label) in [
                    "Polyline Boolean",
                    "Polyline Offset",
                    "Multi Polyline Boolean",
                    "Multi Polyline Offset",
                ]
                .into_iter()
                .enumerate()
                {
                    ui.selectable_value(&mut self.active, index, label);
                }
            });
        });

        let theme = Theme::for_context(ctx);
        match self.active {
            0 => self.pline_boolean.ui(ctx, &theme),
            1 => self.pline_offset.ui(ctx, &theme),
            2 => self.multi_boolean.ui(ctx, &theme),
            _ => self.multi_offset.ui(ctx, &theme),
        }
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
enum BooleanSceneMode {
    #[default]
    None,
    Union,
    Intersection,
    Difference,
    Xor,
    Intersects,
    Slices,
}

impl BooleanSceneMode {
    fn label(self) -> &'static str {
        match self {
            Self::None => "None",
            Self::Union => "Or",
            Self::Intersection => "And",
            Self::Difference => "Not",
            Self::Xor => "Xor",
            Self::Intersects => "Intersects",
            Self::Slices => "Slices",
        }
    }

    fn boolean_mode(self) -> Option<BooleanMode> {
        match self {
            Self::Union => Some(BooleanMode::Union),
            Self::Intersection => Some(BooleanMode::Intersection),
            Self::Difference => Some(BooleanMode::Difference),
            Self::Xor => Some(BooleanMode::Xor),
            _ => None,
        }
    }
}

pub struct PlineBooleanScene {
    polylines: Vec<Polyline>,
    mode: BooleanSceneMode,
    fill: bool,
    show_vertices: bool,
    drag: DragState,
    editor: PolylineEditor,
    last_error: Option<String>,
}

impl Default for PlineBooleanScene {
    fn default() -> Self {
        let first = Polyline::closed(&[
            (10.0, 10.0, -0.5),
            (0.3, 1.0, 0.374794619217547),
            (21.0, 0.0, 0.0),
            (23.0, 0.0, 1.0),
            (32.0, 0.0, -0.5),
            (28.0, 0.0, 0.5),
            (39.0, 21.0, 0.0),
            (28.0, 12.0, 0.5),
        ]);
        let mut second = Polyline::closed(&[
            (10.0, 10.0, -0.5),
            (8.0, 9.0, 0.374794619217547),
            (21.0, 0.0, 0.0),
            (23.0, 0.0, 1.0),
            (32.0, 0.0, -0.5),
            (28.0, 0.0, 0.5),
            (38.0, 19.0, 0.0),
            (28.0, 12.0, 0.5),
        ]);
        second.scale_mut(0.5);
        let polylines = vec![first, second];
        let mut editor = PolylineEditor::dual("Polyline Editor");
        editor.initialize_with_polylines(polylines.clone());
        Self {
            polylines,
            mode: BooleanSceneMode::default(),
            fill: true,
            show_vertices: true,
            drag: DragState::default(),
            editor,
            last_error: None,
        }
    }
}

impl PlineBooleanScene {
    fn ui(&mut self, ctx: &egui::Context, theme: &Theme) {
        SidePanel::right("pline_boolean_controls")
            .default_width(240.0)
            .show(ctx, |ui| {
                ScrollArea::vertical().show(ui, |ui| {
                    ui.heading("Polyline Boolean");
                    mode_combo(ui, "pline_boolean_mode", &mut self.mode);
                    ui.checkbox(&mut self.fill, "Fill");
                    ui.checkbox(&mut self.show_vertices, "Show vertices");
                    if ui.button("Edit Polylines").clicked() {
                        self.editor.show_window();
                    }
                    if let Some(error) = &self.last_error {
                        ui.separator();
                        ui.colored_label(theme.error, error);
                    }
                });
            });

        self.editor.ui(ctx, &mut self.polylines, theme);

        CentralPanel::default().show(ctx, |ui| {
            Plot::new("pline_boolean_plot")
                .data_aspect(1.0)
                .allow_drag(false)
                .show(ui, |plot_ui| {
                    let show_sources = self.mode.boolean_mode().is_none();
                    if show_sources {
                        handle_polyline_drag(plot_ui, &mut self.polylines, &mut self.drag);
                    }
                    self.last_error = None;
                    let vertex = self.show_vertices.then_some(theme.vertex);
                    if show_sources {
                        draw_polyline(
                            plot_ui,
                            "polyline 1",
                            &self.polylines[0],
                            theme.primary,
                            self.fill.then_some(theme.primary.gamma_multiply(0.18)),
                            vertex,
                        );
                        draw_polyline(
                            plot_ui,
                            "polyline 2",
                            &self.polylines[1],
                            theme.secondary,
                            self.fill.then_some(theme.secondary.gamma_multiply(0.18)),
                            vertex,
                        );
                    }

                    match self.mode {
                        BooleanSceneMode::None => {}
                        BooleanSceneMode::Intersects => {
                            match contour_intersections(&self.polylines[0], &self.polylines[1]) {
                                Ok((points, overlaps)) => {
                                    draw_points(plot_ui, "intersections", &points, theme.error);
                                    for (index, overlap) in overlaps.iter().enumerate() {
                                        draw_polyline(
                                            plot_ui,
                                            format!("overlap {index}"),
                                            overlap,
                                            theme.warning,
                                            None,
                                            None,
                                        );
                                    }
                                }
                                Err(error) => self.last_error = Some(error),
                            }
                        }
                        BooleanSceneMode::Slices => {
                            match contour_slices(&self.polylines[0], &self.polylines[1]) {
                                Ok((first, second)) => {
                                    for (index, slice) in
                                        first.iter().chain(second.iter()).enumerate()
                                    {
                                        draw_polyline(
                                            plot_ui,
                                            format!("slice {index}"),
                                            slice,
                                            multi_color(index),
                                            None,
                                            None,
                                        );
                                    }
                                }
                                Err(error) => self.last_error = Some(error),
                            }
                        }
                        mode => {
                            if let Some(op) = mode.boolean_mode() {
                                match boolean_polylines(&self.polylines[0], &self.polylines[1], op)
                                {
                                    Ok(Some(shape)) => draw_shape(
                                        plot_ui,
                                        "boolean result",
                                        &shape,
                                        theme.result,
                                        self.fill.then_some(theme.result.gamma_multiply(0.35)),
                                        None,
                                    ),
                                    Ok(None) => {
                                        self.last_error =
                                            Some("hypercurve reported unresolved topology".into());
                                    }
                                    Err(error) => self.last_error = Some(error),
                                }
                            }
                        }
                    }
                });
        });
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum OffsetMode {
    Offset,
    RawOffset,
    RawOffsetSegments,
    Outline,
}

impl OffsetMode {
    fn label(self) -> &'static str {
        match self {
            Self::Offset => "Offset",
            Self::RawOffset => "Raw Offset",
            Self::RawOffsetSegments => "Raw Offset Segments",
            Self::Outline => "Open Outline",
        }
    }
}

pub struct PlineOffsetScene {
    polylines: Vec<Polyline>,
    mode: OffsetMode,
    offset: f64,
    max_offset_count: usize,
    drag: DragState,
    editor: PolylineEditor,
    last_error: Option<String>,
}

impl Default for PlineOffsetScene {
    fn default() -> Self {
        let polyline = Polyline::closed(&[
            (10.0, 10.0, -0.5),
            (8.0, 9.0, 0.374794619217547),
            (21.0, 0.0, 0.0),
            (23.0, 0.0, 1.0),
            (32.0, 0.0, -0.5),
            (28.0, 0.0, 0.5),
            (39.0, 21.0, 0.0),
            (28.0, 12.0, 0.5),
        ]);
        let polylines = vec![polyline];
        let mut editor = PolylineEditor::single("Vertex Editor");
        editor.initialize_with_polylines(polylines.clone());
        Self {
            polylines,
            mode: OffsetMode::Offset,
            offset: 1.0,
            max_offset_count: 10,
            drag: DragState::default(),
            editor,
            last_error: None,
        }
    }
}

impl PlineOffsetScene {
    fn ui(&mut self, ctx: &egui::Context, theme: &Theme) {
        SidePanel::right("pline_offset_controls")
            .default_width(240.0)
            .show(ctx, |ui| {
                ScrollArea::vertical().show(ui, |ui| {
                    ui.heading("Polyline Offset");
                    egui::ComboBox::from_id_salt("pline_offset_mode")
                        .selected_text(self.mode.label())
                        .show_ui(ui, |ui| {
                            for mode in [
                                OffsetMode::Offset,
                                OffsetMode::RawOffset,
                                OffsetMode::RawOffsetSegments,
                                OffsetMode::Outline,
                            ] {
                                ui.selectable_value(&mut self.mode, mode, mode.label());
                            }
                        });
                    ui.add(Slider::new(&mut self.offset, -50.0..=50.0).text("Offset"));
                    if self.mode == OffsetMode::Offset {
                        ui.add(
                            Slider::new(&mut self.max_offset_count, 0..=50)
                                .integer()
                                .text("Max count"),
                        );
                    }
                    if ui.button("Edit Vertices").clicked() {
                        self.editor.show_window();
                    }
                    if let Some(error) = &self.last_error {
                        ui.separator();
                        ui.colored_label(theme.error, error);
                    }
                });
            });
        self.editor.ui(ctx, &mut self.polylines, theme);

        CentralPanel::default().show(ctx, |ui| {
            Plot::new("pline_offset_plot")
                .data_aspect(1.0)
                .allow_drag(false)
                .show(ui, |plot_ui| {
                    handle_polyline_drag(plot_ui, &mut self.polylines, &mut self.drag);
                    let source = &self.polylines[0];
                    draw_polyline(
                        plot_ui,
                        "source",
                        source,
                        theme.accent,
                        None,
                        Some(theme.vertex),
                    );
                    self.last_error = None;
                    match self.build_offset_state() {
                        Ok(polylines) => {
                            for (index, polyline) in polylines.iter().enumerate() {
                                draw_polyline(
                                    plot_ui,
                                    format!("offset {index}"),
                                    polyline,
                                    multi_color(index),
                                    polyline
                                        .is_closed()
                                        .then_some(multi_color(index).gamma_multiply(0.16)),
                                    None,
                                );
                            }
                        }
                        Err(error) => self.last_error = Some(error),
                    }
                });
        });
    }

    fn build_offset_state(&self) -> Result<Vec<Polyline>, String> {
        let source = &self.polylines[0];
        match self.mode {
            OffsetMode::RawOffset => Ok(source.raw_offset(self.offset)?.into_iter().collect()),
            OffsetMode::RawOffsetSegments => source.raw_offset_segments(self.offset),
            OffsetMode::Outline => Ok(source
                .outline(self.offset.abs().max(0.001), OffsetCap::Round)?
                .into_iter()
                .collect()),
            OffsetMode::Offset => {
                let mut result = Vec::new();
                let mut current = vec![source.clone()];
                for _ in 0..self.max_offset_count {
                    let mut next = Vec::new();
                    for polyline in current {
                        for offset in polyline.offsets_for_display(self.offset)? {
                            next.push(offset.clone());
                            result.push(offset);
                        }
                    }
                    if next.is_empty() {
                        break;
                    }
                    current = next;
                }
                Ok(result)
            }
        }
    }
}

pub struct MultiPlineBooleanScene {
    first: Shape,
    second: Shape,
    op: Option<BooleanMode>,
    drag: ShapeDragState,
    last_error: Option<String>,
}

impl Default for MultiPlineBooleanScene {
    fn default() -> Self {
        let plines = default_multi_boolean_plines();
        Self {
            first: Shape::from_polylines(plines.clone()).translated(-20.0, -20.0),
            second: Shape::from_polylines(plines).translated(20.0, 20.0),
            op: None,
            drag: ShapeDragState::default(),
            last_error: None,
        }
    }
}

impl MultiPlineBooleanScene {
    fn ui(&mut self, ctx: &egui::Context, theme: &Theme) {
        SidePanel::right("multi_boolean_controls")
            .default_width(240.0)
            .show(ctx, |ui| {
                ScrollArea::vertical().show(ui, |ui| {
                    ui.heading("Multi Polyline Boolean");
                    ui.radio_value(&mut self.op, None, "None");
                    ui.radio_value(&mut self.op, Some(BooleanMode::Union), "Or");
                    ui.radio_value(&mut self.op, Some(BooleanMode::Intersection), "And");
                    ui.radio_value(&mut self.op, Some(BooleanMode::Difference), "Not");
                    ui.radio_value(&mut self.op, Some(BooleanMode::Xor), "Xor");
                    if let Some(error) = &self.last_error {
                        ui.separator();
                        ui.colored_label(theme.error, error);
                    }
                });
            });
        CentralPanel::default().show(ctx, |ui| {
            Plot::new("multi_boolean_plot")
                .data_aspect(1.0)
                .allow_drag(false)
                .show(ui, |plot_ui| {
                    handle_shape_drag(plot_ui, &mut self.first, &mut self.second, &mut self.drag);
                    self.last_error = None;
                    if let Some(op) = self.op {
                        match self.first.boolean(&self.second, op) {
                            Ok(Some(result)) => draw_shape(
                                plot_ui,
                                "multi boolean result",
                                &result,
                                theme.result,
                                Some(theme.result.gamma_multiply(0.35)),
                                Some(theme.vertex),
                            ),
                            Ok(None) => {
                                self.last_error =
                                    Some("hypercurve reported unresolved topology".into());
                            }
                            Err(error) => self.last_error = Some(error),
                        }
                    } else {
                        draw_shape(
                            plot_ui,
                            "shape 1",
                            &self.first,
                            theme.primary,
                            Some(theme.primary.gamma_multiply(0.16)),
                            Some(theme.primary),
                        );
                        draw_shape(
                            plot_ui,
                            "shape 2",
                            &self.second,
                            theme.secondary,
                            Some(theme.secondary.gamma_multiply(0.16)),
                            Some(theme.secondary),
                        );
                    }
                });
        });
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
enum MultiOffsetMode {
    #[default]
    Offset,
    OffsetIntersects,
    OffsetLoops,
}

pub struct MultiPlineOffsetScene {
    polylines: Vec<Polyline>,
    mode: MultiOffsetMode,
    offset: f64,
    max_offset_count: usize,
    drag: DragState,
    editor: PolylineEditor,
    last_error: Option<String>,
}

impl Default for MultiPlineOffsetScene {
    fn default() -> Self {
        let polylines = default_multi_offset_plines();
        let mut editor = PolylineEditor::multi("Multi-Polyline Editor");
        editor.initialize_with_polylines(polylines.clone());
        Self {
            polylines,
            mode: MultiOffsetMode::Offset,
            offset: 2.0,
            max_offset_count: 12,
            drag: DragState::default(),
            editor,
            last_error: None,
        }
    }
}

impl MultiPlineOffsetScene {
    fn ui(&mut self, ctx: &egui::Context, theme: &Theme) {
        SidePanel::right("multi_offset_controls")
            .default_width(240.0)
            .show(ctx, |ui| {
                ScrollArea::vertical().show(ui, |ui| {
                    ui.heading("Multi Polyline Offset");
                    egui::ComboBox::from_id_salt("multi_offset_mode")
                        .selected_text(match self.mode {
                            MultiOffsetMode::Offset => "Offset",
                            MultiOffsetMode::OffsetIntersects => "Offset Intersects",
                            MultiOffsetMode::OffsetLoops => "Offset Loops",
                        })
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut self.mode, MultiOffsetMode::Offset, "Offset");
                            ui.selectable_value(
                                &mut self.mode,
                                MultiOffsetMode::OffsetIntersects,
                                "Offset Intersects",
                            );
                            ui.selectable_value(
                                &mut self.mode,
                                MultiOffsetMode::OffsetLoops,
                                "Offset Loops",
                            );
                        });
                    ui.add(Slider::new(&mut self.offset, -50.0..=50.0).text("Offset"));
                    if self.mode == MultiOffsetMode::Offset {
                        ui.add(
                            Slider::new(&mut self.max_offset_count, 0..=50)
                                .integer()
                                .text("Max count"),
                        );
                    }
                    if ui.button("Edit Polylines").clicked() {
                        self.editor.show_window();
                    }
                    if let Some(error) = &self.last_error {
                        ui.separator();
                        ui.colored_label(theme.error, error);
                    }
                });
            });
        self.editor.ui(ctx, &mut self.polylines, theme);

        CentralPanel::default().show(ctx, |ui| {
            Plot::new("multi_offset_plot")
                .data_aspect(1.0)
                .allow_drag(false)
                .show(ui, |plot_ui| {
                    handle_polyline_drag(plot_ui, &mut self.polylines, &mut self.drag);
                    let source = Shape::from_polylines(self.polylines.clone());
                    draw_shape(
                        plot_ui,
                        "multi source",
                        &source,
                        theme.accent,
                        None,
                        Some(theme.vertex),
                    );

                    self.last_error = None;
                    let offset_once = source.offset_once(self.offset);
                    match self.mode {
                        MultiOffsetMode::Offset => {
                            let mut current = offset_once;
                            for index in 0..self.max_offset_count {
                                if current.materials.is_empty() && current.holes.is_empty() {
                                    break;
                                }
                                draw_shape(
                                    plot_ui,
                                    &format!("shape offset {index}"),
                                    &current,
                                    multi_color(index),
                                    None,
                                    None,
                                );
                                current = current.offset_once(self.offset);
                            }
                        }
                        MultiOffsetMode::OffsetLoops => {
                            draw_shape(
                                plot_ui,
                                "offset loops",
                                &offset_once,
                                theme.primary,
                                None,
                                None,
                            );
                        }
                        MultiOffsetMode::OffsetIntersects => {
                            draw_shape(
                                plot_ui,
                                "offset loops",
                                &offset_once,
                                theme.primary,
                                None,
                                None,
                            );
                            let mut points = Vec::new();
                            for i in 0..offset_once.materials.len() {
                                for j in (i + 1)..offset_once.materials.len() {
                                    if let Ok((hits, _)) = contour_intersections(
                                        &offset_once.materials[i],
                                        &offset_once.materials[j],
                                    ) {
                                        points.extend(hits);
                                    }
                                }
                            }
                            draw_points(plot_ui, "offset intersections", &points, theme.error);
                        }
                    }
                });
        });
    }
}

#[derive(Default)]
struct DragState {
    grabbed: Option<(usize, usize)>,
    dragging_plot: bool,
}

#[derive(Default)]
struct ShapeDragState {
    grabbed: Option<(usize, usize, usize)>,
    dragging_plot: bool,
}

fn handle_polyline_drag(
    plot_ui: &mut egui_plot::PlotUi<'_>,
    polylines: &mut [Polyline],
    state: &mut DragState,
) {
    if plot_ui.ctx().input(|i| i.pointer.any_released()) {
        state.grabbed = None;
        state.dragging_plot = false;
        return;
    }
    if let Some((pline_index, vertex_index)) = state.grabbed {
        let delta = plot_ui.pointer_coordinate_drag_delta();
        if let Some(vertex) = polylines
            .get(pline_index)
            .and_then(|polyline| polyline.get(vertex_index))
            .copied()
        {
            polylines[pline_index].set(
                vertex_index,
                vertex.x + f64::from(delta.x),
                vertex.y + f64::from(delta.y),
                vertex.bulge,
            );
        }
        return;
    }
    if state.dragging_plot {
        plot_ui.translate_bounds(-plot_ui.pointer_coordinate_drag_delta());
        return;
    }
    if plot_ui.ctx().input(|i| i.pointer.any_pressed())
        && let Some(coord) = plot_ui.ctx().pointer_interact_pos()
    {
        state.grabbed = find_near_vertex(plot_ui, coord, polylines);
        state.dragging_plot = state.grabbed.is_none();
    }
}

fn handle_shape_drag(
    plot_ui: &mut egui_plot::PlotUi<'_>,
    first: &mut Shape,
    second: &mut Shape,
    state: &mut ShapeDragState,
) {
    if plot_ui.ctx().input(|i| i.pointer.any_released()) {
        state.grabbed = None;
        state.dragging_plot = false;
        return;
    }
    if let Some((shape_index, pline_index, vertex_index)) = state.grabbed {
        let delta = plot_ui.pointer_coordinate_drag_delta();
        let shape = if shape_index == 0 { first } else { second };
        if let Some(vertex) = shape
            .materials
            .get(pline_index)
            .and_then(|polyline| polyline.get(vertex_index))
            .copied()
        {
            shape.materials[pline_index].set(
                vertex_index,
                vertex.x + f64::from(delta.x),
                vertex.y + f64::from(delta.y),
                vertex.bulge,
            );
        }
        return;
    }
    if state.dragging_plot {
        plot_ui.translate_bounds(-plot_ui.pointer_coordinate_drag_delta());
        return;
    }
    if plot_ui.ctx().input(|i| i.pointer.any_pressed())
        && let Some(coord) = plot_ui.ctx().pointer_interact_pos()
    {
        state.grabbed = find_near_vertex(plot_ui, coord, &first.materials)
            .map(|(pline, vertex)| (0, pline, vertex))
            .or_else(|| {
                find_near_vertex(plot_ui, coord, &second.materials)
                    .map(|(pline, vertex)| (1, pline, vertex))
            });
        state.dragging_plot = state.grabbed.is_none();
    }
}

fn mode_combo(ui: &mut egui::Ui, id: &str, mode: &mut BooleanSceneMode) {
    egui::ComboBox::from_id_salt(id)
        .selected_text(mode.label())
        .show_ui(ui, |ui| {
            for value in [
                BooleanSceneMode::None,
                BooleanSceneMode::Union,
                BooleanSceneMode::Intersection,
                BooleanSceneMode::Difference,
                BooleanSceneMode::Xor,
                BooleanSceneMode::Intersects,
                BooleanSceneMode::Slices,
            ] {
                ui.selectable_value(mode, value, value.label());
            }
        });
}

fn default_multi_boolean_plines() -> Vec<Polyline> {
    vec![
        Polyline::closed(&[
            (100.0, 100.0, -0.5),
            (80.0, 90.0, 0.374794619217547),
            (210.0, 0.0, 0.0),
            (230.0, 0.0, 1.0),
            (320.0, 0.0, -0.5),
            (280.0, 0.0, 0.5),
            (390.0, 210.0, 0.0),
            (280.0, 120.0, 0.5),
        ]),
        Polyline::closed(&[
            (150.0, 50.0, 0.0),
            (150.0, 100.0, 0.0),
            (223.74732137849435, 142.16931273980475, 0.0),
            (199.491310072685, 52.51543504258919, 0.5),
        ]),
        Polyline::closed(&[
            (261.11232783167395, 35.79686193615828, -1.0),
            (250.0, 100.0, -1.0),
        ]),
        Polyline::closed(&[
            (320.2986109239592, 103.52378781211337, 0.0),
            (320.5065990423979, 76.14222955572362, -1.0),
        ]),
        Polyline::closed(&[
            (273.6131273938006, -13.968608715397636, -0.3),
            (256.61336060995995, -25.49387433156079, 0.0),
            (249.69820124026208, 27.234215862385582, 0.0),
        ]),
    ]
}

fn default_multi_offset_plines() -> Vec<Polyline> {
    let mut plines = default_multi_boolean_plines();
    plines[3] = Polyline::closed(&[
        (320.5065990423979, 76.14222955572362, -1.0),
        (320.2986109239592, 103.52378781211337, 0.0),
    ]);
    plines
}

fn multi_color(index: usize) -> egui::Color32 {
    const COLORS: [egui::Color32; 8] = [
        egui::Color32::from_rgb(80, 170, 255),
        egui::Color32::from_rgb(255, 130, 130),
        egui::Color32::from_rgb(110, 210, 125),
        egui::Color32::from_rgb(240, 190, 80),
        egui::Color32::from_rgb(185, 135, 255),
        egui::Color32::from_rgb(80, 210, 210),
        egui::Color32::from_rgb(230, 125, 200),
        egui::Color32::from_rgb(170, 190, 90),
    ];
    COLORS[index % COLORS.len()]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn multi_defaults_are_sorted_into_material_and_hole_bins() {
        let shape = Shape::from_polylines(default_multi_boolean_plines());

        assert!(!shape.materials.is_empty());
        assert!(!shape.holes.is_empty());
    }

    #[test]
    fn multi_boolean_defaults_resolve_all_boolean_modes() {
        let plines = default_multi_boolean_plines();
        let first = Shape::from_polylines(plines.clone()).translated(-20.0, -20.0);
        let second = Shape::from_polylines(plines).translated(20.0, 20.0);

        for op in [
            BooleanMode::Union,
            BooleanMode::Intersection,
            BooleanMode::Difference,
            BooleanMode::Xor,
        ] {
            let result = first.boolean(&second, op).unwrap();
            assert!(
                result.is_some(),
                "default multi-polyline boolean returned unresolved topology for {op:?}"
            );
        }
    }

    #[test]
    fn polyline_boolean_defaults_resolve_all_boolean_modes() {
        let scene = PlineBooleanScene::default();

        for op in [
            BooleanMode::Union,
            BooleanMode::Intersection,
            BooleanMode::Difference,
            BooleanMode::Xor,
        ] {
            assert!(
                boolean_polylines(&scene.polylines[0], &scene.polylines[1], op)
                    .unwrap()
                    .is_some(),
                "default polyline boolean returned unresolved topology for {op:?}"
            );
        }
    }

    #[test]
    fn polyline_offset_default_scene_produces_visible_offsets() {
        let scene = PlineOffsetScene::default();

        assert!(
            !scene.build_offset_state().unwrap().is_empty(),
            "default polyline offset scene should produce at least one visible offset"
        );
    }

    #[test]
    fn multi_offset_default_scene_produces_first_visible_offset() {
        let source = Shape::from_polylines(default_multi_offset_plines());

        assert!(
            !source.offset_once(2.0).materials.is_empty()
                || !source.offset_once(2.0).holes.is_empty(),
            "default multi-polyline offset scene should produce at least one visible offset"
        );
    }
}
