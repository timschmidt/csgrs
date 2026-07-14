use egui::{CentralPanel, ScrollArea, SidePanel, Slider, TopBottomPanel};
use egui_plot::Plot;
use hypercurve::OffsetCap;
use serde::{Deserialize, Serialize};

use crate::corner_scenes::{CornerOperation, CornerScene};
use crate::editor::PolylineEditor;
use crate::geometry::{
    BooleanMode, Polyline, Shape, boolean_polylines, contour_intersections, contour_slices,
    curve_showcase_contour, curve_showcase_polylines,
};
use crate::plotting::{draw_points, draw_polyline, draw_shape, find_near_vertex};
use crate::theme::Theme;

const DEFAULT_SHARE_STATE: &str = "_Td6WFoAAATm1rRGAgAhARwAAAAQz1jM4DpvBzZdAD2IisaUU5CGpmN9JTsoyCLgCC9eAQQ27Q7WqXNVd1y0-wb6C1ZgWGQnHCWS18V4qXLh9BwxxVau-JTy2x52gCslzqlWcvTnvroRR15N7jOzVVKyl9y2jv_M52lTtD-auC6XkxvB1uahKO3gEPYUyEv6Gz_MtxPx9VmO4SWWxTvjiLar2l_sqU_RXR2jG_u8ckoi6XsMR4Ypa1_H3UgTxdY1Mw1vVzb03gcFA6p644S5WfbZk6vYaSrOPWgCqmTmvytc3SFWzLb4J14ulU_EaDmga_jUJm7IFOQYHd4BlE7dqpJdM6LeRDGuffyga2fw9t6Ia75wfsjHtwv-nX2EkT4ynH5ftPgW5rXpy5_QFZJ6bXF1csuEPDJOXku8sXjrfiYaLiruWVnSJF2223Y1kml1-fQlOM9ffefyN6IDMvqviF_AiqNkiWIcbISOkWt4bMcU1ukFUpAKq1rJH9UHHZMVfKbNgufEQOHAgpjD2jyFo2QgML-rfFOE_Z2heRMVwNs3TgWO4b4gJqivtckylIX4FKlBdk6vWzNa9mWmWMmNJJRi7Ua2gAAihbBdUjwe9Qg1pYls0TdAX6wl77uxtNfsUE9jsWVykb-A4FK2xtgia3Qftm5ZyOq6Ne-qrvNig0GDmfvD39trEh8IrmXS5HamvtQ-ykAxhEhdRlAa5Z00o7qnOriQ1LVwLjUMfCeVV_qnDfffktSxpcXizohgii-18Ya0KcjSBtuGXkZJspigsiclxxq1BN7_QvzrRXu2fyvGC4lg9aBIDfqVdbjIqec1EPCNBmYdZLENv2Gl__XzrpJZSMq9Br7RtiJDuFeBH0FpE7N4ZXE9uxgYPQ_YmvFoltQBxixjYzsD4xkqrBgwdroopPH3VRhtvhsEHRRtzvpDcW9akf-p1P9XCEKVIqx5tWpILNz3FceiWZ1yXd_aU5csv2HORALO9DzITMTJVtS9XWKsaAnKaKiAgkmygfOJccAOaY34nPhKuoEyQ5N5goJaCCQc1r-EyzsHx2W5JAAvQWxrohY7mPjqS4YNwQJfwVecQ2IpRrlYiJZhx4GGdkyspvtqpNpT60MN3tejmUwo7Vf5ORdM7bdMSUFF1AYcJmp9Qe69FP8srOe-t56Ko343cIaj6wsBwpuMC9TjznzorbaxKNrgrV5uERkmmY9WhEVml5aU5wNfynTUyrLWQnFLIu7TMoUbI1DEizED2yGHVDK1nuULgS5IgXF78UvoLLPPOevuwVAH92wBDhU-VOGNNjh5XOGUtOJWqbAB9YsfYOUQOmUUZCgrqk4ddDuEslYuq0IxEqvHCL_lkpGfMDsjMSDZwPwD18h51w63uWLvrcxDeDK0__IQDUWLbFcpH4up_vSjz-o6ZH42uCS1BJJvaSUCH7pq1BLXPXfpC3N8PdCDD-uzsd0JTWUmOxzVRRKFdzI-CVY0t7-IaQ3IoqOxAqhzRf0aRJiqs6Fngh_Q1UfGVg5M7V2t2GlZhV7egkTiebB35_Q9lwx6hd5sj_FYMkErLxlDb8VDqN8e50BlIO6gbJVyOQZPkIZ61NSLM1K0f-WKisquSNQDWHlM1_bT19tskwgtv7NcNTacTU9krJeELOs0HuZ7QpRKUW7C6EXeUlLLRP7I8phHC2ohj0kL7FkcBSn5yYzz-vyJK0Nng4nsmuqsTXXea8mzrasJv_rzFbyjDS-TYtI-Rb_4gJD77Ky0l6Md3Lc2H9Ybpg8xXwQYKcdK5ZfATErySZwI2cUXriPIQ3oo5kNA-c6g8LW-ubiEPX9bZbstnq399FVV2-d1QerKnoRk-OaNSt-6lYZUAr_DUQwjWBZTIHdjZlV729bgItkLp3E15HLdfYBuzaPt0nTc5V3_MYrbjPP1GqBe0gsIJanMnkImcivePNoZkP3U9vSwPns7TdP0S8fY7oiI8qHYa6ccEnaYkNeSsMva35J_adLKl1nsEEfAISNkpxUu9j0pvwLhQun7FtDStB08iwlgGQwqbt0_dWnYhSNz4DWhnslYJ5iR5QGmfwp-Cjn4FbvmHU0Kg8bfvi0H8iGdcO-wi1y62cXAMXT29ttcq4JhAQjDBiAIln-b1OcUz2gDepJ_cVM2Pw2804zM-PzfDOqVLEGKtWvn8A9o6hUxvfthzJ22G2BU0i-NtwVUlPGoFEOtuDynRckWPik1IUEvHCD13CdFv5-oDMkkPOMPCIItHsFfhQoYoe1XNrHTAynvp8i9UJFPsQT6rpXkyQDIL9uuNPnwiV6OaM4YPIJ_rhuP7Io3TE2SZDbQZY4GQHnEi_UAKfY6RdCCWGSdDaZu1k79aOFpGlGOcKFewbI4eQivTPTSYLl_FETVMzahSO-EfEUiGfREAKMEx51zQMsWOFZGhsRDnjZ7O56rEZO_eNTTKewzw3CSD2z5E7rJQQ9vrsLLEoPDelZs7uH5VNb24eETW5l05USIdpa461PKssUAAAClYjLYKsxz0gAB0g7wdAAAswEN9rHEZ_sCAAAAAARZWg";

pub struct DemoScenes {
    active: usize,
    pline_boolean: PlineBooleanScene,
    pline_offset: PlineOffsetScene,
    multi_boolean: MultiPlineBooleanScene,
    multi_offset: MultiPlineOffsetScene,
    fillet: CornerScene,
    chamfer: CornerScene,
    #[cfg(target_arch = "wasm32")]
    share_status: Option<String>,
}

impl Default for DemoScenes {
    fn default() -> Self {
        Self::shared_url_default()
    }
}

impl DemoScenes {
    /// No-query startup state decoded from the requested shared demo URL.
    fn shared_url_default() -> Self {
        match crate::share::decode_state::<DemoScenesState>(DEFAULT_SHARE_STATE)
            .and_then(Self::from_state)
        {
            Ok(scenes) => return scenes,
            Err(error) => log::warn!("falling back after invalid baked hypercurve state: {error}"),
        }

        Self::fallback_default()
    }

    fn fallback_default() -> Self {
        Self {
            active: 0,
            pline_boolean: PlineBooleanScene::default(),
            pline_offset: PlineOffsetScene::default(),
            multi_boolean: MultiPlineBooleanScene::default(),
            multi_offset: MultiPlineOffsetScene::default(),
            fillet: CornerScene::new(CornerOperation::Fillet, default_fillet_radius()),
            chamfer: CornerScene::new(CornerOperation::Chamfer, default_chamfer_setback()),
            #[cfg(target_arch = "wasm32")]
            share_status: None,
        }
    }

    pub fn new() -> Self {
        #[cfg(target_arch = "wasm32")]
        {
            match crate::share::load_from_location::<DemoScenesState>() {
                Ok(Some(state)) => match Self::from_state(state) {
                    Ok(scenes) => return scenes,
                    Err(error) => {
                        log::warn!("ignoring invalid shared hypercurve UI state: {error}")
                    }
                },
                Ok(None) => {}
                Err(error) => log::warn!("ignoring invalid shared hypercurve UI state: {error}"),
            }
        }

        Self::default()
    }

    pub fn ui(&mut self, ctx: &egui::Context) {
        TopBottomPanel::top("scene_tabs").show(ctx, |ui| {
            ui.horizontal_wrapped(|ui| {
                for (index, label) in [
                    "Polyline Boolean",
                    "Polyline Offset",
                    "Multi Polyline Boolean",
                    "Multi Polyline Offset",
                    "Fillets",
                    "Chamfers",
                ]
                .into_iter()
                .enumerate()
                {
                    ui.selectable_value(&mut self.active, index, label);
                }
                ui.separator();
                ui.hyperlink_to("GitHub", "https://github.com/timschmidt/hypercurve");
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

        let theme = Theme::for_context(ctx);
        match self.active {
            0 => self.pline_boolean.ui(ctx, &theme),
            1 => self.pline_offset.ui(ctx, &theme),
            2 => self.multi_boolean.ui(ctx, &theme),
            3 => self.multi_offset.ui(ctx, &theme),
            4 => self.fillet.ui(ctx, &theme),
            _ => self.chamfer.ui(ctx, &theme),
        }
    }

    fn from_state(state: DemoScenesState) -> Result<Self, String> {
        if state.version != 1 {
            return Err(format!("unsupported state version {}", state.version));
        }
        let mut scenes = Self::fallback_default();
        scenes.active = state.active.min(5);
        scenes.pline_boolean.apply_state(state.pline_boolean)?;
        scenes.pline_offset.apply_state(state.pline_offset)?;
        scenes.multi_boolean.apply_state(state.multi_boolean)?;
        scenes.multi_offset.apply_state(state.multi_offset)?;
        scenes.fillet.apply_amount(state.fillet_radius)?;
        scenes.chamfer.apply_amount(state.chamfer_setback)?;
        Ok(scenes)
    }

    #[cfg(any(target_arch = "wasm32", test))]
    fn state(&self) -> DemoScenesState {
        DemoScenesState {
            version: 1,
            active: self.active,
            pline_boolean: self.pline_boolean.state(),
            pline_offset: self.pline_offset.state(),
            multi_boolean: self.multi_boolean.state(),
            multi_offset: self.multi_offset.state(),
            fillet_radius: self.fillet.amount(),
            chamfer_setback: self.chamfer.amount(),
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct DemoScenesState {
    version: u8,
    active: usize,
    pline_boolean: PlineBooleanSceneState,
    pline_offset: PlineOffsetSceneState,
    multi_boolean: MultiPlineBooleanSceneState,
    multi_offset: MultiPlineOffsetSceneState,
    #[serde(default = "default_fillet_radius")]
    fillet_radius: f64,
    #[serde(default = "default_chamfer_setback")]
    chamfer_setback: f64,
}

const fn default_fillet_radius() -> f64 {
    1.0
}

const fn default_chamfer_setback() -> f64 {
    1.0
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct PlineBooleanSceneState {
    polylines: Vec<Polyline>,
    mode: BooleanSceneMode,
    fill: bool,
    show_vertices: bool,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq, Serialize, Deserialize)]
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
        let first = curve_showcase_contour(0.0, 0.0, 6.2);
        let second = curve_showcase_contour(3.5, 1.8, 4.2);
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
    #[cfg(any(target_arch = "wasm32", test))]
    fn state(&self) -> PlineBooleanSceneState {
        PlineBooleanSceneState {
            polylines: self.polylines.clone(),
            mode: self.mode,
            fill: self.fill,
            show_vertices: self.show_vertices,
        }
    }

    fn apply_state(&mut self, state: PlineBooleanSceneState) -> Result<(), String> {
        validate_polylines(&state.polylines, 2, "polyline boolean")?;
        validate_closed_polylines(&state.polylines, "polyline boolean")?;
        self.polylines = state.polylines;
        self.mode = state.mode;
        self.fill = state.fill;
        self.show_vertices = state.show_vertices;
        self.drag = DragState::default();
        self.editor
            .initialize_with_polylines(self.polylines.clone());
        self.last_error = None;
        Ok(())
    }

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

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
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

#[derive(Clone, Debug, Serialize, Deserialize)]
struct PlineOffsetSceneState {
    polylines: Vec<Polyline>,
    mode: OffsetMode,
    offset: f64,
    max_offset_count: usize,
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
        let polyline = curve_showcase_contour(0.0, 0.0, 6.4);
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
    #[cfg(any(target_arch = "wasm32", test))]
    fn state(&self) -> PlineOffsetSceneState {
        PlineOffsetSceneState {
            polylines: self.polylines.clone(),
            mode: self.mode,
            offset: self.offset,
            max_offset_count: self.max_offset_count,
        }
    }

    fn apply_state(&mut self, state: PlineOffsetSceneState) -> Result<(), String> {
        validate_polylines(&state.polylines, 1, "polyline offset")?;
        validate_finite_state(state.offset, "polyline offset distance")?;
        self.polylines = state.polylines;
        self.mode = state.mode;
        self.offset = state.offset.clamp(-50.0, 50.0);
        self.max_offset_count = state.max_offset_count.min(50);
        self.drag = DragState::default();
        self.editor
            .initialize_with_polylines(self.polylines.clone());
        self.last_error = None;
        Ok(())
    }

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

#[derive(Clone, Debug, Serialize, Deserialize)]
struct MultiPlineBooleanSceneState {
    first: Shape,
    second: Shape,
    op: Option<BooleanMode>,
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
    #[cfg(any(target_arch = "wasm32", test))]
    fn state(&self) -> MultiPlineBooleanSceneState {
        MultiPlineBooleanSceneState {
            first: self.first.clone(),
            second: self.second.clone(),
            op: self.op,
        }
    }

    fn apply_state(&mut self, state: MultiPlineBooleanSceneState) -> Result<(), String> {
        validate_shape(&state.first, "first multi-polyline boolean shape")?;
        validate_shape(&state.second, "second multi-polyline boolean shape")?;
        self.first = state.first;
        self.second = state.second;
        self.op = state.op;
        self.drag = ShapeDragState::default();
        self.last_error = None;
        Ok(())
    }

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

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq, Serialize, Deserialize)]
enum MultiOffsetMode {
    #[default]
    Offset,
    OffsetIntersects,
    OffsetLoops,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct MultiPlineOffsetSceneState {
    polylines: Vec<Polyline>,
    mode: MultiOffsetMode,
    offset: f64,
    max_offset_count: usize,
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
    #[cfg(any(target_arch = "wasm32", test))]
    fn state(&self) -> MultiPlineOffsetSceneState {
        MultiPlineOffsetSceneState {
            polylines: self.polylines.clone(),
            mode: self.mode,
            offset: self.offset,
            max_offset_count: self.max_offset_count,
        }
    }

    fn apply_state(&mut self, state: MultiPlineOffsetSceneState) -> Result<(), String> {
        validate_polylines(&state.polylines, 1, "multi-polyline offset")?;
        validate_closed_polylines(&state.polylines, "multi-polyline offset")?;
        validate_finite_state(state.offset, "multi-polyline offset distance")?;
        self.polylines = state.polylines;
        self.mode = state.mode;
        self.offset = state.offset.clamp(-50.0, 50.0);
        self.max_offset_count = state.max_offset_count.min(50);
        self.drag = DragState::default();
        self.editor
            .initialize_with_polylines(self.polylines.clone());
        self.last_error = None;
        Ok(())
    }

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
    grabbed: Option<(usize, ShapeContourRole, usize, usize)>,
    dragging_plot: bool,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum ShapeContourRole {
    Material,
    Hole,
}

const MAX_SHARED_POLYLINES: usize = 256;
const MAX_SHARED_VERTICES: usize = 16_384;

fn validate_polylines(polylines: &[Polyline], min_count: usize, label: &str) -> Result<(), String> {
    if polylines.len() < min_count {
        return Err(format!("{label} needs at least {min_count} polyline(s)"));
    }
    if polylines.len() > MAX_SHARED_POLYLINES {
        return Err(format!(
            "{label} has {} polylines; the shared-state limit is {MAX_SHARED_POLYLINES}",
            polylines.len()
        ));
    }

    let mut vertices = 0usize;
    for (index, polyline) in polylines.iter().enumerate() {
        if polyline.handles().len() < 2 {
            return Err(format!(
                "{label} polyline {index} needs at least two vertices"
            ));
        }
        vertices = vertices.saturating_add(polyline.handles().len());
        polyline
            .validate_finite()
            .map_err(|error| format!("{label} polyline {index}: {error}"))?;
    }
    if vertices > MAX_SHARED_VERTICES {
        return Err(format!(
            "{label} has {vertices} vertices; the shared-state limit is {MAX_SHARED_VERTICES}"
        ));
    }
    Ok(())
}

fn validate_closed_polylines(polylines: &[Polyline], label: &str) -> Result<(), String> {
    for (index, polyline) in polylines.iter().enumerate() {
        if !polyline.is_closed() {
            return Err(format!("{label} polyline {index} must be closed"));
        }
    }
    Ok(())
}

fn validate_shape(shape: &Shape, label: &str) -> Result<(), String> {
    let polylines = shape
        .materials
        .iter()
        .chain(shape.holes.iter())
        .cloned()
        .collect::<Vec<_>>();
    validate_polylines(&polylines, 1, label)?;
    validate_closed_polylines(&polylines, label)
}

fn validate_finite_state(value: f64, label: &str) -> Result<(), String> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(format!("{label} must be finite"))
    }
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
            .and_then(|polyline| polyline.handle(vertex_index))
        {
            polylines[pline_index].set_handle(
                vertex_index,
                vertex.x + f64::from(delta.x),
                vertex.y + f64::from(delta.y),
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
    if let Some((shape_index, role, pline_index, vertex_index)) = state.grabbed {
        let delta = plot_ui.pointer_coordinate_drag_delta();
        let shape = if shape_index == 0 { first } else { second };
        let polylines = match role {
            ShapeContourRole::Material => &mut shape.materials,
            ShapeContourRole::Hole => &mut shape.holes,
        };
        if let Some(vertex) = polylines
            .get(pline_index)
            .and_then(|polyline| polyline.handle(vertex_index))
        {
            polylines[pline_index].set_handle(
                vertex_index,
                vertex.x + f64::from(delta.x),
                vertex.y + f64::from(delta.y),
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
        state.grabbed = find_near_shape_vertex(plot_ui, coord, 0, first)
            .or_else(|| find_near_shape_vertex(plot_ui, coord, 1, second));
        state.dragging_plot = state.grabbed.is_none();
    }
}

fn find_near_shape_vertex(
    plot_ui: &egui_plot::PlotUi<'_>,
    coord: egui::Pos2,
    shape_index: usize,
    shape: &Shape,
) -> Option<(usize, ShapeContourRole, usize, usize)> {
    find_near_vertex(plot_ui, coord, &shape.materials)
        .map(|(pline, vertex)| (shape_index, ShapeContourRole::Material, pline, vertex))
        .or_else(|| {
            find_near_vertex(plot_ui, coord, &shape.holes)
                .map(|(pline, vertex)| (shape_index, ShapeContourRole::Hole, pline, vertex))
        })
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
    curve_showcase_polylines(0.0, 0.0, 55.0)
}

fn default_multi_offset_plines() -> Vec<Polyline> {
    curve_showcase_polylines(0.0, 0.0, 55.0)
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

    #[test]
    fn demo_state_round_trips_through_share_encoding() {
        let scenes = DemoScenes::default();
        let encoded = crate::share::encode_state(&scenes.state()).unwrap();
        let decoded = crate::share::decode_state::<DemoScenesState>(&encoded).unwrap();
        let restored = DemoScenes::from_state(decoded).unwrap();

        assert_eq!(restored.active, scenes.active);
        assert_eq!(
            restored.pline_boolean.polylines.len(),
            scenes.pline_boolean.polylines.len()
        );
        assert_eq!(
            restored.multi_offset.polylines.len(),
            scenes.multi_offset.polylines.len()
        );
    }

    #[test]
    fn default_state_matches_requested_share_url_layout() {
        let state = DemoScenes::default().state();

        assert_eq!(state.active, 0);
        assert_eq!(state.pline_boolean.polylines.len(), 2);
        assert_eq!(state.pline_boolean.polylines[0].curve_data.len(), 8);
        assert_eq!(state.pline_boolean.polylines[1].curve_data.len(), 8);
        assert_eq!(state.pline_offset.polylines.len(), 1);
        assert_eq!(state.pline_offset.polylines[0].curve_data.len(), 8);
        assert_eq!(state.fillet_radius, default_fillet_radius());
        assert_eq!(state.chamfer_setback, default_chamfer_setback());
        assert_eq!(state.multi_boolean.first.materials.len(), 2);
        assert_eq!(state.multi_boolean.first.holes.len(), 3);
        assert_eq!(state.multi_boolean.second.materials.len(), 2);
        assert_eq!(state.multi_boolean.second.holes.len(), 3);
        assert_eq!(
            state
                .multi_offset
                .polylines
                .iter()
                .map(|polyline| polyline.curve_data.len())
                .collect::<Vec<_>>(),
            vec![8, 4, 2, 4, 2]
        );
    }

    #[test]
    fn demo_state_rejects_non_finite_offsets() {
        let mut state = DemoScenes::default().state();
        state.pline_offset.offset = f64::NAN;

        assert!(DemoScenes::from_state(state).is_err());
    }

    #[test]
    fn demo_state_rejects_incomplete_boolean_inputs() {
        let mut state = DemoScenes::default().state();
        state.pline_boolean.polylines.pop();

        assert!(DemoScenes::from_state(state).is_err());
    }
}
