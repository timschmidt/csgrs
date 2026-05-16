use egui::{ScrollArea, TextEdit, Window};

use crate::geometry::{Polyline, Vertex};
use crate::theme::Theme;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EditorLayout {
    Single,
    Dual,
    Multi,
}

pub struct PolylineEditor {
    title: String,
    layout: EditorLayout,
    open: bool,
    pending: Vec<Polyline>,
    current_json: String,
    pending_json: String,
    json_error: Option<String>,
    tab: EditorTab,
    initialized: bool,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
enum EditorTab {
    #[default]
    Table,
    Json,
}

impl PolylineEditor {
    pub fn single(title: &str) -> Self {
        Self::new(title, EditorLayout::Single)
    }

    pub fn dual(title: &str) -> Self {
        Self::new(title, EditorLayout::Dual)
    }

    pub fn multi(title: &str) -> Self {
        Self::new(title, EditorLayout::Multi)
    }

    fn new(title: &str, layout: EditorLayout) -> Self {
        Self {
            title: title.to_string(),
            layout,
            open: false,
            pending: Vec::new(),
            current_json: String::new(),
            pending_json: String::new(),
            json_error: None,
            tab: EditorTab::Table,
            initialized: false,
        }
    }

    pub fn initialize_with_polylines(&mut self, polylines: Vec<Polyline>) {
        self.pending = polylines;
        self.current_json = self.serialize(&self.pending);
        self.pending_json = self.current_json.clone();
        self.json_error = None;
    }

    pub fn show_window(&mut self) {
        self.open = true;
    }

    pub fn ui(&mut self, ctx: &egui::Context, polylines: &mut Vec<Polyline>, theme: &Theme) {
        if self.open && !self.initialized {
            self.pending = polylines.clone();
            self.current_json = self.serialize(polylines);
            self.pending_json = self.current_json.clone();
            self.json_error = None;
            self.initialized = true;
        }

        let mut open = self.open;
        Window::new(&self.title)
            .open(&mut open)
            .default_width(match self.layout {
                EditorLayout::Single => 760.0,
                EditorLayout::Dual => 860.0,
                EditorLayout::Multi => 920.0,
            })
            .default_height(560.0)
            .resizable(true)
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.selectable_value(&mut self.tab, EditorTab::Table, "Table");
                    ui.selectable_value(&mut self.tab, EditorTab::Json, "JSON");
                });
                ui.separator();
                if let Some(error) = &self.json_error {
                    ui.colored_label(theme.error, error);
                    ui.separator();
                }
                match self.tab {
                    EditorTab::Table => self.table_tab(ui, polylines, theme),
                    EditorTab::Json => self.json_tab(ui, polylines, theme),
                }
            });

        self.open = open;
        if !open {
            self.initialized = false;
        }
    }

    fn table_tab(&mut self, ui: &mut egui::Ui, polylines: &mut Vec<Polyline>, theme: &Theme) {
        ui.horizontal(|ui| {
            if ui.button("Apply Changes").clicked() {
                *polylines = self.pending.clone();
                self.current_json = self.serialize(polylines);
                self.pending_json = self.current_json.clone();
            }
            if ui.button("Cancel").clicked() {
                self.pending = polylines.clone();
                self.pending_json = self.current_json.clone();
            }
            if self.pending != *polylines {
                ui.colored_label(theme.warning, "Changes pending");
            }
        });
        ui.separator();

        match self.layout {
            EditorLayout::Single => self.single_table(ui),
            EditorLayout::Dual => self.dual_table(ui),
            EditorLayout::Multi => self.multi_table(ui),
        }
    }

    fn single_table(&mut self, ui: &mut egui::Ui) {
        ensure_len(&mut self.pending, 1);
        ui.horizontal(|ui| {
            if ui.button("Add Vertex").clicked() {
                add_vertex(&mut self.pending[0]);
            }
            ui.checkbox(&mut self.pending[0].is_closed, "Closed");
        });
        ScrollArea::both().show(ui, |ui| vertex_table(ui, &mut self.pending[0]));
    }

    fn dual_table(&mut self, ui: &mut egui::Ui) {
        ensure_len(&mut self.pending, 2);
        ui.columns(2, |columns| {
            for (index, column) in columns.iter_mut().enumerate() {
                column.heading(format!("Polyline {}", index + 1));
                column.horizontal(|ui| {
                    if ui.button("Add Vertex").clicked() {
                        add_vertex(&mut self.pending[index]);
                    }
                    ui.checkbox(&mut self.pending[index].is_closed, "Closed");
                });
                ScrollArea::both().show(column, |ui| vertex_table(ui, &mut self.pending[index]));
            }
        });
    }

    fn multi_table(&mut self, ui: &mut egui::Ui) {
        if ui.button("Add Polyline").clicked() {
            let mut polyline = Polyline::new();
            polyline.is_closed = true;
            add_vertex(&mut polyline);
            self.pending.push(polyline);
        }
        ScrollArea::vertical().show(ui, |ui| {
            let mut delete = None;
            for index in 0..self.pending.len() {
                egui::CollapsingHeader::new(format!("Polyline {}", index + 1))
                    .default_open(index < 3)
                    .show(ui, |ui| {
                        ui.horizontal(|ui| {
                            if ui.button("Add Vertex").clicked() {
                                add_vertex(&mut self.pending[index]);
                            }
                            ui.checkbox(&mut self.pending[index].is_closed, "Closed");
                            if ui.button("Delete").clicked() {
                                delete = Some(index);
                            }
                        });
                        vertex_table(ui, &mut self.pending[index]);
                    });
            }
            if let Some(index) = delete {
                self.pending.remove(index);
            }
        });
    }

    fn json_tab(&mut self, ui: &mut egui::Ui, polylines: &mut Vec<Polyline>, theme: &Theme) {
        ui.horizontal(|ui| {
            if ui.button("Apply JSON").clicked() {
                match self.parse(&self.pending_json) {
                    Ok(parsed) => {
                        *polylines = parsed;
                        self.pending = polylines.clone();
                        self.current_json = self.serialize(polylines);
                        self.pending_json = self.current_json.clone();
                        self.json_error = None;
                    }
                    Err(error) => self.json_error = Some(error),
                }
            }
            if self.pending_json != self.current_json {
                ui.colored_label(theme.warning, "JSON changes pending");
            }
        });
        ui.separator();
        ui.add_sized(
            ui.available_size(),
            TextEdit::multiline(&mut self.pending_json)
                .font(egui::TextStyle::Monospace)
                .desired_rows(24),
        );
    }

    fn serialize(&self, polylines: &[Polyline]) -> String {
        match self.layout {
            EditorLayout::Single => polylines
                .first()
                .and_then(|polyline| serde_json::to_string_pretty(polyline).ok())
                .unwrap_or_default(),
            EditorLayout::Dual => serde_json::to_string_pretty(&serde_json::json!({
                "polyline1": polylines.first(),
                "polyline2": polylines.get(1),
            }))
            .unwrap_or_default(),
            EditorLayout::Multi => serde_json::to_string_pretty(&serde_json::json!({
                "polylines": polylines,
            }))
            .unwrap_or_default(),
        }
    }

    fn parse(&self, json: &str) -> Result<Vec<Polyline>, String> {
        match self.layout {
            EditorLayout::Single => serde_json::from_str(json)
                .map(|polyline| vec![polyline])
                .map_err(|e| format!("failed to parse polyline: {e}")),
            EditorLayout::Dual => {
                let value: serde_json::Value =
                    serde_json::from_str(json).map_err(|e| format!("failed to parse JSON: {e}"))?;
                let first = value.get("polyline1").ok_or("missing polyline1")?.clone();
                let second = value.get("polyline2").ok_or("missing polyline2")?.clone();
                Ok(vec![
                    serde_json::from_value(first)
                        .map_err(|e| format!("failed to parse polyline1: {e}"))?,
                    serde_json::from_value(second)
                        .map_err(|e| format!("failed to parse polyline2: {e}"))?,
                ])
            }
            EditorLayout::Multi => {
                let value: serde_json::Value =
                    serde_json::from_str(json).map_err(|e| format!("failed to parse JSON: {e}"))?;
                serde_json::from_value(value["polylines"].clone())
                    .map_err(|e| format!("failed to parse polylines: {e}"))
            }
        }
    }
}

fn vertex_table(ui: &mut egui::Ui, polyline: &mut Polyline) {
    let mut delete = None;
    egui::Grid::new(ui.next_auto_id())
        .striped(true)
        .num_columns(5)
        .show(ui, |ui| {
            ui.label("Index");
            ui.label("X");
            ui.label("Y");
            ui.label("Bulge");
            ui.label("");
            ui.end_row();
            for (index, vertex) in polyline.vertex_data.iter_mut().enumerate() {
                ui.label(index.to_string());
                ui.add(egui::DragValue::new(&mut vertex.x).speed(0.1));
                ui.add(egui::DragValue::new(&mut vertex.y).speed(0.1));
                ui.add(egui::DragValue::new(&mut vertex.bulge).speed(0.01));
                if ui.button("Delete").clicked() {
                    delete = Some(index);
                }
                ui.end_row();
            }
        });
    if let Some(index) = delete {
        polyline.remove(index);
    }
}

fn add_vertex(polyline: &mut Polyline) {
    let vertex = polyline
        .iter_vertexes()
        .next_back()
        .map(|last| Vertex::new(last.x + 10.0, last.y + 10.0, 0.0))
        .unwrap_or_else(|| Vertex::new(0.0, 0.0, 0.0));
    polyline.add(vertex.x, vertex.y, vertex.bulge);
}

fn ensure_len(polylines: &mut Vec<Polyline>, len: usize) {
    while polylines.len() < len {
        polylines.push(Polyline::new());
    }
}
