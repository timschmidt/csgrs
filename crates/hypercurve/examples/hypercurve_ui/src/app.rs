use crate::scenes::DemoScenes;

pub struct MainApp {
    scenes: DemoScenes,
}

impl MainApp {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        cc.egui_ctx.style_mut(|style| {
            for font_id in style.text_styles.values_mut() {
                font_id.size += 1.0;
            }
        });
        Self {
            scenes: DemoScenes::default(),
        }
    }
}

impl eframe::App for MainApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.scenes.ui(ctx);
    }
}
