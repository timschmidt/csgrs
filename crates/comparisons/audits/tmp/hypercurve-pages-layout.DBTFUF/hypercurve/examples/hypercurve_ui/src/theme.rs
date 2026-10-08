use egui::Color32;

/// Shared colors for the hypercurve UI test article.
#[derive(Clone, Copy, Debug)]
pub struct Theme {
    pub primary: Color32,
    pub secondary: Color32,
    pub accent: Color32,
    pub result: Color32,
    pub warning: Color32,
    pub error: Color32,
    pub vertex: Color32,
}

impl Theme {
    pub fn for_context(ctx: &egui::Context) -> Self {
        if ctx.style().visuals.dark_mode {
            Self {
                primary: Color32::from_rgb(94, 171, 255),
                secondary: Color32::from_rgb(255, 139, 148),
                accent: Color32::from_rgb(210, 218, 229),
                result: Color32::from_rgb(130, 218, 142),
                warning: Color32::from_rgb(245, 198, 92),
                error: Color32::from_rgb(255, 99, 99),
                vertex: Color32::from_rgb(255, 236, 150),
            }
        } else {
            Self {
                primary: Color32::from_rgb(0, 92, 184),
                secondary: Color32::from_rgb(194, 62, 76),
                accent: Color32::from_rgb(44, 50, 60),
                result: Color32::from_rgb(39, 139, 66),
                warning: Color32::from_rgb(181, 124, 28),
                error: Color32::from_rgb(198, 38, 38),
                vertex: Color32::from_rgb(140, 94, 0),
            }
        }
    }
}
