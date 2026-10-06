#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

mod app;

#[cfg(not(target_arch = "wasm32"))]
fn main() -> eframe::Result {
    env_logger::init();

    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1180.0, 780.0])
            .with_min_inner_size([900.0, 580.0]),
        multisampling: 4,
        ..Default::default()
    };

    eframe::run_native(
        "hypermesh exact CSG demo",
        native_options,
        Box::new(|cc| Ok(Box::new(app::MainApp::new(cc)))),
    )
}

#[cfg(target_arch = "wasm32")]
fn main() {
    use eframe::wasm_bindgen::JsCast as _;

    eframe::WebLogger::init(log::LevelFilter::Debug).ok();

    wasm_bindgen_futures::spawn_local(async {
        let document = web_sys::window()
            .expect("window should exist")
            .document()
            .expect("document should exist");
        let canvas = document
            .get_element_by_id("the_canvas_id")
            .expect("the_canvas_id should exist")
            .dyn_into::<web_sys::HtmlCanvasElement>()
            .expect("the_canvas_id should be a canvas");

        let result = eframe::WebRunner::new()
            .start(
                canvas,
                eframe::WebOptions::default(),
                Box::new(|cc| Ok(Box::new(app::MainApp::new(cc)))),
            )
            .await;

        if let Some(loading) = document.get_element_by_id("loading_text") {
            match result {
                Ok(()) => loading.remove(),
                Err(error) => {
                    loading.set_inner_html("The app failed to start. See the console.");
                    panic!("failed to start eframe: {error:?}");
                }
            }
        }
    });
}
