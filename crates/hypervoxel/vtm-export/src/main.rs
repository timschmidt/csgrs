use std::path::Path;

use voxelis::{
    Lod,
    io::{export::export_model_to_obj, import::import_model_from_vtm},
    world::VoxModel,
};

fn main() {
    #[cfg(feature = "tracy")]
    tracy_client::Client::start();

    #[cfg(feature = "tracy")]
    let _span = tracy_client::span!("vtm-export");

    if std::env::args().len() < 3 {
        eprintln!(
            "Usage: {} <input.vtm> <output.obj>",
            std::env::args().next().unwrap()
        );
        std::process::exit(1);
    }

    let input = std::env::args().nth(1).unwrap();
    let output = std::env::args().nth(2).unwrap();

    let input = Path::new(&input);
    let output = Path::new(&output);

    let name = output.file_stem().unwrap().to_str().unwrap().to_string();

    let model: VoxModel<i32> = import_model_from_vtm(&input, 1024 * 1024 * 1024, None);
    export_model_to_obj(name, &output, &model, Lod::new(0));
}
