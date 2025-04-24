use std::path::Path;

use voxelis::{
    Lod,
    io::{export::export_model_to_obj, import::import_model_from_vtm},
};

fn main() {
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

    let model = import_model_from_vtm(&input);
    export_model_to_obj(name, &output, &model, Lod::new(0));
}
