use std::path::Path;

use voxelis::{io::export::export_model_to_vtm, obj_reader::Obj, voxelizer::Voxelizer};

fn main() {
    if std::env::args().len() < 3 {
        eprintln!(
            "Usage: {} <input.obj> <output.vtm>",
            std::env::args().next().unwrap()
        );
        std::process::exit(1);
    }

    let input = std::env::args().nth(1).unwrap();
    let output = std::env::args().nth(2).unwrap();

    let input = Path::new(&input);
    let output = Path::new(&output);

    let name = output.file_stem().unwrap().to_str().unwrap().to_string();

    let obj = Obj::parse(&input);

    let mut voxelizer = Voxelizer::new(obj);
    voxelizer.voxelize();

    export_model_to_vtm(name, &output, &voxelizer.model);
}
