use std::path::Path;

use voxelis::{
    io::{Obj, export::export_model_to_vtm},
    voxel::voxelizer::Voxelizer,
};

fn main() {
    if std::env::args().len() < 5 {
        eprintln!(
            "Usage: {} <max_depth> <chunk_size_in_m> <input.obj> <output.vtm>",
            std::env::args().next().unwrap()
        );
        std::process::exit(1);
    }

    let max_depth = std::env::args().nth(1).unwrap();
    let chunk_size = std::env::args().nth(2).unwrap();
    let input = std::env::args().nth(3).unwrap();
    let output = std::env::args().nth(4).unwrap();

    let max_depth: usize = max_depth.parse().unwrap();
    let chunk_size: f32 = chunk_size.parse().unwrap();

    println!("Max octree depth: {}", max_depth);
    println!("Voxels per axis: {}", 1 << max_depth);
    println!("Chunk size: {}m", chunk_size);

    let input = Path::new(&input);
    let output = Path::new(&output);

    let name = output.file_stem().unwrap().to_str().unwrap().to_string();

    let obj = Obj::parse(&input);

    let mut voxelizer = Voxelizer::new(max_depth, chunk_size, obj);
    voxelizer.voxelize();

    export_model_to_vtm(name, &output, &voxelizer.model);
}
