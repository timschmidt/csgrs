use csgrs::mesh::Mesh;
use std::fs::File;
use std::io::BufReader;
use std::path::Path;
use std::time::Instant;

fn main() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("benchmarks/data/yeahright/controlmesh.obj");
    for sample in 0..20 {
        let file = File::open(&path).unwrap();
        let start = Instant::now();
        let mesh = Mesh::<()>::from_obj(BufReader::new(file), ()).unwrap();
        println!("{sample},{}", start.elapsed().as_nanos());
        std::hint::black_box(mesh);
    }
}
