use std::hint::black_box;

use hypermesh::TriangleMesh;

fn main() {
    let mesh_count = std::env::args()
        .nth(1)
        .and_then(|value| value.parse::<usize>().ok())
        .unwrap_or(100_000);
    let mut meshes = Vec::with_capacity(mesh_count);
    for _ in 0..mesh_count {
        meshes.push(TriangleMesh::new(Vec::new(), Vec::new()));
    }
    println!("{} cold mesh carriers", black_box(meshes).len());
}
