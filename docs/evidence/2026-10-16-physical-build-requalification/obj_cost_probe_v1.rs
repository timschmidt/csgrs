use csgrs::{GeometryContext, Real, TriangleMesh, io::obj, solid};
use hyperlattice::Point3;
use hypermesh::Triangle;
use std::io::Write;
use std::time::Instant;

fn boxes(count: usize) -> TriangleMesh {
    if count == 0 {
        return solid::empty();
    }
    let cube = solid::cube(Real::one()).unwrap();
    let mut positions = Vec::new();
    let mut triangles = Vec::new();
    for index in 0..count {
        let offset = positions.len();
        let shift = Real::from(u32::try_from(3 * index).unwrap());
        positions.extend(
            cube.positions
                .iter()
                .map(|p| Point3::new(&p.x + &shift, p.y.clone(), p.z.clone())),
        );
        triangles.extend(cube.triangles.iter().map(|triangle| {
            let [a, b, c] = triangle.indices();
            Triangle::new(a + offset, b + offset, c + offset)
        }));
    }
    let mesh = TriangleMesh::new(positions, triangles);
    mesh.validate_closed_pwn(&GeometryContext::STRICT.mesh_context())
        .unwrap();
    let six_volume = mesh.triangles.iter().fold(Real::zero(), |sum, triangle| {
        let [a, b, c] = triangle.indices().map(|i| mesh.positions[i].to_vector());
        sum + a.dot(&b.cross(&c))
    });
    assert_eq!(six_volume, Real::from(u32::try_from(6 * count).unwrap()));
    mesh
}

fn main() {
    let directory = std::path::PathBuf::from(std::env::args_os().nth(1).unwrap());
    assert!(directory.is_dir());
    for (name, components, iterations) in [
        ("empty", 0, 65536),
        ("unit", 1, 4096),
        ("disconnected_256", 256, 32),
    ] {
        let mesh = boxes(components);
        let expected = obj::to_obj(&mesh, "fixture").unwrap();
        let mut file = std::fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(directory.join(format!("{name}.obj")))
            .unwrap();
        file.write_all(expected.as_bytes()).unwrap();
        let mut outputs = Vec::with_capacity(iterations);
        let start = Instant::now();
        for _ in 0..iterations {
            outputs.push(std::hint::black_box(
                obj::to_obj(std::hint::black_box(&mesh), "fixture").unwrap(),
            ));
        }
        let elapsed = start.elapsed().as_nanos();
        assert!(outputs.iter().all(|output| output == &expected));
        let payload_capacity: usize = outputs.iter().map(String::capacity).sum();
        println!(
            "{{\"case\":\"{name}\",\"iterations\":{iterations},\"nanoseconds\":{elapsed},\"output_bytes\":{},\"retained_string_capacity_bytes\":{payload_capacity},\"all_outputs_identical\":true}}",
            expected.len()
        );
    }
}
