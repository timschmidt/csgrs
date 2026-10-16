//! Focused, allocator-instrumented cost of complete STL import and consumption.
use csgrs::{
    GeometryContext, Real, TriangleMesh,
    io::stl,
    solid::{self, SolidExt},
};
use hypermesh::MeshCertainty;
use std::{
    alloc::{GlobalAlloc, Layout, System},
    hint::black_box,
    path::PathBuf,
    sync::atomic::{AtomicUsize, Ordering},
    time::{Duration, Instant},
};

struct Counted;
static CALLS: AtomicUsize = AtomicUsize::new(0);
static BYTES: AtomicUsize = AtomicUsize::new(0);
static LIVE: AtomicUsize = AtomicUsize::new(0);
static PEAK: AtomicUsize = AtomicUsize::new(0);

fn allocated(size: usize) {
    CALLS.fetch_add(1, Ordering::Relaxed);
    BYTES.fetch_add(size, Ordering::Relaxed);
    let live = LIVE.fetch_add(size, Ordering::Relaxed) + size;
    PEAK.fetch_max(live, Ordering::Relaxed);
}

// Every allocation is delegated to System with its original layout. Counters
// observe successful requests without allocating or changing pointer ownership.
unsafe impl GlobalAlloc for Counted {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let ptr = unsafe { System.alloc(layout) };
        if !ptr.is_null() {
            allocated(layout.size());
        }
        ptr
    }
    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        let ptr = unsafe { System.alloc_zeroed(layout) };
        if !ptr.is_null() {
            allocated(layout.size());
        }
        ptr
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        LIVE.fetch_sub(layout.size(), Ordering::Relaxed);
        unsafe { System.dealloc(ptr, layout) };
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, size: usize) -> *mut u8 {
        let next = unsafe { System.realloc(ptr, layout, size) };
        if !next.is_null() {
            LIVE.fetch_sub(layout.size(), Ordering::Relaxed);
            allocated(size);
        }
        next
    }
}

#[global_allocator]
static ALLOCATOR: Counted = Counted;

type Face = [[f32; 3]; 3];

fn faces(cubes: u32) -> Vec<Face> {
    let mut result = Vec::new();
    for index in 0..cubes {
        let x = (3 * index) as f32;
        let p = [
            [x, 0., 0.],
            [x + 1., 0., 0.],
            [x + 1., 1., 0.],
            [x, 1., 0.],
            [x, 0., 1.],
            [x + 1., 0., 1.],
            [x + 1., 1., 1.],
            [x, 1., 1.],
        ];
        for face in [
            [0, 2, 1],
            [0, 3, 2],
            [4, 5, 6],
            [4, 6, 7],
            [0, 4, 7],
            [0, 7, 3],
            [1, 2, 6],
            [1, 6, 5],
            [0, 1, 5],
            [0, 5, 4],
            [3, 7, 6],
            [3, 6, 2],
        ] {
            result.push(face.map(|i| p[i]));
        }
    }
    result
}

fn encode(faces: &[Face], ascii: bool) -> Vec<u8> {
    if ascii {
        let mut result = String::from("solid cost\n");
        for face in faces {
            result.push_str("facet normal 0 0 0\nouter loop\n");
            for [x, y, z] in face {
                result.push_str(&format!("vertex {x:e} {y:e} {z:e}\n"));
            }
            result.push_str("endloop\nendfacet\n");
        }
        result.push_str("endsolid cost\n");
        result.into_bytes()
    } else {
        let mut result = vec![0; 80];
        result.extend_from_slice(&u32::try_from(faces.len()).unwrap().to_le_bytes());
        for face in faces {
            result.extend_from_slice(&[0; 12]);
            for value in face.iter().flatten() {
                result.extend_from_slice(&value.to_le_bytes());
            }
            result.extend_from_slice(&[0; 2]);
        }
        result
    }
}

fn verify(mesh: &TriangleMesh, expected: &[Face], cubes: u32) {
    assert_eq!(mesh.triangles.len(), expected.len());
    if cubes == 0 {
        assert!(mesh.positions.is_empty());
        return;
    }
    let context = GeometryContext::STRICT.mesh_context();
    assert_eq!(
        mesh.validate_closed_pwn(&context).unwrap().certainty,
        MeshCertainty::Certified
    );
    assert!(mesh.is_closed_manifold_geometry(&context).unwrap().value);
    let mut integral = Real::zero();
    for (triangle, face) in mesh.triangles.iter().zip(expected) {
        let points = triangle.indices().map(|index| &mesh.positions[index]);
        for (point, row) in points.into_iter().zip(face) {
            for (value, expected) in [&point.x, &point.y, &point.z].into_iter().zip(row) {
                assert_eq!(value, &Real::try_from(*expected).unwrap());
            }
        }
        let [a, b, c] = points.map(|p| p.to_vector());
        integral += a.dot(&b.cross(&c));
    }
    assert_eq!(integral, Real::from(6 * cubes));
}

fn output_rows(mesh: &TriangleMesh) -> Vec<u8> {
    let mut result = Vec::new();
    result.extend_from_slice(&(mesh.positions.len() as u64).to_le_bytes());
    result.extend_from_slice(&(mesh.triangles.len() as u64).to_le_bytes());
    for point in mesh.positions.iter() {
        for coordinate in [&point.x, &point.y, &point.z] {
            let projection = coordinate.to_f64_lossy().unwrap();
            assert_eq!(coordinate, &Real::try_from(projection).unwrap());
            result.extend_from_slice(&projection.to_le_bytes());
        }
    }
    for triangle in mesh.triangles.iter() {
        for index in triangle.indices() {
            result.extend_from_slice(&(index as u64).to_le_bytes());
        }
    }
    result
}

fn main() {
    let args: Vec<_> = std::env::args().collect();
    let output = PathBuf::from(&args[1]);
    let cubes: u32 = args[2].parse().unwrap();
    let ascii = args[3] == "ascii";
    assert!([0, 1, 256].contains(&cubes));
    let expected = faces(cubes);
    let data = encode(&expected, ascii);
    std::fs::write(output.join("input.stl"), &data).unwrap();
    let iterations = if cubes == 256 { 4 } else { 64 };
    let empty = solid::empty();
    let context = GeometryContext::STRICT.mesh_context();
    let mut outputs = Vec::with_capacity(iterations + 1);
    for (phase, count) in [("first", 1), ("warm", iterations)] {
        let initial_live = LIVE.load(Ordering::Relaxed);
        PEAK.store(initial_live, Ordering::Relaxed);
        CALLS.store(0, Ordering::Relaxed);
        BYTES.store(0, Ordering::Relaxed);
        let mut imports = Duration::ZERO;
        let start = Instant::now();
        for _ in 0..count {
            let before_import = Instant::now();
            let imported = stl::from_stl(black_box(&data)).unwrap();
            imports += before_import.elapsed();
            if cubes != 0 {
                assert_eq!(
                    imported.validate_closed_pwn(&context).unwrap().certainty,
                    MeshCertainty::Certified
                );
            }
            outputs.push(imported.try_union(&empty).unwrap());
        }
        let elapsed = start.elapsed();
        let calls = CALLS.load(Ordering::Relaxed);
        let bytes = BYTES.load(Ordering::Relaxed);
        let peak = PEAK.load(Ordering::Relaxed) - initial_live;
        let live_delta = LIVE.load(Ordering::Relaxed) as i128 - initial_live as i128;
        println!(
            "{{\"cubes\":{cubes},\"ascii\":{ascii},\"phase\":\"{phase}\",\"iterations\":{count},\"elapsed_ns\":{},\"import_ns\":{},\"allocation_calls\":{calls},\"allocated_bytes\":{bytes},\"peak_additional_live_bytes\":{peak},\"retained_delta_bytes\":{live_delta},\"completion\":\"certified\"}}",
            elapsed.as_nanos(),
            imports.as_nanos()
        );
    }
    for mesh in &outputs {
        verify(mesh, &expected, cubes);
    }
    let first = output_rows(&outputs[0]);
    for mesh in &outputs[1..] {
        assert_eq!(output_rows(mesh), first);
    }
    std::fs::write(output.join("output.rows"), first).unwrap();
}
