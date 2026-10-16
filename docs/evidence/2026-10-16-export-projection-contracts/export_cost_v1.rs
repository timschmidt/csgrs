//! Complete export materialization, with independent integer-box semantics.
use csgrs::{
    GeometryContext, Real, TriangleMesh,
    io::{obj, stl},
};
use hyperlattice::Point3;
use hypermesh::{MeshCertainty, Triangle};
use std::{
    alloc::{GlobalAlloc, Layout, System},
    hint::black_box,
    path::PathBuf,
    sync::atomic::{AtomicUsize, Ordering},
    time::Instant,
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
    // Boolean regularization may retriangulate or reorder faces. Establish
    // the independently specified set instead: every face lies on one unit
    // box boundary, points outward, and each closed component has volume one.
    let mut six_volumes = vec![0_i64; cubes as usize];
    for triangle in mesh.triangles.iter() {
        let points = triangle.indices().map(|index| {
            let point = &mesh.positions[index];
            [&point.x, &point.y, &point.z].map(|coordinate| {
                let integer = coordinate.to_f64_lossy().unwrap() as i64;
                assert_eq!(coordinate, &Real::from(integer));
                integer
            })
        });
        assert!(points[0][0] >= 0);
        let component = (points[0][0] / 3) as usize;
        assert!(component < cubes as usize);
        let lower = [(3 * component) as i64, 0, 0];
        for point in points {
            for axis in 0..3 {
                assert!(point[axis] == lower[axis] || point[axis] == lower[axis] + 1);
            }
        }
        let (axis, upper) = (0..3)
            .find_map(|axis| {
                if points.iter().all(|p| p[axis] == lower[axis]) {
                    Some((axis, false))
                } else if points.iter().all(|p| p[axis] == lower[axis] + 1) {
                    Some((axis, true))
                } else {
                    None
                }
            })
            .expect("every triangle lies on its intended box boundary");
        let [a, b, c] = points;
        let u: [i64; 3] = std::array::from_fn(|i| b[i] - a[i]);
        let v: [i64; 3] = std::array::from_fn(|i| c[i] - a[i]);
        let normal = [
            u[1] * v[2] - u[2] * v[1],
            u[2] * v[0] - u[0] * v[2],
            u[0] * v[1] - u[1] * v[0],
        ];
        assert!(if upper {
            normal[axis] > 0
        } else {
            normal[axis] < 0
        });
        // Integer coordinates are at most 766, so these independent integer
        // products and sums fit i64 with ample margin.
        six_volumes[component] += a[0] * (b[1] * c[2] - b[2] * c[1])
            + a[1] * (b[2] * c[0] - b[0] * c[2])
            + a[2] * (b[0] * c[1] - b[1] * c[0]);
    }
    assert!(six_volumes.into_iter().all(|volume| volume == 6));
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
    let encoding = &args[3];
    assert!([0, 1, 256].contains(&cubes));
    assert!(["binary", "ascii", "obj"].contains(&encoding.as_str()));
    let expected = faces(cubes);
    let positions = expected
        .iter()
        .flatten()
        .map(|point| {
            Point3::new(
                Real::try_from(point[0]).unwrap(),
                Real::try_from(point[1]).unwrap(),
                Real::try_from(point[2]).unwrap(),
            )
        })
        .collect();
    let triangles = (0..expected.len())
        .map(|face| Triangle::new(3 * face, 3 * face + 1, 3 * face + 2))
        .collect();
    let mesh = TriangleMesh::new(positions, triangles);
    // Recording scalar input rows precedes measurement. The binary export cache
    // starts empty in this fresh process; scalar ingress is not timed.
    std::fs::write(output.join("input.rows"), output_rows(&mesh)).unwrap();
    let iterations = if cubes == 256 { 4 } else { 64 };
    let mut outputs = Vec::with_capacity(iterations + 1);
    for (phase, count) in [("first", 1), ("warm", iterations)] {
        let initial_live = LIVE.load(Ordering::Relaxed);
        PEAK.store(initial_live, Ordering::Relaxed);
        CALLS.store(0, Ordering::Relaxed);
        BYTES.store(0, Ordering::Relaxed);
        let start = Instant::now();
        for _ in 0..count {
            let bytes = match encoding.as_str() {
                "binary" => stl::to_stl_binary(black_box(&mesh), "cost").unwrap(),
                "ascii" => stl::to_stl_ascii(black_box(&mesh), "cost")
                    .unwrap()
                    .into_bytes(),
                "obj" => obj::to_obj(black_box(&mesh), "cost").unwrap().into_bytes(),
                _ => unreachable!(),
            };
            outputs.push(bytes);
        }
        let elapsed = start.elapsed();
        let calls = CALLS.load(Ordering::Relaxed);
        let bytes = BYTES.load(Ordering::Relaxed);
        let peak = PEAK.load(Ordering::Relaxed) - initial_live;
        let live_delta = LIVE.load(Ordering::Relaxed) as i128 - initial_live as i128;
        let last = outputs.last().unwrap();
        let output_bytes = last.len();
        let output_capacity = last.capacity();
        println!(
            "{{\"cubes\":{cubes},\"encoding\":\"{encoding}\",\"phase\":\"{phase}\",\"iterations\":{count},\"elapsed_ns\":{},\"allocation_calls\":{calls},\"allocated_bytes\":{bytes},\"peak_additional_live_bytes\":{peak},\"retained_delta_bytes\":{live_delta},\"output_bytes\":{output_bytes},\"output_capacity\":{output_capacity},\"completion\":\"verified_exact_integer_projection\"}}",
            elapsed.as_nanos()
        );
    }
    verify(&mesh, &expected, cubes);
    for bytes in &outputs[1..] {
        assert_eq!(bytes, &outputs[0]);
    }
    // Validation and independent integer boundary/volume checks are untimed;
    // every timed export includes its own native checks and all output bytes.
    let restored = match encoding.as_str() {
        "obj" => obj::from_obj(outputs[0].as_slice()).unwrap(),
        _ => stl::from_stl(&outputs[0]).unwrap(),
    };
    verify(&restored, &expected, cubes);
    std::fs::write(output.join("output.data"), &outputs[0]).unwrap();
    std::fs::write(output.join("reimport.rows"), output_rows(&restored)).unwrap();
}
