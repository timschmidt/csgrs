//! Fixed consumer costs for immutable facts and safe public buffer edits.
use hypermesh::{
    BooleanOp, BooleanProgram, MeshCertainty, MeshContext, Point3, PredicatePolicy, Real,
    Triangle, TriangleMesh, boolean, polygon_soup,
};
use std::{hint::black_box, num::NonZeroU32, sync::Arc, time::Instant};
const STRICT: MeshContext = MeshContext::new(PredicatePolicy::STRICT);
#[cfg(feature = "count-alloc")]
mod allocation {
    use std::alloc::{GlobalAlloc, Layout, System};
    use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};
    pub struct Counted;
    static LIVE: AtomicUsize = AtomicUsize::new(0);
    static PEAK: AtomicUsize = AtomicUsize::new(0);
    static CALLS: AtomicUsize = AtomicUsize::new(0);
    static BYTES: AtomicUsize = AtomicUsize::new(0);
    fn allocated(bytes: usize) {
        CALLS.fetch_add(1, Relaxed);
        BYTES.fetch_add(bytes, Relaxed);
        PEAK.fetch_max(LIVE.fetch_add(bytes, Relaxed) + bytes, Relaxed);
    }
    unsafe impl GlobalAlloc for Counted {
        unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
            let result = unsafe { System.alloc(layout) };
            if !result.is_null() {
                allocated(layout.size());
            }
            result
        }
        unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
            LIVE.fetch_sub(layout.size(), Relaxed);
            unsafe { System.dealloc(pointer, layout) };
        }
        unsafe fn realloc(&self, pointer: *mut u8, layout: Layout, size: usize) -> *mut u8 {
            let result = unsafe { System.realloc(pointer, layout, size) };
            if !result.is_null() {
                LIVE.fetch_sub(layout.size(), Relaxed);
                allocated(size);
            }
            result
        }
    }
    pub fn start() -> usize {
        let live = LIVE.load(Relaxed);
        PEAK.store(live, Relaxed);
        CALLS.store(0, Relaxed);
        BYTES.store(0, Relaxed);
        live
    }
    pub fn finish(initial: usize) -> (usize, usize, i128, usize) {
        (
            CALLS.load(Relaxed),
            BYTES.load(Relaxed),
            LIVE.load(Relaxed) as i128 - initial as i128,
            PEAK.load(Relaxed) - initial,
        )
    }
}
#[cfg(feature = "count-alloc")]
#[global_allocator]
static ALLOCATOR: allocation::Counted = allocation::Counted;

fn measure<T>(name: &str, count: usize, body: impl FnOnce() -> T) -> T {
    #[cfg(feature = "count-alloc")]
    let initial = allocation::start();
    let start = Instant::now();
    let result = body();
    let nanos = start.elapsed().as_nanos();
    #[cfg(feature = "count-alloc")]
    let (calls, bytes, retained, peak) = allocation::finish(initial);
    #[cfg(not(feature = "count-alloc"))]
    let (calls, bytes, retained, peak) = (0, 0, 0, 0);
    println!(
        "{{\"name\":\"{name}\",\"iterations\":{count},\"nanos\":{nanos},\"allocation_accounting\":{},\"allocation_calls\":{calls},\"allocated_bytes\":{bytes},\"retained_bytes_delta\":{retained},\"peak_bytes_delta\":{peak}}}",
        cfg!(feature = "count-alloc")
    );
    black_box(result)
}

fn cube(min: i32, max: i32) -> TriangleMesh {
    let point = |x, y, z| Point3::new(Real::from(x), Real::from(y), Real::from(z));
    TriangleMesh::new(
        vec![
            point(min, min, min),
            point(max, min, min),
            point(max, max, min),
            point(min, max, min),
            point(min, min, max),
            point(max, min, max),
            point(max, max, max),
            point(min, max, max),
        ],
        vec![
            Triangle::new(4, 5, 6),
            Triangle::new(4, 6, 7),
            Triangle::new(0, 3, 2),
            Triangle::new(0, 2, 1),
            Triangle::new(1, 2, 6),
            Triangle::new(1, 6, 5),
            Triangle::new(0, 4, 7),
            Triangle::new(0, 7, 3),
            Triangle::new(3, 7, 6),
            Triangle::new(3, 6, 2),
            Triangle::new(0, 1, 5),
            Triangle::new(0, 5, 4),
        ],
    )
}

fn witness(name: &str, value: impl std::fmt::Debug) {
    let text = format!("{value:?}");
    let digest = text.bytes().fold(0xcbf29ce484222325_u64, |hash, byte| {
        (hash ^ u64::from(byte)).wrapping_mul(0x100000001b3)
    });
    println!(
        "{{\"witness\":\"{name}\",\"bytes\":{},\"fnv64\":{digest}}}",
        text.len()
    );
}
fn mesh_witness(name: &str, mesh: &TriangleMesh) {
    witness(name, (&mesh.positions, &mesh.triangles));
}
fn fresh_meshes(count: usize) -> Vec<TriangleMesh> {
    (0..count).map(|_| cube(0, 1)).collect()
}
fn main() {
    let count: usize = std::env::args().nth(1).unwrap().parse().unwrap();
    assert!(count > 0);
    // Compatibility metadata is intentionally separate from exact geometry witnesses.
    let mut compatibility = cube(0, 1);
    let positions_get_mut = Arc::get_mut(&mut compatibility.positions).is_some();
    let triangles_get_mut = Arc::get_mut(&mut compatibility.triangles).is_some();
    println!(
        "{{\"compatibility\":true,\"positions_get_mut\":{positions_get_mut},\"triangles_get_mut\":{triangles_get_mut},\"mesh_bytes\":{}}}",
        std::mem::size_of::<TriangleMesh>()
    );
    drop(compatibility);
    let empty = measure("retain_empty", count, || {
        (0..count)
            .map(|_| TriangleMesh::new(vec![], vec![]))
            .collect::<Vec<_>>()
    });
    witness(
        "retain_empty",
        (
            empty.len(),
            empty[0].positions.len(),
            empty[0].triangles.len(),
        ),
    );
    drop(empty);
    let meshes = measure("retain_cube", count, || fresh_meshes(count));
    mesh_witness("retain_cube", &meshes[count - 1]);
    drop(meshes);
    let meshes = fresh_meshes(count);
    let last = measure("cold_bounds", count, || {
        let mut last = None;
        for mesh in &meshes {
            last = Some(black_box(mesh.exact_bounds(black_box(&STRICT)).unwrap()));
        }
        last.unwrap()
    });
    assert_eq!(last.certainty, MeshCertainty::Certified);
    witness("cold_bounds", last);
    let last = measure("warm_bounds", count * 16, || {
        let mut last = None;
        for _ in 0..16 {
            for mesh in &meshes {
                last = Some(black_box(mesh.exact_bounds(black_box(&STRICT)).unwrap()));
            }
        }
        last.unwrap()
    });
    witness("warm_bounds", last);
    drop(meshes);
    let meshes = fresh_meshes(count);
    let last = measure("cold_box", count, || {
        let mut last = None;
        for mesh in &meshes {
            last = Some(black_box(
                mesh.axis_aligned_box_bounds(black_box(&STRICT)).unwrap(),
            ));
        }
        last.unwrap()
    });
    assert_eq!(last.certainty, MeshCertainty::Certified);
    assert!(last.value.is_some());
    witness("cold_box", last);
    let last = measure("warm_box", count * 16, || {
        let mut last = None;
        for _ in 0..16 {
            for mesh in &meshes {
                last = Some(black_box(
                    mesh.axis_aligned_box_bounds(black_box(&STRICT)).unwrap(),
                ));
            }
        }
        last.unwrap()
    });
    witness("warm_box", last);
    drop(meshes);
    let meshes = fresh_meshes(count);
    for phase in ["cold_soup", "warm_soup"] {
        let last = measure(phase, count, || {
            let mut last = None;
            for mesh in &meshes {
                last = Some(black_box(
                    polygon_soup(black_box(&STRICT), black_box(&[mesh.as_ref()])).unwrap(),
                ));
            }
            last.unwrap()
        });
        assert_eq!(last.certainty, MeshCertainty::Certified);
        witness(
            phase,
            (
                last.certainty,
                &last.value.bounds,
                last.value
                    .polygons
                    .iter()
                    .map(|p| (p.vertices(&STRICT).unwrap(), p.support_plane()))
                    .collect::<Vec<_>>(),
            ),
        );
    }
    let mesh = &meshes[0];
    let reversed = measure("reversed", count, || {
        (0..count)
            .map(|_| black_box(mesh.reversed_winding()))
            .collect::<Vec<_>>()
    });
    mesh_witness("reversed", &reversed[count - 1]);
    drop(reversed);
    let subdivided = measure("subdivided", count, || {
        (0..count)
            .map(|_| black_box(mesh.subdivide_triangles(NonZeroU32::new(1).unwrap()).unwrap()))
            .collect::<Vec<_>>()
    });
    mesh_witness("subdivided", &subdivided[count - 1]);
    drop(subdivided);
    drop(meshes);
    let left = cube(0, 2);
    let right = cube(1, 3);
    let unions = (count / 16).max(1);
    let last = measure("union", unions, || {
        let mut last = None;
        for _ in 0..unions {
            last = Some(black_box(
                boolean(
                    black_box(&STRICT),
                    black_box(&[left.as_ref(), right.as_ref()]),
                    BooleanProgram::Operation(BooleanOp::Union),
                )
                .unwrap(),
            ));
        }
        last.unwrap()
    });
    assert_eq!(last.certainty, MeshCertainty::Certified);
    witness(
        "union",
        (last.certainty, &last.value.vertices, &last.value.results),
    );
    drop(last);
    for phase in ["replace_positions", "make_mut_positions"] {
        let mut meshes = fresh_meshes(count);
        measure(phase, count, || {
            for mesh in &mut meshes {
                if phase == "replace_positions" {
                    mesh.positions = mesh
                        .positions
                        .iter()
                        .cloned()
                        .map(|mut p| {
                            p.x = &p.x + Real::from(10);
                            p
                        })
                        .collect::<Vec<_>>()
                        .into();
                } else {
                    for p in Arc::make_mut(&mut mesh.positions) {
                        p.x = &p.x + Real::from(10);
                    }
                }
                black_box(&*mesh);
            }
        });
        mesh_witness(phase, &meshes[count - 1]);
        let release = if phase == "replace_positions" {
            "drop_replaced"
        } else {
            "drop_make_mut"
        };
        measure(release, count, || drop(meshes));
        witness(release, count);
    }
}
