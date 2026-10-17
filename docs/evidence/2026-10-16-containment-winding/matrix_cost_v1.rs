//! Completed-contract cost observations for independently specified regularized sets.
use hypermesh::{
    MeshCertainty as GeometryCertainty, MeshContext as GeometryContext, PredicatePolicy, Real,
    TriangleMesh,
};
use hypermesh::{Point3, Triangle};
use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicIsize, AtomicUsize, Ordering::Relaxed};
use std::time::Instant;
struct Counting;
static CALLS: AtomicUsize = AtomicUsize::new(0);
static BYTES: AtomicUsize = AtomicUsize::new(0);
static LIVE: AtomicIsize = AtomicIsize::new(0);
static PEAK: AtomicIsize = AtomicIsize::new(0);
fn allocated(n: usize) {
    CALLS.fetch_add(1, Relaxed);
    BYTES.fetch_add(n, Relaxed);
    let live = LIVE.fetch_add(n as isize, Relaxed) + n as isize;
    PEAK.fetch_max(live, Relaxed);
}
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let p = unsafe { System.alloc(layout) };
        if !p.is_null() {
            allocated(layout.size());
        }
        p
    }
    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        let p = unsafe { System.alloc_zeroed(layout) };
        if !p.is_null() {
            allocated(layout.size());
        }
        p
    }
    unsafe fn dealloc(&self, p: *mut u8, layout: Layout) {
        unsafe { System.dealloc(p, layout) };
        LIVE.fetch_sub(layout.size() as isize, Relaxed);
    }
    unsafe fn realloc(&self, p: *mut u8, layout: Layout, n: usize) -> *mut u8 {
        let q = unsafe { System.realloc(p, layout, n) };
        if !q.is_null() {
            LIVE.fetch_sub(layout.size() as isize, Relaxed);
            allocated(n);
        }
        q
    }
}
#[global_allocator]
static ALLOC: Counting = Counting;
#[derive(Clone, Copy)]
struct BoxInput {
    min: [i32; 3],
    max: [i32; 3],
    orientation: i32,
}

fn box_input(min: [i32; 3], max: [i32; 3], orientation: i32) -> BoxInput {
    assert!(orientation == 1 || orientation == -1);
    BoxInput {
        min,
        max,
        orientation,
    }
}

fn mesh(boxes: &[BoxInput]) -> TriangleMesh {
    let mut positions = Vec::new();
    let mut triangles = Vec::new();
    for input in boxes {
        let [a, b, c] = input.min;
        let [x, y, z] = input.max;
        let base = positions.len();
        positions.extend(
            [
                [a, b, c],
                [x, b, c],
                [x, y, c],
                [a, y, c],
                [a, b, z],
                [x, b, z],
                [x, y, z],
                [a, y, z],
            ]
            .map(|p| Point3::new(Real::from(p[0]), Real::from(p[1]), Real::from(p[2]))),
        );
        for mut face in [
            [4, 5, 6],
            [4, 6, 7],
            [0, 3, 2],
            [0, 2, 1],
            [1, 2, 6],
            [1, 6, 5],
            [0, 4, 7],
            [0, 7, 3],
            [3, 7, 6],
            [3, 6, 2],
            [0, 1, 5],
            [0, 5, 4],
        ] {
            if input.orientation < 0 {
                face.reverse();
            }
            triangles.push(Triangle::new(base + face[0], base + face[1], base + face[2]));
        }
    }
    TriangleMesh::new(positions, triangles)
}

fn point(twice: [i32; 3]) -> Point3 {
    let [x, y, z] = twice.map(|value| (Real::from(value) / Real::from(2)).unwrap());
    Point3::new(x, y, z)
}

fn interior(boxes: &[BoxInput], twice: [i32; 3]) -> bool {
    // A radius < 1/2 cannot cross another integer support plane from this
    // half-integer query. Nonzero winding in all eight adjacent open octants
    // therefore proves strict interior, including internal faces/edges/vertices.
    [-1, 1].into_iter().all(|dx| {
        [-1, 1].into_iter().all(|dy| {
            [-1, 1].into_iter().all(|dz| {
                let sample = [4 * twice[0] + dx, 4 * twice[1] + dy, 4 * twice[2] + dz];
                let winding: i32 = boxes
                    .iter()
                    .filter(|input| {
                        (0..3).all(|axis| {
                            8 * input.min[axis] < sample[axis]
                                && sample[axis] < 8 * input.max[axis]
                        })
                    })
                    .map(|input| input.orientation)
                    .sum();
                winding != 0
            })
        })
    })
}



struct Case {
    name: &'static str,
    boxes: Vec<BoxInput>,
    twice: [i32;3],
    pi_translation: bool,
    flat: bool,
}
impl Case {
    fn point(&self) -> Point3 {
        let mut p = point(self.twice);
        if self.pi_translation { p.x = p.x + Real::pi(); }
        p
    }
    fn source(&self) -> TriangleMesh {
        let input = if self.flat {
            TriangleMesh::new(vec![point([0,0,0]),point([4,0,0]),point([0,4,0])],vec![Triangle::new(0,1,2),Triangle::new(2,1,0)])
        } else { mesh(&self.boxes) };
        if self.pi_translation {
            TriangleMesh::new(input.positions.iter().map(|p|Point3::new(p.x.clone()+Real::pi(),p.y.clone(),p.z.clone())).collect(),input.triangles.to_vec())
        } else { input }
    }
    fn expected(&self) -> bool { !self.flat && interior(&self.boxes,self.twice) }
}
fn cases() -> Vec<Case> {
    let mut out = Vec::new();
    let unit=box_input([0,0,0],[2,2,2],1);
    let outer=box_input([0,0,0],[3,3,3],1);
    let inner=box_input([1,1,1],[2,2,2],1);
    for (name,boxes,twice) in [
        ("simple_inside",vec![unit],[1,1,1]),
        ("simple_face",vec![unit],[0,1,1]),
        ("simple_vertex",vec![unit],[0,0,0]),
        ("simple_outside",vec![unit],[5,1,1]),
        ("nested_inside",vec![outer,inner],[3,3,3]),
        ("nested_internal_face",vec![outer,inner],[2,3,3]),
        ("nested_internal_edge",vec![outer,inner],[2,2,3]),
        ("nested_internal_vertex",vec![outer,inner],[2,2,2]),
        ("opposite_inside",vec![unit,box_input(unit.min,unit.max,-1)],[1,1,1]),
        ("opposite_face",vec![unit,box_input(unit.min,unit.max,-1)],[0,1,1]),
        ("overlap_inside",vec![unit,box_input([1,0,0],[3,2,2],1)],[3,1,1]),
        ("shared_face",vec![box_input([0,0,0],[1,2,2],1),box_input([1,0,0],[2,2,2],1)],[2,1,1]),
        ("cavity_inside",vec![outer,box_input(inner.min,inner.max,-1)],[3,3,3]),
        ("cavity_face",vec![outer,box_input(inner.min,inner.max,-1)],[2,3,3]),
        ("tangent_origin",vec![box_input([16,6,4],[17,7,5],1),box_input([-2,-2,-2],[-1,-1,-1],1)],[0,0,0]),
        ("reversed_multiplicity",vec![box_input(unit.min,unit.max,-1),box_input(unit.min,unit.max,-1)],[1,1,1]),
    ] { out.push(Case{name,boxes,twice,pi_translation:false,flat:false}); }
    for (name,twice) in [("symbolic_inside",[1,1,1]),("symbolic_face",[0,1,1]),("symbolic_vertex",[0,0,0]),("symbolic_outside",[5,1,1])] {
        out.push(Case{name,boxes:vec![unit],twice,pi_translation:true,flat:false});
    }
    out.push(Case{name:"flat_face",boxes:Vec::new(),twice:[1,1,0],pi_translation:false,flat:true});
    out
}
fn observation(mesh: &TriangleMesh, context: &GeometryContext, query: &Point3, expected: bool) -> String {
    match mesh.contains_point(context,query) {
        Ok(outcome) => {
            let correct=outcome.value==expected;
            let status=match (correct,outcome.certainty) {
                (true,GeometryCertainty::Certified)=>"certified_completion",
                (true,_)=>"qualified_approximate_completion",
                (false,_)=>"incorrect_result",
            };
            format!("{{\"status\":\"{status}\",\"value\":{},\"certainty\":\"{:?}\"}}",outcome.value,outcome.certainty)
        }
        Err(error) => {
            let status=if matches!(error,hypermesh::HypermeshError::PredicateUndecided{..}) {"undecided"} else {"typed_failure"};
            // Debug descriptions contain quotes; serialize their bytes by Rust
            // debug-string formatting, which is JSON-compatible for these errors.
            format!("{{\"status\":\"{status}\",\"error\":{:?}}}",format!("{error:?}"))
        }
    }
}
fn main() {
    let specs=cases();
    let mut rows=Vec::new();
    for (policy,context) in [("strict",GeometryContext::new(PredicatePolicy::STRICT)),("approximate",GeometryContext::new(PredicatePolicy::APPROXIMATE_512))] {
        for case in &specs {
            let expected=case.expected();
            let query=case.point();
            let mut meshes=Vec::new();
            for phase in ["cold_construct_query","warm_query"] {
                let before_live=LIVE.load(Relaxed);PEAK.store(before_live,Relaxed);
                let before_calls=CALLS.load(Relaxed);let before_bytes=BYTES.load(Relaxed);
                let start=Instant::now();let mut observations=Vec::new();
                if phase=="cold_construct_query" {
                    for _ in 0..8 { let source=case.source();observations.push(observation(&source,&context,&query,expected));meshes.push(source); }
                } else {
                    for source in &meshes { observations.push(observation(source,&context,&query,expected)); }
                }
                let elapsed=start.elapsed().as_nanos();
                let calls=CALLS.load(Relaxed)-before_calls;let bytes=BYTES.load(Relaxed)-before_bytes;
                let retained=LIVE.load(Relaxed)-before_live;let peak=PEAK.load(Relaxed)-before_live;
                rows.push(format!("{{\"case\":\"{}\",\"policy\":\"{policy}\",\"phase\":\"{phase}\",\"expected\":{expected},\"attempted\":8,\"elapsed_ns\":{elapsed},\"allocation_calls\":{calls},\"allocated_bytes\":{bytes},\"retained_requested_bytes_delta\":{retained},\"peak_extra_requested_bytes\":{peak},\"observations\":[{}]}}",case.name,observations.join(",")));
            }
            let before=LIVE.load(Relaxed);drop(meshes);let released=before-LIVE.load(Relaxed);
            rows.push(format!("{{\"case\":\"{}\",\"policy\":\"{policy}\",\"phase\":\"drop_meshes\",\"released_requested_bytes\":{released}}}",case.name));
        }
    }
    println!("[{}]",rows.join(","));
}
