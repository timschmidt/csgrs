use hyperdrc::kicad::{BoardModel, load_kicad_pcb};
use hyperdrc::{PcbRegion, PcbRegionExt};
use serde_json::{Value, json};
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

fn geometry(region: &PcbRegion) -> Value {
    match region.try_to_multipolygon() {
        Ok(polygons) => json!({"status":"ok","rings":polygons.0.iter().map(|polygon| {
            std::iter::once(polygon.exterior()).chain(polygon.interiors()).map(|ring| {
                ring.coords().map(|point| [point.x.to_bits(),point.y.to_bits()]).collect::<Vec<_>>()
            }).collect::<Vec<_>>()
        }).collect::<Vec<_>>()}),
        Err(error) => {
            json!({"status":"error","operation":error.operation,"source":error.source,"detail":error.detail})
        },
    }
}
fn inspect(board: &BoardModel) -> Value {
    json!({"copper":board.copper.iter().map(|feature| json!({
        "layer":feature.layer,"net":feature.net,"kind":format!("{:?}",feature.kind),
        "location":feature.location,"geometry":geometry(&feature.region)
    })).collect::<Vec<_>>(),
    "drills":board.drills.iter().map(|drill| json!({"location":drill.location,"diameter":drill.diameter,"net":drill.net,"plated":drill.plated})).collect::<Vec<_>>(),
    "outline":board.board_outline.as_ref().map(geometry),
    "panel":board.panel_features.as_ref().map(geometry)})
}
fn main() {
    let input = std::env::args().nth(1).unwrap();
    let output = std::env::args().nth(2).unwrap();
    let mut rows = Vec::new();
    let mut snapshots = Vec::new();
    for phase in ["fresh_process_load", "repeat_after_projection_and_drop"] {
        let before_live = LIVE.load(Relaxed);
        PEAK.store(before_live, Relaxed);
        let before_calls = CALLS.load(Relaxed);
        let before_bytes = BYTES.load(Relaxed);
        let start = Instant::now();
        let result = load_kicad_pcb(std::path::Path::new(&input));
        let elapsed = start.elapsed().as_nanos();
        let calls = CALLS.load(Relaxed) - before_calls;
        let bytes = BYTES.load(Relaxed) - before_bytes;
        let live = LIVE.load(Relaxed) - before_live;
        let peak = PEAK.load(Relaxed) - before_live;
        let board = result.expect("the unchanged supported workload must still load");
        let snapshot = inspect(&board);
        let projection_errors = snapshot["copper"]
            .as_array()
            .unwrap()
            .iter()
            .filter(|feature| feature["geometry"]["status"] == "error")
            .count();
        rows.push(json!({"phase":phase,"elapsed_ns":elapsed,"allocation_calls":calls,"allocated_bytes":bytes,
            "retained_requested_bytes_delta":live,"peak_extra_requested_bytes":peak,
            "loaded_copper":board.copper.len(),"loaded_drills":board.drills.len(),"copper_projection_errors":projection_errors,
            "completion":"legacy_board_loaded; exact-stroke fidelity not established"}));
        snapshots.push(snapshot);
        drop(board);
    }
    assert_eq!(snapshots[0], snapshots[1]);
    std::fs::write(output, serde_json::to_vec(&snapshots[0]).unwrap()).unwrap();
    println!("{}", serde_json::to_string_pretty(&rows).unwrap());
}
