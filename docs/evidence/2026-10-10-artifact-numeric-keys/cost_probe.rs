use hyperdrc::checks::{TextArtifact, production_artifact_readiness};
use serde_json::json;
use std::hint::black_box;
use std::time::Instant;

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

fn main() {
    let iterations: usize = std::env::args().nth(1).unwrap().parse().unwrap();
    for family in [
        "ordinary",
        "coincident",
        "near-placements",
        "near-rotations",
        "rational",
        "extremes",
    ] {
        let mut bom = String::from("Ref,Qty,MPN,Rotation\n");
        let mut centroid = String::from(
            "# units mm; origin aux origin; rotation convention clockwise degrees\nRef,X,Y,Rotation,Side\n",
        );
        for index in 0..256 {
            let (x, y, left, right) = match family {
                "ordinary" => (
                    index.to_string(),
                    "0".to_string(),
                    (index % 4 * 90).to_string(),
                    (index % 4 * 90 + 360).to_string(),
                ),
                "coincident" => (
                    (index / 2).to_string(),
                    "0".to_string(),
                    "0".to_string(),
                    "0".to_string(),
                ),
                "near-placements" => (
                    format!("10.{index:08}"),
                    "0".to_string(),
                    "0".to_string(),
                    "0".to_string(),
                ),
                "near-rotations" => (
                    index.to_string(),
                    "0".to_string(),
                    format!("10.{index:08}"),
                    format!("10.{:08}", index + 1),
                ),
                "rational" => (
                    format!("{index}/7"),
                    format!("{index}/11"),
                    format!("{index}/7"),
                    format!("{}/7", index + 2520),
                ),
                "extremes" => (
                    format!("{}e40", index + 1),
                    format!("{}e-40", index + 1),
                    "1e-400".to_string(),
                    "2e-400".to_string(),
                ),
                _ => unreachable!(),
            };
            bom.push_str(&format!("R{},1,RC0603,{left}\n", index + 1));
            centroid.push_str(&format!("R{},{x},{y},{right},Top\n", index + 1));
        }
        let bom = [TextArtifact {
            path: "bom.csv".to_owned(),
            text: bom,
        }];
        let centroid = [TextArtifact {
            path: "centroid.csv".to_owned(),
            text: centroid,
        }];
        let query = || {
            production_artifact_readiness(
                black_box(&bom),
                black_box(&centroid),
                &[],
                &[],
                &[],
                &[],
                &[],
            )
        };
        let witness = |values: &[hyperdrc::report::Violation]| {
            let count = |needle: &str| {
                values
                    .iter()
                    .filter(|v| v.message.as_ref().is_some_and(|m| m.contains(needle)))
                    .count()
            };
            [
                values.len(),
                count("places multiple references"),
                count("BOM rotation for"),
            ]
        };
        let expected = witness(&query());
        #[cfg(feature = "count-alloc")]
        let allocation_start = allocation::start();
        let start = Instant::now();
        for _ in 0..iterations {
            let result = query();
            assert_eq!(witness(&result), expected);
            black_box(result);
        }
        let nanos = start.elapsed().as_nanos();
        #[cfg(feature = "count-alloc")]
        let allocations = allocation::finish(allocation_start);
        #[cfg(not(feature = "count-alloc"))]
        let allocations = (0_usize, 0_usize, 0_i128, 0_usize);
        println!(
            "{}",
            json!({"family":family,"iterations":iterations,"rows_per_artifact":256,"nanos":nanos,"allocations":allocations,"witness":expected})
        );
    }
}
