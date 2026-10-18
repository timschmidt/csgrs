//! Guard retained report payload for repeated native aggregation.
//! The allocator counts requested Rust bytes, not resident memory or allocator overhead.

use hyperdrc::geometry::{empty_profile, polygon_from_points, polygon_to_profile};
use hyperdrc::kicad::{BoardModel, CopperFeature, CopperKind};
use hyperdrc::{PcbRegion, PcbRegionExt, Scalar};
use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicUsize, Ordering};

struct Counted;
static LIVE: AtomicUsize = AtomicUsize::new(0);

// SAFETY: Every allocation operation delegates to System with the original
// pointer and layout. The counter neither reads nor changes allocated storage.
unsafe impl GlobalAlloc for Counted {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let pointer = unsafe { System.alloc(layout) };
        if !pointer.is_null() {
            LIVE.fetch_add(layout.size(), Ordering::Relaxed);
        }
        pointer
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        let pointer = unsafe { System.alloc_zeroed(layout) };
        if !pointer.is_null() {
            LIVE.fetch_add(layout.size(), Ordering::Relaxed);
        }
        pointer
    }

    unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
        LIVE.fetch_sub(layout.size(), Ordering::Relaxed);
        unsafe { System.dealloc(pointer, layout) };
    }

    unsafe fn realloc(&self, pointer: *mut u8, layout: Layout, size: usize) -> *mut u8 {
        let pointer = unsafe { System.realloc(pointer, layout, size) };
        if !pointer.is_null() {
            LIVE.fetch_sub(layout.size(), Ordering::Relaxed);
            LIVE.fetch_add(size, Ordering::Relaxed);
        }
        pointer
    }
}

#[global_allocator]
static ALLOCATOR: Counted = Counted;

fn feature(region: PcbRegion) -> CopperFeature {
    CopperFeature {
        layer: "F.Cu".into(),
        net: None,
        kind: CopperKind::Pad,
        region,
        location: [Scalar::zero(), Scalar::zero()],
    }
}

fn retained_report_bytes(count: usize) -> usize {
    let before = LIVE.load(Ordering::Relaxed);
    let original = polygon_to_profile(
        polygon_from_points(vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]),
        None,
    );
    let mut region = empty_profile(None);
    for _ in 0..count {
        let board = BoardModel {
            source: "retained-report-growth".into(),
            copper: vec![feature(region), feature(original.clone())],
            drills: vec![],
            board_outline: None,
            panel_features: None,
        };
        region = board.all_copper().unwrap();
    }
    assert_eq!(
        hyperlimit::compare_reals(
            &region.try_area().unwrap(),
            &Scalar::one(),
            hyperlimit::PredicatePolicy::STRICT,
        )
        .value(),
        Some(std::cmp::Ordering::Equal),
    );
    assert_eq!(
        region.classify_xy("0.5".parse().unwrap(), "0.5".parse().unwrap()),
        Ok(hypercurve::RegionPointLocation::Inside),
    );
    let report = region.try_to_multipolygon().unwrap();
    assert_eq!(report.0.len(), count);
    drop(report);
    let retained = LIVE.load(Ordering::Relaxed).checked_sub(before).unwrap();
    drop(region);
    drop(original);
    retained
}

// Keep this as a single test in its own integration binary so unrelated tests
// cannot change the process-global allocation counter during a measurement.
#[test]
fn repeated_aggregation_retains_only_linear_report_payload() {
    let small = retained_report_bytes(16);
    let large = retained_report_bytes(64);
    eprintln!("retained requested report bytes: 16={small}, 64={large}");
    assert!(
        large <= small.checked_mul(6).unwrap(),
        "fourfold source growth retained more than sixfold report payload: {small} -> {large}",
    );
}
