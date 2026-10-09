//! Capability costs; new-result and warm native area/contour queries have distinct rows.
use std::hint::black_box;
use std::time::Instant;
use hypercurve::Real;
use hyperdrc::PcbRegionExt;
use serde_json::json;

fn compare(a: &Real,b: &Real) -> std::cmp::Ordering {
    hyperlimit::compare_reals(a,b,hyperlimit::PredicatePolicy::STRICT).value().unwrap()
}
fn main() {
    let mut rows=Vec::new();
    let board_path=std::env::args().nth(1).unwrap();
    let board=hyperdrc::kicad::load_kicad_pcb(std::path::Path::new(&board_path)).unwrap();
    let [left,right]=[&board.copper[1723].region,&board.copper[1724].region];
    let overlap=left.try_intersection(right).unwrap();
    let witness=overlap.try_area().unwrap();
    assert!(compare(&witness,&Real::zero()).is_gt());
    let iterations=25;
    let started=Instant::now();
    for _ in 0..iterations { black_box(left.try_intersection(right).unwrap().try_area().unwrap()); }
    rows.push(json!({"name":"original-cparti-intersection-area","iterations":iterations,"elapsed_ns":started.elapsed().as_nanos(),"witness":witness.to_string()}));
    let iterations=10000;
    let started=Instant::now();
    for _ in 0..iterations { black_box(overlap.try_area().unwrap()); }
    rows.push(json!({"name":"original-cparti-warm-area","iterations":iterations,"elapsed_ns":started.elapsed().as_nanos(),"witness":witness.to_string()}));
    let native=overlap.clone().try_into_native().unwrap();
    assert!(native.native_contours_fast_path().unwrap().is_some());
    let iterations=25;
    let started=Instant::now();
    for _ in 0..iterations {
        let result=left.try_intersection(right).unwrap().try_into_native().unwrap();
        black_box(result.native_contours_fast_path().unwrap().unwrap());
    }
    rows.push(json!({"name":"original-cparti-intersection-native-view","iterations":iterations,"elapsed_ns":started.elapsed().as_nanos(),"witness":"exact-native-contours-available"}));
    let iterations=10000;
    let started=Instant::now();
    for _ in 0..iterations { black_box(native.native_contours_fast_path().unwrap().unwrap()); }
    rows.push(json!({"name":"original-cparti-warm-native-view","iterations":iterations,"elapsed_ns":started.elapsed().as_nanos(),"witness":"exact-native-contours-available"}));
    assert!(compare(&left.try_intersection(right).unwrap().try_area().unwrap(),&witness).is_eq());
    println!("{}",serde_json::to_string(&rows).unwrap());
}
