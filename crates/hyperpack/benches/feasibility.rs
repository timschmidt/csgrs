use std::hint::black_box;
use std::time::Instant;

use hyperpack::{AxisBox3, Bin3, FeasibilityReplay3, Item3, ItemId, Placement3, Real};

fn r(value: i32) -> Real {
    value.into()
}

fn main() {
    let bin = Bin3 {
        size: AxisBox3::new(r(100), r(100), r(10)).unwrap(),
    };
    let mut items = Vec::new();
    let mut placements = Vec::new();
    for index in 0..50 {
        let id = ItemId::new(format!("item-{index}")).unwrap();
        items.push(Item3 {
            id: id.clone(),
            size: AxisBox3::new(r(2), r(2), r(1)).unwrap(),
        });
        placements.push(Placement3 {
            item: id,
            x: r((index % 25) * 2),
            y: r((index / 25) * 2),
            z: r(0),
        });
    }

    let iterations = 1_000_u32;
    let started = Instant::now();
    let mut checks = 0_usize;
    for _ in 0..iterations {
        let replay = FeasibilityReplay3::replay(black_box(&bin), &items, &placements).unwrap();
        checks ^= replay.containment_checks;
        checks ^= replay.no_overlap_checks;
    }
    let elapsed = started.elapsed();
    println!(
        "packing_feasibility_replay: {iterations} iterations in {elapsed:?} ({:?}/iter), checks={checks}",
        elapsed / iterations
    );
}
