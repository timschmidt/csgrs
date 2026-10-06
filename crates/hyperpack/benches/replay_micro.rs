use std::hint::black_box;
use std::time::Instant;

use hypercurve::{Contour2, LineSeg2, Point2, Segment2};
use hyperpack::{
    AxisBox3, Bin3, ExactSearchLimit3, FeasibilityReplay3, IrregularPacking2, IrregularSheetItem2,
    IrregularSheetPlacement2, Item3, ItemId, Placement3, Real, Rect2, SheetBin2, SheetItem2,
    SheetPlacement2, StockBin1, StockItem1, StockPlacement1, analyze_packing_3d,
    branch_and_bound_one_bin_3d, verify_clearance_3d, verify_packing_1d, verify_packing_2d,
};

fn r(value: i32) -> Real {
    value.into()
}

fn irregular_rectangle(name: &str, width: i32, height: i32) -> IrregularSheetItem2 {
    let points = [(0, 0), (width, 0), (width, height), (0, height)]
        .into_iter()
        .map(|(x, y)| Point2::from_values(x, y))
        .collect::<Vec<_>>();
    let segments = (0..points.len())
        .map(|index| {
            Segment2::Line(
                LineSeg2::try_new(
                    points[index].clone(),
                    points[(index + 1) % points.len()].clone(),
                )
                .unwrap(),
            )
        })
        .collect();
    IrregularSheetItem2 {
        id: ItemId::new(name).unwrap(),
        shape: Contour2::try_new(segments).unwrap(),
    }
}

fn main() {
    let bin = Bin3 {
        size: AxisBox3::new(r(200), r(200), r(2)).unwrap(),
    };
    let mut items = Vec::new();
    let mut placements = Vec::new();
    for index in 0..100_i32 {
        let id = ItemId::new(format!("item-{index:03}")).unwrap();
        items.push(Item3 {
            id: id.clone(),
            size: AxisBox3::new(r(2), r(2), r(1)).unwrap(),
        });
        placements.push(Placement3 {
            item: id,
            x: r((index % 50) * 2),
            y: r((index / 50) * 2),
            z: r(0),
        });
    }

    let sheet_bin = SheetBin2::new(Rect2::new(r(200), r(2)).unwrap());
    let sheet_items = (0..100_i32)
        .map(|index| {
            SheetItem2::new(
                ItemId::new(format!("sheet-{index:03}")).unwrap(),
                Rect2::new(r(2), r(2)).unwrap(),
            )
        })
        .collect::<Vec<_>>();
    let sheet_placements = sheet_items
        .iter()
        .enumerate()
        .map(|(index, item)| SheetPlacement2 {
            item: item.id.clone(),
            x: r(index as i32 * 2),
            y: r(0),
        })
        .collect::<Vec<_>>();

    let stock_bin = StockBin1::new(r(200)).unwrap();
    let stock_items = (0..100_i32)
        .map(|index| {
            StockItem1::new(ItemId::new(format!("stock-{index:03}")).unwrap(), r(2)).unwrap()
        })
        .collect::<Vec<_>>();
    let stock_placements = stock_items
        .iter()
        .enumerate()
        .map(|(index, item)| StockPlacement1 {
            item: item.id.clone(),
            start: r(index as i32 * 2),
        })
        .collect::<Vec<_>>();

    let iterations = if cfg!(debug_assertions) { 1 } else { 100 };
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..iterations {
        let report =
            FeasibilityReplay3::replay(black_box(&bin), black_box(&items), black_box(&placements))
                .unwrap();
        checksum ^= report.no_overlap_checks;
    }
    let elapsed = started.elapsed();
    println!(
        "replay_3d_100: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / iterations
    );

    let started = Instant::now();
    for _ in 0..iterations {
        let report = verify_packing_2d(
            black_box(&sheet_bin),
            black_box(&sheet_items),
            black_box(&sheet_placements),
        )
        .unwrap();
        checksum ^= report.no_overlap_checks;
    }
    let elapsed = started.elapsed();
    println!(
        "replay_2d_100: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / iterations
    );

    let started = Instant::now();
    for _ in 0..iterations {
        let report = verify_packing_1d(
            black_box(&stock_bin),
            black_box(&stock_items),
            black_box(&stock_placements),
        )
        .unwrap();
        checksum ^= report.no_overlap_checks;
    }
    let elapsed = started.elapsed();
    println!(
        "replay_1d_100: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / iterations
    );

    let started = Instant::now();
    for _ in 0..iterations {
        let report =
            verify_clearance_3d(black_box(&items), black_box(&placements), Real::zero()).unwrap();
        checksum ^= report.pairs.len();
    }
    let elapsed = started.elapsed();
    println!(
        "clearance_3d_100: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / iterations
    );

    let irregular_bin = SheetBin2::new(Rect2::new(r(8), r(3)).unwrap());
    let irregular_items = vec![
        irregular_rectangle("irregular-a", 2, 3),
        irregular_rectangle("irregular-b", 3, 3),
        irregular_rectangle("irregular-c", 3, 3),
    ];
    let irregular_placements = vec![
        IrregularSheetPlacement2 {
            item: irregular_items[0].id.clone(),
            x: r(0),
            y: r(0),
        },
        IrregularSheetPlacement2 {
            item: irregular_items[1].id.clone(),
            x: r(2),
            y: r(0),
        },
        IrregularSheetPlacement2 {
            item: irregular_items[2].id.clone(),
            x: r(5),
            y: r(0),
        },
    ];
    let irregular = IrregularPacking2::new(black_box(&irregular_items)).expect("irregular fixture");
    let irregular_iterations = if cfg!(debug_assertions) { 1 } else { 1_000 };
    let started = Instant::now();
    for _ in 0..irregular_iterations {
        let packing =
            IrregularPacking2::new(black_box(&irregular_items)).expect("irregular fixture");
        checksum ^= packing.ready_pair_count();
    }
    let elapsed = started.elapsed();
    println!(
        "construct_irregular_3: {irregular_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / irregular_iterations
    );

    let started = Instant::now();
    for _ in 0..irregular_iterations {
        let report = black_box(&irregular)
            .verify(black_box(&irregular_bin), black_box(&irregular_placements))
            .expect("irregular replay");
        checksum ^= report.no_overlap_checks;
    }
    let elapsed = started.elapsed();
    println!(
        "verify_irregular_3: {irregular_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / irregular_iterations
    );

    let proposal_iterations = if cfg!(debug_assertions) { 1 } else { 100 };
    let started = Instant::now();
    for _ in 0..proposal_iterations {
        let report = black_box(&irregular)
            .bottom_left(black_box(&irregular_bin))
            .expect("irregular proposal");
        checksum ^= report.candidates_tested;
    }
    let elapsed = started.elapsed();
    println!(
        "bottom_left_irregular_3: {proposal_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / proposal_iterations
    );

    let unique_items = (1..=100_i32)
        .map(|index| Item3 {
            id: ItemId::new(format!("unique-{index:03}")).unwrap(),
            size: AxisBox3::new(r(index), r(index + 1), r(index + 2)).unwrap(),
        })
        .collect::<Vec<_>>();
    let analysis_iterations = if cfg!(debug_assertions) { 1 } else { 1_000 };
    let started = Instant::now();
    for _ in 0..analysis_iterations {
        let analysis = analyze_packing_3d(black_box(&bin), black_box(&unique_items));
        checksum ^= analysis.demand_classes.len();
    }
    let elapsed = started.elapsed();
    println!(
        "analyze_100_unique: {analysis_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / analysis_iterations
    );

    let started = Instant::now();
    for _ in 0..analysis_iterations {
        let analysis = analyze_packing_3d(black_box(&bin), black_box(&items));
        checksum ^= analysis.demand_classes.len();
    }
    let elapsed = started.elapsed();
    println!(
        "analyze_100_repeated: {analysis_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / analysis_iterations
    );

    let search_bin = Bin3 {
        size: AxisBox3::new(r(24), r(2), r(2)).unwrap(),
    };
    let search_items = items[..12].to_vec();
    let search_iterations = if cfg!(debug_assertions) { 1 } else { 1_000 };
    let started = Instant::now();
    for _ in 0..search_iterations {
        let report = branch_and_bound_one_bin_3d(
            black_box(&search_bin),
            black_box(&search_items),
            ExactSearchLimit3 {
                max_items: 12,
                max_nodes: 100_000,
            },
        )
        .unwrap();
        checksum ^= report.candidate_points;
    }
    let elapsed = started.elapsed();
    println!(
        "exact_search_12: {search_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / search_iterations
    );
}
