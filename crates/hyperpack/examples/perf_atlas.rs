//! Performance and resource atlas for Hyperpack: exact packing replay,
//! constructive 2D sheet and 3D cuboid heuristics, heuristic portfolios,
//! order search, bounded exact search, capacity bounds, model export,
//! irregular (no-fit) nesting and snapshots.
//!
//! Axes:
//! - **part count**: items packed or replayed, for every operation;
//! - **container fill fraction**: total item area (volume) over sheet area
//!   (bin volume), in percent, from sparse sheets to over-full ones where
//!   items must be rejected;
//! - **part complexity**: item mix (uniform squares/cubes, mixed sizes, long
//!   strips or plates) and, for irregular nesting, contour vertex count.
//!
//! Containers are square sheets (cubic bins) sized from the requested fill
//! fraction, 70% unless the sweep varies it. Every heuristic result is
//! replayed exactly by the library; a call fails when that replay is not
//! `Feasible` or search ends without a decision. In resource runs, results
//! report their retained proposal size (`nodes`: candidates plus retained
//! free rectangles/boxes, placements, pairs or disjunctions) and the largest
//! placed coordinate in bits.
//!
//! Run through `scripts/perf-atlas/run.sh`, or directly:
//! `cargo run --release -p hyperpack --all-features --example perf_atlas -- --out atlas.json [--quick] [--resources]`.
use hypercurve::{Contour2, LineSeg2, Point2, Segment2};
use hyperpack::{
    AxisBox3, Bin3, CuboidHeuristicReport3, CuboidPortfolioBudget3, ExactSearchLimit3,
    ExactSearchStatus3, FeasibilityStatus, IrregularPacking2, IrregularSheetItem2,
    IrregularSheetPlacement2, Item3, ItemId, LocalSearchConfig3, MultistartConfig3,
    OrderEvaluation3, PackResult, Placement3, Real, Rect2, ReinsertUnplacedConfig3, SheetBin2,
    SheetHeuristicReport2, SheetItem2, SheetPlacement2, SheetPortfolioBudget2, StockBin1,
    StockItem1, StockPlacement1, SupportPolicy3, TabuSearchConfig3,
};
use perf_atlas::{Atlas, Checked, Rng, Shape, Shaped, ladder, powers_of_two};

#[global_allocator]
static ALLOCATOR: perf_atlas::CountingAllocator = perf_atlas::CountingAllocator;

fn r(value: i64) -> Real {
    Real::from(value)
}

fn id(prefix: &str, index: usize) -> ItemId {
    ItemId::new(format!("{prefix}-{index:05}")).expect("non-empty id")
}

fn bits(value: &Real) -> u64 {
    value.representation_size(1 << 10).max_rational_bits
}

// ---------------------------------------------------------------------------
// Inputs
// ---------------------------------------------------------------------------

/// 2D item mixes: part complexity for rectangle packing.
const SHEET_KINDS: [&str; 3] = ["uniform squares", "mixed rectangles", "long strips"];
/// 3D item mixes.
const CUBOID_KINDS: [&str; 3] = ["uniform cubes", "mixed cuboids", "flat plates"];

#[derive(Clone)]
struct Sheet {
    bin: SheetBin2,
    items: Vec<SheetItem2>,
}

#[derive(Clone)]
struct Cuboids {
    bin: Bin3,
    items: Vec<Item3>,
}

fn rect_dims(kind: &str, rng: &mut Rng) -> (i64, i64) {
    match kind {
        "uniform squares" => (4, 4),
        "long strips" => (rng.range(8, 24), rng.range(1, 3)),
        _ => (rng.range(1, 12), rng.range(1, 12)),
    }
}

fn box_dims(kind: &str, rng: &mut Rng) -> (i64, i64, i64) {
    match kind {
        "uniform cubes" => (2, 2, 2),
        "flat plates" => (rng.range(2, 8), rng.range(2, 8), 1),
        _ => (rng.range(1, 6), rng.range(1, 6), rng.range(1, 6)),
    }
}

/// `n` items of `kind` on a square sheet whose area is `total / fill`.
fn sheet(kind: &str, n: usize, fill_percent: f64, rng: &mut Rng) -> Sheet {
    let dims: Vec<(i64, i64)> = (0..n).map(|_| rect_dims(kind, rng)).collect();
    let area: i64 = dims.iter().map(|(w, h)| w * h).sum();
    let longest = dims.iter().map(|(w, h)| *w.max(h)).max().unwrap_or(1);
    let side = ((area as f64 * 100.0 / fill_percent).sqrt().ceil() as i64).max(longest);
    Sheet {
        bin: SheetBin2::new(Rect2::new(r(side), r(side)).expect("positive sheet")),
        items: dims
            .into_iter()
            .enumerate()
            .map(|(i, (w, h))| SheetItem2::new(id("s", i), Rect2::new(r(w), r(h)).unwrap()))
            .collect(),
    }
}

/// `n` items of `kind` in a cubic bin whose volume is `total / fill`.
fn cuboids(kind: &str, n: usize, fill_percent: f64, rng: &mut Rng) -> Cuboids {
    let dims: Vec<(i64, i64, i64)> = (0..n).map(|_| box_dims(kind, rng)).collect();
    let volume: i64 = dims.iter().map(|(x, y, z)| x * y * z).sum();
    let longest = dims
        .iter()
        .map(|(x, y, z)| *x.max(y).max(z))
        .max()
        .unwrap_or(1);
    let side = ((volume as f64 * 100.0 / fill_percent).cbrt().ceil() as i64).max(longest);
    Cuboids {
        bin: Bin3 {
            size: AxisBox3::new(r(side), r(side), r(side)).expect("positive bin"),
        },
        items: dims
            .into_iter()
            .enumerate()
            .map(|(i, (x, y, z))| Item3 {
                id: id("c", i),
                size: AxisBox3::new(r(x), r(y), r(z)).unwrap(),
            })
            .collect(),
    }
}

/// A known-feasible grid layout of `n` 2x2 squares with `gap` between them
/// (gap 0 is a 100% fill; gap 1 about 44%). `overlap` moves the last item
/// onto its neighbour so replay must find the conflict.
#[derive(Clone)]
struct Layout2 {
    bin: SheetBin2,
    items: Vec<SheetItem2>,
    placements: Vec<SheetPlacement2>,
}

fn grid_layout_2d(n: usize, gap: i64, overlap: bool) -> Layout2 {
    let columns = (n as f64).sqrt().ceil() as i64;
    let rows = (n as i64 + columns - 1) / columns;
    let pitch = 2 + gap;
    let bin = SheetBin2::new(Rect2::new(r(columns * pitch), r(rows.max(1) * pitch)).unwrap());
    let items: Vec<SheetItem2> = (0..n)
        .map(|i| SheetItem2::new(id("g", i), Rect2::new(r(2), r(2)).unwrap()))
        .collect();
    let mut placements: Vec<SheetPlacement2> = items
        .iter()
        .enumerate()
        .map(|(i, item)| SheetPlacement2 {
            item: item.id.clone(),
            x: r(i as i64 % columns * pitch),
            y: r(i as i64 / columns * pitch),
        })
        .collect();
    if overlap && n > 1 {
        let last = placements.len() - 1;
        placements[last].x = placements[last - 1].x.clone() + r(1);
        placements[last].y = placements[last - 1].y.clone();
    }
    Layout2 {
        bin,
        items,
        placements,
    }
}

#[derive(Clone)]
struct Layout3 {
    bin: Bin3,
    items: Vec<Item3>,
    placements: Vec<Placement3>,
}

/// A known-feasible stack of `n` 2x2x1 slabs on a square grid, layers along
/// z, with `gap` between columns. `overlap` puts the last item on its
/// neighbour.
fn grid_layout_3d(n: usize, gap: i64, overlap: bool) -> Layout3 {
    let side = ((n as f64).cbrt().ceil() as i64).max(1);
    let per_layer = side * side;
    let layers = (n as i64 + per_layer - 1) / per_layer;
    let pitch = 2 + gap;
    let bin = Bin3 {
        size: AxisBox3::new(r(side * pitch), r(side * pitch), r(layers.max(1))).unwrap(),
    };
    let items: Vec<Item3> = (0..n)
        .map(|i| Item3 {
            id: id("g", i),
            size: AxisBox3::new(r(2), r(2), r(1)).unwrap(),
        })
        .collect();
    let mut placements: Vec<Placement3> = items
        .iter()
        .enumerate()
        .map(|(i, item)| {
            let i = i as i64;
            let in_layer = i % per_layer;
            Placement3 {
                item: item.id.clone(),
                x: r(in_layer % side * pitch),
                y: r(in_layer / side * pitch),
                z: r(i / per_layer),
            }
        })
        .collect();
    if overlap && n > 1 {
        let last = placements.len() - 1;
        placements[last] = Placement3 {
            item: placements[last].item.clone(),
            ..placements[last - 1].clone()
        };
    }
    Layout3 {
        bin,
        items,
        placements,
    }
}

const REPLAY_KINDS: [(&str, i64, bool); 3] = [
    ("tight grid (100% fill)", 0, false),
    ("spaced grid", 1, false),
    ("one overlap", 0, true),
];

/// `Ok` when replay reached the expected decision.
fn expect(status: FeasibilityStatus, overlap: bool) -> Checked<FeasibilityStatus> {
    let wanted = if overlap {
        FeasibilityStatus::Infeasible
    } else {
        FeasibilityStatus::Feasible
    };
    if status == wanted {
        Ok(status)
    } else {
        Err(format!("replay returned {status:?}, expected {wanted:?}"))
    }
}

/// A convex contour with `vertices` exact rational vertices on a circle of
/// radius `scale` centred at `(scale, scale)`. Vertices come from the
/// rational parametrisation `((1 - t^2) / (1 + t^2), 2t / (1 + t^2))` with
/// `t = tan(theta / 2)` rounded to 1/1024 at evenly spaced angles, so they
/// are distinct, in counter-clockwise order and therefore convex.
fn convex_contour(vertices: usize, scale: i64) -> Contour2 {
    const D: i64 = 1024;
    let k = vertices.max(3);
    let mut previous = i64::MIN;
    let points: Vec<Point2> = (0..k)
        .filter_map(|i| {
            let theta =
                -std::f64::consts::PI + 2.0 * std::f64::consts::PI * (i as f64 + 0.5) / k as f64;
            let a = ((theta / 2.0).tan() * D as f64).round() as i64;
            if a <= previous {
                return None;
            }
            previous = a;
            let w = r(D * D + a * a);
            let x = (r(scale * (D * D - a * a)) / w.clone()).unwrap() + r(scale);
            let y = (r(scale * 2 * a * D) / w).unwrap() + r(scale);
            Some(Point2::new(x, y))
        })
        .collect();
    let segments = (0..points.len())
        .map(|i| {
            Segment2::Line(
                LineSeg2::try_new(points[i].clone(), points[(i + 1) % points.len()].clone())
                    .expect("distinct contour vertices"),
            )
        })
        .collect();
    Contour2::try_new(segments).expect("convex contour")
}

#[derive(Clone)]
struct Nesting {
    bin: SheetBin2,
    items: Vec<IrregularSheetItem2>,
}

/// `n` convex parts of `vertices` vertices and random radius 1..4 on a
/// square sheet at the given fill (by bounding-box area).
fn nesting(n: usize, vertices: usize, fill_percent: f64, rng: &mut Rng) -> Nesting {
    let mut area = 0;
    let mut longest = 1;
    let items = (0..n)
        .map(|i| {
            let scale = rng.range(1, 4);
            area += 4 * scale * scale;
            longest = longest.max(2 * scale);
            IrregularSheetItem2 {
                id: id("p", i),
                shape: convex_contour(vertices, scale),
            }
        })
        .collect();
    let side = ((area as f64 * 100.0 / fill_percent).sqrt().ceil() as i64).max(longest);
    Nesting {
        bin: SheetBin2::new(Rect2::new(r(side), r(side)).unwrap()),
        items,
    }
}

// ---------------------------------------------------------------------------
// Result shapes
// ---------------------------------------------------------------------------

fn sheet_shape(report: &Checked<SheetHeuristicReport2>) -> Option<Shape> {
    let report = report.as_ref().ok()?;
    Some(Shape {
        nodes: (report.candidates.len() + report.free_rects.len()) as u64,
        depth: 0,
        bits: report
            .candidates
            .iter()
            .flat_map(|c| [bits(&c.placement.x), bits(&c.placement.y)])
            .max()
            .unwrap_or(0),
    })
}

fn cuboid_shape(report: &Checked<CuboidHeuristicReport3>) -> Option<Shape> {
    let report = report.as_ref().ok()?;
    Some(Shape {
        nodes: (report.candidates.len() + report.points.len() + report.free_boxes.len()) as u64,
        depth: 0,
        bits: report
            .candidates
            .iter()
            .flat_map(|c| {
                [
                    bits(&c.placement.x),
                    bits(&c.placement.y),
                    bits(&c.placement.z),
                ]
            })
            .max()
            .unwrap_or(0),
    })
}

fn placements_shape(placements: &[Placement3]) -> Shape {
    Shape {
        nodes: placements.len() as u64,
        depth: 0,
        bits: placements
            .iter()
            .flat_map(|p| [bits(&p.x), bits(&p.y), bits(&p.z)])
            .max()
            .unwrap_or(0),
    }
}

fn sheet_checked(
    report: PackResult<SheetHeuristicReport2>,
) -> Shaped<Checked<SheetHeuristicReport2>> {
    let checked = match report {
        Ok(report) if report.replay.status == FeasibilityStatus::Feasible => Ok(report),
        Ok(report) => Err(format!("replay {:?}", report.replay.status)),
        Err(error) => Err(format!("{error:?}")),
    };
    Shaped::new(checked, sheet_shape)
}

fn cuboid_checked(
    report: PackResult<CuboidHeuristicReport3>,
) -> Shaped<Checked<CuboidHeuristicReport3>> {
    let checked = match report {
        Ok(report) if report.replay.feasibility.status == FeasibilityStatus::Feasible => Ok(report),
        Ok(report) => Err(format!("replay {:?}", report.replay.feasibility.status)),
        Err(error) => Err(format!("{error:?}")),
    };
    Shaped::new(checked, cuboid_shape)
}

fn order_checked(best: &OrderEvaluation3) -> Shaped<Checked<Vec<Placement3>>> {
    let checked = if best.replay.feasibility.status == FeasibilityStatus::Feasible {
        Ok(best.placements.clone())
    } else {
        Err(format!("replay {:?}", best.replay.feasibility.status))
    };
    Shaped::new(checked, |placements| {
        placements.as_ref().ok().map(|p| placements_shape(p))
    })
}

// ---------------------------------------------------------------------------
// Operations
// ---------------------------------------------------------------------------

type Sheet2Fn = fn(&SheetBin2, &[SheetItem2]) -> PackResult<SheetHeuristicReport2>;
type Cuboid3Fn = fn(&Bin3, &[Item3]) -> PackResult<CuboidHeuristicReport3>;

const SHEET_HEURISTICS: [(&str, Sheet2Fn); 13] = [
    ("shelf_nfdh", hyperpack::shelf_next_fit_decreasing_height_2d),
    (
        "shelf_ffdh",
        hyperpack::shelf_first_fit_decreasing_height_2d,
    ),
    ("shelf_bfdh", hyperpack::shelf_best_fit_decreasing_height_2d),
    ("skyline_bottom_left", hyperpack::skyline_bottom_left_2d),
    ("skyline_min_waste", hyperpack::skyline_minimum_waste_2d),
    ("maxrects_bssf", hyperpack::maxrects_best_short_side_fit_2d),
    ("maxrects_blsf", hyperpack::maxrects_best_long_side_fit_2d),
    ("maxrects_baf", hyperpack::maxrects_best_area_fit_2d),
    ("maxrects_bottom_left", hyperpack::maxrects_bottom_left_2d),
    (
        "maxrects_contact_point",
        hyperpack::maxrects_contact_point_2d,
    ),
    ("guillotine_baf", hyperpack::guillotine_best_area_fit_2d),
    (
        "guillotine_bssf",
        hyperpack::guillotine_best_short_side_fit_2d,
    ),
    (
        "guillotine_blsf",
        hyperpack::guillotine_best_long_side_fit_2d,
    ),
];

const CUBOID_HEURISTICS: [(&str, Cuboid3Fn); 10] = [
    (
        "ffd_volume",
        hyperpack::cuboid_first_fit_decreasing_volume_3d,
    ),
    (
        "bfd_volume",
        hyperpack::cuboid_best_fit_decreasing_volume_3d,
    ),
    (
        "ffd_max_side",
        hyperpack::cuboid_first_fit_decreasing_max_side_3d,
    ),
    (
        "bfd_max_side",
        hyperpack::cuboid_best_fit_decreasing_max_side_3d,
    ),
    (
        "ffd_footprint",
        hyperpack::cuboid_first_fit_decreasing_footprint_area_3d,
    ),
    (
        "bfd_footprint",
        hyperpack::cuboid_best_fit_decreasing_footprint_area_3d,
    ),
    (
        "extreme_point",
        hyperpack::cuboid_extreme_point_decreasing_volume_3d,
    ),
    (
        "maximal_space",
        hyperpack::cuboid_maximal_space_decreasing_volume_3d,
    ),
    (
        "guillotine_volume",
        hyperpack::cuboid_guillotine_best_volume_fit_3d,
    ),
    ("laff", hyperpack::cuboid_laff_largest_area_fit_first_3d),
];

/// Fill fractions in percent, from sparse to over-full.
const FILLS: [f64; 10] = [
    10.0, 25.0, 40.0, 55.0, 70.0, 80.0, 90.0, 100.0, 115.0, 150.0,
];

fn replay(atlas: &mut Atlas) {
    let counts = powers_of_two(0, 13);

    atlas.operation(
        "replay",
        "verify_packing_2d",
        "part count",
        "Exact 2D containment and pairwise no-overlap replay of a grid layout of 2x2 squares.",
    );
    for (kind, gap, overlap) in REPLAY_KINDS {
        atlas.sweep(
            kind,
            // An overlap needs two parts.
            &counts[usize::from(overlap)..],
            |n, _| grid_layout_2d(n as usize, gap, overlap),
            |l| {
                hyperpack::verify_packing_2d(&l.bin, &l.items, &l.placements)
                    .map_err(|e| format!("{e:?}"))
                    .and_then(|v| expect(v.status, overlap))
            },
        );
    }

    atlas.operation(
        "replay",
        "verify_packing_3d",
        "part count",
        "Exact 3D containment and pairwise no-overlap replay of stacked 2x2x1 slabs.",
    );
    for (kind, gap, overlap) in REPLAY_KINDS {
        atlas.sweep(
            kind,
            // An overlap needs two parts.
            &counts[usize::from(overlap)..],
            |n, _| grid_layout_3d(n as usize, gap, overlap),
            |l| {
                hyperpack::verify_packing_3d(&l.bin, &l.items, &l.placements)
                    .map_err(|e| format!("{e:?}"))
                    .and_then(|v| expect(v.feasibility.status, overlap))
            },
        );
    }

    atlas.operation(
        "replay",
        "verify_packing_1d",
        "part count",
        "Exact 1D stock-cutting replay of unit-gap cuts along one bar.",
    );
    for (kind, gap, overlap) in REPLAY_KINDS {
        atlas.sweep(
            kind,
            // An overlap needs two parts.
            &counts[usize::from(overlap)..],
            |n, _| {
                let n = n as usize;
                let pitch = 2 + gap;
                let items: Vec<StockItem1> = (0..n)
                    .map(|i| StockItem1::new(id("b", i), r(2)).unwrap())
                    .collect();
                let mut placements: Vec<StockPlacement1> = items
                    .iter()
                    .enumerate()
                    .map(|(i, item)| StockPlacement1 {
                        item: item.id.clone(),
                        start: r(i as i64 * pitch),
                    })
                    .collect();
                if overlap && n > 1 {
                    placements[n - 1].start = placements[n - 2].start.clone() + r(1);
                }
                (
                    StockBin1::new(r(n as i64 * pitch)).unwrap(),
                    items,
                    placements,
                )
            },
            |(bin, items, placements)| {
                hyperpack::verify_packing_1d(bin, items, placements)
                    .map_err(|e| format!("{e:?}"))
                    .and_then(|v| expect(v.status, overlap))
            },
        );
    }

    atlas.operation(
        "replay",
        "verify_clearance_3d",
        "part count",
        "Exact pairwise clearance evidence (required clearance 1) for stacked slabs; the spaced grid satisfies it in x/y, layers touch in z.",
    );
    for (kind, gap, overlap) in REPLAY_KINDS {
        atlas.sweep(
            kind,
            // An overlap needs two parts.
            &counts[usize::from(overlap)..],
            |n, _| grid_layout_3d(n as usize, gap, overlap),
            |l| {
                hyperpack::verify_clearance_3d(&l.items, &l.placements, Real::zero())
                    .map(|report| report.pairs.len())
            },
        );
    }

    atlas.operation(
        "replay",
        "verify_support_3d",
        "part count",
        "Exact full-base support check of stacked slabs (every upper slab rests on the one below).",
    );
    for (kind, gap, overlap) in REPLAY_KINDS {
        atlas.sweep(
            kind,
            // An overlap needs two parts.
            &counts[usize::from(overlap)..],
            |n, _| grid_layout_3d(n as usize, gap, overlap),
            |l| {
                hyperpack::verify_support_3d(&l.items, &l.placements, SupportPolicy3::FullBase)
                    .map(|report| report.evidence.len())
            },
        );
    }
}

fn heuristics_2d(atlas: &mut Atlas) {
    let counts = powers_of_two(1, 12);
    for (name, heuristic) in SHEET_HEURISTICS {
        atlas.operation(
            "sheet heuristics",
            name,
            "part count",
            "Constructive 2D rectangle packing at 70% sheet fill, then exact replay; fails unless the replay is feasible.",
        );
        for kind in SHEET_KINDS {
            atlas.sweep(
                kind,
                &counts,
                |n, rng| sheet(kind, n as usize, 70.0, rng),
                |s| sheet_checked(heuristic(&s.bin, &s.items)),
            );
        }
    }
    // Fill fraction at a fixed part count, for the most used families.
    for (name, heuristic, parts) in [
        ("maxrects_bssf", SHEET_HEURISTICS[5].1, 128),
        // Skyline is cubic in the part count; fewer parts keep the sweep under the ceiling.
        ("skyline_bottom_left", SHEET_HEURISTICS[3].1, 48),
        ("shelf_ffdh", SHEET_HEURISTICS[1].1, 128),
        ("guillotine_baf", SHEET_HEURISTICS[10].1, 128),
    ] {
        atlas.operation(
            "sheet heuristics",
            &format!("{name} vs fill"),
            "fill percent",
            &format!("{parts} parts of each mix on sheets from 10% to 150% fill; above 100% items are rejected, and the replay of the placed subset must still be feasible."),
        );
        for kind in SHEET_KINDS {
            atlas.sweep(
                kind,
                &FILLS,
                |fill, rng| sheet(kind, parts, fill, rng),
                |s| sheet_checked(heuristic(&s.bin, &s.items)),
            );
        }
        if name == "maxrects_bssf" {
            atlas.grid(
                "mixed rectangles",
                ("part count", &powers_of_two(2, 10)),
                ("fill percent", &[25.0, 55.0, 80.0, 100.0, 150.0]),
                |n, fill, rng| sheet("mixed rectangles", n as usize, fill, rng),
                |s| sheet_checked(heuristic(&s.bin, &s.items)),
            );
        }
    }
    atlas.operation(
        "sheet heuristics",
        "maxrects_bssf fuzz",
        "part count",
        "Random mixed rectangles at random fill between 30% and 120%.",
    );
    atlas.fuzz(
        "mixed rectangles",
        &powers_of_two(3, 9),
        |n, rng| {
            let fill = 30.0 + 90.0 * rng.unit();
            sheet("mixed rectangles", n as usize, fill, rng)
        },
        |s| sheet_checked(hyperpack::maxrects_best_short_side_fit_2d(&s.bin, &s.items)),
    );
}

fn heuristics_3d(atlas: &mut Atlas) {
    let counts = powers_of_two(1, 11);
    for (name, heuristic) in CUBOID_HEURISTICS {
        atlas.operation(
            "cuboid heuristics",
            name,
            "part count",
            "Constructive 3D cuboid packing at 70% bin fill, then exact replay; fails unless the replay is feasible.",
        );
        for kind in CUBOID_KINDS {
            atlas.sweep(
                kind,
                &counts,
                |n, rng| cuboids(kind, n as usize, 70.0, rng),
                |c| cuboid_checked(heuristic(&c.bin, &c.items)),
            );
        }
    }
    for (name, heuristic) in [
        ("extreme_point", CUBOID_HEURISTICS[6].1),
        ("laff", CUBOID_HEURISTICS[9].1),
    ] {
        atlas.operation(
            "cuboid heuristics",
            &format!("{name} vs fill"),
            "fill percent",
            "64 cuboids in bins from 10% to 150% fill; the replay of the placed subset must be feasible.",
        );
        for kind in CUBOID_KINDS {
            atlas.sweep(
                kind,
                &FILLS,
                |fill, rng| cuboids(kind, 64, fill, rng),
                |c| cuboid_checked(heuristic(&c.bin, &c.items)),
            );
        }
    }
    atlas.operation(
        "cuboid heuristics",
        "extreme_point fuzz",
        "part count",
        "Random mixed cuboids at random fill between 30% and 120%.",
    );
    atlas.fuzz(
        "mixed cuboids",
        &powers_of_two(2, 8),
        |n, rng| {
            let fill = 30.0 + 90.0 * rng.unit();
            cuboids("mixed cuboids", n as usize, fill, rng)
        },
        |c| {
            cuboid_checked(hyperpack::cuboid_extreme_point_decreasing_volume_3d(
                &c.bin, &c.items,
            ))
        },
    );
}

fn portfolios_and_search(atlas: &mut Atlas) {
    atlas.operation(
        "portfolio",
        "auto_sheet_portfolio_2d",
        "part count",
        "Every 2D heuristic in the portfolio, best exact replay kept, at 70% fill.",
    );
    for kind in SHEET_KINDS {
        atlas.sweep(
            kind,
            &powers_of_two(1, 10),
            |n, rng| sheet(kind, n as usize, 70.0, rng),
            |s| {
                hyperpack::auto_sheet_portfolio_2d(
                    &s.bin,
                    &s.items,
                    SheetPortfolioBudget2 {
                        max_algorithms: usize::MAX,
                    },
                )
                .map(|report| report.best.is_some())
                .unwrap_or(false)
            },
        );
    }
    atlas.operation(
        "portfolio",
        "auto_cuboid_portfolio_3d",
        "part count",
        "Every 3D heuristic in the portfolio, best exact replay kept, at 70% fill.",
    );
    for kind in CUBOID_KINDS {
        atlas.sweep(
            kind,
            &powers_of_two(1, 9),
            |n, rng| cuboids(kind, n as usize, 70.0, rng),
            |c| {
                hyperpack::auto_cuboid_portfolio_3d(
                    &c.bin,
                    &c.items,
                    CuboidPortfolioBudget3 {
                        max_algorithms: usize::MAX,
                    },
                )
                .map(|report| report.best.is_some())
                .unwrap_or(false)
            },
        );
    }

    let counts = powers_of_two(1, 8);
    // Over-full bins leave unplaced items, so the searches have work to do.
    let search_kinds = [
        ("mixed cuboids, 90% fill", 90.0),
        ("mixed cuboids, 120% fill", 120.0),
    ];
    atlas.operation(
        "order search",
        "local_search_order_3d",
        "part count",
        "Swap-neighbourhood order search over extreme-point packing: at most 4 steps of 16 neighbours, each an exact replay.",
    );
    for (kind, fill) in search_kinds {
        atlas.sweep(
            kind,
            &counts,
            |n, rng| cuboids("mixed cuboids", n as usize, fill, rng),
            |c| {
                hyperpack::local_search_order_3d(
                    &c.bin,
                    &c.items,
                    LocalSearchConfig3 {
                        max_steps: 4,
                        max_neighbors_per_step: 16,
                    },
                )
                .map_err(|e| format!("{e:?}"))
                .map(|report| order_checked(&report.best))
            },
        );
    }
    atlas.operation(
        "order search",
        "tabu_search_order_3d",
        "part count",
        "Tabu order search: 4 steps of 16 neighbours with tenure 4.",
    );
    for (kind, fill) in search_kinds {
        atlas.sweep(
            kind,
            &counts,
            |n, rng| cuboids("mixed cuboids", n as usize, fill, rng),
            |c| {
                hyperpack::tabu_search_order_3d(
                    &c.bin,
                    &c.items,
                    TabuSearchConfig3 {
                        max_steps: 4,
                        max_neighbors_per_step: 16,
                        tabu_tenure: 4,
                    },
                )
                .map_err(|e| format!("{e:?}"))
                .map(|report| order_checked(&report.best))
            },
        );
    }
    atlas.operation(
        "order search",
        "multistart_order_3d",
        "part count",
        "Eight seeded random orders, each packed and replayed exactly.",
    );
    for (kind, fill) in search_kinds {
        atlas.sweep(
            kind,
            &counts,
            |n, rng| cuboids("mixed cuboids", n as usize, fill, rng),
            |c| {
                hyperpack::multistart_order_3d(
                    &c.bin,
                    &c.items,
                    MultistartConfig3 { seed: 7, starts: 8 },
                )
                .map_err(|e| format!("{e:?}"))
                .and_then(|report| {
                    report
                        .best
                        .map(|best| order_checked(&best.evaluation))
                        .ok_or_else(|| "no evaluation".to_owned())
                })
            },
        );
    }
    atlas.operation(
        "order search",
        "reinsert_unplaced_order_3d",
        "part count",
        "Move unplaced items earlier in the order: 2 passes of at most 16 trials.",
    );
    for (kind, fill) in search_kinds {
        atlas.sweep(
            kind,
            &counts,
            |n, rng| cuboids("mixed cuboids", n as usize, fill, rng),
            |c| {
                hyperpack::reinsert_unplaced_order_3d(
                    &c.bin,
                    &c.items,
                    ReinsertUnplacedConfig3 {
                        max_passes: 2,
                        max_trials_per_pass: 16,
                    },
                )
                .map_err(|e| format!("{e:?}"))
                .map(|report| order_checked(&report.best))
            },
        );
    }

    atlas.operation(
        "exact search",
        "branch_and_bound_one_bin_3d",
        "part count",
        "Bounded exhaustive one-bin search (at most 10^6 nodes); an exhausted node budget is a failure, a proof of infeasibility is not.",
    );
    let small = ladder(1, 16, 12);
    for (kind, fill) in [
        ("uniform cubes, 100% fill", 100.0),
        ("mixed cuboids, 60% fill", 60.0),
        ("mixed cuboids, 95% fill", 95.0),
    ] {
        let mix = if kind.starts_with("uniform") {
            "uniform cubes"
        } else {
            "mixed cuboids"
        };
        atlas.sweep(
            kind,
            &small,
            |n, rng| {
                let mut c = cuboids(mix, n as usize, fill, rng);
                if mix == "uniform cubes" {
                    // An exact-fit row of cubes: the bin is n x 1 x 1 cubes.
                    c.bin = Bin3 {
                        size: AxisBox3::new(r(2 * n as i64), r(2), r(2)).unwrap(),
                    };
                }
                c
            },
            |c| {
                hyperpack::branch_and_bound_one_bin_3d(
                    &c.bin,
                    &c.items,
                    ExactSearchLimit3 {
                        max_items: 64,
                        max_nodes: 1_000_000,
                    },
                )
                .map_err(|e| format!("{e:?}"))
                .and_then(|report| match report.status {
                    ExactSearchStatus3::Unknown => {
                        Err(format!("node budget exhausted after {}", report.nodes))
                    }
                    _ => Ok(report.nodes),
                })
            },
        );
    }
}

fn bounds_and_models(atlas: &mut Atlas) {
    let counts = powers_of_two(1, 14);
    let pair_counts = powers_of_two(1, 12);
    atlas.operation(
        "bounds",
        "capacity_bounds_2d",
        "part count",
        "Exact area and per-item fit bound for a sheet at 70% fill.",
    );
    for kind in SHEET_KINDS {
        atlas.sweep(
            kind,
            &counts,
            |n, rng| sheet(kind, n as usize, 70.0, rng),
            |s| hyperpack::capacity_bounds_2d(&s.bin, &s.items).checked_items,
        );
    }
    atlas.operation(
        "bounds",
        "capacity_bounds_3d",
        "part count",
        "Exact volume and per-item fit bound for a bin at 70% fill.",
    );
    for kind in CUBOID_KINDS {
        atlas.sweep(
            kind,
            &counts,
            |n, rng| cuboids(kind, n as usize, 70.0, rng),
            |c| hyperpack::capacity_bounds_3d(&c.bin, &c.items).checked_items,
        );
    }
    atlas.operation(
        "bounds",
        "pair_incompatibilities_2d",
        "part count",
        "Exact all-pairs check of which two items cannot share the sheet.",
    );
    for kind in SHEET_KINDS {
        atlas.sweep(
            kind,
            &pair_counts,
            |n, rng| sheet(kind, n as usize, 70.0, rng),
            |s| hyperpack::pair_incompatibilities_2d(&s.bin, &s.items).checked_pairs,
        );
    }
    atlas.operation(
        "bounds",
        "pair_incompatibilities_3d",
        "part count",
        "Exact all-pairs check of which two items cannot share the bin.",
    );
    for kind in CUBOID_KINDS {
        atlas.sweep(
            kind,
            &pair_counts,
            |n, rng| cuboids(kind, n as usize, 70.0, rng),
            |c| hyperpack::pair_incompatibilities_3d(&c.bin, &c.items).checked_pairs,
        );
    }
    atlas.operation(
        "bounds",
        "analyze_packing_3d",
        "part count",
        "Demand classes, dimension and grid facts and initial free boxes.",
    );
    for kind in CUBOID_KINDS {
        atlas.sweep(
            kind,
            &counts,
            |n, rng| cuboids(kind, n as usize, 70.0, rng),
            |c| {
                hyperpack::analyze_packing_3d(&c.bin, &c.items)
                    .demand_classes
                    .len()
            },
        );
    }
    atlas.operation(
        "model export",
        "export_no_overlap_model_2d",
        "part count",
        "Placement domains plus one no-overlap disjunction per item pair, for an external MIP/CP solver.",
    );
    for kind in SHEET_KINDS {
        atlas.sweep(
            kind,
            &pair_counts,
            |n, rng| sheet(kind, n as usize, 70.0, rng),
            |s| {
                let report = hyperpack::export_no_overlap_model_2d(&s.bin, &s.items);
                let checked: Checked<_> = match report.status {
                    hyperpack::ModelExportStatus2::Unknown => {
                        Err("unknown export status".to_owned())
                    }
                    _ => Ok(report),
                };
                Shaped::new(checked, |report| {
                    report.as_ref().ok().map(|report| Shape {
                        nodes: (report.domains.len() + report.disjunctions.len()) as u64,
                        depth: 0,
                        bits: 0,
                    })
                })
            },
        );
    }
    atlas.operation(
        "model export",
        "export_no_overlap_model_3d",
        "part count",
        "Placement domains plus one six-way no-overlap disjunction per item pair.",
    );
    for kind in CUBOID_KINDS {
        atlas.sweep(
            kind,
            &pair_counts,
            |n, rng| cuboids(kind, n as usize, 70.0, rng),
            |c| {
                let report = hyperpack::export_no_overlap_model_3d(&c.bin, &c.items);
                let checked: Checked<_> = match report.status {
                    hyperpack::ModelExportStatus3::Unknown => {
                        Err("unknown export status".to_owned())
                    }
                    _ => Ok(report),
                };
                Shaped::new(checked, |report| {
                    report.as_ref().ok().map(|report| Shape {
                        nodes: (report.domains.len() + report.disjunctions.len()) as u64,
                        depth: 0,
                        bits: 0,
                    })
                })
            },
        );
    }
    atlas.operation(
        "snapshot",
        "snapshot_sheet_2d",
        "part count",
        "Canonical text and binary snapshots of a 2D layout.",
    );
    atlas.sweep(
        "text",
        &counts,
        |n, _| grid_layout_2d(n as usize, 1, false),
        |l| hyperpack::snapshot_sheet_2d_text(&l.bin, &l.items, &l.placements).len(),
    );
    atlas.sweep(
        "binary",
        &counts,
        |n, _| grid_layout_2d(n as usize, 1, false),
        |l| hyperpack::snapshot_sheet_2d_binary(&l.bin, &l.items, &l.placements).len(),
    );
    atlas.operation(
        "snapshot",
        "snapshot_packing_3d",
        "part count",
        "Canonical text and binary snapshots of a 3D layout.",
    );
    atlas.sweep(
        "text",
        &counts,
        |n, _| grid_layout_3d(n as usize, 1, false),
        |l| hyperpack::snapshot_packing_3d_text(&l.bin, &l.items, &l.placements).len(),
    );
    atlas.sweep(
        "binary",
        &counts,
        |n, _| grid_layout_3d(n as usize, 1, false),
        |l| hyperpack::snapshot_packing_3d_binary(&l.bin, &l.items, &l.placements).len(),
    );
}

fn nesting_ops(atlas: &mut Atlas) {
    let vertex_kinds = [4usize, 8, 16, 32];
    atlas.operation(
        "irregular nesting",
        "IrregularPacking2::new",
        "part count",
        "Exact convex no-fit (translation obstacle) cache for every part pair; one series per contour vertex count.",
    );
    for vertices in vertex_kinds {
        atlas.sweep(
            &format!("{vertices}-gon parts"),
            &powers_of_two(1, 8),
            |n, rng| nesting(n as usize, vertices, 70.0, rng),
            |nest| {
                Shaped::new(
                    IrregularPacking2::new(&nest.items).map_err(|e| format!("{e:?}")),
                    |packing| {
                        packing.as_ref().ok().map(|p| Shape {
                            nodes: p
                                .pairs()
                                .values()
                                .filter_map(|pair| pair.obstacle())
                                .map(|o| o.boundary().segments().len() as u64)
                                .sum(),
                            depth: 0,
                            bits: 0,
                        })
                    },
                )
            },
        );
    }
    atlas.grid(
        "convex parts",
        ("part count", &powers_of_two(1, 6)),
        ("vertices per part", &[4.0, 8.0, 16.0, 32.0, 64.0]),
        |n, vertices, rng| nesting(n as usize, vertices as usize, 70.0, rng),
        |nest| IrregularPacking2::new(&nest.items).is_ok(),
    );

    atlas.operation(
        "irregular nesting",
        "IrregularPacking2::bottom_left",
        "part count",
        "Translation-only bottom-left nesting from cached no-fit vertices at 50% bounding-box fill; every candidate is an exact replay. The no-fit cache is built outside the timer.",
    );
    for vertices in [4usize, 8, 16] {
        atlas.sweep(
            &format!("{vertices}-gon parts"),
            &powers_of_two(1, 7),
            |n, rng| {
                let nest = nesting(n as usize, vertices, 50.0, rng);
                let packing = IrregularPacking2::new(&nest.items).expect("convex parts");
                (nest.bin, packing)
            },
            |(bin, packing)| {
                packing
                    .bottom_left(bin)
                    .map_err(|e| format!("{e:?}"))
                    .and_then(|report| match report.replay.status {
                        FeasibilityStatus::Feasible => Ok(report.placements.len()),
                        status => Err(format!("replay {status:?}")),
                    })
            },
        );
    }
    atlas.operation(
        "irregular nesting",
        "bottom_left vs fill",
        "fill percent",
        "Bottom-left nesting of 5 octagonal parts on sheets from 10% to 150% bounding-box fill.",
    );
    atlas.sweep(
        "8-gon parts",
        &FILLS,
        |fill, rng| {
            let nest = nesting(5, 8, fill, rng);
            let packing = IrregularPacking2::new(&nest.items).expect("convex parts");
            (nest.bin, packing)
        },
        |(bin, packing)| {
            packing
                .bottom_left(bin)
                .map_err(|e| format!("{e:?}"))
                .and_then(|report| match report.replay.status {
                    FeasibilityStatus::Feasible => Ok(report.placements.len()),
                    status => Err(format!("replay {status:?}")),
                })
        },
    );
    atlas.operation(
        "irregular nesting",
        "IrregularPacking2::verify",
        "part count",
        "Exact replay of a grid nest (parts in 8x8 cells, built outside the timer with the no-fit cache) against containment and cached no-fit regions.",
    );
    for vertices in [4usize, 16] {
        atlas.sweep(
            &format!("{vertices}-gon parts"),
            &powers_of_two(1, 7),
            |n, rng| {
                let nest = nesting(n as usize, vertices, 50.0, rng);
                let packing = IrregularPacking2::new(&nest.items).expect("convex parts");
                let columns = n.sqrt().ceil() as i64;
                let rows = (n as i64 + columns - 1) / columns;
                let placements: Vec<IrregularSheetPlacement2> = nest
                    .items
                    .iter()
                    .enumerate()
                    .map(|(i, item)| IrregularSheetPlacement2 {
                        item: item.id.clone(),
                        x: r(i as i64 % columns * 8),
                        y: r(i as i64 / columns * 8),
                    })
                    .collect();
                let bin = SheetBin2::new(Rect2::new(r(columns * 8), r(rows * 8)).unwrap());
                (bin, packing, placements)
            },
            |(bin, packing, placements)| {
                packing
                    .verify(bin, placements)
                    .map_err(|e| format!("{e:?}"))
                    .and_then(|report| match report.status {
                        FeasibilityStatus::Feasible => Ok(report.no_overlap_checks),
                        status => Err(format!("replay {status:?}")),
                    })
            },
        );
    }
}

fn main() {
    let mut atlas = Atlas::from_args("hyperpack");
    replay(&mut atlas);
    heuristics_2d(&mut atlas);
    heuristics_3d(&mut atlas);
    portfolios_and_search(&mut atlas);
    bounds_and_models(&mut atlas);
    nesting_ops(&mut atlas);
    atlas.finish();
}
