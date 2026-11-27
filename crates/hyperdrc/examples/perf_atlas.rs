//! Performance, memory and depth atlas for HyperDRC: design-rule evaluation
//! over synthetic PCB layouts, the broad-phase spatial indexes behind it,
//! spacing, width and enclosure rules near their thresholds, the exact region
//! operations the checks are built on, and the CAM imports that feed them.
//!
//! Run through `scripts/perf-atlas/run.sh`, or directly:
//! `cargo run --release -p hyperdrc --all-features --example perf_atlas -- --out hyperdrc.json [--quick] [--resources] [--filter TEXT]`.
//!
//! Axes:
//! - feature, drill, island or flash count (powers of two), for rule
//!   evaluation over growing layouts;
//! - clearance relative to the layout pitch, for broad-phase density (how many
//!   candidates each spatial-index query proposes);
//! - distance to the rule edge, `2^-k` mm, for spacing, width and enclosure
//!   rules decided on either side of the threshold. Exact kinds build native
//!   regions or drills from exact rationals and go to `k = 1024`; binary64
//!   kinds stop where a coordinate can no longer carry the offset;
//! - report samples per primitive, and operand polygon counts (also as grids)
//!   for region booleans.
//!
//! Input kinds: sparse and dense grids, dense grids that violate or narrowly
//! pass, clustered layouts, layouts whose bounding boxes overlap while the
//! copper does not (every candidate reaches the exact narrow phase), one-net
//! layouts (rejected before geometry), axis-aligned, 45° (irrational) and round
//! (native arc) features.
//!
//! Every input is built outside the timer, with fresh regions for each call,
//! so caches on an input region never carry over between timed calls. A check
//! that returns a `geometry-uncertainty` finding is marked as failed, and
//! near-threshold checks are also marked when the verdict is on the wrong side
//! of the rule. With `--resources`, check results report their finding count
//! and region results report their native boundary-curve count and nesting
//! depth (2 when the region has holes).
use std::cell::RefCell;
use std::collections::HashMap;
use std::path::PathBuf;

use hypercurve::{Contour2, CurveRegion2, LineSeg2, Point2, Segment2};
use hyperdrc::checks::{
    annular_ring, board_outline_drill_clearance, copper_width_readiness,
    different_net_short_readiness, drill_spacing, drill_to_copper_clearance, exposed_copper,
    mask_island_keepout, min_copper_neck_width, net_spacing, paste_aperture_spacing,
    silkscreen_clearance, skinny_layer_feature_readiness, solder_mask_annular_ring_readiness,
    solder_mask_expansion,
};
use hyperdrc::excellon::parse_excellon;
use hyperdrc::geometry::{
    Polygon, arc_line_polygons, bezier_line_polygons, circle_polygon, line_polygon,
    polygons_to_profile, rect_polygon, rounded_rect_polygon,
};
use hyperdrc::kicad::{BoardModel, CopperFeature, CopperKind, DrillFeature, load_kicad_pcb};
use hyperdrc::scalar::scalar;
use hyperdrc::{LayerMetadata, PcbRegion, PcbRegionExt, Scalar, Violation};
use hyperreal::{Rational, Real};
use perf_atlas::{Atlas, Rng, Shape, Shaped, powers_of_two};

#[global_allocator]
static ALLOCATOR: perf_atlas::CountingAllocator = perf_atlas::CountingAllocator;

/// Spacing rule used by every spacing check, in mm. A dyadic value, so
/// binary64 layouts can sit exactly at `CLEARANCE ± 2^-k`.
const CLEARANCE: f64 = 0.25;
/// Side of a square pad, in mm.
const PAD: f64 = 0.5;
/// Drill diameter, in mm.
const DRILL: f64 = 0.375;
/// Report coordinates are quantized to this grid (2^-10 mm), so exact lifts
/// stay short unless an axis asks for more bits.
const QUANTUM: f64 = 1.0 / 1024.0;

// ---------------------------------------------------------------------------
// Outcomes and shapes

type Findings = Shaped<Result<Vec<Violation>, String>>;

fn finding_shape(value: &Result<Vec<Violation>, String>) -> Option<Shape> {
    value.as_ref().ok().map(|findings| Shape {
        nodes: findings.len() as u64,
        depth: 0,
        bits: 0,
    })
}

fn uncertainty(findings: &[Violation]) -> Option<String> {
    findings
        .iter()
        .find(|finding| finding.check == "geometry-uncertainty")
        .map(|finding| finding.message.clone().unwrap_or_default())
}

/// A check's findings; a `geometry-uncertainty` finding marks the call failed.
fn findings(findings: Vec<Violation>) -> Findings {
    let value = match uncertainty(&findings) {
        Some(reason) => Err(reason),
        None => Ok(findings),
    };
    Shaped::new(value, finding_shape)
}

/// Like [`findings`], and also marks a verdict on the wrong side of the rule.
fn verdict(findings: Vec<Violation>, expect_finding: bool) -> Findings {
    let value = match uncertainty(&findings) {
        Some(reason) => Err(reason),
        None if findings.is_empty() == expect_finding => Err(if expect_finding {
            "expected a finding on the violating side of the rule".to_owned()
        } else {
            format!(
                "unexpected finding on the passing side of the rule: {}",
                findings[0].message.clone().unwrap_or_default()
            )
        }),
        None => Ok(findings),
    };
    Shaped::new(value, finding_shape)
}

fn region_shape(value: &Result<PcbRegion, String>) -> Option<Shape> {
    let native = value.as_ref().ok()?.clone().try_into_native().ok()?;
    let curves: usize = native
        .boundary_loops()
        .iter()
        .map(|boundary| boundary.curves().len())
        .sum();
    let holes = native.loop_role_counts().map(|(_, holes)| holes).ok()?;
    Some(Shape {
        nodes: curves as u64,
        depth: if holes > 0 { 2 } else { 1 },
        bits: 0,
    })
}

/// A region result; nonempty results are forced through exact emptiness.
fn region(value: Result<PcbRegion, impl std::fmt::Display>) -> Shaped<Result<PcbRegion, String>> {
    let value = value.map_err(|error| error.to_string()).and_then(|region| {
        region
            .try_is_empty()
            .map(|_| region)
            .map_err(|error| error.to_string())
    });
    Shaped::new(value, region_shape)
}

// ---------------------------------------------------------------------------
// Layout construction

fn meta(name: &str) -> Option<LayerMetadata> {
    Some(LayerMetadata {
        name: name.to_owned(),
    })
}

fn exact(value: f64) -> Scalar {
    Scalar::try_from(value).expect("finite atlas coordinate")
}

fn quantize(value: f64) -> f64 {
    (value / QUANTUM).round() * QUANTUM
}

fn profile(polygons: Vec<Polygon>, name: &str) -> PcbRegion {
    polygons_to_profile(polygons, meta(name))
}

fn feature(kind: CopperKind, net: &str, at: [f64; 2], polygon: Polygon) -> CopperFeature {
    CopperFeature {
        layer: "F.Cu".to_owned(),
        net: Some(net.to_owned()),
        kind,
        location: at.map(exact),
        region: profile(vec![polygon], "F.Cu"),
    }
}

fn pad(net: &str, at: [f64; 2], side: f64) -> CopperFeature {
    feature(
        CopperKind::Pad,
        net,
        at,
        rect_polygon(at, [side, side], 0.0),
    )
}

fn board(copper: Vec<CopperFeature>, drills: Vec<DrillFeature>) -> BoardModel {
    BoardModel {
        source: "atlas".to_owned(),
        copper,
        drills,
        board_outline: None,
        panel_features: None,
    }
}

fn drill(at: [Scalar; 2], diameter: Scalar, plated: bool) -> DrillFeature {
    DrillFeature {
        location: at,
        diameter,
        net: None,
        plated,
    }
}

/// Grid positions with `pitch`, row-major, and their checkerboard parity.
fn grid(count: usize, pitch: f64) -> Vec<([f64; 2], usize)> {
    let side = (count as f64).sqrt().ceil().max(1.0) as usize;
    (0..count)
        .map(|index| {
            let (column, row) = (index % side, index / side);
            (
                [column as f64 * pitch, row as f64 * pitch],
                (column + row) % 2,
            )
        })
        .collect()
}

/// Clusters of eight features around random centres, quantized to 2^-10 mm.
/// Cluster members can overlap; each cluster mixes two nets.
fn clustered(count: usize, pitch: f64, rng: &mut Rng) -> Vec<([f64; 2], usize)> {
    let frame = (count as f64).sqrt().ceil() * pitch * 2.0;
    let mut out = Vec::with_capacity(count);
    let mut centre = [0.0, 0.0];
    for index in 0..count {
        if index % 8 == 0 {
            centre = [rng.unit() * frame, rng.unit() * frame];
        }
        let offset = [
            (rng.unit() - 0.5) * pitch * 2.0,
            (rng.unit() - 0.5) * pitch * 2.0,
        ];
        out.push((
            [
                quantize(centre[0] + offset[0]),
                quantize(centre[1] + offset[1]),
            ],
            index % 2,
        ));
    }
    out
}

const LAYOUTS: [&str; 4] = [
    "sparse grid (gap 1.5 mm)",
    "dense grid, violating (gap 0.125 mm)",
    "diagonal traces, bounding boxes overlap, passing",
    "clustered",
];

/// A board of `count` copper features in one of [`LAYOUTS`], on two nets.
fn layout(kind: &str, count: usize, rng: &mut Rng) -> BoardModel {
    let nets = ["A", "B"];
    let copper = match kind {
        "sparse grid (gap 1.5 mm)" => grid(count, 2.0)
            .into_iter()
            .map(|(at, parity)| pad(nets[parity], at, PAD))
            .collect(),
        "dense grid, violating (gap 0.125 mm)" => grid(count, PAD + 0.125)
            .into_iter()
            .map(|(at, parity)| pad(nets[parity], at, PAD))
            .collect(),
        "diagonal traces, bounding boxes overlap, passing" => (0..count)
            .map(|index| {
                // 45° traces 0.125 mm wide on a 0.625 mm x pitch: their
                // centrelines are 0.442 mm apart, an edge gap of 0.317 mm,
                // while each bounding box overlaps two neighbours.
                let (column, row) = (index % 64, index / 64);
                let start = [column as f64 * 0.625, row as f64 * 2.0];
                let end = [start[0] + 1.0, start[1] + 1.0];
                let polygon = line_polygon(start, end, 0.125).expect("valid trace");
                feature(
                    CopperKind::Segment,
                    nets[index % 2],
                    [start[0] + 0.5, start[1] + 0.5],
                    polygon,
                )
            })
            .collect(),
        _ => clustered(count, PAD * 2.0, rng)
            .into_iter()
            .map(|(at, parity)| pad(nets[parity], at, PAD))
            .collect(),
    };
    board(copper, Vec::new())
}

/// `2^-k` as an exact scalar, for any `k`.
fn pow2_inverse(k: u32) -> Scalar {
    let mut denominator = Rational::one();
    let mut left = k;
    while left > 0 {
        let step = left.min(62);
        denominator = &denominator * &Rational::new(1_i64 << step);
        left -= step;
    }
    Real::new(&Rational::one() / &denominator)
}

/// An exact axis-aligned rectangle `[x0, x1] × [y0, y1]` as a native region.
fn exact_rect(x0: Scalar, y0: Scalar, x1: Scalar, y1: Scalar, name: &str) -> PcbRegion {
    let points = [
        Point2::new(x0.clone(), y0.clone()),
        Point2::new(x1.clone(), y0),
        Point2::new(x1, y1.clone()),
        Point2::new(x0, y1),
    ];
    let edges = points
        .iter()
        .cloned()
        .zip(points.iter().cloned().cycle().skip(1))
        .map(|(a, b)| LineSeg2::try_new(a, b).map(Segment2::Line))
        .collect::<Result<Vec<_>, _>>()
        .expect("nondegenerate rectangle edges");
    let contour = Contour2::try_new(edges).expect("closed rectangle contour");
    let region =
        CurveRegion2::try_from_native_material_contours(vec![contour]).expect("rectangle region");
    PcbRegion::new(region, meta(name))
}

fn exact_pad(net: &str, region: PcbRegion, at: [Scalar; 2]) -> CopperFeature {
    CopperFeature {
        layer: "F.Cu".to_owned(),
        net: Some(net.to_owned()),
        kind: CopperKind::Pad,
        location: at,
        region,
    }
}

/// Offset from the rule edge for near-threshold kinds: `±2^-k` mm, exactly,
/// or `±1/(3·2^k)` for the non-dyadic kind.
fn delta(k: u32, sign: i32, non_dyadic: bool) -> Scalar {
    let magnitude = if non_dyadic {
        (pow2_inverse(k) / Real::from(3_u8)).expect("nonzero divisor")
    } else {
        pow2_inverse(k)
    };
    if sign < 0 { -magnitude } else { magnitude }
}

/// Binary64 offsets stop where a coordinate near 1 mm can no longer carry them.
/// They start at `k = 4`, so a `−2^-k` offset never flips the sign of a
/// 0.125 mm width, ring or margin.
const F64_KS: [f64; 9] = [4.0, 8.0, 12.0, 16.0, 20.0, 24.0, 32.0, 40.0, 48.0];

fn exact_ks() -> Vec<f64> {
    powers_of_two(0, 10)
}

// ---------------------------------------------------------------------------
// Spacing rules

fn spacing(atlas: &mut Atlas) {
    let counts = powers_of_two(2, 12);
    let clearance = exact(CLEARANCE);
    let min_area = &scalar("1e-9");

    atlas.operation(
        "spacing",
        "net_spacing by feature count",
        "copper features",
        "Different-net copper spacing (0.25 mm) over a board of pads or traces on two nets: spatial-index candidates, bounding-box rejection, then exact overlap and boundary distance.",
    );
    for kind in LAYOUTS {
        atlas.sweep(
            kind,
            &counts,
            |n, rng| layout(kind, n as usize, rng),
            |board| findings(net_spacing(board, &clearance, &[], min_area)),
        );
    }
    atlas.fuzz(
        "random pitch and nets",
        &powers_of_two(2, 9),
        |n, rng| {
            let pitch = quantize(PAD + rng.unit() * 1.0);
            let nets = 1 + rng.below(4) as usize;
            let copper = grid(n as usize, pitch)
                .into_iter()
                .enumerate()
                .map(|(index, (at, _))| pad(&format!("N{}", (index * 7) % nets), at, PAD))
                .collect();
            board(copper, Vec::new())
        },
        |board| findings(net_spacing(board, &clearance, &[], min_area)),
    );

    atlas.operation(
        "spacing",
        "net_spacing by clearance",
        "clearance / pad pitch",
        "Different-net spacing over 256 pads (0.5 mm pads on a 1 mm pitch) as the clearance grows from a fraction of the gap to many pitches, so each spatial-index query proposes more candidates.",
    );
    // 0.5 is the pad gap: below it the index rejects every pair, above it
    // every neighbour within the clearance reaches the exact narrow phase.
    let ratios = [0.125, 0.25, 0.4375, 0.5625, 0.75, 1.0, 1.5, 2.0, 4.0, 8.0];
    for (kind, nets) in [
        ("checkerboard nets", 2_usize),
        ("every pad its own net", 0),
        ("one net (rejected before geometry)", 1),
    ] {
        atlas.sweep(
            kind,
            &ratios,
            move |ratio, _| {
                let copper = grid(256, 1.0)
                    .into_iter()
                    .enumerate()
                    .map(|(index, (at, parity))| {
                        let net = match nets {
                            0 => format!("N{index}"),
                            1 => "N".to_owned(),
                            _ => format!("N{parity}"),
                        };
                        pad(&net, at, PAD)
                    })
                    .collect();
                (board(copper, Vec::new()), exact(ratio))
            },
            |(board, clearance)| findings(net_spacing(board, clearance, &[], min_area)),
        );
    }

    atlas.operation(
        "spacing",
        "net_spacing near threshold",
        "distance to rule edge (2^-k mm)",
        "Two different-net pads whose gap is the clearance ± 2^-k mm. Exact rectangles carry the offset in rationals; 45° pads have irrational corners; round pads are native arcs. Marked failed when the verdict is on the wrong side.",
    );
    let exact_ks = exact_ks();
    for (kind, sign) in [
        ("exact rects, gap c − 2^-k (violates)", -1),
        ("exact rects, gap c + 2^-k (passes)", 1),
    ] {
        atlas.sweep(
            kind,
            &exact_ks,
            move |k, _| {
                let half = exact(PAD / 2.0);
                let left = exact_rect(
                    -half.clone(),
                    -half.clone(),
                    half.clone(),
                    half.clone(),
                    "F.Cu",
                );
                let x0 = half.clone() + exact(CLEARANCE) + delta(k as u32, sign, false);
                let x1 = x0.clone() + exact(PAD);
                let right = exact_rect(x0, -half.clone(), x1, half, "F.Cu");
                board(
                    vec![
                        exact_pad("A", left, [exact(0.0), exact(0.0)]),
                        exact_pad("B", right, [exact(PAD + CLEARANCE), exact(0.0)]),
                    ],
                    Vec::new(),
                )
            },
            move |board| {
                verdict(
                    net_spacing(board, &exact(CLEARANCE), &[], min_area),
                    sign < 0,
                )
            },
        );
    }
    for (kind, round) in [
        ("45° pads, gap c + 2^-k (passes)", false),
        ("round pads, gap c + 2^-k (passes)", true),
    ] {
        atlas.sweep(
            kind,
            &F64_KS,
            move |k, _| {
                let offset = 2f64.powi(-(k as i32));
                let (left, right, dx) = if round {
                    let dx = PAD + CLEARANCE + offset;
                    (
                        circle_polygon([0.0, 0.0], PAD / 2.0, 32),
                        circle_polygon([dx, 0.0], PAD / 2.0, 32),
                        dx,
                    )
                } else {
                    // Corner to corner along x: each 45° square reaches
                    // PAD/√2 from its centre.
                    let dx = PAD * std::f64::consts::SQRT_2 + CLEARANCE + offset;
                    (
                        rect_polygon([0.0, 0.0], [PAD, PAD], 45.0),
                        rect_polygon([dx, 0.0], [PAD, PAD], 45.0),
                        dx,
                    )
                };
                board(
                    vec![
                        feature(CopperKind::Pad, "A", [0.0, 0.0], left),
                        feature(CopperKind::Pad, "B", [dx, 0.0], right),
                    ],
                    Vec::new(),
                )
            },
            move |board| verdict(net_spacing(board, &exact(CLEARANCE), &[], min_area), false),
        );
    }

    atlas.operation(
        "spacing",
        "drill_spacing by drill count",
        "drills",
        "Hole-to-hole spacing (0.25 mm) through the drill spatial index. The large-clearance kind puts every drill in one candidate neighbourhood, so it shows the index's quadratic fallback.",
    );
    let drill_counts = powers_of_two(4, 16);
    for kind in [
        "sparse grid (pitch 2 mm)",
        "dense grid, violating (gap 0.125 mm)",
        "clustered",
        "clearance spans the board",
    ] {
        atlas.sweep(
            kind,
            &drill_counts,
            move |n, rng| {
                let n = n as usize;
                let positions = match kind {
                    "sparse grid (pitch 2 mm)" => grid(n, 2.0),
                    "dense grid, violating (gap 0.125 mm)" => grid(n, DRILL + 0.125),
                    "clustered" => clustered(n, 1.0, rng),
                    _ => grid(n, 2.0),
                };
                let side = (n as f64).sqrt().ceil() * 2.0;
                let clearance = if kind == "clearance spans the board" {
                    exact(side * 2.0)
                } else {
                    exact(CLEARANCE)
                };
                let drills = positions
                    .into_iter()
                    .map(|(at, _)| drill(at.map(exact), exact(DRILL), true))
                    .collect::<Vec<_>>();
                (drills, clearance)
            },
            |(drills, clearance)| findings(drill_spacing(drills, &[], clearance)),
        );
    }
    atlas.fuzz(
        "random holes",
        &powers_of_two(4, 12),
        |n, rng| {
            let side = (n.sqrt() * 1.5).max(1.0);
            (0..n as usize)
                .map(|_| {
                    let at = [quantize(rng.unit() * side), quantize(rng.unit() * side)];
                    let diameter = quantize(0.2 + rng.unit() * 0.6);
                    drill(at.map(exact), exact(diameter), rng.coin())
                })
                .collect::<Vec<_>>()
        },
        |drills| findings(drill_spacing(drills, &[], &exact(CLEARANCE))),
    );

    atlas.operation(
        "spacing",
        "drill_spacing near threshold",
        "distance to rule edge (2^-k mm)",
        "Two exact drills whose edge gap is the clearance ± 2^-k mm (or ± 1/(3·2^k) mm); coordinates are exact rationals, so the decision is made at k bits. Marked failed when the verdict is on the wrong side.",
    );
    for (kind, sign, non_dyadic) in [
        ("gap c − 2^-k (violates)", -1, false),
        ("gap c + 2^-k (passes)", 1, false),
        ("gap c + 1/(3·2^k) (passes)", 1, true),
    ] {
        atlas.sweep(
            kind,
            &exact_ks,
            move |k, _| {
                let diameter = scalar("0.4");
                let x = diameter.clone() + scalar("0.2") + delta(k as u32, sign, non_dyadic);
                vec![
                    drill([exact(0.0), exact(0.0)], diameter.clone(), true),
                    drill([x, exact(0.0)], diameter, true),
                ]
            },
            move |drills| verdict(drill_spacing(drills, &[], &scalar("0.2")), sign < 0),
        );
    }

    atlas.operation(
        "spacing",
        "drill_to_copper_clearance",
        "pads (one unplated drill each)",
        "Hole-to-copper clearance (0.25 mm) for a grid of pads, each with an unplated hole beside it: indexed candidates, then exact distance from the drill circle to copper.",
    );
    for (kind, gap) in [
        ("holes clear of copper (gap 0.5 mm)", 0.5),
        ("holes violating (gap 0.125 mm)", 0.125),
    ] {
        atlas.sweep(
            kind,
            &powers_of_two(2, 11),
            move |n, _| {
                let mut copper = Vec::new();
                let mut drills = Vec::new();
                for (at, parity) in grid(n as usize, 2.5) {
                    copper.push(pad(["A", "B"][parity], at, PAD));
                    let x = at[0] + PAD / 2.0 + gap + DRILL / 2.0;
                    drills.push(drill([exact(x), exact(at[1])], exact(DRILL), false));
                }
                board(copper, drills)
            },
            |board| {
                findings(drill_to_copper_clearance(
                    board,
                    &[],
                    &exact(CLEARANCE),
                    &[],
                    min_area,
                ))
            },
        );
    }

    atlas.operation(
        "spacing",
        "board_outline_drill_clearance",
        "drills",
        "Drill-to-board-edge clearance (0.25 mm) against a rectangular outline sized to the drill field.",
    );
    for (kind, edge) in [
        ("interior holes", 2.0),
        ("holes hugging the edge (violating)", DRILL / 2.0 + 0.125),
    ] {
        atlas.sweep(
            kind,
            &powers_of_two(4, 14),
            move |n, _| {
                let positions = grid(n as usize, 2.0);
                let side = n.sqrt().ceil() * 2.0 - 2.0;
                let outline = profile(
                    vec![rect_polygon(
                        [side / 2.0, side / 2.0],
                        [side + 2.0 * edge, side + 2.0 * edge],
                        0.0,
                    )],
                    "Edge.Cuts",
                );
                let drills = positions
                    .into_iter()
                    .map(|(at, _)| drill(at.map(exact), exact(DRILL), true))
                    .collect::<Vec<_>>();
                (outline, drills)
            },
            |(outline, drills)| {
                findings(board_outline_drill_clearance(
                    "drills",
                    "Edge.Cuts",
                    outline,
                    drills,
                    &[],
                    &exact(CLEARANCE),
                    min_area,
                ))
            },
        );
    }

    type LayerSpacing = fn(&str, &PcbRegion, &Scalar, &Scalar) -> Vec<Violation>;
    let layer_checks: [(&str, &str, LayerSpacing); 2] = [
        (
            "mask_island_keepout",
            "Solder-mask island keepout (twice 0.125 mm) over a layer of rectangular islands, through the layer polygon spatial index.",
            mask_island_keepout,
        ),
        (
            "paste_aperture_spacing",
            "Paste aperture spacing (0.25 mm) over a layer of rectangular apertures, through the layer polygon spatial index.",
            paste_aperture_spacing,
        ),
    ];
    for (name, description, check) in layer_checks {
        atlas.operation("spacing", name, "islands", description);
        for (kind, pitch) in [
            ("sparse (gap 1.5 mm)", 2.0),
            ("dense, violating (gap 0.125 mm)", PAD + 0.125),
            ("dense, passing (gap 0.375 mm)", PAD + 0.375),
        ] {
            atlas.sweep(
                kind,
                &powers_of_two(2, 10),
                move |n, _| {
                    let polygons = grid(n as usize, pitch)
                        .into_iter()
                        .map(|(at, _)| rect_polygon(at, [PAD, PAD], 0.0))
                        .collect();
                    profile(polygons, "F.Mask")
                },
                move |layer| {
                    let rule = if name == "mask_island_keepout" {
                        exact(CLEARANCE / 2.0)
                    } else {
                        exact(CLEARANCE)
                    };
                    findings(check(name, layer, &rule, min_area))
                },
            );
        }
    }

    atlas.operation(
        "spacing",
        "different_net_short_readiness",
        "copper features",
        "Different-net overlap (short) detection: pads in pairs, either overlapping a different-net neighbour or cleanly separated.",
    );
    for (kind, overlap) in [
        ("separated pairs", false),
        ("overlapping pairs (shorts)", true),
    ] {
        atlas.sweep(
            kind,
            &counts,
            move |n, _| {
                let copper = grid(n as usize, 2.0)
                    .into_iter()
                    .enumerate()
                    .map(|(index, (at, _))| {
                        let shift = if index % 2 == 1 && overlap {
                            -1.75
                        } else {
                            0.0
                        };
                        pad(
                            if index % 2 == 0 { "A" } else { "B" },
                            [at[0] + shift, at[1]],
                            PAD,
                        )
                    })
                    .collect();
                board(copper, Vec::new())
            },
            |board| findings(different_net_short_readiness(board, &[], min_area)),
        );
    }

    atlas.operation(
        "spacing",
        "silkscreen_clearance",
        "pads (one silkscreen line each)",
        "Silkscreen-to-pad clearance (0.25 mm): a silkscreen line beside every pad, either clear or intruding, through an indexed intersection with the expanded pads.",
    );
    for (kind, gap) in [
        ("silk clear (gap 0.5 mm)", 0.5),
        ("silk intruding (gap 0.125 mm)", 0.125),
    ] {
        atlas.sweep(
            kind,
            &powers_of_two(2, 10),
            move |n, _| {
                let positions = grid(n as usize, 2.5);
                let pads = positions
                    .iter()
                    .map(|(at, _)| rect_polygon(*at, [PAD, PAD], 0.0))
                    .collect();
                let silk = positions
                    .iter()
                    .filter_map(|(at, _)| {
                        let x = at[0] + PAD / 2.0 + gap + 0.0625;
                        line_polygon([x, at[1] - 0.5], [x, at[1] + 0.5], 0.125)
                    })
                    .collect();
                (profile(silk, "F.Silkscreen"), profile(pads, "F.Mask"))
            },
            |(silk, pads)| {
                findings(silkscreen_clearance(
                    "F.Silkscreen",
                    silk,
                    "F.Mask",
                    pads,
                    &exact(CLEARANCE),
                    min_area,
                ))
            },
        );
    }
}

// ---------------------------------------------------------------------------
// Width rules

/// Two square pads joined by a neck of width `neck`, as one polygon set.
fn dumbbell(at: [f64; 2], neck: f64) -> Vec<Polygon> {
    vec![
        rect_polygon(at, [0.75, 0.75], 0.0),
        rect_polygon([at[0] + 1.5, at[1]], [0.75, 0.75], 0.0),
        rect_polygon([at[0] + 0.75, at[1]], [1.0, neck], 0.0),
    ]
}

fn width(atlas: &mut Atlas) {
    let min_area = &scalar("1e-9");
    let min_width = exact(0.125);

    atlas.operation(
        "width",
        "copper_width_readiness",
        "traces",
        "Minimum copper width (0.125 mm) over traces, measured per feature by its minimum bounding dimension.",
    );
    for (kind, trace_width, angle) in [
        ("axis-aligned, passing (0.25 mm)", 0.25, false),
        ("axis-aligned, violating (0.0625 mm)", 0.0625, false),
        ("45° traces, passing (0.25 mm)", 0.25, true),
    ] {
        atlas.sweep(
            kind,
            &powers_of_two(2, 12),
            move |n, _| {
                let copper = grid(n as usize, 2.0)
                    .into_iter()
                    .map(|(at, _)| {
                        let end = if angle {
                            [at[0] + 1.0, at[1] + 1.0]
                        } else {
                            [at[0] + 1.5, at[1]]
                        };
                        let polygon = line_polygon(at, end, trace_width).expect("valid trace");
                        feature(CopperKind::Segment, "A", at, polygon)
                    })
                    .collect();
                board(copper, Vec::new())
            },
            |board| findings(copper_width_readiness(board, &[], &min_width)),
        );
    }

    atlas.operation(
        "width",
        "copper_width near threshold",
        "distance to rule edge (2^-k mm)",
        "One trace whose width is the 0.125 mm minimum ± 2^-k mm. Marked failed when the verdict is on the wrong side.",
    );
    for (kind, sign, angle) in [
        ("axis-aligned, w − 2^-k (violates)", -1.0, false),
        ("axis-aligned, w + 2^-k (passes)", 1.0, false),
        ("45° trace, w + 2^-k (passes)", 1.0, true),
    ] {
        atlas.sweep(
            kind,
            &F64_KS,
            move |k, _| {
                let trace_width = 0.125 + sign * 2f64.powi(-(k as i32));
                let end = if angle { [1.0, 1.0] } else { [1.5, 0.0] };
                let polygon = line_polygon([0.0, 0.0], end, trace_width).expect("valid trace");
                board(
                    vec![feature(CopperKind::Segment, "A", [0.0, 0.0], polygon)],
                    Vec::new(),
                )
            },
            move |board| {
                verdict(
                    copper_width_readiness(board, &[], &exact(0.125)),
                    sign < 0.0,
                )
            },
        );
    }

    atlas.operation(
        "width",
        "min_copper_neck_width",
        "copper polygons",
        "Minimum copper neck (0.125 mm) by morphological opening of the whole layer: erode by half the width, dilate back, and report copper the opening removed.",
    );
    for (kind, neck) in [
        ("plain pads", None),
        ("dumbbells, wide necks (0.25 mm)", Some(0.25)),
        ("dumbbells, thin necks (0.0625 mm)", Some(0.0625)),
    ] {
        atlas.sweep(
            kind,
            &powers_of_two(0, 9),
            move |n, _| {
                let polygons = grid(n as usize, 3.0)
                    .into_iter()
                    .flat_map(|(at, _)| match neck {
                        Some(neck) => dumbbell(at, neck),
                        None => vec![rect_polygon(at, [0.75, 0.75], 0.0)],
                    })
                    .collect();
                profile(polygons, "F.Cu")
            },
            |layer| {
                findings(min_copper_neck_width(
                    "F.Cu",
                    layer,
                    &exact(0.125),
                    min_area,
                ))
            },
        );
    }

    atlas.operation(
        "width",
        "min_copper_neck_width near threshold",
        "distance to rule edge (2^-k mm)",
        "One dumbbell whose neck is the 0.125 mm minimum ± 2^-k mm. Marked failed when the verdict is on the wrong side.",
    );
    for (kind, sign) in [
        ("neck w − 2^-k (violates)", -1.0),
        ("neck w + 2^-k (passes)", 1.0),
    ] {
        atlas.sweep(
            kind,
            &F64_KS,
            move |k, _| {
                let neck = 0.125 + sign * 2f64.powi(-(k as i32));
                profile(dumbbell([0.0, 0.0], neck), "F.Cu")
            },
            move |layer| {
                verdict(
                    min_copper_neck_width("F.Cu", layer, &exact(0.125), min_area),
                    sign < 0.0,
                )
            },
        );
    }

    atlas.operation(
        "width",
        "skinny_layer_feature_readiness",
        "layer features",
        "Skinny-feature review (0.125 mm) over a layer of slivers, narrow either way.",
    );
    for (kind, size) in [
        ("passing slivers (0.5 × 0.25 mm)", [0.5, 0.25]),
        ("skinny slivers (0.5 × 0.0625 mm)", [0.5, 0.0625]),
    ] {
        atlas.sweep(
            kind,
            &powers_of_two(2, 10),
            move |n, _| {
                let polygons = grid(n as usize, 1.0)
                    .into_iter()
                    .map(|(at, _)| rect_polygon(at, size, 0.0))
                    .collect();
                profile(polygons, "F.Cu")
            },
            |layer| {
                findings(skinny_layer_feature_readiness(
                    "F.Cu",
                    layer,
                    &exact(0.125),
                    min_area,
                ))
            },
        );
    }
}

// ---------------------------------------------------------------------------
// Enclosure rules

fn enclosure(atlas: &mut Atlas) {
    let min_area = &scalar("1e-9");
    let min_ring = exact(0.125);
    // Near-threshold verdicts would otherwise vanish below the area filter.
    let no_area = &scalar("0");

    atlas.operation(
        "enclosure",
        "annular_ring",
        "plated holes",
        "Annular ring (0.125 mm) of every plated hole against its nearest land, from the land's area-equivalent radius.",
    );
    for (kind, round, land) in [
        ("square lands, passing (0.875 mm)", false, 0.875),
        ("square lands, violating (0.5 mm)", false, 0.5),
        ("round lands, passing (0.75 mm)", true, 0.75),
    ] {
        let counts = if round {
            powers_of_two(2, 8)
        } else {
            powers_of_two(2, 10)
        };
        atlas.sweep(
            kind,
            &counts,
            move |n, _| {
                let mut copper = Vec::new();
                let mut drills = Vec::new();
                for (at, _) in grid(n as usize, 2.0) {
                    let polygon = if round {
                        circle_polygon(at, land / 2.0, 32)
                    } else {
                        rect_polygon(at, [land, land], 0.0)
                    };
                    copper.push(feature(CopperKind::Pad, "A", at, polygon));
                    drills.push(drill(at.map(exact), exact(DRILL), true));
                }
                board(copper, drills)
            },
            |board| findings(annular_ring(board, &min_ring, &[])),
        );
    }

    atlas.operation(
        "enclosure",
        "annular_ring near threshold",
        "distance to rule edge (2^-k mm)",
        "One plated hole in a round land whose radius is hole radius + 0.125 mm ± 2^-k mm; the area-equivalent radius is sqrt(area / π) of a native circle. Marked failed when the verdict is on the wrong side.",
    );
    for (kind, sign) in [
        ("ring r − 2^-k (violates)", -1.0),
        ("ring r + 2^-k (passes)", 1.0),
    ] {
        atlas.sweep(
            kind,
            &F64_KS,
            move |k, _| {
                let radius = DRILL / 2.0 + 0.125 + sign * 2f64.powi(-(k as i32));
                board(
                    vec![feature(
                        CopperKind::Pad,
                        "A",
                        [0.0, 0.0],
                        circle_polygon([0.0, 0.0], radius, 32),
                    )],
                    vec![drill([exact(0.0), exact(0.0)], exact(DRILL), true)],
                )
            },
            move |board| verdict(annular_ring(board, &exact(0.125), &[]), sign < 0.0),
        );
    }

    type MaskCheck = fn(&str, &PcbRegion, &str, &PcbRegion, &Scalar, &Scalar) -> Vec<Violation>;
    let mask_checks: [(&str, &str, MaskCheck, f64, f64); 3] = [
        (
            "solder_mask_expansion",
            "Solder-mask openings larger than copper expanded by the 0.125 mm maximum: exact componentwise difference of openings and offset copper.",
            solder_mask_expansion,
            0.0625,
            0.25,
        ),
        (
            "solder_mask_annular_ring_readiness",
            "Solder-mask openings that do not cover copper expanded by the 0.125 mm minimum mask ring.",
            solder_mask_annular_ring_readiness,
            0.25,
            0.0625,
        ),
        (
            "exposed_copper",
            "Copper exposed through solder-mask openings: indexed exact intersection of the copper and mask layers.",
            |copper_name, copper, mask_name, mask, _, min_area| {
                exposed_copper(copper_name, copper, mask_name, mask, min_area)
            },
            0.0625,
            0.25,
        ),
    ];
    for (name, description, check, passing, violating) in mask_checks {
        atlas.operation(
            "enclosure",
            name,
            "pads (one mask opening each)",
            description,
        );
        for (kind, margin) in [
            (format!("opening margin {passing} mm"), passing),
            (format!("opening margin {violating} mm"), violating),
        ] {
            atlas.sweep(
                &kind,
                &powers_of_two(0, 9),
                move |n, _| mask_layers(n as usize, margin),
                move |(copper, mask)| {
                    findings(check(
                        "F.Cu",
                        copper,
                        "F.Mask",
                        mask,
                        &exact(0.125),
                        min_area,
                    ))
                },
            );
        }
        if name == "exposed_copper" {
            atlas.grid(
                "margin 0.0625 mm",
                ("copper pads", &powers_of_two(0, 7)),
                ("mask openings", &powers_of_two(0, 7)),
                |pads, openings, _| {
                    let (copper, _) = mask_layers(pads as usize, 0.0625);
                    let (_, mask) = mask_layers(openings as usize, 0.0625);
                    (copper, mask)
                },
                move |(copper, mask)| {
                    findings(check(
                        "F.Cu",
                        copper,
                        "F.Mask",
                        mask,
                        &exact(0.125),
                        min_area,
                    ))
                },
            );
        } else {
            atlas.operation(
                "enclosure",
                &format!("{name} near threshold"),
                "distance to rule edge (2^-k mm)",
                &format!(
                    "One pad and a rounded mask opening (the pad's exact round-corner offset by its margin) whose margin is the 0.125 mm rule ± 2^-k mm, with no minimum finding area ({name}). Marked failed when the verdict is on the wrong side."
                ),
            );
            for (kind, sign) in [("margin m − 2^-k", -1.0), ("margin m + 2^-k", 1.0)] {
                // Expansion violates above its maximum; the mask ring
                // violates below its minimum.
                let expect_finding = (name == "solder_mask_expansion") == (sign > 0.0);
                let kind = format!(
                    "{kind} ({})",
                    if expect_finding { "violates" } else { "passes" }
                );
                atlas.sweep(
                    &kind,
                    &F64_KS,
                    move |k, _| rounded_mask_layers(0.125 + sign * 2f64.powi(-(k as i32))),
                    move |(copper, mask)| {
                        verdict(
                            check("F.Cu", copper, "F.Mask", mask, &exact(0.125), no_area),
                            expect_finding,
                        )
                    },
                );
            }
        }
    }
}

/// One square pad and its exact round-corner offset by `margin` as the mask
/// opening: square openings exceed a round-corner expansion at the corners.
fn rounded_mask_layers(margin: f64) -> (PcbRegion, PcbRegion) {
    let copper = rect_polygon([0.0, 0.0], [PAD, PAD], 0.0);
    let side = PAD + 2.0 * margin;
    let mask = rounded_rect_polygon([0.0, 0.0], [side, side], margin, 0.0, 8);
    (profile(vec![copper], "F.Cu"), profile(vec![mask], "F.Mask"))
}

/// `count` square pads and a square mask opening around each, `margin` mm
/// larger on every side.
fn mask_layers(count: usize, margin: f64) -> (PcbRegion, PcbRegion) {
    let positions = grid(count, 2.0);
    let copper = positions
        .iter()
        .map(|(at, _)| rect_polygon(*at, [PAD, PAD], 0.0))
        .collect();
    let mask = positions
        .iter()
        .map(|(at, _)| rect_polygon(*at, [PAD + 2.0 * margin, PAD + 2.0 * margin], 0.0))
        .collect();
    (profile(copper, "F.Cu"), profile(mask, "F.Mask"))
}

// ---------------------------------------------------------------------------
// Region geometry

const SHAPES: [&str; 4] = [
    "disjoint squares",
    "overlapping squares (chains)",
    "30° rotated squares",
    "circles (32 report samples)",
];

fn shape_polygons(kind: &str, count: usize) -> Vec<Polygon> {
    grid(count, 1.0)
        .into_iter()
        .map(|(at, _)| match kind {
            "disjoint squares" => rect_polygon(at, [PAD, PAD], 0.0),
            "overlapping squares (chains)" => rect_polygon(at, [1.25, 0.75], 0.0),
            "30° rotated squares" => rect_polygon(at, [PAD, PAD], 30.0),
            _ => circle_polygon(at, PAD / 2.0, 32),
        })
        .collect()
}

/// `count` squares on a 1 mm grid, shifted by `shift` (so two such layers
/// overlap when `shift` is less than the square side).
fn squares(count: usize, shift: f64, name: &str) -> PcbRegion {
    let polygons = grid(count, 1.0)
        .into_iter()
        .map(|(at, _)| rect_polygon([at[0] + shift, at[1] + shift], [0.75, 0.75], 0.0))
        .collect();
    profile(polygons, name)
}

fn geometry(atlas: &mut Atlas) {
    atlas.operation(
        "geometry",
        "polygons_to_profile",
        "polygons",
        "Aggregate exact polygons into one native layer region (the construction every check input goes through). Overlapping squares merge into chains along each row.",
    );
    for kind in SHAPES {
        atlas.sweep(
            kind,
            &powers_of_two(1, 10),
            move |n, _| shape_polygons(kind, n as usize),
            |polygons| region(Ok::<_, String>(profile(polygons.clone(), "F.Cu"))),
        );
    }

    type Boolean =
        fn(&PcbRegion, &PcbRegion) -> Result<PcbRegion, hyperdrc::PcbGeometryUncertainty>;
    let booleans: [(&str, Boolean); 4] = [
        ("try_union", PcbRegion::try_union),
        ("try_intersection", PcbRegion::try_intersection),
        ("try_difference", PcbRegion::try_difference),
        ("try_xor", PcbRegion::try_xor),
    ];
    for (name, op) in booleans {
        atlas.operation(
            "geometry",
            name,
            "squares per operand",
            &format!(
                "Certified regularized {} of two layers of 0.75 mm squares on a 1 mm grid, the second shifted (overlapping by a quarter, or disjoint).",
                name.trim_start_matches("try_")
            ),
        );
        for (kind, shift) in [
            ("overlapping (shift 0.5 mm)", 0.5),
            ("interleaved, disjoint (shift 0.875 mm)", 0.875),
        ] {
            atlas.sweep(
                kind,
                &powers_of_two(0, 10),
                move |n, _| {
                    (
                        squares(n as usize, 0.0, "A"),
                        squares(n as usize, shift, "B"),
                    )
                },
                move |(a, b)| region(op(a, b)),
            );
        }
        if name == "try_union" {
            atlas.grid(
                "overlapping (shift 0.5 mm)",
                ("left squares", &powers_of_two(0, 7)),
                ("right squares", &powers_of_two(0, 7)),
                |a, b, _| (squares(a as usize, 0.0, "A"), squares(b as usize, 0.5, "B")),
                move |(a, b)| region(op(a, b)),
            );
            atlas.fuzz(
                "random squares",
                &powers_of_two(0, 7),
                |n, rng| {
                    let side = n.sqrt().ceil() * 1.5;
                    let mut random = |name: &str| {
                        let polygons = (0..n as usize)
                            .map(|_| {
                                let at = [quantize(rng.unit() * side), quantize(rng.unit() * side)];
                                let size =
                                    [quantize(0.25 + rng.unit()), quantize(0.25 + rng.unit())];
                                rect_polygon(at, size, 0.0)
                            })
                            .collect();
                        profile(polygons, name)
                    };
                    (random("A"), random("B"))
                },
                move |(a, b)| region(op(a, b)),
            );
        }
    }

    atlas.operation(
        "geometry",
        "offset",
        "polygons",
        "Round-corner offset of a layer region (the clearance and mask-expansion primitive): dilation merges neighbours, erosion shrinks or removes them.",
    );
    for (kind, shape, distance) in [
        (
            "disjoint squares, dilate 0.125 mm",
            "disjoint squares",
            0.125,
        ),
        (
            "disjoint squares, dilate 0.375 mm (merging)",
            "disjoint squares",
            0.375,
        ),
        (
            "disjoint squares, erode 0.125 mm",
            "disjoint squares",
            -0.125,
        ),
        (
            "30° rotated squares, dilate 0.125 mm",
            "30° rotated squares",
            0.125,
        ),
    ] {
        atlas.sweep(
            kind,
            &powers_of_two(0, 9),
            move |n, _| profile(shape_polygons(shape, n as usize), "F.Cu"),
            move |layer| region(layer.offset(exact(distance))),
        );
    }

    atlas.operation(
        "geometry",
        "try_area",
        "polygons",
        "Exact filled area of a layer region.",
    );
    for kind in SHAPES {
        atlas.sweep(
            kind,
            &powers_of_two(1, 9),
            move |n, _| profile(shape_polygons(kind, n as usize), "F.Cu"),
            |layer| layer.try_area().ok().map(|area| format!("{area:.6e}")),
        );
    }

    atlas.operation(
        "geometry",
        "try_to_multipolygon",
        "squares per operand",
        "Report projection (finite polygons with holes) of a fresh union result, whose projection cache is cold.",
    );
    atlas.sweep(
        "union of overlapping layers",
        &powers_of_two(0, 9),
        |n, _| {
            squares(n as usize, 0.0, "A")
                .try_union(&squares(n as usize, 0.5, "B"))
                .expect("union")
        },
        |layer| {
            layer
                .try_to_multipolygon()
                .ok()
                .map(|polygons| polygons.0.len())
        },
    );

    atlas.operation(
        "geometry",
        "classify_xy near boundary",
        "distance to boundary (2^-k mm)",
        "Exact point classification 2^-k mm inside a boundary: an exact square's edge (rational point), a 45° square's edge (irrational boundary), and a native circle.",
    );
    let exact_ks = exact_ks();
    atlas.sweep(
        "exact square edge",
        &exact_ks,
        |k, _| {
            let square = exact_rect(exact(-0.5), exact(-0.5), exact(0.5), exact(0.5), "F.Cu");
            let x = exact(0.5) - pow2_inverse(k as u32);
            (square, x, exact(0.0))
        },
        |(square, x, y)| {
            square
                .classify_xy(x.clone(), y.clone())
                .ok()
                .map(|place| format!("{place:?}"))
        },
    );
    for (kind, round) in [("45° square edge", false), ("circle", true)] {
        atlas.sweep(
            kind,
            &F64_KS,
            move |k, _| {
                let polygon = if round {
                    circle_polygon([0.0, 0.0], 0.5, 32)
                } else {
                    rect_polygon([0.0, 0.0], [1.0, 1.0], 45.0)
                };
                // Along the diagonal a 45° square's edge is 0.5 mm from its
                // centre in both coordinates' sum, x + y = √2/2.
                let x = if round {
                    0.5 - 2f64.powi(-(k as i32))
                } else {
                    std::f64::consts::FRAC_1_SQRT_2 - 2f64.powi(-(k as i32))
                };
                (profile(vec![polygon], "F.Cu"), exact(x), exact(0.0))
            },
            |(layer, x, y)| {
                layer
                    .classify_xy(x.clone(), y.clone())
                    .ok()
                    .map(|place| format!("{place:?}"))
            },
        );
    }

    atlas.operation(
        "geometry",
        "primitive construction",
        "report samples",
        "Construct one pad or trace primitive with native arcs or curves and a report ring of the given sample count.",
    );
    let samples = powers_of_two(2, 12);
    atlas.sweep(
        "circle",
        &samples,
        |n, _| n as usize,
        |&n| circle_polygon([0.0, 0.0], 0.5, n).try_area().ok(),
    );
    atlas.sweep(
        "rounded rectangle (samples per corner)",
        &samples,
        |n, _| n as usize,
        |&n| {
            rounded_rect_polygon([0.0, 0.0], [1.0, 0.5], 0.125, 15.0, n)
                .try_area()
                .ok()
        },
    );
    atlas.sweep(
        "135° arc trace",
        &samples,
        |n, _| n as usize,
        |&n| {
            let polygons = arc_line_polygons([0.0, 0.0], [1.0, 0.0], 135.0, 0.125, n);
            (!polygons.is_empty()).then_some(polygons.len())
        },
    );
    atlas.sweep(
        "cubic Bézier trace",
        &samples,
        |n, _| n as usize,
        |&n| {
            let polygons =
                bezier_line_polygons(&[[0.0, 0.0], [0.0, 1.0], [2.0, 1.0], [2.0, 0.0]], 0.125, n);
            (!polygons.is_empty()).then_some(polygons.len())
        },
    );
}

// ---------------------------------------------------------------------------
// CAM imports

/// A Gerber layer of `count` features on a 1 mm grid.
fn gerber(kind: &str, count: usize) -> String {
    let mut out = String::from(
        "%FSLAX46Y46*%\n%MOMM*%\n%ADD10C,0.500000*%\n%ADD11R,0.500000X0.500000*%\n%ADD12C,0.150000*%\n%LPD*%\n",
    );
    out.push_str(match kind {
        "round flashes" => "D10*\n",
        "traces" => "D12*\n",
        _ => "D11*\n",
    });
    let pitch = if kind == "overlapping square flashes" {
        0.375
    } else {
        1.0
    };
    for ([x, y], _) in grid(count, pitch) {
        let (x, y) = ((x * 1e6) as i64, (y * 1e6) as i64);
        if kind == "traces" {
            out.push_str(&format!("X{x}Y{y}D02*\nX{}Y{y}D01*\n", x + 750_000));
        } else {
            out.push_str(&format!("X{x}Y{y}D03*\n"));
        }
    }
    out.push_str("M02*\n");
    out
}

fn excellon(count: usize) -> String {
    let mut out = String::from("M48\nMETRIC,TZ\nT01C0.400\nT02C0.800\n%\nT01\n");
    for (index, ([x, y], _)) in grid(count, 1.0).into_iter().enumerate() {
        if index == count / 2 {
            out.push_str("T02\n");
        }
        out.push_str(&format!(
            "X{:06}Y{:06}\n",
            (x * 1000.0) as i64,
            (y * 1000.0) as i64
        ));
    }
    out.push_str("M30\n");
    out
}

fn kicad(kind: &str, count: usize) -> String {
    let mut out = String::from(
        "(kicad_pcb (version 20240108) (generator pcbnew)\n (layers (0 \"F.Cu\" signal) (31 \"B.Cu\" signal))\n (net 0 \"\") (net 1 \"A\") (net 2 \"B\")\n",
    );
    for ([x, y], parity) in grid(count, 2.0) {
        let net = parity + 1;
        if kind == "segments" {
            out.push_str(&format!(
                " (segment (start {x} {y}) (end {} {y}) (width 0.25) (layer \"F.Cu\") (net {net}))\n",
                x + 1.5
            ));
        } else {
            out.push_str(&format!(
                " (footprint \"R\" (layer \"F.Cu\") (at {x} {y})\n  (pad \"1\" smd rect (at 0 0) (size 0.5 0.5) (layers \"F.Cu\") (net {net} \"{}\")))\n",
                ["A", "B"][parity]
            ));
        }
    }
    out.push_str(")\n");
    out
}

fn imports(atlas: &mut Atlas) {
    atlas.operation(
        "import",
        "PcbRegion::from_gerber",
        "features",
        "Import one RS-274X Gerber layer into a native region: flashes and stroked traces, aggregated into exact topology.",
    );
    for kind in [
        "square flashes",
        "round flashes",
        "traces",
        "overlapping square flashes",
    ] {
        atlas.sweep(
            kind,
            &powers_of_two(0, 10),
            move |n, _| gerber(kind, n as usize),
            |text| region(PcbRegion::from_gerber(text.as_bytes(), meta("F.Cu"))),
        );
    }

    atlas.operation(
        "import",
        "parse_excellon",
        "holes",
        "Parse an Excellon drill file with two tools into exact drill features.",
    );
    atlas.sweep(
        "metric, trailing zeros",
        &powers_of_two(0, 16),
        |n, _| excellon(n as usize),
        |text| {
            let drills = parse_excellon(text);
            (!drills.is_empty()).then_some(drills.len())
        },
    );

    atlas.operation(
        "import",
        "load_kicad_pcb",
        "board items",
        "Load a KiCad board file: S-expression parsing, then exact copper regions for every pad or track.",
    );
    let files: RefCell<HashMap<(String, u64), PathBuf>> = RefCell::new(HashMap::new());
    for kind in ["smd pads", "segments"] {
        atlas.sweep(
            kind,
            &powers_of_two(0, 12),
            |n, _| {
                files
                    .borrow_mut()
                    .entry((kind.to_owned(), n as u64))
                    .or_insert_with(|| {
                        let path = std::env::temp_dir().join(format!(
                            "hyperdrc-perf-atlas-{}-{}-{n}.kicad_pcb",
                            std::process::id(),
                            kind.replace(' ', "-")
                        ));
                        std::fs::write(&path, kicad(kind, n as usize)).expect("write board");
                        path
                    })
                    .clone()
            },
            |path| load_kicad_pcb(path).map(|board| board.copper.len()),
        );
    }
    for path in files.into_inner().into_values() {
        let _ = std::fs::remove_file(path);
    }
}

fn main() {
    let mut atlas = Atlas::from_args("hyperdrc");
    spacing(&mut atlas);
    width(&mut atlas);
    enclosure(&mut atlas);
    geometry(&mut atlas);
    imports(&mut atlas);
    atlas.finish();
}
