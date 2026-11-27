//! Performance atlas for Hypervoxel: exact voxelization, sparse voxel octree
//! (SVO DAG) compaction, surface extraction and grid queries across octree
//! depth, solid shape, scalar representation and occupancy pattern.
//!
//! Every frame covers the unit cube `[0, 1]^3` with `2^depth` cells per axis,
//! so the swept axis is octree depth (resolution doubles per step).
//!
//! Groups and axes:
//! - `voxelize`: exact boxes, half-spaces, convex half-space sets and closed
//!   triangle solids (four classification strategies) and triangle surfaces,
//!   over octree depth; triangle solids also over facet count.
//! - `svo`: sparse grid to SVO DAG compaction and back, and SVO surface
//!   meshing, over depth, for solid, shell, scattered and checkerboard
//!   occupancy (the checkerboard is periodic, so DAG hash-consing collapses
//!   it, while scattered cells share few subtrees).
//! - `surface`: exposed faces, greedy face patches and exact surface triangle
//!   meshes from sparse grids.
//! - `query`: flood-fill components, Manhattan distance fields, axis rays and
//!   3D segment traces, level-of-detail selection, support masks, snapshots,
//!   chunk paging and grid diffs.
//!
//! Run from `crates/hypervoxel` (a separate workspace):
//! `cargo run --release --all-features --example perf_atlas -- --out atlas.json`.
//! With `--resources`, voxelizations report stored cells (`nodes`) and
//! octree depth (`depth`); SVO builds report DAG nodes and depth. With
//! `--counters` and `dispatch-trace`, Hyperreal dispatch, predicate and
//! approximation events are recorded per call.
use hyperreal::{Rational, Real};
use hypervoxel::{
    AddressRay, ChunkPagedSparseGrid, ChunkShape, DeterministicSnapshot, ExactBox,
    ExactConvexHalfSpaceSet, ExactHalfSpace, ExactTriangle3, ExactTriangleSolid,
    ExactTriangleSolidMesh, ExactTriangleSurfaceMesh, GridFrame, LengthUnit, MaterialRegionId,
    QueryRegion, SparseVoxelGrid, SupportDirection, SvoVoxelGrid, VoxelAddress, VoxelCell,
    VoxelSideTables, VoxelizationPolicy, VoxelizationReport, classify_support_mask,
    diff_sparse_grids, extract_exposed_faces, greedy_face_patches, sample_manhattan_distance_field,
    select_lod_cells, sparse_exact_surface_triangle_mesh, svo_exact_surface_triangle_mesh,
    trace_address_ray, trace_address_segment, voxelize_exact_box,
    voxelize_exact_convex_halfspace_set, voxelize_exact_halfspace, voxelize_exact_triangle_solid,
    voxelize_exact_triangle_solid_by_adaptive_axis_sweeps,
    voxelize_exact_triangle_solid_by_axis_sweeps,
    voxelize_exact_triangle_solid_by_local_component_consensus,
    voxelize_exact_triangle_surface_mesh,
};
use perf_atlas::{Atlas, Rng, Shape, Shaped};

/// Heap accounting for `--resources` runs; only this atlas binary installs it.
#[global_allocator]
static ALLOCATOR: perf_atlas::CountingAllocator = perf_atlas::CountingAllocator;

const MATERIAL: MaterialRegionId = MaterialRegionId(1);

fn int(v: i64) -> Real {
    Real::from(v)
}

fn ratio(n: i64, d: i64) -> Real {
    Real::new(Rational::new(n) / Rational::new(d))
}

fn sqrt2() -> Real {
    int(2).sqrt().expect("sqrt(2)")
}

fn depths(lo: u8, hi: u8) -> Vec<f64> {
    (lo..=hi).map(f64::from).collect()
}

/// `[0, 1]^3` with `2^depth` cells per axis.
fn frame(depth: u8) -> GridFrame {
    let pitch = Real::new(Rational::new(1) / Rational::new(1 << depth));
    GridFrame::new(
        [int(0), int(0), int(0)],
        [pitch.clone(), pitch.clone(), pitch],
        depth,
        LengthUnit::Millimeter,
    )
    .expect("frame depth within limits")
}

fn policy() -> VoxelizationPolicy {
    VoxelizationPolicy::conservative_cover()
}

/// Stored cells and octree depth of a voxelization.
fn voxelized<T>(
    result: hypervoxel::HypervoxelResult<T>,
    grid: fn(&T) -> &SparseVoxelGrid,
) -> Shaped<Result<(usize, u8), String>> {
    let value = result
        .map(|r| {
            let g = grid(&r);
            (g.len(), g.frame().depth())
        })
        .map_err(|e| format!("{e:?}"));
    Shaped::new(value, |v| {
        v.as_ref().ok().map(|&(cells, depth)| Shape {
            nodes: cells as u64,
            depth: u64::from(depth),
            bits: 0,
        })
    })
}

fn pair_grid(r: &(SparseVoxelGrid, VoxelizationReport)) -> &SparseVoxelGrid {
    &r.0
}

fn triple_grid<R>(r: &(SparseVoxelGrid, VoxelizationReport, R)) -> &SparseVoxelGrid {
    &r.0
}

const BOX_KINDS: [&str; 4] = [
    "grid-aligned box",
    "rational box",
    "irrational box",
    "thin rational slab",
];

fn exact_box(kind: &str) -> ExactBox {
    let p = |a: Real, b: Real, c: Real| [a, b, c];
    match kind {
        "grid-aligned box" => ExactBox::new(
            p(ratio(1, 4), ratio(1, 4), ratio(1, 4)),
            p(ratio(3, 4), ratio(3, 4), ratio(1, 2)),
        ),
        "rational box" => ExactBox::new(
            p(ratio(1, 3), ratio(1, 5), ratio(1, 7)),
            p(ratio(2, 3), ratio(4, 5), ratio(6, 7)),
        ),
        "irrational box" => {
            let s = sqrt2();
            ExactBox::new(
                p(s.clone() * ratio(1, 8), ratio(1, 5), ratio(1, 7)),
                p(s * ratio(1, 2), ratio(4, 5), ratio(6, 7)),
            )
        }
        // Thinner than one cell at every depth swept, offset off the grid.
        "thin rational slab" => ExactBox::new(
            p(ratio(1, 10), ratio(1, 10), ratio(1, 3)),
            p(ratio(9, 10), ratio(9, 10), ratio(1, 3) + ratio(1, 3000)),
        ),
        other => unreachable!("unknown box kind {other}"),
    }
}

fn halfspace(kind: &str) -> ExactHalfSpace {
    match kind {
        "axis normal" => ExactHalfSpace::new([int(0), int(0), int(1)], ratio(1, 3)),
        "diagonal normal" => ExactHalfSpace::new([int(1), int(1), int(1)], ratio(4, 3)),
        "irrational normal" => ExactHalfSpace::new([int(1), sqrt2(), int(0)], ratio(6, 5)),
        other => unreachable!("unknown half-space kind {other}"),
    }
}

/// Convex solids as half-space sets inside the unit cube.
fn polytope(kind: &str) -> ExactConvexHalfSpaceSet {
    let planes = match kind {
        // |x - 1/2| + |y - 1/2| + |z - 1/2| <= 3/8.
        "octahedron" => (0..8)
            .map(|i| {
                let s = |bit: i64| int(if i & bit == 0 { 1 } else { -1 });
                let (a, b, c) = (s(1), s(2), s(4));
                let offset = ratio(3, 8) + (a.clone() + b.clone() + c.clone()) * ratio(1, 2);
                ExactHalfSpace::new([a, b, c], offset)
            })
            .collect(),
        // A cube rotated about z by the 3-4-5 rotation, side 2/5.
        "rotated cube" => {
            let mut planes = Vec::new();
            for (nx, ny) in [(3, 4), (-4, 3), (-3, -4), (4, -3)] {
                let offset = ratio(nx + ny, 10) + ratio(1, 1);
                planes.push(ExactHalfSpace::new(
                    [ratio(nx, 5), ratio(ny, 5), int(0)],
                    offset * ratio(1, 2),
                ));
            }
            planes.push(ExactHalfSpace::new([int(0), int(0), int(1)], ratio(7, 10)));
            planes.push(ExactHalfSpace::new(
                [int(0), int(0), int(-1)],
                ratio(-3, 10),
            ));
            planes
        }
        // A 64-sided prism with exact rational facet normals.
        "64-plane prism" => {
            let mut planes: Vec<_> = circle(64)
                .into_iter()
                .map(|(cx, cy)| {
                    // Tangent plane at the rational circle point (cx, cy).
                    let (nx, ny) = (cx - ratio(1, 2), cy - ratio(1, 2));
                    let offset = nx.clone() * ratio(1, 2) + ny.clone() * ratio(1, 2) + ratio(9, 64);
                    ExactHalfSpace::new([nx, ny, int(0)], offset)
                })
                .collect();
            planes.push(ExactHalfSpace::new([int(0), int(0), int(1)], ratio(3, 4)));
            planes.push(ExactHalfSpace::new([int(0), int(0), int(-1)], ratio(-1, 4)));
            planes
        }
        other => unreachable!("unknown polytope kind {other}"),
    };
    ExactConvexHalfSpaceSet::new(planes)
}

/// `n` exact rational points on the circle of radius 3/8 around
/// `(1/2, 1/2)`, counter-clockwise, from `t = m / 2^12`.
fn circle(n: usize) -> Vec<(Real, Real)> {
    let d: i64 = 1 << 12;
    let mut out: Vec<(Real, Real)> = Vec::with_capacity(n);
    let mut last = None;
    for i in 0..n {
        let theta = -std::f64::consts::PI + std::f64::consts::TAU * (i as f64 + 0.5) / n as f64;
        let m = (d as f64 * (theta / 2.0).tan()).round() as i64;
        if last == Some(m) {
            continue;
        }
        last = Some(m);
        let den = d * d + m * m;
        out.push((
            ratio(1, 2) + ratio(3 * (d * d - m * m), 8 * den),
            ratio(1, 2) + ratio(3 * (2 * d * m), 8 * den),
        ));
    }
    out
}

fn tri(a: &[Real; 3], b: &[Real; 3], c: &[Real; 3]) -> ExactTriangle3 {
    ExactTriangle3::new([a.clone(), b.clone(), c.clone()], None)
}

/// A closed outward-oriented triangle mesh of the given kind.
fn triangle_mesh(kind: &str) -> ExactTriangleSurfaceMesh {
    let v = |x: Real, y: Real, z: Real| [x, y, z];
    let triangles = match kind {
        "tetrahedron" => {
            let p = [
                v(ratio(1, 8), ratio(1, 8), ratio(1, 8)),
                v(ratio(7, 8), ratio(1, 8), ratio(1, 8)),
                v(ratio(1, 8), ratio(7, 8), ratio(1, 8)),
                v(ratio(1, 8), ratio(1, 8), ratio(7, 8)),
            ];
            [[0, 2, 1], [0, 1, 3], [1, 2, 3], [2, 0, 3]]
                .iter()
                .map(|t| tri(&p[t[0]], &p[t[1]], &p[t[2]]))
                .collect()
        }
        "octahedron" => {
            let (lo, mid, hi) = (ratio(1, 8), ratio(1, 2), ratio(7, 8));
            let p = [
                v(hi.clone(), mid.clone(), mid.clone()),
                v(lo.clone(), mid.clone(), mid.clone()),
                v(mid.clone(), hi.clone(), mid.clone()),
                v(mid.clone(), lo.clone(), mid.clone()),
                v(mid.clone(), mid.clone(), hi),
                v(mid.clone(), mid, lo),
            ];
            [
                [0, 2, 4],
                [2, 1, 4],
                [1, 3, 4],
                [3, 0, 4],
                [2, 0, 5],
                [1, 2, 5],
                [3, 1, 5],
                [0, 3, 5],
            ]
            .iter()
            .map(|t| tri(&p[t[0]], &p[t[1]], &p[t[2]]))
            .collect()
        }
        prism => {
            let sides: usize = prism
                .split('-')
                .next()
                .and_then(|n| n.parse().ok())
                .expect("'<n>-gon prism'");
            return prism_mesh(sides);
        }
    };
    ExactTriangleSurfaceMesh::new(triangles)
}

/// A closed prism over an exact rational `n`-gon: 4n - 4 triangles.
fn prism_mesh(sides: usize) -> ExactTriangleSurfaceMesh {
    let ring = circle(sides.max(3));
    let n = ring.len();
    let bottom: Vec<[Real; 3]> = ring
        .iter()
        .map(|(x, y)| [x.clone(), y.clone(), ratio(1, 4)])
        .collect();
    let top: Vec<[Real; 3]> = ring
        .iter()
        .map(|(x, y)| [x.clone(), y.clone(), ratio(3, 4)])
        .collect();
    let mut triangles = Vec::with_capacity(4 * n);
    for i in 0..n {
        let j = (i + 1) % n;
        triangles.push(tri(&bottom[i], &bottom[j], &top[j]));
        triangles.push(tri(&bottom[i], &top[j], &top[i]));
    }
    for i in 1..n - 1 {
        triangles.push(tri(&top[0], &top[i], &top[i + 1]));
        triangles.push(tri(&bottom[0], &bottom[i + 1], &bottom[i]));
    }
    ExactTriangleSurfaceMesh::new(triangles)
}

fn triangle_solid(kind: &str) -> ExactTriangleSolid {
    ExactTriangleSolid::new(ExactTriangleSolidMesh::new(triangle_mesh(kind), true))
        .expect("closed exact solid")
}

const SOLID_KINDS: [&str; 4] = ["tetrahedron", "octahedron", "16-gon prism", "64-gon prism"];

const OCCUPANCY_KINDS: [&str; 4] = [
    "solid ball",
    "sphere shell",
    "1% scatter",
    "checkerboard slab",
];

/// A sparse grid at `depth` filled with the given occupancy pattern.
fn occupancy(kind: &str, depth: u8, rng: &mut Rng) -> SparseVoxelGrid {
    let mut grid = SparseVoxelGrid::new(frame(depth));
    let n = 1_i64 << depth;
    // Squared distances in units of half cells, from the cube centre.
    let r = (3 * n / 8).max(1);
    let mut put = |x: i64, y: i64, z: i64| {
        let address = VoxelAddress::new(depth, [x as u64, y as u64, z as u64]).expect("in frame");
        grid.set(address, VoxelCell::material(MATERIAL))
            .expect("store cell");
    };
    match kind {
        "solid ball" | "sphere shell" => {
            for x in 0..n {
                for y in 0..n {
                    for z in 0..n {
                        let d = |c: i64| 2 * c + 1 - n;
                        let d2 = d(x) * d(x) + d(y) * d(y) + d(z) * d(z);
                        let (inner, outer) = (4 * (r - 1).max(0).pow(2), 4 * r * r);
                        let inside = d2 <= outer;
                        if inside && (kind == "solid ball" || d2 > inner) {
                            put(x, y, z);
                        }
                    }
                }
            }
        }
        "solid box" => {
            let (lo, hi) = (n / 4, (3 * n / 4).max(n / 4 + 1));
            for x in lo..hi {
                for y in lo..hi {
                    for z in lo..hi {
                        put(x, y, z);
                    }
                }
            }
        }
        "1% scatter" => {
            let count = ((n * n * n) / 100).max(1);
            for _ in 0..count {
                put(
                    rng.range(0, n - 1),
                    rng.range(0, n - 1),
                    rng.range(0, n - 1),
                );
            }
        }
        "checkerboard slab" => {
            for x in 0..n {
                for y in 0..n {
                    for z in 0..n.min(4) {
                        if (x + y + z) % 2 == 0 {
                            put(x, y, z);
                        }
                    }
                }
            }
        }
        other => unreachable!("unknown occupancy kind {other}"),
    }
    grid
}

fn grid_cells(n: usize) -> Shape {
    Shape {
        nodes: n as u64,
        ..Shape::default()
    }
}

#[cfg(feature = "dispatch-trace")]
fn correlation() -> hyperreal::dispatch_trace::TraceCorrelationSummary {
    hyperreal::dispatch_trace::snapshot_trace().correlation_summary()
}

#[cfg(feature = "dispatch-trace")]
fn dispatch_events() -> u64 {
    correlation().dispatch_events
}

#[cfg(feature = "dispatch-trace")]
fn predicate_events() -> u64 {
    correlation().predicate_events
}

#[cfg(feature = "dispatch-trace")]
fn approximation_events() -> u64 {
    correlation().approximation_events
}

fn main() {
    let mut atlas = Atlas::from_args("hypervoxel");
    // Recording dispatch labels allocates, so it runs only when asked for.
    #[cfg(feature = "dispatch-trace")]
    let _recording = atlas.config().counters.then(|| {
        atlas.counter("dispatch events", dispatch_events);
        atlas.counter("predicate events", predicate_events);
        atlas.counter("approximation events", approximation_events);
        hyperreal::dispatch_trace::recording_scope()
    });
    voxelize(&mut atlas);
    // Sparse grids and SVOs keep no caches on their cells, so the read-only
    // queries below reuse one input object instead of rebuilding large grids.
    atlas.reuse_inputs(true);
    svo(&mut atlas);
    surface(&mut atlas);
    query(&mut atlas);
    atlas.finish();
}

fn voxelize(atlas: &mut Atlas) {
    atlas.operation(
        "voxelize",
        "exact box",
        "octree depth",
        "Conservative exact voxelization of an axis-aligned box: interior, boundary and certificates.",
    );
    for kind in BOX_KINDS {
        atlas.sweep(
            kind,
            &depths(1, 8),
            |d, _| (frame(d as u8), exact_box(kind)),
            |(frame, solid)| {
                voxelized(
                    voxelize_exact_box(frame.clone(), solid, MATERIAL, policy()),
                    pair_grid,
                )
            },
        );
    }

    atlas.operation(
        "voxelize",
        "half-space",
        "octree depth",
        "Exact voxelization of the half-space normal . p <= offset clipped to the frame.",
    );
    for kind in ["axis normal", "diagonal normal", "irrational normal"] {
        atlas.sweep(
            kind,
            &depths(1, 7),
            |d, _| (frame(d as u8), halfspace(kind)),
            |(frame, h)| {
                voxelized(
                    voxelize_exact_halfspace(frame.clone(), h, MATERIAL, policy()),
                    pair_grid,
                )
            },
        );
    }

    atlas.operation(
        "voxelize",
        "convex half-space set",
        "octree depth",
        "Exact voxelization of a convex solid given as an intersection of half-spaces.",
    );
    for kind in ["octahedron", "rotated cube", "64-plane prism"] {
        atlas.sweep(
            kind,
            &depths(1, 7),
            |d, _| (frame(d as u8), polytope(kind)),
            |(frame, solid)| {
                voxelized(
                    voxelize_exact_convex_halfspace_set(frame.clone(), solid, MATERIAL, policy()),
                    pair_grid,
                )
            },
        );
    }

    atlas.operation(
        "voxelize",
        "triangle solid",
        "octree depth",
        "Exact voxelization of a closed triangle solid by per-cell classification with ray-parity replay.",
    );
    for kind in SOLID_KINDS {
        atlas.sweep(
            kind,
            &depths(1, 6),
            |d, _| (frame(d as u8), triangle_solid(kind)),
            |(frame, solid)| {
                voxelized(
                    voxelize_exact_triangle_solid(frame.clone(), solid, MATERIAL, policy()),
                    triple_grid,
                )
            },
        );
    }

    atlas.operation(
        "voxelize",
        "triangle solid by strategy",
        "octree depth",
        "One 16-gon prism voxelized by per-cell parity, axis sweeps, adaptive axis sweeps and local component consensus.",
    );
    for strategy in [
        "per-cell parity",
        "axis sweeps",
        "adaptive axis sweeps",
        "local component consensus",
    ] {
        atlas.sweep(
            strategy,
            &depths(1, 7),
            |d, _| (frame(d as u8), triangle_solid("16-gon prism")),
            |(frame, solid)| {
                let f = frame.clone();
                match strategy {
                    "per-cell parity" => voxelized(
                        voxelize_exact_triangle_solid(f, solid, MATERIAL, policy()),
                        triple_grid,
                    ),
                    "axis sweeps" => voxelized(
                        voxelize_exact_triangle_solid_by_axis_sweeps(f, solid, MATERIAL, policy()),
                        triple_grid,
                    ),
                    "adaptive axis sweeps" => voxelized(
                        voxelize_exact_triangle_solid_by_adaptive_axis_sweeps(
                            f,
                            solid,
                            MATERIAL,
                            policy(),
                        ),
                        triple_grid,
                    ),
                    _ => voxelized(
                        voxelize_exact_triangle_solid_by_local_component_consensus(
                            f,
                            solid,
                            MATERIAL,
                            policy(),
                        ),
                        triple_grid,
                    ),
                }
            },
        );
    }

    let sides = [4.0, 8.0, 16.0, 32.0, 64.0, 128.0, 256.0, 512.0];
    atlas.operation(
        "voxelize",
        "triangle solid by facet count",
        "prism side count (4n - 4 triangles)",
        "Exact voxelization of an n-gon prism at depth 3 (8^3 cells), and over side count and depth together.",
    );
    atlas.sweep(
        "depth 3",
        &sides,
        |n, _| (frame(3), triangle_solid(&format!("{n}-gon prism"))),
        |(frame, solid)| {
            voxelized(
                voxelize_exact_triangle_solid(frame.clone(), solid, MATERIAL, policy()),
                triple_grid,
            )
        },
    );
    atlas.grid(
        "per-cell parity",
        ("prism side count", &sides[..6]),
        ("octree depth", &depths(1, 4)),
        |n, d, _| (frame(d as u8), triangle_solid(&format!("{n}-gon prism"))),
        |(frame, solid)| {
            voxelize_exact_triangle_solid(frame.clone(), solid, MATERIAL, policy()).is_ok()
        },
    );

    atlas.operation(
        "voxelize",
        "triangle surface",
        "octree depth",
        "Exact voxelization of the cells touched by a triangle surface (no interior).",
    );
    for kind in SOLID_KINDS {
        atlas.sweep(
            kind,
            &depths(1, 6),
            |d, _| (frame(d as u8), triangle_mesh(kind)),
            |(frame, mesh)| {
                voxelized(
                    voxelize_exact_triangle_surface_mesh(frame.clone(), mesh, MATERIAL, policy()),
                    pair_grid,
                )
            },
        );
    }
}

/// Occupancy patterns whose exposed faces form closed manifold surfaces at
/// most depths, for the exact surface-mesh operations.
const MESHABLE_KINDS: [&str; 4] = ["solid ball", "solid box", "sphere shell", "1% scatter"];

fn occupancy_depths(kind: &str) -> Vec<f64> {
    match kind {
        "solid ball" | "solid box" => depths(1, 7),
        _ => depths(1, 9),
    }
}

fn svo(atlas: &mut Atlas) {
    atlas.operation(
        "svo",
        "sparse to SVO DAG",
        "octree depth",
        "Compact a sparse grid into a hash-consed sparse voxel octree DAG.",
    );
    for kind in OCCUPANCY_KINDS {
        atlas.sweep(
            kind,
            &occupancy_depths(kind),
            |d, rng| occupancy(kind, d as u8, rng),
            |grid| {
                let svo = SvoVoxelGrid::from_sparse_grid(grid).map_err(|e| format!("{e:?}"));
                Shaped::new(svo, |s| {
                    s.as_ref().ok().map(|s| Shape {
                        nodes: s.stats().nodes as u64,
                        depth: u64::from(s.frame().depth()),
                        bits: 0,
                    })
                })
            },
        );
    }

    atlas.operation(
        "svo",
        "SVO DAG to sparse",
        "octree depth",
        "Expand an SVO DAG back to a sparse grid of stored cells.",
    );
    for kind in OCCUPANCY_KINDS {
        atlas.sweep(
            kind,
            &occupancy_depths(kind),
            |d, rng| SvoVoxelGrid::from_sparse_grid(&occupancy(kind, d as u8, rng)).expect("svo"),
            |svo| svo.to_sparse_grid().map(|g| g.len()),
        );
    }

    atlas.operation(
        "svo",
        "SVO surface mesh",
        "octree depth",
        "Exact surface triangle mesh of the exposed faces of an SVO DAG; open or nonmanifold face sets are refused.",
    );
    for kind in MESHABLE_KINDS {
        atlas.sweep(
            kind,
            &occupancy_depths(kind),
            |d, rng| SvoVoxelGrid::from_sparse_grid(&occupancy(kind, d as u8, rng)).expect("svo"),
            |svo| {
                Shaped::new(svo_exact_surface_triangle_mesh(svo), |m| {
                    m.as_ref().ok().map(|m| grid_cells(m.triangles.len()))
                })
            },
        );
    }
}

fn surface(atlas: &mut Atlas) {
    atlas.operation(
        "surface",
        "exposed faces",
        "octree depth",
        "Exact exposed cell faces of a sparse grid (six-neighbour test per stored cell).",
    );
    for kind in OCCUPANCY_KINDS {
        atlas.sweep(
            kind,
            &occupancy_depths(kind),
            |d, rng| occupancy(kind, d as u8, rng),
            |grid| {
                Shaped::new(extract_exposed_faces(grid), |f| {
                    f.as_ref().ok().map(|f| grid_cells(f.len()))
                })
            },
        );
    }

    atlas.operation(
        "surface",
        "greedy face patches",
        "octree depth",
        "Merge exposed faces into maximal coplanar rectangular patches.",
    );
    for kind in OCCUPANCY_KINDS {
        atlas.sweep(
            kind,
            &occupancy_depths(kind),
            |d, rng| extract_exposed_faces(&occupancy(kind, d as u8, rng)).expect("faces"),
            |faces| greedy_face_patches(faces).len(),
        );
    }

    atlas.operation(
        "surface",
        "exact surface triangle mesh",
        "octree depth",
        "Exact welded surface triangle mesh with exact vertex coordinates from a sparse grid; open or nonmanifold face sets are refused.",
    );
    for kind in MESHABLE_KINDS {
        atlas.sweep(
            kind,
            &occupancy_depths(kind),
            |d, rng| occupancy(kind, d as u8, rng),
            |grid| {
                Shaped::new(sparse_exact_surface_triangle_mesh(grid), |m| {
                    m.as_ref().ok().map(|m| grid_cells(m.vertices.len()))
                })
            },
        );
    }
}

fn seed(grid: &SparseVoxelGrid) -> Option<VoxelAddress> {
    grid.iter().next().map(|(address, _)| *address)
}

fn query(atlas: &mut Atlas) {
    atlas.operation(
        "query",
        "connected component",
        "octree depth",
        "Six-connected flood fill of stored cells from one seed.",
    );
    for kind in OCCUPANCY_KINDS {
        atlas.sweep(
            kind,
            &occupancy_depths(kind),
            |d, rng| occupancy(kind, d as u8, rng),
            |grid| {
                let seed = seed(grid)?;
                grid.query_connected_component(seed)
                    .ok()
                    .map(|c| c.addresses.len())
            },
        );
    }

    atlas.operation(
        "query",
        "Manhattan distance field",
        "octree depth",
        "Unsigned Manhattan distance to the nearest stored cell, sampled over the whole frame.",
    );
    for kind in OCCUPANCY_KINDS {
        atlas.sweep(
            kind,
            &depths(1, 7),
            |d, rng| occupancy(kind, d as u8, rng),
            |grid| {
                let depth = grid.frame().depth();
                let last = (1_u64 << depth) - 1;
                sample_manhattan_distance_field(
                    grid,
                    QueryRegion {
                        min: [0; 3],
                        max: [last; 3],
                        depth,
                    },
                )
                .map(|f| f.source_cells)
            },
        );
    }

    atlas.operation(
        "query",
        "axis ray trace",
        "octree depth",
        "Trace an address ray along one axis across the whole frame (2^depth cells).",
    );
    for (kind, axis, direction) in [
        ("+x from the low face", 0, 1),
        ("-z from the high face", 2, -1),
    ] {
        atlas.sweep(
            kind,
            &depths(1, 21),
            |d, _| {
                let depth = d as u8;
                let last = (1_u64 << depth) - 1;
                let start = if direction > 0 {
                    [0, 0, 0]
                } else {
                    [0, 0, last]
                };
                AddressRay {
                    start: VoxelAddress::new(depth, start).expect("address"),
                    axis,
                    direction,
                    max_steps: u64::MAX,
                }
            },
            |ray| trace_address_ray(*ray).map(|t| t.addresses.len()),
        );
    }

    atlas.operation(
        "query",
        "segment trace",
        "octree depth",
        "Trace the cells crossed by an address segment (3D DDA), corner to opposite corner.",
    );
    for (kind, end) in [
        ("main diagonal", [1.0, 1.0, 1.0]),
        ("shallow diagonal", [1.0, 0.25, 0.125]),
    ] {
        atlas.sweep(
            kind,
            &depths(1, 18),
            |d, _| {
                let depth = d as u8;
                let last = ((1_u64 << depth) - 1) as f64;
                let e = end.map(|c: f64| (c * last).round() as u64);
                (
                    VoxelAddress::new(depth, [0, 0, 0]).expect("address"),
                    VoxelAddress::new(depth, e).expect("address"),
                )
            },
            |(a, b)| trace_address_segment(*a, *b).map(|t| t.addresses.len()),
        );
    }

    atlas.operation(
        "query",
        "level-of-detail selection",
        "octree depth",
        "Select coarse cells two levels above the stored depth with conservative aggregates.",
    );
    for kind in OCCUPANCY_KINDS {
        atlas.sweep(
            kind,
            &occupancy_depths(kind)[2..],
            |d, rng| occupancy(kind, d as u8, rng),
            |grid| select_lod_cells(grid, grid.frame().depth() - 2).map(|r| r.selected_cells),
        );
    }

    atlas.operation(
        "query",
        "support mask",
        "octree depth",
        "Additive-manufacturing support check: every target cell against the cell below it.",
    );
    for kind in OCCUPANCY_KINDS {
        atlas.sweep(
            kind,
            &occupancy_depths(kind),
            |d, rng| occupancy(kind, d as u8, rng),
            |grid| {
                let down = SupportDirection::new(2, -1).expect("direction");
                classify_support_mask(grid, grid, down).map(|r| r.checked_cells)
            },
        );
    }

    atlas.operation(
        "query",
        "deterministic snapshot",
        "octree depth",
        "Serialize a solid-ball grid into deterministic text, binary and run-length binary snapshots.",
    );
    for format in ["text", "binary", "run-length binary"] {
        atlas.sweep(
            format,
            &depths(1, 7),
            |d, rng| occupancy("solid ball", d as u8, rng),
            |grid| {
                let tables = VoxelSideTables::default();
                match format {
                    "text" => DeterministicSnapshot::text_v1(grid, &tables),
                    "binary" => DeterministicSnapshot::binary_v1(grid, &tables),
                    _ => DeterministicSnapshot::run_length_binary_v1(grid),
                }
                .bytes
                .len()
            },
        );
    }

    atlas.operation(
        "query",
        "chunk paging",
        "octree depth",
        "Page a sparse grid into 8^3-cell chunks with per-page summaries.",
    );
    for kind in OCCUPANCY_KINDS {
        atlas.sweep(
            kind,
            &occupancy_depths(kind),
            |d, rng| occupancy(kind, d as u8, rng),
            |grid| {
                let shape = ChunkShape::new(3).expect("chunk shape");
                ChunkPagedSparseGrid::from_sparse_grid(grid, shape).map(|g| g.page_count())
            },
        );
    }

    atlas.operation(
        "query",
        "grid diff",
        "octree depth",
        "Exact difference report between a grid and a copy with a few cells changed.",
    );
    for kind in OCCUPANCY_KINDS {
        atlas.sweep(
            kind,
            &occupancy_depths(kind),
            |d, rng| {
                let left = occupancy(kind, d as u8, rng);
                let mut right = left.clone();
                let last = (1_u64 << left.frame().depth()) - 1;
                for i in 0..8 {
                    let a = VoxelAddress::new(
                        left.frame().depth(),
                        [i * 7 % (last + 1), last - i % (last + 1), 0],
                    )
                    .expect("address");
                    right
                        .set(a, VoxelCell::material(MaterialRegionId(2)))
                        .expect("set");
                }
                (left, right)
            },
            |(a, b)| diff_sparse_grids(a, b).compared_addresses,
        );
    }
}
