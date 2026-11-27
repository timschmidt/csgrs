//! Performance atlas for Hypergraphics: the exact-to-`f64` render boundary,
//! camera projection and picking, exact backface orientation, and exact
//! tessellation of polygons, curves, regions and grids for display.
//!
//! Covers vertex export (`to_render_vertices64`, `to_xyz_rgb_f64`), Hypermesh
//! import, point projection, the per-frame projection matrix after many exact
//! camera updates, a screen-resolution pick grid (`unproject_to_z0`),
//! triangle orientation against the eye (whole meshes and near-coplanar
//! eyes), earcut surface meshes, certified curve, path and region line
//! meshes, and grid construction.
//!
//! Axes: vertex, point, triangle, polygon-vertex, segment and hole count;
//! screen resolution (pixels per side); camera updates; flattening tolerance
//! (`2^-k`); distance of the eye to the triangle plane (`2^-k`); grid half
//! steps. Input kinds are coordinate representations (integer, binary64,
//! rational, irrational), polygon and curve shapes, and degenerate views.
//! The OpenGL backend needs a GPU context and is not timed.
//!
//! With `--resources`, results report `nodes` as their element count
//! (vertices, triangles, segments, hits), `depth` as the deepest exact
//! coordinate expression and `bits` as the largest stored rational.
//!
//! Run through `scripts/perf-atlas/run.sh`, or directly:
//! `cargo run --release -p hypergraphics --all-features --example perf_atlas -- --out atlas.json [--quick] [--resources]`.
use std::f64::consts::TAU;

use hypercurve::{
    BezierFlatteningOptions, CubicBezier2, Curve2, CurveGeometry2, CurvePath2, CurveRegion2,
    CurveRegionLoopRole, FillRule, LineSeg2, Point2 as CurvePoint2, QuadraticBezier2,
};
use hypergraphics::camera::{project_point, unproject_to_z0};
use hypergraphics::{
    CertifiedCurveLineMesh, Color3, ExactCamera, ExactMesh, ExactVertex, Point2, Point3,
    PredicatePolicy, Primitive, Rational, Real, ScreenPoint, TriangleOrientation,
    TriangulationContext, Viewport, curve_line_mesh, curve_path_line_mesh, curve_region_line_mesh,
    grid_mesh, polygon_surface_mesh, triangle_mesh,
};
use hypermesh::{Triangle, TriangleMesh};
use perf_atlas::{Atlas, Checked, Outcome, Rng, Shape, Shaped, powers_of_two};

/// Heap accounting for `--resources` runs; only this atlas binary installs it.
#[global_allocator]
static ALLOCATOR: perf_atlas::CountingAllocator = perf_atlas::CountingAllocator;

const STRICT: PredicatePolicy = PredicatePolicy::STRICT;

const COORDINATE_KINDS: [&str; 4] = ["integer", "binary64", "rational", "irrational (+√2)"];

fn int(value: i64) -> Real {
    Real::from(value)
}

fn frac(numerator: i64, denominator: u64) -> Real {
    Real::new(Rational::fraction(numerator, denominator).expect("nonzero denominator"))
}

fn dyadic(value: f64) -> Real {
    Real::try_from(value).expect("finite input")
}

fn sqrt(value: i64) -> Real {
    int(value).sqrt().expect("nonnegative radicand")
}

/// `2^-k` as an exact real.
fn pow2_neg(k: u32) -> Real {
    let mut out = Real::one();
    let mut left = k;
    while left > 0 {
        let step = left.min(62);
        out = (out / int(1 << step)).expect("nonzero power of two");
        left -= step;
    }
    out
}

/// A coordinate near `value` in one representation.
fn coordinate(kind: &str, value: f64, rng: &mut Rng) -> Real {
    match kind {
        "integer" => int(value.round() as i64),
        "binary64" => dyadic(value),
        "rational" => {
            let q = rng.range(1 << 10, 1 << 11);
            frac((value * q as f64).round() as i64, q as u64)
        }
        _ => int(value.round() as i64) + sqrt(2),
    }
}

fn point(kind: &str, rng: &mut Rng) -> Point3 {
    let mut c = || {
        let value = rng.unit() * 200.0 - 100.0;
        coordinate(kind, value, rng)
    };
    let (x, y, z) = (c(), c(), c());
    Point3::new(x, y, z)
}

fn color() -> Color3 {
    Color3::new(0.2, 0.4, 0.8).expect("finite colour")
}

fn vertices(kind: &str, n: usize, rng: &mut Rng) -> ExactMesh {
    ExactMesh::new(
        Primitive::Triangles,
        (0..n)
            .map(|_| ExactVertex::new(point(kind, rng), color()))
            .collect(),
    )
}

fn viewport() -> Viewport {
    Viewport::new(0.0, 0.0, 1920.0, 1080.0).expect("finite viewport")
}

// ------------------------------------------------------------------ shapes

fn reals_shape<'a>(values: impl IntoIterator<Item = &'a Real>) -> Shape {
    values.into_iter().fold(Shape::default(), |shape, value| {
        let size = value.representation_size(1 << 20);
        Shape {
            nodes: shape.nodes + size.nodes as u64,
            depth: shape.depth.max(size.depth as u64),
            bits: shape.bits.max(size.max_rational_bits),
        }
    })
}

fn mesh_shape(mesh: &ExactMesh) -> Shape {
    Shape {
        nodes: mesh.vertex_count() as u64,
        ..reals_shape(
            mesh.vertices()
                .iter()
                .flat_map(|v| [&v.position.x, &v.position.y, &v.position.z]),
        )
    }
}

fn mesh_result_shape(result: &Checked<ExactMesh>) -> Option<Shape> {
    Some(mesh_shape(result.as_ref().ok()?))
}

fn line_mesh_shape(result: &Checked<CertifiedCurveLineMesh>) -> Option<Shape> {
    let mesh = result.as_ref().ok()?;
    Some(Shape {
        nodes: mesh.segment_count() as u64,
        ..mesh_shape(mesh.mesh())
    })
}

fn count_shape(result: &Checked<usize>) -> Option<Shape> {
    Some(Shape {
        nodes: *result.as_ref().ok()? as u64,
        ..Shape::default()
    })
}

/// A lowered camera with the exact state it was lowered from.
struct Frame {
    shown: Checked<()>,
    camera: ExactCamera,
}

impl Outcome for Frame {
    fn succeeded(&self) -> bool {
        self.shown.is_ok()
    }
    fn reason(&self) -> Option<String> {
        self.shown.as_ref().err().cloned()
    }
    fn shape(&self) -> Option<Shape> {
        let c = &self.camera;
        Some(reals_shape([&c.yaw, &c.pitch, &c.zoom]))
    }
}

// ----------------------------------------------------------------- meshes

/// A closed cube surface with `k` x `k` quads per face, as an exact
/// triangle mesh in one coordinate representation.
fn cube_mesh(kind: &str, k: usize) -> ExactMesh {
    let k = k.max(1) as i64;
    let scale = sqrt(2);
    let at = |i: i64| -> Real {
        match kind {
            "rational" => frac(i, k as u64),
            "irrational (+√2)" => int(i) * scale.clone(),
            "binary64" => dyadic(i as f64 * 0.1),
            _ => int(i),
        }
    };
    let faces = [
        (2, 0, 1, 0),
        (2, k, 0, 1),
        (1, 0, 0, 2),
        (1, k, 2, 0),
        (0, 0, 2, 1),
        (0, k, 1, 2),
    ];
    let mut out = Vec::with_capacity((36 * k * k) as usize);
    for (axis, value, u, v) in faces {
        let p = |i: i64, j: i64| {
            let mut c = [0i64; 3];
            c[axis] = value;
            c[u] = i;
            c[v] = j;
            ExactVertex::new(Point3::new(at(c[0]), at(c[1]), at(c[2])), color())
        };
        for i in 0..k {
            for j in 0..k {
                out.extend([p(i, j), p(i + 1, j), p(i + 1, j + 1)]);
                out.extend([p(i, j), p(i + 1, j + 1), p(i, j + 1)]);
            }
        }
    }
    ExactMesh::new(Primitive::Triangles, out)
}

/// `n` triangles in the planes z = 0 or x = 0: an eye on both planes is
/// exactly coplanar with every triangle.
fn coplanar_mesh(n: usize) -> ExactMesh {
    let mut out = Vec::with_capacity(3 * n);
    for i in 0..n as i64 {
        let p =
            |x: i64, y: i64, z: i64| ExactVertex::new(Point3::new(int(x), int(y), int(z)), color());
        if i % 2 == 0 {
            out.extend([p(i, 0, 0), p(i + 1, 0, 0), p(i, 1, 0)]);
        } else {
            out.extend([p(0, i, 0), p(0, i + 1, 0), p(0, i, 1)]);
        }
    }
    ExactMesh::new(Primitive::Triangles, out)
}

fn orientation_input(kind: &str, triangles: usize) -> (ExactMesh, Point3) {
    let k = ((triangles as f64 / 12.0).sqrt().round() as usize).max(1);
    match kind {
        "eye in triangle planes" => (
            coplanar_mesh(triangles),
            Point3::new(Real::zero(), int(-5), Real::zero()),
        ),
        _ => (
            cube_mesh(kind, k),
            Point3::new(frac(1, 2), frac(1, 2), int(3)),
        ),
    }
}

fn orient_all(mesh: &ExactMesh, eye: &Point3) -> Checked<usize> {
    let mut facing = 0;
    for t in 0..mesh.triangle_count() {
        match mesh
            .triangle_orientation_against(t, eye, STRICT)
            .map_err(|e| format!("{e:?}"))?
        {
            TriangleOrientation::Positive(_) => facing += 1,
            TriangleOrientation::Negative(_) | TriangleOrientation::Coplanar(_) => {}
            other => return Err(format!("{other:?}")),
        }
    }
    Ok(facing)
}

// ------------------------------------------------------------ tessellation

/// A counter-clockwise integer polygon of `n` vertices and its hole starts.
fn polygon(kind: &str, n: usize, rng: &mut Rng) -> (Vec<Point2>, Vec<usize>) {
    let n = n.max(4);
    let frame = 1i64 << 20;
    let c = frame / 2;
    let p = |x: i64, y: i64| Point2::new(int(x), int(y));
    match kind {
        "convex" | "star" => {
            let points = (0..n)
                .map(|i| {
                    let theta = TAU * i as f64 / n as f64;
                    let r = if kind == "star" {
                        rng.range(frame / 8, frame / 2) as f64
                    } else {
                        (frame / 2) as f64
                    };
                    p(
                        c + (r * theta.cos()).round() as i64,
                        c + (r * theta.sin()).round() as i64,
                    )
                })
                .collect();
            (points, Vec::new())
        }
        "comb" => {
            let teeth = n - 2;
            let w = (frame / teeth as i64).max(1);
            let mut out: Vec<Point2> = (0..teeth as i64)
                .map(|i| p(i * w, if i % 2 == 0 { 0 } else { frame / 4 }))
                .collect();
            out.push(p((teeth as i64 - 1) * w, frame / 2));
            out.push(p(0, frame / 2));
            (out, Vec::new())
        }
        _ => {
            // A square with n / 4 clockwise square holes on a grid.
            let holes = (n / 4).max(1);
            let g = (holes as f64).sqrt().ceil() as i64;
            let cell = frame / g;
            let mut out = vec![p(0, 0), p(frame, 0), p(frame, frame), p(0, frame)];
            let mut starts = Vec::with_capacity(holes);
            for j in 0..holes as i64 {
                let (x0, y0) = ((j % g) * cell, (j / g) * cell);
                let (a, b) = (cell / 4, cell * 3 / 4);
                starts.push(out.len());
                out.extend([
                    p(x0 + a, y0 + a),
                    p(x0 + a, y0 + b),
                    p(x0 + b, y0 + b),
                    p(x0 + b, y0 + a),
                ]);
            }
            (out, starts)
        }
    }
}

fn curve(kind: &str) -> Curve2 {
    let q = |x: i64, y: i64| CurvePoint2::from_values(x, y);
    Curve2::new(match kind {
        "quadratic" => {
            CurveGeometry2::QuadraticBezier(QuadraticBezier2::new(q(0, 0), q(50, 100), q(100, 0)))
        }
        "cubic (rational controls)" => CurveGeometry2::CubicBezier(CubicBezier2::new(
            q(0, 0),
            CurvePoint2::new(frac(100, 3), frac(200, 7)),
            CurvePoint2::new(frac(200, 3), frac(-100, 9)),
            q(100, 0),
        )),
        "cubic with cusp" => CurveGeometry2::CubicBezier(CubicBezier2::new(
            q(0, 0),
            q(100, 100),
            q(0, 100),
            q(100, 0),
        )),
        _ => CurveGeometry2::CubicBezier(CubicBezier2::new(
            q(0, 0),
            q(0, 100),
            q(100, 100),
            q(100, 0),
        )),
    })
}

fn line(start: CurvePoint2, end: CurvePoint2) -> Curve2 {
    Curve2::new(CurveGeometry2::Line(
        LineSeg2::try_new(start, end).expect("nondegenerate segment"),
    ))
}

/// A connected path of `n` pieces along x.
fn path(kind: &str, n: usize) -> CurvePath2 {
    let q = |x: i64, y: i64| CurvePoint2::from_values(x, y);
    let pieces = (0..n as i64)
        .map(|i| {
            let x = 10 * i;
            let cubic = || {
                Curve2::new(CurveGeometry2::CubicBezier(CubicBezier2::new(
                    q(x, 0),
                    q(x + 3, 6),
                    q(x + 7, -6),
                    q(x + 10, 0),
                )))
            };
            match kind {
                "lines" => line(q(x, 0), q(x + 10, 0)),
                "cubic chain" => cubic(),
                _ if i % 2 == 0 => line(q(x, 0), q(x + 10, 0)),
                _ => cubic(),
            }
        })
        .collect();
    CurvePath2::try_new(pieces).expect("connected path")
}

/// A material outline with one cubic side and `holes` square holes.
fn region(holes: usize) -> CurveRegion2 {
    let g = (holes as f64).sqrt().ceil().max(1.0) as i64;
    let s = 3 * g;
    let q = |x: i64, y: i64| CurvePoint2::from_values(x, y);
    let outer = CurvePath2::try_new(vec![
        line(q(0, 0), q(s, 0)),
        Curve2::new(CurveGeometry2::CubicBezier(CubicBezier2::new(
            q(s, 0),
            q(s + 2, g),
            q(s + 2, 2 * g),
            q(s, s),
        ))),
        line(q(s, s), q(0, s)),
        line(q(0, s), q(0, 0)),
    ])
    .expect("closed outline");
    let mut paths = vec![outer];
    for j in 0..holes as i64 {
        let (x, y) = (3 * (j % g) + 1, 3 * (j / g) + 1);
        paths.push(
            CurvePath2::try_new(vec![
                line(q(x, y), q(x, y + 1)),
                line(q(x, y + 1), q(x + 1, y + 1)),
                line(q(x + 1, y + 1), q(x + 1, y)),
                line(q(x + 1, y), q(x, y)),
            ])
            .expect("closed hole"),
        );
    }
    let mut roles = vec![CurveRegionLoopRole::Material];
    roles.extend(std::iter::repeat_n(CurveRegionLoopRole::Hole, holes));
    let rules = vec![FillRule::NonZero; holes + 1];
    CurveRegion2::try_from_boundary_paths_with_loop_semantics(&paths, &roles, &rules)
        .expect("valid region")
}

fn flattening(k: u32) -> BezierFlatteningOptions {
    BezierFlatteningOptions::try_new(pow2_neg(k), 64).expect("positive tolerance")
}

/// An exact camera after `n` orbit and zoom updates of one kind.
fn camera_after(kind: &str, n: usize) -> ExactCamera {
    let mut camera = ExactCamera::default();
    let (yaw, pitch, zoom) = match kind {
        "dyadic orbit, zoom 2" => (dyadic(0.0625), dyadic(-0.015625), int(2)),
        "rational orbit, zoom 3/2" => (frac(1, 10), frac(-1, 30), frac(3, 2)),
        _ => (
            (Real::pi() / int(180)).expect("nonzero"),
            (Real::pi() / int(-720)).expect("nonzero"),
            sqrt(2),
        ),
    };
    for i in 0..n {
        // Alternate the pitch and zoom directions so the camera stays in its
        // valid domain while its exact state keeps accumulating.
        let (pitch, factor) = if i % 2 == 0 {
            (pitch.clone(), zoom.clone())
        } else {
            (
                -pitch.clone(),
                (Real::one() / zoom.clone()).expect("nonzero"),
            )
        };
        camera.orbit(yaw.clone(), pitch);
        camera
            .zoom_by(factor, STRICT)
            .expect("positive zoom factor");
    }
    camera
}

fn pick_camera(kind: &str) -> ExactCamera {
    let mut camera = ExactCamera::default();
    match kind {
        "top-down" => camera.pitch = (Real::pi() / int(-2)).expect("nonzero") + pow2_neg(20),
        "grazing (pitch -1°)" => camera.pitch = (Real::pi() / int(-180)).expect("nonzero"),
        _ => {}
    }
    camera
}

fn main() {
    let mut atlas = Atlas::from_args("hypergraphics");

    // ------------------------------------------------------------ export

    atlas.operation(
        "export",
        "render vertex export",
        "vertex count",
        "Lower an exact mesh's vertices to f64 render vertices (the explicit lossy display boundary).",
    );
    for kind in COORDINATE_KINDS {
        atlas.sweep(
            kind,
            &powers_of_two(0, 16),
            |n, rng| vertices(kind, n as usize, rng),
            |mesh| mesh.to_render_vertices64().map(|v| v.len()),
        );
    }

    atlas.operation(
        "export",
        "interleaved xyz/rgb export",
        "vertex count",
        "Lower an exact mesh to one interleaved f64 xyz/rgb buffer.",
    );
    for kind in ["rational", "irrational (+√2)"] {
        atlas.sweep(
            kind,
            &powers_of_two(0, 16),
            |n, rng| vertices(kind, n as usize, rng),
            |mesh| mesh.to_xyz_rgb_f64().map(|v| v.len()),
        );
    }

    let triangle_counts: Vec<f64> = (0..=6)
        .map(|e| {
            let k = 1usize << e;
            12.0 * (k * k) as f64
        })
        .collect();
    atlas.operation(
        "export",
        "Hypermesh scene import",
        "triangle count",
        "Convert an indexed exact Hypermesh mesh into an exact scene mesh (three vertices per triangle).",
    );
    for kind in ["integer", "rational"] {
        atlas.sweep(
            kind,
            &triangle_counts,
            |m, _| {
                let k = ((m / 12.0).sqrt().round() as usize).max(1);
                let soup = cube_mesh(kind, k);
                let positions: Vec<Point3> =
                    soup.vertices().iter().map(|v| v.position.clone()).collect();
                let triangles = (0..positions.len() / 3)
                    .map(|t| Triangle::new(3 * t, 3 * t + 1, 3 * t + 2))
                    .collect();
                TriangleMesh::new(positions, triangles)
            },
            |mesh| {
                Shaped::new(
                    triangle_mesh(mesh, color()).map_err(|e| format!("{e:?}")),
                    mesh_result_shape,
                )
            },
        );
    }

    // ------------------------------------------------------------ camera

    atlas.operation(
        "camera",
        "project points",
        "point count",
        "Lower the default orbit camera to an f64 projection and project n exact world points to the screen.",
    );
    for kind in COORDINATE_KINDS {
        atlas.sweep(
            kind,
            &powers_of_two(0, 14),
            |n, rng| {
                (0..n as usize)
                    .map(|_| point(kind, rng))
                    .collect::<Vec<_>>()
            },
            |points| {
                let viewport = viewport();
                let projection = ExactCamera::default()
                    .projection64(viewport, STRICT)
                    .map_err(|e| format!("{e:?}"))?;
                let mut visible = 0usize;
                for p in points {
                    if project_point(&projection, viewport, p)
                        .map_err(|e| format!("{e:?}"))?
                        .is_some()
                    {
                        visible += 1;
                    }
                }
                Ok::<usize, String>(visible)
            },
        );
    }

    atlas.operation(
        "camera",
        "projection after camera updates",
        "camera updates",
        "Lower an exact camera to its f64 projection after n exact orbit and zoom updates: the per-frame cost of an interactive session's accumulated exact state.",
    );
    for kind in [
        "dyadic orbit, zoom 2",
        "rational orbit, zoom 3/2",
        "irrational orbit (π/180), zoom √2",
    ] {
        atlas.sweep(
            kind,
            &powers_of_two(0, 12),
            |n, _| camera_after(kind, n as usize),
            |camera| Frame {
                shown: camera
                    .projection64(viewport(), STRICT)
                    .map(|_| ())
                    .map_err(|e| format!("{e:?}")),
                camera: camera.clone(),
            },
        );
    }

    atlas.operation(
        "camera",
        "pick grid (unproject to z = 0)",
        "resolution (pixels per side)",
        "Unproject every pixel centre of an r x r screen grid onto the ground plane, as a picking or ray-cast pass does.",
    );
    for kind in ["default orbit", "top-down", "grazing (pitch -1°)"] {
        atlas.sweep(
            kind,
            &powers_of_two(1, 11),
            |r, _| {
                let viewport = Viewport::new(0.0, 0.0, r, r).expect("finite viewport");
                let projection = pick_camera(kind)
                    .projection64(viewport, STRICT)
                    .expect("valid camera");
                (projection, viewport, r as usize)
            },
            |(projection, viewport, r)| {
                let pass = || -> Checked<usize> {
                    let mut hits = 0usize;
                    for y in 0..*r {
                        for x in 0..*r {
                            let screen = ScreenPoint::new(x as f64 + 0.5, y as f64 + 0.5)
                                .map_err(|e| format!("{e:?}"))?;
                            if unproject_to_z0(projection, *viewport, screen)
                                .map_err(|e| format!("{e:?}"))?
                                .is_some()
                            {
                                hits += 1;
                            }
                        }
                    }
                    Ok(hits)
                };
                Shaped::new(pass(), count_shape)
            },
        );
    }

    // -------------------------------------------------------- visibility

    atlas.operation(
        "visibility",
        "backface orientation",
        "triangle count",
        "Classify every triangle of a fresh mesh against the eye with the exact orientation predicate (the mesh's orientation cache starts empty).",
    );
    for kind in [
        "integer",
        "rational",
        "irrational (+√2)",
        "eye in triangle planes",
    ] {
        atlas.sweep(
            kind,
            &triangle_counts,
            |m, _| orientation_input(kind, m as usize),
            |(mesh, eye)| Shaped::new(orient_all(mesh, eye), count_shape),
        );
    }

    let ks: Vec<f64> = (0..=256).step_by(16).map(f64::from).collect();
    atlas.operation(
        "visibility",
        "orientation near coplanar eye",
        "eye distance to plane (2^-k)",
        "Orient one large triangle against an eye 2^-k above its plane, in an integer, rational or irrational (√2-offset) frame.",
    );
    for kind in ["integer", "rational", "irrational (+√2)"] {
        atlas.sweep(
            kind,
            &ks,
            |k, _| {
                let offset = match kind {
                    "rational" => frac(1, 3),
                    "irrational (+√2)" => sqrt(2),
                    _ => Real::zero(),
                };
                let p = |x: i64, y: i64| {
                    ExactVertex::new(Point3::new(int(x), int(y), offset.clone()), color())
                };
                let mesh = ExactMesh::new(
                    Primitive::Triangles,
                    vec![p(0, 0), p(1 << 20, 0), p(0, 1 << 20)],
                );
                let eye = Point3::new(int(3), int(5), offset.clone() + pow2_neg(k as u32));
                (mesh, eye)
            },
            |(mesh, eye)| {
                matches!(
                    mesh.triangle_orientation_against(0, eye, STRICT),
                    Ok(TriangleOrientation::Positive(_) | TriangleOrientation::Negative(_))
                )
            },
        );
    }

    // ------------------------------------------------------ tessellation

    let context = TriangulationContext::new(STRICT);
    atlas.operation(
        "tessellation",
        "polygon surface mesh",
        "polygon vertex count",
        "Triangulate an exact integer polygon (earcut via Hypertri) into a surface mesh at z = 0.",
    );
    for kind in ["convex", "star", "comb", "square with holes"] {
        atlas.sweep(
            kind,
            &powers_of_two(2, 12),
            |n, rng| polygon(kind, n as usize, rng),
            |(points, holes)| {
                Shaped::new(
                    polygon_surface_mesh(&context, points, holes, Real::zero(), color())
                        .map(|outcome| outcome.into_value())
                        .map_err(|e| format!("{e:?}")),
                    mesh_result_shape,
                )
            },
        );
    }

    atlas.operation(
        "tessellation",
        "curve line mesh by tolerance",
        "flattening tolerance (2^-k)",
        "Certified flattening of one 100-unit curve into an exact line mesh within 2^-k, with retained chord-error evidence.",
    );
    for kind in [
        "cubic",
        "cubic (rational controls)",
        "quadratic",
        "cubic with cusp",
    ] {
        atlas.sweep(
            kind,
            &(0..=32).step_by(2).map(f64::from).collect::<Vec<_>>(),
            |k, _| (curve(kind), flattening(k as u32)),
            |(c, options)| {
                Shaped::new(
                    curve_line_mesh(c, options, STRICT, Real::zero(), color())
                        .map_err(|e| format!("{e:?}")),
                    line_mesh_shape,
                )
            },
        );
    }

    atlas.operation(
        "tessellation",
        "curve path line mesh",
        "segment count",
        "Certified flattening of a connected path of n pieces within 2^-8 into one line mesh.",
    );
    for kind in ["lines", "cubic chain", "mixed"] {
        atlas.sweep(
            kind,
            &powers_of_two(0, 10),
            |n, _| (path(kind, n as usize), flattening(8)),
            |(p, options)| {
                Shaped::new(
                    curve_path_line_mesh(p, options, STRICT, Real::zero(), color())
                        .map_err(|e| format!("{e:?}")),
                    line_mesh_shape,
                )
            },
        );
    }

    atlas.operation(
        "tessellation",
        "curve region line mesh",
        "hole count",
        "Materialize a region's boundary loops and material/hole roles, then flatten every loop within 2^-6 into one role-coloured line mesh.",
    );
    atlas.sweep(
        "square holes",
        &powers_of_two(0, 8),
        |n, _| region(n as usize),
        |r| {
            Shaped::new(
                curve_region_line_mesh(
                    r,
                    &flattening(6),
                    STRICT,
                    Real::zero(),
                    Color3::GREEN,
                    Color3::RED,
                )
                .map(|m| m.into_mesh())
                .map_err(|e| format!("{e:?}")),
                mesh_result_shape,
            )
        },
    );

    atlas.operation(
        "tessellation",
        "grid mesh and export",
        "grid half steps",
        "Build an exact (2h+1)-line square grid and lower it to an interleaved f64 buffer.",
    );
    for kind in [
        "integer step",
        "rational step (1/3)",
        "irrational step (√2)",
    ] {
        atlas.sweep(
            kind,
            &powers_of_two(0, 12),
            |h, _| {
                let step = match kind {
                    "rational step (1/3)" => frac(1, 3),
                    "irrational step (√2)" => sqrt(2),
                    _ => int(5),
                };
                (h as u32, step)
            },
            |(h, step)| {
                grid_mesh(*h, step.clone(), color())
                    .and_then(|mesh| mesh.to_xyz_rgb_f64())
                    .map(|buffer| buffer.len())
            },
        );
    }

    atlas.finish();
}
