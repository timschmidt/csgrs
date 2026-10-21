//! Strict Booleans and convex hulls of sampled spheres (properties.md
//! target 15).
//!
//! Sampled sphere coordinates are sines and cosines of rational multiples of
//! pi. Coplanar quads, their collinear plane crossings and coincidental
//! coplanarities are exact zeros in a cyclotomic field. These tests run the
//! README fixtures and an odd sample count under the strict policy.
//!
//! Set semantics are checked against an oracle that does not use the Boolean
//! kernel. Cube membership is exact. The sampled sphere is a convex polyhedron
//! inscribed in its ball, so a point farther than the radius is outside. For
//! the tested sample counts each face's vertices lie within 30 degrees of each
//! other on the sphere, so every face plane is at least
//! `cos(30 deg) * radius > 0.8 * radius` from the center, and a point closer
//! than `0.8 * radius` is inside. Oracle points avoid the band between those
//! radii and the cube faces. Volumes are compared only as a binary64
//! cross-check.

use csgrs::{
    GeometryCertainty, GeometryContext,
    solid::{self, SolidExt},
};
use hyperlattice::{Point3, Real};
use hypermesh::{MeshCertainty, MeshContext, TriangleMesh};
use std::time::Instant;

fn r(value: f64) -> Real {
    Real::try_from(value).unwrap()
}

#[derive(Clone, Copy)]
struct Ball {
    center: [f64; 3],
    radius: f64,
}

impl Ball {
    // Some(true) inside the sampled polyhedron, Some(false) outside, None
    // where its boundary may lie.
    fn contains(self, point: [f64; 3]) -> Option<bool> {
        let distance = (0..3)
            .map(|axis| (point[axis] - self.center[axis]).powi(2))
            .sum::<f64>()
            .sqrt();
        if distance < 0.8 * self.radius - 1e-9 {
            Some(true)
        } else if distance > self.radius + 1e-9 {
            Some(false)
        } else {
            None
        }
    }
}

const README_BALL: Ball = Ball {
    center: [0.55, 0.25, 0.2],
    radius: 1.35,
};

// Two points in each region of the README cube and sphere: both, cube only,
// sphere only, and neither.
const README_POINTS: [[f64; 3]; 8] = [
    [0.5, 0.5, 0.5],
    [1.0, 0.3, 0.6],
    [1.5, 1.5, 1.5],
    [1.8, 0.3, 1.8],
    [0.5, 0.25, -0.4],
    [-0.3, 0.5, 0.3],
    [2.5, 2.5, 2.5],
    [-1.0, -1.0, 1.5],
];

fn sampled_sphere(ball: Ball, segments: usize, stacks: usize) -> TriangleMesh {
    let [x, y, z] = ball.center;
    solid::sphere(r(ball.radius), segments, stacks)
        .unwrap()
        .translated(r(x), r(y), r(z))
        .unwrap()
}

// Exact membership in the open cube (0, 2)^3, None on its boundary.
fn cube_contains(point: [f64; 3]) -> Option<bool> {
    if point.iter().any(|&c| c == 0.0 || c == 2.0) {
        return None;
    }
    Some(point.iter().all(|&c| c > 0.0 && c < 2.0))
}

fn point(coordinates: [f64; 3]) -> Point3 {
    Point3::new(r(coordinates[0]), r(coordinates[1]), r(coordinates[2]))
}

// Signed volume from binary64 vertex projections: a cross-check only.
fn approximate_volume(mesh: &TriangleMesh) -> f64 {
    let positions = mesh
        .positions
        .iter()
        .map(|p| [&p.x, &p.y, &p.z].map(|coordinate| coordinate.to_f64_lossy().unwrap()))
        .collect::<Vec<_>>();
    mesh.triangles
        .iter()
        .map(|triangle| {
            let [a, b, c] = triangle.indices().map(|index| positions[index]);
            (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
                + a[2] * (b[0] * c[1] - b[1] * c[0]))
                / 6.0
        })
        .sum()
}

// A certified result whose closed piecewise-constant winding validates with
// certified predicates.
fn certified(outcome: csgrs::GeometryOutcome<TriangleMesh>) -> TriangleMesh {
    assert_eq!(outcome.certainty, GeometryCertainty::Certified);
    let mesh = outcome.value;
    assert!(!mesh.triangles.is_empty());
    let validation = mesh
        .validate_closed_pwn(&MeshContext::new(hyperlimit::PredicatePolicy::STRICT))
        .unwrap();
    assert_eq!(validation.certainty, MeshCertainty::Certified);
    mesh
}

fn assert_membership(
    mesh: &TriangleMesh,
    points: &[[f64; 3]],
    expected: impl Fn([f64; 3]) -> Option<bool>,
    label: &str,
) {
    for &sample in points {
        let inside = expected(sample).expect("oracle points avoid boundaries");
        assert_eq!(
            solid::contains_point(mesh, &point(sample)).unwrap(),
            inside,
            "{label} at {sample:?}"
        );
    }
}

fn readme_booleans(segments: usize, stacks: usize) {
    let ball = README_BALL;
    let cube = solid::cube(r(2.0)).unwrap();
    let sphere = sampled_sphere(ball, segments, stacks);
    let strict = GeometryContext::STRICT;
    let start = Instant::now();
    let union = certified(cube.try_union_with_context(&sphere, &strict).unwrap());
    let difference = certified(cube.try_difference_with_context(&sphere, &strict).unwrap());
    let intersection =
        certified(cube.try_intersection_with_context(&sphere, &strict).unwrap());
    let xor = certified(cube.try_xor_with_context(&sphere, &strict).unwrap());
    eprintln!(
        "{segments}x{stacks}: four strict Booleans in {:?}",
        start.elapsed()
    );

    let both = |p: [f64; 3], op: fn(bool, bool) -> bool| {
        Some(op(cube_contains(p)?, ball.contains(p)?))
    };
    let points = &README_POINTS;
    assert_membership(&union, points, |p| both(p, |a, b| a || b), "union");
    assert_membership(&difference, points, |p| both(p, |a, b| a && !b), "difference");
    assert_membership(
        &intersection,
        points,
        |p| both(p, |a, b| a && b),
        "intersection",
    );
    assert_membership(&xor, points, |p| both(p, |a, b| a != b), "xor");

    let [a, b] = [approximate_volume(&cube), approximate_volume(&sphere)];
    let [u, d, i, x] = [&union, &difference, &intersection, &xor].map(approximate_volume);
    assert!((a - 8.0).abs() < 1e-12);
    assert!(i > 0.0 && i < b.min(a));
    assert!((u + i - a - b).abs() < 1e-9, "inclusion-exclusion");
    assert!((d - (a - i)).abs() < 1e-9, "difference volume");
    assert!((x - (u - i)).abs() < 1e-9, "symmetric difference volume");
}

#[test]
fn readme_sphere_booleans_complete_under_strict_policy() {
    readme_booleans(32, 16);
}

#[test]
fn odd_sample_count_booleans_complete_under_strict_policy() {
    readme_booleans(17, 9);
}

fn assert_sphere_hull(segments: usize, stacks: usize) {
    let ball = Ball {
        center: [-0.5, 0.0, 0.0],
        radius: 1.1,
    };
    let sphere = sampled_sphere(ball, segments, stacks);
    let start = Instant::now();
    let hull =
        certified(solid::convex_hull_with_context(&sphere, &GeometryContext::STRICT).unwrap());
    eprintln!(
        "{segments}x{stacks}: strict sphere hull in {:?}",
        start.elapsed()
    );
    // Every sample lies on the sphere, so every vertex is extreme. A
    // triangulated convex polyhedron with V vertices has 2V - 4 faces.
    let vertices = segments * (stacks - 1) + 2;
    assert_eq!(hull.positions.len(), vertices);
    assert_eq!(hull.triangles.len(), 2 * vertices - 4);
    // The sampled sphere is convex, so its hull is the same set.
    let points = [
        [-0.5, 0.0, 0.0],
        [-0.1, 0.5, -0.4],
        [0.7, 0.0, 0.0],
        [-0.5, -1.2, 0.3],
    ];
    assert_membership(&hull, &points, |p| ball.contains(p), "hull");
    assert!((approximate_volume(&hull) - approximate_volume(&sphere)).abs() < 1e-9);
}

#[test]
fn odd_sample_count_sphere_hull_completes_under_strict_policy() {
    assert_sphere_hull(17, 9);
}

#[test]
#[ignore = "expensive (about 4 minutes in debug, 17 s in release): \
            `cargo test --release --test sampled_sphere_strict -- --ignored`"]
fn readme_sample_count_sphere_hull_completes_under_strict_policy() {
    assert_sphere_hull(32, 16);
}

#[test]
fn readme_hull_of_rotated_cube_and_sphere_union_completes() {
    // The convex_hull example. Its 32 x 16 sphere has four coplanar vertices
    // that are not a sampled quad: cot(pi/16) = tan(7 pi/16).
    let cube = solid::rotate(
        &solid::cube(r(2.0))
            .unwrap()
            .translated(r(0.8), r(0.0), r(0.0))
            .unwrap(),
        r(0.0),
        r(35.0),
        r(0.0),
    )
    .unwrap();
    let sphere = sampled_sphere(
        Ball {
            center: [-0.5, 0.0, 0.0],
            radius: 1.1,
        },
        32,
        16,
    );
    let strict = GeometryContext::STRICT;
    let start = Instant::now();
    let union = certified(cube.try_union_with_context(&sphere, &strict).unwrap());
    let hull = certified(solid::convex_hull_with_context(&union, &strict).unwrap());
    eprintln!("README hull pipeline in {:?}", start.elapsed());
    // Points inside the union are inside its hull.
    for sample in [[-0.5, 0.0, 0.0], [1.5, 1.0, -0.5], [0.4, 1.0, 0.0]] {
        let inside = solid::contains_point(&union, &point(sample)).unwrap();
        assert!(!inside || solid::contains_point(&hull, &point(sample)).unwrap());
    }
    assert!(approximate_volume(&hull) > approximate_volume(&union));
    // Every union vertex stays on or inside the hull: compare against the
    // binary64 projection of each hull face plane.
    let hull_points = hull
        .positions
        .iter()
        .map(|p| [&p.x, &p.y, &p.z].map(|c| c.to_f64_lossy().unwrap()))
        .collect::<Vec<_>>();
    for triangle in hull.triangles.iter() {
        let [a, b, c] = triangle.indices().map(|index| hull_points[index]);
        let (u, v) = (
            [b[0] - a[0], b[1] - a[1], b[2] - a[2]],
            [c[0] - a[0], c[1] - a[1], c[2] - a[2]],
        );
        let normal = [
            u[1] * v[2] - u[2] * v[1],
            u[2] * v[0] - u[0] * v[2],
            u[0] * v[1] - u[1] * v[0],
        ];
        for vertex in union.positions.iter() {
            let p = [&vertex.x, &vertex.y, &vertex.z].map(|c| c.to_f64_lossy().unwrap());
            let side = (0..3).map(|k| normal[k] * (p[k] - a[k])).sum::<f64>();
            assert!(side < 1e-9, "a union vertex lies outside a hull face");
        }
    }
}

#[test]
fn strict_results_compose_through_successive_operations() {
    let cube = solid::cube(r(2.0)).unwrap();
    let first = README_BALL;
    // The second sphere crosses the cavity that the first one carves and
    // leaves the cube through its x = 0 face.
    let second = Ball {
        center: [0.3, 1.2, 0.5],
        radius: 0.7,
    };
    let strict = GeometryContext::STRICT;
    let start = Instant::now();
    let carved = certified(
        cube.try_difference_with_context(&sampled_sphere(first, 32, 16), &strict)
            .unwrap(),
    );
    let carved_time = start.elapsed();
    let filled = certified(
        carved
            .try_union_with_context(&sampled_sphere(second, 32, 16), &strict)
            .unwrap(),
    );
    let filled_time = start.elapsed() - carved_time;
    let hull = certified(solid::convex_hull_with_context(&filled, &strict).unwrap());
    let hull_time = start.elapsed() - carved_time - filled_time;
    eprintln!(
        "successive strict operations: difference {carved_time:?}, union {filled_time:?}, hull {hull_time:?}"
    );

    let expected = |p: [f64; 3]| {
        let carved = cube_contains(p)? && !first.contains(p)?;
        Some(carved || second.contains(p)?)
    };
    let points = [
        [1.5, 1.5, 1.5],
        [0.5, 0.5, 0.5],
        [0.3, 1.2, 0.5],
        [-0.1, 1.2, 0.5],
        [2.5, 2.5, 2.5],
        [-1.0, -1.0, 1.5],
    ];
    assert_membership(&filled, &points, expected, "(cube - sphere) + sphere");
    // The hull of a set containing two opposite cube corners contains the
    // cube's center.
    assert!(solid::contains_point(&hull, &point([1.0, 1.0, 1.0])).unwrap());
    let [c, f, h] = [&carved, &filled, &hull].map(approximate_volume);
    assert!(f > c && h > f);
}
