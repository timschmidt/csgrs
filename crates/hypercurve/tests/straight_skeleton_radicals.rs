//! Straight skeletons of polygons whose edge lengths bring in several
//! distinct square roots complete exactly under STRICT. Edge events are
//! solved along the collapsing edge's own direction, and new wavefront
//! vertices are matched to their event node structurally, so no exact zero
//! across unrelated radicals has to be proved.
use hypercurve::{Contour2, StraightSkeleton2, provisional};

fn skeleton(ring: &[[f64; 2]]) -> StraightSkeleton2 {
    let contour = Contour2::from_finite_ring(ring).expect("valid ring");
    let strict = contour.straight_skeleton().expect("no error");
    assert_eq!(strict.blocker(), None, "{ring:?}");
    let approximate = provisional(|| contour.straight_skeleton())
        .into_unverified()
        .expect("no error");
    assert_eq!(approximate.blocker(), None, "{ring:?}");
    strict.skeleton().expect("complete").clone()
}

fn approx(value: &hyperreal::Real) -> f64 {
    value.to_f64_lossy().expect("finite")
}

/// Even-odd point-in-polygon test on the f64 ring.
fn inside(ring: &[[f64; 2]], x: f64, y: f64) -> bool {
    let mut odd = false;
    for i in 0..ring.len() {
        let [ax, ay] = ring[i];
        let [bx, by] = ring[(i + 1) % ring.len()];
        if (ay > y) != (by > y) && x < ax + (y - ay) * (bx - ax) / (by - ay) {
            odd = !odd;
        }
    }
    odd
}

/// A simple n-gon whose events are all distinct has n source vertices and
/// n - 2 event nodes joined by 2n - 3 arcs into a tree, with every event
/// node strictly inside the polygon.
fn assert_generic_skeleton(ring: &[[f64; 2]]) {
    let graph = skeleton(ring);
    let n = ring.len();
    assert_eq!(graph.nodes().len(), 2 * n - 2, "{ring:?}");
    assert_eq!(graph.arcs().len(), 2 * n - 3, "{ring:?}");
    let mut parent: Vec<usize> = (0..graph.nodes().len()).collect();
    fn root(parent: &mut [usize], mut node: usize) -> usize {
        while parent[node] != node {
            node = parent[node];
        }
        node
    }
    for arc in graph.arcs() {
        let (a, b) = (
            root(&mut parent, arc.start_node()),
            root(&mut parent, arc.end_node()),
        );
        assert_ne!(a, b, "cycle in the skeleton of {ring:?}");
        parent[a] = b;
    }
    for node in graph.nodes() {
        if approx(node.time()) == 0.0 {
            continue;
        }
        let (x, y) = (approx(node.point().x()), approx(node.point().y()));
        assert!(inside(ring, x, y), "event node ({x}, {y}) outside {ring:?}");
    }
}

#[test]
fn reported_polygons_complete_exactly() {
    for ring in [
        &[[0.0, 0.0], [1.0, 0.0], [0.0, 2.0]][..],
        &[[0.0, 0.0], [2.0, 1.0], [0.0, 3.0]],
        &[[0.0, 0.0], [10.0, 0.0], [11.0, 7.0], [1.0, 9.0]],
        &[
            [0.0, 0.0],
            [10.0, 0.0],
            [13.0, 6.0],
            [5.0, 11.0],
            [-2.0, 5.0],
        ],
    ] {
        assert_generic_skeleton(ring);
    }
}

#[test]
fn general_convex_footprints_complete_exactly() {
    for ring in [
        // Every edge length is a distinct radical.
        &[
            [0.0, 0.0],
            [7.0, 2.0],
            [9.0, 8.0],
            [3.0, 11.0],
            [-2.0, 6.0],
            [-1.0, 2.0],
        ][..],
        &[
            [0.0, 0.0],
            [12.5, 0.5],
            [14.25, 9.0],
            [6.0, 13.75],
            [-1.5, 8.0],
        ],
        &[
            [100.0, 200.0],
            [131.0, 197.0],
            [142.0, 223.0],
            [118.0, 241.0],
            [97.0, 229.0],
            [93.0, 211.0],
        ],
    ] {
        assert_generic_skeleton(ring);
    }
}

#[test]
fn concave_footprints_with_split_events_complete_as_trees() {
    for ring in [
        &[
            [0.0, 0.0],
            [10.0, 1.0],
            [9.0, 5.0],
            [5.0, 4.0],
            [4.0, 10.0],
            [-1.0, 9.0],
        ][..],
        &[
            [0.0, 0.0],
            [12.0, 1.0],
            [11.0, 9.0],
            [6.0, 5.0],
            [1.0, 10.0],
        ],
        &[
            [0.0, 0.0],
            [11.0, 1.0],
            [10.0, 10.0],
            [7.0, 9.0],
            [7.5, 3.5],
            [3.0, 3.0],
            [2.0, 10.0],
            [-1.0, 9.0],
        ],
        &[
            [0.0, 0.0],
            [6.0, 1.0],
            [7.0, 4.0],
            [9.0, 0.5],
            [14.0, 2.0],
            [13.0, 9.0],
            [7.0, 6.0],
            [1.0, 10.0],
        ],
    ] {
        assert_generic_skeleton(ring);
    }
}
