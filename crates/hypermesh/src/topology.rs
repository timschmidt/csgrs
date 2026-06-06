use super::graph::MeshSide;
use super::mesh::{ExactMesh, Triangle};

pub(crate) fn mesh_for_side<'a>(
    side: MeshSide,
    left: &'a ExactMesh,
    right: &'a ExactMesh,
) -> &'a ExactMesh {
    match side {
        MeshSide::Left => left,
        MeshSide::Right => right,
    }
}

pub(crate) fn triangle_edges(triangle: [usize; 3]) -> [[usize; 2]; 3] {
    [
        [triangle[0], triangle[1]],
        [triangle[1], triangle[2]],
        [triangle[2], triangle[0]],
    ]
}

pub(crate) fn triangle_edges_tuple(triangle: [usize; 3]) -> [(usize, usize); 3] {
    [
        canonical_edge_tuple(triangle[0], triangle[1]),
        canonical_edge_tuple(triangle[1], triangle[2]),
        canonical_edge_tuple(triangle[2], triangle[0]),
    ]
}

pub(crate) fn triangle_tuple_edges(triangle: Triangle) -> [(usize, usize); 3] {
    triangle_edges_tuple(triangle.0)
}

pub(crate) fn sorted_edge(edge: [usize; 2]) -> [usize; 2] {
    if edge[0] < edge[1] {
        edge
    } else {
        [edge[1], edge[0]]
    }
}

fn canonical_edge_tuple(left: usize, right: usize) -> (usize, usize) {
    if left < right {
        (left, right)
    } else {
        (right, left)
    }
}
