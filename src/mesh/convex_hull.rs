//! Exact convex hull and Minkowski sum operations backed by `hypermesh`.

use std::collections::HashMap;
use std::fmt::Debug;

use hyperlattice::{Point3, Real, Vector3};
use hypermesh::{ExactPointBvh, InputMesh};

use crate::mesh::{Mesh, Polygon};
use crate::vertex::Vertex;

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
struct PositionBucket([Option<u64>; 3]);

impl PositionBucket {
    fn new(point: &Point3) -> Self {
        Self([&point.x, &point.y, &point.z].map(|coordinate| {
            coordinate.to_f64_lossy().map(|value| {
                if value == 0.0 {
                    0.0_f64.to_bits()
                } else {
                    value.to_bits()
                }
            })
        }))
    }
}

fn mesh_from_hull<M: Clone + Debug + Send + Sync>(
    points: &[Point3],
    coplanar_groups: &[Vec<usize>],
    coordinate_ids: Option<&[[u64; 5]]>,
    metadata: M,
) -> Mesh<M> {
    let hull = match coordinate_ids {
        Some(coordinate_ids) => {
            hypermesh::convex_hull_with_retained_facts(points, coplanar_groups, coordinate_ids)
        },
        None => hypermesh::convex_hull_with_coplanar_groups(points, coplanar_groups),
    };
    let hull = match hull {
        Ok(hull) => hull,
        Err(_) => return Mesh::empty(),
    };
    let positions = hull
        .positions
        .into_iter()
        .map(|position| Vertex::new(position, Vector3::z()))
        .collect::<Vec<_>>();
    let polygons = hull
        .triangles
        .into_iter()
        .filter_map(|triangle| {
            let [a, b, c] = triangle.indices();
            let p0 = positions.get(a)?;
            let p1 = positions.get(b)?;
            let p2 = positions.get(c)?;
            let normal = triangle_unit_normal(&p0.position, &p1.position, &p2.position)?;
            Some(Polygon::from_planar_vertices(
                vec![
                    p0.clone().with_normal(normal.clone()),
                    p1.clone().with_normal(normal.clone()),
                    p2.clone().with_normal(normal),
                ],
                metadata.clone(),
            ))
        })
        .collect();
    Mesh::from_polygons(polygons)
}

fn triangle_unit_normal(p0: &Point3, p1: &Point3, p2: &Point3) -> Option<Vector3> {
    (p1 - p0).cross(&(p2 - p0)).normalize_checked().ok()
}

fn unique_mesh_positions<M: Clone + Debug + Send + Sync>(mesh: &Mesh<M>) -> Vec<&Point3> {
    let mut positions = Vec::new();
    let mut bucket_heads = HashMap::<PositionBucket, usize>::new();
    let mut next_in_bucket = Vec::<Option<usize>>::new();
    for vertex in mesh
        .polygons
        .iter()
        .flat_map(|polygon| polygon.vertices.iter())
    {
        let bucket = PositionBucket::new(&vertex.position);
        let mut candidate = bucket_heads.get(&bucket).copied();
        let mut duplicate = false;
        while let Some(candidate_index) = candidate {
            if positions[candidate_index] == &vertex.position {
                duplicate = true;
                break;
            }
            candidate = next_in_bucket[candidate_index];
        }
        if !duplicate {
            next_in_bucket.push(bucket_heads.insert(bucket, positions.len()));
            positions.push(&vertex.position);
        }
    }
    positions
}

fn certified_convex_triangle_surface(mesh: &InputMesh) -> bool {
    let mut edge_directions = HashMap::<(usize, usize), [usize; 2]>::new();
    for triangle in &mesh.triangles {
        let [a, b, c] = triangle.indices();
        for [start, end] in [[a, b], [b, c], [c, a]] {
            let (edge, direction) = if start < end {
                ((start, end), 0)
            } else {
                ((end, start), 1)
            };
            edge_directions.entry(edge).or_default()[direction] += 1;
        }
    }
    if edge_directions.is_empty()
        || edge_directions
            .values()
            .any(|directions| *directions != [1, 1])
    {
        return false;
    }

    let Ok(point_bvh) = ExactPointBvh::build(&mesh.positions) else {
        return false;
    };
    for triangle in &mesh.triangles {
        let [a, b, c] = triangle.indices();
        let mut outside = false;
        if point_bvh
            .query_negative_oriented_plane(
                &mesh.positions,
                &mesh.positions[a],
                &mesh.positions[b],
                &mesh.positions[c],
                |_| outside = true,
            )
            .is_err()
            || outside
        {
            return false;
        }
    }
    true
}

impl<M: Clone + Debug + Send + Sync> Mesh<M> {
    pub(super) fn is_certified_convex_triangle_surface(&self) -> bool {
        self.polygons
            .iter()
            .all(|polygon| polygon.vertices.len() == 3)
            && self
                .to_hypermesh_triangle_mesh()
                .is_ok_and(|input| certified_convex_triangle_surface(&input))
    }

    /// Computes the exact convex hull of all mesh vertices through Hypermesh's
    /// hierarchical point broad phase and certified hull predicates.
    pub fn convex_hull(&self, metadata: M) -> Mesh<M> {
        if self.is_certified_convex_triangle_surface() {
            return Mesh::from_polygons(
                self.polygons
                    .iter()
                    .cloned()
                    .map(|polygon| polygon.map_metadata(|_| metadata.clone()))
                    .collect(),
            );
        }

        let mut points = Vec::new();
        let mut coordinate_ids = Vec::new();
        let mut point_indices = HashMap::new();
        let mut coordinate_aliases = HashMap::<(usize, Option<u64>), Vec<(Real, u64)>>::new();
        let mut coordinate_id_aliases = HashMap::new();
        let mut coplanar_groups = Vec::<Vec<usize>>::new();
        let mut coplanar_group_indices = HashMap::<u64, usize>::new();
        for polygon in &self.polygons {
            let indices = polygon
                .vertices
                .iter()
                .filter(|vertex| vertex.hull_candidate)
                .map(|vertex| {
                    *point_indices.entry(vertex.position_id).or_insert_with(|| {
                        let index = points.len();
                        let coordinate_identity: [u64; 3] = std::array::from_fn(|axis| {
                            if let Some(&id) =
                                coordinate_id_aliases.get(&vertex.coordinate_ids[axis])
                            {
                                return id;
                            }
                            let coordinate = match axis {
                                0 => &vertex.position.x,
                                1 => &vertex.position.y,
                                _ => &vertex.position.z,
                            };
                            let bucket = coordinate.to_f64_lossy().map(f64::to_bits);
                            let entries =
                                coordinate_aliases.entry((axis, bucket)).or_default();
                            let id = entries
                                .iter()
                                .find_map(|(value, id)| (value == coordinate).then_some(*id))
                                .unwrap_or(vertex.coordinate_ids[axis]);
                            if !entries.iter().any(|(_, existing)| *existing == id) {
                                entries.push((coordinate.clone(), id));
                            }
                            coordinate_id_aliases.insert(vertex.coordinate_ids[axis], id);
                            id
                        });
                        points.push(vertex.position.clone());
                        let [surface_id, line_id] = vertex
                            .ruled_line
                            .unwrap_or([vertex.position_id, vertex.position_id]);
                        coordinate_ids.push([
                            coordinate_identity[0],
                            coordinate_identity[1],
                            coordinate_identity[2],
                            surface_id,
                            line_id,
                        ]);
                        index
                    })
                })
                .collect::<Vec<_>>();
            let group_index =
                *coplanar_group_indices
                    .entry(polygon.plane_id)
                    .or_insert_with(|| {
                        coplanar_groups.push(Vec::new());
                        coplanar_groups.len() - 1
                    });
            coplanar_groups[group_index].extend(indices);
        }
        mesh_from_hull(&points, &coplanar_groups, Some(&coordinate_ids), metadata)
    }

    /// Computes `self + other` as the exact hull of all pairwise vertex sums.
    pub fn minkowski_sum(&self, other: &Mesh<M>, metadata: M) -> Mesh<M> {
        let left = unique_mesh_positions(self);
        let right = unique_mesh_positions(other);
        if left.is_empty() || right.is_empty() {
            return Mesh::empty();
        }
        let sums = left
            .iter()
            .flat_map(|left| {
                right
                    .iter()
                    .map(move |right| (*left).clone() + right.to_vector())
            })
            .collect::<Vec<_>>();
        mesh_from_hull(&sums, &[], None, metadata)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use hyperlattice::Real;

    #[test]
    fn convex_surface_certificate_requires_a_closed_consistently_wound_mesh() {
        let points = vec![
            Point3::new(Real::zero(), Real::zero(), Real::zero()),
            Point3::new(Real::from(2), Real::zero(), Real::zero()),
            Point3::new(Real::zero(), Real::from(2), Real::zero()),
            Point3::new(Real::zero(), Real::zero(), Real::from(2)),
        ];
        let hull = hypermesh::convex_hull(&points).unwrap();
        assert!(certified_convex_triangle_surface(&hull));

        let open = InputMesh::new(points, hull.triangles[..3].to_vec());
        assert!(!certified_convex_triangle_surface(&open));
    }

    #[test]
    fn hull_retains_unit_offsets_beyond_f64_resolution() {
        let base = Real::from(1_i64 << 60);
        let points = [
            Point3::new(base.clone(), Real::zero(), Real::zero()),
            Point3::new(base.clone() + Real::one(), Real::zero(), Real::zero()),
            Point3::new(base.clone(), Real::one(), Real::zero()),
            Point3::new(base.clone(), Real::zero(), Real::one()),
        ];
        let coordinate_ids = [
            [0, 1, 2, 3, 4],
            [5, 6, 7, 8, 9],
            [10, 11, 12, 13, 14],
            [15, 16, 17, 18, 19],
        ];
        let mesh = mesh_from_hull(&points, &[], Some(&coordinate_ids), ());

        assert!(
            mesh.polygons
                .iter()
                .flat_map(|polygon| &polygon.vertices)
                .any(|vertex| vertex.position.x == base.clone() + Real::one())
        );
    }
}
