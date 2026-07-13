//! Exact convex hull and Minkowski sum operations backed by `hypermesh`.

use std::collections::HashMap;
use std::fmt::Debug;

use hyperlattice::{Point3, Real, Vector3};

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
            Some(Polygon::new(
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

impl<M: Clone + Debug + Send + Sync> Mesh<M> {
    /// Computes the exact convex hull of all mesh vertices through Hypermesh's
    /// hierarchical point broad phase and certified hull predicates.
    pub fn convex_hull(&self, metadata: M) -> Mesh<M> {
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
