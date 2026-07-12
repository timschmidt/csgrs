//! The [convex hull](https://en.wikipedia.org/wiki/Convex_hull) of a shape is the smallest convex set that contains it.
//! It may be visualized as the shape enclosed by a rubber band stretched around the subset.
//!
//! This is the set:\
//! ![Pre-ConvexHull demo image][Pre-ConvexHull demo image]
//!
//! And this is the convex hull of that set:\
//! ![ConvexHull demo image][ConvexHull demo image]
#![cfg_attr(doc, doc = doc_image_embed::embed_image!("Pre-ConvexHull demo image", "docs/convex_hull_before_nobackground.png"))]
#![cfg_attr(doc, doc = doc_image_embed::embed_image!("ConvexHull demo image", "docs/convex_hull_nobackground.png"))]

use crate::mesh::Mesh;
use crate::mesh::Polygon;
use crate::vertex::Vertex;
use hashbrown::HashSet;
use hyperlattice::{Point3, Real, Vector3};
use std::fmt::Debug;

fn hyper_triangle_unit_normal(p0: &Point3, p1: &Point3, p2: &Point3) -> Option<Vector3> {
    let normal = (p1 - p0).cross(&(p2 - p0));
    normal.normalize_checked().ok()
}

fn hpoint(point: &Point3) -> hyperlimit::Point3 {
    hyperlimit::Point3::new(point.x.clone(), point.y.clone(), point.z.clone())
}

fn points_equal(left: &Point3, right: &Point3) -> Option<bool> {
    hyperlimit::point3_equal(&hpoint(left), &hpoint(right)).value()
}

fn orientation(points: &[Point3], face: [usize; 3], point: usize) -> Option<hyperlimit::Sign> {
    hyperlimit::orient3d(
        &hpoint(&points[face[0]]),
        &hpoint(&points[face[1]]),
        &hpoint(&points[face[2]]),
        &hpoint(&points[point]),
    )
    .value()
}

fn orientation_point(
    points: &[Point3],
    face: [usize; 3],
    point: &Point3,
) -> Option<hyperlimit::Sign> {
    hyperlimit::orient3d(
        &hpoint(&points[face[0]]),
        &hpoint(&points[face[1]]),
        &hpoint(&points[face[2]]),
        &hpoint(point),
    )
    .value()
}

fn exact_hull_faces(input: &[Point3]) -> Option<(Vec<Point3>, Vec<[usize; 3]>)> {
    let mut points = Vec::new();
    for point in input {
        let duplicate = points
            .iter()
            .map(|existing| points_equal(existing, point))
            .collect::<Option<Vec<_>>>()?
            .into_iter()
            .any(|equal| equal);
        if !duplicate {
            points.push(point.clone());
        }
    }
    if points.len() < 4 {
        return None;
    }

    let p0 = 0;
    let p1 = 1;
    let p2 = (1..points.len()).find(|&index| {
        index != p1
            && hyper_triangle_unit_normal(&points[p0], &points[p1], &points[index]).is_some()
    })?;
    let base = [p0, p1, p2];
    let p3 = (1..points.len()).find(|&index| {
        ![p1, p2].contains(&index)
            && matches!(
                orientation(&points, base, index),
                Some(hyperlimit::Sign::Positive | hyperlimit::Sign::Negative)
            )
    })?;

    let seed = [p0, p1, p2, p3];
    let mut faces = vec![[p0, p1, p2], [p0, p3, p1], [p0, p2, p3], [p1, p3, p2]];
    for (face, opposite) in faces.iter_mut().zip([p3, p2, p1, p0]) {
        if orientation(&points, *face, opposite)? == hyperlimit::Sign::Positive {
            face.swap(1, 2);
        }
    }

    let four = Real::from(4_u8);
    let interior_x = (points[p0].x.clone()
        + points[p1].x.clone()
        + points[p2].x.clone()
        + points[p3].x.clone())
        / four.clone();
    let interior_y = (points[p0].y.clone()
        + points[p1].y.clone()
        + points[p2].y.clone()
        + points[p3].y.clone())
        / four.clone();
    let interior_z = (points[p0].z.clone()
        + points[p1].z.clone()
        + points[p2].z.clone()
        + points[p3].z.clone())
        / four;
    let interior = Point3::new(interior_x.ok()?, interior_y.ok()?, interior_z.ok()?);

    for point in 0..points.len() {
        if seed.contains(&point) {
            continue;
        }
        let mut visible = Vec::new();
        for (index, face) in faces.iter().enumerate() {
            if orientation(&points, *face, point)? == hyperlimit::Sign::Positive {
                visible.push(index);
            }
        }
        if visible.is_empty() {
            continue;
        }

        let mut horizon = HashSet::new();
        for &index in &visible {
            let [a, b, c] = faces[index];
            for edge in [(a, b), (b, c), (c, a)] {
                if !horizon.remove(&(edge.1, edge.0)) {
                    horizon.insert(edge);
                }
            }
        }
        let visible = visible.into_iter().collect::<HashSet<_>>();
        faces = faces
            .into_iter()
            .enumerate()
            .filter_map(|(index, face)| (!visible.contains(&index)).then_some(face))
            .collect();

        for (a, b) in horizon {
            let mut face = [a, b, point];
            if hyper_triangle_unit_normal(&points[a], &points[b], &points[point]).is_none() {
                continue;
            }
            if orientation_point(&points, face, &interior)? == hyperlimit::Sign::Positive {
                face.swap(1, 2);
            }
            faces.push(face);
        }
    }

    Some((points, faces))
}

fn exact_convex_hull<M: Clone + Debug + Send + Sync>(
    points: &[Point3],
    metadata: M,
) -> Mesh<M> {
    let Some((points, faces)) = exact_hull_faces(points) else {
        return Mesh::empty();
    };
    let polygons = faces
        .into_iter()
        .filter_map(|[a, b, c]| {
            let normal = hyper_triangle_unit_normal(&points[a], &points[b], &points[c])?;
            Some(Polygon::new(
                vec![
                    Vertex::new(points[a].clone(), normal.clone()),
                    Vertex::new(points[b].clone(), normal.clone()),
                    Vertex::new(points[c].clone(), normal),
                ],
                metadata.clone(),
            ))
        })
        .collect::<Vec<_>>();
    Mesh::from_polygons(polygons)
}

impl<M: Clone + Debug + Send + Sync> Mesh<M> {
    /// Compute the convex hull of all vertices in this Mesh.
    ///
    /// Hull visibility and horizon updates use exact `orient3d` predicates,
    /// and emitted faces retain their original hyperreal points.
    pub fn convex_hull(&self, metadata: M) -> Mesh<M> {
        // Gather all (x, y, z) coordinates from the polygons
        let points: Vec<Point3> = self
            .polygons
            .iter()
            .flat_map(|poly| poly.vertices.iter().map(|v| v.position.clone()))
            .collect();

        exact_convex_hull(&points, metadata)
    }

    /// Compute a convex hull at a finite application or file-format boundary.
    ///
    /// Coordinates are sampled once to `f64` for `chull`, then promoted back
    /// to exact binary rationals. Internal geometry algorithms should continue
    /// to use [`Mesh::convex_hull`].
    #[cfg(feature = "chull-io")]
    pub fn convex_hull_finite_output(&self, metadata: M) -> Mesh<M> {
        use chull::ConvexHullWrapper;

        let points = self
            .polygons
            .iter()
            .flat_map(|polygon| polygon.vertices.iter())
            .filter_map(|vertex| {
                vertex
                    .position
                    .to_f64_array_lossy()
                    .map(|point| point.to_vec())
            })
            .collect::<Vec<_>>();
        let Ok(hull) = ConvexHullWrapper::try_new(&points, None) else {
            return Mesh::empty();
        };
        let (vertices, indices) = hull.vertices_indices();
        let polygons = indices
            .chunks_exact(3)
            .filter_map(|triangle| {
                let p0 =
                    Point3::try_from_f64_array(vertices[triangle[0]].clone().try_into().ok()?)
                        .ok()?;
                let p1 =
                    Point3::try_from_f64_array(vertices[triangle[1]].clone().try_into().ok()?)
                        .ok()?;
                let p2 =
                    Point3::try_from_f64_array(vertices[triangle[2]].clone().try_into().ok()?)
                        .ok()?;
                let normal = hyper_triangle_unit_normal(&p0, &p1, &p2)?;
                Some(Polygon::new(
                    vec![
                        Vertex::new(p0, normal.clone()),
                        Vertex::new(p1, normal.clone()),
                        Vertex::new(p2, normal),
                    ],
                    metadata.clone(),
                ))
            })
            .collect::<Vec<_>>();
        Mesh::from_polygons(polygons)
    }

    /// Compute the Minkowski sum: self ⊕ other
    ///
    /// **Mathematical Foundation**: For convex sets A and B, A ⊕ B = {a + b | a ∈ A, b ∈ B}.
    /// By the Minkowski sum theorem, the convex hull of all pairwise vertex sums equals
    /// the Minkowski sum of the convex hulls of A and B.
    ///
    /// **Algorithm**: O(|A| × |B|) vertex combinations followed by O(n log n) convex hull computation.
    pub fn minkowski_sum(&self, other: &Mesh<M>, metadata: M) -> Mesh<M> {
        // Collect all vertices (x, y, z) from self
        let verts_a: Vec<Point3> = self
            .polygons
            .iter()
            .flat_map(|poly| poly.vertices.iter().map(|v| v.position.clone()))
            .collect();

        // Collect all vertices from other
        let verts_b: Vec<Point3> = other
            .polygons
            .iter()
            .flat_map(|poly| poly.vertices.iter().map(|v| v.position.clone()))
            .collect();

        if verts_a.is_empty() || verts_b.is_empty() {
            return Mesh::empty();
        }

        // For Minkowski, add every point in A to every point in B
        let sum_points: Vec<_> = verts_a
            .iter()
            .flat_map(|a| verts_b.iter().map(move |b| a.clone() + b.to_vector()))
            .collect();

        // Early return if no points generated
        if sum_points.is_empty() {
            return Mesh::empty();
        }

        exact_convex_hull(&sum_points, metadata)
    }
}

#[cfg(test)]
mod tests {
    use super::exact_hull_faces;
    use hyperlattice::{Point3, Real};

    #[test]
    fn exact_hull_retains_unit_offsets_beyond_f64_resolution() {
        let base = Real::from(1_i64 << 60);
        let point = |x: Real, y: u8, z: u8| Point3::new(x, Real::from(y), Real::from(z));
        let points = [
            point(base.clone(), 0, 0),
            point(base.clone() + Real::one(), 0, 0),
            point(base.clone(), 1, 0),
            point(base.clone(), 0, 1),
            point(base.clone(), 0, 0),
        ];

        let (vertices, faces) = exact_hull_faces(&points).unwrap();

        assert_eq!(vertices.len(), 4);
        assert_eq!(faces.len(), 4);
        assert!(
            vertices
                .iter()
                .any(|point| point.x == base.clone() + Real::one())
        );
    }
}
