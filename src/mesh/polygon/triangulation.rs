use super::Polygon;
use crate::vertex::Vertex;
use hyperlattice::{Real, Vector3};
use hyperreal::RealSign;
use std::cmp::Ordering;
use std::num::NonZeroU32;

impl<M: Clone + Send + Sync> Polygon<M> {
    /// Triangulate this polygon using exact projected coordinates.
    pub fn triangulate(&self) -> Vec<[Vertex; 3]> {
        if self.vertices.len() < 3 {
            return Vec::new();
        }
        if self.vertices.len() == 3 {
            return vec![[
                self.vertices[0].clone(),
                self.vertices[1].clone(),
                self.vertices[2].clone(),
            ]];
        }

        let normal = (self.plane.point_b.clone() - self.plane.point_a.clone())
            .cross(&(self.plane.point_c.clone() - self.plane.point_a.clone()));
        let Some(points) = project_to_exact_2d(&self.vertices, &normal) else {
            return Vec::new();
        };
        let reverse_output = winding_is_negative(&points);
        let Ok(indices) = hypertri::earcut(&points, &[]) else {
            return Vec::new();
        };

        indices
            .chunks_exact(3)
            .filter_map(|triangle| {
                let [i0, i1, i2] = [triangle[0], triangle[1], triangle[2]];
                if [i0, i1, i2]
                    .into_iter()
                    .any(|index| index >= self.vertices.len())
                {
                    return None;
                }
                let mut vertices = [
                    self.vertices[i0].clone(),
                    self.vertices[i1].clone(),
                    self.vertices[i2].clone(),
                ];
                if reverse_output {
                    vertices.swap(1, 2);
                }
                Some(vertices)
            })
            .collect()
    }

    /// Uniformly split every triangle into four triangles per level.
    pub fn subdivide_triangles(&self, subdivisions: NonZeroU32) -> Vec<[Vertex; 3]> {
        let mut triangles = self.triangulate();
        for _ in 0..subdivisions.get() {
            let mut next = Vec::with_capacity(triangles.len() * 4);
            for triangle in triangles {
                next.extend(subdivide_triangle(triangle));
            }
            triangles = next;
        }
        triangles
    }

    /// Uniformly subdivide this face into triangular polygons.
    pub fn subdivide_to_polygons(&self, subdivisions: NonZeroU32) -> Vec<Polygon<M>> {
        self.subdivide_triangles(subdivisions)
            .into_iter()
            .map(|triangle| Polygon::new(triangle.to_vec(), self.metadata.clone()))
            .collect()
    }
}

fn project_to_exact_2d(
    vertices: &[Vertex],
    support_normal: &Vector3,
) -> Option<Vec<hypertri::Point2>> {
    let drop_axis = dominant_axis(support_normal)?;
    Some(
        vertices
            .iter()
            .map(|vertex| match drop_axis {
                0 => {
                    hypertri::Point2::new(vertex.position.y.clone(), vertex.position.z.clone())
                },
                1 => {
                    hypertri::Point2::new(vertex.position.z.clone(), vertex.position.x.clone())
                },
                _ => {
                    hypertri::Point2::new(vertex.position.x.clone(), vertex.position.y.clone())
                },
            })
            .collect(),
    )
}

fn dominant_axis(normal: &Vector3) -> Option<usize> {
    let squared = normal.0.each_ref().map(|value| value.clone() * value.clone());
    let mut axis = 0;
    for candidate in 1..3 {
        if matches!(
            hyperlimit::compare_reals(&squared[candidate], &squared[axis]).value(),
            Some(Ordering::Greater)
        ) {
            axis = candidate;
        }
    }
    if matches!(squared[axis].refine_sign_until(128), Some(RealSign::Positive)) {
        Some(axis)
    } else {
        None
    }
}

fn winding_is_negative(points: &[hypertri::Point2]) -> bool {
    let area = points.iter().zip(points.iter().cycle().skip(1)).fold(
        Real::zero(),
        |area, (current, next)| {
            area + current.x.clone() * next.y.clone() - next.x.clone() * current.y.clone()
        },
    );
    matches!(area.refine_sign_until(128), Some(RealSign::Negative))
}

fn subdivide_triangle(triangle: [Vertex; 3]) -> [[Vertex; 3]; 4] {
    let half = (Real::one() / Real::from(2_u8)).expect("two is nonzero");
    let v01 = triangle[0].interpolate(&triangle[1], half.clone());
    let v12 = triangle[1].interpolate(&triangle[2], half.clone());
    let v20 = triangle[2].interpolate(&triangle[0], half);

    [
        [triangle[0].clone(), v01.clone(), v20.clone()],
        [v01.clone(), triangle[1].clone(), v12.clone()],
        [v20.clone(), v12.clone(), triangle[2].clone()],
        [v01, v12, v20],
    ]
}
