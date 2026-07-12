use super::Polygon;
use crate::vertex::Vertex;
use hyperlattice::{Real, Vector3};
use hyperreal::RealSign;
use hypertri::kernel::{ExactKernel, Kernel};
use hypertri::types::Sign;
use std::cmp::Ordering;
use std::num::NonZeroU32;

impl<M: Clone + Send + Sync> Polygon<M> {
    /// Triangulate this polygon using exact projected coordinates.
    pub fn triangulate(&self) -> Vec<[Vertex; 3]> {
        self.triangulate_indices()
            .into_iter()
            .map(|[i0, i1, i2]| {
                [
                    self.vertices[i0].clone(),
                    self.vertices[i1].clone(),
                    self.vertices[i2].clone(),
                ]
            })
            .collect()
    }

    /// Triangulate this polygon into indices of its existing vertices.
    pub fn triangulate_indices(&self) -> Vec<[usize; 3]> {
        if self.vertices.len() < 3 {
            return Vec::new();
        }
        if self.vertices.len() == 3 {
            return vec![[0, 1, 2]];
        }

        let normal = (self.plane.point_b.clone() - self.plane.point_a.clone())
            .cross(&(self.plane.point_c.clone() - self.plane.point_a.clone()));
        let Some(points) = project_to_exact_2d(&self.vertices, &normal) else {
            return Vec::new();
        };
        if let Some(indices) = strictly_convex_fan(&points) {
            return indices;
        }
        let reverse_output = winding_is_negative(&points);
        let Ok(indices) = hypertri::earcut(&points, &[]) else {
            return Vec::new();
        };

        triangle_indices(&indices, self.vertices.len(), reverse_output)
    }

    /// Triangulate for a finite renderer or file-format output boundary.
    ///
    /// Exact projected triangulation remains the primary path. If symbolic
    /// coordinates cannot certify a projection ordering, this method samples
    /// the already finite output coordinates, promotes those samples to exact
    /// binary rationals, and reruns the same exact ear-clipping kernel. The
    /// returned triangles retain the original vertices and therefore only the
    /// boundary scheduling decision is sampled.
    pub fn triangulate_finite_output(&self) -> Vec<[Vertex; 3]> {
        self.triangulate_indices_finite_output()
            .into_iter()
            .map(|[i0, i1, i2]| {
                [
                    self.vertices[i0].clone(),
                    self.vertices[i1].clone(),
                    self.vertices[i2].clone(),
                ]
            })
            .collect()
    }

    /// Triangulate into existing-vertex indices for a finite output boundary.
    pub fn triangulate_indices_finite_output(&self) -> Vec<[usize; 3]> {
        let exact = self.triangulate_indices();
        if !exact.is_empty() || self.vertices.len() < 4 {
            return exact;
        }

        let Some(points) = project_to_finite_exact_2d(&self.vertices) else {
            return Vec::new();
        };
        let reverse_output = winding_is_negative(&points);
        let Ok(indices) = hypertri::earcut(&points, &[]) else {
            return Vec::new();
        };
        triangle_indices(&indices, self.vertices.len(), reverse_output)
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

fn strictly_convex_fan(points: &[hypertri::Point2]) -> Option<Vec<[usize; 3]>> {
    let mut winding = None;
    for index in 0..points.len() {
        let sign = ExactKernel::orient2d(
            &points[index],
            &points[(index + 1) % points.len()],
            &points[(index + 2) % points.len()],
        )
        .ok()?;
        if sign == Sign::Zero {
            return None;
        }
        match winding {
            None => winding = Some(sign),
            Some(expected) if expected == sign => {},
            Some(_) => return None,
        }
    }
    Some((1..points.len() - 1).map(|i| [0, i, i + 1]).collect())
}

fn triangle_indices(
    indices: &[usize],
    vertex_count: usize,
    reverse_output: bool,
) -> Vec<[usize; 3]> {
    indices
        .chunks_exact(3)
        .filter_map(|triangle| {
            let [i0, i1, i2] = [triangle[0], triangle[1], triangle[2]];
            if [i0, i1, i2].into_iter().any(|index| index >= vertex_count) {
                return None;
            }
            let mut triangle = [i0, i1, i2];
            if reverse_output {
                triangle.swap(1, 2);
            }
            Some(triangle)
        })
        .collect()
}

fn project_to_finite_exact_2d(vertices: &[Vertex]) -> Option<Vec<hypertri::Point2>> {
    let finite = vertices
        .iter()
        .map(|vertex| {
            Some([
                vertex.position.x.to_f64_lossy()?,
                vertex.position.y.to_f64_lossy()?,
                vertex.position.z.to_f64_lossy()?,
            ])
        })
        .collect::<Option<Vec<_>>>()?;
    let mut normal = [0.0_f64; 3];
    for (current, next) in finite.iter().zip(finite.iter().cycle().skip(1)) {
        normal[0] += (current[1] - next[1]) * (current[2] + next[2]);
        normal[1] += (current[2] - next[2]) * (current[0] + next[0]);
        normal[2] += (current[0] - next[0]) * (current[1] + next[1]);
    }
    let drop_axis =
        (0..3).max_by(|left, right| normal[*left].abs().total_cmp(&normal[*right].abs()))?;
    if normal[drop_axis] == 0.0 {
        return None;
    }
    finite
        .into_iter()
        .map(|point| {
            let [x, y] = match drop_axis {
                0 => [point[1], point[2]],
                1 => [point[2], point[0]],
                _ => [point[0], point[1]],
            };
            Some(hypertri::Point2::new(
                Real::try_from(x).ok()?,
                Real::try_from(y).ok()?,
            ))
        })
        .collect()
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
    if matches!(
        squared[axis].refine_sign_until(-128),
        Some(RealSign::Positive)
    ) {
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
    matches!(area.refine_sign_until(-128), Some(RealSign::Negative))
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
