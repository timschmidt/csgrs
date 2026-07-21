use super::Polygon;
use crate::mesh::plane::first_nondegenerate_support;
use crate::vertex::Vertex;
use hyperlattice::{Point3, Real, Vector3};
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
        let mut triangles = Vec::with_capacity(self.vertices.len().saturating_sub(2));
        let mut projected = Vec::with_capacity(self.vertices.len());
        self.triangulate_indices_into(&mut triangles, &mut projected);
        triangles
    }

    pub(crate) fn triangulate_indices_into(
        &self,
        triangles: &mut Vec<[usize; 3]>,
        projected: &mut Vec<hypertri::Point2>,
    ) {
        triangulate_positions_into(
            self.vertices.len(),
            |index| &self.vertices[index].position,
            triangles,
            projected,
        );
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
        let mut triangles = Vec::with_capacity(indices.len() / 3);
        extend_triangle_indices(&mut triangles, &indices, self.vertices.len(), reverse_output);
        triangles
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

    /// Uniformly subdivide this triangle into smaller triangles.
    pub fn subdivide_to_triangles(&self, subdivisions: NonZeroU32) -> Vec<Polygon<M>> {
        self.subdivide_triangles(subdivisions)
            .into_iter()
            .map(|triangle| {
                Polygon::from_planar_vertices(triangle.to_vec(), self.metadata.clone())
                    .with_plane_id(self.plane_id)
            })
            .collect()
    }
}

#[cfg(feature = "obj-io")]
pub(crate) fn triangulate_indexed_positions_into(
    positions: &[Point3],
    indices: &[usize],
    triangles: &mut Vec<[usize; 3]>,
    projected: &mut Vec<hypertri::Point2>,
) {
    triangulate_positions_into(
        indices.len(),
        |index| &positions[indices[index]],
        triangles,
        projected,
    );
}

fn triangulate_positions_into<'a>(
    vertex_count: usize,
    position: impl Copy + Fn(usize) -> &'a Point3,
    triangles: &mut Vec<[usize; 3]>,
    projected: &mut Vec<hypertri::Point2>,
) {
    triangles.clear();
    if vertex_count < 3 {
        return;
    }
    if vertex_count == 3 {
        triangles.push([0, 1, 2]);
        return;
    }

    let Some((support, drop_axis, support_sign)) =
        first_nondegenerate_projection(vertex_count, position)
    else {
        return;
    };
    if support == [0, 1, 2]
        && strictly_convex_exact_word_projection(
            vertex_count,
            position,
            drop_axis,
            support_sign,
        )
    {
        triangles.extend((1..vertex_count - 1).map(|i| [0, i, i + 1]));
        return;
    }
    project_to_exact_2d_into(vertex_count, position, drop_axis, projected);
    let first_turn = (support == [0, 1, 2]).then_some(support_sign);
    if strictly_convex(projected, first_turn) {
        triangles.extend((1..projected.len() - 1).map(|i| [0, i, i + 1]));
        return;
    }
    let reverse_output = winding_is_negative(projected);
    let Ok(indices) = hypertri::earcut(projected, &[]) else {
        return;
    };
    extend_triangle_indices(triangles, &indices, vertex_count, reverse_output);
}

fn strictly_convex_exact_word_projection<'a>(
    vertex_count: usize,
    position: impl Copy + Fn(usize) -> &'a Point3,
    drop_axis: usize,
    winding: Sign,
) -> bool {
    let projected = |point: &'a Point3| match drop_axis {
        0 => [&point.y, &point.z],
        1 => [&point.z, &point.x],
        _ => [&point.x, &point.y],
    };
    for index in 1..vertex_count {
        let a = projected(position(index));
        let b = projected(position((index + 1) % vertex_count));
        let c = projected(position((index + 2) % vertex_count));
        let Some(sign) = Real::prepare_affine_det2_exact_word_filter(a, b)
            .and_then(|filter| filter.sign(c))
        else {
            return false;
        };
        let sign = match sign {
            RealSign::Positive => Sign::Positive,
            RealSign::Negative => Sign::Negative,
            RealSign::Zero => return false,
        };
        if sign != winding {
            return false;
        }
    }
    true
}

fn strictly_convex(points: &[hypertri::Point2], first_turn: Option<Sign>) -> bool {
    let mut winding = first_turn;
    for index in usize::from(first_turn.is_some())..points.len() {
        let Ok(sign) = ExactKernel::orient2d(
            &points[index],
            &points[(index + 1) % points.len()],
            &points[(index + 2) % points.len()],
        ) else {
            return false;
        };
        if sign == Sign::Zero {
            return false;
        }
        match winding {
            None => winding = Some(sign),
            Some(expected) if expected == sign => {},
            Some(_) => return false,
        }
    }
    true
}

fn extend_triangle_indices(
    output: &mut Vec<[usize; 3]>,
    indices: &[usize],
    vertex_count: usize,
    reverse_output: bool,
) {
    output.extend(indices.chunks_exact(3).filter_map(|triangle| {
        let [i0, i1, i2] = [triangle[0], triangle[1], triangle[2]];
        if [i0, i1, i2].into_iter().any(|index| index >= vertex_count) {
            return None;
        }
        let mut triangle = [i0, i1, i2];
        if reverse_output {
            triangle.swap(1, 2);
        }
        Some(triangle)
    }));
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

fn project_to_exact_2d_into<'a>(
    vertex_count: usize,
    position: impl Copy + Fn(usize) -> &'a Point3,
    drop_axis: usize,
    projected: &mut Vec<hypertri::Point2>,
) {
    projected.clear();
    projected.extend((0..vertex_count).map(|index| match drop_axis {
        0 => hypertri::Point2::new(position(index).y.clone(), position(index).z.clone()),
        1 => hypertri::Point2::new(position(index).z.clone(), position(index).x.clone()),
        _ => hypertri::Point2::new(position(index).x.clone(), position(index).y.clone()),
    }));
}

fn first_nondegenerate_projection<'a>(
    vertex_count: usize,
    position: impl Copy + Fn(usize) -> &'a Point3,
) -> Option<([usize; 3], usize, Sign)> {
    if vertex_count >= 3
        && let Some((axis, sign)) = Real::exact_rational_dominant_affine_cross_axis(
            [&position(0).x, &position(0).y, &position(0).z],
            [&position(1).x, &position(1).y, &position(1).z],
            [&position(2).x, &position(2).y, &position(2).z],
        )
    {
        let sign = match sign {
            RealSign::Positive => Sign::Positive,
            RealSign::Negative => Sign::Negative,
            RealSign::Zero => return None,
        };
        return Some(([0, 1, 2], axis, sign));
    }

    let (support, normal) = first_nondegenerate_support(vertex_count, position)?;
    let (axis, sign) = dominant_axis(&normal)?;
    Some((support, axis, sign))
}

fn dominant_axis(normal: &Vector3) -> Option<(usize, Sign)> {
    let mut axis = 0;
    for candidate in 1..3 {
        let ordering = match (
            normal.0[candidate].exact_rational_ref(),
            normal.0[axis].exact_rational_ref(),
        ) {
            (Some(candidate), Some(current)) => {
                match (candidate.is_negative(), current.is_negative()) {
                    (false, false) => candidate.partial_cmp(current),
                    (true, true) => current.partial_cmp(candidate),
                    (false, true) => candidate.partial_cmp(&(-current)),
                    (true, false) => (-candidate).partial_cmp(current),
                }
                .unwrap_or(Ordering::Equal)
            },
            _ => {
                let candidate = normal.0[candidate].clone() * normal.0[candidate].clone();
                let current = normal.0[axis].clone() * normal.0[axis].clone();
                hyperlimit::compare_reals(&candidate, &current)
                    .value()
                    .unwrap_or(Ordering::Equal)
            },
        };
        if ordering == Ordering::Greater {
            axis = candidate;
        }
    }
    match normal.0[axis].refine_sign_until(-128) {
        Some(RealSign::Positive) => Some((axis, Sign::Positive)),
        Some(RealSign::Negative) => Some((axis, Sign::Negative)),
        _ => None,
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
