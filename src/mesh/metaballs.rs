//! Provides a `MetaBall` struct and functions for creating a `Mesh` from [MetaBalls](https://en.wikipedia.org/wiki/Metaballs)

use crate::hyper_math::{
    hreal_from_f32, hreal_from_f64, hreal_sign, hreal_to_f64, htriangle_area2_is_nonzero,
    hvector3_from_point3, hvector3_from_vector3,
};
use crate::mesh::Mesh;
use crate::mesh::Polygon;
use crate::vertex::Vertex;
use fast_surface_nets::{SurfaceNetsBuffer, surface_nets};
use hyperlattice::{Point3, Real, Vector3};
use hyperlimit::Point3 as HPoint3;
use hyperlimit::{real_max, real_min};
use hyperreal::RealSign;
use std::fmt::Debug;

#[derive(Debug, Clone)]
pub struct MetaBall {
    pub center: Point3,
    pub radius: Real,
}

/// Diagnostics captured while sampling and meshing 3D metaballs.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct MetaballDiagnostics {
    pub resolution: (u32, u32, u32),
    pub ball_count: usize,
    pub sample_count: usize,
    pub finite_sample_count: usize,
    pub non_finite_sample_count: usize,
    pub negative_sample_count: usize,
    pub zero_sample_count: usize,
    pub positive_sample_count: usize,
    pub min_finite_value: Option<Real>,
    pub max_finite_value: Option<Real>,
    pub crossing_cell_count: usize,
    pub surface_nets_vertex_count: usize,
    pub surface_nets_index_count: usize,
    pub emitted_triangle_count: usize,
    pub skipped_non_finite_triangle_count: usize,
    pub degenerate_triangle_count: usize,
}

impl MetaBall {
    pub const fn new(center: Point3, radius: Real) -> Self {
        Self { center, radius }
    }

    /// **Mathematical Foundation**: Metaball influence function I(p) = r²/|p-c|².
    ///
    /// This public method is an f64 reporting boundary. Internally, radius,
    /// distance, denominator, and division are evaluated as `Real`
    /// through [`MetaBall::influence_hreal`], following Yap's exact-geometric-
    /// computation boundary split (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    /// The implicit-field model follows Blinn, "A Generalization of Algebraic
    /// Surface Drawing," *ACM Transactions on Graphics* 1(3), 1982
    /// (<https://doi.org/10.1145/357306.357310>).
    pub fn influence(&self, p: &Point3) -> Real {
        self.influence_hreal(p).unwrap_or_else(Real::zero)
    }

    /// Return this metaball's finite influence as a hyperreal field value.
    ///
    /// This is the internal sampling primitive. It keeps the implicit scalar
    /// field in hyperreal space until diagnostics or `fast_surface_nets` require
    /// lossy boundary scalars. Exact center singularities are represented by a
    /// large finite sampling sentinel at that extraction boundary rather than by
    /// perturbing every denominator with a tolerance.
    fn influence_hreal(&self, p: &Point3) -> Option<Real> {
        let radius = hreal_from_f64(self.radius.clone()).ok()?;
        if !matches!(hreal_sign(&radius), Some(RealSign::Positive)) {
            return None;
        }

        let distance_squared = hyper_point_distance_squared(p, &self.center)?;
        let radius_squared = radius.clone() * radius;

        // Early termination optimization: if point is very far from the
        // metaball, influence approaches zero and can skip division.
        let threshold_distance_sq = radius_squared.clone() * hreal_from_f64(1000.0).ok()?;
        if matches!(
            hreal_sign(&(distance_squared.clone() - threshold_distance_sq)),
            Some(RealSign::Positive)
        ) {
            return Some(Real::zero());
        }

        if matches!(hreal_sign(&distance_squared), Some(RealSign::Zero)) {
            return singular_metaball_influence();
        }
        let denominator = distance_squared;
        (radius_squared / denominator).ok()
    }
}

fn singular_metaball_influence() -> Option<Real> {
    hreal_from_f64(1.0e10).ok()
}

impl<M: Clone + Debug + Send + Sync> Mesh<M> {
    /// **Creates a Mesh from a list of metaballs** by sampling a 3D grid and using marching cubes.
    ///
    /// - `balls`: slice of metaball definitions (center + radius).
    /// - `resolution`: (nx, ny, nz) defines how many steps along x, y, z.
    /// - `iso_value`: threshold at which the isosurface is extracted.
    /// - `padding`: extra margin around the bounding region (e.g. 0.5) so the surface doesn't get truncated.
    pub fn metaballs(
        balls: &[MetaBall],
        resolution: (usize, usize, usize),
        iso_value: Real,
        padding: Real,
        metadata: M,
    ) -> Mesh<M> {
        Self::metaballs_with_diagnostics(balls, resolution, iso_value, padding, metadata).0
    }

    /// Creates a Mesh from metaballs and returns diagnostics for sampling and
    /// surface-net triangle conversion.
    ///
    /// Metaball centers, radii, padding, iso values, generated vertices, and
    /// generated normals are promoted through hyperreal/hyperlattice boundary
    /// adapters before they contribute to mesh topology. csgrs therefore only
    /// composes the sampled surface into its transitional `Mesh` carrier while
    /// exact-aware predicates remain in the hyper crates, following Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7(1-2), 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>). Surface
    /// extraction follows Gibson, "Constrained Elastic Surface Nets," MERL
    /// TR99-24, 1999.
    pub fn metaballs_with_diagnostics(
        balls: &[MetaBall],
        resolution: (usize, usize, usize),
        iso_value: Real,
        padding: Real,
        metadata: M,
    ) -> (Mesh<M>, MetaballDiagnostics) {
        let nx = resolution.0.max(2) as u32;
        let ny = resolution.1.max(2) as u32;
        let nz = resolution.2.max(2) as u32;
        let mut diagnostics = MetaballDiagnostics {
            resolution: (nx, ny, nz),
            ball_count: balls.len(),
            sample_count: (nx * ny * nz) as usize,
            ..MetaballDiagnostics::default()
        };

        if balls.is_empty() {
            diagnostics.sample_count = 0;
            return (Mesh::empty(), diagnostics);
        }

        let valid_balls = balls
            .iter()
            .filter(|ball| {
                let Some(radius) = hreal_from_f64(ball.radius.clone()).ok() else {
                    return false;
                };
                hvector3_from_point3(&ball.center).is_some()
                    && matches!(hreal_sign(&radius), Some(RealSign::Positive))
            })
            .collect::<Vec<_>>();
        let Some(padding_h) = hreal_from_f64(padding).ok() else {
            diagnostics.non_finite_sample_count = diagnostics.sample_count;
            diagnostics.positive_sample_count = diagnostics.sample_count;
            return (Mesh::empty(), diagnostics);
        };
        let padding_is_negative = matches!(hreal_sign(&padding_h), Some(RealSign::Negative));
        let Some(iso_value_h) = hreal_from_f64(iso_value).ok() else {
            diagnostics.non_finite_sample_count = diagnostics.sample_count;
            diagnostics.positive_sample_count = diagnostics.sample_count;
            return (Mesh::empty(), diagnostics);
        };

        if valid_balls.is_empty() || padding_is_negative {
            diagnostics.non_finite_sample_count = diagnostics.sample_count;
            diagnostics.positive_sample_count = diagnostics.sample_count;
            return (Mesh::empty(), diagnostics);
        }

        let Some((min_pt, max_pt)) = metaball_bounds_hreal(&valid_balls, &padding_h) else {
            diagnostics.non_finite_sample_count = diagnostics.sample_count;
            diagnostics.positive_sample_count = diagnostics.sample_count;
            return (Mesh::empty(), diagnostics);
        };

        let Some(grid) = MetaballSamplingGrid::from_bounds(min_pt, max_pt, nx, ny, nz) else {
            diagnostics.non_finite_sample_count = diagnostics.sample_count;
            diagnostics.positive_sample_count = diagnostics.sample_count;
            return (Mesh::empty(), diagnostics);
        };

        // Create and fill the scalar-field array with "field_value - iso_value"
        // so that the isosurface will be at 0.
        let array_size = (nx * ny * nz) as usize;
        let mut field_values = MetaballSampleField::with_capacity(array_size);

        for iz in 0..nz {
            for iy in 0..ny {
                for ix in 0..nx {
                    let Some(p) = grid.point_at(ix, iy, iz) else {
                        diagnostics.non_finite_sample_count += 1;
                        diagnostics.positive_sample_count += 1;
                        field_values.push_nonfinite_sample();
                        continue;
                    };

                    let Some(field_value_h) =
                        valid_balls.iter().try_fold(Real::zero(), |acc, ball| {
                            ball.influence_hreal(&p).map(|value| acc + value)
                        })
                    else {
                        diagnostics.non_finite_sample_count += 1;
                        diagnostics.positive_sample_count += 1;
                        field_values.push_nonfinite_sample();
                        continue;
                    };
                    let shifted = field_value_h.clone() - iso_value_h.clone();

                    if field_values.push_hyper_sample(shifted.clone()) {
                        diagnostics.finite_sample_count += 1;
                        record_metaball_finite_sample(&mut diagnostics, &field_value_h);
                        match hreal_sign(&shifted) {
                            Some(RealSign::Negative) => diagnostics.negative_sample_count += 1,
                            Some(RealSign::Positive) => diagnostics.positive_sample_count += 1,
                            Some(RealSign::Zero) => diagnostics.zero_sample_count += 1,
                            None => {
                                diagnostics.non_finite_sample_count += 1;
                                diagnostics.positive_sample_count += 1;
                            },
                        }
                    } else {
                        diagnostics.non_finite_sample_count += 1;
                        diagnostics.positive_sample_count += 1;
                    }
                }
            }
        }

        // Use fast-surface-nets to extract a mesh from this 3D scalar field.
        // We'll define a shape type for ndshape:
        #[allow(non_snake_case)]
        #[derive(Clone, Copy)]
        struct GridShape {
            nx: u32,
            ny: u32,
            nz: u32,
        }
        impl fast_surface_nets::ndshape::Shape<3> for GridShape {
            type Coord = u32;

            #[inline]
            fn as_array(&self) -> [Self::Coord; 3] {
                [self.nx, self.ny, self.nz]
            }

            fn size(&self) -> Self::Coord {
                self.nx * self.ny * self.nz
            }

            fn usize(&self) -> usize {
                (self.nx * self.ny * self.nz) as usize
            }

            fn linearize(&self, coords: [Self::Coord; 3]) -> u32 {
                let [x, y, z] = coords;
                (z * self.ny + y) * self.nx + x
            }

            fn delinearize(&self, i: u32) -> [Self::Coord; 3] {
                let x = i % (self.nx);
                let yz = i / (self.nx);
                let y = yz % (self.ny);
                let z = yz / (self.ny);
                [x, y, z]
            }
        }

        let shape = GridShape { nx, ny, nz };
        diagnostics.crossing_cell_count =
            count_crossing_cells(&field_values.hyper_values, nx, ny, nz);

        // We'll collect the output into a SurfaceNetsBuffer
        let mut sn_buffer = SurfaceNetsBuffer::default();

        // The region we pass to surface_nets is the entire 3D range [0..nx, 0..ny, 0..nz]
        // minus 1 in each dimension to avoid indexing past the boundary:
        let (max_x, max_y, max_z) = (nx - 1, ny - 1, nz - 1);

        surface_nets(
            &field_values.surface_nets_values,
            &shape,
            [0, 0, 0],
            [max_x, max_y, max_z],
            &mut sn_buffer,
        );
        diagnostics.surface_nets_vertex_count = sn_buffer.positions.len();
        diagnostics.surface_nets_index_count = sn_buffer.indices.len();

        // Convert the resulting surface net indices/positions into Polygons
        // for the csgrs data structures.
        let mut triangles = Vec::with_capacity(sn_buffer.indices.len() / 3);

        for tri in sn_buffer.indices.chunks_exact(3) {
            let i0 = tri[0] as usize;
            let i1 = tri[1] as usize;
            let i2 = tri[2] as usize;

            let p0_index = sn_buffer.positions[i0];
            let p1_index = sn_buffer.positions[i1];
            let p2_index = sn_buffer.positions[i2];

            let Some(p0_real) = grid.point_from_surface_position(p0_index) else {
                diagnostics.skipped_non_finite_triangle_count += 1;
                continue;
            };
            let Some(p1_real) = grid.point_from_surface_position(p1_index) else {
                diagnostics.skipped_non_finite_triangle_count += 1;
                continue;
            };
            let Some(p2_real) = grid.point_from_surface_position(p2_index) else {
                diagnostics.skipped_non_finite_triangle_count += 1;
                continue;
            };

            // Likewise for the normals if you want them in true world space.
            // Usually you'd need to do an inverse-transpose transform if your
            // scale is non-uniform. For uniform voxels, scaling is simpler:

            let n0 = sn_buffer.normals[i0];
            let n1 = sn_buffer.normals[i1];
            let n2 = sn_buffer.normals[i2];

            let Some(n0v) = vector3_from_f32_boundary(n0) else {
                diagnostics.skipped_non_finite_triangle_count += 1;
                continue;
            };
            let Some(n1v) = vector3_from_f32_boundary(n1) else {
                diagnostics.skipped_non_finite_triangle_count += 1;
                continue;
            };
            let Some(n2v) = vector3_from_f32_boundary(n2) else {
                diagnostics.skipped_non_finite_triangle_count += 1;
                continue;
            };

            if !(finite_point3(&p0_real)
                && finite_point3(&p1_real)
                && finite_point3(&p2_real)
                && finite_vector3(&n0v)
                && finite_vector3(&n1v)
                && finite_vector3(&n2v))
            {
                diagnostics.skipped_non_finite_triangle_count += 1;
                continue;
            }

            if !htriangle_area2_is_nonzero(&p0_real, &p1_real, &p2_real) {
                diagnostics.degenerate_triangle_count += 1;
            }

            // Construct your vertices:
            let v0 = Vertex::new(p0_real, n0v);
            let v1 = Vertex::new(p1_real, n1v);
            let v2 = Vertex::new(p2_real, n2v);

            // Each tri is turned into a Polygon with 3 vertices
            let poly = Polygon::new(vec![v0, v2, v1], metadata.clone());
            triangles.push(poly);
            diagnostics.emitted_triangle_count += 1;
        }

        // Build and return a Mesh from these polygons
        (Mesh::from_polygons(triangles), diagnostics)
    }
}

#[derive(Clone, Debug)]
struct MetaballSampleField {
    hyper_values: Vec<Real>,
    surface_nets_values: Vec<f32>,
}

impl MetaballSampleField {
    fn with_capacity(capacity: usize) -> Self {
        Self {
            hyper_values: Vec::with_capacity(capacity),
            surface_nets_values: Vec::with_capacity(capacity),
        }
    }

    fn push_hyper_sample(&mut self, shifted: Real) -> bool {
        let Some(surface_value) = surface_nets_scalar(&shifted) else {
            self.push_nonfinite_sample();
            return false;
        };
        self.hyper_values.push(shifted);
        self.surface_nets_values.push(surface_value);
        true
    }

    fn push_nonfinite_sample(&mut self) {
        let sentinel = hreal_from_f64(1.0e10).expect("finite metaball sentinel");
        self.hyper_values.push(sentinel);
        self.surface_nets_values.push(1.0e10_f32);
    }
}

fn record_metaball_finite_sample(diagnostics: &mut MetaballDiagnostics, value: &Real) {
    diagnostics.min_finite_value = match diagnostics.min_finite_value.take() {
        Some(current) => hyperlimit::real_min(&current, value).value().cloned(),
        None => Some(value.clone()),
    };
    diagnostics.max_finite_value = match diagnostics.max_finite_value.take() {
        Some(current) => hyperlimit::real_max(&current, value).value().cloned(),
        None => Some(value.clone()),
    };
}

fn metaball_bounds_hreal(balls: &[&MetaBall], padding: &Real) -> Option<(HPoint3, HPoint3)> {
    let mut bounds = balls.iter().map(|ball| {
        let center = HPoint3::new(
            ball.center.x.clone(),
            ball.center.y.clone(),
            ball.center.z.clone(),
        );
        let radius = hreal_from_f64(ball.radius.clone()).ok()?;
        let extent = radius + padding.clone();
        Some((
            HPoint3::new(
                center.x.clone() - extent.clone(),
                center.y.clone() - extent.clone(),
                center.z.clone() - extent.clone(),
            ),
            HPoint3::new(
                center.x + extent.clone(),
                center.y + extent.clone(),
                center.z + extent,
            ),
        ))
    });
    let (mut min_pt, mut max_pt) = bounds.next()??;
    for bounds in bounds {
        let (next_min, next_max) = bounds?;
        min_pt = HPoint3::new(
            real_min(&min_pt.x, &next_min.x).value().cloned()?,
            real_min(&min_pt.y, &next_min.y).value().cloned()?,
            real_min(&min_pt.z, &next_min.z).value().cloned()?,
        );
        max_pt = HPoint3::new(
            real_max(&max_pt.x, &next_max.x).value().cloned()?,
            real_max(&max_pt.y, &next_max.y).value().cloned()?,
            real_max(&max_pt.z, &next_max.z).value().cloned()?,
        );
    }
    Some((min_pt, max_pt))
}

#[derive(Clone, Debug)]
struct MetaballSamplingGrid {
    origin: HPoint3,
    step: HPoint3,
}

impl MetaballSamplingGrid {
    fn from_bounds(
        min_pt: HPoint3,
        max_pt: HPoint3,
        nx: u32,
        ny: u32,
        nz: u32,
    ) -> Option<Self> {
        let origin = min_pt;
        let max = max_pt;
        Some(Self {
            step: HPoint3::new(
                ((max.x - origin.x.clone()) / Real::from(u64::from(nx - 1))).ok()?,
                ((max.y - origin.y.clone()) / Real::from(u64::from(ny - 1))).ok()?,
                ((max.z - origin.z.clone()) / Real::from(u64::from(nz - 1))).ok()?,
            ),
            origin,
        })
    }

    fn point_at(&self, ix: u32, iy: u32, iz: u32) -> Option<Point3> {
        let x = self.origin.x.clone() + self.step.x.clone() * Real::from(u64::from(ix));
        let y = self.origin.y.clone() + self.step.y.clone() * Real::from(u64::from(iy));
        let z = self.origin.z.clone() + self.step.z.clone() * Real::from(u64::from(iz));
        Some(Point3::new(x, y, z))
    }

    fn point_from_surface_position(&self, position: [f32; 3]) -> Option<Point3> {
        let x =
            self.origin.x.clone() + self.step.x.clone() * hreal_from_f32(position[0]).ok()?;
        let y =
            self.origin.y.clone() + self.step.y.clone() * hreal_from_f32(position[1]).ok()?;
        let z =
            self.origin.z.clone() + self.step.z.clone() * hreal_from_f32(position[2]).ok()?;
        Some(Point3::new(x, y, z))
    }
}

fn hyper_point_distance_squared(lhs: &Point3, rhs: &Point3) -> Option<Real> {
    let lhs = hvector3_from_point3(lhs)?;
    let rhs = hvector3_from_point3(rhs)?;
    Some(lhs.squared_distance(&rhs))
}

#[inline]
fn finite_point3(point: &Point3) -> bool {
    hvector3_from_point3(point).is_some()
}

#[inline]
fn finite_vector3(vector: &Vector3) -> bool {
    hvector3_from_vector3(vector).is_some()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hyper_math::{Real, hreal_from_f64, hreal_gt_f64, tolerance};

    fn r(value: f64) -> Real {
        hreal_from_f64(value).expect("test values must be finite")
    }

    fn p3(x: Real, y: f64, z: f64) -> Point3 {
        Point3::new(x, r(y), r(z))
    }

    #[test]
    fn metaball_influence_uses_exact_center_singularity_without_tolerance() {
        let ball = MetaBall::new(Point3::new(r(0.0), r(0.0), r(0.0)), r(1.0));
        let center = ball
            .influence_hreal(&Point3::new(r(0.0), r(0.0), r(0.0)))
            .unwrap();
        let tiny_offset = ball
            .influence_hreal(&p3(tolerance() * r(0.25), 0.0, 0.0))
            .unwrap();

        assert!(hreal_gt_f64(&center, 1.0e9));
        assert!(hreal_gt_f64(&tiny_offset, 1.0e9));
    }

    #[test]
    fn metaball_influence_rejects_exact_zero_radius() {
        let ball = MetaBall::new(Point3::new(r(0.0), r(0.0), r(0.0)), r(0.0));
        assert!(
            ball.influence_hreal(&Point3::new(r(0.0), r(0.0), r(0.0)))
                .is_none()
        );
    }
}

fn surface_nets_scalar(value: &Real) -> Option<f32> {
    let boundary = hreal_to_f64(value)?;
    // Conversion refines the expression and warms its certified approximation;
    // the exact sign query can then reuse that work for the topology decision.
    let sign = hreal_sign(value)?;
    let value = boundary as f32;
    let value = if value == 0.0 {
        match sign {
            RealSign::Negative => -f32::MIN_POSITIVE,
            RealSign::Positive => f32::MIN_POSITIVE,
            RealSign::Zero => 0.0,
        }
    } else if value.is_infinite() {
        match sign {
            RealSign::Negative => -f32::MAX,
            RealSign::Positive => f32::MAX,
            RealSign::Zero => 0.0,
        }
    } else {
        value
    };
    hreal_from_f32(value).ok()?;
    Some(value)
}

fn vector3_from_f32_boundary(vector: [f32; 3]) -> Option<Vector3> {
    Vector3::try_from_f32_array(vector).ok()
}

fn count_crossing_cells(field_values: &[Real], nx: u32, ny: u32, nz: u32) -> usize {
    if nx < 2 || ny < 2 || nz < 2 {
        return 0;
    }

    let index = |x: u32, y: u32, z: u32| -> usize { ((z * ny + y) * nx + x) as usize };
    let mut count = 0;

    for z in 0..(nz - 1) {
        for y in 0..(ny - 1) {
            for x in 0..(nx - 1) {
                let corners = [
                    &field_values[index(x, y, z)],
                    &field_values[index(x + 1, y, z)],
                    &field_values[index(x, y + 1, z)],
                    &field_values[index(x + 1, y + 1, z)],
                    &field_values[index(x, y, z + 1)],
                    &field_values[index(x + 1, y, z + 1)],
                    &field_values[index(x, y + 1, z + 1)],
                    &field_values[index(x + 1, y + 1, z + 1)],
                ];
                let signs = corners.map(hreal_sign);
                let has_negative = signs.contains(&Some(RealSign::Negative));
                let has_positive = signs.contains(&Some(RealSign::Positive));
                let has_zero = signs.contains(&Some(RealSign::Zero));
                if (has_negative && has_positive) || has_zero {
                    count += 1;
                }
            }
        }
    }

    count
}
