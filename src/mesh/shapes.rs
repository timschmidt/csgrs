//! 3D Shapes as `Mesh`s

use crate::csg::CSG;
use crate::errors::ValidationError;
use crate::mesh::Mesh;
use crate::mesh::Polygon;
use crate::mesh::polygon::fresh_plane_id;
#[cfg(feature = "sketch")]
use crate::sketch::Profile;
use crate::vertex::Vertex;
#[cfg(feature = "sketch")]
use hypercurve::triangulate_finite_rings;
use hyperlattice::{Point3, Real, Vector3};
use hyperreal::RealSign;
use std::cmp::Ordering;
use std::collections::HashMap;
use std::fmt::Debug;

/// Accept any finite, strictly positive mesh scalar exactly.
///
/// Mesh constructors are still fed by primitive boundary scalars, but admission
/// decisions should not collapse small nonzero values through a tolerance band.
/// This follows Yap, "Towards Exact Geometric Computation," *Computational
/// Geometry* 7(1-2), 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>).
fn real_cmp(lhs: &Real, rhs: &Real) -> Option<Ordering> {
    hyperlimit::compare_reals(lhs, rhs).value().or_else(|| {
        match (lhs.clone() - rhs.clone()).refine_sign_until(-128) {
            Some(RealSign::Positive) => Ordering::Greater,
            Some(RealSign::Negative) => Ordering::Less,
            Some(RealSign::Zero) => Ordering::Equal,
            None => return None,
        }
        .into()
    })
}

#[cfg(test)]
mod retained_topology_tests {
    use super::*;
    use crate::csg::CSG;

    #[test]
    fn frustum_reuses_ring_positions_through_transform() {
        let mesh = Mesh::frustum_ptp(
            Point3::origin(),
            Point3::new(Real::zero(), Real::zero(), Real::from(2_u8)),
            Real::from(2_u8),
            Real::from(3_u8),
            6,
            (),
        );
        assert_eq!(
            mesh.polygons[0].vertices[2].position,
            mesh.polygons[2].vertices[0].position
        );
        assert_eq!(
            mesh.polygons[0].vertices[2].position_id,
            mesh.polygons[2].vertices[0].position_id
        );

        let rotated = mesh.rotate(Real::from(-35_i8), Real::zero(), Real::zero());
        assert_eq!(
            rotated.polygons[0].vertices[2].position_id,
            rotated.polygons[2].vertices[0].position_id
        );
        let translated = rotated.translate(Real::zero(), Real::from(5_u8), Real::from(2_u8));
        assert_eq!(
            translated.polygons[0].vertices[2].position_id,
            translated.polygons[2].vertices[0].position_id
        );
    }
}

fn hmesh_scalar_positive(value: &Real) -> bool {
    matches!(real_cmp(value, &Real::zero()), Some(Ordering::Greater))
}

fn hmesh_scalar_nonzero(value: &Real) -> bool {
    matches!(
        real_cmp(value, &Real::zero()),
        Some(Ordering::Less | Ordering::Greater)
    )
}

fn hmesh_scalar_nonnegative(value: &Real) -> bool {
    matches!(
        real_cmp(value, &Real::zero()),
        Some(Ordering::Equal | Ordering::Greater)
    )
}

fn real_from_ratio(numerator: u64, denominator: u64) -> Option<Real> {
    (Real::from(numerator) / Real::from(denominator)).ok()
}

fn sampled_sin_cos(index: usize, count: usize, sweep: &Real) -> Option<(Real, Real)> {
    let angle = sweep.to_f64_lossy()? * index as f64 / count as f64;
    let (sin, cos) = angle.sin_cos();
    Some((Real::try_from(sin).ok()?, Real::try_from(cos).ok()?))
}

fn retain_equal_coordinate_ids<'a>(vertices: impl IntoIterator<Item = &'a mut Vertex>) {
    let mut coordinates = HashMap::<(usize, Option<u64>), Vec<(Real, u64)>>::new();
    for vertex in vertices {
        for axis in 0..3 {
            let coordinate = match axis {
                0 => &vertex.position.x,
                1 => &vertex.position.y,
                _ => &vertex.position.z,
            };
            let bucket = coordinate.to_f64_lossy().map(f64::to_bits);
            let entries = coordinates.entry((axis, bucket)).or_default();
            if let Some((_, id)) = entries.iter().find(|(value, _)| value == coordinate) {
                vertex.coordinate_ids[axis] = *id;
            } else {
                entries.push((coordinate.clone(), vertex.coordinate_ids[axis]));
            }
        }
    }
}

fn assembled_arrow<M: Clone + Debug + Send + Sync>(
    start: Point3,
    direction: Vector3,
    segments: usize,
    orientation: bool,
    metadata: M,
) -> Mesh<M> {
    if segments < 3 {
        return Mesh::empty();
    }
    let Some(length) = direction.dot(&direction).sqrt().ok() else {
        return Mesh::empty();
    };
    if !hmesh_scalar_positive(&length) {
        return Mesh::empty();
    }
    let Ok(axis) = direction.normalize_checked() else {
        return Mesh::empty();
    };
    let Ok((axis_x, axis_y)) = axis.orthonormal_basis_checked() else {
        return Mesh::empty();
    };
    let shaft_length = length.clone() * real_from_ratio(4, 5).expect("nonzero denominator");
    let head_length = length.clone() - shaft_length.clone();
    let shaft_radius = length.clone() * real_from_ratio(3, 100).expect("nonzero denominator");
    let head_radius = length * real_from_ratio(3, 50).expect("nonzero denominator");
    let end = start.clone() + direction;
    let head_base = if orientation {
        start.clone() + axis.clone() * head_length
    } else {
        start.clone() + axis.clone() * shaft_length
    };
    let (shaft_start, shaft_end, head_start, head_end, head_r1, head_r2) = if orientation {
        (
            head_base.clone(),
            end,
            start,
            head_base.clone(),
            Real::zero(),
            head_radius.clone(),
        )
    } else {
        (
            start,
            head_base.clone(),
            head_base.clone(),
            end,
            head_radius.clone(),
            Real::zero(),
        )
    };
    let shaft = Mesh::frustum_ptp(
        shaft_start,
        shaft_end,
        shaft_radius.clone(),
        shaft_radius.clone(),
        segments,
        metadata.clone(),
    );
    let head = Mesh::frustum_ptp(
        head_start,
        head_end,
        head_r1,
        head_r2,
        segments,
        metadata.clone(),
    );
    let mut polygons = shaft
        .polygons
        .iter()
        .enumerate()
        .filter(|(index, _)| {
            if orientation {
                index % 3 != 0
            } else {
                index % 3 != 1
            }
        })
        .map(|(_, polygon)| polygon.clone())
        .chain(
            head.polygons
                .iter()
                .enumerate()
                .filter(|(index, _)| index % 2 == 1)
                .map(|(_, polygon)| polygon.clone()),
        )
        .collect::<Vec<_>>();

    let shoulder_normal = if orientation { axis.clone() } else { -axis };
    for segment in 0..segments {
        let Some((sin0, cos0)) = sampled_sin_cos(segment, segments, &Real::tau()) else {
            return Mesh::empty();
        };
        let Some((sin1, cos1)) = sampled_sin_cos(segment + 1, segments, &Real::tau()) else {
            return Mesh::empty();
        };
        let ring_point = |radius: &Real, sin: &Real, cos: &Real| {
            let radial = axis_x.clone() * cos.clone() + axis_y.clone() * sin.clone();
            head_base.clone() + radial * radius.clone()
        };
        let inner0 = Vertex::new(
            ring_point(&shaft_radius, &sin0, &cos0),
            shoulder_normal.clone(),
        );
        let inner1 = Vertex::new(
            ring_point(&shaft_radius, &sin1, &cos1),
            shoulder_normal.clone(),
        );
        let outer0 = Vertex::new(
            ring_point(&head_radius, &sin0, &cos0),
            shoulder_normal.clone(),
        );
        let outer1 = Vertex::new(
            ring_point(&head_radius, &sin1, &cos1),
            shoulder_normal.clone(),
        );
        let vertices = if orientation {
            vec![inner0, outer0, outer1, inner1]
        } else {
            vec![inner0, inner1, outer1, outer0]
        };
        polygons.push(Polygon::new(vertices, metadata.clone()));
    }
    Mesh::from_polygons(polygons)
}

#[cfg(feature = "sketch")]
fn positive_x_half(profile: &Profile) -> Option<Profile> {
    let (min_x, min_y, max_x, max_y) = profile.native_xy_bounds()?;
    match real_cmp(&min_x, &Real::zero())? {
        Ordering::Equal | Ordering::Greater => return Some(profile.clone()),
        Ordering::Less => {},
    }
    if !matches!(real_cmp(&max_x, &Real::zero())?, Ordering::Greater) {
        return None;
    }
    let width = -min_x.clone();
    let height = max_y - min_y.clone();
    if !hmesh_scalar_positive(&width) || !hmesh_scalar_positive(&height) {
        return None;
    }
    let cutter = Profile::rectangle(width, height).translate(min_x, min_y, Real::zero());
    profile.try_difference(&cutter).ok()
}

#[cfg(feature = "sketch")]
fn twisted_profile_extrusion<M: Clone + Debug + Send + Sync>(
    profile: &Profile,
    thickness: &Real,
    total_twist: Real,
    slices: usize,
    metadata: M,
) -> Mesh<M> {
    let profiles = profile.region_profiles();
    if profiles.len() != 1 || !profiles[0].holes().is_empty() {
        return Mesh::empty();
    }
    let mut exterior = profiles[0].material().points().to_vec();
    if exterior.len() > 1 && exterior.first() == exterior.last() {
        exterior.pop();
    }
    if exterior.len() < 3 {
        return Mesh::empty();
    }
    let mut closed = exterior.clone();
    closed.push(exterior[0]);
    let Ok(triangles) = triangulate_finite_rings(&closed, &[]) else {
        return Mesh::empty();
    };
    let transform_point = |point: [f64; 2], level: usize| -> Option<Point3> {
        let t = real_from_ratio(level as u64, slices as u64)?;
        let angle = total_twist.clone() * t.clone();
        let sin = angle.clone().sin();
        let cos = angle.cos();
        let x = Real::try_from(point[0]).ok()?;
        let y = Real::try_from(point[1]).ok()?;
        Some(Point3::new(
            x.clone() * cos.clone() - y.clone() * sin.clone(),
            x * sin + y * cos,
            thickness.clone() * t,
        ))
    };
    let rings = (0..=slices)
        .map(|level| {
            exterior
                .iter()
                .copied()
                .map(|point| transform_point(point, level))
                .collect::<Option<Vec<_>>>()
        })
        .collect::<Option<Vec<_>>>();
    let Some(rings) = rings else {
        return Mesh::empty();
    };
    let mut polygons = Vec::new();
    for triangle in triangles {
        let bottom = triangle.map(|point| transform_point(point, 0));
        let top = triangle.map(|point| transform_point(point, slices));
        let [Some(b0), Some(b1), Some(b2)] = bottom else {
            return Mesh::empty();
        };
        let [Some(t0), Some(t1), Some(t2)] = top else {
            return Mesh::empty();
        };
        polygons.push(Polygon::new(
            vec![
                Vertex::new(b2, -Vector3::z()),
                Vertex::new(b1, -Vector3::z()),
                Vertex::new(b0, -Vector3::z()),
            ],
            metadata.clone(),
        ));
        polygons.push(Polygon::new(
            vec![
                Vertex::new(t0, Vector3::z()),
                Vertex::new(t1, Vector3::z()),
                Vertex::new(t2, Vector3::z()),
            ],
            metadata.clone(),
        ));
    }
    for level in 0..slices {
        for index in 0..exterior.len() {
            let next = (index + 1) % exterior.len();
            let mut side = Polygon::new(
                vec![
                    Vertex::new(rings[level][index].clone(), Vector3::zeros()),
                    Vertex::new(rings[level][next].clone(), Vector3::zeros()),
                    Vertex::new(rings[level + 1][next].clone(), Vector3::zeros()),
                    Vertex::new(rings[level + 1][index].clone(), Vector3::zeros()),
                ],
                metadata.clone(),
            );
            side.set_new_normal();
            polygons.push(side);
        }
    }
    Mesh::from_polygons(polygons)
}

impl<M: Clone + Debug + Send + Sync> Mesh<M> {
    /// **Mathematical Foundations for 3D Box Geometry**
    ///
    /// This module implements mathematically rigorous algorithms for generating
    /// axis-aligned rectangular prisms (cuboids) and cubes based on solid geometry
    /// and computational topology principles.
    ///
    /// ## **Theoretical Foundations**
    ///
    /// ### **Cuboid Geometry**
    /// A right rectangular prism (cuboid) in 3D space is defined by:
    /// - **Vertices**: 8 corner points forming a rectangular parallelepiped
    /// - **Edges**: 12 edges connecting adjacent vertices
    /// - **Faces**: 6 rectangular faces, each with consistent outward normal
    ///
    /// ### **Coordinate System**
    /// Standard axis-aligned cuboid from origin:
    /// ```text
    /// (0,0,0) → (width, length, height)
    /// ```text
    /// This creates a right-handed coordinate system with consistent face orientations.
    ///
    /// ### **Face Normal Calculation**
    /// Each face normal is computed using the right-hand rule:
    /// ```text
    /// n⃗ = (v⃗₁ - v⃗₀) × (v⃗₂ - v⃗₀)
    /// ```text
    /// where vertices are ordered counter-clockwise when viewed from outside.
    ///
    /// ### **Winding Order Convention**
    /// All faces use counter-clockwise vertex ordering when viewed from exterior:
    /// - **Ensures consistent outward normals**
    /// - **Enables proper backface culling**
    /// - **Maintains manifold topology for CSG operations**
    ///
    /// ## **Geometric Properties**
    /// - **Volume**: V = width × length × height
    /// - **Surface Area**: A = 2(wl + wh + lh)
    /// - **Diagonal**: d = √(w² + l² + h²)
    /// - **Centroid**: (w/2, l/2, h/2)
    pub fn cuboid(width: Real, length: Real, height: Real, metadata: M) -> Mesh<M> {
        if !(hmesh_scalar_positive(&width)
            && hmesh_scalar_positive(&length)
            && hmesh_scalar_positive(&height))
        {
            return Mesh::empty();
        }
        // Define the eight corner points of the prism.
        //    (x, y, z)
        let p000 = Point3::origin();
        let p100 = Point3::new(width.clone(), Real::zero(), Real::zero());
        let p110 = Point3::new(width.clone(), length.clone(), Real::zero());
        let p010 = Point3::new(Real::zero(), length.clone(), Real::zero());

        let p001 = Point3::new(Real::zero(), Real::zero(), height.clone());
        let p101 = Point3::new(width.clone(), Real::zero(), height.clone());
        let p111 = Point3::new(width.clone(), length.clone(), height.clone());
        let p011 = Point3::new(Real::zero(), length.clone(), height.clone());

        // We’ll define 6 faces (each a Polygon), in an order that keeps outward-facing normals
        // and consistent (counter-clockwise) vertex winding as viewed from outside the prism.

        // Bottom face (z=0, normal approx. -Z)
        // p000 -> p100 -> p110 -> p010
        let bottom_normal = -Vector3::z();
        let bottom = Polygon::new(
            vec![
                Vertex::new(p000.clone(), bottom_normal.clone()),
                Vertex::new(p010.clone(), bottom_normal.clone()),
                Vertex::new(p110.clone(), bottom_normal.clone()),
                Vertex::new(p100.clone(), bottom_normal.clone()),
            ],
            metadata.clone(),
        );

        // Top face (z=depth, normal approx. +Z)
        // p001 -> p011 -> p111 -> p101
        let top_normal = Vector3::z();
        let top = Polygon::new(
            vec![
                Vertex::new(p001.clone(), top_normal.clone()),
                Vertex::new(p101.clone(), top_normal.clone()),
                Vertex::new(p111.clone(), top_normal.clone()),
                Vertex::new(p011.clone(), top_normal.clone()),
            ],
            metadata.clone(),
        );

        // Front face (y=0, normal approx. -Y)
        // p000 -> p001 -> p101 -> p100
        let front_normal = -Vector3::y();
        let front = Polygon::new(
            vec![
                Vertex::new(p000.clone(), front_normal.clone()),
                Vertex::new(p100.clone(), front_normal.clone()),
                Vertex::new(p101.clone(), front_normal.clone()),
                Vertex::new(p001.clone(), front_normal.clone()),
            ],
            metadata.clone(),
        );

        // Back face (y=height, normal approx. +Y)
        // p010 -> p110 -> p111 -> p011
        let back_normal = Vector3::y();
        let back = Polygon::new(
            vec![
                Vertex::new(p010.clone(), back_normal.clone()),
                Vertex::new(p011.clone(), back_normal.clone()),
                Vertex::new(p111.clone(), back_normal.clone()),
                Vertex::new(p110.clone(), back_normal.clone()),
            ],
            metadata.clone(),
        );

        // Left face (x=0, normal approx. -X)
        // p000 -> p010 -> p011 -> p001
        let left_normal = -Vector3::x();
        let left = Polygon::new(
            vec![
                Vertex::new(p000.clone(), left_normal.clone()),
                Vertex::new(p001.clone(), left_normal.clone()),
                Vertex::new(p011.clone(), left_normal.clone()),
                Vertex::new(p010.clone(), left_normal.clone()),
            ],
            metadata.clone(),
        );

        // Right face (x=width, normal approx. +X)
        // p100 -> p101 -> p111 -> p110
        let right_normal = Vector3::x();
        let right = Polygon::new(
            vec![
                Vertex::new(p100, right_normal.clone()),
                Vertex::new(p110, right_normal.clone()),
                Vertex::new(p111, right_normal.clone()),
                Vertex::new(p101, right_normal),
            ],
            metadata.clone(),
        );

        // Combine all faces into a Mesh
        Mesh::from_polygons(vec![bottom, top, front, back, left, right])
    }

    pub fn cube(width: Real, metadata: M) -> Mesh<M> {
        Self::cuboid(width.clone(), width.clone(), width, metadata)
    }

    /// **Mathematical Foundation: Spherical Mesh Generation**
    ///
    /// Construct a sphere using UV-parameterized quadrilateral tessellation.
    /// This implements the standard spherical coordinate parameterization
    /// with adaptive handling of polar degeneracies.
    ///
    /// ## **Sphere Mathematics**
    ///
    /// ### **Parametric Surface Equations**
    /// The sphere surface is defined by:
    /// ```text
    /// M(u,v) = r(sin(πv)cos(2πu), cos(πv), sin(πv)sin(2πu))
    /// where u ∈ [0,1], v ∈ [0,1]
    /// ```text
    ///
    /// ### **Tessellation Algorithm**
    /// 1. **Parameter Grid**: Create shared pole and longitude/latitude vertices
    /// 2. **Vertex Generation**: Evaluate M(u,v) at grid points
    /// 3. **Triangle Formation**: Split each interior parameter cell diagonally
    /// 4. **Degeneracy Handling**: Emit one triangle per polar cell
    ///
    /// ### **Pole Degeneracy Resolution**
    /// At poles (v=0 or v=1), the parameterization becomes singular:
    /// - **North pole** (v=0): All u values map to same point (0, r, 0)
    /// - **South pole** (v=1): All u values map to same point (0, -r, 0)
    /// - **Solution**: Reuse one exact vertex per pole across all cap triangles
    ///
    /// ### **Normal Vector Computation**
    /// Sphere normals are simply the normalized position vectors:
    /// ```text
    /// n⃗ = p⃗/|p⃗| = (x,y,z)/r
    /// ```text
    /// This is mathematically exact for spheres (no approximation needed).
    ///
    /// ### **Mesh Quality Metrics**
    /// - **Aspect Ratio**: Best when segments ≈ 2×stacks
    /// - **Area Distortion**: Minimal at equator, maximal at poles
    /// - **Angular Distortion**: Increases towards poles (unavoidable)
    ///
    /// ### **Numerical Considerations**
    /// - **Trigonometric Sampling**: Starts from canonical symbolic pi/tau and
    ///   projects the explicitly polygonal sweep once before sampling
    /// - **Pole Handling**: Avoids division by zero at singularities
    /// - **Winding Consistency**: Maintains outward-facing orientation
    ///
    /// ## **Geometric Properties**
    /// - **Surface Area**: A = 4πr²
    /// - **Volume**: V = (4/3)πr³
    /// - **Circumference** (any great circle): C = 2πr
    /// - **Curvature**: Gaussian K = 1/r², Mean H = 1/r
    ///
    /// # Parameters
    /// - `radius`: Sphere radius (> 0)
    /// - `segments`: Longitude divisions (≥ 3, recommend ≥ 8)
    /// - `stacks`: Latitude divisions (≥ 2, recommend ≥ 6)
    /// - `metadata`: Optional metadata for all faces
    ///
    /// Longitude/latitude sweeps use canonical symbolic pi/tau. The explicitly
    /// polygonal sphere projects those sweeps at its tessellation boundary,
    /// retaining exact dyadic coordinates for downstream topology.
    pub fn sphere(radius: Real, segments: usize, stacks: usize, metadata: M) -> Mesh<M> {
        if !hmesh_scalar_positive(&radius) || segments < 3 || stacks < 2 {
            return Mesh::empty();
        }

        let mut longitudes = Vec::with_capacity(segments);
        for longitude in 0..segments {
            let Some(sample) = sampled_sin_cos(longitude, segments, &Real::tau()) else {
                return Mesh::empty();
            };
            longitudes.push(sample);
        }
        let mut latitudes = Vec::with_capacity(stacks - 1);
        for latitude in 1..stacks {
            let Some(sample) = sampled_sin_cos(latitude, stacks, &Real::pi()) else {
                return Mesh::empty();
            };
            latitudes.push(sample);
        }

        let vertex =
            |sin_theta: &Real, cos_theta: &Real, sin_phi: &Real, cos_phi: &Real| -> Vertex {
                let dir = Vector3::from_xyz(
                    cos_theta.clone() * sin_phi.clone(),
                    cos_phi.clone(),
                    sin_theta.clone() * sin_phi.clone(),
                );
                Vertex::new(
                    Point3::new(
                        dir.0[0].clone() * radius.clone(),
                        dir.0[1].clone() * radius.clone(),
                        dir.0[2].clone() * radius.clone(),
                    ),
                    dir,
                )
            };

        let north = Vertex::new(
            Point3::new(Real::zero(), radius.clone(), Real::zero()),
            Vector3::y(),
        );
        let south = Vertex::new(
            Point3::new(Real::zero(), -radius.clone(), Real::zero()),
            -Vector3::y(),
        );
        let mut grid = Vec::with_capacity(segments);
        for (sin_theta, cos_theta) in &longitudes {
            let mut column = Vec::with_capacity(stacks + 1);
            column.push(north.clone());
            for (sin_phi, cos_phi) in &latitudes {
                column.push(vertex(sin_theta, cos_theta, sin_phi, cos_phi));
            }
            column.push(south.clone());
            grid.push(column);
        }

        let mut polygons = Vec::with_capacity(2 * segments * (stacks - 1));
        for longitude in 0..segments {
            let next = (longitude + 1) % segments;
            for latitude in 0..stacks {
                if latitude == 0 {
                    polygons.push(Polygon::new(
                        vec![
                            north.clone(),
                            grid[next][1].clone(),
                            grid[longitude][1].clone(),
                        ],
                        metadata.clone(),
                    ));
                } else if latitude == stacks - 1 {
                    polygons.push(Polygon::new(
                        vec![
                            grid[longitude][latitude].clone(),
                            grid[next][latitude].clone(),
                            south.clone(),
                        ],
                        metadata.clone(),
                    ));
                } else {
                    polygons.push(Polygon::new(
                        vec![
                            grid[longitude][latitude].clone(),
                            grid[next][latitude].clone(),
                            grid[next][latitude + 1].clone(),
                        ],
                        metadata.clone(),
                    ));
                    polygons.push(Polygon::new(
                        vec![
                            grid[longitude][latitude].clone(),
                            grid[next][latitude + 1].clone(),
                            grid[longitude][latitude + 1].clone(),
                        ],
                        metadata.clone(),
                    ));
                }
            }
        }
        Mesh::from_polygons(polygons)
    }

    /// Constructs a frustum between `start` and `end` with bottom radius = `radius1` and
    /// top radius = `radius2`. In the normal case, it creates side quads and cap triangles.
    /// However, if one of the radii is 0 (within tolerance), then the degenerate face is treated
    /// as a single point and the side is stitched using triangles.
    ///
    /// # Parameters
    /// - `start`: the center of the bottom face
    /// - `end`: the center of the top face
    /// - `radius1`: the radius at the bottom face
    /// - `radius2`: the radius at the top face
    /// - `segments`: number of segments around the circle (must be ≥ 3)
    /// - `metadata`: optional metadata
    ///
    /// # Example
    /// ```ignore
    /// use csgrs::mesh::Mesh;
    /// use hyperlattice::Point3;
    /// let bottom = Point3::new(0.0, 0.0, 0.0);
    /// let top = Point3::new(0.0, 0.0, 5.0);
    /// // This will create a cone (bottom degenerate) because radius1 is 0:
    /// let cone = Mesh::<()>::frustum_ptp(bottom, top, 0.0, 2.0, 32, ());
    /// ```
    pub fn frustum_ptp(
        start: Point3,
        end: Point3,
        radius1: Real,
        radius2: Real,
        segments: usize,
        metadata: M,
    ) -> Mesh<M> {
        if segments < 3
            || !hmesh_scalar_nonnegative(&radius1)
            || !hmesh_scalar_nonnegative(&radius2)
        {
            return Mesh::empty();
        }
        // Compute the axis and check that start and end do not coincide.
        //
        // The axis length and checked unit direction are evaluated through
        // `hyperlattice::Vector3`/`Real`, then exported only for the
        // finite mesh construction loops. This follows Yap's exact-geometric-
        // computation boundary discipline, *Computational Geometry* 7(1-2),
        // 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>).
        let ray = &end - &start;
        let Some(axis_length) = ray.dot(&ray).sqrt().ok() else {
            return Mesh::empty();
        };
        if !hmesh_scalar_positive(&axis_length) {
            return Mesh::empty();
        }
        let Ok(axis_z) = ray.normalize_checked() else {
            return Mesh::empty();
        };

        // Pick axes in hyperlattice so primitive floats only carry the final
        // mesh-boundary basis.
        let Ok((axis_x, axis_y)) = axis_z.orthonormal_basis_checked() else {
            return Mesh::empty();
        };

        // The cap centers for the bottom and top.
        let start_v = Vertex::new(start.clone(), -axis_z.clone());
        let end_v = Vertex::new(end.clone(), axis_z.clone());

        let ring_point = |stack: &Real, radius: &Real, sin_angle: &Real, cos_angle: &Real| {
            let radial_dir = Vector3::from_xyz(
                axis_x.0[0].clone() * cos_angle.clone()
                    + axis_y.0[0].clone() * sin_angle.clone(),
                axis_x.0[1].clone() * cos_angle.clone()
                    + axis_y.0[1].clone() * sin_angle.clone(),
                axis_x.0[2].clone() * cos_angle + axis_y.0[2].clone() * sin_angle,
            );
            let position = start.clone()
                + ray.clone() * stack.clone()
                + radial_dir.clone() * radius.clone();
            let axial_component = radius1.clone() - radius2.clone();
            let side_normal = (radial_dir * axis_length.clone()
                + axis_z.clone() * axial_component)
                .normalize_checked()
                .ok()?;
            Some((position, side_normal))
        };

        // Special-case flags for degenerate faces.
        let bottom_degenerate = !hmesh_scalar_nonzero(&radius1);
        let top_degenerate = !hmesh_scalar_nonzero(&radius2);

        // If both faces are degenerate, we cannot build a meaningful volume.
        if bottom_degenerate && top_degenerate {
            return Mesh::empty();
        }

        let mut bottom_ring = Vec::with_capacity(segments);
        let mut top_ring = Vec::with_capacity(segments);
        for i in 0..segments {
            let Some((sin, cos)) = sampled_sin_cos(i, segments, &Real::tau()) else {
                return Mesh::empty();
            };
            if !bottom_degenerate {
                let Some((position, normal)) = ring_point(&Real::zero(), &radius1, &sin, &cos)
                else {
                    return Mesh::empty();
                };
                bottom_ring.push(Vertex::new(position, normal));
            }
            if !top_degenerate {
                let Some((position, normal)) = ring_point(&Real::one(), &radius2, &sin, &cos)
                else {
                    return Mesh::empty();
                };
                top_ring.push(Vertex::new(position, normal));
            }
        }
        if !bottom_degenerate && !top_degenerate {
            let surface_id = crate::vertex::fresh_position_id();
            for (bottom, top) in bottom_ring.iter_mut().zip(&mut top_ring) {
                let line_id = crate::vertex::fresh_position_id();
                bottom.ruled_line = Some([surface_id, line_id]);
                top.ruled_line = Some([surface_id, line_id]);
            }
        }
        retain_equal_coordinate_ids(bottom_ring.iter_mut().chain(&mut top_ring));

        let mut polygons = Vec::with_capacity(3 * segments);
        let bottom_cap_plane_id = fresh_plane_id();
        let top_cap_plane_id = fresh_plane_id();

        // For each slice of the circle (0..segments)
        for i in 0..segments {
            let next = (i + 1) % segments;

            if !bottom_degenerate {
                polygons.push(
                    Polygon::new(
                        vec![
                            start_v.clone().exclude_from_hull(),
                            bottom_ring[next].clone().with_normal(-axis_z.clone()),
                            bottom_ring[i].clone().with_normal(-axis_z.clone()),
                        ],
                        metadata.clone(),
                    )
                    .with_plane_id(bottom_cap_plane_id),
                );
            }
            if !top_degenerate {
                polygons.push(
                    Polygon::new(
                        vec![
                            end_v.clone().exclude_from_hull(),
                            top_ring[i].clone().with_normal(axis_z.clone()),
                            top_ring[next].clone().with_normal(axis_z.clone()),
                        ],
                        metadata.clone(),
                    )
                    .with_plane_id(top_cap_plane_id),
                );
            }

            if bottom_degenerate {
                polygons.push(Polygon::new(
                    vec![start_v.clone(), top_ring[next].clone(), top_ring[i].clone()],
                    metadata.clone(),
                ));
            } else if top_degenerate {
                polygons.push(Polygon::new(
                    vec![
                        bottom_ring[i].clone(),
                        bottom_ring[next].clone(),
                        end_v.clone(),
                    ],
                    metadata.clone(),
                ));
            } else {
                polygons.push(Polygon::new(
                    vec![
                        bottom_ring[i].clone(),
                        bottom_ring[next].clone(),
                        top_ring[next].clone(),
                        top_ring[i].clone(),
                    ],
                    metadata.clone(),
                ));
            }
        }

        Mesh::from_polygons(polygons)
    }

    /// A helper to create a vertical cylinder along Z from z=0..z=height
    /// with the specified radius (NOT diameter).
    pub fn frustum(
        radius1: Real,
        radius2: Real,
        height: Real,
        segments: usize,
        metadata: M,
    ) -> Mesh<M> {
        Mesh::frustum_ptp(
            Point3::origin(),
            Point3::new(Real::zero(), Real::zero(), height),
            radius1,
            radius2,
            segments,
            metadata,
        )
    }

    /// A helper to create a vertical cylinder along Z from z=0..z=height
    /// with the specified radius (NOT diameter).
    pub fn cylinder(radius: Real, height: Real, segments: usize, metadata: M) -> Mesh<M> {
        Mesh::frustum_ptp(
            Point3::origin(),
            Point3::new(Real::zero(), Real::zero(), height),
            radius.clone(),
            radius,
            segments,
            metadata,
        )
    }

    /// Creates a Mesh polyhedron from raw vertex data (`points`) and face indices.
    ///
    /// Raw coordinates are API-boundary data. Each selected point is promoted
    /// through `hyperlattice::Vector3` before a polygon vertex
    /// is built, so `NaN`/infinite values are rejected instead of being
    /// sanitized by the transitional vertex carrier. This follows Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7(1-2), 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    ///
    /// # Parameters
    ///
    /// - `points`: a slice of `[x,y,z]` coordinates.
    /// - `faces`: each element is a list of indices into `points`, describing one face.
    ///   Each face must have at least 3 indices.
    ///
    /// # Example
    /// ```ignore
    /// # use csgrs::mesh::Mesh;
    ///
    /// let pts = &[
    ///     [0.0, 0.0, 0.0], // point0
    ///     [1.0, 0.0, 0.0], // point1
    ///     [1.0, 1.0, 0.0], // point2
    ///     [0.0, 1.0, 0.0], // point3
    ///     [0.5, 0.5, 1.0], // point4 - top
    /// ];
    ///
    /// // Two faces: bottom square [0,1,2,3], and a pyramid side [0,1,4]
    /// let fcs: &[&[usize]] = &[
    ///     &[0, 1, 2, 3],
    ///     &[0, 1, 4],
    ///     &[1, 2, 4],
    ///     &[2, 3, 4],
    ///     &[3, 0, 4],
    /// ];
    ///
    /// let mesh_poly = Mesh::<()>::polyhedron(pts, fcs, ());
    /// ```
    pub fn polyhedron(
        points: &[[Real; 3]],
        faces: &[&[usize]],
        metadata: M,
    ) -> Result<Mesh<M>, ValidationError> {
        let mut polygons = Vec::new();

        for face in faces {
            if face.len() < 3 {
                return Err(ValidationError::InvalidArguments);
            }
            if face
                .iter()
                .enumerate()
                .any(|(index, value)| face[index + 1..].contains(value))
            {
                return Err(ValidationError::InvalidArguments);
            }

            let mut face_vertices = Vec::with_capacity(face.len());
            for &idx in face.iter() {
                if idx >= points.len() {
                    return Err(ValidationError::IndexOutOfRangeWithLen {
                        index: idx,
                        len: points.len(),
                    });
                }
                let [x, y, z] = &points[idx];
                let point = Point3::new(x.clone(), y.clone(), z.clone());
                face_vertices.push(Vertex::new(
                    point,
                    Vector3::zeros(), // we'll set this later
                ));
            }

            let base = face_vertices[0].position.to_vector();
            let support = (1..face_vertices.len() - 1).find_map(|left| {
                (left + 1..face_vertices.len()).find_map(|right| {
                    let a = face_vertices[left].position.to_vector() - base.clone();
                    let b = face_vertices[right].position.to_vector() - base.clone();
                    let normal = a.cross(&b);
                    matches!(
                        real_cmp(&normal.dot(&normal), &Real::zero()),
                        Some(Ordering::Greater)
                    )
                    .then_some((normal, left, right))
                })
            });
            let Some((normal, support_left, support_right)) = support else {
                return Err(ValidationError::InvalidArguments);
            };
            for (index, vertex) in face_vertices.iter().enumerate() {
                if index == 0 || index == support_left || index == support_right {
                    continue;
                }
                let offset = vertex.position.to_vector() - base.clone();
                if !matches!(
                    real_cmp(&normal.dot(&offset), &Real::zero()),
                    Some(Ordering::Equal)
                ) {
                    return Err(ValidationError::InvalidArguments);
                }
            }

            let mut poly = Polygon::new(face_vertices, metadata.clone());
            poly.set_new_normal();
            polygons.push(poly);
        }

        Ok(Mesh::from_polygons(polygons))
    }

    /// Creates a 3D "egg" shape by revolving `Profile::egg()`.
    ///
    /// # Parameters
    /// - `width`: The "width" of the 2D egg outline.
    /// - `length`: The "length" (height) of the 2D egg outline.
    /// - `revolve_segments`: Number of segments for the revolution.
    /// - `outline_segments`: Number of segments for the 2D egg outline itself.
    /// - `metadata`: Optional metadata.
    #[cfg(feature = "sketch")]
    pub fn egg(
        width: Real,
        length: Real,
        revolve_segments: usize,
        outline_segments: usize,
        metadata: M,
    ) -> Self {
        if !hmesh_scalar_positive(&width)
            || !hmesh_scalar_positive(&length)
            || revolve_segments < 3
        {
            return Mesh::empty();
        }

        let egg_2d = Profile::egg(width, length, outline_segments);

        let Some(half_egg) = positive_x_half(&egg_2d) else {
            return Mesh::empty();
        };

        half_egg
            .revolve(Real::from(360_u16), revolve_segments, metadata.clone())
            .map(|mesh| mesh.convex_hull(metadata))
            .unwrap_or_else(|_| Mesh::empty())
    }

    /// Creates a 3D "teardrop" solid by revolving the existing 2D `teardrop` profile 360° around the Y-axis (via revolve).
    ///
    /// # Parameters
    /// - `width`: Width of the 2D teardrop profile.
    /// - `length`: Length of the 2D teardrop profile.
    /// - `revolve_segments`: Number of segments for the revolution (the "circular" direction).
    /// - `shape_segments`: Number of segments for the 2D teardrop outline itself.
    /// - `metadata`: Optional metadata.
    #[cfg(feature = "sketch")]
    pub fn teardrop(
        width: Real,
        length: Real,
        revolve_segments: usize,
        shape_segments: usize,
        metadata: M,
    ) -> Self {
        if !hmesh_scalar_positive(&width)
            || !hmesh_scalar_positive(&length)
            || revolve_segments < 3
        {
            return Mesh::empty();
        }

        // Make a 2D teardrop in the XY plane.
        let td_2d = Profile::teardrop(width, length, shape_segments);

        let Some(half_teardrop) = positive_x_half(&td_2d) else {
            return Mesh::empty();
        };

        // revolve 360 degrees
        half_teardrop
            .revolve(Real::from(360_u16), revolve_segments, metadata.clone())
            .map(|mesh| mesh.convex_hull(metadata))
            .unwrap_or_else(|_| Mesh::empty())
    }

    /// Creates a 3D "teardrop cylinder" by extruding the existing 2D `teardrop` in the Z+ axis.
    ///
    /// # Parameters
    /// - `width`: Width of the 2D teardrop profile.
    /// - `length`: Length of the 2D teardrop profile.
    /// - `revolve_segments`: Number of segments for the revolution (the "circular" direction).
    /// - `shape_segments`: Number of segments for the 2D teardrop outline itself.
    /// - `metadata`: Optional metadata.
    #[cfg(feature = "sketch")]
    pub fn teardrop_cylinder(
        width: Real,
        length: Real,
        height: Real,
        shape_segments: usize,
        metadata: M,
    ) -> Self {
        if !(hmesh_scalar_positive(&width)
            && hmesh_scalar_positive(&length)
            && hmesh_scalar_positive(&height))
        {
            return Mesh::empty();
        }

        // Make a 2D teardrop in the XY plane.
        let td_2d = Profile::teardrop(width, length, shape_segments);
        td_2d.extrude(height, metadata)
    }

    /// Creates an ellipsoid by taking a sphere of radius=1 and scaling it by (rx, ry, rz).
    ///
    /// # Parameters
    /// - `rx`: X-axis radius.
    /// - `ry`: Y-axis radius.
    /// - `rz`: Z-axis radius.
    /// - `segments`: Number of horizontal segments.
    /// - `stacks`: Number of vertical stacks.
    /// - `metadata`: Optional metadata.
    pub fn ellipsoid(
        rx: Real,
        ry: Real,
        rz: Real,
        segments: usize,
        stacks: usize,
        metadata: M,
    ) -> Self {
        if !(hmesh_scalar_positive(&rx)
            && hmesh_scalar_positive(&ry)
            && hmesh_scalar_positive(&rz))
        {
            return Mesh::empty();
        }

        let base_sphere = Self::sphere(Real::one(), segments, stacks, metadata.clone());
        base_sphere.scale(rx, ry, rz)
    }

    /// Creates an arrow Mesh. The arrow is composed of:
    ///   - a cylindrical shaft, and
    ///   - a cone–like head (a frustum from a larger base to a small tip)
    ///
    /// built along the canonical +Z axis. The arrow is then rotated so that +Z aligns with the given
    /// direction, and finally translated so that either its base (if `orientation` is false)
    /// or its tip (if `orientation` is true) is located at `start`.
    ///
    /// The arrow’s dimensions (shaft radius, head dimensions, etc.) are scaled proportionally to the
    /// total arrow length (the norm of the provided direction).
    ///
    /// # Parameters
    /// - `start`: the reference point (base or tip, depending on orientation)
    /// - `direction`: the vector defining arrow length and intended pointing direction
    /// - `segments`: number of segments for approximating the cylinder and frustum
    /// - `orientation`: when false (default) the arrow points away from start (its base is at start); when true the arrow points toward start (its tip is at start).
    /// - `metadata`: optional metadata for the generated polygons.
    ///
    /// Arrow proportions are derived through hyperreal scalar helpers before
    /// delegating shaft/head construction to [`Mesh::cylinder`] and
    /// [`Mesh::frustum_ptp`], following Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn arrow(
        start: Point3,
        direction: Vector3,
        segments: usize,
        orientation: bool,
        metadata: M,
    ) -> Mesh<M> {
        assembled_arrow(start, direction, segments, orientation, metadata)
    }

    /// Regular octahedron scaled by `radius`
    pub fn octahedron(radius: Real, metadata: M) -> Self {
        if !hmesh_scalar_positive(&radius) {
            return Mesh::empty();
        }
        let pts = &[
            [Real::one(), Real::zero(), Real::zero()],
            [-Real::one(), Real::zero(), Real::zero()],
            [Real::zero(), Real::one(), Real::zero()],
            [Real::zero(), -Real::one(), Real::zero()],
            [Real::zero(), Real::zero(), Real::one()],
            [Real::zero(), Real::zero(), -Real::one()],
        ];
        let faces: [&[usize]; 8] = [
            &[0, 2, 4],
            &[2, 1, 4],
            &[1, 3, 4],
            &[3, 0, 4],
            &[5, 2, 0],
            &[5, 1, 2],
            &[5, 3, 1],
            &[5, 0, 3],
        ];
        let scaled: Vec<[Real; 3]> = pts
            .iter()
            .map(|[x, y, z]| {
                [
                    x.clone() * radius.clone(),
                    y.clone() * radius.clone(),
                    z.clone() * radius.clone(),
                ]
            })
            .collect();
        Self::polyhedron(&scaled, &faces, metadata.clone()).unwrap_or_else(|_| Mesh::empty())
    }

    /// Regular icosahedron scaled by `radius`.
    ///
    /// The golden-ratio normalization is evaluated through `Real`
    /// before finite mesh vertices are emitted. This keeps even constant
    /// shape-construction algebra on the exact-aware side of the boundary,
    /// following Yap, "Towards Exact Geometric Computation," *Computational
    /// Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>). The construction is
    /// the classical regular icosahedron coordinate model using the golden
    /// ratio; see Coxeter, *Regular Polytopes*, 3rd ed., 1973.
    pub fn icosahedron(radius: Real, metadata: M) -> Self {
        if !hmesh_scalar_positive(&radius) {
            return Mesh::empty();
        }
        // golden ratio
        let Some(sqrt_five) = Real::from(5_u8).sqrt().ok() else {
            return Mesh::empty();
        };
        let Some(phi) = ((Real::one() + sqrt_five) / Real::from(2_u8)).ok() else {
            return Mesh::empty();
        };
        // normalise so the circum-radius is 1
        let phi_squared = phi.clone() * phi.clone();
        let Some(len) = (Real::one() + phi_squared).sqrt().ok() else {
            return Mesh::empty();
        };
        let Some(a) = (radius / len).ok() else {
            return Mesh::empty();
        };
        let b = phi * a.clone();

        // 12 vertices ----------------------------------------------------
        let pts: [[Real; 3]; 12] = [
            [-a.clone(), b.clone(), Real::zero()],
            [a.clone(), b.clone(), Real::zero()],
            [-a.clone(), -b.clone(), Real::zero()],
            [a.clone(), -b.clone(), Real::zero()],
            [Real::zero(), -a.clone(), b.clone()],
            [Real::zero(), a.clone(), b.clone()],
            [Real::zero(), -a.clone(), -b.clone()],
            [Real::zero(), a.clone(), -b.clone()],
            [b.clone(), Real::zero(), -a.clone()],
            [b.clone(), Real::zero(), a.clone()],
            [-b.clone(), Real::zero(), -a.clone()],
            [-b, Real::zero(), a],
        ];

        // 20 faces (counter-clockwise when viewed from outside) ----------
        let faces: [&[usize]; 20] = [
            &[0, 11, 5],
            &[0, 5, 1],
            &[0, 1, 7],
            &[0, 7, 10],
            &[0, 10, 11],
            &[1, 5, 9],
            &[5, 11, 4],
            &[11, 10, 2],
            &[10, 7, 6],
            &[7, 1, 8],
            &[3, 9, 4],
            &[3, 4, 2],
            &[3, 2, 6],
            &[3, 6, 8],
            &[3, 8, 9],
            &[4, 9, 5],
            &[2, 4, 11],
            &[6, 2, 10],
            &[8, 6, 7],
            &[9, 8, 1],
        ];

        Self::polyhedron(&pts, &faces, metadata).unwrap_or_else(|_| Mesh::empty())
    }

    /// Torus centred at the origin in the *XY* plane.
    ///
    /// * `major_r` – distance from centre to tube centre ( R )  
    /// * `minor_r` – tube radius ( r )  
    /// * `segments_major` – number of segments around the donut  
    /// * `segments_minor` – segments of the tube cross-section
    ///
    /// The torus is composed from a hypercurve-backed circular profile and
    /// revolved into mesh geometry. Radius validation is evaluated through
    /// hyperreal comparisons before crossing back to finite boundary scalars,
    /// following Yap, "Towards Exact Geometric Computation," *Computational
    /// Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    #[cfg(feature = "sketch")]
    pub fn torus(
        major_r: Real,
        minor_r: Real,
        segments_major: usize,
        segments_minor: usize,
        metadata: M,
    ) -> Self {
        if !hmesh_scalar_positive(&major_r)
            || !hmesh_scalar_positive(&minor_r)
            || !matches!(real_cmp(&major_r, &minor_r), Some(Ordering::Greater))
            || segments_major < 3
            || segments_minor < 3
        {
            return Mesh::empty();
        }

        let profile_offset = major_r;
        let circle = Profile::circle(minor_r, segments_minor).translate(
            profile_offset,
            Real::zero(),
            Real::zero(),
        );
        circle
            .revolve(Real::from(360_u16), segments_major, metadata)
            .unwrap_or_else(|_| Mesh::empty())
    }

    #[allow(clippy::too_many_arguments)]
    #[cfg(feature = "sketch")]
    pub fn spur_gear_involute(
        module: Real,
        teeth: usize,
        pressure_angle_deg: Real,
        clearance: Real,
        backlash: Real,
        segments_per_flank: usize,
        thickness: Real,
        metadata: M,
    ) -> Mesh<M> {
        if !hmesh_scalar_positive(&thickness) {
            return Mesh::empty();
        }

        Profile::involute_gear(
            module,
            teeth,
            pressure_angle_deg,
            clearance,
            backlash,
            segments_per_flank,
        )
        .extrude(thickness, metadata)
    }

    #[cfg(feature = "sketch")]
    pub fn spur_gear_cycloid(
        module: Real,
        teeth: usize,
        generating_radius: Real,
        clearance: Real,
        segments_per_flank: usize,
        thickness: Real,
        metadata: M,
    ) -> Mesh<M> {
        if !hmesh_scalar_positive(&thickness) {
            return Mesh::empty();
        }

        Profile::cycloidal_gear(
            module,
            teeth,
            generating_radius,
            clearance,
            segments_per_flank,
        )
        .extrude(thickness, metadata)
    }

    #[allow(clippy::too_many_arguments)]
    #[cfg(feature = "sketch")]
    pub fn helical_involute_gear(
        module: Real,
        teeth: usize,
        pressure_angle_deg: Real,
        clearance: Real,
        backlash: Real,
        segments_per_flank: usize,
        thickness: Real,
        helix_angle_deg: Real, // β
        slices: usize,         // ≥ 2 – axial divisions
        metadata: M,
    ) -> Mesh<M> {
        if slices < 2
            || teeth < 4
            || !hmesh_scalar_positive(&module)
            || !hmesh_scalar_positive(&thickness)
            || !matches!(
                real_cmp(&helix_angle_deg, &Real::from(-90_i8)),
                Some(Ordering::Greater)
            )
            || !matches!(
                real_cmp(&helix_angle_deg, &Real::from(90_i8)),
                Some(Ordering::Less)
            )
        {
            return Mesh::empty();
        }
        let Some(pitch_radius) = real_from_ratio(teeth as u64, 2)
            .map(|teeth_over_two| module.clone() * teeth_over_two)
        else {
            return Mesh::empty();
        };
        let helix_radians = (helix_angle_deg.clone() * Real::pi() / Real::from(180_u16)).ok();
        let Some(helix_radians) = helix_radians else {
            return Mesh::empty();
        };
        let tangent = (helix_radians.clone().sin() / helix_radians.cos()).ok();
        let Some(total_twist) =
            tangent.and_then(|tangent| (thickness.clone() * tangent / pitch_radius).ok())
        else {
            return Mesh::empty();
        };

        let base_slice = Profile::involute_gear(
            module,
            teeth,
            pressure_angle_deg,
            clearance,
            backlash,
            segments_per_flank,
        );
        if base_slice.is_empty() {
            return Mesh::empty();
        }
        twisted_profile_extrusion(&base_slice, &thickness, total_twist, slices, metadata)
    }
}
