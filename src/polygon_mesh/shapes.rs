//! Planar-faced shape constructors for [`PolygonMesh`](super::PolygonMesh).

use std::cmp::Ordering;
use std::fmt::Debug;

use hyperlattice::{Point3, Real, Vector3};

use crate::errors::ValidationError;
use crate::vertex::Vertex;
#[cfg(feature = "sketch")]
use crate::{csg::CSG, sketch::Profile};

use super::{Polygon, PolygonMesh};

const CUBOID_FACES: [[usize; 4]; 6] = [
    [0, 3, 2, 1],
    [4, 5, 6, 7],
    [0, 1, 5, 4],
    [3, 7, 6, 2],
    [0, 4, 7, 3],
    [1, 2, 6, 5],
];

const OCTAHEDRON_FACES: [[usize; 3]; 8] = [
    [0, 2, 4],
    [2, 1, 4],
    [1, 3, 4],
    [3, 0, 4],
    [5, 2, 0],
    [5, 1, 2],
    [5, 3, 1],
    [5, 0, 3],
];

const ICOSAHEDRON_FACES: [[usize; 3]; 20] = [
    [0, 11, 5],
    [0, 5, 1],
    [0, 1, 7],
    [0, 7, 10],
    [0, 10, 11],
    [1, 5, 9],
    [5, 11, 4],
    [11, 10, 2],
    [10, 7, 6],
    [7, 1, 8],
    [3, 9, 4],
    [3, 4, 2],
    [3, 2, 6],
    [3, 6, 8],
    [3, 8, 9],
    [4, 9, 5],
    [2, 4, 11],
    [6, 2, 10],
    [8, 6, 7],
    [9, 8, 1],
];

impl<M: Clone + Debug + Send + Sync> PolygonMesh<M> {
    /// Construct a six-face planar cuboid from the origin to
    /// `(width, length, height)`.
    pub fn cuboid(width: Real, length: Real, height: Real, metadata: M) -> Self {
        if ![&width, &length, &height].into_iter().all(scalar_positive) {
            return Self::empty();
        }

        let zero = Real::zero();
        let positions = [
            Point3::new(zero.clone(), zero.clone(), zero.clone()),
            Point3::new(width.clone(), zero.clone(), zero.clone()),
            Point3::new(width.clone(), length.clone(), zero.clone()),
            Point3::new(zero.clone(), length.clone(), zero),
            Point3::new(Real::zero(), Real::zero(), height.clone()),
            Point3::new(width.clone(), Real::zero(), height.clone()),
            Point3::new(width, length.clone(), height.clone()),
            Point3::new(Real::zero(), length, height),
        ];
        let normals = [
            -Vector3::z(),
            Vector3::z(),
            -Vector3::y(),
            Vector3::y(),
            -Vector3::x(),
            Vector3::x(),
        ];
        let polygons = CUBOID_FACES
            .into_iter()
            .zip(normals)
            .map(|(face, normal)| {
                Polygon::new(
                    face.into_iter()
                        .map(|index| Vertex::new(positions[index].clone(), normal.clone()))
                        .collect(),
                    metadata.clone(),
                )
            })
            .collect();
        Self::from_polygons(polygons)
    }

    /// Construct a six-face planar cube from the origin.
    pub fn cube(width: Real, metadata: M) -> Self {
        Self::cuboid(width.clone(), width.clone(), width, metadata)
    }

    /// Construct a latitude/longitude sphere with triangle pole fans and quad
    /// faces between adjacent latitude rings.
    pub fn sphere(radius: Real, segments: usize, stacks: usize, metadata: M) -> Self {
        if !scalar_positive(&radius) || segments < 3 || stacks < 2 {
            return Self::empty();
        }

        let mut rings = Vec::with_capacity(stacks - 1);
        for stack in 1..stacks {
            let Some((sin_latitude, cos_latitude)) =
                sampled_sin_cos(stack, 2 * stacks, std::f64::consts::TAU)
            else {
                return Self::empty();
            };
            let mut ring = Vec::with_capacity(segments);
            for segment in 0..segments {
                let Some((sin_longitude, cos_longitude)) =
                    sampled_sin_cos(segment, segments, std::f64::consts::TAU)
                else {
                    return Self::empty();
                };
                let normal = Vector3::from_xyz(
                    sin_latitude.clone() * cos_longitude,
                    sin_latitude.clone() * sin_longitude,
                    cos_latitude.clone(),
                );
                ring.push(Vertex::new(
                    Point3::from(normal.clone() * radius.clone()),
                    normal,
                ));
            }
            rings.push(ring);
        }

        let north = Vertex::new(
            Point3::new(Real::zero(), Real::zero(), radius.clone()),
            Vector3::z(),
        );
        let south = Vertex::new(
            Point3::new(Real::zero(), Real::zero(), -radius),
            -Vector3::z(),
        );
        let mut polygons = Vec::with_capacity(segments * stacks);
        for segment in 0..segments {
            let next = (segment + 1) % segments;
            polygons.push(Polygon::new(
                vec![
                    north.clone(),
                    rings[0][segment].clone(),
                    rings[0][next].clone(),
                ],
                metadata.clone(),
            ));
        }
        for pair in rings.windows(2) {
            for segment in 0..segments {
                let next = (segment + 1) % segments;
                polygons.push(Polygon::new(
                    vec![
                        pair[0][segment].clone(),
                        pair[1][segment].clone(),
                        pair[1][next].clone(),
                        pair[0][next].clone(),
                    ],
                    metadata.clone(),
                ));
            }
        }
        let last = rings.last().expect("a valid sphere has a latitude ring");
        for segment in 0..segments {
            let next = (segment + 1) % segments;
            polygons.push(Polygon::new(
                vec![south.clone(), last[next].clone(), last[segment].clone()],
                metadata.clone(),
            ));
        }
        Self::from_polygons(polygons)
    }

    /// Construct a capped frustum between two points. Nondegenerate side
    /// patches remain quads; a cone apex produces triangles.
    pub fn frustum_ptp(
        start: Point3,
        end: Point3,
        radius1: Real,
        radius2: Real,
        segments: usize,
        metadata: M,
    ) -> Self {
        if segments < 3 || !scalar_nonnegative(&radius1) || !scalar_nonnegative(&radius2) {
            return Self::empty();
        }
        let ray = &end - &start;
        let Ok(axis_z) = ray.normalize_checked() else {
            return Self::empty();
        };
        let Ok(axis_length) = ray.dot(&ray).sqrt() else {
            return Self::empty();
        };
        let Ok((axis_x, axis_y)) = axis_z.orthonormal_basis_checked() else {
            return Self::empty();
        };
        let bottom_degenerate = !scalar_nonzero(&radius1);
        let top_degenerate = !scalar_nonzero(&radius2);
        if bottom_degenerate && top_degenerate {
            return Self::empty();
        }

        let mut bottom_ring = Vec::with_capacity(segments);
        let mut top_ring = Vec::with_capacity(segments);
        for segment in 0..segments {
            let Some((sin, cos)) = sampled_sin_cos(segment, segments, std::f64::consts::TAU)
            else {
                return Self::empty();
            };
            let radial = axis_x.clone() * cos + axis_y.clone() * sin;
            let Ok(side_normal) = (radial.clone() * axis_length.clone()
                + axis_z.clone() * (radius1.clone() - radius2.clone()))
            .normalize_checked() else {
                return Self::empty();
            };
            if !bottom_degenerate {
                bottom_ring.push(Vertex::new(
                    start.clone() + radial.clone() * radius1.clone(),
                    side_normal.clone(),
                ));
            }
            if !top_degenerate {
                top_ring.push(Vertex::new(
                    end.clone() + radial * radius2.clone(),
                    side_normal,
                ));
            }
        }

        let bottom_center = Vertex::new(start.clone(), -axis_z.clone());
        let top_center = Vertex::new(end.clone(), axis_z.clone());
        let mut polygons = Vec::with_capacity(3 * segments);
        for segment in 0..segments {
            let next = (segment + 1) % segments;
            if !bottom_degenerate {
                polygons.push(Polygon::new(
                    vec![
                        bottom_center.clone(),
                        bottom_ring[next].clone().with_normal(-axis_z.clone()),
                        bottom_ring[segment].clone().with_normal(-axis_z.clone()),
                    ],
                    metadata.clone(),
                ));
            }
            if !top_degenerate {
                polygons.push(Polygon::new(
                    vec![
                        top_center.clone(),
                        top_ring[segment].clone().with_normal(axis_z.clone()),
                        top_ring[next].clone().with_normal(axis_z.clone()),
                    ],
                    metadata.clone(),
                ));
            }
            let side = if bottom_degenerate {
                vec![
                    Vertex::new(start.clone(), -axis_z.clone()),
                    top_ring[next].clone(),
                    top_ring[segment].clone(),
                ]
            } else if top_degenerate {
                vec![
                    bottom_ring[segment].clone(),
                    bottom_ring[next].clone(),
                    Vertex::new(end.clone(), axis_z.clone()),
                ]
            } else {
                vec![
                    bottom_ring[segment].clone(),
                    bottom_ring[next].clone(),
                    top_ring[next].clone(),
                    top_ring[segment].clone(),
                ]
            };
            polygons.push(Polygon::new(side, metadata.clone()));
        }
        Self::from_polygons(polygons)
    }

    /// Construct a vertical frustum along +Z.
    pub fn frustum(
        radius1: Real,
        radius2: Real,
        height: Real,
        segments: usize,
        metadata: M,
    ) -> Self {
        Self::frustum_ptp(
            Point3::origin(),
            Point3::new(Real::zero(), Real::zero(), height),
            radius1,
            radius2,
            segments,
            metadata,
        )
    }

    /// Construct a vertical cylinder along +Z.
    pub fn cylinder(radius: Real, height: Real, segments: usize, metadata: M) -> Self {
        if !scalar_positive(&radius) || !scalar_positive(&height) || segments < 3 {
            return Self::empty();
        }
        Self::frustum(radius.clone(), radius, height, segments, metadata)
    }

    /// Construct a planar-faced polyhedron while retaining each supplied face.
    pub fn polyhedron(
        points: &[[Real; 3]],
        faces: &[&[usize]],
        metadata: M,
    ) -> Result<Self, ValidationError> {
        let mut polygons = Vec::with_capacity(faces.len());
        for face in faces {
            if face.len() < 3
                || face
                    .iter()
                    .enumerate()
                    .any(|(offset, index)| face[offset + 1..].contains(index))
            {
                return Err(ValidationError::InvalidArguments);
            }
            let mut vertices = Vec::with_capacity(face.len());
            for &index in *face {
                let Some([x, y, z]) = points.get(index) else {
                    return Err(ValidationError::IndexOutOfRangeWithLen {
                        index,
                        len: points.len(),
                    });
                };
                vertices.push(Vertex::new(
                    Point3::new(x.clone(), y.clone(), z.clone()),
                    Vector3::zeros(),
                ));
            }
            let base = vertices[0].position.to_vector();
            let support = (1..vertices.len() - 1).find_map(|left| {
                (left + 1..vertices.len()).find_map(|right| {
                    let normal = (vertices[left].position.to_vector() - base.clone())
                        .cross(&(vertices[right].position.to_vector() - base.clone()));
                    scalar_positive(&normal.dot(&normal)).then_some((normal, left, right))
                })
            });
            let Some((normal, left, right)) = support else {
                return Err(ValidationError::InvalidArguments);
            };
            if vertices.iter().enumerate().any(|(index, vertex)| {
                index != 0
                    && index != left
                    && index != right
                    && !scalar_zero(&normal.dot(&(vertex.position.to_vector() - base.clone())))
            }) {
                return Err(ValidationError::InvalidArguments);
            }
            let mut polygon = Polygon::new(vertices, metadata.clone());
            polygon.set_new_normal();
            polygons.push(polygon);
        }
        Ok(Self::from_polygons(polygons))
    }

    /// Construct an ellipsoid with triangle pole fans and quad latitude bands.
    pub fn ellipsoid(
        rx: Real,
        ry: Real,
        rz: Real,
        segments: usize,
        stacks: usize,
        metadata: M,
    ) -> Self {
        if !scalar_positive(&rx)
            || !scalar_positive(&ry)
            || !scalar_positive(&rz)
            || segments < 3
            || stacks < 2
        {
            return Self::empty();
        }
        let sphere = Self::sphere(Real::one(), segments, stacks, metadata);
        let mut polygons = sphere.into_polygons();
        for polygon in &mut polygons {
            for vertex in polygon.vertices_mut().iter_mut() {
                vertex.position.x *= rx.clone();
                vertex.position.y *= ry.clone();
                vertex.position.z *= rz.clone();
                let (Ok(nx), Ok(ny), Ok(nz)) = (
                    vertex.normal.0[0].clone() / rx.clone(),
                    vertex.normal.0[1].clone() / ry.clone(),
                    vertex.normal.0[2].clone() / rz.clone(),
                ) else {
                    return Self::empty();
                };
                let Ok(normal) = Vector3::from_xyz(nx, ny, nz).normalize_checked() else {
                    return Self::empty();
                };
                vertex.normal = normal;
            }
        }
        Self::from_polygons(polygons)
    }

    /// Construct an arrow aligned with `direction` and anchored at `start`.
    pub fn arrow(
        start: Point3,
        direction: Vector3,
        segments: usize,
        orientation: bool,
        metadata: M,
    ) -> Self {
        if segments < 3 {
            return Self::empty();
        }
        let Ok(length) = direction.dot(&direction).sqrt() else {
            return Self::empty();
        };
        if !scalar_positive(&length) {
            return Self::empty();
        }
        let Ok(axis) = direction.normalize_checked() else {
            return Self::empty();
        };
        let Ok((axis_x, axis_y)) = axis.orthonormal_basis_checked() else {
            return Self::empty();
        };
        let shaft_length = length.clone() * ratio(4, 5);
        let head_length = length.clone() - shaft_length.clone();
        let shaft_radius = length.clone() * ratio(3, 100);
        let head_radius = length * ratio(3, 50);
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
        let shaft = Self::frustum_ptp(
            shaft_start,
            shaft_end,
            shaft_radius.clone(),
            shaft_radius.clone(),
            segments,
            metadata.clone(),
        );
        let head = Self::frustum_ptp(
            head_start,
            head_end,
            head_r1,
            head_r2,
            segments,
            metadata.clone(),
        );
        let mut polygons = shaft
            .into_polygons()
            .into_iter()
            .enumerate()
            .filter(|(index, _)| {
                if orientation {
                    index % 3 != 0
                } else {
                    index % 3 != 1
                }
            })
            .map(|(_, polygon)| polygon)
            .chain(
                head.into_polygons()
                    .into_iter()
                    .enumerate()
                    .filter(|(index, _)| index % 2 == 1)
                    .map(|(_, polygon)| polygon),
            )
            .collect::<Vec<_>>();

        let shoulder_normal = if orientation { axis.clone() } else { -axis };
        for segment in 0..segments {
            let next = (segment + 1) % segments;
            let Some((sin0, cos0)) = sampled_sin_cos(segment, segments, std::f64::consts::TAU)
            else {
                return Self::empty();
            };
            let Some((sin1, cos1)) = sampled_sin_cos(next, segments, std::f64::consts::TAU)
            else {
                return Self::empty();
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
            polygons.push(Polygon::new(
                if orientation {
                    vec![inner0, outer0, outer1, inner1]
                } else {
                    vec![inner0, inner1, outer1, outer0]
                },
                metadata.clone(),
            ));
        }
        Self::from_polygons(polygons)
    }

    /// Construct a regular octahedron centered at the origin.
    pub fn octahedron(radius: Real, metadata: M) -> Self {
        if !scalar_positive(&radius) {
            return Self::empty();
        }
        let negative = -radius.clone();
        let points = [
            Point3::new(radius.clone(), Real::zero(), Real::zero()),
            Point3::new(negative.clone(), Real::zero(), Real::zero()),
            Point3::new(Real::zero(), radius.clone(), Real::zero()),
            Point3::new(Real::zero(), negative, Real::zero()),
            Point3::new(Real::zero(), Real::zero(), radius.clone()),
            Point3::new(Real::zero(), Real::zero(), -radius),
        ];
        Self::from_polygons(
            OCTAHEDRON_FACES
                .into_iter()
                .map(|face| flat_polygon(&points, face, metadata.clone()))
                .collect(),
        )
    }

    /// Construct a regular icosahedron centered at the origin.
    pub fn icosahedron(radius: Real, metadata: M) -> Self {
        if !scalar_positive(&radius) {
            return Self::empty();
        }
        let phi = (1.0_f64 + 5.0_f64.sqrt()) * 0.5;
        let inverse_length = 1.0_f64 / (1.0_f64 + phi * phi).sqrt();
        let a = radius.clone()
            * Real::try_from(inverse_length).expect("finite icosahedron coordinate");
        let b = radius * Real::try_from(phi * inverse_length).expect("finite coordinate");
        let points = [
            Point3::new(-a.clone(), b.clone(), Real::zero()),
            Point3::new(a.clone(), b.clone(), Real::zero()),
            Point3::new(-a.clone(), -b.clone(), Real::zero()),
            Point3::new(a.clone(), -b.clone(), Real::zero()),
            Point3::new(Real::zero(), -a.clone(), b.clone()),
            Point3::new(Real::zero(), a.clone(), b.clone()),
            Point3::new(Real::zero(), -a.clone(), -b.clone()),
            Point3::new(Real::zero(), a.clone(), -b.clone()),
            Point3::new(b.clone(), Real::zero(), -a.clone()),
            Point3::new(b.clone(), Real::zero(), a.clone()),
            Point3::new(-b.clone(), Real::zero(), -a.clone()),
            Point3::new(-b, Real::zero(), a),
        ];
        Self::from_polygons(
            ICOSAHEDRON_FACES
                .into_iter()
                .map(|face| flat_polygon(&points, face, metadata.clone()))
                .collect(),
        )
    }

    /// Construct a torus in the XY plane, retaining one quad per sampled cell.
    pub fn torus(
        major_r: Real,
        minor_r: Real,
        segments_major: usize,
        segments_minor: usize,
        metadata: M,
    ) -> Self {
        if !scalar_positive(&major_r)
            || !scalar_positive(&minor_r)
            || !matches!(scalar_cmp(&major_r, &minor_r), Some(Ordering::Greater))
            || segments_major < 3
            || segments_minor < 3
        {
            return Self::empty();
        }
        let mut grid = Vec::with_capacity(segments_major * segments_minor);
        for major in 0..segments_major {
            let Some((sin_major, cos_major)) =
                sampled_sin_cos(major, segments_major, std::f64::consts::TAU)
            else {
                return Self::empty();
            };
            for minor in 0..segments_minor {
                let Some((sin_minor, cos_minor)) =
                    sampled_sin_cos(minor, segments_minor, std::f64::consts::TAU)
                else {
                    return Self::empty();
                };
                let radial = major_r.clone() + minor_r.clone() * cos_minor.clone();
                let normal = Vector3::from_xyz(
                    cos_major.clone() * cos_minor.clone(),
                    sin_major.clone() * cos_minor.clone(),
                    sin_minor.clone(),
                );
                grid.push(Vertex::new(
                    Point3::new(
                        radial.clone() * cos_major.clone(),
                        radial * sin_major.clone(),
                        minor_r.clone() * sin_minor,
                    ),
                    normal,
                ));
            }
        }
        let index = |major: usize, minor: usize| major * segments_minor + minor;
        let mut polygons = Vec::with_capacity(segments_major * segments_minor);
        for major in 0..segments_major {
            let next_major = (major + 1) % segments_major;
            for minor in 0..segments_minor {
                let next_minor = (minor + 1) % segments_minor;
                polygons.push(Polygon::new(
                    vec![
                        grid[index(major, minor)].clone(),
                        grid[index(next_major, minor)].clone(),
                        grid[index(next_major, next_minor)].clone(),
                        grid[index(major, next_minor)].clone(),
                    ],
                    metadata.clone(),
                ));
            }
        }
        Self::from_polygons(polygons)
    }

    /// Construct an egg by revolving the positive-X half of its 2D profile.
    #[cfg(feature = "sketch")]
    pub fn egg(
        width: Real,
        length: Real,
        revolve_segments: usize,
        outline_segments: usize,
        metadata: M,
    ) -> Self {
        if !scalar_positive(&width) || !scalar_positive(&length) || revolve_segments < 3 {
            return Self::empty();
        }
        let Some(profile) = positive_x_half(&Profile::egg(width, length, outline_segments))
        else {
            return Self::empty();
        };
        profile
            .revolve_polygon_mesh(Real::from(360_u16), revolve_segments, metadata)
            .unwrap_or_else(|_| Self::empty())
    }

    /// Construct a teardrop by revolving the positive-X half of its profile.
    #[cfg(feature = "sketch")]
    pub fn teardrop(
        width: Real,
        length: Real,
        revolve_segments: usize,
        shape_segments: usize,
        metadata: M,
    ) -> Self {
        if !scalar_positive(&width) || !scalar_positive(&length) || revolve_segments < 3 {
            return Self::empty();
        }
        let Some(profile) = positive_x_half(&Profile::teardrop(width, length, shape_segments))
        else {
            return Self::empty();
        };
        profile
            .revolve_polygon_mesh(Real::from(360_u16), revolve_segments, metadata)
            .unwrap_or_else(|_| Self::empty())
    }

    /// Construct a teardrop prism by direct polygon extrusion.
    #[cfg(feature = "sketch")]
    pub fn teardrop_cylinder(
        width: Real,
        length: Real,
        height: Real,
        shape_segments: usize,
        metadata: M,
    ) -> Self {
        if !scalar_positive(&width) || !scalar_positive(&length) || !scalar_positive(&height) {
            return Self::empty();
        }
        Profile::teardrop(width, length, shape_segments).extrude_polygon_mesh(height, metadata)
    }

    /// Construct a spur gear from an involute 2D tooth profile.
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
    ) -> Self {
        if !scalar_positive(&thickness) {
            return Self::empty();
        }
        Profile::involute_gear(
            module,
            teeth,
            pressure_angle_deg,
            clearance,
            backlash,
            segments_per_flank,
        )
        .extrude_polygon_mesh(thickness, metadata)
    }

    /// Construct a spur gear from a cycloidal 2D tooth profile.
    #[cfg(feature = "sketch")]
    pub fn spur_gear_cycloid(
        module: Real,
        teeth: usize,
        generating_radius: Real,
        clearance: Real,
        segments_per_flank: usize,
        thickness: Real,
        metadata: M,
    ) -> Self {
        if !scalar_positive(&thickness) {
            return Self::empty();
        }
        Profile::cycloidal_gear(
            module,
            teeth,
            generating_radius,
            clearance,
            segments_per_flank,
        )
        .extrude_polygon_mesh(thickness, metadata)
    }

    /// Construct a helical involute gear using direct twisted extrusion.
    #[cfg(feature = "sketch")]
    pub fn helical_involute_gear(
        module: Real,
        teeth: usize,
        pressure_angle_deg: Real,
        clearance: Real,
        backlash: Real,
        segments_per_flank: usize,
        thickness: Real,
        helix_angle_deg: Real,
        slices: usize,
        metadata: M,
    ) -> Self {
        if slices < 2
            || teeth < 4
            || !scalar_positive(&module)
            || !scalar_positive(&thickness)
            || !matches!(
                scalar_cmp(&helix_angle_deg, &Real::from(-90_i8)),
                Some(Ordering::Greater)
            )
            || !matches!(
                scalar_cmp(&helix_angle_deg, &Real::from(90_i8)),
                Some(Ordering::Less)
            )
        {
            return Self::empty();
        }
        let pitch_radius = module.clone() * ratio(teeth as u64, 2);
        let Some(helix_radians) = (helix_angle_deg * Real::pi() / Real::from(180_u16)).ok()
        else {
            return Self::empty();
        };
        let Some(total_twist_radians) = (helix_radians.clone().sin() / helix_radians.cos())
            .ok()
            .and_then(|tangent| (thickness.clone() * tangent / pitch_radius).ok())
        else {
            return Self::empty();
        };
        let Some(total_twist_degrees) =
            (total_twist_radians * Real::from(180_u16) / Real::pi()).ok()
        else {
            return Self::empty();
        };
        Profile::involute_gear(
            module,
            teeth,
            pressure_angle_deg,
            clearance,
            backlash,
            segments_per_flank,
        )
        .extrude_twisted_polygon_mesh(
            thickness,
            total_twist_degrees,
            [Real::one(), Real::one()],
            slices,
            metadata,
        )
        .unwrap_or_else(|_| Self::empty())
    }
}

fn flat_polygon<M: Clone + Send + Sync>(
    points: &[Point3],
    face: [usize; 3],
    metadata: M,
) -> Polygon<M> {
    let mut polygon = Polygon::new(
        face.into_iter()
            .map(|index| Vertex::new(points[index].clone(), Vector3::zeros()))
            .collect(),
        metadata,
    );
    polygon.set_new_normal();
    polygon
}

fn sampled_sin_cos(index: usize, count: usize, sweep: f64) -> Option<(Real, Real)> {
    let angle = sweep * index as f64 / count as f64;
    let (sin, cos) = angle.sin_cos();
    Some((Real::try_from(sin).ok()?, Real::try_from(cos).ok()?))
}

fn ratio(numerator: u64, denominator: u64) -> Real {
    (Real::from(numerator) / Real::from(denominator)).expect("nonzero shape ratio denominator")
}

#[cfg(feature = "sketch")]
fn positive_x_half(profile: &Profile) -> Option<Profile> {
    let (min_x, min_y, max_x, max_y) = profile.native_xy_bounds()?;
    match scalar_cmp(&min_x, &Real::zero())? {
        Ordering::Equal | Ordering::Greater => return Some(profile.clone()),
        Ordering::Less => {},
    }
    if !matches!(scalar_cmp(&max_x, &Real::zero()), Some(Ordering::Greater)) {
        return None;
    }
    let width = -min_x.clone();
    let height = max_y - min_y.clone();
    if !scalar_positive(&width) || !scalar_positive(&height) {
        return None;
    }
    let cutter = Profile::rectangle(width, height).translate(min_x, min_y, Real::zero());
    profile.try_difference(&cutter).ok()
}

fn scalar_cmp(left: &Real, right: &Real) -> Option<Ordering> {
    if let (Some(left), Some(right)) = (left.exact_rational_ref(), right.exact_rational_ref())
    {
        return left.partial_cmp(right);
    }
    hyperlimit::compare_reals(left, right).value()
}

fn scalar_positive(value: &Real) -> bool {
    if let Some(value) = value.exact_rational_ref() {
        return value.is_positive();
    }
    matches!(
        hyperlimit::compare_reals(value, &Real::zero()).value(),
        Some(Ordering::Greater)
    )
}

fn scalar_nonnegative(value: &Real) -> bool {
    matches!(
        scalar_cmp(value, &Real::zero()),
        Some(Ordering::Equal | Ordering::Greater)
    )
}

fn scalar_nonzero(value: &Real) -> bool {
    matches!(
        scalar_cmp(value, &Real::zero()),
        Some(Ordering::Less | Ordering::Greater)
    )
}

fn scalar_zero(value: &Real) -> bool {
    matches!(scalar_cmp(value, &Real::zero()), Some(Ordering::Equal))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn polygon_cuboid_retains_six_quads_and_converts_to_twelve_triangles() {
        let polygon_mesh =
            PolygonMesh::cuboid(Real::from(2), Real::from(3), Real::from(5), "cuboid");

        assert_eq!(polygon_mesh.polygons.len(), 6);
        assert!(
            polygon_mesh
                .polygons
                .iter()
                .all(|polygon| polygon.vertices().len() == 4)
        );

        let mesh = polygon_mesh.triangulate();
        assert_eq!(mesh.polygons.len(), 12);
        assert!(mesh.polygons.iter().all(|triangle| {
            triangle.vertices().len() == 3 && triangle.metadata() == &"cuboid"
        }));
    }

    #[test]
    fn polygon_cuboid_rejects_nonpositive_dimensions() {
        assert!(
            PolygonMesh::cuboid(Real::zero(), Real::from(3), Real::from(5), ())
                .polygons
                .is_empty()
        );
        assert!(PolygonMesh::cube(Real::from(-1), ()).polygons.is_empty());
    }

    #[test]
    fn polygon_sphere_retains_quad_latitude_bands() {
        let sphere = PolygonMesh::sphere(Real::from(2), 8, 4, "sphere");

        assert_eq!(sphere.polygons.len(), 32);
        assert_eq!(
            sphere
                .polygons
                .iter()
                .filter(|polygon| polygon.vertices().len() == 3)
                .count(),
            16
        );
        assert_eq!(
            sphere
                .polygons
                .iter()
                .filter(|polygon| polygon.vertices().len() == 4)
                .count(),
            16
        );
        let mesh = sphere.triangulate();
        assert_eq!(mesh.polygons.len(), 48);
        assert!(mesh.is_manifold());
        assert!(
            mesh.polygons
                .iter()
                .all(|triangle| triangle.metadata() == &"sphere")
        );
    }

    #[test]
    fn polygon_frustum_retains_side_quads_and_triangulates_to_a_manifold() {
        let frustum = PolygonMesh::frustum(Real::from(3), Real::from(1), Real::from(5), 8, ());

        assert_eq!(frustum.polygons.len(), 24);
        assert_eq!(
            frustum
                .polygons
                .iter()
                .filter(|polygon| polygon.vertices().len() == 4)
                .count(),
            8
        );
        let mesh = frustum.triangulate();
        assert_eq!(mesh.polygons.len(), 32);
        assert!(mesh.is_manifold());
    }

    #[test]
    fn polygon_cone_uses_apex_triangles() {
        let cone = PolygonMesh::frustum_ptp(
            Point3::origin(),
            Point3::new(Real::zero(), Real::zero(), Real::from(3)),
            Real::zero(),
            Real::from(2),
            7,
            (),
        );

        assert_eq!(cone.polygons.len(), 14);
        assert!(
            cone.polygons
                .iter()
                .all(|polygon| polygon.vertices().len() == 3)
        );
        assert!(cone.triangulate().is_manifold());
    }

    #[test]
    fn polygon_polyhedron_retains_supplied_faces() {
        let points = [
            [Real::zero(), Real::zero(), Real::zero()],
            [Real::from(2), Real::zero(), Real::zero()],
            [Real::from(2), Real::from(2), Real::zero()],
            [Real::zero(), Real::from(2), Real::zero()],
            [Real::zero(), Real::zero(), Real::from(2)],
            [Real::from(2), Real::zero(), Real::from(2)],
            [Real::from(2), Real::from(2), Real::from(2)],
            [Real::zero(), Real::from(2), Real::from(2)],
        ];
        let faces: &[&[usize]] = &[
            &[0, 3, 2, 1],
            &[4, 5, 6, 7],
            &[0, 1, 5, 4],
            &[3, 7, 6, 2],
            &[0, 4, 7, 3],
            &[1, 2, 6, 5],
        ];

        let polyhedron = PolygonMesh::polyhedron(&points, faces, 9).expect("valid cube");

        assert_eq!(polyhedron.polygons.len(), 6);
        assert!(
            polyhedron
                .polygons
                .iter()
                .all(|polygon| polygon.vertices().len() == 4 && polygon.metadata() == &9)
        );
        let mesh = polyhedron.triangulate();
        assert_eq!(mesh.polygons.len(), 12);
        assert!(mesh.is_manifold());
    }

    #[test]
    fn polygon_polyhedron_rejects_nonplanar_face() {
        let points = [
            [Real::zero(), Real::zero(), Real::zero()],
            [Real::one(), Real::zero(), Real::zero()],
            [Real::one(), Real::one(), Real::zero()],
            [Real::zero(), Real::one(), Real::one()],
        ];

        assert_eq!(
            PolygonMesh::polyhedron(&points, &[&[0, 1, 2, 3]], ()),
            Err(ValidationError::InvalidArguments)
        );
    }

    #[test]
    fn polygon_ellipsoid_retains_sphere_face_layout() {
        let ellipsoid =
            PolygonMesh::ellipsoid(Real::from(2), Real::from(3), Real::from(5), 6, 3, ());

        assert_eq!(ellipsoid.polygons.len(), 18);
        assert_eq!(
            ellipsoid
                .polygons
                .iter()
                .filter(|polygon| polygon.vertices().len() == 4)
                .count(),
            6
        );
        assert!(ellipsoid.triangulate().is_manifold());
    }

    #[test]
    fn polygon_platonic_solids_convert_without_changing_face_counts() {
        let octahedron = PolygonMesh::octahedron(Real::from(2), ());
        let icosahedron = PolygonMesh::icosahedron(Real::from(2), ());

        assert_eq!(octahedron.polygons.len(), 8);
        assert_eq!(octahedron.triangulate().polygons.len(), 8);
        assert!(octahedron.triangulate().is_manifold());
        assert_eq!(icosahedron.polygons.len(), 20);
        assert_eq!(icosahedron.triangulate().polygons.len(), 20);
        assert!(icosahedron.triangulate().is_manifold());
    }

    #[test]
    fn polygon_torus_retains_one_quad_per_parameter_cell() {
        let torus = PolygonMesh::torus(Real::from(4), Real::one(), 8, 6, "torus");

        assert_eq!(torus.polygons.len(), 48);
        assert!(torus.polygons.iter().all(|polygon| {
            polygon.vertices().len() == 4 && polygon.metadata() == &"torus"
        }));
        let mesh = torus.triangulate();
        assert_eq!(mesh.polygons.len(), 96);
        assert!(mesh.is_manifold());
    }

    #[test]
    fn curved_polygon_shapes_reject_invalid_tessellation_or_dimensions() {
        assert!(PolygonMesh::sphere(Real::one(), 2, 3, ()).polygons.is_empty());
        assert!(
            PolygonMesh::cylinder(Real::one(), Real::zero(), 8, ())
                .polygons
                .is_empty()
        );
        assert!(
            PolygonMesh::torus(Real::one(), Real::one(), 8, 8, ())
                .polygons
                .is_empty()
        );
    }

    #[test]
    fn polygon_arrow_is_closed_and_retains_ruled_quads() {
        let arrow = PolygonMesh::arrow(
            Point3::origin(),
            Vector3::from_xyz(Real::zero(), Real::zero(), Real::from(10)),
            8,
            false,
            "arrow",
        );

        assert_eq!(arrow.polygons.len(), 32);
        assert_eq!(
            arrow
                .polygons
                .iter()
                .filter(|polygon| polygon.vertices().len() == 4)
                .count(),
            16
        );
        assert!(arrow.triangulate().is_manifold());
    }

    #[cfg(feature = "sketch")]
    #[test]
    fn profile_based_polygon_solids_are_built_without_mesh_round_trips() {
        let egg = PolygonMesh::egg(Real::from(3), Real::from(5), 8, 12, ());
        let teardrop = PolygonMesh::teardrop(Real::from(3), Real::from(5), 8, 12, ());
        let prism = PolygonMesh::teardrop_cylinder(
            Real::from(3),
            Real::from(5),
            Real::from(2),
            12,
            (),
        );

        assert!(!egg.polygons.is_empty());
        assert!(!teardrop.polygons.is_empty());
        assert!(!prism.polygons.is_empty());
        assert!(egg.triangulate().is_manifold());
        assert!(teardrop.triangulate().is_manifold());
        assert!(prism.triangulate().is_manifold());
        assert!(
            prism
                .polygons
                .iter()
                .any(|polygon| polygon.vertices().len() == 4)
        );
    }

    #[cfg(feature = "sketch")]
    #[test]
    fn polygon_gear_constructors_use_direct_extrusion_paths() {
        let involute = PolygonMesh::spur_gear_involute(
            Real::from(2),
            8,
            Real::from(20),
            Real::zero(),
            Real::zero(),
            3,
            Real::from(2),
            (),
        );
        let cycloid = PolygonMesh::spur_gear_cycloid(
            Real::from(2),
            8,
            Real::one(),
            Real::zero(),
            3,
            Real::from(2),
            (),
        );
        let helical = PolygonMesh::helical_involute_gear(
            Real::from(2),
            8,
            Real::from(20),
            Real::zero(),
            Real::zero(),
            3,
            Real::from(2),
            Real::from(15),
            2,
            (),
        );

        assert!(!involute.polygons.is_empty());
        assert!(!cycloid.polygons.is_empty());
        assert!(!helical.polygons.is_empty());
        assert!(
            involute
                .polygons
                .iter()
                .any(|polygon| polygon.vertices().len() == 4)
        );
        assert!(
            cycloid
                .polygons
                .iter()
                .any(|polygon| polygon.vertices().len() == 4)
        );
        assert!(
            helical
                .polygons
                .iter()
                .all(|polygon| polygon.vertices().len() == 3)
        );
    }
}
