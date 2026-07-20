//! Planar n-gon face storage and derived geometry.

use crate::mesh::plane::Plane;
use crate::mesh::{
    CUBOID_CORNER_POSITION_SLOTS, CUBOID_POINT_COORDINATE_SLOTS,
    CUBOID_POSITION_REPRESENTATIVES, OCTAHEDRON_FACES,
};
use crate::vertex::Vertex;
use hashbrown::HashMap;
use hyperlattice::{Aabb, Matrix4, Point3, Real, Vector3};
use hyperreal::Rational;
use std::cell::RefCell;
use std::ops::Range;
use std::ops::{Deref, DerefMut};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, LazyLock, OnceLock};

static NEXT_PLANE_ID: AtomicU64 = AtomicU64::new(1);

#[derive(Clone, Debug)]
struct CachedPolygonNormal {
    vertex_ids: Vec<u64>,
    normal: Vector3,
}

thread_local! {
    static POLYGON_NORMALS: RefCell<HashMap<u64, CachedPolygonNormal>> =
        RefCell::new(HashMap::new());
}

const POLYGON_NORMAL_CACHE_CAPACITY: usize = 8_192;
static EMPTY_VERTEX_BUFFER: LazyLock<Arc<Vec<Vertex>>> =
    LazyLock::new(|| Arc::new(Vec::new()));
static OCTAHEDRON_NORMALS: LazyLock<[Vector3; 8]> = LazyLock::new(|| {
    let component =
        Real::try_from(1.0_f64 / 3.0_f64.sqrt()).expect("finite octahedron normal");
    let negative = -component.clone();
    [
        Vector3::from_xyz(component.clone(), component.clone(), component.clone()),
        Vector3::from_xyz(negative.clone(), component.clone(), component.clone()),
        Vector3::from_xyz(negative.clone(), negative.clone(), component.clone()),
        Vector3::from_xyz(component.clone(), negative.clone(), component.clone()),
        Vector3::from_xyz(component.clone(), component.clone(), negative.clone()),
        Vector3::from_xyz(negative.clone(), component.clone(), negative.clone()),
        Vector3::from_xyz(negative.clone(), negative.clone(), negative.clone()),
        Vector3::from_xyz(component, negative.clone(), negative),
    ]
});

#[derive(Debug)]
enum LazyMappedTransform {
    Translate(Vector3),
    AxisReflection {
        axis: usize,
        value: Real,
    },
    Scale {
        scales: [Real; 3],
        inverse_scales: [Real; 3],
    },
    Affine {
        matrix: Box<Matrix4>,
        normal_matrix: Box<Matrix4>,
        normalize_normals: bool,
        flip_normals: bool,
    },
    Invert,
}

#[derive(Clone, Copy, Debug)]
struct LazyMappedIdentities {
    first_position_id: u64,
    first_coordinate_ids: [u64; 3],
}

#[derive(Debug)]
enum LazySourceVertices {
    Materialized(Arc<Vec<Vertex>>),
    Cuboid {
        coordinate_bounds: [[Real; 2]; 3],
        coordinate_bounds_f64: OnceLock<Option<[[f64; 2]; 3]>>,
        position_vertices: Vec<OnceLock<Box<Vertex>>>,
        vertices: Vec<OnceLock<Box<Vertex>>>,
        first_position_identity: u64,
        first_coordinate_identity: u64,
    },
    Octahedron {
        radius: Real,
        vertices: Vec<OnceLock<Box<Vertex>>>,
        first_vertex_identity: u64,
    },
    Sphere {
        radius: Real,
        longitudes: Vec<(Real, Real)>,
        latitudes: Vec<(Real, Real)>,
        certified_position_bounds: Option<Arc<Vec<CertifiedF64Bounds>>>,
        vertices: Vec<OnceLock<Box<Vertex>>>,
        first_vertex_identity: u64,
    },
    #[cfg(feature = "sketch")]
    Torus {
        major_radius: Real,
        minor_radius: Real,
        major_samples: Vec<(Real, Real)>,
        minor_samples: Vec<(Real, Real)>,
        vertices: Vec<OnceLock<Box<Vertex>>>,
        first_vertex_identity: u64,
    },
    VerticalFrustum {
        radius1: Real,
        radius2: Real,
        height: Real,
        samples: Vec<(Real, Real)>,
        side_normals: Vec<Vector3>,
        vertices: Vec<OnceLock<Box<Vertex>>>,
        first_vertex_identity: u64,
        first_ruled_id: u64,
    },
    #[cfg(feature = "sketch")]
    ConvexExtrusion {
        points: Arc<Vec<[Real; 2]>>,
        edge_normals: Arc<Vec<Vector3>>,
        height: Real,
        vertices: Vec<OnceLock<Box<Vertex>>>,
        first_vertex_identity: u64,
    },
    RigidCopies {
        source_corners: Arc<Vec<Vertex>>,
        matrices: Vec<Matrix4>,
        corner_position_slots: Vec<usize>,
        positions_per_copy: usize,
        vertices: Vec<OnceLock<Box<Vertex>>>,
        first_vertex_identity: u64,
    },
    TranslatedCopies {
        source_corners: Arc<Vec<Vertex>>,
        offsets: Vec<Vector3>,
        corner_position_slots: Vec<usize>,
        corner_coordinate_slots: [Vec<usize>; 3],
        positions_per_copy: usize,
        coordinates_per_copy: [usize; 3],
        vertices: Vec<OnceLock<Box<Vertex>>>,
        first_position_identity: u64,
        first_coordinate_identities: [u64; 3],
    },
    ArcCopies {
        source_corners: Arc<Vec<Vertex>>,
        samples: Vec<(Real, Real)>,
        radius: Real,
        corner_position_slots: Vec<usize>,
        positions_per_copy: usize,
        vertices: Vec<OnceLock<Box<Vertex>>>,
        first_vertex_identity: u64,
    },
    Scaled {
        source: Arc<LazySubdivisionVertexPool>,
        scales: [Real; 3],
        inverse_scales: [Real; 3],
        vertices: Vec<OnceLock<Box<Vertex>>>,
        first_vertex_identity: u64,
    },
    Mapped {
        source: Arc<LazySubdivisionVertexPool>,
        transform: Box<LazyMappedTransform>,
        position_f64: Option<Arc<Vec<[f64; 3]>>>,
        vertices: Vec<OnceLock<Box<Vertex>>>,
        identities: Option<LazyMappedIdentities>,
    },
}

fn finite_cuboid_bounds(coordinate_bounds: &[[Real; 2]; 3]) -> Option<[[f64; 2]; 3]> {
    let finite_bounds = |axis: usize| {
        coordinate_bounds[axis][0]
            .to_f64_lossy()
            .zip(coordinate_bounds[axis][1].to_f64_lossy())
            .map(|(min, max)| [min, max])
    };
    finite_bounds(0)
        .zip(finite_bounds(1))
        .zip(finite_bounds(2))
        .map(|((x, y), z)| [x, y, z])
}

fn cuboid_vertex(
    coordinate_bounds: &[[Real; 2]; 3],
    position_slot: usize,
    face: usize,
    first_position_identity: u64,
    first_coordinate_identity: u64,
) -> Box<Vertex> {
    let coordinate_slots = CUBOID_POINT_COORDINATE_SLOTS[position_slot];
    let position = Point3::new(
        coordinate_bounds[0][coordinate_slots[0]].clone(),
        coordinate_bounds[1][coordinate_slots[1]].clone(),
        coordinate_bounds[2][coordinate_slots[2]].clone(),
    );
    let normal = match face {
        0 => -Vector3::z(),
        1 => Vector3::z(),
        2 => -Vector3::y(),
        3 => Vector3::y(),
        4 => -Vector3::x(),
        5 => Vector3::x(),
        _ => unreachable!("cuboid has six faces"),
    };
    Box::new(Vertex {
        position,
        normal,
        position_id: first_position_identity
            + u64::try_from(position_slot).expect("cuboid position slot fits u64"),
        coordinate_ids: std::array::from_fn(|axis| {
            first_coordinate_identity
                + u64::try_from(axis * 2 + coordinate_slots[axis])
                    .expect("cuboid coordinate slot fits u64")
        }),
        ruled_line: None,
        hull_candidate: true,
    })
}

impl LazySourceVertices {
    fn len(&self) -> usize {
        match self {
            Self::Materialized(vertices) => vertices.len(),
            Self::Cuboid { vertices, .. } => vertices.len(),
            Self::Octahedron { vertices, .. } => vertices.len(),
            Self::Sphere { vertices, .. } => vertices.len(),
            #[cfg(feature = "sketch")]
            Self::Torus { vertices, .. } => vertices.len(),
            Self::VerticalFrustum { vertices, .. } => vertices.len(),
            #[cfg(feature = "sketch")]
            Self::ConvexExtrusion { vertices, .. } => vertices.len(),
            Self::RigidCopies { vertices, .. } => vertices.len(),
            Self::TranslatedCopies { vertices, .. } => vertices.len(),
            Self::ArcCopies { vertices, .. } => vertices.len(),
            Self::Scaled { vertices, .. } => vertices.len(),
            Self::Mapped { vertices, .. } => vertices.len(),
        }
    }

    pub(crate) fn vertex(&self, index: usize) -> &Vertex {
        match self {
            Self::Materialized(vertices) => &vertices[index],
            Self::Cuboid {
                coordinate_bounds,
                vertices,
                first_position_identity,
                first_coordinate_identity,
                ..
            } => vertices[index].get_or_init(|| {
                cuboid_vertex(
                    coordinate_bounds,
                    CUBOID_CORNER_POSITION_SLOTS[index],
                    index / 4,
                    *first_position_identity,
                    *first_coordinate_identity,
                )
            }),
            Self::Octahedron {
                radius,
                vertices,
                first_vertex_identity,
            } => vertices[index].get_or_init(|| {
                let face = index / 3;
                let position_slot = OCTAHEDRON_FACES[face][index % 3];
                let negative_radius = -radius.clone();
                let zero = Real::zero();
                let position = match position_slot {
                    0 => Point3::new(radius.clone(), zero.clone(), zero),
                    1 => Point3::new(negative_radius.clone(), zero.clone(), zero),
                    2 => Point3::new(zero.clone(), radius.clone(), zero),
                    3 => Point3::new(zero.clone(), negative_radius.clone(), zero),
                    4 => Point3::new(zero.clone(), zero.clone(), radius.clone()),
                    5 => Point3::new(zero.clone(), zero, negative_radius),
                    _ => unreachable!("octahedron has six positions"),
                };
                Box::new(Vertex::new_with_reserved_identity(
                    position,
                    OCTAHEDRON_NORMALS[face].clone(),
                    *first_vertex_identity,
                    position_slot,
                ))
            }),
            Self::Sphere {
                radius,
                longitudes,
                latitudes,
                vertices,
                first_vertex_identity,
                ..
            } => vertices[index].get_or_init(|| {
                if index == 0 {
                    return Box::new(Vertex::new_with_reserved_identity(
                        Point3::new(Real::zero(), radius.clone(), Real::zero()),
                        Vector3::y(),
                        *first_vertex_identity,
                        index,
                    ));
                }
                if index + 1 == vertices.len() {
                    return Box::new(Vertex::new_with_reserved_identity(
                        Point3::new(Real::zero(), -radius.clone(), Real::zero()),
                        -Vector3::y(),
                        *first_vertex_identity,
                        index,
                    ));
                }

                let interior = index - 1;
                let longitude = interior / latitudes.len();
                let latitude = interior % latitudes.len();
                let (sin_theta, cos_theta) = &longitudes[longitude];
                let (sin_phi, cos_phi) = &latitudes[latitude];
                let direction = Vector3::from_xyz(
                    cos_theta.clone() * sin_phi.clone(),
                    cos_phi.clone(),
                    sin_theta.clone() * sin_phi.clone(),
                );
                Box::new(Vertex::new_with_reserved_identity(
                    Point3::new(
                        direction.0[0].clone() * radius.clone(),
                        direction.0[1].clone() * radius.clone(),
                        direction.0[2].clone() * radius.clone(),
                    ),
                    direction,
                    *first_vertex_identity,
                    index,
                ))
            }),
            #[cfg(feature = "sketch")]
            Self::Torus {
                major_radius,
                minor_radius,
                major_samples,
                minor_samples,
                vertices,
                first_vertex_identity,
            } => vertices[index].get_or_init(|| {
                let major = index / minor_samples.len();
                let minor = index % minor_samples.len();
                let (sin_theta, cos_theta) = &major_samples[major];
                let (sin_phi, cos_phi) = &minor_samples[minor];
                let radial = major_radius.clone() + minor_radius.clone() * cos_phi.clone();
                Box::new(Vertex::new_with_reserved_identity(
                    Point3::new(
                        radial.clone() * cos_theta.clone(),
                        radial * sin_theta.clone(),
                        minor_radius.clone() * sin_phi.clone(),
                    ),
                    Vector3::from_xyz(
                        cos_phi.clone() * cos_theta.clone(),
                        cos_phi.clone() * sin_theta.clone(),
                        sin_phi.clone(),
                    ),
                    *first_vertex_identity,
                    index,
                ))
            }),
            Self::VerticalFrustum {
                radius1,
                radius2,
                height,
                samples,
                side_normals,
                vertices,
                first_vertex_identity,
                first_ruled_id,
            } => vertices[index].get_or_init(|| {
                if index == 0 {
                    return Box::new(
                        Vertex::new_with_reserved_identity(
                            Point3::origin(),
                            -Vector3::z(),
                            *first_vertex_identity,
                            0,
                        )
                        .exclude_from_hull(),
                    );
                }
                if index == 1 {
                    return Box::new(
                        Vertex::new_with_reserved_identity(
                            Point3::new(Real::zero(), Real::zero(), height.clone()),
                            Vector3::z(),
                            *first_vertex_identity,
                            1,
                        )
                        .exclude_from_hull(),
                    );
                }

                let segment_count = samples.len();
                let ring_offset = index - 2;
                let ring = ring_offset / segment_count;
                let segment = ring_offset % segment_count;
                let (sin, cos) = &samples[segment];
                let is_bottom = matches!(ring, 0 | 2);
                let is_side = ring >= 2;
                let (radius, z, position_slot) = if is_bottom {
                    (radius1, Real::zero(), 2 + segment)
                } else {
                    (radius2, height.clone(), 2 + segment_count + segment)
                };
                let normal = if is_side {
                    if side_normals.is_empty() {
                        Vector3::from_xyz(cos.clone(), sin.clone(), Real::zero())
                    } else {
                        side_normals[segment].clone()
                    }
                } else if is_bottom {
                    -Vector3::z()
                } else {
                    Vector3::z()
                };
                let mut vertex = Vertex::new_with_reserved_identity(
                    Point3::new(radius.clone() * cos.clone(), radius.clone() * sin.clone(), z),
                    normal,
                    *first_vertex_identity,
                    position_slot,
                );
                if is_side {
                    vertex.ruled_line = Some([
                        *first_ruled_id,
                        *first_ruled_id
                            + 1
                            + u64::try_from(segment).expect("frustum segment index fits u64"),
                    ]);
                }
                Box::new(vertex)
            }),
            #[cfg(feature = "sketch")]
            Self::ConvexExtrusion {
                points,
                edge_normals,
                height,
                vertices,
                first_vertex_identity,
            } => vertices[index].get_or_init(|| {
                let point_count = points.len();
                let (point_index, top, normal) = if index < point_count {
                    (index, false, -Vector3::z())
                } else if index < 2 * point_count {
                    (index - point_count, true, Vector3::z())
                } else {
                    let side_corner = index - 2 * point_count;
                    let edge = side_corner / 4;
                    let corner = side_corner % 4;
                    (
                        if matches!(corner, 0 | 3) {
                            edge
                        } else {
                            (edge + 1) % point_count
                        },
                        corner >= 2,
                        edge_normals[edge].clone(),
                    )
                };
                let [x, y] = &points[point_index];
                Box::new(Vertex::new_with_reserved_identity(
                    Point3::new(
                        x.clone(),
                        y.clone(),
                        if top { height.clone() } else { Real::zero() },
                    ),
                    normal,
                    *first_vertex_identity,
                    if top {
                        point_count + point_index
                    } else {
                        point_index
                    },
                ))
            }),
            Self::RigidCopies {
                source_corners,
                matrices,
                corner_position_slots,
                positions_per_copy,
                vertices,
                first_vertex_identity,
            } => vertices[index].get_or_init(|| {
                let corners_per_copy = source_corners.len();
                let copy = index / corners_per_copy;
                let corner = index % corners_per_copy;
                let source = &source_corners[corner];
                let position_slot = corner_position_slots[corner];
                let identity_slot = copy * positions_per_copy + position_slot;
                let mut vertex = Vertex::new_with_reserved_identity(
                    matrices[copy]
                        .transform_point3(&source.position)
                        .expect("rigid copy matrix transforms finite points"),
                    matrices[copy].transform_direction3(&source.normal),
                    *first_vertex_identity,
                    identity_slot,
                );
                vertex.ruled_line = source.ruled_line;
                vertex.hull_candidate = source.hull_candidate;
                Box::new(vertex)
            }),
            Self::TranslatedCopies {
                source_corners,
                offsets,
                corner_position_slots,
                corner_coordinate_slots,
                positions_per_copy,
                coordinates_per_copy,
                vertices,
                first_position_identity,
                first_coordinate_identities,
            } => vertices[index].get_or_init(|| {
                let corners_per_copy = source_corners.len();
                let copy = index / corners_per_copy;
                let corner = index % corners_per_copy;
                let source = &source_corners[corner];
                let position_slot = corner_position_slots[corner];
                Box::new(Vertex {
                    position: source.position.clone() + offsets[copy].clone(),
                    normal: source.normal.clone(),
                    position_id: *first_position_identity
                        + u64::try_from(copy * positions_per_copy + position_slot)
                            .expect("translated copy position slot fits u64"),
                    coordinate_ids: std::array::from_fn(|axis| {
                        first_coordinate_identities[axis]
                            + u64::try_from(
                                copy * coordinates_per_copy[axis]
                                    + corner_coordinate_slots[axis][corner],
                            )
                            .expect("translated copy coordinate slot fits u64")
                    }),
                    ruled_line: source.ruled_line,
                    hull_candidate: source.hull_candidate,
                })
            }),
            Self::ArcCopies {
                source_corners,
                samples,
                radius,
                corner_position_slots,
                positions_per_copy,
                vertices,
                first_vertex_identity,
            } => vertices[index].get_or_init(|| {
                let corners_per_copy = source_corners.len();
                let copy = index / corners_per_copy;
                let corner = index % corners_per_copy;
                let source = &source_corners[corner];
                let (sin, cos) = &samples[copy];
                let translated_x = source.position.x.clone() + radius.clone();
                let position = Point3::new(
                    cos.clone() * translated_x.clone()
                        - sin.clone() * source.position.y.clone(),
                    sin.clone() * translated_x + cos.clone() * source.position.y.clone(),
                    source.position.z.clone(),
                );
                let normal = Vector3::from_xyz(
                    cos.clone() * source.normal.0[0].clone()
                        - sin.clone() * source.normal.0[1].clone(),
                    sin.clone() * source.normal.0[0].clone()
                        + cos.clone() * source.normal.0[1].clone(),
                    source.normal.0[2].clone(),
                );
                let position_slot = corner_position_slots[corner];
                let identity_slot = copy * positions_per_copy + position_slot;
                let mut vertex = Vertex::new_with_reserved_identity(
                    position,
                    normal,
                    *first_vertex_identity,
                    identity_slot,
                );
                vertex.ruled_line = source.ruled_line;
                vertex.hull_candidate = source.hull_candidate;
                Box::new(vertex)
            }),
            Self::Scaled {
                source,
                scales,
                inverse_scales,
                vertices,
                first_vertex_identity,
            } => vertices[index].get_or_init(|| {
                let source = source.vertex(index);
                let mut vertex = Vertex::new_with_reserved_identity(
                    Point3::new(
                        source.position.x.clone() * scales[0].clone(),
                        source.position.y.clone() * scales[1].clone(),
                        source.position.z.clone() * scales[2].clone(),
                    ),
                    Vector3::new([
                        source.normal.0[0].clone() * inverse_scales[0].clone(),
                        source.normal.0[1].clone() * inverse_scales[1].clone(),
                        source.normal.0[2].clone() * inverse_scales[2].clone(),
                    ]),
                    *first_vertex_identity,
                    index,
                );
                vertex.ruled_line = source.ruled_line;
                vertex.hull_candidate = source.hull_candidate;
                Box::new(vertex)
            }),
            Self::Mapped {
                source,
                transform,
                vertices,
                identities,
                ..
            } => vertices[index].get_or_init(|| {
                let source = source.vertex(index);
                if matches!(transform.as_ref(), LazyMappedTransform::Invert) {
                    let mut vertex = source.clone();
                    vertex.flip();
                    return Box::new(vertex);
                }
                let (position, normal) = match transform.as_ref() {
                    LazyMappedTransform::Translate(translation) => (
                        Point3::new(
                            source.position.x.clone() + translation.0[0].clone(),
                            source.position.y.clone() + translation.0[1].clone(),
                            source.position.z.clone() + translation.0[2].clone(),
                        ),
                        source.normal.clone(),
                    ),
                    LazyMappedTransform::AxisReflection { axis, value } => {
                        let mut position = source.position.clone();
                        let coordinate = match *axis {
                            0 => &mut position.x,
                            1 => &mut position.y,
                            2 => &mut position.z,
                            _ => unreachable!("reflection axis is in 0..3"),
                        };
                        *coordinate = Real::from(2_u8) * value.clone() - coordinate.clone();
                        let mut normal = -source.normal.clone();
                        normal.0[*axis] = source.normal.0[*axis].clone();
                        (position, normal)
                    },
                    LazyMappedTransform::Scale {
                        scales,
                        inverse_scales,
                    } => (
                        Point3::new(
                            source.position.x.clone() * scales[0].clone(),
                            source.position.y.clone() * scales[1].clone(),
                            source.position.z.clone() * scales[2].clone(),
                        ),
                        Vector3::new([
                            source.normal.0[0].clone() * inverse_scales[0].clone(),
                            source.normal.0[1].clone() * inverse_scales[1].clone(),
                            source.normal.0[2].clone() * inverse_scales[2].clone(),
                        ]),
                    ),
                    LazyMappedTransform::Affine {
                        matrix,
                        normal_matrix,
                        normalize_normals,
                        flip_normals,
                    } => {
                        let position = matrix
                            .prepare()
                            .transform_point3(&source.position)
                            .expect("retained affine transform preserves finite points");
                        let transformed_normal =
                            normal_matrix.prepare().transform_direction3(&source.normal);
                        let mut normal = if *normalize_normals {
                            finite_normalized_exact_rational(&transformed_normal)
                                .or_else(|| transformed_normal.normalize_checked().ok())
                                .expect("retained affine transform has a valid normal")
                        } else {
                            transformed_normal
                        };
                        if *flip_normals {
                            normal = -normal;
                        }
                        (position, normal)
                    },
                    LazyMappedTransform::Invert => unreachable!(),
                };
                let identities =
                    identities.expect("mapped geometric transforms reserve identities");
                let slot = u64::try_from(index).expect("vertex slot fits u64");
                let mut vertex = Vertex {
                    position,
                    normal,
                    position_id: identities.first_position_id + slot,
                    coordinate_ids: std::array::from_fn(|axis| {
                        identities.first_coordinate_ids[axis] + slot
                    }),
                    ruled_line: source.ruled_line,
                    hull_candidate: source.hull_candidate,
                };
                vertex.ruled_line = source.ruled_line;
                vertex.hull_candidate = source.hull_candidate;
                Box::new(vertex)
            }),
        }
    }

    fn cuboid_position_vertex(&self, position_slot: usize) -> &Vertex {
        match self {
            Self::Cuboid {
                coordinate_bounds,
                position_vertices,
                first_position_identity,
                first_coordinate_identity,
                ..
            } => position_vertices[position_slot].get_or_init(|| {
                cuboid_vertex(
                    coordinate_bounds,
                    position_slot,
                    CUBOID_POSITION_REPRESENTATIVES[position_slot][0],
                    *first_position_identity,
                    *first_coordinate_identity,
                )
            }),
            _ => unreachable!("only cuboid pools have cuboid position representatives"),
        }
    }

    fn position_f64_lossy(&self, index: usize) -> Option<[f64; 3]> {
        match self {
            Self::Cuboid {
                coordinate_bounds,
                coordinate_bounds_f64,
                ..
            } => {
                let coordinate_bounds = coordinate_bounds_f64
                    .get_or_init(|| finite_cuboid_bounds(coordinate_bounds))
                    .as_ref()?;
                let coordinate_slots =
                    CUBOID_POINT_COORDINATE_SLOTS[CUBOID_CORNER_POSITION_SLOTS[index]];
                Some(std::array::from_fn(|axis| {
                    coordinate_bounds[axis][coordinate_slots[axis]]
                }))
            },
            Self::Mapped {
                position_f64: Some(positions),
                ..
            } => positions.get(index).copied(),
            _ => self.vertex(index).position_f64_lossy(),
        }
    }
}

#[derive(Debug)]
pub(crate) struct LazySubdivisionVertexPool {
    source_vertices: LazySourceVertices,
    midpoint_edges: Vec<[usize; 2]>,
    midpoints: Vec<OnceLock<Box<Vertex>>>,
    materialized_polygons: Vec<OnceLock<Arc<Vec<Vertex>>>>,
    first_midpoint_identity: u64,
}

impl LazySubdivisionVertexPool {
    pub(crate) fn new(
        source_vertices: Vec<Vertex>,
        midpoint_edges: Vec<[usize; 2]>,
        polygon_count: usize,
        first_midpoint_identity: u64,
    ) -> Self {
        debug_assert!(midpoint_edges.iter().all(|[left, right]| {
            *left < source_vertices.len() && *right < source_vertices.len()
        }));
        let midpoints = std::iter::repeat_with(OnceLock::new)
            .take(midpoint_edges.len())
            .collect();
        let materialized_polygons = std::iter::repeat_with(OnceLock::new)
            .take(polygon_count)
            .collect();
        Self {
            source_vertices: LazySourceVertices::Materialized(Arc::new(source_vertices)),
            midpoint_edges,
            midpoints,
            materialized_polygons,
            first_midpoint_identity,
        }
    }

    pub(crate) fn new_cuboid(
        dimensions: [Real; 3],
        first_position_identity: u64,
        first_coordinate_identity: u64,
    ) -> Self {
        Self::new_cuboid_with_bounds(
            std::array::from_fn(|axis| [Real::zero(), dimensions[axis].clone()]),
            OnceLock::new(),
            first_position_identity,
            first_coordinate_identity,
        )
    }

    pub(crate) fn new_cuboid_bounds(
        coordinate_bounds: [[Real; 2]; 3],
        first_position_identity: u64,
        first_coordinate_identity: u64,
    ) -> Self {
        let coordinate_bounds_f64 = OnceLock::from(finite_cuboid_bounds(&coordinate_bounds));
        Self::new_cuboid_with_bounds(
            coordinate_bounds,
            coordinate_bounds_f64,
            first_position_identity,
            first_coordinate_identity,
        )
    }

    fn new_cuboid_with_bounds(
        coordinate_bounds: [[Real; 2]; 3],
        coordinate_bounds_f64: OnceLock<Option<[[f64; 2]; 3]>>,
        first_position_identity: u64,
        first_coordinate_identity: u64,
    ) -> Self {
        Self {
            source_vertices: LazySourceVertices::Cuboid {
                coordinate_bounds,
                coordinate_bounds_f64,
                position_vertices: std::iter::repeat_with(OnceLock::new).take(8).collect(),
                vertices: std::iter::repeat_with(OnceLock::new).take(24).collect(),
                first_position_identity,
                first_coordinate_identity,
            },
            midpoint_edges: Vec::new(),
            midpoints: Vec::new(),
            materialized_polygons: std::iter::repeat_with(OnceLock::new).take(6).collect(),
            first_midpoint_identity: first_position_identity,
        }
    }

    pub(crate) fn new_octahedron(radius: Real, first_vertex_identity: u64) -> Self {
        Self {
            source_vertices: LazySourceVertices::Octahedron {
                radius,
                vertices: std::iter::repeat_with(OnceLock::new).take(24).collect(),
                first_vertex_identity,
            },
            midpoint_edges: Vec::new(),
            midpoints: Vec::new(),
            materialized_polygons: std::iter::repeat_with(OnceLock::new).take(8).collect(),
            first_midpoint_identity: first_vertex_identity,
        }
    }

    pub(crate) fn new_sphere(
        radius: Real,
        longitudes: Vec<(Real, Real)>,
        latitudes: Vec<(Real, Real)>,
        certified_position_bounds: Option<Vec<CertifiedF64Bounds>>,
        polygon_count: usize,
        first_vertex_identity: u64,
    ) -> Self {
        let vertex_count = 2 + longitudes.len() * latitudes.len();
        Self {
            source_vertices: LazySourceVertices::Sphere {
                radius,
                longitudes,
                latitudes,
                certified_position_bounds: certified_position_bounds.map(Arc::new),
                vertices: std::iter::repeat_with(OnceLock::new)
                    .take(vertex_count)
                    .collect(),
                first_vertex_identity,
            },
            midpoint_edges: Vec::new(),
            midpoints: Vec::new(),
            materialized_polygons: std::iter::repeat_with(OnceLock::new)
                .take(polygon_count)
                .collect(),
            first_midpoint_identity: first_vertex_identity,
        }
    }

    pub(crate) fn new_scaled(
        source: Arc<Self>,
        scales: [Real; 3],
        inverse_scales: [Real; 3],
        polygon_count: usize,
        first_vertex_identity: u64,
    ) -> Self {
        let vertex_count = source.len();
        Self {
            source_vertices: LazySourceVertices::Scaled {
                source,
                scales,
                inverse_scales,
                vertices: std::iter::repeat_with(OnceLock::new)
                    .take(vertex_count)
                    .collect(),
                first_vertex_identity,
            },
            midpoint_edges: Vec::new(),
            midpoints: Vec::new(),
            materialized_polygons: std::iter::repeat_with(OnceLock::new)
                .take(polygon_count)
                .collect(),
            first_midpoint_identity: first_vertex_identity,
        }
    }

    pub(crate) fn new_translated(
        source: Arc<Self>,
        translation: Vector3,
        position_f64: Option<Arc<Vec<[f64; 3]>>>,
        polygon_count: usize,
        first_position_id: u64,
        first_coordinate_ids: [u64; 3],
    ) -> Self {
        Self::new_mapped(
            source,
            LazyMappedTransform::Translate(translation),
            position_f64,
            polygon_count,
            Some(LazyMappedIdentities {
                first_position_id,
                first_coordinate_ids,
            }),
        )
    }

    pub(crate) fn new_scale_transform(
        source: Arc<Self>,
        scales: [Real; 3],
        inverse_scales: [Real; 3],
        position_f64: Option<Arc<Vec<[f64; 3]>>>,
        polygon_count: usize,
        first_position_id: u64,
        first_coordinate_ids: [u64; 3],
    ) -> Self {
        Self::new_mapped(
            source,
            LazyMappedTransform::Scale {
                scales,
                inverse_scales,
            },
            position_f64,
            polygon_count,
            Some(LazyMappedIdentities {
                first_position_id,
                first_coordinate_ids,
            }),
        )
    }

    pub(crate) fn new_axis_reflected(
        source: Arc<Self>,
        axis: usize,
        value: Real,
        position_f64: Option<Arc<Vec<[f64; 3]>>>,
        polygon_count: usize,
        first_position_id: u64,
        first_coordinate_ids: [u64; 3],
    ) -> Self {
        Self::new_mapped(
            source,
            LazyMappedTransform::AxisReflection { axis, value },
            position_f64,
            polygon_count,
            Some(LazyMappedIdentities {
                first_position_id,
                first_coordinate_ids,
            }),
        )
    }

    pub(crate) fn new_affine_transform(
        source: Arc<Self>,
        matrix: Matrix4,
        normal_matrix: Matrix4,
        normalize_normals: bool,
        flip_normals: bool,
        position_f64: Option<Arc<Vec<[f64; 3]>>>,
        polygon_count: usize,
        first_position_id: u64,
        first_coordinate_ids: [u64; 3],
    ) -> Self {
        Self::new_mapped(
            source,
            LazyMappedTransform::Affine {
                matrix: Box::new(matrix),
                normal_matrix: Box::new(normal_matrix),
                normalize_normals,
                flip_normals,
            },
            position_f64,
            polygon_count,
            Some(LazyMappedIdentities {
                first_position_id,
                first_coordinate_ids,
            }),
        )
    }

    pub(crate) fn new_inverted(
        source: Arc<Self>,
        position_f64: Option<Arc<Vec<[f64; 3]>>>,
        polygon_count: usize,
    ) -> Self {
        Self::new_mapped(
            source,
            LazyMappedTransform::Invert,
            position_f64,
            polygon_count,
            None,
        )
    }

    fn new_mapped(
        source: Arc<Self>,
        transform: LazyMappedTransform,
        position_f64: Option<Arc<Vec<[f64; 3]>>>,
        polygon_count: usize,
        identities: Option<LazyMappedIdentities>,
    ) -> Self {
        let vertex_count = source.len();
        Self {
            source_vertices: LazySourceVertices::Mapped {
                source,
                transform: Box::new(transform),
                position_f64,
                vertices: std::iter::repeat_with(OnceLock::new)
                    .take(vertex_count)
                    .collect(),
                identities,
            },
            midpoint_edges: Vec::new(),
            midpoints: Vec::new(),
            materialized_polygons: std::iter::repeat_with(OnceLock::new)
                .take(polygon_count)
                .collect(),
            first_midpoint_identity: 0,
        }
    }

    #[cfg(feature = "sketch")]
    pub(crate) fn new_torus(
        major_radius: Real,
        minor_radius: Real,
        major_samples: Vec<(Real, Real)>,
        minor_samples: Vec<(Real, Real)>,
        polygon_count: usize,
        first_vertex_identity: u64,
    ) -> Self {
        let vertex_count = major_samples.len() * minor_samples.len();
        Self {
            source_vertices: LazySourceVertices::Torus {
                major_radius,
                minor_radius,
                major_samples,
                minor_samples,
                vertices: std::iter::repeat_with(OnceLock::new)
                    .take(vertex_count)
                    .collect(),
                first_vertex_identity,
            },
            midpoint_edges: Vec::new(),
            midpoints: Vec::new(),
            materialized_polygons: std::iter::repeat_with(OnceLock::new)
                .take(polygon_count)
                .collect(),
            first_midpoint_identity: first_vertex_identity,
        }
    }

    pub(crate) fn new_vertical_frustum(
        radius1: Real,
        radius2: Real,
        height: Real,
        samples: Vec<(Real, Real)>,
        side_normals: Vec<Vector3>,
        polygon_count: usize,
        first_vertex_identity: u64,
        first_ruled_id: u64,
    ) -> Self {
        debug_assert!(side_normals.is_empty() || samples.len() == side_normals.len());
        let vertex_count = 2 + 4 * samples.len();
        Self {
            source_vertices: LazySourceVertices::VerticalFrustum {
                radius1,
                radius2,
                height,
                samples,
                side_normals,
                vertices: std::iter::repeat_with(OnceLock::new)
                    .take(vertex_count)
                    .collect(),
                first_vertex_identity,
                first_ruled_id,
            },
            midpoint_edges: Vec::new(),
            midpoints: Vec::new(),
            materialized_polygons: std::iter::repeat_with(OnceLock::new)
                .take(polygon_count)
                .collect(),
            first_midpoint_identity: first_vertex_identity,
        }
    }

    #[cfg(feature = "sketch")]
    pub(crate) fn new_convex_extrusion(
        points: Arc<Vec<[Real; 2]>>,
        edge_normals: Arc<Vec<Vector3>>,
        height: Real,
        first_vertex_identity: u64,
    ) -> Self {
        debug_assert_eq!(points.len(), edge_normals.len());
        let polygon_count = points.len() + 2;
        let vertex_count = 6 * points.len();
        Self {
            source_vertices: LazySourceVertices::ConvexExtrusion {
                points,
                edge_normals,
                height,
                vertices: std::iter::repeat_with(OnceLock::new)
                    .take(vertex_count)
                    .collect(),
                first_vertex_identity,
            },
            midpoint_edges: Vec::new(),
            midpoints: Vec::new(),
            materialized_polygons: std::iter::repeat_with(OnceLock::new)
                .take(polygon_count)
                .collect(),
            first_midpoint_identity: first_vertex_identity,
        }
    }

    pub(crate) fn new_rigid_copies(
        source_corners: Vec<Vertex>,
        matrices: Vec<Matrix4>,
        corner_position_slots: Vec<usize>,
        positions_per_copy: usize,
        polygon_count: usize,
        first_vertex_identity: u64,
    ) -> Self {
        debug_assert_eq!(source_corners.len(), corner_position_slots.len());
        let vertex_count = source_corners.len() * matrices.len();
        Self {
            source_vertices: LazySourceVertices::RigidCopies {
                source_corners: Arc::new(source_corners),
                matrices,
                corner_position_slots,
                positions_per_copy,
                vertices: std::iter::repeat_with(OnceLock::new)
                    .take(vertex_count)
                    .collect(),
                first_vertex_identity,
            },
            midpoint_edges: Vec::new(),
            midpoints: Vec::new(),
            materialized_polygons: std::iter::repeat_with(OnceLock::new)
                .take(polygon_count)
                .collect(),
            first_midpoint_identity: first_vertex_identity,
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new_translated_copies(
        source_corners: Vec<Vertex>,
        offsets: Vec<Vector3>,
        corner_position_slots: Vec<usize>,
        corner_coordinate_slots: [Vec<usize>; 3],
        positions_per_copy: usize,
        coordinates_per_copy: [usize; 3],
        polygon_count: usize,
        first_position_identity: u64,
        first_coordinate_identities: [u64; 3],
    ) -> Self {
        debug_assert_eq!(source_corners.len(), corner_position_slots.len());
        debug_assert!(
            corner_coordinate_slots
                .iter()
                .all(|slots| slots.len() == source_corners.len())
        );
        let vertex_count = source_corners.len() * offsets.len();
        Self {
            source_vertices: LazySourceVertices::TranslatedCopies {
                source_corners: Arc::new(source_corners),
                offsets,
                corner_position_slots,
                corner_coordinate_slots,
                positions_per_copy,
                coordinates_per_copy,
                vertices: std::iter::repeat_with(OnceLock::new)
                    .take(vertex_count)
                    .collect(),
                first_position_identity,
                first_coordinate_identities,
            },
            midpoint_edges: Vec::new(),
            midpoints: Vec::new(),
            materialized_polygons: std::iter::repeat_with(OnceLock::new)
                .take(polygon_count)
                .collect(),
            first_midpoint_identity: first_position_identity,
        }
    }

    pub(crate) fn new_arc_copies(
        source_corners: Vec<Vertex>,
        samples: Vec<(Real, Real)>,
        radius: Real,
        corner_position_slots: Vec<usize>,
        positions_per_copy: usize,
        polygon_count: usize,
        first_vertex_identity: u64,
    ) -> Self {
        debug_assert_eq!(source_corners.len(), corner_position_slots.len());
        let vertex_count = source_corners.len() * samples.len();
        Self {
            source_vertices: LazySourceVertices::ArcCopies {
                source_corners: Arc::new(source_corners),
                samples,
                radius,
                corner_position_slots,
                positions_per_copy,
                vertices: std::iter::repeat_with(OnceLock::new)
                    .take(vertex_count)
                    .collect(),
                first_vertex_identity,
            },
            midpoint_edges: Vec::new(),
            midpoints: Vec::new(),
            materialized_polygons: std::iter::repeat_with(OnceLock::new)
                .take(polygon_count)
                .collect(),
            first_midpoint_identity: first_vertex_identity,
        }
    }

    pub(crate) fn len(&self) -> usize {
        self.source_vertices.len() + self.midpoint_edges.len()
    }

    pub(crate) fn vertex(&self, index: usize) -> &Vertex {
        if index < self.source_vertices.len() {
            return self.source_vertices.vertex(index);
        }
        let midpoint_index = index - self.source_vertices.len();
        self.midpoints[midpoint_index].get_or_init(|| {
            let [left, right] = self.midpoint_edges[midpoint_index];
            let left = self.source_vertices.vertex(left);
            let right = self.source_vertices.vertex(right);
            let midpoint_real = |left: &Real, right: &Real| {
                if let (Some(left), Some(right)) =
                    (left.exact_rational_ref(), right.exact_rational_ref())
                {
                    Real::from(Rational::average_pair(left, right))
                } else {
                    (left + right)
                        * Real::from(Rational::fraction(1, 2).expect("two is nonzero"))
                }
            };
            Box::new(Vertex::new_with_reserved_identity(
                Point3::new(
                    midpoint_real(&left.position.x, &right.position.x),
                    midpoint_real(&left.position.y, &right.position.y),
                    midpoint_real(&left.position.z, &right.position.z),
                ),
                Vector3::new([
                    midpoint_real(&left.normal.0[0], &right.normal.0[0]),
                    midpoint_real(&left.normal.0[1], &right.normal.0[1]),
                    midpoint_real(&left.normal.0[2], &right.normal.0[2]),
                ]),
                self.first_midpoint_identity,
                midpoint_index,
            ))
        })
    }

    pub(crate) fn cuboid_position_vertex(&self, position_slot: usize) -> &Vertex {
        self.source_vertices.cuboid_position_vertex(position_slot)
    }

    fn position_f64_lossy(&self, index: usize) -> Option<[f64; 3]> {
        if index < self.source_vertices.len() {
            return self.source_vertices.position_f64_lossy(index);
        }
        self.vertex(index).position_f64_lossy()
    }

    fn certified_f64_bounds(&self, indices: &[usize]) -> Option<CertifiedF64Bounds> {
        let LazySourceVertices::Sphere {
            certified_position_bounds: Some(position_bounds),
            ..
        } = &self.source_vertices
        else {
            return None;
        };
        let mut minimum = [f64::INFINITY; 3];
        let mut maximum = [f64::NEG_INFINITY; 3];
        for &index in indices {
            for axis in 0..3 {
                minimum[axis] = minimum[axis].min(position_bounds[index].min[axis]);
                maximum[axis] = maximum[axis].max(position_bounds[index].max[axis]);
            }
        }
        Some(CertifiedF64Bounds {
            min: minimum,
            max: maximum,
        })
    }

    fn materialized_polygon(
        &self,
        polygon_slot: usize,
        indices: &[usize],
    ) -> &Arc<Vec<Vertex>> {
        self.materialized_polygons[polygon_slot].get_or_init(|| {
            Arc::new(
                indices
                    .iter()
                    .map(|&index| self.vertex(index).clone())
                    .collect(),
            )
        })
    }
}

#[derive(Clone, Debug)]
enum IndexedPolygonVertices {
    Triangle([usize; 3]),
    Quad([usize; 4]),
    Ngon(Arc<[usize]>),
}

impl IndexedPolygonVertices {
    fn as_slice(&self) -> &[usize] {
        match self {
            Self::Triangle(indices) => indices,
            Self::Quad(indices) => indices,
            Self::Ngon(indices) => indices,
        }
    }
}

#[derive(Clone, Debug)]
pub(crate) struct PolygonVertices {
    buffer: Arc<Vec<Vertex>>,
    range: Range<usize>,
    indexed_polygon: Option<IndexedPolygonVertices>,
    lazy_subdivision_pool: Option<Arc<LazySubdivisionVertexPool>>,
    lazy_polygon_slot: Option<usize>,
}

impl PolygonVertices {
    fn new(vertices: Vec<Vertex>) -> Self {
        let len = vertices.len();
        Self {
            buffer: Arc::new(vertices),
            range: 0..len,
            indexed_polygon: None,
            lazy_subdivision_pool: None,
            lazy_polygon_slot: None,
        }
    }

    pub(crate) fn from_shared(buffer: Arc<Vec<Vertex>>, range: Range<usize>) -> Self {
        debug_assert!(range.end <= buffer.len());
        Self {
            buffer,
            range,
            indexed_polygon: None,
            lazy_subdivision_pool: None,
            lazy_polygon_slot: None,
        }
    }

    #[cfg(test)]
    pub(crate) fn shares_buffer_with(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.buffer, &other.buffer)
    }

    pub(crate) fn from_lazy_subdivision_triangle(
        pool: Arc<LazySubdivisionVertexPool>,
        triangle_slot: usize,
        indices: [usize; 3],
    ) -> Self {
        debug_assert!(indices.iter().all(|index| *index < pool.len()));
        debug_assert!(triangle_slot < pool.materialized_polygons.len());
        Self {
            buffer: Arc::clone(&EMPTY_VERTEX_BUFFER),
            range: 0..3,
            indexed_polygon: Some(IndexedPolygonVertices::Triangle(indices)),
            lazy_subdivision_pool: Some(pool),
            lazy_polygon_slot: Some(triangle_slot),
        }
    }

    pub(crate) fn from_lazy_indexed_quad(
        pool: Arc<LazySubdivisionVertexPool>,
        polygon_slot: usize,
        indices: [usize; 4],
    ) -> Self {
        debug_assert!(indices.iter().all(|index| *index < pool.len()));
        debug_assert!(polygon_slot < pool.materialized_polygons.len());
        Self {
            buffer: Arc::clone(&EMPTY_VERTEX_BUFFER),
            range: 0..4,
            indexed_polygon: Some(IndexedPolygonVertices::Quad(indices)),
            lazy_subdivision_pool: Some(pool),
            lazy_polygon_slot: Some(polygon_slot),
        }
    }

    pub(crate) fn from_lazy_indexed_polygon(
        pool: Arc<LazySubdivisionVertexPool>,
        polygon_slot: usize,
        indices: Vec<usize>,
    ) -> Self {
        debug_assert!(indices.len() >= 3);
        debug_assert!(indices.iter().all(|index| *index < pool.len()));
        debug_assert!(polygon_slot < pool.materialized_polygons.len());
        let len = indices.len();
        Self {
            buffer: Arc::clone(&EMPTY_VERTEX_BUFFER),
            range: 0..len,
            indexed_polygon: Some(IndexedPolygonVertices::Ngon(indices.into())),
            lazy_subdivision_pool: Some(pool),
            lazy_polygon_slot: Some(polygon_slot),
        }
    }

    pub(crate) fn into_vec(self) -> Vec<Vertex> {
        if let Some(indices) = &self.indexed_polygon {
            return self
                .lazy_subdivision_pool
                .as_ref()
                .expect("indexed polygon retains its lazy vertex pool")
                .materialized_polygon(
                    self.lazy_polygon_slot
                        .expect("indexed polygon retains its pool slot"),
                    indices.as_slice(),
                )
                .as_ref()
                .clone();
        }
        if self.range.start == 0 && self.range.end == self.buffer.len() {
            return Arc::try_unwrap(self.buffer)
                .unwrap_or_else(|vertices| (*vertices).clone());
        }
        self.buffer[self.range].to_vec()
    }

    fn borrowed_iter(&self) -> PolygonVertexIter<'_> {
        if let Some(indices) = &self.indexed_polygon {
            return PolygonVertexIter::Indexed {
                indices: indices.as_slice().iter(),
                pool: self
                    .lazy_subdivision_pool
                    .as_ref()
                    .expect("indexed polygon retains its lazy vertex pool"),
            };
        }
        PolygonVertexIter::Slice(self.buffer[self.range.clone()].iter())
    }

    fn position_f64_iter(&self) -> PolygonPositionF64Iter<'_> {
        if let Some(indices) = &self.indexed_polygon {
            return PolygonPositionF64Iter::Indexed {
                indices: indices.as_slice().iter(),
                pool: self
                    .lazy_subdivision_pool
                    .as_ref()
                    .expect("indexed polygon retains its lazy vertex pool"),
            };
        }
        PolygonPositionF64Iter::Slice(self.buffer[self.range.clone()].iter())
    }

    fn retained_certified_f64_bounds(&self) -> Option<CertifiedF64Bounds> {
        let indices = self.indexed_polygon.as_ref()?.as_slice();
        self.lazy_subdivision_pool
            .as_ref()?
            .certified_f64_bounds(indices)
    }
}

enum PolygonVertexIter<'a> {
    Slice(std::slice::Iter<'a, Vertex>),
    Indexed {
        indices: std::slice::Iter<'a, usize>,
        pool: &'a LazySubdivisionVertexPool,
    },
}

impl<'a> Iterator for PolygonVertexIter<'a> {
    type Item = &'a Vertex;

    fn next(&mut self) -> Option<Self::Item> {
        match self {
            Self::Slice(vertices) => vertices.next(),
            Self::Indexed { indices, pool } => indices.next().map(|&index| pool.vertex(index)),
        }
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        match self {
            Self::Slice(vertices) => vertices.size_hint(),
            Self::Indexed { indices, .. } => indices.size_hint(),
        }
    }
}

impl ExactSizeIterator for PolygonVertexIter<'_> {}

enum PolygonPositionF64Iter<'a> {
    Slice(std::slice::Iter<'a, Vertex>),
    Indexed {
        indices: std::slice::Iter<'a, usize>,
        pool: &'a LazySubdivisionVertexPool,
    },
}

impl Iterator for PolygonPositionF64Iter<'_> {
    type Item = Option<[f64; 3]>;

    fn next(&mut self) -> Option<Self::Item> {
        match self {
            Self::Slice(vertices) => vertices.next().map(Vertex::position_f64_lossy),
            Self::Indexed { indices, pool } => {
                indices.next().map(|&index| pool.position_f64_lossy(index))
            },
        }
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        match self {
            Self::Slice(vertices) => vertices.size_hint(),
            Self::Indexed { indices, .. } => indices.size_hint(),
        }
    }
}

impl ExactSizeIterator for PolygonPositionF64Iter<'_> {}

impl Deref for PolygonVertices {
    type Target = [Vertex];

    fn deref(&self) -> &Self::Target {
        if let Some(indices) = &self.indexed_polygon {
            return self
                .lazy_subdivision_pool
                .as_ref()
                .expect("indexed polygon retains its lazy vertex pool")
                .materialized_polygon(
                    self.lazy_polygon_slot
                        .expect("indexed polygon retains its pool slot"),
                    indices.as_slice(),
                );
        }
        &self.buffer[self.range.clone()]
    }
}

impl DerefMut for PolygonVertices {
    fn deref_mut(&mut self) -> &mut Self::Target {
        if self.indexed_polygon.is_some() {
            let vertices = self.deref().to_vec();
            let len = vertices.len();
            self.buffer = Arc::new(vertices);
            self.range = 0..len;
            self.indexed_polygon = None;
            self.lazy_subdivision_pool = None;
            self.lazy_polygon_slot = None;
        }
        if self.range.start != 0 || self.range.end != self.buffer.len() {
            let vertices = self.to_vec();
            let len = vertices.len();
            self.buffer = Arc::new(vertices);
            self.range = 0..len;
        }
        &mut Arc::make_mut(&mut self.buffer)[self.range.clone()]
    }
}

impl PartialEq for PolygonVertices {
    fn eq(&self, other: &Self) -> bool {
        **self == **other
    }
}

impl IntoIterator for PolygonVertices {
    type Item = Vertex;
    type IntoIter = std::vec::IntoIter<Vertex>;

    fn into_iter(self) -> Self::IntoIter {
        self.into_vec().into_iter()
    }
}

impl<'a> IntoIterator for &'a PolygonVertices {
    type Item = &'a Vertex;
    type IntoIter = std::slice::Iter<'a, Vertex>;

    fn into_iter(self) -> Self::IntoIter {
        self.iter()
    }
}

impl<'a> IntoIterator for &'a mut PolygonVertices {
    type Item = &'a mut Vertex;
    type IntoIter = std::slice::IterMut<'a, Vertex>;

    fn into_iter(self) -> Self::IntoIter {
        self.iter_mut()
    }
}

#[derive(Clone, Debug)]
pub(crate) struct PolygonPlane {
    plane: OnceLock<Arc<Plane>>,
    source_vertices: Option<PolygonVertices>,
}

impl PolygonPlane {
    fn from_vertices(vertices: &PolygonVertices) -> Self {
        Self {
            plane: OnceLock::new(),
            source_vertices: Some(vertices.clone()),
        }
    }

    fn get(&self) -> &Arc<Plane> {
        self.plane.get_or_init(|| {
            Arc::new(Plane::from_vertices(
                self.source_vertices
                    .as_ref()
                    .expect("deferred polygon plane retains source vertices"),
            ))
        })
    }
}

impl Deref for PolygonPlane {
    type Target = Plane;

    fn deref(&self) -> &Self::Target {
        self.get()
    }
}

impl DerefMut for PolygonPlane {
    fn deref_mut(&mut self) -> &mut Self::Target {
        if self.plane.get().is_none() {
            let plane = Arc::new(Plane::from_vertices(
                self.source_vertices
                    .as_ref()
                    .expect("deferred polygon plane retains source vertices"),
            ));
            self.plane
                .set(plane)
                .expect("deferred polygon plane is initialized once");
        }
        Arc::make_mut(
            self.plane
                .get_mut()
                .expect("polygon plane was initialized before mutation"),
        )
    }
}

impl PartialEq for PolygonPlane {
    fn eq(&self, other: &Self) -> bool {
        **self == **other
    }
}

pub(crate) fn fresh_plane_id() -> u64 {
    reserve_plane_ids(1)
}

pub(crate) fn reserve_plane_ids(count: usize) -> u64 {
    NEXT_PLANE_ID.fetch_add(
        u64::try_from(count).expect("plane identity reservation fits u64"),
        Ordering::Relaxed,
    )
}

mod triangulation;

/// A polygon, defined by a list of vertices.
/// - `M` is the generic metadata type stored directly on the polygon. Use
///   `M = ()` for no metadata, or `M = Option<YourMetadata>` for optional metadata.
#[derive(Debug, Clone)]
pub struct Polygon<M: Clone> {
    pub(crate) vertices: PolygonVertices,
    pub(crate) plane: PolygonPlane,
    pub(crate) plane_id: u64,
    bounding_box: OnceLock<Arc<Aabb>>,
    certified_f64_bounds: OnceLock<Option<CertifiedF64Bounds>>,
    prepared_triangle_query: OnceLock<Option<Arc<PreparedTriangleQuery>>>,
    certified_nondegenerate: OnceLock<Option<bool>>,
    pub(crate) metadata: M,
}

/// Outward-rounded binary bounds certified to contain a polygon's exact
/// coordinates.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct CertifiedF64Bounds {
    pub(crate) min: [f64; 3],
    pub(crate) max: [f64; 3],
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct PreparedTriangleQuery {
    pub(crate) edge1: Vector3,
    pub(crate) edge2: Vector3,
    pub(crate) exact_x_axis: OnceLock<Option<PreparedExactXAxisTriangle>>,
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct PreparedExactXAxisTriangle {
    pub(crate) first: [Rational; 3],
    pub(crate) edge1: [Rational; 3],
    pub(crate) edge2: [Rational; 3],
    pub(crate) determinant_unit: Rational,
    pub(crate) normal: [Rational; 3],
    pub(crate) first_dot_normal: Rational,
}

/// Mutable access to a polygon's existing vertices.
///
/// Dropping this guard refreshes geometry derived from vertex positions.
pub struct PolygonVerticesMut<'a, M: Clone + Send + Sync> {
    polygon: &'a mut Polygon<M>,
    original_geometry: Option<Vec<(Point3, Vector3)>>,
}

impl<M: Clone + Send + Sync> Deref for PolygonVerticesMut<'_, M> {
    type Target = [Vertex];

    fn deref(&self) -> &Self::Target {
        &self.polygon.vertices
    }
}

impl<M: Clone + Send + Sync> DerefMut for PolygonVerticesMut<'_, M> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.polygon.vertices
    }
}

impl<M: Clone + Send + Sync> Drop for PolygonVerticesMut<'_, M> {
    fn drop(&mut self) {
        let mut position_changed = false;
        if let Some(original_geometry) = &self.original_geometry {
            for (vertex, (original_position, original_normal)) in
                self.polygon.vertices.iter_mut().zip(original_geometry)
            {
                let vertex_position_changed = vertex.position != *original_position;
                if vertex_position_changed || vertex.normal != *original_normal {
                    vertex.refresh_position_identity();
                }
                if vertex_position_changed {
                    position_changed = true;
                }
            }
        }
        if position_changed {
            self.polygon.plane_id = fresh_plane_id();
        }
        self.polygon.refresh_geometry();
    }
}

impl<M: Clone + PartialEq> PartialEq for Polygon<M> {
    fn eq(&self, other: &Self) -> bool {
        self.vertices == other.vertices
            && self.plane == other.plane
            && self.metadata == other.metadata
    }
}

impl<M: Clone + Send + Sync> Polygon<M> {
    /// Create a polygon from vertices
    pub fn new(vertices: Vec<Vertex>, metadata: M) -> Self {
        assert!(vertices.len() >= 3, "degenerate polygon");

        let vertices = PolygonVertices::new(vertices);
        Polygon {
            plane: PolygonPlane::from_vertices(&vertices),
            vertices,
            plane_id: fresh_plane_id(),
            bounding_box: OnceLock::new(),
            certified_f64_bounds: OnceLock::new(),
            prepared_triangle_query: OnceLock::new(),
            certified_nondegenerate: OnceLock::new(),
            metadata,
        }
    }

    pub(crate) fn from_lazy_indexed_quad(
        pool: Arc<LazySubdivisionVertexPool>,
        polygon_slot: usize,
        indices: [usize; 4],
        metadata: M,
        plane_id: u64,
    ) -> Self {
        let vertices = PolygonVertices::from_lazy_indexed_quad(pool, polygon_slot, indices);
        Self {
            plane: PolygonPlane::from_vertices(&vertices),
            vertices,
            plane_id,
            bounding_box: OnceLock::new(),
            certified_f64_bounds: OnceLock::new(),
            prepared_triangle_query: OnceLock::new(),
            certified_nondegenerate: OnceLock::new(),
            metadata,
        }
    }

    pub(crate) fn from_lazy_indexed_polygon(
        pool: Arc<LazySubdivisionVertexPool>,
        polygon_slot: usize,
        indices: Vec<usize>,
        metadata: M,
        plane_id: u64,
    ) -> Self {
        let vertices = PolygonVertices::from_lazy_indexed_polygon(pool, polygon_slot, indices);
        Self {
            plane: PolygonPlane::from_vertices(&vertices),
            vertices,
            plane_id,
            bounding_box: OnceLock::new(),
            certified_f64_bounds: OnceLock::new(),
            prepared_triangle_query: OnceLock::new(),
            certified_nondegenerate: OnceLock::new(),
            metadata,
        }
    }

    pub(crate) fn from_shared_vertices(
        buffer: Arc<Vec<Vertex>>,
        range: Range<usize>,
        metadata: M,
        plane_id: u64,
    ) -> Self {
        let vertices = PolygonVertices::from_shared(buffer, range);
        Self {
            plane: PolygonPlane::from_vertices(&vertices),
            vertices,
            plane_id,
            bounding_box: OnceLock::new(),
            certified_f64_bounds: OnceLock::new(),
            prepared_triangle_query: OnceLock::new(),
            certified_nondegenerate: OnceLock::new(),
            metadata,
        }
    }

    pub(crate) fn from_lazy_subdivision_triangle(
        pool: Arc<LazySubdivisionVertexPool>,
        triangle_slot: usize,
        indices: [usize; 3],
        metadata: M,
        plane_id: u64,
    ) -> Self {
        let vertices =
            PolygonVertices::from_lazy_subdivision_triangle(pool, triangle_slot, indices);
        Self {
            plane: PolygonPlane::from_vertices(&vertices),
            vertices,
            plane_id,
            bounding_box: OnceLock::new(),
            certified_f64_bounds: OnceLock::new(),
            prepared_triangle_query: OnceLock::new(),
            certified_nondegenerate: OnceLock::new(),
            metadata,
        }
    }

    /// Return this polygon with replacement metadata.
    pub fn with_metadata<NewM: Clone + Send + Sync>(self, metadata: NewM) -> Polygon<NewM> {
        Polygon {
            vertices: self.vertices,
            plane: self.plane,
            plane_id: self.plane_id,
            bounding_box: self.bounding_box,
            certified_f64_bounds: self.certified_f64_bounds,
            prepared_triangle_query: self.prepared_triangle_query,
            certified_nondegenerate: self.certified_nondegenerate,
            metadata,
        }
    }

    pub(crate) const fn with_plane_id(mut self, plane_id: u64) -> Self {
        self.plane_id = plane_id;
        self
    }

    /// Map this polygon's metadata while preserving its geometry.
    pub fn map_metadata<NewM: Clone + Send + Sync, F>(self, f: F) -> Polygon<NewM>
    where
        F: FnOnce(M) -> NewM,
    {
        Polygon {
            vertices: self.vertices,
            plane: self.plane,
            plane_id: self.plane_id,
            bounding_box: self.bounding_box,
            certified_f64_bounds: self.certified_f64_bounds,
            prepared_triangle_query: self.prepared_triangle_query,
            certified_nondegenerate: self.certified_nondegenerate,
            metadata: f(self.metadata),
        }
    }

    /// Axis aligned bounding box of this Polygon (cached after first call)
    pub fn bounding_box(&self) -> Aabb {
        self.bounding_box_ref().clone()
    }

    pub(crate) fn bounding_box_ref(&self) -> &Aabb {
        self.bounding_box
            .get_or_init(|| {
                let Some((mins, maxs)) =
                    point3_bounds(self.vertices.iter().map(|vertex| &vertex.position))
                else {
                    return Arc::new(Aabb::origin());
                };
                Arc::new(Aabb::new(mins, maxs))
            })
            .as_ref()
    }

    pub(crate) fn cached_bounding_box_ref(&self) -> Option<&Aabb> {
        self.bounding_box.get().map(Arc::as_ref)
    }

    pub(crate) fn certified_f64_bounds_ref(&self) -> Option<&CertifiedF64Bounds> {
        self.certified_f64_bounds
            .get_or_init(|| {
                self.vertices
                    .retained_certified_f64_bounds()
                    .or_else(|| certified_f64_bounds(&self.vertices))
            })
            .as_ref()
    }

    pub(crate) fn prepared_triangle_query_ref(&self) -> Option<&PreparedTriangleQuery> {
        self.prepared_triangle_query
            .get_or_init(|| prepared_triangle_query(&self.vertices).map(Arc::new))
            .as_deref()
    }

    #[cfg(feature = "stl-io")]
    pub(crate) fn certified_nondegenerate(&self) -> Option<bool> {
        *self
            .certified_nondegenerate
            .get_or_init(|| self.compute_certified_nondegenerate())
    }

    #[cfg(feature = "stl-io")]
    fn compute_certified_nondegenerate(&self) -> Option<bool> {
        let normal = if let Some(prepared) = self.prepared_triangle_query_ref() {
            prepared.edge1.cross(&prepared.edge2)
        } else {
            let first = self.plane.point_b.clone() - self.plane.point_a.clone();
            let second = self.plane.point_c.clone() - self.plane.point_a.clone();
            first.cross(&second)
        };
        let mut all_exact = true;
        for component in &normal.0 {
            match component.exact_rational_ref() {
                Some(rational) if !rational.is_zero() => return Some(true),
                Some(_) => {},
                None => all_exact = false,
            }
        }
        all_exact.then_some(false)
    }

    #[cfg(test)]
    pub(crate) fn has_cached_bounding_box(&self) -> bool {
        self.bounding_box.get().is_some()
    }

    /// Vertices in winding order.
    pub fn vertices(&self) -> &[Vertex] {
        &self.vertices
    }

    /// Borrow vertices in winding order without forcing indexed storage into
    /// an expanded contiguous buffer.
    pub fn vertex_iter(&self) -> impl ExactSizeIterator<Item = &Vertex> {
        self.vertices.borrowed_iter()
    }

    /// Returns retained finite approximations of vertex positions without
    /// forcing exact lazy transform recipes to materialize.
    ///
    /// This view is intended for rendering, export, and checksums. Geometry
    /// predicates continue to consume the exact [`Self::vertex_iter`] values.
    pub fn position_f64_iter(&self) -> impl ExactSizeIterator<Item = Option<[f64; 3]>> {
        self.vertices.position_f64_iter()
    }

    /// Mutably access vertex values while preserving derived geometry.
    ///
    /// The slice has a fixed length, so the polygon cannot become degenerate
    /// through this API. Its plane and cached bounds are refreshed on drop.
    pub fn vertices_mut(&mut self) -> PolygonVerticesMut<'_, M> {
        let original_geometry = self
            .vertices
            .iter()
            .map(|vertex| (vertex.position.clone(), vertex.normal.clone()))
            .collect();
        PolygonVerticesMut {
            polygon: self,
            original_geometry: Some(original_geometry),
        }
    }

    pub(crate) const fn vertices_mut_with_managed_identity(
        &mut self,
    ) -> PolygonVerticesMut<'_, M> {
        PolygonVerticesMut {
            polygon: self,
            original_geometry: None,
        }
    }

    /// Plane derived from the current vertices.
    pub fn plane(&self) -> &Plane {
        &self.plane
    }

    fn refresh_geometry(&mut self) {
        self.defer_plane_from_vertices();
        self.bounding_box = OnceLock::new();
        self.certified_f64_bounds = OnceLock::new();
        self.prepared_triangle_query = OnceLock::new();
        self.certified_nondegenerate = OnceLock::new();
    }

    pub(crate) fn defer_plane_from_vertices(&mut self) {
        self.plane = PolygonPlane::from_vertices(&self.vertices);
    }

    pub(crate) fn invalidate_bounding_box(&mut self) {
        self.bounding_box = OnceLock::new();
        self.certified_f64_bounds = OnceLock::new();
        self.prepared_triangle_query = OnceLock::new();
    }

    /// Reverses winding order, flips vertices normals, and flips the plane normal
    pub fn flip(&mut self) {
        // 1) reverse vertices
        self.vertices.reverse();
        // 2) flip all vertex normals
        for v in &mut self.vertices {
            v.flip();
        }
        // 3) defer the matching support plane until it is queried
        self.defer_plane_from_vertices();
        self.prepared_triangle_query = OnceLock::new();
    }

    /// Return an iterator over paired vertices each forming an edge of the polygon
    pub fn edges(&self) -> impl Iterator<Item = (&Vertex, &Vertex)> {
        self.vertices.iter().zip(self.vertices.iter().cycle().skip(1))
    }

    /// Return a normal calculated from all polygon vertices.
    ///
    /// The oriented area normal is evaluated with `hyperlattice::Vector3`.
    /// Exact-rational triangle crosses use a finite normalized view for the
    /// mesh's shading-normal attribute; symbolic inputs retain the exact
    /// checked normalization path. Topology predicates never consume this
    /// output attribute.
    pub fn calculate_new_normal(&self) -> Vector3 {
        let n = self.vertices.len();
        if n < 3 {
            return Vector3::z(); // degenerate or empty
        }

        if let Some(normal) = self.cached_new_normal() {
            return normal;
        }

        let normal = if let Some(prepared) = self.prepared_triangle_query_ref() {
            let area_normal = prepared.edge1.cross(&prepared.edge2);
            finite_normalized_exact_rational(&area_normal)
                .or_else(|| area_normal.normalize().ok())
                .unwrap_or_else(Vector3::z)
        } else {
            // Newell's oriented area vector already follows the polygon
            // winding. Recomputing and normalizing the support plane solely to
            // compare its direction duplicates the same exact
            // cross/square-root work.
            hyper_polygon_newell_normal(&self.vertices).unwrap_or_else(Vector3::z)
        };

        self.cache_new_normal(normal.clone());
        normal
    }

    pub(crate) fn calculate_normal_from_retained_area(
        &self,
        area_normal: &Vector3,
    ) -> Vector3 {
        if let Some(normal) = self.cached_new_normal() {
            return normal;
        }
        let normal = finite_normalized_exact_rational(area_normal)
            .or_else(|| area_normal.normalize().ok())
            .unwrap_or_else(Vector3::z);
        self.cache_new_normal(normal.clone());
        normal
    }

    pub(crate) fn cached_new_normal(&self) -> Option<Vector3> {
        POLYGON_NORMALS.with_borrow(|normals| {
            normals.get(&self.plane_id).and_then(|cached| {
                (cached.vertex_ids.len() == self.vertices.len()
                    && cached
                        .vertex_ids
                        .iter()
                        .zip(&self.vertices)
                        .all(|(id, vertex)| *id == vertex.position_id))
                .then(|| cached.normal.clone())
            })
        })
    }

    fn cache_new_normal(&self, normal: Vector3) {
        POLYGON_NORMALS.with_borrow_mut(|normals| {
            if normals.len() == POLYGON_NORMAL_CACHE_CAPACITY {
                normals.clear();
            }
            normals.insert(
                self.plane_id,
                CachedPolygonNormal {
                    vertex_ids: self
                        .vertices
                        .iter()
                        .map(|vertex| vertex.position_id)
                        .collect(),
                    normal,
                },
            );
        });
    }

    /// Recompute this polygon's normal from all vertices, then set all vertices' normals to match (flat shading).
    pub fn set_new_normal(&mut self) {
        // Assign each vertex's normal to match the plane
        let new_normal = self.calculate_new_normal();
        for v in &mut self.vertices {
            v.normal = new_normal.clone();
        }
    }

    /// Returns a reference to the metadata.
    pub const fn metadata(&self) -> &M {
        &self.metadata
    }

    /// Returns a mutable reference to the metadata.
    pub const fn metadata_mut(&mut self) -> &mut M {
        &mut self.metadata
    }

    /// Sets the metadata to the given value.
    pub fn set_metadata(&mut self, data: M) {
        self.metadata = data;
    }
}

fn point3_bounds<'a>(
    points: impl IntoIterator<Item = &'a Point3>,
) -> Option<(Point3, Point3)> {
    let mut points = points.into_iter();
    let first = points.next()?;
    let mut min_x = first.x.clone();
    let mut min_y = first.y.clone();
    let mut min_z = first.z.clone();
    let mut max_x = min_x.clone();
    let mut max_y = min_y.clone();
    let mut max_z = min_z.clone();

    for point in points {
        if super::real_lt(&point.x, &min_x) {
            min_x = point.x.clone();
        }
        if super::real_lt(&point.y, &min_y) {
            min_y = point.y.clone();
        }
        if super::real_lt(&point.z, &min_z) {
            min_z = point.z.clone();
        }
        if super::real_gt(&point.x, &max_x) {
            max_x = point.x.clone();
        }
        if super::real_gt(&point.y, &max_y) {
            max_y = point.y.clone();
        }
        if super::real_gt(&point.z, &max_z) {
            max_z = point.z.clone();
        }
    }

    Some((
        Point3::new(min_x, min_y, min_z),
        Point3::new(max_x, max_y, max_z),
    ))
}

fn prepared_triangle_query(vertices: &[Vertex]) -> Option<PreparedTriangleQuery> {
    if vertices.len() != 3 {
        return None;
    }
    let edge1 = vertices[1].position.clone() - vertices[0].position.clone();
    let edge2 = vertices[2].position.clone() - vertices[0].position.clone();
    Some(PreparedTriangleQuery {
        edge1,
        edge2,
        exact_x_axis: OnceLock::new(),
    })
}

fn certified_f64_bounds(vertices: &[Vertex]) -> Option<CertifiedF64Bounds> {
    let mut minimum = [f64::INFINITY; 3];
    let mut maximum = [f64::NEG_INFINITY; 3];
    for vertex in vertices {
        for (axis, coordinate) in [&vertex.position.x, &vertex.position.y, &vertex.position.z]
            .into_iter()
            .enumerate()
        {
            let [lower, upper] = coordinate.certified_dyadic_interval(-20)?;
            let lower = Real::from(lower).to_f64_lossy()?;
            let upper = Real::from(upper).to_f64_lossy()?;
            if !lower.is_finite() || !upper.is_finite() {
                return None;
            }
            minimum[axis] = minimum[axis].min(lower.next_down());
            maximum[axis] = maximum[axis].max(upper.next_up());
        }
    }
    Some(CertifiedF64Bounds {
        min: minimum,
        max: maximum,
    })
}

fn hyper_polygon_newell_normal(vertices: &[Vertex]) -> Option<Vector3> {
    let points = vertices
        .iter()
        .map(|vertex| vertex.position.to_vector())
        .collect::<Vec<_>>();
    let normal = points
        .iter()
        .zip(points.iter().cycle().skip(1))
        .fold(Vector3::zero(), |acc, (current, next)| {
            acc + current.cross(next)
        });
    normal.normalize().ok()
}

pub(crate) fn finite_normalized_exact_rational(vector: &Vector3) -> Option<Vector3> {
    if vector
        .0
        .iter()
        .any(|component| component.exact_rational_ref().is_none())
    {
        return None;
    }
    let components = [
        vector.0[0].to_f64_lossy()?,
        vector.0[1].to_f64_lossy()?,
        vector.0[2].to_f64_lossy()?,
    ];
    let magnitude = (components[0] * components[0]
        + components[1] * components[1]
        + components[2] * components[2])
        .sqrt();
    if !magnitude.is_finite() || magnitude == 0.0 {
        return None;
    }
    Some(Vector3::new([
        Real::try_from(components[0] / magnitude).ok()?,
        Real::try_from(components[1] / magnitude).ok()?,
        Real::try_from(components[2] / magnitude).ok()?,
    ]))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cloned_polygon_vertices_detach_on_mutation() {
        let polygon = Polygon::new(
            vec![
                Vertex::new(Point3::origin(), Vector3::z()),
                Vertex::new(
                    Point3::new(Real::one(), Real::zero(), Real::zero()),
                    Vector3::z(),
                ),
                Vertex::new(
                    Point3::new(Real::zero(), Real::one(), Real::zero()),
                    Vector3::z(),
                ),
            ],
            (),
        );
        let mut cloned = polygon.clone();
        assert!(Arc::ptr_eq(&polygon.vertices.buffer, &cloned.vertices.buffer));

        cloned.vertices_mut()[0].position.x = Real::from(2_u8);

        assert!(!Arc::ptr_eq(
            &polygon.vertices.buffer,
            &cloned.vertices.buffer
        ));
        assert_eq!(polygon.vertices[0].position.x, Real::zero());
        assert_eq!(cloned.vertices[0].position.x, Real::from(2_u8));
    }

    #[test]
    fn indexed_triangle_materializes_in_order_and_detaches_on_mutation() {
        let source_vertices = [0_u8, 2, 4]
            .into_iter()
            .map(|x| {
                Vertex::new(
                    Point3::new(Real::from(x), Real::zero(), Real::zero()),
                    Vector3::z(),
                )
            })
            .collect();
        let pool = Arc::new(LazySubdivisionVertexPool::new(
            source_vertices,
            vec![[0, 2]],
            1,
            crate::vertex::reserve_position_ids(4),
        ));
        let polygon = Polygon::from_lazy_subdivision_triangle(
            Arc::clone(&pool),
            0,
            [2, 0, 3],
            (),
            fresh_plane_id(),
        );
        let materialized = &pool.materialized_polygons[0];
        assert!(materialized.get().is_none());
        assert_eq!(
            polygon
                .vertices
                .iter()
                .map(|vertex| vertex.position.x.clone())
                .collect::<Vec<_>>(),
            [4_u8, 0, 2].map(Real::from)
        );
        assert!(materialized.get().is_some());

        let mut cloned = polygon.clone();
        cloned.vertices_mut()[0].position.x = Real::from(9_u8);
        assert_eq!(polygon.vertices[0].position.x, Real::from(4_u8));
        assert_eq!(cloned.vertices[0].position.x, Real::from(9_u8));
    }

    #[test]
    fn indexed_quad_materializes_in_order_and_detaches_on_mutation() {
        let source_vertices = [0_u8, 2, 4, 6]
            .into_iter()
            .map(|x| {
                Vertex::new(
                    Point3::new(Real::from(x), Real::zero(), Real::zero()),
                    Vector3::z(),
                )
            })
            .collect();
        let pool = Arc::new(LazySubdivisionVertexPool::new(
            source_vertices,
            Vec::new(),
            1,
            crate::vertex::reserve_position_ids(4),
        ));
        let polygon = Polygon::from_lazy_indexed_quad(
            Arc::clone(&pool),
            0,
            [3, 1, 0, 2],
            (),
            fresh_plane_id(),
        );
        let materialized = &pool.materialized_polygons[0];
        assert!(materialized.get().is_none());
        assert_eq!(
            polygon
                .vertices
                .iter()
                .map(|vertex| vertex.position.x.clone())
                .collect::<Vec<_>>(),
            [6_u8, 2, 0, 4].map(Real::from)
        );
        assert!(materialized.get().is_some());

        let mut cloned = polygon.clone();
        cloned.vertices_mut()[3].position.x = Real::from(9_u8);
        assert_eq!(polygon.vertices[3].position.x, Real::from(4_u8));
        assert_eq!(cloned.vertices[3].position.x, Real::from(9_u8));
    }
}
