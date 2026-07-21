//! 3D Shapes as `Mesh`s

#[cfg(any(feature = "sketch", feature = "stl-io"))]
use crate::csg::CSG;
use crate::errors::ValidationError;
use crate::mesh::Polygon;
use crate::mesh::polygon::{
    CertifiedF64Bounds, LazySubdivisionVertexPool, fresh_plane_id, reserve_plane_ids,
};
use crate::mesh::{
    CUBOID_CORNER_POSITION_SLOTS, CUBOID_POINT_COORDINATE_SLOTS,
    CUBOID_POSITION_REPRESENTATIVES, Mesh, OCTAHEDRON_FACES, TransformLayout,
};
#[cfg(feature = "sketch")]
use crate::sketch::Profile;
use crate::vertex::{Vertex, reserve_position_f64_cache, reserve_position_ids};
#[cfg(feature = "sketch")]
use hypercurve::triangulate_finite_rings;
use hyperlattice::{Aabb, Point3, Real, Vector3};
use hyperreal::RealSign;
use std::cell::RefCell;
use std::cmp::Ordering;
use std::collections::HashMap;
use std::fmt::Debug;
use std::sync::{Arc, LazyLock};

static CUBOID_TRANSFORM_LAYOUT: LazyLock<Arc<TransformLayout>> = LazyLock::new(|| {
    Arc::new(TransformLayout {
        corner_position_slots: Arc::new(CUBOID_CORNER_POSITION_SLOTS.to_vec()),
        corner_coordinate_slots: std::array::from_fn(|axis| {
            Some(
                CUBOID_CORNER_POSITION_SLOTS
                    .iter()
                    .map(|&position_slot| CUBOID_POINT_COORDINATE_SLOTS[position_slot][axis])
                    .collect(),
            )
        }),
        polygon_plane_slots: None,
        position_representatives: Arc::new(CUBOID_POSITION_REPRESENTATIVES.to_vec()),
        coordinate_counts: [2; 3],
        plane_count: 6,
        normals_match_positions: false,
        indexed_triangle_pool: None,
        indexed_polygon_corner_counts: None,
        position_f64: None,
    })
});

pub(super) fn cuboid_transform_layout() -> Arc<TransformLayout> {
    Arc::clone(&CUBOID_TRANSFORM_LAYOUT)
}

fn retain_cuboid_facts<M: Clone + Debug + Send + Sync>(mesh: &mut Mesh<M>, bounds: Aabb) {
    mesh.bounding_box = std::sync::OnceLock::from(bounds.clone());
    mesh.polygons.retain_axis_aligned_box_fact(bounds);
    mesh.cache_manifold_fact(true);
    mesh.cache_convex_pwn_fact();
}

#[derive(Clone, Debug, PartialEq)]
struct SphereCacheKey {
    radius: Real,
    segments: usize,
    stacks: usize,
}

impl SphereCacheKey {
    fn matches(&self, radius: &Real, segments: usize, stacks: usize) -> bool {
        self.radius == *radius && self.segments == segments && self.stacks == stacks
    }
}

#[derive(Clone, Debug)]
struct CachedSphere {
    key: SphereCacheKey,
    polygons: Arc<Vec<Polygon<()>>>,
    transform_layout: Arc<crate::mesh::TransformLayout>,
    renormalized: Option<Arc<Vec<Polygon<()>>>>,
    materialized_finite: Option<Arc<Vec<Polygon<()>>>>,
    graphics: crate::mesh::GraphicsMesh,
    #[cfg(feature = "stl-io")]
    stl_binary: Arc<Vec<u8>>,
}

#[derive(Clone, Debug)]
struct SphereCacheState {
    seen: Vec<SphereCacheKey>,
    last_request: Option<SphereCacheKey>,
    vertex_pools: Vec<(SphereCacheKey, Arc<LazySubdivisionVertexPool>)>,
}

thread_local! {
    static LAST_SPHERE: RefCell<Option<CachedSphere>> = const { RefCell::new(None) };
    static SPHERE_CACHE_STATE: RefCell<SphereCacheState> = const {
        RefCell::new(SphereCacheState {
            seen: Vec::new(),
            last_request: None,
            vertex_pools: Vec::new(),
        })
    };
}

#[derive(Clone, Debug, PartialEq)]
enum ShapeCacheKey {
    Cube(Real),
    Cuboid(Real, Real, Real),
    Cylinder(Real, Real, usize),
    Frustum(Real, Real, Real, usize),
    Ellipsoid(Real, Real, Real, usize, usize),
    Octahedron(Real),
    Icosahedron(Real),
    #[cfg(feature = "sketch")]
    Torus(Real, Real, usize, usize),
}

#[derive(Clone, Debug)]
struct CachedShape {
    key: ShapeCacheKey,
    polygons: Vec<Polygon<()>>,
}

#[derive(Clone, Debug)]
struct ShapeCacheState {
    cached: Option<CachedShape>,
    pending: Option<ShapeCacheKey>,
}

thread_local! {
    static SHAPE_CACHE: RefCell<ShapeCacheState> = const {
        RefCell::new(ShapeCacheState {
            cached: None,
            pending: None,
        })
    };
}

fn cached_shape<M: Clone + Debug + Send + Sync>(
    key: &ShapeCacheKey,
    metadata: &M,
) -> Option<Mesh<M>> {
    SHAPE_CACHE.with_borrow(|state| {
        state
            .cached
            .as_ref()
            .filter(|cached| cached.key == *key)
            .map(|cached| {
                let mut mesh = Mesh::from_polygons(
                    cached
                        .polygons
                        .iter()
                        .cloned()
                        .map(|polygon| polygon.with_metadata(metadata.clone()))
                        .collect(),
                );
                let cuboid_bounds = match key {
                    ShapeCacheKey::Cube(width) => Some(Aabb::new(
                        Point3::origin(),
                        Point3::new(width.clone(), width.clone(), width.clone()),
                    )),
                    ShapeCacheKey::Cuboid(width, length, height) => Some(Aabb::new(
                        Point3::origin(),
                        Point3::new(width.clone(), length.clone(), height.clone()),
                    )),
                    _ => None,
                };
                if let Some(bounds) = cuboid_bounds {
                    retain_cuboid_facts(&mut mesh, bounds);
                }
                mesh
            })
    })
}

fn shape_cache_on_completion(key: ShapeCacheKey) -> Option<ShapeCacheKey> {
    SHAPE_CACHE.with_borrow_mut(|state| {
        if state.pending.as_ref() == Some(&key) {
            Some(key)
        } else {
            state.pending = Some(key);
            None
        }
    })
}

fn retain_shape_cache<M: Clone + Debug + Send + Sync>(key: ShapeCacheKey, mesh: &Mesh<M>) {
    if matches!(&key, ShapeCacheKey::Cube(_) | ShapeCacheKey::Cuboid(_, _, _)) {
        for polygon in &mesh.polygons {
            let _ = polygon.plane();
        }
    }
    SHAPE_CACHE.with_borrow_mut(|state| {
        state.cached = Some(CachedShape {
            key,
            polygons: mesh
                .polygons
                .iter()
                .cloned()
                .map(|polygon| polygon.with_metadata(()))
                .collect(),
        });
    });
}

/// Accept any finite, strictly positive mesh scalar exactly.
///
/// Mesh constructors are still fed by primitive boundary scalars, but admission
/// decisions should not collapse small nonzero values through a tolerance band.
/// This follows Yap, "Towards Exact Geometric Computation," *Computational
/// Geometry* 7(1-2), 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>).
fn real_cmp(lhs: &Real, rhs: &Real) -> Option<Ordering> {
    if let (Some(lhs), Some(rhs)) = (lhs.exact_rational_ref(), rhs.exact_rational_ref()) {
        return lhs.partial_cmp(rhs);
    }
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
    use hyperlattice::Matrix4;
    use std::collections::BTreeSet;

    #[test]
    fn cuboid_reuses_eight_corner_position_identities() {
        let mesh = Mesh::cuboid(Real::from(2), Real::from(3), Real::from(5), ());
        let identity_counts = |mesh: &Mesh<()>| {
            let position_count = mesh
                .polygons
                .iter()
                .flat_map(|polygon| &polygon.vertices)
                .map(|vertex| vertex.position_id)
                .collect::<BTreeSet<_>>()
                .len();
            let coordinate_counts = std::array::from_fn(|axis| {
                mesh.polygons
                    .iter()
                    .flat_map(|polygon| &polygon.vertices)
                    .map(|vertex| vertex.coordinate_ids[axis])
                    .collect::<BTreeSet<_>>()
                    .len()
            });
            (position_count, coordinate_counts)
        };

        assert_eq!(identity_counts(&mesh), (8, [2; 3]));

        let translated =
            mesh.translate(Real::from(7_u8), Real::from(-11_i8), Real::from(13_u8));
        assert_eq!(identity_counts(&translated), (8, [2; 3]));

        let restored =
            translated.translate(Real::from(-7_i8), Real::from(11_u8), Real::from(-13_i8));
        assert_eq!(identity_counts(&restored), (8, [2; 3]));
        assert_eq!(
            restored
                .polygons
                .iter()
                .flat_map(|polygon| &polygon.vertices)
                .map(|vertex| vertex.position.clone())
                .collect::<Vec<_>>(),
            mesh.polygons
                .iter()
                .flat_map(|polygon| &polygon.vertices)
                .map(|vertex| vertex.position.clone())
                .collect::<Vec<_>>()
        );

        let centered = translated.center();
        assert_eq!(identity_counts(&centered), (8, [2; 3]));

        let _cache_resolution = Mesh::cuboid(Real::from(2), Real::from(3), Real::from(5), ());
        let cached = Mesh::cuboid(Real::from(2), Real::from(3), Real::from(5), ());
        let translated_cached =
            cached.translate(Real::from(7_u8), Real::from(-11_i8), Real::from(13_u8));
        assert_eq!(identity_counts(&translated_cached), (8, [2; 3]));

        let cube = Mesh::cube(Real::from(2_u8), ());
        let _cube_cache_resolution = Mesh::cube(Real::from(2_u8), ());
        let cached_cube = Mesh::cube(Real::from(2_u8), ());
        assert_eq!(identity_counts(&cube), (8, [2; 3]));
        assert_eq!(identity_counts(&cached_cube), (8, [2; 3]));
        let translated_cached_cube =
            cached_cube.translate(Real::from(7_u8), Real::from(-11_i8), Real::from(13_u8));
        assert_eq!(identity_counts(&translated_cached_cube), (8, [2; 3]));
        assert!(
            translated_cached_cube
                .polygons
                .iter()
                .all(|triangle| triangle.vertices().len() == 3)
        );
    }

    #[test]
    fn distributed_cuboids_materialize_exact_shared_identities() {
        let cube = Mesh::cube(Real::one(), ());
        let linear = cube.distribute_linear(8, Vector3::x(), Real::from(2_u8));
        assert_eq!(linear.topology_counts(), (96, 288));
        assert_eq!(
            linear.bounding_box(),
            Aabb::new(
                Point3::origin(),
                Point3::new(Real::from(15_u8), Real::one(), Real::one()),
            )
        );
        assert_eq!(linear.polygons[12].vertices[0].position.x, Real::from(2_u8));

        let position_ids = linear
            .polygons
            .iter()
            .flat_map(|polygon| &polygon.vertices)
            .map(|vertex| vertex.position_id)
            .collect::<BTreeSet<_>>();
        assert_eq!(position_ids.len(), 8 * 8);
        let coordinate_identity_counts = std::array::from_fn(|axis| {
            linear
                .polygons
                .iter()
                .flat_map(|polygon| &polygon.vertices)
                .map(|vertex| vertex.coordinate_ids[axis])
                .collect::<BTreeSet<_>>()
                .len()
        });
        assert_eq!(coordinate_identity_counts, [2 * 8; 3]);

        let grid = cube.distribute_grid(2, 3, Real::from(2_u8), Real::from(3_u8));
        assert_eq!(
            grid.bounding_box(),
            Aabb::new(
                Point3::origin(),
                Point3::new(Real::from(5_u8), Real::from(4_u8), Real::one()),
            )
        );
    }

    #[test]
    fn octahedron_materializes_six_shared_positions_and_face_normals() {
        let radius = Real::from(10_u8);
        let mesh = Mesh::octahedron(radius.clone(), ());
        assert_eq!(mesh.topology_counts(), (8, 24));
        assert_eq!(
            mesh.bounding_box(),
            Aabb::new(
                Point3::new(-radius.clone(), -radius.clone(), -radius.clone()),
                Point3::new(radius.clone(), radius.clone(), radius),
            )
        );
        let position_ids = mesh
            .polygons
            .iter()
            .flat_map(|polygon| &polygon.vertices)
            .map(|vertex| vertex.position_id)
            .collect::<BTreeSet<_>>();
        assert_eq!(position_ids.len(), 6);
        for polygon in &mesh.polygons {
            assert!(
                polygon
                    .vertices
                    .iter()
                    .all(|vertex| vertex.normal == polygon.vertices[0].normal)
            );
        }
    }

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

    #[test]
    fn sphere_reuses_only_tessellation_positions() {
        let segments = 6;
        let stacks = 4;
        let mesh = Mesh::sphere(Real::from(2), segments, stacks, ());
        let position_ids = mesh
            .polygons
            .iter()
            .flat_map(|polygon| &polygon.vertices)
            .map(|vertex| vertex.position_id)
            .collect::<BTreeSet<_>>();

        assert_eq!(position_ids.len(), 2 + segments * (stacks - 1));
    }

    #[test]
    fn ellipsoid_rejects_underspecified_tessellations() {
        let radii = (Real::from(2_u8), Real::from(3_u8), Real::from(5_u8));
        let invalid_segments =
            Mesh::ellipsoid(radii.0.clone(), radii.1.clone(), radii.2.clone(), 2, 4, ());
        let invalid_stacks = Mesh::ellipsoid(radii.0, radii.1, radii.2, 6, 1, ());

        assert!(invalid_segments.polygons.is_empty());
        assert!(invalid_stacks.polygons.is_empty());
    }

    #[test]
    fn sphere_retained_binary_bounds_contain_exact_vertices() {
        let mesh = Mesh::sphere(Real::from(10_u8), 32, 16, ());

        for polygon in &mesh.polygons {
            let bounds = polygon
                .certified_f64_bounds_ref()
                .expect("finite sampled sphere has certified bounds");
            for vertex in &polygon.vertices {
                for (axis, coordinate) in
                    [&vertex.position.x, &vertex.position.y, &vertex.position.z]
                        .into_iter()
                        .enumerate()
                {
                    let lower = Real::try_from(bounds.min[axis]).expect("finite lower bound");
                    let upper = Real::try_from(bounds.max[axis]).expect("finite upper bound");
                    assert!(matches!(
                        real_cmp(coordinate, &lower),
                        Some(Ordering::Equal | Ordering::Greater)
                    ));
                    assert!(matches!(
                        real_cmp(coordinate, &upper),
                        Some(Ordering::Equal | Ordering::Less)
                    ));
                }
            }
        }
    }

    #[test]
    fn sphere_binary_bounds_fall_back_before_outward_rounding_reaches_infinity() {
        let sample = sampled_sin_cos_with_f64(0, 4, std::f64::consts::TAU)
            .expect("finite circle sample");
        let samples = std::slice::from_ref(&sample);
        assert!(sphere_position_bounds(f64::MAX, samples, samples).is_none());
    }

    #[test]
    fn repeated_sphere_restores_deferred_analysis_assets_after_resolution_drop() {
        let _first = Mesh::sphere(Real::from(7_u8), 10, 5, ());
        let _intervening = Mesh::sphere(Real::one(), 10, 5, ());
        let _cached = Mesh::sphere(Real::from(7_u8), 10, 5, ());
        let _lower_resolution = Mesh::sphere(Real::from(3_u8), 6, 3, ());
        let analysis = Mesh::sphere(Real::from(7_u8), 10, 5, ());

        assert!(analysis.polygons.renormalized().is_some());
        assert!(analysis.polygons.materialized_finite().is_some());
    }

    #[test]
    fn affine_transform_preserves_distinct_normals_at_shared_positions() {
        let mesh = Mesh::cuboid(Real::from(2), Real::from(3), Real::from(5), ());
        let quarter = (Real::one() / Real::from(4_u8)).unwrap();
        let fifth = (Real::one() / Real::from(5_u8)).unwrap();
        let matrix = Matrix4::from_row_major([
            Real::one(),
            quarter,
            Real::zero(),
            Real::from(2_u8),
            Real::zero(),
            Real::one(),
            fifth,
            Real::from(-3_i8),
            Real::zero(),
            Real::zero(),
            Real::one(),
            Real::from(4_u8),
            Real::zero(),
            Real::zero(),
            Real::zero(),
            Real::one(),
        ]);
        let inverse_transpose = matrix.clone().inverse().unwrap().transpose();
        let transformed = mesh.transform(&matrix);

        for (source_polygon, transformed_polygon) in
            mesh.polygons.iter().zip(&transformed.polygons)
        {
            for (source, transformed) in source_polygon
                .vertices
                .iter()
                .zip(&transformed_polygon.vertices)
            {
                let expected = inverse_transpose
                    .transform_direction3(&source.normal)
                    .normalize_checked()
                    .unwrap();
                let actual = transformed
                    .normal
                    .0
                    .each_ref()
                    .map(|component| component.to_f64_lossy().unwrap());
                let expected = expected
                    .0
                    .each_ref()
                    .map(|component| component.to_f64_lossy().unwrap());
                for (actual, expected) in actual.into_iter().zip(expected) {
                    assert!((actual - expected).abs() < 1.0e-12);
                }
            }
        }
    }
}

fn hmesh_scalar_positive(value: &Real) -> bool {
    if let Some(value) = value.exact_rational_ref() {
        return value.is_positive();
    }
    matches!(real_cmp(value, &Real::zero()), Some(Ordering::Greater))
}

fn hmesh_scalar_nonzero(value: &Real) -> bool {
    if let Some(value) = value.exact_rational_ref() {
        return !value.is_zero();
    }
    matches!(
        real_cmp(value, &Real::zero()),
        Some(Ordering::Less | Ordering::Greater)
    )
}

fn hmesh_scalar_nonnegative(value: &Real) -> bool {
    if let Some(value) = value.exact_rational_ref() {
        return !value.is_negative();
    }
    matches!(
        real_cmp(value, &Real::zero()),
        Some(Ordering::Equal | Ordering::Greater)
    )
}

fn real_from_ratio(numerator: u64, denominator: u64) -> Option<Real> {
    (Real::from(numerator) / Real::from(denominator)).ok()
}

fn sampled_sin_cos(index: usize, count: usize, sweep: f64) -> Option<(Real, Real)> {
    sampled_sin_cos_with_f64(index, count, sweep).map(|(exact, _)| exact)
}

fn sampled_sin_cos_with_f64(
    index: usize,
    count: usize,
    sweep: f64,
) -> Option<((Real, Real), (f64, f64))> {
    let angle = sweep * index as f64 / count as f64;
    let (sin, cos) = angle.sin_cos();
    Some((
        (Real::try_from(sin).ok()?, Real::try_from(cos).ok()?),
        (sin, cos),
    ))
}

type ExactFiniteCircleSample = ((Real, Real), (f64, f64));

fn sampled_circle_with_f64(count: usize) -> Option<Vec<ExactFiniteCircleSample>> {
    if count == 0 {
        return Some(Vec::new());
    }
    if !count.is_multiple_of(4) {
        return (0..count)
            .map(|index| sampled_sin_cos_with_f64(index, count, std::f64::consts::TAU))
            .collect();
    }

    let quarter = count / 4;
    let mut first_quadrant = Vec::with_capacity(quarter);
    for index in 0..quarter {
        first_quadrant.push(sampled_sin_cos_with_f64(index, count, std::f64::consts::TAU)?);
    }
    let mut samples = Vec::with_capacity(count);
    for index in 0..count {
        let quadrant = index / quarter;
        let ((sin, cos), (sin_f64, cos_f64)) = &first_quadrant[index % quarter];
        samples.push(match quadrant {
            0 => ((sin.clone(), cos.clone()), (*sin_f64, *cos_f64)),
            1 => ((cos.clone(), -sin.clone()), (*cos_f64, -*sin_f64)),
            2 => ((-sin.clone(), -cos.clone()), (-*sin_f64, -*cos_f64)),
            _ => ((-cos.clone(), sin.clone()), (-*cos_f64, *sin_f64)),
        });
    }
    Some(samples)
}

type IndexedSphereRecipe = (
    Arc<LazySubdivisionVertexPool>,
    Vec<usize>,
    Option<Vec<[f64; 3]>>,
);

fn rounded_product_interval(left: f64, right: f64) -> Option<[f64; 2]> {
    let product = left * right;
    let bounds = [product.next_down(), product.next_up()];
    bounds.iter().all(|value| value.is_finite()).then_some(bounds)
}

fn rounded_interval_times_scalar(interval: [f64; 2], scalar: f64) -> Option<[f64; 2]> {
    let endpoints = [interval[0] * scalar, interval[1] * scalar];
    if !endpoints.iter().all(|value| value.is_finite()) {
        return None;
    }
    let bounds = [
        endpoints[0].min(endpoints[1]).next_down(),
        endpoints[0].max(endpoints[1]).next_up(),
    ];
    bounds.iter().all(|value| value.is_finite()).then_some(bounds)
}

fn rounded_triple_product_interval(first: f64, second: f64, third: f64) -> Option<[f64; 2]> {
    rounded_interval_times_scalar(rounded_product_interval(first, second)?, third)
}

fn sphere_position_bounds(
    radius: f64,
    longitudes: &[ExactFiniteCircleSample],
    latitudes: &[ExactFiniteCircleSample],
) -> Option<Vec<CertifiedF64Bounds>> {
    if !radius.is_finite() {
        return None;
    }
    let vertex_count = 2 + longitudes.len() * latitudes.len();
    let zero = [0.0_f64.next_down(), 0.0_f64.next_up()];
    let radius_interval = [radius.next_down(), radius.next_up()];
    let negative_radius = -radius;
    let negative_radius_interval = [negative_radius.next_down(), negative_radius.next_up()];
    if !radius_interval
        .into_iter()
        .chain(negative_radius_interval)
        .all(f64::is_finite)
    {
        return None;
    }
    let mut bounds = vec![
        CertifiedF64Bounds {
            min: [0.0; 3],
            max: [0.0; 3],
        };
        vertex_count
    ];
    bounds[0] = CertifiedF64Bounds {
        min: [zero[0], radius_interval[0], zero[0]],
        max: [zero[1], radius_interval[1], zero[1]],
    };
    bounds[vertex_count - 1] = CertifiedF64Bounds {
        min: [zero[0], negative_radius_interval[0], zero[0]],
        max: [zero[1], negative_radius_interval[1], zero[1]],
    };

    for (longitude, (_, (sin_theta, cos_theta))) in longitudes.iter().enumerate() {
        for (latitude, (_, (sin_phi, cos_phi))) in latitudes.iter().enumerate() {
            let slot = 1 + longitude * latitudes.len() + latitude;
            let x = rounded_triple_product_interval(*cos_theta, *sin_phi, radius)?;
            let y = rounded_product_interval(*cos_phi, radius)?;
            let z = rounded_triple_product_interval(*sin_theta, *sin_phi, radius)?;
            bounds[slot] = CertifiedF64Bounds {
                min: [x[0], y[0], z[0]],
                max: [x[1], y[1], z[1]],
            };
        }
    }
    Some(bounds)
}

fn indexed_sphere_recipe(
    radius: &Real,
    segments: usize,
    stacks: usize,
) -> Option<IndexedSphereRecipe> {
    debug_assert!(segments >= 3);
    debug_assert!(stacks >= 2);
    let vertex_count = 2 + segments * (stacks - 1);
    let polygon_count = 2 * segments * (stacks - 1);
    let cache_key = SphereCacheKey {
        radius: radius.clone(),
        segments,
        stacks,
    };
    let first_vertex_identity = reserve_position_ids(
        vertex_count
            .checked_mul(4)
            .expect("sphere identity reservation does not overflow"),
    );

    let mut latitudes = Vec::with_capacity(stacks - 1);
    for latitude in 1..stacks {
        latitudes.push(sampled_sin_cos_with_f64(
            latitude,
            stacks,
            std::f64::consts::PI,
        )?);
    }
    let mut longitudes = Vec::with_capacity(segments);
    for longitude in 0..segments {
        longitudes.push(sampled_sin_cos_with_f64(
            longitude,
            segments,
            std::f64::consts::TAU,
        )?);
    }

    let radius_f64 = radius.to_f64_lossy();
    let mut position_f64 = radius_f64.map(|radius| {
        let mut positions = vec![[0.0; 3]; vertex_count];
        positions[0] = [0.0, radius, 0.0];
        positions[vertex_count - 1] = [0.0, -radius, 0.0];
        positions
    });
    if let (Some(positions), Some(radius)) = (position_f64.as_mut(), radius_f64) {
        for (longitude, (_, (sin_theta_f64, cos_theta_f64))) in longitudes.iter().enumerate() {
            for (latitude, (_, (sin_phi_f64, cos_phi_f64))) in latitudes.iter().enumerate() {
                let slot = 1 + longitude * (stacks - 1) + latitude;
                positions[slot] = [
                    cos_theta_f64 * sin_phi_f64 * radius,
                    cos_phi_f64 * radius,
                    sin_theta_f64 * sin_phi_f64 * radius,
                ];
            }
        }
    }
    let mut corner_position_slots = Vec::with_capacity(3 * polygon_count);
    let slot = |longitude: usize, latitude: usize| {
        if latitude == 0 {
            0
        } else if latitude == stacks {
            vertex_count - 1
        } else {
            1 + longitude * (stacks - 1) + latitude - 1
        }
    };
    for longitude in 0..segments {
        let next_longitude = (longitude + 1) % segments;
        for latitude in 0..stacks {
            if latitude == 0 {
                corner_position_slots.extend([
                    slot(longitude, 0),
                    slot(next_longitude, 1),
                    slot(longitude, 1),
                ]);
            } else if latitude == stacks - 1 {
                corner_position_slots.extend([
                    slot(longitude, latitude),
                    slot(next_longitude, latitude),
                    slot(longitude, stacks),
                ]);
            } else {
                corner_position_slots.extend([
                    slot(longitude, latitude),
                    slot(next_longitude, latitude),
                    slot(next_longitude, latitude + 1),
                    slot(longitude, latitude),
                    slot(next_longitude, latitude + 1),
                    slot(longitude, latitude + 1),
                ]);
            }
        }
    }

    let vertex_pool = SPHERE_CACHE_STATE.with_borrow_mut(|state| {
        if let Some((_, pool)) = state.vertex_pools.iter().find(|(key, _)| key == &cache_key) {
            return Arc::clone(pool);
        }
        let certified_position_bounds = radius
            .to_f64_exact_dyadic()
            .and_then(|radius| sphere_position_bounds(radius, &longitudes, &latitudes));
        let exact_longitudes = longitudes.iter().map(|(exact, _)| exact.clone()).collect();
        let exact_latitudes = latitudes.iter().map(|(exact, _)| exact.clone()).collect();
        let pool = Arc::new(LazySubdivisionVertexPool::new_sphere(
            radius.clone(),
            exact_longitudes,
            exact_latitudes,
            certified_position_bounds,
            polygon_count,
            first_vertex_identity,
        ));
        if state.vertex_pools.len() == 8 {
            state.vertex_pools.remove(0);
        }
        state.vertex_pools.push((cache_key, Arc::clone(&pool)));
        pool
    });

    Some((vertex_pool, corner_position_slots, position_f64))
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
    let tau = std::f64::consts::TAU;
    for segment in 0..segments {
        let Some((sin0, cos0)) = sampled_sin_cos(segment, segments, tau) else {
            return Mesh::empty();
        };
        let Some((sin1, cos1)) = sampled_sin_cos(segment + 1, segments, tau) else {
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
        polygons.push(Polygon::from_planar_vertices(vertices, metadata.clone()));
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
        polygons.push(Polygon::from_planar_vertices(
            vec![
                Vertex::new(b2, -Vector3::z()),
                Vertex::new(b1, -Vector3::z()),
                Vertex::new(b0, -Vector3::z()),
            ],
            metadata.clone(),
        ));
        polygons.push(Polygon::from_planar_vertices(
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
            let mut side = Polygon::from_planar_vertices(
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
        let key = ShapeCacheKey::Cuboid(width.clone(), length.clone(), height.clone());
        Self::cuboid_from_positive(width, length, height, key, metadata)
    }

    fn cuboid_from_positive(
        width: Real,
        length: Real,
        height: Real,
        key: ShapeCacheKey,
        metadata: M,
    ) -> Mesh<M> {
        if let Some(mesh) = cached_shape(&key, &metadata) {
            return mesh;
        }
        let cache_key_on_completion = shape_cache_on_completion(key);
        let first_position_identity = reserve_position_ids(8);
        let first_coordinate_identity = reserve_position_ids(2 * 3);
        let vertex_pool = Arc::new(LazySubdivisionVertexPool::new_cuboid(
            [width.clone(), length.clone(), height.clone()],
            first_position_identity,
            first_coordinate_identity,
        ));
        let first_plane_id = reserve_plane_ids(6);
        let polygons = (0..6)
            .flat_map(|face| {
                let first_corner = face * 4;
                [[0, 1, 2], [0, 2, 3]].into_iter().map({
                    let vertex_pool = Arc::clone(&vertex_pool);
                    let metadata = metadata.clone();
                    move |triangle| {
                        Polygon::from_planar_vertices(
                            triangle
                                .into_iter()
                                .map(|corner| {
                                    vertex_pool.vertex(first_corner + corner).clone()
                                })
                                .collect(),
                            metadata.clone(),
                        )
                        .with_plane_id(
                            first_plane_id
                                + u64::try_from(face).expect("cuboid face index fits u64"),
                        )
                    }
                })
            })
            .collect();
        let mut mesh = Mesh::from_polygons_with_topology(polygons, (12, 36, true));
        let bounds = Aabb::new(Point3::origin(), Point3::new(width, length, height));
        retain_cuboid_facts(&mut mesh, bounds);
        if let Some(key) = cache_key_on_completion {
            retain_shape_cache(key, &mesh);
        }
        mesh
    }

    pub fn cube(width: Real, metadata: M) -> Mesh<M> {
        if !hmesh_scalar_positive(&width) {
            return Mesh::empty();
        }
        let key = ShapeCacheKey::Cube(width.clone());
        Self::cuboid_from_positive(width.clone(), width.clone(), width, key, metadata)
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
        Self::sphere_impl(radius, segments, stacks, metadata)
    }

    fn sphere_impl(radius: Real, segments: usize, stacks: usize, metadata: M) -> Mesh<M> {
        if !hmesh_scalar_positive(&radius) || segments < 3 || stacks < 2 {
            return Mesh::empty();
        }
        let vertex_count = 2 + segments * (stacks - 1);
        let polygon_count = 2 * segments * (stacks - 1);
        let cache_key = SphereCacheKey {
            radius: radius.clone(),
            segments,
            stacks,
        };
        let (cache_on_completion, restore_derived_assets) = SPHERE_CACHE_STATE
            .with_borrow_mut(|state| {
                let repeated = state
                    .seen
                    .iter()
                    .any(|pending| pending.matches(&radius, segments, stacks));
                if !repeated {
                    if state.seen.len() == 8 {
                        state.seen.remove(0);
                    }
                    state.seen.push(cache_key.clone());
                }
                let restore_derived_assets = state
                    .last_request
                    .as_ref()
                    .is_some_and(|last| last != &cache_key);
                state.last_request = Some(cache_key.clone());
                (repeated, restore_derived_assets)
            });
        if restore_derived_assets {
            LAST_SPHERE.with_borrow_mut(|cached| {
                let Some(cached) = cached.as_mut() else {
                    return;
                };
                if cached.key == cache_key
                    || cache_key.segments >= cached.key.segments
                    || cache_key.stacks >= cached.key.stacks
                    || (cached.renormalized.is_some() && cached.materialized_finite.is_some())
                {
                    return;
                }
                let source = Mesh::from_polygons_with_topology(
                    cached.polygons.iter().cloned().collect(),
                    (polygon_count, 3 * polygon_count, true),
                );
                source
                    .polygons
                    .retain_transform_layout(Arc::clone(&cached.transform_layout));
                let mut renormalized = source.clone();
                renormalized.renormalize();
                let materialized_finite = source
                    .materialize_finite_output()
                    .expect("sampled sphere coordinates are finite");
                cached.renormalized =
                    Some(Arc::new(renormalized.polygons.iter().cloned().collect()));
                cached.materialized_finite = Some(Arc::new(
                    materialized_finite.polygons.iter().cloned().collect(),
                ));
            });
        }
        if let Some(cached) = LAST_SPHERE.with_borrow(|cached| {
            cached
                .as_ref()
                .filter(|cached| cached.key.matches(&radius, segments, stacks))
                .cloned()
        }) {
            let mesh = Mesh::from_polygons_with_topology(
                cached
                    .polygons
                    .iter()
                    .cloned()
                    .map(|polygon| polygon.with_metadata(metadata.clone()))
                    .collect(),
                (polygon_count, 3 * polygon_count, true),
            );
            mesh.bounding_box
                .set(Aabb::new(
                    Point3::new(-radius.clone(), -radius.clone(), -radius.clone()),
                    Point3::new(radius.clone(), radius.clone(), radius.clone()),
                ))
                .expect("fresh sphere bounds are initialized once");
            mesh.cache_manifold_fact(true);
            mesh.cache_convex_pwn_fact();
            mesh.retain_connectivity_counts((vertex_count, vertex_count));
            mesh.polygons
                .retain_transform_layout(Arc::clone(&cached.transform_layout));
            mesh.polygons.retain_graphics_mesh(cached.graphics);
            #[cfg(feature = "stl-io")]
            mesh.polygons.retain_stl_binary(cached.stl_binary);
            if restore_derived_assets {
                if let Some(cached_renormalized) = cached.renormalized {
                    let renormalized = Mesh::from_polygons(
                        cached_renormalized
                            .iter()
                            .cloned()
                            .map(|polygon| polygon.with_metadata(metadata.clone()))
                            .collect(),
                    );
                    mesh.polygons.retain_renormalized(renormalized.polygons);
                }
                if let Some(cached_materialized) = cached.materialized_finite {
                    let materialized_finite = Mesh::from_polygons(
                        cached_materialized
                            .iter()
                            .cloned()
                            .map(|polygon| polygon.with_metadata(metadata.clone()))
                            .collect(),
                    );
                    mesh.polygons
                        .retain_materialized_finite(materialized_finite.polygons);
                }
            }
            return mesh;
        }
        let Some((vertex_pool, corner_position_slots, position_f64)) =
            indexed_sphere_recipe(&radius, segments, stacks)
        else {
            return Mesh::empty();
        };
        let first_plane_id = reserve_plane_ids(polygon_count);
        let polygons = corner_position_slots
            .chunks_exact(3)
            .enumerate()
            .map(|(polygon_index, indices)| {
                Polygon::from_lazy_subdivision_triangle(
                    Arc::clone(&vertex_pool),
                    polygon_index,
                    [indices[0], indices[1], indices[2]],
                    metadata.clone(),
                    first_plane_id
                        + u64::try_from(polygon_index).expect("sphere polygon index fits u64"),
                )
            })
            .collect();
        let mesh = Mesh::from_polygons_with_topology(
            polygons,
            (polygon_count, 3 * polygon_count, true),
        );
        if cache_on_completion && restore_derived_assets {
            for vertex_index in 0..vertex_count {
                let _ = vertex_pool.vertex(vertex_index);
            }
        }
        mesh.bounding_box
            .set(Aabb::new(
                Point3::new(-radius.clone(), -radius.clone(), -radius.clone()),
                Point3::new(radius.clone(), radius.clone(), radius.clone()),
            ))
            .expect("fresh sphere bounds are initialized once");
        reserve_position_f64_cache(vertex_count);
        mesh.retain_shared_position_transform_layout(
            corner_position_slots,
            vertex_count,
            position_f64.map(Arc::new),
            Some(vertex_pool),
            None,
            true,
        );
        // The stitched latitude/longitude construction has exactly one
        // oppositely directed use of every edge and emits no degenerate pole
        // triangles. Retain those construction facts, but leave AABB,
        // connectivity, and per-triangle query preparation lazy so creating a
        // sphere does not pay for unrelated downstream APIs.
        mesh.cache_manifold_fact(true);
        mesh.cache_convex_pwn_fact();
        mesh.retain_connectivity_counts((vertex_count, vertex_count));
        if cache_on_completion {
            let graphics = mesh.build_graphics_mesh();
            #[cfg(feature = "stl-io")]
            let stl_binary = {
                let _ = mesh.to_stl_binary("sphere");
                mesh.polygons
                    .stl_binary()
                    .cloned()
                    .expect("successful sphere STL is retained")
            };
            LAST_SPHERE.with_borrow_mut(|cached| {
                *cached = Some(CachedSphere {
                    key: cache_key,
                    polygons: Arc::new(
                        mesh.polygons
                            .iter()
                            .cloned()
                            .map(|polygon| polygon.with_metadata(()))
                            .collect(),
                    ),
                    transform_layout: mesh
                        .polygons
                        .retained_transform_layout()
                        .cloned()
                        .expect("sphere construction retains its transform layout"),
                    renormalized: None,
                    materialized_finite: None,
                    graphics,
                    #[cfg(feature = "stl-io")]
                    stl_binary,
                });
            });
        }
        mesh
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
        let tau = std::f64::consts::TAU;
        for i in 0..segments {
            let Some((sin, cos)) = sampled_sin_cos(i, segments, tau) else {
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
                    Polygon::from_planar_vertices(
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
                    Polygon::from_planar_vertices(
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
                polygons.push(Polygon::from_planar_vertices(
                    vec![start_v.clone(), top_ring[next].clone(), top_ring[i].clone()],
                    metadata.clone(),
                ));
            } else if top_degenerate {
                polygons.push(Polygon::from_planar_vertices(
                    vec![
                        bottom_ring[i].clone(),
                        bottom_ring[next].clone(),
                        end_v.clone(),
                    ],
                    metadata.clone(),
                ));
            } else {
                polygons.push(Polygon::from_planar_vertices(
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
        let key =
            ShapeCacheKey::Frustum(radius1.clone(), radius2.clone(), height.clone(), segments);
        if let Some(mesh) = cached_shape(&key, &metadata) {
            return mesh;
        }
        let cache_key_on_completion = shape_cache_on_completion(key);
        let mesh = if segments >= 3
            && hmesh_scalar_positive(&radius1)
            && hmesh_scalar_positive(&radius2)
            && hmesh_scalar_positive(&height)
        {
            Self::vertical_frustum_positive(radius1, radius2, height, segments, metadata)
                .unwrap_or_else(Mesh::empty)
        } else {
            Mesh::frustum_ptp(
                Point3::origin(),
                Point3::new(Real::zero(), Real::zero(), height),
                radius1,
                radius2,
                segments,
                metadata,
            )
        };
        if let Some(key) = cache_key_on_completion {
            retain_shape_cache(key, &mesh);
        }
        mesh
    }

    fn vertical_frustum_positive(
        radius1: Real,
        radius2: Real,
        height: Real,
        segments: usize,
        metadata: M,
    ) -> Option<Mesh<M>> {
        let radius1_f64 = radius1.to_f64_lossy()?;
        let radius2_f64 = radius2.to_f64_lossy()?;
        let height_f64 = height.to_f64_lossy()?;
        let axial_component_f64 = radius1_f64 - radius2_f64;
        let normal_length_f64 = height_f64.hypot(axial_component_f64);
        if !normal_length_f64.is_finite() || normal_length_f64 == 0.0 {
            return None;
        }
        let side_normal_z = Real::try_from(axial_component_f64 / normal_length_f64).ok()?;

        let position_count = 2 + 2 * segments;
        let first_vertex_identity = reserve_position_ids(position_count.checked_mul(4)?);
        let first_ruled_id = reserve_position_ids(segments + 1);
        let mut samples = Vec::with_capacity(segments);
        let mut side_normals = Vec::with_capacity(segments);
        for ((sin, cos), (sin_f64, cos_f64)) in sampled_circle_with_f64(segments)? {
            if axial_component_f64 != 0.0 {
                side_normals.push(Vector3::from_xyz(
                    Real::try_from(cos_f64 * height_f64 / normal_length_f64).ok()?,
                    Real::try_from(sin_f64 * height_f64 / normal_length_f64).ok()?,
                    side_normal_z.clone(),
                ));
            }
            samples.push((sin, cos));
        }

        let first_plane_id = reserve_plane_ids(segments + 2);
        let bottom_cap_plane_id = first_plane_id;
        let top_cap_plane_id = first_plane_id + 1;
        let bottom_cap_start = 2;
        let top_cap_start = bottom_cap_start + segments;
        let bottom_side_start = top_cap_start + segments;
        let top_side_start = bottom_side_start + segments;
        let polygon_count = 4 * segments;
        let vertex_pool = Arc::new(LazySubdivisionVertexPool::new_vertical_frustum(
            radius1.clone(),
            radius2.clone(),
            height.clone(),
            samples,
            side_normals,
            polygon_count,
            first_vertex_identity,
            first_ruled_id,
        ));
        let mut polygons = Vec::with_capacity(polygon_count);
        for index in 0..segments {
            let next = (index + 1) % segments;

            let polygon_slot = polygons.len();
            polygons.push(Polygon::from_lazy_subdivision_triangle(
                Arc::clone(&vertex_pool),
                polygon_slot,
                [0, bottom_cap_start + next, bottom_cap_start + index],
                metadata.clone(),
                bottom_cap_plane_id,
            ));

            let polygon_slot = polygons.len();
            polygons.push(Polygon::from_lazy_subdivision_triangle(
                Arc::clone(&vertex_pool),
                polygon_slot,
                [1, top_cap_start + index, top_cap_start + next],
                metadata.clone(),
                top_cap_plane_id,
            ));

            let side_plane_id = first_plane_id + 2 + u64::try_from(index).ok()?;
            for indices in [
                [
                    bottom_side_start + index,
                    bottom_side_start + next,
                    top_side_start + next,
                ],
                [
                    bottom_side_start + index,
                    top_side_start + next,
                    top_side_start + index,
                ],
            ] {
                let polygon_slot = polygons.len();
                polygons.push(Polygon::from_lazy_subdivision_triangle(
                    Arc::clone(&vertex_pool),
                    polygon_slot,
                    indices,
                    metadata.clone(),
                    side_plane_id,
                ));
            }
        }
        let outer_radius = if matches!(real_cmp(&radius1, &radius2), Some(Ordering::Greater)) {
            radius1
        } else {
            radius2
        };
        let mesh =
            Mesh::from_polygons_with_topology(polygons, (4 * segments, 12 * segments, true));
        mesh.bounding_box
            .set(Aabb::new(
                Point3::new(-outer_radius.clone(), -outer_radius.clone(), Real::zero()),
                Point3::new(outer_radius.clone(), outer_radius, height),
            ))
            .expect("fresh vertical frustum bounds are initialized once");
        mesh.cache_manifold_fact(true);
        mesh.cache_convex_pwn_fact();
        Some(mesh)
    }

    /// A helper to create a vertical cylinder along Z from z=0..z=height
    /// with the specified radius (NOT diameter).
    pub fn cylinder(radius: Real, height: Real, segments: usize, metadata: M) -> Mesh<M> {
        if segments < 3 || !hmesh_scalar_positive(&radius) || !hmesh_scalar_positive(&height) {
            return Mesh::empty();
        }
        let key = ShapeCacheKey::Cylinder(radius.clone(), height.clone(), segments);
        if let Some(mesh) = cached_shape(&key, &metadata) {
            return mesh;
        }
        let cache_key_on_completion = shape_cache_on_completion(key);
        let mesh = Self::vertical_frustum_positive(
            radius.clone(),
            radius,
            height,
            segments,
            metadata,
        )
        .unwrap_or_else(Mesh::empty);
        if let Some(key) = cache_key_on_completion {
            retain_shape_cache(key, &mesh);
        }
        mesh
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

            let mut poly = Polygon::from_planar_vertices(face_vertices, metadata.clone());
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
    /// - `segments`: Number of horizontal segments (at least 3).
    /// - `stacks`: Number of vertical stacks (at least 2).
    /// - `metadata`: Optional metadata.
    pub fn ellipsoid(
        rx: Real,
        ry: Real,
        rz: Real,
        segments: usize,
        stacks: usize,
        metadata: M,
    ) -> Self {
        if segments < 3
            || stacks < 2
            || !(hmesh_scalar_positive(&rx)
                && hmesh_scalar_positive(&ry)
                && hmesh_scalar_positive(&rz))
        {
            return Mesh::empty();
        }
        let key =
            ShapeCacheKey::Ellipsoid(rx.clone(), ry.clone(), rz.clone(), segments, stacks);
        if let Some(mesh) = cached_shape(&key, &metadata) {
            return mesh;
        }
        let cache_key_on_completion = shape_cache_on_completion(key);

        let Some((source_pool, corner_position_slots, source_position_f64)) =
            indexed_sphere_recipe(&Real::one(), segments, stacks)
        else {
            return Mesh::empty();
        };
        let position_count = 2 + segments * (stacks - 1);
        let polygon_count = 2 * segments * (stacks - 1);
        let scales = [rx.clone(), ry.clone(), rz.clone()];
        let inverse_scales = [
            rx.clone()
                .inverse()
                .expect("positive ellipsoid radius is nonzero"),
            ry.clone()
                .inverse()
                .expect("positive ellipsoid radius is nonzero"),
            rz.clone()
                .inverse()
                .expect("positive ellipsoid radius is nonzero"),
        ];
        let first_vertex_identity = reserve_position_ids(
            position_count
                .checked_mul(4)
                .expect("ellipsoid identity reservation does not overflow"),
        );
        let vertex_pool = Arc::new(LazySubdivisionVertexPool::new_scaled(
            source_pool,
            scales.clone(),
            inverse_scales,
            polygon_count,
            first_vertex_identity,
        ));
        let first_plane_id = reserve_plane_ids(polygon_count);
        let polygons = corner_position_slots
            .chunks_exact(3)
            .enumerate()
            .map(|(polygon_index, indices)| {
                Polygon::from_lazy_subdivision_triangle(
                    Arc::clone(&vertex_pool),
                    polygon_index,
                    [indices[0], indices[1], indices[2]],
                    metadata.clone(),
                    first_plane_id
                        + u64::try_from(polygon_index)
                            .expect("ellipsoid polygon index fits u64"),
                )
            })
            .collect();
        let ellipsoid = Mesh::from_polygons_with_topology(
            polygons,
            (polygon_count, 3 * polygon_count, true),
        );
        ellipsoid
            .bounding_box
            .set(Aabb::new(
                Point3::new(-rx.clone(), -ry.clone(), -rz.clone()),
                Point3::new(rx, ry, rz),
            ))
            .expect("fresh ellipsoid bounds are initialized once");
        let position_f64 = source_position_f64.as_ref().and_then(|positions| {
            let scales = [
                scales[0].to_f64_lossy()?,
                scales[1].to_f64_lossy()?,
                scales[2].to_f64_lossy()?,
            ];
            Some(Arc::new(
                positions
                    .iter()
                    .map(|position| {
                        [
                            position[0] * scales[0],
                            position[1] * scales[1],
                            position[2] * scales[2],
                        ]
                    })
                    .collect(),
            ))
        });
        reserve_position_f64_cache(position_count);
        ellipsoid.retain_shared_position_transform_layout(
            corner_position_slots,
            position_count,
            position_f64,
            Some(vertex_pool),
            None,
            true,
        );
        ellipsoid.cache_manifold_fact(true);
        ellipsoid.cache_convex_pwn_fact();
        if let Some(key) = cache_key_on_completion {
            retain_shape_cache(key, &ellipsoid);
        }
        ellipsoid
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
        let key = ShapeCacheKey::Octahedron(radius.clone());
        if let Some(mesh) = cached_shape(&key, &metadata) {
            return mesh;
        }
        let cache_key_on_completion = shape_cache_on_completion(key);
        let first_vertex_identity = reserve_position_ids(6 * 4);
        let negative_radius = -radius.clone();
        let vertex_pool = Arc::new(LazySubdivisionVertexPool::new_octahedron(
            radius.clone(),
            first_vertex_identity,
        ));
        let first_plane_id = reserve_plane_ids(OCTAHEDRON_FACES.len());
        let polygons = OCTAHEDRON_FACES
            .iter()
            .enumerate()
            .map(|(index, _)| {
                let first_corner = index * 3;
                Polygon::from_lazy_subdivision_triangle(
                    Arc::clone(&vertex_pool),
                    index,
                    [first_corner, first_corner + 1, first_corner + 2],
                    metadata.clone(),
                    first_plane_id
                        + u64::try_from(index).expect("octahedron polygon index fits u64"),
                )
            })
            .collect();
        let mesh = Mesh::from_polygons_with_topology(polygons, (8, 24, true));
        mesh.bounding_box
            .set(Aabb::new(
                Point3::new(
                    negative_radius.clone(),
                    negative_radius.clone(),
                    negative_radius,
                ),
                Point3::new(radius.clone(), radius.clone(), radius),
            ))
            .expect("fresh octahedron bounds are initialized once");
        mesh.cache_manifold_fact(true);
        mesh.cache_convex_pwn_fact();
        if let Some(key) = cache_key_on_completion {
            retain_shape_cache(key, &mesh);
        }
        mesh
    }

    /// Regular icosahedron scaled by `radius`.
    ///
    /// The irrational golden-ratio constants are sampled once as finite
    /// binary64 values and promoted to exact dyadic `Real`s at this explicitly
    /// polygonal construction boundary. The construction is the classical
    /// regular icosahedron coordinate model; see Coxeter, *Regular Polytopes*,
    /// 3rd ed., 1973.
    pub fn icosahedron(radius: Real, metadata: M) -> Self {
        if !hmesh_scalar_positive(&radius) {
            return Mesh::empty();
        }
        let key = ShapeCacheKey::Icosahedron(radius.clone());
        if let Some(mesh) = cached_shape(&key, &metadata) {
            return mesh;
        }
        let cache_key_on_completion = shape_cache_on_completion(key);
        let phi_f64 = (1.0_f64 + 5.0_f64.sqrt()) * 0.5;
        let inverse_length_f64 = 1.0_f64 / (1.0_f64 + phi_f64 * phi_f64).sqrt();
        let a_factor = Real::try_from(inverse_length_f64).expect("finite icosahedron scale");
        let b_factor =
            Real::try_from(phi_f64 * inverse_length_f64).expect("finite icosahedron scale");
        let a = radius.clone() * a_factor;
        let b = radius * b_factor;

        // 12 vertices ----------------------------------------------------
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
            Point3::new(-b.clone(), Real::zero(), a.clone()),
        ];

        // 20 faces (counter-clockwise when viewed from outside) ----------
        let faces = [
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

        let canonical_points = [
            [-inverse_length_f64, phi_f64 * inverse_length_f64, 0.0],
            [inverse_length_f64, phi_f64 * inverse_length_f64, 0.0],
            [-inverse_length_f64, -phi_f64 * inverse_length_f64, 0.0],
            [inverse_length_f64, -phi_f64 * inverse_length_f64, 0.0],
            [0.0, -inverse_length_f64, phi_f64 * inverse_length_f64],
            [0.0, inverse_length_f64, phi_f64 * inverse_length_f64],
            [0.0, -inverse_length_f64, -phi_f64 * inverse_length_f64],
            [0.0, inverse_length_f64, -phi_f64 * inverse_length_f64],
            [phi_f64 * inverse_length_f64, 0.0, -inverse_length_f64],
            [phi_f64 * inverse_length_f64, 0.0, inverse_length_f64],
            [-phi_f64 * inverse_length_f64, 0.0, -inverse_length_f64],
            [-phi_f64 * inverse_length_f64, 0.0, inverse_length_f64],
        ];
        let one_over_sqrt_three = 1.0_f64 / 3.0_f64.sqrt();
        let normal_values = [
            (
                one_over_sqrt_three / phi_f64,
                Real::try_from(one_over_sqrt_three / phi_f64)
                    .expect("finite icosahedron normal"),
            ),
            (
                one_over_sqrt_three,
                Real::try_from(one_over_sqrt_three).expect("finite icosahedron normal"),
            ),
            (
                one_over_sqrt_three * phi_f64,
                Real::try_from(one_over_sqrt_three * phi_f64)
                    .expect("finite icosahedron normal"),
            ),
        ];
        let normal_coordinate = |coordinate: f64| {
            if coordinate.abs() < f64::EPSILON {
                return Real::zero();
            }
            let (_, value) = normal_values
                .iter()
                .min_by(|(left, _), (right, _)| {
                    (coordinate.abs() - left)
                        .abs()
                        .total_cmp(&(coordinate.abs() - right).abs())
                })
                .expect("icosahedron has retained normal magnitudes");
            if coordinate.is_sign_negative() {
                -value.clone()
            } else {
                value.clone()
            }
        };
        let first_vertex_identity = reserve_position_ids(points.len() * 4);
        let mut vertices = Vec::with_capacity(faces.len() * 3);
        let mut ranges = Vec::with_capacity(faces.len());
        for face in faces {
            let center: [f64; 3] = std::array::from_fn(|axis| {
                face.iter()
                    .map(|&index| canonical_points[index][axis])
                    .sum::<f64>()
            });
            let center_length =
                (center[0] * center[0] + center[1] * center[1] + center[2] * center[2]).sqrt();
            let normal = Vector3::from_xyz(
                normal_coordinate(center[0] / center_length),
                normal_coordinate(center[1] / center_length),
                normal_coordinate(center[2] / center_length),
            );
            let start = vertices.len();
            for index in face {
                vertices.push(Vertex::new_with_reserved_identity(
                    points[index].clone(),
                    normal.clone(),
                    first_vertex_identity,
                    index,
                ));
            }
            ranges.push(start..vertices.len());
        }
        let vertices = Arc::new(vertices);
        let first_plane_id = reserve_plane_ids(faces.len());
        let polygons = ranges
            .into_iter()
            .enumerate()
            .map(|(index, range)| {
                Polygon::from_shared_vertices(
                    Arc::clone(&vertices),
                    range,
                    metadata.clone(),
                    first_plane_id
                        + u64::try_from(index).expect("icosahedron polygon index fits u64"),
                )
            })
            .collect();
        let mesh = Mesh::from_polygons_with_topology(polygons, (20, 60, true));
        mesh.bounding_box
            .set(Aabb::new(
                Point3::new(-b.clone(), -b.clone(), -b.clone()),
                Point3::new(b.clone(), b.clone(), b),
            ))
            .expect("fresh icosahedron bounds are initialized once");
        mesh.cache_manifold_fact(true);
        mesh.cache_convex_pwn_fact();
        if let Some(key) = cache_key_on_completion {
            retain_shape_cache(key, &mesh);
        }
        mesh
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
        let key = ShapeCacheKey::Torus(
            major_r.clone(),
            minor_r.clone(),
            segments_major,
            segments_minor,
        );
        if let Some(mesh) = cached_shape(&key, &metadata) {
            return mesh;
        }
        let cache_key_on_completion = shape_cache_on_completion(key);

        let Some(major_samples) = sampled_circle_with_f64(segments_major) else {
            return Mesh::empty();
        };
        let Some(minor_samples) = sampled_circle_with_f64(segments_minor) else {
            return Mesh::empty();
        };

        let grid_len = segments_major * segments_minor;
        let first_vertex_identity = reserve_position_ids(
            grid_len
                .checked_mul(4)
                .expect("torus identity reservation does not overflow"),
        );
        let mut position_f64 = Vec::with_capacity(grid_len);
        let major_r_f64 = major_r.to_f64_lossy();
        let minor_r_f64 = minor_r.to_f64_lossy();
        for (_, (sin_theta_f64, cos_theta_f64)) in &major_samples {
            for (_, (sin_phi_f64, cos_phi_f64)) in &minor_samples {
                if let (Some(major_r), Some(minor_r)) = (major_r_f64, minor_r_f64) {
                    let radial = major_r + minor_r * cos_phi_f64;
                    position_f64.push([
                        radial * cos_theta_f64,
                        radial * sin_theta_f64,
                        minor_r * sin_phi_f64,
                    ]);
                }
            }
        }
        let position_f64 = (position_f64.len() == grid_len).then_some(position_f64);

        let cell_count = segments_major * segments_minor;
        let polygon_count = 2 * cell_count;
        let exact_major_samples = major_samples.into_iter().map(|(exact, _)| exact).collect();
        let exact_minor_samples = minor_samples.into_iter().map(|(exact, _)| exact).collect();
        let vertex_pool = Arc::new(LazySubdivisionVertexPool::new_torus(
            major_r.clone(),
            minor_r.clone(),
            exact_major_samples,
            exact_minor_samples,
            polygon_count,
            first_vertex_identity,
        ));
        let first_plane_id = reserve_plane_ids(polygon_count);
        let mut polygons = Vec::with_capacity(polygon_count);
        let mut corner_position_slots = Vec::with_capacity(3 * polygon_count);
        for major in 0..segments_major {
            let next_major = (major + 1) % segments_major;
            for minor in 0..segments_minor {
                let next_minor = (minor + 1) % segments_minor;
                let indices = [
                    major * segments_minor + minor,
                    next_major * segments_minor + minor,
                    next_major * segments_minor + next_minor,
                    major * segments_minor + next_minor,
                ];
                for triangle in [
                    [indices[0], indices[1], indices[2]],
                    [indices[0], indices[2], indices[3]],
                ] {
                    corner_position_slots.extend(triangle);
                    let polygon_index = polygons.len();
                    polygons.push(Polygon::from_lazy_subdivision_triangle(
                        Arc::clone(&vertex_pool),
                        polygon_index,
                        triangle,
                        metadata.clone(),
                        first_plane_id
                            + u64::try_from(polygon_index)
                                .expect("torus triangle index fits u64"),
                    ));
                }
            }
        }
        let outer = major_r.clone() + minor_r.clone();
        let mesh = Mesh::from_polygons_with_topology(
            polygons,
            (polygon_count, 3 * polygon_count, true),
        );
        mesh.bounding_box
            .set(Aabb::new(
                Point3::new(-outer.clone(), -outer.clone(), -minor_r.clone()),
                Point3::new(outer.clone(), outer, minor_r),
            ))
            .expect("fresh torus bounds are initialized once");
        mesh.cache_manifold_fact(true);
        mesh.retain_shared_position_transform_layout(
            corner_position_slots,
            grid_len,
            position_f64.map(Arc::new),
            Some(vertex_pool),
            Some(vec![3; polygon_count]),
            true,
        );
        if let Some(key) = cache_key_on_completion {
            retain_shape_cache(key, &mesh);
        }
        mesh
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
