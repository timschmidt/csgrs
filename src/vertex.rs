//! Struct and functions for working with `Vertex`s.

use hashbrown::HashMap;
use hyperlattice::{Point3, Real, Vector3};
use hyperreal::RealSign;
use std::cell::RefCell;
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};

static NEXT_POSITION_ID: AtomicU64 = AtomicU64::new(1);
const FINITE_POSITION_CACHE_CAPACITY: usize = 262_144;
type FinitePositionRangeCache = (usize, Vec<(u64, Arc<Vec<[f64; 3]>>)>);

thread_local! {
    static FINITE_POSITIONS: RefCell<HashMap<u64, [f64; 3]>> = RefCell::new(HashMap::new());
    static FINITE_POSITION_RANGES: RefCell<FinitePositionRangeCache> =
        const { RefCell::new((0, Vec::new())) };
}

pub(crate) fn fresh_position_id() -> u64 {
    reserve_position_ids(1)
}

pub(crate) fn reserve_position_ids(count: usize) -> u64 {
    NEXT_POSITION_ID.fetch_add(
        u64::try_from(count).expect("position identity reservation fits u64"),
        Ordering::Relaxed,
    )
}

pub(crate) fn cache_position_f64(position_id: u64, position: Option<[f64; 3]>) {
    let Some(position) = position else {
        return;
    };
    FINITE_POSITIONS.with_borrow_mut(|positions| {
        if positions.len() == FINITE_POSITION_CACHE_CAPACITY {
            positions.clear();
        }
        positions.insert(position_id, position);
    });
}

pub(crate) fn cache_position_f64_range<I>(
    first_position_id: u64,
    position_count: usize,
    finite_positions: I,
) where
    I: IntoIterator<Item = Option<[f64; 3]>>,
{
    FINITE_POSITIONS.with_borrow_mut(|positions| {
        if positions.len().saturating_add(position_count) >= FINITE_POSITION_CACHE_CAPACITY {
            positions.clear();
        }
        positions.reserve(position_count);
        for (offset, position) in finite_positions.into_iter().enumerate() {
            if let Some(position) = position {
                positions.insert(
                    first_position_id
                        + u64::try_from(offset).expect("position cache offset fits u64"),
                    position,
                );
            }
        }
    });
}

pub(crate) fn cache_shared_position_f64_range(
    first_position_id: u64,
    finite_positions: Arc<Vec<[f64; 3]>>,
) {
    if finite_positions.is_empty() {
        return;
    }
    FINITE_POSITION_RANGES.with_borrow_mut(|(position_count, ranges)| {
        if position_count.saturating_add(finite_positions.len())
            >= FINITE_POSITION_CACHE_CAPACITY
        {
            *position_count = 0;
            ranges.clear();
        }
        *position_count += finite_positions.len();
        ranges.push((first_position_id, finite_positions));
    });
}

pub(crate) fn reserve_position_f64_cache(additional: usize) {
    FINITE_POSITIONS.with_borrow_mut(|positions| {
        if positions.len().saturating_add(additional) >= FINITE_POSITION_CACHE_CAPACITY {
            positions.clear();
        }
        positions.reserve(additional);
    });
}

/// A vertex of a polygon, holding position and normal.
#[derive(Debug, Clone)]
pub struct Vertex {
    pub position: Point3,
    pub normal: Vector3,
    pub(crate) position_id: u64,
    pub(crate) coordinate_ids: [u64; 3],
    pub(crate) ruled_line: Option<[u64; 2]>,
    pub(crate) hull_candidate: bool,
}

impl PartialEq for Vertex {
    fn eq(&self, other: &Self) -> bool {
        self.position == other.position && self.normal == other.normal
    }
}

impl Default for Vertex {
    fn default() -> Self {
        Self::new(Point3::origin(), Vector3::z())
    }
}

impl Vertex {
    /// Create a new [`Vertex`].
    #[inline]
    pub fn new(position: Point3, normal: Vector3) -> Self {
        Self {
            position,
            normal,
            position_id: fresh_position_id(),
            coordinate_ids: [
                fresh_position_id(),
                fresh_position_id(),
                fresh_position_id(),
            ],
            ruled_line: None,
            hull_candidate: true,
        }
    }

    pub(crate) fn new_with_reserved_identity(
        position: Point3,
        normal: Vector3,
        first_identity: u64,
        slot: usize,
    ) -> Self {
        let offset = u64::try_from(
            slot.checked_mul(4)
                .expect("vertex identity slot multiplication does not overflow"),
        )
        .expect("vertex identity offset fits u64");
        Self {
            position,
            normal,
            position_id: first_identity + offset,
            coordinate_ids: [
                first_identity + offset + 1,
                first_identity + offset + 2,
                first_identity + offset + 3,
            ],
            ruled_line: None,
            hull_candidate: true,
        }
    }

    pub(crate) fn refresh_position_identity(&mut self) {
        self.position_id = fresh_position_id();
        self.coordinate_ids = [
            fresh_position_id(),
            fresh_position_id(),
            fresh_position_id(),
        ];
        self.ruled_line = None;
    }

    pub(crate) fn with_normal(mut self, normal: Vector3) -> Self {
        self.normal = normal;
        self
    }

    /// Returns a retained primitive approximation of this position.
    ///
    /// Transform paths populate this view from the same affine map used for
    /// the exact position. It is only an export/rendering convenience and is
    /// never consumed by topology or predicates.
    pub fn position_f64_lossy(&self) -> Option<[f64; 3]> {
        FINITE_POSITIONS
            .with_borrow(|positions| positions.get(&self.position_id).copied())
            .or_else(|| {
                FINITE_POSITION_RANGES.with_borrow(|(_, ranges)| {
                    ranges
                        .iter()
                        .rev()
                        .find_map(|(first_position_id, positions)| {
                            let offset = self.position_id.checked_sub(*first_position_id)?;
                            positions.get(usize::try_from(offset).ok()?).copied()
                        })
                })
            })
            .or_else(|| {
                Some([
                    self.position.x.to_f64_lossy()?,
                    self.position.y.to_f64_lossy()?,
                    self.position.z.to_f64_lossy()?,
                ])
            })
    }

    pub(crate) const fn exclude_from_hull(mut self) -> Self {
        self.hull_candidate = false;
        self
    }

    /// Flip vertex normal.
    pub fn flip(&mut self) {
        self.normal = -self.normal.clone();
    }

    /// Compute the linear interpolation between `self` and `other`.
    pub fn interpolate(&self, other: &Vertex, t: Real) -> Vertex {
        let new_position = self.position.lerp(&other.position, &t);
        let new_normal = self.normal.lerp(&other.normal, &t);
        Vertex::new(new_position, new_normal)
    }

    /// Compute spherical linear interpolation for normal vectors.
    pub fn slerp_interpolate(&self, other: &Vertex, t: Real) -> Vertex {
        let new_position = self.position.lerp(&other.position, &t);

        let Ok(n0) = self.normal.normalize_checked() else {
            return self.interpolate(other, t);
        };
        let Ok(n1) = other.normal.normalize_checked() else {
            return self.interpolate(other, t);
        };

        let dot = n0.dot(&n1);
        if real_exactly_zero(&(dot.clone() - Real::one()))
            || real_exactly_zero(&(dot.clone() + Real::one()))
        {
            let one_minus_t = Real::one() - t.clone();
            let new_normal = Vector3::weighted_sum(&[n0, n1], &[one_minus_t, t])
                .and_then(|normal| normal.normalize_checked().ok())
                .unwrap_or_else(Vector3::z);
            return Vertex::new(new_position, new_normal);
        }

        let Ok(omega) = n0.angle_to(&n1) else {
            return self.interpolate(other, t);
        };
        let sin_omega = omega.clone().sin();
        if real_exactly_zero(&sin_omega) {
            return self.interpolate(other, t);
        }

        let one_minus_t = Real::one() - t.clone();
        let a_sin = (one_minus_t * omega.clone()).sin();
        let b_sin = (t * omega).sin();
        let Ok(a) = a_sin / sin_omega.clone() else {
            return self.interpolate(other, Real::zero());
        };
        let Ok(b) = b_sin / sin_omega else {
            return self.interpolate(other, Real::zero());
        };

        let new_normal = Vector3::weighted_sum(&[n0, n1], &[a, b])
            .and_then(|normal| normal.normalize_checked().ok())
            .unwrap_or_else(Vector3::z);
        Vertex::new(new_position, new_normal)
    }

    /// Compute Euclidean distance between vertex positions.
    pub fn distance_to(&self, other: &Vertex) -> Real {
        self.distance_squared_to(other)
            .sqrt()
            .unwrap_or_else(|_| Real::zero())
    }

    /// Compute squared Euclidean distance between vertex positions.
    pub fn distance_squared_to(&self, other: &Vertex) -> Real {
        self.position
            .to_vector()
            .squared_distance(&other.position.to_vector())
    }

    /// Compute angle between normal vectors in radians.
    pub fn normal_angle_to(&self, other: &Vertex) -> Real {
        self.normal
            .angle_to(&other.normal)
            .unwrap_or_else(|_| Real::zero())
    }

    /// Compute weighted average of vertex positions and normals.
    pub fn weighted_average(vertices: &[(Vertex, Real)]) -> Option<Vertex> {
        if vertices.is_empty() {
            return None;
        }

        let normalized_weights = normalized_weights(vertices.iter().map(|(_, w)| w.clone()))?;
        let positions = vertices
            .iter()
            .map(|(vertex, _)| vertex.position.clone())
            .collect::<Vec<_>>();
        let normals = vertices
            .iter()
            .map(|(vertex, _)| vertex.normal.clone())
            .collect::<Vec<_>>();
        let weighted_position = Point3::weighted_sum(&positions, &normalized_weights)?;
        let weighted_normal = Vector3::weighted_sum(&normals, &normalized_weights)?;
        let normalized_normal = weighted_normal
            .normalize_checked()
            .unwrap_or_else(|_| Vector3::z());

        Some(Vertex::new(weighted_position, normalized_normal))
    }

    /// Interpolate vertex using barycentric coordinates.
    pub fn barycentric_interpolate(
        v1: &Vertex,
        v2: &Vertex,
        v3: &Vertex,
        u: Real,
        v: Real,
        w: Real,
    ) -> Vertex {
        let weights = normalized_barycentric_weights(u, v, w).unwrap_or_else(equal_thirds);
        let [u, v, w] = weights;
        let weights = [u, v, w];
        let new_position = Point3::weighted_sum(
            &[
                v1.position.clone(),
                v2.position.clone(),
                v3.position.clone(),
            ],
            &weights,
        )
        .unwrap_or_else(Point3::origin);
        let blended_normal = Vector3::weighted_sum(
            &[v1.normal.clone(), v2.normal.clone(), v3.normal.clone()],
            &weights,
        )
        .unwrap_or_else(Vector3::z);
        let new_normal = blended_normal
            .normalize_checked()
            .unwrap_or_else(|_| Vector3::z());

        Vertex::new(new_position, new_normal)
    }
}

fn normalized_barycentric_weights(u: Real, v: Real, w: Real) -> Option<[Real; 3]> {
    let total = u.clone() + v.clone() + w.clone();
    if real_exactly_zero(&total) {
        return None;
    }
    Some([
        (u / total.clone()).ok()?,
        (v / total.clone()).ok()?,
        (w / total).ok()?,
    ])
}

fn equal_thirds() -> [Real; 3] {
    let third = (Real::one() / Real::from(3_u8)).expect("nonzero exact denominator");
    [third.clone(), third.clone(), third]
}

fn real_exactly_zero(value: &Real) -> bool {
    matches!(value.refine_sign_until(-128), Some(RealSign::Zero))
}

fn real_positive(value: &Real) -> bool {
    matches!(value.refine_sign_until(-128), Some(RealSign::Positive))
}

fn normalized_weights(weights: impl IntoIterator<Item = Real>) -> Option<Vec<Real>> {
    let weights = weights.into_iter().collect::<Vec<_>>();
    if weights.is_empty() {
        return None;
    }

    let total = Real::sum_refs(weights.iter());
    if !real_positive(&total) {
        return None;
    }

    weights
        .into_iter()
        .map(|weight| (weight / total.clone()).ok())
        .collect()
}

fn real_from_usize(value: usize) -> Option<Real> {
    Some(Real::from(u64::try_from(value).ok()?))
}

fn real_mean(values: &[Real]) -> Option<Real> {
    Real::mean(values)
}

fn real_sample_stddev(values: &[Real]) -> Option<Real> {
    Real::sample_stddev(values)
}

fn real_clamp_unit(value: Real) -> Real {
    let zero = Real::zero();
    let one = Real::one();
    if matches!(
        hyperlimit::compare_reals(&value, &zero).value(),
        Some(std::cmp::Ordering::Less)
    ) {
        return zero;
    }
    if matches!(
        hyperlimit::compare_reals(&value, &one).value(),
        Some(std::cmp::Ordering::Greater)
    ) {
        return one;
    }
    value
}

fn point_distance(lhs: &Point3, rhs: &Point3) -> Option<Real> {
    lhs.to_vector().squared_distance(&rhs.to_vector()).sqrt().ok()
}

fn vector_distance(lhs: &Vector3, rhs: &Vector3) -> Option<Real> {
    lhs.squared_distance(rhs).sqrt().ok()
}

fn cotangent_at_opposite(
    center: &Vertex,
    neighbor: &Vertex,
    opposite: &Vertex,
) -> Option<Real> {
    let edge1 = &center.position - &opposite.position;
    let edge2 = &neighbor.position - &opposite.position;
    let cross = edge1.cross(&edge2);
    let cross_len = cross.dot(&cross).sqrt().ok()?;
    if !real_positive(&cross_len) {
        return None;
    }
    (edge1.dot(&edge2) / cross_len).ok()
}

impl Vertex {
    /// Compute cotangent weights for discrete Laplacian operators.
    pub fn compute_cotangent_weight(
        center: &Vertex,
        neighbor: &Vertex,
        triangle_vertices: &[&Vertex],
    ) -> Real {
        if triangle_vertices.len() < 3 {
            return Real::one();
        }

        let mut cotangents = Vec::new();
        for i in 0..triangle_vertices.len() {
            let v1 = triangle_vertices[i];
            let v2 = triangle_vertices[(i + 1) % triangle_vertices.len()];
            let v3 = triangle_vertices[(i + 2) % triangle_vertices.len()];

            let contains_edge = (v1.position == center.position
                && v2.position == neighbor.position)
                || (v2.position == center.position && v3.position == neighbor.position)
                || (v3.position == center.position && v1.position == neighbor.position)
                || (v1.position == neighbor.position && v2.position == center.position)
                || (v2.position == neighbor.position && v3.position == center.position)
                || (v3.position == neighbor.position && v1.position == center.position);

            if contains_edge {
                let opposite = if v1.position != center.position
                    && v1.position != neighbor.position
                {
                    v1
                } else if v2.position != center.position && v2.position != neighbor.position {
                    v2
                } else {
                    v3
                };

                if let Some(cotangent) = cotangent_at_opposite(center, neighbor, opposite) {
                    cotangents.push(cotangent);
                }
            }
        }

        if cotangents.is_empty() {
            return Real::one();
        }
        let denom =
            Real::from(2_u8) * real_from_usize(cotangents.len()).unwrap_or_else(Real::one);
        (Real::sum_refs(cotangents.iter()) / denom).unwrap_or_else(|_| Real::one())
    }

    /// Analyze vertex connectivity by graph index.
    pub fn analyze_connectivity_with_index(
        vertex_index: usize,
        adjacency_map: &HashMap<usize, Vec<usize>>,
    ) -> (usize, Real) {
        let valence = adjacency_map
            .get(&vertex_index)
            .map(|neighbors| neighbors.len())
            .unwrap_or(0);

        let target_valence = 6_usize;
        let regularity = if valence > 0 {
            let deviation = valence.abs_diff(target_valence);
            let deviation = real_from_usize(deviation).unwrap_or_else(Real::zero);
            let target = real_from_usize(target_valence).unwrap_or_else(Real::one);
            let ratio = (deviation / target).unwrap_or_else(|_| Real::zero());
            (Real::one() / (Real::one() + ratio)).unwrap_or_else(|_| Real::zero())
        } else {
            Real::zero()
        };

        (valence, regularity)
    }

    /// Search for this vertex in an adjacency-position map.
    pub fn analyze_connectivity_by_position(
        &self,
        adjacency_map: &HashMap<usize, Vec<usize>>,
        vertex_positions: &HashMap<usize, Point3>,
    ) -> (usize, Real) {
        let self_position = hyperlimit::Point3::new(
            self.position.x.clone(),
            self.position.y.clone(),
            self.position.z.clone(),
        );
        let vertex_index = vertex_positions.iter().find_map(|(&idx, position)| {
            let candidate_position = hyperlimit::Point3::new(
                position.x.clone(),
                position.y.clone(),
                position.z.clone(),
            );
            matches!(
                hyperlimit::point3_equal(&self_position, &candidate_position).value(),
                Some(true)
            )
            .then_some(idx)
        });

        vertex_index
            .map(|idx| Self::analyze_connectivity_with_index(idx, adjacency_map))
            .unwrap_or((0, Real::zero()))
    }

    /// Estimate discrete mean curvature using the angle deficit method.
    pub fn estimate_mean_curvature(&self, neighbors: &[Vertex], face_areas: &[Real]) -> Real {
        if neighbors.len() < 3 {
            return Real::zero();
        }

        let mut angles = Vec::with_capacity(neighbors.len());
        for i in 0..neighbors.len() {
            let prev = &neighbors[(i + neighbors.len() - 1) % neighbors.len()];
            let next = &neighbors[(i + 1) % neighbors.len()];
            let v1 = &prev.position - &self.position;
            let v2 = &next.position - &self.position;
            if let Ok(angle) = v1.angle_to(&v2) {
                angles.push(angle);
            }
        }

        let angle_sum = Real::sum_refs(angles.iter());
        let finite_face_areas = face_areas
            .iter()
            .filter(|area| real_positive(area))
            .cloned()
            .collect::<Vec<_>>();
        let mixed_area = real_mean(&finite_face_areas).unwrap_or_else(Real::one);

        if !real_positive(&mixed_area) {
            return Real::zero();
        }
        let full_turn = Real::from(2_u8) * Real::pi();
        ((full_turn - angle_sum) / mixed_area).unwrap_or_else(|_| Real::zero())
    }

    /// Comprehensive vertex quality assessment.
    pub fn comprehensive_quality_analysis(
        &self,
        vertex_index: usize,
        adjacency_map: &HashMap<usize, Vec<usize>>,
        vertex_positions: &HashMap<usize, Point3>,
        vertex_normals: &HashMap<usize, Vector3>,
    ) -> (Real, Real, Real, Real) {
        let (valence, regularity) =
            Self::analyze_connectivity_with_index(vertex_index, adjacency_map);

        if valence == 0 {
            return (Real::zero(), Real::zero(), Real::zero(), Real::zero());
        }

        let Some(neighbors) = adjacency_map.get(&vertex_index) else {
            return (Real::zero(), Real::zero(), Real::zero(), Real::zero());
        };
        let mut edge_lengths = Vec::new();
        let mut neighbor_normals = Vec::new();

        for &neighbor_idx in neighbors {
            if let Some(neighbor_position) = vertex_positions.get(&neighbor_idx)
                && let Some(edge_length) = point_distance(&self.position, neighbor_position)
            {
                edge_lengths.push(edge_length);
            }
            if let Some(neighbor_normal) = vertex_normals.get(&neighbor_idx) {
                neighbor_normals.push(neighbor_normal.clone());
            }
        }

        let edge_uniformity = if edge_lengths.len() > 1 {
            let mean_edge = real_mean(&edge_lengths).unwrap_or_else(Real::zero);
            let std_dev = real_sample_stddev(&edge_lengths).unwrap_or_else(Real::zero);
            if real_positive(&mean_edge) {
                let ratio = (std_dev / mean_edge).unwrap_or_else(|_| Real::zero());
                real_clamp_unit(
                    (Real::one() / (Real::one() + ratio)).unwrap_or_else(|_| Real::zero()),
                )
            } else {
                Real::zero()
            }
        } else {
            Real::one()
        };

        let normal_variation = if neighbor_normals.len() > 1 {
            let max_angle = neighbor_normals
                .iter()
                .filter_map(|neighbor_normal| self.normal.angle_to(neighbor_normal).ok())
                .reduce(|current, angle| {
                    hyperlimit::real_max(&current, &angle)
                        .value()
                        .cloned()
                        .unwrap_or(current)
                });

            max_angle
                .and_then(|angle| (angle / Real::pi()).ok())
                .map(real_clamp_unit)
                .map(|ratio| Real::one() - ratio)
                .unwrap_or_else(Real::one)
        } else {
            Real::one()
        };

        let curvature = if neighbor_normals.is_empty() {
            Real::zero()
        } else {
            let avg_normal = Vector3::mean(&neighbor_normals).unwrap_or_else(Vector3::zero);
            vector_distance(&self.normal, &avg_normal).unwrap_or_else(Real::zero)
        };

        (regularity, curvature, edge_uniformity, normal_variation)
    }
}

/// Vertex clustering for mesh simplification.
pub struct VertexCluster {
    /// Representative position.
    pub position: Point3,
    /// Averaged normal vector.
    pub normal: Vector3,
    /// Number of vertices in cluster.
    pub count: usize,
    /// Bounding radius of cluster.
    pub radius: Real,
}

impl VertexCluster {
    /// Create a new vertex cluster from a collection of vertices.
    pub fn from_vertices(vertices: &[Vertex]) -> Option<Self> {
        if vertices.is_empty() {
            return None;
        }

        let positions = vertices
            .iter()
            .map(|vertex| vertex.position.clone())
            .collect::<Vec<_>>();
        let centroid = Point3::centroid(&positions)?;

        let normals = vertices
            .iter()
            .map(|vertex| vertex.normal.clone())
            .collect::<Vec<_>>();
        let avg_normal = Vector3::mean(&normals).unwrap_or_else(Vector3::z);
        let normalized_normal = avg_normal
            .normalize_checked()
            .unwrap_or_else(|_| Vector3::z());

        let radius = vertices
            .iter()
            .filter_map(|v| point_distance(&v.position, &centroid))
            .reduce(|current, radius| {
                hyperlimit::real_max(&current, &radius)
                    .value()
                    .cloned()
                    .unwrap_or(current)
            })
            .unwrap_or_else(Real::zero);

        Some(VertexCluster {
            position: centroid,
            normal: normalized_normal,
            count: vertices.len(),
            radius,
        })
    }

    /// Convert cluster back to a representative vertex.
    pub fn to_vertex(&self) -> Vertex {
        Vertex::new(self.position.clone(), self.normal.clone())
    }
}
