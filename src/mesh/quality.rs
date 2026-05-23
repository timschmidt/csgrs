//! Mesh quality metrics for triangles, vertices, and aggregate mesh health.

use crate::mesh::Mesh;
use crate::vertex::Vertex;
use hyperlattice::{Point3, Real};
use hyperreal::RealSign;
use std::cmp::Ordering;
use std::fmt::Debug;

#[cfg(feature = "parallel")]
use rayon::prelude::*;

fn degenerate_triangle_quality() -> TriangleQuality {
    let sentinel = Real::from(1_000_000_000_u64);
    TriangleQuality {
        aspect_ratio: sentinel.clone(),
        min_angle: Real::zero(),
        max_angle: Real::zero(),
        edge_ratio: sentinel,
        area: Real::zero(),
        quality_score: Real::zero(),
    }
}

fn hyper_edge_length(a: &Point3, b: &Point3) -> Option<Real> {
    let edge = b - a;
    edge.dot(&edge).sqrt().ok()
}

fn real_cmp(lhs: &Real, rhs: &Real) -> Ordering {
    hyperlimit::compare_reals(lhs, rhs)
        .value()
        .unwrap_or_else(|| match (lhs.clone() - rhs.clone()).refine_sign_until(128) {
            Some(RealSign::Positive) => Ordering::Greater,
            Some(RealSign::Negative) => Ordering::Less,
            Some(RealSign::Zero) | None => Ordering::Equal,
        })
}

fn real_lt(lhs: &Real, rhs: &Real) -> bool {
    matches!(real_cmp(lhs, rhs), Ordering::Less)
}

fn real_gt(lhs: &Real, rhs: &Real) -> bool {
    matches!(real_cmp(lhs, rhs), Ordering::Greater)
}

fn real_zero(value: &Real) -> bool {
    matches!(value.refine_sign_until(128), Some(RealSign::Zero))
}

fn real_clamp(value: Real, min: &Real, max: &Real) -> Real {
    if real_lt(&value, min) {
        min.clone()
    } else if real_gt(&value, max) {
        max.clone()
    } else {
        value
    }
}

fn boundary_scalar_min(values: impl IntoIterator<Item = Real>) -> Option<Real> {
    values
        .into_iter()
        .reduce(|best, value| if real_lt(&value, &best) { value } else { best })
}

fn real_mean(values: &[Real]) -> Option<Real> {
    if values.is_empty() {
        return None;
    }
    let sum = values
        .iter()
        .cloned()
        .fold(Real::zero(), |acc, value| acc + value);
    (sum / Real::from(values.len() as u64)).ok()
}

fn real_sample_stddev(values: &[Real]) -> Option<Real> {
    if values.len() < 2 {
        return Some(Real::zero());
    }
    let mean = real_mean(values)?;
    let sum_sq = values.iter().fold(Real::zero(), |acc, value| {
        let delta = value.clone() - mean.clone();
        acc + delta.clone() * delta
    });
    (sum_sq / Real::from((values.len() - 1) as u64))
        .ok()?
        .sqrt()
        .ok()
}

fn degrees_to_radians(degrees: Real) -> Option<Real> {
    (degrees * Real::pi() / Real::from(180_u8)).ok()
}

fn triangle_area(a: &Point3, b: &Point3, c: &Point3) -> Option<Real> {
    let ab = b - a;
    let ac = c - a;
    let cross = ab.cross(&ac);
    let magnitude = cross.dot(&cross).sqrt().ok()?;
    (magnitude / Real::from(2_u8)).ok()
}

fn hyper_triangle_quality(vertices: &[Vertex]) -> Option<TriangleQuality> {
    if vertices.len() != 3 {
        return None;
    }

    let len_ab = hyper_edge_length(&vertices[0].position, &vertices[1].position)?;
    let len_bc = hyper_edge_length(&vertices[1].position, &vertices[2].position)?;
    let len_ca = hyper_edge_length(&vertices[2].position, &vertices[0].position)?;

    if !real_gt(&len_ab, &Real::zero())
        || !real_gt(&len_bc, &Real::zero())
        || !real_gt(&len_ca, &Real::zero())
    {
        return None;
    }

    let Some(area) = triangle_area(
        &vertices[0].position,
        &vertices[1].position,
        &vertices[2].position,
    ) else {
        let min_ab = hyperlimit::real_min(&len_ab, &len_bc).value().cloned()?;
        let min_edge = hyperlimit::real_min(&min_ab, &len_ca).value().cloned()?;
        let max_ab = hyperlimit::real_max(&len_ab, &len_bc).value().cloned()?;
        let max_edge = hyperlimit::real_max(&max_ab, &len_ca).value().cloned()?;
        let edge_ratio = (max_edge / min_edge).ok()?;
        return Some(TriangleQuality {
            aspect_ratio: Real::from(1_000_000_000_u64),
            min_angle: Real::zero(),
            max_angle: Real::zero(),
            edge_ratio,
            area: Real::zero(),
            quality_score: Real::zero(),
        });
    };
    if real_zero(&area) {
        return None;
    }

    let two = Real::from(2_u8);
    let angle = |opposite: &Real, side_a: &Real, side_b: &Real| {
        let numerator = side_a.clone() * side_a.clone() + side_b.clone() * side_b.clone()
            - opposite.clone() * opposite.clone();
        let denominator = two.clone() * side_a.clone() * side_b.clone();
        let cos_angle =
            real_clamp((numerator / denominator).ok()?, &(-Real::one()), &Real::one());
        cos_angle.acos().ok()
    };

    let min_ab = hyperlimit::real_min(&len_ab, &len_bc).value().cloned()?;
    let min_edge = hyperlimit::real_min(&min_ab, &len_ca).value().cloned()?;
    let max_ab = hyperlimit::real_max(&len_ab, &len_bc).value().cloned()?;
    let max_edge = hyperlimit::real_max(&max_ab, &len_ca).value().cloned()?;
    let edge_ratio = (max_edge / min_edge).ok()?;

    let min_angle = if hyperlimit::real_le(&len_ab, &len_bc).value()?
        && hyperlimit::real_le(&len_ab, &len_ca).value()?
    {
        angle(&len_ab, &len_bc, &len_ca)?
    } else if hyperlimit::real_le(&len_bc, &len_ab).value()?
        && hyperlimit::real_le(&len_bc, &len_ca).value()?
    {
        angle(&len_bc, &len_ca, &len_ab)?
    } else {
        angle(&len_ca, &len_ab, &len_bc)?
    };
    let max_angle = if hyperlimit::real_ge(&len_ab, &len_bc).value()?
        && hyperlimit::real_ge(&len_ab, &len_ca).value()?
    {
        angle(&len_ab, &len_bc, &len_ca)?
    } else if hyperlimit::real_ge(&len_bc, &len_ab).value()?
        && hyperlimit::real_ge(&len_bc, &len_ca).value()?
    {
        angle(&len_bc, &len_ca, &len_ab)?
    } else {
        angle(&len_ca, &len_ab, &len_bc)?
    };

    let semiperimeter =
        ((len_ab.clone() + len_bc.clone() + len_ca.clone()) / Real::from(2_u8)).ok()?;
    let circumradius = (len_ab * len_bc * len_ca / (Real::from(4_u8) * area.clone())).ok()?;
    let inradius = (area.clone() / semiperimeter).ok()?;
    let aspect_ratio = (circumradius / inradius).ok()?;

    let zero = Real::zero();
    let one = Real::one();
    let angle_quality = real_clamp(
        (min_angle.clone() / (Real::pi() / Real::from(6_u8)).ok()?).ok()?,
        &zero,
        &one,
    );
    let shape_quality = real_clamp((Real::one() / aspect_ratio.clone()).ok()?, &zero, &one);
    let edge_quality = real_clamp((Real::from(3_u8) / edge_ratio.clone()).ok()?, &zero, &one);

    let two_fifths = (Real::from(2_u8) / Real::from(5_u8)).ok()?;
    let one_fifth = (Real::one() / Real::from(5_u8)).ok()?;
    let quality_score = real_clamp(
        two_fifths.clone() * angle_quality
            + two_fifths * shape_quality
            + one_fifth * edge_quality,
        &zero,
        &one,
    );

    Some(TriangleQuality {
        aspect_ratio,
        min_angle,
        max_angle,
        edge_ratio,
        area,
        quality_score,
    })
}

/// **Mathematical Foundation: Triangle Quality Metrics**
///
/// Comprehensive triangle quality assessment for mesh optimization:
///
/// ## **Aspect Ratio**
/// Measures shape quality as ratio of circumradius to inradius:
/// ```text
/// Q = R / (2r) = abc / (8A·r)
/// ```
/// Where R = circumradius, r = inradius, A = area, a,b,c = edge lengths
/// - **Perfect triangle**: Q = 1 (equilateral)
/// - **Poor quality**: Q > 10 (very elongated/thin)
///
/// ## **Minimum Angle**
/// The smallest interior angle θ_min:
/// - **Good quality**: θ_min > 30°
/// - **Poor quality**: θ_min < 10° (sliver triangles)
///
/// ## **Edge Length Ratio**
/// Maximum to minimum edge length ratio:
/// ```text
/// R_edge = max(a,b,c) / min(a,b,c)
/// ```
/// - **Well-proportioned**: R_edge < 3
/// - **Degenerate**: R_edge > 10
#[derive(Debug, Clone)]
pub struct TriangleQuality {
    /// Aspect ratio (circumradius to inradius ratio)
    pub aspect_ratio: Real,
    /// Minimum interior angle in radians
    pub min_angle: Real,
    /// Maximum interior angle in radians  
    pub max_angle: Real,
    /// Edge length ratio (longest/shortest)
    pub edge_ratio: Real,
    /// Triangle area
    pub area: Real,
    /// Quality score (0-1, where 1 is perfect)
    pub quality_score: Real,
}

/// **Mathematical Foundation: Mesh Quality Assessment and Optimization**
///
/// Advanced mesh processing algorithms for quality improvement:
///
/// ## **Quality Metrics**
/// - **Shape Quality**: Aspect ratio, angle bounds, edge ratios
/// - **Connectivity**: Vertex valence, edge regularity
/// - **Geometric**: Surface smoothness, feature preservation
///
/// ## **Adaptive Refinement**
/// - **Curvature-based**: Refine high-curvature regions
/// - **Error-driven**: Refine based on approximation error
/// - **Feature-preserving**: Maintain sharp edges and corners
///
/// ## **Smoothing Algorithms**
/// - **Laplacian**: Simple position averaging
/// - **Taubin**: Feature-preserving with shrinkage correction
/// - **Bilateral**: Edge-preserving smoothing
#[derive(Debug, Clone)]
pub struct MeshQualityMetrics {
    /// Average triangle quality score
    pub avg_quality: Real,
    /// Minimum triangle quality in mesh
    pub min_quality: Real,
    /// Percentage of high-quality triangles (score > 0.7)
    pub high_quality_ratio: Real,
    /// Number of sliver triangles (min angle < 10°)
    pub sliver_count: usize,
    /// Average edge length
    pub avg_edge_length: Real,
    /// Edge length standard deviation
    pub edge_length_std: Real,
}

impl<M: Clone + Debug + Send + Sync> Mesh<M> {
    /// **Mathematical Foundation: Comprehensive Triangle Quality Analysis**
    ///
    /// Analyze triangle quality using multiple geometric metrics:
    ///
    /// ## **Quality Assessment Algorithm**
    /// For each triangle with vertices A, B, C:
    /// 1. **Edge lengths**: a = |BC|, b = |CA|, c = |AB|
    /// 2. **Area**: A = ½|AB⃗ × AC⃗|
    /// 3. **Angles**: Using law of cosines: cos(θ) = (b² + c² - a²)/(2bc)
    /// 4. **Circumradius**: R = abc/(4A)
    /// 5. **Inradius**: r = A/s, where s = (a+b+c)/2
    /// 6. **Quality score**: Weighted combination of all metrics
    ///
    /// Edge lengths, areas, law-of-cosines angle terms, radii, and normalized
    /// quality scores are measured through `hyperlattice::Vector3` and
    /// `Real`, then exported to finite scalars for this reporting
    /// API. This keeps degeneracy decisions on the exact-aware side of the
    /// boundary, following Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>). The angle computation
    /// uses the classical law of cosines; see Euclid, *Elements*, Book II,
    /// Proposition 12/13.
    ///
    /// Returns quality metrics for each triangle in the mesh.
    pub fn analyze_triangle_quality(&self) -> Vec<TriangleQuality> {
        let triangulated = self.triangulate();

        #[cfg(feature = "parallel")]
        let qualities: Vec<TriangleQuality> = triangulated
            .polygons
            .par_iter()
            .map(|poly| Self::compute_triangle_quality(&poly.vertices))
            .collect();

        #[cfg(not(feature = "parallel"))]
        let qualities: Vec<TriangleQuality> = triangulated
            .polygons
            .iter()
            .map(|poly| Self::compute_triangle_quality(&poly.vertices))
            .collect();

        qualities
    }

    /// Compute comprehensive quality metrics for a single triangle
    fn compute_triangle_quality(vertices: &[Vertex]) -> TriangleQuality {
        hyper_triangle_quality(vertices).unwrap_or_else(degenerate_triangle_quality)
    }

    /// **Mathematical Foundation: Mesh Quality Assessment**
    ///
    /// Compute comprehensive mesh quality metrics:
    ///
    /// ## **Statistical Measures**
    /// - **Average quality**: Overall mesh shape quality
    /// - **Quality distribution**: Histogram of triangle qualities  
    /// - **Outlier detection**: Identification of problematic triangles
    ///
    /// ## **Geometric Measures**
    /// - **Edge length distribution**: Uniformity of mesh resolution
    /// - **Valence distribution**: Vertex connectivity regularity
    /// - **Aspect ratio bounds**: Shape quality bounds
    ///
    /// Aggregate comparisons are evaluated through the same
    /// `Real` boundary comparators used by per-triangle metrics;
    /// the sliver threshold is converted through the shared degree adapter
    /// rather than primitive `to_radians`. This keeps reporting decisions
    /// aligned with Yap's exact-geometric-computation boundary discipline
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>) and the standard
    /// angle-threshold quality criteria described in Shewchuk, "What Is a Good
    /// Linear Element? Interpolation, Conditioning, and Quality Measures,"
    /// 2002 (<https://people.eecs.berkeley.edu/~jrs/papers/elemj.pdf>).
    ///
    /// Provides quantitative assessment for mesh optimization decisions.
    pub fn compute_mesh_quality(&self) -> MeshQualityMetrics {
        let qualities = self.analyze_triangle_quality();

        if qualities.is_empty() {
            return MeshQualityMetrics {
                avg_quality: Real::zero(),
                min_quality: Real::zero(),
                high_quality_ratio: Real::zero(),
                sliver_count: 0,
                avg_edge_length: Real::zero(),
                edge_length_std: Real::zero(),
            };
        }

        let quality_scores = qualities
            .iter()
            .map(|q| q.quality_score.clone())
            .collect::<Vec<_>>();
        let avg_quality = real_mean(&quality_scores).unwrap_or_else(Real::zero);

        let min_quality =
            boundary_scalar_min(qualities.iter().map(|q| q.quality_score.clone()))
                .unwrap_or_else(Real::zero);

        let high_quality_count = qualities
            .iter()
            .filter(|q| {
                real_gt(
                    &q.quality_score,
                    &(Real::from(7_u8) / Real::from(10_u8)).unwrap(),
                )
            })
            .count();
        let high_quality_ratio = (Real::from(high_quality_count as u64)
            / Real::from(qualities.len() as u64))
        .unwrap_or_else(|_| Real::zero());

        let sliver_threshold =
            degrees_to_radians(Real::from(10_u8)).unwrap_or_else(Real::zero);
        let sliver_count = qualities
            .iter()
            .filter(|q| real_lt(&q.min_angle, &sliver_threshold))
            .count();

        // Compute edge length statistics
        let edge_lengths: Vec<Real> = self
            .polygons
            .iter()
            .flat_map(|poly| {
                poly.vertices
                    .windows(2)
                    .filter_map(|w| hyper_edge_length(&w[0].position, &w[1].position))
                    .chain(
                        poly.vertices
                            .last()
                            .and_then(|last| {
                                hyper_edge_length(&poly.vertices[0].position, &last.position)
                            })
                            .into_iter(),
                    )
            })
            .collect();

        let avg_edge_length = real_mean(&edge_lengths).unwrap_or_else(Real::zero);

        let edge_length_std = real_sample_stddev(&edge_lengths).unwrap_or_else(Real::zero);

        MeshQualityMetrics {
            avg_quality,
            min_quality,
            high_quality_ratio,
            sliver_count,
            avg_edge_length,
            edge_length_std,
        }
    }
}
