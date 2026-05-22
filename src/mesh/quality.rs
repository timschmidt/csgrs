//! Mesh quality metrics for triangles, vertices, and aggregate mesh health.

use crate::float_types::{
    PI, Real, hdegrees_to_radians, hreal_clamp_hreal, hreal_cmp_f64, hreal_div,
    hreal_from_f64, hreal_gt_f64, hreal_max_pair, hreal_mean, hreal_min, hreal_min_pair,
    hreal_sample_stddev, hreal_sqrt_ref, hreal_sqrt_to_f64, hreal_to_f64,
    htriangle_area_hreal, hvector3_from_point3,
};
use crate::mesh::Mesh;
use crate::vertex::Vertex;
use nalgebra::Point3;
use std::cmp::Ordering;
use std::fmt::Debug;

#[cfg(feature = "parallel")]
use rayon::prelude::*;

fn degenerate_triangle_quality() -> TriangleQuality {
    TriangleQuality {
        aspect_ratio: Real::INFINITY,
        min_angle: 0.0,
        max_angle: 0.0,
        edge_ratio: Real::INFINITY,
        area: 0.0,
        quality_score: 0.0,
    }
}

fn hyper_edge_length(a: &Point3<Real>, b: &Point3<Real>) -> Option<Real> {
    let a = hvector3_from_point3(a)?;
    let b = hvector3_from_point3(b)?;
    hreal_sqrt_to_f64(&a.squared_distance(&b))
}

fn boundary_scalar_cmp(lhs: Real, rhs: Real) -> Ordering {
    hreal_cmp_f64(lhs, rhs)
}

fn boundary_scalar_lt(lhs: Real, rhs: Real) -> bool {
    matches!(boundary_scalar_cmp(lhs, rhs), Ordering::Less)
}

fn boundary_scalar_gt(lhs: Real, rhs: Real) -> bool {
    matches!(boundary_scalar_cmp(lhs, rhs), Ordering::Greater)
}

fn boundary_scalar_min(values: impl IntoIterator<Item = Real>) -> Option<Real> {
    let values = values.into_iter().collect::<Vec<_>>();
    hreal_min(&values)
}

fn hreal_min3(
    a: &hyperreal::Real,
    b: &hyperreal::Real,
    c: &hyperreal::Real,
) -> Option<hyperreal::Real> {
    hreal_min_pair(&hreal_min_pair(a, b)?, c)
}

fn hreal_max3(
    a: &hyperreal::Real,
    b: &hyperreal::Real,
    c: &hyperreal::Real,
) -> Option<hyperreal::Real> {
    hreal_max_pair(&hreal_max_pair(a, b)?, c)
}

fn hreal_is_le(lhs: &hyperreal::Real, rhs: &hyperreal::Real) -> Option<bool> {
    hyperlimit::real_le(lhs, rhs).value()
}

fn hreal_is_ge(lhs: &hyperreal::Real, rhs: &hyperreal::Real) -> Option<bool> {
    hyperlimit::real_ge(lhs, rhs).value()
}

fn hyper_triangle_quality(vertices: &[Vertex]) -> Option<TriangleQuality> {
    if vertices.len() != 3 {
        return None;
    }

    let a = hvector3_from_point3(&vertices[0].position)?;
    let b = hvector3_from_point3(&vertices[1].position)?;
    let c = hvector3_from_point3(&vertices[2].position)?;

    let len_ab = hreal_sqrt_ref(&a.squared_distance(&b))?;
    let len_bc = hreal_sqrt_ref(&b.squared_distance(&c))?;
    let len_ca = hreal_sqrt_ref(&c.squared_distance(&a))?;

    if !hreal_gt_f64(&len_ab, 0.0)
        || !hreal_gt_f64(&len_bc, 0.0)
        || !hreal_gt_f64(&len_ca, 0.0)
    {
        return None;
    }

    let Some(area) = htriangle_area_hreal(
        &vertices[0].position,
        &vertices[1].position,
        &vertices[2].position,
    ) else {
        let min_edge = hreal_min3(&len_ab, &len_bc, &len_ca)?;
        let max_edge = hreal_max3(&len_ab, &len_bc, &len_ca)?;
        let edge_ratio = (max_edge / min_edge).ok()?;
        return Some(TriangleQuality {
            aspect_ratio: Real::INFINITY,
            min_angle: 0.0,
            max_angle: 0.0,
            edge_ratio: hreal_to_f64(&edge_ratio)?,
            area: 0.0,
            quality_score: 0.0,
        });
    };

    let two = hreal_from_f64(2.0).ok()?;
    let angle =
        |opposite: &hyperreal::Real, side_a: &hyperreal::Real, side_b: &hyperreal::Real| {
            let numerator = side_a.clone() * side_a.clone() + side_b.clone() * side_b.clone()
                - opposite.clone() * opposite.clone();
            let denominator = two.clone() * side_a.clone() * side_b.clone();
            let cos_angle = hreal_clamp_hreal((numerator / denominator).ok()?, -1.0, 1.0)?;
            hyperlattice::acos(cos_angle).ok()
        };

    let min_edge = hreal_min3(&len_ab, &len_bc, &len_ca)?;
    let max_edge = hreal_max3(&len_ab, &len_bc, &len_ca)?;
    let edge_ratio = (max_edge / min_edge).ok()?;

    let min_angle = if hreal_is_le(&len_ab, &len_bc)? && hreal_is_le(&len_ab, &len_ca)? {
        angle(&len_ab, &len_bc, &len_ca)?
    } else if hreal_is_le(&len_bc, &len_ab)? && hreal_is_le(&len_bc, &len_ca)? {
        angle(&len_bc, &len_ca, &len_ab)?
    } else {
        angle(&len_ca, &len_ab, &len_bc)?
    };
    let max_angle = if hreal_is_ge(&len_ab, &len_bc)? && hreal_is_ge(&len_ab, &len_ca)? {
        angle(&len_ab, &len_bc, &len_ca)?
    } else if hreal_is_ge(&len_bc, &len_ab)? && hreal_is_ge(&len_bc, &len_ca)? {
        angle(&len_bc, &len_ca, &len_ab)?
    } else {
        angle(&len_ca, &len_ab, &len_bc)?
    };

    let semiperimeter = ((len_ab.clone() + len_bc.clone() + len_ca.clone())
        / hreal_from_f64(2.0).ok()?)
    .ok()?;
    let circumradius =
        (len_ab * len_bc * len_ca / (hreal_from_f64(4.0).ok()? * area.clone())).ok()?;
    let inradius = (area.clone() / semiperimeter).ok()?;
    let aspect_ratio = (circumradius / inradius).ok()?;

    let angle_quality = hreal_clamp_hreal(
        (min_angle.clone() / hreal_from_f64(PI / 6.0).ok()?).ok()?,
        0.0,
        1.0,
    )?;
    let shape_quality = hreal_clamp_hreal(
        (hreal_from_f64(1.0).ok()? / aspect_ratio.clone()).ok()?,
        0.0,
        1.0,
    )?;
    let edge_quality = hreal_clamp_hreal(
        (hreal_from_f64(3.0).ok()? / edge_ratio.clone()).ok()?,
        0.0,
        1.0,
    )?;

    let quality_score = hreal_clamp_hreal(
        hreal_from_f64(0.4).ok()? * angle_quality
            + hreal_from_f64(0.4).ok()? * shape_quality
            + hreal_from_f64(0.2).ok()? * edge_quality,
        0.0,
        1.0,
    )?;

    Some(TriangleQuality {
        aspect_ratio: hreal_to_f64(&aspect_ratio)?,
        min_angle: hreal_to_f64(&min_angle)?,
        max_angle: hreal_to_f64(&max_angle)?,
        edge_ratio: hreal_to_f64(&edge_ratio)?,
        area: hreal_to_f64(&area)?,
        quality_score: hreal_to_f64(&quality_score)?,
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
    /// `hyperreal::Real`, then exported to finite scalars for this reporting
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
    /// `hyperreal::Real` boundary comparators used by per-triangle metrics;
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
                avg_quality: 0.0,
                min_quality: 0.0,
                high_quality_ratio: 0.0,
                sliver_count: 0,
                avg_edge_length: 0.0,
                edge_length_std: 0.0,
            };
        }

        let quality_scores = qualities.iter().map(|q| q.quality_score).collect::<Vec<_>>();
        let avg_quality = hreal_mean(&quality_scores).unwrap_or(0.0);

        let min_quality =
            boundary_scalar_min(qualities.iter().map(|q| q.quality_score)).unwrap_or(0.0);

        let high_quality_count = qualities
            .iter()
            .filter(|q| boundary_scalar_gt(q.quality_score, 0.7))
            .count();
        let high_quality_ratio =
            hreal_div(high_quality_count as Real, qualities.len() as Real).unwrap_or(0.0);

        let sliver_threshold = hdegrees_to_radians(10.0).unwrap_or(0.0);
        let sliver_count = qualities
            .iter()
            .filter(|q| boundary_scalar_lt(q.min_angle, sliver_threshold))
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

        let avg_edge_length = hreal_mean(&edge_lengths).unwrap_or(0.0);

        let edge_length_std = hreal_sample_stddev(&edge_lengths).unwrap_or(0.0);

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
