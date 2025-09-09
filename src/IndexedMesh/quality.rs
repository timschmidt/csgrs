//! Mesh quality analysis and optimization for IndexedMesh with indexed connectivity

use crate::IndexedMesh::IndexedMesh;
use crate::float_types::{PI, Real};

use std::fmt::Debug;

#[cfg(feature = "parallel")]
use rayon::prelude::*;

/// **Mathematical Foundation: Triangle Quality Metrics with Indexed Connectivity**
///
/// Comprehensive triangle quality assessment optimized for indexed mesh representations:
///
/// ## **Indexed Connectivity Advantages**
/// - **Direct Vertex Access**: O(1) vertex lookup using indices
/// - **Memory Efficiency**: No vertex duplication in quality computations
/// - **Cache Performance**: Better memory locality through index-based access
/// - **Precision Preservation**: Avoids coordinate copying and floating-point errors
///
/// ## **Quality Metrics**
/// - **Aspect Ratio**: R/(2r) where R=circumradius, r=inradius
/// - **Minimum Angle**: Smallest interior angle (sliver detection)
/// - **Edge Length Ratio**: max_edge/min_edge (shape regularity)
/// - **Area**: Triangle area for size-based analysis
/// - **Quality Score**: Weighted combination (0-1 scale)
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

/// **Mathematical Foundation: Mesh Quality Assessment with Indexed Connectivity**
///
/// Advanced mesh processing algorithms optimized for indexed representations:
///
/// ## **Statistical Analysis**
/// - **Quality Distribution**: Histogram of triangle quality scores
/// - **Outlier Detection**: Identification of problematic triangles
/// - **Performance Metrics**: Edge length uniformity and valence regularity
///
/// ## **Optimization Benefits**
/// - **Index-based Iteration**: Direct access to vertex data via indices
/// - **Reduced Memory Allocation**: No temporary vertex copies
/// - **Vectorized Operations**: Better SIMD utilization through structured access
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

impl<S: Clone + Debug + Send + Sync> IndexedMesh<S> {
    /// **Mathematical Foundation: Optimized Triangle Quality Analysis**
    ///
    /// Analyze triangle quality using indexed connectivity for superior performance:
    ///
    /// ## **Algorithm Optimization**
    /// 1. **Direct Index Access**: Vertices accessed via indices, no coordinate lookup
    /// 2. **Vectorized Computation**: SIMD-friendly operations on vertex arrays
    /// 3. **Memory Locality**: Sequential access patterns for cache efficiency
    /// 4. **Parallel Processing**: Optional parallelization using rayon
    ///
    /// ## **Quality Computation Pipeline**
    /// For each triangle with vertex indices [i, j, k]:
    /// 1. **Vertex Retrieval**: vertices[i], vertices[j], vertices[k]
    /// 2. **Geometric Analysis**: Edge lengths, angles, area computation
    /// 3. **Quality Metrics**: Aspect ratio, edge ratio, quality score
    /// 4. **Statistical Aggregation**: Min, max, average quality measures
    ///
    /// Returns quality metrics for each triangle in the mesh.
    pub fn analyze_triangle_quality(&self) -> Vec<TriangleQuality> {
        let triangulated = self.triangulate();

        #[cfg(feature = "parallel")]
        let qualities: Vec<TriangleQuality> = triangulated
            .polygons
            .par_iter()
            .map(|poly| {
                Self::compute_triangle_quality_indexed(&triangulated.vertices, &poly.indices)
            })
            .collect();

        #[cfg(not(feature = "parallel"))]
        let qualities: Vec<TriangleQuality> = triangulated
            .polygons
            .iter()
            .map(|poly| {
                Self::compute_triangle_quality_indexed(&triangulated.vertices, &poly.indices)
            })
            .collect();

        qualities
    }

    /// **Mathematical Foundation: Optimized Triangle Quality Computation**
    ///
    /// Compute comprehensive quality metrics for a single triangle using indexed access:
    ///
    /// ## **Geometric Computations**
    /// - **Edge Vectors**: Direct computation from indexed vertices
    /// - **Area Calculation**: Cross product magnitude / 2
    /// - **Angle Computation**: Law of cosines with numerical stability
    /// - **Circumradius**: R = abc/(4A) where a,b,c are edge lengths
    /// - **Inradius**: r = A/s where s is semiperimeter
    ///
    /// ## **Quality Score Formula**
    /// ```text
    /// Q = 0.4 × angle_quality + 0.4 × shape_quality + 0.2 × edge_quality
    /// ```
    /// Where each component is normalized to [0,1] range.
    fn compute_triangle_quality_indexed(
        vertices: &[crate::IndexedMesh::vertex::IndexedVertex],
        indices: &[usize],
    ) -> TriangleQuality {
        if indices.len() != 3 {
            return TriangleQuality {
                aspect_ratio: Real::INFINITY,
                min_angle: 0.0,
                max_angle: 0.0,
                edge_ratio: Real::INFINITY,
                area: 0.0,
                quality_score: 0.0,
            };
        }

        // Direct indexed vertex access - O(1) lookup
        let a = vertices[indices[0]].pos;
        let b = vertices[indices[1]].pos;
        let c = vertices[indices[2]].pos;

        // Edge vectors and lengths
        let ab = b - a;
        let bc = c - b;
        let ca = a - c;

        let len_ab = ab.norm();
        let len_bc = bc.norm();
        let len_ca = ca.norm();

        // Handle degenerate cases
        if len_ab < Real::EPSILON || len_bc < Real::EPSILON || len_ca < Real::EPSILON {
            return TriangleQuality {
                aspect_ratio: Real::INFINITY,
                min_angle: 0.0,
                max_angle: 0.0,
                edge_ratio: Real::INFINITY,
                area: 0.0,
                quality_score: 0.0,
            };
        }

        // Triangle area using cross product
        let area = 0.5 * ab.cross(&(-ca)).norm();

        if area < Real::EPSILON {
            return TriangleQuality {
                aspect_ratio: Real::INFINITY,
                min_angle: 0.0,
                max_angle: 0.0,
                edge_ratio: len_ab.max(len_bc).max(len_ca) / len_ab.min(len_bc).min(len_ca),
                area: 0.0,
                quality_score: 0.0,
            };
        }

        // Interior angles using law of cosines with numerical stability
        let angle_a = Self::safe_acos(
            (len_bc.powi(2) + len_ca.powi(2) - len_ab.powi(2)) / (2.0 * len_bc * len_ca),
        );
        let angle_b = Self::safe_acos(
            (len_ca.powi(2) + len_ab.powi(2) - len_bc.powi(2)) / (2.0 * len_ca * len_ab),
        );
        let angle_c = Self::safe_acos(
            (len_ab.powi(2) + len_bc.powi(2) - len_ca.powi(2)) / (2.0 * len_ab * len_bc),
        );

        let min_angle = angle_a.min(angle_b).min(angle_c);
        let max_angle = angle_a.max(angle_b).max(angle_c);

        // Edge length ratio
        let min_edge = len_ab.min(len_bc).min(len_ca);
        let max_edge = len_ab.max(len_bc).max(len_ca);
        let edge_ratio = max_edge / min_edge;

        // Aspect ratio (circumradius to inradius ratio)
        let semiperimeter = (len_ab + len_bc + len_ca) / 2.0;
        let circumradius = (len_ab * len_bc * len_ca) / (4.0 * area);
        let inradius = area / semiperimeter;
        let aspect_ratio = circumradius / inradius;

        // Quality score: weighted combination of metrics
        let angle_quality = (min_angle / (PI / 6.0)).min(1.0); // Normalized to 30°
        let shape_quality = (1.0 / aspect_ratio).min(1.0);
        let edge_quality = (3.0 / edge_ratio).min(1.0);

        let quality_score =
            (0.4 * angle_quality + 0.4 * shape_quality + 0.2 * edge_quality).clamp(0.0, 1.0);

        TriangleQuality {
            aspect_ratio,
            min_angle,
            max_angle,
            edge_ratio,
            area,
            quality_score,
        }
    }

    /// Safe arccosine computation with clamping to avoid NaN
    fn safe_acos(x: Real) -> Real {
        x.clamp(-1.0, 1.0).acos()
    }

    /// **Mathematical Foundation: Comprehensive Mesh Quality Assessment**
    ///
    /// Compute mesh-wide quality statistics using indexed connectivity:
    ///
    /// ## **Statistical Measures**
    /// - **Quality Distribution**: Mean, min, max triangle quality
    /// - **Outlier Analysis**: Sliver triangle detection (min_angle < 10°)
    /// - **Uniformity Metrics**: Edge length variation analysis
    ///
    /// ## **Performance Optimization**
    /// - **Index-based Edge Extraction**: Direct polygon traversal
    /// - **Vectorized Statistics**: SIMD-friendly aggregation operations
    /// - **Memory Efficiency**: Single-pass computation without temporary storage
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

        let total_quality: Real = qualities.iter().map(|q| q.quality_score).sum();
        let avg_quality = total_quality / qualities.len() as Real;

        let min_quality = qualities
            .iter()
            .map(|q| q.quality_score)
            .fold(Real::INFINITY, |a, b| a.min(b));

        let high_quality_count = qualities.iter().filter(|q| q.quality_score > 0.7).count();
        let high_quality_ratio = high_quality_count as Real / qualities.len() as Real;

        let sliver_count = qualities
            .iter()
            .filter(|q| q.min_angle < (10.0 as Real).to_radians())
            .count();

        // Compute edge length statistics using indexed connectivity
        let edge_lengths: Vec<Real> = self
            .polygons
            .iter()
            .flat_map(|poly| {
                (0..poly.indices.len()).map(move |i| {
                    let v1 = &self.vertices[poly.indices[i]];
                    let v2 = &self.vertices[poly.indices[(i + 1) % poly.indices.len()]];
                    (v2.pos - v1.pos).norm()
                })
            })
            .collect();

        let avg_edge_length = if !edge_lengths.is_empty() {
            edge_lengths.iter().sum::<Real>() / edge_lengths.len() as Real
        } else {
            0.0
        };

        let edge_length_variance = if edge_lengths.len() > 1 {
            let variance: Real = edge_lengths
                .iter()
                .map(|&len| (len - avg_edge_length).powi(2))
                .sum::<Real>()
                / (edge_lengths.len() - 1) as Real;
            variance.sqrt()
        } else {
            0.0
        };

        MeshQualityMetrics {
            avg_quality,
            min_quality,
            high_quality_ratio,
            sliver_count,
            avg_edge_length,
            edge_length_std: edge_length_variance,
        }
    }
}
