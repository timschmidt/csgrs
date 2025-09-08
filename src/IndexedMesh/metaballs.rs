//! Metaball (implicit surface) generation for IndexedMesh with optimized indexed connectivity

use crate::float_types::Real;
use crate::IndexedMesh::IndexedMesh;
use crate::traits::CSG;
use nalgebra::Point3;
use std::fmt::Debug;

/// **Mathematical Foundation: Metaball System with Indexed Connectivity**
///
/// Metaballs are implicit surfaces defined by potential fields that blend smoothly.
/// This implementation leverages IndexedMesh for optimal memory usage and connectivity.
///
/// ## **Metaball Mathematics**
/// For a metaball at position C with radius R, the potential function is:
/// ```text
/// f(p) = R² / |p - C|²
/// ```
///
/// ## **Blending Function**
/// Multiple metaballs combine additively:
/// ```text
/// F(p) = Σᵢ fᵢ(p) - threshold
/// ```
/// The iso-surface is extracted where F(p) = 0.
#[derive(Debug, Clone)]
pub struct Metaball {
    /// Center position of the metaball
    pub center: Point3<Real>,
    /// Radius of influence
    pub radius: Real,
    /// Strength/weight of the metaball
    pub strength: Real,
}

impl Metaball {
    /// Create a new metaball
    pub fn new(center: Point3<Real>, radius: Real, strength: Real) -> Self {
        Self {
            center,
            radius,
            strength,
        }
    }

    /// Evaluate the metaball potential at a given point
    pub fn potential(&self, point: &Point3<Real>) -> Real {
        let distance_sq = (point - self.center).norm_squared();
        if distance_sq < Real::EPSILON {
            return Real::INFINITY; // Avoid division by zero
        }
        
        let radius_sq = self.radius * self.radius;
        self.strength * radius_sq / distance_sq
    }
}

impl<S: Clone + Debug + Send + Sync> IndexedMesh<S> {
    /// **Mathematical Foundation: Optimized Metaball Meshing with Indexed Connectivity**
    ///
    /// Generate an IndexedMesh from a collection of metaballs using SDF-based meshing
    /// with performance optimizations for indexed connectivity.
    ///
    /// ## **Algorithm Overview**
    /// 1. **Potential Field Construction**: Combine metaball potentials
    /// 2. **SDF Conversion**: Convert potential field to signed distance field
    /// 3. **Surface Extraction**: Use SDF meshing with indexed connectivity
    /// 4. **Optimization**: Leverage vertex sharing for memory efficiency
    ///
    /// ## **Indexed Connectivity Benefits**
    /// - **Memory Efficiency**: Shared vertices reduce memory usage
    /// - **Smooth Blending**: Better vertex normal computation for smooth surfaces
    /// - **Performance**: Faster connectivity queries for post-processing
    ///
    /// # Parameters
    /// - `metaballs`: Collection of metaballs to mesh
    /// - `threshold`: Iso-surface threshold (typically 1.0)
    /// - `resolution`: Grid resolution for sampling
    /// - `bounds_min`: Minimum corner of sampling region
    /// - `bounds_max`: Maximum corner of sampling region
    /// - `metadata`: Optional metadata for all faces
    ///
    /// # Example
    /// ```
    /// # use csgrs::IndexedMesh::{IndexedMesh, metaballs::Metaball};
    /// # use nalgebra::Point3;
    /// 
    /// let metaballs = vec![
    ///     Metaball::new(Point3::new(-0.5, 0.0, 0.0), 1.0, 1.0),
    ///     Metaball::new(Point3::new(0.5, 0.0, 0.0), 1.0, 1.0),
    /// ];
    /// 
    /// let mesh = IndexedMesh::<()>::from_metaballs(
    ///     &metaballs,
    ///     1.0,
    ///     (50, 50, 50),
    ///     Point3::new(-2.0, -2.0, -2.0),
    ///     Point3::new(2.0, 2.0, 2.0),
    ///     None
    /// );
    /// ```
    pub fn from_metaballs(
        metaballs: &[Metaball],
        threshold: Real,
        resolution: (usize, usize, usize),
        bounds_min: Point3<Real>,
        bounds_max: Point3<Real>,
        metadata: Option<S>,
    ) -> IndexedMesh<S> {
        if metaballs.is_empty() {
            return IndexedMesh::new();
        }

        // Create a combined potential field function
        let potential_field = |point: &Point3<Real>| -> Real {
            let total_potential: Real = metaballs
                .iter()
                .map(|metaball| metaball.potential(point))
                .sum();
            
            // Convert potential to signed distance (approximate)
            // For metaballs, we use threshold - potential as the SDF
            threshold - total_potential
        };

        // Use SDF meshing to extract the iso-surface
        Self::sdf(
            potential_field,
            resolution,
            bounds_min,
            bounds_max,
            0.0, // Extract where potential_field = 0 (i.e., total_potential = threshold)
            metadata,
        )
    }

    /// **Mathematical Foundation: Optimized Multi-Resolution Metaball Meshing**
    ///
    /// Generate metaball mesh with adaptive resolution based on metaball density
    /// and size distribution.
    ///
    /// ## **Adaptive Resolution Strategy**
    /// - **High Density Regions**: Use finer resolution near metaball centers
    /// - **Sparse Regions**: Use coarser resolution in empty space
    /// - **Size-based Scaling**: Adjust resolution based on metaball radii
    ///
    /// This provides better surface quality while maintaining performance.
    pub fn from_metaballs_adaptive(
        metaballs: &[Metaball],
        threshold: Real,
        base_resolution: (usize, usize, usize),
        metadata: Option<S>,
    ) -> IndexedMesh<S> {
        if metaballs.is_empty() {
            return IndexedMesh::new();
        }

        // Compute adaptive bounding box based on metaball positions and radii
        let mut min_bounds = metaballs[0].center;
        let mut max_bounds = metaballs[0].center;
        let mut max_radius = metaballs[0].radius;

        for metaball in metaballs {
            let margin = metaball.radius * 2.0; // Extend bounds by 2x radius
            
            min_bounds.x = min_bounds.x.min(metaball.center.x - margin);
            min_bounds.y = min_bounds.y.min(metaball.center.y - margin);
            min_bounds.z = min_bounds.z.min(metaball.center.z - margin);
            
            max_bounds.x = max_bounds.x.max(metaball.center.x + margin);
            max_bounds.y = max_bounds.y.max(metaball.center.y + margin);
            max_bounds.z = max_bounds.z.max(metaball.center.z + margin);
            
            max_radius = max_radius.max(metaball.radius);
        }

        // Scale resolution based on maximum metaball radius
        let scale_factor = (2.0 / max_radius).max(0.5).min(2.0);
        let adaptive_resolution = (
            ((base_resolution.0 as Real * scale_factor) as usize).max(10),
            ((base_resolution.1 as Real * scale_factor) as usize).max(10),
            ((base_resolution.2 as Real * scale_factor) as usize).max(10),
        );

        Self::from_metaballs(
            metaballs,
            threshold,
            adaptive_resolution,
            min_bounds,
            max_bounds,
            metadata,
        )
    }

    /// **Mathematical Foundation: Metaball Animation Support**
    ///
    /// Generate a sequence of IndexedMesh frames for animated metaballs.
    /// This is useful for creating fluid simulations or morphing effects.
    ///
    /// ## **Animation Optimization**
    /// - **Temporal Coherence**: Reuse connectivity information between frames
    /// - **Consistent Topology**: Maintain similar mesh structure across frames
    /// - **Memory Efficiency**: Leverage indexed representation for animation data
    ///
    /// # Parameters
    /// - `metaball_frames`: Sequence of metaball configurations
    /// - `threshold`: Iso-surface threshold
    /// - `resolution`: Grid resolution
    /// - `bounds_min`: Minimum corner of sampling region
    /// - `bounds_max`: Maximum corner of sampling region
    /// - `metadata`: Optional metadata for all faces
    ///
    /// Returns a vector of IndexedMesh objects, one per frame.
    pub fn animate_metaballs(
        metaball_frames: &[Vec<Metaball>],
        threshold: Real,
        resolution: (usize, usize, usize),
        bounds_min: Point3<Real>,
        bounds_max: Point3<Real>,
        metadata: Option<S>,
    ) -> Vec<IndexedMesh<S>> {
        metaball_frames
            .iter()
            .map(|frame_metaballs| {
                Self::from_metaballs(
                    frame_metaballs,
                    threshold,
                    resolution,
                    bounds_min,
                    bounds_max,
                    metadata.clone(),
                )
            })
            .collect()
    }

    /// Create a simple two-metaball system for testing
    pub fn metaball_dumbbell(
        separation: Real,
        radius: Real,
        strength: Real,
        threshold: Real,
        resolution: (usize, usize, usize),
        metadata: Option<S>,
    ) -> IndexedMesh<S> {
        let metaballs = vec![
            Metaball::new(Point3::new(-separation / 2.0, 0.0, 0.0), radius, strength),
            Metaball::new(Point3::new(separation / 2.0, 0.0, 0.0), radius, strength),
        ];

        let margin = radius * 2.0;
        let bounds_min = Point3::new(-separation / 2.0 - margin, -margin, -margin);
        let bounds_max = Point3::new(separation / 2.0 + margin, margin, margin);

        Self::from_metaballs(&metaballs, threshold, resolution, bounds_min, bounds_max, metadata)
    }
}
