//! Triply Periodic Minimal Surfaces (TPMS) generation for IndexedMesh with optimized indexed connectivity

use crate::IndexedMesh::IndexedMesh;
use crate::float_types::Real;
use nalgebra::Point3;
use std::fmt::Debug;

impl<S: Clone + Debug + Send + Sync> IndexedMesh<S> {
    /// **Mathematical Foundation: Gyroid TPMS with Indexed Connectivity**
    ///
    /// Generate a Gyroid triply periodic minimal surface using SDF-based meshing
    /// with optimized indexed connectivity for superior performance.
    ///
    /// ## **Gyroid Mathematics**
    /// The Gyroid is defined by the implicit equation:
    /// ```text
    /// F(x,y,z) = sin(x)cos(y) + sin(y)cos(z) + sin(z)cos(x) = 0
    /// ```
    ///
    /// ## **Indexed Connectivity Advantages**
    /// - **Memory Efficiency**: Shared vertices reduce memory usage by ~50%
    /// - **Topology Preservation**: Maintains complex TPMS connectivity
    /// - **Performance**: Better cache locality for surface operations
    /// - **Manifold Guarantee**: Ensures valid 2-manifold structure
    ///
    /// ## **Applications**
    /// - **Tissue Engineering**: Scaffolds with controlled porosity
    /// - **Heat Exchangers**: Optimal surface area to volume ratio
    /// - **Metamaterials**: Lightweight structures with unique properties
    /// - **Filtration**: Complex pore networks
    ///
    /// # Parameters
    /// - `scale`: Scaling factor for the periodic structure
    /// - `thickness`: Wall thickness (0.0 = minimal surface)
    /// - `resolution`: Grid resolution for sampling
    /// - `bounds_min`: Minimum corner of sampling region
    /// - `bounds_max`: Maximum corner of sampling region
    /// - `metadata`: Optional metadata for all faces
    ///
    /// # Example
    /// ```
    /// # use csgrs::IndexedMesh::IndexedMesh;
    /// # use nalgebra::Point3;
    ///
    /// let gyroid = IndexedMesh::<()>::gyroid(
    ///     2.0 * std::f64::consts::PI,  // One period
    ///     0.1,                         // Thin walls
    ///     (64, 64, 64),               // High resolution
    ///     Point3::new(-1.0, -1.0, -1.0),
    ///     Point3::new(1.0, 1.0, 1.0),
    ///     None
    /// );
    /// ```
    pub fn gyroid(
        scale: Real,
        thickness: Real,
        resolution: (usize, usize, usize),
        bounds_min: Point3<Real>,
        bounds_max: Point3<Real>,
        metadata: Option<S>,
    ) -> IndexedMesh<S> {
        let gyroid_sdf = move |point: &Point3<Real>| -> Real {
            let x = point.x * scale;
            let y = point.y * scale;
            let z = point.z * scale;

            let gyroid_value = x.sin() * y.cos() + y.sin() * z.cos() + z.sin() * x.cos();

            // Convert to signed distance with thickness
            gyroid_value.abs() - thickness
        };

        Self::sdf(gyroid_sdf, resolution, bounds_min, bounds_max, 0.0, metadata)
    }

    /// **Mathematical Foundation: Schwarz P TPMS with Indexed Connectivity**
    ///
    /// Generate a Schwarz P (Primitive) triply periodic minimal surface.
    ///
    /// ## **Schwarz P Mathematics**
    /// The Schwarz P surface is defined by:
    /// ```text
    /// F(x,y,z) = cos(x) + cos(y) + cos(z) = 0
    /// ```
    ///
    /// ## **Properties**
    /// - **Cubic Symmetry**: Invariant under 90° rotations
    /// - **High Porosity**: Excellent for fluid flow applications
    /// - **Structural Strength**: Good mechanical properties
    ///
    /// # Parameters
    /// - `scale`: Scaling factor for the periodic structure
    /// - `thickness`: Wall thickness (0.0 = minimal surface)
    /// - `resolution`: Grid resolution for sampling
    /// - `bounds_min`: Minimum corner of sampling region
    /// - `bounds_max`: Maximum corner of sampling region
    /// - `metadata`: Optional metadata for all faces
    pub fn schwarz_p(
        scale: Real,
        thickness: Real,
        resolution: (usize, usize, usize),
        bounds_min: Point3<Real>,
        bounds_max: Point3<Real>,
        metadata: Option<S>,
    ) -> IndexedMesh<S> {
        let schwarz_p_sdf = move |point: &Point3<Real>| -> Real {
            let x = point.x * scale;
            let y = point.y * scale;
            let z = point.z * scale;

            let schwarz_value = x.cos() + y.cos() + z.cos();

            // Convert to signed distance with thickness
            schwarz_value.abs() - thickness
        };

        Self::sdf(
            schwarz_p_sdf,
            resolution,
            bounds_min,
            bounds_max,
            0.0,
            metadata,
        )
    }

    /// **Mathematical Foundation: Schwarz D TPMS with Indexed Connectivity**
    ///
    /// Generate a Schwarz D (Diamond) triply periodic minimal surface.
    ///
    /// ## **Schwarz D Mathematics**
    /// The Schwarz D surface is defined by:
    /// ```text
    /// F(x,y,z) = sin(x)sin(y)sin(z) + sin(x)cos(y)cos(z) +
    ///            cos(x)sin(y)cos(z) + cos(x)cos(y)sin(z) = 0
    /// ```
    ///
    /// ## **Properties**
    /// - **Diamond-like Structure**: Similar to diamond crystal lattice
    /// - **High Surface Area**: Excellent for catalytic applications
    /// - **Interconnected Channels**: Good for mass transport
    ///
    /// # Parameters
    /// - `scale`: Scaling factor for the periodic structure
    /// - `thickness`: Wall thickness (0.0 = minimal surface)
    /// - `resolution`: Grid resolution for sampling
    /// - `bounds_min`: Minimum corner of sampling region
    /// - `bounds_max`: Maximum corner of sampling region
    /// - `metadata`: Optional metadata for all faces
    pub fn schwarz_d(
        scale: Real,
        thickness: Real,
        resolution: (usize, usize, usize),
        bounds_min: Point3<Real>,
        bounds_max: Point3<Real>,
        metadata: Option<S>,
    ) -> IndexedMesh<S> {
        let schwarz_d_sdf = move |point: &Point3<Real>| -> Real {
            let x = point.x * scale;
            let y = point.y * scale;
            let z = point.z * scale;

            let schwarz_d_value = x.sin() * y.sin() * z.sin()
                + x.sin() * y.cos() * z.cos()
                + x.cos() * y.sin() * z.cos()
                + x.cos() * y.cos() * z.sin();

            // Convert to signed distance with thickness
            schwarz_d_value.abs() - thickness
        };

        Self::sdf(
            schwarz_d_sdf,
            resolution,
            bounds_min,
            bounds_max,
            0.0,
            metadata,
        )
    }

    /// **Mathematical Foundation: Neovius TPMS with Indexed Connectivity**
    ///
    /// Generate a Neovius triply periodic minimal surface.
    ///
    /// ## **Neovius Mathematics**
    /// The Neovius surface is defined by:
    /// ```text
    /// F(x,y,z) = 3(cos(x) + cos(y) + cos(z)) + 4cos(x)cos(y)cos(z) = 0
    /// ```
    ///
    /// ## **Properties**
    /// - **Complex Topology**: More intricate than Schwarz surfaces
    /// - **Variable Porosity**: Non-uniform pore distribution
    /// - **Aesthetic Appeal**: Visually interesting structure
    ///
    /// # Parameters
    /// - `scale`: Scaling factor for the periodic structure
    /// - `thickness`: Wall thickness (0.0 = minimal surface)
    /// - `resolution`: Grid resolution for sampling
    /// - `bounds_min`: Minimum corner of sampling region
    /// - `bounds_max`: Maximum corner of sampling region
    /// - `metadata`: Optional metadata for all faces
    pub fn neovius(
        scale: Real,
        thickness: Real,
        resolution: (usize, usize, usize),
        bounds_min: Point3<Real>,
        bounds_max: Point3<Real>,
        metadata: Option<S>,
    ) -> IndexedMesh<S> {
        let neovius_sdf = move |point: &Point3<Real>| -> Real {
            let x = point.x * scale;
            let y = point.y * scale;
            let z = point.z * scale;

            let neovius_value =
                3.0 * (x.cos() + y.cos() + z.cos()) + 4.0 * x.cos() * y.cos() * z.cos();

            // Convert to signed distance with thickness
            neovius_value.abs() - thickness
        };

        Self::sdf(neovius_sdf, resolution, bounds_min, bounds_max, 0.0, metadata)
    }

    /// **Mathematical Foundation: I-WP TPMS with Indexed Connectivity**
    ///
    /// Generate an I-WP (Wrapped Package) triply periodic minimal surface.
    ///
    /// ## **I-WP Mathematics**
    /// The I-WP surface is defined by:
    /// ```text
    /// F(x,y,z) = cos(x)cos(y) + cos(y)cos(z) + cos(z)cos(x) -
    ///            cos(x)cos(y)cos(z) = 0
    /// ```
    ///
    /// ## **Properties**
    /// - **Wrapped Structure**: Resembles wrapped packages
    /// - **Moderate Porosity**: Balanced solid/void ratio
    /// - **Good Connectivity**: Well-connected pore network
    ///
    /// # Parameters
    /// - `scale`: Scaling factor for the periodic structure
    /// - `thickness`: Wall thickness (0.0 = minimal surface)
    /// - `resolution`: Grid resolution for sampling
    /// - `bounds_min`: Minimum corner of sampling region
    /// - `bounds_max`: Maximum corner of sampling region
    /// - `metadata`: Optional metadata for all faces
    pub fn i_wp(
        scale: Real,
        thickness: Real,
        resolution: (usize, usize, usize),
        bounds_min: Point3<Real>,
        bounds_max: Point3<Real>,
        metadata: Option<S>,
    ) -> IndexedMesh<S> {
        let i_wp_sdf = move |point: &Point3<Real>| -> Real {
            let x = point.x * scale;
            let y = point.y * scale;
            let z = point.z * scale;

            let i_wp_value = x.cos() * y.cos() + y.cos() * z.cos() + z.cos() * x.cos()
                - x.cos() * y.cos() * z.cos();

            // Convert to signed distance with thickness
            i_wp_value.abs() - thickness
        };

        Self::sdf(i_wp_sdf, resolution, bounds_min, bounds_max, 0.0, metadata)
    }

    /// **Mathematical Foundation: Custom TPMS with Indexed Connectivity**
    ///
    /// Generate a custom triply periodic minimal surface from a user-defined function.
    ///
    /// ## **Custom TPMS Design**
    /// This allows for:
    /// - **Novel Structures**: Create new TPMS variants
    /// - **Parameter Studies**: Explore design space systematically
    /// - **Optimization**: Fine-tune properties for specific applications
    ///
    /// # Parameters
    /// - `tpms_function`: Function defining the TPMS implicit surface
    /// - `thickness`: Wall thickness (0.0 = minimal surface)
    /// - `resolution`: Grid resolution for sampling
    /// - `bounds_min`: Minimum corner of sampling region
    /// - `bounds_max`: Maximum corner of sampling region
    /// - `metadata`: Optional metadata for all faces
    ///
    /// # Example
    /// ```
    /// # use csgrs::IndexedMesh::IndexedMesh;
    /// # use nalgebra::Point3;
    ///
    /// // Custom TPMS combining Gyroid and Schwarz P
    /// let custom_tpms = |point: &Point3<f64>| -> f64 {
    ///     let x = point.x * 2.0 * std::f64::consts::PI;
    ///     let y = point.y * 2.0 * std::f64::consts::PI;
    ///     let z = point.z * 2.0 * std::f64::consts::PI;
    ///     
    ///     let gyroid = x.sin() * y.cos() + y.sin() * z.cos() + z.sin() * x.cos();
    ///     let schwarz_p = x.cos() + y.cos() + z.cos();
    ///     
    ///     0.5 * gyroid + 0.5 * schwarz_p
    /// };
    ///
    /// let mesh = IndexedMesh::<()>::custom_tpms(
    ///     custom_tpms,
    ///     0.1,
    ///     (64, 64, 64),
    ///     Point3::new(-1.0, -1.0, -1.0),
    ///     Point3::new(1.0, 1.0, 1.0),
    ///     None
    /// );
    /// ```
    pub fn custom_tpms<F>(
        tpms_function: F,
        thickness: Real,
        resolution: (usize, usize, usize),
        bounds_min: Point3<Real>,
        bounds_max: Point3<Real>,
        metadata: Option<S>,
    ) -> IndexedMesh<S>
    where
        F: Fn(&Point3<Real>) -> Real + Sync + Send,
    {
        let tpms_sdf = move |point: &Point3<Real>| -> Real {
            let tpms_value = tpms_function(point);

            // Convert to signed distance with thickness
            tpms_value.abs() - thickness
        };

        Self::sdf(tpms_sdf, resolution, bounds_min, bounds_max, 0.0, metadata)
    }
}
