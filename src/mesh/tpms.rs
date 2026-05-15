//! Triply‑Periodic Minimal Surfaces rewritten to leverage the generic
//! signed‑distance mesher in `sdf.rs`.

use crate::csg::CSG;
use crate::float_types::Real;
use crate::mesh::Mesh;
use nalgebra::Point3;
use std::fmt::Debug;

impl<M: Clone + Debug + Send + Sync> Mesh<M> {
    /// **Generic helper** – build a TPMS inside `self` from the provided SDF.
    ///
    /// * `sdf_fn`     – smooth signed‑distance field _f(p)_; 0‑level set is the surface
    /// * `resolution` – voxel grid sampling resolution `(nx, ny, nz)`
    /// * `iso_value`  – iso‑contour value (normally 0.0)
    ///
    /// The result is sampled inside `self`'s bounding box. The TPMS output is an
    /// open implicit surface, so it is not boolean-intersected with `self`; BSP
    /// solid booleans assume closed volumes and can delete valid sheet triangles.
    #[inline]
    fn tpms_from_sdf<F>(
        &self,
        sdf_fn: F,
        resolution: (usize, usize, usize),
        iso_value: Real,
        metadata: M,
    ) -> Mesh<M>
    where
        F: Fn(&Point3<Real>) -> Real + Send + Sync,
    {
        let aabb = self.bounding_box();
        let min_pt = aabb.mins;
        let max_pt = aabb.maxs;
        // Mesh the implicit surface with the generic surface‑nets backend
        Mesh::sdf(sdf_fn, resolution, min_pt, max_pt, iso_value, metadata)
    }

    /// Build a capped, finite-thickness TPMS solid inside `self`'s bounding box.
    ///
    /// The implicit solid is:
    ///
    /// `max(abs(f(p) - iso_value) - thickness / 2, box_sdf(p)) <= 0`
    ///
    /// This creates a wall around the TPMS sheet and caps it where it meets the
    /// bounding box. That makes a closed mesh suitable for solid workflows,
    /// unlike the raw zero-level TPMS sheet.
    #[inline]
    fn tpms_solid_from_sdf<F>(
        &self,
        sdf_fn: F,
        resolution: (usize, usize, usize),
        iso_value: Real,
        thickness: Real,
        metadata: M,
    ) -> Mesh<M>
    where
        F: Fn(&Point3<Real>) -> Real + Send + Sync,
    {
        if !thickness.is_finite() || thickness <= 0.0 {
            return Mesh::empty(metadata);
        }

        let aabb = self.bounding_box();
        let min_pt = aabb.mins;
        let max_pt = aabb.maxs;
        let half_thickness = thickness * 0.5;
        let step = max_axis_step(&min_pt, &max_pt, resolution);
        let padding = step.max(thickness);
        let sample_min = Point3::from(min_pt.coords - nalgebra::Vector3::repeat(padding));
        let sample_max = Point3::from(max_pt.coords + nalgebra::Vector3::repeat(padding));

        Mesh::sdf(
            move |p: &Point3<Real>| {
                let sheet = (sdf_fn(p) - iso_value).abs() - half_thickness;
                let bounds = axis_aligned_box_sdf(p, &min_pt, &max_pt);
                sheet.max(bounds)
            },
            resolution,
            sample_min,
            sample_max,
            0.0,
            metadata,
        )
    }

    // ------------  Specific minimal‑surface flavours  --------------------

    /// Gyroid surface:  `sin x cos y + sin y cos z + sin z cos x = iso`
    /// after scaling coordinates by `2π / period`.
    ///
    /// `period` is a spatial wavelength in model units; larger values repeat
    /// more slowly.
    /// **Mathematical Foundation**: Gyroid is a triply periodic minimal surface with zero mean curvature.
    /// **Optimization**: Pre-compute trigonometric values for better performance.
    pub fn gyroid(
        &self,
        resolution: usize,
        period: Real,
        iso_value: Real,
        metadata: M,
    ) -> Mesh<M> {
        let res = (resolution.max(2), resolution.max(2), resolution.max(2));
        let scale = std::f64::consts::TAU as Real / period;
        self.tpms_from_sdf(
            move |p: &Point3<Real>| {
                // Pre-compute scaled coordinates for efficiency
                let x_scaled = p.x * scale;
                let y_scaled = p.y * scale;
                let z_scaled = p.z * scale;

                // Pre-compute trigonometric values to avoid redundant calculations
                let (sin_x, cos_x) = x_scaled.sin_cos();
                let (sin_y, cos_y) = y_scaled.sin_cos();
                let (sin_z, cos_z) = z_scaled.sin_cos();

                // **Mathematical Formula**: Gyroid surface equation
                // G(x,y,z) = sin(x)cos(y) + sin(y)cos(z) + sin(z)cos(x)
                (sin_x * cos_y) + (sin_y * cos_z) + (sin_z * cos_x)
            },
            res,
            iso_value,
            metadata,
        )
    }

    /// Generate a capped, finite-thickness gyroid solid inside `self`'s
    /// bounding box.
    pub fn gyroid_solid(
        &self,
        resolution: usize,
        period: Real,
        iso_value: Real,
        thickness: Real,
        metadata: M,
    ) -> Mesh<M> {
        let res = (resolution.max(2), resolution.max(2), resolution.max(2));
        let scale = std::f64::consts::TAU as Real / period;
        self.tpms_solid_from_sdf(
            move |p: &Point3<Real>| {
                let x_scaled = p.x * scale;
                let y_scaled = p.y * scale;
                let z_scaled = p.z * scale;
                let (sin_x, cos_x) = x_scaled.sin_cos();
                let (sin_y, cos_y) = y_scaled.sin_cos();
                let (sin_z, cos_z) = z_scaled.sin_cos();
                (sin_x * cos_y) + (sin_y * cos_z) + (sin_z * cos_x)
            },
            res,
            iso_value,
            thickness,
            metadata,
        )
    }

    /// Schwarz‑P surface:  `cos x + cos y + cos z = iso`  (default iso = 0)
    /// after scaling coordinates by `2π / period`.
    /// **Mathematical Foundation**: Schwarz P-surface has constant mean curvature and cubic symmetry.
    /// **Optimization**: Use direct cosine computation for this simpler surface equation.
    pub fn schwarz_p(
        &self,
        resolution: usize,
        period: Real,
        iso_value: Real,
        metadata: M,
    ) -> Mesh<M> {
        let res = (resolution.max(2), resolution.max(2), resolution.max(2));
        let scale = std::f64::consts::TAU as Real / period;
        self.tpms_from_sdf(
            move |p: &Point3<Real>| {
                // Pre-compute scaled coordinates
                let x_scaled = p.x * scale;
                let y_scaled = p.y * scale;
                let z_scaled = p.z * scale;

                // **Mathematical Formula**: Schwarz P-surface equation
                // P(x,y,z) = cos(x) + cos(y) + cos(z)
                x_scaled.cos() + y_scaled.cos() + z_scaled.cos()
            },
            res,
            iso_value,
            metadata,
        )
    }

    /// Generate a capped, finite-thickness Schwarz-P solid inside `self`'s
    /// bounding box.
    pub fn schwarz_p_solid(
        &self,
        resolution: usize,
        period: Real,
        iso_value: Real,
        thickness: Real,
        metadata: M,
    ) -> Mesh<M> {
        let res = (resolution.max(2), resolution.max(2), resolution.max(2));
        let scale = std::f64::consts::TAU as Real / period;
        self.tpms_solid_from_sdf(
            move |p: &Point3<Real>| {
                let x_scaled = p.x * scale;
                let y_scaled = p.y * scale;
                let z_scaled = p.z * scale;
                x_scaled.cos() + y_scaled.cos() + z_scaled.cos()
            },
            res,
            iso_value,
            thickness,
            metadata,
        )
    }

    /// Schwarz‑D (Diamond) surface:  `sin x sin y sin z + sin x cos y cos z + ... = iso`
    /// after scaling coordinates by `2π / period`.
    /// **Mathematical Foundation**: Diamond surface exhibits tetrahedral symmetry and is self-intersecting.
    /// **Optimization**: Pre-compute all trigonometric values for maximum efficiency.
    pub fn schwarz_d(
        &self,
        resolution: usize,
        period: Real,
        iso_value: Real,
        metadata: M,
    ) -> Mesh<M> {
        let res = (resolution.max(2), resolution.max(2), resolution.max(2));
        let scale = std::f64::consts::TAU as Real / period;
        self.tpms_from_sdf(
            move |p: &Point3<Real>| {
                // Pre-compute scaled coordinates
                let x_scaled = p.x * scale;
                let y_scaled = p.y * scale;
                let z_scaled = p.z * scale;

                // Pre-compute all trigonometric values once
                let (sin_x, cos_x) = x_scaled.sin_cos();
                let (sin_y, cos_y) = y_scaled.sin_cos();
                let (sin_z, cos_z) = z_scaled.sin_cos();

                // **Mathematical Formula**: Schwarz Diamond surface equation
                // D(x,y,z) = sin(x)sin(y)sin(z) + sin(x)cos(y)cos(z) + cos(x)sin(y)cos(z) + cos(x)cos(y)sin(z)
                (sin_x * sin_y * sin_z)
                    + (sin_x * cos_y * cos_z)
                    + (cos_x * sin_y * cos_z)
                    + (cos_x * cos_y * sin_z)
            },
            res,
            iso_value,
            metadata,
        )
    }

    /// Generate a capped, finite-thickness Schwarz-D solid inside `self`'s
    /// bounding box.
    pub fn schwarz_d_solid(
        &self,
        resolution: usize,
        period: Real,
        iso_value: Real,
        thickness: Real,
        metadata: M,
    ) -> Mesh<M> {
        let res = (resolution.max(2), resolution.max(2), resolution.max(2));
        let scale = std::f64::consts::TAU as Real / period;
        self.tpms_solid_from_sdf(
            move |p: &Point3<Real>| {
                let x_scaled = p.x * scale;
                let y_scaled = p.y * scale;
                let z_scaled = p.z * scale;
                let (sin_x, cos_x) = x_scaled.sin_cos();
                let (sin_y, cos_y) = y_scaled.sin_cos();
                let (sin_z, cos_z) = z_scaled.sin_cos();
                (sin_x * sin_y * sin_z)
                    + (sin_x * cos_y * cos_z)
                    + (cos_x * sin_y * cos_z)
                    + (cos_x * cos_y * sin_z)
            },
            res,
            iso_value,
            thickness,
            metadata,
        )
    }
}

fn axis_aligned_box_sdf(p: &Point3<Real>, min: &Point3<Real>, max: &Point3<Real>) -> Real {
    let center = Point3::from((min.coords + max.coords) * 0.5);
    let half = (max.coords - min.coords) * 0.5;
    let q = (p - center).abs() - half;
    let outside = q.map(|component| component.max(0.0)).norm();
    let inside = q.x.max(q.y).max(q.z).min(0.0);
    outside + inside
}

fn max_axis_step(
    min: &Point3<Real>,
    max: &Point3<Real>,
    resolution: (usize, usize, usize),
) -> Real {
    let span = max - min;
    let dx = span.x.abs() / (resolution.0.max(2) as Real - 1.0);
    let dy = span.y.abs() / (resolution.1.max(2) as Real - 1.0);
    let dz = span.z.abs() / (resolution.2.max(2) as Real - 1.0);
    dx.max(dy).max(dz)
}
