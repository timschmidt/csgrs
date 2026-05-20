//! Triply‑Periodic Minimal Surfaces rewritten to leverage the generic
//! signed‑distance mesher in `sdf.rs`.

use crate::csg::CSG;
use crate::float_types::{
    HReal, Real, hreal_div, hreal_from_f64, hreal_gt_f64, hreal_to_f64, hvector3_from_vector3,
    tolerance,
};
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
        let sample_min =
            Point3::new(min_pt.x - padding, min_pt.y - padding, min_pt.z - padding);
        let sample_max =
            Point3::new(max_pt.x + padding, max_pt.y + padding, max_pt.z + padding);

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
        let Some(scale) = tpms_scale(period) else {
            return Mesh::empty(metadata);
        };
        self.tpms_from_sdf(
            move |p: &Point3<Real>| tpms_gyroid_value(p, scale).unwrap_or(Real::INFINITY),
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
        let Some(scale) = tpms_scale(period) else {
            return Mesh::empty(metadata);
        };
        self.tpms_solid_from_sdf(
            move |p: &Point3<Real>| tpms_gyroid_value(p, scale).unwrap_or(Real::INFINITY),
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
        let Some(scale) = tpms_scale(period) else {
            return Mesh::empty(metadata);
        };
        self.tpms_from_sdf(
            move |p: &Point3<Real>| tpms_schwarz_p_value(p, scale).unwrap_or(Real::INFINITY),
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
        let Some(scale) = tpms_scale(period) else {
            return Mesh::empty(metadata);
        };
        self.tpms_solid_from_sdf(
            move |p: &Point3<Real>| tpms_schwarz_p_value(p, scale).unwrap_or(Real::INFINITY),
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
        let Some(scale) = tpms_scale(period) else {
            return Mesh::empty(metadata);
        };
        self.tpms_from_sdf(
            move |p: &Point3<Real>| tpms_schwarz_d_value(p, scale).unwrap_or(Real::INFINITY),
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
        let Some(scale) = tpms_scale(period) else {
            return Mesh::empty(metadata);
        };
        self.tpms_solid_from_sdf(
            move |p: &Point3<Real>| tpms_schwarz_d_value(p, scale).unwrap_or(Real::INFINITY),
            res,
            iso_value,
            thickness,
            metadata,
        )
    }
}

/// Return the TPMS angular scale `2π / period` in hyperreal space.
///
/// The integer/grid APIs still expose primitive periods, but the wavelength
/// validation and reciprocal are promoted before any implicit-field samples are
/// evaluated. This follows Yap's exact-geometric-computation split between
/// primitive input boundaries and exact-aware predicates
/// (<https://doi.org/10.1016/0925-7721(95)00040-2>). The TPMS families here
/// follow Schoen, "Infinite Periodic Minimal Surfaces Without Self-
/// Intersections," NASA Technical Note D-5541, 1970.
fn tpms_scale(period: Real) -> Option<Real> {
    let period = hreal_from_f64(period).ok()?;
    if !hreal_gt_f64(&period, tolerance()) {
        return None;
    }
    hreal_to_f64(&(hreal_from_f64(std::f64::consts::TAU).ok()? / period).ok()?)
}

fn tpms_scaled_axes(point: &Point3<Real>, scale: Real) -> Option<(HReal, HReal, HReal)> {
    let scale = hreal_from_f64(scale).ok()?;
    Some((
        hreal_from_f64(point.x).ok()? * scale.clone(),
        hreal_from_f64(point.y).ok()? * scale.clone(),
        hreal_from_f64(point.z).ok()? * scale,
    ))
}

/// Evaluate Schoen's gyroid approximation in hyperreal space.
fn tpms_gyroid_value(point: &Point3<Real>, scale: Real) -> Option<Real> {
    let (x, y, z) = tpms_scaled_axes(point, scale)?;
    let sin_x = x.clone().sin();
    let cos_x = x.cos();
    let sin_y = y.clone().sin();
    let cos_y = y.cos();
    let sin_z = z.clone().sin();
    let cos_z = z.cos();
    hreal_to_f64(&(sin_x * cos_y + sin_y * cos_z + sin_z * cos_x))
}

/// Evaluate Schwarz's primitive cubic surface approximation in hyperreal space.
fn tpms_schwarz_p_value(point: &Point3<Real>, scale: Real) -> Option<Real> {
    let (x, y, z) = tpms_scaled_axes(point, scale)?;
    hreal_to_f64(&(x.cos() + y.cos() + z.cos()))
}

/// Evaluate Schwarz's diamond surface approximation in hyperreal space.
fn tpms_schwarz_d_value(point: &Point3<Real>, scale: Real) -> Option<Real> {
    let (x, y, z) = tpms_scaled_axes(point, scale)?;
    let sin_x = x.clone().sin();
    let cos_x = x.cos();
    let sin_y = y.clone().sin();
    let cos_y = y.cos();
    let sin_z = z.clone().sin();
    let cos_z = z.cos();
    hreal_to_f64(
        &(sin_x.clone() * sin_y.clone() * sin_z.clone()
            + sin_x * cos_y.clone() * cos_z.clone()
            + cos_x.clone() * sin_y * cos_z
            + cos_x * cos_y * sin_z),
    )
}

fn axis_aligned_box_sdf(p: &Point3<Real>, min: &Point3<Real>, max: &Point3<Real>) -> Real {
    // The outside distance is a vector magnitude, so evaluate it in
    // hyperlattice and export only the finite SDF boundary value. This keeps
    // TPMS solid capping aligned with Yap's exact-geometric-computation
    // boundary split (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    let center = Point3::from((min.coords + max.coords) * 0.5);
    let half = (max.coords - min.coords) * 0.5;
    let q = (p - center).abs() - half;
    let outside_vec = q.map(|component| component.max(0.0));
    let outside = hvector3_from_vector3(&outside_vec)
        .and_then(|vector| hreal_to_f64(&vector.magnitude().ok()?))
        .unwrap_or(Real::INFINITY);
    let inside = q.x.max(q.y).max(q.z).min(0.0);
    outside + inside
}

fn max_axis_step(
    min: &Point3<Real>,
    max: &Point3<Real>,
    resolution: (usize, usize, usize),
) -> Real {
    let span = max - min;
    let dx = hreal_div(span.x.abs(), resolution.0.max(2) as Real - 1.0).unwrap_or(0.0);
    let dy = hreal_div(span.y.abs(), resolution.1.max(2) as Real - 1.0).unwrap_or(0.0);
    let dz = hreal_div(span.z.abs(), resolution.2.max(2) as Real - 1.0).unwrap_or(0.0);
    dx.max(dy).max(dz)
}
