//! Triply‑Periodic Minimal Surfaces rewritten to leverage the generic
//! signed‑distance mesher in `sdf.rs`.

use crate::csg::CSG;
use crate::hyper_math::{
    Real, hpoint_lerp, hreal_abs, hreal_div, hreal_from_f64, hreal_gt_f64, hreal_max,
    hreal_min, hreal_sub, hreal_sum, hvector3_from_vector3, tau,
};
use crate::mesh::Mesh;
use hyperlattice::{Point3, Vector3};
use std::fmt::Debug;

fn invalid_sdf_value() -> Real {
    hreal_from_f64(1.0e10).expect("finite SDF sentinel")
}

impl<M: Clone + Debug + Send + Sync> Mesh<M> {
    /// **Generic helper** – build a TPMS inside `self` from the provided SDF.
    ///
    /// * `sdf_fn`     – smooth signed‑distance field _f(p)_; 0‑level set is the surface
    /// * `resolution` – voxel grid sampling resolution `(nx, ny, nz)`
    /// * `iso_value`  – iso‑contour value (normally 0.0)
    ///
    /// The result is sampled inside `self`'s bounding box. The TPMS output is an
    /// open implicit surface, so it is not boolean-intersected with `self`;
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
        F: Fn(&Point3) -> Real,
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
        F: Fn(&Point3) -> Real,
    {
        let Some(thickness_h) = hreal_from_f64(thickness).ok() else {
            return Mesh::empty();
        };
        if !hreal_gt_f64(&thickness_h, 0.0) {
            return Mesh::empty();
        }

        let aabb = self.bounding_box();
        let min_pt = aabb.mins;
        let max_pt = aabb.maxs;
        let Some(half) = hreal_from_f64(0.5).ok() else {
            return Mesh::empty();
        };
        let half_thickness = thickness_h.clone() * half;
        let step = max_axis_step(&min_pt, &max_pt, resolution);
        let Some(padding) = hreal_max(&[step, thickness_h.clone()]) else {
            return Mesh::empty();
        };
        let Some(negative_padding) = hreal_sub(Real::zero(), padding.clone()) else {
            return Mesh::empty();
        };
        let Some(sample_min) = hpoint_pad(&min_pt, negative_padding) else {
            return Mesh::empty();
        };
        let Some(sample_max) = hpoint_pad(&max_pt, padding) else {
            return Mesh::empty();
        };

        Mesh::sdf(
            move |p: &Point3| {
                let sheet = hreal_sub(sdf_fn(p), iso_value.clone())
                    .and_then(hreal_abs)
                    .and_then(|distance| hreal_sub(distance, half_thickness.clone()))
                    .unwrap_or_else(invalid_sdf_value);
                let bounds = axis_aligned_box_sdf(p, &min_pt, &max_pt);
                hreal_max(&[sheet, bounds]).unwrap_or_else(invalid_sdf_value)
            },
            resolution,
            sample_min,
            sample_max,
            Real::zero(),
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
            return Mesh::empty();
        };
        self.tpms_from_sdf(
            move |p: &Point3| {
                tpms_gyroid_value(p, scale.clone()).unwrap_or_else(invalid_sdf_value)
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
        let Some(scale) = tpms_scale(period) else {
            return Mesh::empty();
        };
        self.tpms_solid_from_sdf(
            move |p: &Point3| {
                tpms_gyroid_value(p, scale.clone()).unwrap_or_else(invalid_sdf_value)
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
        let Some(scale) = tpms_scale(period) else {
            return Mesh::empty();
        };
        self.tpms_from_sdf(
            move |p: &Point3| {
                tpms_schwarz_p_value(p, scale.clone()).unwrap_or_else(invalid_sdf_value)
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
        let Some(scale) = tpms_scale(period) else {
            return Mesh::empty();
        };
        self.tpms_solid_from_sdf(
            move |p: &Point3| {
                tpms_schwarz_p_value(p, scale.clone()).unwrap_or_else(invalid_sdf_value)
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
        let Some(scale) = tpms_scale(period) else {
            return Mesh::empty();
        };
        self.tpms_from_sdf(
            move |p: &Point3| {
                tpms_schwarz_d_value(p, scale.clone()).unwrap_or_else(invalid_sdf_value)
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
        let Some(scale) = tpms_scale(period) else {
            return Mesh::empty();
        };
        self.tpms_solid_from_sdf(
            move |p: &Point3| {
                tpms_schwarz_d_value(p, scale.clone()).unwrap_or_else(invalid_sdf_value)
            },
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
    if !hreal_gt_f64(&period, 0.0) {
        return None;
    }
    (tau() / period).ok()
}

fn tpms_scaled_axes(point: &Point3, scale: Real) -> Option<(Real, Real, Real)> {
    Some((
        point.x.clone() * scale.clone(),
        point.y.clone() * scale.clone(),
        point.z.clone() * scale,
    ))
}

/// Evaluate Schoen's gyroid approximation in hyperreal space.
fn tpms_gyroid_value(point: &Point3, scale: Real) -> Option<Real> {
    let (x, y, z) = tpms_scaled_axes(point, scale)?;
    let sin_x = x.clone().sin();
    let cos_x = x.cos();
    let sin_y = y.clone().sin();
    let cos_y = y.cos();
    let sin_z = z.clone().sin();
    let cos_z = z.cos();
    Some(sin_x * cos_y + sin_y * cos_z + sin_z * cos_x)
}

/// Evaluate Schwarz's primitive cubic surface approximation in hyperreal space.
fn tpms_schwarz_p_value(point: &Point3, scale: Real) -> Option<Real> {
    let (x, y, z) = tpms_scaled_axes(point, scale)?;
    Some(x.cos() + y.cos() + z.cos())
}

/// Evaluate Schwarz's diamond surface approximation in hyperreal space.
fn tpms_schwarz_d_value(point: &Point3, scale: Real) -> Option<Real> {
    let (x, y, z) = tpms_scaled_axes(point, scale)?;
    let sin_x = x.clone().sin();
    let cos_x = x.cos();
    let sin_y = y.clone().sin();
    let cos_y = y.cos();
    let sin_z = z.clone().sin();
    let cos_z = z.cos();
    Some(
        sin_x.clone() * sin_y.clone() * sin_z.clone()
            + sin_x * cos_y.clone() * cos_z.clone()
            + cos_x.clone() * sin_y * cos_z
            + cos_x * cos_y * sin_z,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hyper_math::tolerance;

    #[test]
    fn tpms_scale_accepts_any_exact_positive_period() {
        assert!(tpms_scale(tolerance() * 0.25).is_some());
        assert!(tpms_scale(tolerance()).is_some());
        assert!(tpms_scale(Real::one()).is_some());
    }

    #[test]
    fn tpms_scale_rejects_zero_and_negative_periods() {
        assert!(tpms_scale(Real::zero()).is_none());
        assert!(tpms_scale(-tolerance()).is_none());
    }
}

fn axis_aligned_box_sdf(p: &Point3, min: &Point3, max: &Point3) -> Real {
    // The outside distance is a vector magnitude, so evaluate it in
    // hyperlattice and export only the finite SDF boundary value. This keeps
    // TPMS solid capping aligned with Yap's exact-geometric-computation
    // boundary split (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    let half_scalar = (Real::one() / Real::from(2_u8)).expect("nonzero exact denominator");
    let center = hpoint_lerp(min, max, half_scalar).unwrap_or_else(|| min.clone());
    let half = Vector3::from_xyz(
        haxis_half_extent(min.x.clone(), max.x.clone()).unwrap_or_else(invalid_sdf_value),
        haxis_half_extent(min.y.clone(), max.y.clone()).unwrap_or_else(invalid_sdf_value),
        haxis_half_extent(min.z.clone(), max.z.clone()).unwrap_or_else(invalid_sdf_value),
    );
    let q = Vector3::from_xyz(
        haxis_abs_offset(p.x.clone(), center.x, half.0[0].clone())
            .unwrap_or_else(invalid_sdf_value),
        haxis_abs_offset(p.y.clone(), center.y, half.0[1].clone())
            .unwrap_or_else(invalid_sdf_value),
        haxis_abs_offset(p.z.clone(), center.z, half.0[2].clone())
            .unwrap_or_else(invalid_sdf_value),
    );
    let outside_vec = Vector3::from_xyz(
        hreal_max(&[q.0[0].clone(), Real::zero()]).unwrap_or_else(invalid_sdf_value),
        hreal_max(&[q.0[1].clone(), Real::zero()]).unwrap_or_else(invalid_sdf_value),
        hreal_max(&[q.0[2].clone(), Real::zero()]).unwrap_or_else(invalid_sdf_value),
    );
    let outside = hvector3_from_vector3(&outside_vec)
        .and_then(|vector| vector.magnitude().ok())
        .unwrap_or_else(invalid_sdf_value);
    let inside = hreal_max(&[q.0[0].clone(), q.0[1].clone(), q.0[2].clone()])
        .and_then(|max_component| hreal_min(&[max_component, Real::zero()]))
        .unwrap_or_else(invalid_sdf_value);
    hreal_sum(&[outside, inside]).unwrap_or_else(invalid_sdf_value)
}

fn hpoint_pad(point: &Point3, padding: Real) -> Option<Point3> {
    Some(Point3::new(
        hreal_sum(&[point.x.clone(), padding.clone()])?,
        hreal_sum(&[point.y.clone(), padding.clone()])?,
        hreal_sum(&[point.z.clone(), padding])?,
    ))
}

fn haxis_half_extent(min: Real, max: Real) -> Option<Real> {
    hreal_sub(max, min)
        .and_then(hreal_abs)
        .and_then(|span| hreal_div(span, 2.0))
}

fn haxis_abs_offset(value: Real, center: Real, half_extent: Real) -> Option<Real> {
    hreal_sub(value, center)
        .and_then(hreal_abs)
        .and_then(|distance| hreal_sub(distance, half_extent))
}

fn max_axis_step(min: &Point3, max: &Point3, resolution: (usize, usize, usize)) -> Real {
    let dx = hreal_sub(max.x.clone(), min.x.clone())
        .and_then(hreal_abs)
        .and_then(|span| hreal_div(span, Real::from((resolution.0.max(2) - 1) as u64)))
        .unwrap_or_else(Real::zero);
    let dy = hreal_sub(max.y.clone(), min.y.clone())
        .and_then(hreal_abs)
        .and_then(|span| hreal_div(span, Real::from((resolution.1.max(2) - 1) as u64)))
        .unwrap_or_else(Real::zero);
    let dz = hreal_sub(max.z.clone(), min.z.clone())
        .and_then(hreal_abs)
        .and_then(|span| hreal_div(span, Real::from((resolution.2.max(2) - 1) as u64)))
        .unwrap_or_else(Real::zero);
    hreal_max(&[dx, dy, dz]).unwrap_or_else(Real::zero)
}
