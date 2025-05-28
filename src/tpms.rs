//! Triply‑Periodic Minimal Surfaces rewritten to leverage the generic
//! signed‑distance mesher in `sdf.rs`.

use crate::csg::CSG;
use crate::float_types::Real;
use nalgebra::Point3;
use std::fmt::Debug;

impl<S: Clone + Debug + Send + Sync> CSG<S> {
    /// **Generic helper** – build a TPMS inside `self` from the provided SDF.
    ///
    /// * `sdf_fn`     – smooth signed‑distance field _f(p)_; 0‑level set is the surface
    /// * `resolution` – voxel grid sampling resolution `(nx, ny, nz)`
    /// * `iso_value`  – iso‑contour value (normally 0.0)
    ///
    /// The result is intersected against `self`, so the surface only appears inside
    /// the original solid’s bounding box / volume.
    #[inline]
    fn tpms_from_sdf<F>(
        &self,
        sdf_fn: F,
        resolution: (usize, usize, usize),
        iso_value: Real,
        metadata: Option<S>,
    ) -> CSG<S>
    where
        F: Fn(&Point3<Real>) -> Real + Send + Sync,
    {
        let aabb = self.bounding_box();
        let min_pt = aabb.mins;
        let max_pt = aabb.maxs;
        // Mesh the implicit surface with the generic surface‑nets backend
        let surf = CSG::sdf(sdf_fn, resolution, min_pt, max_pt, iso_value, metadata);
        // Clip the infinite TPMS down to the original shape’s volume
        surf.intersection(self)
    }

    // ------------  Specific minimal‑surface flavours  --------------------

    /// Gyroid surface:  `sin x cos y + sin y cos z + sin z cos x = iso`  
    /// (`period` rescales the spatial frequency; larger => slower repeat)
    pub fn gyroid(
        &self,
        resolution: usize,
        period: Real,
        iso_value: Real,
        metadata: Option<S>,
    ) -> CSG<S> {
        let res = (resolution.max(2), resolution.max(2), resolution.max(2));
        let period_inv = 1.0 / period;
        self.tpms_from_sdf(
            move |p: &Point3<Real>| {
                let x = p.x * period_inv;
                let y = p.y * period_inv;
                let z = p.z * period_inv;
                (x.sin() * y.cos()) + (y.sin() * z.cos()) + (z.sin() * x.cos())
            },
            res,
            iso_value,
            metadata,
        )
    }

    /// Schwarz‑P surface:  `cos x + cos y + cos z = iso`  (default iso = 0)
    pub fn schwarz_p(
        &self,
        resolution: usize,
        period: Real,
        iso_value: Real,
        metadata: Option<S>,
    ) -> CSG<S> {
        let res = (resolution.max(2), resolution.max(2), resolution.max(2));
        let period_inv = 1.0 / period;
        self.tpms_from_sdf(
            move |p: &Point3<Real>| {
                let x = p.x * period_inv;
                let y = p.y * period_inv;
                let z = p.z * period_inv;
                x.cos() + y.cos() + z.cos()
            },
            res,
            iso_value,
            metadata,
        )
    }

    /// Schwarz‑D (Diamond) surface:  `sin x sin y sin z + sin x cos y cos z + ... = iso`
    pub fn schwarz_d(
        &self,
        resolution: usize,
        period: Real,
        iso_value: Real,
        metadata: Option<S>,
    ) -> CSG<S> {
        let res = (resolution.max(2), resolution.max(2), resolution.max(2));
        let period_inv = 1.0 / period;
        self.tpms_from_sdf(
            move |p: &Point3<Real>| {
                let x = p.x * period_inv;
                let y = p.y * period_inv;
                let z = p.z * period_inv;
                (x.sin() * y.sin() * z.sin())
                    + (x.sin() * y.cos() * z.cos())
                    + (x.cos() * y.sin() * z.cos())
                    + (x.cos() * y.cos() * z.sin())
            },
            res,
            iso_value,
            metadata,
        )
    }
}

