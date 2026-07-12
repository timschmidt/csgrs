//! Native hypercurve offset and skeleton-facing Profile operations.
//!
//! Offset construction is a classical CAD operation: a polygonal profile is
//! displaced by a prescribed normal distance, then intersections, collapses,
//! and component changes must be regularized. The primitive offset/cap
//! decomposition used here follows Tiller and Hanson, "Offsets of
//! Two-Dimensional Profiles," *IEEE Computer Graphics and Applications* 4(9),
//! 1984 (<https://doi.org/10.1109/MCG.1984.276011>). Analytic offset caveats
//! for curves follow Farouki and Neff, "Analytic properties of plane offset
//! curves," *Computer Aided Geometric Design* 7(1-4), 1990
//! (<https://doi.org/10.1016/0167-8396(90)90023-K>).
//!
//! `Profile` does not own a second finite polygon datatype. Supported offsets
//! stay in [`hypercurve::Region2`], [`hypercurve::Contour2`], and
//! [`hypercurve::CurveString2`]. Cases that require global trimming or
//! component merging are kept visible at the native hypercurve boundary instead
//! of being routed through a finite compatibility cache. That exact-object
//! boundary follows Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7(1-2), 1997
//! (<https://doi.org/10.1016/0925-7721(95)00040-2>).
//!
//! Straight skeletons are a separate topological construction; see Aichholzer
//! et al., "A Novel Type of Skeleton for Polygons," *Journal of Universal
//! Computer Science* 1(12), 1995
//! (<https://doi.org/10.3217/jucs-001-12-0752>). Until that algorithm is owned
//! by hypercurve, this module only emits native inward diagnostic rays for
//! simple finite profile projections rather than importing a separate skeleton
//! geometry model.

use crate::hyper_math::{Real, hreal_abs, hreal_sign, hreal_try_cmp};
use crate::sketch::Profile;
use hypercurve::{
    Classification, Contour2, CurvePolicy, CurveString2, FiniteRegionProfile2, OffsetCap,
    Region2,
};
use hyperreal::RealSign;

#[cfg(feature = "offset")]
use geo::{Buffer as _, Coord, LineString, MultiPolygon, Polygon as GeoPolygon};

impl Profile {
    fn preserve_offset_wires(&self, sketch: &mut Profile) {
        if !self.wires().is_empty() {
            sketch.append_native_wires(self.wires().iter().cloned());
        }
    }

    fn empty_offset_result(&self) -> Profile {
        Profile::from_region_and_wires_with_origin(
            Region2::empty(),
            Vec::new(),
            self.origin.clone(),
            self.origin_transform.clone(),
        )
    }

    /// Return native hypercurve topology unchanged for an exact zero offset.
    ///
    /// The identity case remains entirely in the hyperreal kernel and does not
    /// consult or regenerate a finite compatibility representation. This keeps
    /// boundary ownership aligned with Yap's exact geometric computation model
    /// (1997) while preserving the zero-radius offset identity discussed in
    /// Farouki and Neff (1990).
    fn native_zero_offset(&self, distance: Real) -> Option<Profile> {
        matches!(hreal_try_cmp(&distance, 0.0), Some(std::cmp::Ordering::Equal)).then(|| {
            Profile::from_region_and_wires_with_origin(
                self.region.clone(),
                self.wires.clone(),
                self.origin.clone(),
                self.origin_transform.clone(),
            )
        })
    }

    /// Build a native filled outline around wire-only sketches.
    ///
    /// Hypercurve owns open-profile offset/cap construction through
    /// [`CurveString2::offset_outline`], so wire-only offsets compose directly
    /// into filled [`Region2`] topology. This is the native counterpart to the
    /// cap-and-join decomposition of Tiller and Hanson (1984).
    fn native_wire_outline_offset(&self, distance: Real, cap: OffsetCap) -> Option<Profile> {
        if !self.region.is_empty()
            || self.wires.is_empty()
            || !matches!(
                hreal_try_cmp(&distance, 0.0),
                Some(std::cmp::Ordering::Greater)
            )
        {
            return None;
        }

        let policy = CurvePolicy::certified();
        let half_width = hreal_abs(distance)?;
        let mut contours = Vec::with_capacity(self.wires.len());
        for wire in &self.wires {
            let contour = match wire.offset_outline(half_width.clone(), cap, &policy).ok()? {
                Classification::Decided(contour) => contour,
                Classification::Uncertain(_) => return None,
            };
            contours.push(contour);
        }

        let region = match Region2::from_boundary_contours(contours.clone(), &policy).ok()? {
            Classification::Decided(region) => region,
            Classification::Uncertain(_) => Region2::from_material_contours(contours),
        };
        Some(Profile::from_region_and_wires_with_origin(
            region,
            Vec::new(),
            self.origin.clone(),
            self.origin_transform.clone(),
        ))
    }

    /// Try a native hypercurve sharp offset.
    ///
    /// This covers the conservative contour-wise case where every material and
    /// hole boundary can be offset independently and checked for self-contact by
    /// hypercurve. Material contours move outward relative to the filled region;
    /// hole contours move into the void for positive distances and away from it
    /// for negative distances. General regularization requiring trimming,
    /// collapsed holes, splits, or merged components belongs in hypercurve
    /// rather than in a finite bridge.
    fn native_sharp_offset(&self, distance: Real) -> Option<Profile> {
        if self.region.is_empty()
            || !self.wires.is_empty()
            || !Self::region_has_nonzero_area(&self.region)
        {
            return None;
        }

        let policy = CurvePolicy::certified();
        let mut material = Vec::with_capacity(self.region.material_contours().len());
        for contour in self.region.material_contours() {
            material.push(native_role_offset_contour(
                contour,
                distance.clone(),
                false,
                &policy,
            )?);
        }

        let mut holes = Vec::with_capacity(self.region.hole_contours().len());
        for contour in self.region.hole_contours() {
            holes.push(native_role_offset_contour(
                contour,
                distance.clone(),
                true,
                &policy,
            )?);
        }

        let mut sketch = Profile::from_region(Region2::new(material, holes));
        sketch.origin = self.origin.clone();
        sketch.origin_transform = self.origin_transform.clone();
        Some(sketch)
    }

    /// Offset native Profile topology with sharp joins where hypercurve can
    /// certify the operation.
    ///
    /// A zero offset is identity. Wire-only input becomes a filled native
    /// outline. Filled regions use contour-wise raw offsets when hypercurve can
    /// prove the result does not need global regularization. Unsupported cases
    /// return an empty native sketch instead of manufacturing CAD state through
    /// another geometry crate.
    pub fn offset(&self, distance: Real) -> Profile {
        if let Some(sketch) = self.native_zero_offset(distance.clone()) {
            return sketch;
        }
        if let Some(sketch) =
            self.native_wire_outline_offset(distance.clone(), OffsetCap::Square)
        {
            return sketch;
        }
        if let Some(sketch) = self.native_sharp_offset(distance) {
            return sketch;
        }
        self.empty_offset_result()
    }

    /// Offset native Profile topology with rounded caps for wire-only input.
    ///
    /// Filled-region rounded regularization is not delegated to a finite
    /// compatibility datatype. Until rounded regularized region offsets are
    /// native in hypercurve, filled regions use the same certified raw-contour
    /// path as [`Profile::offset`].
    pub fn offset_rounded(&self, distance: Real) -> Profile {
        if let Some(sketch) = self.native_zero_offset(distance.clone()) {
            return sketch;
        }
        if let Some(sketch) =
            self.native_wire_outline_offset(distance.clone(), OffsetCap::Round)
        {
            return sketch;
        }
        if let Some(sketch) = self.native_sharp_offset(distance) {
            return sketch;
        }
        self.empty_offset_result()
    }

    /// Build a rounded offset at a finite application/output boundary.
    ///
    /// Native hypercurve offsetting remains the primary CAD path. This method
    /// exists for callers such as OpenSCAD import that already define a finite
    /// tessellation boundary and require global buffer trimming across complex
    /// multi-contour profiles. Buffered rings are promoted immediately back to
    /// native [`Region2`] topology.
    #[cfg(feature = "offset")]
    pub fn offset_rounded_finite_output(&self, distance: Real) -> Profile {
        let Some(distance) = distance.to_f64_lossy() else {
            return self.empty_offset_result();
        };
        let polygons = self
            .region_profiles()
            .into_iter()
            .filter_map(|profile| {
                let exterior = finite_ring_to_geo(profile.material().points())?;
                let holes = profile
                    .holes()
                    .iter()
                    .filter_map(|hole| finite_ring_to_geo(hole.points()))
                    .collect::<Vec<_>>();
                Some(GeoPolygon::new(exterior, holes))
            })
            .collect::<Vec<_>>();
        if polygons.is_empty() {
            return self.empty_offset_result();
        }

        let buffered = MultiPolygon(polygons).buffer(distance);
        let mut material = Vec::with_capacity(buffered.0.len());
        let mut holes = Vec::new();
        for polygon in &buffered.0 {
            let Some(exterior) = geo_ring_to_contour(polygon.exterior()) else {
                return self.empty_offset_result();
            };
            material.push(exterior);
            holes.extend(polygon.interiors().iter().filter_map(geo_ring_to_contour));
        }
        let region = Region2::new(material, holes);
        Profile::from_region_and_wires_with_origin(
            region,
            Vec::new(),
            self.origin.clone(),
            self.origin_transform.clone(),
        )
    }

    /// Return native inward skeleton-facing linework for finite profile views.
    ///
    /// The returned wires are [`CurveString2`] values constructed from the
    /// finite boundary projection of the native region. They are diagnostic
    /// rays from each material vertex toward a profile centroid, not a complete
    /// straight-skeleton algorithm. This keeps the API hyper-only while leaving
    /// the true Aichholzer et al. (1995) wavefront algorithm to be completed in
    /// hypercurve.
    pub fn straight_skeleton(&self, orientation: bool) -> Profile {
        let wires = if orientation {
            inward_profile_rays(&self.region_profiles())
        } else {
            Vec::new()
        };

        let mut sketch = Profile::from_region_and_wires_with_origin(
            Region2::empty(),
            wires,
            self.origin.clone(),
            self.origin_transform.clone(),
        );
        self.preserve_offset_wires(&mut sketch);
        sketch
    }
}

#[cfg(feature = "offset")]
fn finite_ring_to_geo(points: &[[f64; 2]]) -> Option<LineString<f64>> {
    (points.len() >= 4).then(|| {
        LineString::new(
            points
                .iter()
                .map(|point| Coord {
                    x: point[0],
                    y: point[1],
                })
                .collect(),
        )
    })
}

#[cfg(feature = "offset")]
fn geo_ring_to_contour(ring: &LineString<f64>) -> Option<Contour2> {
    let points = ring
        .points()
        .map(|point| [point.x(), point.y()])
        .collect::<Vec<_>>();
    Contour2::from_finite_ring(&points).ok()
}

fn native_role_offset_contour(
    contour: &Contour2,
    distance: Real,
    is_hole: bool,
    policy: &CurvePolicy,
) -> Option<Contour2> {
    let signed_area = contour.signed_area().ok()??;
    let sign = hreal_sign(&signed_area)?;
    if sign == RealSign::Zero {
        return None;
    }

    let signed_distance = match (is_hole, sign) {
        (false, RealSign::Positive) | (true, RealSign::Negative) => -distance,
        (false, RealSign::Negative) | (true, RealSign::Positive) => distance,
        (_, RealSign::Zero) => return None,
    };
    match contour.offset_left_checked(signed_distance, policy).ok()? {
        Classification::Decided(offset) => Some(offset),
        Classification::Uncertain(_) => None,
    }
}

fn inward_profile_rays(profiles: &[FiniteRegionProfile2]) -> Vec<CurveString2> {
    profiles
        .iter()
        .flat_map(|profile| {
            let Some(center) = profile.material().vertex_centroid() else {
                return Vec::new();
            };
            profile
                .material()
                .points()
                .iter()
                .cloned()
                .filter(|point| point != &center)
                .filter_map(|point| {
                    CurveString2::from_finite_line_string(&[point, center]).ok()
                })
                .collect::<Vec<_>>()
        })
        .collect()
}
