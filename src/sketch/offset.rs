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
//! stay in [`hypercurve::CurveRegion2`], [`hypercurve::Contour2`], and
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

use crate::csg::CSG;
use crate::errors::{ProfileOffsetError, ProfileStraightSkeletonError};
use crate::hyper_math::{Real, hreal_abs, hreal_try_cmp};
use crate::sketch::Profile;
use hypercurve::{
    BezierFlatteningOptions, Classification, CurvePolicy, CurveRegion2, CurveString2,
    FiniteRegionProfile2, LineSeg2, OffsetCap, Segment2, StraightSkeletonReport2,
};

impl Profile {
    fn preserve_offset_wires(&self, sketch: &mut Profile) {
        if !self.wires().is_empty() {
            sketch.append_native_wires(self.wires().iter().cloned());
        }
        if !self.curve_paths().is_empty() {
            sketch.curve_wires.extend(self.curve_paths().iter().cloned());
            sketch.invalidate_bounding_box();
        }
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
            Profile::from_topology_with_origin(
                self.region.clone(),
                self.wires.clone(),
                self.curve_wires.clone(),
                self.origin.clone(),
                self.origin_transform.clone(),
            )
        })
    }

    /// Build a native filled outline around wire-only sketches.
    ///
    /// Hypercurve owns open-profile offset/cap construction through
    /// [`CurveString2::offset_outline`], so wire-only offsets compose directly
    /// into filled [`CurveRegion2`] topology. This is the native counterpart to the
    /// cap-and-join decomposition of Tiller and Hanson (1984).
    fn native_wire_outline_offset(&self, distance: Real, cap: OffsetCap) -> Option<Profile> {
        if !self.region.is_empty()
            || self.wires.is_empty()
            || !self.curve_wires.is_empty()
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

        let region =
            match CurveRegion2::try_from_native_boundary_contours(contours.clone(), &policy)
                .ok()?
            {
                Classification::Decided(region) => region,
                Classification::Uncertain(_) => {
                    CurveRegion2::try_from_native_material_contours(contours, &policy).ok()?
                },
            };
        Some(Profile::from_topology_with_origin(
            region,
            Vec::new(),
            Vec::new(),
            self.origin.clone(),
            self.origin_transform.clone(),
        ))
    }

    /// Ask the unified curve-region carrier to offset any certified native
    /// line/arc image it owns, and retain the result as `CurveRegion2`.
    fn native_curve_region_offset(
        &self,
        distance: Real,
    ) -> Result<Option<Profile>, ProfileOffsetError> {
        if self.region.is_empty() {
            return Ok(None);
        }
        debug_assert!(self.wires.is_empty() && self.curve_wires.is_empty());

        let offset = self.region.offset(distance, &CurvePolicy::certified())?;
        let region = match offset {
            Classification::Decided(region) => region,
            Classification::Uncertain(hypercurve::UncertaintyReason::Unsupported) => {
                return Err(ProfileOffsetError::HigherOrderCurves);
            },
            Classification::Uncertain(reason) => {
                return Err(ProfileOffsetError::Uncertain(reason));
            },
        };
        Ok(Some(Profile::from_curve_topology_with_origin(
            region,
            Vec::new(),
            self.origin.clone(),
            self.origin_transform.clone(),
        )))
    }

    /// Offset native Profile topology with sharp joins where hypercurve can
    /// certify the operation.
    ///
    /// A zero offset is identity. Wire-only input becomes a filled native
    /// outline. Filled regions use checked Hypercurve contour offsets.
    ///
    /// # Panics
    ///
    /// Panics when the native kernel cannot construct the requested offset.
    /// Use [`Profile::try_offset`] when unsupported topology should be handled
    /// without panicking.
    pub fn offset(&self, distance: Real) -> Profile {
        self.try_offset(distance)
            .unwrap_or_else(|error| panic!("profile offset failed: {error}"))
    }

    /// Try to offset native Profile topology with sharp joins.
    ///
    /// A zero offset preserves every exact curve family without projection.
    /// Nonzero offsets currently accept Hypercurve's certified line/arc region
    /// and `CurveString2` paths; higher-order curves return an explicit error
    /// instead of being discarded.
    pub fn try_offset(&self, distance: Real) -> Result<Profile, ProfileOffsetError> {
        if let Some(sketch) = self.native_zero_offset(distance.clone()) {
            return Ok(sketch);
        }
        if self.is_empty() {
            return Ok(self.clone());
        }
        if !self.curve_wires.is_empty() {
            return Err(ProfileOffsetError::HigherOrderCurves);
        }
        if !self.region.is_empty() && !self.wires.is_empty() {
            return Err(ProfileOffsetError::UnsupportedTopology {
                join_style: "sharp-join",
            });
        }
        if let Some(sketch) = self.native_curve_region_offset(distance.clone())? {
            return Ok(sketch);
        }
        if let Some(sketch) =
            self.native_wire_outline_offset(distance.clone(), OffsetCap::Square)
        {
            return Ok(sketch);
        }
        Err(ProfileOffsetError::UnsupportedTopology {
            join_style: "sharp-join",
        })
    }

    /// Offset filled higher-order topology through certified exact-scalar segmentation.
    ///
    /// Native line/arc regions remain lossless. Other materialized curve
    /// families are subdivided with the caller's explicit chord-error budget,
    /// then offset by Hypercurve's exact line topology engine. No `f64`
    /// coordinate conversion occurs, but the segmented source boundary is an
    /// explicitly lossy approximation.
    pub fn try_offset_with_certified_segmentation(
        &self,
        distance: Real,
        options: &BezierFlatteningOptions,
    ) -> Result<Profile, ProfileOffsetError> {
        if let Some(sketch) = self.native_zero_offset(distance.clone()) {
            return Ok(sketch);
        }
        if self.is_empty() {
            return Ok(self.clone());
        }
        if !self.wires.is_empty() || !self.curve_wires.is_empty() {
            return Err(ProfileOffsetError::UnsupportedTopology {
                join_style: "certified-segmented",
            });
        }
        let offset = self.region.offset_with_certified_segmentation(
            distance,
            options,
            &CurvePolicy::certified(),
        )?;
        let result = match offset {
            Classification::Decided(result) => result,
            Classification::Uncertain(reason) => {
                return Err(ProfileOffsetError::Uncertain(reason));
            },
        };
        Ok(Profile::from_curve_topology_with_origin(
            result.into_region(),
            Vec::new(),
            self.origin.clone(),
            self.origin_transform.clone(),
        ))
    }

    /// Panicking convenience wrapper for [`Self::try_offset_with_certified_segmentation`].
    pub fn offset_with_certified_segmentation(
        &self,
        distance: Real,
        options: &BezierFlatteningOptions,
    ) -> Profile {
        self.try_offset_with_certified_segmentation(distance, options)
            .unwrap_or_else(|error| {
                panic!("certified segmented profile offset failed: {error}")
            })
    }

    /// Offset native Profile topology with rounded caps for wire-only input.
    ///
    /// Filled-region rounded joins are not yet native in hypercurve, so filled
    /// regions use the same checked sharp-contour path as [`Profile::offset`].
    ///
    /// # Panics
    ///
    /// Panics when the native kernel cannot construct the requested offset.
    /// Use [`Profile::try_offset_rounded`] when unsupported topology should be
    /// handled without panicking.
    pub fn offset_rounded(&self, distance: Real) -> Profile {
        self.try_offset_rounded(distance)
            .unwrap_or_else(|error| panic!("rounded profile offset failed: {error}"))
    }

    /// Try to offset native Profile topology with rounded open-wire caps.
    ///
    /// Filled line/arc regions currently follow the checked sharp-contour path.
    /// Higher-order curves return an explicit error for nonzero distances.
    pub fn try_offset_rounded(&self, distance: Real) -> Result<Profile, ProfileOffsetError> {
        if let Some(sketch) = self.native_zero_offset(distance.clone()) {
            return Ok(sketch);
        }
        if self.is_empty() {
            return Ok(self.clone());
        }
        if !self.curve_wires.is_empty() {
            return Err(ProfileOffsetError::HigherOrderCurves);
        }
        if !self.region.is_empty() && !self.wires.is_empty() {
            return Err(ProfileOffsetError::UnsupportedTopology {
                join_style: "rounded-cap",
            });
        }
        if let Some(sketch) = self.native_curve_region_offset(distance.clone())? {
            return Ok(sketch);
        }
        if let Some(sketch) =
            self.native_wire_outline_offset(distance.clone(), OffsetCap::Round)
        {
            return Ok(sketch);
        }
        Err(ProfileOffsetError::UnsupportedTopology {
            join_style: "rounded-cap",
        })
    }

    /// Return native inward diagnostic rays for finite profile views.
    ///
    /// The returned wires are [`CurveString2`] values constructed from the
    /// finite boundary projection of the native region. They are diagnostic
    /// rays from each material vertex toward a profile centroid, not a complete
    /// straight-skeleton algorithm. This accurately named operation remains
    /// available for visualization and debugging only.
    pub fn inward_vertex_rays(&self, orientation: bool) -> Profile {
        let wires = if orientation {
            inward_profile_rays(&self.region_profiles())
        } else {
            Vec::new()
        };

        let mut sketch = Profile::from_topology_with_origin(
            CurveRegion2::empty(),
            wires,
            Vec::new(),
            self.origin.clone(),
            self.origin_transform.clone(),
        );
        self.preserve_offset_wires(&mut sketch);
        sketch
    }

    /// Return Hypercurve's report for the exact interior straight skeleton.
    ///
    /// The current exact kernel accepts one simple, convex, line-only material
    /// contour without holes or open wires. Concave inputs retain Hypercurve's
    /// typed split-event blocker instead of falling back to diagnostic rays.
    pub fn straight_skeleton_report(
        &self,
    ) -> Result<StraightSkeletonReport2, ProfileStraightSkeletonError> {
        if !self.wires().is_empty() || !self.curve_paths().is_empty() {
            return Err(ProfileStraightSkeletonError::UnsupportedTopology {
                requirement: "one filled contour without open wires",
            });
        }
        let policy = CurvePolicy::certified();
        let native = match self.region.native_contours_fast_path(&policy)? {
            Classification::Decided(native) => native,
            Classification::Uncertain(reason) => {
                return Err(ProfileStraightSkeletonError::Uncertain(reason));
            },
        };
        if native.material_contours().len() != 1 || !native.hole_contours().is_empty() {
            return Err(ProfileStraightSkeletonError::UnsupportedTopology {
                requirement: "exactly one material contour and no holes",
            });
        }
        native.material_contours()[0]
            .straight_skeleton(&policy)
            .map_err(ProfileStraightSkeletonError::from)
    }

    /// Construct exact straight-skeleton arcs as native Hypercurve wires.
    pub fn try_straight_skeleton(&self) -> Result<Profile, ProfileStraightSkeletonError> {
        let report = self.straight_skeleton_report()?;
        if let Some(blocker) = report.blocker().cloned() {
            return Err(ProfileStraightSkeletonError::Blocked(blocker));
        }
        let skeleton = report
            .skeleton()
            .expect("a blocker-free complete straight-skeleton report has a graph");
        let mut wires = Vec::with_capacity(skeleton.arcs().len());
        for arc in skeleton.arcs() {
            let start = skeleton.nodes()[arc.start_node()].point().clone();
            let end = skeleton.nodes()[arc.end_node()].point().clone();
            let line = LineSeg2::try_new(start, end)?;
            wires.push(CurveString2::try_new(vec![Segment2::Line(line)])?);
        }
        Ok(Profile::from_topology_with_origin(
            CurveRegion2::empty(),
            wires,
            Vec::new(),
            self.origin.clone(),
            self.origin_transform.clone(),
        ))
    }

    /// Construct the exact interior straight skeleton for supported profiles.
    ///
    /// `orientation` is retained for source compatibility. `false` returns an
    /// empty profile; `true` executes the genuine Hypercurve wavefront kernel.
    ///
    /// # Panics
    ///
    /// Panics when the source needs unsupported split events or topology.
    pub fn straight_skeleton(&self, orientation: bool) -> Profile {
        if !orientation {
            return Profile::empty();
        }
        self.try_straight_skeleton()
            .unwrap_or_else(|error| panic!("profile straight skeleton failed: {error}"))
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
