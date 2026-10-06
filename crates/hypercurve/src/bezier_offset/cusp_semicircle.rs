//! Selected algebraic cusp-semicircle carrier: construction, incidence, contacts, parameter maps and evaluation.

use super::*;

mod chord_kernel;
mod fragment;
mod frames;
mod pair_kernel;
mod parallel_kernel;
mod parameters;
mod points;
mod rational_kernel;

impl BezierAlgebraicCuspSemicircle2 {
    #[inline]
    pub(super) fn turn_sign(&self) -> Real {
        Real::from(if self.data.clockwise { -1_i8 } else { 1_i8 })
    }

    /// Returns the retained cusp parameter shared by all curve coefficients.
    pub(crate) fn cusp_parameter(&self) -> &BezierAlgebraicParameter2 {
        &self
            .data
            .frame
            .rational()
            .expect("a rational selected-circle operation requires its one-field frame")
            .data
            .parameter
    }

    pub(super) fn selected_frame_parameter(&self) -> Option<CurveParameter2> {
        match &self.data.frame {
            BezierSelectedCircleFrame2::Rational(frame) => {
                Some(BezierParameter2::Algebraic(frame.data.parameter.clone()).into())
            }
            BezierSelectedCircleFrame2::ParallelNormal(frame) => {
                Some(frame.center_parameter.clone())
            }
            BezierSelectedCircleFrame2::ChordNormal(_)
            | BezierSelectedCircleFrame2::SelectedRadial(_) => None,
        }
    }

    pub(crate) fn uses_selected_parallel_normal_frame(&self) -> bool {
        self.data.frame.parallel_normal().is_some()
    }

    pub(crate) fn uses_selected_chord_normal_frame(&self) -> bool {
        self.data.frame.chord_normal().is_some()
    }

    pub(crate) fn uses_selected_radial_frame(&self) -> bool {
        self.data.frame.selected_radial().is_some()
    }

    /// Coordinate-backed centers keep the direct system. Retained centers
    /// already own correlated fields; projecting their coordinates separately
    /// before incidence would discard that correlation and duplicate work.
    pub(super) fn uses_retained_circle_parallel_system(&self) -> bool {
        self.uses_selected_radial_frame()
            || self
                .data
                .frame
                .chord_normal()
                .is_some_and(|frame| frame.center.coordinates().is_none())
    }

    /// Reports whether this selected-radial center already owns the dense or
    /// recursive chord map required to append another quadratic line-contact
    /// level.  The first pair-radial generation deliberately keeps its
    /// procedural chord ancestry so that ancestry can establish the dense
    /// base; every later generation can take the smaller direct line path.
    pub(super) fn has_recursive_selected_radial_line_parent(&self) -> bool {
        let Some(frame) = self.data.frame.selected_radial() else {
            return false;
        };
        let Some((map, _, _)) = frame.center_parameter.coincident_chord_source() else {
            return false;
        };
        map.recursive_quadratic_line_system().is_some()
            || map.chord_normal_projective_system().is_some()
    }

    /// Builds a circle in the exact orthonormal frame of a regular source
    /// point. The center's parallel locus may have a cusp at that parameter.
    ///
    /// The center is `center_support(center_parameter)`.  Its source unit left
    /// normal supplies the parameter-zero radial direction; the signed radius
    /// therefore places both diameter endpoints without adjoining the source
    /// speed square root to the selected parameter field.  The construction
    /// policy actually consumed by construction is retained. An exact build
    /// requested through APPROXIMATE_512 therefore remains STRICT-replayable,
    /// while a consumed approximate terminal decision can never be replayed
    /// later as STRICT evidence.
    pub(crate) fn from_selected_parallel_normal(
        center_support: BezierParallel2,
        center_parameter: CurveParameter2,
        radial_distance: Real,
        clockwise: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Self>>> {
        match real_sign(&radial_distance, policy) {
            Some(RealSign::Zero) => return Ok(Classification::Decided(None)),
            Some(RealSign::Positive | RealSign::Negative) => {}
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        if let Classification::Uncertain(reason) =
            center_support.certify_source_frame_at(&center_parameter, policy)?
        {
            return Ok(Classification::Uncertain(reason));
        }
        // The radial frame uses the source unit normal, not the derivative
        // of the center locus. Its finiteness and positive speed are already
        // certified above, including when the parallel derivative vanishes.
        Ok(Classification::Decided(Some(Self {
            data: Arc::new(BezierAlgebraicCuspSemicircleData2 {
                parallel_system_cache: Mutex::default(),
                frame: BezierSelectedCircleFrame2::ParallelNormal(Arc::new(
                    BezierSelectedParallelNormalFrameData2 {
                        center_support,
                        center_parameter,
                        policy: policy.retained_object_policy(),
                    },
                )),
                radial_distance,
                clockwise,
            }),
        })))
    }

    /// Builds a selected circle around arbitrary retained center evidence,
    /// using one algebraic chord's exact unit left normal as its local radial
    /// frame.  The center and the chord's two endpoint fields remain separate;
    /// represented chart points are published as lazy normal/tangent
    /// displacements instead of a flattened algebraic coordinate tower.
    pub(crate) fn from_retained_center_and_chord_normal(
        center: CurvePoint2,
        anchor: BezierAlgebraicChord2,
        radial_distance: Real,
        clockwise: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Self>>> {
        anchor.validate_policy(policy)?;
        match real_sign(&radial_distance, policy) {
            Some(RealSign::Zero) => return Ok(Classification::Decided(None)),
            Some(RealSign::Positive | RealSign::Negative) => {}
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        Ok(Classification::Decided(Some(Self {
            data: Arc::new(BezierAlgebraicCuspSemicircleData2 {
                parallel_system_cache: Mutex::default(),
                frame: BezierSelectedCircleFrame2::ChordNormal(Arc::new(
                    BezierSelectedChordNormalFrameData2 {
                        anchor,
                        center,
                        policy: policy.retained_object_policy(),
                    },
                )),
                radial_distance,
                clockwise,
            }),
        })))
    }

    /// Builds a circle centered at one mapped point on an existing selected
    /// circle, using that source circle's contact radial as the local unit
    /// frame.  The mapped parameter retains the complete circle-pair branch;
    /// no Cartesian coordinate or primitive element is constructed.
    pub(crate) fn from_selected_circle_radial(
        support: &Self,
        center_parameter: BezierAlgebraicCuspSemicircleParameter2,
        normal_denominator: Real,
        radial_distance: Real,
        clockwise: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Self>>> {
        match real_sign(&normal_denominator, &CurveContext::STRICT) {
            Some(RealSign::Positive | RealSign::Negative) => {}
            Some(RealSign::Zero) => {
                return Err(CurveError::Topology(
                    "selected radial circle retained a zero normal denominator".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        match real_sign(&radial_distance, policy) {
            Some(RealSign::Zero) => return Ok(Classification::Decided(None)),
            Some(RealSign::Positive | RealSign::Negative) => {}
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        center_parameter.validate_policy(policy)?;
        let BezierAlgebraicCuspSemicircleParameter2::Mapped(center_parameter) = center_parameter
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        if center_parameter.semicircle_carrier() != support {
            return Err(CurveError::Topology(
                "selected radial center parameter referenced a different support circle".into(),
            ));
        }
        Ok(Classification::Decided(Some(Self {
            data: Arc::new(BezierAlgebraicCuspSemicircleData2 {
                parallel_system_cache: Mutex::default(),
                frame: BezierSelectedCircleFrame2::SelectedRadial(Arc::new(
                    BezierSelectedRadialFrameData2 {
                        center_parameter,
                        normal_denominator,
                        similarity_source: None,
                        policy: policy.retained_object_policy(),
                    },
                )),
                radial_distance,
                clockwise,
            }),
        })))
    }

    /// Builds the shared selected-algebraic circle carrier around a retained
    /// center with one certified cardinal start radius.
    ///
    /// The carrier was originally introduced for analytic-parallel cusps, but
    /// its exact incidence, splitting, winding, and pair equations depend only
    /// on a rational center and orthonormal frame in one selected field. An
    /// axis-aligned round join therefore reuses that authority by supplying a
    /// fixed cardinal normal. `radial_distance` remains signed so the same
    /// rational half-circle parameterization covers expansion and erosion.
    #[cfg(test)]
    pub(crate) fn from_retained_axis_aligned_center(
        center: &CurvePoint2,
        cardinal_normal: (i8, i8),
        radial_distance: Real,
        clockwise: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Self>>> {
        if !matches!(cardinal_normal, (1 | -1, 0) | (0, 1 | -1)) {
            return Err(CurveError::Topology(
                "selected algebraic circle frame did not receive a cardinal unit normal".into(),
            ));
        }
        match real_sign(&radial_distance, policy) {
            Some(RealSign::Zero) => return Ok(Classification::Decided(None)),
            Some(RealSign::Positive | RealSign::Negative) => {}
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let CurvePoint2(CurvePointData2::Algebraic(center)) = center else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let parameter = match algebraic_chord_image_parameter(center, policy)? {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let [source_x_numerator, source_y_numerator, denominator] =
            match algebraic_chord_owned_coordinate_polynomials(center, policy)? {
                Classification::Decided(coordinates) => coordinates,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let normal_x_numerator = polynomial_scale(&denominator, &Real::from(cardinal_normal.0));
        let normal_y_numerator = polynomial_scale(&denominator, &Real::from(cardinal_normal.1));
        Ok(Classification::Decided(Some(Self {
            data: Arc::new(BezierAlgebraicCuspSemicircleData2 {
                parallel_system_cache: Mutex::default(),
                frame: BezierSelectedCircleFrame2::Rational(BezierParallelAlgebraicCuspFrame2 {
                    data: Arc::new(BezierParallelAlgebraicCuspFrameData2 {
                        parallel: None,
                        cardinal_normal: Some(cardinal_normal),
                        represented_unit_normal: None,
                        direct_center: Some(center.clone()),
                        parameter,
                        source_x_numerator,
                        source_y_numerator,
                        normal_x_numerator,
                        normal_y_numerator,
                        denominator,
                    }),
                }),
                radial_distance,
                clockwise,
            }),
        })))
    }

    /// Builds a selected-algebraic circle around a retained one-field center
    /// with a represented exact unit radial direction.
    ///
    /// Unlike the cardinal offset fast path, this constructor admits an exact
    /// oblique frame. The unit certificate is proved under STRICT before it is
    /// retained, so APPROXIMATE_512 can never turn an approximate construction
    /// direction into persistent circle evidence.
    pub(crate) fn from_retained_center_and_certified_unit_normal(
        center: &CurvePoint2,
        unit_normal: (Real, Real),
        radial_distance: Real,
        clockwise: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Self>>> {
        // The crate-private caller obtains this vector either by dividing a
        // certified nonzero line direction by its exact square-root norm or
        // from a retained chord's unit-tangent certificate. Re-expanding the
        // unit equation here can turn that construction identity into an
        // undecidable rearranged radical equality under STRICT.
        Self::from_retained_center_frame(center, unit_normal, radial_distance, clockwise, policy)
    }

    /// Builds a selected-algebraic circle from a center lying on an exact
    /// concentric image of a retained circular arc.
    ///
    /// The caller certifies `N = (center - support_center) /
    /// normal_denominator` as the authored arc's unit left normal. Keeping the
    /// center and normal in the same selected field avoids adjoining a second
    /// square root or replaying the circle equation. If the center has
    /// homogeneous coordinates `(X, Y, W)`, the shared frame is represented
    /// exactly over denominator `W * normal_denominator`:
    ///
    /// - center: `(X * normal_denominator, Y * normal_denominator)`;
    /// - normal: `(X - center_x * W, Y - center_y * W)`.
    ///
    /// Other retained point forms reuse the general chord-normal frame with
    /// an exact similarity of the certified radial as its tangent direction.
    pub(crate) fn from_retained_center_and_certified_concentric_normal(
        center: &CurvePoint2,
        support_center: &Point2,
        normal_denominator: Real,
        radial_distance: Real,
        clockwise: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Self>>> {
        match real_sign(&normal_denominator, &CurveContext::STRICT) {
            Some(RealSign::Positive | RealSign::Negative) => {}
            Some(RealSign::Zero) => {
                return Err(CurveError::Topology(
                    "selected concentric circle frame retained a zero normal denominator".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        match real_sign(&radial_distance, policy) {
            Some(RealSign::Zero) => return Ok(Classification::Decided(None)),
            Some(RealSign::Positive | RealSign::Negative) => {}
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        if let CurvePoint2(CurvePointData2::AlgebraicCuspChord(point)) = center {
            let (map, contact) = point.map_contact();
            map.validate_policy(policy)?;
            let support = &map.data.semicircle;
            if support.exact_center(&CurveContext::STRICT)?.as_ref() == Some(support_center)
                && compare_reals(
                    &(support.radial_distance() * support.radial_distance()),
                    &(&normal_denominator * &normal_denominator),
                    &CurveContext::STRICT,
                ) == Some(std::cmp::Ordering::Equal)
            {
                let (center_parameter, _) = map.contact_evidence(contact);
                return Self::from_selected_circle_radial(
                    support,
                    center_parameter,
                    normal_denominator,
                    radial_distance,
                    clockwise,
                    policy,
                );
            }
        }
        let CurvePoint2(CurvePointData2::Algebraic(center)) = center else {
            // The certified radial has length |normal_denominator|. Its
            // clockwise rotation divided by that signed denominator has
            // unit left normal (center-support_center)/normal_denominator.
            // Reuse the general chord frame for independently retained point
            // fields instead of requiring a single-field coordinate image.
            let inverse = (Real::one() / &normal_denominator)?;
            let transform = Similarity2::try_from_real_affine(
                Real::zero(),
                inverse.clone(),
                -&inverse,
                Real::zero(),
                -support_center.y() * &inverse,
                support_center.x() * inverse,
            )?;
            let tangent = CurvePoint2::from(BezierSimilarityPoint2::new(
                center.clone(),
                transform,
                policy,
            ));
            let anchor = match BezierAlgebraicChord2::try_new_from_certified_distinct_endpoints(
                Point2::from_values(0, 0).into(),
                tangent,
                policy,
            )? {
                Classification::Decided(anchor) => anchor,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            return Self::from_retained_center_and_chord_normal(
                center.clone(),
                anchor,
                radial_distance,
                clockwise,
                policy,
            );
        };
        let parameter = match algebraic_chord_image_parameter(center, policy)? {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let [center_x_numerator, center_y_numerator, center_denominator] =
            match algebraic_chord_owned_coordinate_polynomials(center, policy)? {
                Classification::Decided(coordinates) => coordinates,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let source_x_numerator = polynomial_scale(&center_x_numerator, &normal_denominator);
        let source_y_numerator = polynomial_scale(&center_y_numerator, &normal_denominator);
        let normal_x_numerator = polynomial_subtract(
            &center_x_numerator,
            &polynomial_scale(&center_denominator, support_center.x()),
        );
        let normal_y_numerator = polynomial_subtract(
            &center_y_numerator,
            &polynomial_scale(&center_denominator, support_center.y()),
        );
        let denominator = polynomial_scale(&center_denominator, &normal_denominator);
        Ok(Classification::Decided(Some(Self {
            data: Arc::new(BezierAlgebraicCuspSemicircleData2 {
                parallel_system_cache: Mutex::default(),
                frame: BezierSelectedCircleFrame2::Rational(BezierParallelAlgebraicCuspFrame2 {
                    data: Arc::new(BezierParallelAlgebraicCuspFrameData2 {
                        parallel: None,
                        cardinal_normal: None,
                        represented_unit_normal: None,
                        direct_center: Some(center.clone()),
                        parameter,
                        source_x_numerator,
                        source_y_numerator,
                        normal_x_numerator,
                        normal_y_numerator,
                        denominator,
                    }),
                }),
                radial_distance,
                clockwise,
            }),
        })))
    }

    pub(super) fn from_retained_center_frame(
        center: &CurvePoint2,
        unit_normal: (Real, Real),
        radial_distance: Real,
        clockwise: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Self>>> {
        match real_sign(&radial_distance, policy) {
            Some(RealSign::Zero) => return Ok(Classification::Decided(None)),
            Some(RealSign::Positive | RealSign::Negative) => {}
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let CurvePoint2(CurvePointData2::Algebraic(center)) = center else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let parameter = match algebraic_chord_image_parameter(center, policy)? {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let [source_x_numerator, source_y_numerator, denominator] =
            match algebraic_chord_owned_coordinate_polynomials(center, policy)? {
                Classification::Decided(coordinates) => coordinates,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let normal_x_numerator = polynomial_scale(&denominator, &unit_normal.0);
        let normal_y_numerator = polynomial_scale(&denominator, &unit_normal.1);
        let represented_unit_normal = Arc::new(unit_normal);
        Ok(Classification::Decided(Some(Self {
            data: Arc::new(BezierAlgebraicCuspSemicircleData2 {
                parallel_system_cache: Mutex::default(),
                frame: BezierSelectedCircleFrame2::Rational(BezierParallelAlgebraicCuspFrame2 {
                    data: Arc::new(BezierParallelAlgebraicCuspFrameData2 {
                        parallel: None,
                        cardinal_normal: None,
                        represented_unit_normal: Some(represented_unit_normal),
                        direct_center: Some(center.clone()),
                        parameter,
                        source_x_numerator,
                        source_y_numerator,
                        normal_x_numerator,
                        normal_y_numerator,
                        denominator,
                    }),
                }),
                radial_distance,
                clockwise,
            }),
        })))
    }

    #[inline]
    pub(super) fn center_parallel_distance(&self) -> Real {
        self.data.frame.center_parallel_distance()
    }

    pub(crate) fn start_parallel(&self) -> Option<BezierParallel2> {
        let parallel = self.data.frame.source_parallel()?;
        Some(parallel.with_distance(parallel.distance() + &self.data.radial_distance))
    }

    pub(crate) fn end_parallel(&self) -> Option<BezierParallel2> {
        let parallel = self.data.frame.source_parallel()?;
        Some(parallel.with_distance(parallel.distance() - &self.data.radial_distance))
    }

    pub(super) fn source_parallel(&self) -> Option<&BezierParallel2> {
        self.data.frame.source_parallel()
    }

    /// Returns the signed radius along the cusp frame's unit left normal.
    pub(crate) fn radial_distance(&self) -> &Real {
        &self.data.radial_distance
    }

    /// Reports whether this selected circle can enter rational-image-only
    /// accelerators without discarding a retained center frame.
    pub(crate) fn has_rational_frame(&self) -> bool {
        self.data.frame.rational().is_some()
    }

    /// Classifies two parameter charts retained on the same selected-circle
    /// frame. `Some(false)` is the same half chart, `Some(true)` is its
    /// complementary half, and `None` leaves unrelated frames to the general
    /// geometry authority.
    pub(crate) fn shared_frame_chart_relation(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> Classification<Option<bool>> {
        if self.data.clockwise != other.data.clockwise
            || !self.data.frame.shares_storage(&other.data.frame)
        {
            return Classification::Decided(None);
        }
        let same = compare_reals(
            &self.data.radial_distance,
            &other.data.radial_distance,
            policy,
        );
        if same == Some(std::cmp::Ordering::Equal) {
            return Classification::Decided(Some(false));
        }
        let complementary = compare_reals(
            &self.data.radial_distance,
            &-other.data.radial_distance.clone(),
            policy,
        );
        if complementary == Some(std::cmp::Ordering::Equal) {
            Classification::Decided(Some(true))
        } else if same.is_some() && complementary.is_some() {
            Classification::Decided(None)
        } else {
            Classification::Uncertain(UncertaintyReason::Ordering)
        }
    }

    /// Recognizes the same retained complete circle without comparing center
    /// coordinates or materializing either selected field. Opposite signed
    /// radii merely exchange the two diameter endpoints, while traversal and
    /// half-chart orientation do not change the supporting circle.
    pub(crate) fn shares_structural_supporting_circle(&self, other: &Self) -> bool {
        self.data.frame == other.data.frame
            && (self.data.radial_distance == other.data.radial_distance
                || self.data.radial_distance == -other.data.radial_distance.clone())
    }

    /// Returns whether traversal follows the clockwise half circle.
    pub(crate) fn is_clockwise(&self) -> bool {
        self.data.clockwise
    }

    /// Builds the exact concentric left parallel of this selected half circle.
    ///
    /// `radial_distance` is signed because its sign selects which diameter
    /// endpoint is parameter zero.  The geometric radius grows under a left
    /// offset for clockwise traversal and shrinks for counterclockwise
    /// traversal, while retaining that endpoint sign until a genuine radius
    /// collapse.  Crossing the center is still a regular circle whenever the
    /// resulting signed radius is nonzero; the changed sign exactly records
    /// the reversed differential of that parallel branch.
    pub(crate) fn offset_left(
        &self,
        distance: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Self>>> {
        let radial_sign = match real_sign(&self.data.radial_distance, policy) {
            Some(RealSign::Positive) => Real::one(),
            Some(RealSign::Negative) => -Real::one(),
            Some(RealSign::Zero) => {
                return Err(CurveError::Topology(
                    "selected algebraic semicircle retained a zero radius".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        let oriented_distance = if self.data.clockwise {
            radial_sign * distance
        } else {
            -(radial_sign * distance)
        };
        let radial_distance = &self.data.radial_distance + oriented_distance;
        match real_sign(&radial_distance, policy) {
            Some(RealSign::Zero) => Ok(Classification::Decided(None)),
            Some(RealSign::Positive | RealSign::Negative) => {
                Ok(Classification::Decided(Some(Self {
                    data: Arc::new(BezierAlgebraicCuspSemicircleData2 {
                        parallel_system_cache: Mutex::default(),
                        frame: self.data.frame.clone(),
                        radial_distance,
                        clockwise: self.data.clockwise,
                    }),
                })))
            }
            None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
    }

    /// Scales this carrier's signed radius without changing its exact center,
    /// frame, or rational half-circle parameterization.
    ///
    /// This is used when a mapped cut is nonrational on one coincident circle
    /// but is an authored endpoint of another. Scaling both coincident radial
    /// vectors by the same exact factor preserves the shared geometric point
    /// without adjoining either selected center field to the other.
    pub(crate) fn scaled_radial_distance(
        &self,
        scale: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Self>>> {
        let radial_distance = &self.data.radial_distance * scale;
        match real_sign(&radial_distance, policy) {
            Some(RealSign::Zero) => Ok(Classification::Decided(None)),
            Some(RealSign::Positive | RealSign::Negative) => {
                Ok(Classification::Decided(Some(Self {
                    data: Arc::new(BezierAlgebraicCuspSemicircleData2 {
                        parallel_system_cache: Mutex::default(),
                        frame: self.data.frame.clone(),
                        radial_distance,
                        clockwise: self.data.clockwise,
                    }),
                })))
            }
            None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
    }

    /// Returns the same geometric half circle with traversal reversed.
    #[cfg(test)]
    pub(crate) fn reversed(&self) -> Self {
        Self {
            data: Arc::new(BezierAlgebraicCuspSemicircleData2 {
                parallel_system_cache: Mutex::default(),
                frame: self.data.frame.clone(),
                radial_distance: -self.data.radial_distance.clone(),
                clockwise: !self.data.clockwise,
            }),
        }
    }

    /// Returns the other oriented half of the same supporting circle while
    /// preserving counterclockwise/clockwise traversal sense.
    pub(crate) fn complementary_half(&self) -> Self {
        Self {
            data: Arc::new(BezierAlgebraicCuspSemicircleData2 {
                parallel_system_cache: Mutex::default(),
                frame: self.data.frame.clone(),
                radial_distance: -self.data.radial_distance.clone(),
                clockwise: self.data.clockwise,
            }),
        }
    }

    /// Applies a certified similarity without reconstructing the cusp proof.
    ///
    /// The retained parameter remains valid because curve parameterization is
    /// unchanged. Uniform scale multiplies the signed radius, while reflection
    /// also reverses the transformed source's left normal and circle traversal.
    pub(crate) fn transform_similarity(&self, transform: &Similarity2) -> CurveResult<Self> {
        BezierAlgebraicCuspSemicircleSimilarityCache2::default().semicircle(self, transform)
    }

    pub(super) fn represented_frame_scales(
        &self,
        parameter: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<(Real, Real, Real)>> {
        match in_closed_unit_interval(parameter, policy) {
            Some(true) => {}
            Some(false) => return Err(CurveError::InvalidCurveRange),
            None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }
        let one_minus = Real::one() - parameter;
        let denominator = &one_minus * &one_minus + parameter * parameter;
        match real_sign(&denominator, policy) {
            Some(RealSign::Positive) => {}
            Some(RealSign::Zero | RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "semicircle parameter denominator was not positive".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let radial_coefficient = Real::one() - Real::from(2_i8) * parameter;
        let tangent_coefficient = Real::from(2_i8) * parameter * &one_minus;
        let normal_scale = &denominator * self.center_parallel_distance()
            + &radial_coefficient * &self.data.radial_distance;
        let tangent_scale = self.turn_sign() * tangent_coefficient * &self.data.radial_distance;
        Ok(Classification::Decided((
            denominator,
            normal_scale,
            tangent_scale,
        )))
    }

    /// Evaluates an exact point at a represented rational-curve parameter.
    ///
    /// The rational half-circle parameterization is
    /// `C + (A R + B turn*perp(R))/D`, where
    /// `A=1-2u`, `B=2u(1-u)`, and `D=(1-u)^2+u^2`.
    pub(crate) fn point_at(
        &self,
        parameter: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RationalBezierAlgebraicPointImage2>> {
        let (denominator, normal_scale, tangent_scale) =
            match self.represented_frame_scales(parameter, policy)? {
                Classification::Decided(scales) => scales,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let Some(frame) = self.data.frame.rational() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        Ok(Classification::Decided(
            frame.point_image_from_frame_scales(
                &denominator,
                &normal_scale,
                &tangent_scale,
                policy,
            )?,
        ))
    }

    /// Evaluates a represented circle parameter without requiring the center
    /// and source speed radical to share one algebraic-number field.
    pub(crate) fn point_evidence_at(
        &self,
        parameter: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<CurvePoint2>> {
        let (denominator, normal_scale, tangent_scale) =
            match self.represented_frame_scales(parameter, policy)? {
                Classification::Decided(scales) => scales,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        if let Some(frame) = self.data.frame.rational() {
            return Ok(Classification::Decided(CurvePoint2::from(
                frame.point_image_from_frame_scales(
                    &denominator,
                    &normal_scale,
                    &tangent_scale,
                    policy,
                )?,
            )));
        }
        if let Some(frame) = self.data.frame.selected_radial() {
            if !policy.accepts_retained_policy(frame.policy) {
                return Err(CurveError::Topology(
                    "a selected radial circle crossed predicate policies".into(),
                ));
            }
            let common_denominator = &denominator * &frame.normal_denominator;
            match real_sign(&common_denominator, &CurveContext::STRICT) {
                Some(RealSign::Positive | RealSign::Negative) => {}
                Some(RealSign::Zero) => {
                    return Err(CurveError::Topology(
                        "a selected radial point had a zero frame denominator".into(),
                    ));
                }
                None => {
                    return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
                }
            }
            let radial_scale = Real::one() + (&normal_scale / &common_denominator)?;
            let perpendicular_scale = (&tangent_scale / common_denominator)?;
            return Ok(Classification::Decided(CurvePoint2::from(
                BezierAlgebraicCuspChordDerivedPoint2::from_mapped_source_with_rotation(
                    frame.center_parameter.clone(),
                    frame.center_parameter.retained_point_evidence().cloned(),
                    radial_scale,
                    perpendicular_scale,
                ),
            )));
        }
        if let Some(frame) = self.data.frame.chord_normal() {
            if !policy.accepts_retained_policy(frame.policy) {
                return Err(CurveError::Topology(
                    "a chord-normal selected circle crossed predicate policies".into(),
                ));
            }
            let normal_distance = (&normal_scale / &denominator)?;
            // The circle chart rotates the left normal counterclockwise,
            // which is the negative unit tangent of the anchor chord.
            let tangent_distance = -(&tangent_scale / denominator)?;
            let point = frame.anchor.normal_displaced_point_evidence(
                frame.center.clone(),
                normal_distance,
                policy,
            )?;
            return Ok(Classification::Decided(
                frame
                    .anchor
                    .tangent_displaced_point_evidence(point, tangent_distance, policy)?,
            ));
        }
        let frame = self
            .data
            .frame
            .parallel_normal()
            .expect("every non-rational selected-circle frame is parallel-normal");
        if !policy.accepts_retained_policy(frame.policy) {
            return Err(CurveError::Topology(
                "a parallel-normal selected circle crossed predicate policies".into(),
            ));
        }
        let normal_distance = (&normal_scale / &denominator)?;
        // The selected-circle chart uses rotate90(N), which is the negative
        // source unit tangent for the source left normal N.  The analytic
        // point carrier stores displacement along the positive source tangent.
        let tangent_distance = -(&tangent_scale / denominator)?;
        if let Some(center_parameter) = frame.center_parameter.scalar() {
            let parallel = frame.center_support.with_distance(normal_distance.clone());
            if let Classification::Decided(point) =
                parallel.point_at_with_policy(center_parameter, policy)?
            {
                let tangent = match parallel.source_tangent_at(center_parameter, policy)? {
                    Classification::Decided(tangent) => tangent,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let speed = (&tangent.0 * &tangent.0 + &tangent.1 * &tangent.1).sqrt()?;
                let tangent_x = (&tangent.0 * &tangent_distance / &speed)?;
                let tangent_y = (tangent.1 * &tangent_distance / speed)?;
                return Ok(Classification::Decided(CurvePoint2::from(
                    point.translated(tangent_x, tangent_y),
                )));
            }
        }
        Ok(Classification::Decided(CurvePoint2::from(
            BezierAnalyticParallelPoint2::new_with_region_parameter_and_tangent_distance(
                frame.center_support.with_distance(normal_distance),
                &frame.center_parameter,
                tangent_distance,
                policy,
            )
            .expect("a parallel-normal frame owns a scalar parameter"),
        )))
    }

    /// Evaluates the exact traversal tangent at a represented parameter.
    pub(crate) fn tangent_at(
        &self,
        parameter: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RationalBezierAlgebraicTangentImage2>> {
        let Some(frame) = self.data.frame.rational() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        match in_closed_unit_interval(parameter, policy) {
            Some(true) => {}
            Some(false) => return Err(CurveError::InvalidCurveRange),
            None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }
        let one_minus = Real::one() - parameter;
        let denominator = &one_minus * &one_minus + parameter * parameter;
        match real_sign(&denominator, policy) {
            Some(RealSign::Positive) => {}
            Some(RealSign::Zero | RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "semicircle tangent denominator was not positive".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let radial_coefficient = Real::from(-4_i8) * parameter * &one_minus;
        let tangent_coefficient =
            self.turn_sign() * Real::from(2_i8) * (Real::one() - Real::from(2_i8) * parameter);
        frame.tangent_image_from_frame_scales(
            &(&self.data.radial_distance * radial_coefficient),
            &(&self.data.radial_distance * tangent_coefficient),
            &(&denominator * &denominator),
            policy,
        )
    }

    pub(crate) fn start_point_evidence(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<CurvePoint2>> {
        self.point_evidence_at(&Real::zero(), policy)
    }

    pub(crate) fn end_point_evidence(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<CurvePoint2>> {
        self.point_evidence_at(&Real::one(), policy)
    }

    pub(crate) fn center_point_evidence(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<CurvePoint2>> {
        if self.data.frame.rational().is_some() {
            return Ok(Classification::Decided(CurvePoint2::from(
                self.center_point_image(policy)?,
            )));
        }
        if let Some(frame) = self.data.frame.selected_radial() {
            if !policy.accepts_retained_policy(frame.policy) {
                return Err(CurveError::Topology(
                    "a selected radial circle crossed predicate policies".into(),
                ));
            }
            return Ok(Classification::Decided(CurvePoint2::from(
                BezierAlgebraicCuspChordDerivedPoint2::from_mapped_source(
                    frame.center_parameter.clone(),
                    frame.center_parameter.retained_point_evidence().cloned(),
                    Real::one(),
                ),
            )));
        }
        if let Some(frame) = self.data.frame.chord_normal() {
            if !policy.accepts_retained_policy(frame.policy) {
                return Err(CurveError::Topology(
                    "a chord-normal selected circle crossed predicate policies".into(),
                ));
            }
            return Ok(Classification::Decided(frame.center.clone()));
        }
        let frame = self
            .data
            .frame
            .parallel_normal()
            .expect("every non-rational selected-circle frame is parallel-normal");
        if !policy.accepts_retained_policy(frame.policy) {
            return Err(CurveError::Topology(
                "a parallel-normal selected circle crossed predicate policies".into(),
            ));
        }
        Ok(Classification::Decided(CurvePoint2::from(
            BezierAnalyticParallelPoint2::new_with_region_parameter_and_tangent_distance(
                frame.center_support.clone(),
                &frame.center_parameter,
                Real::zero(),
                policy,
            )
            .expect("a parallel-normal frame owns a scalar parameter"),
        )))
    }

    /// Returns the center when both coordinates can be materialized as exact
    /// [`Real`] values. The coordinates need not have rational normal forms.
    pub(crate) fn exact_center(&self, policy: &CurveContext) -> CurveResult<Option<Point2>> {
        if let Some(frame) = self.data.frame.parallel_normal() {
            if !policy.accepts_retained_policy(frame.policy) {
                return Err(CurveError::Topology(
                    "a parallel-normal center crossed predicate policies".into(),
                ));
            }
            return match frame.center_parameter.as_bezier_parameter() {
                Some(BezierParameter2::Exact(parameter)) => Ok(
                    match policy.strict_predicate_pass(|| {
                        frame.center_support.point_at_with_policy(parameter, policy)
                    })? {
                        Classification::Decided(center) => Some(center),
                        Classification::Uncertain(_) => None,
                    },
                ),
                Some(BezierParameter2::Algebraic(parameter)) => {
                    let curve = match policy.strict_predicate_pass(|| {
                        frame
                            .center_support
                            .exact_rational_parallel_component(policy)
                    })? {
                        Classification::Decided(Some(curve)) => curve,
                        Classification::Decided(None) | Classification::Uncertain(_) => {
                            return Ok(None);
                        }
                    };
                    Ok(RationalBezierAlgebraicPointImage2::from_parametric_source(
                        curve,
                        parameter.clone(),
                        policy,
                    )
                    .exact_point(&CurveContext::STRICT))
                }
                None => Ok(None),
            };
        }
        if let Some(frame) = self.data.frame.selected_radial() {
            if !policy.accepts_retained_policy(frame.policy) {
                return Err(CurveError::Topology(
                    "a selected radial center crossed predicate policies".into(),
                ));
            }
            if let BezierAlgebraicCuspSemicircleMappedParameterData2::Pair {
                map,
                contact,
                first,
            } = frame.center_parameter.as_ref()
                && let Classification::Decided([x, y]) = map
                    .represented_selected_radial_derived_point(
                        contact,
                        *first,
                        &Real::one(),
                        &Real::zero(),
                        &Real::zero(),
                        &Real::zero(),
                        policy,
                    )?
                && let (Some(x), Some(y)) = (x.exact_point_witness(), y.exact_point_witness())
            {
                return Ok(Some(Point2::new(x.clone(), y.clone())));
            }
        }
        Ok(match self.center_point_evidence(policy)? {
            Classification::Decided(CurvePoint2(CurvePointData2::Exact(center))) => Some(center),
            Classification::Decided(CurvePoint2(CurvePointData2::Algebraic(center))) => {
                center.exact_point(&CurveContext::STRICT)
            }
            Classification::Decided(center) => {
                // The imported point may already have scalar witnesses even
                // though its public carrier deliberately retains its source
                // and parameter. Reuse that view before rebuilding separate
                // coordinate roots for the circle's elimination system.
                match policy.bounded_exact_predicate_pass(|| {
                    recursive_projective_point_source(&center, policy)
                })? {
                    Classification::Decided(Some(
                        BezierRecursiveProjectivePointSource2::Recursive(point),
                    )) => point.exact_point_with_retained_witnesses(),
                    Classification::Decided(Some(
                        BezierRecursiveProjectivePointSource2::Exact(point),
                    )) => Some(point),
                    Classification::Decided(Some(
                        BezierRecursiveProjectivePointSource2::Algebraic(point),
                    )) => point.exact_point(&CurveContext::STRICT),
                    Classification::Decided(None) | Classification::Uncertain(_) => None,
                }
            }
            Classification::Uncertain(_) => None,
        })
    }

    /// Retains a caller-certified tangent contact on a general selected-frame
    /// circle without rebuilding the circle/parallel elimination system.
    ///
    /// `other_radial_sign` is the sign of the center-to-contact radius along
    /// `other`'s source left normal.  The fillet solver already proves the
    /// incidence and supplies that orientation from the authored offset; this
    /// method independently certifies that the resulting angular parameter is
    /// at the caller-retained location on this selected half. Endpoint
    /// contacts remain mapped so later edits retain their tangent authority.
    pub(crate) fn certified_selected_parallel_contact_parameter(
        &self,
        other: BezierParallel2,
        other_parameter: CurveParameter2,
        location: BezierAlgebraicCuspSemicircleContactLocation2,
        other_radial_sign: RealSign,
        tangent_cross_sign: RealSign,
        tangent_dot_sign: RealSign,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicircleParameter2>> {
        let frame_policy = if let Some(frame) = self.data.frame.parallel_normal() {
            frame.policy
        } else if let Some(frame) = self.data.frame.chord_normal() {
            frame.policy
        } else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        if !policy.accepts_retained_policy(frame_policy) {
            return Err(CurveError::Topology(
                "a selected parallel contact crossed predicate policies".into(),
            ));
        }
        if other_radial_sign == RealSign::Zero {
            return Err(CurveError::Topology(
                "a nonzero fillet retained a zero contact-radius orientation".into(),
            ));
        }
        if tangent_cross_sign == RealSign::Zero && tangent_dot_sign == RealSign::Zero {
            return Err(CurveError::Topology(
                "a regular fillet contact retained a zero tangent relation".into(),
            ));
        }
        match other.parallel_derivative_scale_sign(&other_parameter, policy)? {
            Classification::Decided(RealSign::Positive | RealSign::Negative) => {}
            Classification::Decided(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let radial_sign = match real_sign(self.radial_distance(), policy) {
            Some(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
            Some(RealSign::Zero) => {
                return Err(CurveError::Topology(
                    "a selected circle retained a zero signed radius".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        let mut parameter =
            BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedParallelContact {
                semicircle: self.clone(),
                parallel: other,
                parameter: other_parameter,
                location,
                radial_product_sign: product_sign(radial_sign, other_radial_sign),
                tangent_cross_sign,
                tangent_dot_sign,
                policy: *policy,
            };
        let expected_orders = match location {
            BezierAlgebraicCuspSemicircleContactLocation2::Interior => [
                (Real::zero(), std::cmp::Ordering::Greater),
                (Real::one(), std::cmp::Ordering::Less),
            ],
            BezierAlgebraicCuspSemicircleContactLocation2::Start => [
                (Real::zero(), std::cmp::Ordering::Equal),
                (Real::one(), std::cmp::Ordering::Less),
            ],
            BezierAlgebraicCuspSemicircleContactLocation2::End => [
                (Real::zero(), std::cmp::Ordering::Greater),
                (Real::one(), std::cmp::Ordering::Equal),
            ],
        };
        for (boundary, expected) in expected_orders {
            match parameter.selected_parallel_contact_order_to_real(&boundary, policy)? {
                Classification::Decided(order) if order == expected => {}
                Classification::Decided(_order) => {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "selected-parallel-contact-half",
                        match (expected, _order) {
                            (std::cmp::Ordering::Greater, std::cmp::Ordering::Less) => {
                                "start-expected-greater-got-less"
                            }
                            (std::cmp::Ordering::Greater, std::cmp::Ordering::Equal) => {
                                "start-expected-greater-got-equal"
                            }
                            (std::cmp::Ordering::Less, std::cmp::Ordering::Greater) => {
                                "end-expected-less-got-greater"
                            }
                            (std::cmp::Ordering::Less, std::cmp::Ordering::Equal) => {
                                "end-expected-less-got-equal"
                            }
                            _ => "unexpected-order",
                        },
                    );
                    return Err(CurveError::Topology(
                        "a certified fillet contact lay outside its selected circle half".into(),
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        // Publication follows every construction predicate. A requested
        // approximate policy does not weaken certified contacts, while a
        // genuinely approximate frame remains an explicit dependency.
        let BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedParallelContact {
            policy: retained_policy,
            ..
        } = &mut parameter
        else {
            unreachable!()
        };
        *retained_policy = policy.retained_object_policy_with_dependencies([frame_policy]);
        Ok(Classification::Decided(
            BezierAlgebraicCuspSemicircleParameter2::Mapped(Arc::new(parameter)),
        ))
    }

    /// Retains an authored round-join contact whose radial direction is the
    /// left normal of a selected-circle endpoint tangent.
    ///
    /// The caller supplies the exact endpoint evidence and the signs relating
    /// each signed radial to its traversal tangent. The companion endpoint's
    /// original two-normal contact map remains the sole angular predicate
    /// authority; this constructor only selects the corresponding open point
    /// on this half circle.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn certified_selected_circular_tangent_contact_parameter(
        &self,
        companion: BezierAlgebraicCuspSemicircleFragment2,
        companion_at_start: bool,
        parallel: BezierParallel2,
        parameter: CurveParameter2,
        source_direction: RealSign,
        radial_product_sign: RealSign,
        point: CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicircleParameter2>> {
        if source_direction == RealSign::Zero || radial_product_sign == RealSign::Zero {
            return Err(CurveError::Topology(
                "a retained round contact supplied a zero orientation factor".into(),
            ));
        }
        companion.validate_policy(policy)?;
        let mut parameter =
            BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedCircularTangentContact {
                semicircle: self.clone(),
                companion,
                companion_at_start,
                parallel,
                parameter,
                source_direction,
                radial_product_sign,
                point,
                policy: *policy,
            };
        for (boundary, expected) in [
            (Real::zero(), std::cmp::Ordering::Greater),
            (Real::one(), std::cmp::Ordering::Less),
        ] {
            match parameter.selected_circular_tangent_contact_order_to_real(&boundary, policy)? {
                Classification::Decided(order) if order == expected => {}
                Classification::Decided(_) => {
                    return Err(CurveError::Topology(
                        "a certified round contact lay outside its selected circle half".into(),
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        let BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedCircularTangentContact {
            companion,
            policy: retained_policy,
            ..
        } = &mut parameter
        else {
            unreachable!()
        };
        *retained_policy = policy.retained_object_policy_with_dependencies(
            self.data
                .frame
                .evidence_policy()
                .into_iter()
                .chain(Some(companion.data.policy)),
        );
        Ok(Classification::Decided(
            BezierAlgebraicCuspSemicircleParameter2::Mapped(Arc::new(parameter)),
        ))
    }

    /// Retains the second endpoint of a pair-native selected fillet circle.
    /// The circle frame already owns the exact pair contact that selected its
    /// center, so this adds only the signed radial relation and the exact
    /// endpoint point evidence needed by downstream topology.
    pub(crate) fn certified_selected_pair_contact_parameter(
        &self,
        companion_source: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierAlgebraicCuspSemicircleParameter2>>> {
        let Some(frame) = self.data.frame.selected_radial() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        if !policy.accepts_retained_policy(frame.policy) {
            return Err(CurveError::Topology(
                "a selected pair contact crossed predicate policies".into(),
            ));
        }
        let BezierAlgebraicCuspSemicircleMappedParameterData2::Pair {
            map,
            contact,
            first,
        } = frame.center_parameter.as_ref()
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        if !policy.accepts_retained_policy(map.data.policy) {
            return Err(CurveError::Topology(
                "a selected pair contact map crossed predicate policies".into(),
            ));
        }
        let (anchor_support, companion_support, companion_parameter) = if *first {
            (
                &map.data.first_semicircle,
                &map.data.second_semicircle,
                map.second_contact_parameter(contact),
            )
        } else {
            (
                &map.data.second_semicircle,
                &map.data.first_semicircle,
                map.first_contact_parameter(contact),
            )
        };
        if anchor_support != frame.center_parameter.semicircle_carrier()
            || companion_source.data.frame != companion_support.data.frame
            || companion_source.is_clockwise() != companion_support.is_clockwise()
        {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        let expected_point = match companion_parameter.concentric_offset_point_evidence(
            companion_support,
            companion_source,
            policy,
        )? {
            Classification::Decided(Some(point)) => point,
            Classification::Decided(None) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let companion_radial_numerator =
            companion_source.radial_distance() - companion_support.radial_distance();
        let mut radial_product_sign = RealSign::Positive;
        for factor in [
            self.radial_distance(),
            &frame.normal_denominator,
            &companion_radial_numerator,
            companion_support.radial_distance(),
        ] {
            let sign = match real_sign(factor, policy) {
                Some(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
                Some(RealSign::Zero) => {
                    return Err(CurveError::Topology(
                        "a retained pair fillet radial orientation collapsed".into(),
                    ));
                }
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            };
            radial_product_sign = product_sign(radial_product_sign, sign);
        }
        if map.data.first_semicircle.is_clockwise() != map.data.second_semicircle.is_clockwise() {
            radial_product_sign = product_sign(radial_product_sign, RealSign::Negative);
        }
        let mut parameter =
            BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedPairContact {
                semicircle: self.clone(),
                map: map.clone(),
                contact: contact.clone(),
                anchor_first: *first,
                radial_product_sign,
                point: expected_point,
                policy: *policy,
            };
        for (boundary, expected) in [
            (Real::zero(), std::cmp::Ordering::Greater),
            (Real::one(), std::cmp::Ordering::Less),
        ] {
            match parameter.selected_pair_contact_order_to_real(&boundary, policy)? {
                Classification::Decided(order) if order == expected => {}
                Classification::Decided(_) => {
                    return Ok(Classification::Decided(None));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        let BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedPairContact {
            policy: retained_policy,
            ..
        } = &mut parameter
        else {
            unreachable!()
        };
        *retained_policy =
            policy.retained_object_policy_with_dependencies([frame.policy, map.data.policy]);
        Ok(Classification::Decided(Some(
            BezierAlgebraicCuspSemicircleParameter2::Mapped(Arc::new(parameter)),
        )))
    }

    /// Retains a general round-join endpoint on a selected chord normal.
    ///
    /// The checks below reconstruct and compare the exact source chord point
    /// displaced by the supplied signed radial distance along the chord's unit
    /// left normal. This accepts both the compact retained-parallel evidence
    /// and represented-vector translations without weakening the proof. The
    /// The retained anchor selects either a represented unit tangent or a
    /// general algebraic chord frame. After that frame proof, both cases use
    /// the same incidence, radius, angular-order, and policy authority; no
    /// coordinate compositum or circle/chord elimination system is built.
    /// `incidence_certified` is reserved for endpoints authored by the same
    /// construction that supplied the signed displacement.
    pub(crate) fn certified_chord_normal_contact_parameter(
        &self,
        anchor_tangent: BezierSelectedChordNormalAnchor2,
        chord: BezierAlgebraicChord2,
        point: CurvePoint2,
        contact_radial_distance: Real,
        incidence_certified: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicircleParameter2>> {
        chord.validate_policy(policy)?;
        let center = match &anchor_tangent {
            BezierSelectedChordNormalAnchor2::Represented(anchor_tangent) => {
                let Some(frame_normal) = self.data.frame.represented_unit_normal()? else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                };
                let expected_normal = (-anchor_tangent.1.clone(), anchor_tangent.0.clone());
                if (frame_normal.0 - &expected_normal.0).zero_status() != ZeroKnowledge::Zero
                    || (frame_normal.1 - &expected_normal.1).zero_status() != ZeroKnowledge::Zero
                {
                    return Err(CurveError::Topology(
                        "a selected chord-normal contact did not share its circle frame".into(),
                    ));
                }
                match self.center_point_evidence(policy)? {
                    Classification::Decided(center) => center,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            BezierSelectedChordNormalAnchor2::RetainedChord(anchor) => {
                anchor.validate_policy(policy)?;
                let Some(frame) = self.data.frame.chord_normal() else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                };
                if !policy.accepts_retained_policy(frame.policy) {
                    return Err(CurveError::Topology(
                        "a selected chord-normal contact crossed predicate policies".into(),
                    ));
                }
                if frame.anchor != *anchor {
                    return Err(CurveError::Topology(
                        "a selected chord-normal contact did not share its anchor frame".into(),
                    ));
                }
                frame.center.clone()
            }
            BezierSelectedChordNormalAnchor2::RetainedCircleChord { .. }
            | BezierSelectedChordNormalAnchor2::RetainedCircleRationalChord { .. } => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
        };
        let contact_radial_distance = if incidence_certified {
            contact_radial_distance
        } else {
            match chord.certified_normal_displacement_distance(
                &center,
                &point,
                &contact_radial_distance,
                policy,
            )? {
                Classification::Decided(distance) => distance,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        };
        let radius_residual = self.radial_distance() * self.radial_distance()
            - &contact_radial_distance * &contact_radial_distance;
        match real_sign(&radius_residual, policy) {
            Some(RealSign::Zero) => {}
            Some(RealSign::Positive | RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "a selected chord-normal contact had unequal radial lengths".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let radial_product_sign =
            match real_sign(&(self.radial_distance() * &contact_radial_distance), policy) {
                Some(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
                Some(RealSign::Zero) => {
                    return Err(CurveError::Topology(
                        "a selected chord-normal contact retained a zero radius".into(),
                    ));
                }
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            };
        let mut parameter =
            BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedChordNormalContact {
                semicircle: self.clone(),
                anchor_tangent,
                chord,
                radial_product_sign,
                point,
                policy: *policy,
            };
        for (boundary, expected) in [
            (Real::zero(), std::cmp::Ordering::Greater),
            (Real::one(), std::cmp::Ordering::Less),
        ] {
            match parameter.selected_chord_normal_contact_order_to_real(&boundary, policy)? {
                Classification::Decided(order) if order == expected => {}
                Classification::Decided(_) => {
                    return Err(CurveError::Topology(
                        "a certified chord-normal contact lay outside its selected circle half"
                            .into(),
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "curve-region-exact-offset-tangent",
            "selected-chord-normal-contact",
        );
        let BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedChordNormalContact {
            chord,
            policy: retained_policy,
            ..
        } = &mut parameter
        else {
            unreachable!()
        };
        *retained_policy = policy.retained_object_policy_with_dependencies(
            self.data
                .frame
                .evidence_policy()
                .into_iter()
                .chain(Some(chord.data.policy)),
        );
        Ok(Classification::Decided(
            BezierAlgebraicCuspSemicircleParameter2::Mapped(Arc::new(parameter)),
        ))
    }

    /// Retains a chord-normal fillet endpoint on a circle whose parameter-zero
    /// radius is selected by an analytic-parallel tangent.
    ///
    /// `point` must be the exact center of this circle displaced along
    /// `chord`'s unit left normal. A retained chord-parallel point supplies its
    /// authored distance directly; represented-vector translations use the
    /// explicit `contact_radial_distance`. The two signed radial distances
    /// must have equal magnitude. The caller also supplies the already-replayed
    /// nonzero tangent cross that selected this circle half, so construction
    /// does not repeat a three-axis boundary predicate merely to prove that
    /// the contact is not a diameter endpoint.
    pub(crate) fn certified_selected_chord_parallel_normal_contact_parameter(
        &self,
        chord: BezierAlgebraicChord2,
        point: CurvePoint2,
        contact_radial_distance: Real,
        tangent_cross_sign: RealSign,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicircleParameter2>> {
        chord.validate_policy(policy)?;
        let Some(frame) = self.data.frame.parallel_normal() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        if !policy.accepts_retained_policy(frame.policy) {
            return Err(CurveError::Topology(
                "a selected chord/parallel-normal contact crossed predicate policies".into(),
            ));
        }
        if tangent_cross_sign == RealSign::Zero {
            return Err(CurveError::Topology(
                "a selected chord/parallel-normal interior contact had parallel tangents".into(),
            ));
        }
        let center = CurvePoint2::from(
            BezierAnalyticParallelPoint2::new_with_region_parameter_and_tangent_distance(
                frame.center_support.clone(),
                &frame.center_parameter,
                Real::zero(),
                policy,
            )
            .expect("a parallel-normal frame owns a scalar parameter"),
        );
        let contact_radial_distance = match chord.certified_normal_displacement_distance(
            &center,
            &point,
            &contact_radial_distance,
            policy,
        )? {
            Classification::Decided(distance) => distance,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let radius_residual = self.radial_distance() * self.radial_distance()
            - &contact_radial_distance * &contact_radial_distance;
        match real_sign(&radius_residual, policy) {
            Some(RealSign::Zero) => {}
            Some(RealSign::Positive | RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "a selected chord/parallel-normal contact had unequal radial lengths".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let radial_product_sign =
            match real_sign(&(self.radial_distance() * &contact_radial_distance), policy) {
                Some(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
                Some(RealSign::Zero) => {
                    return Err(CurveError::Topology(
                        "a selected chord/parallel-normal contact retained a zero radius".into(),
                    ));
                }
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            };
        let policy =
            policy.retained_object_policy_with_dependencies([frame.policy, chord.data.policy]);
        let parameter = BezierAlgebraicCuspSemicircleParameter2::Mapped(Arc::new(
            BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedChordParallelNormalContact {
                semicircle: self.clone(),
                parallel: frame.center_support.clone(),
                parallel_parameter: frame.center_parameter.clone(),
                chord,
                radial_product_sign,
                point,
                policy,
            },
        ));
        Ok(Classification::Decided(parameter))
    }

    /// Retains the chord endpoint of a selected-concentric fillet directly
    /// from the circle/chord contact that authored its center frame.
    ///
    /// The center parameter already owns the exact tangent cross and dot
    /// tower between the selected anchor circle and the parallel chord. The
    /// terminal circle starts on that anchor radial, so its angular parameter
    /// is a linear combination in the same tower. Reusing it avoids solving a
    /// recursively selected circle against the source chord a second time.
    pub(crate) fn certified_selected_chord_retained_contact_parameter(
        &self,
        chord: BezierAlgebraicChord2,
        point: CurvePoint2,
        contact_radial_distance: Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierAlgebraicCuspSemicircleParameter2>>> {
        chord.validate_policy(policy)?;
        let Some(frame) = self.data.frame.selected_radial() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        if !policy.accepts_retained_policy(frame.policy) {
            return Err(CurveError::Topology(
                "a retained chord contact crossed selected-radial policies".into(),
            ));
        }
        let (
            anchor_tangent,
            anchor_clockwise,
            construction_radial_distance,
            target_tangent_reversed,
        ) = match frame.center_parameter.as_ref() {
            BezierAlgebraicCuspSemicircleMappedParameterData2::Chord { map, contact } => {
                map.validate_policy(policy)?;
                let target_tangent_reversed = match map
                    .data
                    .chord
                    .certified_parallel_tangent_reversal_to(&chord, policy)?
                {
                    Classification::Decided(reversed) => reversed,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                if map.retained_offset_system().is_none()
                    && map.represented_oblique_system().is_none()
                    && map.axis_system().is_none()
                    && map.recursive_quadratic_line_system().is_none()
                    && !map.has_chord_normal_projective_system()
                {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                }
                (
                    BezierSelectedChordNormalAnchor2::RetainedCircleChord {
                        map: map.clone(),
                        contact: contact.clone(),
                    },
                    map.data.semicircle.is_clockwise(),
                    map.data
                        .chord
                        .retained_normal_offset_distance_with_tangent_reversal(
                            target_tangent_reversed,
                        )
                        .or_else(|| {
                            map.data
                                .chord
                                .exact_left_normal_support_distance_to(&chord, policy)
                        })
                        .map(|distance| -distance),
                    target_tangent_reversed,
                )
            }
            BezierAlgebraicCuspSemicircleMappedParameterData2::Rational { map, contact } => {
                if !policy.accepts_retained_policy(map.data.policy) {
                    return Err(CurveError::Topology(
                        "a retained rational chord contact crossed predicate policies".into(),
                    ));
                }
                let BezierAlgebraicCuspSemicircleRationalCorrelation2::MapWithChordTangent {
                    chord: mapped_chord,
                    ..
                } = &contact.correlation
                else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                };
                let target_tangent_reversed =
                    match mapped_chord.certified_parallel_tangent_reversal_to(&chord, policy)? {
                        Classification::Decided(reversed) => reversed,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                (
                    BezierSelectedChordNormalAnchor2::RetainedCircleRationalChord {
                        map: map.clone(),
                        contact: contact.clone(),
                    },
                    map.data.semicircle.is_clockwise(),
                    mapped_chord
                        .retained_normal_offset_distance_with_tangent_reversal(
                            target_tangent_reversed,
                        )
                        .or_else(|| {
                            mapped_chord.exact_left_normal_support_distance_to(&chord, policy)
                        })
                        .map(|distance| -distance),
                    target_tangent_reversed,
                )
            }
            _ => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
        };
        let contact_radial_distance = if let Some(distance) = construction_radial_distance {
            // The circle/chord center lies on a procedural normal offset of
            // this source chord, and the fillet cut preserves the contact's
            // affine parameter through its finite-envelope rechart. The
            // signed ancestry distance is therefore the complete incidence
            // certificate; rebuilding coordinate equality would discard that
            // correlation and join the same recursive fields a second time.
            distance
        } else {
            let center_parameter =
                BezierAlgebraicCuspSemicircleParameter2::Mapped(frame.center_parameter.clone());
            let center = match center_parameter
                .coincident_point_evidence(frame.center_parameter.semicircle_carrier(), policy)?
            {
                Classification::Decided(Some(center)) => center,
                Classification::Decided(None) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            match chord.certified_normal_displacement_distance(
                &center,
                &point,
                &contact_radial_distance,
                policy,
            )? {
                Classification::Decided(distance) => distance,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        };
        let radius_residual = self.radial_distance() * self.radial_distance()
            - &contact_radial_distance * &contact_radial_distance;
        match real_sign(&radius_residual, policy) {
            Some(RealSign::Zero) => {}
            Some(RealSign::Positive | RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "a retained chord contact had unequal radial lengths".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let anchor_normal_sign = if anchor_clockwise {
            Real::one()
        } else {
            Real::from(-1_i8)
        };
        let radial_product_sign = match real_sign(
            &(self.radial_distance()
                * &contact_radial_distance
                * &frame.normal_denominator
                * anchor_normal_sign),
            policy,
        ) {
            Some(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
            Some(RealSign::Zero) => {
                return Err(CurveError::Topology(
                    "a retained chord contact kept a zero radial frame".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        let mut parameter =
            BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedChordNormalContact {
                semicircle: self.clone(),
                anchor_tangent,
                chord,
                radial_product_sign,
                point,
                policy: *policy,
            };
        let is_interior = |parameter: &BezierAlgebraicCuspSemicircleMappedParameterData2|
         -> CurveResult<Classification<bool>> {
            for (boundary, expected) in [
                (Real::zero(), std::cmp::Ordering::Greater),
                (Real::one(), std::cmp::Ordering::Less),
            ] {
                match parameter.selected_chord_normal_contact_order_to_real(&boundary, policy)? {
                    Classification::Decided(order) if order == expected => {}
                    Classification::Decided(_) => {
                        return Ok(Classification::Decided(false));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            Ok(Classification::Decided(true))
        };
        let mut interior = is_interior(&parameter)?;
        if matches!(interior, Classification::Decided(false)) && target_tangent_reversed {
            // A reversed target tangent reverses both its signed normal
            // distance and its normal basis. Their product therefore leaves
            // two equivalent ancestry descriptions of the contact radial.
            // Exactly one antipodal sign can lie strictly inside this selected
            // semicircle; use the retained angular predicate to choose that
            // chart without deriving a special case from the source kind.
            let BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedChordNormalContact {
                radial_product_sign,
                ..
            } = &mut parameter
            else {
                unreachable!("the retained chord parameter was just constructed")
            };
            *radial_product_sign = product_sign(*radial_product_sign, RealSign::Negative);
            interior = is_interior(&parameter)?;
        }
        let BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedChordNormalContact {
            chord,
            policy: retained_policy,
            ..
        } = &mut parameter
        else {
            unreachable!()
        };
        *retained_policy =
            policy.retained_object_policy_with_dependencies([frame.policy, chord.data.policy]);
        match interior {
            Classification::Decided(true) => Ok(Classification::Decided(Some(
                BezierAlgebraicCuspSemicircleParameter2::Mapped(Arc::new(parameter)),
            ))),
            Classification::Decided(false) => Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    pub(crate) fn start_point_image(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<RationalBezierAlgebraicPointImage2> {
        let distance = self.center_parallel_distance() + &self.data.radial_distance;
        self.data
            .frame
            .point_image_at_parallel_distance(&distance, policy)
    }

    pub(crate) fn end_point_image(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<RationalBezierAlgebraicPointImage2> {
        let distance = self.center_parallel_distance() - &self.data.radial_distance;
        self.data
            .frame
            .point_image_at_parallel_distance(&distance, policy)
    }

    pub(crate) fn center_point_image(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<RationalBezierAlgebraicPointImage2> {
        if let Some(center) = &self.data.frame.rational_required()?.data.direct_center {
            return Ok(center.clone());
        }
        self.data
            .frame
            .point_image_at_parallel_distance(&self.center_parallel_distance(), policy)
    }

    /// Certifies whether retained rational-circle metadata describes this
    /// complete supporting circle.  Coordinate comparisons stay in the cusp
    /// root's local field, so a structurally retained circle does not need to
    /// pass through the more expensive identically-zero resultant path.
    pub(super) fn has_same_supporting_circle(
        &self,
        center: &Point2,
        radius_squared: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        let center_image = self.center_point_image(policy)?;
        let mut uncertainty = None;
        for (use_x, coordinate) in [(true, center.x()), (false, center.y())] {
            let order = center_image.coordinate_order_to_real(use_x, coordinate, policy)?;
            match order {
                Classification::Decided(std::cmp::Ordering::Equal) => {}
                Classification::Decided(std::cmp::Ordering::Less | std::cmp::Ordering::Greater) => {
                    return Ok(Classification::Decided(false));
                }
                Classification::Uncertain(reason) => {
                    uncertainty.get_or_insert(reason);
                }
            }
        }
        let radius_difference = self.radial_distance() * self.radial_distance() - radius_squared;
        let radius_sign = real_sign(&radius_difference, policy);
        match radius_sign {
            Some(RealSign::Zero) => {}
            Some(RealSign::Positive | RealSign::Negative) => {
                return Ok(Classification::Decided(false));
            }
            None => {
                uncertainty.get_or_insert(UncertaintyReason::RealSign);
            }
        }
        Ok(uncertainty.map_or(Classification::Decided(true), Classification::Uncertain))
    }

    /// Certifies retained circular-conic metadata against an arbitrary-depth
    /// selected-radial frame without materializing Cartesian coordinates.
    /// This is a structural fast path only: a non-match or an unproved
    /// equality rejoins the complete recursive incidence projection.
    pub(super) fn recursive_selected_radial_has_same_supporting_circle(
        &self,
        center: &Point2,
        radius_squared: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        if let Some(retained_center) = self.exact_center(policy)? {
            for (actual, expected) in [
                (retained_center.x(), center.x()),
                (retained_center.y(), center.y()),
            ] {
                match compare_reals(actual, expected, &CurveContext::STRICT) {
                    Some(std::cmp::Ordering::Equal) => {}
                    Some(std::cmp::Ordering::Less | std::cmp::Ordering::Greater) => {
                        return Ok(Classification::Decided(false));
                    }
                    None => {
                        return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
                    }
                }
            }
        } else {
            let frame = match self.recursive_circle_frame_authority(policy)? {
                Classification::Decided(Some(frame)) => frame,
                Classification::Decided(None) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            for (coordinate, expected) in
                [(&frame.center.x, center.x()), (&frame.center.y, center.y())]
            {
                let difference = coordinate
                    .subtract(&frame.center.denominator.scale(expected).ok_or_else(|| {
                        CurveError::Topology(
                            "a recursive supporting-circle center exceeded its field budget".into(),
                        )
                    })?)
                    .ok_or_else(|| {
                        CurveError::Topology(
                            "a recursive supporting-circle center crossed retained fields".into(),
                        )
                    })?;
                if difference.is_structurally_zero() {
                    continue;
                }
                match policy.strict_predicate_pass(|| difference.sign(policy))? {
                    Classification::Decided(RealSign::Zero) => {}
                    Classification::Decided(RealSign::Positive | RealSign::Negative) => {
                        return Ok(Classification::Decided(false));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
        }
        let retained_radius_squared = self.radial_distance() * self.radial_distance();
        Ok(
            match compare_reals(
                &retained_radius_squared,
                radius_squared,
                &CurveContext::STRICT,
            ) {
                Some(std::cmp::Ordering::Equal) => Classification::Decided(true),
                Some(std::cmp::Ordering::Less | std::cmp::Ordering::Greater) => {
                    Classification::Decided(false)
                }
                None => Classification::Uncertain(UncertaintyReason::RealSign),
            },
        )
    }

    /// Proves that this supporting circle cannot meet one retained exact
    /// circle.  The ordinary circle-circle discriminant is signed directly in
    /// the cusp parameter's local field; a negative value excludes both
    /// transverse and tangent contacts without materializing either center.
    pub(super) fn is_disjoint_from_supporting_circle(
        &self,
        center: &Point2,
        radius_squared: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        let Some(frame) = self.data.frame.rational() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let frame = &frame.data;
        let (center_x, center_y) = self
            .data
            .frame
            .point_numerators_at_parallel_distance(&self.center_parallel_distance())?;
        let delta_x =
            polynomial_subtract(&polynomial_scale(&frame.denominator, center.x()), &center_x);
        let delta_y =
            polynomial_subtract(&polynomial_scale(&frame.denominator, center.y()), &center_y);
        let center_distance_squared = polynomial_add(
            &polynomial_multiply(&delta_x, &delta_x),
            &polynomial_multiply(&delta_y, &delta_y),
        );
        let denominator_squared = polynomial_multiply(&frame.denominator, &frame.denominator);
        let first_radius_squared = self.radial_distance() * self.radial_distance();
        let center_line = polynomial_add(
            &center_distance_squared,
            &polynomial_scale(
                &denominator_squared,
                &(&first_radius_squared - radius_squared),
            ),
        );
        let discriminant = polynomial_subtract(
            &polynomial_scale(
                &polynomial_multiply(&center_distance_squared, &denominator_squared),
                &(Real::from(4_i8) * first_radius_squared),
            ),
            &polynomial_multiply(&center_line, &center_line),
        );
        Ok(
            match signed_coefficients_at_parameter(
                &discriminant,
                &BezierParameter2::Algebraic(self.cusp_parameter().clone()),
                policy,
            )? {
                Classification::Decided(RealSign::Negative) => Classification::Decided(true),
                Classification::Decided(RealSign::Zero | RealSign::Positive) => {
                    Classification::Decided(false)
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
        )
    }

    /// Returns a conservative exact box for the complete selected half circle.
    ///
    /// A center certified in the source's unit domain can reuse its box,
    /// expanded by `|parallel distance| + |radius|`. Extended fillets may
    /// retain a center outside that domain; those use the selected center's
    /// own enclosure instead.
    pub(crate) fn conservative_bounds(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Aabb2>> {
        if let Some(parallel) = self.source_parallel()
            && let Some(parameter) = self.selected_frame_parameter()
            && let Some((lower, upper)) = parameter.finite_envelope_bounds()
            && in_closed_unit_interval(lower, &CurveContext::STRICT) == Some(true)
            && in_closed_unit_interval(upper, &CurveContext::STRICT) == Some(true)
            && let Classification::Decided(source) = parallel.source().certified_bounds()
        {
            let expansion = parallel.distance().abs() + self.data.radial_distance.abs();
            return Ok(Classification::Decided(Aabb2::new_unchecked(
                Point2::new(source.min_x() - &expansion, source.min_y() - &expansion),
                Point2::new(source.max_x() + &expansion, source.max_y() + expansion),
            )));
        }
        self.conservative_bounds_refined(0, policy)
    }

    /// Returns a progressively tighter exact box around this selected half circle.
    ///
    /// The ordinary broad phase deliberately expands the complete source-curve
    /// box because that is almost free.  A surviving candidate can instead
    /// refine the one selected center field and expand only that point by the
    /// exact radius.  Every returned box remains conservative; refinement is
    /// therefore useful for proving separation but never manufactures a
    /// contact or equality decision.
    pub(crate) fn conservative_bounds_refined(
        &self,
        refinement_steps: usize,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Aabb2>> {
        let center = match self.center_point_evidence(policy)? {
            Classification::Decided(center) => center,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let center =
            match algebraic_chord_endpoint_bounds_refined(&center, refinement_steps, policy) {
                Classification::Decided(center) => center,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let radius = self.data.radial_distance.abs();
        Ok(Classification::Decided(Aabb2::new_unchecked(
            Point2::new(center.min_x() - &radius, center.min_y() - &radius),
            Point2::new(center.max_x() + &radius, center.max_y() + radius),
        )))
    }

    /// Signs one retained point's squared-distance residual against this
    /// supporting circle. Construction identities and short exact enclosure
    /// checks precede replay in the least shared retained field, which can
    /// prove incidence even when independent Cartesian bounds cannot.
    pub(crate) fn retained_point_incidence_sign(
        &self,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        if let CurvePoint2(CurvePointData2::AlgebraicCuspChord(point)) = point {
            let (map, _) = point.map_contact();
            map.validate_policy(policy)?;
            if self.shares_structural_supporting_circle(&map.data.semicircle) {
                return Ok(Classification::Decided(RealSign::Zero));
            }
        }
        if let CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(point)) = point {
            match point.concentric_circle_incidence_sign(self, policy)? {
                Classification::Decided(Some(sign)) => {
                    return Ok(Classification::Decided(sign));
                }
                Classification::Decided(None) => {}
                Classification::Uncertain(_) => {}
            }
        }
        if let CurvePoint2(CurvePointData2::AlgebraicChordParallel(point)) = point
            && let Some(sign) = point.concentric_circle_incidence_sign(self, policy)?
        {
            return Ok(sign);
        }
        if matches!(
            point,
            CurvePoint2(CurvePointData2::AlgebraicChordParallel(_))
        ) {
            // The chord/parallel point's own recursive field can replay the
            // circle residual without materializing a coordinate frame.
            if let Classification::Decided(center) = self.center_point_evidence(policy)?
                && let Classification::Decided(Some(sign)) =
                    recursive_projective_point_evidence_circle_residual_sign(
                        point,
                        &center,
                        &(self.radial_distance() * self.radial_distance()),
                        policy,
                    )?
            {
                return Ok(Classification::Decided(sign));
            }
            match self.represented_point_incidence_sign(point, policy, policy)? {
                decided @ Classification::Decided(_) => return Ok(decided),
                Classification::Uncertain(_) => {}
            }
        }
        let refined = self.retained_point_incidence_sign_by_refinement(point, policy, 4, false)?;
        if matches!(refined, Classification::Decided(_)) {
            return Ok(refined);
        }
        if let Classification::Decided(center) = self.center_point_evidence(policy)?
            && let Classification::Decided(Some(sign)) =
                recursive_projective_point_evidence_circle_residual_sign(
                    point,
                    &center,
                    &(self.radial_distance() * self.radial_distance()),
                    policy,
                )?
        {
            return Ok(Classification::Decided(sign));
        }
        self.retained_point_incidence_sign_by_refinement(point, policy, 512, true)
    }

    /// Inverts a certified point on this complete supporting circle. The
    /// returned parameter retains which of the two half charts owns the point.
    /// A radial chord has exactly one finite circle contact, at its supplied
    /// endpoint; reuse that incidence instead of selecting another root.
    pub(crate) fn parameter_at_certified_incident_point(
        &self,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<CurveParameter2>> {
        let halves = [self.clone(), self.complementary_half()];
        let publish = |parameter: BezierAlgebraicCuspSemicircleParameter2, complementary| {
            if complementary {
                // Both closed half charts contain the diameter endpoints.
                // Canonical ownership keeps a remote authored endpoint from
                // being mistaken for an admissible complementary extension.
                if let Ok(Classification::Decided(
                    BezierAlgebraicCuspSemicircleParameterBracket2::Exact(value),
                )) = policy.strict_predicate_pass(|| parameter.parameter_bracket(0, policy))
                {
                    if value.zero_status() == ZeroKnowledge::Zero {
                        return CurveParameter2::from_algebraic_cusp(
                            BezierAlgebraicCuspSemicircleParameter2::Exact(Real::one()),
                        );
                    }
                    if (value - Real::one()).zero_status() == ZeroKnowledge::Zero {
                        return CurveParameter2::from_algebraic_cusp(
                            BezierAlgebraicCuspSemicircleParameter2::Exact(Real::zero()),
                        );
                    }
                }
                CurveParameter2::from_algebraic_cusp_complement(parameter)
            } else {
                CurveParameter2::from_algebraic_cusp(parameter)
            }
        };
        if let CurvePoint2(CurvePointData2::Endpoint(endpoint)) = point
            && let crate::BezierSplitFragment2::AlgebraicCuspSemicircle(source) =
                endpoint.fragment.as_ref()
        {
            for (index, half) in halves.iter().enumerate() {
                let fragment = BezierAlgebraicCuspSemicircleFragment2::full(half.clone(), policy);
                if let Classification::Decided(Some(parameter)) =
                    policy.strict_predicate_pass(|| {
                        fragment.parameter_of_shared_circle_endpoint(source, endpoint.start, policy)
                    })?
                {
                    return Ok(Classification::Decided(publish(parameter, index != 0)));
                }
            }
        }
        if let CurvePoint2(CurvePointData2::AlgebraicCuspChord(point)) = point {
            let parameter = BezierAlgebraicCuspSemicircleParameter2::Mapped(point.data.clone());
            if let Classification::Decided(Some(complementary)) =
                policy.strict_predicate_pass(|| {
                    self.shared_frame_chart_relation(point.data.semicircle_carrier(), policy)
                })
            {
                return Ok(Classification::Decided(publish(parameter, complementary)));
            }
        }
        if let CurvePoint2(CurvePointData2::AnalyticParallel(point)) = point
            && let Some(frame) = self.data.frame.parallel_normal()
            && policy.accepts_retained_policy(point.data.policy)
            && policy.accepts_retained_policy(frame.policy)
            && point.data.frame_tangent.is_none()
            && point.data.parallel.source() == frame.center_support.source()
            && point
                .data
                .parameter
                .matches_region_parameter(&frame.center_parameter)
            && point.data.translation_x.zero_status() == ZeroKnowledge::Zero
            && point.data.translation_y.zero_status() == ZeroKnowledge::Zero
        {
            // In the retained normal/tangent frame, a circle point already
            // has exact scalar displacements n and b. Inverting the rational
            // half chart gives u=b/(r+n+b); no selected Cartesian coordinate
            // or additional root is needed. The diameter endpoints are the
            // only zero-b cases and keep the base chart's ownership.
            let normal = point.data.parallel.distance() - frame.center_support.distance();
            let tangent = -&point.data.tangent_distance * self.turn_sign();
            let parameter =
                match real_sign(&(&tangent * self.radial_distance()), &CurveContext::STRICT) {
                    Some(RealSign::Zero) => {
                        match real_sign(&(&normal * self.radial_distance()), &CurveContext::STRICT)
                        {
                            Some(RealSign::Positive) => Some((Real::zero(), false)),
                            Some(RealSign::Negative) => Some((Real::one(), false)),
                            _ => None,
                        }
                    }
                    Some(sign) => {
                        let complementary = sign == RealSign::Negative;
                        let radius = if complementary {
                            -self.radial_distance().clone()
                        } else {
                            self.radial_distance().clone()
                        };
                        Some(((&tangent / (radius + normal + &tangent))?, complementary))
                    }
                    None => None,
                };
            if let Some((parameter, complementary)) = parameter {
                return Ok(Classification::Decided(publish(
                    BezierAlgebraicCuspSemicircleParameter2::Exact(parameter),
                    complementary,
                )));
            }
        }
        let center = match self.center_point_evidence(policy)? {
            Classification::Decided(center) => center,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let radial = match BezierAlgebraicChord2::try_new_from_certified_distinct_endpoints(
            center,
            point.clone(),
            policy,
        )? {
            Classification::Decided(radial) => radial,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        for (index, half) in halves.iter().enumerate() {
            let contacts = match half
                .chord_intersections_with_certified_endpoint_incidence(&radial, false, policy)?
            {
                Classification::Decided(contacts) => contacts,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            if let Some(contact) = contacts
                .into_iter()
                .find(|contact| contact.chord_parameter.is_endpoint_of(&radial, false))
            {
                return Ok(Classification::Decided(publish(
                    contact.cusp_parameter,
                    index != 0,
                )));
            }
        }
        Err(CurveError::Topology(
            "a certified circle point was absent from both half charts".into(),
        ))
    }

    /// Signs one procedural displacement point without materializing its unit
    /// direction or Cartesian coordinates.
    ///
    /// With source direction `D`, positive speed `s = sqrt(D dot D)`, source
    /// radial vector `V`, displacement distance `d`, and either `W = D` or
    /// `W = left_normal(D)`, multiplying the circle residual by positive
    /// `s^2` leaves the one-radical expression
    ///
    /// `s^2 (V dot V + d^2 - r^2) + s (2 d V dot W)`.
    ///
    /// This keeps a correlated trim endpoint out of a standalone primitive
    /// field and lets the retained root support supply the exact direction.
    pub(super) fn represented_point_incidence_sign(
        &self,
        point: &CurvePoint2,
        construction_policy: &CurveContext,
        predicate_policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let CurvePoint2(CurvePointData2::AlgebraicChordParallel(parallel)) = point else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        if !parallel.accepts_policy(construction_policy) {
            return Err(CurveError::Topology(
                "a displaced point entered circle incidence under a different policy".into(),
            ));
        }
        let origin = match represented_point_evidence_coordinates(
            parallel.source_endpoint(),
            construction_policy,
        )? {
            Classification::Decided(origin) => origin,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let source_support = parallel.data.source.retained_support();
        let support_start = match represented_point_evidence_coordinates(
            source_support.start(),
            construction_policy,
        )? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let support_end = match represented_point_evidence_coordinates(
            source_support.end(),
            construction_policy,
        )? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let center = match self.represented_circle_frame(construction_policy)? {
            Classification::Decided(frame) => frame.center,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let represented = [
            origin[0].clone(),
            origin[1].clone(),
            support_start[0].clone(),
            support_start[1].clone(),
            support_end[0].clone(),
            support_end[1].clone(),
            center[0].clone(),
            center[1].clone(),
        ];
        let Some((mut sources, coordinates)) = represented_affine_tensor_basis(&represented) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let [
            origin_x,
            origin_y,
            support_start_x,
            support_start_y,
            support_end_x,
            support_end_y,
            center_x,
            center_y,
        ]: [DenseTensorPolynomial; 8] = coordinates
            .try_into()
            .expect("a represented displacement incidence retains eight coordinates");
        // `represented_affine_tensor_basis` reserves its final singleton axis
        // for an output. Treat it as one exact zero source so the tuple-sign
        // kernel can reduce every axis uniformly without reallocating the
        // coordinate tensors.
        sources.push(AlgebraicRootRepresentation::from_exact_value(&Real::zero()));
        let reduce = |polynomial| dense_reduce_selected_tuple_relations(polynomial, &sources);
        let constant = |value: &Real| {
            DenseTensorPolynomial::from_axis_polynomial(
                sources.len(),
                0,
                std::slice::from_ref(value),
            )
        };
        let Some((q, rational, radical)) = (|| {
            let mut dx = reduce(support_end_x.subtract(&support_start_x)?)?;
            let mut dy = reduce(support_end_y.subtract(&support_start_y)?)?;
            if parallel
                .data
                .source
                .retained_support_orientation_is_reversed()
            {
                dx = reduce(dx.scale(&Real::from(-1_i8))?)?;
                dy = reduce(dy.scale(&Real::from(-1_i8))?)?;
            }
            let q = reduce(dx.multiply(&dx)?.add(&dy.multiply(&dy)?)?)?;
            let vx = reduce(
                origin_x
                    .subtract(&center_x)?
                    .add(&constant(&parallel.data.translation_x)?)?,
            )?;
            let vy = reduce(
                origin_y
                    .subtract(&center_y)?
                    .add(&constant(&parallel.data.translation_y)?)?,
            )?;
            let radial_offset = &parallel.data.distance * &parallel.data.distance
                - self.radial_distance() * self.radial_distance();
            let radial_squared = reduce(
                vx.multiply(&vx)?
                    .add(&vy.multiply(&vy)?)?
                    .add(&constant(&radial_offset)?)?,
            )?;
            let rational = reduce(q.multiply(&radial_squared)?)?;
            let (wx, wy) = match parallel.data.direction {
                BezierAlgebraicChordUnitDisplacement2::LeftNormal => {
                    (dy.scale(&Real::from(-1_i8))?, dx)
                }
                BezierAlgebraicChordUnitDisplacement2::Tangent => (dx, dy),
            };
            let radical = reduce(
                vx.multiply(&wx)?
                    .add(&vy.multiply(&wy)?)?
                    .scale(&(Real::from(2_i8) * &parallel.data.distance))?,
            )?;
            Some((q, rational, radical))
        })() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        match dense_polynomial_tuple_sign(&q, &sources, &construction_policy.strict_counterpart())?
        {
            Classification::Decided(RealSign::Positive) => {}
            Classification::Decided(RealSign::Zero) => {
                return Err(CurveError::Topology(
                    "a displaced point retained a collapsed source support".into(),
                ));
            }
            Classification::Decided(RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "a displaced point retained negative source speed squared".into(),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let incidence = dense_positive_square_root_sum_sign(
            &rational,
            &radical,
            &q,
            &sources,
            predicate_policy,
        )?;
        Ok(incidence)
    }

    pub(super) fn retained_point_incidence_sign_by_refinement(
        &self,
        point: &CurvePoint2,
        construction_policy: &CurveContext,
        maximum_refinement_steps: usize,
        permit_terminal_approximation: bool,
    ) -> CurveResult<Classification<RealSign>> {
        let center = match self.center_point_evidence(construction_policy)? {
            Classification::Decided(center) => center,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let radius_squared = self.radial_distance() * self.radial_distance();
        let radius_squared = RealInterval {
            lower: radius_squared.clone(),
            upper: radius_squared,
        };
        let mut terminal_refined = false;
        for refinement_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
            if refinement_steps > maximum_refinement_steps {
                break;
            }
            let (Classification::Decided(point), Classification::Decided(center)) = (
                algebraic_chord_endpoint_bounds_refined(
                    point,
                    refinement_steps,
                    construction_policy,
                ),
                algebraic_chord_endpoint_bounds_refined(
                    &center,
                    refinement_steps,
                    construction_policy,
                ),
            ) else {
                continue;
            };
            terminal_refined |= refinement_steps == 512;
            let delta = |axis| {
                real_interval_from_axis(&point, axis)
                    .subtract(&real_interval_from_axis(&center, axis))
            };
            let delta_x = delta(Axis2::X);
            let delta_y = delta(Axis2::Y);
            let Some(residual) = delta_x.square().and_then(|x| {
                delta_y
                    .square()
                    .map(|y| x.add(&y).subtract(&radius_squared))
            }) else {
                continue;
            };
            let zero = Real::zero();
            if compare_reals(&residual.lower, &zero, &CurveContext::STRICT)
                == Some(std::cmp::Ordering::Greater)
            {
                return Ok(Classification::Decided(RealSign::Positive));
            }
            if compare_reals(&residual.upper, &zero, &CurveContext::STRICT)
                == Some(std::cmp::Ordering::Less)
            {
                return Ok(Classification::Decided(RealSign::Negative));
            }
            if compare_reals(&residual.lower, &zero, &CurveContext::STRICT)
                == Some(std::cmp::Ordering::Equal)
                && compare_reals(&residual.upper, &zero, &CurveContext::STRICT)
                    == Some(std::cmp::Ordering::Equal)
            {
                return Ok(Classification::Decided(RealSign::Zero));
            }
        }
        if permit_terminal_approximation
            && terminal_refined
            && construction_policy.permits_approximate_512()
        {
            construction_policy.observe_approximate_512();
            Ok(Classification::Decided(RealSign::Zero))
        } else {
            Ok(Classification::Uncertain(UncertaintyReason::Ordering))
        }
    }

    /// Returns only a reusable STRICT incidence sign for an optional
    /// broad-phase certificate.
    ///
    /// `construction_policy` validates the point's carrier identity, while the
    /// exact radial residual is tried first, followed by bounded interval
    /// refinement with STRICT residual comparisons. An unresolved interval
    /// remains uncertain even when the carrier was constructed under
    /// APPROXIMATE_512; any approximation consumed while recovering its
    /// conservative endpoint bounds remains observed by the outer operation.
    pub(crate) fn strict_point_incidence_sign(
        &self,
        point: &CurvePoint2,
        construction_policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        if let CurvePoint2(CurvePointData2::AlgebraicChordParallel(parallel)) = point
            && let Some(sign) =
                parallel.concentric_circle_incidence_sign(self, construction_policy)?
        {
            return Ok(sign);
        }
        if matches!(
            point,
            CurvePoint2(CurvePointData2::AlgebraicChordParallel(_))
        ) {
            let refined = self.retained_point_incidence_sign_by_refinement(
                point,
                construction_policy,
                4,
                false,
            )?;
            if matches!(refined, Classification::Decided(_)) {
                return Ok(refined);
            }
            return self.represented_point_incidence_sign(
                point,
                construction_policy,
                &construction_policy.strict_counterpart(),
            );
        }
        let CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(derived)) = point else {
            return self.retained_point_incidence_sign_by_refinement(
                point,
                construction_policy,
                4,
                false,
            );
        };
        derived.data.source.validate_policy(construction_policy)?;
        if let Some(sign) = derived.equal_radius_pair_other_circle_incidence_sign(self)? {
            return Ok(Classification::Decided(sign));
        }
        if let Some(sign) = derived
            .unshifted_concentric_circle_incidence_residual(self)
            .and_then(|residual| real_sign(&residual, &CurveContext::STRICT))
        {
            return Ok(Classification::Decided(sign));
        }
        self.retained_point_incidence_sign_by_refinement(point, construction_policy, 4, false)
    }

    /// Proves that the complete supporting circle misses an exact outer box.
    ///
    /// A strict coordinate separation against `center +/- |radius|` excludes
    /// every point of the circle and therefore every retained subarc. The
    /// selected center stays in its existing field; no resultant or coordinate
    /// algebraic number is constructed.
    pub(crate) fn certifiably_disjoint_from_bounds(
        &self,
        bounds: &Aabb2,
        policy: &CurveContext,
    ) -> CurveResult<bool> {
        let radius = self.data.radial_distance.abs();
        if self.data.frame.rational().is_some() {
            let center = self.center_point_image(policy)?;
            for (use_x, lower, upper) in [
                (true, bounds.min().x(), bounds.max().x()),
                (false, bounds.min().y(), bounds.max().y()),
            ] {
                if center.coordinate_order_to_real(use_x, &(lower - &radius), policy)?
                    == Classification::Decided(std::cmp::Ordering::Less)
                    || center.coordinate_order_to_real(use_x, &(upper + &radius), policy)?
                        == Classification::Decided(std::cmp::Ordering::Greater)
                {
                    return Ok(true);
                }
            }
            return Ok(false);
        }
        {
            let center = match self.center_point_evidence(policy)? {
                Classification::Decided(center) => center,
                Classification::Uncertain(_) => return Ok(false),
            };
            for (use_x, lower, upper) in [
                (true, bounds.min().x(), bounds.max().x()),
                (false, bounds.min().y(), bounds.max().y()),
            ] {
                let axis = if use_x { Axis2::X } else { Axis2::Y };
                if BezierAlgebraicChord2::point_axis_order_to_real(
                    &center,
                    axis,
                    &(lower - &radius),
                    policy,
                )? == Classification::Decided(std::cmp::Ordering::Less)
                    || BezierAlgebraicChord2::point_axis_order_to_real(
                        &center,
                        axis,
                        &(upper + &radius),
                        policy,
                    )? == Classification::Decided(std::cmp::Ordering::Greater)
                {
                    return Ok(true);
                }
            }
        }
        Ok(false)
    }

    pub(super) fn forward_ray_rational_contacts(
        &self,
        origin: &Point2,
        direction_x: &Real,
        direction_y: &Real,
        retain_parameter_map: bool,
        policy: &CurveContext,
    ) -> CurveResult<
        Classification<(
            RationalBezier2,
            BezierAlgebraicCuspSemicircleRationalIntersections2,
            Option<BezierAlgebraicCuspSemicircleRationalParameterMap2>,
        )>,
    > {
        let direction_squared = direction_x * direction_x + direction_y * direction_y;
        match real_sign(&direction_squared, policy) {
            Some(RealSign::Positive) => {}
            Some(RealSign::Zero) => return Err(CurveError::ZeroLengthLine),
            Some(RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "ray direction had a negative squared norm".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let bounds = match self.conservative_circle_cover_bounds(policy)? {
            Classification::Decided(bounds) => bounds,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        // `reach` dominates the absolute delta on either axis for every point
        // in the box. Hence
        //   |dot(Q-origin,d)| <= reach * (|dx|+|dy|).
        // Dividing that bound by |d|^2 and adding one puts every positive line
        // parameter strictly before the constructed endpoint.
        let reach = (bounds.min_x() - origin.x()).abs()
            + (bounds.max_x() - origin.x()).abs()
            + (bounds.min_y() - origin.y()).abs()
            + (bounds.max_y() - origin.y()).abs()
            + Real::one();
        let direction_l1 = direction_x.abs() + direction_y.abs();
        let scale = Real::one() + (reach * direction_l1 / direction_squared)?;
        let endpoint = Point2::new(
            origin.x() + &scale * direction_x,
            origin.y() + &scale * direction_y,
        );
        let ray = RationalBezier2::try_new(
            vec![origin.clone(), endpoint],
            vec![Real::one(), Real::one()],
        )?;
        let (intersections, parameter_map) = match self.rational_intersections_internal(
            &ray,
            &crate::CurveParameterRange2::unit(),
            retain_parameter_map,
            policy,
        )? {
            Classification::Decided(result) => result,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(Classification::Decided((ray, intersections, parameter_map)))
    }

    pub(super) fn selected_radial_frame_source<'a>(
        &'a self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierSelectedRadialCircleFrameSource2<'a>>> {
        let frame = self.data.frame.selected_radial().ok_or_else(|| {
            CurveError::Topology(
                "a non-radial selected circle entered the pair-radial frame system".into(),
            )
        })?;
        if !policy.accepts_retained_policy(frame.policy) {
            return Err(CurveError::Topology(
                "a pair-radial selected circle crossed predicate policies".into(),
            ));
        }
        let mut center_parameter = frame.center_parameter.as_ref();
        let mut similarity_transports = Vec::new();
        let (pair_map, pair_contact, support_first) = loop {
            match center_parameter {
                BezierAlgebraicCuspSemicircleMappedParameterData2::Pair {
                    map,
                    contact,
                    first,
                } => break (map, contact, *first),
                BezierAlgebraicCuspSemicircleMappedParameterData2::SimilarityTransport {
                    source,
                    point,
                    policy: transport_policy,
                    ..
                } => {
                    if !policy.accepts_retained_policy(*transport_policy) {
                        return Err(CurveError::Topology(
                            "a pair-radial similarity crossed predicate policies".into(),
                        ));
                    }
                    let BezierAlgebraicCuspSemicircleParameter2::Mapped(source) = source else {
                        return Err(CurveError::Topology(
                            "a pair-radial similarity lost its mapped source parameter".into(),
                        ));
                    };
                    let CurvePoint2(CurvePointData2::Similarity(point)) = point else {
                        return Err(CurveError::Topology(
                            "a pair-radial similarity lost its exact transform provenance".into(),
                        ));
                    };
                    if !policy.accepts_retained_policy(point.data.policy) {
                        return Err(CurveError::Topology(
                            "a pair-radial similarity point crossed predicate policies".into(),
                        ));
                    }
                    similarity_transports.push(&point.data.transform);
                    center_parameter = source.as_ref();
                }
                _ => return Ok(Classification::Uncertain(UncertaintyReason::Unsupported)),
            }
        };
        if !policy.accepts_retained_policy(pair_map.data.policy)
            || !(-1..=1).contains(&pair_contact.branch)
        {
            return Err(CurveError::Topology(
                "a pair-radial frame lost its authored pair policy or branch".into(),
            ));
        }
        let expected_support = if support_first {
            &pair_map.data.first_semicircle
        } else {
            &pair_map.data.second_semicircle
        };
        if expected_support != center_parameter.semicircle_carrier() {
            return Err(CurveError::Topology(
                "a pair-radial frame lost its source-circle identity".into(),
            ));
        }
        let mut similarity: Option<Similarity2> = None;
        for transform in similarity_transports.into_iter().rev() {
            similarity = Some(match similarity {
                Some(current) => current.then(transform),
                None => transform.clone(),
            });
        }
        Ok(Classification::Decided(
            BezierSelectedRadialCircleFrameSource2 {
                frame,
                pair_map,
                pair_contact,
                support_first,
                similarity,
            },
        ))
    }

    /// Resolves the compact pair-field frame behind a selected-radial circle,
    /// including exact similarity transport. Recursively authored source maps
    /// deliberately fall through to the represented frame authority.
    pub(super) fn selected_radial_frame_system(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierSelectedRadialCircleFrameSystem2>> {
        let BezierSelectedRadialCircleFrameSource2 {
            frame,
            pair_map,
            pair_contact,
            support_first,
            similarity: combined_similarity,
        } = match self.selected_radial_frame_source(policy)? {
            Classification::Decided(source) => source,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let (Some(first_source_frame), Some(second_source_frame)) = (
            pair_map.data.first_semicircle.data.frame.rational(),
            pair_map.data.second_semicircle.data.frame.rational(),
        ) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let first_cusp_parameter =
            BezierParameter2::Algebraic(first_source_frame.data.parameter.clone());
        let second_cusp_parameter =
            BezierParameter2::Algebraic(second_source_frame.data.parameter.clone());
        let mut first_frame = match pair_map
            .data
            .first_semicircle
            .normalized_circle_frame(policy)?
        {
            Classification::Decided(frame) => frame,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let mut second_frame = match pair_map
            .data
            .second_semicircle
            .normalized_circle_frame(policy)?
        {
            Classification::Decided(frame) => frame,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if first_frame.cusp_parameter != first_cusp_parameter
            || second_frame.cusp_parameter != second_cusp_parameter
        {
            return Err(CurveError::Topology(
                "a pair-radial frame changed its selected root authority".into(),
            ));
        }

        let mut branch = pair_contact.branch;
        let pair_scale_squared = if let Some(transform) = &combined_similarity {
            first_frame.transform_center_similarity(transform);
            second_frame.transform_center_similarity(transform);
            if transform.reverses_orientation() {
                branch = -branch;
            }
            transform.scale() * transform.scale()
        } else {
            Real::one()
        };
        let first_radius_squared = pair_map.data.first_semicircle.radial_distance()
            * pair_map.data.first_semicircle.radial_distance();
        let first_radius_squared = first_radius_squared * &pair_scale_squared;
        let second_radius_squared = pair_map.data.second_semicircle.radial_distance()
            * pair_map.data.second_semicircle.radial_distance();
        let second_radius_squared = second_radius_squared * pair_scale_squared;
        let pair_map = pair_map.clone();
        let normal_denominator = frame.normal_denominator.clone();
        let system = (|| {
            let axis = |coefficients: &[Real], axis| {
                TrivariatePolynomial::from_axis_polynomial(coefficients, axis)
            };
            let ax = axis(&first_frame.center_x, 0)?;
            let ay = axis(&first_frame.center_y, 0)?;
            let aw = axis(&first_frame.denominator, 0)?;
            let bx = axis(&second_frame.center_x, 1)?;
            let by = axis(&second_frame.center_y, 1)?;
            let bw = axis(&second_frame.denominator, 1)?;

            // With positive selected denominators, delta=(dx,dy)/D is C2-C1.
            let dx = TrivariatePolynomial::sum_products(&[(&bx, &aw, false), (&ax, &bw, true)])?;
            let dy = TrivariatePolynomial::sum_products(&[(&by, &aw, false), (&ay, &bw, true)])?;
            let common_denominator = aw.multiply(&bw)?;
            let common_denominator_squared = common_denominator.multiply(&common_denominator)?;
            let center_distance_squared =
                TrivariatePolynomial::sum_products(&[(&dx, &dx, false), (&dy, &dy, false)])?;
            let center_line = center_distance_squared.add(
                &common_denominator_squared
                    .scale(&(&first_radius_squared - &second_radius_squared))?,
            )?;
            let twice_center_distance = center_distance_squared.scale(&Real::from(2_i8))?;
            let discriminant = center_distance_squared
                .multiply(&common_denominator_squared)?
                .scale(&(Real::from(4_i8) * &first_radius_squared))?
                .subtract(&center_line.multiply(&center_line)?)?;
            let denominator = twice_center_distance.multiply(&common_denominator)?;

            // P=C1+R1, where R1=(L*delta + branch*sqrt(K)*J(delta))/(2*q*D).
            let first_center_x = ax.multiply(&bw)?;
            let first_center_y = ay.multiply(&bw)?;
            let center_x_rational = TrivariatePolynomial::sum_products(&[
                (&twice_center_distance, &first_center_x, false),
                (&center_line, &dx, false),
            ])?;
            let center_y_rational = TrivariatePolynomial::sum_products(&[
                (&twice_center_distance, &first_center_y, false),
                (&center_line, &dy, false),
            ])?;
            let center_x_radical = dy.scale(&Real::from(-1_i8))?;
            let center_y_radical = dx.clone();

            // The selected radial is R1 on the first support and R1-delta on
            // the second. Both use the same authored square-root sheet.
            let support_line = if support_first {
                center_line
            } else {
                center_line.subtract(&twice_center_distance)?
            };
            let radial_x_rational = support_line.multiply(&dx)?;
            let radial_y_rational = support_line.multiply(&dy)?;

            let reduce = |polynomial: TrivariatePolynomial| {
                trivariate_reduce_parameter_pair_relations(
                    &polynomial,
                    &first_cusp_parameter,
                    &second_cusp_parameter,
                )
                .unwrap_or(polynomial)
            };
            Some(BezierSelectedRadialCircleFrameSystem2 {
                canonical_pair_field: combined_similarity.is_none(),
                branch,
                discriminant: reduce(discriminant),
                denominator: reduce(denominator),
                center_x: SquareRootExpression {
                    rational: reduce(center_x_rational),
                    radical: reduce(center_x_radical.clone()),
                },
                center_y: SquareRootExpression {
                    rational: reduce(center_y_rational),
                    radical: reduce(center_y_radical.clone()),
                },
                radial_x: SquareRootExpression {
                    rational: reduce(radial_x_rational),
                    radical: reduce(center_x_radical),
                },
                radial_y: SquareRootExpression {
                    rational: reduce(radial_y_rational),
                    radical: reduce(center_y_radical),
                },
                normal_denominator,
                pair_map,
            })
        })();
        Ok(system.map_or(
            Classification::Uncertain(UncertaintyReason::Unsupported),
            Classification::Decided,
        ))
    }
}

impl BezierAlgebraicCuspNormalizedCircleFrame2 {
    /// Moves only this frame's homogeneous center into a similarity image.
    ///
    /// Pair-radial incidence uses the two selected support centers but not
    /// their local unit-normal frames.  Keeping the original positive
    /// denominator and transforming its two numerators is therefore the
    /// smallest exact covariant representation of a transported pair.
    pub(super) fn transform_center_similarity(&mut self, transform: &Similarity2) {
        let zero = Real::zero();
        let length = self
            .center_x
            .len()
            .max(self.center_y.len())
            .max(self.denominator.len());
        let mut center_x = Vec::with_capacity(length);
        let mut center_y = Vec::with_capacity(length);
        for index in 0..length {
            let (x, y) = transform.transform_homogeneous_coordinates(
                self.center_x.get(index).unwrap_or(&zero),
                self.center_y.get(index).unwrap_or(&zero),
                self.denominator.get(index).unwrap_or(&zero),
            );
            center_x.push(x);
            center_y.push(y);
        }
        self.center_x = polynomial_trim_structural_zeros(center_x);
        self.center_y = polynomial_trim_structural_zeros(center_y);
    }
}

impl BezierAlgebraicCuspRetainedOffsetChordNestedExpression2 {
    pub(super) fn from_retained(
        retained: SquareRootExpression<TrivariatePolynomial>,
    ) -> Option<Self> {
        Some(Self {
            retained,
            candidate: SquareRootExpression::from_rational(
                TrivariatePolynomial::from_axis_polynomial(&[Real::zero()], 0)?,
            )?,
        })
    }

    pub(super) fn from_rational(rational: TrivariatePolynomial) -> Option<Self> {
        Self::from_retained(SquareRootExpression::from_rational(rational)?)
    }

    pub(super) fn add(&self, other: &Self) -> Option<Self> {
        Some(Self {
            retained: self.retained.add(&other.retained)?,
            candidate: self.candidate.add(&other.candidate)?,
        })
    }

    pub(super) fn subtract(&self, other: &Self) -> Option<Self> {
        Some(Self {
            retained: self.retained.subtract(&other.retained)?,
            candidate: self.candidate.subtract(&other.candidate)?,
        })
    }

    pub(super) fn scale(&self, scale: &Real) -> Option<Self> {
        Some(Self {
            retained: self.retained.scale(scale)?,
            candidate: self.candidate.scale(scale)?,
        })
    }

    pub(super) fn multiply_rational(&self, polynomial: &TrivariatePolynomial) -> Option<Self> {
        Some(Self {
            retained: self.retained.multiply_rational(polynomial)?,
            candidate: self.candidate.multiply_rational(polynomial)?,
        })
    }

    pub(super) fn linear_combination(terms: &[(&Self, &Real)]) -> Option<Self> {
        let retained = terms
            .iter()
            .map(|(expression, scale)| (&expression.retained, *scale))
            .collect::<Vec<_>>();
        let candidate = terms
            .iter()
            .map(|(expression, scale)| (&expression.candidate, *scale))
            .collect::<Vec<_>>();
        Some(Self {
            retained: SquareRootExpression::linear_combination(&retained)?,
            candidate: SquareRootExpression::linear_combination(&candidate)?,
        })
    }
}

impl BezierAlgebraicCuspSemicircleAlgebraicRay2 {
    pub(crate) fn endpoint_side_signs(
        &self,
        point: &RationalBezierAlgebraicPointPredicate2<'_>,
        side_x: &Real,
        side_y: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<[RealSign; 2]>> {
        let mut signs = [RealSign::Zero; 2];
        for (index, endpoint) in [&self.start, &self.end].into_iter().enumerate() {
            let endpoint = match endpoint.predicate_evaluator(policy)? {
                Classification::Decided(endpoint) => endpoint,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            signs[index] = match signed_algebraic_point_linear_difference(
                &endpoint, point, side_x, side_y, policy,
            )? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        }
        Ok(Classification::Decided(signs))
    }

    pub(crate) fn contains_point(
        &self,
        point: &RationalBezierAlgebraicPointPredicate2<'_>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        let start = match self.start.predicate_evaluator(policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let end = match self.end.predicate_evaluator(policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let center = match self.center.predicate_evaluator(policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        match algebraic_point_circle_residual_sign(&center, point, &self.radius_squared, policy)? {
            Classification::Decided(RealSign::Positive | RealSign::Negative) => {
                return Ok(Classification::Decided(false));
            }
            Classification::Decided(RealSign::Zero) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        Ok(
            algebraic_point_oriented_line_side(&start, &end, point, policy)?.map(|side| {
                if self.clockwise {
                    side != crate::classify::LineSide::Right
                } else {
                    side != crate::classify::LineSide::Left
                }
            }),
        )
    }

    /// Omits this finite arc's transverse contact at an algebraic side-ray
    /// origin while preserving any second forward circle contact.
    pub(crate) fn forward_ray_winding_delta_skipping_incident_origin(
        &self,
        point: &RationalBezierAlgebraicPointPredicate2<'_>,
        direction_x: &Real,
        direction_y: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<i32>>> {
        match self.contains_point(point, policy)? {
            Classification::Decided(true) => {}
            Classification::Decided(false) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let center = match self.center.predicate_evaluator(policy)? {
            Classification::Decided(center) => center,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let radial_sign = match signed_algebraic_point_linear_difference(
            point,
            &center,
            direction_x,
            direction_y,
            policy,
        )? {
            Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
            Classification::Decided(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        // The ordinary minor-arc predicate evaluates a point on the circle
        // from its outside limit.  An inward ray therefore includes the
        // origin crossing; an outward ray does not.  Remove exactly that
        // signed crossing while retaining a second forward contact when it
        // lies on this same finite semicircle.
        let skipped_origin_delta = if radial_sign == RealSign::Negative {
            if self.clockwise { 1 } else { -1 }
        } else {
            0
        };
        self.forward_ray_winding_delta(point, direction_x, direction_y, true, policy)
            .map(|delta| delta.map(|delta| Some(delta - skipped_origin_delta)))
    }

    pub(crate) fn forward_ray_winding_delta(
        &self,
        point: &RationalBezierAlgebraicPointPredicate2<'_>,
        direction_x: &Real,
        direction_y: &Real,
        point_on_supporting_circle: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<i32>> {
        let start = match self.start.predicate_evaluator(policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let end = match self.end.predicate_evaluator(policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let side_x = -direction_y.clone();
        let side_y = direction_x.clone();
        let order = |first: &RationalBezierAlgebraicPointPredicate2<'_>,
                     second: &RationalBezierAlgebraicPointPredicate2<'_>,
                     x_factor: &Real,
                     y_factor: &Real| {
            algebraic_point_linear_order(first, second, x_factor, y_factor, policy)
        };
        let start_y = match order(&start, point, &side_x, &side_y)? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let end_y = match order(&end, point, &side_x, &side_y)? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let line_side = match algebraic_point_oriented_line_side(&start, &end, point, policy)? {
            Classification::Decided(side) => side,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let inside_circle = if point_on_supporting_circle {
            false
        } else {
            let center = match self.center.predicate_evaluator(policy)? {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            match algebraic_point_circle_residual_sign(
                &center,
                point,
                &self.radius_squared,
                policy,
            )? {
                Classification::Decided(sign) => sign == RealSign::Negative,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        };
        let is_ccw = !self.clockwise;
        let point_is_left = if is_ccw {
            line_side == crate::classify::LineSide::Left
        } else {
            line_side != crate::classify::LineSide::Right
        };
        let decision = crate::contour::minor_arc_winding_decision(
            start_y != std::cmp::Ordering::Greater,
            end_y == std::cmp::Ordering::Greater,
            point_is_left,
            inside_circle,
            is_ccw,
        );
        let (lower, upper, delta) = match decision {
            crate::contour::MinorArcWindingDecision::Delta(delta) => {
                return Ok(Classification::Decided(delta));
            }
            crate::contour::MinorArcWindingDecision::PointBetweenStartAndEnd(delta) => {
                (&start, &end, delta)
            }
            crate::contour::MinorArcWindingDecision::PointBetweenEndAndStart(delta) => {
                (&end, &start, delta)
            }
        };
        let lower_x = match order(lower, point, direction_x, direction_y)? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if lower_x != std::cmp::Ordering::Less {
            return Ok(Classification::Decided(0));
        }
        let upper_x = match order(upper, point, direction_x, direction_y)? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(Classification::Decided(
            if upper_x == std::cmp::Ordering::Greater {
                delta
            } else {
                0
            },
        ))
    }
}

impl BezierAlgebraicCuspSemicircleContactLocation2 {
    pub(crate) fn endpoint_parameter(self) -> Option<BezierAlgebraicCuspSemicircleParameter2> {
        match self {
            Self::Start => Some(BezierAlgebraicCuspSemicircleParameter2::Exact(Real::zero())),
            Self::End => Some(BezierAlgebraicCuspSemicircleParameter2::Exact(Real::one())),
            Self::Interior => None,
        }
    }

    /// Exact enclosure certified by finite-chord parameter classification.
    /// Endpoints are identities; an interior contact is strictly inside the
    /// unit interval. This avoids re-isolating a deep recursive scalar merely
    /// to obtain bounds already proved by its constructing kernel.
    pub(super) fn certified_unit_bounds(self) -> (Real, Real) {
        match self {
            Self::Interior => (Real::zero(), Real::one()),
            Self::Start => (Real::zero(), Real::zero()),
            Self::End => (Real::one(), Real::one()),
        }
    }
}

impl BezierAlgebraicCuspSemicircleMappedOverlap2 {
    pub(crate) const fn other_range(&self) -> &CurveParameterRange2 {
        &self.other_range
    }

    pub(super) fn mapped_other_range(&self) -> CurveResult<CurveParameterRange2> {
        if self.map_reversed {
            Ok(CurveParameterRange2::new_validated(
                self.other_range
                    .start()
                    .unit_complement()
                    .ok_or(CurveError::InvalidCurveParameter)?,
                self.other_range
                    .end()
                    .unit_complement()
                    .ok_or(CurveError::InvalidCurveParameter)?,
            ))
        } else {
            Ok(self.other_range.clone())
        }
    }

    pub(crate) fn cusp_start_parameter(&self) -> BezierAlgebraicCuspSemicircleParameter2 {
        self.cusp_start.clone()
    }

    pub(crate) fn cusp_end_parameter(&self) -> BezierAlgebraicCuspSemicircleParameter2 {
        self.cusp_end.clone()
    }

    pub(crate) const fn orientation(&self) -> CurveOverlapOrientation2 {
        self.orientation
    }

    pub(crate) fn parameter_ranges(&self) -> (CurveParameterRange2, CurveParameterRange2) {
        (
            CurveParameterRange2::new_validated(
                CurveParameter2::from_algebraic_cusp(self.cusp_start_parameter()),
                CurveParameter2::from_algebraic_cusp(self.cusp_end_parameter()),
            ),
            self.other_range.clone(),
        )
    }

    pub(crate) fn map_parameter(
        &self,
        parameter: &CurveParameter2,
        cusp_to_other: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<CurveParameter2>>> {
        if cusp_to_other {
            let Some(parameter) = parameter.as_algebraic_cusp() else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            return Ok(self.other_parameter_for_cusp(parameter, policy)?.map(Some));
        }
        Ok(self
            .cusp_parameter_for_other(parameter, policy)?
            .map(|parameter| Some(CurveParameter2::from_algebraic_cusp(parameter))))
    }

    pub(crate) fn has_positive_overlap(
        &self,
        cusp_fragment: &CurveParameterRange2,
        other_fragment: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        let (cusp_overlap, other_overlap) = self.parameter_ranges();
        crate::bezier_split::corresponding_parameter_ranges_are_positive(
            &cusp_overlap,
            &other_overlap,
            cusp_fragment,
            other_fragment,
            policy,
            |parameter| self.map_parameter(parameter, true, policy),
        )
    }

    pub(super) fn cusp_parameter_at_other_endpoint(
        &self,
        other_start: bool,
    ) -> BezierAlgebraicCuspSemicircleParameter2 {
        let cusp_start = other_start == (self.orientation == CurveOverlapOrientation2::Same);
        if cusp_start {
            self.cusp_start_parameter()
        } else {
            self.cusp_end_parameter()
        }
    }

    pub(super) fn other_parameter_at_cusp_endpoint(&self, cusp_start: bool) -> CurveParameter2 {
        let other_start = cusp_start == (self.orientation == CurveOverlapOrientation2::Same);
        if other_start {
            self.other_range.start().clone()
        } else {
            self.other_range.end().clone()
        }
    }

    pub(super) fn map_policy(&self) -> CurveContext {
        match &self.parameter_map {
            BezierAlgebraicCuspSemicircleMappedOverlapMap2::Rational(map) => map.data.policy,
            BezierAlgebraicCuspSemicircleMappedOverlapMap2::Parallel(map) => map.data.policy,
        }
    }

    pub(super) fn retain_inverse_authority(
        &self,
        retained: &Classification<CurveParameter2>,
        source: &BezierAlgebraicCuspSemicircleParameter2,
        policy: &CurveContext,
    ) {
        let Classification::Decided(target_parameter) = retained else {
            return;
        };
        let cache = match &self.parameter_map {
            BezierAlgebraicCuspSemicircleMappedOverlapMap2::Rational(target) => {
                &target.data.parameter_cache
            }
            BezierAlgebraicCuspSemicircleMappedOverlapMap2::Parallel(target) => {
                &target.data.parameter_cache
            }
        };
        cache.retain_cusp_parameter(target_parameter.clone(), source, policy);
    }

    /// Maps one parameter on this published overlap to its compact
    /// cusp-circle parameter without constructing another selected-root field.
    pub(crate) fn cusp_parameter_for_other(
        &self,
        parameter: &CurveParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicircleParameter2>> {
        if !policy.accepts_retained_policy(self.map_policy()) {
            return Err(CurveError::Topology(
                "mapped cusp overlap was replayed under a different predicate policy".into(),
            ));
        }
        for (endpoint, other_start) in [
            (self.other_range.start(), true),
            (self.other_range.end(), false),
        ] {
            match parameter.same_value(endpoint, policy)? {
                Classification::Decided(true) => {
                    return Ok(Classification::Decided(
                        self.cusp_parameter_at_other_endpoint(other_start),
                    ));
                }
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        let retained = match &self.parameter_map {
            BezierAlgebraicCuspSemicircleMappedOverlapMap2::Rational(map) => map
                .data
                .parameter_cache
                .retained_cusp_parameter(parameter, policy),
            BezierAlgebraicCuspSemicircleMappedOverlapMap2::Parallel(map) => map
                .data
                .parameter_cache
                .retained_cusp_parameter(parameter, policy),
        };
        let correlated = match retained {
            Some(Some(cusp)) => return Ok(Classification::Decided(cusp)),
            Some(None) => true,
            None => false,
        };
        let rational_correlation = if correlated {
            BezierAlgebraicCuspSemicircleRationalCorrelation2::Map
        } else {
            BezierAlgebraicCuspSemicircleRationalCorrelation2::Independent
        };
        let map_parameter = if self.map_reversed {
            parameter
                .unit_complement()
                .ok_or(CurveError::InvalidCurveParameter)?
        } else {
            parameter.clone()
        };
        let parameter = match &self.parameter_map {
            BezierAlgebraicCuspSemicircleMappedOverlapMap2::Rational(map) => {
                map.mapped_parameter(BezierAlgebraicCuspSemicircleRationalMapContact2 {
                    other_parameter: map_parameter,
                    location: BezierAlgebraicCuspSemicircleContactLocation2::Interior,
                    correlation: rational_correlation,
                })
            }
            BezierAlgebraicCuspSemicircleMappedOverlapMap2::Parallel(map) => {
                let map_parameter = match policy.strict_predicate_pass(|| {
                    map_parameter.promoted_bezier_parameter_complete(policy)
                })? {
                    Classification::Decided(parameter) => parameter,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let contact = BezierAlgebraicCuspSemicircleParallelContact2 {
                    parallel_parameter: map_parameter,
                    tangent_cross_sign: None,
                    location: BezierAlgebraicCuspSemicircleContactLocation2::Interior,
                    correlation: if correlated {
                        BezierAlgebraicCuspSemicircleParallelCorrelation2::Map
                    } else {
                        BezierAlgebraicCuspSemicircleParallelCorrelation2::Independent
                    },
                };
                map.contact_parameter(&contact)
            }
        };
        Ok(Classification::Decided(parameter))
    }

    /// Inverts this overlap at one compact cusp parameter.
    ///
    /// Cuts authored by this same map reuse their retained analytic parameter.
    /// An independently represented `Real` cut projects the exact radial-dot
    /// equality in the retained cusp field and replays its unsquared branch.
    /// General unrelated mapped fields remain explicit until the region
    /// carrier can retain their cross-field correspondence directly.
    pub(crate) fn other_parameter_for_cusp(
        &self,
        parameter: &BezierAlgebraicCuspSemicircleParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<CurveParameter2>> {
        parameter.validate_policy(policy)?;
        if !policy.accepts_retained_policy(self.map_policy()) {
            return Err(CurveError::Topology(
                "mapped cusp overlap was replayed under a different predicate policy".into(),
            ));
        }
        for (endpoint, cusp_start) in [(&self.cusp_start, true), (&self.cusp_end, false)] {
            match parameter.cmp_by_refinement(endpoint, policy)? {
                Classification::Decided(std::cmp::Ordering::Equal) => {
                    return Ok(Classification::Decided(
                        self.other_parameter_at_cusp_endpoint(cusp_start),
                    ));
                }
                Classification::Decided(std::cmp::Ordering::Less | std::cmp::Ordering::Greater) => {
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }

        if let BezierAlgebraicCuspSemicircleParameter2::Mapped(data) = parameter {
            let same_map_parameter = match (&self.parameter_map, data.as_ref()) {
                (
                    BezierAlgebraicCuspSemicircleMappedOverlapMap2::Rational(overlap_map),
                    BezierAlgebraicCuspSemicircleMappedParameterData2::Rational { map, contact },
                ) => map
                    .parameterization_orientation(overlap_map)
                    .map(|orientation| (contact.other_parameter.clone(), orientation)),
                (
                    BezierAlgebraicCuspSemicircleMappedOverlapMap2::Parallel(overlap_map),
                    BezierAlgebraicCuspSemicircleMappedParameterData2::Parallel { map, contact },
                ) => map
                    .parameterization_orientation(overlap_map)
                    .map(|orientation| {
                        (
                            CurveParameter2::from(contact.parallel_parameter.clone()),
                            orientation,
                        )
                    }),
                _ => None,
            };
            if let Some((parameter, orientation)) = same_map_parameter {
                let reverse =
                    self.map_reversed != (orientation == CurveOverlapOrientation2::Reversed);
                let parameter = if reverse {
                    parameter.unit_complement().ok_or_else(|| {
                        CurveError::Topology(
                            "a mapped rational parameter had no unit complement".into(),
                        )
                    })?
                } else {
                    parameter
                };
                return Ok(Classification::Decided(parameter));
            }

            if let BezierAlgebraicCuspSemicircleMappedOverlapMap2::Rational(target) =
                &self.parameter_map
                && let Some((source, contact)) = data.coincident_rational_source()
            {
                if !policy.accepts_retained_policy(source.data.policy) {
                    return Err(CurveError::Topology(
                        "mapped rational source used a different predicate policy".into(),
                    ));
                }
                match RationalBezierOverlapParameterCorrespondence2::map_region_parameter_between_curves(
                    &source.data.curve,
                    &target.data.curve,
                    &contact.other_parameter,
                    policy,
                )? {
                    Classification::Decided(Some(parameter)) => {
                        let parameter = if self.map_reversed {
                            match parameter.unit_complement() {
                                Some(complement) => complement,
                                None => match promote_curve_region_bezier_parameter(
                                    &parameter, policy,
                                )? {
                                    Classification::Decided(parameter) => {
                                        CurveParameter2::from(parameter.unit_complement())
                                    }
                                    Classification::Uncertain(reason) => {
                                        return Ok(Classification::Uncertain(reason));
                                    }
                                },
                            }
                        } else {
                            parameter
                        };
                        return retain_direct_overlap_parameter(parameter, &self.other_range, policy);
                    }
                    Classification::Decided(None) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }

            if let BezierAlgebraicCuspSemicircleMappedOverlapMap2::Rational(target) =
                &self.parameter_map
                && let Some((source, contact, first, source_reversed)) =
                    data.coincident_pair_source()
            {
                let target_reversed = source_reversed
                    ^ (self.orientation == CurveOverlapOrientation2::Reversed)
                    ^ self.map_reversed;
                let candidates = match source.rational_parameters_for_contact(
                    contact,
                    first,
                    &target.data.curve,
                    &self.mapped_other_range()?,
                    target_reversed,
                    policy,
                )? {
                    Classification::Decided(candidates) => candidates,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                return retain_unique_overlap_parameter(
                    candidates,
                    &self.other_range,
                    self.map_reversed,
                    true,
                    policy,
                    |_| Ok(Classification::Decided(RealSign::Zero)),
                );
            }

            if let BezierAlgebraicCuspSemicircleMappedOverlapMap2::Parallel(target) =
                &self.parameter_map
                && let Some((source, contact, first, source_reversed)) =
                    data.coincident_pair_source()
            {
                let target_reversed = source_reversed
                    ^ (self.orientation == CurveOverlapOrientation2::Reversed)
                    ^ self.map_reversed;
                let candidates = match source.parallel_parameters_for_contact(
                    contact,
                    first,
                    &target.data.parallel,
                    &self.mapped_other_range()?,
                    target_reversed,
                    policy,
                )? {
                    Classification::Decided(candidates) => candidates,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                return retain_unique_overlap_parameter(
                    curve_region_parameters_from_bezier(candidates),
                    &self.other_range,
                    self.map_reversed,
                    true,
                    policy,
                    |_| Ok(Classification::Decided(RealSign::Zero)),
                );
            }

            if let Some((source, start, evidence_policy)) = data.coincident_pair_endpoint_source() {
                if !policy.accepts_retained_policy(evidence_policy) {
                    return Err(CurveError::Topology(
                        "cusp-pair overlap endpoint was replayed under a different predicate policy"
                            .into(),
                    ));
                }
                let parameter = if start { Real::zero() } else { Real::one() };
                let candidates = match &self.parameter_map {
                    BezierAlgebraicCuspSemicircleMappedOverlapMap2::Rational(target) => {
                        rational_parameters_for_cusp_endpoint(
                            source,
                            &parameter,
                            &target.data.curve,
                            &self.mapped_other_range()?,
                            policy,
                        )?
                    }
                    BezierAlgebraicCuspSemicircleMappedOverlapMap2::Parallel(target) => {
                        parallel_parameters_for_cusp_endpoint(
                            source,
                            &parameter,
                            &target.data.parallel,
                            &self.mapped_other_range()?,
                            policy,
                        )?
                    }
                };
                let candidates = match candidates {
                    Classification::Decided(candidates) => candidates,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                return retain_unique_overlap_parameter(
                    candidates,
                    &self.other_range,
                    self.map_reversed,
                    true,
                    policy,
                    |_| Ok(Classification::Decided(RealSign::Zero)),
                );
            }

            let equivalent_cross_map_parameter = match (&self.parameter_map, data.as_ref()) {
                (
                    BezierAlgebraicCuspSemicircleMappedOverlapMap2::Rational(target),
                    BezierAlgebraicCuspSemicircleMappedParameterData2::Parallel {
                        map: source,
                        contact,
                    },
                ) => {
                    match rational_parallel_parameter_orientation_at_cut(
                        target,
                        source,
                        &contact.parallel_parameter,
                        false,
                        policy,
                    )? {
                        Classification::Decided(Some(orientation)) => Some((
                            CurveParameter2::from(contact.parallel_parameter.clone()),
                            orientation,
                        )),
                        Classification::Decided(None) => None,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                (
                    BezierAlgebraicCuspSemicircleMappedOverlapMap2::Parallel(target),
                    BezierAlgebraicCuspSemicircleMappedParameterData2::Rational {
                        map: source,
                        contact,
                    },
                ) => {
                    let source_parameter = match promote_curve_region_bezier_parameter(
                        &contact.other_parameter,
                        policy,
                    )? {
                        Classification::Decided(parameter) => parameter,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    match rational_parallel_parameter_orientation_at_cut(
                        source,
                        target,
                        &source_parameter,
                        true,
                        policy,
                    )? {
                        Classification::Decided(Some(orientation)) => {
                            Some((contact.other_parameter.clone(), orientation))
                        }
                        Classification::Decided(None) => None,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                _ => None,
            };
            if let Some((parameter, orientation)) = equivalent_cross_map_parameter {
                let reverse =
                    self.map_reversed != (orientation == CurveOverlapOrientation2::Reversed);
                let parameter = if reverse {
                    parameter.unit_complement().ok_or_else(|| {
                        CurveError::Topology(
                            "a cross-map rational parameter had no unit complement".into(),
                        )
                    })?
                } else {
                    parameter
                };
                return retain_direct_overlap_parameter(parameter, &self.other_range, policy);
            }

            if let Some(tangent_cross) = data.ordinary_carrier_tangent_cross_sign(policy)? {
                match tangent_cross {
                    Classification::Decided(RealSign::Zero) => {}
                    Classification::Decided(RealSign::Negative | RealSign::Positive) => {
                        let direct_candidates = match data.chamfer_exact_point(policy)? {
                            Classification::Decided(Some(point)) => data
                                .exact_point_circle_tangent_parameter_candidates_on_target(
                                    &point,
                                    &self.parameter_map,
                                    &self.mapped_other_range()?,
                                    policy,
                                )?,
                            Classification::Decided(None) => Classification::Decided(None),
                            Classification::Uncertain(reason) => Classification::Uncertain(reason),
                        };
                        let candidates = match direct_candidates {
                            decided @ Classification::Decided(Some(_)) => decided,
                            Classification::Decided(None) | Classification::Uncertain(_) => {
                                if let Some(source) = data.mapped_point_source(policy)? {
                                    let tangent_candidates = if let Some(point) =
                                        source.algebraic_point_image(policy)?
                                    {
                                        data.one_field_circle_tangent_parameter_candidates_on_target(
                                            &point,
                                            &self.parameter_map,
                                            &self.mapped_other_range()?,
                                            policy,
                                        )?
                                    } else {
                                        match source.exact_point(policy)? {
                                            Classification::Decided(Some(point)) => data
                                                .exact_point_circle_tangent_parameter_candidates_on_target(
                                                    &point,
                                                    &self.parameter_map,
                                                    &self.mapped_other_range()?,
                                                    policy,
                                                )?,
                                            Classification::Decided(None) => {
                                                Classification::Decided(None)
                                            }
                                            Classification::Uncertain(reason) => {
                                                Classification::Uncertain(reason)
                                            }
                                        }
                                    };
                                    match tangent_candidates {
                                        decided @ Classification::Decided(Some(_)) => decided,
                                        Classification::Decided(None)
                                        | Classification::Uncertain(_) => source
                                            .point_parameter_candidates_on_target(
                                                &self.parameter_map,
                                                &self.mapped_other_range()?,
                                                policy,
                                            )?,
                                    }
                                } else {
                                    data.retained_point_parameter_candidates_on_target(
                                        &self.parameter_map,
                                        &self.mapped_other_range()?,
                                        policy,
                                    )?
                                }
                            }
                        };
                        let candidates = match candidates {
                            Classification::Decided(Some(candidates)) => candidates,
                            Classification::Decided(None) => {
                                return Ok(Classification::Uncertain(
                                    UncertaintyReason::Unsupported,
                                ));
                            }
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        };
                        let retained = retain_unique_overlap_parameter(
                            candidates,
                            &self.other_range,
                            self.map_reversed,
                            true,
                            policy,
                            |_| Ok(Classification::Decided(RealSign::Zero)),
                        )?;
                        self.retain_inverse_authority(&retained, parameter, policy);
                        return Ok(retained);
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }

            if let Some(source) = data.mapped_point_source(policy)?
                && let Classification::Decided(Some(candidates)) = source
                    .point_parameter_candidates_on_target(
                        &self.parameter_map,
                        &self.mapped_other_range()?,
                        policy,
                    )?
                && !candidates.is_empty()
            {
                let retained = retain_unique_overlap_parameter(
                    candidates,
                    &self.other_range,
                    self.map_reversed,
                    true,
                    policy,
                    |_| Ok(Classification::Decided(RealSign::Zero)),
                )?;
                self.retain_inverse_authority(&retained, parameter, policy);
                return Ok(retained);
            }

            let source = data.coincident_tangent_power_source(policy)?;
            if let Some((source_parameter, source_tangent, source_policy)) = source {
                if !policy.accepts_retained_policy(source_policy) {
                    return Err(CurveError::Topology(
                        "mapped tangent source used a different predicate policy".into(),
                    ));
                }
                let target_tangent = match &self.parameter_map {
                    BezierAlgebraicCuspSemicircleMappedOverlapMap2::Rational(target) => {
                        rational_parametric_tangent_numerator(
                            target.data.curve.homogeneous_power_basis()?,
                        )
                    }
                    BezierAlgebraicCuspSemicircleMappedOverlapMap2::Parallel(target) => {
                        let differential = target.data.parallel.differential()?;
                        [
                            differential.tangent_x.clone(),
                            differential.tangent_y.clone(),
                        ]
                    }
                };
                let candidates = match mapped_circle_tangent_parameter_candidates(
                    source_parameter.as_ref(),
                    &source_tangent,
                    &target_tangent,
                    &self.mapped_other_range()?,
                    policy,
                )? {
                    Classification::Decided(candidates) => candidates,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let retained = retain_unique_overlap_parameter(
                    candidates,
                    &self.other_range,
                    self.map_reversed,
                    false,
                    policy,
                    |_| Ok(Classification::Decided(RealSign::Zero)),
                )?;
                self.retain_inverse_authority(&retained, parameter, policy);
                return Ok(retained);
            }

            match data.retained_point_parameter_candidates_on_target(
                &self.parameter_map,
                &self.mapped_other_range()?,
                policy,
            )? {
                Classification::Decided(Some(candidates)) => {
                    let retained = retain_unique_overlap_parameter(
                        candidates,
                        &self.other_range,
                        self.map_reversed,
                        true,
                        policy,
                        |_| Ok(Classification::Decided(RealSign::Zero)),
                    )?;
                    self.retain_inverse_authority(&retained, parameter, policy);
                    return Ok(retained);
                }
                Classification::Decided(None) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }

            match parameter.scalar_value(policy)? {
                Classification::Decided(Some(parameter)) => {
                    return self.other_parameter_for_cusp(
                        &BezierAlgebraicCuspSemicircleParameter2::Exact(parameter),
                        policy,
                    );
                }
                Classification::Decided(None) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }

        let BezierAlgebraicCuspSemicircleParameter2::Exact(parameter) = parameter else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        match &self.parameter_map {
            BezierAlgebraicCuspSemicircleMappedOverlapMap2::Rational(map) => {
                rational_overlap_parameter_for_exact_cusp(
                    map,
                    parameter,
                    &self.other_range,
                    self.map_reversed,
                    policy,
                )
            }
            BezierAlgebraicCuspSemicircleMappedOverlapMap2::Parallel(map) => {
                parallel_overlap_parameter_for_exact_cusp(
                    map,
                    parameter,
                    &self.other_range,
                    self.map_reversed,
                    policy,
                )
            }
        }
    }
}

impl BezierAlgebraicCuspSemicircleMappedPointParameter2 {
    pub(super) fn as_ref(&self) -> BezierAlgebraicCuspSemicircleMappedPointParameterRef2<'_> {
        match self {
            Self::Ordinary(parameter) => {
                BezierAlgebraicCuspSemicircleMappedPointParameterRef2::Ordinary(parameter)
            }
            Self::Selected(parameter) => {
                BezierAlgebraicCuspSemicircleMappedPointParameterRef2::Selected(parameter)
            }
        }
    }
}

impl BezierAlgebraicCuspSemicircleMappedPointSource2 {
    pub(super) fn from_borrowed(
        source: BezierAlgebraicCuspSemicircleMappedTangentSource2<'_>,
    ) -> Self {
        match source {
            BezierAlgebraicCuspSemicircleMappedTangentSource2::Rational {
                curve,
                parameter,
                policy,
            } => Self::Rational {
                curve: curve.clone(),
                parameter: BezierAlgebraicCuspSemicircleMappedPointParameter2::Ordinary(
                    parameter.clone(),
                ),
                policy,
            },
            BezierAlgebraicCuspSemicircleMappedTangentSource2::Parallel {
                parallel,
                parameter,
                policy,
            } => Self::Parallel {
                parallel: parallel.clone(),
                parameter: BezierAlgebraicCuspSemicircleMappedPointParameter2::Ordinary(
                    parameter.clone(),
                ),
                policy,
            },
        }
    }

    pub(super) fn policy(&self) -> CurveContext {
        match self {
            Self::Rational { policy, .. } | Self::Parallel { policy, .. } => *policy,
        }
    }

    pub(super) fn transform_similarity(self, transform: &Similarity2) -> CurveResult<Self> {
        Ok(match self {
            Self::Rational {
                curve,
                parameter,
                policy,
            } => Self::Rational {
                curve: curve.transform_similarity(transform),
                parameter,
                policy,
            },
            Self::Parallel {
                parallel,
                parameter,
                policy,
            } => Self::Parallel {
                parallel: parallel.transform_similarity(transform)?,
                parameter,
                policy,
            },
        })
    }

    /// Publishes the selected source point in one ordinary algebraic field
    /// when the owned carrier has an exact rational image. This is a cold
    /// carrier-switch bridge; it never approximates an analytic parallel.
    pub(super) fn algebraic_point_image(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Option<RationalBezierAlgebraicPointImage2>> {
        let source_policy = self.policy();
        if !policy.accepts_retained_policy(source_policy) {
            return Err(CurveError::Topology(
                "mapped point carrier crossed predicate policies".into(),
            ));
        }
        let parameter = match self {
            Self::Rational {
                parameter: BezierAlgebraicCuspSemicircleMappedPointParameter2::Ordinary(parameter),
                ..
            }
            | Self::Parallel {
                parameter: BezierAlgebraicCuspSemicircleMappedPointParameter2::Ordinary(parameter),
                ..
            } => parameter,
            Self::Rational {
                parameter: BezierAlgebraicCuspSemicircleMappedPointParameter2::Selected(_),
                ..
            }
            | Self::Parallel {
                parameter: BezierAlgebraicCuspSemicircleMappedPointParameter2::Selected(_),
                ..
            } => {
                return Ok(None);
            }
        };
        let BezierParameter2::Algebraic(parameter) = parameter else {
            return Ok(None);
        };
        let curve = match self {
            Self::Rational { curve, .. } => Some(curve.clone()),
            Self::Parallel { parallel, .. } => match policy
                .strict_predicate_pass(|| parallel.exact_rational_parallel_component(policy))?
            {
                Classification::Decided(curve) => curve,
                Classification::Uncertain(_) => None,
            },
        };
        let Some(curve) = curve else {
            return Ok(None);
        };
        Ok(Some(
            RationalBezierAlgebraicPointImage2::from_parametric_source(
                curve,
                parameter.clone(),
                policy,
            ),
        ))
    }

    pub(super) fn exact_point(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Point2>>> {
        let source_policy = self.policy();
        if !policy.accepts_retained_policy(source_policy) {
            return Err(CurveError::Topology(
                "mapped point carrier crossed predicate policies".into(),
            ));
        }
        let parameter = match self {
            Self::Rational { parameter, .. } | Self::Parallel { parameter, .. } => {
                match parameter {
                    BezierAlgebraicCuspSemicircleMappedPointParameter2::Ordinary(parameter) => {
                        match parameter {
                            BezierParameter2::Exact(parameter) => parameter.clone(),
                            BezierParameter2::Algebraic(parameter) => {
                                match policy.strict_predicate_pass(|| {
                                    parameter.represented_exact_point_with_policy(policy)
                                })? {
                                    Classification::Decided(Some(parameter)) => parameter,
                                    Classification::Decided(None)
                                    | Classification::Uncertain(_) => {
                                        return Ok(Classification::Decided(None));
                                    }
                                }
                            }
                        }
                    }
                    BezierAlgebraicCuspSemicircleMappedPointParameter2::Selected(parameter) => {
                        let Some(parameter) = parameter.represented_value() else {
                            return Ok(Classification::Decided(None));
                        };
                        parameter.clone()
                    }
                }
            }
        };
        match self {
            Self::Rational { curve, .. } => Ok(curve
                .point_at_affine_classified(&parameter, policy)
                .map(Some)),
            Self::Parallel { parallel, .. } => {
                Ok(parallel.point_at_with_policy(&parameter, policy)?.map(Some))
            }
        }
    }

    /// Materializes an ordinary mapped carrier point only when a mixed-field
    /// predicate requires standalone Cartesian algebraic numbers. Rational
    /// carriers reuse their lazy point image, while analytic parallels reuse
    /// the single authoritative normalized-tangent point kernel.
    pub(super) fn represented_coordinates(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<[AlgebraicRootRepresentation; 2]>> {
        let source_policy = self.policy();
        if !policy.accepts_retained_policy(source_policy) {
            return Err(CurveError::Topology(
                "mapped point carrier crossed predicate policies".into(),
            ));
        }
        match self {
            Self::Rational {
                curve, parameter, ..
            } => match parameter {
                BezierAlgebraicCuspSemicircleMappedPointParameter2::Ordinary(parameter) => {
                    match parameter {
                        BezierParameter2::Exact(parameter) => Ok(curve
                            .point_at_affine_classified(parameter, policy)
                            .map(|point| {
                                [
                                    AlgebraicRootRepresentation::from_exact_value(point.x()),
                                    AlgebraicRootRepresentation::from_exact_value(point.y()),
                                ]
                            })),
                        BezierParameter2::Algebraic(parameter) => {
                            Ok(RationalBezierAlgebraicPointImage2::from_parametric_source(
                                curve.clone(),
                                parameter.clone(),
                                policy,
                            )
                            .represented_coordinates(policy)
                            .map(Classification::Decided)
                            .unwrap_or(Classification::Uncertain(UncertaintyReason::Unsupported)))
                        }
                    }
                }
                BezierAlgebraicCuspSemicircleMappedPointParameter2::Selected(parameter) => {
                    BezierAnalyticParallelPoint2::new_selected_fiber(
                        curve.parallel_left(Real::zero())?,
                        parameter.clone(),
                        &source_policy,
                    )
                    .represented_coordinates(policy)
                }
            },
            Self::Parallel {
                parallel,
                parameter,
                ..
            } => match parameter {
                BezierAlgebraicCuspSemicircleMappedPointParameter2::Ordinary(parameter) => {
                    BezierAnalyticParallelPoint2::new(
                        parallel.clone(),
                        parameter.clone(),
                        &source_policy,
                    )
                    .represented_coordinates(policy)
                }
                BezierAlgebraicCuspSemicircleMappedPointParameter2::Selected(parameter) => {
                    BezierAnalyticParallelPoint2::new_selected_fiber(
                        parallel.clone(),
                        parameter.clone(),
                        &source_policy,
                    )
                    .represented_coordinates(policy)
                }
            },
        }
    }

    pub(super) fn point_parameter_candidates_on_target(
        &self,
        target: &BezierAlgebraicCuspSemicircleMappedOverlapMap2,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Vec<CurveParameter2>>>> {
        match target {
            BezierAlgebraicCuspSemicircleMappedOverlapMap2::Rational(target) => self
                .point_parameter_candidates(
                    &target.data.curve.parallel_left(Real::zero())?,
                    range,
                    policy,
                ),
            BezierAlgebraicCuspSemicircleMappedOverlapMap2::Parallel(target) => {
                self.point_parameter_candidates(&target.data.parallel, range, policy)
            }
        }
    }

    /// Inverts the retained point itself. Rational images share one finite
    /// incidence authority; a genuinely analytic source keeps one local paired
    /// fallback without intersecting unused portions of either carrier.
    pub(super) fn point_parameter_candidates(
        &self,
        target: &BezierParallel2,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Vec<CurveParameter2>>>> {
        if !policy.accepts_retained_policy(self.policy()) {
            return Err(CurveError::Topology(
                "mapped point carrier crossed predicate policies".into(),
            ));
        }
        if let Classification::Decided(Some(point)) = self.exact_point(policy)? {
            let inverse =
                one_field_point_parameter_candidates(&point.into(), target, range, policy)?;
            if matches!(inverse, Classification::Decided(_)) {
                return Ok(inverse);
            }
        }
        let (parallel, parameter) = match self {
            Self::Rational {
                curve, parameter, ..
            } => {
                let source = curve.homogeneous_power_basis()?;
                return parametric_point_parameter_candidates(
                    parameter.as_ref(),
                    [&source.x_numerator, &source.y_numerator, &source.weight],
                    target,
                    range,
                    policy,
                );
            }
            Self::Parallel {
                parallel,
                parameter,
                ..
            } => (parallel, parameter.as_ref()),
        };
        let distance_sign = match real_sign(parallel.distance(), policy) {
            Some(sign) => sign,
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        if distance_sign == RealSign::Zero {
            let source = parallel.source_power_basis()?;
            return parametric_point_parameter_candidates(
                parameter,
                [
                    source.x_numerator,
                    source.y_numerator,
                    source.weight.unwrap_or(&[Real::one()]),
                ],
                target,
                range,
                policy,
            );
        }
        let source_range = match mapped_point_regular_source_range(parallel, parameter, policy)? {
            Classification::Decided(range) => range,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        if let Classification::Decided(Some(component)) =
            parallel.exact_rational_parallel_component_on_regular_range(&source_range, policy)?
        {
            let source = component.curve().homogeneous_power_basis()?;
            return parametric_point_parameter_candidates(
                parameter,
                [&source.x_numerator, &source.y_numerator, &source.weight],
                target,
                range,
                policy,
            );
        }
        let intersections = match parallel.parallel_intersections_in_domain(
            target,
            [
                CurveParameterDomain2::new(&source_range, None),
                CurveParameterDomain2::new(range, None),
            ],
            ParameterComponentQuery2::RetainFinite,
            policy,
        )? {
            Classification::Decided(intersections) => intersections,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let (intersections, positive_dimensional) = intersections.into_parts();
        debug_assert!(
            positive_dimensional.is_empty(),
            "finite inverse retains component correspondences"
        );
        let candidates = match parameter.matching_target_parameters(
            intersections.contacts().iter().map(|contact| {
                (
                    contact.first_parameter().clone(),
                    contact.second_parameter().clone(),
                )
            }),
            policy,
        )? {
            Classification::Decided(candidates) => candidates,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let overlap_curves = if intersections.overlaps().is_empty() {
            None
        } else {
            Some((
                parallel.source().to_rational_bezier()?,
                target.source().to_rational_bezier()?,
            ))
        };
        finish_mapped_point_parameter_candidates(
            candidates,
            intersections.component_overlaps(),
            intersections.parameter_components(),
            intersections.overlaps(),
            overlap_curves
                .as_ref()
                .map(|(source, target)| (source, target)),
            parameter,
            intersections.is_complete(),
            policy,
        )
    }

    /// Publishes the carrier tangent without flattening its compact parameter
    /// authority. Point and tangent correspondence therefore retain the same
    /// selected root and differ only in the polynomial geometry they replay.
    pub(super) fn tangent_power_source(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<
        Option<(
            BezierAlgebraicCuspSemicircleMappedPointParameter2,
            [Vec<Real>; 2],
            CurveContext,
        )>,
    > {
        let (parameter, tangent, source_policy) = match self {
            Self::Rational {
                curve,
                parameter,
                policy: source_policy,
            } => (
                parameter.clone(),
                rational_parametric_tangent_numerator(curve.homogeneous_power_basis()?),
                *source_policy,
            ),
            Self::Parallel {
                parallel,
                parameter,
                policy: source_policy,
            } => {
                let differential = parallel.differential()?;
                (
                    parameter.clone(),
                    [
                        differential.tangent_x.clone(),
                        differential.tangent_y.clone(),
                    ],
                    *source_policy,
                )
            }
        };
        if !policy.accepts_retained_policy(source_policy) {
            return Err(CurveError::Topology(
                "mapped tangent crossed predicate policies".into(),
            ));
        }
        Ok(Some((parameter, tangent, source_policy)))
    }
}

impl BezierAlgebraicCuspSemicirclePairOverlap2 {
    pub(crate) fn orientation(&self) -> CurveOverlapOrientation2 {
        self.data.orientation
    }

    #[cfg(test)]
    pub(crate) fn first_boundaries(&self) -> [BezierAlgebraicCuspSemicirclePairEndpoint2; 2] {
        self.data.first_boundaries
    }

    #[cfg(test)]
    pub(crate) fn second_boundaries(&self) -> [BezierAlgebraicCuspSemicirclePairEndpoint2; 2] {
        self.data.second_boundaries
    }

    pub(super) fn semicircle(&self, first: bool) -> &BezierAlgebraicCuspSemicircle2 {
        let (first_semicircle, second_semicircle) = match &self.data.parameter_map {
            BezierAlgebraicCuspSemicirclePairOverlapParameterMapData2::ExactEndpoints {
                first_semicircle,
                second_semicircle,
            }
            | BezierAlgebraicCuspSemicirclePairOverlapParameterMapData2::Represented {
                first_semicircle,
                second_semicircle,
                ..
            }
            | BezierAlgebraicCuspSemicirclePairOverlapParameterMapData2::SimilarityTransport {
                first_semicircle,
                second_semicircle,
                ..
            } => (first_semicircle, second_semicircle),
        };
        if first {
            first_semicircle
        } else {
            second_semicircle
        }
    }

    pub(super) fn has_exact_endpoint_map(&self) -> bool {
        match &self.data.parameter_map {
            BezierAlgebraicCuspSemicirclePairOverlapParameterMapData2::ExactEndpoints {
                ..
            } => true,
            BezierAlgebraicCuspSemicirclePairOverlapParameterMapData2::Represented { .. } => false,
            BezierAlgebraicCuspSemicirclePairOverlapParameterMapData2::SimilarityTransport {
                source,
                ..
            } => source.has_exact_endpoint_map(),
        }
    }

    pub(super) fn shares_parameter_map(
        &self,
        source_first: bool,
        other: &Self,
        other_source_first: bool,
    ) -> bool {
        if source_first == other_source_first && Arc::ptr_eq(&self.data, &other.data) {
            return true;
        }
        // Each half-circle chart is injective. The same two exact carriers
        // therefore determine the same parameter map independently of which
        // operand order, intersection, or similarity pass retained its proof.
        // Boundary labels identify the overlap domain, not the chart map.
        self.data.orientation == other.data.orientation
            && self.data.policy == other.data.policy
            && self.semicircle(source_first) == other.semicircle(other_source_first)
            && self.semicircle(!source_first) == other.semicircle(!other_source_first)
    }

    pub(super) fn boundary_parameter(
        &self,
        endpoint: BezierAlgebraicCuspSemicirclePairEndpoint2,
        first: bool,
    ) -> BezierAlgebraicCuspSemicircleParameter2 {
        match (first, endpoint) {
            (true, BezierAlgebraicCuspSemicirclePairEndpoint2::FirstStart)
            | (false, BezierAlgebraicCuspSemicirclePairEndpoint2::SecondStart) => {
                BezierAlgebraicCuspSemicircleParameter2::Exact(Real::zero())
            }
            (true, BezierAlgebraicCuspSemicirclePairEndpoint2::FirstEnd)
            | (false, BezierAlgebraicCuspSemicirclePairEndpoint2::SecondEnd) => {
                BezierAlgebraicCuspSemicircleParameter2::Exact(Real::one())
            }
            _ => BezierAlgebraicCuspSemicircleParameter2::Mapped(Arc::new(
                BezierAlgebraicCuspSemicircleMappedParameterData2::PairOverlap {
                    overlap: self.clone(),
                    endpoint,
                    first,
                },
            )),
        }
    }

    pub(crate) fn first_start_parameter(&self) -> BezierAlgebraicCuspSemicircleParameter2 {
        self.boundary_parameter(self.data.first_boundaries[0], true)
    }

    pub(crate) fn first_end_parameter(&self) -> BezierAlgebraicCuspSemicircleParameter2 {
        self.boundary_parameter(self.data.first_boundaries[1], true)
    }

    pub(crate) fn second_start_parameter(&self) -> BezierAlgebraicCuspSemicircleParameter2 {
        self.boundary_parameter(self.data.second_boundaries[0], false)
    }

    pub(crate) fn second_end_parameter(&self) -> BezierAlgebraicCuspSemicircleParameter2 {
        self.boundary_parameter(self.data.second_boundaries[1], false)
    }

    /// Maps one retained local parameter across this coincident-circle overlap.
    ///
    /// Full aligned overlaps reuse the source value directly (or its exact
    /// unit complement). General overlaps retain one shared correspondence
    /// pointer plus the source cut, avoiding a primitive element for the two
    /// selected cusp fields and any field carried by the source cut itself.
    pub(crate) fn map_parameter(
        &self,
        parameter: &BezierAlgebraicCuspSemicircleParameter2,
        source_first: bool,
    ) -> BezierAlgebraicCuspSemicircleParameter2 {
        // Cancel a certified inverse before the full-overlap path can wrap a
        // mapped cut in another unit complement. Pair enumeration order does
        // not change which exact source and destination charts compose.
        if let BezierAlgebraicCuspSemicircleParameter2::Mapped(data) = parameter
            && let BezierAlgebraicCuspSemicircleMappedParameterData2::PairOverlapMap {
                overlap,
                source,
                source_first: mapped_source_first,
            } = data.as_ref()
            && overlap.shares_parameter_map(*mapped_source_first, self, !source_first)
        {
            return source.clone();
        }
        if self.has_exact_endpoint_map() {
            return if self.data.orientation == CurveOverlapOrientation2::Same {
                parameter.clone()
            } else if let BezierAlgebraicCuspSemicircleParameter2::Exact(parameter) = parameter {
                BezierAlgebraicCuspSemicircleParameter2::Exact(Real::one() - parameter)
            } else {
                BezierAlgebraicCuspSemicircleParameter2::Mapped(Arc::new(
                    BezierAlgebraicCuspSemicircleMappedParameterData2::PairOverlapMap {
                        overlap: self.clone(),
                        source: parameter.clone(),
                        source_first,
                    },
                ))
            };
        }

        let boundaries = if source_first {
            self.data.first_boundaries
        } else {
            self.data.second_boundaries
        };
        for endpoint in boundaries {
            if parameter.shares_exact_evidence(&self.boundary_parameter(endpoint, source_first)) {
                return self.boundary_parameter(endpoint, !source_first);
            }
        }
        BezierAlgebraicCuspSemicircleParameter2::Mapped(Arc::new(
            BezierAlgebraicCuspSemicircleMappedParameterData2::PairOverlapMap {
                overlap: self.clone(),
                source: parameter.clone(),
                source_first,
            },
        ))
    }

    pub(crate) fn parameter_ranges(&self) -> (CurveParameterRange2, CurveParameterRange2) {
        (
            CurveParameterRange2::new_validated(
                CurveParameter2::from_algebraic_cusp(self.first_start_parameter()),
                CurveParameter2::from_algebraic_cusp(self.first_end_parameter()),
            ),
            CurveParameterRange2::new_validated(
                CurveParameter2::from_algebraic_cusp(self.second_start_parameter()),
                CurveParameter2::from_algebraic_cusp(self.second_end_parameter()),
            ),
        )
    }

    /// Decides positive retained overlap without constructing unused inverse
    /// cuts for fillet coincidence classification.
    pub(crate) fn has_positive_overlap(
        &self,
        first_fragment: &CurveParameterRange2,
        second_fragment: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        let (first_overlap, second_overlap) = self.parameter_ranges();
        crate::bezier_split::corresponding_parameter_ranges_are_positive(
            &first_overlap,
            &second_overlap,
            first_fragment,
            second_fragment,
            policy,
            |parameter| {
                let Some(parameter) = parameter.as_algebraic_cusp() else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                };
                Ok(Classification::Decided(Some(
                    CurveParameter2::from_algebraic_cusp(self.map_parameter(parameter, true)),
                )))
            },
        )
    }

    /// Recognizes two local parameters joined by this exact coincident-circle
    /// map. This is structural endpoint evidence: no scalar or point equality
    /// predicate is needed when a retained boundary crosses independent circle
    /// frames through the map that authored its second endpoint.
    pub(super) fn maps_parameter_evidence(
        &self,
        source_semicircle: &BezierAlgebraicCuspSemicircle2,
        source_parameter: &BezierAlgebraicCuspSemicircleParameter2,
        target_semicircle: &BezierAlgebraicCuspSemicircle2,
        target_parameter: &BezierAlgebraicCuspSemicircleParameter2,
    ) -> bool {
        let source_first = if self.semicircle(true) == source_semicircle
            && self.semicircle(false) == target_semicircle
        {
            true
        } else if self.semicircle(false) == source_semicircle
            && self.semicircle(true) == target_semicircle
        {
            false
        } else {
            return false;
        };
        self.map_parameter(source_parameter, source_first)
            .shares_exact_evidence(target_parameter)
    }

    pub(super) fn endpoint_location(
        endpoint: BezierAlgebraicCuspSemicirclePairEndpoint2,
        first: bool,
    ) -> BezierAlgebraicCuspSemicircleContactLocation2 {
        match (first, endpoint) {
            (true, BezierAlgebraicCuspSemicirclePairEndpoint2::FirstStart)
            | (false, BezierAlgebraicCuspSemicirclePairEndpoint2::SecondStart) => {
                BezierAlgebraicCuspSemicircleContactLocation2::Start
            }
            (true, BezierAlgebraicCuspSemicirclePairEndpoint2::FirstEnd)
            | (false, BezierAlgebraicCuspSemicirclePairEndpoint2::SecondEnd) => {
                BezierAlgebraicCuspSemicircleContactLocation2::End
            }
            _ => BezierAlgebraicCuspSemicircleContactLocation2::Interior,
        }
    }

    pub(super) fn endpoint_order_to_real(
        &self,
        endpoint: BezierAlgebraicCuspSemicirclePairEndpoint2,
        first: bool,
        parameter: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        if let BezierAlgebraicCuspSemicirclePairOverlapParameterMapData2::SimilarityTransport {
            source,
            ..
        } = &self.data.parameter_map
        {
            return source.endpoint_order_to_real(endpoint, first, parameter, policy);
        }
        let location = Self::endpoint_location(endpoint, first);
        if let Some(order) =
            algebraic_cusp_semicircle_endpoint_contact_order(location, parameter, policy)
        {
            return Ok(order);
        }
        match in_closed_unit_interval(parameter, policy) {
            Some(true) => {}
            Some(false) => return Err(CurveError::InvalidBezierParameter),
            None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }
        let one_minus = Real::one() - parameter;
        let denominator = &one_minus * &one_minus + parameter * parameter;
        match real_sign(&denominator, policy) {
            Some(RealSign::Positive) => {}
            Some(RealSign::Zero | RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "semicircle overlap parameter denominator was not positive".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let endpoint_sign = match (first, endpoint) {
            (true, BezierAlgebraicCuspSemicirclePairEndpoint2::SecondStart)
            | (false, BezierAlgebraicCuspSemicirclePairEndpoint2::FirstStart) => Real::one(),
            (true, BezierAlgebraicCuspSemicirclePairEndpoint2::SecondEnd)
            | (false, BezierAlgebraicCuspSemicirclePairEndpoint2::FirstEnd) => Real::from(-1_i8),
            _ => {
                return Err(CurveError::Topology(
                    "semicircle overlap interior boundary named its own endpoint".into(),
                ));
            }
        };
        let radial_coefficient = Real::one() - Real::from(2_i8) * parameter;
        let sign = match &self.data.parameter_map {
            BezierAlgebraicCuspSemicirclePairOverlapParameterMapData2::Represented {
                radial_dot,
                first_radius_squared,
                second_radius_squared,
                ..
            } => {
                let radius_squared = if first {
                    first_radius_squared
                } else {
                    second_radius_squared
                };
                let scale = &denominator * &endpoint_sign;
                let offset = -(&radial_coefficient * radius_squared);
                match represented_affine_coordinate(&[(radial_dot, &scale)], &offset) {
                    Classification::Decided(predicate) => {
                        represented_policy_sign(&predicate, policy)
                    }
                    Classification::Uncertain(reason) => Classification::Uncertain(reason),
                }
            }
            BezierAlgebraicCuspSemicirclePairOverlapParameterMapData2::ExactEndpoints {
                ..
            }
            | BezierAlgebraicCuspSemicirclePairOverlapParameterMapData2::SimilarityTransport {
                ..
            } => {
                return Err(CurveError::Topology(
                    "full semicircle overlap requested an interior parameter map".into(),
                ));
            }
        };
        Ok(match sign {
            Classification::Decided(RealSign::Positive) => {
                Classification::Decided(std::cmp::Ordering::Less)
            }
            Classification::Decided(RealSign::Negative) => {
                Classification::Decided(std::cmp::Ordering::Greater)
            }
            Classification::Decided(RealSign::Zero) => {
                Classification::Decided(std::cmp::Ordering::Equal)
            }
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        })
    }

    pub(super) fn mapped_exact_parameter_order_to_real(
        &self,
        source: &Real,
        source_first: bool,
        target: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        if let BezierAlgebraicCuspSemicirclePairOverlapParameterMapData2::SimilarityTransport {
            source: overlap,
            ..
        } = &self.data.parameter_map
        {
            return overlap.mapped_exact_parameter_order_to_real(
                source,
                source_first,
                target,
                policy,
            );
        }
        for parameter in [source, target] {
            match in_closed_unit_interval(parameter, policy) {
                Some(true) => {}
                Some(false) => return Err(CurveError::InvalidBezierParameter),
                None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
            }
        }
        let source_one_minus = Real::one() - source;
        let source_denominator = &source_one_minus * &source_one_minus + source * source;
        let target_one_minus = Real::one() - target;
        let target_denominator = &target_one_minus * &target_one_minus + target * target;
        for denominator in [&source_denominator, &target_denominator] {
            match real_sign(denominator, policy) {
                Some(RealSign::Positive) => {}
                Some(RealSign::Zero | RealSign::Negative) => {
                    return Err(CurveError::Topology(
                        "semicircle overlap map denominator was not positive".into(),
                    ));
                }
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            }
        }

        let first_clockwise = match &self.data.parameter_map {
            BezierAlgebraicCuspSemicirclePairOverlapParameterMapData2::Represented {
                first_clockwise,
                ..
            } => *first_clockwise,
            BezierAlgebraicCuspSemicirclePairOverlapParameterMapData2::ExactEndpoints {
                ..
            }
            | BezierAlgebraicCuspSemicirclePairOverlapParameterMapData2::SimilarityTransport {
                ..
            } => {
                let mapped = if self.data.orientation == CurveOverlapOrientation2::Same {
                    source.clone()
                } else {
                    Real::one() - source
                };
                return Ok(compare_reals(&mapped, target, policy)
                    .map(Classification::Decided)
                    .unwrap_or(Classification::Uncertain(UncertaintyReason::Ordering)));
            }
        };
        let source_clockwise = if source_first {
            first_clockwise
        } else {
            first_clockwise ^ (self.data.orientation == CurveOverlapOrientation2::Reversed)
        };
        let source_turn = if source_clockwise { -1_i8 } else { 1_i8 };
        let cross_direction = if source_first {
            source_turn
        } else {
            -source_turn
        };
        let source_radial = Real::one() - Real::from(2_i8) * source;
        let source_tangent =
            Real::from(cross_direction) * Real::from(2_i8) * source * &source_one_minus;
        let target_radial = Real::one() - Real::from(2_i8) * target;
        let radial_dot_scale = &source_radial * &target_denominator;
        let radial_cross_scale = &source_tangent * &target_denominator;
        let radius_scale = &source_denominator * &target_radial;
        let sign = match &self.data.parameter_map {
            BezierAlgebraicCuspSemicirclePairOverlapParameterMapData2::Represented {
                radial_dot,
                radial_cross,
                first_radius_squared,
                second_radius_squared,
                ..
            } => {
                let target_radius_squared = if source_first {
                    second_radius_squared
                } else {
                    first_radius_squared
                };
                let offset = -(target_radius_squared * &radius_scale);
                match represented_affine_coordinate(
                    &[
                        (radial_dot, &radial_dot_scale),
                        (radial_cross, &radial_cross_scale),
                    ],
                    &offset,
                ) {
                    Classification::Decided(predicate) => {
                        represented_policy_sign(&predicate, policy)
                    }
                    Classification::Uncertain(reason) => Classification::Uncertain(reason),
                }
            }
            BezierAlgebraicCuspSemicirclePairOverlapParameterMapData2::ExactEndpoints {
                ..
            }
            | BezierAlgebraicCuspSemicirclePairOverlapParameterMapData2::SimilarityTransport {
                ..
            } => unreachable!("exact overlap maps returned before angular predicate replay"),
        };
        Ok(match sign {
            Classification::Decided(RealSign::Positive) => {
                Classification::Decided(std::cmp::Ordering::Less)
            }
            Classification::Decided(RealSign::Negative) => {
                Classification::Decided(std::cmp::Ordering::Greater)
            }
            Classification::Decided(RealSign::Zero) => {
                Classification::Decided(std::cmp::Ordering::Equal)
            }
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        })
    }

    pub(super) fn mapped_parameter_order_to_real(
        &self,
        source: &BezierAlgebraicCuspSemicircleParameter2,
        source_first: bool,
        target: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        if let BezierAlgebraicCuspSemicircleParameter2::Exact(source) = source {
            return self.mapped_exact_parameter_order_to_real(source, source_first, target, policy);
        }
        match in_closed_unit_interval(target, policy) {
            Some(true) => {}
            Some(false) => return Err(CurveError::InvalidBezierParameter),
            None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }
        // A partial coincident-circle overlap does not define the inverse map
        // outside its retained target range. In particular, mapping target
        // endpoint 0 or 1 back through that partial overlap can select the
        // antipodal source point and invert the global range test. Refine the
        // already-authoritative forward image instead. This proves strict
        // interior cuts are inside [0, 1] without adjoining the source field
        // to either selected-circle field.
        if compare_reals(target, &Real::zero(), policy) == Some(std::cmp::Ordering::Equal)
            || compare_reals(target, &Real::one(), policy) == Some(std::cmp::Ordering::Equal)
        {
            for refinement_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
                let bracket = match self.mapped_parameter_bracket(
                    source,
                    source_first,
                    refinement_steps,
                    policy,
                )? {
                    Classification::Decided(bracket) => bracket,
                    Classification::Uncertain(_) => continue,
                };
                let (start, end) = cusp_semicircle_parameter_bracket_bounds(&bracket);
                if compare_reals(end, target, policy) == Some(std::cmp::Ordering::Less) {
                    return Ok(Classification::Decided(std::cmp::Ordering::Less));
                }
                if compare_reals(start, target, policy) == Some(std::cmp::Ordering::Greater) {
                    return Ok(Classification::Decided(std::cmp::Ordering::Greater));
                }
                if compare_reals(start, end, policy) == Some(std::cmp::Ordering::Equal)
                    && compare_reals(start, target, policy) == Some(std::cmp::Ordering::Equal)
                {
                    return Ok(Classification::Decided(std::cmp::Ordering::Equal));
                }
            }
            return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
        }
        let inverse = self.map_parameter(
            &BezierAlgebraicCuspSemicircleParameter2::Exact(target.clone()),
            !source_first,
        );
        Ok(source.cmp_by_refinement(&inverse, policy)?.map(|order| {
            if self.data.orientation == CurveOverlapOrientation2::Same {
                order
            } else {
                order.reverse()
            }
        }))
    }

    pub(super) fn mapped_parameter_bracket(
        &self,
        source: &BezierAlgebraicCuspSemicircleParameter2,
        source_first: bool,
        refinement_steps: usize,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicircleParameterBracket2>> {
        let source_bracket = match source.parameter_bracket(refinement_steps, policy)? {
            Classification::Decided(bracket) => bracket,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if self.has_exact_endpoint_map() {
            return if self.data.orientation == CurveOverlapOrientation2::Same {
                Ok(Classification::Decided(source_bracket))
            } else {
                complement_cusp_parameter_bracket(source_bracket, policy)
            };
        }
        let mapped_bracket = |source: &Real| {
            refine_algebraic_cusp_semicircle_parameter_bracket(None, refinement_steps, |target| {
                self.mapped_exact_parameter_order_to_real(source, source_first, target, policy)
            })
        };
        let (source_start, source_end) = cusp_semicircle_parameter_bracket_bounds(&source_bracket);
        let first = match mapped_bracket(source_start)? {
            Classification::Decided(bracket) => bracket,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if source_start == source_end {
            return Ok(Classification::Decided(first));
        }
        let second = match mapped_bracket(source_end)? {
            Classification::Decided(bracket) => bracket,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let (first_start, first_end) = cusp_semicircle_parameter_bracket_bounds(&first);
        let (second_start, second_end) = cusp_semicircle_parameter_bracket_bounds(&second);
        let start = match compare_reals(first_start, second_start, policy) {
            Some(std::cmp::Ordering::Greater) => second_start.clone(),
            Some(std::cmp::Ordering::Equal | std::cmp::Ordering::Less) => first_start.clone(),
            None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        };
        let end = match compare_reals(first_end, second_end, policy) {
            Some(std::cmp::Ordering::Less) => second_end.clone(),
            Some(std::cmp::Ordering::Equal | std::cmp::Ordering::Greater) => first_end.clone(),
            None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        };
        match compare_reals(&start, &end, policy) {
            Some(std::cmp::Ordering::Equal) => {
                return Ok(Classification::Decided(
                    BezierAlgebraicCuspSemicircleParameterBracket2::Exact(start),
                ));
            }
            Some(std::cmp::Ordering::Less) => {}
            Some(std::cmp::Ordering::Greater) => {
                return Err(CurveError::Topology(
                    "semicircle overlap map produced an inverted bracket".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }
        Ok(
            match BezierParameterInterval::try_new_with_policy(start, end, policy)? {
                Classification::Decided(interval) => Classification::Decided(
                    BezierAlgebraicCuspSemicircleParameterBracket2::Interval(interval),
                ),
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
        )
    }
}

impl BezierAlgebraicCuspSemicircleParallelContact2 {
    pub(crate) fn retained_cusp_parameter(
        &self,
    ) -> Option<BezierAlgebraicCuspSemicircleParameter2> {
        match &self.correlation {
            BezierAlgebraicCuspSemicircleParallelCorrelation2::Retained(parameter) => {
                Some(parameter.clone())
            }
            _ => self.location.endpoint_parameter(),
        }
    }
}

impl BezierAlgebraicCuspSemicircleParallelParameterMap2 {
    pub(super) fn tangent_cross_dot_source_sign(
        &self,
        contact: &BezierAlgebraicCuspSemicircleParallelContact2,
        cross_scale: &Real,
        dot_scale: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        if !policy.accepts_retained_policy(self.data.policy) {
            return Err(CurveError::Topology(
                "a pair-radial parallel tangent map crossed predicate policies".into(),
            ));
        }
        match &self.data.system {
            BezierAlgebraicCuspSemicircleParallelParameterMapSystem2::Represented { system } => {
                system.tangent_cross_dot_source_sign(
                    &contact.parallel_parameter,
                    cross_scale,
                    dot_scale,
                    policy,
                )
            }
            BezierAlgebraicCuspSemicircleParallelParameterMapSystem2::Recursive { system } => {
                system.tangent_cross_dot_source_sign(
                    &CurveParameter2::from(contact.parallel_parameter.clone()),
                    cross_scale,
                    dot_scale,
                    policy,
                )
            }
            BezierAlgebraicCuspSemicircleParallelParameterMapSystem2::OneField { .. } => {
                Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
            }
        }
    }

    /// Identifies a rebuilt copy of the same native-parameter-to-cusp map.
    pub(super) fn shares_parameterization(&self, other: &Self) -> bool {
        if self.data.policy != other.data.policy {
            return false;
        }
        match (&self.data.system, &other.data.system) {
            (
                BezierAlgebraicCuspSemicircleParallelParameterMapSystem2::OneField {
                    cusp_parameter: first_parameter,
                    incidence: first_incidence,
                    diameter: first_diameter,
                    radius_squared_denominator: first_radius,
                    speed_squared: first_speed,
                },
                BezierAlgebraicCuspSemicircleParallelParameterMapSystem2::OneField {
                    cusp_parameter: second_parameter,
                    incidence: second_incidence,
                    diameter: second_diameter,
                    radius_squared_denominator: second_radius,
                    speed_squared: second_speed,
                },
            ) => {
                first_parameter == second_parameter
                    && first_incidence == second_incidence
                    && first_diameter == second_diameter
                    && first_radius == second_radius
                    && first_speed == second_speed
            }
            (
                BezierAlgebraicCuspSemicircleParallelParameterMapSystem2::Recursive {
                    system: first,
                },
                BezierAlgebraicCuspSemicircleParallelParameterMapSystem2::Recursive {
                    system: second,
                },
            ) => {
                Arc::ptr_eq(first, second)
                    || (self.data.semicircle == other.data.semicircle
                        && self.data.parallel == other.data.parallel)
            }
            (
                BezierAlgebraicCuspSemicircleParallelParameterMapSystem2::Represented {
                    system: first,
                },
                BezierAlgebraicCuspSemicircleParallelParameterMapSystem2::Represented {
                    system: second,
                },
            ) => Arc::ptr_eq(first, second),
            _ => false,
        }
    }

    pub(super) fn parameterization_orientation(
        &self,
        other: &Self,
    ) -> Option<CurveOverlapOrientation2> {
        if Arc::ptr_eq(&self.data, &other.data) || self.shares_parameterization(other) {
            return Some(CurveOverlapOrientation2::Same);
        }
        let (
            BezierAlgebraicCuspSemicircleParallelParameterMapSystem2::OneField {
                cusp_parameter: first_parameter,
                incidence: first_incidence,
                diameter: first_diameter,
                radius_squared_denominator: first_radius,
                speed_squared: first_speed,
            },
            BezierAlgebraicCuspSemicircleParallelParameterMapSystem2::OneField {
                cusp_parameter: second_parameter,
                incidence: second_incidence,
                diameter: second_diameter,
                radius_squared_denominator: second_radius,
                speed_squared: second_speed,
            },
        ) = (&self.data.system, &other.data.system)
        else {
            return None;
        };
        (first_parameter == second_parameter
            && first_incidence == &bivariate_complement_second_parameter(second_incidence)
            && first_diameter.rational
                == bivariate_complement_second_parameter(&second_diameter.rational)
            && first_diameter.radical
                == bivariate_complement_second_parameter(&second_diameter.radical)
            && first_radius == &bivariate_complement_second_parameter(second_radius)
            && first_speed == &bivariate_complement_second_parameter(second_speed))
            .then_some(CurveOverlapOrientation2::Reversed)
    }

    /// Orders one analytic-parallel contact against a represented semicircle parameter.
    pub(super) fn contact_order_to_real(
        &self,
        contact: &BezierAlgebraicCuspSemicircleParallelContact2,
        parameter: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        if let BezierAlgebraicCuspSemicircleParallelCorrelation2::Retained(retained) =
            &contact.correlation
        {
            return retained.order_to_real(parameter, policy);
        }
        if let Some(order) =
            algebraic_cusp_semicircle_endpoint_contact_order(contact.location, parameter, policy)
        {
            return Ok(order);
        }
        match in_closed_unit_interval(parameter, policy) {
            Some(true) => {}
            Some(false) => return Err(CurveError::InvalidBezierParameter),
            None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }
        let one_minus = Real::one() - parameter;
        let denominator = &one_minus * &one_minus + parameter * parameter;
        match real_sign(&denominator, policy) {
            Some(RealSign::Positive) => {}
            Some(RealSign::Zero | RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "semicircle parameter-order denominator was not positive".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let radial_coefficient = Real::one() - Real::from(2_i8) * parameter;
        let sign = match &self.data.system {
            BezierAlgebraicCuspSemicircleParallelParameterMapSystem2::OneField {
                cusp_parameter,
                incidence,
                diameter,
                radius_squared_denominator,
                speed_squared,
            } => {
                let predicate = BezierAlgebraicCuspTwoTermExpression2 {
                    rational: bivariate_scaled_difference(
                        &diameter.rational,
                        &denominator,
                        radius_squared_denominator,
                        &radial_coefficient,
                    ),
                    radical: bivariate_scale(diameter.radical.clone(), &denominator),
                };
                if matches!(
                    contact.correlation,
                    BezierAlgebraicCuspSemicircleParallelCorrelation2::Map
                ) {
                    algebraic_cusp_correlated_radical_sum_sign(
                        incidence,
                        &predicate,
                        speed_squared,
                        cusp_parameter,
                        &contact.parallel_parameter,
                        policy,
                    )?
                } else {
                    algebraic_cusp_independent_radical_sum_sign(
                        &predicate,
                        speed_squared,
                        cusp_parameter,
                        &contact.parallel_parameter,
                        policy,
                    )?
                }
            }
            BezierAlgebraicCuspSemicircleParallelParameterMapSystem2::Represented { system } => {
                system.diameter_parameter_sign(
                    &contact.parallel_parameter,
                    &denominator,
                    &radial_coefficient,
                    policy,
                )?
            }
            BezierAlgebraicCuspSemicircleParallelParameterMapSystem2::Recursive { system } => {
                system.diameter_parameter_sign(
                    &CurveParameter2::from(contact.parallel_parameter.clone()),
                    &denominator,
                    &radial_coefficient,
                    policy,
                )?
            }
        };
        Ok(match sign {
            // The normalized diameter coordinate decreases strictly with u.
            Classification::Decided(RealSign::Positive) => {
                Classification::Decided(std::cmp::Ordering::Less)
            }
            Classification::Decided(RealSign::Negative) => {
                Classification::Decided(std::cmp::Ordering::Greater)
            }
            Classification::Decided(RealSign::Zero) => {
                Classification::Decided(std::cmp::Ordering::Equal)
            }
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        })
    }

    /// Refines one analytic-parallel contact to an exact witness or rational bracket.
    pub(super) fn contact_parameter_bracket(
        &self,
        contact: &BezierAlgebraicCuspSemicircleParallelContact2,
        refinement_steps: usize,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicircleParameterBracket2>> {
        let cached = self.data.parameter_cache.cached_parameter_bracket(
            &contact.parallel_parameter,
            contact.location,
            policy,
        );
        if let Some((completed_steps, bracket)) = &cached
            && *completed_steps >= refinement_steps
        {
            return Ok(Classification::Decided(bracket.clone()));
        }
        let completed_steps = cached.as_ref().map_or(0, |(steps, _)| *steps);
        let bracket = refine_algebraic_cusp_semicircle_parameter_bracket(
            cached.as_ref().map(|(_, bracket)| bracket),
            refinement_steps.saturating_sub(completed_steps),
            |parameter| self.contact_order_to_real(contact, parameter, policy),
        )?;
        drop(cached);
        if let Classification::Decided(bracket) = &bracket {
            self.data.parameter_cache.retain_parameter_bracket(
                contact.parallel_parameter.clone(),
                contact.location,
                refinement_steps,
                bracket.clone(),
                policy,
            );
        }
        Ok(bracket)
    }

    pub(crate) fn contact_parameter(
        &self,
        contact: &BezierAlgebraicCuspSemicircleParallelContact2,
    ) -> BezierAlgebraicCuspSemicircleParameter2 {
        contact.retained_cusp_parameter().unwrap_or_else(|| {
            BezierAlgebraicCuspSemicircleParameter2::Mapped(Arc::new(
                BezierAlgebraicCuspSemicircleMappedParameterData2::Parallel {
                    map: self.clone(),
                    contact: contact.clone(),
                },
            ))
        })
    }

    /// Retains a caller-certified interior tangency without solving the same
    /// circle/parallel intersection system a second time.
    ///
    /// The fillet kernel calls this only after its radius-offset intersection
    /// has selected `parallel_parameter`, constructed the circle from that
    /// center, and proved both trim contacts are distinct. The shared map keeps
    /// the exact incidence relation needed by later point and order predicates.
    pub(crate) fn certified_interior_tangent_parameter(
        &self,
        parallel_parameter: BezierParameter2,
    ) -> BezierAlgebraicCuspSemicircleParameter2 {
        self.contact_parameter(&BezierAlgebraicCuspSemicircleParallelContact2 {
            parallel_parameter,
            tangent_cross_sign: Some(RealSign::Zero),
            location: BezierAlgebraicCuspSemicircleContactLocation2::Interior,
            correlation: BezierAlgebraicCuspSemicircleParallelCorrelation2::Map,
        })
    }
}

impl BezierAlgebraicCuspSemicircleParallelParameterMapData2 {
    pub(super) fn one_field_system(
        &self,
    ) -> Option<(
        &BezierParameter2,
        &BivariatePolynomial,
        &BezierAlgebraicCuspTwoTermExpression2,
        &BivariatePolynomial,
        &BivariatePolynomial,
    )> {
        let BezierAlgebraicCuspSemicircleParallelParameterMapSystem2::OneField {
            cusp_parameter,
            incidence,
            diameter,
            radius_squared_denominator,
            speed_squared,
        } = &self.system
        else {
            return None;
        };
        Some((
            cusp_parameter,
            incidence,
            diameter,
            radius_squared_denominator,
            speed_squared,
        ))
    }
}

impl BezierAlgebraicCuspSemicircleParameterCache2 {
    pub(super) fn cached_represented_diameter_coordinate(
        &self,
        parameter: &BezierParameter2,
        policy: &CurveContext,
    ) -> Option<AlgebraicRootRepresentation> {
        self.entries.lock().expect("cusp parameter cache mutex poisoned").iter().find_map(|entry| match entry {
            BezierAlgebraicCuspSemicircleParameterCacheEntry2::RepresentedDiameterCoordinate { parameter: cached, evidence }
                if cached == parameter
                    && (!evidence.1.selects_approximate_512() || policy.permits_approximate_512()) => {
                if evidence.1.selects_approximate_512() {
                    policy.observe_approximate_512();
                }
                Some(evidence.0.clone())
            }
            _ => None,
        })
    }

    pub(super) fn retain_represented_diameter_coordinate(
        &self,
        parameter: BezierParameter2,
        coordinate: AlgebraicRootRepresentation,
        policy: &CurveContext,
    ) {
        let retained_policy = policy.retained_object_policy();
        let mut cache = self
            .entries
            .lock()
            .expect("cusp parameter cache mutex poisoned");
        for entry in cache.iter_mut() {
            let BezierAlgebraicCuspSemicircleParameterCacheEntry2::RepresentedDiameterCoordinate {
                parameter: cached,
                evidence,
            } = entry
            else {
                continue;
            };
            if cached != &parameter {
                continue;
            }
            if evidence.1.selects_approximate_512() && !retained_policy.selects_approximate_512() {
                **evidence = (coordinate, retained_policy);
            }
            return;
        }
        cache.push(
            BezierAlgebraicCuspSemicircleParameterCacheEntry2::RepresentedDiameterCoordinate {
                parameter,
                evidence: Box::new((coordinate, retained_policy)),
            },
        );
    }

    pub(super) fn cached_parameter_bracket(
        &self,
        parameter: &BezierParameter2,
        location: BezierAlgebraicCuspSemicircleContactLocation2,
        policy: &CurveContext,
    ) -> Option<(usize, BezierAlgebraicCuspSemicircleParameterBracket2)> {
        self.entries
            .lock()
            .expect("cusp parameter cache mutex poisoned")
            .iter()
            .filter_map(|entry| match entry {
                BezierAlgebraicCuspSemicircleParameterCacheEntry2::ParameterBracket {
                    parameter: cached,
                    evidence,
                } if cached == parameter
                    && evidence.location == location
                    && policy.accepts_retained_policy(evidence.policy)
                    && (!evidence.policy.selects_approximate_512()
                        || policy.permits_approximate_512()) =>
                {
                    Some(evidence.as_ref())
                }
                _ => None,
            })
            .max_by_key(|evidence| {
                (
                    evidence.refinement_steps,
                    !evidence.policy.selects_approximate_512(),
                )
            })
            .map(|evidence| {
                if evidence.policy.selects_approximate_512() {
                    policy.observe_approximate_512();
                }
                (evidence.refinement_steps, evidence.bracket.clone())
            })
    }

    pub(super) fn retain_parameter_bracket(
        &self,
        parameter: BezierParameter2,
        location: BezierAlgebraicCuspSemicircleContactLocation2,
        refinement_steps: usize,
        bracket: BezierAlgebraicCuspSemicircleParameterBracket2,
        policy: &CurveContext,
    ) {
        let retained_policy = policy.retained_object_policy();
        let new_evidence = BezierAlgebraicCuspSemicircleCachedParameterBracket2 {
            location,
            refinement_steps,
            bracket,
            policy: retained_policy,
        };
        let mut cache = self
            .entries
            .lock()
            .expect("cusp parameter cache mutex poisoned");
        for entry in cache.iter_mut() {
            let BezierAlgebraicCuspSemicircleParameterCacheEntry2::ParameterBracket {
                parameter: cached,
                evidence,
            } = entry
            else {
                continue;
            };
            if cached == &parameter
                && evidence.location == location
                && evidence.policy == retained_policy
            {
                if evidence.refinement_steps < refinement_steps {
                    **evidence = new_evidence;
                }
                return;
            }
        }
        cache.push(
            BezierAlgebraicCuspSemicircleParameterCacheEntry2::ParameterBracket {
                parameter,
                evidence: Box::new(new_evidence),
            },
        );
    }

    pub(super) fn cached_scalar_value(
        &self,
        parameter: &BezierParameter2,
        policy: &CurveContext,
    ) -> Option<Option<Real>> {
        self.entries
            .lock()
            .expect("cusp parameter cache mutex poisoned")
            .iter()
            .find_map(|entry| match entry {
                BezierAlgebraicCuspSemicircleParameterCacheEntry2::ScalarValue {
                    parameter: cached,
                    value,
                } if cached == parameter => Some(value.clone()),
                BezierAlgebraicCuspSemicircleParameterCacheEntry2::Approximate512ScalarValue {
                    parameter: cached,
                    value,
                } if cached == parameter && policy.permits_approximate_512() => {
                    policy.observe_approximate_512();
                    Some(value.as_ref().clone())
                }
                _ => None,
            })
    }

    pub(super) fn retain_scalar_value(
        &self,
        parameter: BezierParameter2,
        value: Option<Real>,
        policy: &CurveContext,
    ) {
        let approximate = policy.retained_object_policy().selects_approximate_512();
        let mut cache = self
            .entries
            .lock()
            .expect("cusp parameter cache mutex poisoned");
        let index = cache.iter().position(|entry| match entry {
            BezierAlgebraicCuspSemicircleParameterCacheEntry2::ScalarValue {
                parameter: cached,
                ..
            }
            | BezierAlgebraicCuspSemicircleParameterCacheEntry2::Approximate512ScalarValue {
                parameter: cached,
                ..
            } => cached == &parameter,
            _ => false,
        });
        if approximate
            && index.is_some_and(|index| {
                matches!(
                    &cache[index],
                    BezierAlgebraicCuspSemicircleParameterCacheEntry2::ScalarValue { .. }
                )
            })
        {
            return;
        }
        let entry = if approximate {
            BezierAlgebraicCuspSemicircleParameterCacheEntry2::Approximate512ScalarValue {
                parameter,
                value: Box::new(value),
            }
        } else {
            BezierAlgebraicCuspSemicircleParameterCacheEntry2::ScalarValue { parameter, value }
        };
        if let Some(index) = index {
            cache[index] = entry;
        } else {
            cache.push(entry);
        }
    }

    pub(super) fn retained_cusp_parameter(
        &self,
        parameter: &CurveParameter2,
        policy: &CurveContext,
    ) -> Option<Option<BezierAlgebraicCuspSemicircleParameter2>> {
        let (cusp, retained_policy) = self
            .entries
            .lock()
            .expect("cusp parameter cache mutex poisoned")
            .iter()
            .find_map(|entry| match entry {
                BezierAlgebraicCuspSemicircleParameterCacheEntry2::RetainedCusp {
                    parameter: cached,
                    cusp,
                    policy: retained_policy,
                } if cached == parameter
                    && policy.accepts_retained_policy(*retained_policy)
                    && (!retained_policy.selects_approximate_512()
                        || policy.permits_approximate_512()) =>
                {
                    Some((cusp.clone(), *retained_policy))
                }
                _ => None,
            })?;
        if retained_policy.selects_approximate_512() {
            policy.observe_approximate_512();
        }
        Some(cusp.upgrade().and_then(|cusp| {
            let cusp = BezierAlgebraicCuspSemicircleParameter2::Mapped(cusp);
            cusp.validate_policy(policy).is_ok().then_some(cusp)
        }))
    }

    pub(super) fn retain_cusp_parameter(
        &self,
        parameter: CurveParameter2,
        cusp: &BezierAlgebraicCuspSemicircleParameter2,
        policy: &CurveContext,
    ) {
        let BezierAlgebraicCuspSemicircleParameter2::Mapped(mapped) = cusp else {
            return;
        };
        if cusp.validate_policy(policy).is_err() {
            return;
        }
        let retained_policy = if cusp.validate_policy(&policy.strict_counterpart()).is_ok() {
            policy.retained_object_policy()
        } else {
            *policy
        };
        let mut cache = self
            .entries
            .lock()
            .expect("cusp parameter cache mutex poisoned");
        let index = cache.iter().position(|entry| matches!(entry,
            BezierAlgebraicCuspSemicircleParameterCacheEntry2::RetainedCusp { parameter: cached, .. } if cached == &parameter
        ));
        if retained_policy.selects_approximate_512() && index.is_some_and(|index| matches!(&cache[index],
            BezierAlgebraicCuspSemicircleParameterCacheEntry2::RetainedCusp { policy, .. } if !policy.selects_approximate_512()
        )) { return; }
        let entry = BezierAlgebraicCuspSemicircleParameterCacheEntry2::RetainedCusp {
            parameter,
            cusp: Arc::downgrade(mapped),
            policy: retained_policy,
        };
        if let Some(index) = index {
            cache[index] = entry;
        } else {
            cache.push(entry);
        }
    }
}

impl BezierAlgebraicCuspSemicircleRationalDiameter2 {
    pub(super) fn is_second_complement_of(&self, other: &Self) -> bool {
        match (self, other) {
            (Self::Rational(first), Self::Rational(second)) => {
                first == &bivariate_complement_second_parameter(second)
            }
            (
                Self::ParallelNormal {
                    coordinate: first,
                    speed_squared: first_speed,
                },
                Self::ParallelNormal {
                    coordinate: second,
                    speed_squared: second_speed,
                },
            ) => {
                first.rational == bivariate_complement_second_parameter(&second.rational)
                    && first.radical == bivariate_complement_second_parameter(&second.radical)
                    && first_speed == &bivariate_complement_second_parameter(second_speed)
            }
            (Self::Rational(_), Self::ParallelNormal { .. })
            | (Self::ParallelNormal { .. }, Self::Rational(_)) => false,
        }
    }
}

impl BezierAlgebraicCuspSemicircleRationalParameterMap2 {
    /// Recovers a contact parameter retained by the exact circle/chord
    /// adapter. Restrict this bridge to chord-backed evidence: the same cache
    /// also records inverse overlap mappings, which may point back into this
    /// rational map and must not recursively replay themselves.
    pub(super) fn retained_chord_cusp_parameter(
        &self,
        parameter: &CurveParameter2,
        policy: &CurveContext,
    ) -> Option<BezierAlgebraicCuspSemicircleParameter2> {
        let cusp = self
            .data
            .parameter_cache
            .retained_cusp_parameter(parameter, policy)??;
        matches!(
            &cusp,
            BezierAlgebraicCuspSemicircleParameter2::Mapped(data)
                if matches!(
                    data.as_ref(),
                    BezierAlgebraicCuspSemicircleMappedParameterData2::Chord { .. }
                )
        )
        .then_some(cusp)
    }

    /// Replays one exact angular linear form at a retained rational contact.
    ///
    /// The ordinary one-field intersection map already owns the correlated
    /// circle/rational incidence. Rebuilding only the two tangent
    /// polynomials and reducing them into that same field is substantially
    /// smaller than a second circle-pair solve, and preserves the contact's
    /// original root correlation for recursive round construction.
    pub(super) fn tangent_cross_dot_linear_combination_sign(
        &self,
        contact: &BezierAlgebraicCuspSemicircleRationalMapContact2,
        cross_scale: &Real,
        dot_scale: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        if !policy.accepts_retained_policy(self.data.policy) {
            return Err(CurveError::Topology(
                "a rational circle-contact tangent map crossed predicate policies".into(),
            ));
        }
        let BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::OneField {
            cusp_parameter,
            incidence,
            ..
        } = &self.data.system
        else {
            return self.selected_radial_tangent_cross_dot_linear_combination_sign(
                contact,
                cross_scale,
                dot_scale,
                policy,
            );
        };
        let system = self.data.semicircle.rational_system(&self.data.curve)?;
        let reduce = |polynomial: &BivariatePolynomial| {
            bivariate_reduce_axis(
                polynomial,
                self.data.semicircle.cusp_parameter().polynomial(),
                CurveResultantParameter::First,
                policy,
            )
        };
        let tangent_cross = match reduce(&system.tangent_cross)? {
            Classification::Decided(polynomial) => polynomial,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let angular_tangent = match reduce(&system.angular_tangent)? {
            Classification::Decided(polynomial) => polynomial,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        // The stored angular expression is the circle radius crossed with
        // the rational tangent. Convert it to the circle-tangent dot product
        // with the same convention as the selected-fiber map.
        let tangent_dot_scale = dot_scale * self.data.semicircle.turn_sign();
        let predicate = bivariate_add(
            &bivariate_scale(tangent_cross, cross_scale),
            &bivariate_scale(angular_tangent, &tangent_dot_scale),
        );
        let correlated_incidence = match &contact.correlation {
            BezierAlgebraicCuspSemicircleRationalCorrelation2::Map
            | BezierAlgebraicCuspSemicircleRationalCorrelation2::MapWithChordTangent { .. } => {
                Some(incidence)
            }
            BezierAlgebraicCuspSemicircleRationalCorrelation2::Independent => None,
            BezierAlgebraicCuspSemicircleRationalCorrelation2::Relation(incidence) => {
                Some(incidence.as_ref())
            }
        };
        bivariate_sign_at_cusp_and_region_parameter(
            &predicate,
            cusp_parameter,
            &contact.other_parameter,
            correlated_incidence,
            policy,
        )
    }

    /// Replays one exact linear combination of the selected-circle tangent
    /// crossed and dotted with a retained rational-line tangent.
    ///
    /// A pair-radial circle already constructs both three-axis expressions
    /// while solving the contact. Retaining them in the parameter map lets a
    /// recursively authored fillet reuse that same root correlation instead
    /// of invoking a circle-pair resultant around the selected center.
    pub(super) fn selected_radial_tangent_cross_dot_linear_combination_sign(
        &self,
        contact: &BezierAlgebraicCuspSemicircleRationalMapContact2,
        cross_scale: &Real,
        dot_scale: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        if !policy.accepts_retained_policy(self.data.policy) {
            return Err(CurveError::Topology(
                "a pair-radial rational tangent map crossed predicate policies".into(),
            ));
        }
        if self.data.curve.degree() != 1
            || !matches!(
                contact.correlation,
                BezierAlgebraicCuspSemicircleRationalCorrelation2::MapWithChordTangent { .. }
            )
        {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        if let Some(BezierAlgebraicCuspSemicircleParameter2::Mapped(data)) =
            self.retained_chord_cusp_parameter(&contact.other_parameter, policy)
            && let BezierAlgebraicCuspSemicircleMappedParameterData2::Chord {
                map,
                contact: chord_contact,
            } = data.as_ref()
            && let BezierAlgebraicCuspSemicircleRationalCorrelation2::MapWithChordTangent {
                chord,
                circle_cross_chord,
            } = &contact.correlation
            && let Some(reversed) = map.data.chord.shared_tangent_orientation(chord)
        {
            let retained_cross = if reversed {
                product_sign(chord_contact.tangent_cross_sign, RealSign::Negative)
            } else {
                chord_contact.tangent_cross_sign
            };
            if retained_cross != *circle_cross_chord {
                return Err(CurveError::Topology(
                    "an exact-linear rational contact changed its retained tangent orientation"
                        .into(),
                ));
            }
            return Ok(map
                .retained_tangent_cross_dot_linear_combination_sign(
                    chord_contact,
                    cross_scale,
                    dot_scale,
                    policy,
                )?
                .map(|sign| {
                    if reversed {
                        product_sign(sign, RealSign::Negative)
                    } else {
                        sign
                    }
                }));
        }
        if let BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::Recursive { system } =
            &self.data.system
        {
            return system.tangent_cross_dot_source_sign(
                &contact.other_parameter,
                cross_scale,
                dot_scale,
                policy,
            );
        }
        let other_parameter =
            match promote_curve_region_bezier_parameter(&contact.other_parameter, policy)? {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::SelectedRadial {
            pair_map,
            branch,
            discriminant,
            tangent_cross,
            angular_tangent,
            ..
        } = &self.data.system
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some([first_parameter, second_parameter]) = pair_map.compact_source_parameters() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        // The retained angular expression is `cross(Q-C,Q')`. Under the
        // semicircle's traversal, dot(turn*J(Q-C), Q') is turn times this
        // expression, matching the contact's published tangent dot.
        let tangent_dot_scale = dot_scale * self.data.semicircle.turn_sign();
        let Some(rational) = TrivariatePolynomial::linear_combination(&[
            (&tangent_cross.rational, cross_scale),
            (&angular_tangent.rational, &tangent_dot_scale),
        ]) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some(radical) = TrivariatePolynomial::linear_combination(&[
            (&tangent_cross.radical, cross_scale),
            (&angular_tangent.radical, &tangent_dot_scale),
        ]) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        algebraic_cusp_trivariate_square_root_sum_sign(
            &SquareRootExpression { rational, radical },
            discriminant,
            &first_parameter,
            &second_parameter,
            &other_parameter,
            *branch,
            policy,
        )
    }

    /// Identifies a rebuilt copy of the same native-parameter-to-cusp map.
    /// Cache identity and contact correlation are deliberately excluded: the
    /// retained equations and policy completely define the monotone map.
    pub(super) fn shares_parameterization(&self, other: &Self) -> bool {
        if self.data.policy != other.data.policy {
            return false;
        }
        match (&self.data.system, &other.data.system) {
            (
                BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::OneField {
                    cusp_parameter: first_parameter,
                    incidence: first_incidence,
                    diameter: first_diameter,
                    radius_squared_denominator: first_radius,
                },
                BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::OneField {
                    cusp_parameter: second_parameter,
                    incidence: second_incidence,
                    diameter: second_diameter,
                    radius_squared_denominator: second_radius,
                },
            ) => {
                first_parameter == second_parameter
                    && first_incidence == second_incidence
                    && first_diameter == second_diameter
                    && first_radius == second_radius
            }
            // Rebuilt pair-radial systems deliberately keep allocation
            // identity as their compact authority.  Comparing dense three-
            // axis tensors here would enlarge a cold overlap optimization.
            (
                BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::SelectedRadial { .. },
                BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::SelectedRadial { .. },
            )
            | (
                BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::OneField { .. },
                BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::SelectedRadial { .. },
            )
            | (
                BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::SelectedRadial { .. },
                BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::OneField { .. },
            ) => false,
            (
                BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::Recursive {
                    system: first,
                },
                BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::Recursive {
                    system: second,
                },
            ) => self.data.curve == other.data.curve && Arc::ptr_eq(first, second),
            (BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::Recursive { .. }, _)
            | (_, BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::Recursive { .. }) => {
                false
            }
            (
                BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::Represented {
                    frame: first,
                },
                BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::Represented {
                    frame: second,
                },
            ) => self.data.curve == other.data.curve && first == second,
            (
                BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::ChordNormalProjective {
                    system: first,
                },
                BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::ChordNormalProjective {
                    system: second,
                },
            ) => self.data.curve == other.data.curve && Arc::ptr_eq(first, second),
            (
                BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::Represented { .. },
                BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::OneField { .. }
                | BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::SelectedRadial { .. },
            )
            | (
                BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::OneField { .. }
                | BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::SelectedRadial { .. },
                BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::Represented { .. },
            ) => false,
            (
                BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::ChordNormalProjective {
                    ..
                },
                _,
            )
            | (
                _,
                BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::ChordNormalProjective {
                    ..
                },
            ) => false,
        }
    }

    pub(super) fn parameterization_orientation(
        &self,
        other: &Self,
    ) -> Option<CurveOverlapOrientation2> {
        if Arc::ptr_eq(&self.data, &other.data) || self.shares_parameterization(other) {
            return Some(CurveOverlapOrientation2::Same);
        }
        let (
            BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::OneField {
                cusp_parameter: first_parameter,
                incidence: first_incidence,
                diameter: first_diameter,
                radius_squared_denominator: first_radius,
            },
            BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::OneField {
                cusp_parameter: second_parameter,
                incidence: second_incidence,
                diameter: second_diameter,
                radius_squared_denominator: second_radius,
            },
        ) = (&self.data.system, &other.data.system)
        else {
            return None;
        };
        (first_parameter == second_parameter
            && first_incidence == &bivariate_complement_second_parameter(second_incidence)
            && first_diameter.is_second_complement_of(second_diameter)
            && first_radius == &bivariate_complement_second_parameter(second_radius)
            && self.data.policy == other.data.policy)
            .then_some(CurveOverlapOrientation2::Reversed)
    }

    /// Orders one exact contact parameter against a represented parameter.
    pub(super) fn mapped_contact_order_to_real(
        &self,
        contact: &BezierAlgebraicCuspSemicircleRationalMapContact2,
        parameter: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        if let Some(order) =
            algebraic_cusp_semicircle_endpoint_contact_order(contact.location, parameter, policy)
        {
            return Ok(order);
        }
        if let Some(cusp) = self.retained_chord_cusp_parameter(&contact.other_parameter, policy) {
            return cusp.order_to_real(parameter, policy);
        }
        let represented = contact
            .other_parameter
            .as_bezier_parameter()
            .and_then(|parameter| {
                self.data
                    .parameter_cache
                    .cached_scalar_value(parameter, policy)
            })
            .flatten();
        if let Some(represented) = represented {
            return Ok(compare_reals(&represented, parameter, policy)
                .map(Classification::Decided)
                .unwrap_or(Classification::Uncertain(UncertaintyReason::Ordering)));
        }
        match in_closed_unit_interval(parameter, policy) {
            Some(true) => {}
            Some(false) => return Err(CurveError::InvalidBezierParameter),
            None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }
        let one_minus = Real::one() - parameter;
        let denominator = &one_minus * &one_minus + parameter * parameter;
        match real_sign(&denominator, policy) {
            Some(RealSign::Positive) => {}
            Some(RealSign::Zero | RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "semicircle parameter-order denominator was not positive".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let radial_coefficient = Real::one() - Real::from(2_i8) * parameter;
        if let BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::Recursive { system } =
            &self.data.system
        {
            let sign = system.diameter_parameter_sign(
                &contact.other_parameter,
                &denominator,
                &radial_coefficient,
                policy,
            )?;
            return Ok(match sign {
                Classification::Decided(RealSign::Positive) => {
                    Classification::Decided(std::cmp::Ordering::Less)
                }
                Classification::Decided(RealSign::Negative) => {
                    Classification::Decided(std::cmp::Ordering::Greater)
                }
                Classification::Decided(RealSign::Zero) => {
                    Classification::Decided(std::cmp::Ordering::Equal)
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            });
        }
        // Only branches without an in-field predicate need the global
        // promotion of the contact parameter.
        macro_rules! promoted_other {
            () => {
                match promote_curve_region_bezier_parameter(&contact.other_parameter, policy)? {
                    Classification::Decided(parameter) => parameter,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            };
        }
        let sign = match &self.data.system {
            BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::OneField {
                cusp_parameter,
                incidence,
                diameter,
                radius_squared_denominator,
            } => {
                let correlated_incidence = match &contact.correlation {
                    BezierAlgebraicCuspSemicircleRationalCorrelation2::Map
                    | BezierAlgebraicCuspSemicircleRationalCorrelation2::MapWithChordTangent {
                        ..
                    } => Some(incidence),
                    BezierAlgebraicCuspSemicircleRationalCorrelation2::Independent => None,
                    BezierAlgebraicCuspSemicircleRationalCorrelation2::Relation(incidence) => {
                        Some(incidence.as_ref())
                    }
                };
                match diameter {
                    BezierAlgebraicCuspSemicircleRationalDiameter2::Rational(diameter) => {
                        let predicate = bivariate_scaled_difference(
                            diameter,
                            &denominator,
                            radius_squared_denominator,
                            &radial_coefficient,
                        );
                        bivariate_sign_at_cusp_and_region_parameter(
                            &predicate,
                            cusp_parameter,
                            &contact.other_parameter,
                            correlated_incidence,
                            policy,
                        )?
                    }
                    BezierAlgebraicCuspSemicircleRationalDiameter2::ParallelNormal {
                        coordinate,
                        speed_squared,
                    } => {
                        let expression = BezierAlgebraicCuspTwoTermExpression2 {
                            rational: bivariate_scaled_difference(
                                &coordinate.rational,
                                &denominator,
                                radius_squared_denominator,
                                &radial_coefficient,
                            ),
                            radical: bivariate_scale(coordinate.radical.clone(), &denominator),
                        };
                        if let Some(incidence) = correlated_incidence {
                            algebraic_cusp_correlated_radical_sum_sign(
                                incidence,
                                &expression,
                                speed_squared,
                                cusp_parameter,
                                &promoted_other!(),
                                policy,
                            )?
                        } else {
                            algebraic_cusp_independent_radical_sum_sign(
                                &expression,
                                speed_squared,
                                cusp_parameter,
                                &promoted_other!(),
                                policy,
                            )?
                        }
                    }
                }
            }
            BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::SelectedRadial {
                pair_map,
                branch,
                discriminant,
                diameter,
                radius_squared_denominator,
                ..
            } => {
                let negative_radial = -radial_coefficient;
                let Some(rational) = TrivariatePolynomial::linear_combination(&[
                    (&diameter.rational, &denominator),
                    (radius_squared_denominator, &negative_radial),
                ]) else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                };
                let Some(radical) = diameter.radical.scale(&denominator) else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                };
                let Some([first_cusp_parameter, second_cusp_parameter]) =
                    pair_map.compact_source_parameters()
                else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                };
                algebraic_cusp_trivariate_square_root_components_sign(
                    &rational,
                    &radical,
                    discriminant,
                    &first_cusp_parameter,
                    &second_cusp_parameter,
                    &promoted_other!(),
                    *branch,
                    policy,
                )?
            }
            BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::Recursive { .. } => {
                unreachable!("recursive rational maps retain their native parameter")
            }
            BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::ChordNormalProjective {
                system,
            } => system.diameter_parameter_sign(
                &promoted_other!(),
                &denominator,
                &radial_coefficient,
                policy,
            )?,
            BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::Represented { frame } => {
                represented_circle_diameter_predicate_sign(
                    frame,
                    &self.data.curve,
                    &promoted_other!(),
                    &radial_coefficient,
                    &denominator,
                    &self.data.parameter_cache,
                    policy,
                )?
            }
        };
        Ok(match sign {
            // A(u) is strictly decreasing: a larger diameter coordinate is an
            // earlier semicircle parameter.
            Classification::Decided(RealSign::Positive) => {
                Classification::Decided(std::cmp::Ordering::Less)
            }
            Classification::Decided(RealSign::Negative) => {
                Classification::Decided(std::cmp::Ordering::Greater)
            }
            Classification::Decided(RealSign::Zero) => {
                Classification::Decided(std::cmp::Ordering::Equal)
            }
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        })
    }

    pub(crate) fn contact_parameter(
        &self,
        contact: &BezierAlgebraicCuspSemicircleRationalContact2,
    ) -> BezierAlgebraicCuspSemicircleParameter2 {
        self.mapped_parameter(BezierAlgebraicCuspSemicircleRationalMapContact2 {
            other_parameter: contact.other_parameter.clone(),
            location: contact.location,
            correlation: BezierAlgebraicCuspSemicircleRationalCorrelation2::Map,
        })
    }

    pub(super) fn contact_parameter_with_chord_tangent(
        &self,
        contact: &BezierAlgebraicCuspSemicircleRationalContact2,
        chord: &BezierAlgebraicChord2,
    ) -> BezierAlgebraicCuspSemicircleParameter2 {
        self.mapped_parameter(BezierAlgebraicCuspSemicircleRationalMapContact2 {
            other_parameter: contact.other_parameter.clone(),
            location: contact.location,
            correlation: BezierAlgebraicCuspSemicircleRationalCorrelation2::MapWithChordTangent {
                chord: chord.clone(),
                circle_cross_chord: contact.tangent_cross_sign,
            },
        })
    }

    pub(super) fn mapped_parameter(
        &self,
        contact: BezierAlgebraicCuspSemicircleRationalMapContact2,
    ) -> BezierAlgebraicCuspSemicircleParameter2 {
        algebraic_cusp_semicircle_endpoint_parameter(contact.location).unwrap_or_else(|| {
            if let Some(parameter) =
                self.retained_chord_cusp_parameter(&contact.other_parameter, &self.data.policy)
            {
                let preserves_tangent = match &contact.correlation {
                    BezierAlgebraicCuspSemicircleRationalCorrelation2::Map => true,
                    BezierAlgebraicCuspSemicircleRationalCorrelation2::MapWithChordTangent {
                        chord,
                        ..
                    } => matches!(
                        &parameter,
                        BezierAlgebraicCuspSemicircleParameter2::Mapped(data)
                            if matches!(
                                data.as_ref(),
                                BezierAlgebraicCuspSemicircleMappedParameterData2::Chord {
                                    map,
                                    ..
                                } if map.data.chord.shared_tangent_orientation(chord).is_some()
                            )
                    ),
                    BezierAlgebraicCuspSemicircleRationalCorrelation2::Independent
                    | BezierAlgebraicCuspSemicircleRationalCorrelation2::Relation(_) => false,
                };
                if preserves_tangent {
                    return parameter;
                }
            }
            BezierAlgebraicCuspSemicircleParameter2::Mapped(Arc::new(
                BezierAlgebraicCuspSemicircleMappedParameterData2::Rational {
                    map: self.clone(),
                    contact,
                },
            ))
        })
    }
}

impl BezierAlgebraicCuspSemicircleRationalParameterMapData2 {
    pub(super) fn one_field_system(
        &self,
    ) -> Option<(
        &BezierParameter2,
        &BivariatePolynomial,
        &BezierAlgebraicCuspSemicircleRationalDiameter2,
        &BivariatePolynomial,
    )> {
        let BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::OneField {
            cusp_parameter,
            incidence,
            diameter,
            radius_squared_denominator,
        } = &self.system
        else {
            return None;
        };
        Some((
            cusp_parameter,
            incidence,
            diameter,
            radius_squared_denominator,
        ))
    }
}

impl BezierAlgebraicCuspSemicircleRetainedChordContact2 {
    /// Combines this contact's retained cross and dot certificates with the
    /// circle orientation. At a tangent, the circle lies on the inward side
    /// of its tangent line, independent of how the contact was constructed.
    pub(crate) fn tangent_topology(
        &self,
        semicircle: &BezierAlgebraicCuspSemicircle2,
        chord: &BezierAlgebraicChord2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<(RealSign, crate::classify::LineSide)>>> {
        if self.tangent_cross_sign != RealSign::Zero {
            return Ok(Classification::Decided(None));
        }
        let dot = match self.tangent_dot_sign(semicircle, chord, policy)? {
            Classification::Decided(RealSign::Zero) => {
                return Err(CurveError::Topology(
                    "nonzero circle/line tangents have zero cross and dot products".into(),
                ));
            }
            Classification::Decided(dot) => dot,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let circle_side = if (dot == RealSign::Positive) != semicircle.is_clockwise() {
            crate::classify::LineSide::Left
        } else {
            crate::classify::LineSide::Right
        };
        Ok(Classification::Decided(Some((dot, circle_side))))
    }

    /// Replays the tangent dot retained by the contact's authoritative
    /// circle/chord map before reconstructing either carrier support.
    /// Re-clipped descendant chords transport only the exact tangent
    /// orientation; contacts without mapped chord evidence retain the
    /// complete center-side fallback.
    pub(crate) fn tangent_dot_sign(
        &self,
        semicircle: &BezierAlgebraicCuspSemicircle2,
        chord: &BezierAlgebraicChord2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        if let BezierAlgebraicCuspSemicircleParameter2::Mapped(data) = &self.cusp_parameter
            && let BezierAlgebraicCuspSemicircleMappedParameterData2::Chord { map, contact } =
                data.as_ref()
            && map.data.semicircle == *semicircle
            && let Some(reversed) = map.data.chord.shared_tangent_orientation(chord)
        {
            let sign = map.retained_tangent_cross_dot_linear_combination_sign(
                contact,
                &Real::zero(),
                &Real::one(),
                policy,
            )?;
            #[cfg(feature = "dispatch-trace")]
            if matches!(sign, Classification::Decided(_)) {
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-circle-chord-tangent-dot",
                    "retained-contact-map",
                );
            }
            return Ok(sign.map(|sign| {
                if reversed {
                    product_sign(sign, RealSign::Negative)
                } else {
                    sign
                }
            }));
        }
        semicircle.chord_tangent_dot_sign(chord, policy)
    }
}

impl BezierAlgebraicCuspSemicircleRetainedParallelContact2 {
    pub(crate) fn other_parameter(&self) -> &CurveParameter2 {
        &self.other_parameter
    }

    pub(crate) fn cusp_parameter(&self) -> BezierAlgebraicCuspSemicircleParameter2 {
        self.retained.cusp_parameter.clone()
    }

    pub(crate) fn point_evidence(&self) -> CurvePoint2 {
        self.retained.point.clone()
    }

    pub(crate) fn location(&self) -> BezierAlgebraicCuspSemicircleContactLocation2 {
        match &self.retained.cusp_parameter {
            BezierAlgebraicCuspSemicircleParameter2::Exact(parameter)
                if parameter == &Real::zero() =>
            {
                BezierAlgebraicCuspSemicircleContactLocation2::Start
            }
            BezierAlgebraicCuspSemicircleParameter2::Exact(parameter)
                if parameter == &Real::one() =>
            {
                BezierAlgebraicCuspSemicircleContactLocation2::End
            }
            BezierAlgebraicCuspSemicircleParameter2::Exact(_)
            | BezierAlgebraicCuspSemicircleParameter2::Mapped(_) => {
                BezierAlgebraicCuspSemicircleContactLocation2::Interior
            }
        }
    }

    pub(crate) const fn tangent_cross_sign(&self) -> RealSign {
        self.retained.tangent_cross_sign
    }

    pub(crate) const fn tangent_dot_sign(&self) -> RealSign {
        self.tangent_dot_sign
    }

    pub(crate) const fn tangent_topology(&self) -> Option<(RealSign, crate::classify::LineSide)> {
        match self.circle_side_of_parallel {
            Some(side) => Some((self.tangent_dot_sign, side)),
            None => None,
        }
    }
}

impl BezierAlgebraicCuspSemicircleSelectedFiberContact2 {
    pub(super) fn retained(
        &self,
    ) -> (
        &BezierAlgebraicSelectedFiberParameter2,
        BezierAlgebraicCuspSemicircleContactLocation2,
        RealSign,
    ) {
        match self.data.as_ref() {
            BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedFiberRational {
                other_parameter,
                location,
                tangent_cross_sign,
                ..
            }
            | BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedFiberParallel {
                other_parameter,
                location,
                tangent_cross_sign,
                ..
            } => (other_parameter, *location, *tangent_cross_sign),
            _ => unreachable!("a selected-fiber contact owns its mapped-data variant"),
        }
    }

    pub(crate) fn other_parameter(&self) -> &BezierAlgebraicSelectedFiberParameter2 {
        self.retained().0
    }

    pub(crate) fn location(&self) -> BezierAlgebraicCuspSemicircleContactLocation2 {
        self.retained().1
    }

    pub(crate) fn tangent_cross_sign(&self) -> RealSign {
        self.retained().2
    }

    /// Returns the exact sign of the selected-circle tangent dotted with the
    /// other carrier tangent at this contact.
    ///
    /// General analytic parallels retain the original two-normal expression,
    /// so no unit tangent or primitive element is constructed. Rational
    /// parallels rebuild their one-normal system only on this uncommon
    /// angular query; the contact's selected-fiber parameter remains the
    /// evaluation authority.
    pub(crate) fn tangent_dot_sign(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        match self.data.as_ref() {
            BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedFiberParallel {
                map,
                other_parameter,
                ..
            } => map.tangent_cross_dot_linear_combination_sign(
                other_parameter,
                &Real::zero(),
                &Real::one(),
                policy,
            ),
            BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedFiberRational {
                map,
                other_parameter,
                ..
            } => map.tangent_cross_dot_linear_combination_sign(
                other_parameter,
                &Real::zero(),
                &Real::one(),
                policy,
            ),
            _ => unreachable!("a selected-fiber contact owns its mapped-data variant"),
        }
    }

    pub(crate) fn cusp_parameter(&self) -> BezierAlgebraicCuspSemicircleParameter2 {
        algebraic_cusp_semicircle_endpoint_parameter(self.location()).unwrap_or_else(|| {
            BezierAlgebraicCuspSemicircleParameter2::Mapped(Arc::clone(&self.data))
        })
    }

    pub(crate) fn point_evidence(&self) -> CurvePoint2 {
        self.data
            .retained_or_selected_point_evidence()
            .expect("a selected-fiber contact owns compact point evidence")
    }

    #[cfg(test)]
    pub(super) fn point_coordinate_order_to_real(
        &self,
        axis: Axis2,
        value: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        match self.data.as_ref() {
            BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedFiberRational {
                map,
                other_parameter,
                ..
            } => map.point_coordinate_order_to_real(other_parameter, axis, value, policy),
            BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedFiberParallel {
                map,
                other_parameter,
                ..
            } => map.point_coordinate_order_to_real(other_parameter, axis, value, policy),
            _ => unreachable!("a selected-fiber contact owns its mapped-data variant"),
        }
    }
}

impl BezierAlgebraicCuspSemicircleSelectedFiberParallelParameterMap2 {
    pub(super) fn mapped_data(
        &self,
        other_parameter: BezierAlgebraicSelectedFiberParameter2,
        location: BezierAlgebraicCuspSemicircleContactLocation2,
        tangent_cross_sign: RealSign,
    ) -> Arc<BezierAlgebraicCuspSemicircleMappedParameterData2> {
        Arc::new(
            BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedFiberParallel {
                map: self.clone(),
                other_parameter,
                location,
                tangent_cross_sign,
            },
        )
    }

    pub(super) fn contact(
        &self,
        other_parameter: BezierAlgebraicSelectedFiberParameter2,
        location: BezierAlgebraicCuspSemicircleContactLocation2,
        tangent_cross_sign: RealSign,
    ) -> BezierAlgebraicCuspSemicircleSelectedFiberContact2 {
        BezierAlgebraicCuspSemicircleSelectedFiberContact2 {
            data: self.mapped_data(other_parameter, location, tangent_cross_sign),
        }
    }

    pub(super) fn validate_policy(&self, policy: &CurveContext) -> CurveResult<()> {
        if !policy.accepts_retained_policy(self.data.policy) {
            return Err(CurveError::Topology(
                "a selected-fiber parallel map crossed predicate policies".into(),
            ));
        }
        Ok(())
    }

    /// Signs one linear combination of the selected-circle tangent crossed
    /// and dotted with the candidate parallel's source tangent.
    ///
    /// Both terms are lifted to the same positive `W^2*|T_center|` scale, so
    /// they may be added before invoking the authoritative two-normal
    /// selected-fiber predicate. This is the retained angular authority used
    /// by carrier-switch joins; it never constructs either unit tangent.
    pub(super) fn tangent_cross_dot_linear_combination_sign(
        &self,
        other_parameter: &BezierAlgebraicSelectedFiberParameter2,
        cross_scale: &Real,
        dot_scale: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        self.validate_policy(policy)?;
        let zero = BivariatePolynomial::new(vec![vec![Real::zero()]]);
        let expression = BezierParallelTwoNormalExpression2 {
            product: bivariate_scale(self.data.tangent_dot_source.product.clone(), dot_scale),
            center: bivariate_add(
                &bivariate_scale(self.data.tangent_cross_source.rational.clone(), cross_scale),
                &bivariate_scale(self.data.tangent_dot_source.center.clone(), dot_scale),
            ),
            candidate: if dot_scale.zero_status() == ZeroKnowledge::Zero {
                zero
            } else {
                bivariate_scale(self.data.tangent_dot_source.candidate.clone(), dot_scale)
            },
            rational: bivariate_add(
                &bivariate_scale(self.data.tangent_cross_source.radical.clone(), cross_scale),
                &bivariate_scale(self.data.tangent_dot_source.rational.clone(), dot_scale),
            ),
        };
        other_parameter.two_normal_sum_sign(
            &expression,
            &self.data.center_speed_squared,
            &self.data.candidate_speed_squared,
            policy,
        )
    }

    pub(super) fn contact_order_to_real(
        &self,
        other_parameter: &BezierAlgebraicSelectedFiberParameter2,
        location: BezierAlgebraicCuspSemicircleContactLocation2,
        parameter: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        self.validate_policy(policy)?;
        if let Some(order) =
            algebraic_cusp_semicircle_endpoint_contact_order(location, parameter, policy)
        {
            return Ok(order);
        }
        match in_closed_unit_interval(parameter, policy) {
            Some(true) => {}
            Some(false) => return Err(CurveError::InvalidBezierParameter),
            None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }
        let one_minus = Real::one() - parameter;
        let denominator = &one_minus * &one_minus + parameter * parameter;
        match real_sign(&denominator, policy) {
            Some(RealSign::Positive) => {}
            Some(RealSign::Zero | RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "semicircle parameter-order denominator was not positive".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let radial_coefficient = Real::one() - Real::from(2_i8) * parameter;
        let expression = BezierParallelTwoNormalExpression2 {
            product: bivariate_scaled_difference(
                &self.data.diameter.product,
                &denominator,
                &self.data.radius_squared_denominator,
                &radial_coefficient,
            ),
            center: bivariate_scale(self.data.diameter.center.clone(), &denominator),
            candidate: bivariate_scale(self.data.diameter.candidate.clone(), &denominator),
            rational: bivariate_scale(self.data.diameter.rational.clone(), &denominator),
        };
        Ok(other_parameter
            .two_normal_sum_sign(
                &expression,
                &self.data.center_speed_squared,
                &self.data.candidate_speed_squared,
                policy,
            )?
            .map(|sign| match sign {
                RealSign::Positive => std::cmp::Ordering::Less,
                RealSign::Zero => std::cmp::Ordering::Equal,
                RealSign::Negative => std::cmp::Ordering::Greater,
            }))
    }

    pub(super) fn point_bounds_refined(
        &self,
        other_parameter: &BezierAlgebraicSelectedFiberParameter2,
        refinement_steps: usize,
        policy: &CurveContext,
    ) -> Classification<Aabb2> {
        if self.validate_policy(policy).is_err() {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        }
        let other_parameter = match other_parameter.refined(refinement_steps, policy) {
            Ok(Classification::Decided(parameter)) => parameter,
            Ok(Classification::Uncertain(reason)) => {
                return Classification::Uncertain(reason);
            }
            Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
        };
        analytic_parallel_point_bounds_over_interval(
            &self.data.parallel,
            &RealInterval {
                lower: other_parameter.root().lower.clone(),
                upper: other_parameter.root().upper.clone(),
            },
            &Real::zero(),
            &Real::zero(),
            &Real::zero(),
        )
    }

    #[cfg(test)]
    pub(super) fn point_coordinate_order_to_real(
        &self,
        other_parameter: &BezierAlgebraicSelectedFiberParameter2,
        axis: Axis2,
        value: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        self.validate_policy(policy)?;
        Ok(retained_bounds_axis_order_to_real(
            |refinement_steps| self.point_bounds_refined(other_parameter, refinement_steps, policy),
            axis,
            value,
            policy,
        ))
    }
}

impl BezierAlgebraicCuspSemicircleSelectedFiberRationalOverlap2 {
    pub(super) fn corresponding_endpoints(
        &self,
    ) -> [(
        &BezierAlgebraicSelectedFiberParameter2,
        &BezierAlgebraicCuspSemicircleParameter2,
    ); 2] {
        let [first, second] = match self.orientation {
            CurveOverlapOrientation2::Same => [&self.other_start, &self.other_end],
            CurveOverlapOrientation2::Reversed => [&self.other_end, &self.other_start],
        };
        [(first, &self.cusp_start), (second, &self.cusp_end)]
    }

    pub(crate) fn other_start_parameter(&self) -> BezierAlgebraicSelectedFiberParameter2 {
        self.other_start.clone()
    }

    pub(crate) fn other_end_parameter(&self) -> BezierAlgebraicSelectedFiberParameter2 {
        self.other_end.clone()
    }

    pub(crate) fn cusp_start_parameter(&self) -> BezierAlgebraicCuspSemicircleParameter2 {
        self.cusp_start.clone()
    }

    pub(crate) fn cusp_end_parameter(&self) -> BezierAlgebraicCuspSemicircleParameter2 {
        self.cusp_end.clone()
    }

    pub(crate) const fn orientation(&self) -> CurveOverlapOrientation2 {
        self.orientation
    }

    pub(crate) fn parameter_ranges(&self) -> (CurveParameterRange2, CurveParameterRange2) {
        (
            CurveParameterRange2::new_validated(
                CurveParameter2::from_algebraic_cusp(self.cusp_start_parameter()),
                CurveParameter2::from_algebraic_cusp(self.cusp_end_parameter()),
            ),
            CurveParameterRange2::new_validated(
                CurveParameter2::from_selected_fiber(self.other_start_parameter()),
                CurveParameter2::from_selected_fiber(self.other_end_parameter()),
            ),
        )
    }

    pub(crate) fn map_parameter(
        &self,
        parameter: &CurveParameter2,
        cusp_to_other: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<CurveParameter2>>> {
        if cusp_to_other {
            let Some(parameter) = parameter.as_algebraic_cusp() else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            return Ok(self
                .other_parameter_for_cusp(parameter, policy)?
                .map(|parameter| Some(CurveParameter2::from_selected_fiber(parameter))));
        }
        let parameter = match self.retain_unique_other_parameter(vec![parameter.clone()], policy)? {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        Ok(self
            .cusp_parameter_for_other(&parameter, policy)?
            .map(|parameter| Some(CurveParameter2::from_algebraic_cusp(parameter))))
    }

    pub(crate) fn has_positive_overlap(
        &self,
        cusp_fragment: &CurveParameterRange2,
        other_fragment: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        let (cusp_overlap, other_overlap) = self.parameter_ranges();
        crate::bezier_split::corresponding_parameter_ranges_are_positive(
            &cusp_overlap,
            &other_overlap,
            cusp_fragment,
            other_fragment,
            policy,
            |parameter| self.map_parameter(parameter, true, policy),
        )
    }

    pub(super) fn endpoint_location(
        &self,
        parameter: &BezierAlgebraicSelectedFiberParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierAlgebraicCuspSemicircleContactLocation2>>> {
        for (endpoint, cusp_parameter) in self.corresponding_endpoints() {
            match parameter.cmp_by_refinement(endpoint, policy)? {
                Classification::Decided(std::cmp::Ordering::Equal) => {
                    let location = match cusp_parameter {
                        BezierAlgebraicCuspSemicircleParameter2::Exact(value)
                            if value == &Real::zero() =>
                        {
                            BezierAlgebraicCuspSemicircleContactLocation2::Start
                        }
                        BezierAlgebraicCuspSemicircleParameter2::Exact(value)
                            if value == &Real::one() =>
                        {
                            BezierAlgebraicCuspSemicircleContactLocation2::End
                        }
                        _ => BezierAlgebraicCuspSemicircleContactLocation2::Interior,
                    };
                    return Ok(Classification::Decided(Some(location)));
                }
                Classification::Decided(_) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        Ok(Classification::Decided(None))
    }

    pub(crate) fn cusp_parameter_for_other(
        &self,
        parameter: &BezierAlgebraicSelectedFiberParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicircleParameter2>> {
        self.map.validate_policy(policy)?;
        for (endpoint, cusp_parameter) in self.corresponding_endpoints() {
            match parameter.cmp_by_refinement(endpoint, policy)? {
                Classification::Decided(std::cmp::Ordering::Equal) => {
                    return Ok(Classification::Decided(cusp_parameter.clone()));
                }
                Classification::Decided(_) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        if let Some(Some(cusp)) = self.map.data.parameter_cache.retained_cusp_parameter(
            &CurveParameter2::from_selected_fiber(parameter.clone()),
            policy,
        ) {
            return Ok(Classification::Decided(cusp));
        }
        Ok(Classification::Decided(self.map.mapped_parameter(
            parameter.clone(),
            BezierAlgebraicCuspSemicircleContactLocation2::Interior,
            RealSign::Zero,
        )))
    }

    pub(crate) fn point_evidence_for_other(
        &self,
        parameter: &BezierAlgebraicSelectedFiberParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<CurvePoint2>> {
        self.map.validate_policy(policy)?;
        let location = match self.endpoint_location(parameter, policy)? {
            Classification::Decided(Some(location)) => location,
            Classification::Decided(None) => {
                BezierAlgebraicCuspSemicircleContactLocation2::Interior
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(Classification::Decided(
            self.map
                .contact(parameter.clone(), location, RealSign::Zero)
                .point_evidence(),
        ))
    }

    pub(super) fn contains_other_parameter(
        &self,
        parameter: &BezierAlgebraicSelectedFiberParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        let start = match parameter.cmp_by_refinement(&self.other_start, policy)? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let end = match parameter.cmp_by_refinement(&self.other_end, policy)? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(Classification::Decided(
            start != std::cmp::Ordering::Less && end != std::cmp::Ordering::Greater,
        ))
    }

    pub(super) fn retain_unique_other_parameter(
        &self,
        candidates: Vec<CurveParameter2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicSelectedFiberParameter2>> {
        let retained_parameter = self
            .other_start
            .data
            .authority
            .data
            .retained_parameter
            .clone();
        let mut retained = None;
        for candidate in candidates {
            let candidate = if let Some(candidate) = candidate.as_selected_fiber()
                && candidate.data.authority.data.retained_parameter == retained_parameter
            {
                candidate.clone()
            } else {
                let candidate = if let Some(candidate) = candidate.as_bezier_parameter() {
                    candidate.clone()
                } else {
                    match policy.strict_predicate_pass(|| {
                        candidate.promoted_bezier_parameter_complete(policy)
                    })? {
                        Classification::Decided(candidate) => candidate,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                };
                BezierAlgebraicSelectedFiberAuthority2::from_bezier_parameter(
                    retained_parameter.clone(),
                    candidate,
                    policy,
                )
            };
            match self.contains_other_parameter(&candidate, policy)? {
                Classification::Decided(true) => {}
                Classification::Decided(false) => continue,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            if retained.is_some() {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            retained = Some(candidate);
        }
        retained.map_or(
            Ok(Classification::Uncertain(UncertaintyReason::Boundary)),
            |parameter| Ok(Classification::Decided(parameter)),
        )
    }

    pub(crate) fn other_parameter_for_cusp(
        &self,
        parameter: &BezierAlgebraicCuspSemicircleParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicSelectedFiberParameter2>> {
        let result = self.solve_other_parameter_for_cusp(parameter, policy)?;
        if let Classification::Decided(target) = &result {
            self.map.data.parameter_cache.retain_cusp_parameter(
                CurveParameter2::from_selected_fiber(target.clone()),
                parameter,
                policy,
            );
        }
        Ok(result)
    }

    pub(super) fn solve_other_parameter_for_cusp(
        &self,
        parameter: &BezierAlgebraicCuspSemicircleParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicSelectedFiberParameter2>> {
        parameter.validate_policy(policy)?;
        self.map.validate_policy(policy)?;
        for (other_parameter, cusp_parameter) in self.corresponding_endpoints() {
            match parameter.cmp_by_refinement(cusp_parameter, policy)? {
                Classification::Decided(std::cmp::Ordering::Equal) => {
                    return Ok(Classification::Decided(other_parameter.clone()));
                }
                Classification::Decided(_) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        if let BezierAlgebraicCuspSemicircleParameter2::Mapped(data) = parameter
            && let BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedFiberRational {
                map,
                other_parameter,
                ..
            } = data.as_ref()
            && map.data.semicircle == self.map.data.semicircle
            && map.data.curve == self.map.data.curve
        {
            return match self.contains_other_parameter(other_parameter, policy)? {
                Classification::Decided(true) => {
                    Ok(Classification::Decided(other_parameter.clone()))
                }
                Classification::Decided(false) => {
                    Ok(Classification::Uncertain(UncertaintyReason::Boundary))
                }
                Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
            };
        }
        if let BezierAlgebraicCuspSemicircleParameter2::Mapped(data) = parameter
            && let Some(tangent_cross) = data.ordinary_carrier_tangent_cross_sign(policy)?
        {
            match tangent_cross {
                Classification::Decided(RealSign::Zero) => {}
                Classification::Decided(RealSign::Negative | RealSign::Positive) => {
                    let direct_candidates = match data.chamfer_exact_point(policy)? {
                        Classification::Decided(Some(point)) => self
                            .map
                            .data
                            .curve
                            .retained_circle_point_parameters(&point, policy)?
                            .map(|parameters| {
                                Some(curve_region_parameters_from_bezier(parameters))
                            }),
                        Classification::Decided(None) => Classification::Decided(None),
                        Classification::Uncertain(reason) => Classification::Uncertain(reason),
                    };
                    let candidates = match direct_candidates {
                        decided @ Classification::Decided(Some(_)) => decided,
                        Classification::Decided(None) | Classification::Uncertain(_) => {
                            if let Some(source) = data.mapped_point_source(policy)? {
                                source.point_parameter_candidates(
                                    &self.map.data.curve.parallel_left(Real::zero())?,
                                    &self.parameter_ranges().1,
                                    policy,
                                )?
                            } else {
                                data.retained_point_parameter_candidates_on_rational_target(
                                    &self.map.data.curve,
                                    &self.parameter_ranges().1,
                                    policy,
                                )?
                            }
                        }
                    };
                    let candidates = match candidates {
                        Classification::Decided(Some(candidates)) => candidates,
                        Classification::Decided(None) => {
                            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                        }
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    return self.retain_unique_other_parameter(candidates, policy);
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        if let BezierAlgebraicCuspSemicircleParameter2::Mapped(data) = parameter
            && let Some(source) = data.mapped_point_source(policy)?
            && let Classification::Decided(Some(candidates)) = source.point_parameter_candidates(
                &self.map.data.curve.parallel_left(Real::zero())?,
                &self.parameter_ranges().1,
                policy,
            )?
            && !candidates.is_empty()
        {
            return self.retain_unique_other_parameter(candidates, policy);
        }
        if let BezierAlgebraicCuspSemicircleParameter2::Mapped(data) = parameter
            && let Some((source_parameter, source_tangent, source_policy)) =
                data.coincident_tangent_power_source(policy)?
        {
            if !policy.accepts_retained_policy(source_policy) {
                return Err(CurveError::Topology(
                    "selected overlap tangent source used a different predicate policy".into(),
                ));
            }
            let target_tangent = rational_parametric_tangent_numerator(
                self.map.data.curve.homogeneous_power_basis()?,
            );
            let candidates = match mapped_circle_tangent_parameter_candidates(
                source_parameter.as_ref(),
                &source_tangent,
                &target_tangent,
                &self.parameter_ranges().1,
                policy,
            )? {
                Classification::Decided(candidates) => candidates,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            return self.retain_unique_other_parameter(candidates, policy);
        }
        if let BezierAlgebraicCuspSemicircleParameter2::Mapped(data) = parameter {
            match data.retained_point_parameter_candidates_on_rational_target(
                &self.map.data.curve,
                &self.parameter_ranges().1,
                policy,
            )? {
                Classification::Decided(Some(candidates)) => {
                    return self.retain_unique_other_parameter(candidates, policy);
                }
                Classification::Decided(None) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        if matches!(
            parameter,
            BezierAlgebraicCuspSemicircleParameter2::Mapped(_)
        ) {
            match parameter.scalar_value(policy)? {
                Classification::Decided(Some(parameter)) => {
                    return self.other_parameter_for_exact_cusp(&parameter, policy);
                }
                Classification::Decided(None) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        let BezierAlgebraicCuspSemicircleParameter2::Exact(parameter) = parameter else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        self.other_parameter_for_exact_cusp(parameter, policy)
    }

    pub(super) fn other_parameter_for_exact_cusp(
        &self,
        parameter: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicSelectedFiberParameter2>> {
        let one_minus = Real::one() - parameter;
        let denominator = &one_minus * &one_minus + parameter * parameter;
        match real_sign(&denominator, policy) {
            Some(RealSign::Positive) => {}
            Some(RealSign::Zero | RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "selected overlap inverse-map denominator was not positive".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let radial = Real::one() - Real::from(2_i8) * parameter;
        let predicate = BezierAlgebraicCuspTwoTermExpression2 {
            rational: bivariate_scaled_difference(
                &self.map.data.diameter.rational,
                &denominator,
                &self.map.data.radius_squared_denominator,
                &radial,
            ),
            radical: bivariate_scale(self.map.data.diameter.radical.clone(), &denominator),
        };
        let incidence = bivariate_subtract(
            &bivariate_multiply(
                &bivariate_multiply(&predicate.rational, &predicate.rational),
                &self.map.data.speed_squared,
            ),
            &bivariate_multiply(&predicate.radical, &predicate.radical),
        );
        let center_parameter = self
            .map
            .data
            .semicircle
            .selected_frame_parameter()
            .ok_or_else(|| {
                CurveError::Topology("selected overlap inverse lost its center".into())
            })?;
        let center_parameter =
            match promote_curve_region_bezier_parameter(&center_parameter, policy)? {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
        let BezierParameter2::Algebraic(center_parameter) = center_parameter else {
            return Err(CurveError::Topology(
                "selected overlap inverse lost its algebraic center".into(),
            ));
        };
        let (_, range) = self.parameter_ranges();
        let candidates = match selected_fiber_parameters_in_range(
            &incidence,
            &center_parameter,
            &range,
            policy,
        )? {
            Classification::Decided(Some(candidates)) => candidates,
            Classification::Decided(None) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let mut retained = None;
        for candidate in candidates {
            match candidate.radical_sum_sign(&predicate, &self.map.data.speed_squared, policy)? {
                Classification::Decided(RealSign::Zero) => {}
                Classification::Decided(RealSign::Negative | RealSign::Positive) => continue,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            match self.contains_other_parameter(&candidate, policy)? {
                Classification::Decided(true) => {}
                Classification::Decided(false) => continue,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            if retained.is_some() {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            retained = Some(candidate);
        }
        retained.map_or(
            Ok(Classification::Uncertain(UncertaintyReason::Boundary)),
            |parameter| Ok(Classification::Decided(parameter)),
        )
    }
}

impl BezierAlgebraicCuspSemicircleSelectedFiberRationalParameterMap2 {
    pub(super) fn mapped_data(
        &self,
        other_parameter: BezierAlgebraicSelectedFiberParameter2,
        location: BezierAlgebraicCuspSemicircleContactLocation2,
        tangent_cross_sign: RealSign,
    ) -> Arc<BezierAlgebraicCuspSemicircleMappedParameterData2> {
        Arc::new(
            BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedFiberRational {
                map: self.clone(),
                other_parameter,
                location,
                tangent_cross_sign,
            },
        )
    }

    pub(super) fn mapped_parameter(
        &self,
        other_parameter: BezierAlgebraicSelectedFiberParameter2,
        location: BezierAlgebraicCuspSemicircleContactLocation2,
        tangent_cross_sign: RealSign,
    ) -> BezierAlgebraicCuspSemicircleParameter2 {
        algebraic_cusp_semicircle_endpoint_parameter(location).unwrap_or_else(|| {
            BezierAlgebraicCuspSemicircleParameter2::Mapped(self.mapped_data(
                other_parameter,
                location,
                tangent_cross_sign,
            ))
        })
    }

    pub(super) fn contact(
        &self,
        other_parameter: BezierAlgebraicSelectedFiberParameter2,
        location: BezierAlgebraicCuspSemicircleContactLocation2,
        tangent_cross_sign: RealSign,
    ) -> BezierAlgebraicCuspSemicircleSelectedFiberContact2 {
        BezierAlgebraicCuspSemicircleSelectedFiberContact2 {
            data: self.mapped_data(other_parameter, location, tangent_cross_sign),
        }
    }

    pub(super) fn validate_policy(&self, policy: &CurveContext) -> CurveResult<()> {
        if !policy.accepts_retained_policy(self.data.policy) {
            return Err(CurveError::Topology(
                "a selected-fiber rational map crossed predicate policies".into(),
            ));
        }
        Ok(())
    }

    /// Signs one exact linear combination of the selected-circle tangent
    /// crossed and dotted with the mapped rational tangent. Both retained
    /// terms use the same positive speed scale, so angular consumers can
    /// replay the original contact without rebuilding its intersection
    /// system or promoting its selected parameter.
    pub(super) fn tangent_cross_dot_linear_combination_sign(
        &self,
        other_parameter: &BezierAlgebraicSelectedFiberParameter2,
        cross_scale: &Real,
        dot_scale: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        self.validate_policy(policy)?;
        let dot_scale = dot_scale * self.data.semicircle.turn_sign();
        let expression = BezierAlgebraicCuspTwoTermExpression2 {
            rational: bivariate_add(
                &bivariate_scale(self.data.tangent_cross.rational.clone(), cross_scale),
                &bivariate_scale(self.data.angular_tangent.rational.clone(), &dot_scale),
            ),
            radical: bivariate_add(
                &bivariate_scale(self.data.tangent_cross.radical.clone(), cross_scale),
                &bivariate_scale(self.data.angular_tangent.radical.clone(), &dot_scale),
            ),
        };
        other_parameter.radical_sum_sign(&expression, &self.data.speed_squared, policy)
    }

    pub(super) fn contact_order_to_real(
        &self,
        other_parameter: &BezierAlgebraicSelectedFiberParameter2,
        location: BezierAlgebraicCuspSemicircleContactLocation2,
        parameter: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        self.validate_policy(policy)?;
        if let Some(order) =
            algebraic_cusp_semicircle_endpoint_contact_order(location, parameter, policy)
        {
            return Ok(order);
        }
        match in_closed_unit_interval(parameter, policy) {
            Some(true) => {}
            Some(false) => return Err(CurveError::InvalidBezierParameter),
            None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }
        let one_minus = Real::one() - parameter;
        let denominator = &one_minus * &one_minus + parameter * parameter;
        match real_sign(&denominator, policy) {
            Some(RealSign::Positive) => {}
            Some(RealSign::Zero | RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "semicircle parameter-order denominator was not positive".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let radial_coefficient = Real::one() - Real::from(2_i8) * parameter;
        let expression = BezierAlgebraicCuspTwoTermExpression2 {
            rational: bivariate_scaled_difference(
                &self.data.diameter.rational,
                &denominator,
                &self.data.radius_squared_denominator,
                &radial_coefficient,
            ),
            radical: bivariate_scale(self.data.diameter.radical.clone(), &denominator),
        };
        Ok(other_parameter
            .radical_sum_sign(&expression, &self.data.speed_squared, policy)?
            .map(|sign| match sign {
                // The directed diameter coordinate decreases with the local
                // semicircle parameter.
                RealSign::Positive => std::cmp::Ordering::Less,
                RealSign::Zero => std::cmp::Ordering::Equal,
                RealSign::Negative => std::cmp::Ordering::Greater,
            }))
    }

    pub(super) fn point_coordinate_order_to_real(
        &self,
        other_parameter: &BezierAlgebraicSelectedFiberParameter2,
        axis: Axis2,
        value: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        self.validate_policy(policy)?;
        let source = self.data.curve.homogeneous_power_basis()?;
        let numerator = match axis {
            Axis2::X => &source.x_numerator,
            Axis2::Y => &source.y_numerator,
        };
        let predicate = bivariate_outer_product(
            &[Real::one()],
            &polynomial_subtract(numerator, &polynomial_scale(&source.weight, value)),
        );
        let numerator_sign = match other_parameter.predicate_sign(&predicate, policy)? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let weight_sign = match other_parameter.predicate_sign(
            &bivariate_outer_product(&[Real::one()], &source.weight),
            policy,
        )? {
            Classification::Decided(RealSign::Positive) => RealSign::Positive,
            Classification::Decided(RealSign::Negative) => RealSign::Negative,
            Classification::Decided(RealSign::Zero) => {
                return Err(CurveError::Topology(
                    "a finite selected-fiber rational contact had a zero denominator".into(),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(Classification::Decided(
            match product_sign(numerator_sign, weight_sign) {
                RealSign::Negative => std::cmp::Ordering::Less,
                RealSign::Zero => std::cmp::Ordering::Equal,
                RealSign::Positive => std::cmp::Ordering::Greater,
            },
        ))
    }

    pub(super) fn point_bounds_refined(
        &self,
        other_parameter: &BezierAlgebraicSelectedFiberParameter2,
        refinement_steps: usize,
        policy: &CurveContext,
    ) -> Classification<Aabb2> {
        if self.validate_policy(policy).is_err() {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        }
        let other_parameter = match other_parameter.refined(refinement_steps, policy) {
            Ok(Classification::Decided(parameter)) => parameter,
            Ok(Classification::Uncertain(reason)) => {
                return Classification::Uncertain(reason);
            }
            Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
        };
        let parameter = RealInterval {
            lower: other_parameter.root().lower.clone(),
            upper: other_parameter.root().upper.clone(),
        };
        rational_bezier_point_bounds_over_interval(&self.data.curve, &parameter)
    }
}

impl BezierAlgebraicCuspSemicircleSimilarityCache2 {
    pub(crate) fn chord(
        &mut self,
        source: &BezierAlgebraicChord2,
        transform: &Similarity2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicChord2>> {
        if let Some(transformed) = self
            .chords
            .iter()
            .find(|(cached, _)| Arc::ptr_eq(&cached.data, &source.data))
            .map(|(_, transformed)| transformed.clone())
        {
            return Ok(Classification::Decided(transformed));
        }
        let transformed = match source.transform_similarity_cached(transform, policy, self)? {
            Classification::Decided(transformed) => transformed,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        self.chords.push((source.clone(), transformed.clone()));
        Ok(Classification::Decided(transformed))
    }

    pub(super) fn frame(
        &mut self,
        source: &BezierSelectedCircleFrame2,
        transform: &Similarity2,
    ) -> CurveResult<BezierSelectedCircleFrame2> {
        if let Some(transformed) = self
            .frames
            .iter()
            .find(|(cached, _)| cached == source)
            .map(|(_, transformed)| transformed.clone())
        {
            return Ok(transformed);
        }
        let transformed = match source {
            BezierSelectedCircleFrame2::ChordNormal(frame) => {
                let anchor = match self.chord(&frame.anchor, transform, &frame.policy)? {
                    Classification::Decided(anchor) => anchor,
                    Classification::Uncertain(reason) => {
                        return Err(CurveError::Topology(format!(
                            "a chord-normal circle similarity could not retain its anchor: {reason:?}"
                        )));
                    }
                };
                BezierSelectedCircleFrame2::ChordNormal(Arc::new(
                    BezierSelectedChordNormalFrameData2 {
                        anchor,
                        center: CurvePoint2::from(BezierSimilarityPoint2::new(
                            frame.center.clone(),
                            transform.clone(),
                            &frame.policy,
                        )),
                        policy: frame.policy,
                    },
                ))
            }
            BezierSelectedCircleFrame2::SelectedRadial(frame) => {
                let source_semicircle = frame.center_parameter.semicircle_carrier().clone();
                let source_parameter =
                    BezierAlgebraicCuspSemicircleParameter2::Mapped(frame.center_parameter.clone());
                let transformed_parameter =
                    self.parameter(&source_parameter, &source_semicircle, transform)?;
                let BezierAlgebraicCuspSemicircleParameter2::Mapped(center_parameter) =
                    transformed_parameter
                else {
                    return Err(CurveError::Topology(
                        "a mapped selected-radial center became an inline parameter".into(),
                    ));
                };
                let mut normal_denominator = &frame.normal_denominator * transform.scale();
                if transform.reverses_orientation() {
                    normal_denominator = -normal_denominator;
                }
                BezierSelectedCircleFrame2::SelectedRadial(Arc::new(
                    BezierSelectedRadialFrameData2 {
                        center_parameter,
                        normal_denominator,
                        similarity_source: frame.similarity_source.clone(),
                        policy: frame.policy,
                    },
                ))
            }
            BezierSelectedCircleFrame2::Rational(_)
            | BezierSelectedCircleFrame2::ParallelNormal(_) => {
                source.transform_similarity(transform)?
            }
        };
        self.frames.push((source.clone(), transformed.clone()));
        Ok(transformed)
    }

    pub(super) fn semicircle(
        &mut self,
        source: &BezierAlgebraicCuspSemicircle2,
        transform: &Similarity2,
    ) -> CurveResult<BezierAlgebraicCuspSemicircle2> {
        if let Some(transformed) = self
            .semicircles
            .iter()
            .find(|(cached, _)| Arc::ptr_eq(&cached.data, &source.data))
            .map(|(_, transformed)| transformed.clone())
        {
            return Ok(transformed);
        }
        let mut radial_distance = source.radial_distance() * transform.scale();
        if transform.reverses_orientation() {
            radial_distance = -radial_distance;
        }
        let mut frame = self.frame(&source.data.frame, transform)?;
        if let (
            BezierSelectedCircleFrame2::SelectedRadial(source_frame),
            BezierSelectedCircleFrame2::SelectedRadial(transformed_frame),
        ) = (&source.data.frame, &frame)
        {
            let similarity_source = source_frame.similarity_source.clone().unwrap_or_else(|| {
                Arc::new(BezierSelectedRadialSimilaritySource2 {
                    frame: source_frame.clone(),
                    radial_distance: source.radial_distance().clone(),
                    clockwise: source.is_clockwise(),
                })
            });
            frame = BezierSelectedCircleFrame2::SelectedRadial(Arc::new(
                BezierSelectedRadialFrameData2 {
                    center_parameter: transformed_frame.center_parameter.clone(),
                    normal_denominator: transformed_frame.normal_denominator.clone(),
                    similarity_source: Some(similarity_source),
                    policy: transformed_frame.policy,
                },
            ));
        }
        let transformed = BezierAlgebraicCuspSemicircle2 {
            data: Arc::new(BezierAlgebraicCuspSemicircleData2 {
                parallel_system_cache: Mutex::default(),
                frame,
                radial_distance,
                clockwise: source.is_clockwise() ^ transform.reverses_orientation(),
            }),
        };
        self.semicircles.push((source.clone(), transformed.clone()));
        Ok(transformed)
    }

    pub(super) fn overlap(
        &mut self,
        overlap: &BezierAlgebraicCuspSemicirclePairOverlap2,
        transform: &Similarity2,
    ) -> CurveResult<BezierAlgebraicCuspSemicirclePairOverlap2> {
        if let Some(transformed) = self
            .overlaps
            .iter()
            .find(|(cached, _)| Arc::ptr_eq(&cached.data, &overlap.data))
            .map(|(_, transformed)| transformed.clone())
        {
            return Ok(transformed);
        }
        let first_semicircle = self.semicircle(overlap.semicircle(true), transform)?;
        let second_semicircle = self.semicircle(overlap.semicircle(false), transform)?;
        let source = match &overlap.data.parameter_map {
            BezierAlgebraicCuspSemicirclePairOverlapParameterMapData2::SimilarityTransport {
                source,
                ..
            } => source.clone(),
            BezierAlgebraicCuspSemicirclePairOverlapParameterMapData2::ExactEndpoints {
                ..
            }
            | BezierAlgebraicCuspSemicirclePairOverlapParameterMapData2::Represented { .. } => {
                overlap.clone()
            }
        };
        let transformed = BezierAlgebraicCuspSemicirclePairOverlap2 {
            data: Arc::new(BezierAlgebraicCuspSemicirclePairOverlapData2 {
                parameter_map:
                    BezierAlgebraicCuspSemicirclePairOverlapParameterMapData2::SimilarityTransport {
                        first_semicircle,
                        second_semicircle,
                        source,
                    },
                first_boundaries: overlap.data.first_boundaries,
                second_boundaries: overlap.data.second_boundaries,
                orientation: overlap.data.orientation,
                policy: overlap.data.policy,
            }),
        };
        self.overlaps.push((overlap.clone(), transformed.clone()));
        Ok(transformed)
    }

    pub(super) fn parameter(
        &mut self,
        parameter: &BezierAlgebraicCuspSemicircleParameter2,
        source_carrier: &BezierAlgebraicCuspSemicircle2,
        transform: &Similarity2,
    ) -> CurveResult<BezierAlgebraicCuspSemicircleParameter2> {
        let BezierAlgebraicCuspSemicircleParameter2::Mapped(data) = parameter else {
            return Ok(parameter.clone());
        };
        if let Some(transformed) = self
            .parameters
            .iter()
            .find(|(cached, cached_carrier, _)| {
                Arc::ptr_eq(cached, data) && cached_carrier == source_carrier
            })
            .map(|(_, _, transformed)| transformed.clone())
        {
            return Ok(transformed);
        }
        let transformed = match data.as_ref() {
            BezierAlgebraicCuspSemicircleMappedParameterData2::PairOverlap {
                overlap,
                endpoint,
                first,
            } if overlap.semicircle(*first) == source_carrier => {
                Some(BezierAlgebraicCuspSemicircleParameter2::Mapped(Arc::new(
                    BezierAlgebraicCuspSemicircleMappedParameterData2::PairOverlap {
                        overlap: self.overlap(overlap, transform)?,
                        endpoint: *endpoint,
                        first: *first,
                    },
                )))
            }
            BezierAlgebraicCuspSemicircleMappedParameterData2::PairOverlapMap {
                overlap,
                source,
                source_first,
            } if overlap.semicircle(!*source_first) == source_carrier => {
                let transformed_overlap = self.overlap(overlap, transform)?;
                let transformed_source =
                    self.parameter(source, overlap.semicircle(*source_first), transform)?;
                Some(BezierAlgebraicCuspSemicircleParameter2::Mapped(Arc::new(
                    BezierAlgebraicCuspSemicircleMappedParameterData2::PairOverlapMap {
                        overlap: transformed_overlap,
                        source: transformed_source,
                        source_first: *source_first,
                    },
                )))
            }
            BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedFiberRational {
                map,
                other_parameter,
                location,
                tangent_cross_sign,
            } if &map.data.semicircle == source_carrier => {
                let transformed_curve = map.data.curve.transform_similarity(transform);
                let orientation_scale = Real::from(if transform.reverses_orientation() {
                    -1_i8
                } else {
                    1_i8
                });
                let transformed_cross =
                    |expression: &BezierAlgebraicCuspTwoTermExpression2| {
                        BezierAlgebraicCuspTwoTermExpression2 {
                            rational: bivariate_scale(
                                expression.rational.clone(),
                                &orientation_scale,
                            ),
                            radical: bivariate_scale(
                                expression.radical.clone(),
                                &orientation_scale,
                            ),
                        }
                    };
                let transformed_map =
                    BezierAlgebraicCuspSemicircleSelectedFiberRationalParameterMap2 {
                        data: Arc::new(
                            BezierAlgebraicCuspSemicircleSelectedFiberRationalParameterMapData2 {
                                semicircle: self.semicircle(&map.data.semicircle, transform)?,
                                curve: transformed_curve,
                                isolated_incidence: map.data.isolated_incidence.clone(),
                                diameter: map.data.diameter.clone(),
                                radius_squared_denominator: map
                                    .data
                                    .radius_squared_denominator
                                    .clone(),
                                speed_squared: map.data.speed_squared.clone(),
                                tangent_cross: transformed_cross(&map.data.tangent_cross),
                                angular_tangent: transformed_cross(&map.data.angular_tangent),
                                policy: map.data.policy,
                                parameter_cache: BezierAlgebraicCuspSemicircleParameterCache2::default(),
                            },
                        ),
                    };
                Some(BezierAlgebraicCuspSemicircleParameter2::Mapped(Arc::new(
                    BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedFiberRational {
                        map: transformed_map,
                        other_parameter: other_parameter.clone(),
                        location: *location,
                        tangent_cross_sign: if transform.reverses_orientation() {
                            product_sign(*tangent_cross_sign, RealSign::Negative)
                        } else {
                            *tangent_cross_sign
                        },
                    },
                )))
            }
            BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedFiberParallel {
                map,
                other_parameter,
                location,
                tangent_cross_sign,
            } if &map.data.semicircle == source_carrier => {
                let transformed_map =
                    BezierAlgebraicCuspSemicircleSelectedFiberParallelParameterMap2 {
                        data: Arc::new(
                            BezierAlgebraicCuspSemicircleSelectedFiberParallelParameterMapData2 {
                                semicircle: self.semicircle(&map.data.semicircle, transform)?,
                                parallel: map.data.parallel.transform_similarity(transform)?,
                                diameter: map.data.diameter.clone(),
                                radius_squared_denominator: map
                                    .data
                                    .radius_squared_denominator
                                    .clone(),
                                tangent_cross_source: if transform.reverses_orientation() {
                                    BezierAlgebraicCuspTwoTermExpression2 {
                                        rational: bivariate_scale(
                                            map.data.tangent_cross_source.rational.clone(),
                                            &Real::from(-1_i8),
                                        ),
                                        radical: bivariate_scale(
                                            map.data.tangent_cross_source.radical.clone(),
                                            &Real::from(-1_i8),
                                        ),
                                    }
                                } else {
                                    map.data.tangent_cross_source.clone()
                                },
                                tangent_dot_source: map.data.tangent_dot_source.clone(),
                                center_speed_squared: map.data.center_speed_squared.clone(),
                                candidate_speed_squared: map.data.candidate_speed_squared.clone(),
                                policy: map.data.policy,
                            },
                        ),
                    };
                Some(BezierAlgebraicCuspSemicircleParameter2::Mapped(Arc::new(
                    BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedFiberParallel {
                        map: transformed_map,
                        other_parameter: other_parameter.clone(),
                        location: *location,
                        tangent_cross_sign: if transform.reverses_orientation() {
                            product_sign(*tangent_cross_sign, RealSign::Negative)
                        } else {
                            *tangent_cross_sign
                        },
                    },
                )))
            }
            BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedParallelContact {
                semicircle,
                parallel,
                parameter,
                location,
                radial_product_sign,
                tangent_cross_sign,
                tangent_dot_sign,
                policy,
            } if semicircle == source_carrier => {
                let transformed_parallel = parallel.transform_similarity(transform)?;
                Some(BezierAlgebraicCuspSemicircleParameter2::Mapped(Arc::new(
                    BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedParallelContact {
                        semicircle: self.semicircle(semicircle, transform)?,
                        parallel: transformed_parallel.clone(),
                        parameter: parameter.clone(),
                        location: *location,
                        radial_product_sign: *radial_product_sign,
                        tangent_cross_sign: if transform.reverses_orientation() {
                            product_sign(*tangent_cross_sign, RealSign::Negative)
                        } else {
                            *tangent_cross_sign
                        },
                        tangent_dot_sign: *tangent_dot_sign,
                        policy: *policy,
                    },
                )))
            }
            BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedCircularTangentContact {
                semicircle,
                companion,
                companion_at_start,
                parallel,
                parameter,
                source_direction,
                radial_product_sign,
                point,
                policy,
            } if semicircle == source_carrier => {
                let transformed_semicircle = self.semicircle(semicircle, transform)?;
                let transformed_companion =
                    companion.transform_similarity_cached(transform, self)?;
                let transformed_parallel = parallel.transform_similarity(transform)?;
                let transformed_point = CurvePoint2::from(
                    BezierSimilarityPoint2::new(point.clone(), transform.clone(), policy),
                );
                Some(BezierAlgebraicCuspSemicircleParameter2::Mapped(Arc::new(
                    BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedCircularTangentContact {
                        semicircle: transformed_semicircle,
                        companion: transformed_companion,
                        companion_at_start: *companion_at_start,
                        parallel: transformed_parallel,
                        parameter: parameter.clone(),
                        source_direction: *source_direction,
                        radial_product_sign: *radial_product_sign,
                        point: transformed_point,
                        policy: *policy,
                    },
                )))
            }
            BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedChordNormalContact {
                semicircle,
                anchor_tangent,
                chord,
                radial_product_sign,
                point,
                policy,
            } if semicircle == source_carrier => match anchor_tangent {
                BezierSelectedChordNormalAnchor2::Represented(anchor_tangent) => {
                    let transformed_point = CurvePoint2::from(
                        BezierSimilarityPoint2::new(point.clone(), transform.clone(), policy),
                    );
                    let transformed_chord = match self.chord(chord, transform, policy)? {
                        Classification::Decided(chord) => chord,
                        Classification::Uncertain(reason) => {
                            return Err(CurveError::Topology(format!(
                                "a selected-circle similarity could not transport its chord evidence: {reason:?}"
                            )));
                        }
                    };
                    let transformed_anchor = transform
                        .transform_vector_coordinates(&anchor_tangent.0, &anchor_tangent.1);
                    Some(BezierAlgebraicCuspSemicircleParameter2::Mapped(Arc::new(
                        BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedChordNormalContact {
                            semicircle: self.semicircle(semicircle, transform)?,
                            anchor_tangent: BezierSelectedChordNormalAnchor2::Represented(
                                transformed_anchor,
                            ),
                            chord: transformed_chord,
                            radial_product_sign: *radial_product_sign,
                            point: transformed_point,
                            policy: *policy,
                        },
                    )))
                }
                BezierSelectedChordNormalAnchor2::RetainedChord(anchor) => {
                    let transformed_point = CurvePoint2::from(
                        BezierSimilarityPoint2::new(point.clone(), transform.clone(), policy),
                    );
                    let transformed_chord = match self.chord(chord, transform, policy)? {
                        Classification::Decided(chord) => chord,
                        Classification::Uncertain(reason) => {
                            return Err(CurveError::Topology(format!(
                                "a selected-circle similarity could not transport its chord evidence: {reason:?}"
                            )));
                        }
                    };
                    let transformed_anchor = match self.chord(anchor, transform, policy)? {
                        Classification::Decided(anchor) => anchor,
                        Classification::Uncertain(reason) => {
                            return Err(CurveError::Topology(format!(
                                "a selected-circle similarity could not transport its chord-normal anchor: {reason:?}"
                            )));
                        }
                    };
                    Some(BezierAlgebraicCuspSemicircleParameter2::Mapped(Arc::new(
                        BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedChordNormalContact {
                            semicircle: self.semicircle(semicircle, transform)?,
                            anchor_tangent: BezierSelectedChordNormalAnchor2::RetainedChord(
                                transformed_anchor,
                            ),
                            chord: transformed_chord,
                            radial_product_sign: *radial_product_sign,
                            point: transformed_point,
                            policy: *policy,
                        },
                    )))
                }
                BezierSelectedChordNormalAnchor2::RetainedCircleChord { .. }
                | BezierSelectedChordNormalAnchor2::RetainedCircleRationalChord { .. } => None,
            },
            BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedChordParallelNormalContact {
                semicircle,
                parallel,
                parallel_parameter,
                chord,
                radial_product_sign,
                point,
                policy,
            } if semicircle == source_carrier => {
                let transformed_chord = match self.chord(chord, transform, policy)? {
                    Classification::Decided(chord) => chord,
                    Classification::Uncertain(reason) => {
                        return Err(CurveError::Topology(format!(
                            "a selected-circle similarity could not transport its chord evidence: {reason:?}"
                        )));
                    }
                };
                Some(BezierAlgebraicCuspSemicircleParameter2::Mapped(Arc::new(
                    BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedChordParallelNormalContact {
                        semicircle: self.semicircle(semicircle, transform)?,
                        parallel: parallel.transform_similarity(transform)?,
                        parallel_parameter: parallel_parameter.clone(),
                        chord: transformed_chord,
                        radial_product_sign: *radial_product_sign,
                        point: CurvePoint2::from(
                            BezierSimilarityPoint2::new(
                                point.clone(),
                                transform.clone(),
                                policy,
                            ),
                        ),
                        policy: *policy,
                    },
                )))
            }
            BezierAlgebraicCuspSemicircleMappedParameterData2::SimilarityTransport {
                semicircle,
                source,
                point,
                policy,
            } if semicircle == source_carrier => {
                let CurvePoint2(CurvePointData2::Similarity(point)) = point else {
                    return Err(CurveError::Topology(
                        "a selected-circle similarity lost its exact transform provenance".into(),
                    ));
                };
                if !policy.accepts_retained_policy(point.data.policy) {
                    return Err(CurveError::Topology(
                        "a selected-circle similarity point crossed predicate policies".into(),
                    ));
                }
                let combined = point.data.transform.then(transform);
                Some(BezierAlgebraicCuspSemicircleParameter2::Mapped(Arc::new(
                    BezierAlgebraicCuspSemicircleMappedParameterData2::SimilarityTransport {
                        semicircle: self.semicircle(semicircle, transform)?,
                        source: source.clone(),
                        point: CurvePoint2::from(
                            BezierSimilarityPoint2::new(
                                point.data.source.clone(),
                                combined,
                                policy,
                            ),
                        ),
                        policy: *policy,
                    },
                )))
            }
            BezierAlgebraicCuspSemicircleMappedParameterData2::Rational { .. }
            | BezierAlgebraicCuspSemicircleMappedParameterData2::Parallel { .. }
            | BezierAlgebraicCuspSemicircleMappedParameterData2::Pair { .. }
            | BezierAlgebraicCuspSemicircleMappedParameterData2::PairOverlap { .. }
            | BezierAlgebraicCuspSemicircleMappedParameterData2::PairOverlapMap { .. } => None,
            BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedFiberRational { .. }
            | BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedFiberParallel { .. }
            | BezierAlgebraicCuspSemicircleMappedParameterData2::Chord { .. }
            | BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedParallelContact {
                ..
            }
            | BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedCircularTangentContact {
                ..
            }
            | BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedPairContact { .. }
            | BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedChordNormalContact {
                ..
            }
            | BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedChordParallelNormalContact {
                ..
            }
            | BezierAlgebraicCuspSemicircleMappedParameterData2::SimilarityTransport { .. }
            | BezierAlgebraicCuspSemicircleMappedParameterData2::Chamfer { .. } => None,
        };
        let transformed = match transformed {
            Some(transformed) => transformed,
            None => {
                let policy = parameter.evidence_policy().ok_or_else(|| {
                    CurveError::Topology(
                        "a mapped selected-circle parameter lost its policy".into(),
                    )
                })?;
                let source_point = if data.semicircle_carrier() == source_carrier {
                    parameter.coincident_point_evidence(source_carrier, &policy)?
                } else {
                    parameter.concentric_offset_point_evidence(
                        data.semicircle_carrier(),
                        source_carrier,
                        &policy,
                    )?
                };
                let point = match source_point {
                    Classification::Decided(Some(point)) => point,
                    Classification::Decided(None) => {
                        return Err(CurveError::Topology(
                            "a mapped selected-circle parameter lost its point evidence".into(),
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Err(CurveError::Topology(format!(
                            "a selected-circle similarity could not retain its point evidence: {reason:?}"
                        )));
                    }
                };
                BezierAlgebraicCuspSemicircleParameter2::Mapped(Arc::new(
                    BezierAlgebraicCuspSemicircleMappedParameterData2::SimilarityTransport {
                        semicircle: self.semicircle(source_carrier, transform)?,
                        source: parameter.clone(),
                        point: CurvePoint2::from(BezierSimilarityPoint2::new(
                            point,
                            transform.clone(),
                            &policy,
                        )),
                        policy,
                    },
                ))
            }
        };
        self.parameters
            .push((data.clone(), source_carrier.clone(), transformed.clone()));
        Ok(transformed)
    }
}

impl BezierParallelAlgebraicCuspFrame2 {
    #[inline]
    pub(super) fn center_parallel_distance(&self) -> Real {
        self.data
            .parallel
            .as_ref()
            .map_or_else(Real::zero, |parallel| parallel.distance().clone())
    }

    /// Recovers an exact cardinal frame certificate from the retained
    /// polynomial normal when construction did not cache one. Only STRICT
    /// zero proofs are accepted because this certificate is reusable.
    pub(super) fn certified_cardinal_normal(&self) -> CurveResult<Option<(i8, i8)>> {
        if let Some(normal) = self.data.cardinal_normal {
            return Ok(Some(normal));
        }
        let parameter = BezierParameter2::Algebraic(self.data.parameter.clone());
        for candidate @ (x, y) in [(1_i8, 0_i8), (-1, 0), (0, 1), (0, -1)] {
            let residual_x = polynomial_subtract(
                &self.data.normal_x_numerator,
                &polynomial_scale(&self.data.denominator, &Real::from(x)),
            );
            let residual_y = polynomial_subtract(
                &self.data.normal_y_numerator,
                &polynomial_scale(&self.data.denominator, &Real::from(y)),
            );
            if signed_coefficients_at_parameter(&residual_x, &parameter, &CurveContext::STRICT)?
                == Classification::Decided(RealSign::Zero)
                && signed_coefficients_at_parameter(&residual_y, &parameter, &CurveContext::STRICT)?
                    == Classification::Decided(RealSign::Zero)
            {
                return Ok(Some(candidate));
            }
        }
        Ok(None)
    }

    pub(super) fn transform_similarity(&self, transform: &Similarity2) -> CurveResult<Self> {
        debug_assert!(self.data.direct_center.is_none() || self.data.parallel.is_none());
        let zero = Real::zero();
        let source_length = self
            .data
            .source_x_numerator
            .len()
            .max(self.data.source_y_numerator.len())
            .max(self.data.denominator.len());
        let mut source_x_numerator = Vec::with_capacity(source_length);
        let mut source_y_numerator = Vec::with_capacity(source_length);
        for index in 0..source_length {
            let (x, y) = transform.transform_homogeneous_coordinates(
                self.data.source_x_numerator.get(index).unwrap_or(&zero),
                self.data.source_y_numerator.get(index).unwrap_or(&zero),
                self.data.denominator.get(index).unwrap_or(&zero),
            );
            source_x_numerator.push(x * transform.scale());
            source_y_numerator.push(y * transform.scale());
        }

        let normal_length = self
            .data
            .normal_x_numerator
            .len()
            .max(self.data.normal_y_numerator.len());
        let orientation = Real::from(if transform.reverses_orientation() {
            -1_i8
        } else {
            1_i8
        });
        let mut normal_x_numerator = Vec::with_capacity(normal_length);
        let mut normal_y_numerator = Vec::with_capacity(normal_length);
        for index in 0..normal_length {
            let (x, y) = transform.transform_vector_coordinates(
                self.data.normal_x_numerator.get(index).unwrap_or(&zero),
                self.data.normal_y_numerator.get(index).unwrap_or(&zero),
            );
            normal_x_numerator.push(&orientation * x);
            normal_y_numerator.push(&orientation * y);
        }

        let cardinal_normal = self.data.cardinal_normal.and_then(|(x, y)| {
            let (x, y) = transform.transform_vector_coordinates(&Real::from(x), &Real::from(y));
            let x = &orientation * x;
            let y = &orientation * y;
            match (
                real_sign(&x, &CurveContext::STRICT),
                real_sign(&y, &CurveContext::STRICT),
            ) {
                (Some(RealSign::Positive), Some(RealSign::Zero)) => Some((1, 0)),
                (Some(RealSign::Negative), Some(RealSign::Zero)) => Some((-1, 0)),
                (Some(RealSign::Zero), Some(RealSign::Positive)) => Some((0, 1)),
                (Some(RealSign::Zero), Some(RealSign::Negative)) => Some((0, -1)),
                _ => None,
            }
        });
        let represented_unit_normal = self
            .data
            .represented_unit_normal
            .as_ref()
            .map(|normal| {
                let (x, y) = transform.transform_vector_coordinates(&normal.0, &normal.1);
                Ok::<_, CurveError>(Arc::new((
                    (&orientation * x / transform.scale())?,
                    (&orientation * y / transform.scale())?,
                )))
            })
            .transpose()?;
        let source_x_numerator = polynomial_trim_structural_zeros(source_x_numerator);
        let source_y_numerator = polynomial_trim_structural_zeros(source_y_numerator);
        let normal_x_numerator = polynomial_trim_structural_zeros(normal_x_numerator);
        let normal_y_numerator = polynomial_trim_structural_zeros(normal_y_numerator);
        let denominator = polynomial_scale(&self.data.denominator, transform.scale());
        let direct_center = self.data.direct_center.as_ref().map(|center| {
            RationalBezierAlgebraicPointImage2::from_retained_expression(
                self.data.parameter.clone(),
                center.parameter().clone(),
                source_x_numerator.clone(),
                source_y_numerator.clone(),
                denominator.clone(),
                "retained an exact transformed algebraic circle center",
            )
        });
        Ok(Self {
            data: Arc::new(BezierParallelAlgebraicCuspFrameData2 {
                parallel: self
                    .data
                    .parallel
                    .as_ref()
                    .map(|parallel| parallel.transform_similarity(transform))
                    .transpose()?,
                cardinal_normal,
                represented_unit_normal,
                direct_center,
                parameter: self.data.parameter.clone(),
                source_x_numerator,
                source_y_numerator,
                normal_x_numerator,
                normal_y_numerator,
                denominator,
            }),
        })
    }

    pub(super) fn point_numerators_at_parallel_distance(
        &self,
        distance: &Real,
    ) -> (Vec<Real>, Vec<Real>) {
        (
            polynomial_add(
                &self.data.source_x_numerator,
                &polynomial_scale(&self.data.normal_x_numerator, distance),
            ),
            polynomial_add(
                &self.data.source_y_numerator,
                &polynomial_scale(&self.data.normal_y_numerator, distance),
            ),
        )
    }

    /// Recovers a represented constant unit normal when both homogeneous
    /// normal numerators are structurally proportional to the frame
    /// denominator. This covers exact similarities of direct cardinal frames
    /// without storing two extra scalars on every selected circle.
    pub(super) fn represented_unit_normal(&self) -> CurveResult<Option<(Real, Real)>> {
        let denominator = &self.data.denominator;
        let Some(pivot) = denominator
            .iter()
            .position(|coefficient| coefficient.zero_status() == ZeroKnowledge::NonZero)
        else {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "selected-circle-represented-normal",
                "no-pivot",
            );
            return Ok(None);
        };
        let zero = Real::zero();
        let proportional_scale = |numerator: &[Real]| -> CurveResult<Option<Real>> {
            let pivot_numerator = numerator.get(pivot).unwrap_or(&zero);
            let scale = (pivot_numerator / &denominator[pivot])?;
            let count = numerator.len().max(denominator.len());
            for index in 0..count {
                let numerator_coefficient = numerator.get(index).unwrap_or(&zero);
                let denominator_coefficient = denominator.get(index).unwrap_or(&zero);
                if Real::diff_of_products(
                    numerator_coefficient,
                    &denominator[pivot],
                    pivot_numerator,
                    denominator_coefficient,
                )
                .zero_status()
                    != ZeroKnowledge::Zero
                {
                    return Ok(None);
                }
            }
            Ok(Some(scale))
        };
        let (Some(x), Some(y)) = (
            proportional_scale(&self.data.normal_x_numerator)?,
            proportional_scale(&self.data.normal_y_numerator)?,
        ) else {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "selected-circle-represented-normal",
                "not-proportional",
            );
            return Ok(None);
        };
        if (Real::dot2_refs([&x, &y], [&x, &y]) - Real::one()).zero_status() != ZeroKnowledge::Zero
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "selected-circle-represented-normal",
                "unit-unresolved",
            );
            return Ok(None);
        }
        Ok(Some((x, y)))
    }

    /// Signs the retained unit normal dotted with a represented vector in the
    /// frame's single algebraic field.  Evaluating the homogeneous numerator
    /// and denominator separately preserves cancellations that are lost when
    /// the center and a displaced endpoint are reconstructed independently.
    pub(super) fn normal_dot_vector_sign(
        &self,
        vector: &(Real, Real),
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        if let Some(normal) = &self.data.represented_unit_normal {
            let negative_normal_x = -normal.0.clone();
            let dot = Real::diff_of_products(&normal.1, &vector.1, &negative_normal_x, &vector.0);
            return Ok(real_sign(&dot, policy)
                .map(Classification::Decided)
                .unwrap_or(Classification::Uncertain(UncertaintyReason::RealSign)));
        }
        let parameter = BezierParameter2::Algebraic(self.data.parameter.clone());
        let numerator = polynomial_add(
            &polynomial_scale(&self.data.normal_x_numerator, &vector.0),
            &polynomial_scale(&self.data.normal_y_numerator, &vector.1),
        );
        let numerator = match signed_coefficients_at_parameter(&numerator, &parameter, policy)? {
            Classification::Decided(RealSign::Zero) => {
                return Ok(Classification::Decided(RealSign::Zero));
            }
            Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let denominator =
            match signed_coefficients_at_parameter(&self.data.denominator, &parameter, policy)? {
                Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
                Classification::Decided(RealSign::Zero) => {
                    return Err(CurveError::Topology(
                        "a selected-circle frame retained a zero homogeneous denominator".into(),
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        Ok(Classification::Decided(product_sign(
            numerator,
            denominator,
        )))
    }

    pub(super) fn point_image_from_frame_scales(
        &self,
        source_scale: &Real,
        normal_scale: &Real,
        tangent_scale: &Real,
        policy: &CurveContext,
    ) -> CurveResult<RationalBezierAlgebraicPointImage2> {
        let x_numerator = polynomial_subtract(
            &polynomial_add(
                &polynomial_scale(&self.data.source_x_numerator, source_scale),
                &polynomial_scale(&self.data.normal_x_numerator, normal_scale),
            ),
            &polynomial_scale(&self.data.normal_y_numerator, tangent_scale),
        );
        let y_numerator = polynomial_add(
            &polynomial_add(
                &polynomial_scale(&self.data.source_y_numerator, source_scale),
                &polynomial_scale(&self.data.normal_y_numerator, normal_scale),
            ),
            &polynomial_scale(&self.data.normal_x_numerator, tangent_scale),
        );
        let denominator = polynomial_scale(&self.data.denominator, source_scale);
        let image = rational_point_image_from_power_basis(
            &self.data.parameter,
            x_numerator.clone(),
            y_numerator.clone(),
            denominator.clone(),
            policy,
        )?;
        Ok(match image {
            Classification::Decided(image) => image,
            Classification::Uncertain(UncertaintyReason::Boundary) => {
                return Err(CurveError::Topology(
                    "a certified selected-circle frame point acquired a zero denominator".into(),
                ));
            }
            Classification::Uncertain(_) => {
                // The selected frame and nonzero source scale already certify this affine denominator.
                RationalBezierAlgebraicPointImage2::from_retained_expression(
                    self.data.parameter.clone(),
                    crate::bezier_algebraic_image::parameter_representation(
                        &self.data.parameter,
                        &policy.strict_counterpart(),
                    ),
                    x_numerator,
                    y_numerator,
                    denominator,
                    "retained a certified algebraic cusp-frame point expression",
                )
            }
        })
    }

    pub(super) fn tangent_image_from_frame_scales(
        &self,
        normal_scale: &Real,
        tangent_scale: &Real,
        denominator_scale: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RationalBezierAlgebraicTangentImage2>> {
        let dx_numerator = polynomial_subtract(
            &polynomial_scale(&self.data.normal_x_numerator, normal_scale),
            &polynomial_scale(&self.data.normal_y_numerator, tangent_scale),
        );
        let dy_numerator = polynomial_add(
            &polynomial_scale(&self.data.normal_y_numerator, normal_scale),
            &polynomial_scale(&self.data.normal_x_numerator, tangent_scale),
        );
        rational_tangent_image_from_power_basis(
            &self.data.parameter,
            dx_numerator,
            dy_numerator,
            polynomial_scale(&self.data.denominator, denominator_scale),
            policy,
        )
    }

    pub(crate) fn point_image_at_parallel_distance(
        &self,
        distance: &Real,
        policy: &CurveContext,
    ) -> CurveResult<RationalBezierAlgebraicPointImage2> {
        self.point_image_from_frame_scales(&Real::one(), distance, &Real::zero(), policy)
    }
}
