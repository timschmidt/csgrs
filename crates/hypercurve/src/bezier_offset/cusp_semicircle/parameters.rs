//! Cusp-semicircle parameters and their chord, mapped and pair parameter maps.

use super::*;

impl BezierAlgebraicCuspSemicircleChordParameterMap2 {
    /// Publishes a completed contact solve with the requirements of both
    /// participating supports and the decisions actually consumed.
    pub(in crate::bezier_offset) fn new(
        semicircle: BezierAlgebraicCuspSemicircle2,
        chord: BezierAlgebraicChord2,
        system: BezierAlgebraicCuspSemicircleChordParameterMapSystem2,
        finite_chord_domain: bool,
        policy: &CurveContext,
    ) -> Self {
        let policy = policy.retained_object_policy_with_dependencies(
            semicircle
                .data
                .frame
                .evidence_policy()
                .into_iter()
                .chain(Some(chord.data.policy)),
        );
        Self {
            data: Arc::new(BezierAlgebraicCuspSemicircleChordParameterMapData2 {
                semicircle,
                chord,
                system,
                finite_chord_domain,
                policy,
                recursive_import_field: OnceLock::new(),
            }),
        }
    }

    pub(in crate::bezier_offset) fn axis_system(
        &self,
    ) -> Option<&BezierAlgebraicCuspSemicircleAxisChordParameterMapSystem2> {
        match &self.data.system {
            BezierAlgebraicCuspSemicircleChordParameterMapSystem2::Axis(system) => Some(system),
            BezierAlgebraicCuspSemicircleChordParameterMapSystem2::Oblique(_)
            | BezierAlgebraicCuspSemicircleChordParameterMapSystem2::RepresentedOblique(_)
            | BezierAlgebraicCuspSemicircleChordParameterMapSystem2::RetainedOffset(_)
            | BezierAlgebraicCuspSemicircleChordParameterMapSystem2::RecursiveQuadraticLine(_)
            | BezierAlgebraicCuspSemicircleChordParameterMapSystem2::SelectedRadial(_)
            | BezierAlgebraicCuspSemicircleChordParameterMapSystem2::ChordNormalProjective(_) => {
                None
            }
        }
    }

    pub(in crate::bezier_offset) fn axis_direction(
        &self,
    ) -> Option<BezierAlgebraicChordAxisDirection2> {
        self.axis_system().map(|system| system.direction)
    }

    pub(in crate::bezier_offset) fn oblique_system(
        &self,
    ) -> Option<&BezierAlgebraicCuspSemicircleObliqueChordParameterMapSystem2> {
        match &self.data.system {
            BezierAlgebraicCuspSemicircleChordParameterMapSystem2::Axis(_)
            | BezierAlgebraicCuspSemicircleChordParameterMapSystem2::RepresentedOblique(_)
            | BezierAlgebraicCuspSemicircleChordParameterMapSystem2::RetainedOffset(_)
            | BezierAlgebraicCuspSemicircleChordParameterMapSystem2::RecursiveQuadraticLine(_)
            | BezierAlgebraicCuspSemicircleChordParameterMapSystem2::SelectedRadial(_)
            | BezierAlgebraicCuspSemicircleChordParameterMapSystem2::ChordNormalProjective(_) => {
                None
            }
            BezierAlgebraicCuspSemicircleChordParameterMapSystem2::Oblique(system) => Some(system),
        }
    }

    pub(in crate::bezier_offset) fn represented_oblique_system(
        &self,
    ) -> Option<&BezierRepresentedCircleChordParameterMapSystem2> {
        match &self.data.system {
            BezierAlgebraicCuspSemicircleChordParameterMapSystem2::RepresentedOblique(system) => {
                Some(system)
            }
            _ => None,
        }
    }

    pub(in crate::bezier_offset) fn represented_oblique_contact(
        &self,
        contact: &BezierAlgebraicCuspSemicircleChordContact2,
    ) -> CurveResult<&BezierRepresentedCircleChordContactData2> {
        let system = self.represented_oblique_system().ok_or_else(|| {
            CurveError::Topology(
                "a nonrepresented cusp/chord map requested represented contact data".into(),
            )
        })?;
        system
            .contacts
            .iter()
            .find(|candidate| candidate.branch == contact.branch)
            .ok_or_else(|| {
                CurveError::Topology(
                    "a represented cusp/chord contact lost its selected branch".into(),
                )
            })
    }

    /// Materializes `C + a(P-C) + b*J(P-C) + T` from the represented cold
    /// path.  Every output coordinate is selected under STRICT before it can
    /// become persistent point evidence.
    #[allow(clippy::too_many_arguments)]
    pub(in crate::bezier_offset) fn represented_oblique_derived_coordinates(
        &self,
        contact: &BezierAlgebraicCuspSemicircleChordContact2,
        radial_scale: &Real,
        perpendicular_scale: &Real,
        translation_x: &Real,
        translation_y: &Real,
    ) -> CurveResult<Classification<[AlgebraicRootRepresentation; 2]>> {
        let system = self.represented_oblique_system().ok_or_else(|| {
            CurveError::Topology(
                "a nonrepresented cusp/chord map requested represented coordinates".into(),
            )
        })?;
        let data = self.represented_oblique_contact(contact)?;
        let center_scale = Real::one() - radial_scale;
        let negative_perpendicular = -perpendicular_scale.clone();
        let x = represented_affine_coordinate(
            &[
                (&data.point[0], radial_scale),
                (&data.point[1], &negative_perpendicular),
                (&system.center[0], &center_scale),
                (&system.center[1], perpendicular_scale),
            ],
            translation_x,
        );
        let y = represented_affine_coordinate(
            &[
                (&data.point[0], perpendicular_scale),
                (&data.point[1], radial_scale),
                (&system.center[0], &negative_perpendicular),
                (&system.center[1], &center_scale),
            ],
            translation_y,
        );
        Ok(match (x, y) {
            (Classification::Decided(x), Classification::Decided(y)) => {
                Classification::Decided([x, y].map(|coordinate| {
                    hypersolve::compact_algebraic_root_low_degree_witness(&coordinate)
                        .unwrap_or(coordinate)
                }))
            }
            (Classification::Uncertain(UncertaintyReason::Unsupported), _)
            | (_, Classification::Uncertain(UncertaintyReason::Unsupported)) => {
                Classification::Uncertain(UncertaintyReason::Unsupported)
            }
            _ => Classification::Uncertain(UncertaintyReason::Predicate),
        })
    }

    /// Materializes two homogeneous expressions from the compact three-root
    /// oblique contact map. The map's contact discriminant is adjoined once,
    /// and both coordinates reuse that selected radical and denominator.
    pub(in crate::bezier_offset) fn oblique_represented_expression_coordinates(
        &self,
        contact: &BezierAlgebraicCuspSemicircleChordContact2,
        expressions: [&SquareRootExpression<TrivariatePolynomial>; 2],
    ) -> CurveResult<Classification<[AlgebraicRootRepresentation; 2]>> {
        let system = self.oblique_system().ok_or_else(|| {
            CurveError::Topology(
                "a nonoblique cusp/chord map requested oblique represented coordinates".into(),
            )
        })?;
        let sources = [
            bezier_parameter_root_representation(&system.first_parameter),
            bezier_parameter_root_representation(&system.second_parameter),
            bezier_parameter_root_representation(&system.cusp_parameter),
        ];
        let dense = |polynomial: &TrivariatePolynomial| {
            dense_reduce_selected_tuple_relations(polynomial.to_dense_polynomial()?, &sources)
        };
        let Some(discriminant) = dense(&system.discriminant)
            .and_then(|polynomial| dense_tensor_with_output_axis(&polynomial))
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let discriminant_value = match represented_dense_value_refined(&discriminant, &sources) {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let radical =
            square_root_algebraic_root_representation(&discriminant_value, contact.branch);
        let signed_radical = match radical.status {
            AlgebraicRootSquareRootStatus::Transformed => radical
                .representation
                .expect("an oblique circle/chord contact retains its selected radical"),
            AlgebraicRootSquareRootStatus::UndecidedSign => {
                return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
            }
            AlgebraicRootSquareRootStatus::InvalidEvidence
            | AlgebraicRootSquareRootStatus::InvalidBranch
            | AlgebraicRootSquareRootStatus::NegativeRadicand
            | AlgebraicRootSquareRootStatus::NonzeroZeroBranch
            | AlgebraicRootSquareRootStatus::InvalidTransformedEvidence => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
        };
        let Some(denominator) = dense(&system.common_denominator)
            .and_then(|polynomial| dense_tensor_with_output_axis(&polynomial))
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some(zero) = DenseTensorPolynomial::zero(vec![1; sources.len() + 1]) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let coordinate = |expression: &SquareRootExpression<TrivariatePolynomial>| {
            let retained = dense(&expression.rational)
                .and_then(|polynomial| dense_tensor_with_output_axis(&polynomial));
            let candidate = dense(&expression.radical)
                .and_then(|polynomial| dense_tensor_with_output_axis(&polynomial));
            match (retained, candidate) {
                (Some(retained), Some(candidate)) => represented_tensor_nested_ratio(
                    &retained,
                    &candidate,
                    &denominator,
                    &zero,
                    &discriminant,
                    &sources,
                    &signed_radical,
                ),
                _ => Classification::Uncertain(UncertaintyReason::Unsupported),
            }
        };
        let [x, y] = expressions.map(coordinate);
        Ok(match (x, y) {
            (Classification::Decided(x), Classification::Decided(y)) => {
                Classification::Decided([x, y].map(|coordinate| {
                    hypersolve::compact_algebraic_root_low_degree_witness(&coordinate)
                        .unwrap_or(coordinate)
                }))
            }
            (Classification::Uncertain(UncertaintyReason::Unsupported), _)
            | (_, Classification::Uncertain(UncertaintyReason::Unsupported)) => {
                Classification::Uncertain(UncertaintyReason::Unsupported)
            }
            _ => Classification::Uncertain(UncertaintyReason::Predicate),
        })
    }

    pub(in crate::bezier_offset) fn oblique_represented_contact_coordinates(
        &self,
        contact: &BezierAlgebraicCuspSemicircleChordContact2,
    ) -> CurveResult<Classification<[AlgebraicRootRepresentation; 2]>> {
        let system = self.oblique_system().ok_or_else(|| {
            CurveError::Topology(
                "a nonoblique cusp/chord map requested oblique contact coordinates".into(),
            )
        })?;
        self.oblique_represented_expression_coordinates(contact, [&system.point_x, &system.point_y])
    }

    #[allow(clippy::too_many_arguments)]
    pub(in crate::bezier_offset) fn oblique_represented_derived_coordinates(
        &self,
        contact: &BezierAlgebraicCuspSemicircleChordContact2,
        radial_scale: &Real,
        perpendicular_scale: &Real,
        translation_x: &Real,
        translation_y: &Real,
    ) -> CurveResult<Classification<[AlgebraicRootRepresentation; 2]>> {
        let system = self.oblique_system().ok_or_else(|| {
            CurveError::Topology(
                "a nonoblique cusp/chord map requested oblique derived coordinates".into(),
            )
        })?;
        let center_x =
            SquareRootExpression::from_rational(system.center_x.clone()).ok_or_else(|| {
                CurveError::Topology("an oblique center exceeded its tensor budget".into())
            })?;
        let center_y =
            SquareRootExpression::from_rational(system.center_y.clone()).ok_or_else(|| {
                CurveError::Topology("an oblique center exceeded its tensor budget".into())
            })?;
        let translation_x = SquareRootExpression::from_rational(
            system
                .common_denominator
                .scale(translation_x)
                .ok_or_else(|| {
                    CurveError::Topology("an oblique translation exceeded its tensor budget".into())
                })?,
        )
        .ok_or_else(|| {
            CurveError::Topology("an oblique translation exceeded its tensor budget".into())
        })?;
        let translation_y = SquareRootExpression::from_rational(
            system
                .common_denominator
                .scale(translation_y)
                .ok_or_else(|| {
                    CurveError::Topology("an oblique translation exceeded its tensor budget".into())
                })?,
        )
        .ok_or_else(|| {
            CurveError::Topology("an oblique translation exceeded its tensor budget".into())
        })?;
        let center_scale = Real::one() - radial_scale;
        let negative_perpendicular = -perpendicular_scale.clone();
        let x = SquareRootExpression::linear_combination(&[
            (&system.point_x, radial_scale),
            (&system.point_y, &negative_perpendicular),
            (&center_x, &center_scale),
            (&center_y, perpendicular_scale),
            (&translation_x, &Real::one()),
        ])
        .ok_or_else(|| {
            CurveError::Topology(
                "an oblique derived x coordinate exceeded its tensor budget".into(),
            )
        })?;
        let y = SquareRootExpression::linear_combination(&[
            (&system.point_x, perpendicular_scale),
            (&system.point_y, radial_scale),
            (&center_x, &negative_perpendicular),
            (&center_y, &center_scale),
            (&translation_y, &Real::one()),
        ])
        .ok_or_else(|| {
            CurveError::Topology(
                "an oblique derived y coordinate exceeded its tensor budget".into(),
            )
        })?;
        self.oblique_represented_expression_coordinates(contact, [&x, &y])
    }

    pub(in crate::bezier_offset) fn represented_oblique_tangent_cross_dot_linear_combination_sign(
        &self,
        contact: &BezierAlgebraicCuspSemicircleChordContact2,
        cross_scale: &Real,
        dot_scale: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let data = self.represented_oblique_contact(contact)?;
        Ok(
            match represented_affine_coordinate(
                &[
                    (&data.tangent_cross, cross_scale),
                    (&data.tangent_dot, dot_scale),
                ],
                &Real::zero(),
            ) {
                Classification::Decided(value) => represented_policy_sign(&value, policy),
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
        )
    }

    /// Materializes one axis-circle contact only when a later recursive
    /// construction needs standalone Cartesian coordinates.
    ///
    /// The hot Boolean path keeps the two selected carrier parameters and the
    /// correlated circle/line radical in their compact map.  A fillet of that
    /// contact can subsequently use the point as a new circle center, at
    /// which point the rank-independent represented kernel needs a persistent
    /// algebraic scalar for each coordinate.  Eliminate the two retained
    /// source roots here and use the contact branch solely to select the exact
    /// square-root sheet; no approximate coordinate enters the construction.
    pub(in crate::bezier_offset) fn axis_represented_coordinates(
        &self,
        contact: &BezierAlgebraicCuspSemicircleChordContact2,
    ) -> CurveResult<Classification<[AlgebraicRootRepresentation; 2]>> {
        let system = self.axis_system().ok_or_else(|| {
            CurveError::Topology(
                "a nonaxis cusp/chord map requested axis contact coordinates".into(),
            )
        })?;
        let sources = [
            bezier_parameter_root_representation(&system.cusp_parameter),
            bezier_parameter_root_representation(&system.support_parameter),
        ];
        let Some(discriminant) = bivariate_tensor_with_output_axis(&system.discriminant) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let discriminant_value = match represented_dense_value_refined(&discriminant, &sources) {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let radical =
            square_root_algebraic_root_representation(&discriminant_value, contact.branch);
        let radical = match radical.status {
            AlgebraicRootSquareRootStatus::Transformed => radical
                .representation
                .expect("an axis circle/chord contact retains its selected radical"),
            AlgebraicRootSquareRootStatus::UndecidedSign => {
                return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
            }
            AlgebraicRootSquareRootStatus::InvalidEvidence
            | AlgebraicRootSquareRootStatus::InvalidBranch
            | AlgebraicRootSquareRootStatus::NegativeRadicand
            | AlgebraicRootSquareRootStatus::NonzeroZeroBranch
            | AlgebraicRootSquareRootStatus::InvalidTransformedEvidence => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
        };
        let Some(denominator) = bivariate_tensor_with_output_axis(&system.common_denominator)
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some(zero) = DenseTensorPolynomial::zero(vec![1, 1, 1]) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let coordinate = |expression: &BezierAlgebraicCuspTwoTermExpression2| {
            let retained = bivariate_tensor_with_output_axis(&expression.rational)?;
            let candidate = bivariate_tensor_with_output_axis(&expression.radical)?;
            Some(represented_tensor_nested_ratio(
                &retained,
                &candidate,
                &denominator,
                &zero,
                &discriminant,
                &sources,
                &radical,
            ))
        };
        let (Some(x), Some(y)) = (coordinate(&system.point_x), coordinate(&system.point_y)) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        Ok(match (x, y) {
            (Classification::Decided(x), Classification::Decided(y)) => {
                Classification::Decided([x, y].map(|coordinate| {
                    hypersolve::compact_algebraic_root_low_degree_witness(&coordinate)
                        .unwrap_or(coordinate)
                }))
            }
            (Classification::Uncertain(UncertaintyReason::Unsupported), _)
            | (_, Classification::Uncertain(UncertaintyReason::Unsupported)) => {
                Classification::Uncertain(UncertaintyReason::Unsupported)
            }
            _ => Classification::Uncertain(UncertaintyReason::Predicate),
        })
    }

    pub(in crate::bezier_offset) fn retained_offset_system(
        &self,
    ) -> Option<&BezierAlgebraicCuspSemicircleRetainedOffsetChordParameterMapSystem2> {
        match &self.data.system {
            BezierAlgebraicCuspSemicircleChordParameterMapSystem2::RetainedOffset(system) => {
                Some(system)
            }
            BezierAlgebraicCuspSemicircleChordParameterMapSystem2::Axis(_)
            | BezierAlgebraicCuspSemicircleChordParameterMapSystem2::Oblique(_)
            | BezierAlgebraicCuspSemicircleChordParameterMapSystem2::RepresentedOblique(_)
            | BezierAlgebraicCuspSemicircleChordParameterMapSystem2::RecursiveQuadraticLine(_)
            | BezierAlgebraicCuspSemicircleChordParameterMapSystem2::SelectedRadial(_)
            | BezierAlgebraicCuspSemicircleChordParameterMapSystem2::ChordNormalProjective(_) => {
                None
            }
        }
    }

    pub(in crate::bezier_offset) fn recursive_quadratic_line_system(
        &self,
    ) -> Option<&BezierRecursiveQuadraticLineParameterMapSystem2> {
        match &self.data.system {
            BezierAlgebraicCuspSemicircleChordParameterMapSystem2::RecursiveQuadraticLine(
                system,
            ) => Some(system),
            BezierAlgebraicCuspSemicircleChordParameterMapSystem2::Axis(_)
            | BezierAlgebraicCuspSemicircleChordParameterMapSystem2::Oblique(_)
            | BezierAlgebraicCuspSemicircleChordParameterMapSystem2::RepresentedOblique(_)
            | BezierAlgebraicCuspSemicircleChordParameterMapSystem2::RetainedOffset(_)
            | BezierAlgebraicCuspSemicircleChordParameterMapSystem2::SelectedRadial(_)
            | BezierAlgebraicCuspSemicircleChordParameterMapSystem2::ChordNormalProjective(_) => {
                None
            }
        }
    }

    pub(in crate::bezier_offset) fn selected_radial_system(
        &self,
    ) -> Option<&BezierSelectedRadialCircleChordParameterMapSystem2> {
        match &self.data.system {
            BezierAlgebraicCuspSemicircleChordParameterMapSystem2::SelectedRadial(system) => {
                Some(system)
            }
            BezierAlgebraicCuspSemicircleChordParameterMapSystem2::Axis(_)
            | BezierAlgebraicCuspSemicircleChordParameterMapSystem2::Oblique(_)
            | BezierAlgebraicCuspSemicircleChordParameterMapSystem2::RepresentedOblique(_)
            | BezierAlgebraicCuspSemicircleChordParameterMapSystem2::RetainedOffset(_)
            | BezierAlgebraicCuspSemicircleChordParameterMapSystem2::RecursiveQuadraticLine(_)
            | BezierAlgebraicCuspSemicircleChordParameterMapSystem2::ChordNormalProjective(_) => {
                None
            }
        }
    }

    pub(in crate::bezier_offset) fn retain_recursive_import_field(
        &self,
        field: RecursiveQuadraticField,
    ) -> RecursiveQuadraticField {
        let _ = self.data.recursive_import_field.set(field);
        self.data
            .recursive_import_field
            .get()
            .expect("a recursive import field was initialized above")
            .clone()
    }

    /// Imports the compact cardinal circle/chord map without eliminating its
    /// two selected source parameters. The contact radical is one positive
    /// base generator; its retained branch is only a coefficient sign.
    pub(in crate::bezier_offset) fn axis_recursive_contact_frame(
        &self,
        contact: &BezierAlgebraicCuspSemicircleChordContact2,
    ) -> CurveResult<Classification<Option<BezierRecursiveQuadraticChordContactFrame2>>> {
        let Some(system) = self.axis_system() else {
            return Ok(Classification::Decided(None));
        };
        if !(-1..=1).contains(&contact.branch) {
            return Err(CurveError::Topology(
                "an axis chord contact retained an invalid radical branch".into(),
            ));
        }
        let sources = vec![
            bezier_parameter_root_representation(&system.cusp_parameter),
            bezier_parameter_root_representation(&system.support_parameter),
        ];
        let dense = |polynomial: &BivariatePolynomial| {
            dense_reduce_selected_tuple_relations(bivariate_dense_tensor(polynomial)?, &sources)
        };
        let field = if let Some(field) = self.data.recursive_import_field.get() {
            field.clone()
        } else {
            let Some(field) = dense(&system.discriminant).and_then(|discriminant| {
                recursive_quadratic_pair_base(sources.clone(), discriminant)
            }) else {
                return Ok(Classification::Decided(None));
            };
            self.retain_recursive_import_field(field)
        };
        let RecursiveQuadraticField::Base(base) = &field else {
            return Err(CurveError::Topology(
                "an axis chord map retained a nonbase recursive import field".into(),
            ));
        };
        let rational = |polynomial: &BivariatePolynomial| {
            recursive_quadratic_rational_value(base, dense(polynomial)?)
        };
        let value = |expression: &BezierAlgebraicCuspTwoTermExpression2| {
            recursive_quadratic_pair_value(
                base,
                dense(&expression.rational)?,
                dense(&expression.radical)?,
                contact.branch,
            )
        };
        let Some(frame) = (|| {
            let denominator = rational(&system.common_denominator)?;
            Some(BezierRecursiveQuadraticChordContactFrame2 {
                field: field.clone(),
                point: BezierRecursiveQuadraticProjectivePoint2 {
                    x: value(&system.point_x)?,
                    y: value(&system.point_y)?,
                    denominator: denominator.clone(),
                },
                center: BezierRecursiveQuadraticProjectivePoint2 {
                    x: rational(&system.center_x)?,
                    y: rational(&system.center_y)?,
                    denominator,
                },
            })
        })() else {
            return Ok(Classification::Decided(None));
        };
        normalize_recursive_contact_frame(frame).map(|frame| frame.map(Some))
    }

    /// Imports the compact three-parameter oblique map into the same
    /// recursive authority used by cardinal and later composed contacts.
    pub(in crate::bezier_offset) fn oblique_recursive_contact_frame(
        &self,
        contact: &BezierAlgebraicCuspSemicircleChordContact2,
    ) -> CurveResult<Classification<Option<BezierRecursiveQuadraticChordContactFrame2>>> {
        let Some(system) = self.oblique_system() else {
            return Ok(Classification::Decided(None));
        };
        if !(-1..=1).contains(&contact.branch) {
            return Err(CurveError::Topology(
                "an oblique chord contact retained an invalid radical branch".into(),
            ));
        }
        let sources = vec![
            bezier_parameter_root_representation(&system.first_parameter),
            bezier_parameter_root_representation(&system.second_parameter),
            bezier_parameter_root_representation(&system.cusp_parameter),
        ];
        let dense = |polynomial: &TrivariatePolynomial| {
            dense_reduce_selected_tuple_relations(polynomial.to_dense_polynomial()?, &sources)
        };
        let field = if let Some(field) = self.data.recursive_import_field.get() {
            field.clone()
        } else {
            let Some(field) = dense(&system.discriminant).and_then(|discriminant| {
                recursive_quadratic_pair_base(sources.clone(), discriminant)
            }) else {
                return Ok(Classification::Decided(None));
            };
            self.retain_recursive_import_field(field)
        };
        let RecursiveQuadraticField::Base(base) = &field else {
            return Err(CurveError::Topology(
                "an oblique chord map retained a nonbase recursive import field".into(),
            ));
        };
        let rational = |polynomial: &TrivariatePolynomial| {
            recursive_quadratic_rational_value(base, dense(polynomial)?)
        };
        let value = |expression: &SquareRootExpression<TrivariatePolynomial>| {
            recursive_quadratic_pair_value(
                base,
                dense(&expression.rational)?,
                dense(&expression.radical)?,
                contact.branch,
            )
        };
        let Some(frame) = (|| {
            let denominator = rational(&system.common_denominator)?;
            Some(BezierRecursiveQuadraticChordContactFrame2 {
                field: field.clone(),
                point: BezierRecursiveQuadraticProjectivePoint2 {
                    x: value(&system.point_x)?,
                    y: value(&system.point_y)?,
                    denominator: denominator.clone(),
                },
                center: BezierRecursiveQuadraticProjectivePoint2 {
                    x: rational(&system.center_x)?,
                    y: rational(&system.center_y)?,
                    denominator,
                },
            })
        })() else {
            return Ok(Classification::Decided(None));
        };
        normalize_recursive_contact_frame(frame).map(|frame| frame.map(Some))
    }

    /// Imports a procedural offset chord as the exact tower already encoded
    /// by its historical map: a positive source-speed base generator followed
    /// by one positive line/circle contact generator.
    pub(in crate::bezier_offset) fn retained_offset_recursive_contact_frame(
        &self,
        contact: &BezierAlgebraicCuspSemicircleChordContact2,
    ) -> CurveResult<Classification<Option<BezierRecursiveQuadraticChordContactFrame2>>> {
        let Some(system) = self.retained_offset_system() else {
            return Ok(Classification::Decided(None));
        };
        if !(-1..=1).contains(&contact.branch) {
            return Err(CurveError::Topology(
                "a retained-offset chord contact retained an invalid radical branch".into(),
            ));
        }
        let sources = vec![
            bezier_parameter_root_representation(&system.first_parameter),
            bezier_parameter_root_representation(&system.second_parameter),
            bezier_parameter_root_representation(&system.cusp_parameter),
        ];
        let dense = |polynomial: &TrivariatePolynomial| {
            dense_reduce_selected_tuple_relations(polynomial.to_dense_polynomial()?, &sources)
        };
        let field = if let Some(field) = self.data.recursive_import_field.get() {
            field.clone()
        } else {
            let Some(base_field) = dense(&system.speed_squared).and_then(|speed_squared| {
                recursive_quadratic_pair_base(sources.clone(), speed_squared)
            }) else {
                return Ok(Classification::Decided(None));
            };
            let RecursiveQuadraticField::Base(base) = &base_field else {
                unreachable!("a retained-offset import begins in its dense base field")
            };
            let speed_value = |expression: &SquareRootExpression<TrivariatePolynomial>| {
                recursive_quadratic_pair_value(
                    base,
                    dense(&expression.rational)?,
                    dense(&expression.radical)?,
                    1,
                )
            };
            let Some(discriminant) = speed_value(&system.contact_discriminant) else {
                return Ok(Classification::Decided(None));
            };
            let candidate = match discriminant.sign(&CurveContext::STRICT)? {
                Classification::Decided(RealSign::Positive) if contact.branch != 0 => {
                    base_field.extension(discriminant).ok_or_else(|| {
                        CurveError::Topology(
                            "a retained-offset contact could not extend its speed field".into(),
                        )
                    })?
                }
                Classification::Decided(RealSign::Zero) if contact.branch == 0 => base_field,
                Classification::Decided(RealSign::Negative) => {
                    return Err(CurveError::Topology(
                        "a retained-offset contact retained a negative discriminant".into(),
                    ));
                }
                Classification::Decided(_) => {
                    return Err(CurveError::Topology(
                        "a retained-offset contact branch disagreed with its discriminant".into(),
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            self.retain_recursive_import_field(candidate)
        };
        let (base, extensions) = field.base_and_extension_path();
        if extensions.len() > 1 {
            return Err(CurveError::Topology(
                "a retained-offset import acquired an unexpected quadratic depth".into(),
            ));
        }
        let speed_value = |expression: &SquareRootExpression<TrivariatePolynomial>| {
            recursive_quadratic_pair_value(
                &base,
                dense(&expression.rational)?,
                dense(&expression.radical)?,
                1,
            )
        };
        let rational = |polynomial: &TrivariatePolynomial| {
            recursive_quadratic_rational_value(&base, dense(polynomial)?)
        };
        let nested = |expression: &BezierAlgebraicCuspRetainedOffsetChordNestedExpression2| {
            let retained = speed_value(&expression.retained)?;
            if contact.branch == 0 {
                return Some(retained);
            }
            field.element(
                retained,
                speed_value(&expression.candidate)?.scale(&Real::from(contact.branch))?,
            )
        };
        let Some(frame) = (|| {
            let denominator = field.lift(&rational(&system.common_denominator)?)?;
            Some(BezierRecursiveQuadraticChordContactFrame2 {
                field: field.clone(),
                point: BezierRecursiveQuadraticProjectivePoint2 {
                    x: nested(&system.point_x)?,
                    y: nested(&system.point_y)?,
                    denominator: denominator.clone(),
                },
                center: BezierRecursiveQuadraticProjectivePoint2 {
                    x: field.lift(&rational(&system.center_x)?)?,
                    y: field.lift(&rational(&system.center_y)?)?,
                    denominator,
                },
            })
        })() else {
            return Ok(Classification::Decided(None));
        };
        normalize_recursive_contact_frame(frame).map(|frame| frame.map(Some))
    }

    /// Imports the historical four-root selected-radial chord map into the
    /// recursive quadratic tower without materializing either contact
    /// coordinate as an independent algebraic number.  The pair-contact
    /// radical becomes the base field's first positive root and the later
    /// line-contact radical becomes one recursive extension.
    pub(in crate::bezier_offset) fn selected_radial_recursive_contact_frame(
        &self,
        contact: &BezierAlgebraicCuspSemicircleChordContact2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierRecursiveQuadraticChordContactFrame2>>> {
        self.validate_policy(policy)?;
        let Some(system) = self.selected_radial_system() else {
            return Ok(Classification::Decided(None));
        };
        if !(-1..=1).contains(&system.pair_branch) || !(-1..=1).contains(&contact.branch) {
            return Err(CurveError::Topology(
                "a selected-radial chord map retained an invalid radical branch".into(),
            ));
        }
        let Some(parameters) = selected_radial_chord_parameters(system) else {
            return Ok(Classification::Decided(None));
        };
        let sources = parameters
            .iter()
            .map(bezier_parameter_root_representation)
            .collect::<Vec<_>>();
        let dense = |polynomial: &DenseTensorPolynomial| {
            dense_reduce_selected_tuple_relations(polynomial.clone(), &sources)
        };
        let field = if let Some(field) = self.data.recursive_import_field.get() {
            field.clone()
        } else {
            let Some(base_field) = dense(&system.pair_discriminant).and_then(|discriminant| {
                recursive_quadratic_pair_base(sources.clone(), discriminant)
            }) else {
                return Ok(Classification::Decided(None));
            };
            let RecursiveQuadraticField::Base(base) = &base_field else {
                unreachable!("a selected-radial import begins in its dense base field")
            };
            let pair_value = |expression: &SquareRootExpression<DenseTensorPolynomial>| {
                recursive_quadratic_pair_value(
                    base,
                    dense(&expression.rational)?,
                    dense(&expression.radical)?,
                    system.pair_branch,
                )
            };
            let Some(discriminant) = pair_value(&system.chord_discriminant) else {
                return Ok(Classification::Decided(None));
            };
            let candidate = match discriminant.sign(&CurveContext::STRICT)? {
                Classification::Decided(RealSign::Positive) if contact.branch != 0 => {
                    base_field.extension(discriminant).ok_or_else(|| {
                        CurveError::Topology(
                            "a selected-radial chord contact could not extend its pair field"
                                .into(),
                        )
                    })?
                }
                Classification::Decided(RealSign::Zero) if contact.branch == 0 => base_field,
                Classification::Decided(RealSign::Negative) => {
                    return Err(CurveError::Topology(
                        "a selected-radial chord contact retained a negative discriminant".into(),
                    ));
                }
                Classification::Decided(_) => {
                    return Err(CurveError::Topology(
                        "a selected-radial contact branch disagreed with its discriminant".into(),
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            self.retain_recursive_import_field(candidate)
        };
        let (base, extensions) = field.base_and_extension_path();
        if extensions.len() > 1 {
            return Err(CurveError::Topology(
                "a selected-radial import acquired an unexpected quadratic depth".into(),
            ));
        }
        let pair_value = |expression: &SquareRootExpression<DenseTensorPolynomial>| {
            recursive_quadratic_pair_value(
                &base,
                dense(&expression.rational)?,
                dense(&expression.radical)?,
                system.pair_branch,
            )
        };
        let rational = |polynomial: &DenseTensorPolynomial| {
            recursive_quadratic_rational_value(&base, dense(polynomial)?)
        };
        let nested = |expression: &BezierSelectedRadialCircleChordNestedExpression2| {
            let retained = pair_value(&expression.retained)?;
            if contact.branch == 0 {
                return Some(retained);
            }
            field.element(
                retained,
                pair_value(&expression.candidate)?.scale(&Real::from(contact.branch))?,
            )
        };
        let Some(frame) = (|| {
            let denominator = field.lift(&rational(&system.common_denominator)?)?;
            Some(BezierRecursiveQuadraticChordContactFrame2 {
                field: field.clone(),
                point: BezierRecursiveQuadraticProjectivePoint2 {
                    x: nested(&system.point_x)?,
                    y: nested(&system.point_y)?,
                    denominator: denominator.clone(),
                },
                center: BezierRecursiveQuadraticProjectivePoint2 {
                    x: field.lift(&pair_value(&system.center_x)?)?,
                    y: field.lift(&pair_value(&system.center_y)?)?,
                    denominator,
                },
            })
        })() else {
            return Ok(Classification::Decided(None));
        };
        normalize_recursive_contact_frame(frame).map(|frame| frame.map(Some))
    }

    /// Reuses the dense chord-normal map's cached contact field and exposes
    /// its point and supporting center through the common projective frame.
    pub(in crate::bezier_offset) fn chord_normal_recursive_contact_frame(
        &self,
        contact: &BezierAlgebraicCuspSemicircleChordContact2,
    ) -> CurveResult<Classification<Option<BezierRecursiveQuadraticChordContactFrame2>>> {
        let Some(system) = self.chord_normal_projective_system() else {
            return Ok(Classification::Decided(None));
        };
        let Some(field) = system.recursive_contact_field(contact)? else {
            return Ok(Classification::Decided(None));
        };
        let RecursiveQuadraticField::Base(base) = &field else {
            return Err(CurveError::Topology(
                "a chord-normal contact retained a nonbase import field".into(),
            ));
        };
        let value = |expression: &TwoSquareRootExpression<DenseTensorPolynomial>| {
            RecursiveQuadraticValue::from_base(base.clone(), expression.clone())
        };
        let Some(frame) = (|| {
            let denominator =
                TwoSquareRootExpression::from_rational(system.geometry.common_denominator.clone())
                    .and_then(|expression| {
                        RecursiveQuadraticValue::from_base(base.clone(), expression)
                    })?;
            Some(BezierRecursiveQuadraticChordContactFrame2 {
                field: field.clone(),
                point: BezierRecursiveQuadraticProjectivePoint2 {
                    x: value(&system.geometry.point_x)?,
                    y: value(&system.geometry.point_y)?,
                    denominator: denominator.clone(),
                },
                center: BezierRecursiveQuadraticProjectivePoint2 {
                    x: value(&system.geometry.center_x)?,
                    y: value(&system.geometry.center_y)?,
                    denominator,
                },
            })
        })() else {
            return Ok(Classification::Decided(None));
        };
        normalize_recursive_contact_frame(frame).map(|frame| frame.map(Some))
    }

    /// Imports the historical represented oblique contact as one selected
    /// recursive projective frame. Coordinate roots remain separate dense
    /// axes, but every candidate is evaluated at their authored isolated
    /// tuple; no primitive element or rounded Cartesian point is constructed.
    pub(in crate::bezier_offset) fn represented_oblique_recursive_contact_frame(
        &self,
        contact: &BezierAlgebraicCuspSemicircleChordContact2,
    ) -> CurveResult<Classification<Option<BezierRecursiveQuadraticChordContactFrame2>>> {
        let Some(system) = self.represented_oblique_system() else {
            return Ok(Classification::Decided(None));
        };
        let contact = self.represented_oblique_contact(contact)?;
        let mut sources = Vec::with_capacity(4);
        for source in system.center.iter().chain(&contact.point) {
            if !sources.contains(source) {
                sources.push(source.clone());
            }
        }
        let Some(one) = DenseTensorPolynomial::try_new(vec![1; sources.len()], vec![Real::one()])
        else {
            return Ok(Classification::Decided(None));
        };
        let Some(field) = RecursiveQuadraticField::base(sources.clone(), one.clone(), one) else {
            return Ok(Classification::Decided(None));
        };
        let RecursiveQuadraticField::Base(base) = &field else {
            unreachable!("a represented contact import begins at its dense base")
        };
        let coordinate = |source: &AlgebraicRootRepresentation| {
            let axis = sources.iter().position(|candidate| candidate == source)?;
            recursive_quadratic_rational_value(
                base,
                DenseTensorPolynomial::from_axis_polynomial(
                    sources.len(),
                    axis,
                    &[Real::zero(), Real::one()],
                )?,
            )
        };
        let Some(frame) = (|| {
            let denominator = field.constant(Real::one())?;
            Some(BezierRecursiveQuadraticChordContactFrame2 {
                field: field.clone(),
                point: BezierRecursiveQuadraticProjectivePoint2 {
                    x: coordinate(&contact.point[0])?,
                    y: coordinate(&contact.point[1])?,
                    denominator: denominator.clone(),
                },
                center: BezierRecursiveQuadraticProjectivePoint2 {
                    x: coordinate(&system.center[0])?,
                    y: coordinate(&system.center[1])?,
                    denominator,
                },
            })
        })() else {
            return Ok(Classification::Decided(None));
        };
        normalize_recursive_contact_frame(frame).map(|frame| frame.map(Some))
    }

    /// Single exact bridge from every compact contact map that naturally
    /// lives in a quadratic tower. Callers consume one point/center frame and
    /// no longer need to know which historical solver authored it.
    pub(in crate::bezier_offset) fn recursive_contact_frame(
        &self,
        contact: &BezierAlgebraicCuspSemicircleChordContact2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierRecursiveQuadraticChordContactFrame2>>> {
        self.validate_policy(policy)?;
        match &self.data.system {
            BezierAlgebraicCuspSemicircleChordParameterMapSystem2::RecursiveQuadraticLine(
                system,
            ) => {
                let retained = system.contact(contact.branch)?;
                let field = retained.point.denominator.field();
                let Some(center) = system.center.lifted_to(&field) else {
                    return Ok(Classification::Decided(None));
                };
                Ok(Classification::Decided(Some(
                    BezierRecursiveQuadraticChordContactFrame2 {
                        field,
                        point: retained.point.clone(),
                        center,
                    },
                )))
            }
            BezierAlgebraicCuspSemicircleChordParameterMapSystem2::SelectedRadial(_) => {
                self.selected_radial_recursive_contact_frame(contact, policy)
            }
            BezierAlgebraicCuspSemicircleChordParameterMapSystem2::ChordNormalProjective(_) => {
                self.chord_normal_recursive_contact_frame(contact)
            }
            BezierAlgebraicCuspSemicircleChordParameterMapSystem2::RepresentedOblique(_) => {
                self.represented_oblique_recursive_contact_frame(contact)
            }
            BezierAlgebraicCuspSemicircleChordParameterMapSystem2::RetainedOffset(_) => {
                self.retained_offset_recursive_contact_frame(contact)
            }
            BezierAlgebraicCuspSemicircleChordParameterMapSystem2::Oblique(_) => {
                self.oblique_recursive_contact_frame(contact)
            }
            BezierAlgebraicCuspSemicircleChordParameterMapSystem2::Axis(_) => {
                self.axis_recursive_contact_frame(contact)
            }
        }
    }

    pub(in crate::bezier_offset) fn chord_normal_projective_system(
        &self,
    ) -> Option<&BezierChordNormalDenseChordParameterMapSystem2> {
        match &self.data.system {
            BezierAlgebraicCuspSemicircleChordParameterMapSystem2::ChordNormalProjective(
                system,
            ) => Some(system),
            _ => None,
        }
    }

    pub(in crate::bezier_offset) fn has_chord_normal_projective_system(&self) -> bool {
        self.chord_normal_projective_system().is_some()
    }

    pub(in crate::bezier_offset) fn chord_normal_projective_parameter<'a>(
        &self,
        contact: &'a BezierAlgebraicCuspSemicircleChordContact2,
    ) -> CurveResult<&'a BezierParameter2> {
        self.has_chord_normal_projective_system()
            .then_some(())
            .ok_or_else(|| {
                CurveError::Topology(
                    "a nonprojective cusp/chord map used the chord-normal tensor kernel".into(),
                )
            })?;
        contact.projective_parameter.as_ref().ok_or_else(|| {
            CurveError::Topology("a chord-normal contact lost its target parameter".into())
        })
    }

    pub(in crate::bezier_offset) fn chord_normal_dense_expression_sign(
        &self,
        contact: &BezierAlgebraicCuspSemicircleChordContact2,
        expression: &TwoSquareRootExpression<DenseTensorPolynomial>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let system = self.chord_normal_projective_system().ok_or_else(|| {
            CurveError::Topology(
                "a nonprojective cusp/chord map used a chord-normal expression".into(),
            )
        })?;
        system.projective.expression_sign(
            expression,
            self.chord_normal_projective_parameter(contact)?,
            policy,
        )
    }

    pub(in crate::bezier_offset) fn chord_normal_dense_derived_coordinate_expression(
        &self,
        axis: Axis2,
        radial_scale: &Real,
        translation: &Real,
    ) -> Option<TwoSquareRootExpression<DenseTensorPolynomial>> {
        let system = self.chord_normal_projective_system()?;
        let (point, center) = match axis {
            Axis2::X => (&system.geometry.point_x, &system.geometry.center_x),
            Axis2::Y => (&system.geometry.point_y, &system.geometry.center_y),
        };
        let translated = TwoSquareRootExpression::from_rational(
            system.geometry.common_denominator.scale(translation)?,
        )?;
        point
            .scale(radial_scale)?
            .add(&center.scale(&(Real::one() - radial_scale))?)?
            .add(&translated)?
            .reduced(&system.projective.source_representations)
    }

    pub(in crate::bezier_offset) fn chord_normal_dense_represented_coordinates(
        &self,
        contact: &BezierAlgebraicCuspSemicircleChordContact2,
    ) -> CurveResult<Classification<[AlgebraicRootRepresentation; 2]>> {
        let system = self.chord_normal_projective_system().ok_or_else(|| {
            CurveError::Topology(
                "a nonprojective cusp/chord map requested chord-normal coordinates".into(),
            )
        })?;
        let parameter = self.chord_normal_projective_parameter(contact)?;
        match represented_chord_parameter_coordinates(
            &self.data.chord,
            parameter,
            &self.data.policy,
        )? {
            Classification::Decided(coordinates) => {
                return Ok(Classification::Decided(coordinates));
            }
            Classification::Uncertain(UncertaintyReason::Predicate) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
            }
            Classification::Uncertain(UncertaintyReason::Unsupported) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        for radical in [
            &system.geometry.point_x.first,
            &system.geometry.point_x.second,
            &system.geometry.point_x.product,
            &system.geometry.point_y.first,
            &system.geometry.point_y.second,
            &system.geometry.point_y.product,
        ] {
            if polynomial_coefficients_are_identically_zero(
                radical.coefficients(),
                &CurveContext::STRICT,
            ) != Classification::Decided(true)
            {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
        }
        let sources = system.projective.sources_with_target(parameter);
        let Some(denominator) = dense_tensor_with_output_axis(&system.geometry.common_denominator)
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let coordinate = |numerator: &DenseTensorPolynomial| {
            let numerator = dense_tensor_with_output_axis(numerator)?;
            Some(represented_tensor_ratio(&numerator, &denominator, &sources))
        };
        let (Some(x), Some(y)) = (
            coordinate(&system.geometry.point_x.rational),
            coordinate(&system.geometry.point_y.rational),
        ) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        Ok(match (x, y) {
            (Classification::Decided(x), Classification::Decided(y)) => {
                Classification::Decided([x, y])
            }
            (Classification::Uncertain(UncertaintyReason::Unsupported), _)
            | (_, Classification::Uncertain(UncertaintyReason::Unsupported)) => {
                Classification::Uncertain(UncertaintyReason::Unsupported)
            }
            _ => Classification::Uncertain(UncertaintyReason::Predicate),
        })
    }

    pub(in crate::bezier_offset) fn selected_radial_nested_sign(
        &self,
        expression: &BezierSelectedRadialCircleChordNestedExpression2,
        branch: i8,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let system = self.selected_radial_system().ok_or_else(|| {
            CurveError::Topology(
                "a non-radial cusp/chord map used the pair-radial predicate kernel".into(),
            )
        })?;
        selected_radial_chord_nested_expression_sign(system, expression, branch, policy)
    }

    pub(in crate::bezier_offset) fn selected_radial_derived_coordinate_expression(
        &self,
        axis: Axis2,
        radial_scale: &Real,
        translation: &Real,
    ) -> Option<BezierSelectedRadialCircleChordNestedExpression2> {
        let system = self.selected_radial_system()?;
        let (point, center) = match axis {
            Axis2::X => (&system.point_x, &system.center_x),
            Axis2::Y => (&system.point_y, &system.center_y),
        };
        let center =
            BezierSelectedRadialCircleChordNestedExpression2::from_retained(center.clone())?;
        let translated = BezierSelectedRadialCircleChordNestedExpression2::from_rational(
            system.common_denominator.scale(translation)?,
        )?;
        BezierSelectedRadialCircleChordNestedExpression2::linear_combination(&[
            (point, radial_scale),
            (&center, &(Real::one() - radial_scale)),
            (&translated, &Real::one()),
        ])
    }

    pub(in crate::bezier_offset) fn trivariate_radical_sign(
        &self,
        expression: &SquareRootExpression<TrivariatePolynomial>,
        branch: i8,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        self.trivariate_radical_components_sign(
            &expression.rational,
            &expression.radical,
            branch,
            policy,
        )
    }

    pub(in crate::bezier_offset) fn trivariate_radical_components_sign(
        &self,
        rational: &TrivariatePolynomial,
        radical: &TrivariatePolynomial,
        branch: i8,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let system = self.oblique_system().ok_or_else(|| {
            CurveError::Topology("an axis cusp/chord map used the oblique radical kernel".into())
        })?;
        algebraic_cusp_trivariate_square_root_components_sign(
            rational,
            radical,
            &system.discriminant,
            &system.first_parameter,
            &system.second_parameter,
            &system.cusp_parameter,
            branch,
            policy,
        )
    }

    pub(in crate::bezier_offset) fn retained_offset_nested_sign(
        &self,
        expression: &BezierAlgebraicCuspRetainedOffsetChordNestedExpression2,
        branch: i8,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let system = self.retained_offset_system().ok_or_else(|| {
            CurveError::Topology(
                "a non-offset cusp/chord map used the retained-offset radical kernel".into(),
            )
        })?;
        retained_offset_chord_nested_expression_sign(system, expression, branch, policy)
    }

    pub(in crate::bezier_offset) fn retained_tangent_cross_dot_linear_combination_sign(
        &self,
        contact: &BezierAlgebraicCuspSemicircleChordContact2,
        cross_scale: &Real,
        dot_scale: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        self.validate_policy(policy)?;
        // Every contact constructor already certifies and retains the exact
        // circle-tangent/chord-tangent cross sign.  When the dot coefficient
        // vanishes (notably at both selected-half endpoints), that sign is
        // the complete predicate authority regardless of the map's scalar
        // representation. Avoid expanding the same fact in a deeper field.
        if dot_scale.zero_status() == ZeroKnowledge::Zero
            && let Some(scale_sign) = real_sign(cross_scale, policy)
        {
            return Ok(Classification::Decided(product_sign(
                contact.tangent_cross_sign,
                scale_sign,
            )));
        }
        if let Some(system) = self.axis_system() {
            // The selected-circle tangent is `turn * perp(P-C)`, up to one
            // positive scale. For cardinal chord tangent D,
            //
            //   cross(T,D) = -turn * dot(P-C,D)
            //   dot(T,D)   =  turn * cross(P-C,D).
            //
            // Sign the requested linear combination in the existing
            // two-field axis system. This keeps a line/circle fillet center's
            // retained axis contact authoritative during terminal-circle
            // reconstruction instead of escalating to a new compositum.
            let (direction_x, direction_y) = system.direction.cardinal_components();
            let turn = Real::from(if self.data.semicircle.is_clockwise() {
                -1_i8
            } else {
                1_i8
            });
            let radial_x_scale = &turn
                * &(dot_scale * Real::from(direction_y) - cross_scale * Real::from(direction_x));
            let radial_y_scale = &turn
                * &(-(dot_scale * Real::from(direction_x)) - cross_scale * Real::from(direction_y));
            let radial_x = BezierAlgebraicCuspTwoTermExpression2 {
                rational: bivariate_subtract(&system.point_x.rational, &system.center_x),
                radical: system.point_x.radical.clone(),
            };
            let radial_y = BezierAlgebraicCuspTwoTermExpression2 {
                rational: bivariate_subtract(&system.point_y.rational, &system.center_y),
                radical: system.point_y.radical.clone(),
            };
            let expression = BezierAlgebraicCuspTwoTermExpression2 {
                rational: bivariate_add(
                    &bivariate_scale(radial_x.rational, &radial_x_scale),
                    &bivariate_scale(radial_y.rational, &radial_y_scale),
                ),
                radical: bivariate_add(
                    &bivariate_scale(radial_x.radical, &radial_x_scale),
                    &bivariate_scale(radial_y.radical, &radial_y_scale),
                ),
            };
            return self.radical_sign(&expression, contact.branch, policy);
        }
        if self.represented_oblique_system().is_some() {
            return self.represented_oblique_tangent_cross_dot_linear_combination_sign(
                contact,
                cross_scale,
                dot_scale,
                policy,
            );
        }
        if let Some(system) = self.recursive_quadratic_line_system() {
            let retained = system.contact(contact.branch)?;
            if cross_scale.zero_status() == ZeroKnowledge::Zero
                && let Some(tangent_dot_sign) = retained.tangent_dot_sign
                && let Some(scale_sign) = real_sign(dot_scale, policy)
            {
                return Ok(Classification::Decided(product_sign(
                    tangent_dot_sign,
                    scale_sign,
                )));
            }
            let turn = Real::from(if self.data.semicircle.is_clockwise() {
                -1_i8
            } else {
                1_i8
            });
            return system.tangent_cross_dot_linear_combination_sign(
                contact,
                cross_scale,
                dot_scale,
                &turn,
                policy,
            );
        }
        let angular_scale = dot_scale
            * Real::from(if self.data.semicircle.is_clockwise() {
                -1_i8
            } else {
                1_i8
            });
        if let Some(system) = self.chord_normal_projective_system() {
            let Some(expression) = system.tangent_cross.scale(cross_scale).and_then(|cross| {
                system
                    .angular_tangent
                    .scale(&angular_scale)
                    .and_then(|dot| cross.add(&dot))
            }) else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            return system.projective.expression_sign(
                &expression,
                self.chord_normal_projective_parameter(contact)?,
                policy,
            );
        }
        let system = self.retained_offset_system().ok_or_else(|| {
            CurveError::Topology(
                "a non-offset cusp/chord map used retained-offset tangent evidence".into(),
            )
        })?;
        if !(-1..=1).contains(&contact.branch) {
            return Err(CurveError::Topology(
                "a retained circle/chord contact lost its support branch".into(),
            ));
        }
        let turn = Real::from(if self.data.semicircle.is_clockwise() {
            -1_i8
        } else {
            1_i8
        });
        let Some(candidate) =
            TrivariatePolynomial::from_axis_polynomial(&[(-turn * cross_scale)], 0)
                .and_then(SquareRootExpression::from_rational)
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some(retained) = system.tangent_dot.scale(dot_scale) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        retained_offset_chord_nested_expression_sign(
            system,
            &BezierAlgebraicCuspRetainedOffsetChordNestedExpression2 {
                retained,
                candidate,
            },
            contact.branch,
            policy,
        )
    }

    pub(in crate::bezier_offset) fn retained_offset_contact_order_to_real(
        &self,
        contact: &BezierAlgebraicCuspSemicircleChordContact2,
        parameter: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        if let Some(order) = algebraic_cusp_semicircle_endpoint_contact_order(
            contact.cusp_location,
            parameter,
            policy,
        ) {
            return Ok(order);
        }
        match in_closed_unit_interval(parameter, policy) {
            Some(true) => {}
            Some(false) => return Err(CurveError::InvalidBezierParameter),
            None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }
        let system = self.retained_offset_system().ok_or_else(|| {
            CurveError::Topology("a non-offset map requested retained-offset ordering".into())
        })?;
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
        let Some(predicate) = system
            .diameter_side
            .scale(&denominator)
            .and_then(|diameter| {
                BezierAlgebraicCuspRetainedOffsetChordNestedExpression2::from_rational(
                    system
                        .radius_squared_denominator
                        .scale(&radial_coefficient)?,
                )
                .and_then(|radius| diameter.subtract(&radius))
            })
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        Ok(
            match self.retained_offset_nested_sign(&predicate, contact.branch, policy)? {
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
            },
        )
    }

    pub(in crate::bezier_offset) fn validate_policy(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<()> {
        if !policy.accepts_retained_policy(self.data.policy) {
            return Err(CurveError::Topology(
                "cusp/chord parameter map was replayed under a different predicate policy".into(),
            ));
        }
        Ok(())
    }

    pub(in crate::bezier_offset) fn radical_sign(
        &self,
        expression: &BezierAlgebraicCuspTwoTermExpression2,
        branch: i8,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let system = self.axis_system().ok_or_else(|| {
            CurveError::Topology("an oblique cusp/chord map used the axis radical kernel".into())
        })?;
        algebraic_cusp_correlated_square_root_sum_sign(
            &system.incidence,
            &algebraic_cusp_branched_expression(expression, branch),
            &system.discriminant,
            &system.cusp_parameter,
            &system.support_parameter,
            policy,
        )
    }

    pub(in crate::bezier_offset) fn contact_order_to_real(
        &self,
        contact: &BezierAlgebraicCuspSemicircleChordContact2,
        parameter: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        if self.represented_oblique_system().is_some() {
            match in_closed_unit_interval(parameter, policy) {
                Some(true) => {}
                Some(false) => return Err(CurveError::InvalidBezierParameter),
                None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
            }
            return match &self.represented_oblique_contact(contact)?.cusp_parameter {
                BezierRepresentedCircleChordAngularParameter2::Materialized(cusp_parameter) => {
                    cusp_parameter
                        .cmp_by_refinement(&BezierParameter2::Exact(parameter.clone()), policy)
                }
                BezierRepresentedCircleChordAngularParameter2::Recursive(cusp_parameter) => {
                    cusp_parameter.order_to_real(parameter, policy)
                }
            };
        }
        if let Some(system) = self.recursive_quadratic_line_system() {
            return system.contact_order_to_real(contact, parameter, policy);
        }
        if let Some(system) = self.chord_normal_projective_system() {
            if let Some(order) = algebraic_cusp_semicircle_endpoint_contact_order(
                contact.cusp_location,
                parameter,
                policy,
            ) {
                return Ok(order);
            }
            match in_closed_unit_interval(parameter, policy) {
                Some(true) => {}
                Some(false) => return Err(CurveError::InvalidBezierParameter),
                None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
            }
            let one_minus = Real::one() - parameter;
            let denominator = &one_minus * &one_minus + parameter * parameter;
            let radial_coefficient = Real::one() - Real::from(2_i8) * parameter;
            let Some(predicate) =
                system
                    .projective
                    .diameter
                    .scale(&denominator)
                    .and_then(|diameter| {
                        system
                            .projective
                            .radius_squared_denominator
                            .scale(&radial_coefficient)
                            .and_then(|radius| diameter.subtract(&radius))
                    })
            else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            return Ok(
                match self.chord_normal_dense_expression_sign(contact, &predicate, policy)? {
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
                },
            );
        }
        if self.retained_offset_system().is_some() {
            return self.retained_offset_contact_order_to_real(contact, parameter, policy);
        }
        if self.oblique_system().is_some() {
            return self.oblique_contact_order_to_real(contact, parameter, policy);
        }
        if self.selected_radial_system().is_some() {
            return self.selected_radial_contact_order_to_real(contact, parameter, policy);
        }
        let Some(system) = self.axis_system() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        if let Some(order) = algebraic_cusp_semicircle_endpoint_contact_order(
            contact.cusp_location,
            parameter,
            policy,
        ) {
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
        let predicate = BezierAlgebraicCuspTwoTermExpression2 {
            rational: bivariate_scaled_difference(
                &system.diameter_side.rational,
                &denominator,
                &system.radius_squared_denominator,
                &radial_coefficient,
            ),
            radical: bivariate_scale(
                system.diameter_side.radical.clone(),
                &(denominator * Real::from(contact.branch)),
            ),
        };
        Ok(match self.radical_sign(&predicate, 1, policy)? {
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

    pub(in crate::bezier_offset) fn selected_radial_contact_order_to_real(
        &self,
        contact: &BezierAlgebraicCuspSemicircleChordContact2,
        parameter: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        if let Some(order) = algebraic_cusp_semicircle_endpoint_contact_order(
            contact.cusp_location,
            parameter,
            policy,
        ) {
            return Ok(order);
        }
        match in_closed_unit_interval(parameter, policy) {
            Some(true) => {}
            Some(false) => return Err(CurveError::InvalidBezierParameter),
            None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }
        let system = self.selected_radial_system().ok_or_else(|| {
            CurveError::Topology("a non-radial map used pair-radial parameter order".into())
        })?;
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
        let Some(predicate) = system.diameter.scale(&denominator).and_then(|diameter| {
            BezierSelectedRadialCircleChordNestedExpression2::from_rational(
                system
                    .radius_squared_denominator
                    .scale(&radial_coefficient)?,
            )
            .and_then(|radius| diameter.subtract(&radius))
        }) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        Ok(
            match self.selected_radial_nested_sign(&predicate, contact.branch, policy)? {
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
            },
        )
    }

    pub(in crate::bezier_offset) fn oblique_contact_order_to_real(
        &self,
        contact: &BezierAlgebraicCuspSemicircleChordContact2,
        parameter: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        if let Some(order) = algebraic_cusp_semicircle_endpoint_contact_order(
            contact.cusp_location,
            parameter,
            policy,
        ) {
            return Ok(order);
        }
        match in_closed_unit_interval(parameter, policy) {
            Some(true) => {}
            Some(false) => return Err(CurveError::InvalidBezierParameter),
            None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }
        let system = self.oblique_system().ok_or_else(|| {
            CurveError::Topology("an axis cusp/chord map used oblique parameter order".into())
        })?;
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
        let Some(rational) =
            system
                .diameter_side
                .rational
                .scale(&denominator)
                .and_then(|diameter| {
                    system
                        .radius_squared_denominator
                        .scale(&radial_coefficient)
                        .and_then(|radius| diameter.subtract(&radius))
                })
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some(radical) = system.diameter_side.radical.scale(&denominator) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        Ok(
            match self.trivariate_radical_sign(
                &SquareRootExpression { rational, radical },
                contact.branch,
                policy,
            )? {
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
            },
        )
    }

    /// Builds both contact carriers from one shared allocation.
    pub(crate) fn contact_evidence(
        &self,
        contact: &BezierAlgebraicCuspSemicircleChordContact2,
    ) -> (BezierAlgebraicCuspSemicircleParameter2, CurvePoint2) {
        let data = Arc::new(BezierAlgebraicCuspSemicircleMappedParameterData2::Chord {
            map: self.clone(),
            contact: contact.clone(),
        });
        let parameter = algebraic_cusp_semicircle_endpoint_parameter(contact.cusp_location)
            .unwrap_or_else(|| BezierAlgebraicCuspSemicircleParameter2::Mapped(data.clone()));
        (
            parameter,
            CurvePoint2::from(BezierAlgebraicCuspChordPoint2 { data }),
        )
    }

    /// Signs one affine expression in the retained contact P and its circle
    /// center C: a.P + b.C + offset. Coordinate, derived-point, and equality
    /// queries share this field dispatch; construction policy only controls
    /// replay authority, while the requested policy controls this decision.
    pub(in crate::bezier_offset) fn affine_order(
        &self,
        contact: &BezierAlgebraicCuspSemicircleChordContact2,
        point_factors: [&Real; 2],
        center_factors: [&Real; 2],
        offset: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        self.validate_policy(policy)?;
        let [px, py] = point_factors;
        let [cx, cy] = center_factors;
        let one = Real::one();
        let sign = if [px, py, cx, cy]
            .iter()
            .all(|factor| factor.zero_status() == ZeroKnowledge::Zero)
        {
            real_sign(offset, policy)
                .map(Classification::Decided)
                .unwrap_or(Classification::Uncertain(UncertaintyReason::RealSign))
        } else if let Some(system) = self.represented_oblique_system() {
            let point = &self.represented_oblique_contact(contact)?.point;
            match represented_affine_coordinate(
                &[
                    (&point[0], px),
                    (&point[1], py),
                    (&system.center[0], cx),
                    (&system.center[1], cy),
                ],
                offset,
            ) {
                Classification::Decided(predicate) => represented_policy_sign(&predicate, policy),
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            }
        } else if self.recursive_quadratic_line_system().is_some()
            || self.selected_radial_system().is_some()
        {
            let frame = match self.recursive_contact_frame(contact, policy)? {
                Classification::Decided(Some(frame)) => frame,
                Classification::Decided(None) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let point = &frame.point;
            let predicate = (|| {
                let predicate = point.linear_numerator(px, py, &(-offset))?;
                if [cx, cy]
                    .iter()
                    .all(|factor| factor.zero_status() == ZeroKnowledge::Zero)
                {
                    return Some(predicate);
                }
                let center = &frame.center;
                let center_predicate = center.x.scale(cx)?.add(&center.y.scale(cy)?)?;
                predicate
                    .multiply(&center.denominator)?
                    .add(&center_predicate.multiply(&point.denominator)?)
            })();
            let Some(predicate) = predicate else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            predicate.sign(policy)?
        } else if let Some(system) = self.chord_normal_projective_system() {
            let predicate = (|| {
                let mut terms = [
                    (&system.geometry.point_x, px),
                    (&system.geometry.point_y, py),
                    (&system.geometry.center_x, cx),
                    (&system.geometry.center_y, cy),
                ]
                .into_iter()
                .filter(|(_, factor)| factor.zero_status() != ZeroKnowledge::Zero);
                let (expression, scale) = terms.next()?;
                let mut numerator = expression.scale(scale)?;
                for (expression, scale) in terms {
                    numerator = numerator.add(&expression.scale(scale)?)?;
                }
                if offset.zero_status() != ZeroKnowledge::Zero {
                    numerator = numerator.add(&TwoSquareRootExpression::from_rational(
                        system.geometry.common_denominator.scale(offset)?,
                    )?)?;
                }
                // The dense denominator need not be positive: N/D has the
                // same sign as N*D, never just the sign of N.
                numerator
                    .multiply_rational(&system.geometry.common_denominator)?
                    .reduced(&system.projective.source_representations)
            })();
            let Some(predicate) = predicate else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            self.chord_normal_dense_expression_sign(contact, &predicate, policy)?
        } else if let Some(system) = self.retained_offset_system() {
            let predicate = (|| {
                let mut predicate =
                    BezierAlgebraicCuspRetainedOffsetChordNestedExpression2::linear_combination(
                        &[(&system.point_x, px), (&system.point_y, py)],
                    )?;
                predicate.retained.rational = TrivariatePolynomial::linear_combination(&[
                    (&predicate.retained.rational, &one),
                    (&system.center_x, cx),
                    (&system.center_y, cy),
                    (&system.common_denominator, offset),
                ])?;
                Some(predicate)
            })();
            let Some(predicate) = predicate else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            self.retained_offset_nested_sign(&predicate, contact.branch, policy)?
        } else if let Some(system) = self.oblique_system() {
            let rational = TrivariatePolynomial::linear_combination(&[
                (&system.point_x.rational, px),
                (&system.point_y.rational, py),
                (&system.center_x, cx),
                (&system.center_y, cy),
                (&system.common_denominator, offset),
            ]);
            let radical = TrivariatePolynomial::linear_combination(&[
                (&system.point_x.radical, px),
                (&system.point_y.radical, py),
            ]);
            let (Some(rational), Some(radical)) = (rational, radical) else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            self.trivariate_radical_components_sign(&rational, &radical, contact.branch, policy)?
        } else if let Some(system) = self.axis_system() {
            let combine = |terms: &[(&BivariatePolynomial, &Real)]| {
                terms
                    .iter()
                    .filter(|(_, scale)| scale.zero_status() != ZeroKnowledge::Zero)
                    .map(|(polynomial, scale)| bivariate_scale((*polynomial).clone(), scale))
                    .reduce(|sum, term| bivariate_add(&sum, &term))
                    .unwrap_or_else(|| BivariatePolynomial::new(vec![vec![Real::zero()]]))
            };
            let predicate = BezierAlgebraicCuspTwoTermExpression2 {
                rational: combine(&[
                    (&system.point_x.rational, px),
                    (&system.point_y.rational, py),
                    (&system.center_x, cx),
                    (&system.center_y, cy),
                    (&system.common_denominator, offset),
                ]),
                radical: combine(&[(&system.point_x.radical, px), (&system.point_y.radical, py)]),
            };
            self.radical_sign(&predicate, contact.branch, policy)?
        } else {
            Classification::Uncertain(UncertaintyReason::Unsupported)
        };
        Ok(sign.map(|sign| match sign {
            RealSign::Negative => std::cmp::Ordering::Less,
            RealSign::Zero => std::cmp::Ordering::Equal,
            RealSign::Positive => std::cmp::Ordering::Greater,
        }))
    }

    pub(in crate::bezier_offset) fn derived_coordinate_expression(
        &self,
        axis: Axis2,
        radial_scale: &Real,
        translation: &Real,
    ) -> BezierAlgebraicCuspTwoTermExpression2 {
        let system = self
            .axis_system()
            .expect("axis-derived cusp/chord point retained an axis map");
        let (point, center) = match axis {
            Axis2::X => (&system.point_x, &system.center_x),
            Axis2::Y => (&system.point_y, &system.center_y),
        };
        let one_minus_scale = Real::one() - radial_scale;
        BezierAlgebraicCuspTwoTermExpression2 {
            rational: bivariate_add(
                &bivariate_add(
                    &bivariate_scale(point.rational.clone(), radial_scale),
                    &bivariate_scale(center.clone(), &one_minus_scale),
                ),
                &bivariate_scale(system.common_denominator.clone(), translation),
            ),
            radical: bivariate_scale(point.radical.clone(), radial_scale),
        }
    }

    pub(in crate::bezier_offset) fn oblique_derived_coordinate_expression(
        &self,
        axis: Axis2,
        radial_scale: &Real,
        translation: &Real,
    ) -> Option<SquareRootExpression<TrivariatePolynomial>> {
        let system = self.oblique_system()?;
        let (point, center) = match axis {
            Axis2::X => (&system.point_x, &system.center_x),
            Axis2::Y => (&system.point_y, &system.center_y),
        };
        let one_minus_scale = Real::one() - radial_scale;
        let rational = TrivariatePolynomial::linear_combination(&[
            (&point.rational, radial_scale),
            (center, &one_minus_scale),
            (&system.common_denominator, translation),
        ])?;
        Some(SquareRootExpression {
            rational,
            radical: point.radical.scale(radial_scale)?,
        })
    }

    pub(in crate::bezier_offset) fn retained_offset_derived_coordinate_expression(
        &self,
        axis: Axis2,
        radial_scale: &Real,
        translation: &Real,
    ) -> Option<BezierAlgebraicCuspRetainedOffsetChordNestedExpression2> {
        let system = self.retained_offset_system()?;
        let (point, center) = match axis {
            Axis2::X => (&system.point_x, &system.center_x),
            Axis2::Y => (&system.point_y, &system.center_y),
        };
        let center =
            BezierAlgebraicCuspRetainedOffsetChordNestedExpression2::from_rational(center.clone())?;
        let translated = BezierAlgebraicCuspRetainedOffsetChordNestedExpression2::from_rational(
            system.common_denominator.scale(translation)?,
        )?;
        BezierAlgebraicCuspRetainedOffsetChordNestedExpression2::linear_combination(&[
            (point, radial_scale),
            (&center, &(Real::one() - radial_scale)),
            (&translated, &Real::one()),
        ])
    }

    pub(in crate::bezier_offset) fn recursive_derived_projective_point(
        &self,
        contact: &BezierAlgebraicCuspSemicircleChordContact2,
        radial_scale: &Real,
        perpendicular_scale: &Real,
        translation_x: &Real,
        translation_y: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierRecursiveQuadraticProjectivePoint2>>> {
        let frame = match self.recursive_contact_frame(contact, policy)? {
            Classification::Decided(Some(frame)) => frame,
            Classification::Decided(None) => {
                return Ok(Classification::Decided(None));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(
            match frame.point.rotated_radial_image(
                &frame.center,
                radial_scale,
                perpendicular_scale,
                translation_x,
                translation_y,
            ) {
                Some(point) => Classification::Decided(Some(point)),
                None => Classification::Uncertain(UncertaintyReason::Unsupported),
            },
        )
    }

    pub(in crate::bezier_offset) fn chord_normal_dense_expressions_bounds_refined(
        &self,
        contact: &BezierAlgebraicCuspSemicircleChordContact2,
        point_x: &TwoSquareRootExpression<DenseTensorPolynomial>,
        point_y: &TwoSquareRootExpression<DenseTensorPolynomial>,
        refinement_steps: usize,
    ) -> Classification<Aabb2> {
        let Some(system) = self.chord_normal_projective_system() else {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        };
        let Ok(target) = self.chord_normal_projective_parameter(contact) else {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        };
        let mut sources = system
            .projective
            .source_representations
            .iter()
            .map(|source| refined_represented_root(source, refinement_steps))
            .collect::<Vec<_>>();
        let target = target
            .clone()
            .refined_isolating_interval(refinement_steps, &CurveContext::STRICT);
        sources.push(bezier_parameter_root_representation(&target));
        let Some(common_denominator) =
            dense_polynomial_value_interval(&system.geometry.common_denominator, &sources)
        else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        let coordinate = |expression| {
            dense_two_positive_square_root_interval(
                expression,
                &system.projective.first_speed_squared,
                &system.projective.second_speed_squared,
                &sources,
            )?
            .divide(&common_denominator)
        };
        let (Some(x), Some(y)) = (coordinate(point_x), coordinate(point_y)) else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        Classification::Decided(Aabb2::new_unchecked(
            Point2::new(x.lower, y.lower),
            Point2::new(x.upper, y.upper),
        ))
    }

    pub(in crate::bezier_offset) fn contact_bounds_refined(
        &self,
        contact: &BezierAlgebraicCuspSemicircleChordContact2,
        refinement_steps: usize,
    ) -> Classification<Aabb2> {
        if self.represented_oblique_system().is_some() {
            let Ok(data) = self.represented_oblique_contact(contact) else {
                return Classification::Uncertain(UncertaintyReason::Unsupported);
            };
            let [x, y] = data
                .point
                .each_ref()
                .map(|coordinate| refined_represented_root(coordinate, refinement_steps));
            return Classification::Decided(Aabb2::new_unchecked(
                Point2::new(x.interval.lower, y.interval.lower),
                Point2::new(x.interval.upper, y.interval.upper),
            ));
        }
        if let Some(system) = self.recursive_quadratic_line_system() {
            return system.contact_bounds_refined(contact, refinement_steps);
        }
        if let Some(system) = self.chord_normal_projective_system() {
            return self.chord_normal_dense_expressions_bounds_refined(
                contact,
                &system.geometry.point_x,
                &system.geometry.point_y,
                refinement_steps,
            );
        }
        if let Some(system) = self.retained_offset_system() {
            return self.retained_offset_expressions_bounds_refined(
                contact,
                &system.point_x,
                &system.point_y,
                refinement_steps,
            );
        }
        if let Some(system) = self.oblique_system() {
            return self.oblique_expressions_bounds_refined(
                contact,
                &system.point_x,
                &system.point_y,
                refinement_steps,
            );
        }
        if let Some(system) = self.selected_radial_system() {
            return self.selected_radial_expressions_bounds_refined(
                contact,
                &system.point_x,
                &system.point_y,
                refinement_steps,
            );
        }
        let Some(system) = self.axis_system() else {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        };
        self.expressions_bounds_refined(contact, &system.point_x, &system.point_y, refinement_steps)
    }

    pub(in crate::bezier_offset) fn derived_contact_bounds_refined(
        &self,
        contact: &BezierAlgebraicCuspSemicircleChordContact2,
        radial_scale: &Real,
        translation_x: &Real,
        translation_y: &Real,
        refinement_steps: usize,
    ) -> Classification<Aabb2> {
        if self.represented_oblique_system().is_some() {
            let point = match self.represented_oblique_derived_coordinates(
                contact,
                radial_scale,
                &Real::zero(),
                translation_x,
                translation_y,
            ) {
                Ok(Classification::Decided(point)) => point,
                Ok(Classification::Uncertain(reason)) => {
                    return Classification::Uncertain(reason);
                }
                Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
            };
            let [x, y] = point
                .each_ref()
                .map(|coordinate| refined_represented_root(coordinate, refinement_steps));
            return Classification::Decided(Aabb2::new_unchecked(
                Point2::new(x.interval.lower, y.interval.lower),
                Point2::new(x.interval.upper, y.interval.upper),
            ));
        }
        if let Some(system) = self.recursive_quadratic_line_system() {
            let Ok(contact) = system.contact(contact.branch) else {
                return Classification::Uncertain(UncertaintyReason::Unsupported);
            };
            // A contact can extend its center's field by a discriminant root.
            // Reuse the projective transform's exact ancestor-field lift.
            let Some(point) = contact.point.rotated_radial_image(
                &system.center,
                radial_scale,
                &Real::zero(),
                translation_x,
                translation_y,
            ) else {
                return Classification::Uncertain(UncertaintyReason::Unsupported);
            };
            return point.bounds_refined(refinement_steps);
        }
        if self.chord_normal_projective_system().is_some() {
            let (Some(point_x), Some(point_y)) = (
                self.chord_normal_dense_derived_coordinate_expression(
                    Axis2::X,
                    radial_scale,
                    translation_x,
                ),
                self.chord_normal_dense_derived_coordinate_expression(
                    Axis2::Y,
                    radial_scale,
                    translation_y,
                ),
            ) else {
                return Classification::Uncertain(UncertaintyReason::Unsupported);
            };
            return self.chord_normal_dense_expressions_bounds_refined(
                contact,
                &point_x,
                &point_y,
                refinement_steps,
            );
        }
        if self.retained_offset_system().is_some() {
            let (Some(point_x), Some(point_y)) = (
                self.retained_offset_derived_coordinate_expression(
                    Axis2::X,
                    radial_scale,
                    translation_x,
                ),
                self.retained_offset_derived_coordinate_expression(
                    Axis2::Y,
                    radial_scale,
                    translation_y,
                ),
            ) else {
                return Classification::Uncertain(UncertaintyReason::Unsupported);
            };
            return self.retained_offset_expressions_bounds_refined(
                contact,
                &point_x,
                &point_y,
                refinement_steps,
            );
        }
        if self.selected_radial_system().is_some() {
            let (Some(point_x), Some(point_y)) = (
                self.selected_radial_derived_coordinate_expression(
                    Axis2::X,
                    radial_scale,
                    translation_x,
                ),
                self.selected_radial_derived_coordinate_expression(
                    Axis2::Y,
                    radial_scale,
                    translation_y,
                ),
            ) else {
                return Classification::Uncertain(UncertaintyReason::Unsupported);
            };
            return self.selected_radial_expressions_bounds_refined(
                contact,
                &point_x,
                &point_y,
                refinement_steps,
            );
        }
        if self.oblique_system().is_some() {
            let (Some(point_x), Some(point_y)) = (
                self.oblique_derived_coordinate_expression(Axis2::X, radial_scale, translation_x),
                self.oblique_derived_coordinate_expression(Axis2::Y, radial_scale, translation_y),
            ) else {
                return Classification::Uncertain(UncertaintyReason::Unsupported);
            };
            return self.oblique_expressions_bounds_refined(
                contact,
                &point_x,
                &point_y,
                refinement_steps,
            );
        }
        if self.axis_system().is_none() {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        }
        let point_x = self.derived_coordinate_expression(Axis2::X, radial_scale, translation_x);
        let point_y = self.derived_coordinate_expression(Axis2::Y, radial_scale, translation_y);
        self.expressions_bounds_refined(contact, &point_x, &point_y, refinement_steps)
    }

    pub(in crate::bezier_offset) fn retained_offset_expressions_bounds_refined(
        &self,
        contact: &BezierAlgebraicCuspSemicircleChordContact2,
        point_x: &BezierAlgebraicCuspRetainedOffsetChordNestedExpression2,
        point_y: &BezierAlgebraicCuspRetainedOffsetChordNestedExpression2,
        refinement_steps: usize,
    ) -> Classification<Aabb2> {
        let Some(system) = self.retained_offset_system() else {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        };
        let first = system
            .first_parameter
            .clone()
            .refined_isolating_interval(refinement_steps, &self.data.policy);
        let second = system
            .second_parameter
            .clone()
            .refined_isolating_interval(refinement_steps, &self.data.policy);
        let cusp = system
            .cusp_parameter
            .clone()
            .refined_isolating_interval(refinement_steps, &self.data.policy);
        let first = real_interval_from_parameter(&first);
        let second = real_interval_from_parameter(&second);
        let cusp = real_interval_from_parameter(&cusp);
        let evaluate = |polynomial: &TrivariatePolynomial| {
            trivariate_power_basis_interval(polynomial, &first, &second, &cusp)
        };
        let Some(speed) =
            evaluate(&system.speed_squared).and_then(|value| value.nonnegative_square_root(None))
        else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        let speed_value = |expression: &SquareRootExpression<TrivariatePolynomial>| {
            let rational = evaluate(&expression.rational)?;
            let radical = evaluate(&expression.radical)?.multiply(&speed)?;
            Some(rational.add(&radical))
        };
        let Some(contact_discriminant) = speed_value(&system.contact_discriminant)
            .and_then(|value| value.nonnegative_square_root(None))
        else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        let branch_interval = |value: RealInterval, branch: i8| {
            if branch < 0 {
                RealInterval {
                    lower: -value.upper,
                    upper: -value.lower,
                }
            } else if branch == 0 {
                RealInterval {
                    lower: Real::zero(),
                    upper: Real::zero(),
                }
            } else {
                value
            }
        };
        let Some(common_denominator) = evaluate(&system.common_denominator) else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        let coordinate = |expression: &BezierAlgebraicCuspRetainedOffsetChordNestedExpression2| {
            let retained = speed_value(&expression.retained)?;
            let candidate = speed_value(&expression.candidate)?.multiply(&contact_discriminant)?;
            retained
                .add(&branch_interval(candidate, contact.branch))
                .divide(&common_denominator)
        };
        let (Some(x), Some(y)) = (coordinate(point_x), coordinate(point_y)) else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        Classification::Decided(Aabb2::new_unchecked(
            Point2::new(x.lower, y.lower),
            Point2::new(x.upper, y.upper),
        ))
    }

    pub(in crate::bezier_offset) fn selected_radial_expressions_bounds_refined(
        &self,
        contact: &BezierAlgebraicCuspSemicircleChordContact2,
        point_x: &BezierSelectedRadialCircleChordNestedExpression2,
        point_y: &BezierSelectedRadialCircleChordNestedExpression2,
        refinement_steps: usize,
    ) -> Classification<Aabb2> {
        let Some(system) = self.selected_radial_system() else {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        };
        let Some([first_cusp_parameter, second_cusp_parameter]) =
            system.pair_map.compact_source_parameters()
        else {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        };
        let parameters = [
            &first_cusp_parameter,
            &second_cusp_parameter,
            &system.first_parameter,
            &system.second_parameter,
        ]
        .map(|parameter| {
            real_interval_from_parameter(
                &parameter
                    .clone()
                    .refined_isolating_interval(refinement_steps, &self.data.policy),
            )
        });
        let evaluate = |polynomial: &DenseTensorPolynomial| {
            quadrivariate_power_basis_interval(
                polynomial,
                [
                    &parameters[0],
                    &parameters[1],
                    &parameters[2],
                    &parameters[3],
                ],
            )
        };
        let Some(pair_discriminant) = evaluate(&system.pair_discriminant)
            .and_then(|value| value.nonnegative_square_root(None))
        else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        let branch_interval = |value: RealInterval, branch: i8| {
            if branch < 0 {
                RealInterval {
                    lower: -value.upper,
                    upper: -value.lower,
                }
            } else if branch == 0 {
                RealInterval {
                    lower: Real::zero(),
                    upper: Real::zero(),
                }
            } else {
                value
            }
        };
        let pair_value = |expression: &SquareRootExpression<DenseTensorPolynomial>| {
            let rational = evaluate(&expression.rational)?;
            let radical = evaluate(&expression.radical)?.multiply(&pair_discriminant)?;
            Some(rational.add(&branch_interval(radical, system.pair_branch)))
        };
        let Some(chord_discriminant) = pair_value(&system.chord_discriminant)
            .and_then(|value| value.nonnegative_square_root(None))
        else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        let Some(common_denominator) = evaluate(&system.common_denominator) else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        let coordinate = |expression: &BezierSelectedRadialCircleChordNestedExpression2| {
            let retained = pair_value(&expression.retained)?;
            let candidate = pair_value(&expression.candidate)?.multiply(&chord_discriminant)?;
            retained
                .add(&branch_interval(candidate, contact.branch))
                .divide(&common_denominator)
        };
        let (Some(x), Some(y)) = (coordinate(point_x), coordinate(point_y)) else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        Classification::Decided(Aabb2::new_unchecked(
            Point2::new(x.lower, y.lower),
            Point2::new(x.upper, y.upper),
        ))
    }

    pub(in crate::bezier_offset) fn expressions_bounds_refined(
        &self,
        contact: &BezierAlgebraicCuspSemicircleChordContact2,
        point_x: &BezierAlgebraicCuspTwoTermExpression2,
        point_y: &BezierAlgebraicCuspTwoTermExpression2,
        refinement_steps: usize,
    ) -> Classification<Aabb2> {
        let Some(system) = self.axis_system() else {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        };
        let cusp = system
            .cusp_parameter
            .clone()
            .refined_isolating_interval(refinement_steps, &self.data.policy);
        let support = system
            .support_parameter
            .clone()
            .refined_isolating_interval(refinement_steps, &self.data.policy);
        let cusp = real_interval_from_parameter(&cusp);
        let support = real_interval_from_parameter(&support);
        let evaluate = |polynomial: &BivariatePolynomial| {
            RealInterval::evaluate_bivariate_power_basis(polynomial, &cusp, &support)
        };
        let Some(discriminant) =
            evaluate(&system.discriminant).and_then(|value| value.nonnegative_square_root(None))
        else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        let Some(common_denominator) = evaluate(&system.common_denominator) else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        let coordinate = |expression: &BezierAlgebraicCuspTwoTermExpression2| {
            let rational = evaluate(&expression.rational)?;
            let radical = evaluate(&expression.radical)?.multiply(&discriminant)?;
            let radical = if contact.branch == -1 {
                RealInterval {
                    lower: -radical.upper,
                    upper: -radical.lower,
                }
            } else if contact.branch == 0 {
                RealInterval {
                    lower: Real::zero(),
                    upper: Real::zero(),
                }
            } else {
                radical
            };
            rational.add(&radical).divide(&common_denominator)
        };
        let (Some(x), Some(y)) = (coordinate(point_x), coordinate(point_y)) else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        Classification::Decided(Aabb2::new_unchecked(
            Point2::new(x.lower, y.lower),
            Point2::new(x.upper, y.upper),
        ))
    }

    pub(in crate::bezier_offset) fn oblique_expressions_bounds_refined(
        &self,
        contact: &BezierAlgebraicCuspSemicircleChordContact2,
        point_x: &SquareRootExpression<TrivariatePolynomial>,
        point_y: &SquareRootExpression<TrivariatePolynomial>,
        refinement_steps: usize,
    ) -> Classification<Aabb2> {
        let Some(system) = self.oblique_system() else {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        };
        let first = system
            .first_parameter
            .clone()
            .refined_isolating_interval(refinement_steps, &self.data.policy);
        let second = system
            .second_parameter
            .clone()
            .refined_isolating_interval(refinement_steps, &self.data.policy);
        let cusp = system
            .cusp_parameter
            .clone()
            .refined_isolating_interval(refinement_steps, &self.data.policy);
        let first = real_interval_from_parameter(&first);
        let second = real_interval_from_parameter(&second);
        let cusp = real_interval_from_parameter(&cusp);
        let evaluate = |polynomial: &TrivariatePolynomial| {
            trivariate_power_basis_interval(polynomial, &first, &second, &cusp)
        };
        let Some(discriminant) =
            evaluate(&system.discriminant).and_then(|value| value.nonnegative_square_root(None))
        else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        let Some(common_denominator) = evaluate(&system.common_denominator) else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        let coordinate = |expression: &SquareRootExpression<TrivariatePolynomial>| {
            let rational = evaluate(&expression.rational)?;
            let radical = evaluate(&expression.radical)?.multiply(&discriminant)?;
            let radical = if contact.branch == -1 {
                RealInterval {
                    lower: -radical.upper,
                    upper: -radical.lower,
                }
            } else if contact.branch == 0 {
                RealInterval {
                    lower: Real::zero(),
                    upper: Real::zero(),
                }
            } else {
                radical
            };
            rational.add(&radical).divide(&common_denominator)
        };
        let (Some(x), Some(y)) = (coordinate(point_x), coordinate(point_y)) else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        Classification::Decided(Aabb2::new_unchecked(
            Point2::new(x.lower, y.lower),
            Point2::new(x.upper, y.upper),
        ))
    }
}

impl BezierAlgebraicCuspSemicircleMappedParameterData2 {
    /// Returns the selected semicircle on which this retained parameter was
    /// authored. This identity prevents a genuinely mapped point from being
    /// replayed after its fragment was transformed without transforming the
    /// correlation itself.
    pub(in crate::bezier_offset) fn semicircle_carrier(&self) -> &BezierAlgebraicCuspSemicircle2 {
        match self {
            Self::Rational { map, .. } => &map.data.semicircle,
            Self::SelectedFiberRational { map, .. } => &map.data.semicircle,
            Self::SelectedFiberParallel { map, .. } => &map.data.semicircle,
            Self::Parallel { map, .. } => &map.data.semicircle,
            Self::SelectedParallelContact { semicircle, .. } => semicircle,
            Self::SelectedCircularTangentContact { semicircle, .. } => semicircle,
            Self::SelectedPairContact { semicircle, .. } => semicircle,
            Self::SelectedChordNormalContact { semicircle, .. }
            | Self::SelectedChordParallelNormalContact { semicircle, .. } => semicircle,
            Self::Pair { map, first, .. } => {
                if *first {
                    &map.data.first_semicircle
                } else {
                    &map.data.second_semicircle
                }
            }
            Self::Chord { map, .. } => &map.data.semicircle,
            Self::PairOverlap { overlap, first, .. } => overlap.semicircle(*first),
            Self::PairOverlapMap {
                overlap,
                source_first,
                ..
            } => overlap.semicircle(!*source_first),
            Self::SimilarityTransport { semicircle, .. } => semicircle,
            Self::Chamfer { semicircle, .. } => semicircle,
        }
    }

    /// Returns point evidence retained by this mapped parameter or by the
    /// physical source of an exact overlap transport.
    pub(in crate::bezier_offset) fn retained_point_evidence(&self) -> Option<&CurvePoint2> {
        match self {
            Self::SelectedCircularTangentContact { point, .. }
            | Self::SelectedPairContact { point, .. }
            | Self::SelectedChordNormalContact { point, .. }
            | Self::SelectedChordParallelNormalContact { point, .. }
            | Self::SimilarityTransport { point, .. }
            | Self::Chamfer { point, .. } => Some(point),
            Self::PairOverlapMap { source, .. } => {
                let BezierAlgebraicCuspSemicircleParameter2::Mapped(source) = source else {
                    return None;
                };
                source.retained_point_evidence()
            }
            Self::Chord { map, contact } => match contact.chord_location {
                BezierAlgebraicCuspSemicircleContactLocation2::Start => {
                    Some(map.data.chord.start())
                }
                BezierAlgebraicCuspSemicircleContactLocation2::End => Some(map.data.chord.end()),
                BezierAlgebraicCuspSemicircleContactLocation2::Interior => None,
            },
            Self::Rational { .. }
            | Self::SelectedFiberRational { .. }
            | Self::SelectedFiberParallel { .. }
            | Self::Parallel { .. }
            | Self::SelectedParallelContact { .. }
            | Self::Pair { .. }
            | Self::PairOverlap { .. } => None,
        }
    }

    /// Publishes point evidence for every mapped parameter that already owns
    /// a compact point carrier. Selected-fiber contacts, including their
    /// coincident-overlap transports, use the same one-word derived-point
    /// representation exposed by their public contact handle; no ordinary
    /// parameter or Cartesian coordinate is constructed.
    pub(in crate::bezier_offset) fn retained_or_selected_point_evidence(
        self: &Arc<Self>,
    ) -> Option<CurvePoint2> {
        if let Some(point) = self.retained_point_evidence() {
            return Some(point.clone());
        }
        match self.as_ref() {
            Self::SelectedFiberRational { .. } | Self::SelectedFiberParallel { .. } => Some(
                CurvePoint2::from(BezierAlgebraicCuspChordDerivedPoint2::from_mapped_source(
                    self.clone(),
                    None,
                    Real::one(),
                )),
            ),
            Self::PairOverlapMap { source, .. } => {
                let BezierAlgebraicCuspSemicircleParameter2::Mapped(source) = source else {
                    return None;
                };
                source.retained_or_selected_point_evidence()
            }
            _ => None,
        }
    }

    pub(in crate::bezier_offset) fn isolated_circle_incidence_parameter(
        &self,
    ) -> Option<&BezierAlgebraicSelectedFiberParameter2> {
        match self {
            Self::SelectedFiberRational {
                map,
                other_parameter,
                ..
            } if map.data.isolated_incidence.as_ref() == Some(&other_parameter.data.authority) => {
                Some(other_parameter)
            }
            Self::PairOverlapMap { source, .. } => {
                let BezierAlgebraicCuspSemicircleParameter2::Mapped(source) = source else {
                    return None;
                };
                source.isolated_circle_incidence_parameter()
            }
            _ => None,
        }
    }

    /// The native source chart already certifying this contact point. Equal
    /// source parameters on equal signed parallels identify the point even
    /// when one contact stores an ordinary root and another a selected fiber.
    pub(in crate::bezier_offset) fn coincident_parametric_source(
        &self,
    ) -> CurveResult<Option<(RationalBezier2, Real, CurveParameter2)>> {
        Ok(Some(match self {
            Self::Rational { map, contact } => (
                map.data.curve.clone(),
                Real::zero(),
                contact.other_parameter.clone(),
            ),
            Self::SelectedFiberRational {
                map,
                other_parameter,
                ..
            } => (
                map.data.curve.clone(),
                Real::zero(),
                CurveParameter2::from_selected_fiber(other_parameter.clone()),
            ),
            Self::SelectedFiberParallel {
                map,
                other_parameter,
                ..
            } => (
                map.data.parallel.source().to_rational_bezier()?,
                map.data.parallel.distance().clone(),
                CurveParameter2::from_selected_fiber(other_parameter.clone()),
            ),
            Self::Parallel { map, contact } => (
                map.data.parallel.source().to_rational_bezier()?,
                map.data.parallel.distance().clone(),
                CurveParameter2::from(contact.parallel_parameter.clone()),
            ),
            Self::SelectedParallelContact {
                parallel,
                parameter,
                ..
            } => (
                parallel.source().to_rational_bezier()?,
                parallel.distance().clone(),
                parameter.clone(),
            ),
            Self::PairOverlapMap { source, .. } => {
                let BezierAlgebraicCuspSemicircleParameter2::Mapped(source) = source else {
                    return Ok(None);
                };
                return source.coincident_parametric_source();
            }
            _ => {
                // Join and tangent-contact maps already retain their exact
                // point. If it is an untranslated, undisplaced analytic
                // source evaluation, preserve that source identity across
                // the angular map instead of comparing two scalar images.
                let Some(CurvePoint2(CurvePointData2::AnalyticParallel(point))) =
                    self.retained_point_evidence()
                else {
                    return Ok(None);
                };
                let Some((parallel, parameter)) = point.native_parallel_evaluation() else {
                    return Ok(None);
                };
                (
                    parallel.source().to_rational_bezier()?,
                    parallel.distance().clone(),
                    parameter,
                )
            }
        }))
    }

    /// Views a selected-fiber source point through the shared analytic-point
    /// carrier, following point-preserving coincident-overlap transports. A
    /// rational source is its exact zero-distance parallel. The view preserves
    /// the selected parameter for local identity and coordinate queries;
    /// consumers can request a recursive or represented field separately.
    pub(in crate::bezier_offset) fn selected_fiber_analytic_point(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Option<BezierAnalyticParallelPoint2>> {
        Ok(match self {
            Self::SelectedFiberRational {
                map,
                other_parameter,
                ..
            } => {
                map.validate_policy(policy)?;
                Some(BezierAnalyticParallelPoint2::new_selected_fiber(
                    map.data.curve.parallel_left(Real::zero())?,
                    other_parameter.clone(),
                    policy,
                ))
            }
            Self::SelectedFiberParallel {
                map,
                other_parameter,
                ..
            } => {
                map.validate_policy(policy)?;
                Some(BezierAnalyticParallelPoint2::new_selected_fiber(
                    map.data.parallel.clone(),
                    other_parameter.clone(),
                    policy,
                ))
            }
            Self::PairOverlapMap {
                overlap, source, ..
            } => {
                if !policy.accepts_retained_policy(overlap.data.policy) {
                    return Err(CurveError::Topology(
                        "mapped overlap point used a different predicate policy".into(),
                    ));
                }
                let BezierAlgebraicCuspSemicircleParameter2::Mapped(source) = source else {
                    return Ok(None);
                };
                return source.selected_fiber_analytic_point(policy);
            }
            _ => None,
        })
    }

    pub(in crate::bezier_offset) fn selected_parallel_contact_order_to_real(
        &self,
        represented: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        let Self::SelectedParallelContact {
            semicircle,
            parallel,
            parameter,
            location,
            radial_product_sign,
            tangent_cross_sign,
            tangent_dot_sign,
            policy: _,
        } = self
        else {
            return Err(CurveError::Topology(
                "a non-selected contact requested selected-frame angular ordering".into(),
            ));
        };
        match in_closed_unit_interval(represented, policy) {
            Some(true) => {}
            Some(false) => return Err(CurveError::InvalidBezierParameter),
            None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }
        if let Some(order) =
            algebraic_cusp_semicircle_endpoint_contact_order(*location, represented, policy)
        {
            return Ok(order);
        }
        let one_minus = Real::one() - represented;
        let radial = Real::one() - Real::from(2_i8) * represented;
        let tangential = Real::from(2_i8) * represented * one_minus;
        let turn = Real::from(if semicircle.is_clockwise() {
            -1_i8
        } else {
            1_i8
        });
        let signed_term =
            |coefficient: &Real, term_sign: RealSign| -> CurveResult<Classification<RealSign>> {
                Ok(match real_sign(coefficient, policy) {
                    Some(coefficient_sign) => {
                        Classification::Decided(product_sign(coefficient_sign, term_sign))
                    }
                    None => Classification::Uncertain(UncertaintyReason::RealSign),
                })
            };
        let certified = if semicircle.data.frame.chord_normal().is_some() {
            None
        } else if tangential.zero_status() == ZeroKnowledge::Zero
            || *tangent_dot_sign == RealSign::Zero
        {
            Some(signed_term(
                &(-(radial.clone() * &turn)),
                *tangent_cross_sign,
            )?)
        } else if radial.zero_status() == ZeroKnowledge::Zero
            || *tangent_cross_sign == RealSign::Zero
        {
            Some(signed_term(&tangential, *tangent_dot_sign)?)
        } else {
            None
        };
        let raw_sign = if let Some(certified) = certified {
            match certified {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        } else if let Some(frame) = semicircle.data.frame.chord_normal() {
            match frame
                .anchor
                .tangent_cross_dot_parallel_source_linear_combination_sign(
                    parallel,
                    parameter,
                    &(-(radial * &turn)),
                    &tangential,
                    policy,
                )? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        } else {
            let anchor = semicircle.source_parallel().ok_or_else(|| {
                CurveError::Topology(
                    "a selected parallel contact lost its source-normal frame".into(),
                )
            })?;
            let anchor_parameter = semicircle.selected_frame_parameter().ok_or_else(|| {
                CurveError::Topology(
                    "a selected parallel contact had no single center parameter".into(),
                )
            })?;
            let anchor_parameter =
                match promote_curve_region_bezier_parameter(&anchor_parameter, policy)? {
                    Classification::Decided(parameter) => parameter,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };

            let anchor_tangent = anchor.differential()?;
            let contact_tangent = parallel.differential()?;
            let tangent_cross = bivariate_subtract(
                &bivariate_outer_product(&anchor_tangent.tangent_x, &contact_tangent.tangent_y),
                &bivariate_outer_product(&anchor_tangent.tangent_y, &contact_tangent.tangent_x),
            );
            let tangent_dot = bivariate_add(
                &bivariate_outer_product(&anchor_tangent.tangent_x, &contact_tangent.tangent_x),
                &bivariate_outer_product(&anchor_tangent.tangent_y, &contact_tangent.tangent_y),
            );
            let predicate = bivariate_subtract(
                &bivariate_scale(tangent_dot, &tangential),
                &bivariate_scale(tangent_cross, &(radial * turn)),
            );
            // This two-axis theorem requires a univariate parameter. Keep
            // the original local contact authority in the published map.
            let parameter = match promote_curve_region_bezier_parameter(parameter, policy)? {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            match signed_bivariate_at_parameter_pair(
                &predicate,
                &anchor_parameter,
                &parameter,
                policy,
            )? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        };
        let sign = if *radial_product_sign == RealSign::Negative {
            product_sign(raw_sign, RealSign::Negative)
        } else {
            raw_sign
        };
        #[cfg(feature = "dispatch-trace")]
        {
            let sign_name = |value| match value {
                RealSign::Negative => "negative",
                RealSign::Zero => "zero",
                RealSign::Positive => "positive",
            };
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "selected-parallel-contact-raw-sign",
                sign_name(raw_sign),
            );
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "selected-parallel-contact-radial-product-sign",
                sign_name(*radial_product_sign),
            );
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "selected-parallel-contact-final-sign",
                sign_name(sign),
            );
        }
        Ok(Classification::Decided(match sign {
            // Positive is sin(represented_angle - contact_angle).
            RealSign::Positive => std::cmp::Ordering::Less,
            RealSign::Negative => std::cmp::Ordering::Greater,
            RealSign::Zero => std::cmp::Ordering::Equal,
        }))
    }

    /// Returns the selected circle's increasing-parameter tangent orientation
    /// relative to the retained contact source tangent.
    ///
    /// The mapped contact radius is `s * left_normal(T_source)`. Rotating it
    /// into the circle tangent gives `-turn * s * T_source`; the retained
    /// radial-product sign and signed circle radius recover `s` without
    /// evaluating either selected point.
    pub(in crate::bezier_offset) fn selected_parallel_contact_source_tangent_dot_sign(
        &self,
        candidate: &BezierParallel2,
        candidate_parameter: &BezierParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<RealSign>>> {
        let Self::SelectedParallelContact {
            semicircle,
            parallel,
            parameter,
            radial_product_sign,
            policy: contact_policy,
            ..
        } = self
        else {
            return Ok(Classification::Decided(None));
        };
        if !policy.accepts_retained_policy(*contact_policy)
            || parallel.source() != candidate.source()
        {
            return Ok(Classification::Decided(None));
        }
        match parameter.same_value(&candidate_parameter.clone().into(), policy)? {
            Classification::Decided(true) => {}
            Classification::Decided(false) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let radial_sign = match real_sign(semicircle.radial_distance(), policy) {
            Some(sign @ (RealSign::Negative | RealSign::Positive)) => sign,
            Some(RealSign::Zero) => {
                return Err(CurveError::Topology(
                    "a selected parallel contact retained a zero circle radius".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        let negative_turn = if semicircle.is_clockwise() {
            RealSign::Positive
        } else {
            RealSign::Negative
        };
        Ok(Classification::Decided(Some(product_sign(
            negative_turn,
            product_sign(radial_sign, *radial_product_sign),
        ))))
    }

    pub(in crate::bezier_offset) fn selected_circular_tangent_contact_order_to_real(
        &self,
        represented: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        let Self::SelectedCircularTangentContact {
            semicircle,
            companion,
            companion_at_start,
            parallel,
            parameter,
            source_direction,
            radial_product_sign,
            ..
        } = self
        else {
            return Err(CurveError::Topology(
                "a non-circular contact requested selected-circle angular ordering".into(),
            ));
        };
        match in_closed_unit_interval(represented, policy) {
            Some(true) => {}
            Some(false) => return Err(CurveError::InvalidBezierParameter),
            None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }
        let one_minus = Real::one() - represented;
        let radial = Real::one() - Real::from(2_i8) * represented;
        let tangential = Real::from(2_i8) * represented * one_minus;
        let turn = Real::from(if semicircle.is_clockwise() {
            -1_i8
        } else {
            1_i8
        });
        // The companion primitive evaluates T_companion x T_anchor. The
        // selected-circle angular predicate is
        //   tangential*(T_anchor dot T_companion)
        //     - radial*turn*(T_anchor x T_companion),
        // hence the positive cross coefficient below after swapping the
        // cross operands.
        let cross_scale = radial * turn;
        let raw_sign = match companion.endpoint_tangent_cross_dot_authored_parallel_contact(
            *companion_at_start,
            *source_direction,
            &cross_scale,
            &tangential,
            policy,
        )? {
            Classification::Decided(Some(sign)) => sign,
            Classification::Decided(None) => {
                match companion.endpoint_tangent_cross_dot_retained_parallel_by_chords(
                    *companion_at_start,
                    parallel,
                    parameter,
                    *source_direction,
                    &cross_scale,
                    &tangential,
                    policy,
                )? {
                    Classification::Decided(Some(sign)) => sign,
                    Classification::Decided(None) => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let sign = product_sign(raw_sign, *radial_product_sign);
        Ok(Classification::Decided(match sign {
            RealSign::Positive => std::cmp::Ordering::Less,
            RealSign::Negative => std::cmp::Ordering::Greater,
            RealSign::Zero => std::cmp::Ordering::Equal,
        }))
    }

    pub(in crate::bezier_offset) fn selected_pair_contact_order_to_real(
        &self,
        represented: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        let Self::SelectedPairContact {
            semicircle,
            map,
            contact,
            anchor_first,
            radial_product_sign,
            ..
        } = self
        else {
            return Err(CurveError::Topology(
                "a non-pair contact requested selected-circle angular ordering".into(),
            ));
        };
        match in_closed_unit_interval(represented, policy) {
            Some(true) => {}
            Some(false) => return Err(CurveError::InvalidBezierParameter),
            None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }
        let one_minus = Real::one() - represented;
        let radial = Real::one() - Real::from(2_i8) * represented;
        let tangential = Real::from(2_i8) * represented * one_minus;
        let turn = semicircle.turn_sign();
        let mut cross_scale = -(radial * turn);
        if !anchor_first {
            cross_scale = -cross_scale;
        }
        let raw_sign = match map.tangent_cross_dot_linear_combination_sign(
            contact,
            &cross_scale,
            &tangential,
            policy,
        )? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let sign = product_sign(raw_sign, *radial_product_sign);
        Ok(Classification::Decided(match sign {
            RealSign::Positive => std::cmp::Ordering::Less,
            RealSign::Negative => std::cmp::Ordering::Greater,
            RealSign::Zero => std::cmp::Ordering::Equal,
        }))
    }

    pub(in crate::bezier_offset) fn selected_chord_normal_contact_order_to_real(
        &self,
        represented: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        let Self::SelectedChordNormalContact {
            semicircle,
            anchor_tangent,
            chord,
            radial_product_sign,
            ..
        } = self
        else {
            return Err(CurveError::Topology(
                "a non-chord contact requested selected-circle angular ordering".into(),
            ));
        };
        match in_closed_unit_interval(represented, policy) {
            Some(true) => {}
            Some(false) => return Err(CurveError::InvalidBezierParameter),
            None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }
        let one_minus = Real::one() - represented;
        let radial = Real::one() - Real::from(2_i8) * represented;
        let tangential = Real::from(2_i8) * represented * one_minus;
        let turn = Real::from(if semicircle.is_clockwise() {
            -1_i8
        } else {
            1_i8
        });
        // For represented radial R(u) and contact radial left_normal(D), the
        // sign of sin(angle(u)-angle(contact)) is the same as
        //   tangential * (T_anchor dot D)
        //     - radial * turn * (T_anchor x D).
        // The positive chord speed cancels. The retained radial-product sign
        // covers either equal signed radii or the complementary chart.
        let cross_scale = -(radial * turn);
        let raw_sign = match match anchor_tangent {
            BezierSelectedChordNormalAnchor2::Represented(anchor_tangent) => chord
                .tangent_cross_dot_vector_linear_combination_sign(
                    anchor_tangent,
                    &cross_scale,
                    &tangential,
                    policy,
                )?,
            BezierSelectedChordNormalAnchor2::RetainedChord(anchor) => anchor
                .tangent_cross_dot_linear_combination_sign(
                    chord,
                    &cross_scale,
                    &tangential,
                    policy,
                )?,
            BezierSelectedChordNormalAnchor2::RetainedCircleChord { map, contact } => {
                // `radial_product_sign` already expresses the contact radius
                // in the target chord's left-normal frame, including any
                // tangent reversal from this mapped anchor chord. Flipping
                // the predicate again would count that reversal twice.
                map.retained_tangent_cross_dot_linear_combination_sign(
                    contact,
                    &cross_scale,
                    &tangential,
                    policy,
                )?
            }
            BezierSelectedChordNormalAnchor2::RetainedCircleRationalChord { map, contact } => map
                .selected_radial_tangent_cross_dot_linear_combination_sign(
                contact,
                &cross_scale,
                &tangential,
                policy,
            )?,
        } {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let sign = product_sign(raw_sign, *radial_product_sign);
        Ok(Classification::Decided(match sign {
            RealSign::Positive => std::cmp::Ordering::Less,
            RealSign::Negative => std::cmp::Ordering::Greater,
            RealSign::Zero => std::cmp::Ordering::Equal,
        }))
    }

    pub(in crate::bezier_offset) fn selected_chord_parallel_normal_contact_order_to_real(
        &self,
        represented: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        let Self::SelectedChordParallelNormalContact {
            semicircle,
            parallel,
            parallel_parameter,
            chord,
            radial_product_sign,
            ..
        } = self
        else {
            return Err(CurveError::Topology(
                "a non-chord contact requested selected parallel-frame angular ordering".into(),
            ));
        };
        match in_closed_unit_interval(represented, policy) {
            Some(true) => {}
            Some(false) => return Err(CurveError::InvalidBezierParameter),
            None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }
        // Construction retains a nonzero tangent cross after selecting the
        // unique half-circle chart, so this mapped contact is strictly inside
        // that chart. Preserve those two endpoint orders without rebuilding
        // the higher-dimensional chord/parallel predicate during splitting.
        if represented.zero_status() == ZeroKnowledge::Zero {
            return Ok(Classification::Decided(std::cmp::Ordering::Greater));
        }
        if (represented - Real::one()).zero_status() == ZeroKnowledge::Zero {
            return Ok(Classification::Decided(std::cmp::Ordering::Less));
        }
        let one_minus = Real::one() - represented;
        let radial = Real::one() - Real::from(2_i8) * represented;
        let tangential = Real::from(2_i8) * represented * one_minus;
        let turn = Real::from(if semicircle.is_clockwise() {
            -1_i8
        } else {
            1_i8
        });
        // The selected chart starts on left_normal(T_source), while the
        // contact radial is left_normal(T_chord).  The angular predicate is
        //   tangential * (T_source dot T_chord)
        //     - radial * turn * (T_source x T_chord).
        // The chord primitive signs the swapped cross product, so its cross
        // coefficient is positive `radial * turn`.
        let raw_sign = match chord.tangent_cross_dot_parallel_source_linear_combination_sign(
            parallel,
            parallel_parameter,
            &(radial * turn),
            &tangential,
            policy,
        )? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let sign = product_sign(raw_sign, *radial_product_sign);
        Ok(Classification::Decided(match sign {
            RealSign::Positive => std::cmp::Ordering::Less,
            RealSign::Negative => std::cmp::Ordering::Greater,
            RealSign::Zero => std::cmp::Ordering::Equal,
        }))
    }

    /// Returns the original mapped carrier below every exact
    /// coincident-circle transport.
    ///
    /// `PairOverlapMap` changes only the selected-circle parameterization; it
    /// does not move the physical point.  Geometry certificates such as a
    /// common cardinal coordinate must therefore be replayed against this
    /// base carrier, even when the destination overlap uses a general
    /// correlated parameter map rather than identity or unit complement.
    pub(in crate::bezier_offset) fn coincident_base_data(&self) -> &Self {
        match self {
            Self::PairOverlapMap {
                source: BezierAlgebraicCuspSemicircleParameter2::Mapped(source),
                ..
            } => source.coincident_base_data(),
            _ => self,
        }
    }

    /// Recovers the rational carrier at the base of any exact coincident-circle
    /// pair-overlap transports.
    ///
    /// A pair-overlap map changes only which cusp semicircle parameterizes the
    /// point; its two supporting circles and the geometric point are certified
    /// identical. Peeling that wrapper is therefore sound for a rational
    /// carrier-to-carrier correspondence and avoids constructing a field for
    /// either cusp root. Other pair evidence does not retain a rational source
    /// point and deliberately remains explicit.
    pub(in crate::bezier_offset) fn coincident_rational_source(
        &self,
    ) -> Option<(
        &BezierAlgebraicCuspSemicircleRationalParameterMap2,
        &BezierAlgebraicCuspSemicircleRationalMapContact2,
    )> {
        match self {
            Self::Rational { map, contact } => Some((map, contact)),
            Self::PairOverlapMap { source, .. } => {
                let BezierAlgebraicCuspSemicircleParameter2::Mapped(source) = source else {
                    return None;
                };
                source.coincident_rational_source()
            }
            Self::Parallel { .. } | Self::Pair { .. } | Self::PairOverlap { .. } => None,
            Self::SelectedFiberRational { .. }
            | Self::SelectedFiberParallel { .. }
            | Self::SelectedParallelContact { .. }
            | Self::SelectedCircularTangentContact { .. }
            | Self::SelectedPairContact { .. }
            | Self::SelectedChordNormalContact { .. }
            | Self::SelectedChordParallelNormalContact { .. }
            | Self::Chord { .. }
            | Self::SimilarityTransport { .. }
            | Self::Chamfer { .. } => None,
        }
    }

    /// Recovers a pair contact beneath coincident-circle transports together
    /// with the orientation from its original carrier to the current one.
    pub(in crate::bezier_offset) fn coincident_pair_source(
        &self,
    ) -> Option<(
        &BezierAlgebraicCuspSemicirclePairParameterMap2,
        &BezierAlgebraicCuspSemicirclePairContact2,
        bool,
        bool,
    )> {
        match self {
            Self::Pair {
                map,
                contact,
                first,
            } => Some((map, contact, *first, false)),
            Self::PairOverlapMap {
                overlap, source, ..
            } => {
                let BezierAlgebraicCuspSemicircleParameter2::Mapped(source) = source else {
                    return None;
                };
                let (map, contact, first, reversed) = source.coincident_pair_source()?;
                Some((
                    map,
                    contact,
                    first,
                    reversed ^ (overlap.data.orientation == CurveOverlapOrientation2::Reversed),
                ))
            }
            Self::Rational { .. } | Self::Parallel { .. } | Self::PairOverlap { .. } => None,
            Self::SelectedFiberRational { .. }
            | Self::SelectedFiberParallel { .. }
            | Self::SelectedParallelContact { .. }
            | Self::SelectedCircularTangentContact { .. }
            | Self::SelectedPairContact { .. }
            | Self::SelectedChordNormalContact { .. }
            | Self::SelectedChordParallelNormalContact { .. }
            | Self::Chord { .. }
            | Self::SimilarityTransport { .. }
            | Self::Chamfer { .. } => None,
        }
    }

    /// Recovers a pair-native fillet contact beneath coincident-circle
    /// transports. The returned orientation records whether the transported
    /// carrier tangent is reversed relative to the circle that authored the
    /// selected pair contact.
    pub(in crate::bezier_offset) fn coincident_selected_pair_contact(
        &self,
    ) -> Option<(
        &BezierAlgebraicCuspSemicircle2,
        &BezierAlgebraicCuspSemicirclePairParameterMap2,
        &BezierAlgebraicCuspSemicirclePairContact2,
        bool,
        RealSign,
        CurveContext,
        bool,
    )> {
        match self {
            Self::SelectedPairContact {
                semicircle,
                map,
                contact,
                anchor_first,
                radial_product_sign,
                policy,
                ..
            } => Some((
                semicircle,
                map,
                contact,
                *anchor_first,
                *radial_product_sign,
                *policy,
                false,
            )),
            Self::PairOverlapMap {
                overlap, source, ..
            } => {
                let BezierAlgebraicCuspSemicircleParameter2::Mapped(source) = source else {
                    return None;
                };
                let (semicircle, map, contact, anchor_first, radial_product_sign, policy, reversed) =
                    source.coincident_selected_pair_contact()?;
                Some((
                    semicircle,
                    map,
                    contact,
                    anchor_first,
                    radial_product_sign,
                    policy,
                    reversed ^ (overlap.data.orientation == CurveOverlapOrientation2::Reversed),
                ))
            }
            Self::Rational { .. }
            | Self::SelectedFiberRational { .. }
            | Self::SelectedFiberParallel { .. }
            | Self::Parallel { .. }
            | Self::SelectedParallelContact { .. }
            | Self::SelectedCircularTangentContact { .. }
            | Self::SelectedChordNormalContact { .. }
            | Self::SelectedChordParallelNormalContact { .. }
            | Self::Pair { .. }
            | Self::Chord { .. }
            | Self::PairOverlap { .. }
            | Self::SimilarityTransport { .. }
            | Self::Chamfer { .. } => None,
        }
    }

    /// Recovers a selected-circle/chord contact beneath any number of exact
    /// coincident-circle transports. The Boolean overlap orientation records
    /// whether the destination traversal tangent is reversed relative to the
    /// circle that authored the chord contact.
    pub(in crate::bezier_offset) fn coincident_chord_source(
        &self,
    ) -> Option<(
        &BezierAlgebraicCuspSemicircleChordParameterMap2,
        &BezierAlgebraicCuspSemicircleChordContact2,
        bool,
    )> {
        match self {
            Self::Chord { map, contact } => Some((map, contact, false)),
            Self::PairOverlapMap {
                overlap, source, ..
            } => {
                let BezierAlgebraicCuspSemicircleParameter2::Mapped(source) = source else {
                    return None;
                };
                let (map, contact, reversed) = source.coincident_chord_source()?;
                Some((
                    map,
                    contact,
                    reversed ^ (overlap.data.orientation == CurveOverlapOrientation2::Reversed),
                ))
            }
            Self::Rational { .. }
            | Self::SelectedFiberRational { .. }
            | Self::SelectedFiberParallel { .. }
            | Self::Parallel { .. }
            | Self::SelectedParallelContact { .. }
            | Self::SelectedCircularTangentContact { .. }
            | Self::SelectedPairContact { .. }
            | Self::SelectedChordNormalContact { .. }
            | Self::SelectedChordParallelNormalContact { .. }
            | Self::Pair { .. }
            | Self::PairOverlap { .. }
            | Self::SimilarityTransport { .. }
            | Self::Chamfer { .. } => None,
        }
    }

    /// Recovers only the chord topology needed to author an exact endpoint
    /// tangent. A represented support-line fast path deliberately keeps its
    /// point in the smaller rational map, so it carries this authority without
    /// pretending to own the full multi-field chord point system.
    pub(in crate::bezier_offset) fn coincident_chord_tangent_source(
        &self,
    ) -> Option<(
        &BezierAlgebraicCuspSemicircle2,
        &BezierAlgebraicChord2,
        RealSign,
        CurveContext,
        bool,
    )> {
        match self {
            Self::Rational { map, contact } => match &contact.correlation {
                BezierAlgebraicCuspSemicircleRationalCorrelation2::MapWithChordTangent {
                    chord,
                    circle_cross_chord,
                } => Some((
                    &map.data.semicircle,
                    chord,
                    *circle_cross_chord,
                    map.data.policy,
                    false,
                )),
                _ => None,
            },
            Self::Chord { map, contact } => Some((
                &map.data.semicircle,
                &map.data.chord,
                contact.tangent_cross_sign,
                map.data.policy,
                false,
            )),
            Self::SelectedChordNormalContact {
                semicircle,
                chord,
                policy,
                ..
            }
            | Self::SelectedChordParallelNormalContact {
                semicircle,
                chord,
                policy,
                ..
            } => Some((semicircle, chord, RealSign::Zero, *policy, false)),
            Self::PairOverlapMap {
                overlap,
                source,
                source_first,
            } => {
                let BezierAlgebraicCuspSemicircleParameter2::Mapped(source) = source else {
                    return None;
                };
                let (_, chord, cross, policy, reversed) =
                    source.coincident_chord_tangent_source()?;
                let policy = if overlap.data.policy.accepts_retained_policy(policy) {
                    overlap.data.policy
                } else if policy.accepts_retained_policy(overlap.data.policy) {
                    policy
                } else {
                    return None;
                };
                Some((
                    overlap.semicircle(!*source_first),
                    chord,
                    cross,
                    policy,
                    reversed ^ (overlap.data.orientation == CurveOverlapOrientation2::Reversed),
                ))
            }
            Self::Parallel { .. }
            | Self::SelectedFiberRational { .. }
            | Self::SelectedFiberParallel { .. }
            | Self::SelectedParallelContact { .. }
            | Self::SelectedCircularTangentContact { .. }
            | Self::SelectedPairContact { .. }
            | Self::Pair { .. }
            | Self::PairOverlap { .. }
            | Self::SimilarityTransport { .. }
            | Self::Chamfer { .. } => None,
        }
    }

    /// Returns the original one-word chord-contact parameter allocation below
    /// coincident-circle wrappers. Keeping that allocation beside the current
    /// mapped parameter preserves both the original point system and the
    /// destination selected-circle center without constructing a compositum.
    pub(in crate::bezier_offset) fn coincident_chord_parameter(
        self: &Arc<Self>,
    ) -> Option<Arc<Self>> {
        match self.as_ref() {
            Self::Chord { .. } => Some(self.clone()),
            Self::PairOverlapMap { source, .. } => {
                let BezierAlgebraicCuspSemicircleParameter2::Mapped(source) = source else {
                    return None;
                };
                source.coincident_chord_parameter()
            }
            Self::Rational { .. }
            | Self::SelectedFiberRational { .. }
            | Self::SelectedFiberParallel { .. }
            | Self::Parallel { .. }
            | Self::SelectedParallelContact { .. }
            | Self::SelectedCircularTangentContact { .. }
            | Self::SelectedPairContact { .. }
            | Self::SelectedChordNormalContact { .. }
            | Self::SelectedChordParallelNormalContact { .. }
            | Self::Pair { .. }
            | Self::PairOverlap { .. }
            | Self::SimilarityTransport { .. }
            | Self::Chamfer { .. } => None,
        }
    }

    /// Returns whether two mapped parameters name the same physical
    /// circle-circle contact, regardless of which participating semicircle
    /// owns the local parameter or how many coincident-circle transports wrap
    /// it. This identity is valid for the original contact point only; radial
    /// offsets about the two different circle centers must not reuse it.
    pub(in crate::bezier_offset) fn shares_coincident_pair_point(&self, other: &Self) -> bool {
        let (Some((first_map, first_contact, _, _)), Some((second_map, second_contact, _, _))) = (
            self.coincident_pair_source(),
            other.coincident_pair_source(),
        ) else {
            return false;
        };
        Arc::ptr_eq(&first_map.data, &second_map.data) && first_contact == second_contact
    }

    /// Recovers the exact endpoint carrier beneath coincident-circle overlap
    /// transports. A standalone overlap boundary is always endpoint zero or
    /// one of one participating semicircle, even when its parameter on the
    /// other carrier is non-rational.
    pub(in crate::bezier_offset) fn coincident_pair_endpoint_source(
        &self,
    ) -> Option<(&BezierAlgebraicCuspSemicircle2, bool, CurveContext)> {
        match self {
            Self::PairOverlap {
                overlap, endpoint, ..
            } => {
                let (semicircle, start) = match endpoint {
                    BezierAlgebraicCuspSemicirclePairEndpoint2::FirstStart => {
                        (overlap.semicircle(true), true)
                    }
                    BezierAlgebraicCuspSemicirclePairEndpoint2::FirstEnd => {
                        (overlap.semicircle(true), false)
                    }
                    BezierAlgebraicCuspSemicirclePairEndpoint2::SecondStart => {
                        (overlap.semicircle(false), true)
                    }
                    BezierAlgebraicCuspSemicirclePairEndpoint2::SecondEnd => {
                        (overlap.semicircle(false), false)
                    }
                };
                Some((semicircle, start, overlap.data.policy))
            }
            Self::PairOverlapMap { source, .. } => {
                let BezierAlgebraicCuspSemicircleParameter2::Mapped(source) = source else {
                    return None;
                };
                source.coincident_pair_endpoint_source()
            }
            Self::Rational { .. } | Self::Parallel { .. } | Self::Pair { .. } => None,
            Self::SelectedFiberRational { .. }
            | Self::SelectedFiberParallel { .. }
            | Self::SelectedParallelContact { .. }
            | Self::SelectedCircularTangentContact { .. }
            | Self::SelectedPairContact { .. }
            | Self::SelectedChordNormalContact { .. }
            | Self::SelectedChordParallelNormalContact { .. }
            | Self::Chord { .. }
            | Self::SimilarityTransport { .. }
            | Self::Chamfer { .. } => None,
        }
    }

    /// Recovers a regular mapped carrier beneath coincident-circle wrappers.
    /// Its tangent line identifies an interior point of one selected
    /// semicircle up to the antipode, which the destination overlap range
    /// excludes exactly.
    pub(in crate::bezier_offset) fn coincident_tangent_source(
        &self,
    ) -> Option<BezierAlgebraicCuspSemicircleMappedTangentSource2<'_>> {
        match self {
            Self::Rational { map, contact } => Some(
                BezierAlgebraicCuspSemicircleMappedTangentSource2::Rational {
                    curve: &map.data.curve,
                    parameter: contact.other_parameter.as_bezier_parameter()?,
                    policy: map.data.policy,
                },
            ),
            Self::Parallel { map, contact } => Some(
                BezierAlgebraicCuspSemicircleMappedTangentSource2::Parallel {
                    parallel: &map.data.parallel,
                    parameter: &contact.parallel_parameter,
                    policy: map.data.policy,
                },
            ),
            Self::SelectedParallelContact {
                parallel,
                parameter,
                policy,
                ..
            } => Some(
                BezierAlgebraicCuspSemicircleMappedTangentSource2::Parallel {
                    parallel,
                    parameter: parameter.as_bezier_parameter()?,
                    policy: *policy,
                },
            ),
            Self::PairOverlapMap { source, .. } => {
                let BezierAlgebraicCuspSemicircleParameter2::Mapped(source) = source else {
                    return None;
                };
                source.coincident_tangent_source()
            }
            Self::Pair { .. } | Self::PairOverlap { .. } => None,
            Self::SelectedFiberRational { .. }
            | Self::SelectedFiberParallel { .. }
            | Self::SelectedCircularTangentContact { .. }
            | Self::SelectedPairContact { .. }
            | Self::SelectedChordNormalContact { .. }
            | Self::SelectedChordParallelNormalContact { .. }
            | Self::Chord { .. }
            | Self::SimilarityTransport { .. }
            | Self::Chamfer { .. } => None,
        }
    }

    /// Returns the exact source and unit rotation retained by a chamfer cut.
    pub(in crate::bezier_offset) fn chamfer_rotation_source<'a>(
        &'a self,
        policy: &CurveContext,
    ) -> CurveResult<
        Option<(
            &'a Arc<BezierAlgebraicCuspSemicircleMappedParameterData2>,
            Similarity2,
        )>,
    > {
        let Self::Chamfer {
            semicircle,
            source,
            point,
            policy: transport_policy,
            ..
        } = self
        else {
            return Ok(None);
        };
        if !policy.accepts_retained_policy(*transport_policy) {
            return Err(CurveError::Topology(
                "mapped chamfer point used a different predicate policy".into(),
            ));
        }
        let BezierAlgebraicCuspSemicircleParameter2::Mapped(source) = source else {
            return Ok(None);
        };
        let CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(point)) = point else {
            return Err(CurveError::Topology(
                "mapped chamfer point lost its exact rotation provenance".into(),
            ));
        };
        let BezierAlgebraicCuspDerivedPointSource2::Mapped {
            parameter: point_source,
            ..
        } = &point.data.source
        else {
            return Err(CurveError::Topology(
                "mapped chamfer rotation lost its source parameter".into(),
            ));
        };
        if !Arc::ptr_eq(point_source, source) {
            return Err(CurveError::Topology(
                "mapped chamfer rotation changed its source parameter".into(),
            ));
        }
        let Some(center) = semicircle.exact_center(policy)? else {
            return Ok(None);
        };
        let radial = &point.data.radial_scale;
        let perpendicular = &point.data.perpendicular_scale;
        let transform = Similarity2::try_from_real_affine(
            radial.clone(),
            -perpendicular,
            perpendicular.clone(),
            radial.clone(),
            (Real::one() - radial) * center.x() + perpendicular * center.y(),
            (Real::one() - radial) * center.y() - perpendicular * center.x(),
        )?;
        Ok(Some((source, transform)))
    }

    /// Evaluates a represented ordinary source point before applying its
    /// chamfer rotation. This avoids rebuilding and reclassifying an entire
    /// analytic parallel when only one exact point is required.
    pub(in crate::bezier_offset) fn chamfer_exact_point(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Point2>>> {
        let Self::Chamfer { source, .. } = self else {
            return Ok(Classification::Decided(None));
        };
        let BezierAlgebraicCuspSemicircleParameter2::Mapped(source) = source else {
            return Ok(Classification::Decided(None));
        };
        let Some(source) = source.mapped_point_source(policy)? else {
            return Ok(Classification::Decided(None));
        };
        let represented_parallel = match &source {
            BezierAlgebraicCuspSemicircleMappedPointSource2::Parallel {
                parameter:
                    BezierAlgebraicCuspSemicircleMappedPointParameter2::Ordinary(
                        BezierParameter2::Exact(_),
                    ),
                ..
            } => true,
            BezierAlgebraicCuspSemicircleMappedPointSource2::Parallel {
                parameter: BezierAlgebraicCuspSemicircleMappedPointParameter2::Selected(parameter),
                ..
            } => parameter.represented_value().is_some(),
            _ => false,
        };
        if !represented_parallel {
            return Ok(Classification::Decided(None));
        }
        let point = source.exact_point(policy)?;
        let Some((_, transform)) = self.chamfer_rotation_source(policy)? else {
            return Ok(Classification::Decided(None));
        };
        Ok(point.map(|point| point.map(|point| transform.transform_point(&point))))
    }

    /// Recovers an owned carrier and its exact parameter for point
    /// correspondence. Selected parameters remain in their local fiber;
    /// similarity and represented-center chamfer transports transform only
    /// the carrier, so the parameter identity remains unchanged.
    pub(in crate::bezier_offset) fn mapped_point_source(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Option<BezierAlgebraicCuspSemicircleMappedPointSource2>> {
        match self {
            Self::Rational { map, .. } => {
                if !policy.accepts_retained_policy(map.data.policy) {
                    return Err(CurveError::Topology(
                        "mapped rational point source used a different predicate policy".into(),
                    ));
                }
                Ok(self
                    .coincident_tangent_source()
                    .map(BezierAlgebraicCuspSemicircleMappedPointSource2::from_borrowed))
            }
            Self::Parallel { map, .. } => {
                if !policy.accepts_retained_policy(map.data.policy) {
                    return Err(CurveError::Topology(
                        "mapped analytic point source used a different predicate policy".into(),
                    ));
                }
                Ok(self
                    .coincident_tangent_source()
                    .map(BezierAlgebraicCuspSemicircleMappedPointSource2::from_borrowed))
            }
            Self::SelectedFiberRational {
                map,
                other_parameter,
                ..
            } => {
                map.validate_policy(policy)?;
                Ok(Some(
                    BezierAlgebraicCuspSemicircleMappedPointSource2::Rational {
                        curve: map.data.curve.clone(),
                        parameter: BezierAlgebraicCuspSemicircleMappedPointParameter2::Selected(
                            other_parameter.clone(),
                        ),
                        policy: map.data.policy,
                    },
                ))
            }
            Self::SelectedFiberParallel {
                map,
                other_parameter,
                ..
            } => {
                map.validate_policy(policy)?;
                Ok(Some(
                    BezierAlgebraicCuspSemicircleMappedPointSource2::Parallel {
                        parallel: map.data.parallel.clone(),
                        parameter: BezierAlgebraicCuspSemicircleMappedPointParameter2::Selected(
                            other_parameter.clone(),
                        ),
                        policy: map.data.policy,
                    },
                ))
            }
            Self::PairOverlapMap { source, .. } => {
                let BezierAlgebraicCuspSemicircleParameter2::Mapped(source) = source else {
                    return Ok(None);
                };
                source.mapped_point_source(policy)
            }
            Self::SimilarityTransport {
                source,
                point,
                policy: transport_policy,
                ..
            } => {
                if !policy.accepts_retained_policy(*transport_policy) {
                    return Err(CurveError::Topology(
                        "mapped point similarity used a different predicate policy".into(),
                    ));
                }
                let CurvePoint2(CurvePointData2::Similarity(point)) = point else {
                    return Err(CurveError::Topology(
                        "mapped point similarity lost its exact transform provenance".into(),
                    ));
                };
                if !policy.accepts_retained_policy(point.data.policy) {
                    return Err(CurveError::Topology(
                        "mapped point similarity evidence used a different predicate policy".into(),
                    ));
                }
                let BezierAlgebraicCuspSemicircleParameter2::Mapped(source) = source else {
                    return Ok(None);
                };
                let Some(source) = source.mapped_point_source(policy)? else {
                    return Ok(None);
                };
                Ok(Some(source.transform_similarity(&point.data.transform)?))
            }
            Self::Chamfer { .. } => {
                if self.retained_one_field_point_image(policy)?.is_some() {
                    return Ok(None);
                }
                let Some((source, transform)) = self.chamfer_rotation_source(policy)? else {
                    return Ok(None);
                };
                let Some(source) = source.mapped_point_source(policy)? else {
                    return Ok(None);
                };
                Ok(Some(source.transform_similarity(&transform)?))
            }
            Self::SelectedParallelContact { .. }
            | Self::SelectedCircularTangentContact { .. }
            | Self::SelectedPairContact { .. }
            | Self::SelectedChordNormalContact { .. }
            | Self::SelectedChordParallelNormalContact { .. }
            | Self::Pair { .. }
            | Self::Chord { .. }
            | Self::PairOverlap { .. } => Ok(None),
        }
    }

    /// Recovers target-carrier parameters from a point retained directly on
    /// this selected circle. The probe is exact retained geometry, and every
    /// carrier contact is replayed against the authored point before its
    /// parameter is admitted.
    pub(in crate::bezier_offset) fn retained_point_parameter_candidates_on_target(
        self: &Arc<Self>,
        target: &BezierAlgebraicCuspSemicircleMappedOverlapMap2,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Vec<CurveParameter2>>>> {
        if let Some(point) = self.retained_one_field_point_image(policy)? {
            match self.one_field_circle_tangent_parameter_candidates_on_target(
                &point, target, range, policy,
            )? {
                Classification::Decided(Some(parameters)) => {
                    return Ok(Classification::Decided(Some(parameters)));
                }
                Classification::Decided(None) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        match target {
            BezierAlgebraicCuspSemicircleMappedOverlapMap2::Rational(target) => self
                .retained_point_parameter_candidates_on_rational_target(
                    &target.data.curve,
                    range,
                    policy,
                ),
            BezierAlgebraicCuspSemicircleMappedOverlapMap2::Parallel(target) => {
                if let Some(point) = self.retained_one_field_point_image(policy)? {
                    return one_field_point_parameter_candidates(
                        &point.into(),
                        &target.data.parallel,
                        range,
                        policy,
                    );
                }
                if let Classification::Decided(Some(curve)) = target
                    .data
                    .parallel
                    .exact_circular_parallel_component(&policy.strict_counterpart())?
                    && let decided @ Classification::Decided(Some(_)) = self
                        .retained_point_parameter_candidates_on_rational_target(
                            &curve, range, policy,
                        )?
                {
                    // Preserve the authored analytic parameter while solving
                    // the retained point through its exact rational circle.
                    // This avoids rebuilding a multi-field chord/parallel
                    // system for geometry already proved to be circular.
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "mapped-circle-retained-point-inverse",
                        "exact-analytic-circle-component",
                    );
                    return Ok(decided);
                }
                let (chord, point) = match self.retained_point_probe_chord(policy)? {
                    Classification::Decided(Some(probe)) => probe,
                    Classification::Decided(None) => {
                        return Ok(Classification::Decided(None));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let contacts = match chord.parallel_intersections_on_regular_range(
                    &target.data.parallel,
                    range,
                    policy,
                )? {
                    Classification::Decided(
                        BezierAlgebraicChordParallelIntersections2::Contacts(contacts),
                    ) => contacts,
                    Classification::Decided(
                        BezierAlgebraicChordParallelIntersections2::CoincidentSupportComponent {
                            ..
                        }
                        | BezierAlgebraicChordParallelIntersections2::DegenerateProjection,
                    ) => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let mut candidates = Vec::with_capacity(contacts.len());
                for contact in contacts {
                    match point.same_point(contact.point(), policy) {
                        Classification::Decided(true) => {
                            candidates.push(contact.parallel_parameter().clone());
                        }
                        Classification::Decided(false) => {}
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                Ok(Classification::Decided(Some(candidates)))
            }
        }
    }

    pub(in crate::bezier_offset) fn retained_point_parameter_candidates_on_rational_target(
        self: &Arc<Self>,
        target: &RationalBezier2,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Vec<CurveParameter2>>>> {
        if let Some(point) = self.retained_one_field_point_image(policy)? {
            return one_field_point_parameter_candidates(
                &point.into(),
                &target.parallel_left(Real::zero())?,
                range,
                policy,
            );
        }
        let (chord, point) = match self.retained_point_probe_chord(policy)? {
            Classification::Decided(Some(probe)) => probe,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let intersections = chord.rational_intersections(target, range, None, policy)?;
        let contacts = match intersections {
            Classification::Decided(BezierAlgebraicChordRationalIntersections2::Contacts(
                contacts,
            )) => contacts,
            Classification::Decided(
                BezierAlgebraicChordRationalIntersections2::Overlaps(_)
                | BezierAlgebraicChordRationalIntersections2::ContactsAndOverlaps { .. }
                | BezierAlgebraicChordRationalIntersections2::DegenerateProjection
                | BezierAlgebraicChordRationalIntersections2::NotSourceRelated,
            ) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let mut candidates = Vec::with_capacity(contacts.len());
        for contact in contacts {
            let same_point = point.same_point(contact.point(), policy);
            match same_point {
                Classification::Decided(true) => {
                    candidates.push(contact.other_parameter().clone());
                }
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        Ok(Classification::Decided(Some(candidates)))
    }

    pub(in crate::bezier_offset) fn retained_one_field_point_image(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Option<RationalBezierAlgebraicPointImage2>> {
        Ok(match self.retained_point_evidence() {
            Some(CurvePoint2(CurvePointData2::Algebraic(point))) => Some(point.clone()),
            Some(CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(point))) => {
                point.exact_one_field_point_image(policy)?
            }
            Some(
                CurvePoint2(CurvePointData2::Exact(_))
                | CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
                | CurvePoint2(CurvePointData2::AlgebraicCuspChord(_))
                | CurvePoint2(CurvePointData2::AlgebraicChordParallel(_))
                | CurvePoint2(CurvePointData2::AnalyticParallel(_))
                | CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)),
            )
            | None => None,
        })
    }

    pub(in crate::bezier_offset) fn one_field_circle_tangent_parameter_candidates_on_target(
        &self,
        point: &RationalBezierAlgebraicPointImage2,
        target: &BezierAlgebraicCuspSemicircleMappedOverlapMap2,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Vec<CurveParameter2>>>> {
        let (parameter, tangent) = match self.one_field_circle_tangent_source(point, policy)? {
            Classification::Decided(Some(source)) => source,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        self.circle_tangent_parameter_candidates_on_target(
            &parameter, &tangent, target, range, policy,
        )
    }

    pub(in crate::bezier_offset) fn exact_point_circle_tangent_parameter_candidates_on_target(
        &self,
        point: &Point2,
        target: &BezierAlgebraicCuspSemicircleMappedOverlapMap2,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Vec<CurveParameter2>>>> {
        let Some(center) = self.semicircle_carrier().exact_center(policy)? else {
            return Ok(Classification::Decided(None));
        };
        let radial_x = point.x() - center.x();
        let radial_y = point.y() - center.y();
        match real_sign(
            &(&radial_x * &radial_x + &radial_y * &radial_y),
            &CurveContext::STRICT,
        ) {
            Some(RealSign::Positive) => {}
            Some(RealSign::Zero) => {
                return Err(CurveError::Topology(
                    "a retained circle point coincided with its nonzero-radius center".into(),
                ));
            }
            Some(RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "a retained circle radius squared was negative".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        self.circle_tangent_parameter_candidates_on_target(
            &BezierParameter2::Exact(Real::zero()),
            &[vec![-radial_y], vec![radial_x]],
            target,
            range,
            policy,
        )
    }

    pub(in crate::bezier_offset) fn circle_tangent_parameter_candidates_on_target(
        &self,
        parameter: &BezierParameter2,
        tangent: &[Vec<Real>; 2],
        target: &BezierAlgebraicCuspSemicircleMappedOverlapMap2,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Vec<CurveParameter2>>>> {
        let target_tangent = match target {
            BezierAlgebraicCuspSemicircleMappedOverlapMap2::Rational(target) => {
                rational_parametric_tangent_numerator(target.data.curve.homogeneous_power_basis()?)
            }
            BezierAlgebraicCuspSemicircleMappedOverlapMap2::Parallel(target) => {
                let differential = target.data.parallel.differential()?;
                [
                    differential.tangent_x.clone(),
                    differential.tangent_y.clone(),
                ]
            }
        };
        Ok(mapped_circle_tangent_parameter_candidates(
            BezierAlgebraicCuspSemicircleMappedPointParameterRef2::Ordinary(parameter),
            tangent,
            &target_tangent,
            range,
            policy,
        )?
        .map(Some))
    }

    /// Recovers the supporting-circle tangent at a point whose coordinates
    /// and exact center already inhabit one selected algebraic field. A circle
    /// overlap needs only this line: the antipodal solution lies outside an
    /// interior half-circle overlap range.
    pub(in crate::bezier_offset) fn one_field_circle_tangent_source(
        &self,
        point: &RationalBezierAlgebraicPointImage2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<(BezierParameter2, [Vec<Real>; 2])>>> {
        let parameter = match algebraic_chord_image_parameter(point, policy)? {
            Classification::Decided(parameter) => BezierParameter2::Algebraic(parameter),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let [x, y, denominator] = match algebraic_chord_owned_coordinate_polynomials(point, policy)?
        {
            Classification::Decided(coordinates) => coordinates,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let Some(center) = self.semicircle_carrier().exact_center(policy)? else {
            return Ok(Classification::Decided(None));
        };
        let radial_x = polynomial_subtract(&x, &polynomial_scale(&denominator, center.x()));
        let radial_y = polynomial_subtract(&y, &polynomial_scale(&denominator, center.y()));
        Ok(Classification::Decided(Some((
            parameter,
            [polynomial_scale(&radial_y, &Real::from(-1_i8)), radial_x],
        ))))
    }

    pub(in crate::bezier_offset) fn retained_point_probe_chord(
        self: &Arc<Self>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<(BezierAlgebraicChord2, CurvePoint2)>>> {
        let point = if let Some(point) = self.retained_or_selected_point_evidence() {
            point
        } else if let Some(data) = self.coincident_chord_parameter() {
            CurvePoint2::from(BezierAlgebraicCuspChordPoint2 { data })
        } else {
            return Ok(Classification::Decided(None));
        };
        if let CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(derived)) = &point {
            if let Some(image) = derived.exact_one_field_point_image(policy)? {
                let point = CurvePoint2::from(image);
                let start = match BezierAlgebraicChord2::translated_endpoint(
                    &point,
                    &Real::zero(),
                    &Real::from(-1_i8),
                    policy,
                )? {
                    Classification::Decided(point) => point,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let end = match BezierAlgebraicChord2::translated_endpoint(
                    &point,
                    &Real::zero(),
                    &Real::one(),
                    policy,
                )? {
                    Classification::Decided(point) => point,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                return Ok(Classification::Decided(Some((
                    BezierAlgebraicChord2::from_certified_axis_aligned_endpoints(
                        start,
                        end,
                        BezierAlgebraicChordAxisDirection2::PositiveY,
                        policy,
                    ),
                    point,
                ))));
            }
            let start = CurvePoint2::from(derived.translated(&Real::zero(), &Real::from(-1_i8)));
            let end = CurvePoint2::from(derived.translated(&Real::zero(), &Real::one()));
            return Ok(Classification::Decided(Some((
                BezierAlgebraicChord2::from_certified_axis_aligned_endpoints(
                    start,
                    end,
                    BezierAlgebraicChordAxisDirection2::PositiveY,
                    policy,
                ),
                point,
            ))));
        }
        // A coincident-circle transport preserves its source circle's center.
        // Reuse that exact authored center when the retained point descends
        // from a chord contact, so the probe and the contact remain in one
        // recursive frame instead of reconstructing an equivalent center in
        // the destination overlap carrier.
        let center_carrier = match &point {
            CurvePoint2(CurvePointData2::AlgebraicCuspChord(point)) => {
                &point.map_contact().0.data.semicircle
            }
            _ => self.semicircle_carrier(),
        };
        let center = match center_carrier.center_point_evidence(policy)? {
            Classification::Decided(center) => center,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        // Every mapped circle point is separated from its center by the
        // carrier's certified nonzero radius. Preserve that construction fact
        // instead of replaying a cross-field Cartesian equality.
        let retained_point = point.clone();
        Ok(
            BezierAlgebraicChord2::try_new_from_certified_distinct_endpoints(
                center, point, policy,
            )?
            .map(|chord| Some((chord, retained_point))),
        )
    }

    /// Signs one retained linear combination of the circle tangent crossed
    /// and dotted with its ordinary mapped carrier tangent.
    pub(in crate::bezier_offset) fn ordinary_carrier_tangent_cross_dot_linear_combination_sign(
        &self,
        cross_scale: &Real,
        dot_scale: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Option<Classification<RealSign>>> {
        policy.strict_predicate_pass(|| {
            Ok(match self {
                Self::Rational { map, contact } => {
                    Some(map.tangent_cross_dot_linear_combination_sign(
                        contact,
                        cross_scale,
                        dot_scale,
                        policy,
                    )?)
                }
                Self::Parallel { map, contact } => Some(
                    if dot_scale.zero_status() == ZeroKnowledge::Zero
                        && let Some(cross_scale_sign) =
                            real_sign(cross_scale, &CurveContext::STRICT)
                        && let Some(sign) = contact.tangent_cross_sign
                    {
                        Classification::Decided(product_sign(sign, cross_scale_sign))
                    } else {
                        map.data
                            .semicircle
                            .parallel_contact_tangent_cross_dot_source_sign(
                                &map.data.parallel,
                                contact,
                                cross_scale,
                                dot_scale,
                                policy,
                            )?
                    },
                ),
                Self::SelectedFiberRational {
                    map,
                    other_parameter,
                    tangent_cross_sign,
                    ..
                } => {
                    map.validate_policy(policy)?;
                    if dot_scale.zero_status() == ZeroKnowledge::Zero
                        && let Some(scale) = real_sign(cross_scale, &CurveContext::STRICT)
                    {
                        Some(Classification::Decided(product_sign(
                            *tangent_cross_sign,
                            scale,
                        )))
                    } else {
                        Some(map.tangent_cross_dot_linear_combination_sign(
                            other_parameter,
                            cross_scale,
                            dot_scale,
                            policy,
                        )?)
                    }
                }
                Self::SelectedFiberParallel {
                    map,
                    other_parameter,
                    tangent_cross_sign,
                    ..
                } => {
                    map.validate_policy(policy)?;
                    if dot_scale.zero_status() == ZeroKnowledge::Zero
                        && let Some(scale) = real_sign(cross_scale, &CurveContext::STRICT)
                    {
                        Some(Classification::Decided(product_sign(
                            *tangent_cross_sign,
                            scale,
                        )))
                    } else {
                        Some(map.tangent_cross_dot_linear_combination_sign(
                            other_parameter,
                            cross_scale,
                            dot_scale,
                            policy,
                        )?)
                    }
                }
                Self::PairOverlapMap { source, .. }
                | Self::SimilarityTransport { source, .. }
                | Self::Chamfer { source, .. } => {
                    let BezierAlgebraicCuspSemicircleParameter2::Mapped(source) = source else {
                        return Ok(None);
                    };
                    return source.ordinary_carrier_tangent_cross_dot_linear_combination_sign(
                        cross_scale,
                        dot_scale,
                        policy,
                    );
                }
                Self::SelectedParallelContact { .. }
                | Self::SelectedCircularTangentContact { .. }
                | Self::SelectedPairContact { .. }
                | Self::SelectedChordNormalContact { .. }
                | Self::SelectedChordParallelNormalContact { .. }
                | Self::Pair { .. }
                | Self::Chord { .. }
                | Self::PairOverlap { .. } => None,
            })
        })
    }

    /// Returns the exact circle-tangent/source-tangent cross sign for an
    /// ordinary mapped carrier when that relation is retained by this map.
    pub(in crate::bezier_offset) fn ordinary_carrier_tangent_cross_sign(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Option<Classification<RealSign>>> {
        self.ordinary_carrier_tangent_cross_dot_linear_combination_sign(
            &Real::one(),
            &Real::zero(),
            policy,
        )
    }

    /// Recovers a regular tangent source through exact local transports.
    ///
    /// Similarities use their linear part. Chamfer cuts use the same retained
    /// center-relative rotation that constructs their point. Coincident and
    /// concentric transports can change radius, but their tangent lines remain
    /// parallel. The mapped-overlap inverse needs only that line; its regular
    /// cell rejects the antipode and proves the unique target parameter.
    pub(in crate::bezier_offset) fn coincident_tangent_power_source(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<
        Option<(
            BezierAlgebraicCuspSemicircleMappedPointParameter2,
            [Vec<Real>; 2],
            CurveContext,
        )>,
    > {
        if let Some(source) = self.coincident_tangent_source() {
            if let Some(sign) = self.ordinary_carrier_tangent_cross_sign(policy)? {
                match sign {
                    Classification::Decided(RealSign::Zero) => {}
                    Classification::Decided(RealSign::Negative | RealSign::Positive)
                    | Classification::Uncertain(_) => return Ok(None),
                }
            }
            return Ok(Some((
                BezierAlgebraicCuspSemicircleMappedPointParameter2::Ordinary(
                    source.parameter().clone(),
                ),
                source.tangent_power_basis()?,
                source.policy(),
            )));
        }
        match self {
            Self::SelectedFiberRational {
                tangent_cross_sign, ..
            }
            | Self::SelectedFiberParallel {
                tangent_cross_sign, ..
            } => {
                if *tangent_cross_sign != RealSign::Zero {
                    return Ok(None);
                }
                let Some(source) = self.mapped_point_source(policy)? else {
                    return Ok(None);
                };
                source.tangent_power_source(policy)
            }
            Self::PairOverlapMap { source, .. } => {
                let BezierAlgebraicCuspSemicircleParameter2::Mapped(source) = source else {
                    return Ok(None);
                };
                source.coincident_tangent_power_source(policy)
            }
            Self::SimilarityTransport {
                source,
                point,
                policy: transport_policy,
                ..
            } => {
                if !policy.accepts_retained_policy(*transport_policy) {
                    return Err(CurveError::Topology(
                        "mapped tangent similarity used a different predicate policy".into(),
                    ));
                }
                let CurvePoint2(CurvePointData2::Similarity(point)) = point else {
                    return Err(CurveError::Topology(
                        "mapped tangent similarity lost its exact transform provenance".into(),
                    ));
                };
                if !policy.accepts_retained_policy(point.data.policy) {
                    return Err(CurveError::Topology(
                        "mapped tangent similarity point used a different predicate policy".into(),
                    ));
                }
                let BezierAlgebraicCuspSemicircleParameter2::Mapped(source) = source else {
                    return Ok(None);
                };
                let Some((parameter, tangent, source_policy)) =
                    source.coincident_tangent_power_source(policy)?
                else {
                    return Ok(None);
                };
                if !policy.accepts_retained_policy(source_policy) {
                    return Err(CurveError::Topology(
                        "mapped tangent similarity source used a different predicate policy".into(),
                    ));
                }
                let (m00, m01, m10, m11, _, _) = point.data.transform.affine_components();
                let transformed = [
                    polynomial_add(
                        &polynomial_scale(&tangent[0], m00),
                        &polynomial_scale(&tangent[1], m01),
                    ),
                    polynomial_add(
                        &polynomial_scale(&tangent[0], m10),
                        &polynomial_scale(&tangent[1], m11),
                    ),
                ];
                Ok(Some((parameter, transformed, *transport_policy)))
            }
            Self::Chamfer {
                source,
                point,
                policy: transport_policy,
                ..
            } => {
                if !policy.accepts_retained_policy(*transport_policy) {
                    return Err(CurveError::Topology(
                        "mapped chamfer tangent used a different predicate policy".into(),
                    ));
                }
                let BezierAlgebraicCuspSemicircleParameter2::Mapped(source) = source else {
                    return Ok(None);
                };
                let Some((parameter, tangent, source_policy)) =
                    source.coincident_tangent_power_source(policy)?
                else {
                    return Ok(None);
                };
                if !policy.accepts_retained_policy(source_policy) {
                    return Err(CurveError::Topology(
                        "mapped chamfer tangent source used a different predicate policy".into(),
                    ));
                }
                let CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(point)) = point else {
                    return Err(CurveError::Topology(
                        "mapped chamfer tangent lost its exact rotation provenance".into(),
                    ));
                };
                let BezierAlgebraicCuspDerivedPointSource2::Mapped {
                    parameter: point_source,
                    ..
                } = &point.data.source
                else {
                    return Err(CurveError::Topology(
                        "mapped chamfer rotation lost its source parameter".into(),
                    ));
                };
                if !Arc::ptr_eq(point_source, source) {
                    return Err(CurveError::Topology(
                        "mapped chamfer rotation changed its source parameter".into(),
                    ));
                }
                let radial = &point.data.radial_scale;
                let perpendicular = &point.data.perpendicular_scale;
                let transformed = [
                    polynomial_subtract(
                        &polynomial_scale(&tangent[0], radial),
                        &polynomial_scale(&tangent[1], perpendicular),
                    ),
                    polynomial_add(
                        &polynomial_scale(&tangent[0], perpendicular),
                        &polynomial_scale(&tangent[1], radial),
                    ),
                ];
                Ok(Some((parameter, transformed, *transport_policy)))
            }
            Self::Rational { .. }
            | Self::Parallel { .. }
            | Self::SelectedParallelContact { .. }
            | Self::SelectedCircularTangentContact { .. }
            | Self::SelectedPairContact { .. }
            | Self::SelectedChordNormalContact { .. }
            | Self::SelectedChordParallelNormalContact { .. }
            | Self::Pair { .. }
            | Self::Chord { .. }
            | Self::PairOverlap { .. } => Ok(None),
        }
    }

    /// Encloses a rational point whose parameter is retained directly in one
    /// selected algebraic fiber. The curve parameter and its complete reduced
    /// incidence are refined together; Cartesian evaluation then uses exact
    /// interval arithmetic and never materializes a global norm polynomial.
    pub(in crate::bezier_offset) fn selected_fiber_point_bounds_refined(
        &self,
        refinement_steps: usize,
        policy: &CurveContext,
    ) -> Option<Classification<Aabb2>> {
        match self {
            Self::SelectedFiberRational {
                map,
                other_parameter,
                ..
            } => Some(map.point_bounds_refined(other_parameter, refinement_steps, policy)),
            Self::SelectedFiberParallel {
                map,
                other_parameter,
                ..
            } => Some(map.point_bounds_refined(other_parameter, refinement_steps, policy)),
            _ => None,
        }
    }

    /// Encloses a point authored by the ordinary rational-contact map.
    ///
    /// The target parameter is already an exact retained root. Evaluating the
    /// target curve over its refined isolating interval gives a convergent
    /// Cartesian enclosure without representing the selected circle center or
    /// adjoining the contact root to that center's field.
    pub(in crate::bezier_offset) fn coincident_rational_point_bounds_refined(
        &self,
        refinement_steps: usize,
        policy: &CurveContext,
    ) -> Option<Classification<Aabb2>> {
        let BezierAlgebraicCuspSemicircleMappedTangentSource2::Rational {
            curve,
            parameter,
            policy: source_policy,
        } = self.coincident_tangent_source()?
        else {
            return None;
        };
        if !policy.accepts_retained_policy(source_policy) {
            return Some(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        Some(rational_bezier_point_bounds_refined(
            curve,
            parameter,
            refinement_steps,
            policy,
        ))
    }

    /// Encloses the point authored by an analytic-parallel map without
    /// adjoining its speed square root to the selected cusp field.
    ///
    /// The retained native parameter independently encloses every polynomial
    /// input to `P(t) + d*(-H_y,H_x)/sqrt(H dot H)`. Exact interval arithmetic
    /// therefore yields a conservative point box even when the parameter and
    /// selected circle center inhabit different fields. Refinement can prove
    /// every strict downstream order; equality remains policy-terminal.
    pub(in crate::bezier_offset) fn coincident_parallel_point_bounds_refined(
        &self,
        refinement_steps: usize,
        policy: &CurveContext,
    ) -> Option<Classification<Aabb2>> {
        let BezierAlgebraicCuspSemicircleMappedTangentSource2::Parallel {
            parallel,
            parameter,
            policy: source_policy,
        } = self.coincident_tangent_source()?
        else {
            return None;
        };
        if !policy.accepts_retained_policy(source_policy) {
            return Some(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        Some(analytic_parallel_point_bounds_refined(
            parallel,
            parameter,
            &Real::zero(),
            &Real::zero(),
            &Real::zero(),
            refinement_steps,
            policy,
        ))
    }

    /// Encloses a retained selected-circle/chord contact in its existing
    /// correlated map. Coincident-circle wrappers preserve that map allocation,
    /// so the derived-point broad phase can refine the original two-field
    /// point without first materializing Cartesian algebraic coordinates.
    pub(in crate::bezier_offset) fn coincident_chord_point_bounds_refined(
        self: &Arc<Self>,
        refinement_steps: usize,
        policy: &CurveContext,
    ) -> Option<Classification<Aabb2>> {
        let data = self.coincident_chord_parameter()?;
        Some(
            BezierAlgebraicCuspChordPoint2 { data }
                .conservative_bounds_refined(refinement_steps, policy),
        )
    }

    /// Encloses a retained circle-circle contact from the exact represented
    /// coordinates already owned by its pair map. Reconstructing an angular
    /// bracket from that same map would duplicate the contact solve and can
    /// turn broad-phase refinement into an unrelated parameter-equality
    /// proof.
    pub(in crate::bezier_offset) fn coincident_pair_point_bounds_refined(
        &self,
        refinement_steps: usize,
        policy: &CurveContext,
    ) -> Option<Classification<Aabb2>> {
        let (map, contact, _, _) = self.coincident_pair_source()?;
        if !policy.accepts_retained_policy(map.data.policy) {
            return Some(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        if let Some(recursive) = map.recursive_contact_data(contact) {
            return Some(recursive.frame.point.bounds_refined(refinement_steps));
        }
        let (_, represented) = map.represented_contact_data(contact)?;
        Some(represented_point_bounds_refined(
            &represented.point[0],
            &represented.point[1],
            refinement_steps,
        ))
    }

    /// Restricts the general complement predicate to analytic-parallel maps.
    /// That branch proves every equality under STRICT even when the retained
    /// construction policy is APPROXIMATE_512.
    pub(in crate::bezier_offset) fn parallel_complementary_to(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Option<Classification<bool>>> {
        if !matches!(
            (self, other),
            (Self::Parallel { .. }, Self::Parallel { .. })
        ) {
            return Ok(None);
        }
        Ok(Some(self.is_complementary_to(other, policy)?))
    }

    /// Proves that two mapped cuts are complementary parameters on the same
    /// selected semicircle without adjoining their carrier fields.
    pub(in crate::bezier_offset) fn is_complementary_to(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        if self.semicircle_carrier() != other.semicircle_carrier() {
            return Ok(Classification::Decided(false));
        }
        if let (
            Self::PairOverlapMap {
                overlap: first_overlap,
                source: first_source,
                source_first: first_side,
            },
            Self::PairOverlapMap {
                overlap: second_overlap,
                source: second_source,
                source_first: second_side,
            },
        ) = (self, other)
            && first_overlap.shares_parameter_map(*first_side, second_overlap, *second_side)
            && first_overlap.has_exact_endpoint_map()
        {
            // A full coincident semicircle maps its local parameter by either
            // identity or unit complement.  Both affine involutions preserve
            // the relation u+v=1, so replay the distinct source carriers'
            // exact complement proof rather than comparing two nested fields.
            let (
                BezierAlgebraicCuspSemicircleParameter2::Mapped(first_source),
                BezierAlgebraicCuspSemicircleParameter2::Mapped(second_source),
            ) = (first_source, second_source)
            else {
                return Ok(Classification::Decided(false));
            };
            return first_source.is_complementary_to(second_source, policy);
        }
        // Rational materialization preserves geometry, including source
        // reversal. All three map combinations therefore share the same
        // parameter-direction proof before replaying their diameter relation.
        let (first, second) = if matches!(
            (self, other),
            (Self::Parallel { .. }, Self::Rational { .. })
        ) {
            (other, self)
        } else {
            (self, other)
        };
        let parameter = |map: &Self| match map {
            Self::Rational { contact, .. } => Some(contact.other_parameter.clone()),
            Self::Parallel { contact, .. } => {
                Some(CurveParameter2::from(contact.parallel_parameter.clone()))
            }
            _ => None,
        };
        let (Some(first_parameter), Some(second_parameter)) = (parameter(first), parameter(second))
        else {
            return Ok(Classification::Decided(false));
        };
        policy.strict_predicate_pass(|| {
            let mut uncertainty = None;
            let mut shared_parameter = None;
            for orientation in [
                CurveOverlapOrientation2::Same,
                CurveOverlapOrientation2::Reversed,
            ] {
                let second_parameter = if orientation == CurveOverlapOrientation2::Same {
                    second_parameter.clone()
                } else {
                    second_parameter
                        .unit_complement()
                        .ok_or(CurveError::InvalidCurveParameter)?
                };
                match first_parameter.same_value(&second_parameter, policy)? {
                    Classification::Decided(true) => {}
                    Classification::Decided(false) => continue,
                    Classification::Uncertain(reason) => {
                        uncertainty.get_or_insert(reason);
                        continue;
                    }
                }
                if shared_parameter.is_none() {
                    shared_parameter =
                        match promote_curve_region_bezier_parameter(&first_parameter, policy)? {
                            Classification::Decided(parameter) => Some(parameter),
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        };
                }
                let parameter = shared_parameter
                    .as_ref()
                    .expect("the shared cut was promoted once");
                let complementary = match (first, second) {
                    (Self::Rational { map: first, .. }, Self::Rational { map: second, .. }) => {
                        rational_parameters_are_complementary_at_cut(
                            first,
                            second,
                            parameter,
                            orientation,
                            policy,
                        )
                    }
                    (Self::Parallel { map: first, .. }, Self::Parallel { map: second, .. }) => {
                        parallel_parameters_are_complementary_at_cut(
                            first,
                            second,
                            parameter,
                            orientation,
                            policy,
                        )
                    }
                    (
                        Self::Rational { map: rational, .. },
                        Self::Parallel { map: parallel, .. },
                    ) => rational_parallel_diameter_relation_at_cut(
                        rational,
                        parallel,
                        parameter,
                        orientation,
                        true,
                        policy,
                    ),
                    _ => unreachable!("map kinds were validated and rational/parallel was ordered"),
                }?;
                match complementary {
                    Classification::Decided(true) => return Ok(Classification::Decided(true)),
                    Classification::Decided(false) => {}
                    Classification::Uncertain(reason) => {
                        uncertainty.get_or_insert(reason);
                    }
                }
            }
            Ok(uncertainty.map_or(Classification::Decided(false), Classification::Uncertain))
        })
    }
}

impl BezierAlgebraicCuspSemicircleParameter2 {
    /// Returns the selected-circle carrier retained by a mapped parameter.
    /// Exact endpoint parameters intentionally have no implicit carrier.
    pub(crate) fn mapped_semicircle_carrier(&self) -> Option<&BezierAlgebraicCuspSemicircle2> {
        let Self::Mapped(data) = self else {
            return None;
        };
        Some(data.semicircle_carrier())
    }

    pub(crate) fn retains_pair_contact(&self) -> bool {
        matches!(
            self,
            Self::Mapped(data)
                if matches!(
                    data.as_ref(),
                    BezierAlgebraicCuspSemicircleMappedParameterData2::Pair { .. }
                )
        )
    }

    pub(in crate::bezier_offset) fn evidence_policy(&self) -> Option<CurveContext> {
        let Self::Mapped(data) = self else {
            return None;
        };
        Some(match data.as_ref() {
            BezierAlgebraicCuspSemicircleMappedParameterData2::Rational { map, .. } => {
                map.data.policy
            }
            BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedFiberRational {
                map,
                ..
            } => map.data.policy,
            BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedFiberParallel {
                map,
                ..
            } => map.data.policy,
            BezierAlgebraicCuspSemicircleMappedParameterData2::Parallel { map, .. } => {
                map.data.policy
            }
            BezierAlgebraicCuspSemicircleMappedParameterData2::Pair { map, .. } => map.data.policy,
            BezierAlgebraicCuspSemicircleMappedParameterData2::Chord { map, .. } => map.data.policy,
            BezierAlgebraicCuspSemicircleMappedParameterData2::PairOverlap { overlap, .. } => {
                overlap.data.policy
            }
            BezierAlgebraicCuspSemicircleMappedParameterData2::PairOverlapMap {
                overlap, source, ..
            } => {
                // Both the correspondence and its source point contribute
                // evidence. Validation below still checks each dependency,
                // including incompatible preview contexts.
                source.evidence_policy()
                    .filter(|source| source.accepts_retained_policy(overlap.data.policy))
                    .unwrap_or(overlap.data.policy)
            }
            BezierAlgebraicCuspSemicircleMappedParameterData2::SimilarityTransport {
                policy,
                ..
            } => *policy,
            BezierAlgebraicCuspSemicircleMappedParameterData2::Chamfer { policy, .. } => *policy,
            BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedParallelContact {
                policy,
                ..
            }
            | BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedCircularTangentContact {
                policy,
                ..
            }
            | BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedPairContact {
                policy,
                ..
            }
            | BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedChordNormalContact {
                policy,
                ..
            }
            | BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedChordParallelNormalContact {
                policy,
                ..
            } => *policy,
        })
    }

    /// Peels one circle-pair parameter that is exactly an earlier parameter
    /// on the same local half-circle chart. This is a scalar identity, not a
    /// geometric approximation; the pair kernel retained it while proving an
    /// authored tangency.
    pub(in crate::bezier_offset) fn retained_pair_parameter(&self) -> Option<(&Self, bool)> {
        let Self::Mapped(data) = self else {
            return None;
        };
        let BezierAlgebraicCuspSemicircleMappedParameterData2::Pair {
            map,
            contact,
            first,
        } = data.as_ref()
        else {
            return None;
        };
        map.retained_contact_parameter_for_side(contact, *first)
    }

    /// Recovers the exact center-relative radial stored by the translated
    /// circle-pair lane, transporting it through any enclosing similarity.
    /// Translation cancels completely; the linear image is all that angular
    /// parameter comparison needs.
    pub(in crate::bezier_offset) fn translated_pair_contact_radial(
        &self,
    ) -> Option<(
        BezierAlgebraicCuspSemicircle2,
        [Real; 2],
        BezierAlgebraicCuspSemicircleContactLocation2,
    )> {
        let Self::Mapped(data) = self else {
            return None;
        };
        match data.as_ref() {
            BezierAlgebraicCuspSemicircleMappedParameterData2::Pair {
                map,
                contact,
                first,
            } => {
                let (_, represented) = map.represented_contact_data(contact)?;
                let (semicircle, parameter, location) = if *first {
                    (
                        &map.data.first_semicircle,
                        &represented.first_parameter,
                        contact.first_location,
                    )
                } else {
                    (
                        &map.data.second_semicircle,
                        &represented.second_parameter,
                        contact.second_location,
                    )
                };
                let BezierRepresentedCircleContactParameterData2::ExactContactRadial(radial) =
                    parameter
                else {
                    return None;
                };
                Some((semicircle.clone(), radial.clone(), location))
            }
            BezierAlgebraicCuspSemicircleMappedParameterData2::SimilarityTransport {
                semicircle,
                source,
                point,
                ..
            } => {
                let (_, radial, location) = source.translated_pair_contact_radial()?;
                let CurvePoint2(CurvePointData2::Similarity(point)) = point else {
                    return None;
                };
                let radial = point
                    .data
                    .transform
                    .transform_vector_coordinates(&radial[0], &radial[1]);
                Some((semicircle.clone(), [radial.0, radial.1], location))
            }
            _ => None,
        }
    }

    /// Orders two translated pair contacts on concentric copies of one exact
    /// selected-circle frame. Their shared center and start radial cancel;
    /// only the two retained exact radial vectors and traversal turn remain.
    pub(in crate::bezier_offset) fn translated_pair_contact_order(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> Option<Classification<std::cmp::Ordering>> {
        let (first_circle, first_radial, first_location) = self.translated_pair_contact_radial()?;
        let (second_circle, second_radial, second_location) =
            other.translated_pair_contact_radial()?;
        if !first_circle
            .data
            .frame
            .shares_storage(&second_circle.data.frame)
            || first_circle.is_clockwise() != second_circle.is_clockwise()
        {
            return None;
        }
        let radius_product_sign = match (
            real_sign(first_circle.radial_distance(), policy),
            real_sign(second_circle.radial_distance(), policy),
        ) {
            (
                Some(first @ (RealSign::Positive | RealSign::Negative)),
                Some(second @ (RealSign::Positive | RealSign::Negative)),
            ) => product_sign(first, second),
            _ => {
                return Some(Classification::Uncertain(UncertaintyReason::RealSign));
            }
        };
        let radial_cross =
            &first_radial[0] * &second_radial[1] - &first_radial[1] * &second_radial[0];
        let cross = match real_sign(&radial_cross, policy) {
            Some(sign) => product_sign(sign, radius_product_sign),
            None => return Some(Classification::Uncertain(UncertaintyReason::Predicate)),
        };
        let turn = if first_circle.is_clockwise() {
            RealSign::Negative
        } else {
            RealSign::Positive
        };
        Some(Classification::Decided(match product_sign(cross, turn) {
            RealSign::Positive => std::cmp::Ordering::Less,
            RealSign::Negative => std::cmp::Ordering::Greater,
            RealSign::Zero => {
                let radial_dot =
                    &first_radial[0] * &second_radial[0] + &first_radial[1] * &second_radial[1];
                match real_sign(&radial_dot, policy)
                    .map(|sign| product_sign(sign, radius_product_sign))
                {
                    Some(RealSign::Positive) => std::cmp::Ordering::Equal,
                    Some(RealSign::Negative) => match (first_location, second_location) {
                        (
                            BezierAlgebraicCuspSemicircleContactLocation2::Start,
                            BezierAlgebraicCuspSemicircleContactLocation2::End,
                        ) => std::cmp::Ordering::Less,
                        (
                            BezierAlgebraicCuspSemicircleContactLocation2::End,
                            BezierAlgebraicCuspSemicircleContactLocation2::Start,
                        ) => std::cmp::Ordering::Greater,
                        _ => {
                            return Some(Classification::Uncertain(UncertaintyReason::Predicate));
                        }
                    },
                    Some(RealSign::Zero) | None => {
                        return Some(Classification::Uncertain(UncertaintyReason::Predicate));
                    }
                }
            }
        }))
    }

    pub(in crate::bezier_offset) fn translated_pair_or_exact_order(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> Option<Classification<std::cmp::Ordering>> {
        if let Some(order) = self.translated_pair_contact_order(other, policy) {
            return Some(order);
        }
        let endpoint_order = |location, parameter: &Real| {
            if parameter == &Real::zero() {
                Some(match location {
                    BezierAlgebraicCuspSemicircleContactLocation2::Start => {
                        std::cmp::Ordering::Equal
                    }
                    BezierAlgebraicCuspSemicircleContactLocation2::Interior
                    | BezierAlgebraicCuspSemicircleContactLocation2::End => {
                        std::cmp::Ordering::Greater
                    }
                })
            } else if parameter == &Real::one() {
                Some(match location {
                    BezierAlgebraicCuspSemicircleContactLocation2::Start
                    | BezierAlgebraicCuspSemicircleContactLocation2::Interior => {
                        std::cmp::Ordering::Less
                    }
                    BezierAlgebraicCuspSemicircleContactLocation2::End => std::cmp::Ordering::Equal,
                })
            } else {
                None
            }
        };
        if let Some((_, _, location)) = self.translated_pair_contact_radial()
            && let Self::Exact(parameter) = other
        {
            return endpoint_order(location, parameter).map(Classification::Decided);
        }
        if let Some((_, _, location)) = other.translated_pair_contact_radial()
            && let Self::Exact(parameter) = self
        {
            return endpoint_order(location, parameter)
                .map(std::cmp::Ordering::reverse)
                .map(Classification::Decided);
        }
        None
    }

    pub(in crate::bezier_offset) fn validate_policy(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<()> {
        if let Some(evidence_policy) = self.evidence_policy()
            && !policy.accepts_retained_policy(evidence_policy)
        {
            return Err(CurveError::Topology(
                "algebraic cusp cut was replayed under a different predicate policy".into(),
            ));
        }
        if let Self::Mapped(data) = self
            && let BezierAlgebraicCuspSemicircleMappedParameterData2::PairOverlapMap {
                source, ..
            } = data.as_ref()
        {
            source.validate_policy(policy)?;
        }
        if let Self::Mapped(data) = self
            && let BezierAlgebraicCuspSemicircleMappedParameterData2::Chamfer { source, .. } =
                data.as_ref()
        {
            source.validate_policy(policy)?;
        }
        if let Self::Mapped(data) = self
            && let BezierAlgebraicCuspSemicircleMappedParameterData2::SimilarityTransport {
                source,
                ..
            } = data.as_ref()
        {
            source.validate_policy(policy)?;
        }
        Ok(())
    }

    /// Isolates this local half-circle parameter for explicit finite output.
    ///
    /// The exact mapped parameter remains unchanged.  Bisection uses its
    /// authoritative order predicate and returns a rational enclosure plus a
    /// display-only representative; none of these values may feed topology.
    pub(crate) fn finite_projection_interval(
        &self,
        refinement_steps: usize,
        policy: &CurveContext,
    ) -> CurveResult<Classification<(Real, Real, Real)>> {
        self.validate_policy(policy)?;
        if let Self::Exact(parameter) = self {
            return Ok(Classification::Decided((
                parameter.clone(),
                parameter.clone(),
                parameter.clone(),
            )));
        }
        let mut lower = Real::zero();
        let mut upper = Real::one();
        for _ in 0..refinement_steps {
            let midpoint = ((&lower + &upper) / Real::from(2_u8))?;
            match self.order_to_real(&midpoint, policy)? {
                Classification::Decided(std::cmp::Ordering::Less) => upper = midpoint,
                Classification::Decided(std::cmp::Ordering::Greater) => lower = midpoint,
                Classification::Decided(std::cmp::Ordering::Equal) => {
                    return Ok(Classification::Decided((
                        midpoint.clone(),
                        midpoint.clone(),
                        midpoint,
                    )));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        let representative = ((&lower + &upper) / Real::from(2_u8))?;
        Ok(Classification::Decided((lower, representative, upper)))
    }

    /// Projects a cut into a scalar when its retained equations provide a
    /// witness. The result can be any exact `Real`; this is not a rationality
    /// test. Rational and analytic source maps reconstruct candidate values
    /// in the selected cusp field and replay the map under the requested
    /// policy. Cuts without an available scalar witness remain mapped.
    pub(in crate::bezier_offset) fn scalar_value(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Real>>> {
        self.validate_policy(policy)?;
        match self {
            Self::Exact(parameter) => Ok(Classification::Decided(Some(parameter.clone()))),
            Self::Mapped(data) => match data.as_ref() {
                BezierAlgebraicCuspSemicircleMappedParameterData2::Rational { map, contact } => {
                    rational_mapped_cusp_scalar_value(map, contact, policy)
                }
                BezierAlgebraicCuspSemicircleMappedParameterData2::Parallel { map, contact } => {
                    parallel_mapped_cusp_scalar_value(map, contact, policy)
                }
                BezierAlgebraicCuspSemicircleMappedParameterData2::PairOverlapMap {
                    overlap,
                    source,
                    ..
                } if overlap.has_exact_endpoint_map() => {
                    Ok(source.scalar_value(policy)?.map(|parameter| {
                        parameter.map(|parameter| {
                            if overlap.data.orientation == CurveOverlapOrientation2::Same {
                                parameter
                            } else {
                                Real::one() - parameter
                            }
                        })
                    }))
                }
                BezierAlgebraicCuspSemicircleMappedParameterData2::Pair { .. }
                | BezierAlgebraicCuspSemicircleMappedParameterData2::PairOverlap { .. }
                | BezierAlgebraicCuspSemicircleMappedParameterData2::PairOverlapMap { .. } => {
                    Ok(Classification::Decided(None))
                }
                BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedFiberRational {
                    ..
                }
                | BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedFiberParallel {
                    ..
                }
                | BezierAlgebraicCuspSemicircleMappedParameterData2::Chord { .. } => {
                    Ok(Classification::Decided(None))
                }
                BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedParallelContact {
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
                } => Ok(Classification::Decided(None)),
                BezierAlgebraicCuspSemicircleMappedParameterData2::SimilarityTransport {
                    source,
                    ..
                } => source.scalar_value(policy),
                BezierAlgebraicCuspSemicircleMappedParameterData2::Chamfer {
                    source,
                    half_angle,
                    ..
                } => Ok(match source.scalar_value(policy)? {
                    Classification::Decided(Some(source)) => {
                        cusp_chamfer_parameter_value(&source, half_angle, policy)?.map(Some)
                    }
                    Classification::Decided(None) => Classification::Decided(None),
                    Classification::Uncertain(reason) => Classification::Uncertain(reason),
                }),
            },
        }
    }

    /// Replays the geometric cut point on the smallest retained exact
    /// carrier that already owns it.
    ///
    /// Rational overlap cuts use the other curve's selected parameter rather
    /// than adjoining it to the cusp field. Coincident pair-overlap endpoints
    /// similarly reuse the endpoint field of the participating semicircle.
    /// This keeps each returned point in one local field; callers may align a
    /// rational companion endpoint to that field without a primitive element.
    pub(in crate::bezier_offset) fn coincident_point_image(
        &self,
        semicircle: &BezierAlgebraicCuspSemicircle2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<RationalBezierAlgebraicPointImage2>>> {
        self.validate_policy(policy)?;
        // A selected parallel-normal circle deliberately retains its point in
        // the source parameter field plus one speed radical.  It has no
        // single algebraic-cusp field in which a constant point image could
        // be published.  Its callers use `coincident_point_evidence` below,
        // which replays the narrowest retained carrier instead.
        if semicircle.data.frame.rational().is_none() {
            return Ok(Classification::Decided(None));
        }
        match self.scalar_value(policy)? {
            Classification::Decided(Some(parameter)) => {
                return Ok(semicircle.point_at(&parameter, policy)?.map(Some));
            }
            Classification::Decided(None) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let Self::Mapped(data) = self else {
            unreachable!("an inline cusp parameter always has a scalar value");
        };
        if data.semicircle_carrier() != semicircle {
            return Ok(Classification::Decided(None));
        }
        if matches!(
            data.as_ref(),
            BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedParallelContact { .. }
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
        ) {
            return Ok(Classification::Decided(None));
        }
        if let Some((map, contact)) = data.coincident_rational_source() {
            let Some(other_parameter) = contact.other_parameter.as_bezier_parameter() else {
                return Ok(Classification::Decided(None));
            };
            return Ok(match other_parameter {
                BezierParameter2::Algebraic(parameter) => Classification::Decided(Some(
                    RationalBezierAlgebraicPointImage2::from_parametric_source(
                        map.data.curve.clone(),
                        parameter.clone(),
                        policy,
                    ),
                )),
                BezierParameter2::Exact(parameter) => map
                    .data
                    .curve
                    .point_at_classified(parameter, policy)
                    .map(|point| {
                        Some(algebraic_constant_point_image(
                            &point,
                            semicircle.cusp_parameter(),
                            policy,
                        ))
                    }),
            });
        }
        if let Some((source, start, source_policy)) = data.coincident_pair_endpoint_source() {
            if !policy.accepts_retained_policy(source_policy) {
                return Err(CurveError::Topology(
                    "algebraic cusp endpoint source used a different predicate policy".into(),
                ));
            }
            return Ok(Classification::Decided(Some(if start {
                source.start_point_image(policy)?
            } else {
                source.end_point_image(policy)?
            })));
        }
        if let Some(BezierAlgebraicCuspSemicircleMappedTangentSource2::Parallel {
            parallel,
            parameter: BezierParameter2::Exact(parameter),
            ..
        }) = data.coincident_tangent_source()
        {
            return Ok(parallel.point_at(parameter, policy)?.map(|point| {
                Some(algebraic_constant_point_image(
                    &point,
                    semicircle.cusp_parameter(),
                    policy,
                ))
            }));
        }
        Ok(Classification::Decided(None))
    }

    /// Replays the geometric cut using the narrowest retained point carrier.
    ///
    /// Most mapped cuts already have a one-field rational point image.  A
    /// selected circle/axis-chord contact deliberately remains a correlated
    /// two-field radical point, so return its existing one-word point handle
    /// instead of trying to flatten it into a local algebraic image.
    pub(crate) fn coincident_point_evidence(
        &self,
        semicircle: &BezierAlgebraicCuspSemicircle2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<CurvePoint2>>> {
        self.validate_policy(policy)?;
        if let Self::Exact(parameter) = self
            && semicircle.data.frame.rational().is_none()
        {
            return Ok(semicircle.point_evidence_at(parameter, policy)?.map(Some));
        }
        if let Self::Mapped(data) = self
            && let Some(point) = data.retained_or_selected_point_evidence()
        {
            // The parameter owns this exact point identity, including a
            // certified chord endpoint. Reuse it across every selected-frame
            // kind instead of constructing a second coordinate field.
            return Ok(Classification::Decided(
                (data.semicircle_carrier() == semicircle).then_some(point),
            ));
        }
        if let Self::Mapped(data) = self
            && let BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedParallelContact {
                semicircle: source,
                parallel,
                parameter,
                ..
            } = data.as_ref()
        {
            return Ok(Classification::Decided((source == semicircle).then(|| {
                CurvePoint2::from(
                    BezierAnalyticParallelPoint2::new_with_region_parameter_and_tangent_distance(
                        parallel.clone(),
                        parameter,
                        Real::zero(),
                        policy,
                    )
                    .expect("a retained parallel contact has a scalar parameter"),
                )
            })));
        }
        if let Self::Mapped(data) = self
            && data.semicircle_carrier() == semicircle
            && matches!(
                data.as_ref(),
                BezierAlgebraicCuspSemicircleMappedParameterData2::Chord { .. }
            )
        {
            return Ok(Classification::Decided(Some(CurvePoint2::from(
                BezierAlgebraicCuspChordPoint2 { data: data.clone() },
            ))));
        }
        if let Self::Mapped(data) = self
            && data.semicircle_carrier() == semicircle
            && let Some(chord_parameter) = data.coincident_chord_parameter()
        {
            return Ok(Classification::Decided(Some(CurvePoint2::from(
                BezierAlgebraicCuspChordDerivedPoint2::from_mapped_source(
                    data.clone(),
                    Some(CurvePoint2::from(BezierAlgebraicCuspChordPoint2 {
                        data: chord_parameter,
                    })),
                    Real::one(),
                ),
            ))));
        }
        if let Self::Mapped(data) = self
            && data.semicircle_carrier() == semicircle
            && let Some(BezierAlgebraicCuspSemicircleMappedTangentSource2::Parallel {
                parallel,
                parameter,
                ..
            }) = data.coincident_tangent_source()
            && parallel.distance().zero_status() == ZeroKnowledge::Zero
        {
            let source = parallel.source().to_rational_bezier()?;
            match exact_contact_point_evidence(&source, parameter, policy)? {
                Classification::Decided(point) => return Ok(Classification::Decided(Some(point))),
                Classification::Uncertain(UncertaintyReason::Boundary) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(_) => {}
            }
        }
        if semicircle.data.frame.rational().is_none()
            && let Self::Mapped(data) = self
            && data.semicircle_carrier() == semicircle
            && let Some((map, contact)) = data.coincident_rational_source()
            && let Classification::Decided(point) = rational_point_evidence_at_region_parameter(
                &map.data.curve,
                &contact.other_parameter,
                policy,
            )?
        {
            return Ok(Classification::Decided(Some(point)));
        }
        match self.coincident_point_image(semicircle, policy)? {
            Classification::Decided(Some(point)) => {
                return Ok(Classification::Decided(Some(CurvePoint2::from(point))));
            }
            Classification::Decided(None) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let Self::Mapped(data) = self else {
            unreachable!("an exact cusp parameter has a coincident point image");
        };
        if data.semicircle_carrier() == semicircle
            && (matches!(
                data.coincident_tangent_source(),
                Some(BezierAlgebraicCuspSemicircleMappedTangentSource2::Parallel { .. })
            ) || data.coincident_pair_source().is_some())
        {
            return Ok(Classification::Decided(Some(CurvePoint2::from(
                BezierAlgebraicCuspChordDerivedPoint2::from_mapped_source(
                    data.clone(),
                    None,
                    Real::one(),
                ),
            ))));
        }
        Ok(Classification::Decided(None))
    }

    /// Replays this mapped point after an exact concentric radius change.
    ///
    /// Represented parameters evaluate directly on the target carrier. A
    /// correlated coincident-circle boundary instead reuses the participating
    /// circle on which that point is structurally parameter 0 or 1, scaling
    /// its signed radius by the same target/source ratio. Other genuinely
    /// multi-field cuts remain explicit and decline this path.
    pub(in crate::bezier_offset) fn concentric_offset_point_image(
        &self,
        source: &BezierAlgebraicCuspSemicircle2,
        offset: &BezierAlgebraicCuspSemicircle2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<RationalBezierAlgebraicPointImage2>>> {
        self.validate_policy(policy)?;
        if source.data.frame != offset.data.frame || source.is_clockwise() != offset.is_clockwise()
        {
            return Ok(Classification::Decided(None));
        }
        if source == offset {
            return self.coincident_point_image(source, policy);
        }
        match self.scalar_value(policy)? {
            Classification::Decided(Some(parameter)) => {
                return Ok(offset.point_at(&parameter, policy)?.map(Some));
            }
            Classification::Decided(None) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let Self::Mapped(data) = self else {
            unreachable!("an inline cusp parameter always has a scalar value");
        };
        if data.semicircle_carrier() != source {
            return Ok(Classification::Decided(None));
        }
        let Some((endpoint_circle, start, endpoint_policy)) =
            data.coincident_pair_endpoint_source()
        else {
            return Ok(Classification::Decided(None));
        };
        if !policy.accepts_retained_policy(endpoint_policy) {
            return Err(CurveError::Topology(
                "coincident circle endpoint was replayed under a different predicate policy".into(),
            ));
        }
        let radial_scale = (offset.radial_distance() / source.radial_distance())?;
        let endpoint_circle = match endpoint_circle.scaled_radial_distance(&radial_scale, policy)? {
            Classification::Decided(Some(circle)) => circle,
            Classification::Decided(None) => {
                return Err(CurveError::Topology(
                    "nonzero concentric offset collapsed a coincident endpoint carrier".into(),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(Classification::Decided(Some(if start {
            endpoint_circle.start_point_image(policy)?
        } else {
            endpoint_circle.end_point_image(policy)?
        })))
    }

    /// Replays a mapped endpoint on a concentric circle without flattening
    /// independent selected fields.
    ///
    /// Circle/chord contacts retain their specialized radical system. Other
    /// mapped cuts whose coincident source point already has exact evidence
    /// keep that point beside the selected-center proof and reuse the same
    /// affine derived carrier.
    pub(crate) fn concentric_offset_point_evidence(
        &self,
        source: &BezierAlgebraicCuspSemicircle2,
        offset: &BezierAlgebraicCuspSemicircle2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<CurvePoint2>>> {
        self.validate_policy(policy)?;
        if source.data.frame != offset.data.frame || source.is_clockwise() != offset.is_clockwise()
        {
            return Ok(Classification::Decided(None));
        }
        if source == offset {
            return self.coincident_point_evidence(source, policy);
        }
        match self.scalar_value(policy)? {
            Classification::Decided(Some(parameter)) => {
                return Ok(offset.point_evidence_at(&parameter, policy)?.map(Some));
            }
            Classification::Decided(None) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        match self.concentric_offset_point_image(source, offset, policy)? {
            Classification::Decided(Some(point)) => {
                return Ok(Classification::Decided(Some(CurvePoint2::from(point))));
            }
            Classification::Decided(None) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let Self::Mapped(data) = self else {
            unreachable!("an exact cusp parameter has a concentric point image");
        };
        if data.semicircle_carrier() != source {
            return Ok(Classification::Decided(None));
        }
        let radial_scale = (offset.radial_distance() / source.radial_distance())?;
        if matches!(
            data.as_ref(),
            BezierAlgebraicCuspSemicircleMappedParameterData2::Chord { .. }
        ) {
            return Ok(Classification::Decided(Some(CurvePoint2::from(
                BezierAlgebraicCuspChordPoint2 { data: data.clone() }.radial_scaled(radial_scale),
            ))));
        }
        if let Some(chord_parameter) = data.coincident_chord_parameter() {
            return Ok(Classification::Decided(Some(CurvePoint2::from(
                BezierAlgebraicCuspChordDerivedPoint2::from_mapped_source(
                    data.clone(),
                    Some(CurvePoint2::from(BezierAlgebraicCuspChordPoint2 {
                        data: chord_parameter,
                    })),
                    radial_scale,
                ),
            ))));
        }
        if matches!(
            data.coincident_tangent_source(),
            Some(BezierAlgebraicCuspSemicircleMappedTangentSource2::Parallel { .. })
        ) || data.coincident_pair_source().is_some()
        {
            return Ok(Classification::Decided(Some(CurvePoint2::from(
                BezierAlgebraicCuspChordDerivedPoint2::from_mapped_source(
                    data.clone(),
                    None,
                    radial_scale,
                ),
            ))));
        }
        let point = match self.coincident_point_evidence(source, policy)? {
            Classification::Decided(Some(point)) => point,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(Classification::Decided(Some(CurvePoint2::from(
            BezierAlgebraicCuspChordDerivedPoint2::from_mapped_source(
                data.clone(),
                Some(point),
                radial_scale,
            ),
        ))))
    }

    /// Peels only certified coincident-circle transports from this parameter.
    ///
    /// A `PairOverlapMap` changes the selected semicircle and its local
    /// parameter, but its construction proves that the geometric point is the
    /// source point.  Retaining the base parameter lets independently nested
    /// rational, analytic-parallel, pair, and chord cuts share that exact
    /// identity without flattening either selected field.
    pub(in crate::bezier_offset) fn coincident_base_parameter(&self) -> &Self {
        match self {
            Self::Mapped(data) => match data.as_ref() {
                BezierAlgebraicCuspSemicircleMappedParameterData2::PairOverlapMap {
                    source,
                    ..
                } => source.coincident_base_parameter(),
                _ => self,
            },
            Self::Exact(_) => self,
        }
    }

    pub(in crate::bezier_offset) fn shares_coincident_point_evidence(&self, other: &Self) -> bool {
        let (Self::Mapped(_), Self::Mapped(_)) = (
            self.coincident_base_parameter(),
            other.coincident_base_parameter(),
        ) else {
            // An inline scalar parameter has no carrier identity. Equal
            // numbers on independently parameterized circles therefore
            // cannot certify equal physical points after wrappers are peeled.
            return false;
        };
        self.coincident_base_parameter()
            .shares_exact_evidence(other.coincident_base_parameter())
    }

    /// Whether two retained values are the same scalar on the same local
    /// selected-circle chart, even when a concentric offset changed the
    /// physical carrier and endpoint point.
    ///
    /// Selected pair contacts are defined entirely by their source pair's
    /// tangent cross/dot authority plus the chart frame, traversal, anchor
    /// side, and radial orientation. The signed radius magnitude and cached
    /// Cartesian point do not enter that angular equation.
    pub(in crate::bezier_offset) fn shares_exact_local_parameter_authority(
        &self,
        other: &Self,
    ) -> bool {
        let (Self::Mapped(first), Self::Mapped(second)) = (self, other) else {
            return false;
        };
        let (
            BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedPairContact {
                semicircle: first_semicircle,
                map: first_map,
                contact: first_contact,
                anchor_first: first_anchor,
                radial_product_sign: first_radial,
                policy: first_policy,
                ..
            },
            BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedPairContact {
                semicircle: second_semicircle,
                map: second_map,
                contact: second_contact,
                anchor_first: second_anchor,
                radial_product_sign: second_radial,
                policy: second_policy,
                ..
            },
        ) = (first.as_ref(), second.as_ref())
        else {
            return false;
        };
        first_semicircle.data.frame == second_semicircle.data.frame
            && first_semicircle.is_clockwise() == second_semicircle.is_clockwise()
            && Arc::ptr_eq(&first_map.data, &second_map.data)
            && first_contact == second_contact
            && first_anchor == second_anchor
            && first_radial == second_radial
            && first_policy == second_policy
    }

    pub(crate) fn shares_exact_evidence(&self, other: &Self) -> bool {
        match (self, other) {
            (Self::Exact(first), Self::Exact(second)) => first == second,
            (Self::Mapped(first), Self::Mapped(second)) => {
                Arc::ptr_eq(first, second) || match (first.as_ref(), second.as_ref()) {
                    (
                        BezierAlgebraicCuspSemicircleMappedParameterData2::Rational {
                            map: first_map,
                            contact: first_contact,
                        },
                        BezierAlgebraicCuspSemicircleMappedParameterData2::Rational {
                            map: second_map,
                            contact: second_contact,
                        },
                    ) => {
                        Arc::ptr_eq(&first_map.data, &second_map.data)
                            && first_contact == second_contact
                    }
                    (
                        BezierAlgebraicCuspSemicircleMappedParameterData2::Parallel {
                            map: first_map,
                            contact: first_contact,
                        },
                        BezierAlgebraicCuspSemicircleMappedParameterData2::Parallel {
                            map: second_map,
                            contact: second_contact,
                        },
                    ) => {
                        Arc::ptr_eq(&first_map.data, &second_map.data)
                            && first_contact == second_contact
                    }
                    (
                        BezierAlgebraicCuspSemicircleMappedParameterData2::Pair {
                            map: first_map,
                            contact: first_contact,
                            first: first_side,
                        },
                        BezierAlgebraicCuspSemicircleMappedParameterData2::Pair {
                            map: second_map,
                            contact: second_contact,
                            first: second_side,
                        },
                    ) => {
                        Arc::ptr_eq(&first_map.data, &second_map.data)
                            && first_contact == second_contact
                            && first_side == second_side
                    }
                    (
                        BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedPairContact {
                            semicircle: first_semicircle,
                            map: first_map,
                            contact: first_contact,
                            anchor_first: first_side,
                            radial_product_sign: first_radial,
                            point: first_point,
                            ..
                        },
                        BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedPairContact {
                            semicircle: second_semicircle,
                            map: second_map,
                            contact: second_contact,
                            anchor_first: second_side,
                            radial_product_sign: second_radial,
                            point: second_point,
                            ..
                        },
                    ) => {
                        first_semicircle == second_semicircle
                            && Arc::ptr_eq(&first_map.data, &second_map.data)
                            && first_contact == second_contact
                            && first_side == second_side
                            && first_radial == second_radial
                            && first_point == second_point
                    }
                    (
                        BezierAlgebraicCuspSemicircleMappedParameterData2::Chord {
                            map: first_map,
                            contact: first_contact,
                        },
                        BezierAlgebraicCuspSemicircleMappedParameterData2::Chord {
                            map: second_map,
                            contact: second_contact,
                        },
                    ) => {
                        Arc::ptr_eq(&first_map.data, &second_map.data)
                            && first_contact == second_contact
                    }
                    (
                        BezierAlgebraicCuspSemicircleMappedParameterData2::PairOverlap {
                            overlap: first_overlap,
                            endpoint: first_endpoint,
                            first: first_side,
                        },
                        BezierAlgebraicCuspSemicircleMappedParameterData2::PairOverlap {
                            overlap: second_overlap,
                            endpoint: second_endpoint,
                            first: second_side,
                        },
                    ) => {
                        first_overlap.shares_parameter_map(true, second_overlap, true)
                            && first_endpoint == second_endpoint
                            && first_side == second_side
                    }
                    (
                        BezierAlgebraicCuspSemicircleMappedParameterData2::PairOverlapMap {
                            overlap: first_overlap,
                            source: first_source,
                            source_first: first_side,
                        },
                        BezierAlgebraicCuspSemicircleMappedParameterData2::PairOverlapMap {
                            overlap: second_overlap,
                            source: second_source,
                            source_first: second_side,
                        },
                    ) => {
                        first_overlap.shares_parameter_map(
                            *first_side,
                            second_overlap,
                            *second_side,
                        ) && first_source.shares_exact_evidence(second_source)
                    }
                    (
                        BezierAlgebraicCuspSemicircleMappedParameterData2::Chamfer {
                            semicircle: first_semicircle,
                            source: first_source,
                            half_angle: first_half_angle,
                            ..
                        },
                        BezierAlgebraicCuspSemicircleMappedParameterData2::Chamfer {
                            semicircle: second_semicircle,
                            source: second_source,
                            half_angle: second_half_angle,
                            ..
                        },
                    ) => {
                        first_semicircle == second_semicircle
                            && first_source.shares_exact_evidence(second_source)
                            && first_half_angle == second_half_angle
                    }
                    (
                        BezierAlgebraicCuspSemicircleMappedParameterData2::SimilarityTransport {
                            semicircle: first_semicircle,
                            source: first_source,
                            point: first_point,
                            ..
                        },
                        BezierAlgebraicCuspSemicircleMappedParameterData2::SimilarityTransport {
                            semicircle: second_semicircle,
                            source: second_source,
                            point: second_point,
                            ..
                        },
                    ) => {
                        first_semicircle == second_semicircle
                            && first_source.shares_exact_evidence(second_source)
                            && first_point == second_point
                    }
                    _ => false,
                }
            }
            (Self::Exact(_), Self::Mapped(_)) | (Self::Mapped(_), Self::Exact(_)) => false,
        }
    }

    pub(in crate::bezier_offset) fn pair_overlap_evidence(
        &self,
    ) -> Option<&BezierAlgebraicCuspSemicirclePairOverlap2> {
        let Self::Mapped(parameter) = self else {
            return None;
        };
        match parameter.as_ref() {
            BezierAlgebraicCuspSemicircleMappedParameterData2::PairOverlap { overlap, .. }
            | BezierAlgebraicCuspSemicircleMappedParameterData2::PairOverlapMap {
                overlap, ..
            } => Some(overlap),
            _ => None,
        }
    }

    pub(in crate::bezier_offset) fn correlated_chord_same_value(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Option<Classification<bool>>> {
        let (Self::Mapped(first), Self::Mapped(second)) = (self, other) else {
            return Ok(None);
        };
        let (
            BezierAlgebraicCuspSemicircleMappedParameterData2::Chord {
                map: first_map,
                contact: first_contact,
            },
            BezierAlgebraicCuspSemicircleMappedParameterData2::Chord {
                map: second_map,
                contact: second_contact,
            },
        ) = (first.as_ref(), second.as_ref())
        else {
            return Ok(None);
        };
        first_map.validate_policy(policy)?;
        second_map.validate_policy(policy)?;
        if first_map.data.semicircle != second_map.data.semicircle {
            return Ok(None);
        }
        if first_map.recursive_quadratic_line_system().is_some()
            || second_map.recursive_quadratic_line_system().is_some()
            || first_map.has_chord_normal_projective_system()
            || second_map.has_chord_normal_projective_system()
        {
            let first_point = BezierAlgebraicCuspChordPoint2 {
                data: first.clone(),
            };
            let second_point = BezierAlgebraicCuspChordPoint2 {
                data: second.clone(),
            };
            match first_point.same_point(&second_point, policy) {
                decided @ Classification::Decided(_) => return Ok(Some(decided)),
                Classification::Uncertain(_) => {
                    // Independently replayed compact/projective maps can lose
                    // a shared local point field while retaining the same
                    // circle, affine support, traversal axis, and contact
                    // branch. Those four facts identify the contact exactly;
                    // continue into that representation-independent proof.
                }
            }
        }
        let collinearity = first_map
            .data
            .chord
            .support_collinearity(&second_map.data.chord, policy)?;
        match collinearity {
            Classification::Decided(true) => {}
            // Distinct circle chords can still share one circle point.  A
            // non-collinear support therefore cannot certify inequality.
            Classification::Decided(false) => return Ok(None),
            Classification::Uncertain(reason) => {
                return Ok(Some(Classification::Uncertain(reason)));
            }
        }
        if first_map.data.chord.data.parameter_axis.axis
            != second_map.data.chord.data.parameter_axis.axis
        {
            return Ok(None);
        }
        let second_branch = if first_map
            .data
            .chord
            .data
            .parameter_axis
            .coordinate_increases
            == second_map
                .data
                .chord
                .data
                .parameter_axis
                .coordinate_increases
        {
            second_contact.branch
        } else {
            -second_contact.branch
        };
        Ok(Some(Classification::Decided(
            first_contact.branch == second_branch,
        )))
    }

    pub(in crate::bezier_offset) fn correlated_chord_rational_same_value(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Option<Classification<bool>>> {
        let (
            BezierAlgebraicCuspSemicircleParameter2::Mapped(first),
            BezierAlgebraicCuspSemicircleParameter2::Mapped(second),
        ) = (self, other)
        else {
            return Ok(None);
        };
        let (rational_map, rational_contact, chord_data) = match (first.as_ref(), second.as_ref()) {
            (
                BezierAlgebraicCuspSemicircleMappedParameterData2::Rational { map, contact },
                BezierAlgebraicCuspSemicircleMappedParameterData2::Chord { .. },
            ) => (map, contact, second.clone()),
            (
                BezierAlgebraicCuspSemicircleMappedParameterData2::Chord { .. },
                BezierAlgebraicCuspSemicircleMappedParameterData2::Rational { map, contact },
            ) => (map, contact, first.clone()),
            _ => return Ok(None),
        };
        // These contacts were produced by different pair maps, so pointer
        // equality of their mapped parameters cannot prove equality. Replay
        // the chord's original exact branch predicate against the rational
        // contact point instead of trying to separate equal root intervals.
        let chord_point = BezierAlgebraicCuspChordPoint2 { data: chord_data };
        let (chord_map, chord_contact) = chord_point.map_contact();
        if rational_map.data.semicircle != chord_map.data.semicircle {
            return Ok(None);
        }
        if let (Some(rational_line), Some(chord_line)) = (
            rational_map.data.curve.exact_linear_parameterization_line(),
            chord_map.data.chord.strict_provenance_support_line(policy),
        ) {
            let strict = &CurveContext::STRICT;
            let (rational_x, rational_y) = rational_line.delta();
            let (chord_x, chord_y) = chord_line.delta();
            let direction_cross =
                Real::diff_of_products(&rational_x, &chord_y, &rational_y, &chord_x);
            let support_x = chord_line.start().x() - rational_line.start().x();
            let support_y = chord_line.start().y() - rational_line.start().y();
            let support_cross =
                Real::diff_of_products(&rational_x, &support_y, &rational_y, &support_x);
            if real_sign(&direction_cross, strict) == Some(RealSign::Zero)
                && real_sign(&support_cross, strict) == Some(RealSign::Zero)
            {
                let tangent_dot = &rational_x * &chord_x + &rational_y * &chord_y;
                let orientation = match real_sign(&tangent_dot, strict) {
                    Some(RealSign::Positive) => RealSign::Positive,
                    Some(RealSign::Negative) => RealSign::Negative,
                    Some(RealSign::Zero) => {
                        return Err(CurveError::Topology(
                            "coincident nondegenerate line carriers had orthogonal traversals"
                                .into(),
                        ));
                    }
                    None => {
                        return Ok(Some(Classification::Uncertain(UncertaintyReason::RealSign)));
                    }
                };
                let intersections = match rational_map.data.semicircle.rational_intersections(
                    &rational_map.data.curve,
                    &crate::CurveParameterRange2::unit(),
                    policy,
                )? {
                    Classification::Decided(intersections) => intersections,
                    Classification::Uncertain(reason) => {
                        return Ok(Some(Classification::Uncertain(reason)));
                    }
                };
                let contacts = match intersections {
BezierAlgebraicCuspSemicircleRationalIntersections2::Mapped { contacts, overlaps } if overlaps.is_empty() => contacts,
other => return match other {
                        BezierAlgebraicCuspSemicircleRationalIntersections2::Mapped { .. } => {
                            Err(CurveError::Topology(
                                "a linear carrier replayed as a coincident circle".into(),
                            ))
                        }
                        BezierAlgebraicCuspSemicircleRationalIntersections2::DegenerateProjection => {
                            Ok(Some(Classification::Uncertain(
                                UncertaintyReason::Unsupported,
                            )))
                        }
                        BezierAlgebraicCuspSemicircleRationalIntersections2::SelectedFiber { .. } => Ok(Some(Classification::Uncertain(
                            UncertaintyReason::Unsupported,
                        ))),
                    },
};
                let mut replayed_contact = None;
                for candidate in contacts {
                    match candidate
                        .other_parameter
                        .same_value(&rational_contact.other_parameter, policy)?
                    {
                        Classification::Decided(true) => {
                            if replayed_contact.replace(candidate).is_some() {
                                return Err(CurveError::Topology(
                                    "one rational parameter replayed as multiple circle contacts"
                                        .into(),
                                ));
                            }
                        }
                        Classification::Decided(false) => {}
                        Classification::Uncertain(reason) => {
                            return Ok(Some(Classification::Uncertain(reason)));
                        }
                    }
                }
                let Some(replayed_contact) = replayed_contact else {
                    return Err(CurveError::Topology(
                        "a retained rational circle contact was absent from exact replay".into(),
                    ));
                };
                return Ok(Some(Classification::Decided(
                    replayed_contact.location == chord_contact.cusp_location
                        && replayed_contact.tangent_cross_sign
                            == product_sign(chord_contact.tangent_cross_sign, orientation),
                )));
            }
        }
        let rational_parameter =
            match promote_curve_region_bezier_parameter(&rational_contact.other_parameter, policy)?
            {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => {
                    return Ok(Some(Classification::Uncertain(reason)));
                }
            };
        let point = match rational_point_evidence_at_parameter(
            &rational_map.data.curve,
            &rational_parameter,
            policy,
        )? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Some(Classification::Uncertain(reason)));
            }
        };
        Ok(Some(chord_point.same_point_evidence(&point, policy)))
    }

    /// Returns the exact unit-complement relation for two independently
    /// mapped analytic parallel cuts. Other parameter representations do not
    /// participate in this specialized certificate.
    pub(in crate::bezier_offset) fn parallel_complementary_to(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Option<Classification<bool>>> {
        self.validate_policy(policy)?;
        other.validate_policy(policy)?;
        let (Self::Mapped(first), Self::Mapped(second)) = (self, other) else {
            return Ok(None);
        };
        first.parallel_complementary_to(second, policy)
    }

    pub(crate) fn order_to_real(
        &self,
        parameter: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        self.validate_policy(policy)?;
        match self {
            Self::Exact(exact) => Ok(compare_reals(exact, parameter, policy)
                .map(Classification::Decided)
                .unwrap_or(Classification::Uncertain(UncertaintyReason::Ordering))),
            Self::Mapped(data) => match data.as_ref() {
                BezierAlgebraicCuspSemicircleMappedParameterData2::Rational { map, contact } => {
                    map.mapped_contact_order_to_real(contact, parameter, policy)
                }
                BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedFiberRational {
                    map,
                    other_parameter,
                    location,
                    ..
                } => map.contact_order_to_real(other_parameter, *location, parameter, policy),
                BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedFiberParallel {
                    map,
                    other_parameter,
                    location,
                    ..
                } => map.contact_order_to_real(other_parameter, *location, parameter, policy),
                BezierAlgebraicCuspSemicircleMappedParameterData2::Parallel { map, contact } => {
                    map.contact_order_to_real(contact, parameter, policy)
                }
                BezierAlgebraicCuspSemicircleMappedParameterData2::Pair {
                    map,
                    contact,
                    first,
                } => {
                    map.represented_contact_order_to_real_for_side(contact, *first, parameter, policy)
                }
                BezierAlgebraicCuspSemicircleMappedParameterData2::Chord { map, contact } => {
                    map.contact_order_to_real(contact, parameter, policy)
                }
                BezierAlgebraicCuspSemicircleMappedParameterData2::PairOverlap {
                    overlap,
                    endpoint,
                    first,
                } => overlap.endpoint_order_to_real(*endpoint, *first, parameter, policy),
                BezierAlgebraicCuspSemicircleMappedParameterData2::PairOverlapMap {
                    overlap,
                    source,
                    source_first,
                } => overlap.mapped_parameter_order_to_real(source, *source_first, parameter, policy),
                BezierAlgebraicCuspSemicircleMappedParameterData2::SimilarityTransport {
                    source,
                    ..
                } => source.order_to_real(parameter, policy),
                BezierAlgebraicCuspSemicircleMappedParameterData2::Chamfer {
                    source,
                    half_angle,
                    ..
                } => cusp_chamfer_parameter_order_to_real(source, half_angle, parameter, policy),
                selected @ BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedParallelContact {
                    ..
                } => selected.selected_parallel_contact_order_to_real(parameter, policy),
                selected @ BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedCircularTangentContact {
                    ..
                } => selected.selected_circular_tangent_contact_order_to_real(parameter, policy),
                selected @ BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedPairContact {
                    ..
                } => selected.selected_pair_contact_order_to_real(parameter, policy),
                selected @ BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedChordNormalContact {
                    ..
                } => selected.selected_chord_normal_contact_order_to_real(parameter, policy),
                selected @ BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedChordParallelNormalContact {
                    ..
                } => selected.selected_chord_parallel_normal_contact_order_to_real(parameter, policy),
            },
        }
    }

    pub(crate) fn parameter_bracket(
        &self,
        refinement_steps: usize,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicircleParameterBracket2>> {
        self.validate_policy(policy)?;
        let data = match self {
            Self::Exact(parameter) => {
                return Ok(Classification::Decided(
                    BezierAlgebraicCuspSemicircleParameterBracket2::Exact(parameter.clone()),
                ));
            }
            Self::Mapped(data) => data,
        };
        let location = match data.as_ref() {
            BezierAlgebraicCuspSemicircleMappedParameterData2::Rational { contact, .. } => {
                contact.location
            }
            BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedFiberRational { location, .. }
            | BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedFiberParallel { location, .. }
            | BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedParallelContact { location, .. } => *location,
            BezierAlgebraicCuspSemicircleMappedParameterData2::Parallel { contact, .. }
                if contact.location != BezierAlgebraicCuspSemicircleContactLocation2::Interior =>
            {
                contact.location
            }
            BezierAlgebraicCuspSemicircleMappedParameterData2::Parallel { map, contact } => {
                return map.contact_parameter_bracket(contact, refinement_steps, policy);
            }
            BezierAlgebraicCuspSemicircleMappedParameterData2::Pair { contact, first, .. } => {
                if *first { contact.first_location } else { contact.second_location }
            }
            BezierAlgebraicCuspSemicircleMappedParameterData2::Chord { contact, .. } => {
                contact.cusp_location
            }
            BezierAlgebraicCuspSemicircleMappedParameterData2::PairOverlap { endpoint, first, .. } => {
                BezierAlgebraicCuspSemicirclePairOverlap2::endpoint_location(*endpoint, *first)
            }
            BezierAlgebraicCuspSemicircleMappedParameterData2::PairOverlapMap { overlap, source, source_first } => {
                return overlap.mapped_parameter_bracket(source, *source_first, refinement_steps, policy);
            }
            BezierAlgebraicCuspSemicircleMappedParameterData2::SimilarityTransport { source, .. } => {
                return source.parameter_bracket(refinement_steps, policy);
            }
            BezierAlgebraicCuspSemicircleMappedParameterData2::Chamfer { source, half_angle, .. } => {
                return cusp_chamfer_parameter_bracket(source, half_angle, refinement_steps, policy);
            }
            BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedCircularTangentContact { .. }
            | BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedPairContact { .. }
            | BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedChordNormalContact { .. }
            | BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedChordParallelNormalContact { .. } => {
                BezierAlgebraicCuspSemicircleContactLocation2::Interior
            }
        };
        let endpoint = match location {
            BezierAlgebraicCuspSemicircleContactLocation2::Start => Real::zero(),
            BezierAlgebraicCuspSemicircleContactLocation2::End => Real::one(),
            BezierAlgebraicCuspSemicircleContactLocation2::Interior => {
                return refine_algebraic_cusp_semicircle_parameter_bracket(
                    None,
                    refinement_steps,
                    |parameter| self.order_to_real(parameter, policy),
                );
            }
        };
        Ok(Classification::Decided(
            BezierAlgebraicCuspSemicircleParameterBracket2::Exact(endpoint),
        ))
    }

    /// Orders two incident points in the shared recursive selected-circle
    /// frame. For a closed half-circle chart, the oriented cross product of
    /// their center-relative radii has the traversal order sign; a zero cross
    /// with a positive dot is the same point. This keeps correlated chord
    /// contacts and authored chord-normal endpoints in their existing
    /// quadratic tower instead of comparing unrelated scalar brackets.
    pub(in crate::bezier_offset) fn recursive_projective_order(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Option<Classification<std::cmp::Ordering>>> {
        let (Self::Mapped(first), Self::Mapped(second)) = (self, other) else {
            return Ok(None);
        };
        let semicircle = first.semicircle_carrier();
        if semicircle != second.semicircle_carrier() {
            return Ok(None);
        }
        let first = match self.coincident_point_evidence(semicircle, policy)? {
            Classification::Decided(Some(point)) => point,
            Classification::Decided(None) => return Ok(None),
            Classification::Uncertain(reason) => {
                return Ok(Some(Classification::Uncertain(reason)));
            }
        };
        let second = match other.coincident_point_evidence(semicircle, policy)? {
            Classification::Decided(Some(point)) => point,
            Classification::Decided(None) => return Ok(None),
            Classification::Uncertain(reason) => {
                return Ok(Some(Classification::Uncertain(reason)));
            }
        };
        recursive_projective_incident_point_order(
            &first,
            &second,
            semicircle,
            || {
                Ok(match (&first, &second) {
                    (CurvePoint2(CurvePointData2::AlgebraicCuspChord(contact)), point)
                    | (point, CurvePoint2(CurvePointData2::AlgebraicCuspChord(contact))) => {
                        contact.contact_support_separates_point(point, policy)?
                            == Classification::Decided(true)
                    }
                    _ => false,
                })
            },
            policy,
        )
    }

    /// Reuses point identity on a shared source chart before comparing
    /// independently mapped angular parameters or Cartesian coordinates.
    pub(in crate::bezier_offset) fn shares_parametric_source_point(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<bool> {
        // Suppressing new terminal decisions does not strengthen existing
        // approximate incidence. Only certified constructions may contribute
        // a scalar identity that can later be replayed under STRICT.
        let strict = policy.strict_counterpart();
        if self.validate_policy(&strict).is_err() || other.validate_policy(&strict).is_err() {
            return Ok(false);
        }
        let (Self::Mapped(first), Self::Mapped(second)) = (self, other) else {
            return Ok(false);
        };
        if first.semicircle_carrier() != second.semicircle_carrier() {
            return Ok(false);
        }
        let (
            Some((first_curve, first_distance, first_parameter)),
            Some((second_curve, second_distance, second_parameter)),
        ) = (
            first.coincident_parametric_source()?,
            second.coincident_parametric_source()?,
        )
        else {
            return Ok(false);
        };
        if first_curve != second_curve || first_distance != second_distance {
            return Ok(false);
        }
        // Both constructions certify incidence on this very circle and source
        // chart. Each source parameter therefore satisfies the selected fiber's
        // circle-incidence equation. Strict containment in its singleton proves
        // the roots equal without evaluating either global projection polynomial.
        // This uses the two geometric incidence proofs, not interval overlap.
        let same_root = policy.strict_predicate_pass(|| -> CurveResult<bool> {
            for (selected, incident) in [
                (
                    first.isolated_circle_incidence_parameter(),
                    &second_parameter,
                ),
                (
                    second.isolated_circle_incidence_parameter(),
                    &first_parameter,
                ),
            ] {
                let Some(selected) = selected else {
                    continue;
                };
                let lower =
                    CurveParameter2::from(BezierParameter2::Exact(selected.root().lower.clone()));
                let upper =
                    CurveParameter2::from(BezierParameter2::Exact(selected.root().upper.clone()));
                if incident.cmp_by_refinement(&lower, policy)?
                    == Classification::Decided(std::cmp::Ordering::Greater)
                    && incident.cmp_by_refinement(&upper, policy)?
                        == Classification::Decided(std::cmp::Ordering::Less)
                {
                    if let Some(parameter) = incident.as_bezier_parameter() {
                        selected.retain_certified_parameter(parameter.clone());
                    }
                    return Ok(true);
                }
            }
            Ok(first_parameter.same_value(&second_parameter, policy)?
                == Classification::Decided(true))
        })?;
        #[cfg(feature = "dispatch-trace")]
        if same_root {
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-circle-parameter-order",
                "shared-source-identity",
            );
        }
        Ok(same_root)
    }

    /// Reuses tangent incidence on the open half-circle. Parallel tangent
    /// lines identify one point there; the antipodal ambiguity is excluded by
    /// the strict angular-domain proofs. The source tangent stays unnormalized.
    pub(in crate::bezier_offset) fn shares_tangent_point(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<bool> {
        let strict = policy.strict_counterpart();
        if self.validate_policy(&strict).is_err() || other.validate_policy(&strict).is_err() {
            return Ok(false);
        }
        let (Self::Mapped(first), Self::Mapped(second)) = (self, other) else {
            return Ok(false);
        };
        if first.semicircle_carrier() != second.semicircle_carrier() {
            return Ok(false);
        }
        for (chord_source, other_source) in [(first, second), (second, first)] {
            let Some((_, chord, RealSign::Zero, chord_policy, _)) =
                chord_source.coincident_chord_tangent_source()
            else {
                continue;
            };
            if !strict.accepts_retained_policy(chord_policy) {
                continue;
            }
            if let Some((_, other_chord, RealSign::Zero, source_policy, _)) =
                other_source.coincident_chord_tangent_source()
            {
                if !strict.accepts_retained_policy(source_policy)
                    || chord.tangent_cross_sign(other_chord, &strict)?
                        != Classification::Decided(RealSign::Zero)
                {
                    continue;
                }
            } else {
                let Some((_, [x, y], source_policy)) =
                    other_source.coincident_tangent_power_source(&strict)?
                else {
                    continue;
                };
                let [x, y] = [x, y].map(polynomial_trim_structural_zeros);
                if !strict.accepts_retained_policy(source_policy) || x.len() > 1 || y.len() > 1 {
                    continue;
                }
                let tangent = (
                    x.first().cloned().unwrap_or_else(Real::zero),
                    y.first().cloned().unwrap_or_else(Real::zero),
                );
                if real_sign(
                    &(&tangent.0 * &tangent.0 + &tangent.1 * &tangent.1),
                    &strict,
                ) != Some(RealSign::Positive)
                    || chord.tangent_cross_vector_sign(&tangent, &strict)?
                        != Classification::Decided(RealSign::Zero)
                {
                    continue;
                }
            }
            for parameter in [self, other] {
                if parameter.order_to_real(&Real::zero(), &strict)?
                    != Classification::Decided(std::cmp::Ordering::Greater)
                    || parameter.order_to_real(&Real::one(), &strict)?
                        != Classification::Decided(std::cmp::Ordering::Less)
                {
                    return Ok(false);
                }
            }
            return Ok(true);
        }
        Ok(false)
    }

    /// The cosine of this chart's angle, retained in its selected field.
    /// It decreases strictly with the public half-circle parameter regardless
    /// of the circle's center, signed radius, or traversal direction.
    pub(in crate::bezier_offset) fn recursive_cosine(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Option<BezierRecursiveQuadraticProjectiveScalar2>> {
        if let Self::Mapped(data) = self {
            match data.as_ref() {
                BezierAlgebraicCuspSemicircleMappedParameterData2::Pair {
                    map,
                    contact,
                    first,
                } => {
                    if let Some(data) = map.recursive_contact_data(contact) {
                        let angular = &data.angular[usize::from(!first)];
                        return Ok(Some(BezierRecursiveQuadraticProjectiveScalar2 {
                            numerator: angular.diameter.clone(),
                            denominator: angular.radius_squared_denominator.clone(),
                        }));
                    }
                }
                BezierAlgebraicCuspSemicircleMappedParameterData2::SimilarityTransport {
                    source,
                    ..
                } => return source.recursive_cosine(policy),
                _ => {}
            }
        }
        let Some((circle, radial, location)) = self.translated_pair_contact_radial() else {
            return Ok(None);
        };
        let Classification::Decided(Some(frame)) =
            circle.recursive_circle_frame_authority(policy)?
        else {
            return Ok(None);
        };
        let Some((radial, denominator)) = (|| {
            Some((
                [
                    frame.field.constant(radial[0].clone())?,
                    frame.field.constant(radial[1].clone())?,
                ],
                frame.field.constant(Real::one())?,
            ))
        })() else {
            return Ok(None);
        };
        // A translated pair already owns its center-relative radial. Import
        // that vector into the retained frame; reconstructing world-space
        // points would obscure the same angular identity behind cancellation.
        Ok(
            match circle.recursive_pair_contact_side(
                &frame.center,
                &frame.support_center,
                &frame.normal_denominator,
                &radial,
                &denominator,
                Some(location),
            )? {
                Classification::Decided(Some(side)) => {
                    Some(BezierRecursiveQuadraticProjectiveScalar2 {
                        numerator: side.angular.diameter,
                        denominator: side.angular.radius_squared_denominator,
                    })
                }
                Classification::Decided(None) | Classification::Uncertain(_) => None,
            },
        )
    }

    pub(crate) fn cmp_by_refinement(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        self.validate_policy(policy)?;
        other.validate_policy(policy)?;
        if self.shares_exact_evidence(other) {
            return Ok(Classification::Decided(std::cmp::Ordering::Equal));
        }
        if self.shares_exact_local_parameter_authority(other) {
            return Ok(Classification::Decided(std::cmp::Ordering::Equal));
        }
        if self.shares_parametric_source_point(other, policy)? {
            return Ok(Classification::Decided(std::cmp::Ordering::Equal));
        }
        if self.shares_tangent_point(other, policy)? {
            return Ok(Classification::Decided(std::cmp::Ordering::Equal));
        }
        if let Some(Classification::Decided(order)) =
            policy.strict_predicate_pass(|| self.translated_pair_contact_order(other, policy))
        {
            return Ok(Classification::Decided(order));
        }
        if let Self::Mapped(data) = self
            && let BezierAlgebraicCuspSemicircleMappedParameterData2::SimilarityTransport {
                source,
                ..
            } = data.as_ref()
        {
            return source.cmp_by_refinement(other, policy);
        }
        if let Self::Mapped(data) = other
            && let BezierAlgebraicCuspSemicircleMappedParameterData2::SimilarityTransport {
                source,
                ..
            } = data.as_ref()
        {
            return self.cmp_by_refinement(source, policy);
        }
        match (
            self.retained_pair_parameter(),
            other.retained_pair_parameter(),
        ) {
            (Some((first, first_complement)), Some((second, second_complement)))
                if first_complement == second_complement =>
            {
                return Ok(first.cmp_by_refinement(second, policy)?.map(|order| {
                    if first_complement {
                        order.reverse()
                    } else {
                        order
                    }
                }));
            }
            (Some((source, false)), _) => {
                return source.cmp_by_refinement(other, policy);
            }
            (_, Some((source, false))) => {
                return self.cmp_by_refinement(source, policy);
            }
            _ => {}
        }
        if let Self::Mapped(data) = self
            && let BezierAlgebraicCuspSemicircleMappedParameterData2::Chamfer {
                source,
                half_angle,
                ..
            } = data.as_ref()
            && source.shares_exact_evidence(other)
        {
            return Ok(real_sign(half_angle, policy)
                .map(|sign| {
                    Classification::Decided(match sign {
                        RealSign::Negative => std::cmp::Ordering::Less,
                        RealSign::Zero => std::cmp::Ordering::Equal,
                        RealSign::Positive => std::cmp::Ordering::Greater,
                    })
                })
                .unwrap_or(Classification::Uncertain(UncertaintyReason::RealSign)));
        }
        if let Self::Mapped(data) = other
            && let BezierAlgebraicCuspSemicircleMappedParameterData2::Chamfer {
                source,
                half_angle,
                ..
            } = data.as_ref()
            && source.shares_exact_evidence(self)
        {
            return Ok(real_sign(half_angle, policy)
                .map(|sign| {
                    Classification::Decided(match sign {
                        RealSign::Negative => std::cmp::Ordering::Greater,
                        RealSign::Zero => std::cmp::Ordering::Equal,
                        RealSign::Positive => std::cmp::Ordering::Less,
                    })
                })
                .unwrap_or(Classification::Uncertain(UncertaintyReason::RealSign)));
        }
        if let (Self::Mapped(first), Self::Mapped(second)) = (self, other)
            && let (
                BezierAlgebraicCuspSemicircleMappedParameterData2::Chamfer {
                    semicircle: first_semicircle,
                    source: first_source,
                    half_angle: first_half_angle,
                    ..
                },
                BezierAlgebraicCuspSemicircleMappedParameterData2::Chamfer {
                    semicircle: second_semicircle,
                    source: second_source,
                    half_angle: second_half_angle,
                    ..
                },
            ) = (first.as_ref(), second.as_ref())
            && first_semicircle == second_semicircle
            && first_source.shares_exact_evidence(second_source)
        {
            return Ok(compare_reals(first_half_angle, second_half_angle, policy)
                .map(Classification::Decided)
                .unwrap_or(Classification::Uncertain(UncertaintyReason::Ordering)));
        }
        if let (Self::Mapped(first), Self::Mapped(second)) = (self, other)
            && let (
                BezierAlgebraicCuspSemicircleMappedParameterData2::PairOverlapMap {
                    overlap: first_overlap,
                    source: first_source,
                    source_first: first_source_side,
                },
                BezierAlgebraicCuspSemicircleMappedParameterData2::PairOverlapMap {
                    overlap: second_overlap,
                    source: second_source,
                    source_first: second_source_side,
                },
            ) = (first.as_ref(), second.as_ref())
            && first_overlap.shares_parameter_map(
                *first_source_side,
                second_overlap,
                *second_source_side,
            )
        {
            // One coincident-circle overlap is monotone on its retained
            // source range. Preserve the source order directly instead of
            // reconstructing two destination brackets through the same
            // selected-root correspondence.
            return Ok(first_source
                .cmp_by_refinement(second_source, policy)?
                .map(|order| {
                    if first_overlap.data.orientation == CurveOverlapOrientation2::Same {
                        order
                    } else {
                        order.reverse()
                    }
                }));
        }
        if let Some(Classification::Decided(true)) =
            self.correlated_chord_same_value(other, policy)?
        {
            return Ok(Classification::Decided(std::cmp::Ordering::Equal));
        }
        if let Some(Classification::Decided(true)) =
            self.correlated_chord_rational_same_value(other, policy)?
        {
            return Ok(Classification::Decided(std::cmp::Ordering::Equal));
        }
        if let Some(Classification::Decided(true)) =
            self.parallel_complementary_to(other, policy)?
        {
            // For v=1-u, ordering u against v is exactly ordering u against
            // 1/2. This keeps independently parameterized coincident
            // analytic carriers out of a redundant two-map bracket replay.
            let half = (Real::one() / Real::from(2_i8))?;
            return self.order_to_real(&half, policy);
        }
        if let Self::Exact(parameter) = other {
            return self.order_to_real(parameter, policy);
        }
        if let Self::Exact(parameter) = self {
            return Ok(other
                .order_to_real(parameter, policy)?
                .map(|order| order.reverse()));
        }
        let cosine_order = policy.bounded_exact_predicate_pass(|| -> CurveResult<_> {
            let (Some(first), Some(second)) = (
                self.recursive_cosine(policy)?,
                other.recursive_cosine(policy)?,
            ) else {
                return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
            };
            let coordinate = |scalar| {
                BezierRecursiveProjectiveParameter2::new_with_certified_bounds(
                    scalar,
                    Some((Real::from(-1_i8), Real::one())),
                    policy,
                )
            };
            let first = match coordinate(first)? {
                Classification::Decided(value) => value,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            let second = match coordinate(second)? {
                Classification::Decided(value) => value,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            Ok(first
                .cmp_by_refinement(&second, policy)?
                .map(std::cmp::Ordering::reverse))
        })?;
        if let Classification::Decided(order) = cosine_order {
            return Ok(Classification::Decided(order));
        }
        if let Some(Classification::Decided(order)) = policy
            .bounded_exact_predicate_pass(|| self.recursive_projective_order(other, policy))?
        {
            return Ok(Classification::Decided(order));
        }
        const MAX_REFINEMENT_STEPS: usize =
            (-hypersolve::PredicatePolicy::MAX_REFINEMENT_PRECISION) as usize;

        let maximum_refinement_steps = if policy.has_bounded_exact_predicate_budget() {
            8
        } else {
            MAX_REFINEMENT_STEPS
        };
        let mut refinement_steps = 0_usize;
        loop {
            let first = match self.parameter_bracket(refinement_steps, policy)? {
                Classification::Decided(bracket) => bracket,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let second = match other.parameter_bracket(refinement_steps, policy)? {
                Classification::Decided(bracket) => bracket,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let (first_start, first_end) = cusp_semicircle_parameter_bracket_bounds(&first);
            let (second_start, second_end) = cusp_semicircle_parameter_bracket_bounds(&second);
            if let (
                BezierAlgebraicCuspSemicircleParameterBracket2::Exact(first),
                BezierAlgebraicCuspSemicircleParameterBracket2::Exact(second),
            ) = (&first, &second)
            {
                return Ok(compare_reals(first, second, policy)
                    .map(Classification::Decided)
                    .unwrap_or(Classification::Uncertain(UncertaintyReason::Ordering)));
            }
            if matches!(
                compare_reals(first_end, second_start, policy),
                Some(std::cmp::Ordering::Less)
            ) {
                return Ok(Classification::Decided(std::cmp::Ordering::Less));
            }
            if matches!(
                compare_reals(second_end, first_start, policy),
                Some(std::cmp::Ordering::Less)
            ) {
                return Ok(Classification::Decided(std::cmp::Ordering::Greater));
            }
            if refinement_steps == maximum_refinement_steps {
                break;
            }
            refinement_steps = if refinement_steps == 0 {
                1
            } else {
                refinement_steps
                    .saturating_mul(2)
                    .min(maximum_refinement_steps)
            };
        }
        if policy.permits_approximate_512() {
            policy.observe_approximate_512();
            return Ok(Classification::Decided(std::cmp::Ordering::Equal));
        }
        Ok(Classification::Uncertain(UncertaintyReason::Ordering))
    }

    pub(crate) fn strict_scalar_between(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Real>> {
        match self.cmp_by_refinement(other, policy)? {
            Classification::Decided(std::cmp::Ordering::Less) => {}
            Classification::Decided(std::cmp::Ordering::Equal | std::cmp::Ordering::Greater) => {
                return Err(CurveError::InvalidBezierRange);
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let mut refinement_steps = 0_usize;
        loop {
            let first = match self.parameter_bracket(refinement_steps, policy)? {
                Classification::Decided(bracket) => bracket,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let second = match other.parameter_bracket(refinement_steps, policy)? {
                Classification::Decided(bracket) => bracket,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let (_, first_end) = cusp_semicircle_parameter_bracket_bounds(&first);
            let (second_start, _) = cusp_semicircle_parameter_bracket_bounds(&second);
            if compare_reals(first_end, second_start, policy) == Some(std::cmp::Ordering::Less) {
                return Ok(Classification::Decided(
                    ((first_end + second_start) / Real::from(2_i8))?,
                ));
            }
            refinement_steps = refinement_steps
                .checked_mul(2)
                .and_then(|steps| steps.checked_add(1))
                .ok_or_else(|| {
                    CurveError::Topology("algebraic cusp cut gap refinement overflow".into())
                })?;
        }
    }
}

impl BezierAlgebraicCuspSemicirclePairParameterMap2 {
    pub(in crate::bezier_offset) fn compact_source_parameters(
        &self,
    ) -> Option<[BezierParameter2; 2]> {
        let first = self.data.first_semicircle.data.frame.rational()?;
        let second = self.data.second_semicircle.data.frame.rational()?;
        Some([
            BezierParameter2::Algebraic(first.data.parameter.clone()),
            BezierParameter2::Algebraic(second.data.parameter.clone()),
        ])
    }

    pub(in crate::bezier_offset) fn represented_contact_data(
        &self,
        contact: &BezierAlgebraicCuspSemicirclePairContact2,
    ) -> Option<(
        &BezierRepresentedCirclePairParameterMapSystem2,
        &BezierRepresentedCirclePairContactData2,
    )> {
        let BezierCirclePairParameterMapSystem2::Represented(system) = &self.data.system else {
            return None;
        };
        let data = system
            .contacts
            .iter()
            .find(|candidate| candidate.branch == contact.branch)?;
        Some((system, data))
    }

    pub(in crate::bezier_offset) fn recursive_contact_data(
        &self,
        contact: &BezierAlgebraicCuspSemicirclePairContact2,
    ) -> Option<&BezierRecursiveCirclePairContactData2> {
        let BezierCirclePairParameterMapSystem2::Recursive(system) = &self.data.system else {
            return None;
        };
        system
            .contacts
            .iter()
            .find(|candidate| candidate.branch == contact.branch)
    }

    /// Replays the ordinary circle-pair construction directly over the least
    /// shared recursive field of its two centers. With `D=C2-C1`, `Q=D·D`,
    /// `L=Q+r1²-r2²`, and `S=4Qr1²-L²`, each contact is
    ///
    /// `C1 + (L D + branch sqrt(S) J(D))/(2Q)`.
    ///
    /// Projective center denominators are carried through this identity, and
    /// the sole new positive discriminant root becomes one quadratic
    /// extension. No Cartesian coordinate is eliminated independently.
    pub(in crate::bezier_offset) fn recursive_geometric_pair_contact_frame(
        &self,
        contact: &BezierAlgebraicCuspSemicirclePairContact2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierRecursiveQuadraticPairContactFrame2>>> {
        if !(-1..=1).contains(&contact.branch) {
            return Err(CurveError::Topology(
                "a recursive pair contact retained an invalid radical branch".into(),
            ));
        }
        if let Some(data) = self.recursive_contact_data(contact) {
            return Ok(Classification::Decided(Some(data.frame.clone())));
        }
        let first_center = match self.data.first_semicircle.center_point_evidence(policy)? {
            Classification::Decided(center) => center,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let second_center = match self.data.second_semicircle.center_point_evidence(policy)? {
            Classification::Decided(center) => center,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let center_sources = [&first_center, &second_center];
        let centers = match recursive_projective_evidence_points(&center_sources, policy)? {
            Classification::Decided(Some(centers)) => centers,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let [first, second]: [BezierRecursiveQuadraticProjectivePoint2; 2] = centers
            .try_into()
            .expect("a recursive pair construction retains two centers");
        let first = match positive_recursive_projective_point(first)? {
            Classification::Decided(center) => center,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let second = match positive_recursive_projective_point(second)? {
            Classification::Decided(center) => center,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let parent = first.denominator.field();
        if !parent.same_field(&second.denominator.field()) {
            return Err(CurveError::Topology(
                "a recursive circle pair failed to share its center field".into(),
            ));
        }
        let first_radius_squared = self.data.first_semicircle.radial_distance()
            * self.data.first_semicircle.radial_distance();
        let second_radius_squared = self.data.second_semicircle.radial_distance()
            * self.data.second_semicircle.radial_distance();
        let Some((common_denominator, dx, dy, q, line, discriminant)) = (|| {
            let common_denominator = first.denominator.multiply(&second.denominator)?;
            let dx = second
                .x
                .multiply(&first.denominator)?
                .subtract(&first.x.multiply(&second.denominator)?)?;
            let dy = second
                .y
                .multiply(&first.denominator)?
                .subtract(&first.y.multiply(&second.denominator)?)?;
            let q = dx.multiply(&dx)?.add(&dy.multiply(&dy)?)?;
            let denominator_squared = common_denominator.square()?;
            let line = q.add(
                &denominator_squared.scale(&(&first_radius_squared - &second_radius_squared))?,
            )?;
            let discriminant = q
                .multiply(&denominator_squared)?
                .scale(&(Real::from(4_i8) * &first_radius_squared))?
                .subtract(&line.square()?)?;
            Some((common_denominator, dx, dy, q, line, discriminant))
        })() else {
            return Ok(Classification::Decided(None));
        };
        let (field, signed_root) = if contact.branch == 0 {
            let Some(zero) = parent.constant(Real::zero()) else {
                return Ok(Classification::Decided(None));
            };
            (parent.clone(), zero)
        } else {
            let Some(field) = parent.extension(discriminant) else {
                return Ok(Classification::Decided(None));
            };
            let Some(root) = field.element(
                parent.constant(Real::zero()).ok_or_else(|| {
                    CurveError::Topology(
                        "a recursive pair discriminant lost its zero coefficient".into(),
                    )
                })?,
                parent.constant(Real::from(contact.branch)).ok_or_else(|| {
                    CurveError::Topology(
                        "a recursive pair discriminant lost its branch coefficient".into(),
                    )
                })?,
            ) else {
                return Ok(Classification::Decided(None));
            };
            (field, root)
        };
        let Some((first, second, common_denominator, dx, dy, q, line)) = (|| {
            Some((
                first.lifted_to(&field)?,
                second.lifted_to(&field)?,
                field.lift(&common_denominator)?,
                field.lift(&dx)?,
                field.lift(&dy)?,
                field.lift(&q)?,
                field.lift(&line)?,
            ))
        })() else {
            return Ok(Classification::Decided(None));
        };
        let Some((point_x, point_y, denominator)) = (|| {
            let first_x = q
                .multiply(&second.denominator)?
                .multiply(&first.x)?
                .scale(&Real::from(2_i8))?;
            let first_y = q
                .multiply(&second.denominator)?
                .multiply(&first.y)?
                .scale(&Real::from(2_i8))?;
            let point_x = first_x
                .add(&line.multiply(&dx)?)?
                .subtract(&signed_root.multiply(&dy)?)?;
            let point_y = first_y
                .add(&line.multiply(&dy)?)?
                .add(&signed_root.multiply(&dx)?)?;
            let denominator = q.multiply(&common_denominator)?.scale(&Real::from(2_i8))?;
            Some((point_x, point_y, denominator))
        })() else {
            return Ok(Classification::Decided(None));
        };
        Ok(Classification::Decided(Some(
            BezierRecursiveQuadraticPairContactFrame2 {
                field,
                point: BezierRecursiveQuadraticProjectivePoint2 {
                    x: point_x,
                    y: point_y,
                    denominator,
                },
                centers: [first, second],
            },
        )))
    }

    /// Imports an arbitrary retained circle-pair contact and either support
    /// center into one exact recursive coefficient field. The native
    /// center/discriminant construction is authoritative; independently
    /// represented Cartesian roots remain a complete cold fallback for a
    /// center family that cannot yet expose recursive projective evidence.
    pub(in crate::bezier_offset) fn recursive_represented_contact_frame(
        &self,
        contact: &BezierAlgebraicCuspSemicirclePairContact2,
        first: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierRecursiveQuadraticChordContactFrame2>>> {
        if !policy.accepts_retained_policy(self.data.policy) {
            return Err(CurveError::Topology(
                "a represented pair contact entered recursive authority under a different policy"
                    .into(),
            ));
        }
        let selected = |frame: &BezierRecursiveQuadraticPairContactFrame2| {
            BezierRecursiveQuadraticChordContactFrame2 {
                field: frame.field.clone(),
                point: frame.point.clone(),
                center: frame.centers[usize::from(!first)].clone(),
            }
        };
        if let Some(contact_data) = self.recursive_contact_data(contact) {
            return Ok(Classification::Decided(Some(selected(&contact_data.frame))));
        }
        let Some((system, contact_data)) = self.represented_contact_data(contact) else {
            return Err(CurveError::Topology(
                "a circle-pair contact lost its recursive branch data".into(),
            ));
        };
        if let Some(frame) = contact_data.recursive_contact_frame.get() {
            return Ok(Classification::Decided(Some(selected(frame))));
        }
        let mut deferred_reason = None;
        let frame = match self.recursive_geometric_pair_contact_frame(contact, policy)? {
            Classification::Decided(Some(frame)) => Some(frame),
            Classification::Decided(None) => None,
            Classification::Uncertain(reason) => {
                deferred_reason = Some(reason);
                None
            }
        };
        let frame = if let Some(frame) = frame {
            frame
        } else {
            let represented = [
                contact_data.point[0].clone(),
                contact_data.point[1].clone(),
                system.first_center[0].clone(),
                system.first_center[1].clone(),
                system.second_center[0].clone(),
                system.second_center[1].clone(),
            ];
            let Some((sources, coordinates)) = represented_affine_tensor_basis(&represented) else {
                return Ok(deferred_reason
                    .map_or(Classification::Decided(None), Classification::Uncertain));
            };
            let [point_x, point_y, first_x, first_y, second_x, second_y]: [DenseTensorPolynomial;
                6] = coordinates
                .try_into()
                .expect("a represented pair frame retains six coordinates");
            let remove_output_axis = |polynomial: DenseTensorPolynomial| {
                polynomial.remove_certified_independent_axis(
                    sources.len(),
                    hypersolve::PredicatePolicy::MAX_REFINEMENT_PRECISION,
                )
            };
            let Some((point_x, point_y, first_x, first_y, second_x, second_y, one)) = (|| {
                Some((
                    remove_output_axis(point_x)?,
                    remove_output_axis(point_y)?,
                    remove_output_axis(first_x)?,
                    remove_output_axis(first_y)?,
                    remove_output_axis(second_x)?,
                    remove_output_axis(second_y)?,
                    DenseTensorPolynomial::try_new(vec![1; sources.len()], vec![Real::one()])?,
                ))
            })(
            ) else {
                return Ok(deferred_reason
                    .map_or(Classification::Decided(None), Classification::Uncertain));
            };
            let Some(field) = RecursiveQuadraticField::base(sources, one.clone(), one.clone())
            else {
                return Ok(deferred_reason
                    .map_or(Classification::Decided(None), Classification::Uncertain));
            };
            let RecursiveQuadraticField::Base(base) = &field else {
                unreachable!("a represented pair contact begins in its dense base field")
            };
            let Some((point_x, point_y, first_x, first_y, second_x, second_y, denominator)) =
                (|| {
                    Some((
                        recursive_quadratic_rational_value(base, point_x)?,
                        recursive_quadratic_rational_value(base, point_y)?,
                        recursive_quadratic_rational_value(base, first_x)?,
                        recursive_quadratic_rational_value(base, first_y)?,
                        recursive_quadratic_rational_value(base, second_x)?,
                        recursive_quadratic_rational_value(base, second_y)?,
                        recursive_quadratic_rational_value(base, one)?,
                    ))
                })()
            else {
                return Ok(deferred_reason
                    .map_or(Classification::Decided(None), Classification::Uncertain));
            };
            BezierRecursiveQuadraticPairContactFrame2 {
                field,
                point: BezierRecursiveQuadraticProjectivePoint2 {
                    x: point_x,
                    y: point_y,
                    denominator: denominator.clone(),
                },
                centers: [
                    BezierRecursiveQuadraticProjectivePoint2 {
                        x: first_x,
                        y: first_y,
                        denominator: denominator.clone(),
                    },
                    BezierRecursiveQuadraticProjectivePoint2 {
                        x: second_x,
                        y: second_y,
                        denominator,
                    },
                ],
            }
        };
        let _ = contact_data.recursive_contact_frame.set(frame);
        let frame = contact_data
            .recursive_contact_frame
            .get()
            .expect("a recursive pair contact publishes its retained frame");
        Ok(Classification::Decided(Some(selected(frame))))
    }

    /// Returns an exact source-parameter identity retained by a structurally
    /// authored tangent. The Boolean/corner kernels may intersect a
    /// concentric offset of a recursive circle with one of the circles that
    /// authored its center. On that parent side the new circle-pair map does
    /// not define another algebraic number: it deliberately stores the
    /// already-authoritative source parameter, optionally through the unit
    /// complement. Exposing that relation keeps subsequent finite-fragment
    /// ordering out of duplicate interval refinement.
    pub(in crate::bezier_offset) fn retained_contact_parameter_for_side(
        &self,
        contact: &BezierAlgebraicCuspSemicirclePairContact2,
        first: bool,
    ) -> Option<(&BezierAlgebraicCuspSemicircleParameter2, bool)> {
        let (_, data) = self.represented_contact_data(contact)?;
        let parameter = if first {
            &data.first_parameter
        } else {
            &data.second_parameter
        };
        match parameter {
            BezierRepresentedCircleContactParameterData2::Retained {
                parameter,
                unit_complement,
            } => Some((parameter, *unit_complement)),
            BezierRepresentedCircleContactParameterData2::Materialized(_)
            | BezierRepresentedCircleContactParameterData2::ExactContactRadial(_)
            | BezierRepresentedCircleContactParameterData2::AuthoredPairAngular { .. } => None,
        }
    }

    /// Signs an exact linear combination of the two participating carrier
    /// tangents at one retained pair contact.
    ///
    /// The represented map stores exact oriented tangent cross and dot values,
    /// so later topology can combine them without reconstructing either
    /// contact coordinate or replaying the circle-pair solve.
    pub(crate) fn tangent_cross_dot_linear_combination_sign(
        &self,
        contact: &BezierAlgebraicCuspSemicirclePairContact2,
        cross_scale: &Real,
        dot_scale: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        if !policy.accepts_retained_policy(self.data.policy) {
            return Err(CurveError::Topology(
                "cusp-pair tangent predicate was replayed under a different policy".into(),
            ));
        }
        if !(-1..=1).contains(&contact.branch) {
            return Err(CurveError::Topology(
                "cusp-pair contact retained an invalid support branch".into(),
            ));
        }
        if let Some(data) = self.recursive_contact_data(contact) {
            if let Some(tangent_dot_sign) = data.tangent_dot_sign {
                let Some(dot_scale_sign) = real_sign(dot_scale, policy) else {
                    return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
                };
                return Ok(Classification::Decided(product_sign(
                    tangent_dot_sign,
                    dot_scale_sign,
                )));
            }
            let expression = data
                .tangent_cross
                .scale(cross_scale)
                .and_then(|cross| cross.add(&data.tangent_dot.scale(dot_scale)?))
                .ok_or_else(|| {
                    CurveError::Topology(
                        "a recursive circle-pair tangent predicate exceeded its field budget"
                            .into(),
                    )
                })?;
            return expression.sign(policy);
        }
        let Some((_, data)) = self.represented_contact_data(contact) else {
            return Err(CurveError::Topology(
                "a circle-pair contact lost its tangent branch data".into(),
            ));
        };
        Ok(
            match represented_affine_coordinate(
                &[
                    (&data.tangent_cross, cross_scale),
                    (&data.tangent_dot, dot_scale),
                ],
                &Real::zero(),
            ) {
                Classification::Decided(value) => represented_policy_sign(&value, policy),
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
        )
    }

    pub(crate) fn tangent_dot_sign(
        &self,
        contact: &BezierAlgebraicCuspSemicirclePairContact2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        self.tangent_cross_dot_linear_combination_sign(contact, &Real::zero(), &Real::one(), policy)
    }
    /// Materializes one recursively authored circle contact as two exact
    /// represented algebraic coordinates. Construction is deliberately
    /// STRICT even when the enclosing operation permits terminal
    /// APPROXIMATE_512 equality: the operation policy may decide a predicate,
    /// but it may never select an algebraic sheet.
    pub(in crate::bezier_offset) fn represented_selected_radial_contact_point(
        &self,
        contact: &BezierAlgebraicCuspSemicirclePairContact2,
    ) -> CurveResult<Classification<[AlgebraicRootRepresentation; 2]>> {
        self.represented_selected_radial_derived_point(
            contact,
            true,
            &Real::one(),
            &Real::zero(),
            &Real::zero(),
            &Real::zero(),
            &self.data.policy,
        )
    }

    /// Materializes `C + a(P-C) + b*J(P-C) + T` for one retained pair
    /// contact `P` and either participating support center `C`. Every selected
    /// root and nested radical remains correlated until the final coordinate
    /// norm, after which a STRICT interval selects the authored real root.
    #[allow(clippy::too_many_arguments)]
    pub(in crate::bezier_offset) fn represented_selected_radial_derived_point(
        &self,
        contact: &BezierAlgebraicCuspSemicirclePairContact2,
        first: bool,
        radial_scale: &Real,
        perpendicular_scale: &Real,
        translation_x: &Real,
        translation_y: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<[AlgebraicRootRepresentation; 2]>> {
        self.represented_selected_radial_affine(
            contact,
            first,
            &Real::one(),
            radial_scale,
            perpendicular_scale,
            translation_x,
            translation_y,
            policy,
        )
    }

    pub(in crate::bezier_offset) fn represented_selected_radial_vector(
        &self,
        contact: &BezierAlgebraicCuspSemicirclePairContact2,
        first: bool,
        radial_scale: &Real,
        perpendicular_scale: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<[AlgebraicRootRepresentation; 2]>> {
        self.represented_selected_radial_affine(
            contact,
            first,
            &Real::zero(),
            radial_scale,
            perpendicular_scale,
            &Real::zero(),
            &Real::zero(),
            policy,
        )
    }

    #[allow(clippy::too_many_arguments)]
    pub(in crate::bezier_offset) fn represented_selected_radial_affine(
        &self,
        contact: &BezierAlgebraicCuspSemicirclePairContact2,
        first: bool,
        center_bias: &Real,
        radial_scale: &Real,
        perpendicular_scale: &Real,
        translation_x: &Real,
        translation_y: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<[AlgebraicRootRepresentation; 2]>> {
        if !policy.accepts_retained_policy(self.data.policy) {
            return Err(CurveError::Topology(
                "a represented selected-radial point crossed predicate policies".into(),
            ));
        }
        if !(-1..=1).contains(&contact.branch) {
            return Err(CurveError::Topology(
                "a represented selected-radial contact retained an invalid branch".into(),
            ));
        }
        if let Some((system, data)) = self.represented_contact_data(contact) {
            let center = if first {
                &system.first_center
            } else {
                &system.second_center
            };
            let parameter = if first {
                &data.first_parameter
            } else {
                &data.second_parameter
            };
            if let BezierRepresentedCircleContactParameterData2::ExactContactRadial(radial) =
                parameter
            {
                let delta_x = radial_scale * &radial[0] - perpendicular_scale * &radial[1];
                let delta_y = perpendicular_scale * &radial[0] + radial_scale * &radial[1];
                let x = represented_affine_coordinate(
                    &[(&center[0], center_bias)],
                    &(translation_x + delta_x),
                );
                let y = represented_affine_coordinate(
                    &[(&center[1], center_bias)],
                    &(translation_y + delta_y),
                );
                return Ok(match (x, y) {
                    (Classification::Decided(x), Classification::Decided(y)) => {
                        Classification::Decided([x, y])
                    }
                    (Classification::Uncertain(UncertaintyReason::Unsupported), _)
                    | (_, Classification::Uncertain(UncertaintyReason::Unsupported)) => {
                        Classification::Uncertain(UncertaintyReason::Unsupported)
                    }
                    _ => Classification::Uncertain(UncertaintyReason::Predicate),
                });
            }
            let center_scale = center_bias - radial_scale;
            let negative_perpendicular = -perpendicular_scale.clone();
            let x = represented_affine_coordinate(
                &[
                    (&data.point[0], radial_scale),
                    (&data.point[1], &negative_perpendicular),
                    (&center[0], &center_scale),
                    (&center[1], perpendicular_scale),
                ],
                translation_x,
            );
            let y = represented_affine_coordinate(
                &[
                    (&data.point[0], perpendicular_scale),
                    (&data.point[1], radial_scale),
                    (&center[0], &negative_perpendicular),
                    (&center[1], &center_scale),
                ],
                translation_y,
            );
            return Ok(match (x, y) {
                (Classification::Decided(x), Classification::Decided(y)) => {
                    Classification::Decided([x, y])
                }
                (Classification::Uncertain(UncertaintyReason::Unsupported), _)
                | (_, Classification::Uncertain(UncertaintyReason::Unsupported)) => {
                    Classification::Uncertain(UncertaintyReason::Unsupported)
                }
                _ => Classification::Uncertain(UncertaintyReason::Predicate),
            });
        }
        if self.recursive_contact_data(contact).is_some() {
            let frame = match self.recursive_represented_contact_frame(contact, first, policy)? {
                Classification::Decided(Some(frame)) => frame,
                Classification::Decided(None) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let point = &frame.point;
            let center = &frame.center;
            let center_scale = center_bias - radial_scale;
            let negative_perpendicular = -perpendicular_scale.clone();
            let Some((common_denominator, x, y)) = (|| {
                let common_denominator = point.denominator.multiply(&center.denominator)?;
                let translated = |translation: &Real| common_denominator.scale(translation);
                let x = point
                    .x
                    .scale(radial_scale)?
                    .add(&point.y.scale(&negative_perpendicular)?)?
                    .multiply(&center.denominator)?
                    .add(
                        &center
                            .x
                            .scale(&center_scale)?
                            .add(&center.y.scale(perpendicular_scale)?)?
                            .multiply(&point.denominator)?,
                    )?
                    .add(&translated(translation_x)?)?;
                let y = point
                    .x
                    .scale(perpendicular_scale)?
                    .add(&point.y.scale(radial_scale)?)?
                    .multiply(&center.denominator)?
                    .add(
                        &center
                            .x
                            .scale(&negative_perpendicular)?
                            .add(&center.y.scale(&center_scale)?)?
                            .multiply(&point.denominator)?,
                    )?
                    .add(&translated(translation_y)?)?;
                Some((common_denominator, x, y))
            })() else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            let x = BezierRecursiveQuadraticProjectiveScalar2 {
                numerator: x,
                denominator: common_denominator.clone(),
            }
            .represented_value(policy)?;
            let y = BezierRecursiveQuadraticProjectiveScalar2 {
                numerator: y,
                denominator: common_denominator,
            }
            .represented_value(policy)?;
            return Ok(match (x, y) {
                (Classification::Decided(x), Classification::Decided(y)) => {
                    Classification::Decided([x, y])
                }
                (Classification::Uncertain(UncertaintyReason::Unsupported), _)
                | (_, Classification::Uncertain(UncertaintyReason::Unsupported)) => {
                    Classification::Uncertain(UncertaintyReason::Unsupported)
                }
                (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
                    Classification::Uncertain(reason)
                }
            });
        }
        Err(CurveError::Topology(
            "a circle-pair map lost its retained contact data".into(),
        ))
    }

    pub(in crate::bezier_offset) fn opposite_contact_carrier(
        &self,
        contact: &BezierAlgebraicCuspSemicirclePairContact2,
        first: bool,
    ) -> (
        &BezierAlgebraicCuspSemicircle2,
        BezierAlgebraicCuspSemicircleContactLocation2,
    ) {
        if first {
            (&self.data.second_semicircle, contact.second_location)
        } else {
            (&self.data.first_semicircle, contact.first_location)
        }
    }

    /// Replays one retained circle-circle contact against a rational carrier
    /// of either participating circle.
    ///
    /// Intersecting the opposite semicircle with the rational carrier yields
    /// the same physical contact without combining the two selected cusp-root
    /// fields. Endpoint location and oriented tangent cross sign distinguish
    /// the retained support branch exactly; the caller subsequently restricts
    /// the candidates to one published regular overlap cell.
    pub(in crate::bezier_offset) fn rational_parameters_for_contact(
        &self,
        contact: &BezierAlgebraicCuspSemicirclePairContact2,
        first: bool,
        target: &RationalBezier2,
        range: &CurveParameterRange2,
        target_reversed_from_pair_carrier: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<BezierParameter2>>> {
        if !policy.accepts_retained_policy(self.data.policy) {
            return Err(CurveError::Topology(
                "cusp-pair parameter map was replayed under a different predicate policy".into(),
            ));
        }
        let (opposite, expected_location) = self.opposite_contact_carrier(contact, first);
        let tangent_factor = if first == target_reversed_from_pair_carrier {
            RealSign::Positive
        } else {
            RealSign::Negative
        };
        let expected_tangent = product_sign(contact.tangent_cross_sign, tangent_factor);
        let intersections = match opposite.rational_intersections(target, range, policy)? {
            Classification::Decided(intersections) => intersections,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let contacts = match intersections {
            BezierAlgebraicCuspSemicircleRationalIntersections2::Mapped { contacts, overlaps }
                if overlaps.is_empty() =>
            {
                contacts
            }
            other => return match other {
                BezierAlgebraicCuspSemicircleRationalIntersections2::DegenerateProjection => {
                    Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
                }
                BezierAlgebraicCuspSemicircleRationalIntersections2::Mapped { .. } => {
                    Err(CurveError::Topology(
                        "a transverse cusp-pair contact replayed as a coincident rational circle"
                            .into(),
                    ))
                }
                BezierAlgebraicCuspSemicircleRationalIntersections2::SelectedFiber { .. } => {
                    Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
                }
            },
        };
        let mut parameters = Vec::with_capacity(contacts.len());
        for candidate in contacts {
            if candidate.location != expected_location
                || candidate.tangent_cross_sign != expected_tangent
            {
                continue;
            }
            match promote_curve_region_bezier_parameter(&candidate.other_parameter, policy)? {
                Classification::Decided(parameter) => parameters.push(parameter),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        Ok(Classification::Decided(parameters))
    }

    /// Replays one direct pair contact on a caller-supplied rational carrier
    /// of either participating circle and publishes its narrow one-field point
    /// evidence. This is an output-representation bridge only: the pair kernel
    /// remains the contact and branch authority.
    pub(crate) fn rational_point_evidence_for_contact(
        &self,
        contact: &BezierAlgebraicCuspSemicirclePairContact2,
        first: bool,
        target: &RationalBezier2,
        target_reversed_from_pair_carrier: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<CurvePoint2>>> {
        let parameters = match self.rational_parameters_for_contact(
            contact,
            first,
            target,
            &CurveParameterRange2::unit(),
            target_reversed_from_pair_carrier,
            policy,
        )? {
            Classification::Decided(parameters) => parameters,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let mut points = Vec::with_capacity(parameters.len());
        for parameter in parameters {
            match rational_point_evidence_at_parameter(target, &parameter, policy)? {
                Classification::Decided(point) => points.push(point),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        Ok(Classification::Decided(points))
    }

    /// Replays the same pair contact against a genuinely analytic carrier.
    pub(in crate::bezier_offset) fn parallel_parameters_for_contact(
        &self,
        contact: &BezierAlgebraicCuspSemicirclePairContact2,
        first: bool,
        target: &BezierParallel2,
        range: &CurveParameterRange2,
        target_reversed_from_pair_carrier: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<BezierParameter2>>> {
        if !policy.accepts_retained_policy(self.data.policy) {
            return Err(CurveError::Topology(
                "cusp-pair parameter map was replayed under a different predicate policy".into(),
            ));
        }
        let (opposite, expected_location) = self.opposite_contact_carrier(contact, first);
        let tangent_factor = if first == target_reversed_from_pair_carrier {
            RealSign::Positive
        } else {
            RealSign::Negative
        };
        let expected_tangent = product_sign(contact.tangent_cross_sign, tangent_factor);
        let intersections = match opposite.parallel_intersections(target, range, None, policy)? {
            Classification::Decided(intersections) => intersections,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let contacts = match intersections {
            BezierAlgebraicCuspSemicircleParallelIntersections2::Mapped { contacts, overlaps }
                if overlaps.is_empty() =>
            {
                contacts
            }
            other => return match other {
                BezierAlgebraicCuspSemicircleParallelIntersections2::DegenerateProjection => {
                    Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
                }
                BezierAlgebraicCuspSemicircleParallelIntersections2::Mapped { .. } => {
                    Err(CurveError::Topology(
                        "a transverse cusp-pair contact replayed as a coincident analytic circle"
                            .into(),
                    ))
                }
                BezierAlgebraicCuspSemicircleParallelIntersections2::CoincidentCircleComponent => {
                    Err(CurveError::Topology(
                        "a transverse cusp-pair contact replayed on a coincident incident circle"
                            .into(),
                    ))
                }
                BezierAlgebraicCuspSemicircleParallelIntersections2::SelectedFiber { .. }
                | BezierAlgebraicCuspSemicircleParallelIntersections2::RetainedContacts(_) => {
                    Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
                }
            },
        };
        Ok(Classification::Decided(
            contacts
                .into_iter()
                .filter(|candidate| {
                    candidate.location == expected_location
                        && candidate.tangent_cross_sign == Some(expected_tangent)
                })
                .map(|candidate| candidate.parallel_parameter)
                .collect(),
        ))
    }

    pub(in crate::bezier_offset) fn represented_contact_order_to_real_for_side(
        &self,
        contact: &BezierAlgebraicCuspSemicirclePairContact2,
        first: bool,
        parameter: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        let (location, semicircle) = if first {
            (contact.first_location, &self.data.first_semicircle)
        } else {
            (contact.second_location, &self.data.second_semicircle)
        };
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
        if let Some(data) = self.recursive_contact_data(contact) {
            let angular = &data.angular[usize::from(!first)];
            let one_minus = Real::one() - parameter;
            let parameter_denominator = &one_minus * &one_minus + parameter * parameter;
            let radial_coefficient = Real::one() - Real::from(2_i8) * parameter;
            let predicate = angular
                .diameter
                .scale(&parameter_denominator)
                .and_then(|diameter| {
                    angular
                        .radius_squared_denominator
                        .scale(&radial_coefficient)
                        .and_then(|radius| diameter.subtract(&radius))
                })
                .ok_or_else(|| {
                    CurveError::Topology(
                        "a recursive circle-pair angular predicate exceeded its field budget"
                            .into(),
                    )
                })?;
            return Ok(predicate.sign(policy)?.map(|sign| match sign {
                RealSign::Positive => std::cmp::Ordering::Less,
                RealSign::Zero => std::cmp::Ordering::Equal,
                RealSign::Negative => std::cmp::Ordering::Greater,
            }));
        }
        let Some((_, data)) = self.represented_contact_data(contact) else {
            return Err(CurveError::Topology(
                "a circle-pair parameter lost its branch data".into(),
            ));
        };
        let contact_parameter = if first {
            &data.first_parameter
        } else {
            &data.second_parameter
        };
        match contact_parameter {
            BezierRepresentedCircleContactParameterData2::Materialized(contact_parameter) => {
                contact_parameter
                    .cmp_by_refinement(&BezierParameter2::Exact(parameter.clone()), policy)
            }
            BezierRepresentedCircleContactParameterData2::Retained {
                parameter: contact_parameter,
                unit_complement,
            } => {
                let comparison = contact_parameter.cmp_by_refinement(
                    &BezierAlgebraicCuspSemicircleParameter2::Exact(if *unit_complement {
                        Real::one() - parameter
                    } else {
                        parameter.clone()
                    }),
                    policy,
                )?;
                Ok(if *unit_complement {
                    comparison.map(std::cmp::Ordering::reverse)
                } else {
                    comparison
                })
            }
            BezierRepresentedCircleContactParameterData2::AuthoredPairAngular {
                map,
                contact,
                dot_scale,
                oriented_cross_scale,
                radius_squared,
            } => {
                let Some((_, source)) = map.represented_contact_data(contact) else {
                    return Err(CurveError::Topology(
                        "an authored tangent parameter lost its represented source contact".into(),
                    ));
                };
                let one_minus = Real::one() - parameter;
                let cross_scale = oriented_cross_scale * one_minus;
                let dot_scale = -(dot_scale * parameter);
                let predicate = match represented_affine_coordinate(
                    &[
                        (&source.tangent_cross, &cross_scale),
                        (&source.tangent_dot, &dot_scale),
                    ],
                    &(-parameter * radius_squared),
                ) {
                    Classification::Decided(predicate) => predicate,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                Ok(
                    represented_policy_sign(&predicate, policy).map(|sign| match sign {
                        RealSign::Negative => std::cmp::Ordering::Less,
                        RealSign::Zero => std::cmp::Ordering::Equal,
                        RealSign::Positive => std::cmp::Ordering::Greater,
                    }),
                )
            }
            BezierRepresentedCircleContactParameterData2::ExactContactRadial(contact_radial) => {
                let frame = match semicircle.represented_circle_frame(policy)? {
                    Classification::Decided(frame) => frame,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let one_minus = Real::one() - parameter;
                let negative_parameter = -parameter.clone();
                let turn = semicircle.turn_sign();
                let x_scale = &one_minus * &turn * &contact_radial[1]
                    + &negative_parameter * &contact_radial[0];
                let y_scale = -(&one_minus * &turn * &contact_radial[0])
                    + &negative_parameter * &contact_radial[1];
                let radius_squared = semicircle.radial_distance() * semicircle.radial_distance();
                let offset = -parameter * &radius_squared;
                let ordering = |sign| match sign {
                    RealSign::Negative => std::cmp::Ordering::Less,
                    RealSign::Zero => std::cmp::Ordering::Equal,
                    RealSign::Positive => std::cmp::Ordering::Greater,
                };
                if let Some(sign) =
                    represented_exact_radial_linear_sign(&frame, &x_scale, &y_scale, &offset)
                {
                    return Ok(Classification::Decided(ordering(sign)));
                }
                let [dot, cross] = match represented_circle_dot_cross_from_exact_radial(
                    &frame,
                    contact_radial,
                    &turn,
                ) {
                    Classification::Decided(values) => values,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let predicate = match represented_affine_coordinate(
                    &[(&cross, &one_minus), (&dot, &negative_parameter)],
                    &offset,
                ) {
                    Classification::Decided(predicate) => predicate,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                Ok(represented_policy_sign(&predicate, policy).map(ordering))
            }
        }
    }

    pub(crate) fn first_contact_parameter(
        &self,
        contact: &BezierAlgebraicCuspSemicirclePairContact2,
    ) -> BezierAlgebraicCuspSemicircleParameter2 {
        self.contact_parameter_for_side(contact, true)
    }

    pub(crate) fn second_contact_parameter(
        &self,
        contact: &BezierAlgebraicCuspSemicirclePairContact2,
    ) -> BezierAlgebraicCuspSemicircleParameter2 {
        self.contact_parameter_for_side(contact, false)
    }

    pub(in crate::bezier_offset) fn contact_parameter_for_side(
        &self,
        contact: &BezierAlgebraicCuspSemicirclePairContact2,
        first: bool,
    ) -> BezierAlgebraicCuspSemicircleParameter2 {
        let location = if first {
            contact.first_location
        } else {
            contact.second_location
        };
        algebraic_cusp_semicircle_endpoint_parameter(location).unwrap_or_else(|| {
            BezierAlgebraicCuspSemicircleParameter2::Mapped(Arc::new(
                BezierAlgebraicCuspSemicircleMappedParameterData2::Pair {
                    map: self.clone(),
                    contact: contact.clone(),
                    first,
                },
            ))
        })
    }
}
