//! Common rational supports are split into monotone source-abscissa charts.

use super::*;
use crate::curve::CurveSourceSpan2;

pub(super) struct Span<'a> {
    pub(super) span: &'a CurveSourceSpan2,
    pub(super) index: usize,
    pub(super) rational: &'a RationalBezier2,
    pub(super) range: &'a CurveParameterRange2,
    pub(super) envelope: &'a CurveParameterRange2,
    pub(super) section: &'a Section,
    pub(super) source_x: &'a [Real; 3],
    pub(super) reflected: bool,
}

#[derive(Clone, Debug)]
pub(crate) struct PolynomialRationalCorrespondence2 {
    branch: Curve2,
    fragment: Curve2,
    rational: RationalBezier2,
    local_range: CurveParameterRange2,
    abscissas: [CurveParameter2; 2],
    source_x: [Real; 3],
    branch_first: bool,
    first_range: CurveParameterRange2,
    second_range: CurveParameterRange2,
}

impl PolynomialRationalCorrespondence2 {
    fn local_parameter(
        &self,
        x: &CurveParameter2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Option<CurveParameter2>> {
        for (abscissa, parameter) in self
            .abscissas
            .iter()
            .zip([self.local_range.start(), self.local_range.end()])
        {
            if order(x, abscissa, policy)? == Ordering::Equal {
                return Ok(Some(parameter.clone()));
            }
        }
        let source_x = CurveParameterRange2::new_validated(
            self.abscissas[0].clone(),
            self.abscissas[1].clone(),
        );
        if !decided(
            CurveParameterDomain2::new(&source_x, None).contains_finite_parameter(x, policy),
        )? {
            return Ok(None);
        }
        // The inverse needs only x. Lifting it to (x,0) avoids embedding an
        // unrelated selected ordinate into the rational parameter solver.
        let point = x.affine_point(Real::one(), Real::zero(), Real::zero(), Real::zero())?;
        let candidates = decided(
            crate::bezier_offset::recursive_projective_point_rational_axis_parameters(
                &point,
                &self.rational,
                crate::Axis2::X,
                &self.local_range,
                policy,
            ),
        )?
        .ok_or_else(|| blocked(UncertaintyReason::Unsupported))?;
        match candidates.len() {
            0 => Ok(None),
            1 => Ok(candidates.into_iter().next()),
            _ => Err(invalid(CurveError::Topology(
                "a monotone rational abscissa has multiple inverse parameters".into(),
            ))),
        }
    }

    pub(crate) fn map_parameter(
        &self,
        parameter: &CurveParameter2,
        forward: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<CurveParameter2>>> {
        let map = || -> ExactCurveResult<Option<CurveParameter2>> {
            if forward == self.branch_first {
                if !decided(
                    CurveParameterDomain2::new(self.branch.parameter_domain(), None)
                        .contains_finite_parameter(parameter, policy),
                )? {
                    return Ok(None);
                }
                return self.local_parameter(parameter, policy);
            }
            if !decided(
                CurveParameterDomain2::new(&self.local_range, None)
                    .contains_finite_parameter(parameter, policy),
            )? {
                return Ok(None);
            }
            let point = self
                .fragment
                .point_at_with_policy(parameter, policy)?
                .into_value();
            let x = abscissa(&point, &self.source_x, policy)?;
            Ok(decided(
                CurveParameterDomain2::new(self.branch.parameter_domain(), None)
                    .contains_finite_parameter(&x, policy),
            )?
            .then_some(x))
        };
        match map() {
            Ok(value) => Ok(Classification::Decided(value)),
            Err(ExactCurveError::Invalid { cause, .. }) => Err(cause),
            Err(ExactCurveError::Blocked(blocker)) => {
                Ok(Classification::Uncertain(blocker.reason()))
            }
        }
    }

    pub(crate) fn clipped_ranges(
        &self,
        first: &CurveParameterRange2,
        second: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<(CurveParameterRange2, CurveParameterRange2)>>> {
        crate::bezier_split::clip_corresponding_parameter_ranges(
            &self.first_range,
            &self.second_range,
            first,
            second,
            policy,
            |parameter| self.map_parameter(parameter, true, policy),
            |parameter| self.map_parameter(parameter, false, policy),
        )
    }
}

fn order(
    a: &CurveParameter2,
    b: &CurveParameter2,
    policy: &CurveContext,
) -> ExactCurveResult<Ordering> {
    decided(a.cmp_by_refinement(b, policy))
}

fn abscissa(
    point: &CurvePoint2,
    form: &[Real; 3],
    policy: &CurveContext,
) -> ExactCurveResult<CurveParameter2> {
    decided(crate::bezier_offset::point_affine_form_parameter(
        point, form, policy,
    ))
}

type Sections = (
    Vec<CurveIntersectionContact2>,
    Vec<CurveIntersectionOverlap2>,
);

pub(super) fn intersect(
    branch: &Curve2,
    input: Span<'_>,
    branch_first: bool,
    policy: &CurveContext,
) -> ExactCurveResult<Option<Sections>> {
    let basis = input.rational.homogeneous_power_basis().map_err(invalid)?;
    let dx = polynomial_subtract(
        &polynomial_multiply(&polynomial_derivative(&basis.x_numerator), &basis.weight),
        &polynomial_multiply(&basis.x_numerator, &polynomial_derivative(&basis.weight)),
    );
    if polynomial_is_zero(&dx, policy)? {
        // A constant-abscissa trace needs a vertical/collapsed section contract.
        return Ok(None);
    }
    let (lower, upper) = input
        .envelope
        .scalar_endpoints()
        .expect("finite discovery envelope");
    let derivative = decided(BezierParameterPolynomial::try_new_power_basis_with_policy(
        dx, policy,
    ))?;
    let roots = decided(derivative.isolate_interval_roots(lower, upper, policy))?;
    let mut breaks = vec![input.range.start().clone()];
    for root in roots {
        let root = CurveParameter2::from(root);
        if order(&root, input.range.start(), policy)? == Ordering::Greater
            && order(&root, input.range.end(), policy)? == Ordering::Less
            && order(breaks.last().expect("start"), &root, policy)? != Ordering::Equal
        {
            breaks.push(root);
        }
    }
    breaks.push(input.range.end().clone());
    let fragment = input.span.curve();
    let mut contacts = Vec::new();
    let mut overlaps = Vec::new();
    for endpoints in breaks.windows(2) {
        let local_range =
            CurveParameterRange2::new_validated(endpoints[0].clone(), endpoints[1].clone());
        let mut source = PolynomialRationalCorrespondence2 {
            branch: branch.clone(),
            fragment: fragment.clone(),
            rational: input.rational.clone(),
            local_range,
            abscissas: [
                abscissa(
                    &fragment
                        .point_at_with_policy(&endpoints[0], policy)?
                        .into_value(),
                    input.source_x,
                    policy,
                )?,
                abscissa(
                    &fragment
                        .point_at_with_policy(&endpoints[1], policy)?
                        .into_value(),
                    input.source_x,
                    policy,
                )?,
            ],
            source_x: input.source_x.clone(),
            branch_first,
            first_range: CurveParameterRange2::unit(),
            second_range: CurveParameterRange2::unit(),
        };
        let decreasing = match order(&source.abscissas[0], &source.abscissas[1], policy)? {
            Ordering::Less => false,
            Ordering::Greater => true,
            Ordering::Equal => {
                return Err(invalid(CurveError::Topology(
                    "a nonconstant monotone rational abscissa has equal endpoints".into(),
                )));
            }
        };
        let [mut low, mut high] = source.abscissas.clone();
        if decreasing {
            std::mem::swap(&mut low, &mut high);
        }
        if order(&low, branch.parameter_domain().start(), policy)? == Ordering::Less {
            low = branch.parameter_domain().start().clone();
        }
        if order(&high, branch.parameter_domain().end(), policy)? == Ordering::Greater {
            high = branch.parameter_domain().end().clone();
        }
        let ordering = order(&low, &high, policy)?;
        if ordering == Ordering::Greater {
            continue;
        }
        let inverse = |x: &CurveParameter2| {
            source.local_parameter(x, policy)?.ok_or_else(|| {
                invalid(CurveError::Topology(
                    "a monotone common support omitted an admitted inverse".into(),
                ))
            })
        };
        let low_t = inverse(&low)?;
        let high_t = inverse(&high)?;
        if ordering == Ordering::Less {
            let sample = decided(low.strict_scalar_between_ordered(&high, policy))?;
            let t = inverse(&sample.clone().into())?;
            let point = fragment.point_at_with_policy(&t, policy)?.into_value();
            let expected = branch
                .point_at_with_policy(&sample.into(), policy)?
                .into_value();
            // The source graph slab contains no interior branch event. A
            // continuous, pole-free rational component has a constant ordered
            // fiber index throughout this connected monotone interval.
            if decided(Ok(point.same_point(&expected, policy)))? {
                let mut x_range = CurveParameterRange2::new_validated(low, high);
                let mut rational_range = CurveParameterRange2::new_validated(low_t, high_t);
                let reverse_first = if branch_first {
                    branch.source_traversal_is_reversed()
                } else {
                    decreasing ^ input.span.fragment.source_is_reversed()
                };
                if reverse_first {
                    x_range = CurveParameterRange2::new_validated(
                        x_range.end().clone(),
                        x_range.start().clone(),
                    );
                    rational_range = CurveParameterRange2::new_validated(
                        rational_range.end().clone(),
                        rational_range.start().clone(),
                    );
                }
                let (first_range, second_range) = if branch_first {
                    (x_range, rational_range)
                } else {
                    (rational_range, x_range)
                };
                source.first_range = first_range.clone();
                source.second_range = second_range.clone();
                let reversed = decreasing
                    ^ branch.source_traversal_is_reversed()
                    ^ input.span.fragment.source_is_reversed();
                overlaps.push(CurveIntersectionOverlap2 {
                    first_span_index: if branch_first { 0 } else { input.index },
                    second_span_index: if branch_first { input.index } else { 0 },
                    first_range,
                    second_range,
                    orientation: if reversed {
                        CurveOverlapOrientation2::Reversed
                    } else {
                        CurveOverlapOrientation2::Same
                    },
                    endpoint_inclusion: [true, true],
                    parameter_correspondence: CurveOverlapCorrespondence2::PolynomialRational(
                        Arc::new(source),
                    ),
                });
                continue;
            }
        }
        // A different selected branch can still meet this component at a
        // singular or boundary endpoint. A rejected interior is not disjointness.
        for (x, t) in [(low, low_t), (high, high_t)] {
            let point = fragment.point_at_with_policy(&t, policy)?.into_value();
            let expected = branch.point_at_with_policy(&x, policy)?.into_value();
            if !decided(Ok(point.same_point(&expected, policy)))? {
                continue;
            }
            let branch_location = CurveLocation2::new(
                0,
                CurveSpanRange2::from_affine_chart(&Real::one(), &Real::zero()),
                x,
            );
            let rational_location = CurveLocation2::new(input.index, input.span.chart(), t.clone());
            let (first, second) = if branch_first {
                (branch_location, rational_location)
            } else {
                (rational_location, branch_location)
            };
            let flip = branch.source_traversal_is_reversed()
                ^ input.span.fragment.source_is_reversed()
                ^ !branch_first
                ^ input.reflected;
            let cross = input
                .section
                .cross(&t, policy)?
                .map(|sign| if flip { negate(sign) } else { sign });
            append_contact(
                &mut contacts,
                CurveIntersectionContact2 {
                    first,
                    second,
                    point: expected,
                    tangent_cross_sign: cross,
                    certified_transverse: cross.is_some_and(|v| v != RealSign::Zero),
                },
                policy,
            )?;
        }
    }
    Ok(Some((contacts, overlaps)))
}
