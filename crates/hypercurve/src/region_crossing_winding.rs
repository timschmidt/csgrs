//! Retained winding transitions for strict line crossings.
//!
//! A complete, unique set of proper line crossings can carry more topology
//! than split markers alone: crossing an oriented opposite edge changes its
//! exact winding number by the sign of the two traversal directions. This
//! module validates that narrow proof and maps it back onto materialized
//! contour fragments. Any endpoint, tangent, arc, overlap, duplicate,
//! unresolved ordering, or non-closing transition set rejects the proof.

use hyperreal::{Real, RealSign};

use crate::classify::{compare_reals, real_sign};
use crate::events::CertifiedLineCrossingEvent;
use crate::{
    ContourIntersection, ContourOperand, CurveContext, IntersectionKind, Point2, RegionContourKey,
    RegionContourRole, RegionIntersectionSet, RegionSide, RegionView2, Segment2, SegmentKind,
};

#[derive(Clone, Debug)]
enum RegionLineCrossingParameter<'a> {
    Materialized(&'a Real),
    Certified {
        event_index: usize,
        operand: ContourOperand,
    },
}

#[derive(Clone, Debug)]
enum RegionLineCrossingPoint<'a> {
    Materialized(&'a Point2),
    Certified(usize),
}

#[derive(Clone, Debug)]
pub(crate) struct RegionLineCrossing<'a> {
    pub(crate) segment_index: usize,
    parameter: RegionLineCrossingParameter<'a>,
    point: RegionLineCrossingPoint<'a>,
    pub(crate) winding_delta: i32,
}

#[derive(Clone, Debug, Default)]
pub(crate) struct RegionLineCrossingWindingIndex<'a> {
    first: Vec<RegionLineCrossing<'a>>,
    second: Vec<RegionLineCrossing<'a>>,
    first_segment_offsets: Vec<usize>,
    second_segment_offsets: Vec<usize>,
    certified_line_crossings: Option<&'a [CertifiedLineCrossingEvent]>,
}

impl<'a> RegionLineCrossingWindingIndex<'a> {
    pub(crate) fn event_set_may_support_propagation(intersections: &RegionIntersectionSet) -> bool {
        intersections.point_event_count() != 0
    }

    pub(crate) fn from_intersections(
        first: &RegionView2<'_>,
        second: &RegionView2<'_>,
        intersections: &'a RegionIntersectionSet,
        policy: &CurveContext,
    ) -> Option<Self> {
        if first.material_contours().len() != 1
            || second.material_contours().len() != 1
            || !first.hole_contours().is_empty()
            || !second.hole_contours().is_empty()
            || intersections.pairs().len() != 1
        {
            return None;
        }

        let pair = &intersections.pairs()[0];
        if pair.first() != RegionContourKey::new(RegionSide::First, RegionContourRole::Material, 0)
            || pair.second()
                != RegionContourKey::new(RegionSide::Second, RegionContourRole::Material, 0)
            || pair.intersections().is_empty()
        {
            return None;
        }

        let first_contour = first.material_contours()[0];
        let second_contour = second.material_contours()[0];
        let crossing_capacity = pair.intersections().len();
        let certified_line_crossings = pair
            .intersections()
            .retained_certified_line_crossings()
            .map(|crossings| crossings.as_slice());
        let mut index = Self {
            first: Vec::with_capacity(crossing_capacity),
            second: Vec::with_capacity(crossing_capacity),
            first_segment_offsets: Vec::new(),
            second_segment_offsets: Vec::new(),
            certified_line_crossings,
        };
        let crossing_delta = |event_index: usize,
                              a_segment_index: usize,
                              b_segment_index: usize| {
            if let Some(delta) = pair
                .intersections()
                .certified_line_crossing_delta(event_index)
            {
                return Some(delta);
            }
            let Segment2::Line(first_line) = first_contour.segments().get(a_segment_index)? else {
                return None;
            };
            let Segment2::Line(second_line) = second_contour.segments().get(b_segment_index)?
            else {
                return None;
            };
            if let Some(delta) =
                crate::intersect::certified_line_crossing_winding_delta(first_line, second_line)
            {
                return Some(delta);
            }
            let (first_dx, first_dy) = first_line.delta();
            let (second_dx, second_dy) = second_line.delta();
            let determinant = Real::diff_of_products(&second_dx, &first_dy, &second_dy, &first_dx);
            match real_sign(&determinant, policy) {
                Some(RealSign::Positive) => Some(1),
                Some(RealSign::Negative) => Some(-1),
                Some(RealSign::Zero) | None => None,
            }
        };
        let mut crossing_count = 0_usize;
        if let Some(crossings) = index.certified_line_crossings {
            for (event_index, event) in crossings.iter().enumerate() {
                let a_segment_index = usize::from(event.a_segment_index);
                let b_segment_index = usize::from(event.b_segment_index);
                let first_delta = crossing_delta(event_index, a_segment_index, b_segment_index)?;
                index.first.push(RegionLineCrossing {
                    segment_index: a_segment_index,
                    parameter: RegionLineCrossingParameter::Certified {
                        event_index,
                        operand: ContourOperand::First,
                    },
                    point: RegionLineCrossingPoint::Certified(event_index),
                    winding_delta: first_delta,
                });
                index.second.push(RegionLineCrossing {
                    segment_index: b_segment_index,
                    parameter: RegionLineCrossingParameter::Certified {
                        event_index,
                        operand: ContourOperand::Second,
                    },
                    point: RegionLineCrossingPoint::Certified(event_index),
                    winding_delta: -first_delta,
                });
                crossing_count += 1;
            }
        } else {
            for (event_index, event) in pair.intersections().events().iter().enumerate() {
                let ContourIntersection::Point(point) = event else {
                    return None;
                };
                if point.kind != IntersectionKind::Crossing
                    || point.a_segment_kind != SegmentKind::Line
                    || point.b_segment_kind != SegmentKind::Line
                {
                    return None;
                }

                let first_delta =
                    crossing_delta(event_index, point.a_segment_index, point.b_segment_index)?;
                index.first.push(RegionLineCrossing {
                    segment_index: point.a_segment_index,
                    parameter: RegionLineCrossingParameter::Materialized(&point.a_param),
                    point: RegionLineCrossingPoint::Materialized(&point.point),
                    winding_delta: first_delta,
                });
                index.second.push(RegionLineCrossing {
                    segment_index: point.b_segment_index,
                    parameter: RegionLineCrossingParameter::Materialized(&point.b_param),
                    point: RegionLineCrossingPoint::Materialized(&point.point),
                    winding_delta: -first_delta,
                });
                crossing_count += 1;
            }
        }

        // The fixed-product comparator wins once sorting dominates, while the
        // smaller ordinary comparator preserves instruction locality below here.
        const MIN_NORMALIZED_PRODUCT_CROSSINGS: usize = 1_024;
        if index.certified_line_crossings.is_some()
            && crossing_count >= MIN_NORMALIZED_PRODUCT_CROSSINGS
        {
            index.first_segment_offsets = sort_segment_groups_and_validate_unique_normalized(
                &mut index.first,
                index.certified_line_crossings,
                policy,
                first_contour.len(),
            )?;
            index.second_segment_offsets = sort_segment_groups_and_validate_unique_normalized(
                &mut index.second,
                index.certified_line_crossings,
                policy,
                second_contour.len(),
            )?;
        } else {
            if !sort_and_validate_unique(&mut index.first, index.certified_line_crossings, policy)
                || !sort_and_validate_unique(
                    &mut index.second,
                    index.certified_line_crossings,
                    policy,
                )
            {
                return None;
            }
            index.first_segment_offsets =
                segment_crossing_offsets(&index.first, first_contour.len())?;
            index.second_segment_offsets =
                segment_crossing_offsets(&index.second, second_contour.len())?;
        }

        (crossing_count != 0
            && index.crossing_count(pair.first()) == crossing_count
            && index.crossing_count(pair.second()) == crossing_count
            && index.winding_delta_sum(pair.first()) == 0
            && index.winding_delta_sum(pair.second()) == 0)
            .then_some(index)
    }

    pub(crate) fn crossing_count(&self, key: RegionContourKey) -> usize {
        self.crossings_for_key(key)
            .map_or(0, <[RegionLineCrossing]>::len)
    }

    fn winding_delta_sum(&self, key: RegionContourKey) -> i64 {
        self.crossings_for_key(key).map_or(0, |crossings| {
            crossings
                .iter()
                .map(|crossing| i64::from(crossing.winding_delta))
                .sum()
        })
    }

    pub(crate) fn delta_for_next_fragment(
        &self,
        key: RegionContourKey,
        previous_segment_index: usize,
        current_segment_index: usize,
        segment_transition_index: &mut usize,
    ) -> Option<i32> {
        if previous_segment_index != current_segment_index {
            *segment_transition_index = 0;
            return Some(0);
        }
        // Compact fragments retain source order, and every same-segment
        // boundary corresponds one-to-one with the next certified crossing.
        let crossing = self
            .crossings_for_segment(key, previous_segment_index)?
            .get(*segment_transition_index)?;
        *segment_transition_index += 1;
        Some(crossing.winding_delta)
    }

    fn crossings_for_key(&self, key: RegionContourKey) -> Option<&[RegionLineCrossing<'a>]> {
        if key.role != RegionContourRole::Material || key.index != 0 {
            return None;
        }
        Some(match key.side {
            RegionSide::First => &self.first,
            RegionSide::Second => &self.second,
        })
    }

    pub(crate) fn crossings_for_segment(
        &self,
        key: RegionContourKey,
        segment_index: usize,
    ) -> Option<&[RegionLineCrossing<'a>]> {
        let crossings = self.crossings_for_key(key)?;
        let offsets = match key.side {
            RegionSide::First => &self.first_segment_offsets,
            RegionSide::Second => &self.second_segment_offsets,
        };
        let start = *offsets.get(segment_index)?;
        let end = *offsets.get(segment_index + 1)?;
        Some(&crossings[start..end])
    }

    pub(crate) fn materialized_parameter(
        &self,
        crossing: &RegionLineCrossing<'a>,
    ) -> Option<&'a Real> {
        match crossing.parameter {
            RegionLineCrossingParameter::Materialized(parameter) => Some(parameter),
            RegionLineCrossingParameter::Certified {
                event_index,
                operand,
            } => self
                .certified_line_crossings
                .expect("certified parameter references retained crossings")[event_index]
                .materialized_parameter(operand),
        }
    }

    pub(crate) fn point<'b>(&'b self, crossing: &'b RegionLineCrossing<'a>) -> &'b Point2 {
        match crossing.point {
            RegionLineCrossingPoint::Materialized(point) => point,
            RegionLineCrossingPoint::Certified(event_index) => {
                &self
                    .certified_line_crossings
                    .expect("certified point references retained crossings")[event_index]
                    .point
            }
        }
    }
}

fn segment_crossing_offsets(
    crossings: &[RegionLineCrossing<'_>],
    segment_count: usize,
) -> Option<Vec<usize>> {
    let mut offsets = Vec::with_capacity(segment_count + 1);
    let mut crossing_index = 0;
    for segment_index in 0..segment_count {
        offsets.push(crossing_index);
        while crossings
            .get(crossing_index)
            .is_some_and(|crossing| crossing.segment_index == segment_index)
        {
            crossing_index += 1;
        }
    }
    offsets.push(crossing_index);
    (crossing_index == crossings.len()).then_some(offsets)
}

fn sort_and_validate_unique(
    crossings: &mut [RegionLineCrossing<'_>],
    certified: Option<&[CertifiedLineCrossingEvent]>,
    policy: &CurveContext,
) -> bool {
    // A lossy preview is only an ordering hint. The exact adjacent check below
    // certifies the candidate order; ambiguity falls back to an all-exact sort.
    let preview = certified.is_none().then(|| {
        crossings
            .iter()
            .map(|crossing| {
                let RegionLineCrossingParameter::Materialized(parameter) = crossing.parameter
                else {
                    return None;
                };
                parameter
                    .to_f64_lossy()
                    .filter(|value| value.is_finite())
                    .map(|parameter| (crossing.clone(), parameter))
            })
            .collect::<Option<Vec<_>>>()
    });
    if let Some(Some(mut preview)) = preview {
        preview.sort_unstable_by(|(left, left_parameter), (right, right_parameter)| {
            left.segment_index
                .cmp(&right.segment_index)
                .then_with(|| left_parameter.total_cmp(right_parameter))
        });
        for (crossing, (ordered, _)) in crossings.iter_mut().zip(preview) {
            *crossing = ordered;
        }
        if crossing_order_is_certified(crossings, certified, policy) {
            return true;
        }
    }

    let mut order_decided = true;
    crossings.sort_unstable_by(|left, right| {
        left.segment_index.cmp(&right.segment_index).then_with(
            || match compare_crossing_parameters(left, right, certified, policy) {
                Some(ordering) => ordering,
                None => {
                    order_decided = false;
                    std::cmp::Ordering::Equal
                }
            },
        )
    });
    order_decided && crossing_order_is_certified(crossings, certified, policy)
}

#[cold]
#[inline(never)]
fn sort_segment_groups_and_validate_unique_normalized(
    crossings: &mut [RegionLineCrossing<'_>],
    certified: Option<&[CertifiedLineCrossingEvent]>,
    policy: &CurveContext,
    segment_count: usize,
) -> Option<Vec<usize>> {
    crossings.sort_unstable_by_key(|crossing| crossing.segment_index);
    let offsets = segment_crossing_offsets(crossings, segment_count)?;

    for range in offsets.windows(2) {
        let segment_crossings = &mut crossings[range[0]..range[1]];
        const MAX_FUSED_INSERTION_CROSSINGS: usize = 16;
        if segment_crossings.len() > MAX_FUSED_INSERTION_CROSSINGS {
            let mut order_decided = true;
            segment_crossings.sort_unstable_by(|left, right| {
                match compare_crossing_parameters_normalized(left, right, certified, policy) {
                    Some(ordering) => ordering,
                    None => {
                        order_decided = false;
                        std::cmp::Ordering::Equal
                    }
                }
            });
            if !order_decided
                || !segment_crossings.windows(2).all(|window| {
                    compare_crossing_parameters_normalized(
                        &window[0], &window[1], certified, policy,
                    ) == Some(std::cmp::Ordering::Less)
                })
            {
                return None;
            }
            continue;
        }
        for current in 1..segment_crossings.len() {
            let mut insertion = current;
            while insertion != 0 {
                match compare_crossing_parameters_normalized(
                    &segment_crossings[insertion - 1],
                    &segment_crossings[current],
                    certified,
                    policy,
                ) {
                    Some(std::cmp::Ordering::Less) => break,
                    Some(std::cmp::Ordering::Greater) => insertion -= 1,
                    Some(std::cmp::Ordering::Equal) | None => return None,
                }
            }
            segment_crossings[insertion..=current].rotate_right(1);
        }
    }
    Some(offsets)
}

fn compare_crossing_parameters(
    left: &RegionLineCrossing<'_>,
    right: &RegionLineCrossing<'_>,
    certified: Option<&[CertifiedLineCrossingEvent]>,
    policy: &CurveContext,
) -> Option<std::cmp::Ordering> {
    match (&left.parameter, &right.parameter) {
        (
            RegionLineCrossingParameter::Materialized(left),
            RegionLineCrossingParameter::Materialized(right),
        ) => compare_reals(left, right, policy),
        (
            RegionLineCrossingParameter::Certified {
                event_index: left,
                operand: left_operand,
            },
            RegionLineCrossingParameter::Certified {
                event_index: right,
                operand: right_operand,
            },
        ) if left_operand == right_operand => {
            let crossings = certified?;
            crossings[*left].compare_parameter(&crossings[*right], *left_operand, policy)
        }
        _ => None,
    }
}

fn compare_crossing_parameters_normalized(
    left: &RegionLineCrossing<'_>,
    right: &RegionLineCrossing<'_>,
    certified: Option<&[CertifiedLineCrossingEvent]>,
    policy: &CurveContext,
) -> Option<std::cmp::Ordering> {
    match (&left.parameter, &right.parameter) {
        (
            RegionLineCrossingParameter::Certified {
                event_index: left,
                operand: left_operand,
            },
            RegionLineCrossingParameter::Certified {
                event_index: right,
                operand: right_operand,
            },
        ) if left_operand == right_operand => {
            let crossings = certified?;
            crossings[*left].compare_parameter_normalized(&crossings[*right], *left_operand, policy)
        }
        _ => compare_crossing_parameters(left, right, certified, policy),
    }
}

fn crossing_order_is_certified(
    crossings: &[RegionLineCrossing<'_>],
    certified: Option<&[CertifiedLineCrossingEvent]>,
    policy: &CurveContext,
) -> bool {
    crossings.windows(2).all(|window| {
        window[0].segment_index < window[1].segment_index
            || window[0].segment_index == window[1].segment_index
                && compare_crossing_parameters(&window[0], &window[1], certified, policy)
                    == Some(std::cmp::Ordering::Less)
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn crossing<'a>(
        segment_index: usize,
        parameter: &'a Real,
        point: &'a Point2,
    ) -> RegionLineCrossing<'a> {
        RegionLineCrossing {
            segment_index,
            parameter: RegionLineCrossingParameter::Materialized(parameter),
            point: RegionLineCrossingPoint::Materialized(point),
            winding_delta: 1,
        }
    }

    #[test]
    fn lossy_crossing_order_is_exactly_certified_and_rejects_duplicates() {
        let policy = CurveContext::STRICT;
        let point = Point2::new(Real::zero(), Real::zero());
        let large = 1_i128 << 100;
        let lower = Real::from(large);
        let upper = Real::from(large + 1);
        assert_eq!(lower.to_f64_lossy(), upper.to_f64_lossy());

        let mut crossings = vec![
            crossing(1, &upper, &point),
            crossing(0, &upper, &point),
            crossing(1, &lower, &point),
        ];
        assert!(sort_and_validate_unique(&mut crossings, None, &policy));
        assert_eq!(crossings[0].segment_index, 0);
        assert!(matches!(
            crossings[1].parameter,
            RegionLineCrossingParameter::Materialized(parameter) if parameter == &lower
        ));
        assert!(matches!(
            crossings[2].parameter,
            RegionLineCrossingParameter::Materialized(parameter) if parameter == &upper
        ));

        let mut duplicates = vec![crossing(0, &lower, &point), crossing(0, &lower, &point)];
        assert!(!sort_and_validate_unique(&mut duplicates, None, &policy));
    }

    #[test]
    fn grouped_crossing_sort_builds_offsets_and_certifies_local_order() {
        let policy = CurveContext::STRICT;
        let point = Point2::new(Real::zero(), Real::zero());
        let lower = Real::from(1_i8);
        let upper = Real::from(2_i8);
        let mut crossings = vec![
            crossing(2, &upper, &point),
            crossing(0, &upper, &point),
            crossing(2, &lower, &point),
            crossing(1, &lower, &point),
        ];

        let offsets =
            sort_segment_groups_and_validate_unique_normalized(&mut crossings, None, &policy, 3)
                .unwrap();
        assert_eq!(offsets, [0, 1, 2, 4]);
        assert_eq!(
            compare_crossing_parameters(&crossings[2], &crossings[3], None, &policy),
            Some(std::cmp::Ordering::Less)
        );

        let mut duplicates = vec![crossing(2, &lower, &point), crossing(2, &lower, &point)];
        assert!(
            sort_segment_groups_and_validate_unique_normalized(&mut duplicates, None, &policy, 3,)
                .is_none()
        );
    }

    #[test]
    fn grouped_crossing_sort_bounds_dense_segment_insertion_work() {
        let policy = CurveContext::STRICT;
        let point = Point2::new(Real::zero(), Real::zero());
        let parameters = (0..32).map(Real::from).collect::<Vec<_>>();
        let mut crossings = parameters
            .iter()
            .rev()
            .map(|parameter| crossing(0, parameter, &point))
            .collect::<Vec<_>>();

        let offsets =
            sort_segment_groups_and_validate_unique_normalized(&mut crossings, None, &policy, 1)
                .unwrap();
        assert_eq!(offsets, [0, parameters.len()]);
        assert!(crossings.windows(2).all(|window| {
            compare_crossing_parameters(&window[0], &window[1], None, &policy)
                == Some(std::cmp::Ordering::Less)
        }));

        let mut duplicates = parameters
            .iter()
            .map(|parameter| crossing(0, parameter, &point))
            .collect::<Vec<_>>();
        duplicates.push(crossing(0, &parameters[0], &point));
        assert!(
            sort_segment_groups_and_validate_unique_normalized(&mut duplicates, None, &policy, 1,)
                .is_none()
        );
    }
}
