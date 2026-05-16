//! Triangle classification predicates.

use crate::classify::TriangleLocation;
use crate::geometry::{Point2, Triangle2Facts, triangle2_facts};
use crate::predicate::{Certainty, Escalation, PredicateOutcome, PredicatePolicy, Sign};
use crate::predicates::orient::orient2d_with_policy;

/// Reusable exact predicates for one 2D triangle.
///
/// A prepared triangle stores borrowed vertices, [`Triangle2Facts`], and the
/// orientation result under the policy used at preparation time. This is useful
/// for ear-clipping and CDT validation loops that classify many candidate
/// points against the same triangle. It remains a predicate helper: ear nodes,
/// face ids, cavity ownership, and triangulation policy stay in `hypertri`.
///
/// The orientation-side test is the standard triangle containment classifier
/// from computational geometry; see de Berg, Cheong, van Kreveld, and Overmars,
/// *Computational Geometry: Algorithms and Applications*, 3rd ed., Springer,
/// 2008. Caching the object facts follows Yap's exact-geometric-computation
/// model; see Yap, "Towards Exact Geometric Computation," *Computational
/// Geometry* 7.1-2 (1997).
#[derive(Clone, Copy, Debug)]
pub struct PreparedTriangle2<'a> {
    a: &'a Point2,
    b: &'a Point2,
    c: &'a Point2,
    facts: Triangle2Facts,
    orientation: PredicateOutcome<Sign>,
    policy: PredicatePolicy,
}

impl<'a> PreparedTriangle2<'a> {
    /// Prepare a triangle using the default predicate policy.
    pub fn new(a: &'a Point2, b: &'a Point2, c: &'a Point2) -> Self {
        Self::with_policy(a, b, c, PredicatePolicy::default())
    }

    /// Prepare a triangle using an explicit predicate policy.
    pub fn with_policy(
        a: &'a Point2,
        b: &'a Point2,
        c: &'a Point2,
        policy: PredicatePolicy,
    ) -> Self {
        crate::trace_dispatch!("hyperlimit", "prepared_triangle2", "new");
        let facts = triangle2_facts(a, b, c);
        let orientation = triangle_orientation_with_policy_and_facts(a, b, c, policy, facts);
        Self::from_parts(a, b, c, facts, orientation, policy)
    }

    /// Prepare a triangle from caller-cached facts and orientation.
    ///
    /// The caller must pass facts and orientation for the same vertex triple and
    /// policy. Conservative facts merely leave fast paths unused, but
    /// non-conservative facts or an orientation from different vertices can
    /// change the classified result.
    pub const fn from_parts(
        a: &'a Point2,
        b: &'a Point2,
        c: &'a Point2,
        facts: Triangle2Facts,
        orientation: PredicateOutcome<Sign>,
        policy: PredicatePolicy,
    ) -> Self {
        Self {
            a,
            b,
            c,
            facts,
            orientation,
            policy,
        }
    }

    /// Return vertex `a`.
    pub const fn a(&self) -> &'a Point2 {
        self.a
    }

    /// Return vertex `b`.
    pub const fn b(&self) -> &'a Point2 {
        self.b
    }

    /// Return vertex `c`.
    pub const fn c(&self) -> &'a Point2 {
        self.c
    }

    /// Return cached structural facts.
    pub const fn facts(&self) -> Triangle2Facts {
        self.facts
    }

    /// Return the cached orientation result.
    pub const fn orientation(&self) -> PredicateOutcome<Sign> {
        self.orientation
    }

    /// Return the policy used to compute the cached orientation.
    pub const fn policy(&self) -> PredicatePolicy {
        self.policy
    }

    /// Classify a point using the policy captured at preparation time.
    pub fn classify_point(&self, point: &Point2) -> PredicateOutcome<TriangleLocation> {
        classify_point_triangle_impl(
            self.a,
            self.b,
            self.c,
            point,
            self.policy,
            Some(self.facts),
            Some(self.orientation),
        )
    }

    /// Classify a point with an explicit predicate policy.
    ///
    /// The cached orientation is reused when `policy` matches the preparation
    /// policy. If a different policy is requested, orientation is recomputed
    /// under that policy while cached structural facts are still reused.
    pub fn classify_point_with_policy(
        &self,
        point: &Point2,
        policy: PredicatePolicy,
    ) -> PredicateOutcome<TriangleLocation> {
        let cached_orientation = if policy == self.policy {
            Some(self.orientation)
        } else {
            None
        };
        classify_point_triangle_impl(
            self.a,
            self.b,
            self.c,
            point,
            policy,
            Some(self.facts),
            cached_orientation,
        )
    }
}

/// Classify `point` relative to triangle `abc`.
pub fn classify_point_triangle(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    point: &Point2,
) -> PredicateOutcome<TriangleLocation> {
    classify_point_triangle_with_policy(a, b, c, point, PredicatePolicy::default())
}

/// Classify `point` relative to triangle `abc` with an explicit escalation
/// policy.
pub fn classify_point_triangle_with_policy(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    point: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<TriangleLocation> {
    classify_point_triangle_impl(a, b, c, point, policy, None, None)
}

/// Classify `point` relative to triangle `abc` using cached structural facts.
pub fn classify_point_triangle_with_facts(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    point: &Point2,
    facts: Triangle2Facts,
) -> PredicateOutcome<TriangleLocation> {
    classify_point_triangle_with_policy_and_facts(a, b, c, point, PredicatePolicy::default(), facts)
}

/// Classify `point` relative to triangle `abc` with both an explicit policy and
/// cached structural facts.
///
/// Cached facts can certify structurally degenerate triangles without building
/// the orientation determinant. Non-degenerate containment still uses exact
/// orientation signs for the three triangle edges.
pub fn classify_point_triangle_with_policy_and_facts(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    point: &Point2,
    policy: PredicatePolicy,
    facts: Triangle2Facts,
) -> PredicateOutcome<TriangleLocation> {
    classify_point_triangle_impl(a, b, c, point, policy, Some(facts), None)
}

fn classify_point_triangle_impl(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    point: &Point2,
    policy: PredicatePolicy,
    facts: Option<Triangle2Facts>,
    cached_orientation: Option<PredicateOutcome<Sign>>,
) -> PredicateOutcome<TriangleLocation> {
    let triangle_outcome = cached_orientation
        .unwrap_or_else(|| triangle_orientation_with_optional_facts(a, b, c, policy, facts));

    let triangle = match triangle_outcome {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => DecidedSign {
            sign: value,
            certainty,
            stage,
        },
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::Unknown { needed, stage };
        }
    };

    if triangle.sign == Sign::Zero {
        return PredicateOutcome::decided(
            TriangleLocation::Degenerate,
            triangle.certainty,
            triangle.stage,
        );
    }

    let ab = match orient2d_with_policy(a, b, point, policy) {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => DecidedSign {
            sign: value,
            certainty,
            stage,
        },
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::Unknown { needed, stage };
        }
    };
    let bc = match orient2d_with_policy(b, c, point, policy) {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => DecidedSign {
            sign: value,
            certainty,
            stage,
        },
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::Unknown { needed, stage };
        }
    };
    let ca = match orient2d_with_policy(c, a, point, policy) {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => DecidedSign {
            sign: value,
            certainty,
            stage,
        },
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::Unknown { needed, stage };
        }
    };

    let certainty =
        combine_certainties([triangle.certainty, ab.certainty, bc.certainty, ca.certainty]);
    let stage = combine_stages([triangle.stage, ab.stage, bc.stage, ca.stage]);
    let edge_signs = [ab.sign, bc.sign, ca.sign];

    let opposite = match triangle.sign {
        Sign::Positive => Sign::Negative,
        Sign::Negative => Sign::Positive,
        Sign::Zero => unreachable!("degenerate triangle returned early"),
    };

    if edge_signs.contains(&opposite) {
        return PredicateOutcome::decided(TriangleLocation::Outside, certainty, stage);
    }

    let zero_count = edge_signs
        .iter()
        .filter(|&&sign| sign == Sign::Zero)
        .count();
    let location = match zero_count {
        0 => TriangleLocation::Inside,
        1 => TriangleLocation::OnEdge,
        _ => TriangleLocation::OnVertex,
    };

    PredicateOutcome::decided(location, certainty, stage)
}

fn triangle_orientation_with_optional_facts(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    policy: PredicatePolicy,
    facts: Option<Triangle2Facts>,
) -> PredicateOutcome<Sign> {
    if let Some(facts) = facts {
        triangle_orientation_with_policy_and_facts(a, b, c, policy, facts)
    } else {
        orient2d_with_policy(a, b, c, policy)
    }
}

fn triangle_orientation_with_policy_and_facts(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    policy: PredicatePolicy,
    facts: Triangle2Facts,
) -> PredicateOutcome<Sign> {
    if facts.known_degenerate() == Some(true) {
        // Same-axis and duplicate-vertex degeneracies can be certified from
        // exact zero/nonzero structure before constructing the orientation
        // determinant. This is the local version of the retained-object facts
        // advocated by Yap (1997); it is still an exact predicate result.
        PredicateOutcome::decided(Sign::Zero, Certainty::Exact, Escalation::Structural)
    } else {
        orient2d_with_policy(a, b, c, policy)
    }
}

#[derive(Clone, Copy)]
struct DecidedSign {
    sign: Sign,
    certainty: Certainty,
    stage: Escalation,
}

fn combine_certainties(values: [Certainty; 4]) -> Certainty {
    values
        .into_iter()
        .max_by_key(|certainty| certainty_rank(*certainty))
        .unwrap_or(Certainty::Exact)
}

fn certainty_rank(certainty: Certainty) -> u8 {
    match certainty {
        Certainty::Exact => 0,
        Certainty::Filtered => 1,
    }
}

fn combine_stages(values: [Escalation; 4]) -> Escalation {
    values
        .into_iter()
        .max_by_key(|stage| stage_rank(*stage))
        .unwrap_or(Escalation::Undecided)
}

fn stage_rank(stage: Escalation) -> u8 {
    match stage {
        Escalation::Structural => 0,
        Escalation::Filter => 1,
        Escalation::Exact => 2,
        Escalation::Refined => 3,
        Escalation::Undecided => 4,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn real(value: f64) -> hyperreal::Real {
        hyperreal::Real::try_from(value).expect("finite test Real")
    }

    fn p2(x: f64, y: f64) -> Point2 {
        Point2::new(real(x), real(y))
    }

    #[test]
    fn classifies_point_inside_triangle() {
        let a = p2(0.0, 0.0);
        let b = p2(2.0, 0.0);
        let c = p2(0.0, 2.0);
        let point = p2(0.5, 0.5);

        assert_eq!(
            classify_point_triangle(&a, &b, &c, &point).value(),
            Some(TriangleLocation::Inside)
        );
    }

    #[test]
    fn classifies_point_on_triangle_edge() {
        let a = p2(0.0, 0.0);
        let b = p2(2.0, 0.0);
        let c = p2(0.0, 2.0);
        let point = p2(1.0, 0.0);

        assert_eq!(
            classify_point_triangle(&a, &b, &c, &point).value(),
            Some(TriangleLocation::OnEdge)
        );
    }

    #[test]
    fn classifies_degenerate_triangle() {
        let a = p2(0.0, 0.0);
        let b = p2(1.0, 1.0);
        let c = p2(2.0, 2.0);
        let point = p2(1.0, 1.0);

        assert_eq!(
            classify_point_triangle(&a, &b, &c, &point).value(),
            Some(TriangleLocation::Degenerate)
        );
    }

    #[test]
    fn fact_aware_classifier_uses_structural_triangle_degeneracy() {
        let a = p2(0.0, 0.0);
        let b = p2(2.0, 0.0);
        let c = p2(5.0, 0.0);
        let point = p2(1.0, 0.0);
        let facts = triangle2_facts(&a, &b, &c);
        let policy = PredicatePolicy {
            allow_exact: false,
            allow_refinement: false,
            ..PredicatePolicy::STRICT
        };

        assert_eq!(facts.known_degenerate(), Some(true));
        assert_eq!(
            classify_point_triangle_with_policy_and_facts(&a, &b, &c, &point, policy, facts)
                .value(),
            Some(TriangleLocation::Degenerate)
        );
    }

    #[test]
    fn prepared_triangle_classifies_points_with_cached_orientation() {
        let a = p2(0.0, 0.0);
        let b = p2(3.0, 0.0);
        let c = p2(0.0, 3.0);
        let inside = p2(1.0, 1.0);
        let outside = p2(3.0, 3.0);

        let prepared = PreparedTriangle2::new(&a, &b, &c);
        assert_eq!(prepared.orientation().value(), Some(Sign::Positive));
        assert_eq!(prepared.facts().known_non_degenerate(), Some(true));
        assert_eq!(
            prepared.classify_point(&inside).value(),
            Some(TriangleLocation::Inside)
        );
        assert_eq!(
            prepared.classify_point(&outside).value(),
            Some(TriangleLocation::Outside)
        );
    }
}
