//! Caller-certified collinear point sets for constrained triangulation.

use crate::context::TriangulationCertainty;
use crate::error::{Error, Result};
use crate::types::Point2;
use hyperlimit::{Certainty, Escalation, PredicateOutcome, PredicatePolicy, Sign};

/// Sets of point indices that the caller has proved collinear.
///
/// Each line names indices into the point slice of one triangulation whose
/// points lie on one straight line, by a certificate the caller holds, such as
/// the construction that placed them there. Given these facts, a
/// triangulation takes the orientation of any three points of one line as
/// zero instead of evaluating the determinant, and decides only their order
/// along it by exact comparisons. The facts are trusted: a set that is not
/// collinear gives wrong topology. A point may lie on several lines, and a set
/// of fewer than three points states nothing.
///
/// The certainty is that of the weakest evidence behind the facts. Facts that
/// consumed the approximate terminal are used only under `APPROXIMATE_512`,
/// and using one makes the outcome approximate; under `STRICT` they are
/// ignored and every determinant is evaluated.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CollinearPointFacts {
    memberships: Vec<(usize, u32)>,
    lines: u32,
    certainty: TriangulationCertainty,
}

impl CollinearPointFacts {
    /// Empty facts whose evidence has the given certainty.
    pub const fn new(certainty: TriangulationCertainty) -> Self {
        Self {
            memberships: Vec::new(),
            lines: 0,
            certainty,
        }
    }

    /// Record that the points with these indices lie on one line.
    ///
    /// Repeated indices are ignored, as is a line of fewer than three distinct
    /// points.
    pub fn push_line(&mut self, points: &[usize]) -> Result<()> {
        if points.len() < 3 {
            return Ok(());
        }
        let line = self.lines;
        let start = self.memberships.len();
        self.memberships
            .try_reserve(points.len())
            .map_err(|_| Error::InvalidInput {
                reason: "collinear point facts exceed available memory",
            })?;
        self.memberships
            .extend(points.iter().map(|&point| (point, line)));
        self.memberships[start..].sort_unstable();
        let mut distinct = start;
        for index in start..self.memberships.len() {
            if index == start || self.memberships[index] != self.memberships[distinct - 1] {
                self.memberships[distinct] = self.memberships[index];
                distinct += 1;
            }
        }
        self.memberships.truncate(distinct);
        if distinct - start < 3 {
            self.memberships.truncate(start);
            return Ok(());
        }
        self.lines = line.checked_add(1).ok_or(Error::InvalidInput {
            reason: "too many collinear point facts",
        })?;
        Ok(())
    }

    /// Whether no line of three or more points has been recorded.
    pub fn is_empty(&self) -> bool {
        self.lines == 0
    }

    /// Certainty of the evidence behind the facts.
    pub const fn certainty(&self) -> TriangulationCertainty {
        self.certainty
    }
}

/// Lookup of [`CollinearPointFacts`] by point reference for one triangulation
/// of one borrowed point slice.
///
/// Triangulation algorithms pass references into the caller's slice to the
/// operation's predicate evaluator. The slice stays borrowed for the whole
/// operation, so no other live point can occupy its memory, and a reference
/// whose address falls inside it at a whole element offset is that element.
/// Any other reference, such as a point an algorithm constructed itself, has
/// no index and no facts.
#[derive(Debug)]
pub(crate) struct CollinearIndex {
    base: usize,
    len: usize,
    memberships: Vec<(usize, u32)>,
    certainty: Certainty,
}

impl CollinearIndex {
    pub(crate) fn new(points: &[Point2], facts: &CollinearPointFacts) -> Result<Self> {
        let mut memberships = Vec::new();
        memberships
            .try_reserve_exact(facts.memberships.len())
            .map_err(|_| Error::InvalidInput {
                reason: "collinear point facts exceed available memory",
            })?;
        memberships.extend_from_slice(&facts.memberships);
        if memberships.iter().any(|&(point, _)| point >= points.len()) {
            return Err(Error::InvalidInput {
                reason: "collinear point fact references an absent point",
            });
        }
        memberships.sort_unstable();
        Ok(Self {
            base: points.as_ptr() as usize,
            len: points.len(),
            memberships,
            certainty: match facts.certainty {
                TriangulationCertainty::Certified => Certainty::Exact,
                TriangulationCertainty::Approximate512Consumed => Certainty::Approximate,
            },
        })
    }

    fn index_of(&self, point: &Point2) -> Option<usize> {
        let offset = (point as *const Point2 as usize).checked_sub(self.base)?;
        let size = core::mem::size_of::<Point2>();
        if offset % size != 0 {
            return None;
        }
        let index = offset / size;
        (index < self.len).then_some(index)
    }

    fn lines_of(&self, point: usize) -> &[(usize, u32)] {
        let start = self
            .memberships
            .partition_point(|&(candidate, _)| candidate < point);
        let end = self
            .memberships
            .partition_point(|&(candidate, _)| candidate <= point);
        &self.memberships[start..end]
    }

    /// Whether the three points lie on one recorded line.
    fn share_line(&self, a: &Point2, b: &Point2, c: &Point2) -> bool {
        let (Some(a), Some(b), Some(c)) = (self.index_of(a), self.index_of(b), self.index_of(c))
        else {
            return false;
        };
        let [first, second, third] = [self.lines_of(a), self.lines_of(b), self.lines_of(c)];
        first.iter().any(|&(_, line)| {
            second.iter().any(|&(_, other)| other == line)
                && third.iter().any(|&(_, other)| other == line)
        })
    }

    /// The certified zero orientation of three points of one recorded line,
    /// when `policy` accepts the facts.
    pub(crate) fn known_zero(
        &self,
        a: &Point2,
        b: &Point2,
        c: &Point2,
        policy: PredicatePolicy,
    ) -> Option<PredicateOutcome<Sign>> {
        (policy.accepts(self.certainty) && self.share_line(a, b, c)).then_some(
            PredicateOutcome::decided(Sign::Zero, self.certainty, Escalation::Structural),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cdt::constrained_triangulation_convex_hull_with_collinear_facts as triangulate;
    use crate::context::TriangulationContext;
    use crate::types::{Constraint, Real};

    fn unresolved_zero() -> Real {
        let sine = Real::e().sin();
        let cosine = Real::e().cos();
        crate::test_uncertainty::terminal_zero(&sine * &sine + &cosine * &cosine - Real::one())
    }

    #[test]
    fn collinearity_lookup_requires_the_original_point_buffer_and_admissible_policy() {
        // The exact integer determinant is 1*4 - 2*2 = 0. The copied
        // coordinates denote the same points but have no source membership.
        let source = vec![
            Point2::new(Real::from(0), Real::from(0)),
            Point2::new(Real::from(1), Real::from(2)),
            Point2::new(Real::from(2), Real::from(4)),
        ];
        let copied = source.clone();
        for certainty in [
            TriangulationCertainty::Certified,
            TriangulationCertainty::Approximate512Consumed,
        ] {
            let mut facts = CollinearPointFacts::new(certainty);
            facts.push_line(&[0, 1, 2]).unwrap();
            let index = CollinearIndex::new(&source, &facts).unwrap();
            for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
                let outcome = index.known_zero(&source[0], &source[1], &source[2], policy);
                let expected = match certainty {
                    TriangulationCertainty::Certified => Some(Certainty::Exact),
                    TriangulationCertainty::Approximate512Consumed
                        if policy == PredicatePolicy::APPROXIMATE_512 =>
                    {
                        Some(Certainty::Approximate)
                    }
                    _ => None,
                };
                match (outcome, expected) {
                    (
                        Some(PredicateOutcome::Decided {
                            value: Sign::Zero,
                            certainty,
                            ..
                        }),
                        Some(expected),
                    ) => {
                        assert_eq!(certainty, expected);
                    }
                    (None, None) => {}
                    other => panic!("unexpected source-bound fact: {other:?}"),
                }
                for [a, b, c] in [
                    [&copied[0], &copied[1], &copied[2]],
                    [&copied[0], &source[1], &source[2]],
                    [&source[0], &copied[1], &source[2]],
                    [&source[0], &source[1], &copied[2]],
                ] {
                    assert!(index.known_zero(a, b, c, policy).is_none());
                }
            }
        }
    }

    /// A square crossed by a constrained chain on the line `y = x / 2 + 2`,
    /// whose middle point is on it only by an identity no proof decides. No
    /// other three points are collinear.
    fn diagonal_chain() -> (Vec<Point2>, Vec<Constraint>) {
        let point = |x: i64, y: Real| Point2::new(Real::from(x), y);
        let points = vec![
            point(0, Real::from(0)),
            point(16, Real::from(0)),
            point(16, Real::from(16)),
            point(0, Real::from(16)),
            point(2, Real::from(3)),
            point(6, Real::from(5) + unresolved_zero()),
            point(10, Real::from(7)),
        ];
        let constraints = [[0, 1], [1, 2], [2, 3], [3, 0], [4, 5], [5, 6]]
            .map(|[from, to]| Constraint::new(from, to))
            .to_vec();
        (points, constraints)
    }

    #[test]
    fn recorded_collinearity_completes_a_triangulation_the_determinant_cannot() {
        let (points, constraints) = diagonal_chain();
        let strict = TriangulationContext::new(PredicatePolicy::STRICT);
        let approximate = TriangulationContext::new(PredicatePolicy::APPROXIMATE_512);
        let none = CollinearPointFacts::new(TriangulationCertainty::Certified);
        assert!(matches!(
            triangulate(&strict, &points, &constraints, &none),
            Err(Error::PredicateUndecided { .. })
        ));

        let mut certified = CollinearPointFacts::new(TriangulationCertainty::Certified);
        certified.push_line(&[6, 4, 5, 4]).unwrap();
        assert!(!certified.is_empty());
        let outcome = triangulate(&strict, &points, &constraints, &certified).unwrap();
        assert_eq!(outcome.certainty, TriangulationCertainty::Certified);
        let has_edge = |[from, to]: [usize; 2]| {
            outcome
                .value
                .iter()
                .any(|triangle| triangle.contains(&from) && triangle.contains(&to))
        };
        assert!(has_edge([4, 5]) && has_edge([5, 6]));
        // No triangle uses the three collinear points.
        assert!(
            !outcome
                .value
                .iter()
                .any(|triangle| [4, 5, 6].iter().all(|point| triangle.contains(point)))
        );

        // Facts that consumed the terminal are no strict proof.
        let mut qualified =
            CollinearPointFacts::new(TriangulationCertainty::Approximate512Consumed);
        qualified.push_line(&[4, 5, 6]).unwrap();
        assert!(matches!(
            triangulate(&strict, &points, &constraints, &qualified),
            Err(Error::PredicateUndecided { .. })
        ));
        let outcome = triangulate(&approximate, &points, &constraints, &qualified).unwrap();
        assert_eq!(
            outcome.certainty,
            TriangulationCertainty::Approximate512Consumed
        );

        // Lines of fewer than three distinct points state nothing; an absent
        // point is invalid.
        let mut short = CollinearPointFacts::new(TriangulationCertainty::Certified);
        short.push_line(&[4, 5, 5, 4]).unwrap();
        assert!(short.is_empty());
        let mut absent = CollinearPointFacts::new(TriangulationCertainty::Certified);
        absent.push_line(&[4, 5, 7]).unwrap();
        assert!(matches!(
            triangulate(&strict, &points, &constraints, &absent),
            Err(Error::InvalidInput { .. })
        ));
    }

    #[test]
    fn recorded_collinearity_decides_overlapping_constraints() {
        // Constraints 4-6 and 5-7 overlap on the line `y = x / 2 + 2`, from
        // point 5 to point 6. Point 5 lies on the line only by an identity no
        // proof decides, so without facts the crossing check is undecided.
        let (mut points, _) = diagonal_chain();
        points.push(Point2::new(Real::from(14), Real::from(9)));
        let constraints = [[0, 1], [1, 2], [2, 3], [3, 0], [4, 6], [5, 7]]
            .map(|[from, to]| Constraint::new(from, to))
            .to_vec();
        let strict = TriangulationContext::new(PredicatePolicy::STRICT);
        let none = CollinearPointFacts::new(TriangulationCertainty::Certified);
        assert!(matches!(
            triangulate(&strict, &points, &constraints, &none),
            Err(Error::PredicateUndecided { .. })
        ));

        // With the line recorded, the four orientations of the overlap test
        // are known zeros and only the order along the line is compared, so
        // the overlap is reported exactly: this entry point requires
        // planarized constraints.
        let mut line = CollinearPointFacts::new(TriangulationCertainty::Certified);
        line.push_line(&[4, 5]).unwrap();
        assert!(line.is_empty());
        line.push_line(&[4, 5, 6, 7]).unwrap();
        assert_eq!(line.certainty(), TriangulationCertainty::Certified);
        assert!(matches!(
            triangulate(&strict, &points, &constraints, &line),
            Err(Error::InvalidInput {
                reason: "overlapping constraints are not supported"
            })
        ));
    }
}
