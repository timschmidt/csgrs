//! Deterministic exact interior witnesses for split cells.
//!
//! Volumetric winding materialization needs a point that is strictly inside a
//! retained source-face cell but still replayable from exact source geometry.
//! This module keeps that policy as data: each witness is an exact positive
//! barycentric combination of the cell triangle vertices. The classifier tries
//! the centroid first, then a fixed lattice of rational interior points before
//! declaring the cell undecided.

use hyperlimit::Point3;

use hyperreal::Real;

/// Exact positive barycentric witness for one triangulated source-face cell.
///
/// The witness stores integer weights and their denominator so a retained
/// volumetric classification can replay the exact representative point from
/// source vertices. All production witnesses are strict interior points:
/// every weight is positive and the denominator is exactly the weight sum.
/// Keeping that certificate next to the winding result makes the sample choice
/// replayable without implicit perturbation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct ExactTriangleInteriorWitness {
    /// Positive barycentric weights for the three triangle vertices.
    pub(crate) weights: [i64; 3],
    /// Sum of [`Self::weights`].
    pub(crate) denominator: i64,
}

impl ExactTriangleInteriorWitness {
    /// Build a strict interior witness from positive integer weights.
    ///
    /// This constructor is intentionally `const` for the built-in retry
    /// lattice. Runtime callers should still use [`Self::validate`] before
    /// accepting externally supplied witness data.
    pub(crate) const fn new(weights: [i64; 3]) -> Self {
        let denominator = match weights[0].checked_add(weights[1]) {
            Some(partial) => match partial.checked_add(weights[2]) {
                Some(sum) => sum,
                None => i64::MIN,
            },
            None => i64::MIN,
        };
        Self {
            weights,
            denominator,
        }
    }

    /// Return whether the witness is a strict positive barycentric point.
    pub(crate) const fn is_strict_interior(self) -> bool {
        self.denominator > 0
            && self.weights[0] > 0
            && self.weights[1] > 0
            && self.weights[2] > 0
            && match self.weights[0].checked_add(self.weights[1]) {
                Some(partial) => match partial.checked_add(self.weights[2]) {
                    Some(sum) => self.denominator == sum,
                    None => false,
                },
                None => false,
            }
    }

    /// Validate the retained witness shape.
    pub(crate) fn validate(self) -> Result<(), ExactTriangleInteriorWitnessError> {
        if self.is_strict_interior() {
            Ok(())
        } else {
            Err(ExactTriangleInteriorWitnessError::NotStrictInterior)
        }
    }

    /// Materialize the exact representative point for a triangle.
    ///
    /// The arithmetic stays in [`Real`], so replaying this method reproduces
    /// the exact point used by the winding report.
    pub(crate) fn point_for_triangle(
        self,
        a: &Point3,
        b: &Point3,
        c: &Point3,
    ) -> Result<Point3, ExactTriangleInteriorWitnessError> {
        self.validate()?;
        let inv = (Real::from(1) / &Real::from(self.denominator))
            .expect("strict interior witness denominator is nonzero");
        Ok(Point3::new(
            weighted_real(&a.x, &b.x, &c.x, self.weights, &inv),
            weighted_real(&a.y, &b.y, &c.y, self.weights, &inv),
            weighted_real(&a.z, &b.z, &c.z, self.weights, &inv),
        ))
    }
}

/// Validation failure for retained triangle interior witnesses.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ExactTriangleInteriorWitnessError {
    /// A witness had a non-positive weight, non-positive denominator, or a
    /// denominator that did not equal the exact weight sum.
    NotStrictInterior,
}

/// Deterministic retry lattice for volumetric region classification.
///
/// The ordering keeps the historical centroid and quarter-point retries first
/// for compatibility, then expands to asymmetric interior witnesses. The
/// asymmetric points matter for adversarial arrangements where symmetric
/// points sit on retained boundaries or ray-degenerate loci. No randomness or
/// floating perturbation is introduced; every candidate remains replayable.
pub(crate) const EXACT_TRIANGLE_INTERIOR_WITNESSES: &[ExactTriangleInteriorWitness] = &[
    ExactTriangleInteriorWitness::new([1, 1, 1]),
    ExactTriangleInteriorWitness::new([2, 1, 1]),
    ExactTriangleInteriorWitness::new([1, 2, 1]),
    ExactTriangleInteriorWitness::new([1, 1, 2]),
    ExactTriangleInteriorWitness::new([3, 1, 1]),
    ExactTriangleInteriorWitness::new([1, 3, 1]),
    ExactTriangleInteriorWitness::new([1, 1, 3]),
    ExactTriangleInteriorWitness::new([2, 2, 1]),
    ExactTriangleInteriorWitness::new([2, 1, 2]),
    ExactTriangleInteriorWitness::new([1, 2, 2]),
    ExactTriangleInteriorWitness::new([3, 2, 1]),
    ExactTriangleInteriorWitness::new([3, 1, 2]),
    ExactTriangleInteriorWitness::new([2, 3, 1]),
    ExactTriangleInteriorWitness::new([1, 3, 2]),
    ExactTriangleInteriorWitness::new([2, 1, 3]),
    ExactTriangleInteriorWitness::new([1, 2, 3]),
    ExactTriangleInteriorWitness::new([4, 2, 1]),
    ExactTriangleInteriorWitness::new([4, 1, 2]),
    ExactTriangleInteriorWitness::new([2, 4, 1]),
    ExactTriangleInteriorWitness::new([1, 4, 2]),
    ExactTriangleInteriorWitness::new([2, 1, 4]),
    ExactTriangleInteriorWitness::new([1, 2, 4]),
    ExactTriangleInteriorWitness::new([5, 3, 2]),
    ExactTriangleInteriorWitness::new([5, 2, 3]),
    ExactTriangleInteriorWitness::new([3, 5, 2]),
    ExactTriangleInteriorWitness::new([2, 5, 3]),
    ExactTriangleInteriorWitness::new([3, 2, 5]),
    ExactTriangleInteriorWitness::new([2, 3, 5]),
];

fn weighted_real(a: &Real, b: &Real, c: &Real, weights: [i64; 3], inv_denominator: &Real) -> Real {
    (a.clone() * Real::from(weights[0])
        + b.clone() * Real::from(weights[1])
        + c.clone() * Real::from(weights[2]))
        * inv_denominator
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exact_triangle_witness_lattice_is_strict_and_unique() {
        assert_eq!(
            EXACT_TRIANGLE_INTERIOR_WITNESSES[0],
            ExactTriangleInteriorWitness::new([1, 1, 1])
        );

        let mut seen = Vec::new();
        for witness in EXACT_TRIANGLE_INTERIOR_WITNESSES {
            witness.validate().unwrap();
            assert!(!seen.contains(&(witness.weights, witness.denominator)));
            seen.push((witness.weights, witness.denominator));
        }
    }

    #[test]
    fn exact_triangle_witness_rejects_boundary_or_inconsistent_weights() {
        assert_eq!(
            ExactTriangleInteriorWitness {
                weights: [1, 0, 1],
                denominator: 2
            }
            .validate()
            .unwrap_err(),
            ExactTriangleInteriorWitnessError::NotStrictInterior
        );
        assert_eq!(
            ExactTriangleInteriorWitness {
                weights: [1, 1, 1],
                denominator: 4
            }
            .validate()
            .unwrap_err(),
            ExactTriangleInteriorWitnessError::NotStrictInterior
        );
        assert_eq!(
            ExactTriangleInteriorWitness::new([i64::MAX, 1, 1])
                .validate()
                .unwrap_err(),
            ExactTriangleInteriorWitnessError::NotStrictInterior
        );
    }
}
