//! Predicate-backed geometric checks used by exact mesh validation.
//!
//! Triangle degeneracy is tested by exact 2D orientation in coordinate
//! projections. If every projection has zero orientation, the three 3D points
//! are collinear. The orientation predicates are delegated to `hyperlimit`,
//! following Shewchuk, "Adaptive Precision Floating-Point Arithmetic and Fast
//! Robust Geometric Predicates," *Discrete & Computational Geometry* 18
//! (1997), and Yap, "Towards Exact Geometric Computation," *Computational
//! Geometry* 7.1-2 (1997): the predicate certificate, not an epsilon, is the
//! topology authority.

use hyperlimit::{Point2, Point3, PredicateReport, Sign, orient2d_report};

use super::provenance::PredicateUse;

/// Result of exact triangle degeneracy classification.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TriangleDegeneracy {
    /// At least one coordinate projection has non-zero certified orientation.
    NonDegenerate,
    /// All coordinate projections are exactly collinear.
    Degenerate,
    /// A needed predicate could not be decided by the enabled exact route.
    Unknown,
}

/// Predicate reports retained while classifying one triangle.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TrianglePredicateReport {
    /// Degeneracy result.
    pub degeneracy: TriangleDegeneracy,
    /// Predicate certificates used by the classification.
    pub predicates: Vec<PredicateUse>,
}

impl TrianglePredicateReport {
    /// Return whether all retained predicate routes were proof-producing.
    pub fn all_proof_producing(&self) -> bool {
        self.predicates
            .iter()
            .copied()
            .all(PredicateUse::is_proof_producing)
    }
}

/// Classify whether three exact 3D points form a non-degenerate triangle.
pub fn classify_triangle_degeneracy(a: &Point3, b: &Point3, c: &Point3) -> TrianglePredicateReport {
    let reports = [
        orient2d_report(&xy(a), &xy(b), &xy(c)),
        orient2d_report(&xz(a), &xz(b), &xz(c)),
        orient2d_report(&yz(a), &yz(b), &yz(c)),
    ];

    let mut predicates = Vec::with_capacity(reports.len());
    let mut all_zero = true;

    for report in reports {
        predicates.push(PredicateUse::from_certificate(report.certificate));
        match report_sign(report) {
            Some(Sign::Positive | Sign::Negative) => {
                return TrianglePredicateReport {
                    degeneracy: TriangleDegeneracy::NonDegenerate,
                    predicates,
                };
            }
            Some(Sign::Zero) => {}
            None => all_zero = false,
        }
    }

    TrianglePredicateReport {
        degeneracy: if all_zero {
            TriangleDegeneracy::Degenerate
        } else {
            TriangleDegeneracy::Unknown
        },
        predicates,
    }
}

fn report_sign(report: PredicateReport<Sign>) -> Option<Sign> {
    report.value()
}

fn xy(point: &Point3) -> Point2 {
    Point2::new(point.x.clone(), point.y.clone())
}

fn xz(point: &Point3) -> Point2 {
    Point2::new(point.x.clone(), point.z.clone())
}

fn yz(point: &Point3) -> Point2 {
    Point2::new(point.y.clone(), point.z.clone())
}
