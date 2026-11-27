//! Exact 2D coordinate expressions and conservative structural support snapshots.
//!
//! Coordinates are ordered x,y; displacement is to-from and the orientation determinant
//! is the exterior product of b-a and c-a. Expressions retain Real arithmetic without
//! rounding, a predicate policy, a certainty receipt, or source-incidence evidence.
//! The predicate layer must decide any sign, equality, sidedness, or topology claim.
//!
//! Fact producers inspect scalar zero knowledge without starting refinement. Unknown
//! is not false, and two nonzero products can cancel. Public packet fields may be
//! fabricated or changed; methods trust them without binding them to coordinates or
//! revalidating consistency. Recompute facts after changing the represented inputs.
//! Existing scalar cached proofs can strengthen later queries.
//!
//! Product-sum masks expose only the first 64 terms. Counts scan every term but cast
//! to u32; neither mask nor count is an unlimited-size certificate. Fixed array shapes
//! do not bound nested scalar work, coefficient growth, or retained memory. Expression
//! construction uses the lattice RealKernelExt path, whose definite-zero pruning is
//! narrower than the zero_status queries used by the fact producers.

use crate::{Axis2, Real, RealKernelExt, ZeroKnowledge};

/// Snapshot of zero knowledge for two displacement components and their support axis.
///
/// Producer-created axis information requires one known-nonzero component and one
/// known-zero component, without a sign or unit-length requirement. This can select
/// axis-specialized algebra; it is not a topology or incidence receipt. Public fields
/// are independently editable, and equality compares packets rather than coordinates.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Displacement2Facts {
    /// Zero-status snapshot for dx,dy in that order; does not certify a later edited component array.
    pub component_zero: [ZeroKnowledge; 2],
    /// Sole nonzero support axis in a producer-created packet; public edits need not agree with component_zero.
    pub known_axis: Option<Axis2>,
    /// True when both producer-queried components were Zero; false is absence of that proof, not proof of a nonzero displacement. Public edits are not validated.
    pub known_zero: bool,
}

/// Structural zero knowledge for two factors and their product, without multiplying.
///
/// A known-zero factor dominates; two known-nonzero factors prove a nonzero product;
/// all other combinations remain Unknown. Both factor queries run in left/right
/// order even when the first is zero. Public fields can contradict one another after
/// construction; query methods inspect term_zero only and do not revalidate it.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ProductTerm2Facts {
    /// Left/right scalar zero knowledge as queried, even if the first factor is already known zero.
    pub factor_zero: [ZeroKnowledge; 2],
    /// Stored product zero status; queries trust it without rechecking factor_zero.
    pub term_zero: ZeroKnowledge,
}

impl ProductTerm2Facts {
    /// Test whether the stored term_zero is Zero; factor statuses are not rechecked.
    pub const fn known_zero(self) -> bool {
        matches!(self.term_zero, ZeroKnowledge::Zero)
    }

    /// Test whether the stored term_zero is NonZero; factor statuses are not rechecked.
    pub const fn known_nonzero(self) -> bool {
        matches!(self.term_zero, ZeroKnowledge::NonZero)
    }

    /// Test whether the stored term_zero is Unknown; factor fields are not inspected.
    pub const fn unknown_zero(self) -> bool {
        matches!(self.term_zero, ZeroKnowledge::Unknown)
    }
}

/// Array of pairwise-product zero statuses, independent of product signs and cancellation.
///
/// All-zero terms imply a zero sum, but nonzero terms may cancel. Fields and new()
/// accept caller-provided statuses without binding them to factors or checking proofs.
/// Masks truncate to the first 64 terms. Counts scan all TERMS statuses and cast to
/// u32, truncating counts above u32::MAX; all_terms_known_zero inherits that limit.
/// An empty packet has zero masks/counts and reports all terms known zero.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ProductSum2Facts<const TERMS: usize> {
    /// One status per product in original order; all TERMS entries are retained even though masks expose at most 64.
    pub term_zero: [ZeroKnowledge; TERMS],
}

/// Support snapshot for the determinant `(b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x)`.
///
/// Records two displacements and two product statuses without constructing the full
/// determinant or deciding its sign. Producers recognize zero when both terms are
/// known zero, and nonzero when exactly one is known zero and the other nonzero.
/// Two nonzero terms remain undecided even if their cancellation is easy by another
/// method. Public fields are editable; accessors do not validate mutual consistency,
/// source identity, or geometric validity.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Orient2Facts {
    /// Snapshot for b-a, or the first displacement explicitly supplied by the caller.
    pub ab: Displacement2Facts,
    /// Snapshot for c-a, or the second displacement explicitly supplied by the caller.
    pub ac: Displacement2Facts,
    /// Statuses of ab.x*ac.y and ab.y*ac.x in that order; the subtraction sign does not affect individual zero statuses.
    pub determinant_terms: ProductSum2Facts<2>,
    /// Some(true) means known zero, Some(false) means known nonzero, and None means unresolved by support; publicly editable without revalidation.
    pub known_zero: Option<bool>,
}

impl<const TERMS: usize> ProductSum2Facts<TERMS> {
    /// Store the supplied term statuses unchanged, including empty arrays; does not verify underlying factor proofs.
    pub const fn new(term_zero: [ZeroKnowledge; TERMS]) -> Self {
        Self { term_zero }
    }

    /// Return the stored status at index.
    ///
    /// # Panics
    ///
    /// Panics when index>=TERMS, including every index for an empty packet.
    pub const fn term_zero(self, index: usize) -> ZeroKnowledge {
        self.term_zero[index]
    }

    /// Return bits for stored Zero statuses, only at term indices 0..min(TERMS,64); later terms are omitted.
    pub fn known_zero_mask(self) -> u64 {
        term_mask(self.term_zero, ZeroKnowledge::Zero)
    }

    /// Return bits for stored NonZero statuses, only at term indices 0..min(TERMS,64); later terms are omitted.
    pub fn known_nonzero_mask(self) -> u64 {
        term_mask(self.term_zero, ZeroKnowledge::NonZero)
    }

    /// Return bits for stored Unknown statuses, only at term indices 0..min(TERMS,64); later terms are omitted.
    pub fn unknown_zero_mask(self) -> u64 {
        term_mask(self.term_zero, ZeroKnowledge::Unknown)
    }

    /// Count stored Zero statuses across all TERMS entries, then cast usize to u32; counts above u32::MAX truncate, unlike the first-64-term masks.
    pub fn known_zero_count(self) -> u32 {
        self.term_zero
            .into_iter()
            .filter(|status| matches!(status, ZeroKnowledge::Zero))
            .count() as u32
    }

    /// Count stored NonZero statuses across all TERMS entries, then cast usize to u32; counts above u32::MAX truncate, unlike the first-64-term masks.
    pub fn known_nonzero_count(self) -> u32 {
        self.term_zero
            .into_iter()
            .filter(|status| matches!(status, ZeroKnowledge::NonZero))
            .count() as u32
    }

    /// Count stored Unknown statuses across all TERMS entries, then cast usize to u32; counts above u32::MAX truncate, unlike the first-64-term masks.
    pub fn unknown_zero_count(self) -> u32 {
        self.term_zero
            .into_iter()
            .filter(|status| matches!(status, ZeroKnowledge::Unknown))
            .count() as u32
    }

    /// Compare the u32-truncated zero count, converted to usize, with TERMS.
    ///
    /// True for an empty array. For TERMS<=u32::MAX this tests every status; larger
    /// all-zero arrays cannot be recognized by this count-based implementation. This
    /// method does not detect cancellation among terms that are not individually zero.
    pub fn all_terms_known_zero(self) -> bool {
        self.known_zero_count() as usize == TERMS
    }
}

impl Orient2Facts {
    /// Query first-displacement facts, second-displacement facts, then both determinant product terms.
    ///
    /// Coordinates are already-computed dx,dy pairs. Reuses these scalars instead of
    /// rebuilding differences, but product-term queries revisit their zero knowledge.
    /// The retained fact order is intentional. Derives zero/nonzero only from the two
    /// term statuses; no determinant construction, sign refinement, or policy occurs.
    pub fn from_displacements(ab: [&Real; 2], ac: [&Real; 2]) -> Self {
        let ab_facts = Displacement2Facts::from_components(ab);
        let ac_facts = Displacement2Facts::from_components(ac);
        let determinant_terms = product_sum2_facts([[ab[0], ac[1]], [ab[1], ac[0]]]);
        let known_zero = determinant_zero_from_terms(determinant_terms);

        Self {
            ab: ab_facts,
            ac: ac_facts,
            determinant_terms,
            known_zero,
        }
    }

    /// Return the stored known_zero option unchanged, without reconstructing or validating the determinant.
    pub const fn known_zero(self) -> Option<bool> {
        self.known_zero
    }

    /// Negate the stored known_zero boolean inside Some, preserving None; no new scalar proof is computed.
    pub const fn known_nonzero(self) -> Option<bool> {
        match self.known_zero {
            Some(true) => Some(false),
            Some(false) => Some(true),
            None => None,
        }
    }

    /// Return the two stored Some axis fields when present; does not require different axes or validate them against components.
    pub const fn known_axis_pair(self) -> Option<(Axis2, Axis2)> {
        match (self.ab.known_axis, self.ac.known_axis) {
            (Some(ab), Some(ac)) => Some((ab, ac)),
            _ => None,
        }
    }
}

impl Displacement2Facts {
    /// Query dx.zero_status then dy.zero_status without rebuilding a displacement.
    ///
    /// Both known zero set known_zero. Exactly one NonZero and one Zero set the support
    /// axis, regardless of sign/magnitude. Unknown yields no axis proof. These queries
    /// consume structural/cached knowledge without starting a sign-refinement loop.
    pub fn from_components(components: [&Real; 2]) -> Self {
        let component_zero = [components[0].zero_status(), components[1].zero_status()];
        let known_zero = matches!(component_zero, [ZeroKnowledge::Zero, ZeroKnowledge::Zero]);
        let known_axis = match component_zero {
            [ZeroKnowledge::NonZero, ZeroKnowledge::Zero] => Some(Axis2::X),
            [ZeroKnowledge::Zero, ZeroKnowledge::NonZero] => Some(Axis2::Y),
            _ => None,
        };

        Self {
            component_zero,
            known_axis,
            known_zero,
        }
    }

    /// Index the stored component status with Axis2::index (X=0, Y=1), without querying a scalar.
    pub fn component_zero(self, axis: Axis2) -> ZeroKnowledge {
        self.component_zero[axis.index()]
    }

    /// Return bits for stored Zero statuses, with dx at bit 0 and dy at bit 1; known_axis/known_zero fields are not consulted.
    pub fn known_zero_mask(self) -> u8 {
        let mut mask = 0;
        if matches!(self.component_zero[0], ZeroKnowledge::Zero) {
            mask |= Axis2::X.bit();
        }
        if matches!(self.component_zero[1], ZeroKnowledge::Zero) {
            mask |= Axis2::Y.bit();
        }
        mask
    }

    /// Return bits for stored NonZero statuses, with dx at bit 0 and dy at bit 1; known_axis/known_zero fields are not consulted.
    pub fn known_nonzero_mask(self) -> u8 {
        let mut mask = 0;
        if matches!(self.component_zero[0], ZeroKnowledge::NonZero) {
            mask |= Axis2::X.bit();
        }
        if matches!(self.component_zero[1], ZeroKnowledge::NonZero) {
            mask |= Axis2::Y.bit();
        }
        mask
    }

    /// Return bits for stored Unknown statuses, with dx at bit 0 and dy at bit 1; known_axis/known_zero fields are not consulted.
    pub fn unknown_zero_mask(self) -> u8 {
        let mut mask = 0;
        if matches!(self.component_zero[0], ZeroKnowledge::Unknown) {
            mask |= Axis2::X.bit();
        }
        if matches!(self.component_zero[1], ZeroKnowledge::Unknown) {
            mask |= Axis2::Y.bit();
        }
        mask
    }

    /// Count stored Zero statuses from the corresponding two-bit component mask; the result is between zero and two.
    pub fn known_zero_count(self) -> u32 {
        self.known_zero_mask().count_ones()
    }

    /// Count stored NonZero statuses from the corresponding two-bit component mask; the result is between zero and two.
    pub fn known_nonzero_count(self) -> u32 {
        self.known_nonzero_mask().count_ones()
    }

    /// Count stored Unknown statuses from the corresponding two-bit component mask; the result is between zero and two.
    pub fn unknown_zero_count(self) -> u32 {
        self.unknown_zero_mask().count_ones()
    }
}

/// Scan only indices below min(TERMS,64), setting bit i when its stored status equals needle.
///
/// No shifts beyond u64 width occur. Later terms are deliberately absent from the
/// mask, so a zero mask is not an all-terms statement for arrays longer than 64.
fn term_mask<const TERMS: usize>(statuses: [ZeroKnowledge; TERMS], needle: ZeroKnowledge) -> u64 {
    let mut mask = 0;
    let limit = TERMS.min(64);
    for (index, status) in statuses.into_iter().enumerate().take(limit) {
        if status == needle {
            mask |= 1_u64 << index;
        }
    }
    mask
}

/// Return Some(true) for two known-zero terms, Some(false) for one zero and one nonzero, otherwise None.
///
/// Tests all-zero before the one-zero/one-nonzero branch and retains the count-query
/// order. Two nonzero products may cancel, so their difference is not classified.
/// This trusts the supplied status packet and provides no determinant sign.
fn determinant_zero_from_terms(terms: ProductSum2Facts<2>) -> Option<bool> {
    if terms.all_terms_known_zero() {
        Some(true)
    } else if terms.known_zero_count() == 1 && terms.known_nonzero_count() == 1 {
        Some(false)
    } else {
        None
    }
}

/// Construct `[to.x-from.x, to.y-from.y]` with borrowed Real subtraction in axis order.
///
/// Retains exact expressions for reuse by distance/orientation callers. No zero,
/// axis, point-identity, incidence, or topology decision is made here.
pub fn displacement2(from: [&Real; 2], to: [&Real; 2]) -> [Real; 2] {
    crate::trace_dispatch!("hyperlattice_algebra2", "helper", "displacement2");
    [to[0] - from[0], to[1] - from[1]]
}

/// Construct to-from once, query its component zero facts, then discard the temporary scalars.
///
/// Use displacement2 and Displacement2Facts::from_components when differences must
/// also be retained. This returns a detached structural snapshot without refinement
/// or a policy-qualified geometric decision.
pub fn displacement2_facts(from: [&Real; 2], to: [&Real; 2]) -> Displacement2Facts {
    crate::trace_dispatch!("hyperlattice_algebra2", "helper", "displacement2-facts");
    let delta = displacement2(from, to);
    Displacement2Facts::from_components([&delta[0], &delta[1]])
}

/// Query both factor zero statuses in left/right order without constructing their product.
///
/// Either Zero makes the term Zero; two NonZero statuses make it NonZero; otherwise
/// it is Unknown. Does not short-circuit the second query when the first is zero,
/// compare signs, or refine an unknown. Cached scalar proofs can affect the result.
pub fn product_term2_facts(term: [&Real; 2]) -> ProductTerm2Facts {
    crate::trace_dispatch!("hyperlattice_algebra2", "helper", "product-term2-facts");
    let factor_zero = [term[0].zero_status(), term[1].zero_status()];
    let term_zero = if matches!(factor_zero[0], ZeroKnowledge::Zero)
        || matches!(factor_zero[1], ZeroKnowledge::Zero)
    {
        ZeroKnowledge::Zero
    } else if matches!(
        factor_zero,
        [ZeroKnowledge::NonZero, ZeroKnowledge::NonZero]
    ) {
        ZeroKnowledge::NonZero
    } else {
        ZeroKnowledge::Unknown
    };

    ProductTerm2Facts {
        factor_zero,
        term_zero,
    }
}

/// Map terms in array order through product_term2_facts, retaining each product status.
///
/// No products or sum are constructed. This is support metadata for choosing algebra
/// paths, not a test for cancellation or the sign of a sum. Empty input returns an
/// empty packet; masks/counts retain the limits documented on ProductSum2Facts.
pub fn product_sum2_facts<const TERMS: usize>(
    terms: [[&Real; 2]; TERMS],
) -> ProductSum2Facts<TERMS> {
    crate::trace_dispatch!("hyperlattice_algebra2", "helper", "product-sum2-facts");
    ProductSum2Facts::new(terms.map(|term| product_term2_facts(term).term_zero))
}

/// Construct b-a first and c-a second, then query their determinant support snapshot.
///
/// Can identify zero terms in duplicate-anchor/axis-collinear cases and a nonzero
/// single term for perpendicular axes. Does not decide cancellation between two
/// nonzero products or sidedness. Use orient2_expr separately for the determinant;
/// the returned packet does not retain the temporary displacement scalars.
pub fn orient2_expr_facts(a: [&Real; 2], b: [&Real; 2], c: [&Real; 2]) -> Orient2Facts {
    crate::trace_dispatch!("hyperlattice_algebra2", "helper", "orient2-expr-facts");
    let [abx, aby] = displacement2(a, b);
    let [acx, acy] = displacement2(a, c);
    Orient2Facts::from_displacements([&abx, &aby], [&acx, &acy])
}

/// Construct the sum of `terms[i][0]*terms[i][1]`, adding for true and subtracting for false.
///
/// Delegates to the lattice RealKernelExt implementation, not Hyperreal's generic
/// signed_product_sum recognizer. It scans definite-zero factors in order; zero,
/// one, or two retained terms use specialized routes. With three or more it forwards
/// the original array, including skipped zero terms, to the active reducer. Empty
/// input returns exact zero. Nonzero term counts here include unknown factors.
///
/// The active reducer tries exact-rational aggregation before ordered generic
/// multiplication/addition. No zero-status packet is accepted or consumed, and no
/// policy, sign decision, total-work budget, or certainty receipt is added.
pub fn signed_product_sum2<const TERMS: usize>(
    positive_terms: [bool; TERMS],
    terms: [[&Real; 2]; TERMS],
) -> Real {
    crate::trace_dispatch!("hyperlattice_algebra2", "helper", "signed-product-sum2");
    Real::signed_product_sum2(positive_terms, terms)
}

/// Add every pairwise product by passing all-true signs to signed_product_sum2.
///
/// Positive refers to addition signs, not factor or result nonnegativity. This is
/// useful for dot products and squared lengths; empty input returns exact zero.
pub fn positive_product_sum2<const TERMS: usize>(terms: [[&Real; 2]; TERMS]) -> Real {
    crate::trace_dispatch!("hyperlattice_algebra2", "helper", "positive-product-sum2");
    signed_product_sum2([true; TERMS], terms)
}

/// Construct `left.x*right.y-left.y*right.x` through the signed pairwise-product reducer.
///
/// This exterior product is oriented parallelogram area in Cartesian coordinates.
/// It is a scalar expression, not a sign or sidedness decision; swapping operands
/// negates the mathematical value. No normalized vectors or unit scale are required.
pub fn wedge2(left: [&Real; 2], right: [&Real; 2]) -> Real {
    crate::trace_dispatch!("hyperlattice_algebra2", "helper", "wedge2");
    signed_product_sum2([true, false], [[left[0], right[1]], [left[1], right[0]]])
}

/// Construct `left.x*right.x+left.y*right.y` from borrowed coordinate pairs.
///
/// Provides the dot expression without requiring Vector2 carriers. Delegates to
/// positive_product_sum2 and its definite-zero scan/exact-rational reducer; does
/// not classify orthogonality, magnitude, or the sign of the result.
pub fn dot2(left: [&Real; 2], right: [&Real; 2]) -> Real {
    crate::trace_dispatch!("hyperlattice_algebra2", "helper", "dot2");
    positive_product_sum2([[left[0], right[0]], [left[1], right[1]]])
}

/// Construct `x*x+y*y` from an existing coordinate pair without taking a square root.
///
/// Allows reuse of displacement scalars for distance/solver expressions. The exact
/// mathematical result is nonnegative, but this API returns the expression without
/// a sign, zero, or topology receipt and does not imply an external cache hit.
pub fn squared_norm2(vector: [&Real; 2]) -> Real {
    crate::trace_dispatch!("hyperlattice_algebra2", "helper", "squared-norm2");
    positive_product_sum2([[vector[0], vector[0]], [vector[1], vector[1]]])
}

/// Construct b-a, then its squared norm, preserving the exact polynomial expression.
///
/// No square root or distance comparison occurs. Callers comparing distances still
/// need the appropriate predicate decision; this expression carries no tolerance,
/// point-identity proof, or approximation guarantee.
pub fn squared_distance2(a: [&Real; 2], b: [&Real; 2]) -> Real {
    crate::trace_dispatch!("hyperlattice_algebra2", "helper", "squared-distance2");
    let [dx, dy] = displacement2(a, b);
    squared_norm2([&dx, &dy])
}

/// Construct the exterior product of b-a and c-a, in that displacement order.
///
/// The mathematical value is twice the signed triangle area: positive for a
/// counterclockwise nondegenerate triple in the usual x-right/y-up convention.
/// Construction does not determine that sign or classify degeneracy; hyperlimit
/// owns predicate-policy decisions and their evidence.
pub fn orient2_expr(a: [&Real; 2], b: [&Real; 2], c: [&Real; 2]) -> Real {
    crate::trace_dispatch!("hyperlattice_algebra2", "helper", "orient2-expr");
    let [abx, aby] = displacement2(a, b);
    let [acx, acy] = displacement2(a, c);
    wedge2([&abx, &aby], [&acx, &acy])
}

/// Finite exact arithmetic/support regressions; do not establish general symbolic decisiveness, resource bounds, or dispatch counts.
#[cfg(test)]
mod tests {
    use super::*;

    /// Local test alias for the actual exact scalar carrier; no alternative arithmetic implementation is used.
    type S = Real;

    /// Lift an i32 fixture coefficient exactly into the local Real alias.
    fn s(value: i32) -> S {
        value.into()
    }

    /// Assert the exact wedge of (3,4) and (5,6) is -2.
    #[test]
    fn wedge2_builds_signed_area_expression() {
        let left = [s(3), s(4)];
        let right = [s(5), s(6)];

        assert_eq!(wedge2([&left[0], &left[1]], [&right[0], &right[1]]), s(-2));
    }

    /// Assert the exact displacement from (1,2) to (4,6) is (3,4).
    #[test]
    fn displacement2_builds_coordinate_difference() {
        let from = [s(1), s(2)];
        let to = [s(4), s(6)];

        assert_eq!(
            displacement2([&from[0], &from[1]], [&to[0], &to[1]]),
            [s(3), s(4)]
        );
    }

    /// Check a horizontal displacement has X support and complementary masks, and a duplicate point has two zero components.
    #[test]
    fn displacement2_facts_expose_axis_and_zero_masks() {
        let from = [s(1), s(2)];
        let horizontal = [s(4), s(2)];
        let same = [s(1), s(2)];

        let horizontal_facts =
            displacement2_facts([&from[0], &from[1]], [&horizontal[0], &horizontal[1]]);
        assert_eq!(horizontal_facts.known_axis, Some(Axis2::X));
        assert_eq!(horizontal_facts.known_zero_mask(), Axis2::Y.bit());
        assert_eq!(horizontal_facts.known_nonzero_mask(), Axis2::X.bit());

        let same_facts = displacement2_facts([&from[0], &from[1]], [&same[0], &same[1]]);
        assert!(same_facts.known_zero);
        assert_eq!(same_facts.known_zero_count(), 2);
    }

    /// Check two positive products sum to 41 and an ordered two-product difference is -1 using exact integer expectations.
    #[test]
    fn product_sum_helpers_preserve_short_exact_expressions() {
        let a = s(2);
        let b = s(3);
        let c = s(5);
        let d = s(7);

        assert_eq!(positive_product_sum2([[&a, &b], [&c, &d]]), s(41));
        assert_eq!(
            signed_product_sum2([true, false], [[&a, &d], [&b, &c]]),
            s(-1)
        );
    }

    /// Check one zero and one nonzero product yield the expected statuses/masks and no unknown terms; does not decide cancellation.
    #[test]
    fn product_sum_facts_track_structurally_zero_terms() {
        let zero = s(0);
        let one = s(1);
        let two = s(2);
        let three = s(3);

        let facts = product_sum2_facts([[&zero, &two], [&one, &three]]);
        assert_eq!(facts.term_zero(0), ZeroKnowledge::Zero);
        assert_eq!(facts.term_zero(1), ZeroKnowledge::NonZero);
        assert_eq!(facts.known_zero_mask(), 0b01);
        assert_eq!(facts.known_nonzero_mask(), 0b10);
        assert_eq!(facts.unknown_zero_count(), 0);
        assert!(!facts.all_terms_known_zero());
    }

    /// Check same-axis displacements give known zero, perpendicular axes give known nonzero, and a dense fixture remains unclassified by support alone.
    #[test]
    fn orient2_expr_facts_track_axis_collinear_and_perpendicular_cases() {
        let a = [s(0), s(0)];
        let x_axis = [s(4), s(0)];
        let farther_x_axis = [s(9), s(0)];
        let y_axis = [s(0), s(3)];
        let dense = [s(2), s(5)];
        let other_dense = [s(9), s(7)];

        let collinear = orient2_expr_facts(
            [&a[0], &a[1]],
            [&x_axis[0], &x_axis[1]],
            [&farther_x_axis[0], &farther_x_axis[1]],
        );
        assert_eq!(collinear.known_axis_pair(), Some((Axis2::X, Axis2::X)));
        assert_eq!(collinear.known_zero(), Some(true));
        assert_eq!(collinear.determinant_terms.known_zero_count(), 2);

        let perpendicular = orient2_expr_facts(
            [&a[0], &a[1]],
            [&x_axis[0], &x_axis[1]],
            [&y_axis[0], &y_axis[1]],
        );
        assert_eq!(perpendicular.known_axis_pair(), Some((Axis2::X, Axis2::Y)));
        assert_eq!(perpendicular.known_nonzero(), Some(true));
        assert_eq!(perpendicular.determinant_terms.known_zero_count(), 1);
        assert_eq!(perpendicular.determinant_terms.known_nonzero_count(), 1);

        let dense_case = orient2_expr_facts(
            [&a[0], &a[1]],
            [&dense[0], &dense[1]],
            [&other_dense[0], &other_dense[1]],
        );
        assert_eq!(dense_case.known_axis_pair(), None);
        assert_eq!(dense_case.known_zero(), None);
    }

    /// Check the exact squared norm of supplied (3,4) components is 25; the test does not measure or assert cache reuse.
    #[test]
    fn squared_norm2_reuses_cached_displacement() {
        let dx = s(3);
        let dy = s(4);

        assert_eq!(squared_norm2([&dx, &dy]), s(25));
    }

    /// Assert the exact determinant of (0,0),(4,0),(0,3) is +12, independently of any sign-policy wrapper.
    #[test]
    fn orient2_expr_matches_counter_clockwise_triangle() {
        let a = [s(0), s(0)];
        let b = [s(4), s(0)];
        let c = [s(0), s(3)];

        assert_eq!(
            orient2_expr([&a[0], &a[1]], [&b[0], &b[1]], [&c[0], &c[1]]),
            s(12)
        );
    }

    /// Assert the squared distance between (1,2) and (4,6) is exactly 25; no dispatch count is asserted.
    #[test]
    fn squared_distance2_avoids_square_root() {
        let a = [s(1), s(2)];
        let b = [s(4), s(6)];

        assert_eq!(squared_distance2([&a[0], &a[1]], [&b[0], &b[1]]), s(25));
    }
}
