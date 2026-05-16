//! Two-dimensional scalar algebra helpers.
//!
//! These helpers produce exact scalar expressions and deliberately do not
//! classify topology. Callers that need sidedness, intersection, or containment
//! decisions should pass the resulting scalar to `hyperlimit` so predicate
//! policy and provenance stay at the predicate layer.

use crate::{Axis2, Real, RealKernelExt, ZeroStatus};

/// Cheap structural facts known about a 2D displacement.
///
/// These facts describe coordinate differences, not geometry topology. A
/// known-zero displacement can help a caller skip work or choose an
/// axis-specialized exact kernel, but final duplicate, incidence, sidedness,
/// or containment decisions should still be certified by `hyperlimit`.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Displacement2Facts {
    /// Zero status for `[dx, dy]`.
    pub component_zero: [ZeroStatus; 2],
    /// Axis occupied by a known nonzero component when the other component is
    /// known zero.
    pub known_axis: Option<Axis2>,
    /// Whether both displacement components are known zero.
    pub known_zero: bool,
}

/// Cheap zero facts for one pairwise product `left * right`.
///
/// This records only structural zero knowledge for the product term. It does
/// not simplify symbolic expressions, compare magnitudes, or decide signs.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ProductTerm2Facts {
    /// Zero status for the two product factors.
    pub factor_zero: [ZeroStatus; 2],
    /// Zero status for the product term itself.
    pub term_zero: ZeroStatus,
}

impl ProductTerm2Facts {
    /// Return whether the product term is known to be exactly zero.
    pub const fn known_zero(self) -> bool {
        matches!(self.term_zero, ZeroStatus::Zero)
    }

    /// Return whether the product term is known not to be zero.
    pub const fn known_nonzero(self) -> bool {
        matches!(self.term_zero, ZeroStatus::NonZero)
    }

    /// Return whether the product term zero status is unknown.
    pub const fn unknown_zero(self) -> bool {
        matches!(self.term_zero, ZeroStatus::Unknown)
    }
}

/// Cheap zero facts for a short sum of pairwise products.
///
/// These facts are useful next to [`signed_product_sum2`] and
/// [`positive_product_sum2`]. They describe which product terms are
/// structurally zero, not whether the whole sum cancels to zero.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ProductSum2Facts<const TERMS: usize> {
    /// Zero status for each pairwise product term.
    pub term_zero: [ZeroStatus; TERMS],
}

/// Cheap structural facts for a 2D orientation determinant.
///
/// The determinant itself is
/// `(b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)`.
/// This type records the displacement and product-term zero structure behind
/// that expression without deciding the determinant sign. It is intended for
/// predicate layers that want to select sparse or axis-specialized exact
/// kernels before constructing every scalar term. This retained determinant
/// shape follows Yap's exact-geometric-computation separation of object facts
/// from predicate decisions; see Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Orient2Facts {
    /// Structural facts for `b - a`.
    pub ab: Displacement2Facts,
    /// Structural facts for `c - a`.
    pub ac: Displacement2Facts,
    /// Zero status of the two determinant product terms.
    pub determinant_terms: ProductSum2Facts<2>,
    /// Whether zero/nonzero determinant status is known from structural facts.
    ///
    /// `Some(true)` means the determinant is known to be exactly zero.
    /// `Some(false)` means cancellation cannot make the determinant zero
    /// because exactly one signed product term is structurally nonzero.
    pub known_zero: Option<bool>,
}

impl<const TERMS: usize> ProductSum2Facts<TERMS> {
    /// Build product-sum facts from term facts.
    pub const fn new(term_zero: [ZeroStatus; TERMS]) -> Self {
        Self { term_zero }
    }

    /// Return the zero status for one product term.
    ///
    /// # Panics
    ///
    /// Panics when `index >= TERMS`.
    pub const fn term_zero(self, index: usize) -> ZeroStatus {
        self.term_zero[index]
    }

    /// Return a bit mask of product terms known to be zero.
    ///
    /// Bits above 63 are not represented. This is sufficient for the fixed
    /// short determinant, distance, and cofactor expressions this module is
    /// meant to expose.
    pub fn known_zero_mask(self) -> u64 {
        term_mask(self.term_zero, ZeroStatus::Zero)
    }

    /// Return a bit mask of product terms known to be nonzero.
    ///
    /// Bits above 63 are not represented.
    pub fn known_nonzero_mask(self) -> u64 {
        term_mask(self.term_zero, ZeroStatus::NonZero)
    }

    /// Return a bit mask of product terms with unknown zero status.
    ///
    /// Bits above 63 are not represented.
    pub fn unknown_zero_mask(self) -> u64 {
        term_mask(self.term_zero, ZeroStatus::Unknown)
    }

    /// Count product terms known to be exactly zero.
    pub fn known_zero_count(self) -> u32 {
        self.term_zero
            .into_iter()
            .filter(|status| matches!(status, ZeroStatus::Zero))
            .count() as u32
    }

    /// Count product terms known to be nonzero.
    pub fn known_nonzero_count(self) -> u32 {
        self.term_zero
            .into_iter()
            .filter(|status| matches!(status, ZeroStatus::NonZero))
            .count() as u32
    }

    /// Count product terms with unknown zero status.
    pub fn unknown_zero_count(self) -> u32 {
        self.term_zero
            .into_iter()
            .filter(|status| matches!(status, ZeroStatus::Unknown))
            .count() as u32
    }

    /// Return whether every product term is known to be zero.
    pub fn all_terms_known_zero(self) -> bool {
        self.known_zero_count() as usize == TERMS
    }
}

impl Orient2Facts {
    /// Build orientation determinant facts from already-computed displacements.
    ///
    /// Callers that already materialized `b - a` and `c - a` can use this form
    /// to avoid recomputing displacement scalars while still preserving the
    /// compact two-term determinant shape. The sparse-product motivation is the
    /// same fixed-size version of Gustavson's sparse multiplication idea:
    /// Gustavson, "Two Fast Algorithms for Sparse Matrices: Multiplication and
    /// Permuted Transposition," *ACM Transactions on Mathematical Software*
    /// 4.3 (1978).
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

    /// Return whether the orientation determinant is structurally known to be
    /// zero or nonzero.
    pub const fn known_zero(self) -> Option<bool> {
        self.known_zero
    }

    /// Return whether the determinant is known to be nonzero.
    pub const fn known_nonzero(self) -> Option<bool> {
        match self.known_zero {
            Some(true) => Some(false),
            Some(false) => Some(true),
            None => None,
        }
    }

    /// Return the known support-axis pair for `b - a` and `c - a`, if both
    /// displacements are axis-certified and non-degenerate.
    pub const fn known_axis_pair(self) -> Option<(Axis2, Axis2)> {
        match (self.ab.known_axis, self.ac.known_axis) {
            (Some(ab), Some(ac)) => Some((ab, ac)),
            _ => None,
        }
    }
}

impl Displacement2Facts {
    /// Build displacement facts from already-computed `[dx, dy]` components.
    ///
    /// This constructor exists so higher crates can cache exact displacements
    /// once and reuse their structural facts without rebuilding scalar
    /// differences. The retained-object strategy follows Yap's exact geometric
    /// computation guidance: exploit cheap object structure before lower-level
    /// scalar evaluation, while keeping final predicate decisions separate. See
    /// Yap, "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7.1-2 (1997).
    pub fn from_components(components: [&Real; 2]) -> Self {
        let component_zero = [components[0].zero_status(), components[1].zero_status()];
        let known_zero = matches!(component_zero, [ZeroStatus::Zero, ZeroStatus::Zero]);
        let known_axis = match component_zero {
            [ZeroStatus::NonZero, ZeroStatus::Zero] => Some(Axis2::X),
            [ZeroStatus::Zero, ZeroStatus::NonZero] => Some(Axis2::Y),
            _ => None,
        };

        Self {
            component_zero,
            known_axis,
            known_zero,
        }
    }

    /// Return the zero status for one displacement component.
    pub fn component_zero(self, axis: Axis2) -> ZeroStatus {
        self.component_zero[axis.index()]
    }

    /// Return a bit mask of components known to be exactly zero.
    ///
    /// Bit 0 is `dx` and bit 1 is `dy`.
    pub fn known_zero_mask(self) -> u8 {
        let mut mask = 0;
        if matches!(self.component_zero[0], ZeroStatus::Zero) {
            mask |= Axis2::X.bit();
        }
        if matches!(self.component_zero[1], ZeroStatus::Zero) {
            mask |= Axis2::Y.bit();
        }
        mask
    }

    /// Return a bit mask of components known to be nonzero.
    pub fn known_nonzero_mask(self) -> u8 {
        let mut mask = 0;
        if matches!(self.component_zero[0], ZeroStatus::NonZero) {
            mask |= Axis2::X.bit();
        }
        if matches!(self.component_zero[1], ZeroStatus::NonZero) {
            mask |= Axis2::Y.bit();
        }
        mask
    }

    /// Return a bit mask of components whose zero status is unknown.
    pub fn unknown_zero_mask(self) -> u8 {
        let mut mask = 0;
        if matches!(self.component_zero[0], ZeroStatus::Unknown) {
            mask |= Axis2::X.bit();
        }
        if matches!(self.component_zero[1], ZeroStatus::Unknown) {
            mask |= Axis2::Y.bit();
        }
        mask
    }

    /// Count components known to be exactly zero.
    pub fn known_zero_count(self) -> u32 {
        self.known_zero_mask().count_ones()
    }

    /// Count components known to be nonzero.
    pub fn known_nonzero_count(self) -> u32 {
        self.known_nonzero_mask().count_ones()
    }

    /// Count components whose zero status is unknown.
    ///
    /// Structural-dispatch note: count helpers let triangulation and curve
    /// layers pick sparse exact kernels without depending on this crate's mask
    /// layout. Sparse product-sum dispatch is motivated by Gustavson, "Two Fast
    /// Algorithms for Sparse Matrices: Multiplication and Permuted
    /// Transposition," *ACM Transactions on Mathematical Software* 4.3 (1978).
    pub fn unknown_zero_count(self) -> u32 {
        self.unknown_zero_mask().count_ones()
    }
}

fn term_mask<const TERMS: usize>(statuses: [ZeroStatus; TERMS], needle: ZeroStatus) -> u64 {
    let mut mask = 0;
    let limit = TERMS.min(64);
    for (index, status) in statuses.into_iter().enumerate().take(limit) {
        if status == needle {
            mask |= 1_u64 << index;
        }
    }
    mask
}

fn determinant_zero_from_terms(terms: ProductSum2Facts<2>) -> Option<bool> {
    if terms.all_terms_known_zero() {
        Some(true)
    } else if terms.known_zero_count() == 1 && terms.known_nonzero_count() == 1 {
        Some(false)
    } else {
        None
    }
}

/// Returns the 2D displacement vector `to - from` as `[dx, dy]`.
///
/// This is pure coordinate algebra: it constructs reusable exact scalar
/// differences without deciding whether the displacement is zero, horizontal,
/// or otherwise geometrically special. Callers that need a certified equality
/// or incidence decision should ask `hyperlimit`.
pub fn displacement2(from: [&Real; 2], to: [&Real; 2]) -> [Real; 2] {
    crate::trace_dispatch!("hyperlattice_algebra2", "helper", "displacement2");
    [to[0] - from[0], to[1] - from[1]]
}

/// Return structural facts about the 2D displacement `to - from`.
///
/// This helper computes the exact coordinate differences only to query their
/// cheap zero facts. Use [`displacement2`] when the caller also needs the
/// scalar differences themselves.
pub fn displacement2_facts(from: [&Real; 2], to: [&Real; 2]) -> Displacement2Facts {
    crate::trace_dispatch!("hyperlattice_algebra2", "helper", "displacement2-facts");
    let delta = displacement2(from, to);
    Displacement2Facts::from_components([&delta[0], &delta[1]])
}

/// Return structural zero facts for one pairwise product term.
pub fn product_term2_facts(term: [&Real; 2]) -> ProductTerm2Facts {
    crate::trace_dispatch!("hyperlattice_algebra2", "helper", "product-term2-facts");
    let factor_zero = [term[0].zero_status(), term[1].zero_status()];
    let term_zero = if matches!(factor_zero[0], ZeroStatus::Zero)
        || matches!(factor_zero[1], ZeroStatus::Zero)
    {
        ZeroStatus::Zero
    } else if matches!(factor_zero, [ZeroStatus::NonZero, ZeroStatus::NonZero]) {
        ZeroStatus::NonZero
    } else {
        ZeroStatus::Unknown
    };

    ProductTerm2Facts {
        factor_zero,
        term_zero,
    }
}

/// Return structural zero facts for a short sum of pairwise products.
///
/// Product-sum facts let callers choose sparse exact kernels without expanding
/// every product first. This follows the sparse-product observation in
/// Gustavson, "Two Fast Algorithms for Sparse Matrices: Multiplication and
/// Permuted Transposition," *ACM Transactions on Mathematical Software* 4.3
/// (1978), while preserving Bareiss-style delayed normalization for the
/// surviving exact product sum.
pub fn product_sum2_facts<const TERMS: usize>(
    terms: [[&Real; 2]; TERMS],
) -> ProductSum2Facts<TERMS> {
    crate::trace_dispatch!("hyperlattice_algebra2", "helper", "product-sum2-facts");
    ProductSum2Facts::new(terms.map(|term| product_term2_facts(term).term_zero))
}

/// Return structural facts for the orientation determinant of `a`, `b`, `c`.
///
/// This helper computes the two anchored displacement vectors once and records
/// the zero structure of the two signed product terms. The result can identify
/// duplicate-anchor and axis-collinear cases structurally, and can identify
/// perpendicular axis cases as nonzero, but it deliberately leaves sidedness to
/// `hyperlimit`. The determinant expression itself is still available through
/// [`orient2_expr`].
pub fn orient2_expr_facts(a: [&Real; 2], b: [&Real; 2], c: [&Real; 2]) -> Orient2Facts {
    crate::trace_dispatch!("hyperlattice_algebra2", "helper", "orient2-expr-facts");
    let [abx, aby] = displacement2(a, b);
    let [acx, acy] = displacement2(a, c);
    Orient2Facts::from_displacements([&abx, &aby], [&acx, &acy])
}

/// Returns a short signed sum of pairwise products.
///
/// Each term is `terms[i][0] * terms[i][1]`; `positive_terms[i]` selects
/// whether that product is added or subtracted. This exposes the exact reducer
/// used by small determinants and cofactors without exposing kernel internals.
/// Real kernels can prune structurally zero factors and share exact-rational
/// denominator work, following Bareiss-style fraction-free/delayed
/// normalization. See Bareiss, "Sylvester's Identity and Multistep
/// Integer-Preserving Gaussian Elimination," *Mathematics of Computation*
/// 22.103 (1968).
///
/// Structural-dispatch note: callers that already carry zero masks or
/// exact-rational/dyadic facts can use this helper to preserve the compact
/// product-sum shape instead of materializing independent products and a
/// left-deep addition tree.
pub fn signed_product_sum2<const TERMS: usize>(
    positive_terms: [bool; TERMS],
    terms: [[&Real; 2]; TERMS],
) -> Real {
    crate::trace_dispatch!("hyperlattice_algebra2", "helper", "signed-product-sum2");
    Real::signed_product_sum2(positive_terms, terms)
}

/// Returns a short positive sum of pairwise products.
///
/// This is a convenience wrapper around [`signed_product_sum2`] for dot-like
/// and squared-length expressions.
pub fn positive_product_sum2<const TERMS: usize>(terms: [[&Real; 2]; TERMS]) -> Real {
    crate::trace_dispatch!("hyperlattice_algebra2", "helper", "positive-product-sum2");
    signed_product_sum2([true; TERMS], terms)
}

/// Returns the 2D exterior product `left.x * right.y - left.y * right.x`.
///
/// This is the scalar expression behind planar orientation and signed area.
/// It is kept in `hyperlattice` because it is pure algebra over two vectors,
/// not a predicate decision. The implementation routes the short determinant
/// through the Real signed-product reducer so exact rationals can share
/// denominator work, following the fraction-free/delayed-normalization
/// strategy of Bareiss, "Sylvester's Identity and Multistep Integer-Preserving
/// Gaussian Elimination," *Mathematics of Computation* 22.103 (1968).
pub fn wedge2(left: [&Real; 2], right: [&Real; 2]) -> Real {
    crate::trace_dispatch!("hyperlattice_algebra2", "helper", "wedge2");
    signed_product_sum2([true, false], [[left[0], right[1]], [left[1], right[0]]])
}

/// Returns the 2D dot product `left.x * right.x + left.y * right.y`.
///
/// This helper is useful for curve, solver, and projection code that only owns
/// coordinates, not a [`Vector2`](crate::Vector2). It preserves exact scalar
/// structure and prunes structurally zero lanes before calling the Real kernel
/// reducer.
pub fn dot2(left: [&Real; 2], right: [&Real; 2]) -> Real {
    crate::trace_dispatch!("hyperlattice_algebra2", "helper", "dot2");
    positive_product_sum2([[left[0], right[0]], [left[1], right[1]]])
}

/// Returns the squared 2D norm `x * x + y * y`.
///
/// This helper exists for callers that already cached a displacement vector
/// and want to compare lengths without rebuilding coordinate differences or
/// constructing a square root. It is pure algebra; nearest-candidate and
/// topology decisions still belong in `hyperlimit` predicate helpers.
pub fn squared_norm2(vector: [&Real; 2]) -> Real {
    crate::trace_dispatch!("hyperlattice_algebra2", "helper", "squared-norm2");
    positive_product_sum2([[vector[0], vector[0]], [vector[1], vector[1]]])
}

/// Returns the squared 2D distance between two points.
///
/// No square root is taken. Keeping this as a polynomial lets exact callers
/// compare squared distances or feed residuals into solvers without forcing
/// approximation.
pub fn squared_distance2(a: [&Real; 2], b: [&Real; 2]) -> Real {
    crate::trace_dispatch!("hyperlattice_algebra2", "helper", "squared-distance2");
    let [dx, dy] = displacement2(a, b);
    squared_norm2([&dx, &dy])
}

/// Builds the exact scalar expression for the orientation determinant of
/// points `a`, `b`, and `c`.
///
/// The return value is the determinant
/// `(b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)`.
/// `hyperlattice` stops at expression construction; `hyperlimit` owns the
/// exact sign decision. Keeping this split follows Yap's exact-geometric
/// computation boundary: algebraic objects may carry structure, while geometric
/// decisions live in a predicate layer. See Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997).
pub fn orient2_expr(a: [&Real; 2], b: [&Real; 2], c: [&Real; 2]) -> Real {
    crate::trace_dispatch!("hyperlattice_algebra2", "helper", "orient2-expr");
    let [abx, aby] = displacement2(a, b);
    let [acx, acy] = displacement2(a, c);
    wedge2([&abx, &aby], [&acx, &acy])
}

#[cfg(test)]
mod tests {
    use super::*;

    type S = Real;

    fn s(value: i32) -> S {
        value.into()
    }

    #[test]
    fn wedge2_builds_signed_area_expression() {
        let left = [s(3), s(4)];
        let right = [s(5), s(6)];

        assert_eq!(wedge2([&left[0], &left[1]], [&right[0], &right[1]]), s(-2));
    }

    #[test]
    fn displacement2_builds_coordinate_difference() {
        let from = [s(1), s(2)];
        let to = [s(4), s(6)];

        assert_eq!(
            displacement2([&from[0], &from[1]], [&to[0], &to[1]]),
            [s(3), s(4)]
        );
    }

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

    #[test]
    fn product_sum_facts_track_structurally_zero_terms() {
        let zero = s(0);
        let one = s(1);
        let two = s(2);
        let three = s(3);

        let facts = product_sum2_facts([[&zero, &two], [&one, &three]]);
        assert_eq!(facts.term_zero(0), ZeroStatus::Zero);
        assert_eq!(facts.term_zero(1), ZeroStatus::NonZero);
        assert_eq!(facts.known_zero_mask(), 0b01);
        assert_eq!(facts.known_nonzero_mask(), 0b10);
        assert_eq!(facts.unknown_zero_count(), 0);
        assert!(!facts.all_terms_known_zero());
    }

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

    #[test]
    fn squared_norm2_reuses_cached_displacement() {
        let dx = s(3);
        let dy = s(4);

        assert_eq!(squared_norm2([&dx, &dy]), s(25));
    }

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

    #[test]
    fn squared_distance2_avoids_square_root() {
        let a = [s(1), s(2)];
        let b = [s(4), s(6)];

        assert_eq!(squared_distance2([&a[0], &a[1]], [&b[0], &b[1]]), s(25));
    }
}
