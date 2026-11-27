//! Division-free plane intersections and raw homogeneous coordinate carriers.
//!
//! Constructs exact Real expressions for cofactor points and Pluecker lines while
//! leaving rank, nonzero-carrier validity, incidence decisions, and affine existence
//! to consumers. An all-zero tuple can be returned for a degenerate construction;
//! it is not a valid projective point/line or a selected member of an intersection.
//!
//! No constructor normalizes a homogeneous scale or validates a Pluecker relation.
//! Derived equality compares corresponding stored coordinates with Real equality,
//! not equivalence under a nonzero common scale. Public field changes alter the
//! represented object; fact queries describe current coordinates without attaching
//! construction provenance, policy receipts, or topology evidence.
//!
//! Rational sparse rows and dyadic affine quotients use scalar-owned specializations
//! before the general expressions. Arithmetic stays with Hyperreal. Fixed tuple
//! sizes do not bound nested scalar work, coefficient growth, or retained memory;
//! these APIs provide no separate work budget or cancellation argument.

use crate::{BlasResult, Point3, Real};

/// Borrowed coefficients of the homogeneous equation n·(x,y,z)+offset*w=0.
///
/// The affine chart w=1 gives n·point+offset=0. Implementors supply consistent
/// coefficients across reads; the trait does not require a unit/nonzero normal,
/// validate a plane, or bind coefficients to a proof of geometric identity.
pub trait Plane3Coefficients {
    /// Borrow the normal coefficients consistently for all reads of this plane.
    fn normal(&self) -> &Point3;

    /// Borrow the additive coefficient of w in the homogeneous equation.
    fn offset(&self) -> &Real;
}

/// Unnormalized Real coefficients for an affine or projective plane equation.
///
/// A zero normal and nonzero offset describe the projective plane at infinity and
/// an unsatisfiable affine equation; all-zero coefficients do not define a projective
/// plane. Both are accepted. Derived equality is coefficientwise, not equality of
/// plane images under rescaling.
#[derive(Clone, Debug, PartialEq)]
pub struct ProjectivePlane3 {
    /// Coefficients n of the plane equation, stored in a Point3 without unit-length or nonzero validation.
    pub normal: Point3,
    /// Additive coefficient of w; in the affine chart it is the constant term.
    pub offset: Real,
}

impl ProjectivePlane3 {
    /// Store the supplied plane coefficients unchanged, accepting zero/degenerate equations.
    pub const fn new(normal: Point3, offset: Real) -> Self {
        Self { normal, offset }
    }
}

impl Plane3Coefficients for ProjectivePlane3 {
    /// Borrow the stored normal without normalization or validation.
    fn normal(&self) -> &Point3 {
        &self.normal
    }

    /// Borrow the stored offset without arithmetic or validation.
    fn offset(&self) -> &Real {
        &self.offset
    }
}

/// Unnormalized homogeneous tuple `(x:y:z:w)`, without a nonzero-carrier certificate.
///
/// For nonzero w the affine image is (x/w,y/w,z/w). A nonzero tuple with w=0 is
/// at infinity; the all-zero tuple is invalid as a projective point but remains
/// representable. Equality compares coordinates rather than homogeneous equivalence.
/// No incidence/source record accompanies these four publicly mutable Reals.
#[derive(Clone, Debug, PartialEq)]
pub struct HomogeneousPoint3 {
    /// First homogeneous numerator; not an affine coordinate until divided by a valid w.
    pub x: Real,
    /// Second homogeneous numerator, sharing the stored scale w.
    pub y: Real,
    /// Third homogeneous numerator, sharing the stored scale w.
    pub z: Real,
    /// Homogeneous weight; zero is accepted and cannot serve as an affine divisor.
    pub w: Real,
}

impl HomogeneousPoint3 {
    /// Store the four coordinates unchanged, including all-zero tuples and weights at infinity.
    pub const fn new(x: Real, y: Real, z: Real, w: Real) -> Self {
        Self { x, y, z, w }
    }

    /// Borrow the current x,y,z,w fields in that order without arithmetic or validation.
    pub fn coordinates(&self) -> [&Real; 4] {
        [&self.x, &self.y, &self.z, &self.w]
    }

    /// Aggregate exact-rational structural metadata for the current x,y,z,w fields; this does not certify a nonzero point or affine existence.
    pub fn coordinate_facts(&self) -> crate::RealExactSetFacts {
        Real::exact_set_facts(self.coordinates())
    }

    /// Construct affine quotients when the current weight can be used as a scalar divisor.
    ///
    /// If all four fields expose exact rationals and w=1, clones x,y,z. Otherwise an
    /// all-dyadic tuple uses Hyperreal's known-dyadic quotient kernel for x,y,z in order.
    /// Every other representation uses borrowed Real division in the same order.
    /// No floating conversion or projective scale normalization occurs.
    ///
    /// Propagates the first quotient error, including DivideByZero for exact zero w;
    /// general division may need the scalar layer's nonzero reasoning and can fail when
    /// it cannot establish the required domain. This is not the stricter structural-only
    /// reciprocal_checked API. A successful Point3 carries the resulting expressions,
    /// not a predicate certainty receipt or source-incidence proof. No partial point is
    /// returned on failure, and no separate total-work bound is enforced.
    pub fn to_affine_point(&self) -> BlasResult<Point3> {
        if let [Some(x), Some(y), Some(z), Some(weight)] =
            self.coordinates().map(Real::exact_rational_ref)
        {
            if weight.is_one() {
                crate::trace_dispatch!(
                    "hyperlattice_projective",
                    "affine-point",
                    "exact-unit-weight"
                );
                return Ok(Point3::new(self.x.clone(), self.y.clone(), self.z.clone()));
            }
            if [x, y, z, weight].into_iter().all(|value| value.is_dyadic()) {
                crate::trace_dispatch!(
                    "hyperlattice_projective",
                    "affine-point",
                    "exact-dyadic-quotients"
                );
                return Ok(Point3::new(
                    Real::exact_rational_quotient_known_dyadic(&self.x, &self.w)?,
                    Real::exact_rational_quotient_known_dyadic(&self.y, &self.w)?,
                    Real::exact_rational_quotient_known_dyadic(&self.z, &self.w)?,
                ));
            }
        }
        crate::trace_dispatch!("hyperlattice_projective", "affine-point", "general");
        Ok(Point3::new(
            (&self.x / &self.w)?,
            (&self.y / &self.w)?,
            (&self.z / &self.w)?,
        ))
    }

    /// Delegate the exact four-term homogeneous incidence form to homogeneous_point_plane_expression.
    ///
    /// This constructs an expression; it does not classify its sign/zero status or
    /// validate either carrier. Its sign depends on the chosen homogeneous scales.
    pub fn plane_expression<P>(&self, plane: &P) -> Real
    where
        P: Plane3Coefficients + ?Sized,
    {
        homogeneous_point_plane_expression(self, plane)
    }
}

/// Pluecker coordinates (direction,moment), with moment=p×direction for an affine point p.
///
/// A valid projective line requires a nonzero six-tuple and direction·moment=0.
/// Nonzero direction gives an affine line; zero direction with nonzero moment can
/// represent a line at infinity. Neither condition is checked or normalized here.
/// Derived equality compares the stored coordinates, not the line image up to scale.
#[derive(Clone, Debug, PartialEq)]
pub struct HomogeneousLine3 {
    /// Direction component of the six-tuple; not checked to be nonzero.
    pub direction: Point3,
    /// Moment component; its orthogonality to direction and relation to an incident point are unchecked.
    pub moment: Point3,
}

impl HomogeneousLine3 {
    /// Store direction and moment unchanged without enforcing nonzero or orthogonality constraints.
    pub const fn new(direction: Point3, moment: Point3) -> Self {
        Self { direction, moment }
    }

    /// Delegate the line/plane cofactor construction without checking a unique finite intersection.
    ///
    /// Parallel or contained configurations can produce w=0 or an all-zero tuple.
    /// Carrier validity and affine conversion are separate consumer obligations.
    pub fn intersect_plane<P>(&self, plane: &P) -> HomogeneousPoint3
    where
        P: Plane3Coefficients + ?Sized,
    {
        intersect_homogeneous_line_plane(self, plane)
    }

    /// Aggregate structural metadata in direction-then-moment coordinate order; this does not validate the Pluecker relation or line nonzeroness.
    pub fn coordinate_facts(&self) -> crate::RealExactSetFacts {
        Real::exact_set_facts([
            &self.direction.x,
            &self.direction.y,
            &self.direction.z,
            &self.moment.x,
            &self.moment.y,
            &self.moment.z,
        ])
    }
}

/// Return signed 3×3 minors of the three ordered plane coefficient rows.
///
/// For normal matrix N and offset column d, x,y,z are minus det(N) with the
/// respective column replaced by d; w=det(N). For nonsingular N this is Cramer's
/// rule without division. Nonzero output with w=0 is a point at infinity; rank
/// below three yields an all-zero tuple, not a chosen point of a larger solution set.
///
/// First asks the scalar sparse-row kernel, which requires every coefficient to
/// expose an exact rational and a row with exactly one nonzero coefficient. That
/// path preserves the signed cofactor tuple, not merely its projective class. A
/// decline expands the same cofactors with signed product sums. No rank, normal,
/// incidence, or affine-domain decision is made, and no receipt is returned.
pub fn intersect_three_planes<P>(first: &P, second: &P, third: &P) -> HomogeneousPoint3
where
    P: Plane3Coefficients + ?Sized,
{
    let n1 = first.normal();
    let n2 = second.normal();
    let n3 = third.normal();

    let rows = [
        [&n1.x, &n1.y, &n1.z, first.offset()],
        [&n2.x, &n2.y, &n2.z, second.offset()],
        [&n3.x, &n3.y, &n3.z, third.offset()],
    ];
    if let Some([x, y, z, w]) = Real::exact_rational_sparse_homogeneous_plane_intersection3(rows) {
        crate::trace_dispatch!(
            "hyperlattice_projective",
            "three-plane-intersection",
            "exact-rational-sparse-row"
        );
        return HomogeneousPoint3::new(x, y, z, w);
    }
    crate::trace_dispatch!(
        "hyperlattice_projective",
        "three-plane-intersection",
        "expanded-fallback"
    );
    expanded_three_plane_intersection(rows)
}

/// Expand the three negative replacement-column determinants and the normal determinant.
///
/// Rows are ordered (a,b,c,d) coefficients. This fallback retains the same signed
/// tuple as the sparse rational route, without checking its nonzeroness or dividing.
/// It delegates each ordered six-term expression to det3/neg_det3.
fn expanded_three_plane_intersection(rows: [[&Real; 4]; 3]) -> HomogeneousPoint3 {
    HomogeneousPoint3::new(
        neg_det3(
            rows[0][3], rows[0][1], rows[0][2], rows[1][3], rows[1][1], rows[1][2], rows[2][3],
            rows[2][1], rows[2][2],
        ),
        neg_det3(
            rows[0][0], rows[0][3], rows[0][2], rows[1][0], rows[1][3], rows[1][2], rows[2][0],
            rows[2][3], rows[2][2],
        ),
        neg_det3(
            rows[0][0], rows[0][1], rows[0][3], rows[1][0], rows[1][1], rows[1][3], rows[2][0],
            rows[2][1], rows[2][3],
        ),
        det3(
            rows[0][0], rows[0][1], rows[0][2], rows[1][0], rows[1][1], rows[1][2], rows[2][0],
            rows[2][1], rows[2][2],
        ),
    )
}

/// Construct direction=n1×n2 and moment=d1*n2-d2*n1 from the ordered planes.
///
/// These division-free expressions satisfy the Pluecker relation algebraically.
/// Parallel affine planes can yield zero direction with nonzero moment (a line at
/// infinity); dependent coefficient rows yield the invalid all-zero carrier. No
/// plane validity, rank, normalization, or unique affine intersection check occurs.
pub fn intersect_two_planes<P>(first: &P, second: &P) -> HomogeneousLine3
where
    P: Plane3Coefficients + ?Sized,
{
    let direction = cross(first.normal(), second.normal());
    let moment = Point3::new(
        sub(
            &mul(first.offset(), &second.normal().x),
            &mul(second.offset(), &first.normal().x),
        ),
        sub(
            &mul(first.offset(), &second.normal().y),
            &mul(second.offset(), &first.normal().y),
        ),
        sub(
            &mul(first.offset(), &second.normal().z),
            &mul(second.offset(), &first.normal().z),
        ),
    );
    HomogeneousLine3::new(direction, moment)
}

/// Return the homogeneous tuple (n×moment-d*direction, n·direction).
///
/// For a valid Pluecker line this represents its plane intersection when the tuple
/// is nonzero. A contained line can give all zeros; a parallel affine line can meet
/// at infinity. The caller owns line/plane validity, including the Pluecker relation;
/// arbitrary invalid inputs are still evaluated. No division, selection among
/// solutions, or incidence decision occurs here.
pub fn intersect_homogeneous_line_plane<P>(line: &HomogeneousLine3, plane: &P) -> HomogeneousPoint3
where
    P: Plane3Coefficients + ?Sized,
{
    let n_cross_m = cross(plane.normal(), &line.moment);
    let d_u = Point3::new(
        mul(plane.offset(), &line.direction.x),
        mul(plane.offset(), &line.direction.y),
        mul(plane.offset(), &line.direction.z),
    );
    HomogeneousPoint3::new(
        sub(&n_cross_m.x, &d_u.x),
        sub(&n_cross_m.y, &d_u.y),
        sub(&n_cross_m.z, &d_u.z),
        dot(plane.normal(), &line.direction),
    )
}

/// Build a*x+b*y+c*z+d*w as one positive four-product Real sum.
///
/// A zero result is the incidence equation for valid projective carriers, but this
/// function only constructs the expression. It also returns zero for an all-zero
/// point or plane without diagnosing invalid input. Changing representative scale
/// changes the expression scale/sign; it is not an affine sidedness classification.
pub fn homogeneous_point_plane_expression<P>(point: &HomogeneousPoint3, plane: &P) -> Real
where
    P: Plane3Coefficients + ?Sized,
{
    Real::signed_product_sum(
        [true; 4],
        [
            [&plane.normal().x, &point.x],
            [&plane.normal().y, &point.y],
            [&plane.normal().z, &point.z],
            [plane.offset(), &point.w],
        ],
    )
}

/// Expand the right-handed 3D cross product in x,y,z order using borrowed scalar products and differences.
fn cross(left: &Point3, right: &Point3) -> Point3 {
    Point3::new(
        sub(&mul(&left.y, &right.z), &mul(&left.z, &right.y)),
        sub(&mul(&left.z, &right.x), &mul(&left.x, &right.z)),
        sub(&mul(&left.x, &right.y), &mul(&left.y, &right.x)),
    )
}

/// Build the three coordinate products as one positive Real signed-product sum, without a sign decision.
fn dot(left: &Point3, right: &Point3) -> Real {
    Real::signed_product_sum(
        [true; 3],
        [
            [&left.x, &right.x],
            [&left.y, &right.y],
            [&left.z, &right.z],
        ],
    )
}

/// Evaluate the row-major determinant aei+bfg+cdh-ceg-bdi-afh in the retained term order.
///
/// Uses the scalar signed-product-sum kernel directly; it does not classify
/// singularity or construct inverse coefficients.
#[allow(clippy::too_many_arguments)]
fn det3(
    a: &Real,
    b: &Real,
    c: &Real,
    d: &Real,
    e: &Real,
    f: &Real,
    g: &Real,
    h: &Real,
    i: &Real,
) -> Real {
    Real::signed_product_sum(
        [true, true, true, false, false, false],
        [
            [a, e, i],
            [b, f, g],
            [c, d, h],
            [c, e, g],
            [b, d, i],
            [a, f, h],
        ],
    )
}

/// Evaluate the negative row-major determinant with the same six products and inverted sign flags, without a separate negation node.
#[allow(clippy::too_many_arguments)]
fn neg_det3(
    a: &Real,
    b: &Real,
    c: &Real,
    d: &Real,
    e: &Real,
    f: &Real,
    g: &Real,
    h: &Real,
    i: &Real,
) -> Real {
    Real::signed_product_sum(
        [false, false, false, true, true, true],
        [
            [a, e, i],
            [b, f, g],
            [c, d, h],
            [c, e, g],
            [b, d, i],
            [a, f, h],
        ],
    )
}

/// Borrowed Real subtraction used by component formulas; no local guard, rounding, or policy decision is added.
fn sub(left: &Real, right: &Real) -> Real {
    left - right
}

/// Borrowed Real multiplication used by component formulas; scalar structure and evaluation remain Hyperreal-owned.
fn mul(left: &Real, right: &Real) -> Real {
    left * right
}

/// Projective construction and conversion regressions; finite fixtures do not establish general closure or cost bounds.
#[cfg(test)]
mod tests {
    use super::*;

    /// Lift an i32 fixture coefficient exactly into Real.
    fn r(value: i32) -> Real {
        value.into()
    }

    /// Construct an integer-coordinate Point3 fixture without projective or geometric validation.
    fn p3(x: i32, y: i32, z: i32) -> Point3 {
        Point3::new(r(x), r(y), r(z))
    }

    /// Build a raw ProjectivePlane3 fixture from four exact integer coefficients, including degenerate choices.
    fn plane(a: i32, b: i32, c: i32, d: i32) -> ProjectivePlane3 {
        ProjectivePlane3::new(p3(a, b, c), r(d))
    }

    /// Parse an exact numerator/denominator fixture; unwrap panics for an invalid fraction such as denominator zero.
    fn q(numerator: i64, denominator: u64) -> Real {
        format!("{numerator}/{denominator}").parse().unwrap()
    }

    /// Borrow a 3×4 coefficient array in row/column order for the scalar cofactor kernel.
    fn coefficient_refs(coefficients: &[[Real; 4]; 3]) -> [[&Real; 4]; 3] {
        std::array::from_fn(|row| std::array::from_fn(|column| &coefficients[row][column]))
    }

    /// Clone each coefficient row into a plane carrier without changing row order or validating rank.
    fn coefficient_planes(coefficients: &[[Real; 4]; 3]) -> [ProjectivePlane3; 3] {
        std::array::from_fn(|row| {
            ProjectivePlane3::new(
                Point3::new(
                    coefficients[row][0].clone(),
                    coefficients[row][1].clone(),
                    coefficients[row][2].clone(),
                ),
                coefficients[row][3].clone(),
            )
        })
    }

    /// Assert all six mixed 2×2 coordinate minors vanish.
    ///
    /// For two nonzero tuples this tests proportionality; it does not establish that
    /// either tuple is nonzero. An all-zero tuple passes against any tuple. Callers must
    /// separately establish projective validity before treating this as point equality.
    fn assert_projectively_equal(left: &HomogeneousPoint3, right: &HomogeneousPoint3) {
        let left = [&left.x, &left.y, &left.z, &left.w];
        let right = [&right.x, &right.y, &right.z, &right.w];
        for first in 0..4 {
            for second in (first + 1)..4 {
                assert_eq!(left[first] * right[second], left[second] * right[first]);
            }
        }
    }

    /// Check the three coordinate planes yield exactly (1,2,3,1) and affine point (1,2,3).
    #[test]
    fn three_plane_intersection_preserves_homogeneous_point() {
        let x_eq_1 = plane(1, 0, 0, -1);
        let y_eq_2 = plane(0, 1, 0, -2);
        let z_eq_3 = plane(0, 0, 1, -3);

        let point = intersect_three_planes(&x_eq_1, &y_eq_2, &z_eq_3);
        assert_eq!(point, HomogeneousPoint3::new(r(1), r(2), r(3), r(1)));
        assert_eq!(point.to_affine_point().unwrap(), p3(1, 2, 3));
    }

    /// Compare direct sparse, aggregate, and expanded tuples for all twelve sparse positions and two denominator recipes.
    ///
    /// The sparse coefficient is signed 41/3; retained rows use dyadic or selected odd
    /// denominators. Asserts coordinate equality, mixed-minor equality, and nonzero
    /// expanded output for these 24 cases. Shared scalar kernels mean this is path
    /// agreement plus validity for the fixtures, not an independent universal cofactor proof.
    #[test]
    fn sparse_exact_row_matches_expanded_cofactors_for_every_position() {
        /// Fixed positive coefficients for retained rows in the sparse-position fixtures.
        const NUMERATORS: [[i64; 4]; 3] = [[2, 3, 5, 7], [11, 13, 17, 19], [23, 29, 31, 37]];
        /// Four positive odd denominators cycled through the retained-row fixture recipe.
        const ODD_DENOMINATORS: [u64; 4] = [3, 5, 7, 11];

        for non_dyadic_retained_rows in [false, true] {
            for sparse_row in 0..3 {
                for sparse_column in 0..4 {
                    let mut coefficients = std::array::from_fn(|row| {
                        std::array::from_fn(|column| {
                            let denominator = if non_dyadic_retained_rows {
                                ODD_DENOMINATORS[(row + column) % 4]
                            } else {
                                1_u64 << ((row * 4 + column) % 8)
                            };
                            q(NUMERATORS[row][column], denominator)
                        })
                    });
                    coefficients[sparse_row] = std::array::from_fn(|_| Real::zero());
                    coefficients[sparse_row][sparse_column] = q(
                        if (sparse_row + sparse_column) % 2 == 0 {
                            41
                        } else {
                            -41
                        },
                        3,
                    );

                    let [x, y, z, w] = Real::exact_rational_sparse_homogeneous_plane_intersection3(
                        coefficient_refs(&coefficients),
                    )
                    .expect("the exact matrix has a one-coefficient row");
                    let direct = HomogeneousPoint3::new(x, y, z, w);
                    let planes = coefficient_planes(&coefficients);
                    let aggregate = intersect_three_planes(&planes[0], &planes[1], &planes[2]);
                    let expanded =
                        expanded_three_plane_intersection(coefficient_refs(&coefficients));

                    assert_eq!(aggregate, direct);
                    assert_eq!(aggregate, expanded);
                    assert_projectively_equal(&aggregate, &expanded);
                    assert!(
                        expanded
                            .coordinates()
                            .into_iter()
                            .any(|value| !value.exact_rational_ref().unwrap().is_zero())
                    );
                }
            }
        }
    }

    /// Check sparse-kernel decline for dense rational and sparse symbolic rows.
    ///
    /// The symbolic aggregate result must retain at least one coordinate without a
    /// borrowed rational representation. This test does not assert every resulting
    /// coordinate, incidence relation, or dispatch-trace count.
    #[test]
    fn nonsparse_or_symbolic_rows_use_the_expanded_fallback() {
        let nonsparse = [
            [r(1), r(2), r(3), r(4)],
            [r(5), r(6), r(7), r(8)],
            [r(9), r(10), r(11), r(12)],
        ];
        assert!(
            Real::exact_rational_sparse_homogeneous_plane_intersection3(coefficient_refs(
                &nonsparse
            ))
            .is_none()
        );

        let symbolic = [
            [Real::pi(), Real::zero(), Real::zero(), Real::zero()],
            [r(1), r(2), r(3), r(4)],
            [r(5), r(7), r(11), r(13)],
        ];
        assert!(
            Real::exact_rational_sparse_homogeneous_plane_intersection3(coefficient_refs(
                &symbolic
            ))
            .is_none()
        );
        let planes = coefficient_planes(&symbolic);
        let fallback = intersect_three_planes(&planes[0], &planes[1], &planes[2]);
        assert!(
            fallback
                .coordinates()
                .into_iter()
                .any(|value| value.exact_rational_ref().is_none())
        );
    }

    /// Compare aggregate and expanded tuples for a two-sparse-row case and a dependent-row case.
    ///
    /// The latter must be the all-zero tuple, explicitly exercising construction without
    /// a projective-validity certificate. The proportionality helper alone would not
    /// prove validity for either case.
    #[test]
    fn sparse_intersection_handles_multiple_sparse_and_degenerate_rows() {
        for (degenerate, coefficients) in [
            (
                false,
                [
                    [r(2), r(0), r(0), r(0)],
                    [r(0), r(3), r(0), r(0)],
                    [r(1), r(2), r(5), r(7)],
                ],
            ),
            (
                true,
                [
                    [r(2), r(0), r(0), r(0)],
                    [r(1), r(3), r(5), r(7)],
                    [r(2), r(6), r(10), r(14)],
                ],
            ),
        ] {
            let planes = coefficient_planes(&coefficients);
            let aggregate = intersect_three_planes(&planes[0], &planes[1], &planes[2]);
            let expanded = expanded_three_plane_intersection(coefficient_refs(&coefficients));
            assert_eq!(aggregate, expanded);
            assert_projectively_equal(&aggregate, &expanded);
            if degenerate {
                assert_eq!(aggregate, HomogeneousPoint3::new(r(0), r(0), r(0), r(0)));
                assert_eq!(aggregate, expanded);
            }
        }
    }

    /// Compare wide dyadic affine conversion with three separately constructed general Real quotients.
    ///
    /// Uses numerators exceeding u128 and weight 3/8. This verifies the fixture's scalar
    /// path agreement, not a separate arithmetic implementation or a performance bound.
    #[test]
    fn wide_dyadic_affine_point_matches_independent_exact_division() {
        let value = |text: &str| text.parse::<Real>().unwrap();
        let point = HomogeneousPoint3::new(
            value("680564733841876926926749214863536422913/128"),
            value("-340282366920938463463374607431768211507/32"),
            value("5/256"),
            value("3/8"),
        );
        let expected = Point3::new(
            (&point.x / &point.w).unwrap(),
            (&point.y / &point.w).unwrap(),
            (&point.z / &point.w).unwrap(),
        );

        assert_eq!(point.to_affine_point().unwrap(), expected);
    }

    /// Check coordinate-plane composition through a Pluecker line preserves the expected direction, moment, cofactor tuple, and affine point.
    #[test]
    fn two_plane_line_and_line_plane_match_three_plane_intersection() {
        let x_eq_1 = plane(1, 0, 0, -1);
        let y_eq_2 = plane(0, 1, 0, -2);
        let z_eq_3 = plane(0, 0, 1, -3);

        let line = intersect_two_planes(&x_eq_1, &y_eq_2);
        assert_eq!(line.direction, p3(0, 0, 1));
        assert_eq!(line.moment, p3(2, -1, 0));

        let point = line.intersect_plane(&z_eq_3);
        assert_eq!(point, intersect_three_planes(&x_eq_1, &y_eq_2, &z_eq_3));
        assert_eq!(point.to_affine_point().unwrap(), p3(1, 2, 3));
    }

    /// Check a parallel-plane fixture has w=0 and affine conversion returns an error.
    ///
    /// Does not assert the complete homogeneous tuple, nonzero-carrier validity, or the
    /// specific error variant; those are stronger claims than this regression alone.
    #[test]
    fn parallel_planes_produce_point_at_infinity_without_affine_division() {
        let x_eq_1 = plane(1, 0, 0, -1);
        let x_eq_2 = plane(1, 0, 0, -2);
        let z_eq_0 = plane(0, 0, 1, 0);

        let point = intersect_three_planes(&x_eq_1, &x_eq_2, &z_eq_0);
        assert_eq!(point.w, r(0));
        assert!(point.to_affine_point().is_err());
    }
}
