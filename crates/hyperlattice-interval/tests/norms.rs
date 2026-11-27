//! Finite qualification corpus for Cartesian norms, measures and dominance margins.
//!
//! Integer-grid expectations are independent scalar formulas. For this corpus,
//! absolute-value extrema occur at endpoints or zero, and separate rows/columns
//! can attain their extrema independently. The sampled integer members therefore
//! include the continuous extrema used here; this is not a general theorem for
//! arbitrary piecewise-linear maps or correlated input sets.

#![cfg(any(
    feature = "fixed",
    feature = "rational",
    feature = "dyadic",
    feature = "float-enclosure"
))]
use hyperlattice_interval::hyperinterval::IntervalError;
use hyperlattice_interval::{
    IntervalArithmetic, IntervalLatticeError, IntervalMatrix, IntervalVector,
};

// Independent oracle: enumerate integer members, not interval endpoint formulas.
// Absolute-value breakpoints are at zero, so integer endpoints and these interior
// grid points include every extremum of the piecewise-linear functions used here.
/// Visit every integer tuple in a small ordered inclusive box, coordinate zero fastest.
///
/// The zero-dimensional box visits its single empty tuple once. Fixture bounds
/// must be ordered and permit the loop's integer increments; this helper neither
/// validates malformed bounds nor limits the number or cost of callback calls.
fn enumerate<const N: usize>(bounds: [(i32, i32); N], mut visit: impl FnMut([i32; N])) {
    let mut point = bounds.map(|(lo, _)| lo);
    loop {
        visit(point);
        let mut index = 0;
        while index < N && point[index] == bounds[index].1 {
            point[index] = bounds[index].0;
            index += 1;
        }
        if index == N {
            break;
        }
        point[index] += 1;
    }
}
/// Compute componentwise discrete extrema of an integer-valued fixture callback.
///
/// Initialize each output with sentinel extrema and visit the entire input grid.
/// A zero-dimensional input still evaluates once; zero output dimension returns
/// an empty array. Continuous-range claims require the corpus-specific extrema
/// argument in the module documentation, not merely enumeration.
fn ranges<const N: usize, const M: usize>(
    bounds: [(i32, i32); N],
    value: impl Fn([i32; N]) -> [i32; M],
) -> [(i32, i32); M] {
    let mut result = [(i32::MAX, i32::MIN); M];
    enumerate(bounds, |point| {
        for (range, value) in result.iter_mut().zip(value(point)) {
            range.0 = range.0.min(value);
            range.1 = range.1.max(value);
        }
    });
    result
}
/// Check 125 three-coordinate boxes, 625 square boxes and a rectangular witness.
///
/// The constructor and fresh-context factory instantiate the same independent
/// integer expectations for each scalar family. Checks cover both vector/matrix
/// norms, signed measures, row/column dominance, identity subtraction and its
/// involution, contraction margins, transpose correspondences and subsequent
/// norm queries. A 2-by-3 box exercises unequal dimensions; negative measures,
/// empty products and zero-dimensional units have explicit expectations.
/// Each query starts with a fresh context; cumulative-budget behavior is tested
/// separately. Small integer arithmetic here must remain representable.
fn qualify<I: IntervalArithmetic + core::fmt::Debug + PartialEq>(
    scalar: impl Fn(i32, i32) -> I,
    mut context: impl FnMut() -> I::Context,
) {
    let intervals = [(-2, -1), (-1, 1), (0, 0), (1, 2), (-2, 2)];
    for word in 0..125 {
        let mut digits = word;
        let bounds = core::array::from_fn::<_, 3, _>(|_| {
            let v = intervals[digits % 5];
            digits /= 5;
            v
        });
        let expected = ranges(bounds, |p| {
            [
                p.iter().map(|x| x.abs()).sum(),
                p.iter().map(|x| x.abs()).max().unwrap(),
            ]
        });
        let vector = IntervalVector::new(bounds.map(|(a, b)| scalar(a, b))).unwrap();
        assert_eq!(
            vector.norm_one(&mut context()).unwrap(),
            scalar(expected[0].0, expected[0].1)
        );
        assert_eq!(
            vector.norm_infinity(&mut context()).unwrap(),
            scalar(expected[1].0, expected[1].1)
        );
    }
    for word in 0..625 {
        let mut digits = word;
        let bounds = core::array::from_fn::<_, 4, _>(|_| {
            let v = intervals[digits % 5];
            digits /= 5;
            v
        });
        let expected = ranges(bounds, |[a, b, c, d]| {
            [
                (a.abs() + c.abs()).max(b.abs() + d.abs()),
                (a.abs() + b.abs()).max(c.abs() + d.abs()),
                (a + c.abs()).max(d + b.abs()),
                (a + b.abs()).max(d + c.abs()),
                a.abs() - b.abs(),
                d.abs() - c.abs(),
                a.abs() - c.abs(),
                d.abs() - b.abs(),
                1 - a,
                -b,
                -c,
                1 - d,
            ]
        });
        let entry = |index: usize| scalar(bounds[index].0, bounds[index].1);
        let matrix = IntervalMatrix::new([[entry(0), entry(1)], [entry(2), entry(3)]]).unwrap();
        for (actual, index) in [
            (matrix.norm_one(&mut context()), 0),
            (matrix.norm_infinity(&mut context()), 1),
            (matrix.measure_one(&mut context()), 2),
            (matrix.measure_infinity(&mut context()), 3),
        ] {
            assert_eq!(
                actual.unwrap(),
                scalar(expected[index].0, expected[index].1),
                "word={word},operation={index}"
            );
        }
        let remainder = matrix.identity_minus(&mut context()).unwrap();
        for (index, lane) in remainder.rows().unwrap().iter().flatten().enumerate() {
            let (lo, hi) = expected[index + 8];
            assert_eq!(lane, &scalar(lo, hi));
        }
        assert_eq!(remainder.identity_minus(&mut context()).unwrap(), matrix);
        assert_eq!(
            matrix.contraction_margin_one(&mut context()).unwrap(),
            scalar(1 - expected[0].1, 1 - expected[0].0)
        );
        assert_eq!(
            matrix.contraction_margin_infinity(&mut context()).unwrap(),
            scalar(1 - expected[1].1, 1 - expected[1].0)
        );
        assert_eq!(
            matrix
                .transpose(&mut context())
                .unwrap()
                .contraction_margin_one(&mut context())
                .unwrap(),
            matrix.contraction_margin_infinity(&mut context()).unwrap()
        );
        assert_eq!(
            matrix
                .row_dominance_margins(&mut context())
                .unwrap()
                .lanes(),
            Some(&[
                scalar(expected[4].0, expected[4].1),
                scalar(expected[5].0, expected[5].1)
            ])
        );
        assert_eq!(
            matrix
                .column_dominance_margins(&mut context())
                .unwrap()
                .lanes(),
            Some(&[
                scalar(expected[6].0, expected[6].1),
                scalar(expected[7].0, expected[7].1)
            ])
        );
        assert_eq!(
            matrix
                .row_dominance_margins(&mut context())
                .unwrap()
                .norm_infinity(&mut context())
                .unwrap(),
            matrix
                .transpose(&mut context())
                .unwrap()
                .column_dominance_margins(&mut context())
                .unwrap()
                .norm_infinity(&mut context())
                .unwrap()
        );
    }
    let bounds = [(-2, 1), (-1, 3), (2, 4), (-3, 0), (1, 2), (-1, 1)];
    let expected = ranges(bounds, |[a, b, c, d, e, f]| {
        [
            (a.abs() + d.abs())
                .max(b.abs() + e.abs())
                .max(c.abs() + f.abs()),
            (a.abs() + b.abs() + c.abs()).max(d.abs() + e.abs() + f.abs()),
        ]
    });
    let m = IntervalMatrix::new(core::array::from_fn::<_, 2, _>(|r| {
        core::array::from_fn::<_, 3, _>(|c| {
            let (a, b) = bounds[r * 3 + c];
            scalar(a, b)
        })
    }))
    .unwrap();
    assert_eq!(
        m.norm_one(&mut context()).unwrap(),
        scalar(expected[0].0, expected[0].1)
    );
    assert_eq!(
        m.norm_infinity(&mut context()).unwrap(),
        scalar(expected[1].0, expected[1].1)
    );
    let transpose = m.transpose(&mut context()).unwrap();
    assert_eq!(
        transpose.norm_one(&mut context()).unwrap(),
        m.norm_infinity(&mut context()).unwrap()
    );

    let negative = IntervalMatrix::new([
        [scalar(-5, -3), scalar(0, 1)],
        [scalar(-1, 0), scalar(-4, -2)],
    ])
    .unwrap();
    for measure in [
        negative.measure_one(&mut context()),
        negative.measure_infinity(&mut context()),
    ] {
        assert_eq!(measure.unwrap(), scalar(-4, -1));
    }
    let point = IntervalMatrix::new([
        [scalar(-3, -3), scalar(0, 0)],
        [scalar(0, 0), scalar(-2, -2)],
    ])
    .unwrap();
    assert_eq!(
        point.measure_infinity(&mut context()).unwrap(),
        scalar(-2, -2)
    );

    let zero = scalar(0, 0);
    let unit = IntervalVector::<I, 0>::new([]).unwrap();
    assert_eq!(unit.norm_one(&mut context()).unwrap(), zero);
    assert_eq!(unit.norm_infinity(&mut context()).unwrap(), zero);
    for m in [
        IntervalMatrix::<I, 0, 3>::new([])
            .unwrap()
            .norm_one(&mut context()),
        IntervalMatrix::<I, 0, 3>::new([])
            .unwrap()
            .norm_infinity(&mut context()),
        IntervalMatrix::<I, 3, 0>::new([[], [], []])
            .unwrap()
            .norm_one(&mut context()),
        IntervalMatrix::<I, 3, 0>::new([[], [], []])
            .unwrap()
            .norm_infinity(&mut context()),
    ] {
        assert_eq!(m.unwrap(), zero);
    }
    let matrix = IntervalMatrix::<I, 0, 0>::new([]).unwrap();
    assert_eq!(matrix.norm_one(&mut context()).unwrap(), zero);
    assert_eq!(matrix.identity_minus(&mut context()).unwrap(), matrix);
    assert_eq!(
        matrix.contraction_margin_one(&mut context()).unwrap(),
        scalar(1, 1)
    );
    assert_eq!(
        matrix.contraction_margin_infinity(&mut context()).unwrap(),
        scalar(1, 1)
    );
    let empty = IntervalMatrix::<I, 2, 2>::empty().unwrap();
    assert!(empty.identity_minus(&mut context()).unwrap().is_empty());
    assert!(
        empty
            .contraction_margin_one(&mut context())
            .unwrap()
            .is_empty()
    );
    assert!(
        empty
            .contraction_margin_infinity(&mut context())
            .unwrap()
            .is_empty()
    );
    for measure in [
        matrix.measure_one(&mut context()),
        matrix.measure_infinity(&mut context()),
    ] {
        assert_eq!(
            measure,
            Err(IntervalLatticeError::Scalar(IntervalError::UnboundedImage))
        );
    }
    assert_eq!(matrix.row_dominance_margins(&mut context()).unwrap(), unit);
    assert_eq!(
        matrix.column_dominance_margins(&mut context()).unwrap(),
        unit
    );
    for result in [
        IntervalVector::<I, 0>::empty()
            .unwrap()
            .norm_one(&mut context()),
        IntervalVector::<I, 2>::empty()
            .unwrap()
            .norm_infinity(&mut context()),
        IntervalMatrix::<I, 0, 0>::empty()
            .unwrap()
            .measure_one(&mut context()),
        IntervalMatrix::<I, 2, 2>::empty()
            .unwrap()
            .measure_infinity(&mut context()),
        IntervalMatrix::<I, 2, 3>::empty()
            .unwrap()
            .norm_one(&mut context()),
        IntervalMatrix::<I, 2, 3>::empty()
            .unwrap()
            .norm_infinity(&mut context()),
    ] {
        assert!(result.unwrap().is_empty());
    }
    assert!(
        IntervalMatrix::<I, 2, 2>::empty()
            .unwrap()
            .row_dominance_margins(&mut context())
            .unwrap()
            .is_empty()
    );
    assert!(
        IntervalMatrix::<I, 0, 0>::empty()
            .unwrap()
            .column_dominance_margins(&mut context())
            .unwrap()
            .is_empty()
    );
}
#[cfg(feature = "fixed")]
#[test]
/// Run the common exact integer corpus on i32 and i64 fixed intervals at scale four.
fn fixed_norms_measures_and_margins() {
    use hyperlattice_interval::hyperinterval::FixedInterval;
    qualify(
        |a, b| FixedInterval::<i32, 4>::new(a * 16, b * 16).unwrap(),
        || (),
    );
    qualify(
        |a, b| FixedInterval::<i64, 4>::new(i64::from(a) * 16, i64::from(b) * 16).unwrap(),
        || (),
    );
}
#[cfg(feature = "rational")]
#[test]
/// Run the common corpus with exact rational endpoints and a unit context.
fn rational_norms_measures_and_margins() {
    use hyperlattice_interval::hyperinterval::RationalInterval;
    qualify(
        |a, b| RationalInterval::new(a.into(), b.into()).unwrap(),
        || (),
    );
}
#[cfg(feature = "dyadic")]
#[test]
/// Run the common corpus with exact dyadic endpoints and fresh adequate budgets.
fn dyadic_norms_measures_and_margins() {
    use hyperlattice_interval::hyperinterval::{DyadicBudget, DyadicInterval};
    qualify(
        |a, b| DyadicInterval::new(a.into(), b.into(), 0).unwrap(),
        || DyadicBudget::new(256, 1_000_000),
    );
}
#[cfg(feature = "float-enclosure")]
#[test]
/// Run both IEEE backends on integer fixtures whose expected extrema are exact.
/// The feature-enabled test requires a supported floating environment; it does
/// not treat an unsupported-environment error as a successful enclosure.
fn float_norms_measures_and_margins() {
    use hyperlattice_interval::hyperinterval::{F32Enclosure, F64Enclosure};
    qualify(|a, b| F32Enclosure::new(a as f32, b as f32).unwrap(), || ());
    qualify(|a, b| F64Enclosure::new(a.into(), b.into()).unwrap(), || ());
}

#[cfg(feature = "fixed")]
#[test]
/// Distinguish raw fixed units from real one, and signed measures from absolute norms.
///
/// At i64/F63, raw -1 can be reflected to raw +1 although real one cannot fit.
/// The minimum raw diagonal remains a valid signed measure while its absolute
/// image overflows. Summing two maximum values overflows; their maximum fits.
fn primitive_extremes_and_overflow_are_explicit() {
    use hyperlattice_interval::hyperinterval::FixedInterval;
    /// Highest valid i64 scale: raw integer endpoints represent multiples of 2^-63.
    type I = FixedInterval<i64, 63>;
    let x = IntervalVector::new([I::point(-1).unwrap()]).unwrap(); // +1 cannot be stored
    assert_eq!(x.norm_one(&mut ()).unwrap(), I::point(1).unwrap());
    let negative = IntervalMatrix::new([[I::point(i64::MIN).unwrap()]]).unwrap();
    assert_eq!(
        negative.measure_one(&mut ()).unwrap(),
        I::point(i64::MIN).unwrap()
    );
    for result in [
        negative.norm_one(&mut ()),
        negative
            .row_dominance_margins(&mut ())
            .map(|v| v.into_lanes().unwrap()[0]),
    ] {
        assert_eq!(
            result,
            Err(IntervalLatticeError::Scalar(IntervalError::Overflow))
        );
    }
    let vector = IntervalVector::new([I::point(i64::MAX).unwrap(); 2]).unwrap();
    assert_eq!(
        vector.norm_one(&mut ()),
        Err(IntervalLatticeError::Scalar(IntervalError::Overflow))
    );
    assert_eq!(
        vector.norm_infinity(&mut ()).unwrap(),
        I::point(i64::MAX).unwrap()
    );
}

#[cfg(feature = "float-enclosure")]
#[test]
/// Check subnormal absolute images, outward half-ulp sums and typed overflow.
/// Both IEEE precisions retain tiny terms; a maximum can fit when a sum cannot.
fn ieee_subnormals_and_nonrepresentable_sums_remain_enclosed() {
    use hyperlattice_interval::hyperinterval::{F32Enclosure, F64Enclosure};
    /// Instantiate the same endpoint assertions with matching interval and float types.
    /// `$half_ulp` is half the gap above one for that precision; expected bounds use
    /// adjacent representable values rather than rounding the exact sum to one.
    macro_rules! case {
        ($i:ty,$f:ty,$half_ulp:expr) => {{
            let point = |x| <$i>::point(x).unwrap();
            let tiny = <$f>::from_bits(1);
            let v = IntervalVector::new([point(tiny), point(-tiny)]).unwrap();
            assert_eq!(v.norm_one(&mut ()).unwrap(), point(<$f>::from_bits(2)));
            assert_eq!(v.norm_infinity(&mut ()).unwrap(), point(tiny));
            let v = IntervalVector::new([point(1.0), point($half_ulp)]).unwrap();
            assert_eq!(
                v.norm_one(&mut ()).unwrap(),
                <$i>::new(1.0, <$f>::from_bits((1.0 as $f).to_bits() + 1)).unwrap()
            );
            let v = IntervalVector::new([point(<$f>::MAX); 2]).unwrap();
            assert_eq!(
                v.norm_one(&mut ()),
                Err(IntervalLatticeError::Scalar(IntervalError::Overflow))
            );
            assert_eq!(v.norm_infinity(&mut ()).unwrap(), point(<$f>::MAX));
        }};
    }
    case!(F32Enclosure, f32, 2_f32.powi(-24));
    case!(F64Enclosure, f64, 2_f64.powi(-53));
}

#[cfg(feature = "rational")]
#[test]
/// Check explicit rational norm, signed-measure and dominance endpoints.
/// The unequal denominators exercise noninteger extrema independently of the
/// integer-grid oracle and include negative measures and both margin orientations.
fn fractional_ranges_preserve_exact_lower_and_upper_bounds() {
    use hyperlattice_interval::hyperinterval::RationalInterval as I;
    let scalar = |a: &str, b: &str| I::new(a.parse().unwrap(), b.parse().unwrap()).unwrap();
    let matrix = IntervalMatrix::new([
        [scalar("-5/3", "-4/3"), scalar("1/7", "2/7")],
        [scalar("-1/11", "1/11"), scalar("-3/2", "-5/4")],
    ])
    .unwrap();
    assert_eq!(matrix.norm_one(&mut ()).unwrap(), scalar("39/28", "25/14"));
    assert_eq!(
        matrix.norm_infinity(&mut ()).unwrap(),
        scalar("31/21", "41/21")
    );
    assert_eq!(
        matrix.measure_one(&mut ()).unwrap(),
        scalar("-19/14", "-27/28")
    );
    assert_eq!(
        matrix.measure_infinity(&mut ()).unwrap(),
        scalar("-3/2", "-22/21")
    );
    assert_eq!(
        matrix.row_dominance_margins(&mut ()).unwrap().lanes(),
        Some(&[scalar("22/21", "32/21"), scalar("51/44", "3/2")])
    );
    assert_eq!(
        matrix.column_dominance_margins(&mut ()).unwrap().lanes(),
        Some(&[scalar("41/33", "5/3"), scalar("27/28", "19/14")])
    );
}

#[cfg(feature = "dyadic")]
#[test]
/// Exercise shared dyadic work/size limits across eleven reduction operations.
///
/// Measured exact work admits a retry and one fewer unit fails; low coefficient
/// capacity fails separately. Sources remain unchanged. A dominance-to-norm chain
/// shares one context and checks literal endpoints; empty/unit shortcuts are
/// checked with zero work. These are resource witnesses, not cancellation tests.
fn all_reductions_share_budget_and_allow_retry() {
    use hyperlattice_interval::hyperinterval::{DyadicBudget, DyadicInterval as I};
    /// Measure one operation, then check work exhaustion, size exhaustion and retry.
    /// The initial successful result is reused to test budget admission, not offered
    /// as an independent geometry oracle. Exactly the measured work must suffice
    /// with none remaining; each failed or successful attempt uses a fresh budget.
    fn check<T: core::fmt::Debug + PartialEq>(
        operation: impl Fn(&mut DyadicBudget) -> Result<T, IntervalLatticeError>,
    ) {
        let mut context = DyadicBudget::new(256, 1_000_000);
        let expected = operation(&mut context).unwrap();
        let used = 1_000_000 - context.remaining_work();
        assert!(used > 0);
        assert_eq!(
            operation(&mut DyadicBudget::new(256, used - 1)),
            Err(IntervalLatticeError::Scalar(IntervalError::WorkExhausted))
        );
        let mut exact = DyadicBudget::new(256, used);
        assert_eq!(operation(&mut exact).unwrap(), expected);
        assert_eq!(exact.remaining_work(), 0);
        assert_eq!(
            operation(&mut DyadicBudget::new(1, 1_000_000)),
            Err(IntervalLatticeError::Scalar(IntervalError::SizeExhausted))
        );
        assert_eq!(
            operation(&mut DyadicBudget::new(256, 1_000_000)).unwrap(),
            expected
        );
    }
    let scalar = |a: i32, b: i32| I::new(a.into(), b.into(), 0).unwrap();
    let vector = IntervalVector::new([scalar(-3, 5), scalar(7, 9), scalar(-8, -6)]).unwrap();
    check(|c| vector.norm_one(c));
    check(|c| vector.norm_infinity(c));
    let matrix = IntervalMatrix::new([
        [scalar(-5, -4), scalar(2, 3)],
        [scalar(-2, 1), scalar(-7, -6)],
    ])
    .unwrap();
    let original = matrix.clone();
    check(|c| matrix.norm_one(c));
    check(|c| matrix.norm_infinity(c));
    check(|c| matrix.measure_one(c));
    check(|c| matrix.measure_infinity(c));
    check(|c| matrix.row_dominance_margins(c));
    check(|c| matrix.column_dominance_margins(c));
    check(|c| matrix.identity_minus(c));
    check(|c| matrix.contraction_margin_one(c));
    check(|c| matrix.contraction_margin_infinity(c));
    let empty = IntervalMatrix::<I, 2, 2>::empty().unwrap();
    assert!(
        empty
            .identity_minus(&mut DyadicBudget::new(0, 0))
            .unwrap()
            .is_empty()
    );
    assert!(
        empty
            .contraction_margin_one(&mut DyadicBudget::new(0, 0))
            .unwrap()
            .is_empty()
    );
    assert!(
        empty
            .contraction_margin_infinity(&mut DyadicBudget::new(0, 0))
            .unwrap()
            .is_empty()
    );
    let unit = IntervalMatrix::<I, 0, 0>::new([]).unwrap();
    assert_eq!(
        unit.identity_minus(&mut DyadicBudget::new(0, 0)).unwrap(),
        unit
    );
    assert_eq!(matrix, original);
    let mut context = DyadicBudget::new(256, 1_000_000);
    let margins = matrix.row_dominance_margins(&mut context).unwrap();
    assert_eq!(margins.norm_one(&mut context).unwrap(), scalar(5, 10));
    let used = 1_000_000 - context.remaining_work();
    let mut small = DyadicBudget::new(256, used - 1);
    let margins = matrix.row_dominance_margins(&mut small).unwrap();
    assert_eq!(
        margins.norm_one(&mut small),
        Err(IntervalLatticeError::Scalar(IntervalError::WorkExhausted))
    );
    assert!(
        IntervalMatrix::<I, 2, 2>::empty()
            .unwrap()
            .norm_one(&mut DyadicBudget::new(1, 0))
            .unwrap()
            .is_empty()
    );
}

#[cfg(feature = "fixed")]
#[test]
/// Contrast the exact line (x,1-x) with its independent Cartesian projection.
/// For x in `[0,1]`, projection permits a one-norm range `[0,2]`. Composing the sum
/// before projection retains the relation and yields exactly one.
fn norms_of_cartesian_projections_do_not_replace_retained_correlations() {
    use hyperlattice_interval::AffineMap;
    use hyperlattice_interval::hyperinterval::FixedInterval as F;
    /// Whole-unit fixed backend for the retained affine relation fixture.
    type I = F<i32, 0>;
    let p = |x| I::point(x).unwrap();
    let source = IntervalVector::new([I::new(0, 1).unwrap()]).unwrap();
    let map = AffineMap::new(
        IntervalMatrix::new([[p(1)], [p(-1)]]).unwrap(),
        IntervalVector::new([p(0), p(1)]).unwrap(),
    )
    .unwrap();
    let image = map.image(source); // exact (x,1-x), nonnegative, one-norm identically 1
    let projection = image.enclosure(&mut ()).unwrap();
    assert_eq!(projection.norm_one(&mut ()).unwrap(), I::new(0, 2).unwrap()); // the box has additional members
    let sum = AffineMap::new(
        IntervalMatrix::new([[p(1), p(1)]]).unwrap(),
        IntervalVector::new([p(0)]).unwrap(),
    )
    .unwrap();
    assert_eq!(
        sum.image(image)
            .enclosure(&mut ())
            .unwrap()
            .norm_one(&mut ())
            .unwrap(),
        p(1)
    );
}
