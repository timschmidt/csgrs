//! Finite qualification of retained affine composition and explicit projection.
//!
//! Independent integer-corner evaluation supplies coordinate extrema for the small
//! linear fixtures. Further cases exercise lost correlations after boxing, off-grid
//! definitions, singular maps, empty/unit domains and shared dyadic resource limits.

#![cfg(any(
    feature = "fixed",
    feature = "rational",
    feature = "dyadic",
    feature = "float-enclosure"
))]
use hyperlattice_interval::{
    AffineMap, AffineSource, IntervalArithmetic, IntervalMatrix, IntervalVector,
};

/// Build a point map from small integer coefficients using a fixture scalar constructor.
/// Both carriers and singleton-map validation must succeed; malformed or unavailable
/// fixture values panic. No interval coefficient composition supplies oracle values.
fn map<I: IntervalArithmetic, const R: usize, const C: usize>(
    coefficients: [[i32; C]; R],
    translation: [i32; R],
    scalar: &impl Fn(i32, i32) -> I,
) -> AffineMap<I, R, C> {
    AffineMap::new(
        IntervalMatrix::new(coefficients.map(|row| row.map(|x| scalar(x, x)))).unwrap(),
        IntervalVector::new(translation.map(|x| scalar(x, x))).unwrap(),
    )
    .unwrap()
}

// Independent oracle: evaluate point maps at each source-box corner with i64
// integers. Do not compose interval coefficients or call any production kernel.
/// Evaluate two point maps at all four corners of an ordered two-coordinate box.
///
/// Coordinatewise affine extrema occur at corners, so these min/max values are
/// independent exact hull expectations for the bounded integer corpus. Intermediate
/// calculations use i64 and final bounds must fit i32; this helper does not provide
/// checked arbitrary-coefficient arithmetic or support nonlinear extrema.
fn corners(
    a: [[i32; 2]; 2],
    b: [[i32; 2]; 2],
    ta: [i32; 2],
    tb: [i32; 2],
    domain: [(i32, i32); 2],
) -> [(i32, i32); 2] {
    let mut bounds = [(i64::MAX, i64::MIN); 2];
    for x in [domain[0].0, domain[0].1] {
        for y in [domain[1].0, domain[1].1] {
            let first =
                a.map(|row| i64::from(row[0]) * i64::from(x) + i64::from(row[1]) * i64::from(y));
            let first = [first[0] + i64::from(ta[0]), first[1] + i64::from(ta[1])];
            for r in 0..2 {
                let value = i64::from(b[r][0]) * first[0]
                    + i64::from(b[r][1]) * first[1]
                    + i64::from(tb[r]);
                bounds[r] = (bounds[r].0.min(value), bounds[r].1.max(value));
            }
        }
    }
    bounds.map(|(lo, hi)| (lo.try_into().unwrap(), hi.try_into().unwrap()))
}

/// Check all 6,561 pairs of 2-by-2 matrices with entries -1, 0 or 1.
///
/// Translations come from selected matrix entries; the fixed source box and direct
/// corner oracle determine independent endpoint expectations. Additional fixtures
/// contrast retained cancellation with premature boxing, apply inverse shears,
/// restrict original parameters, preserve empty results and rebuild owned parts.
/// Each query uses a fresh context; shared-budget behavior is tested separately.
fn qualify<I: IntervalArithmetic + core::fmt::Debug + PartialEq>(
    scalar: impl Fn(i32, i32) -> I,
    mut context: impl FnMut() -> I::Context,
) {
    let domain = IntervalVector::new([scalar(-2, 1), scalar(-1, 3)]).unwrap();
    let matrices: Vec<_> = (-1..=1)
        .flat_map(|a| {
            (-1..=1).flat_map(move |b| {
                (-1..=1).flat_map(move |c| (-1..=1).map(move |d| [[a, b], [c, d]]))
            })
        })
        .collect();
    for a in &matrices {
        for b in &matrices {
            let ta = [a[0][0], a[1][1]];
            let tb = [b[0][1], b[1][0]];
            let first = map(*a, ta, &scalar).image(domain.clone());
            let image = map(*b, tb, &scalar).image(first);
            let expected =
                corners(*a, *b, ta, tb, [(-2, 1), (-1, 3)]).map(|(lo, hi)| scalar(lo, hi));
            assert_eq!(
                image.enclosure(&mut context()).unwrap().lanes(),
                Some(&expected)
            );
            assert_eq!(image.parameter_domain(), &domain);
            assert_eq!(
                image.source().map().linear().rows().unwrap()[0][0],
                scalar(a[0][0], a[0][0])
            );
        }
    }
    let parameter = IntervalVector::new([scalar(-1, 1)]).unwrap();
    let duplicated = map([[1], [1]], [3, -4], &scalar).image(parameter.clone());
    let projected = duplicated.enclosure(&mut context()).unwrap();
    let difference = map([[1, -1]], [0], &scalar);
    let boxed = difference
        .clone()
        .image(projected)
        .enclosure(&mut context())
        .unwrap();
    let correlated = difference
        .image(duplicated)
        .enclosure(&mut context())
        .unwrap();
    assert_eq!(boxed.lanes(), Some(&[scalar(5, 9)]));
    assert_eq!(correlated.lanes(), Some(&[scalar(7, 7)]));

    let shear = map([[1, 1], [0, 1]], [1, -1], &scalar);
    let inverse = map([[1, -1], [0, 1]], [-2, 1], &scalar);
    let image = inverse.image(shear.image(domain.clone()));
    assert_eq!(image.enclosure(&mut context()).unwrap(), domain);
    let selection = IntervalVector::new([scalar(0, 2), scalar(0, 1)]).unwrap();
    let restricted = image
        .restrict_parameters(&selection, &mut context())
        .unwrap();
    let expected = IntervalVector::new([scalar(0, 1), scalar(0, 1)]).unwrap();
    assert_eq!(restricted.parameter_domain(), &expected);
    assert_eq!(restricted.enclosure(&mut context()).unwrap(), expected);
    assert_eq!(image.enclosure(&mut context()).unwrap(), domain);
    let excluded = IntervalVector::new([scalar(3, 4), scalar(0, 1)]).unwrap();
    let empty = image
        .restrict_parameters(&excluded, &mut context())
        .unwrap();
    assert!(empty.is_empty() && empty.enclosure(&mut context()).unwrap().is_empty());

    // Exercise the sealed generic source API separately from inherent methods.
    /// Exercise parameter access, emptiness and restriction through the sealed generic trait.
    /// The fixture supplies a nonempty source and adequate context; violation or a
    /// restriction error panics. The parameter type is inherited from that source.
    fn restrict<I: IntervalArithmetic, S: AffineSource<I, 2>>(
        s: &S,
        r: &S::Parameters,
        c: &mut I::Context,
    ) -> S {
        assert!(!s.is_empty());
        let _ = s.parameter_domain();
        s.restrict_parameters(r, c).unwrap()
    }
    assert_eq!(
        restrict(&image, &selection, &mut context()).parameter_domain(),
        &expected
    );
    let (source, last) = image.into_parts();
    let (linear, translation) = last.into_parts();
    let rebuilt = AffineMap::new(linear, translation).unwrap().image(source);
    assert_eq!(rebuilt.enclosure(&mut context()).unwrap(), domain);
}

#[cfg(feature = "fixed")]
#[test]
/// Run the finite integer matrix-pair corpus on whole-unit i32 and i64 backends.
fn exhaustive_fixed_affine_chains() {
    use hyperlattice_interval::hyperinterval::FixedInterval;
    qualify(|a, b| FixedInterval::<i32, 0>::new(a, b).unwrap(), || ());
    qualify(
        |a, b| FixedInterval::<i64, 0>::new(a.into(), b.into()).unwrap(),
        || (),
    );
}

#[cfg(feature = "rational")]
#[test]
/// Run the finite matrix-pair corpus with exact rational interval arithmetic.
fn exhaustive_rational_affine_chains() {
    use hyperlattice_interval::hyperinterval::RationalInterval;
    qualify(
        |a, b| RationalInterval::new(a.into(), b.into()).unwrap(),
        || (),
    );
}

#[cfg(feature = "dyadic")]
#[test]
/// Run the finite matrix-pair corpus with exact dyadic values and fresh ample budgets.
fn exhaustive_dyadic_affine_chains() {
    use hyperlattice_interval::hyperinterval::{DyadicBudget, DyadicInterval};
    qualify(
        |a, b| DyadicInterval::new(a.into(), b.into(), 0).unwrap(),
        || DyadicBudget::new(256, 1_000_000),
    );
}

#[cfg(feature = "float-enclosure")]
#[test]
/// Run both IEEE backends on small integer fixtures with exactly representable results.
/// Enabled tests require a supported floating environment; errors are not skipped.
fn exhaustive_strict_float_affine_chains() {
    use hyperlattice_interval::hyperinterval::{F32Enclosure, F64Enclosure};
    qualify(|a, b| F32Enclosure::new(a as f32, b as f32).unwrap(), || ());
    qualify(|a, b| F64Enclosure::new(a.into(), b.into()).unwrap(), || ());
}

#[cfg(feature = "fixed")]
#[test]
/// Recover exact values by composing before projecting unrepresentable intermediates.
/// An exact quarter lies between fixed half-grid points, yet subsequent doubling
/// returns the original half. A doubled maximum i32 point overflows on projection,
/// but a later zero map collapses the retained definition to exact zero.
fn off_grid_definitions_and_overflow_remain_consumable() {
    use hyperlattice_interval::IntervalLatticeError;
    use hyperlattice_interval::hyperinterval::{FixedInterval, IntervalError};
    /// Half-unit fixed grid whose exact quarter-point image requires outward projection.
    type I = FixedInterval<i32, 1>;
    let point = |n| I::point(n).unwrap();
    let source = IntervalVector::new([point(1)]).unwrap(); // exact 1/2
    let half = AffineMap::new(
        IntervalMatrix::new([[point(1)]]).unwrap(),
        IntervalVector::new([point(0)]).unwrap(),
    )
    .unwrap();
    let twice = AffineMap::new(
        IntervalMatrix::new([[point(4)]]).unwrap(),
        IntervalVector::new([point(0)]).unwrap(),
    )
    .unwrap();
    let quarter = half.image(source);
    assert_eq!(
        quarter.enclosure(&mut ()).unwrap().lanes(),
        Some(&[I::new(0, 1).unwrap()])
    );
    assert_eq!(twice.image(quarter).enclosure(&mut ()).unwrap(), source);

    /// Whole-unit fixed grid for overflowing projection followed by a zero map.
    type J = FixedInterval<i32, 0>;
    let j = |a, b| J::new(a, b).unwrap();
    let large = map([[2]], [0], &j).image(IntervalVector::new([j(i32::MAX, i32::MAX)]).unwrap());
    assert_eq!(
        large.enclosure(&mut ()),
        Err(IntervalLatticeError::Scalar(IntervalError::Overflow))
    );
    let collapsed = map([[0]], [0], &j).image(large);
    assert_eq!(
        collapsed.enclosure(&mut ()).unwrap().lanes(),
        Some(&[j(0, 0)])
    );
}

#[cfg(feature = "float-enclosure")]
#[test]
/// Keep half of the least positive IEEE subnormal as an exact retained definition.
/// Its projection widens to include zero, while later doubling restores the original
/// point. Optional Serde roundtrip preserves the definition before that next map.
fn exact_subnormal_definitions_survive_projection_underflow() {
    use hyperlattice_interval::hyperinterval::{F32Enclosure, F64Enclosure};
    /// Instantiate subnormal projection/reuse assertions for matching IEEE interval/float types.
    /// Use the least positive subnormal and exact half/two coefficients; no rounded
    /// intermediate enclosure is adopted as the source of the next retained operation.
    macro_rules! case {
        ($ty:ty,$float:ty) => {{
            let p = |x| <$ty>::point(x).unwrap();
            let tiny = <$float>::from_bits(1);
            let source = IntervalVector::new([p(tiny)]).unwrap();
            let half = AffineMap::new(
                IntervalMatrix::new([[p(0.5)]]).unwrap(),
                IntervalVector::new([p(0.)]).unwrap(),
            )
            .unwrap();
            let twice = AffineMap::new(
                IntervalMatrix::new([[p(2.)]]).unwrap(),
                IntervalVector::new([p(0.)]).unwrap(),
            )
            .unwrap();
            let subgrid = half.image(source);
            #[cfg(feature = "serde")]
            {
                let text = serde_json::to_string(&subgrid).unwrap();
                let replay: hyperlattice_interval::AffineBox<$ty, 1, 1> =
                    serde_json::from_str(&text).unwrap();
                assert_eq!(replay, subgrid);
                assert_eq!(twice.image(replay).enclosure(&mut ()).unwrap(), source);
            }
            assert_eq!(
                subgrid.enclosure(&mut ()).unwrap().lanes(),
                Some(&[<$ty>::new(0., tiny).unwrap()])
            );
            assert_eq!(twice.image(subgrid).enclosure(&mut ()).unwrap(), source);
        }};
    }
    case!(F32Enclosure, f32);
    case!(F64Enclosure, f64);
}

#[cfg(feature = "fixed")]
#[test]
/// Check typed map validation and nonempty images under dimension-changing/singular maps.
/// Empty and nonsingleton coefficients fail with explicit messages and no nested
/// cause. Unit-domain constant maps, maps to dimension zero, empty restriction and
/// rank-one cancellation exercise distinct set semantics with literal expectations.
fn invalid_maps_empty_units_and_singular_outputs_are_distinct() {
    use core::error::Error;
    use hyperlattice_interval::IntervalLatticeError;
    use hyperlattice_interval::hyperinterval::FixedInterval;
    /// Whole-unit fixed backend for validation, dimension-change and rank-one fixtures.
    type I = FixedInterval<i32, 0>;
    let p = |a, b| I::new(a, b).unwrap();
    let zero = IntervalVector::new([p(0, 0)]).unwrap();
    for bad in [
        IntervalMatrix::empty().unwrap(),
        IntervalMatrix::new([[p(0, 1)]]).unwrap(),
    ] {
        let error = AffineMap::new(bad, zero).unwrap_err();
        assert_eq!(error, IntervalLatticeError::NonPointAffineMatrix);
        assert!(error.source().is_none());
        assert_eq!(
            error.to_string(),
            "affine matrix must contain exactly one matrix"
        );
    }
    for bad in [
        IntervalVector::empty().unwrap(),
        IntervalVector::new([p(0, 1)]).unwrap(),
    ] {
        let error = AffineMap::new(IntervalMatrix::new([[p(1, 1)]]).unwrap(), bad).unwrap_err();
        assert_eq!(error, IntervalLatticeError::NonPointAffineTranslation);
        assert!(error.source().is_none());
        assert_eq!(
            error.to_string(),
            "affine translation must contain exactly one vector"
        );
    }
    let unit = IntervalVector::<I, 0>::new([]).unwrap();
    let constant = map([[], []], [3, -2], &p);
    let image = constant.image(unit);
    assert!(!image.is_empty());
    assert_eq!(
        image.enclosure(&mut ()).unwrap().lanes(),
        Some(&[p(3, 3), p(-2, -2)])
    );
    assert!(
        constant
            .image(IntervalVector::empty().unwrap())
            .enclosure(&mut ())
            .unwrap()
            .is_empty()
    );
    let to_unit = map::<I, 0, 2>([], [], &p);
    let image = to_unit.image(IntervalVector::new([p(-1, 1), p(-2, 2)]).unwrap());
    assert_eq!(image.enclosure(&mut ()).unwrap(), unit);
    let twice = constant.image(image);
    assert_eq!(
        twice.enclosure(&mut ()).unwrap().lanes(),
        Some(&[p(3, 3), p(-2, -2)])
    );
    let empty = twice
        .restrict_parameters(&IntervalVector::empty().unwrap(), &mut ())
        .unwrap();
    assert!(empty.enclosure(&mut ()).unwrap().is_empty());
    let rank_one =
        map([[1, 2], [2, 4]], [0, 0], &p).image(IntervalVector::new([p(-1, 1), p(-1, 1)]).unwrap());
    let annihilate = map([[2, -1]], [0], &p).image(rank_one);
    assert_eq!(
        annihilate.enclosure(&mut ()).unwrap().lanes(),
        Some(&[p(0, 0)])
    );
}

#[cfg(feature = "dyadic")]
#[test]
/// Check the retained map 30*x+42, ordered restriction copies and shared dyadic budgets.
/// Literal full/selected bounds supply independent expectations. One fewer work
/// unit fails, the measured allowance succeeds and a one-bit size cap fails distinctly.
/// Queries leave the original domain reusable; an already restricted empty image
/// projects without work. Measured admission is not an independent geometry oracle.
fn nested_queries_and_owned_restrictions_share_budget_and_retry() {
    use hyperlattice_interval::IntervalLatticeError;
    use hyperlattice_interval::hyperinterval::{DyadicBudget, DyadicInterval, IntervalError};
    let scalar = |a: i32, b: i32| DyadicInterval::new(a.into(), b.into(), 0).unwrap();
    let source = IntervalVector::new([scalar(-1, 2)]).unwrap();
    let image = map([[5]], [7], &scalar)
        .image(map([[3]], [4], &scalar).image(map([[2]], [1], &scalar).image(source.clone())));
    let mut budget = DyadicBudget::new(256, 1_000_000);
    let expected = image.enclosure(&mut budget).unwrap();
    assert_eq!(expected.lanes(), Some(&[scalar(12, 102)])); // 30*x+42
    let spent = 1_000_000 - budget.remaining_work();
    assert_eq!(
        image.enclosure(&mut DyadicBudget::new(256, spent - 1)),
        Err(IntervalLatticeError::Scalar(IntervalError::WorkExhausted))
    );
    let mut exact = DyadicBudget::new(256, spent);
    assert_eq!(image.enclosure(&mut exact).unwrap(), expected);
    assert_eq!(exact.remaining_work(), 0);
    assert_eq!(
        image.enclosure(&mut DyadicBudget::new(1, 1_000_000)),
        Err(IntervalLatticeError::Scalar(IntervalError::SizeExhausted))
    );
    let selection = IntervalVector::new([scalar(0, 1)]).unwrap();
    let mut budget = DyadicBudget::new(256, 1_000_000);
    let selected = image.restrict_parameters(&selection, &mut budget).unwrap();
    assert_eq!(
        selected
            .enclosure(&mut DyadicBudget::new(256, 1_000_000))
            .unwrap()
            .lanes(),
        Some(&[scalar(42, 72)])
    );
    let spent = 1_000_000 - budget.remaining_work();
    assert_eq!(
        image.restrict_parameters(&selection, &mut DyadicBudget::new(256, spent - 1)),
        Err(IntervalLatticeError::Scalar(IntervalError::WorkExhausted))
    );
    assert_eq!(
        image
            .restrict_parameters(&selection, &mut DyadicBudget::new(256, spent))
            .unwrap(),
        selected
    );
    assert_eq!(image.parameter_domain(), &source);
    assert_eq!(
        image
            .enclosure(&mut DyadicBudget::new(256, 1_000_000))
            .unwrap(),
        expected
    );
    let empty = image
        .restrict_parameters(
            &IntervalVector::empty().unwrap(),
            &mut DyadicBudget::new(256, 1_000_000),
        )
        .unwrap();
    assert!(
        empty
            .enclosure(&mut DyadicBudget::new(1, 0))
            .unwrap()
            .is_empty()
    );
}
