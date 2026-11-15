use hyperreal::Real;
use hypervoxel::{CellBounds, ExactAffineTransform, HypervoxelError};

#[test]
fn finite_coefficient_can_have_undecided_exact_order() {
    // Independent sine addition identity, deliberately outside the scalar
    // normal form. Keep this fixture self-contained for packaged tests.
    let zero = (Real::e() + Real::one()).sin()
        - Real::e().sin() * Real::one().cos()
        - Real::e().cos() * Real::one().sin();
    assert!(zero.certified_dyadic_interval(-16).is_some());
    let transform = ExactAffineTransform::new(
        [
            [zero, 0.into(), 0.into()],
            [0.into(), 1.into(), 0.into()],
            [0.into(), 0.into(), 1.into()],
        ],
        [0.into(), 0.into(), 0.into()],
    );
    let bounds = CellBounds {
        min: [1.into(), 1.into(), 1.into()],
        max: [2.into(), 2.into(), 2.into()],
    };
    assert!(matches!(
        transform.map_bounds(&bounds),
        Err(HypervoxelError::UnknownScalarOrdering {
            field: "affine term order"
        })
    ));
    #[cfg(feature = "interval-enclosure")]
    {
        use hyperlattice::interval::hyperinterval::{DyadicBudget, DyadicInterval};
        let mut transform = transform;
        transform.translation[0] = Real::from(3);
        let source = bounds;
        let mut arithmetic = DyadicBudget::new(4096, 1_000_000_000);
        let result = transform
            .enclose_bounds(&source, -16, &mut arithmetic)
            .unwrap();
        let lanes = result.bounds().lanes().unwrap();
        // The exact x image is {3}; its certified enclosure can be wider.
        assert!(
            DyadicInterval::new(3.into(), 3.into(), 0)
                .unwrap()
                .subset_of(&lanes[0], &mut arithmetic)
                .unwrap()
        );
        assert!(
            lanes[0]
                .subset_of(
                    &DyadicInterval::new(2.into(), 4.into(), 0).unwrap(),
                    &mut arithmetic
                )
                .unwrap()
        );
        assert_eq!(
            lanes[1],
            DyadicInterval::new(1.into(), 2.into(), 0).unwrap()
        );
        assert_eq!(
            lanes[2],
            DyadicInterval::new(1.into(), 2.into(), 0).unwrap()
        );
        assert_eq!(result.precision(), -16);
    }
}

#[cfg(feature = "interval-enclosure")]
mod native {
    use super::*;
    use hyperlattice::interval::hyperinterval::{DyadicBudget, DyadicInterval, IntervalError};
    use hyperlattice::{IntervalLatticeError, IntervalMatrix, IntervalVector};

    fn budget() -> DyadicBudget {
        DyadicBudget::new(4096, 1_000_000_000)
    }
    fn interval(lo: i64, hi: i64, exponent: i32) -> DyadicInterval {
        DyadicInterval::new(lo.into(), hi.into(), exponent).unwrap()
    }
    fn unit() -> CellBounds {
        CellBounds {
            min: [0.into(), 0.into(), 0.into()],
            max: [1.into(), 1.into(), 1.into()],
        }
    }

    #[test]
    fn generated_integer_maps_match_independent_corner_extrema() {
        for seed in -8_i64..=8 {
            let coefficients = core::array::from_fn::<_, 3, _>(|r| {
                core::array::from_fn::<_, 3, _>(|c| (seed + 3 * r as i64 - 2 * c as i64) % 5)
            });
            let translation = [seed, -seed, 2 * seed];
            let lo = [-3, 1, -2];
            let hi = [2, 4, 5];
            let transform = ExactAffineTransform::new(
                coefficients.map(|row| row.map(Real::from)),
                translation.map(Real::from),
            );
            let source = CellBounds {
                min: lo.map(Real::from),
                max: hi.map(Real::from),
            };
            let result = transform.enclose_bounds(&source, 0, &mut budget()).unwrap();
            for axis in 0..3 {
                let values: Vec<i64> = (0..8)
                    .map(|corner| {
                        translation[axis]
                            + (0..3)
                                .map(|j| {
                                    coefficients[axis][j]
                                        * if corner & (1 << j) == 0 { lo[j] } else { hi[j] }
                                })
                                .sum::<i64>()
                    })
                    .collect();
                assert_eq!(
                    result.bounds().lanes().unwrap()[axis],
                    interval(
                        *values.iter().min().unwrap(),
                        *values.iter().max().unwrap(),
                        0
                    ),
                    "seed {seed}, axis {axis}"
                );
            }
            let reversed = CellBounds {
                min: source.max.clone(),
                max: source.min.clone(),
            };
            assert_eq!(
                transform
                    .enclose_bounds(&reversed, 0, &mut budget())
                    .unwrap()
                    .bounds(),
                result.bounds()
            );
            assert_eq!(result.source(), &source);
        }
    }

    #[test]
    fn rational_algebraic_and_singular_images_keep_exact_sources() {
        let q = Real::from(hyperreal::Rational::fraction(1, 3).unwrap());
        let sqrt2 = Real::from(2).sqrt().unwrap();
        let transform = ExactAffineTransform::new(
            [
                [sqrt2.clone(), 0.into(), 0.into()],
                [0.into(), q.clone(), 0.into()],
                [0.into(), 0.into(), 0.into()],
            ],
            [0.into(), 0.into(), q.clone()],
        );
        let result = transform
            .enclose_bounds(&unit(), -12, &mut budget())
            .unwrap();
        let a = result.bounds().lanes().unwrap();
        // 5792^2 < 2*4096^2 < 5793^2 and 1365/4096 < 1/3 < 1366/4096.
        // Hyperreal's certified enclosure need not be the tightest grid hull.
        // The independently squared rational upper bound encloses sqrt(2).
        assert!(
            interval(0, 5793, -12)
                .subset_of(&a[0], &mut budget())
                .unwrap()
        );
        assert!(a[0].subset_of(&interval(0, 3, -1), &mut budget()).unwrap());
        assert_eq!(a[1], interval(0, 1366, -12));
        assert_eq!(a[2], interval(1365, 1366, -12));
        assert_eq!(result.transform().matrix[0][0], sqrt2);
        assert_eq!(result.transform().translation[2], q);
        let point = CellBounds {
            min: [0.into(), 0.into(), 0.into()],
            max: [0.into(), 0.into(), 0.into()],
        };
        assert_eq!(
            transform
                .enclose_bounds(&point, -12, &mut budget())
                .unwrap()
                .bounds()
                .lanes()
                .unwrap()[0],
            interval(0, 0, 0)
        );
    }

    #[test]
    fn retained_definition_is_immutable_and_output_is_composable() {
        let mut transform = ExactAffineTransform::identity();
        let mut source = unit();
        let result = transform.enclose_bounds(&source, 0, &mut budget()).unwrap();
        transform.translation[0] = 50.into();
        source.min[0] = 25.into();
        assert_eq!(result.source(), &unit());
        assert_eq!(result.transform().translation[0], Real::zero());
        let map = IntervalMatrix::new([
            [interval(1, 1, 0), interval(-2, -2, 0), interval(0, 0, 0)],
            [interval(0, 0, 0), interval(1, 1, 0), interval(0, 0, 0)],
            [interval(0, 0, 0), interval(0, 0, 0), interval(0, 0, 0)],
        ])
        .unwrap();
        let next = map
            .mul_vector(result.bounds(), &mut budget())
            .unwrap()
            .add(
                &IntervalVector::new([interval(3, 3, 0), interval(0, 0, 0), interval(7, 7, 0)])
                    .unwrap(),
                &mut budget(),
            )
            .unwrap();
        assert_eq!(
            next.lanes().unwrap(),
            &[interval(1, 4, 0), interval(0, 1, 0), interval(7, 7, 0)]
        );
    }

    #[test]
    fn budget_failures_preserve_typed_causes_and_independent_retry() {
        let transform = ExactAffineTransform::identity();
        let mut ample = budget();
        let start = ample.remaining_work();
        let expected = transform.enclose_bounds(&unit(), -16, &mut ample).unwrap();
        let used = start - ample.remaining_work();
        let mut exact = DyadicBudget::new(4096, used);
        assert_eq!(
            transform
                .enclose_bounds(&unit(), -16, &mut exact)
                .unwrap()
                .bounds(),
            expected.bounds()
        );
        assert_eq!(exact.remaining_work(), 0);
        let error = transform
            .enclose_bounds(&unit(), -16, &mut DyadicBudget::new(4096, used - 1))
            .unwrap_err();
        assert!(matches!(
            error,
            HypervoxelError::IntervalEnclosure(IntervalLatticeError::Scalar(
                IntervalError::WorkExhausted
            ))
        ));
        assert!(
            std::error::Error::source(&error)
                .unwrap()
                .source()
                .is_some()
        );
        assert!(matches!(
            transform.enclose_bounds(&unit(), i32::MIN, &mut budget()),
            Err(HypervoxelError::IntervalEnclosure(
                IntervalLatticeError::Scalar(IntervalError::SizeExhausted)
            ))
        ));
        assert_eq!(
            transform
                .enclose_bounds(&unit(), -16, &mut budget())
                .unwrap()
                .bounds(),
            expected.bounds()
        );
    }

    #[test]
    fn fresh_process_reconstructs_the_definition_and_bounds() {
        let output = std::process::Command::new(std::env::current_exe().unwrap())
            .args([
                "--exact",
                "native::rational_algebraic_and_singular_images_keep_exact_sources",
                "--nocapture",
            ])
            .output()
            .unwrap();
        assert!(
            output.status.success(),
            "{}",
            String::from_utf8_lossy(&output.stderr)
        );
        assert!(String::from_utf8_lossy(&output.stdout).contains("1 passed"));
    }
}
