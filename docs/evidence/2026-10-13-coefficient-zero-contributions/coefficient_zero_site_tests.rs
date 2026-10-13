use super::*;
use crate::coefficient_zero_diagnostic::{Report, Snapshot};

fn coefficient(shape: usize, zero: bool) -> RadicalCoefficient {
    let atom = Computable {
        internal: Arc::new(Node::new(
            Approximation::AtanRational(Rational::fraction(1, 37).unwrap()),
            BoundCache::Invalid,
            ExactSignCache::Unknown,
        )),
        signal: None,
    };
    RadicalCoefficient {
        rational: Rational::from(i32::from(!zero)),
        radicals: if shape & 1 != 0 {
            vec![
                (Rational::from(2), Rational::one()),
                (Rational::from(3), Rational::from(2)),
            ]
        } else {
            Vec::new()
        },
        atoms: if shape & 2 != 0 {
            vec![(atom, Rational::from(3))]
        } else {
            Vec::new()
        },
    }
}

#[test]
fn coefficient_zero_sites_match_all_shapes_and_stored_zero_flags() {
    for left in 0..4 {
        for right in 0..4 {
            for flags in 0..4 {
                // Construct inputs while recording is inactive.
                let left_zero = flags & 2 != 0;
                let right_zero = flags & 1 != 0;
                let first = coefficient(left, left_zero);
                let second = coefficient(right, right_zero);
                let report = Report::begin();
                let product = first.multiply(&second);
                let actual = report.finish();
                let eligible = matches!(
                    (left, right),
                    (0, 0) | (0, 1) | (0, 2) | (0, 3) | (1, 0) | (1, 1) | (2, 0) | (3, 0)
                );
                assert_eq!(product.is_some(), eligible);
                let mut expected = Snapshot {
                    eligible: [[0; 5]; 64],
                    linear: [[0; 2]; 4],
                };
                if eligible {
                    let lr = u64::from(left & 1 != 0) * 2;
                    let la = u64::from(left & 2 != 0);
                    let rr = u64::from(right & 1 != 0) * 2;
                    let ra = u64::from(right & 2 != 0);
                    expected.eligible[(4 * left + right) * 4 + flags] = [1, lr, la, rr, ra];
                    expected.linear = [
                        [lr, lr * u64::from(right_zero)],
                        [rr, rr * u64::from(left_zero)],
                        [la, la * u64::from(right_zero)],
                        [ra, ra * u64::from(left_zero)],
                    ];
                }
                assert_eq!(actual, expected, "shape/zero case {left}/{right}/{flags}");
            }
        }
    }
}

#[test]
fn coefficient_zero_sites_count_actual_entries_before_insertion_declines() {
    for left_zero in [false, true] {
        let first = RadicalCoefficient {
            rational: Rational::from(i32::from(!left_zero)),
            radicals: [2, 3, 5, 7, 11, 13, 17, 19]
                .into_iter()
                .map(|value| (Rational::from(value), Rational::one()))
                .collect(),
            atoms: Vec::new(),
        };
        let second = RadicalCoefficient {
            rational: Rational::one(),
            radicals: [23, 29, 31]
                .into_iter()
                .map(|value| (Rational::from(value), Rational::one()))
                .collect(),
            atoms: Vec::new(),
        };
        let report = Report::begin();
        assert!(first.multiply(&second).is_none());
        let actual = report.finish();
        let index = 20 + 2 * usize::from(left_zero);
        assert_eq!(actual.eligible[index], [1, 8, 0, 3, 0]);
        assert_eq!(actual.eligible.iter().map(|row| row[0]).sum::<u64>(), 1);
        assert_eq!(
            actual.linear,
            if left_zero {
                // Three zero products precede the later radical cross-product decline.
                [[8, 0], [3, 3], [0, 0], [0, 0]]
            } else {
                // The first new right radicand exceeds the eight-term limit.
                [[8, 0], [1, 0], [0, 0], [0, 0]]
            }
        );
    }
}
