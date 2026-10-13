use super::*;
use num::BigUint;

fn raw_coefficient(a: Rational, b: Rational, root: Rational) -> RadicalCoefficient {
    RadicalCoefficient {
        rational: a,
        radicals: vec![(root, b)],
        atoms: Vec::new(),
    }
}

fn small(a: i64, b: i64, root: i64) -> RadicalCoefficient {
    raw_coefficient(Rational::new(a), Rational::new(b), Rational::new(root))
}

fn atom(index: i64) -> Computable {
    Computable {
        internal: Arc::new(Node::new(
            Approximation::AtanRational(Rational::fraction(index + 1, 37).unwrap()),
            BoundCache::Invalid,
            ExactSignCache::Unknown,
        )),
        signal: None,
    }
}

fn assert_same(expected: Option<RadicalCoefficient>, actual: Option<RadicalCoefficient>) {
    match (expected, actual) {
        (None, None) => {}
        (Some(expected), Some(actual)) => {
            assert_eq!(actual.rational, expected.rational);
            assert_eq!(actual.radicals, expected.radicals);
            assert_eq!(actual.atoms.len(), expected.atoms.len());
            for ((a, x), (b, y)) in actual.atoms.iter().zip(&expected.atoms) {
                assert!(Computable::internal_structural_eq(a, b));
                assert_eq!(x, y);
                assert!(a.cached().is_none());
                assert!(b.cached().is_none());
            }
            for ((a, _), (b, _)) in actual.radicals.iter().zip(&expected.radicals) {
                assert_eq!(a.is_internally_unreduced(), b.is_internally_unreduced());
            }
        }
        _ => panic!("original and candidate disagree on recognition success"),
    }
}

// The original general multiplier is retained only as a differential test oracle.
impl RadicalCoefficient {
    fn original_product(self, other: &Self) -> Option<Self> {
        if (!self.atoms.is_empty() && (!other.radicals.is_empty() || !other.atoms.is_empty()))
            || (!other.atoms.is_empty() && !self.radicals.is_empty())
        {
            return None;
        }
        let mut result = Self::rational(&self.rational * &other.rational);
        for (radicand, coefficient) in &self.radicals {
            result.insert_radical(radicand.clone(), coefficient * &other.rational)?;
        }
        for (radicand, coefficient) in &other.radicals {
            result.insert_radical(radicand.clone(), coefficient * &self.rational)?;
        }
        for (left, left_coefficient) in self.radicals {
            for (right, right_coefficient) in &other.radicals {
                let coefficient = &left_coefficient * right_coefficient;
                if left == *right {
                    result.rational = result.rational + coefficient * &left;
                } else {
                    let (scale, radicand) = (&left * right).extract_square_reduced_retained();
                    result.insert_radical(radicand, coefficient * scale)?;
                }
            }
        }
        for (atom, coefficient) in self.atoms {
            result.insert_atom(atom, coefficient * &other.rational);
        }
        for (atom, coefficient) in &other.atoms {
            result.insert_atom(atom.clone(), coefficient * &self.rational);
        }
        Some(result)
    }
}

#[test]
fn affine_conjugate_and_unit_products_have_independent_exact_answers() {
    let affine = small(3, 2, 13).multiply(&small(5, 7, 13)).unwrap();
    assert_eq!(affine.rational, Rational::new(197));
    assert_eq!(
        affine.radicals,
        vec![(Rational::new(13), Rational::new(31))]
    );
    let conjugate = small(3, 2, 13).multiply(&small(3, -2, 13)).unwrap();
    assert_eq!(conjugate.rational, Rational::new(-43));
    assert!(conjugate.radicals.is_empty());
    let pure = small(0, 2, 13).multiply(&small(0, 7, 13)).unwrap();
    assert_eq!(pure.rational, Rational::new(182));
    assert!(pure.radicals.is_empty());
    let unit = small(3, 2, 1).multiply(&small(5, 7, 1)).unwrap();
    assert_eq!(unit.rational, Rational::new(60));
    assert!(unit.radicals.is_empty());
}

#[test]
fn signed_zero_and_fractional_grid_matches_original_general_multiplication() {
    let values = [
        Rational::new(-3),
        Rational::zero(),
        Rational::one(),
        Rational::fraction(2, 7).unwrap(),
    ];
    for root in [0, 1, 2, 9, 13] {
        for a in &values {
            for b in &values {
                for c in &values {
                    for d in &values {
                        let left = raw_coefficient(a.clone(), b.clone(), Rational::new(root));
                        let right = raw_coefficient(c.clone(), d.clone(), Rational::new(root));
                        assert_same(left.clone().original_product(&right), left.multiply(&right));
                    }
                }
            }
        }
    }
}

#[test]
fn unequal_and_wide_radical_shapes_keep_order_and_original_declines() {
    let roots = [2, 3, 5, 7, 11, 13, 17, 19, 23];
    for left_width in [0, 1, 2, 8, 9] {
        for right_width in [0, 1, 2, 8, 9] {
            for rational in [-1, 0, 2] {
                let make = |width, reverse| RadicalCoefficient {
                    rational: Rational::new(rational),
                    radicals: (0..width)
                        .map(|i| {
                            let index = if reverse { 8 - i } else { i };
                            (Rational::new(roots[index]), Rational::new(rational + 1))
                        })
                        .collect(),
                    atoms: Vec::new(),
                };
                let left = make(left_width, false);
                let right = make(right_width, true);
                assert_same(left.clone().original_product(&right), left.multiply(&right));
            }
        }
    }
    let product = small(1, 1, 2).multiply(&small(1, 1, 3)).unwrap();
    assert_eq!(product.rational, Rational::one());
    assert_eq!(
        product.radicals,
        [2, 3, 6].map(|root| (Rational::new(root), Rational::one()))
    );
}

#[test]
fn equal_distinct_payloads_follow_the_first_surviving_linear_contribution() {
    for (a, c, take_right) in [(3, 0, true), (0, 5, false), (3, 5, false)] {
        let fresh_root = || {
            Rational::from_parts_raw_unreduced(
                Sign::Plus,
                BigUint::from(26_u8),
                BigUint::from(2_u8),
            )
        };
        let left_root = fresh_root();
        let right_root = fresh_root();
        assert_eq!(left_root, right_root);
        assert_ne!(left_root.storage_identity(), right_root.storage_identity());
        let expected_identity = if take_right {
            right_root.storage_identity()
        } else {
            left_root.storage_identity()
        };
        let left = raw_coefficient(Rational::new(a), Rational::new(2), left_root);
        let right = raw_coefficient(Rational::new(c), Rational::new(7), right_root);
        let expected = left.clone().original_product(&right).unwrap();
        let actual = left.multiply(&right).unwrap();
        assert_eq!(expected.radicals[0].0.storage_identity(), expected_identity);
        assert_eq!(actual.radicals[0].0.storage_identity(), expected_identity);
        assert_same(Some(expected), Some(actual));
    }
}

#[test]
fn consumed_vector_reuse_and_empty_atom_capacity_are_observable() {
    let mut left = small(3, 2, 13);
    left.atoms = Vec::with_capacity(3);
    let pointer = left.radicals.as_ptr() as usize;
    let capacity = left.radicals.capacity();
    let right = small(5, 7, 13);
    let actual = left.multiply(&right).unwrap();
    assert_eq!(actual.rational, Rational::new(197));
    assert_eq!(
        actual.radicals,
        vec![(Rational::new(13), Rational::new(31))]
    );
    assert_eq!(actual.atoms.capacity(), 0);
    assert_eq!(right.rational, Rational::new(5));
    assert_eq!(right.radicals, vec![(Rational::new(13), Rational::new(7))]);
    println!(
        "SAME_RADICAND_STORAGE\t{}\t{}\t{}\t{}",
        usize::from(actual.radicals.as_ptr() as usize == pointer),
        capacity,
        actual.radicals.capacity(),
        actual.atoms.capacity()
    );
}

#[test]
fn cancelled_products_discard_radical_and_atom_entries() {
    let mut left = small(3, 2, 13);
    left.radicals.reserve(7);
    left.atoms = Vec::with_capacity(3);
    let actual = left.multiply(&small(3, -2, 13)).unwrap();
    assert_eq!(actual.rational, Rational::new(-43));
    assert!(actual.radicals.is_empty());
    assert!(actual.atoms.is_empty());
    assert_eq!(actual.atoms.capacity(), 0);
    println!(
        "SAME_RADICAND_CANCEL\t{}\t{}",
        actual.radicals.capacity(),
        actual.atoms.capacity()
    );
}

#[test]
fn wide_unreduced_and_aliased_coefficients_keep_exact_results() {
    let raw = Rational::from_parts_raw_unreduced(
        Sign::Plus,
        (BigUint::from(1_u8) << 300_u32) + BigUint::from(14_u8),
        BigUint::from(14_u8),
    );
    let dyadic = Rational::from_parts_raw_unreduced(
        Sign::Minus,
        BigUint::from(17_u8),
        BigUint::from(1_u8) << 1100_u32,
    );
    let root = Rational::from_parts_raw_unreduced(
        Sign::Plus,
        BigUint::from(13_u8) << 512_u32,
        BigUint::from(1_u8) << 512_u32,
    );
    let values = [raw, dyadic, root.clone(), Rational::zero(), Rational::one()];
    for a in &values {
        for b in &values {
            let left = raw_coefficient(a.clone(), b.clone(), root.clone());
            let right = raw_coefficient(b.clone(), a.clone(), root.clone());
            assert_same(left.clone().original_product(&right), left.multiply(&right));
        }
    }
}

#[test]
fn opaque_inputs_keep_original_declines_linear_results_and_graph_release() {
    let opaque = atom(7);
    let weak = Arc::downgrade(&opaque.internal);
    for radicals in [false, true] {
        for other_radicals in [false, true] {
            let mut left = if radicals {
                small(3, 2, 13)
            } else {
                RadicalCoefficient::rational(Rational::new(3))
            };
            left.atoms.push((opaque.clone(), Rational::new(5)));
            let right = if other_radicals {
                small(3, 7, 13)
            } else {
                RadicalCoefficient::rational(Rational::new(-2))
            };
            assert_same(left.clone().original_product(&right), left.multiply(&right));
        }
    }
    assert_eq!(Arc::strong_count(&opaque.internal), 1);
    assert!(opaque.cached().is_none());
    drop(opaque);
    assert!(weak.upgrade().is_none());
}
