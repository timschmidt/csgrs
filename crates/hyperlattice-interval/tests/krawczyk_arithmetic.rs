#![cfg(any(feature = "rational", feature = "dyadic"))]
use hyperlattice_interval::{
    IntervalArithmetic, IntervalLatticeError, IntervalMatrix, IntervalVector,
};

// Arithmetic client only: callers must establish point x0/C, f(x0), and an
// enclosing Jacobian on the relevant box before interpreting a Krawczyk image.
fn image<I: IntervalArithmetic, const N: usize>(
    c: &IntervalMatrix<I, N, N>,
    j: &IntervalMatrix<I, N, N>,
    x: &IntervalVector<I, N>,
    x0: &IntervalVector<I, N>,
    f0: &IntervalVector<I, N>,
    context: &mut I::Context,
) -> Result<IntervalVector<I, N>, IntervalLatticeError> {
    let center = x0.sub(&c.mul_vector(f0, context)?, context)?;
    let remainder = c.mul_matrix(j, context)?.identity_minus(context)?;
    remainder.affine_enclosure(&x.sub(x0, context)?, &center, context)
}

#[cfg(feature = "rational")]
mod rational {
    use super::*;
    use hyperlattice_interval::hyperinterval::RationalInterval as I;
    fn q(a: &str, b: &str) -> I {
        I::new(a.parse().unwrap(), b.parse().unwrap()).unwrap()
    }
    type Matrix = IntervalMatrix<I, 2, 2>;
    type Vector = IntervalVector<I, 2>;
    type Inputs = (Matrix, Matrix, Vector, Vector, Vector);

    fn inputs() -> Inputs {
        // f=(x²+y-3, x+y²-5), exact root (1,2), C=J(1,2)^-1.
        // The radii 1/8 and 1/4 are deliberately unequal.
        (
            Matrix::new([
                [q("4/7", "4/7"), q("-1/7", "-1/7")],
                [q("-1/7", "-1/7"), q("2/7", "2/7")],
            ])
            .unwrap(),
            Matrix::new([
                [q("7/4", "9/4"), q("1", "1")],
                [q("1", "1"), q("7/2", "9/2")],
            ])
            .unwrap(),
            Vector::new([q("7/8", "9/8"), q("7/4", "9/4")]).unwrap(),
            Vector::new([q("1", "1"), q("2", "2")]).unwrap(),
            Vector::new([q("0", "0"), q("0", "0")]).unwrap(),
        )
    }

    fn qualify((c, j, x, x0, f0): &Inputs) {
        let remainder = c
            .mul_matrix(j, &mut ())
            .unwrap()
            .identity_minus(&mut ())
            .unwrap();
        assert_eq!(
            remainder,
            Matrix::new([
                [q("-1/7", "1/7"), q("-1/14", "1/14")],
                [q("-1/28", "1/28"), q("-1/7", "1/7")],
            ])
            .unwrap()
        );
        assert_eq!(
            remainder.contraction_margin_one(&mut ()).unwrap(),
            q("11/14", "1")
        );
        assert_eq!(
            remainder.contraction_margin_infinity(&mut ()).unwrap(),
            q("11/14", "1")
        );
        let k = image(c, j, x, x0, f0, &mut ()).unwrap();
        // Independent exact hand derivation: row radii are 1/28 and 9/224.
        assert_eq!(
            k,
            Vector::new([q("27/28", "29/28"), q("439/224", "457/224")]).unwrap()
        );
        assert!(k.interior_subset_of(x, &mut ()).unwrap());
        let restricted = x.intersection(&k, &mut ()).unwrap();
        let next = image(c, j, &restricted, x0, f0, &mut ()).unwrap();
        assert_eq!(
            next,
            Vector::new([q("3111/3136", "3161/3136"), q("6250/3136", "6294/3136")]).unwrap()
        );
    }

    #[test]
    fn coupled_quadratic_unequal_radius_image_and_reuse() {
        qualify(&inputs());
    }

    #[test]
    fn two_roots_and_nonzero_residual_do_not_become_contraction() {
        // f=x²-1, x0=1, C=1/2 on [-2,2] contains both roots.
        let c = IntervalMatrix::new([[q("1/2", "1/2")]]).unwrap();
        let j = IntervalMatrix::new([[q("-4", "4")]]).unwrap();
        let x = IntervalVector::new([q("-2", "2")]).unwrap();
        let x0 = IntervalVector::new([q("1", "1")]).unwrap();
        let f0 = IntervalVector::new([q("0", "0")]).unwrap();
        let remainder = c
            .mul_matrix(&j, &mut ())
            .unwrap()
            .identity_minus(&mut ())
            .unwrap();
        assert_eq!(
            remainder.contraction_margin_infinity(&mut ()).unwrap(),
            q("-2", "1")
        );
        let k = image(&c, &j, &x, &x0, &f0, &mut ()).unwrap();
        assert_eq!(k.lanes(), Some(&[q("-8", "4")]));
        assert!(!k.subset_of(&x, &mut ()).unwrap());
        // f=2x-3, x0=1, f0=-1: the Newton term must not be discarded.
        let j = IntervalMatrix::new([[q("2", "2")]]).unwrap();
        let f0 = IntervalVector::new([q("-1", "-1")]).unwrap();
        assert_eq!(
            image(&c, &j, &x, &x0, &f0, &mut ()).unwrap().lanes(),
            Some(&[q("3/2", "3/2")])
        );
        assert!(
            image(&c, &j, &IntervalVector::empty().unwrap(), &x0, &f0, &mut ())
                .unwrap()
                .is_empty()
        );
    }

    #[cfg(feature = "serde")]
    #[test]
    fn replay_recomputes_arithmetic_in_a_fresh_process() {
        const KEY: &str = "HYPERLATTICE_CONTRACTION_REPLAY";
        if let Ok(text) = std::env::var(KEY) {
            let values: Inputs = serde_json::from_str(&text).unwrap();
            assert_eq!(serde_json::to_string(&values).unwrap(), text);
            qualify(&values);
            return;
        }
        let output = std::process::Command::new(std::env::current_exe().unwrap())
            .args([
                "--exact",
                "rational::replay_recomputes_arithmetic_in_a_fresh_process",
                "--nocapture",
            ])
            .env(KEY, serde_json::to_string(&inputs()).unwrap())
            .output()
            .unwrap();
        assert!(
            output.status.success(),
            "{} {}",
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
    }
}

#[cfg(feature = "dyadic")]
#[test]
fn composed_dyadic_client_shares_budget_and_retries_unchanged_inputs() {
    use hyperlattice_interval::hyperinterval::{DyadicBudget, DyadicInterval as I, IntervalError};
    let q = |a: i32, b: i32, e| I::new(a.into(), b.into(), e).unwrap();
    let c = IntervalMatrix::new([[q(1, 1, -1), q(0, 0, 0)], [q(0, 0, 0), q(1, 1, -2)]]).unwrap();
    let j = IntervalMatrix::new([[q(7, 9, -2), q(0, 0, 0)], [q(0, 0, 0), q(7, 9, -1)]]).unwrap();
    let x = IntervalVector::new([q(7, 9, -3), q(7, 9, -2)]).unwrap();
    let x0 = IntervalVector::new([q(1, 1, 0), q(2, 2, 0)]).unwrap();
    let f0 = IntervalVector::new([q(0, 0, 0), q(0, 0, 0)]).unwrap();
    let expected = IntervalVector::new([q(63, 65, -6), q(63, 65, -5)]).unwrap();
    let mut budget = DyadicBudget::new(128, 1_000_000);
    assert_eq!(image(&c, &j, &x, &x0, &f0, &mut budget).unwrap(), expected);
    let used = 1_000_000 - budget.remaining_work();
    assert!(used > 0);
    assert_eq!(
        image(&c, &j, &x, &x0, &f0, &mut DyadicBudget::new(128, used - 1)),
        Err(IntervalLatticeError::Scalar(IntervalError::WorkExhausted))
    );
    assert_eq!(
        image(&c, &j, &x, &x0, &f0, &mut DyadicBudget::new(1, 1_000_000)),
        Err(IntervalLatticeError::Scalar(IntervalError::SizeExhausted))
    );
    let mut retry = DyadicBudget::new(128, used);
    assert_eq!(image(&c, &j, &x, &x0, &f0, &mut retry).unwrap(), expected);
    assert_eq!(retry.remaining_work(), 0);
    assert_eq!(x.lanes().unwrap(), &[q(7, 9, -3), q(7, 9, -2)]);
}
