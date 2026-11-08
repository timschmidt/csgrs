//! Resultants retain exact selected coefficient fields and typed resource limits.

#[path = "support/recursive_quadratic.rs"]
#[allow(dead_code)]
mod support;

use hyperreal::{Real, RealSign};
use hypersolve::OrderedFieldPolynomialContext;
use hypersolve::classification::Classification;
use hypersolve::ordered_field_resultant::{
    BoundedOrderedFieldResultantError as Error, OrderedFieldResultantConfig as Config,
    resultant_bivariate_ordered_field as resultant,
    resultant_bivariate_perturbation_ordered_field as perturbation,
};
use hypersolve::recursive_quadratic_field::RecursiveQuadraticField;
use hypersolve::selected_polynomial_field::{SelectedPolynomialContext, SelectedPolynomialField};
use std::cmp::Ordering;
use support::*;

struct Scalars {
    fail: bool,
}

#[test]
fn formal_perturbation_separates_common_support_from_isolated_contacts() {
    let mut field = Scalars { fail: false };
    // F=(y²-x)(y-2). P=y-2. Res(F,P+z)=z*((2-z)²-x).
    let f = p(&[&[0, 0, -2, 1], &[2, -1]]);
    let g = p(&[&[-2, 1]]);
    let report = perturbation(&f, &g, Config::default(), &mut field)
        .unwrap()
        .unwrap();
    assert_eq!(report.first_nonzero_power, 1);
    assert_eq!(report.coefficients, vec![Real::from(4), Real::from(-1)]);
    // The selected noncommon support y²=x has no contact for x in [0,1].
    // The projection still retains the actual common-support crossing x=4.
    assert!(report.coefficient_operations > 0);
    // With no common support, the zeroth coefficient is the ordinary resultant.
    let f = p(&[&[0, 0, 1], &[-1]]);
    let report = perturbation(&f, &g, Config::default(), &mut field)
        .unwrap()
        .unwrap();
    assert_eq!(report.first_nonzero_power, 0);
    assert_eq!(report.coefficients, vec![Real::from(4), Real::from(-1)]);
    // Every root is common when P=0, including repeated roots of F.
    let report = perturbation(&f, &[], Config::default(), &mut field)
        .unwrap()
        .unwrap();
    assert_eq!(report.first_nonzero_power, 2);
    assert_eq!(report.coefficients, vec![Real::one()]);
    assert!(
        perturbation(&[], &g, Config::default(), &mut field)
            .unwrap()
            .is_none()
    );
}

#[test]
fn perturbation_preserves_degree_drop_boundary_and_repeated_common_support() {
    let mut field = Scalars { fail: false };
    // F=(xy-1)(y-x), P=x(xy-1). The common root 1/x escapes
    // at x=0, while the retained branch y=x has a genuine contact there.
    // Res(F,P+z)=x*z*(x³-x+z), so its first coefficient is x⁴-x².
    let report = perturbation(
        &p(&[&[0, -1], &[1, 0, 1], &[0, -1]]),
        &p(&[&[], &[-1], &[0, 1]]),
        Config::default(),
        &mut field,
    )
    .unwrap()
    .unwrap();
    assert_eq!(report.first_nonzero_power, 1);
    assert_eq!(
        report.coefficients,
        vec![0, 0, -1, 0, 1]
            .into_iter()
            .map(Real::from)
            .collect::<Vec<_>>()
    );
    // F=(y-2)², P=y-2: common-root multiplicity belongs to F.
    let report = perturbation(
        &p(&[&[4, -4, 1]]),
        &p(&[&[-2, 1]]),
        Config::default(),
        &mut field,
    )
    .unwrap()
    .unwrap();
    assert_eq!(report.first_nonzero_power, 2);
    assert_eq!(report.coefficients, vec![Real::one()]);
}

#[test]
fn perturbation_retains_multiple_common_factors_and_arbitrary_source_degree() {
    use hypersolve::{BivariatePolynomial, bivariate_arithmetic::bivariate_multiply};
    let mut field = Scalars { fail: false };
    for degree in [1_usize, 2, 5, 13] {
        let mut source = p(&[&[], &[-1]]);
        source[0].resize(degree + 1, Real::zero());
        source[0][degree] = Real::one();
        let common = BivariatePolynomial::new(p(&[&[-2, 1]]));
        let f = bivariate_multiply(&BivariatePolynomial::new(source), &common);
        let result = perturbation(
            &f.coefficients,
            &common.coefficients,
            Config::default(),
            &mut field,
        )
        .unwrap()
        .unwrap();
        assert_eq!(result.first_nonzero_power, 1);
        let sign = if degree.is_multiple_of(2) {
            Real::one()
        } else {
            Real::from(-1)
        };
        assert_eq!(
            result.coefficients,
            vec![Real::from(1_u64 << degree) * &sign, -sign]
        );
    }
    let common = BivariatePolynomial::new(p(&[&[6, -5, 1]])); // (y-2)(y-3)
    let f = bivariate_multiply(&BivariatePolynomial::new(p(&[&[0, 0, 1], &[-1]])), &common);
    let result = perturbation(
        &f.coefficients,
        &common.coefficients,
        Config::default(),
        &mut field,
    )
    .unwrap()
    .unwrap();
    assert_eq!(result.first_nonzero_power, 2);
    assert_eq!(
        result.coefficients,
        vec![36, -13, 1]
            .into_iter()
            .map(Real::from)
            .collect::<Vec<_>>()
    );
    // Equal eliminated degrees exercise the shared Bezout determinant.
    let result = perturbation(
        &p(&[&[0, -2, 1], &[2, -1]]),
        &p(&[&[-2, -1, 1]]),
        Config::default(),
        &mut field,
    )
    .unwrap()
    .unwrap();
    assert_eq!(result.first_nonzero_power, 1);
    assert_eq!(
        result.coefficients,
        vec![-2, -1, 1]
            .into_iter()
            .map(Real::from)
            .collect::<Vec<_>>()
    );
}

#[test]
fn perturbation_counts_inner_field_work_and_retries_after_exhaustion() {
    let f = p(&[&[0, 0, -2, 1], &[2, -1]]);
    let g = p(&[&[-2, 1]]);
    let mut field = Scalars { fail: false };
    let report = perturbation(&f, &g, Config::default(), &mut field)
        .unwrap()
        .unwrap();
    for limit in [0, report.coefficient_operations - 1] {
        assert!(
            matches!(perturbation(&f, &g, Config { max_coefficient_operations: limit, ..Config::default() }, &mut field), Err(Error::OperationLimit { limit: actual }) if actual == limit)
        );
    }
    let retry = perturbation(
        &f,
        &g,
        Config {
            max_coefficient_operations: report.coefficient_operations,
            ..Config::default()
        },
        &mut field,
    )
    .unwrap()
    .unwrap();
    assert_eq!(retry.coefficients, report.coefficients);
    assert_eq!(retry.coefficient_operations, report.coefficient_operations);
    assert!(matches!(
        perturbation(
            &f,
            &g,
            Config {
                max_matrix_entries: 0,
                ..Config::default()
            },
            &mut field
        ),
        Err(Error::MatrixLimit {
            required: 16,
            limit: 0
        })
    ));
    field.fail = true;
    assert!(matches!(
        perturbation(&f, &g, Config::default(), &mut field),
        Err(Error::Coefficient("coefficient authority"))
    ));
}

#[test]
fn perturbation_retains_an_independently_selected_irreducible_quintic_field() {
    for policy in [TestContext::strict(), TestContext::approximate()] {
        let base = SelectedPolynomialField::from_recursive(
            RecursiveQuadraticField::base(
                Vec::new(),
                constant_tensor(0, real(1)),
                constant_tensor(0, real(1)),
            )
            .unwrap(),
        );
        // The reciprocal polynomial is Eisenstein at 2, so the selected
        // coefficient has field degree five, beyond a quadratic-tower shortcut.
        let coefficients = [-1, 2, 2, 2, 2, 2]
            .map(|n| base.constant(n.into()).unwrap())
            .to_vec();
        let Classification::Decided(field) = base
            .select(coefficients, 0.into(), 1.into(), &policy)
            .unwrap()
        else {
            panic!("one exact selected root")
        };
        let a = field.generator().unwrap();
        let c = |n| field.constant(Real::from(n)).unwrap();
        let f = vec![
            vec![c(0), c(0), a.scale(&(-1).into()).unwrap(), c(1)],
            vec![a.clone(), c(-1)],
        ];
        let g = vec![vec![a.scale(&(-1).into()).unwrap(), c(1)]];
        let mut context = SelectedPolynomialContext {
            field: field.clone(),
            policy: &policy,
        };
        let result = perturbation(&f, &g, Config::default(), &mut context)
            .unwrap()
            .unwrap();
        assert_eq!(result.first_nonzero_power, 1);
        assert_eq!(result.coefficients.len(), 2);
        for difference in [
            result.coefficients[0]
                .subtract(&a.multiply(&a).unwrap())
                .unwrap(),
            result.coefficients[1].add(&c(1)).unwrap(),
        ] {
            assert_eq!(
                difference.sign(&policy).unwrap(),
                Classification::Decided(RealSign::Zero)
            );
        }
        assert_eq!(policy.approximations(), 0);
    }
}

struct CancellingScalars {
    inner: Scalars,
    token: hypersolve::work_budget::CancellationToken,
    calls: usize,
    at: usize,
}
impl CancellingScalars {
    fn tick(&mut self) {
        self.calls += 1;
        if self.calls == self.at {
            self.token.cancel();
        }
    }
}
impl OrderedFieldPolynomialContext<Real> for CancellingScalars {
    type Error = &'static str;
    fn constant(&mut self, v: &Real) -> Result<Real, Self::Error> {
        self.tick();
        self.inner.constant(v)
    }
    fn add(&mut self, a: &Real, b: &Real) -> Result<Real, Self::Error> {
        self.tick();
        self.inner.add(a, b)
    }
    fn multiply(&mut self, a: &Real, b: &Real) -> Result<Real, Self::Error> {
        self.tick();
        self.inner.multiply(a, b)
    }
    fn scale(&mut self, a: &Real, b: &Real) -> Result<Real, Self::Error> {
        self.tick();
        self.inner.scale(a, b)
    }
    fn sign(&mut self, a: &Real) -> Result<Ordering, Self::Error> {
        self.tick();
        self.inner.sign(a)
    }
    fn sign_if_separated(&mut self, a: &Real) -> Result<Option<Ordering>, Self::Error> {
        self.tick();
        self.inner.sign_if_separated(a)
    }
    fn normalize_positive_scale(&mut self, _: &mut [Real]) {
        panic!("retain actual coefficients")
    }
}

#[test]
fn perturbation_does_not_publish_after_cancellation_in_the_final_coefficient_scan() {
    use hypersolve::work_budget::{
        CancellationToken, WorkBudget, WorkKind, with_cancellable_budget,
    };
    let f = p(&[&[0, 0, -2, 1], &[2, -1]]);
    let g = p(&[&[-2, 1]]);
    let complete = perturbation(&f, &g, Config::default(), &mut Scalars { fail: false })
        .unwrap()
        .unwrap();
    for at in [10, complete.coefficient_operations] {
        let token = CancellationToken::new();
        let mut field = CancellingScalars {
            inner: Scalars { fail: false },
            token: token.clone(),
            calls: 0,
            at,
        };
        let mut declined = false;
        let outcome = with_cancellable_budget(WorkBudget::UNLIMITED, &token, || {
            let result = perturbation(&f, &g, Config::default(), &mut field);
            declined = matches!(result, Err(Error::BudgetExhausted));
            result
        });
        assert_eq!(outcome.result.unwrap_err().kind, WorkKind::Cancellation);
        assert!(
            declined,
            "the inner API must decline before publishing a completed polynomial"
        );
    }
    let retry = perturbation(&f, &g, Config::default(), &mut Scalars { fail: false })
        .unwrap()
        .unwrap();
    assert_eq!(retry.coefficients, complete.coefficients);
}
impl OrderedFieldPolynomialContext<Real> for Scalars {
    type Error = &'static str;
    fn constant(&mut self, value: &Real) -> Result<Real, Self::Error> {
        Ok(value.clone())
    }
    fn add(&mut self, a: &Real, b: &Real) -> Result<Real, Self::Error> {
        Ok(a + b)
    }
    fn multiply(&mut self, a: &Real, b: &Real) -> Result<Real, Self::Error> {
        Ok(a * b)
    }
    fn scale(&mut self, value: &Real, scale: &Real) -> Result<Real, Self::Error> {
        Ok(value * scale)
    }
    fn sign(&mut self, value: &Real) -> Result<Ordering, Self::Error> {
        if self.fail {
            Err("coefficient authority")
        } else {
            value.partial_cmp(&Real::zero()).ok_or("undecided")
        }
    }
    fn sign_if_separated(&mut self, value: &Real) -> Result<Option<Ordering>, Self::Error> {
        self.sign(value).map(Some)
    }
    fn normalize_positive_scale(&mut self, _: &mut [Real]) {
        panic!("a determinant must retain actual coefficient values")
    }
}
fn p(rows: &[&[i32]]) -> Vec<Vec<Real>> {
    rows.iter()
        .map(|r| r.iter().copied().map(Real::from).collect())
        .collect()
}

#[test]
fn exact_resultant_values_degeneracies_and_orientation() {
    let mut field = Scalars { fail: false };
    // Res_y(y³-x,y²-1)=(1-x)(-1-x)=x²-1.
    let f = p(&[&[0, 0, 0, 1], &[-1]]);
    let g = p(&[&[-1, 0, 1]]);
    let result = resultant(&f, &g, Config::default(), &mut field).unwrap();
    assert_eq!(result.coefficients, p(&[&[-1, 0, 1]])[0]);
    // Leading coefficient factors belong to the determinant too:
    // Res_y(2y-x,3y-1)=3x-2, without projective normalization.
    assert_eq!(
        resultant(
            &p(&[&[0, 2], &[-1]]),
            &p(&[&[-1, 3]]),
            Config::default(),
            &mut field,
        )
        .unwrap()
        .coefficients,
        vec![Real::from(-2), Real::from(3)]
    );
    // Reversing two odd-degree operands negates the actual determinant.
    let f = p(&[&[0, 1], &[-1]]);
    let g = p(&[&[-2, 1]]);
    assert_eq!(
        resultant(&f, &g, Config::default(), &mut field)
            .unwrap()
            .coefficients,
        vec![Real::from(-2), Real::one()]
    );
    assert_eq!(
        resultant(&g, &f, Config::default(), &mut field)
            .unwrap()
            .coefficients,
        vec![Real::from(2), Real::from(-1)]
    );
    let common = resultant(&f, &f, Config::default(), &mut field).unwrap();
    assert_eq!(common.coefficients, vec![Real::zero()]);
    assert_eq!(
        resultant(&[], &g, Config::default(), &mut field)
            .unwrap()
            .coefficients,
        vec![Real::zero()]
    );
    assert_eq!(
        resultant(&p(&[&[3]]), &p(&[&[2]]), Config::default(), &mut field)
            .unwrap()
            .coefficients,
        vec![Real::one()]
    );
    // Degree thirteen is an ordinary input, not a fixed low-degree family.
    let mut first = vec![vec![Real::zero(); 14], vec![Real::from(-1)]];
    first[0][13] = Real::one();
    assert_eq!(
        resultant(&first, &g, Config::default(), &mut field)
            .unwrap()
            .coefficients,
        vec![Real::from(-8192), Real::one()]
    );
    // A constant in y can still be a polynomial in retained x.
    assert_eq!(
        resultant(
            &p(&[&[0], &[1]]),
            &p(&[&[1, 0, 1]]),
            Config::default(),
            &mut field
        )
        .unwrap()
        .coefficients,
        vec![Real::zero(), Real::zero(), Real::one()]
    );
}

#[test]
fn resource_and_coefficient_failures_never_become_partial_polynomials() {
    let f = p(&[&[0, 1], &[-1]]);
    let g = p(&[&[-2, 1]]);
    let mut field = Scalars { fail: false };
    assert!(matches!(
        resultant(
            &f,
            &g,
            Config {
                max_coefficient_operations: 0,
                ..Config::default()
            },
            &mut field
        ),
        Err(Error::OperationLimit { limit: 0 })
    ));
    assert!(matches!(
        resultant(
            &f,
            &g,
            Config {
                max_matrix_entries: 0,
                ..Config::default()
            },
            &mut field
        ),
        Err(Error::MatrixLimit {
            required: 4,
            limit: 0
        })
    ));
    field.fail = true;
    assert!(matches!(
        resultant(&f, &g, Config::default(), &mut field),
        Err(Error::Coefficient("coefficient authority"))
    ));
    field.fail = false;
    let result = resultant(&f, &g, Config::default(), &mut field).unwrap();
    assert_eq!(result.coefficients, vec![Real::from(-2), Real::one()]);
    assert!(result.coefficient_operations > 0);
    let exact_budget = Config {
        max_coefficient_operations: result.coefficient_operations,
        ..Config::default()
    };
    assert!(resultant(&f, &g, exact_budget, &mut field).is_ok());
    assert!(matches!(
        resultant(
            &f,
            &g,
            Config {
                max_coefficient_operations: result.coefficient_operations - 1,
                ..Config::default()
            },
            &mut field
        ),
        Err(Error::OperationLimit { .. })
    ));
}

#[test]
fn selected_quintic_coefficients_remain_in_their_original_field() {
    let base = SelectedPolynomialField::from_recursive(
        RecursiveQuadraticField::base(
            Vec::new(),
            constant_tensor(0, real(1)),
            constant_tensor(0, real(1)),
        )
        .unwrap(),
    );
    let policy = TestContext::strict();
    let values = [-1, 1, 0, 0, 0, 1]
        .map(|n| base.constant(n.into()).unwrap())
        .to_vec();
    let Classification::Decided(field) = base.select(values, 0.into(), 1.into(), &policy).unwrap()
    else {
        panic!("one selected quintic root")
    };
    let a = field.generator().unwrap();
    let c = |n| field.constant(Real::from(n)).unwrap();
    let f = vec![vec![c(0), c(1)], vec![c(0)], vec![c(-1)]];
    let g = vec![vec![a.scale(&Real::from(-1)).unwrap(), c(1)]];
    let mut context = SelectedPolynomialContext {
        field: field.clone(),
        policy: &policy,
    };
    let result = resultant(&f, &g, Config::default(), &mut context).unwrap();
    assert_eq!(result.coefficients.len(), 3);
    let zero = |value: &hypersolve::selected_polynomial_field::SelectedPolynomialValue| {
        assert_eq!(
            value.sign(&policy).unwrap(),
            Classification::Decided(RealSign::Zero)
        )
    };
    zero(&result.coefficients[0].add(&a).unwrap());
    zero(&result.coefficients[1]);
    zero(&result.coefficients[2].subtract(&c(1)).unwrap());
}

#[test]
fn power_resultants_preserve_all_factors_and_their_multiplicities() {
    let mut field = Scalars { fail: false };
    for m in 1_usize..=5 {
        for n in 1_usize..=5 {
            // Res_y(y^m-x,y^n-1)=(-1)^m(1-x^(n/gcd(m,n)))^gcd(m,n).
            let (mut a, mut b) = (m, n);
            while b != 0 {
                (a, b) = (b, a % b);
            }
            let common = a;
            let stride = n / common;
            let mut f = vec![vec![Real::zero(); m + 1], vec![Real::from(-1)]];
            f[0][m] = Real::one();
            let mut g = vec![vec![Real::zero(); n + 1]];
            g[0][0] = Real::from(-1);
            g[0][n] = Real::one();
            let mut expected = vec![Real::zero(); n + 1];
            let mut choose = 1_i32;
            for j in 0..=common {
                expected[j * stride] = Real::from(if (m + j).is_multiple_of(2) {
                    choose
                } else {
                    -choose
                });
                if j < common {
                    choose =
                        choose * i32::try_from(common - j).unwrap() / i32::try_from(j + 1).unwrap();
                }
            }
            assert_eq!(
                resultant(&f, &g, Config::default(), &mut field)
                    .unwrap()
                    .coefficients,
                expected,
                "degrees {m}, {n}"
            );
        }
    }
}
