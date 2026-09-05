use hyperreal::Real;
use hypersolve::{
    Constraint, Expr, Problem, QuadraticResidual, SymbolId, UnivariateQuadraticResidual,
};
use num::{BigInt, BigRational, One, Zero};
use proptest::prelude::*;
use std::collections::{BTreeMap, HashMap};

fn symbol(index: u32) -> Expr {
    Expr::symbol(SymbolId(index), format!("x{index}"))
}

fn problem(count: u32) -> Problem {
    let mut problem = Problem::default();
    for index in 0..count {
        problem.add_variable(format!("x{index}"), Real::zero());
    }
    problem
}

#[test]
fn distinct_high_degrees_and_monomials_cannot_cancel() {
    let x = symbol(0);
    let y = symbol(1);
    let mut problem = problem(2);
    let expressions = [
        x.clone().powi(2) * (x.clone() - x.clone().powi(2)),
        x.clone().powi(2) * (x.clone() - y.clone()),
        x.clone().powi(2) + x.clone().powi(2) * (x.clone() - y),
    ];
    for expression in expressions {
        assert!(UnivariateQuadraticResidual::from_expr(&expression, &problem).is_none());
        assert!(QuadraticResidual::from_expr(&expression, &problem).is_none());
        problem.add_constraint(Constraint::equality("nonquadratic", expression));
    }
    let analysis = problem.analyze();
    assert!(analysis.quadratic_residuals().iter().all(Option::is_none));
    assert!(
        analysis
            .univariate_quadratic_residuals()
            .iter()
            .all(Option::is_none)
    );
}

#[test]
fn genuine_higher_degree_cancellation_preserves_the_quadratic() {
    let p = problem(2);
    let x = symbol(0);
    let y = symbol(1);
    let cubic = x.clone().powi(2) * x.clone();
    let fourth = cubic.clone() * x.clone();
    let expressions = [
        cubic.clone() * Expr::int(2) - cubic.clone() - cubic.clone() + x.clone(),
        (cubic.clone() + x.clone()) * (x.clone() - Expr::int(1)) - fourth + cubic.clone(),
        cubic.clone() * Expr::int(0) + x.clone(),
        cubic.clone() / Expr::int(2) + cubic.clone() / Expr::int(2) - cubic + x.clone(),
    ];
    let bindings = HashMap::from([(SymbolId(0), Real::from(3)), (SymbolId(1), Real::from(2))]);
    for expression in expressions {
        let exact = expression.eval_real(&bindings).unwrap();
        let uni = UnivariateQuadraticResidual::from_expr(&expression, &p).unwrap();
        let multi = QuadraticResidual::from_expr(&expression, &p).unwrap();
        assert_eq!(uni.eval_real(&p.variables, &bindings).unwrap(), exact);
        assert_eq!(multi.eval_real(&p.variables, &bindings).unwrap(), exact);
    }
    let xyx = x.clone() * y.clone() * x.clone();
    let expression = xyx - y * x.clone().powi(2) + x;
    let multi = QuadraticResidual::from_expr(&expression, &p).unwrap();
    assert_eq!(
        multi.eval_real(&p.variables, &bindings).unwrap(),
        Real::from(3)
    );
}

#[test]
fn cancellation_does_not_erase_unbound_symbols_or_domain_obligations() {
    let p = problem(1);
    let x = symbol(0);
    let y = symbol(1);
    for expression in [
        y.clone() - y.clone(),
        y.clone().powi(2) * y.clone() * Expr::int(0),
        x.clone().powi(0),
        x.clone().powi(0) * Expr::int(0),
        x.clone() / Expr::int(0),
        (x.clone() / x.clone()) * Expr::int(0),
    ] {
        assert!(QuadraticResidual::from_expr(&expression, &p).is_none());
        assert!(UnivariateQuadraticResidual::from_expr(&expression, &p).is_none());
    }
}

#[test]
fn bounded_cancellation_exhaustion_is_unknown_not_zero() {
    let p = problem(1);
    let x = symbol(0);
    let mut power = x.clone();
    for _ in 0..6 {
        power = power.powi(2);
    }
    let degree64 = power.clone() - power.clone() + x.clone();
    assert!(UnivariateQuadraticResidual::from_expr(&degree64, &p).is_some());
    assert!(QuadraticResidual::from_expr(&degree64, &p).is_some());
    let degree128 = power.clone().powi(2) - power.powi(2) + x;
    assert!(UnivariateQuadraticResidual::from_expr(&degree128, &p).is_none());
    assert!(QuadraticResidual::from_expr(&degree128, &p).is_none());

    let p = problem(17);
    let sum = (1..17).fold(symbol(0), |sum, i| sum + symbol(i));
    let square = sum.clone().powi(2);
    assert!(QuadraticResidual::from_expr(&square, &p).is_some());
    let cube = square * sum;
    assert!(QuadraticResidual::from_expr(&(cube.clone() - cube), &p).is_none());
}

#[test]
fn scalar_zero_filter_preserves_named_and_opaque_coefficient_families() {
    let p = problem(1);
    let x = symbol(0);
    let pi = Real::pi();
    let e = Real::e();
    let pi2 = &pi * &pi;
    let sqrt2 = Real::from(2).sqrt().unwrap();
    let ln2 = Real::from(2).ln().unwrap();
    let ln3 = Real::from(3).ln().unwrap();
    let seventh = Real::from(hyperreal::Rational::fraction(1, 7).unwrap());
    let fifth = Real::from(hyperreal::Rational::fraction(1, 5).unwrap());
    let values = [
        seventh.clone(),
        pi.clone(),
        pi2.clone(),
        pi.clone().inverse().unwrap(),
        &pi * &e,
        (&e / &pi).unwrap(),
        &pi * &sqrt2,
        &pi2 * &e,
        &pi - Real::from(3),
        &(&pi2 * &e) * &sqrt2,
        sqrt2,
        Real::from(2).exp().unwrap(),
        ln3.clone(),
        (Real::from(2) * &e).ln().unwrap(),
        &ln2 * &ln3,
        Real::from(2).log10().unwrap(),
        Real::from(3).log2().unwrap(),
        seventh.clone().exp10().unwrap(),
        seventh.exp2().unwrap(),
        fifth.clone().sin_pi(),
        fifth.tan_pi().unwrap(),
        Real::one().sin(),
    ];
    assert_eq!(values.len(), 22);
    for value in values {
        let coefficient = Expr::real(value.clone());
        let expression =
            (x.clone() * coefficient.clone() + coefficient.clone()) * (x.clone() - Expr::int(1));
        let uni = UnivariateQuadraticResidual::from_expr(&expression, &p).unwrap();
        let multi = QuadraticResidual::from_expr(&expression, &p).unwrap();
        assert_eq!(uni.quadratic(), &value);
        assert_eq!(uni.linear(), &Real::zero());
        assert_eq!(uni.constant(), &-value.clone());
        assert_eq!(multi.quadratic_terms()[0].coefficient, value);
        assert!(multi.linear_terms().is_empty());
        let cubic = x.clone().powi(2) * x.clone() * coefficient;
        let cancelled = cubic.clone() * Expr::int(2) - cubic.clone() - cubic + x.clone();
        let uni = UnivariateQuadraticResidual::from_expr(&cancelled, &p).unwrap();
        let multi = QuadraticResidual::from_expr(&cancelled, &p).unwrap();
        assert_eq!(uni.linear(), &Real::one());
        assert_eq!(uni.quadratic(), &Real::zero());
        assert_eq!(multi.linear_terms()[0].coefficient, Real::one());
        assert!(multi.quadratic_terms().is_empty());
    }
}

#[derive(Clone, Debug)]
enum Node {
    Constant(i16),
    Variable(u32),
    Add(Box<Self>, Box<Self>),
    Sub(Box<Self>, Box<Self>),
    Mul(Box<Self>, Box<Self>),
    Square(Box<Self>),
    Scale(Box<Self>, i16),
    Divide(Box<Self>, i16),
}

type Oracle = BTreeMap<[u16; 3], BigRational>;

fn add_oracle(mut first: Oracle, second: Oracle) -> Oracle {
    for (powers, coefficient) in second {
        *first.entry(powers).or_insert_with(BigRational::zero) += coefficient;
    }
    first.retain(|_, coefficient| !coefficient.is_zero());
    first
}

fn multiply_oracle(first: Oracle, second: Oracle) -> Oracle {
    let mut result = Oracle::new();
    for (a, ca) in first {
        for (b, cb) in &second {
            let powers = std::array::from_fn(|axis| a[axis] + b[axis]);
            *result.entry(powers).or_insert_with(BigRational::zero) += &ca * cb;
        }
    }
    result.retain(|_, coefficient| !coefficient.is_zero());
    result
}

fn scale_oracle(mut polynomial: Oracle, scale: BigRational) -> Oracle {
    polynomial.values_mut().for_each(|c| *c *= &scale);
    polynomial.retain(|_, c| !c.is_zero());
    polynomial
}

impl Node {
    fn expression(&self) -> Expr {
        match self {
            Self::Constant(n) => Expr::int(i64::from(*n)),
            Self::Variable(i) => symbol(*i),
            Self::Add(a, b) => a.expression() + b.expression(),
            Self::Sub(a, b) => a.expression() - b.expression(),
            Self::Mul(a, b) => a.expression() * b.expression(),
            Self::Square(a) => a.expression().powi(2),
            Self::Scale(a, n) => a.expression() * Expr::int(i64::from(*n)),
            Self::Divide(a, n) => a.expression() / Expr::int(i64::from(*n)),
        }
    }

    fn oracle(&self) -> Oracle {
        match self {
            Self::Constant(n) => scale_oracle(
                BTreeMap::from([([0; 3], BigRational::one())]),
                BigRational::from_integer(BigInt::from(*n)),
            ),
            Self::Variable(i) => {
                let mut powers = [0; 3];
                powers[*i as usize] = 1;
                BTreeMap::from([(powers, BigRational::one())])
            }
            Self::Add(a, b) => add_oracle(a.oracle(), b.oracle()),
            Self::Sub(a, b) => {
                add_oracle(a.oracle(), scale_oracle(b.oracle(), -BigRational::one()))
            }
            Self::Mul(a, b) => multiply_oracle(a.oracle(), b.oracle()),
            Self::Square(a) => multiply_oracle(a.oracle(), a.oracle()),
            Self::Scale(a, n) => {
                scale_oracle(a.oracle(), BigRational::from_integer(BigInt::from(*n)))
            }
            Self::Divide(a, n) => scale_oracle(
                a.oracle(),
                BigRational::new(BigInt::one(), BigInt::from(*n)),
            ),
        }
    }
}

fn nodes(variables: u32) -> BoxedStrategy<Node> {
    prop_oneof![
        (-3_i16..=3).prop_map(Node::Constant),
        (0..variables).prop_map(Node::Variable)
    ]
    .prop_recursive(3, 64, 3, |inner| {
        prop_oneof![
            (inner.clone(), inner.clone()).prop_map(|(a, b)| Node::Add(Box::new(a), Box::new(b))),
            (inner.clone(), inner.clone()).prop_map(|(a, b)| Node::Sub(Box::new(a), Box::new(b))),
            (inner.clone(), inner.clone()).prop_map(|(a, b)| Node::Mul(Box::new(a), Box::new(b))),
            inner.clone().prop_map(|a| Node::Square(Box::new(a))),
            (inner.clone(), -3_i16..=3).prop_map(|(a, n)| Node::Scale(Box::new(a), n)),
            (inner, prop_oneof![-3_i16..=-1, 1_i16..=3])
                .prop_map(|(a, n)| Node::Divide(Box::new(a), n)),
        ]
    })
    .boxed()
}

fn rational(value: &Real) -> BigRational {
    let value = value.exact_rational_ref().unwrap();
    let mut numerator = BigInt::from(value.numerator().clone());
    if value.is_negative() {
        numerator = -numerator;
    }
    BigRational::new(numerator, BigInt::from(value.denominator().clone()))
}

fn actual_multi(poly: &QuadraticResidual) -> Oracle {
    let mut result = BTreeMap::from([([0; 3], rational(poly.constant()))]);
    for term in poly.linear_terms() {
        let mut powers = [0; 3];
        powers[term.symbol.0 as usize] = 1;
        result.insert(powers, rational(&term.coefficient));
    }
    for term in poly.quadratic_terms() {
        let mut powers = [0; 3];
        powers[term.first.0 as usize] += 1;
        powers[term.second.0 as usize] += 1;
        result.insert(powers, rational(&term.coefficient));
    }
    result.retain(|_, c| !c.is_zero());
    result
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(2048))]
    #[test]
    fn multivariate_extraction_matches_independent_rational_coefficients(node in nodes(3)) {
        let oracle = node.oracle();
        let quadratic = oracle.keys().all(|powers| powers.iter().sum::<u16>() <= 2);
        let actual = QuadraticResidual::from_expr(&node.expression(), &problem(3));
        prop_assert_eq!(actual.is_some(), quadratic);
        if let Some(actual) = actual {
            prop_assert_eq!(actual_multi(&actual), oracle);
        }
    }

    #[test]
    fn univariate_extraction_matches_independent_rational_coefficients(node in nodes(1)) {
        let expression = node.expression();
        let oracle = node.oracle();
        let quadratic = oracle.keys().all(|powers| powers[0] <= 2);
        let has_variable = !expression.structural_facts().dependencies.is_empty();
        let actual = UnivariateQuadraticResidual::from_expr(&expression, &problem(1));
        prop_assert_eq!(actual.is_some(), quadratic && has_variable);
        if let Some(actual) = actual {
            for (degree, coefficient) in [actual.constant(), actual.linear(), actual.quadratic()].iter().enumerate() {
                let expected = oracle.get(&[degree as u16,0,0]).cloned().unwrap_or_default();
                prop_assert_eq!(rational(coefficient), expected);
            }
        }
    }
}
