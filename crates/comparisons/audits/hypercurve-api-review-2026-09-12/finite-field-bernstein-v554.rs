fn exact_nonrational_bernstein_interval_roots(
    polynomial: &BezierParameterPolynomial,
    lower: &Real,
    upper: &Real,
    policy: &CurveContext,
    trace: &mut BezierRootIsolationTrace2,
) -> CurveResult<Option<Vec<BezierParameter2>>> {
    use hypersolve::{
        OrderedFieldPolynomialContext, OrderedFieldRootIsolationConfig,
        OrderedFieldRootIsolationStatus, isolate_ordered_field_polynomial_roots,
    };
    if polynomial.degree() < 2
        || polynomial
            .coefficients()
            .iter()
            .all(|x| x.exact_rational_ref().is_some())
    {
        return Ok(None);
    }
    struct ScalarField<'a>(&'a CurveContext);
    impl OrderedFieldPolynomialContext<Real> for ScalarField<'_> {
        type Error = ();
        fn constant(&mut self, value: &Real) -> Result<Real, ()> {
            Ok(value.clone())
        }
        fn add(&mut self, left: &Real, right: &Real) -> Result<Real, ()> {
            Ok(left + right)
        }
        fn multiply(&mut self, left: &Real, right: &Real) -> Result<Real, ()> {
            Ok(left * right)
        }
        fn scale(&mut self, value: &Real, scale: &Real) -> Result<Real, ()> {
            Ok(value * scale)
        }
        fn normalize_positive_scale(&mut self, _: &mut [Real]) {}
        fn sign(&mut self, value: &Real) -> Result<Ordering, ()> {
            self.sign_if_separated(value)?.ok_or(())
        }
        fn sign_if_separated(&mut self, value: &Real) -> Result<Option<Ordering>, ()> {
            Ok(self
                .0
                .strict_predicate_pass(|| real_sign(value, self.0))
                .map(|s| match s {
                    RealSign::Negative => Ordering::Less,
                    RealSign::Zero => Ordering::Equal,
                    RealSign::Positive => Ordering::Greater,
                }))
        }
    }
    let report = match isolate_ordered_field_polynomial_roots(
        polynomial.coefficients().to_vec(),
        lower,
        upper,
        OrderedFieldRootIsolationConfig {
            max_subdivision_depth: 64,
            refinement_steps: 0,
        },
        &mut ScalarField(policy),
    ) {
        Ok(report) => report,
        Err(()) => {
            eprintln!(
                "V554 finite field isolation sign unavailable degree={}",
                polynomial.degree()
            );
            return Ok(None);
        }
    };
    eprintln!(
        "V554 finite field isolation degree={} status={:?} subdivisions={} roots={}",
        polynomial.degree(),
        report.status,
        report.subdivision_steps,
        report.intervals.len()
    );
    if report.status != OrderedFieldRootIsolationStatus::Isolated {
        return Ok(None);
    }
    trace.bisections += report.subdivision_steps;
    let mut roots = Vec::with_capacity(report.intervals.len());
    for interval in report.intervals {
        if let Some(root) = interval.exact_root {
            roots.push(BezierParameter2::Exact(root));
        } else {
            let interval =
                match BezierParameterInterval::try_new(interval.lower, interval.upper, policy)? {
                    Classification::Decided(interval) => interval,
                    Classification::Uncertain(_) => return Ok(None),
                };
            roots.push(BezierParameter2::Algebraic(
                BezierAlgebraicParameter2::from_certified_simple_singleton(
                    polynomial.clone(),
                    interval,
                ),
            ));
        }
    }
    Ok(Some(roots))
}
