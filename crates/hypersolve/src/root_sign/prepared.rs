//! Operation-owned Sturm sequence for repeated counts on one polynomial.
use super::*;

pub(crate) struct PreparedSturmChain<C> {
    polynomials: Vec<Vec<C>>,
}

impl<C: Clone> PreparedSturmChain<C> {
    pub(crate) fn new<F: OrderedFieldPolynomialContext<C>>(
        defining: &[C],
        field: &mut F,
    ) -> Result<Option<Self>, F::Error> {
        let mut first = defining.to_vec();
        field.normalize_positive_scale(&mut first);
        trim_polynomial(&mut first, field)?;
        let Some(leading) = first.last() else {
            return Ok(None);
        };
        if first.len() == 1 {
            return Ok((field.sign(leading)? != Ordering::Equal).then_some(Self {
                polynomials: vec![first],
            }));
        }
        let second = first
            .iter()
            .enumerate()
            .skip(1)
            .map(|(power, coefficient)| field.scale(coefficient, &Real::from(power as u128)))
            .collect::<Result<Vec<_>, _>>()?;
        let budget = RemainderBudget::new(field.remainder_elimination_budget());
        let mut polynomials = vec![first.clone()];
        Ok(
            visit_signed_remainders(first, second, budget, field, |polynomial, _| {
                polynomials.push(polynomial.to_vec());
                Ok(())
            })?
            .map(|()| Self { polynomials }),
        )
    }

    pub(crate) fn count<F: OrderedFieldPolynomialContext<C>>(
        &self,
        lower: &Real,
        upper: &Real,
        field: &mut F,
    ) -> Result<Option<usize>, F::Error> {
        if sign(&(upper - lower)) != Some(Ordering::Greater) {
            return Ok(None);
        }
        let first = &self.polynomials[0];
        if first.len() == 1 {
            return Ok(Some(0));
        }
        let mut previous = [
            field_polynomial_sign(first, lower, field)?,
            field_polynomial_sign(first, upper, field)?,
        ];
        if previous.contains(&Ordering::Equal) {
            return Ok(None);
        }
        let mut variations = [0_usize; 2];
        for polynomial in &self.polynomials[1..] {
            for (endpoint, point) in [lower, upper].into_iter().enumerate() {
                let current = field_polynomial_sign(polynomial, point, field)?;
                if current != Ordering::Equal {
                    variations[endpoint] += usize::from(previous[endpoint] != current);
                    previous[endpoint] = current;
                }
            }
        }
        Ok(variations[0].checked_sub(variations[1]))
    }
}
