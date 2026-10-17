//! Exact period reduction of rectangle evidence, without changing source charts.
use crate::topology::*;
use std::cmp::Ordering;

/// Splits each positive rectangle at the canonical U seam. Each input must
/// span at most one turn; reducing a multiple cover would erase multiplicity.
/// At most two output rectangles represent each input. Their interiors are
/// disjoint within that input. Different inputs remain distinct: the caller
/// must still reject overlaps and prove the required complete occupancy.
pub(super) fn rebase(rectangles: &[[Real; 4]]) -> Result<Option<Vec<[Real; 4]>>, BuildError> {
    let order =
        |a: &Real, b: &Real| decided_model_order(compare_reals(a, b, crate::STRICT_PREDICATES));
    let tau = Real::tau();
    let capacity = rectangles
        .len()
        .checked_mul(2)
        .ok_or(GeometryError::Numeric(hyperreal::Problem::Exhausted))?;
    let mut result = Vec::new();
    result
        .try_reserve_exact(capacity)
        .map_err(|_| GeometryError::Numeric(hyperreal::Problem::Exhausted))?;
    for [u0, u1, v0, v1] in rectangles {
        let width = u1 - u0;
        if order(&width, &Real::zero())? != Ordering::Greater
            || order(&width, &tau)? == Ordering::Greater
            || order(v0, v1)? != Ordering::Less
        {
            return Ok(None);
        }
        let turns = (u0 / &tau)
            .map_err(GeometryError::Numeric)?
            .floor_certified()
            .map_err(GeometryError::Numeric)?;
        let first = u0 - Real::new(hyperreal::Rational::from_bigint(turns)) * &tau;
        let last = &first + width;
        if order(&last, &tau)? == Ordering::Greater {
            result.push([first, tau.clone(), v0.clone(), v1.clone()]);
            result.push([Real::zero(), last - &tau, v0.clone(), v1.clone()]);
        } else {
            result.push([first, last, v0.clone(), v1.clone()]);
        }
    }
    Ok(Some(result))
}

#[cfg(test)]
mod tests {
    use super::*;
    fn equal(a: &Real, b: &Real) {
        assert!(real_values_equal(a, b).unwrap());
    }
    #[test]
    fn rebasing_preserves_width_height_and_multiplicity_across_arbitrary_turns() {
        let quarter = (Real::pi() / Real::from(2)).unwrap();
        for turns in [-1_000_000, -2, 0, 3, 1_000_000] {
            let start = Real::from(turns) * Real::tau() - &quarter;
            let input = [
                start.clone(),
                &start + Real::pi(),
                Real::from(3),
                Real::from(7),
            ];
            let result = rebase(&[input.clone(), input]).unwrap().unwrap();
            assert_eq!(result.len(), 4, "coincident copies remain distinct");
            for pair in result.chunks(2) {
                equal(&pair[0][0], &(Real::from(3) * &quarter));
                equal(&pair[0][1], &Real::tau());
                equal(&pair[1][0], &Real::zero());
                equal(&pair[1][1], &quarter);
                equal(
                    &((&pair[0][1] - &pair[0][0]) + (&pair[1][1] - &pair[1][0])),
                    &Real::pi(),
                );
                for rectangle in pair {
                    equal(&rectangle[2], &Real::from(3));
                    equal(&rectangle[3], &Real::from(7));
                }
            }
        }
    }
    #[test]
    fn gaps_are_preserved_and_more_than_one_turn_cannot_be_collapsed() {
        let tiny = (Real::one() / Real::from(1_000_000_000_000i64)).unwrap();
        let start = Real::tau() * Real::from(3);
        let rectangle = |width| [start.clone(), &start + width, Real::zero(), Real::one()];
        let short = rebase(&[rectangle(Real::tau() - &tiny)]).unwrap().unwrap();
        assert_eq!(short.len(), 1);
        equal(&(&short[0][1] - &short[0][0]), &(Real::tau() - &tiny));
        for width in [
            Real::zero(),
            -&tiny,
            Real::tau() + &tiny,
            Real::tau() * Real::from(2),
        ] {
            assert!(rebase(&[rectangle(width)]).unwrap().is_none());
        }
        let full = rebase(&[rectangle(Real::tau())]).unwrap().unwrap();
        assert_eq!(full.len(), 1);
        equal(&full[0][0], &Real::zero());
        equal(&full[0][1], &Real::tau());
    }
}
