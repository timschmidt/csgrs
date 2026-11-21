//! Exact point order without expanding an irrelevant exponent gap.
use crate::{DyadicBudget, IntervalError};
use core::cmp::Ordering;
use num_bigint::BigInt;
use num_traits::Zero;

pub(crate) fn compare(
    a: &BigInt,
    e: i32,
    b: &BigInt,
    f: i32,
    budget: &mut DyadicBudget,
) -> Result<Ordering, IntervalError> {
    budget.reserve(a.bits().max(b.bits()), 1, false)?;
    let sa = a.cmp(&BigInt::zero());
    let sb = b.cmp(&BigInt::zero());
    if sa != sb {
        return Ok(sa.cmp(&sb));
    }
    if sa == Ordering::Equal {
        return Ok(Ordering::Equal);
    }
    let size = (i128::from(a.bits()) + i128::from(e)).cmp(&(i128::from(b.bits()) + i128::from(f)));
    if size != Ordering::Equal {
        return Ok(if sa == Ordering::Less {
            size.reverse()
        } else {
            size
        });
    }
    // Equal top exponents make the aligned width no larger than the larger
    // existing mantissa. Charge before shifting, including on 32-bit hosts.
    budget.reserve(a.bits().max(b.bits()), 1, false)?;
    if e >= f {
        let shift =
            usize::try_from(i64::from(e) - i64::from(f)).map_err(|_| IntervalError::Overflow)?;
        Ok((a << shift).cmp(b))
    } else {
        let shift =
            usize::try_from(i64::from(f) - i64::from(e)).map_err(|_| IntervalError::Overflow)?;
        Ok(a.cmp(&(b << shift)))
    }
}
