type Precision = i32;
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(verus_keep_ghost, verus_verify)]
pub(crate) enum BoundInfo {
    // Unknown means the expression may still be zero or either sign; callers
    // must not use it to short-circuit exact predicates.
    Unknown,
    // Exact structural zero, usually from rational leaves or annihilating
    // products.
    Zero,
    // NonZero may have an unknown sign or an upper bound on its MSD. An exact
    // MSD also supplies a lower magnitude bound; an inexact MSD does not.
    NonZero {
        sign: Option<Sign>,
        msd: Option<Precision>,
        exact_msd: bool,
    },
}

impl BoundInfo {
    #[cfg_attr(verus_keep_ghost, allow(unused, verus_impl_method_marker))]
    #[cfg_attr(verus_keep_ghost, verus_spec(result =>
        ensures match sign {
            Sign::NoSign => result == Self::Zero,
            _ => result == Self::NonZero { sign: Some(sign), msd, exact_msd },
        },
    ))]
    fn with_sign_msd(sign: Sign, msd: Option<Precision>, exact_msd: bool) -> Self {
        match sign {
            Sign::NoSign => Self::Zero,
            _ => Self::NonZero {
                sign: Some(sign),
                msd,
                exact_msd,
            },
        }
    }
}
