fn observation(bound: BoundInfo) -> crate::Observation {
    match bound {
        BoundInfo::Unknown => crate::Observation::Unknown,
        BoundInfo::Zero => crate::Observation::Zero,
        BoundInfo::NonZero { sign, msd, exact_msd } => crate::Observation::NonZero {
            sign: sign.map(|sign| match sign {
                Sign::Minus => -1,
                Sign::NoSign => 0,
                Sign::Plus => 1,
            }),
            msd,
            exact: exact_msd,
        },
    }
}
