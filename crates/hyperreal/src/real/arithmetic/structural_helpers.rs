fn real_sign_from_num(sign: Sign) -> RealSign {
    match sign {
        Sign::Minus => RealSign::Negative,
        Sign::NoSign => RealSign::Zero,
        Sign::Plus => RealSign::Positive,
    }
}

fn num_sign_from_real(sign: RealSign) -> Sign {
    match sign {
        RealSign::Negative => Sign::Minus,
        RealSign::Zero => Sign::NoSign,
        RealSign::Positive => Sign::Plus,
    }
}

fn multiply_public_sign(left: Option<RealSign>, right: Option<RealSign>) -> Option<RealSign> {
    match (left?, right?) {
        (RealSign::Zero, _) | (_, RealSign::Zero) => Some(RealSign::Zero),
        (RealSign::Positive, RealSign::Positive) | (RealSign::Negative, RealSign::Negative) => {
            Some(RealSign::Positive)
        }
        (RealSign::Positive, RealSign::Negative) | (RealSign::Negative, RealSign::Positive) => {
            Some(RealSign::Negative)
        }
    }
}

fn equality_certificate_from_sign_certificate(
    certificate: RealSignCertificate,
) -> RealEqualityCertificate {
    match certificate {
        RealSignCertificate::StructuralFacts | RealSignCertificate::ExactZeroScale => {
            RealEqualityCertificate::DifferenceStructuralFacts
        }
        RealSignCertificate::BoundedRefinement { min_precision } => {
            RealEqualityCertificate::BoundedRefinement { min_precision }
        }
    }
}

fn ordering_certificate_from_sign_certificate(
    certificate: RealSignCertificate,
) -> RealOrderingCertificate {
    match certificate {
        RealSignCertificate::StructuralFacts | RealSignCertificate::ExactZeroScale => {
            RealOrderingCertificate::DifferenceStructuralFacts
        }
        RealSignCertificate::BoundedRefinement { min_precision } => {
            RealOrderingCertificate::BoundedRefinement { min_precision }
        }
    }
}

fn ordering_from_real_sign(sign: RealSign) -> Ordering {
    match sign {
        RealSign::Negative => Ordering::Less,
        RealSign::Zero => Ordering::Equal,
        RealSign::Positive => Ordering::Greater,
    }
}

fn structural_cmp_from_ordering(ordering: Ordering) -> StructuralComparison {
    match ordering {
        Ordering::Less => StructuralComparison::Less,
        Ordering::Equal => StructuralComparison::Equal,
        Ordering::Greater => StructuralComparison::Greater,
    }
}

fn domain_from_sign_nonnegative(sign: Option<RealSign>) -> DomainStatus {
    match sign {
        Some(RealSign::Positive | RealSign::Zero) => DomainStatus::Valid,
        Some(RealSign::Negative) => DomainStatus::Invalid,
        None => DomainStatus::Unknown,
    }
}

fn domain_from_sign_positive(sign: Option<RealSign>) -> DomainStatus {
    match sign {
        Some(RealSign::Positive) => DomainStatus::Valid,
        Some(RealSign::Negative | RealSign::Zero) => DomainStatus::Invalid,
        None => DomainStatus::Unknown,
    }
}

fn domain_from_zero_nonzero(zero: ZeroKnowledge) -> DomainStatus {
    match zero {
        ZeroKnowledge::NonZero => DomainStatus::Valid,
        ZeroKnowledge::Zero => DomainStatus::Invalid,
        ZeroKnowledge::Unknown => DomainStatus::Unknown,
    }
}

fn domain_abs_cmp_one(comparison: StructuralComparison, closed: bool) -> DomainStatus {
    match (comparison, closed) {
        (StructuralComparison::Less, _) => DomainStatus::Valid,
        (StructuralComparison::Equal, true) => DomainStatus::Valid,
        (StructuralComparison::Equal, false) | (StructuralComparison::Greater, _) => {
            DomainStatus::Invalid
        }
        (StructuralComparison::Unknown, _) => DomainStatus::Unknown,
    }
}

fn domain_cmp_one_ge(comparison: StructuralComparison) -> DomainStatus {
    match comparison {
        StructuralComparison::Equal | StructuralComparison::Greater => DomainStatus::Valid,
        StructuralComparison::Less => DomainStatus::Invalid,
        StructuralComparison::Unknown => DomainStatus::Unknown,
    }
}

#[inline]
fn primitive_facts_from_base(facts: &RealStructuralFacts) -> PrimitiveFacts {
    if facts.zero == ZeroKnowledge::Zero {
        return PrimitiveFacts {
            f32: PrimitiveFloatStatus::Zero,
            f64: PrimitiveFloatStatus::Zero,
        };
    }
    let Some(magnitude) = facts.magnitude else {
        return PrimitiveFacts {
            f32: PrimitiveFloatStatus::Unknown,
            f64: PrimitiveFloatStatus::Unknown,
        };
    };
    if !magnitude.exact_msd {
        return PrimitiveFacts {
            f32: PrimitiveFloatStatus::Unknown,
            f64: PrimitiveFloatStatus::Unknown,
        };
    }

    PrimitiveFacts {
        f32: primitive_float_status_from_msd(magnitude.msd, -150, -126, 127),
        f64: primitive_float_status_from_msd(magnitude.msd, -1075, -1022, 1023),
    }
}

#[inline]
fn primitive_float_status_from_msd(
    msd: i32,
    underflow_floor: i32,
    normal_floor: i32,
    overflow_ceiling: i32,
) -> PrimitiveFloatStatus {
    if msd < underflow_floor {
        PrimitiveFloatStatus::SubnormalOrUnderflows
    } else if msd > overflow_ceiling {
        PrimitiveFloatStatus::Overflows
    } else if msd < normal_floor {
        PrimitiveFloatStatus::SubnormalOrUnderflows
    } else {
        PrimitiveFloatStatus::NormalFinite
    }
}

fn structural_kind_for_class(class: &Class) -> StructuralKind {
    match class {
        One => StructuralKind::ExactRational,
        Pi | PiPow(_) | PiInv => StructuralKind::PiLike,
        Exp(_) | PiExp(_) | PiInvExp(_) => StructuralKind::ExpLike,
        Sqrt(_) | PiSqrt(_) => StructuralKind::SqrtLike,
        Ln(_) | LnAffine(_) | LnProduct(_) | Log10(_) | Log2(_) => StructuralKind::LogLike,
        SinPi(_) | TanPi(_) => StructuralKind::TrigExact,
        ConstProduct(_) | ConstOffset(_) | ConstProductSqrt(_) => StructuralKind::ProductConstant,
        Irrational => StructuralKind::ComputableOpaque,
    }
}

fn symbolic_degree_for_class(class: &Class) -> ExpressionDegree {
    match class {
        Irrational => ExpressionDegree::Unknown,
        One | Pi | PiPow(_) | PiInv | PiExp(_) | PiInvExp(_) | PiSqrt(_) | ConstProduct(_)
        | ConstOffset(_) | ConstProductSqrt(_) | Sqrt(_) | Exp(_) | Ln(_) | LnAffine(_)
        | LnProduct(_) | Log10(_) | Log2(_) | SinPi(_) | TanPi(_) => ExpressionDegree::Constant,
    }
}

fn symbolic_dependencies_for_class(class: &Class) -> SymbolicDependencyMask {
    match class {
        One => SymbolicDependencyMask::NONE,
        Pi | PiPow(_) | PiInv => SymbolicDependencyMask::PI,
        Exp(_) => SymbolicDependencyMask::EXP,
        PiExp(_) | PiInvExp(_) => SymbolicDependencyMask::PI.union(SymbolicDependencyMask::EXP),
        PiSqrt(_) => SymbolicDependencyMask::PI.union(SymbolicDependencyMask::SQRT),
        ConstProduct(product) => pi_exp_dependency_mask(product.pi_power, &product.exp_power),
        ConstOffset(offset) => pi_exp_dependency_mask(offset.pi_power, &offset.exp_power),
        ConstProductSqrt(product) => pi_exp_dependency_mask(product.pi_power, &product.exp_power)
            .union(SymbolicDependencyMask::SQRT),
        Sqrt(_) => SymbolicDependencyMask::SQRT,
        Ln(_) | LnAffine(_) | LnProduct(_) | Log10(_) | Log2(_) => SymbolicDependencyMask::LOG,
        SinPi(_) | TanPi(_) => SymbolicDependencyMask::TRIG.union(SymbolicDependencyMask::PI),
        Irrational => SymbolicDependencyMask::OPAQUE,
    }
}

fn pi_exp_dependency_mask(pi_power: i16, exp_power: &Rational) -> SymbolicDependencyMask {
    let mut mask = SymbolicDependencyMask::NONE;
    if pi_power != 0 {
        mask = mask.union(SymbolicDependencyMask::PI);
    }
    if exp_power.sign() != Sign::NoSign {
        mask = mask.union(SymbolicDependencyMask::EXP);
    }
    mask
}

fn facts_from_rational(rational: &Rational, exact_rational: bool) -> RealStructuralFacts {
    let sign = real_sign_from_num(rational.sign());
    let magnitude = rational.msd_exact().map(|msd| MagnitudeBits {
        msd,
        exact_msd: true,
    });

    RealStructuralFacts {
        sign: Some(sign),
        zero: if sign == RealSign::Zero {
            ZeroKnowledge::Zero
        } else {
            ZeroKnowledge::NonZero
        },
        exact_rational,
        magnitude,
    }
}
