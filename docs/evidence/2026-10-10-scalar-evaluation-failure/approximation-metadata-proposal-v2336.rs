// Connect bit-pattern premises to the endpoint contract without allocating
// endpoint integers. Actual BigUint metadata and producer callers remain open.
proof fn approximation_bit_pattern_endpoints(
    magnitude: nat, bits: u64, precision: i32,
    all_ones: bool, power_of_two: bool, exponent: i32,
)
    requires bits > 1,
        crate::magnitude_model::has_bit_length(magnitude, bits as nat),
        all_ones == (magnitude + 1 == pow2(bits as nat)),
        power_of_two == (magnitude == pow2((bits as nat - 1) as nat)),
        call_ensures(crate::verified::word::checked_bit_length_msd,
            (bits, if all_ones { 0u64 } else { 1u64 }, false, precision), Some(exponent)),
    ensures magnitude > 1,
        (magnitude as real + 1real) * binary_unit(precision as int)
            < 2real * binary_unit(exponent as int),
        !power_of_two && !all_ones ==> binary_unit(exponent as int)
            <= (magnitude as real - 1real) * binary_unit(precision as int),
{
    let lower = pow2((bits as nat - 1) as nat);
    let upper = pow2(bits as nat);
    let unit = binary_unit(precision as int);
    vstd::arithmetic::power::lemma_pow0(2);
    lemma_pow2_strictly_increases(0, (bits as nat - 1) as nat);
    assert(magnitude > 1);
    binary_unit_positive(precision as int);
    binary_unit_composes(precision as int, bits as int);
    binary_unit_composes(precision as int, bits as int - 1);
    assert(binary_unit(bits as int) == upper as real) by (nonlinear_arith)
        requires binary_unit(bits as int) == upper as real / 1real;
    assert(binary_unit(bits as int - 1) == lower as real) by (nonlinear_arith)
        requires binary_unit(bits as int - 1) == lower as real / 1real;
    if all_ones {
        assert(exponent as int == precision as int + bits as int);
        assert((magnitude as real + 1real) * unit < 2real * binary_unit(exponent as int))
            by (nonlinear_arith)
            requires unit > 0real, upper > 0,
                magnitude + 1 == upper,
                binary_unit(exponent as int) == unit * upper as real;
    } else {
        assert(exponent as int == precision as int + bits as int - 1);
        assert(magnitude + 1 < upper);
        binary_unit_successor(exponent as int);
        assert((magnitude as real + 1real) * unit < 2real * binary_unit(exponent as int))
            by (nonlinear_arith)
            requires unit > 0real,
                magnitude as real + 1real < upper as real,
                2real * binary_unit(exponent as int) == unit * upper as real;
        if !power_of_two {
            assert(lower <= magnitude - 1);
            assert(binary_unit(exponent as int) <= (magnitude as real - 1real) * unit)
                by (nonlinear_arith)
                requires unit > 0real, lower as real <= magnitude as real - 1real,
                    binary_unit(exponent as int) == unit * lower as real;
        }
    }
}

// This is a conditional bridge through two actual production contracts. It
// does not assert that BigInt imports or bound_from_approx establish its inputs.
proof fn approximation_metadata_produces_valid_bound(
    value: real, approximation: int, magnitude: nat, magnitude_bits: u64,
    precision: i32, reaches_next_binade: bool, is_power_of_two: bool,
    sign: Sign, msd: Option<i32>, exact_msd: bool, result: BoundInfo,
)
    requires magnitude_bits > 1,
        magnitude as int == if approximation < 0 { -approximation } else { approximation },
        crate::magnitude_model::has_bit_length(magnitude, magnitude_bits as nat),
        reaches_next_binade == (magnitude + 1 == pow2(magnitude_bits as nat)),
        is_power_of_two == (magnitude == pow2((magnitude_bits as nat - 1) as nat)),
        sign == if approximation > 0 { Sign::Plus } else { Sign::Minus },
        approximates(value, approximation, binary_unit(precision as int)),
        call_ensures(crate::verified::word::checked_bit_length_msd,
            (magnitude_bits, if reaches_next_binade { 0u64 } else { 1u64 }, false, precision), msd),
        exact_msd == (msd.is_some() && !is_power_of_two && !reaches_next_binade),
        call_ensures(BoundInfo::with_sign_msd, (sign, msd, exact_msd), result),
    ensures bound_denotes(result, value),
{
    vstd::arithmetic::power::lemma_pow0(2);
    lemma_pow2_strictly_increases(0, (magnitude_bits as nat - 1) as nat);
    assert(magnitude > 1);
    assert(approximation > 1 || approximation < -1);
    assert(absolute(approximation as real) == magnitude as real);
    if let Some(exponent) = msd {
        approximation_bit_pattern_endpoints(magnitude, magnitude_bits, precision,
            reaches_next_binade, is_power_of_two, exponent);
    }
    separated_approximation_yields_a_bound(value, approximation, binary_unit(precision as int),
        sign, msd, exact_msd, result);
}
