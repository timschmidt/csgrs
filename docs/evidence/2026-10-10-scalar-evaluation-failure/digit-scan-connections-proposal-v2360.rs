pub open spec fn normalized_digits(digits: Seq<nat>, width: nat) -> bool {
    (forall|index: int| 0 <= index < digits.len() ==> #[trigger] digits[index] < pow2(width))
        && (digits.len() == 0 || digits[digits.len() - 1] > 0)
}

pub proof fn normalized_zero_iff_empty(digits: Seq<nat>, width: nat)
    requires normalized_digits(digits, width),
    ensures digit_value(digits, width) == 0 <==> digits.len() == 0,
{
    digit_prefix_zero(digits, width, digits.len() as int);
    if digits.len() > 0 && digit_value(digits, width) == 0 {
        assert(digits[digits.len() - 1] == 0);
    }
}

pub proof fn normalized_digit_bit_length(digits: Seq<nat>, width: nat, top_bits: nat)
    requires normalized_digits(digits, width), digits.len() > 0,
        crate::magnitude_model::has_bit_length(digits[digits.len() - 1], top_bits),
    ensures crate::magnitude_model::has_bit_length(digit_value(digits, width),
        width * (digits.len() - 1) + top_bits),
{
    let last = digits.len() as int - 1;
    let top = digits[last];
    let lower = digit_prefix(digits, width, last);
    let offset = width * last as nat;
    let unit = digit_weight(width, last as nat);
    let bits = offset + top_bits;
    digit_prefix_bound(digits, width, last);
    lemma_pow2_pos(offset);
    lemma_pow2_adds(offset, (top_bits - 1) as nat);
    lemma_pow2_adds(offset, top_bits);
    assert(pow2((bits - 1) as nat) <= digit_value(digits, width)
        && digit_value(digits, width) < pow2(bits)) by (nonlinear_arith)
        requires unit > 0, 0 <= lower < unit,
            pow2((top_bits - 1) as nat) <= top, top < pow2(top_bits),
            digit_value(digits, width) == lower + top * unit,
            pow2((bits - 1) as nat) == unit * pow2((top_bits - 1) as nat),
            pow2(bits) == unit * pow2(top_bits);
}

pub proof fn zero_scan_determines_power_of_two(
    digits: Seq<nat>, width: nat, index: int, zeros: nat, odd: nat, bits: nat,
)
    requires width > 0, 0 <= index < digits.len(), zeros < width,
        forall|lower: int| 0 <= lower < index ==> #[trigger] digits[lower] == 0,
        odd > 0, odd % 2 == 1, digits[index] == odd * pow2(zeros),
        crate::magnitude_model::has_bit_length(digit_value(digits, width), bits),
    ensures (width * index as nat + zeros == (bits - 1) as nat)
        == (digit_value(digits, width) == pow2((bits - 1) as nat)),
{
    first_nonzero_digit_factor(digits, width, index, zeros, odd);
    let count = width * index as nat + zeros;
    let value = digit_value(digits, width);
    crate::bit_length_model::trailing_zeros_determine_power_of_two(
        value, bits, count, value / pow2(count));
}

pub proof fn one_scan_determines_all_ones(
    digits: Seq<nat>, width: nat, index: int, ones: nat, odd: nat, bits: nat,
)
    requires width > 0, 0 <= index < digits.len(), ones <= width,
        forall|lower: int| 0 <= lower < index ==> #[trigger] digits[lower] + 1 == pow2(width),
        digits[index] + 1 < pow2(width),
        odd > 0, odd % 2 == 1, digits[index] + 1 == odd * pow2(ones),
        crate::magnitude_model::has_bit_length(digit_value(digits, width), bits),
    ensures (width * index as nat + ones == bits)
        == (digit_value(digits, width) + 1 == pow2(bits)),
{
    first_nonmaximal_digit_factor(digits, width, index, ones, odd);
    let count = width * index as nat + ones;
    let value = digit_value(digits, width);
    crate::bit_length_model::trailing_ones_determine_all_ones(
        value, bits, count, (value + 1) / pow2(count));
}

pub proof fn full_digit_scan_determines_all_ones(digits: Seq<nat>, width: nat, bits: nat)
    requires forall|index: int| 0 <= index < digits.len()
        ==> #[trigger] digits[index] + 1 == pow2(width),
        crate::magnitude_model::has_bit_length(digit_value(digits, width), bits),
    ensures (width * digits.len() == bits) == (digit_value(digits, width) + 1 == pow2(bits)),
{
    all_full_digits_factor(digits, width);
    let count = width * digits.len();
    let value = digit_value(digits, width);
    crate::bit_length_model::trailing_ones_determine_all_ones(
        value, bits, count, (value + 1) / pow2(count));
}
