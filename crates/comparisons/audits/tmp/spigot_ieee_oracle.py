from fractions import Fraction
import random
import spigot

MODE_PAIRS = [
    ("down", spigot.RD), ("up", spigot.RU),
    ("zero", spigot.RZ), ("away", spigot.RA),
    ("nearest_even", spigot.RNE), ("nearest_odd", spigot.RNO),
    ("nearest_down", spigot.RND), ("nearest_up", spigot.RNU),
    ("nearest_zero", spigot.RNZ), ("nearest_away", spigot.RNA),
]

FORMATS = {
    16: (5, 10, "to_ieee_h"),
    32: (8, 23, "to_ieee_s"),
    64: (11, 52, "to_ieee_d"),
}

def scale_pow2(value, exponent):
    if exponent >= 0:
        return value * (1 << exponent)
    return value / (1 << -exponent)

def decode_positive(code, expbits, fracbits):
    fracmask = (1 << fracbits) - 1
    expfield = code >> fracbits
    fraction = code & fracmask
    bias = (1 << (expbits - 1)) - 1
    if expfield == 0:
        significand = fraction
        exponent = 1 - bias - fracbits
    else:
        significand = (1 << fracbits) + fraction
        exponent = expfield - bias - fracbits
    return scale_pow2(Fraction(significand), exponent)

def choose(x, lower_value, lower_bits, upper_value, upper_bits, name):
    if x == lower_value:
        return lower_bits
    if x == upper_value:
        return upper_bits
    if name == "down":
        return lower_bits
    if name == "up":
        return upper_bits
    if name == "zero":
        return lower_bits if x > 0 else upper_bits
    if name == "away":
        return upper_bits if x > 0 else lower_bits
    dlower, dupper = x - lower_value, upper_value - x
    if dlower < dupper:
        return lower_bits
    if dupper < dlower:
        return upper_bits
    if name == "nearest_even":
        return lower_bits if lower_bits % 2 == 0 else upper_bits
    if name == "nearest_odd":
        return lower_bits if lower_bits % 2 != 0 else upper_bits
    if name == "nearest_down":
        return lower_bits
    if name == "nearest_up":
        return upper_bits
    if name == "nearest_zero":
        return lower_bits if x > 0 else upper_bits
    if name == "nearest_away":
        return upper_bits if x > 0 else lower_bits
    raise AssertionError(name)

def check(bits, positive_code, mix_num, mix_den, negative, case_no):
    expbits, fracbits, method_name = FORMATS[bits]
    signbit = 1 << (bits - 1)
    lo_mag = decode_positive(positive_code, expbits, fracbits)
    hi_mag = decode_positive(positive_code + 1, expbits, fracbits)
    mag = (lo_mag * (mix_den - mix_num) + hi_mag * mix_num) / mix_den
    if negative:
        x = -mag
        lower_value, upper_value = -hi_mag, -lo_mag
        lower_bits = signbit | (positive_code + 1)
        upper_bits = signbit | positive_code
    else:
        x = mag
        lower_value, upper_value = lo_mag, hi_mag
        lower_bits, upper_bits = positive_code, positive_code + 1
    obj = spigot.fraction(x.numerator, x.denominator)
    method = getattr(obj, method_name)
    for name, mode in MODE_PAIRS:
        expected_bits = (0 if x == 0 else choose(
            x, lower_value, lower_bits, upper_value, upper_bits, name))
        expected = f"{expected_bits:0{bits // 4}x}"
        actual = method(rmode=mode)
        if actual != expected:
            raise AssertionError(
                f"case={case_no} bits={bits} code={positive_code:#x} "
                f"mix={mix_num}/{mix_den} negative={negative} mode={name} "
                f"x={x} actual={actual} expected={expected}")

rng = random.Random(0x754)
cases = []
for bits, (expbits, fracbits, _) in FORMATS.items():
    max_finite = (((1 << expbits) - 2) << fracbits) | ((1 << fracbits) - 1)
    targeted = {
        0, 1, 2, (1 << fracbits) - 2, (1 << fracbits) - 1,
        1 << fracbits, (1 << fracbits) + 1,
        max_finite - 2, max_finite - 1,
    }
    for exponent_field in range(1, (1 << expbits) - 1):
        if exponent_field in {1, 2, (1 << expbits) // 2,
                              (1 << expbits) - 3, (1 << expbits) - 2}:
            targeted.add((exponent_field << fracbits) - 1)
            targeted.add(exponent_field << fracbits)
    codes = list(targeted)
    random_count = 7000 if bits == 16 else 4000
    codes.extend(rng.randrange(0, max_finite) for _ in range(random_count))
    for code in codes:
        for mix_num, mix_den in ((0, 1), (1, 3), (1, 2), (2, 3)):
            cases.append((bits, code, mix_num, mix_den, False))
            cases.append((bits, code, mix_num, mix_den, True))

for case_no, args in enumerate(cases):
    check(*args, case_no)

print(f"cases={len(cases)} mode_checks={len(cases) * len(MODE_PAIRS)}")
