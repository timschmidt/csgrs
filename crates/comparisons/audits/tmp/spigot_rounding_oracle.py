from fractions import Fraction
import random
import spigot

DIGITS = "0123456789abcdefghijklmnopqrstuvwxyz"

MODES = [
    spigot.RD, spigot.RU, spigot.RZ, spigot.RA,
    spigot.RNE, spigot.RNO, spigot.RND, spigot.RNU,
    spigot.RNZ, spigot.RNA,
]
rz_integer_failures = []

def round_integer(x, mode):
    lo = x.numerator // x.denominator
    if x == lo:
        return lo
    hi = lo + 1
    if mode == spigot.RD:
        return lo
    if mode == spigot.RU:
        return hi
    if mode == spigot.RZ:
        return lo if x > 0 else hi
    if mode == spigot.RA:
        return hi if x > 0 else lo
    dlo, dhi = x - lo, hi - x
    if dlo < dhi:
        return lo
    if dhi < dlo:
        return hi
    if mode == spigot.RNE:
        return lo if lo % 2 == 0 else hi
    if mode == spigot.RNO:
        return lo if lo % 2 != 0 else hi
    if mode == spigot.RND:
        return lo
    if mode == spigot.RNU:
        return hi
    if mode == spigot.RNZ:
        return lo if x > 0 else hi
    if mode == spigot.RNA:
        return hi if x > 0 else lo
    raise AssertionError(mode)

def parse_base(text, base):
    negative = text.startswith("-")
    if text[:1] in "+-":
        text = text[1:]
    whole, point, frac = text.partition(".")
    value = 0
    for char in whole:
        digit = DIGITS.index(char.lower())
        assert digit < base, (text, base, char)
        value = value * base + digit
    denom = 1
    for char in frac:
        digit = DIGITS.index(char.lower())
        assert digit < base, (text, base, char)
        value = value * base + digit
        denom *= base
    result = Fraction(value, denom)
    return -result if negative else result

def check_case(case_no, x, base, digitlimit):
    value = spigot.fraction(x.numerator, x.denominator)
    scale = Fraction(base) ** digitlimit
    for mode in MODES:
        expected_q = round_integer(x * scale, mode)
        expected = Fraction(expected_q, 1) / scale
        text = value.base_format_str(
            base=base, digitlimit=digitlimit, rmode=mode)
        actual = parse_base(text, base)
        if actual != expected:
            raise AssertionError(
                f"case={case_no} x={x} base={base} d={digitlimit} "
                f"mode={mode} text={text} actual={actual} expected={expected}")
        actual_int = value.to_int(rmode=mode)
        expected_int = round_integer(x, mode)
        if actual_int != expected_int:
            if mode == spigot.RZ:
                rz_integer_failures.append(
                    (case_no, x, actual_int, expected_int))
            else:
                raise AssertionError(
                    f"integer case={case_no} x={x} mode={mode} "
                    f"actual={actual_int} expected={expected_int}")

rng = random.Random(0x535049474F54)
cases = []
for i in range(6000):
    base = rng.randrange(2, 37)
    digitlimit = rng.randrange(-5, 21)
    numerator = rng.randrange(-(1 << 180), 1 << 180)
    denominator = rng.randrange(1, 1 << 96)
    cases.append((Fraction(numerator, denominator), base, digitlimit))

for i in range(4000):
    base = rng.randrange(2, 37)
    digitlimit = rng.randrange(-5, 21)
    q = rng.randrange(-(1 << 100), 1 << 100)
    scale = Fraction(base) ** digitlimit
    cases.append((Fraction(2 * q + 1, 2) / scale, base, digitlimit))

for i, (x, base, digitlimit) in enumerate(cases):
    check_case(i, x, base, digitlimit)

print(f"cases={len(cases)} mode_checks={len(cases) * len(MODES) * 2} "
      f"rz_integer_failures={len(rz_integer_failures)} "
      f"failures={rz_integer_failures}")
