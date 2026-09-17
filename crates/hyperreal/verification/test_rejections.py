#!/usr/bin/env python3
"""Check that executable contracts and arithmetic theorems reject arithmetic defects."""

from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from verify_verus import ROOT, verifier  # noqa: E402
from test_bound_rejections import main as reject_bound_mutations  # noqa: E402
from test_cache_rejections import main as reject_cache_mutations  # noqa: E402
from test_structural_rejections import main as reject_structural_mutations  # noqa: E402


MODEL_FILES = {"magnitude_model.rs", "integer_approximation_model.rs", "real_approximation_model.rs"}


MUTATIONS = [
    ("word.rs", "(false, (-(precision as i64)) as u32)", "(false, precision as u32)"),
    ("word.rs", "(true, (precision - 1) as u32)", "(true, precision as u32)"),
    ("float.rs", "exponent: -149,", "exponent: -148,"),
    ("float.rs", "exponent: -1074,", "exponent: -1073,"),
    ("word.rs", "Some(left.cmp(&right))", "Some(right.cmp(&left))"),
    ("word.rs", "Some((false, 0))", "Some((false, 1))"),
    ("word.rs", "value.checked_mul(1_u128 << shift)", "value.checked_add(1_u128 << shift)"),
    ("word.rs", "    half\n}", "    value / 2\n}"),
    ("word.rs", "    half\n}", "    ((value as u32) >> 1) as i32\n}"),
    ("word.rs", "(if below { 1_i128 } else { 0_i128 })",
     "(if below { 0_i128 } else { 0_i128 })"),
    ("word.rs", "+ offset as i128;", "+ 0_i128;"),
    ("word.rs", "difference < i32::MIN as i128 || difference > i32::MAX as i128",
     "difference <= i32::MIN as i128 || difference > i32::MAX as i128"),
    ("word.rs", "difference < i32::MIN as i128 || difference > i32::MAX as i128",
     "difference < i32::MIN as i128 || difference >= i32::MAX as i128"),
    ("division.rs", "return difference;", "return 0;"),
    ("division.rs", "smaller = remainder;", "smaller = 0;"),
    ("gcd.rs", "64 + ((value >> 64) as u64).trailing_zeros()",
     "63 + ((value >> 64) as u64).trailing_zeros()"),
    ("gcd.rs", "table[start] = small_gcd((start / 64) as u8, (start % 64) as u8);",
     "table[start] = 0;"),
    ("gcd.rs", "return left << common_shift;", "return left;"),
    ("gcd.rs", "return 1_u128 << shift;", "return 1_u128;"),
    ("gcd.rs", "divisor << common_shift\n", "divisor\n"),
    ("fraction.rs", "(numerator / divisor, denominator / divisor)", "(numerator, denominator)"),
    ("fraction.rs", "(numerator / divisor, denominator / divisor)", "(0, denominator / divisor)"),
    ("fraction.rs", "(numerator / divisor, denominator / divisor)", "(numerator / divisor, 1)"),
    ("fraction.rs", "let mut value = 1_u128;", "let mut value = 0_u128;"),
    ("fraction.rs", "value.checked_mul(factors[index])", "value.checked_add(factors[index])"),
    ("fraction.rs", "Some((magnitude, denominator))", "None"),
    ("fraction.rs", "Some((magnitude, denominator))", "Some((denominator, magnitude))"),
    ("limbs.rs", "return ordering;", "return Ordering::Equal;"),
    ("limbs.rs", "first_borrow || second_borrow", "first_borrow && second_borrow"),
    ("limbs.rs", "words[index] = shifted;\n        index += 1;",
     "words[index] = 0;\n        index += 1;"),
    ("limbs.rs", "return index as u32 * 64 + word.trailing_zeros();",
     "return word.trailing_zeros();"),
    ("limbs.rs", "words[index] = shifted;\n    }\n    proof! {\n        let source = word_left_shifted",
     "words[index] = 0;\n    }\n    proof! {\n        let source = word_left_shifted"),
    ("limbs.rs", "Some(words[0] as u128 | (words[1] as u128) << 64)",
     "Some(words[0] as u128)"),
    ("fixed_gcd.rs", "shift: common_shift,", "shift: 0,"),
    ("lehmer.rs", "value >= -(u64::MAX as i128) && value <= u64::MAX as i128",
     "value >= -(u64::MAX as i128) && value < u64::MAX as i128"),
    ("lehmer.rs", "Some(if value < 0 {\n        (-value) as u64\n    } else {\n        value as u64\n    })", "Some(0)"),
    ("lehmer.rs", "Some((left_magnitude, right_magnitude, (left < 0) == (right < 0)))",
     "Some((left_magnitude, right_magnitude, (left < 0) != (right < 0)))"),
    ("lehmer.rs", "Some([a, b, c, d])", "Some([a, b, c, 0])"),
    ("lehmer.rs", "if steps >= 2 { Some([a, b, c, d]) } else { None }", "None"),
    ("lehmer.rs", "        a = c;", "        a = d;"),
    ("product.rs", "addend as u128 + left as u128 * right as u128 + carry as u128;",
     "addend as u128 + left as u128 * right as u128;"),
    ("product.rs", "    product\n}", "    [0_u64; O]\n}"),
    ("product.rs", "let limbs = [word as u64, (word >> 64) as u64];",
     "let limbs = [word as u64, (word >> 63) as u64];"),
    ("product.rs", "multiply(&split_u128(left), &split_u128(right))",
     "multiply(&split_u128(left), &split_u128(left))"),
    ("accumulate.rs", "return Some(index);", "return None;"),
    ("accumulate.rs", "    digit\n}", "    0\n}"),
    ("accumulate.rs", "first_carry || second_carry", "first_carry && second_carry"),
    ("accumulate.rs", "return Some(());", "return None;"),
    ("accumulate.rs", "    Some(())\n}", "    None\n}"),
    ("accumulate.rs", "let product = super::product::multiply_u128(left, right);\n    add_shifted(accumulator, &product, shift)",
     "let product = super::product::multiply_u128(left, right);\n    add_shifted(accumulator, &product, 0)"),
    ("accumulate.rs", "return add_shifted(accumulator, &product, shift);",
     "return add_shifted(accumulator, &product, 0);"),
    ("limbs.rs", "    Some(output)\n}", "    Some([0_u64; O])\n}"),
    ("limbs.rs", "    Some(output)\n}", "    None\n}"),
    ("dyadic.rs", "Some((false, magnitude))", "Some((true, magnitude))"),
    ("dyadic.rs", "    reduced_shift\n}", "    denominator_shift\n}"),
    ("dyadic.rs", "    Some((minus, magnitude, reduced_shift))\n}", "    None\n}"),
    ("dyadic.rs", "    Some((minus, magnitude, reduced_shift))\n}",
     "    Some((minus, magnitude, denominator_shift))\n}"),
    ("aggregate.rs", "maximum = maximum.max(shift);", "maximum = shift;"),
    ("aggregate.rs", "shifts[index] = shift;", "shifts[index] = 0;"),
    ("aggregate.rs", ".checked_add(products[index].right_shift)?",
     ".checked_sub(products[index].right_shift)?"),
    ("aggregate.rs", "if product.active {\n            let shift", "if !product.active {\n            let shift"),
    ("aggregate.rs", "if product.negative {", "if !product.negative {"),
    ("aggregate.rs", "super::accumulate::add_product(accumulator, left, product.right, shift)",
     "super::accumulate::add_product(accumulator, left, 0, shift)"),
    ("aggregate.rs", "super::accumulate::add_wide_product(accumulator, &left, product.right, shift)",
     "super::accumulate::add_wide_product(accumulator, &left, 0, shift)"),
    ("aggregate.rs", "super::dyadic::finish(positive, negative, maximum)",
     "super::dyadic::finish(negative, positive, maximum)"),
    ("aggregate.rs", "Some(result) => Some(result)", "Some(result) => None"),
    ("aggregate.rs", "Some((false, zero, 0))", "Some((true, zero, 0))"),
    ("word.rs", "checked_shift_left(value, shift as u32)", "checked_shift_left(value, (shift % 64) as u32)"),
    ("dyadic.rs", "            negative: false,\n            magnitude: 0,",
     "            negative: true,\n            magnitude: 0,"),
    ("dyadic.rs", "        magnitude: reduced,", "        magnitude: magnitude,"),
    ("dyadic.rs", "        magnitude: reduced,\n        shift,",
     "        magnitude: reduced,\n        shift: denominator_shift,"),
    ("dyadic.rs", "Some(normalize_word(negative, magnitude, scale))", "None"),
    ("dyadic.rs", "Some(normalize_word(negative, magnitude, scale))", "Some(normalize_word(!negative, magnitude, scale))"),
    ("dyadic.rs", "        !right.negative,\n        right_magnitude,", "        right.negative,\n        right_magnitude,"),
    ("aggregate.rs", "if !product.active {\n        return Some((false, 0));",
     "if product.active {\n        return Some((false, 0));"),
    ("aggregate.rs", "let magnitude = left.checked_mul(product.right)?;", "let magnitude = left.checked_add(product.right)?;"),
    ("aggregate.rs", "Some((product.negative && scaled != 0, scaled))", "Some((!product.negative && scaled != 0, scaled))"),
    ("aggregate.rs", "Some(super::dyadic::normalize_word(negative, magnitude, maximum))", "None"),
    ("aggregate.rs", "Some(super::dyadic::normalize_word(negative, magnitude, maximum))",
     "Some(super::dyadic::normalize_word(negative, magnitude, 0))"),
    ("real_approximation_model.rs", "|| rounded_after_shift(approximation, 1) == -(-value).floor()",
     "&& rounded_after_shift(approximation, 1) == -(-value).floor()"),
    ("real_approximation_model.rs", "ensures approximation > 1 ==> value > 0real,",
     "ensures approximation >= 1 ==> value > 0real,"),
    ("integer_approximation_model.rs", "(value / pow2(preliminary_bits) as int + 1) / 2",
     "(value / pow2(preliminary_bits) as int - 1) / 2"),
    ("integer_approximation_model.rs", "requires denominator > 0, gap > 0,",
     "requires denominator > 0, gap >= 0,"),
    ("magnitude_model.rs", "- if below_aligned(numerator, denominator, numerator_bits, denominator_bits)",
     "+ if below_aligned(numerator, denominator, numerator_bits, denominator_bits)"),
    ("magnitude_model.rs", "&& numerator * binary_denominator(exponent) < 2 * binary_numerator(exponent) * denominator",
     "&& numerator * binary_denominator(exponent) <= 2 * binary_numerator(exponent) * denominator"),
    ("real_approximation_model.rs", "if shift >= 0 { value * pow2(shift as nat) }",
     "if shift >= 0 { value * pow2(shift as nat + 1) }"),
    ("real_approximation_model.rs", "else { rounded_after_shift(value, (-shift - 1) as nat) }",
     "else { rounded_after_shift(value, (-shift - 1) as nat) + 1 }"),
    ("real_approximation_model.rs", "-(pow2(bits) as int) < approximation < pow2(bits),",
     "-(pow2(bits) as int) <= approximation <= pow2(bits),"),
    ("real_approximation_model.rs", "-2real * left_bound <= left <= 2real * left_bound,",
     "-3real * left_bound <= left <= 3real * left_bound,"),
    ("real_approximation_model.rs", "ensures -unit / 2real <= (a * b) as real * (left_unit * right_unit) - left * right",
     "ensures -unit / 4real <= (a * b) as real * (left_unit * right_unit) - left * right"),
    ("real_approximation_model.rs", "ensures approximates(left * right,\n        scaled_integer(a * b, precision - left_magnitude - right_magnitude - 6),",
     "ensures approximates(left * right,\n        scaled_integer(a * b, precision - left_magnitude - right_magnitude - 5),"),
    ("real_approximation_model.rs", "requires approximates(right, 0, binary_unit(precision - left_magnitude - 3)),",
     "requires approximates(right, 8, binary_unit(precision - left_magnitude - 3)),"),
    ("real_approximation_model.rs", "requires strictly_approximates(left, a, unit), strictly_approximates(right, b, unit),",
     "requires approximates(left, a, unit), approximates(right, b, unit),"),
    ("real_approximation_model.rs", "ensures a >= b + 2 ==> left > right,",
     "ensures a >= b + 1 ==> left > right,"),
    ("real_approximation_model.rs", "(-2real * right_bound < b as real * right_unit < 2real * right_bound)\n            ==> -unit / 2real <",
     "(-2real * right_bound <= b as real * right_unit <= 2real * right_bound)\n            ==> -unit / 2real <"),
]


def main():
    verus = verifier()
    with tempfile.TemporaryDirectory(prefix="hyperreal-proof-rejections-") as temporary:
        root = Path(temporary)
        source = root / "verified"
        shutil.copytree(ROOT / "src/verified", source)
        for name in MODEL_FILES:
            shutil.copy2(ROOT / "verification" / name, root / name)
        paths = {name: (root / name if name in MODEL_FILES else source / name)
                 for name, _, _ in MUTATIONS}
        originals = {name: path.read_text() for name, path in paths.items()}
        for name, before, _ in MUTATIONS:
            if originals[name].count(before) != 1:
                sys.exit(f"Mutation no longer uniquely matches {name}: {before}")
        (root / "lib.rs").write_text(
            "#![feature(proc_macro_hygiene)]\nmod verified;\nmod magnitude_model;\nmod integer_approximation_model;\nmod real_approximation_model;\n")
        command = [str(verus), "--edition=2024", "--crate-type=lib", "--no-cheating", str(root / "lib.rs")]
        baseline = subprocess.run(command, capture_output=True, text=True)
        if baseline.returncode:
            sys.exit(f"Unmutated kernel proofs failed:\n{baseline.stdout}{baseline.stderr}")
        for name, before, after in MUTATIONS:
            path = paths[name]
            original = originals[name]
            path.write_text(original.replace(before, after))
            result = subprocess.run(command, capture_output=True, text=True)
            path.write_text(original)
            failures = ["postcondition not satisfied", "invariant not satisfied"]
            if name in MODEL_FILES:
                failures += ["assertion failed", "requires not satisfied", "precondition not satisfied"]
            if result.returncode == 0 or not any(message in result.stderr for message in failures):
                sys.exit(f"Expected contract rejection for {name}: {after}\n{result.stdout}{result.stderr}")
            print(f"Rejected incorrect {name}: {after}")
        # The production runner's --no-cheating gate must also reject an
        # attempted proof bypass, even if every postcondition becomes vacuous.
        path = source / "word.rs"
        original = path.read_text()
        before = "let left = left_numerator.checked_mul(right_denominator)?;"
        path.write_text(original.replace(before, "proof! { assume(false); }\n" + before))
        result = subprocess.run(command, capture_output=True, text=True)
        if result.returncode == 0 or "not allowed" not in result.stderr:
            sys.exit(f"Expected rejection of assume(false):\n{result.stdout}{result.stderr}")
        print("Rejected attempted assumption bypass")
    reject_bound_mutations()
    reject_cache_mutations()
    reject_structural_mutations()


if __name__ == "__main__":
    main()
