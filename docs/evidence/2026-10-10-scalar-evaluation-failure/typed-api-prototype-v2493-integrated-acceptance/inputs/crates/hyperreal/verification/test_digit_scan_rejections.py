#!/usr/bin/env python3
"""Reject defects in conditional generic digit scans and metadata connections."""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from verify_verus import ROOT, verifier  # noqa: E402

MUTATIONS = [
    ('digit_prefix_bound',
     '#[trigger] digits[index] < pow2(width),',
     '#[trigger] digits[index] <= pow2(width),'),
    ('digit_prefix_zero',
     'ensures digit_prefix(digits, width, length) == 0 <==>',
     'ensures digit_prefix(digits, width, length) == 1 <==>'),
    ('digit_prefix_full',
     '#[trigger] digits[index] + 1 == pow2(width),',
     '#[trigger] digits[index] + 1 <= pow2(width),'),
    ('upper_digits_preserve_odd_factor',
     'requires odd > 0, odd % 2 == 1, shift < width,',
     'requires odd > 0, shift < width,'),
    ('upper_digits_preserve_odd_factor',
     'requires odd > 0, odd % 2 == 1, shift < width,',
     'requires odd > 0, odd % 2 == 1, shift <= width,'),
    ('all_full_digits_factor',
     'ensures has_odd_factor(digit_value(digits, width) + 1, width * digits.len()),',
     'ensures has_odd_factor(digit_value(digits, width) + 1, width * digits.len() + 1),'),
    ('first_nonzero_digit_factor',
     '#[trigger] digits[lower] == 0,',
     '#[trigger] digits[lower] <= 1,'),
    ('first_nonzero_digit_factor',
     'odd > 0, odd % 2 == 1, digits[index] == odd * pow2(zeros),',
     'odd > 0, odd % 2 == 1, digits[index] + 1 == odd * pow2(zeros),'),
    ('first_nonmaximal_digit_factor',
     '#[trigger] digits[lower] + 1 == pow2(width),',
     '#[trigger] digits[lower] + 1 <= pow2(width),'),
    ('first_nonmaximal_digit_factor',
     'odd > 0, odd % 2 == 1, digits[index] + 1 == odd * pow2(ones),',
     'odd > 0, odd % 2 == 1, digits[index] + 2 == odd * pow2(ones),'),
    ('normalized_zero_iff_empty', 'requires normalized_digits(digits, width),', 'requires true,'),
    ('normalized_digit_bit_length',
     'requires normalized_digits(digits, width), digits.len() > 0,',
     'requires digits.len() > 0,'),
    ('normalized_digit_bit_length',
     'has_bit_length(digits[digits.len() - 1], top_bits),',
     'has_bit_length(digits[digits.len() - 1], top_bits + 1),'),
    ('zero_scan_determines_power_of_two',
     'ensures (width * index as nat + zeros == (bits - 1) as nat)',
     'ensures (width * index as nat + zeros == bits)'),
    ('zero_scan_determines_power_of_two',
     'crate::magnitude_model::has_bit_length(digit_value(digits, width), bits),',
     'bits > 0,'),
    ('one_scan_determines_all_ones',
     'ensures (width * index as nat + ones == bits)',
     'ensures (width * index as nat + ones + 1 == bits)'),
    ('full_digit_scan_determines_all_ones',
     'ensures (width * digits.len() == bits)',
     'ensures (width * digits.len() + 1 == bits)'),
    ('u32_digit_bit_length',
     'has_bit_length(word as nat, (32 - u32_leading_zeros(word)) as nat),',
     'has_bit_length(word as nat, (31 - u32_leading_zeros(word)) as nat),'),
    ('u32_digit_bit_length',
     '0 <= u32_leading_zeros(word) <= 32,',
     '0 <= u32_leading_zeros(word) < 32,'),
]


def mutate_function(source, name, before, after):
    anchor = "proof fn " + name + "("
    assert source.count(anchor) == 1, name
    start = source.index(anchor)
    end = source.index("\n}\n", start) + 3
    function = source[start:end]
    assert function.count(before) == 1, (name, before)
    return source[:start] + function.replace(before, after) + source[end:]


def main():
    verus = verifier()
    with tempfile.TemporaryDirectory(prefix="hyperreal-digit-scan-rejections-") as temporary:
        root = Path(temporary)
        shutil.copytree(ROOT / "src/verified", root / "verified")
        for name in ["magnitude_model.rs", "bit_length_model.rs", "digit_scan_model.rs"]:
            shutil.copy2(ROOT / "verification" / name, root / name)
        (root / "lib.rs").write_text(
            "#![feature(proc_macro_hygiene)]\nmod verified;\nmod magnitude_model;\nmod bit_length_model;\nmod digit_scan_model;\n")
        model = root / "digit_scan_model.rs"
        original = model.read_text()
        changes = [(name, mutate_function(original, name, before, after))
                   for name, before, after in MUTATIONS]
        command = [str(verus), "--edition=2024", "--crate-type=lib", "--no-cheating", str(root / "lib.rs")]
        baseline = subprocess.run(command, capture_output=True, text=True)
        if baseline.returncode:
            sys.exit(f"Unmutated digit-scan proofs failed:\n{baseline.stdout}{baseline.stderr}")
        # The complete baseline above verifies every unchanged supporting module.
        command = command[:-1] + ['--verify-only-module', 'digit_scan_model', command[-1]]
        for name, changed in changes:
            model.write_text(changed)
            result = subprocess.run(command, capture_output=True, text=True)
            model.write_text(original)
            failures = ["postcondition not satisfied", "assertion failed",
                        "requires not satisfied", "precondition not satisfied"]
            if result.returncode == 0 or not any(message in result.stderr for message in failures):
                sys.exit(f"Expected digit-scan rejection for {name}:\n{result.stdout}{result.stderr}")
            print(f"Rejected incorrect digit scan: {name}", flush=True)


if __name__ == "__main__":
    main()
