#!/usr/bin/env python3
"""Reject defects in the conditional bit-count and bit-pattern model bridges."""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from verify_verus import ROOT, verifier  # noqa: E402

MUTATIONS = [
    ("trailing_zeros_factor_u32", "(value >> u32_trailing_zeros(value)) % 2 == 1,",
     "(value >> u32_trailing_zeros(value)) % 2 == 0,"),
    ("trailing_ones_factor_u32", "u32_trailing_ones(word) <= 32,",
     "u32_trailing_ones(word) < 32,"),
    ("trailing_ones_factor_u32", "let magnitude = word as nat + 1;\n    assert((word as u128)",
     "let magnitude = word as nat + 2;\n    assert((word as u128)"),
    ("trailing_ones_factor_u64", "u64_trailing_ones(word) <= 64,",
     "u64_trailing_ones(word) < 64,"),
    ("trailing_ones_factor_u64", "let complement = !word;", "let complement = word;"),
    ("odd_factor_of_power_is_unit", "odd % 2 == 1,", "true,"),
    ("odd_factor_of_power_is_unit", "odd * pow2(shift) == pow2(exponent),",
     "odd * pow2(shift) == pow2(exponent + 1),"),
    ("trailing_zeros_determine_power_of_two", "ensures (zeros == (bits - 1) as nat)",
     "ensures (zeros == bits)"),
    ("trailing_zeros_determine_power_of_two", "requires has_bit_length(magnitude, bits),",
     "requires bits > 0,"),
    ("trailing_ones_determine_all_ones", "magnitude + 1 == odd * pow2(ones),",
     "magnitude == odd * pow2(ones),"),
    ("trailing_ones_determine_all_ones", "requires has_bit_length(magnitude, bits),",
     "requires bits > 0,"),
    ("trailing_ones_determine_all_ones", "ensures (ones == bits)",
     "ensures (ones + 1 == bits)"),
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
    with tempfile.TemporaryDirectory(prefix="hyperreal-bit-metadata-rejections-") as temporary:
        root = Path(temporary)
        shutil.copytree(ROOT / "src/verified", root / "verified")
        for name in ["magnitude_model.rs", "bit_length_model.rs"]:
            shutil.copy2(ROOT / "verification" / name, root / name)
        (root / "lib.rs").write_text(
            "#![feature(proc_macro_hygiene)]\nmod verified;\nmod magnitude_model;\nmod bit_length_model;\n")
        model = root / "bit_length_model.rs"
        original = model.read_text()
        changes = [(name, mutate_function(original, name, before, after))
                   for name, before, after in MUTATIONS]
        command = [str(verus), "--edition=2024", "--crate-type=lib", "--no-cheating", str(root / "lib.rs")]
        baseline = subprocess.run(command, capture_output=True, text=True)
        if baseline.returncode:
            sys.exit(f"Unmutated bit-metadata proofs failed:\n{baseline.stdout}{baseline.stderr}")
        for name, changed in changes:
            model.write_text(changed)
            result = subprocess.run(command, capture_output=True, text=True)
            model.write_text(original)
            failures = ["postcondition not satisfied", "assertion failed",
                        "requires not satisfied", "precondition not satisfied"]
            if result.returncode == 0 or not any(message in result.stderr for message in failures):
                sys.exit(f"Expected bit-metadata rejection for {name}:\n{result.stdout}{result.stderr}")
            print(f"Rejected incorrect bit metadata: {name}", flush=True)


if __name__ == "__main__":
    main()
