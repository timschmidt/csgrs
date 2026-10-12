#!/usr/bin/env python3
"""Reject defects in typed word views, scans, truncation and range premises."""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from verify_verus import ROOT, verifier  # noqa: E402

MUTATIONS = [
    ('trimmed_zero_suffix_preserves_value',
     '#[trigger] digits[index] == 0,',
     '#[trigger] digits[index] <= 1,'),
    ('u64_digit_prefix_matches_production',
     '== crate::verified::limbs::prefix(words, length),',
     '== crate::verified::limbs::prefix(words, length) + 1,'),
    ('digit_scan_count_fits_u64',
     'requires width > 0, index < length, within < width,\n'
     '        width * length <= u64::MAX,\n'
     '    ensures',
     'requires width > 0, index < length, within <= width,\n'
     '        width * length <= u64::MAX,\n'
     '    ensures'),
    ('digit_scan_count_fits_u64', 'width * length <= u64::MAX,\n    ensures', 'true,\n    ensures'),
    ('typed_digit_bounds_u32',
     'word_digits_u32(words)[index] < pow2(32),',
     'word_digits_u32(words)[index] < pow2(31),'),
    ('typed_normalized_bit_length_u32', 'requires normalized_u32(words),', 'requires true,'),
    ('typed_zero_scan_u32',
     'requires first_nonzero_u32(words, index),',
     'requires 0 <= index < words.len(), words[index] != 0,'),
    ('typed_one_scan_u32',
     'requires first_nonmaximal_u32(words, index),',
     'requires 0 <= index < words.len(), words[index] != u32::MAX,'),
    ('typed_zero_scan_none_u32', '#[trigger] words[index] == 0,', '#[trigger] words[index] <= 1,'),
    ('typed_one_scan_none_u32',
     '#[trigger] words[index] == u32::MAX,',
     '#[trigger] words[index] >= u32::MAX - 1,'),
    ('typed_zero_suffix_truncation_u32',
     'length == 0 || words[length - 1] > 0,',
     'length == 0 || words[length - 1] >= 0,'),
    ('typed_zero_suffix_truncation_u32',
     '#[trigger] words[index] == 0,',
     '#[trigger] words[index] <= 1,'),
    ('typed_digit_bounds_u64',
     'word_digits_u64(words)[index] < pow2(64),',
     'word_digits_u64(words)[index] < pow2(63),'),
    ('typed_normalized_bit_length_u64', 'requires normalized_u64(words),', 'requires true,'),
    ('typed_zero_scan_u64',
     'requires first_nonzero_u64(words, index),',
     'requires 0 <= index < words.len(), words[index] != 0,'),
    ('typed_one_scan_u64',
     'requires first_nonmaximal_u64(words, index),',
     'requires 0 <= index < words.len(), words[index] != u64::MAX,'),
    ('typed_zero_scan_none_u64', '#[trigger] words[index] == 0,', '#[trigger] words[index] <= 1,'),
    ('typed_one_scan_none_u64',
     '#[trigger] words[index] == u64::MAX,',
     '#[trigger] words[index] >= u64::MAX - 1,'),
    ('typed_zero_suffix_truncation_u64',
     'length == 0 || words[length - 1] > 0,',
     'length == 0 || words[length - 1] >= 0,'),
    ('typed_zero_suffix_truncation_u64',
     '#[trigger] words[index] == 0,',
     '#[trigger] words[index] <= 1,'),
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
    with tempfile.TemporaryDirectory(prefix="hyperreal-word-scan-rejections-") as temporary:
        root = Path(temporary)
        shutil.copytree(ROOT / "src/verified", root / "verified")
        for name in ["magnitude_model.rs", "bit_length_model.rs", "digit_scan_model.rs", "word_scan_model.rs"]:
            shutil.copy2(ROOT / "verification" / name, root / name)
        (root / "lib.rs").write_text(
            "#![feature(proc_macro_hygiene)]\nmod verified;\nmod magnitude_model;\nmod bit_length_model;\nmod digit_scan_model;\nmod word_scan_model;\n")
        model = root / "word_scan_model.rs"
        original = model.read_text()
        changes = [(name, mutate_function(original, name, before, after))
                   for name, before, after in MUTATIONS]
        command = [str(verus), "--edition=2024", "--crate-type=lib", "--no-cheating", str(root / "lib.rs")]
        baseline = subprocess.run(command, capture_output=True, text=True)
        if baseline.returncode:
            sys.exit(f"Unmutated word-scan proofs failed:\n{baseline.stdout}{baseline.stderr}")
        # The complete baseline above verifies every unchanged supporting module.
        command = command[:-1] + ['--verify-only-module', 'word_scan_model', command[-1]]
        for name, changed in changes:
            model.write_text(changed)
            result = subprocess.run(command, capture_output=True, text=True)
            model.write_text(original)
            failures = ["postcondition not satisfied", "assertion failed",
                        "requires not satisfied", "precondition not satisfied"]
            if result.returncode == 0 or not any(message in result.stderr for message in failures):
                sys.exit(f"Expected word-scan rejection for {name}:\n{result.stdout}{result.stderr}")
            print(f"Rejected incorrect word scan: {name}", flush=True)


if __name__ == "__main__":
    main()
