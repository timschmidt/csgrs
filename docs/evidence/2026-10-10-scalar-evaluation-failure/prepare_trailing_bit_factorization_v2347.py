"""Freeze a standalone factorization probe for both dependency digit widths."""
import ast
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()

WORD = """
proof fn zero_count_factor_TYPE(value: TYPE)
    requires value > 0,
    ensures
        TYPE_trailing_zeros(value) < WIDTH,
        0 < (value >> TYPE_trailing_zeros(value)) <= value,
        (value >> TYPE_trailing_zeros(value)) % 2 == 1,
        value == (value >> TYPE_trailing_zeros(value)) * pow2(TYPE_trailing_zeros(value) as nat),
{
    axiom_TYPE_trailing_zeros(value);
    let shift = TYPE_trailing_zeros(value) as TYPE;
    let odd = value >> shift;
    assert((value >> shift) % 2 == 1) by (bit_vector)
        requires (value >> shift) & 1TYPE == 1TYPE;
    assert(0 < odd <= value) by (bit_vector)
        requires odd == value >> shift, odd % 2 == 1;
    vstd::bits::lemma_TYPE_mul_pow2_le_max_iff_max_shr(odd, shift, value);
    vstd::bits::lemma_TYPE_shl_is_mul(odd, shift);
    assert((value >> shift) << shift == value) by (bit_vector)
        requires shift < WIDTH, value << ((WIDTH - shift) as TYPE) == 0;
}

proof fn one_count_factor_TYPE(word: TYPE)
    ensures
        TYPE_trailing_ones(word) <= WIDTH,
        ({let unit = pow2(TYPE_trailing_ones(word) as nat);
          let magnitude = word as nat + 1;
          &&& magnitude / unit > 0
          &&& (magnitude / unit) % 2 == 1
          &&& magnitude == (magnitude / unit) * unit}),
{
    axiom_TYPE_trailing_ones(word);
    let complement = !word;
    axiom_TYPE_trailing_zeros(complement);
    let shift = TYPE_trailing_ones(word) as nat;
    lemma2_toWIDTH();
    lemma_pow2_pos(shift);
    let unit = pow2(shift);
    let magnitude = word as nat + 1;
    assert((word as u128) + (complement as u128) == TYPE::MAX as u128) by (bit_vector)
        requires complement == !word;
    if complement == 0 {
        assert(shift == WIDTH);
        assert(magnitude == unit);
        lemma_div_self(unit as int);
    } else {
        zero_count_factor_TYPE(complement);
        let odd = (complement >> TYPE_trailing_zeros(complement)) as nat;
        let other = pow2((WIDTH - shift) as nat);
        lemma_pow2_adds(shift, (WIDTH - shift) as nat);
        assert(magnitude == (other as int - odd as int) * unit) by (nonlinear_arith)
            requires word as nat + complement as nat == TYPE::MAX,
                magnitude == word as nat + 1,
                complement == odd * unit,
                pow2(WIDTH) == TYPE::MAX as nat + 1,
                pow2(WIDTH) == unit * other;
        assert(other > odd) by (nonlinear_arith)
            requires magnitude > 0, unit > 0,
                magnitude == (other as int - odd as int) * unit;
        let quotient = (other - odd) as nat;
        lemma_fundamental_div_mod_converse_div(magnitude as int, unit as int, quotient as int, 0);
        lemma_pow2_unfold((WIDTH - shift) as nat);
        let half = pow2((LAST - shift) as nat);
        lemma_fundamental_div_mod(odd as int, 2);
        assert(quotient as int == 2 * (half as int - (odd as int) / 2 - 1) + 1);
        lemma_mod_multiples_vanish(half as int - (odd as int) / 2 - 1, 1, 2);
    }
}
"""


def main():
    source = E / "trailing-bit-factorization-proposal-v2347.rs"
    runner = E / "probe_trailing_bit_factorization_v2347.py"
    preparation = E / "trailing-bit-factorization-preparation-v2347.json"
    assert not source.exists() and not runner.exists() and not preparation.exists()
    previous = E / "trailing-bit-support-v2346"
    result = json.loads((previous / "result.json").read_text())
    proof = json.loads((previous / "stdout.json").read_text())["verification-results"]
    assert result["exit_code"] == 0 and proof["verified"] == 4 and proof["errors"] == 0
    text = "use vstd::prelude::*;\nuse vstd::arithmetic::{div_mod::*, power2::*};\nuse vstd::std_specs::bits::*;\n\nverus! {\n"
    for width in [32, 64]:
        text += WORD.replace("TYPE", f"u{width}").replace("WIDTH", str(width)).replace("LAST", str(width - 1))
    source.write_text(text + "}\n")
    parent = E / "probe_trailing_bit_support_v2346.py"
    original = parent.read_text()
    tree = ast.parse(original)
    assignment = next(n for n in tree.body if isinstance(n, ast.Assign)
        and any(isinstance(t, ast.Name) and t.id == "PROBE" for t in n.targets))
    lines = original.splitlines(True)
    generated = "".join(lines[:assignment.lineno - 1])
    generated += 'PROBE = (E / "trailing-bit-factorization-proposal-v2347.rs").read_text()\n'
    generated += "".join(lines[assignment.end_lineno:])
    generated = generated.replace('out = E / "trailing-bit-support-v2346"',
                                  'out = E / "trailing-bit-factorization-v2347"')
    generated = generated.replace(
        'Do the existing pinned u32/u64 trailing-ones specifications and actual primitive method contracts establish equality with trailing zeros of the complement?',
        'Do the pinned word bit-count contracts prove that a word plus one factors into the reported power of two and a positive odd quotient, including full all-ones words, for u32 and u64?')
    generated = generated.replace(
        'Probe pinned standard-library bit contracts without changing the candidate.',
        'Probe word-plus-one factorization without changing the candidate.')
    ast.parse(generated)
    runner.write_text(generated)
    preparation.write_text(json.dumps({"status": "Frozen before factorization probing",
        "preparation_sha256": sha(Path(__file__)), "predecessor_runner": parent.name,
        "predecessor_runner_sha256": sha(parent), "support_result_sha256": sha(previous / "result.json"),
        "proposal": source.name, "proposal_sha256": sha(source), "runner": runner.name,
        "runner_sha256": sha(runner), "digit_widths": [32, 64],
        "scope": "Standalone mathematical support probe only. The zero-factor proof follows the existing verified u64 proof's standard-library argument at both native widths; no production body is replaced. BigUint storage/import, normalized limb scans, length bounds and producer callers are still open. SOURCE and retained coverage stay unchanged."}, indent=2) + "\n")
    print(json.dumps({"frozen_widths": [32, 64], "candidate_source_changed": False}))


if __name__ == "__main__":
    main()
