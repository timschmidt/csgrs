# Finite conversion and integer contracts — checkpoint 45

The 2,401-fixture corpus covers signed finite dyadics with up to 257 mantissa
bits and peak exponents from -1100 through 1100. Structured patterns and explicit
quarter-grid neighbors cover normal/subnormal boundaries, finite overflow,
signed underflow to zero, even/odd midpoint decisions and integer half ties.
All five explicit rounding modes run. Zero denominators, nonfinite inputs,
excessive exponents, malformed data and invalid memory contracts are absent.

The native GMP oracle rounds exact signed dyadics on a binary64 or integer grid.
The independent BigInt checker regenerates the entire corpus and searches the
monotone positive binary64 bit encoding to locate exact neighboring values.
It compares exact midpoint rationals, using mathematical 2^1024 as the upper
overflow neighbor, then verifies all 64 output bits. This does not reuse the
native grid-selection algorithm or MPFR as an output oracle. Integer outputs
have full-value neighboring-integer/halfway certificates. Finite binary64
imports are decoded exactly; no comparison is restricted to residues.

Qualified results: 117,705 native assertions independently rechecked; 12,005
binary64 outputs, including 2,401 nearest-even cases and 164 midpoint fixtures;
11,281 finite imports; 12,005 integer values/exactness flags; 7,985 signed-word
conversions called only after the exact expected integer proves they fit;
14,406 floor/ceil/nint outputs. There are 9,604 public aliases across integer
operations and frexp. Both frexp exponents and full mantissas, bounded magnitude
exponents, integrality, seven power comparisons and the previous-value signed/
absolute comparison are checked. Input values remain unchanged. The corpus
includes 1,848 subnormal outputs, 460 negative zeros and 724 overflow infinities;
these are repeated mode/route counts, not independent discovered identities.

The first harness failed one assertion: it incorrectly expected zero's signed
magnitude exponent to be zero. ARF's manual lines395..400 specifies
-ARF_PREC_EXACT, exactly matching the donor result. The two fmpz bounds explicitly
leave zero unspecified (manual lines385..393). A separately generated v2 changes
only that block: skips those two zero calls, checks the documented signed
sentinel and serializes its full decimal integer as text. The original source,
28,664-byte binary, failed gate and complete output are preserved. V2's non-bound
outputs match v1 exactly. This was an audit-harness contract error, not a donor
numerical defect. The corrected native run passes; no failing case was removed
from its fixture inventory.

V2 native and focused Memcheck stdout are byte-identical (2,781,706 bytes each).
Memcheck reports zero errors, suppressed contexts or live blocks, and all
133,965 allocations freed; 3,847,142 cumulative bytes include oracle/setup.
This is not donor-only allocation, peak demand, RSS or a performance benchmark.
Only native64 ADX with FE_TONEAREST is qualified. Explicit ARF modes are distinct
from the hardware rounding environment. No upstream suite or old failed
numerical/FENV/LLL/thread-memory campaign was rerun.

Source progress adds the complete 238-line fmpz/set.c import implementation.
fmpz/get.c and the selected ARF files were already covered; rereads receive no
new credit. The earlier guessed get_d/get_d_2exp/int.c paths did not exist and
receive none. Source-only raw-array import preconditions are not runtime tested.
The existing 104-file ARF source scope remains complete, not recursive GMP/MPFR
or the entire donor repository.

Hyper comparison: rational conversion already separates exact dyadic export
from lossy/fallback conversion, keeps sticky round-to-odd compression for normal
dyadics and refuses nonfinite intermediate denominators. Its certified integer
rounding is ties-away, not ARF nearest-even; near_integer is a total multivalued
adjacent-integer operation. Finite ARF ordering does not replace partial exact
real comparison. The new donor evidence supplies useful qualification patterns,
not a justified replacement implementation. No production change, transfer or
new speed/size claim is selected; all 956 live source hashes and the five prior
continuation transfers remain unchanged. Inherited checkpoint43 regression
gates bind those same sources; no fresh Rust or whole-native build is performed.

The two new executables total 57,336 bytes in the existing /tmp audit directory.
Three complete numerical logs total 8,340,310 bytes in the workspace. Preserve
all earlier evidence; no cleanup, deletion, donor edit, commit or push occurred.
Approximate-dot reference pairing, add_si's unreachable upstream alias branch,
fixed-grid wrappers and other uncovered operations remain separate follow-ups.
Recursive support, generic field/matrix/algebraic work, every remaining original
reference and complete inventory reconciliation still prevent audit completion.
