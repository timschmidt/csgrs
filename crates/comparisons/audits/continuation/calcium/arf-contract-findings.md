# ARF contracts and test-source closure — checkpoint 44

This closes the remaining 47 test/driver files and header/manual gaps: 5,783
disjoint ARF lines. All 104 inventoried files in `src/arf/`, `src/arf.h` and
`doc/source/arf.rst` are source-read. Two random-helper excerpts add 66 lines
outside that slice and remain partial; recursive arithmetic/MPFR support is
not declared complete. Source reads do not imply every test or numerical path ran.

Test-source findings at the pinned FLINT version:

- `t-add.c:98` and `t-sub.c:59` overwrite the selected rounding mode with
  truncation toward zero. Their main loops do not exercise all five modes.
- `t-add_si.c:63` uses a random upper limit of one, making its in-place branch
  unreachable. `ulong_extras.h:74` and `randomisation.c:35` prove the limit-one
  result is always zero; this is not inferred from a random sample.
- `t-get_d.c:30` and `:113` select four values and list all four before a
  nearest-even default, so those nearest branches are unreachable. The middle
  loop intentionally chooses four directed modes as well.
- The root/sqrt/reciprocal-sqrt tests choose only four directed modes. Their
  reference calculations also use MPFR, so agreement is not independent of
  the root backend. The new exact-power checks include nearest-even and its
  exact midpoint neighbors, without running exceptional/raw-memory cases.
- `t-approx_dot.c:184` uses `revx`, not `revy`, to index the second vector in
  the reference absolute-product error sum. Opposite vector orientations do
  not use the tested pairings there. This is a source-level oracle issue,
  not a newly reproduced numerical or memory failure in the library.
- Other wrapper tests generally check conversion to the same scalar kernels,
  dirty destinations, exactness flags and public aliases. File/string tests
  predominantly check valid self-generated roundtrips; they do not qualify
  arbitrary malformed input or failures. Those paths were not executed here.

The added native harness checks signed add/sub/div and positive roots of degrees
1, 2, 3, 5, 7 and 17, plus reciprocal square root. Precisions include one bit and
word-boundary neighbors through 257 bits. Finite inputs include bit-pattern and
exponent contrasts, exact powers, `(odd^k + delta)/2^k` halfway neighbors,
zero numerators and cancellation. Denominators remain nonzero. All five rounding
modes and supported whole-object input/output aliases are explicitly enumerated.

The GMP oracle scales an exact rational to the target bit grid, obtains an
integer quotient/root, and uses exact halfway powers for nearest-even. The
separate BigInt checker regenerates every input and validates each complete
result against neighboring representable powers or exact halfway powers. It
does not repeat GMP's root-search algorithm, use residues instead of full values,
or rely on MPFR output agreement. Input preservation and exactness flags are
separate checks. Native/Memcheck output equality also binds all alias results.

Hyper comparison: finite-dyadic total ordering and nearest-even rounding cannot
replace partial computable-real comparison or certified ties-away integer
rounding. Hyper already tests positive/negative half ties and explicitly provides
`near_integer` as a multivalued adjacent-integer operation. Its square-root path
uses guarded integer seeds/Newton refinement, and bounded-degree roots enclose
integer powers while preserving the one-unit approximation contract. Negative
odd roots are supported at the public real boundary. Round-to-odd sticky-bit
compression already appears in Hyper's normal dyadic binary64 conversion.
No nonredundant production transfer with demonstrated benefit is selected here.

The finite native campaign is not a benchmark against Hyper. Its cumulative
allocation totals include the exact oracle and setup; they are not donor-only
allocation, peak-memory or RSS measurements. Nearest-binary64 output and the
integer-rounding/comparison routines read in this pass still need independent
qualification where prior campaigns do not cover them. The earlier numerical,
FENV, LLL and thread-memory failures remain preserved and are not rerun here.
