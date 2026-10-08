# Planning-only lower factorial bound

This isolated candidate changes `e_terms_for_precision`, not the exact series,
final rounding or any shared-cache contract. It is not selected for production.

At loop entry, maintain `0 < L_n = mantissa * 2^shift <= n!`.
Initially `L_0 = 1`. The exact word product `next = mantissa * (n+1)` obeys
`next * 2^shift <= (n+1)!`. Its bit length plus `shift` is the bit length of
that positive lower bound. If it exceeds `needed_bits`, the true factorial is
at least `2^needed_bits`. The candidate thus never returns fewer terms than
the exact-threshold baseline. This argument does not require equality of their
term counts.

Otherwise let `d = max(bits(next)-64, 0)`. Assigning
`mantissa = floor(next/2^d)` and `shift += d` preserves the lower inequality
and a nonzero mantissa of at most 64 bits. The multiplication needs at most
96 bits: a u64 mantissa times a u32 factor. The right shift is at most 32,
and narrowing to u64 is exact after it. These bounds also apply when the
implementation uses u128 compiler helpers on a narrower target; speed/size
on those targets is a separate qualification requirement.

When truncation occurs it retains more than half the positive product; when
it does not occur it is exact. From n >= 4, each accepted update therefore
at least doubles L_n. For the baseline's supported precision arithmetic
(negative p other than i32::MIN, or nonnegative p), `needed_bits <= 2^31+3`.
Termination occurs before the u32 counter or its n+1 addition can overflow.
Before stopping, `shift <= needed_bits`; adding a product bit length <= 96
is far below u64 overflow. The pre-existing `-p` expression and its unsupported
i32::MIN behavior are unchanged; this is not an extreme-precision repair.

For n >= 1, the omitted positive series has
`sum_(k=n+1)^infinity 1/k! < 2/(n+1)!`.
For negative p the stopping condition gives an error below `2^(p-3)`;
rounding the exact partial rational adds at most `2^(p-1)`. Their sum is below
one unit at scale p. For nonnegative p, `needed_bits=4` gives tail < 1/8,
again below the remaining rounding budget. Extra terms only reduce this tail.
This preserves the approximation contract, not necessarily a bit-identical
rounded integer for every possible request or cache history.

Qualification must check exact-threshold neighborhoods, direct numeric kernels,
public first-use/refinement/shared-cache behavior, and independent directed
MPFR enclosures. Benchmarks must distinguish repeated uncached kernels from
fresh-process public e, cached queries and refinement. Allocation instrumentation
must be separate from CPU timing. No retained change is justified by this proof
alone, and no new donor-source credit is claimed for this experiment.
