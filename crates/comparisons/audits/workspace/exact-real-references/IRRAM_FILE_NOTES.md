# iRRAM audit checkpoint

Repository: `iRRAM`, source snapshot in `exact-real-references/iRRAM`. Source inventory is `IRRAM_FILE_INVENTORY.tsv`; all source files are pending except those explicitly marked READ.

### `src/REALS.cc` (780 lines)

Core scalar backend using a dual representation: exact MPFR value/error metadata after materialization, or a deferred double interval (`dp`) before conversion. Arithmetic computes local MPFR precision from operand value sizes, error sizes, and a configurable stack precision policy; multiplication/division propagate interval error through `sizetype` arithmetic. Comparisons return a three-valued `LAZY_BOOLEAN`, failing closed when intervals overlap. Approximation, size, and bound queries use per-thread caches only outside resource-limited regions; cache misses trigger `REITERATE` rather than guessing. `scale` uses a native MP shift when available, otherwise exponentiation-by-squaring. The dual interval/MP representation and explicit fail-closed comparison are architecturally relevant, but Hyperreal already has exact structural bounds and MPFR-backed demand scheduling; no unmeasured replacement is selected. Potential future benchmark target: native exponent-shift scaling and operand-size-aware local precision policy.

### `src/limits.cc` (944 lines)

Implements adaptive limit/lipschitz operators with iteration-stack precision escalation, retry-on-`Iteration`, local error propagation from Lipschitz bounds, monotone fixed-point iteration, and best-success fallback. The pattern of accepting the best certified approximation while escalating only when error fails to improve is relevant to Hyper’s demand scheduler, but Hyper already has adaptive interval refinement; no replacement selected without a benchmark.

### `src/sqrt.cc` (137 lines)

iRRAM uses MPFR sqrt with size/error-aware local precision and a fallback Newton iteration with scaling; zero/small inputs are short-circuited from size bounds. The guarded crossover and demand-sized seed mirror Hyper’s existing integer-only sqrt crossover; no additional change selected.

### `src/exp_log.cc` (178 lines)

Exp uses range reduction by ln2, adaptive Taylor/block series, repeated squaring, and Lipschitz error propagation; log switches from Taylor to AGM at a precision threshold and scales arguments by powers of two. These are relevant algorithmically, but Hyper already has certified range reduction/series scheduling and no measured replacement justified a change.

### `src/sin_cos.cc` (412 lines)

iRRAM implements grouped Taylor evaluation with precision-dependent term blocks, exact triple-angle and modulo-2pi range reduction, and inverse/hyperbolic functions built from these kernels. The reduction strategy is mathematically sound and performance-oriented, but Hyper already has certified range reduction and analytic-continuation nodes for difficult neighborhoods; no unbenchmarked replacement selected.

### `src/STREAMS.cc` (303 lines)

Stream I/O is wrapped with iteration-aware thread-local request/output counters and replay caches so speculative `REITERATE` passes reproduce side effects deterministically; continuous sections prohibit direct I/O and recreate stream state from cache. This is valuable for effectful exact-real runtimes, but Hyper scalar evaluation has no I/O effects, so no direct change selected.

### `src/DYADIC.cc` (262 lines)

Dyadic arithmetic is a thin MPFR-backed value type with precision-parameterized add/sub/mul/div, exact comparisons, absolute value, and native binary shift scaling. Hyper already uses integer/dyadic-style bounds and demand-sized MP arithmetic; no change selected.

### `src/INTERVAL.cc` (320 lines)

Interval arithmetic propagates endpoint bounds, uses lazy Boolean tests for width/range reduction, and evaluates sin/cos extrema by checking critical multiples after modulo-2pi reduction. The implementation exposes apparent endpoint typos in `interval_hull`/`intersect` (`x.upp` reused), so it is not a safe source to transplant; Hyper’s interval code is independently certified.

### `src/LAZY_BOOLEAN.cc` (113 lines)

Three-valued booleans fail closed by triggering `REITERATE` on unresolved conversion, while `choose` scans true values and caches resolved branch indices outside limits. This matches Hyper’s explicit unresolved-decision handling; no change selected.

### `src/REALLIB.cc` (319 lines)

Provides decimal parsing, modulo/power, branch-safe min/max, and matrix exponential/steady-state routines. Matrix algorithms are allocation-heavy and domain-specific; scalar power/min/max patterns are already present in Hyper, so no change selected.

### `src/REALmain.cc` (197 lines)

Initializes a geometric precision schedule from runtime flags, exposes diagnostics, and keeps precision state thread-local. Hyper’s scheduler is compile-time/runtime configured with stronger structural bounds; no worthwhile transplant identified.

### `src/RATIONAL.cc`, `src/GMP_int_ext.c`, `src/GMP_rat_ext.c` (465, 209, 276 lines)

These files wrap canonical GMP integer/rational operations, exact shifts, roots, powers, and string conversion. They use explicit temporary allocation/free pools and canonicalize rationals after mutation. Hyper already has normalized integer/rational constants and binary scaling; no worthwhile scalar change selected.

### `src/MPFR/MPFR_ext.c`, `src/MPFR/mpfr_extension.cc`, `src/convert.cc`, `src/errno.cc` (10, 79, 68, 8 lines)

MPFR extension state is thread-local with bounded free-variable pools; wrappers bridge MPFR inputs to iRRAM demand evaluation and retry until MPFR rounding is certified. Conversion uses size-aware approximation and underflow-to-zero. Hyper’s MPFR bridge and conversion policies already provide equivalent certified behavior; no change selected.

### `src/pi_ln2.cc`, `src/stack.cc` (158, 172 lines)

Constants pi/ln2 are cached per thread and refined through AGM/Borwein or Machin approximants; stack state carries thread-local precision, comparison mode, and a module/Lipschitz search that probes argument error until evaluation succeeds. Hyper already caches constants and schedules demand from structural error bounds; no change selected.

### `src/COMPLEX.cc` (620 lines)

Complex arithmetic propagates shared scalar error, uses multivalued limit selection for square-root branch choice, and composes elementary functions from real kernels. Several inverse-function formulas are branch/argument fragile (for example asin/acos discard part of complex-log information), so this is not a safe exactness transplant; Hyper has no equivalent complex tower yet.

### `src/REALMATRIX.cc` and `src/SPARSEREALMATRIX.cc` (295, 591 lines)

Dense and sparse matrices implement Gaussian elimination with heuristic upper-bound pivot selection; sparse storage maintains row/column linked lists plus a locality hotspot. Both mutate and allocate heavily, and sparse elimination has aliasing/indexing complexity. Hyper’s scalar DAG architecture cannot benefit directly; no change selected.

### `src/MPFR/MPFR_ext.h` (459 lines)

Header-only MPFR kernels choose precision from operand exponents and requested error, strip trailing zero limbs, and use native shift/sqrt operations. This confirms iRRAM’s operand-size-aware precision policy and exact binary scaling, but Hyper already applies stronger structural scheduling and integer-only sqrt fast paths; no replacement selected.
