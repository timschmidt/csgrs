# Feature tradeoff ledger

[The admission protocol](../admission.md#bounded-code-and-binary-size-exceptions)
favors features that add capability while improving performance, reducing
memory use and shrinking code. Every retained feature with any of those
effects, or a trade between them, is listed here with its measured effects.
Reconsider this ledger at least at each coordinated release and phase close.
For each entry, decide whether to keep, simplify, merge or remove it, measured
against the current codebase. Record the decision and the date in the entry.

Signs are relative to the revision before the feature. "Development" sizes are
tests, benches, examples and probes, which are recorded but do not block
retention. "Shipped" sizes are linked release or deployed images.

## Entries

### Adaptive Taylor order continuation (22 November 2026)

- **Capability:** none new; the adaptive controller continues the previous
  order's prepared stages instead of re-deriving them.
- **Performance:** controller instructions −3.45%; Van der Pol maximum order 8
  −8.16%.
- **Memory:** prepared programs share their stage prefix.
- **Code size:** development artifacts within the then-current caps.
- **Evidence:** `docs/evidence/2026-11-22-interval-adaptive-order-continuation`.
- **Reconsider:** if order preparation moves into Hypersolve or caches
  across fields.

### Exact work-budget receipts (23 November 2026)

- **Capability:** exact `u64` limits and observations on 32-bit targets.
- **Performance:** unchanged on the success path; denial receipts decode
  through accessors.
- **Memory:** receipt and `Option` sizes unchanged on every target.
- **Code size:** shipped Interface WASM −929 B final, −935 B raw; development
  Hypersolve library tests +10,952 B in the default profile (−8,224 B with
  one codegen unit).
- **Evidence:** `docs/evidence/2026-11-23-interval-work-budget-receipt`.
- **Reconsider:** if `BudgetExceeded` gains kinds or fields; the packed
  layout must still keep 16 bytes on 32-bit targets.

### Correlated flow rounding (23 November 2026)

- **Capability:** nonlinear correlated horizons that previously stopped after
  two steps with `SizeExhausted` now complete or reach the work budget.
- **Performance:** rotation probe 1.12 s → 0.43 s at noise cap 2.
- **Memory:** coefficient sizes bounded by the policy grid instead of growing
  per step.
- **Code size:** development `flow_noise` named text +2,904 B.
- **Evidence:** `docs/evidence/2026-11-23-interval-flow-noise-rounding`.
- **Reconsider:** with general nonlinear remainder arithmetic, which may own
  the rounding.

### Relative noise-column cutoff (23 November 2026)

- **Capability:** caps of 32 and 64 noise columns complete instead of
  exhausting budgets.
- **Performance:** rotation at cap 16 1.95 s → 0.40 s with equal widths.
- **Memory:** retained columns drop from 16 to at most 4 in rotation.
- **Code size:** development `flow_noise` named text +1,814 B.
- **Evidence:** `docs/evidence/2026-11-23-interval-flow-noise-rounding`.
- **Reconsider:** when adaptive column strategies beyond a fixed relative
  cutoff are evaluated.

### Exact positive integer interval powers (24 November 2026)

- **Capability:** none new; identical results through a shorter path.
- **Performance:** long correlated rotation −15.0% instructions over 8 steps
  and −12.5% over 64.
- **Memory:** avoids the extended-interval and two-component intermediates of
  the general rational-power path.
- **Code size:** shipped Interface WASM images byte-identical; development
  Hyperdynamics artifacts about +1,350 B named text; several Hypersolve
  benches shrink by up to 7.3 KB.
- **Evidence:** `docs/evidence/2026-11-24-interval-positive-power`.
- **Reconsider:** if Hyperinterval's general rational power gains its own
  integer fast path, which would make this one redundant.

### Interval Taylor-mode remainder recurrences (25 November 2026)

- **Capability:** tighter high-order Taylor remainders (1/(1+x) on
  [-1/4, 1/4] at order 6: 2e26 → 1.2e-3); models whose generated partials
  failed with a zero-containing enclosure now build.
- **Performance:** transcendental and rational model builds 4.7–34% faster;
  tiny polynomial builds paid a fixed 1–3 µs (+3% to +8%), removed by the
  next entry.
- **Memory:** dense series per reached node, up to the remainder degree.
- **Code size:** development artifacts only; no shipped image enables Taylor
  features.
- **Evidence:** `docs/evidence/2026-11-25-interval-taylor-recurrence`.
- **Reconsider:** if centered or mean-value forms are added for the degree-zero
  images, or if sparse series would pay off on many variables.

### Exact Taylor coefficients from the same recurrences (26 November 2026)

- **Capability:** none new; identical coefficients, remainders and images.
  The generated partial formulas remain available on request and as the
  fallback.
- **Performance:** model construction 3–49% faster at order 2 and 66–99.8%
  faster at order 8 (rational conic coordinate 170 ms → 0.32 ms); polynomial
  re-differentiation −23% to −45%.
- **Memory:** models no longer retain the generated formula program, about
  42,000 nodes at order 8 for that conic; they keep the request list.
- **Code size:** development Taylor benches and the `flow_noise` example
  +32–42 KB `.text` (a second monomorphization over exact `Real`); no
  shipped image enables Taylor features.
- **Evidence:** `docs/evidence/2026-11-26-interval-taylor-exact-recurrence`.
- **Reconsider:** if hyperreal gains cheaper exact series arithmetic, or if the
  formula fallback is proved unreachable and can be removed.

### Opt-in Taylor-mode ODE step coefficients (26 November 2026)

- **Capability:** `ExpressionProgram::ode_taylor_coefficients` in Hypersolve;
  `FlowTaylorCoefficients::Recurrence` in Hyperdynamics (opt-in).
- **Performance:** validated-dynamics corpus 11–37% faster on nontrivial
  fields; constant and turnaround fields 3–6% slower; widths equal to
  2.3e-11 relative, or narrower (5% on a transcendental quotient field).
- **Memory:** per-step dense univariate series per reached node, up to the
  remainder degree; no retained formula change.
- **Code size:** not measured separately; no shipped image enables Taylor
  features.
- **Evidence:** `docs/evidence/2026-11-26-interval-ode-taylor-recurrence`.
- **Decision (26 November 2026):** made the default, including adaptive and
  QR programs, under the owner's fluid-API, CAD/CAM/Sim-first guidance.
- **Reconsider:** drop the formula evaluation path if the recurrence is never
  unavailable in practice.


### Exact torus cell ranges in Hypersdf (27 November 2026)

- **Capability:** `classify_cell` now certifies torus cells inside, and
  certifies them outside down to a gap of at least 2^-40. Before, every cell
  just outside the surface was `Boundary`, and none were certified inside.
- **Method:** F(u, a) = (u + a + c)^2 - 4 R^2 u is convex in the exact
  radial and axial ranges u and a. A ring torus takes three evaluations: the
  maximum is on edge a = a1 and the minimum on edge a = a0. Other tori use
  the cheap two-term bound when it already excludes zero, and otherwise the
  corners and edge stationary points.
- **Effect:** on the 16^3 atlas torus grid, boundary cells go from 1,488 to
  472 and inside cells from 0 to 96. Whole-grid classification takes the
  same time at 8^3, 1.13x at 16^3 and 1.2x at 24^3. Callers that subdivide
  boundary cells do fewer of them.
- **Code size:** one exact-range function and three tests.
- **Reconsider:** when another non-monotone primitive (capsule, rounded box)
  shows the same dependency loss in the atlas.

### Interned rationals keep only process-lifetime results (27 November 2026)

- **Capability:** none new. It removes the process-lifetime retention found
  by the hyperphysics resource atlas, where spring residue was about equal to
  the peak heap.
- **Rule:** an interned value keeps a result only if the result is interned
  or word-sized. A kept word-sized result becomes process-lifetime and obeys
  the same rule, so each slot pins at most one small value.
- **Memory:** a 1024-step exact spring integration that starts from interned
  states leaves 2.4 KB and then 1.9 KB on two runs, and nothing more after
  that. It used to leave 3.8 MB per run; the pinned amount grew about n^1.75
  with the step count.
- **Performance:**
  - Hyperreal rational micro benchmarks: 151 benchmarks, two alternating
    passes per side, geometric mean 1.0001 (neutral). The largest slowdown on
    a reachable path is `exact_rational_sparse_homogeneous_plane_intersection3`
    at +4.6%. `rational_clone_pair` at +13% is code layout: it clones large
    non-interned values and never reaches the changed code.
  - The spring probe is about 25% faster (65 ms to 46–50 ms).
  - The pinned Hypersolve dispatch traces (polynomial import, Jacobian rules)
    are unchanged.
- **Rejected designs:**
  - Sealing every slot on interned values lost warm hits:
    `rational_matmul_8` +5%, `tanh_rational_one` +10%, and the polynomial
    dispatch count went from 11 to 15 events.
  - Keeping only interned results lost the warm retained inverse, negation,
    product and square-reduction hits in the Jacobian trace.
- **Code size:** one fact bit, which narrows the dyadic shift cache from 23
  to 22 bits; one predicate on the five cold retention paths; two retention
  tests.
- **Reconsider:** if pooled values become per-thread or reclaimable, or if
  cached results gain a weak-only mode.

### QR frame proposal backoff (25 November 2026)

- **Capability:** none new; a cheaper frame-proposal schedule.
- **Performance:** probe corpus 1.0–5.1× faster; perf-atlas QR integrate
  sweep reaches one more horizon per series before the quick-mode ceiling.
- **Widths:** identical endpoint diameters in 11 of 12 probe runs; 4% wider in
  the sheared order-3 pendulum, where a proposal won one window later.
- **Code size:** all-feature release perf atlas +11,456 B text, including the
  new series.
- **Evidence:** `docs/evidence/2026-11-25-interval-qr-proposal-backoff`.
- **Decision (26 November 2026):** `Backoff { maximum_interval: 16 }` is the
  default under the owner's fluid-API, CAD/CAM/Sim-first guidance; serialized
  options without the field keep `EveryWindow`.
- **Reconsider:** with adaptive intervals or cost-predicted proposals.
