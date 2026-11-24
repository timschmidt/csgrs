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
