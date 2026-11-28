# Changelog

## 0.1.0 (unreleased)

- `FlowNoiseRequest` and `FlowNoiseEvolutionOptions` have a `remainder` field
  (`hypersolve::expression_program::TaylorRemainder`). It chooses the remainder
  form of the order-one endpoint model built over the noise columns.
  - `FlowNoiseEvolutionOptions::default()` uses `Adaptive` (see
    `docs/MIGRATIONS.md`). Across five nonlinear fields it was 2.5–3.5×
    faster from narrow boxes, with widths +0.1–1.4%, and identical to
    `Lagrange` from a wide box, at about 4% extra cost.
  - Set `Lagrange` to reproduce earlier results.
  - `MeanValue` reuses the regularity Jacobian instead of bounding
    second-order coefficients over every column.
  - On the validated-dynamics comparison, `oscillator_long` takes 328 ms
    instead of 701 ms and `riccati_long` 10.1-11.6 ms instead of 16.4-20.1 ms.
    Endpoint widths are identical, or wider by up to 0.14% on the Riccati box.
  - Struct literals of either type need the new field (serialized values
    default it).

- `FlowTaylorProgram::with_coefficients(FlowTaylorCoefficients::Recurrence)`
  encloses each step's trajectory Taylor coefficients with Hypersolve's
  Taylor-mode ODE recurrence instead of natural evaluation of the prepared
  Lie-derivative stage formulas, falling back to those formulas whenever a
  recurrence is unavailable on the step's boxes. `restrict_order` keeps the
  method. `Recurrence` is the default (owner decision, 26 November), so
  programs prepared by the adaptive controller and QR evolution use it as
  well; `Formulas` reproduces earlier exact outputs. On the validated-dynamics
  corpus, nontrivial fields run 11–37% faster with endpoint and tube widths
  equal to about 1e-11 relative; trivial fields (constant, turnaround) run
  3–6% slower. A transcendental quotient field gave a 5% narrower endpoint.
- `QrProposalPolicy::default()` is now `Backoff { maximum_interval: 16 }`
  (owner decision, 26 November). Options serialized without the field still
  read as `EveryWindow`. See `docs/MIGRATIONS.md`.

- `QrEvolutionOptions::proposals` selects a frame-proposal schedule. The
  opt-in `QrProposalPolicy::Backoff { maximum_interval }` runs only the
  incoming chart for doubling runs of windows after it wins, and compares every
  window again after a proposed win (the default since the entry above).
  Windows gain `deferred()` and `choice()`; `comparison()`
  is `None` for a deferred window. Options serialized without the field
  deserialize as `EveryWindow`.

- Add opt-in `flow-sensitivity`: `EvolutionReport::sensitivity` encloses the
  derivative of the flow with respect to the initial state over a certified
  native evolution, at each segment end, across each segment and at the
  horizon. It reuses the source's Picard Jacobians and contraction bounds,
  sums time-ordered Peano–Baker terms as independent interval products with a
  certified geometric tail, and composes segments on the left. Stopped
  prefixes keep their status, and failures publish nothing.
  `FlowTaylorEvolution`, `AdaptiveTaylorEvolution`, `CoordinateEvolution`,
  `QrEvolution` and `FlowNoiseEvolution` gain the same `sensitivity` method.
  Noise runs reuse the Picard proof each correlated segment consumed, which
  starts from the affine state's hull. Chart runs conjugate
  chart-coordinate derivatives by the exact chart and inverse, and QR runs
  compose the exact linear part of each certified frame transition.
  `FlowSensitivity<N, S>` names its source type, and `SensitivityError<F>`
  carries that source's own failure. Read status and reached time through
  `source()`.

- Correlated flow endpoints can fold columns whose certified variation is
  below `2^-column_cutoff_bits` times the largest into residuals, rebuilding
  the noise model with the significant prefix as its capacity.
  `FlowNoiseEvolutionOptions::default()` uses a 16-bit cutoff. Transported
  linear boxes previously kept negligible fresh residual columns up to
  `maximum_noise`, which cost time and exhausted budgets at large capacities.

- Correlated flow endpoints can round their exact center and columns down
  onto the policy grid (`round_coefficients`), adding each enclosed rounding
  error, scaled by its parameter domain, to its row's residual. Single
  `FlowNoiseRequest`s keep it off; `FlowNoiseEvolutionOptions::default()`
  enables it. Exact coefficients previously grew until nonlinear horizons
  such as Van der Pol, Lotka-Volterra and Lorenz stopped after two steps with
  `SizeExhausted`.

- The adaptive Taylor controller prepares each higher candidate order by
  continuing the previous order's retained stages, differentiating only the
  missing normalized Lie coefficients. Prepared programs equal fresh
  `flow_taylor` preparations stage by stage.

- Add opt-in `flow-noise-evolution`: `FlowTaylorProgram::integrate_noise`
  searches bounded native step durations while carrying the complete affine
  outer endpoint into each next search, reusing accepted Picard proofs and
  retaining every attempt, certified prefix and typed failure.

- Add opt-in correlated Taylor flow endpoints with retained affine columns,
  explicit fresh residual inputs, complete trajectory/parameter remainder proofs,
  bounded column selection, checked continuation and replay from original inputs.

- Share prepared trajectory coefficient graphs across lower polynomial orders,
  retaining the complete selected remainder stage and original source guards.

- Add an optional Taylor noise field-image consumer with exact cancellation
  assertions and subsequent use of its explicit Cartesian enclosure.


- Add opt-in automatic finite-window QR evolution with exact frame/transport proofs,
  explicit Picard or trajectory Taylor endpoints, retained attempts and typed failure.

- Add opt-in bounded adaptive trajectory Taylor order/step selection, with exact
  local remainder limits and retained native searches, rejected quality and proofs.

- `EnclosureBudget::graph_budget` exposes a bounded view for domain-adapter
  source validation/compilation and bookkeeping. It shares the existing work
  counter and limits without exposing mutable counter replacement.

- Add optional whole-horizon trajectory Taylor evolution with retained native search history and complete remainders.

- Add opt-in exact QR coordinate construction with orthogonality and reconstruction evidence.

- Add opt-in exact source-bound coordinate transitions for direct chart transport.

- Add optional trajectory Taylor enclosures over certified Picard tubes, with
  source-bound normalized Lie coefficients, complete next-order remainders,
  analytic time/state domain proofs, typed failure and fresh-process replay.

- Add fixed exact state charts and source-bound Picard tube/endpoint transport with explicit final materialization failures.
- Add a nonlinear-field affine Taylor observable example with explicit downstream projection.
- Add optional midpoint-centered field Taylor models, retaining exact source,
  time/state bindings, coefficient/remainder evidence and native images.
  Subdomain reuse preserves the original grid and global remainder. Empty/unit
  products, shared budgets and typed failures remain explicit.

- Add optional finite endpoint covers with explicit infinity-norm epsilon
  thickening, exact initial partitions and retained whole-cell/prefix evidence.
  Outward endpoint transport is checked before continuation and final publication.
  Bounded displacement-based proposals reuse the existing Picard authority;
  the original `integrate` proposal strategy and its reports remain unchanged.
  Stopped requests expose no full cover. Replay rebuilds source/request/policies.

- Add owned, replayable dyadic expression fields with explicit time/state
  dimensions, immutable structural identity and regenerated state Jacobians.
  Reuse native Cartesian carriers and source-domain checks, including empty and
  zero-dimensional products. Shared graph/arithmetic budgets preserve typed
  exhaustion and retry; scalar refinement and payload costs retain their
  documented accounting exclusions.
