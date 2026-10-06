# HyperPhysics reference and performance audit

This audit maps every README reference to the code that owns the corresponding
physical or numerical idea. A reference is marked as an implemented exact
kernel, an explicit adapter contract, a delegated responsibility, or a deferred
solver. Optimization candidates were retained only after release measurements,
semantic tests, and exact dispatch tracing.

## Retained results

Times are medians from three sequential executions of the same optimized
benchmark binary on the audit machine. The small harness uses fixed exact inputs
and `black_box`; differential property tests separately cover generated inputs.

| Kernel | Before | After | Change |
| --- | ---: | ---: | ---: |
| exact triangle point report | 3.669 us | 1.287 us | 64.9% faster |
| property resolution plus elastic derivation | 1.199 us | 1.113 us | 7.2% faster |
| velocity-first Euler step replay | 1.804 us | 1.746 us | 3.2% faster |
| composed lumped-RC thermal step | 1.229 us | 1.191 us | 3.1% faster |

The triangle report formerly built its normal, plane numerator, and oriented
edge signs for evidence, then immediately asked HyperLimit to reconstruct the
same predicates for the coarse classification. It now derives the enum from its
retained certified signs. Generated integer triangles and query points are
differentially checked against `hyperlimit::classify_point_triangle3`.

Property resolution now collects provenance, exact agreement/conflict,
interval, and proposal state in one pass instead of allocating temporary
matching and exact-value vectors. Elastic derivation moves the owned resolved
values rather than cloning them.

The mechanics step shares one exact `dt / mass` quotient across the three force
components and passes the already-proved positive mass into diagnostics. The
thermal RC report similarly passes its already-proved positive time step into
the nested energy-balance report. Public standalone diagnostics and transient
reports still perform their own validation.

The benchmark suite now includes a 256-triangle mass-properties sentinel so
face-accumulation changes are not judged only on a four-triangle tetrahedron.

## Reference disposition

### Bender and Koschier: divergence-free SPH

The [DFSPH paper](https://animation.rwth-aachen.de/media/papers/2015-SCA-DFSPH.pdf)
uses separate pressure solves for density and velocity-divergence error. The
crate records `FluidPolicy::Dfsph`, exact particles/boundaries, and exact
mass/momentum conservation, but does not pretend that these setup carriers are
a DFSPH solver. Neighbor search, kernels, pressure iterations, density and
divergence residuals, and adaptive stopping criteria remain an external solver
proposal that must return replayable diagnostics.

### Beer: absorption

[Beer's 1852 paper](https://doi.org/10.1002/andp.18521620505) motivates the
exact optical-depth carrier `alpha * thickness`. A slab report retains that
compact fact and now exposes on-demand exact-real evaluation of
`exp(-optical_depth)`, avoiding transcendental construction when a caller only
needs depth or process metadata.

### Carslaw and Jaeger: heat conduction

The steady slab, contact-resistance, transient energy-balance, and lumped RC
reports cover the closed-form one-dimensional cases used by the crate. General
initial/boundary-value heat equations, eigenfunction expansions, multidimensional
domains, and moving interfaces remain FEM/FVM or external adapter work.

### Fick: diffusion

[Fick's original paper](https://doi.org/10.1002/andp.18551700105) is represented
by exact species diffusion coefficients and the explicit-grid diagnostic
`D dt / h^2`. HyperPhysics deliberately reports this stability quantity without
claiming to advance a reaction-diffusion field; flux discretization, boundary
conditions, reactions, and nonlinear coupling belong to a certified solver.

### Fourier: analytical heat theory

[Fourier's treatise](https://www.cambridge.org/core/books/analytical-theory-of-heat/F6D4802336FABD1116DDA4AA3FE6EFAA)
supports the exact `q = k A DeltaT / L` kernel and the separation between compact
source/boundary facts and field evolution. Spectral heat solutions are deferred
because the crate currently owns report carriers, not a spatial heat solver.

### Gilbert, Johnson, and Keerthi: convex distance

The [GJK paper](https://doi.org/10.1109/56.2083) motivates exact support-map
reports and the explicit zero-direction witness. Only axis-aligned boxes are
currently certified convex support shapes; their contact and distance questions
have simpler specialized interval solutions. A general GJK simplex iteration is
deferred until more convex shape families expose exact support maps and the
termination decision can report certified progress or uncertainty.

### Ihmsen et al.: implicit incompressible SPH

The [IISPH paper](https://cg.informatik.uni-freiburg.de/publications/2013_TVCG_IISPH.pdf)
derives a pressure system from a symmetric pressure force and discretized
continuity equation. `FluidPolicy::Iisph` names that adapter boundary. The crate
retains exact setup and conservation facts; pressure matrices, neighbor caches,
iterations, density-error tolerances, and time integration remain solver-owned.

### Jacobs: stereolithography working curve

The vat-photopolymer report implements Jacobs' working curve
`C_d = D_p ln(E / E_c)` with exact-real logarithms, retains concentrations and
exposure policy, and certifies the layer-depth margin without a primitive-float
threshold. The model is kept distinct from conversion kinetics, oxygen
inhibition, shrinkage, and post-cure mechanics because the working-curve
assumptions do not determine those quantities.

### Lambert: photometry

[Lambert's *Photometria*](https://www.deutsche-digitale-bibliothek.de/item/7IXRYTA4IBPQ6UMQWYLPVXMV4IGJQE3Q)
underlies the attenuation/photometric boundary but not a full radiometric
transport solver. Exact optical depth, normal interface classification, and
normal-incidence reports are implemented; angular radiance transport,
scattering, polarization, and detector response remain explicit adapters.

### Landau and Lifshitz: elasticity

The property graph implements the isotropic shear relation
`G = E / (2(1 + nu))`, records both sources and the isotropic assumption, and
rejects a non-positive denominator. `PropertyTensor` carries anisotropic data.
A full constitutive report must separately certify the three-dimensional
stability range, including the bulk-modulus limit `nu < 1/2`; that broader check
is not silently imposed on a function that returns only shear modulus.

### Marsden and West: variational integrators

The [variational-integrator review](https://doi.org/10.1017/S096249290100006X)
revealed that the existing mechanics update was mislabeled: it advances
velocity first and uses `v_(n+1)` for position, so it is symplectic Euler rather
than explicit Euler. `symplectic_euler_replay` and
`IntegrationPolicy::SymplecticEulerReplay` now name the actual scheme. The old
entry point remains behavior- and report-compatible. Higher-order variational
schemes, discrete constraints, and Noether diagnostics remain deferred.

### Maxwell: electromagnetic field theory

The [Royal Society manuscript](https://makingscience.royalsociety.org/items/pt_72_7/paper-a-dynamical-theory-of-the-electromagnetic-field-by-j-james-clerk-maxwell)
maps to exact field/material/boundary carriers and local linear isotropic
constitutive replay `D = epsilon E`, `J = sigma E`. Maxwell curl evolution,
charge/flux conservation, waves, and energy balance require spatial
discretization and remain FDTD/FEM/BEM/MoM adapter responsibilities.

### Mirtich: polyhedral mass properties

[Mirtich's publication record](https://people.eecs.berkeley.edu/~jfc/mirtich/publications.html)
motivates exact oriented volume, first moments, second moments, center of mass,
and inertia. HyperPhysics uses oriented origin-tetrahedron integrals. It already
accumulates common-denominator numerators once per mesh and factors the
second-moment polynomial through the vertex sum, preserving exact orientation
and parallel-axis certificates.

### Monaghan: SPH

[Monaghan's review](https://doi.org/10.1146/annurev.aa.30.090192.002551)
maps to particle mass, position, velocity, smoothing length, material, boundary,
and conservation carriers plus `FluidPolicy::Sph`. Kernel choice, consistency,
artificial viscosity, neighbors, density estimates, and evolution are not
invented by this adapter layer.

### Stewart and Trinkle: nonsmooth contact stepping

The [impulse-momentum time-stepping paper](https://www.cse.lehigh.edu/~trink/Papers/STicra00.pdf)
motivates the `Complementarity` policy, exact contact material domains, exact
broad-phase classification, and residual replay through HyperSolve. The crate
does not yet assemble or solve the simultaneous-contact complementarity system;
friction cones, impulses, active contact sets, and convergence certificates are
deferred together rather than approximated piecemeal.

### Stratton: electromagnetic boundary-value theory

Stratton's boundary-value treatment maps to explicit electromagnetic regimes,
field regions, material assumptions, and boundary-condition roles. The current
implementation stops at setup and local constitutive reports; waveguide modes,
Green functions, radiation conditions, and frequency-domain field solutions
remain solver adapters.

### Yap: exact geometric computation

[Yap's EGC paper](https://doi.org/10.1016/0925-7721(95)00040-2) governs every
decision boundary: exact object facts and certified signs decide contact,
geometry, physical domains, and acceptance. Unknown results stay typed errors
or statuses. The `dispatch-trace` feature forwards tracing through HyperReal,
HyperLattice, HyperLimit, and HyperMesh; its regression test proves the audited
triangle and mechanics paths request no approximation and produce no unknown
fact event.

## Rejected experiments

- Directly dividing the first-moment numerator by four times the signed-volume
  numerator was algebraically correct but regressed tetrahedral mass properties
  from 11.091 to 11.335 us (2.2%). The staged exact expressions were restored.
- Flattening the six symmetric second-moment accumulators changed the
  256-triangle median from 615.1 to 611.1 us (0.65%), inside noise, while making
  the tensor index mapping less direct. The matrix-shaped accumulator was
  restored.
- Combining AABB relation and overlap calculation locally matched HyperLimit in
  generated tests but regressed 856 to 908 ns (6.1%). HyperLimit's interval
  classifier and short-circuit path were restored; the differential tests stay.
- Enforcing the complete isotropic bulk-stability range inside the shear-only
  derivation raised the combined property sentinel to 1.459 us and exceeded the
  function's stated responsibility. Full constitutive-tensor stability remains
  a separate future report.

## Validation protocol

Run from the crate root:

```sh
cargo fmt --all -- --check
cargo test --all-targets --locked
cargo test --features dispatch-trace --test dispatch_trace --locked
cargo clippy --all-targets --locked -- -D warnings
cargo check --all-targets --locked
RUSTDOCFLAGS="-D warnings" cargo doc --no-deps --locked
cargo bench --bench mass_properties --locked
```

The default suite checks exact values, invalid domains, inward orientation,
zero-volume and zero-extent cases, provenance/conflict semantics, and generated
geometry. The feature-gated trace test checks that representative optimized
paths remain on exact/certified dispatch routes.
