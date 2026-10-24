# Release

This is the release procedure for the crates in this repository, and the
state of the current release. It applies [P37](../properties.md) and the
shipped-user-path rules in [`admission.md`](admission.md#the-shipped-user-path).

## Current release

| Crate | Version | On crates.io | Notes |
| --- | --- | --- | --- |
| `hyperreal` | 0.14.0 | 0.13.0 | breaking |
| `hyperlattice` | 0.7.0 | 0.6.0 | breaking |
| `hyperlimit` | 0.5.0 | 0.4.0 | breaking |
| `hypertri` | 0.5.0 | 0.4.0 | breaking |
| `hypersolve` | 0.4.0 | 0.3.0 | breaking |
| `hypercurve` | 0.4.0 | 0.3.0 | breaking |
| `hypermesh` | 0.1.0 | — | first release |
| `hyperphysics` | 0.3.0 | 0.2.1 | breaking |
| `hypersdf` | 0.2.0 | — | first release; blocked, see below |
| `csgrs` | 0.23.0 | 0.20.1 | breaking; blocked through `hypersdf` |
| `csgrs-ffi` | 0.1.0 | — | first release; blocked through `csgrs` |
| `hypergraphics` | 0.1.0 | — | first release |
| `hyperevolution` | 0.3.0 | 0.2.0 | breaking |
| `hyperpack` | 0.3.0 | 0.2.0 | breaking |
| `hyperparts` | 0.3.0 | — | first release |
| `hyperpath` | 0.3.0 | 0.2.1 | breaking |
| `hyperdrc` | 0.3.0 | — | first release; blocked through `csgrs` |
| `hypercircuit` | 0.3.0 | 0.2.0 | breaking; blocked through `csgrs` |

`hypervoxel`, `hyperbrep`, `hyperbrep-io` and `hyperspatial` are not part of
this release. Breaking bumps follow `cargo-semver-checks` against the latest
crates.io version; each crate's `CHANGELOG.md` lists the changes.

### Blocker: `hypervoxel` and `voxelis`

crates.io requires every dependency, including optional ones, to be
published. `hypersdf` has an optional `hypervoxel` dependency (its
`hypervoxel-adapter` feature), so `hypersdf` 0.2.0 cannot be published until
`hypervoxel` 0.3.0 is. In turn, `hypervoxel` has an optional dependency on its
vendored `voxelis` 25.5.0 (`legacy-voxelis`), while crates.io has `voxelis`
25.4.0 from its upstream authors. Through `csgrs`'s optional `hypersdf`
dependency, the chain also holds back `csgrs`, `csgrs-ffi`, `hyperdrc` and
`hypercircuit`.

The crates that do not depend on `hypersdf` can be published now:
`hyperreal`, `hyperlattice`, `hyperlimit`, `hypertri`, `hypersolve`,
`hypercurve`, `hypermesh`, `hyperphysics`, `hypergraphics`, `hyperevolution`,
`hyperpack`, `hyperparts` and `hyperpath`.

### Known limitations

Every repository example completes under `STRICT`, and every result in the
[successive-operations corpus](successive-operations.md) stays exact,
certified and closed, with bounded coordinate size and a per-step cost that
grows more slowly than the result. These limits remain, each tracked where it
will be fixed:

- **Decidable domain.** `STRICT` decides exact zeros over rationals,
  `sin`/`cos` of rational multiples of pi, and sqrt(2), sqrt(3), sqrt(6),
  within the cyclotomic proof's budgets. Other transcendental or radical
  inputs can end `PredicateUndecided`, a typed outcome. Beyond those budgets
  the bounded ring recognizers decide identities in independent atoms and a
  few built-in relations, among them `sin^2 + cos^2 = 1` for each angle,
  up to degree 12. Booleans of near-coincident rotations composed from two
  such angles, such as binary64 rotations by 13.1 and 17.3 degrees and back,
  can still meet a zero above those bounds
  ([hypermesh future work](../crates/hypermesh/FUTURE_WORK.md#near-coincident-rotations-beyond-the-cyclotomic-domain)). A sweep along a
  general sampled path has exact frames in nested square roots and does not
  compose with a later strict Boolean ([properties.md](../properties.md)
  target 15).
- **Twisted side walls.** `curve::extrude_twisted` checks that its result is
  a closed manifold, not that it is embedded. A thin profile can get side
  walls that cross: the strip `[1, 1 + 2^-20] x [0, 2]` with a 90 degree
  twist in three slices and end scale `(1/2, 2/3)` gives a closed mesh with
  negative signed volume.
- **Opposite revolution loops.** A partial `curve::revolve` turn revolves
  loops on opposite sides of the axis separately, through opposite angular
  ranges. Beyond a half turn, where their mirror images overlap, the result
  is a closed mesh with winding number two there, not an embedded solid. A
  full turn revolves the profile's fold instead and is embedded.
- **Hull rows.** A second strict hull of the same mesh object in one process
  can triangulate coplanar hull faces differently (same set, certified),
  depending on facts cached by the first hull
  ([hypermesh future work](../crates/hypermesh/FUTURE_WORK.md#convex-hulls-of-sampled-spheres)).

## Checks

Run these after the last change, in addition to the
[pre-push verification](../AGENTS.md):

1. `scripts/release-check.sh` packages every released crate and verifies that
   each package builds from its own contents. It then builds a project outside
   the repository that depends on the released versions, resolved only from
   the unpacked packages, and runs:
   - the csgrs user path (`scripts/release-consumer/user_path.rs`): construct,
     transform, compose under `STRICT` with certified certainty, validate the
     closed manifold and exact volume, export STL, OBJ and PLY, reimport STL
     and OBJ, and check each format's counts and round trip;
   - every README quick start, extracted from its `quickstart` markers;
   - C and C++ programs (`scripts/release-consumer/ffi_smoke.c`,
     `bindings_smoke.cpp`) linked against the packaged `csgrs-ffi` static
     library and its shipped header, a Python program through
     `bindings/python`, the Go binding tests in `bindings/go`, and a strict
     `tsc` check of `bindings/js/csgrs.ts`. Go and the TypeScript compiler
     must be on `PATH`; without `tsc`, the script runs TypeScript 5.7.3
     through `npx`.

   Before unpacking, it also builds each crate's documentation with its
   docs.rs configuration and `-D warnings`. Add `--toolchain 1.95.0` to repeat
   everything on the minimum supported compiler. The `release` job of
   `.github/workflows/rust.yml` runs it on stable and on 1.95.0.
2. `cargo semver-checks check-release -p <crate>` for each crate already on
   crates.io. It needs a release that reads rustdoc JSON v60 or newer
   (0.51.0 or later).
3. `cargo deny --all-features check advisories`. Known: `ttf-parser`
   (`csgrs` `truetype-text`) and `paste` (through `image`) are unmaintained
   with no upgrade; `cgmath` and `smartstring` are development-only.
4. `cargo hack check --locked --each-feature -p <crate>` builds every feature
   of every released crate on its own and with no features.

Until a version is on crates.io, the consumer resolves it from the unpacked
`.crate` files. Those are the files `cargo publish` uploads, but publication
itself, and the crates.io and docs.rs builds, stay unverified until it happens.

## Publishing

Publish in dependency order, waiting for each crate to appear in the index:

1. `hyperreal`, `hyperlattice`, `hyperlimit`, `hypertri`, `hypersolve`,
   `hypercurve`, `hypermesh`, `hyperphysics`
2. `hypergraphics`, `hyperevolution`, `hyperpack`, `hyperparts`, `hyperpath`
3. after `hypervoxel` 0.3.0 is on crates.io: `hypersdf`, `csgrs`,
   `csgrs-ffi`, `hyperdrc`, `hypercircuit`

`cargo publish --locked -p <crate>` publishes one crate. Cargo 1.90 and later
can publish several in order with repeated `-p` flags. After each step, check
the crates.io README rendering and the docs.rs build, then tag the release
commit as `<crate>-v<version>`.
