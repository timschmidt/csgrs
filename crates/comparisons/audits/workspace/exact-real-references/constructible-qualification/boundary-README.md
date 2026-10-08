# Constructible initial native boundaries — 2026-09-06

Main source audit remains fourfiles/490physical lines, pinned
46d760cbd2d21f955ec96c8fe2c13fdf3b2dd9d0. The original donor and all downloaded
dependency sources are unchanged. This is initial boundary qualification,
not completion of the native test/benchmark or Hyper transfer audit.

## Acquired dependencies and reproducible build

Pinned public Cabal archives, with regular-file/directory-only member lists
checked before extraction:

- [binary-search0.0](https://hackage.haskell.org/package/binary-search-0.0/binary-search-0.0.tar.gz),
  SHA256a8a50515e504693597110034fa66abc5af954fcd8f80742ad6e0f4cb020abd4d.
  This deliberately selects the original scalar search API permitted by the
  donor's >=0.0dependency, not an assertion that0.0is latest.
- [complex-generic0.1.1.1](https://hackage.haskell.org/package/complex-generic-0.1.1.1/complex-generic-0.1.1.1.tar.gz),
  SHA2561f535c9ab52930cfae7665b659713214af81ab6ffdfddb42c540bad8522a8b0f.
- [integer-roots1.0.4.0](https://hackage.haskell.org/package/integer-roots-1.0.4.0/integer-roots-1.0.4.0.tar.gz),
  SHA256a50c8287fe5f84a66bc196864e23cfc4bb9ecd10c7d664383c0c00e8f1896526.

GHC9.6.7 directly compiles the original modules with three dependency include
directories, `-hide-package integer-gmp` to select the manifest's modern
ghc-bignum backend, `CCACHE_DISABLE=1`, and workspace build/TMPDIR
`.audit-constructible-build.l4UDoe`. The cached GHC is under
`/tmp/aern2-stack.ryVCFK/programs/x86_64-linux/ghc-tinfo6-9.6.7/bin/ghc`.
O0andO2 use`-fforce-recomp -rtsopts -odir ... -hidir ...`, with explicit
separate output binaries. No source compatibility patch was required.

This is direct GHC module compilation, not a successful unmodified Cabal
solver configuration: complex-generic's archived manifest restricts base<4.10
and template-haskell<2.12, older than this compiler. The warning about tabs
in binary-search is preserved. Initial dependency compilation failed at
ccache's read-only cache; the approved retry with caching disabled passed.
The initial missing-dependency and ccache logs remain available.

GNUtar comparison reported expected archive-owner UID/GID differences after
unprivileged extraction. `verify-dependencies.mjs` checks tar header checksums
and every regular member's exact bytes against the frozen archive, independently
of filesystem ownership. That check does not grant source-reading credit.

Selected dependency read coverage is5files/466lines in
`DEPENDENCY_READ_COVERAGE.tsv`; other compiled dependency modules have no
full-read credit. Numeric.Search.Integer's125lines were read completely,
including exponential expansion and binary search: `search p` finds the least
integer satisfying an upward-closed predicate. Its discrete-log documentation
example uses a downward-closed predicate and is not used by the donor.
The214line TH module derives ordinary complex instances; its final example
helper is commented out. Neither helper suggests a production Hyper change.

## Native results

`BoundaryProbe.hs` is frozenSHA256
373bc9aaecbf219e3ecc7f266e709da023a4c69985064ed5d37c0c578a5cec03.
O0binarySHA256c7b553011b21752028816c05cd6e66da30908b3f42e720892313db5da190850c;
O2binarySHA25658068f545126e8266557167be7934a58520bb621817494d4d6483f9be05cea8c.
Both terminalexit2,99of109checks pass,10fail, noexception/cap. Failures:

- Five negative irrational properFraction cases,−sqrt(n),n=2,3,5,7,10.
  Donorlines388–391 subtract1from the least integer strictly greater thanx,
  hence choose floor(x), not truncation towardzero, for irrational values.
  The resulting nonzero fractional part is positive for these negative inputs.
  The [Haskell2010contract](https://www.haskell.org/onlinereport/haskell2010/haskellch6.html#x13-1390006.4.6)
  requires the fractional part to have the input's sign. Positive irrational
  and signed rational controls pass. Exact reconstruction alone is insufficient.
- Five finite-view cases: sqrt(2*10^(2k))/10^k at k=154,155,160,200,500.
  Every exact comparison with sqrt2passes, but `fromConstruct :: Double`
  is not finite in the known interval(1,2). k=0and100controls pass. This
  concerns the floating view, not corruption of the exact field object.
  Source221–244 avoids opposite-sign addition by conjugation, but does not
  generally scale intermediate factors to prevent overflow/underflow.

Other controls:41exact-square cases,40field-joining identities and two signed
nested-radical identities pass. This is not a full property suite, and no
performance inference follows from these short correctness probes.

## Hyper comparison

The external `hyper_boundary.rs` uses only public APIs. It initially used
privatefold and was corrected before executing the controls; both compile
failure logs are retained. The seven exact128bit checks construct the same
expression through public Computable operations, independently of Real's
public conversion/identity checks.

Bothdebug and release pass all31checks:10signed radical truncation/fraction
cases,7exact scaled-root identities,7finiteDouble views with directed4096bit
MPFR error bounds, and7Computable approximations checked against the same
directed reference. Their process exits are0, no60secondcaps.

SourceSHA256b9959e008eb0b408b8eaf4375981fee540e12da6d3a56332198b37993c9d81a8;
debugbinarya0e9fa70cd90628a2fc1575ff38844c9eda1848849e46fa9b43e562a5801d0c3;
releasebinary30740644b5194c18d88f1ab12001d8eb3abe8ef6e2c223881cbb19d0617d5836.
The existing numbers-qualification Cargo harness is reused only to avoid
duplicating its pinned Rug/Hyper dependencies. The frozen outputs are here.

These boundaries already work in Hyper; no new production change is warranted
from them. Next: broader original-module tests, field-join and nested-radical
benchmarks, and compare the donor's norm/square-membership algorithms with
Hyper's existing algebraic certificates. No benchmark, whole-donor completion,
or arbitrary computable-real decidability claim is made.
