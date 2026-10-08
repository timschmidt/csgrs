# Workspace reference optimization audit

This is the cross-repository ledger for the active paper-to-implementation
optimization goal. A paper is complete only when its primary source has been
read, every plausible stack target has been considered, viable experiments
have been measured, exactness/correctness gates pass, and each idea is marked
kept, rejected, already satisfied, or architecture-inapplicable.

## Inventory marker

- 55 level-two reference/research headings in 52 Markdown files across 26 repositories.
- Repository heading counts: alumina-firmware 1; alumina-interface 1; csgrs 2; hyperbrep 1;
  hypercircuit 1; hypercurve 1; hyperdrc 3; hyperevolution 1; hypergraphics 1;
  hyperlattice 1; hyperlimit 1; hypermesh 1; hyperpack 2; hyperparts 1;
  hyperpath 1; hyperphysics 1; hyperreal 1; hypersdf 1; hypersolve 1; hypertri 1;
  hypervoxel 10; openscad-rs 1; synaps-cad 1; t-deck-async-drivers-rs 17;
  tools 1; videos 1.
- Hardware datasheets, standards, API documentation, books, papers, and
  implementation-lineage links remain distinct evidence classes; only actual
  papers count as papers, but every reference still receives an applicability
  disposition.
- Deduplication is complete. Repeated citations share one algorithm audit, while
  each repository's audit records the local applicability decision. Nested
  HyperVoxel citations and the video-only Simulation of Simplicity citation are
  called out explicitly below.

## Paper audit ledger

| Source | Status | Cross-stack findings and evidence |
| --- | --- | --- |
| Yap, “Towards Exact Geometric Computation” (1997) | Read completely; implementation audit complete | Detailed section-by-section evidence and accepted/rejected experiments are in `YAP_PROGRESS.md`. Prepared exact-rational filters produced large repeated-query wins; common-denominator one-shot expansions were rejected where they regressed compact rational kernels. |
| Farouki and Rajan, “Algorithms for Polynomials in Bernstein Form” (1988) | Accessible-primary-source and current-stack application audit complete; original article text remains unavailable | Kept in Hypercurve: reuse invariant Sturm sequences during algebraic refinement (`7c30edc`, focused ordering 99.257 to 64.005 us, -35.5%); short-circuit opposite endpoint derivative signs in Bernstein form before conversion/Sturm (`5807c55`, 11.895 to 5.155 us, -56.7%); use the scaled conversion identity `a_k = C(n,k) sum_i (-1)^(k-i) C(k,i) b_i` with a retained Pascal triangle (`575741d`, degree-32 conversion 114.425 to 88.911 us, -22.3%); pair the equal-weight `X'W` and `XW'` terms in rational derivative-numerator construction (`c89ce95`, degree-12 cold monotonicity 100.828 to 70.814 us, -29.8%); fall back from the small-integer binomial fast path to exact `Real` binomials, allowing degree-40 rational monotonicity certification beyond the former `u64` ceiling; and retain only one in-place de Casteljau row while extracting both subdivision diagonals (`c89ce95`, algebraic graph overlap 396.1 to 389.7 us, -1.6%, temporary subdivision storage from O(n^2) to O(n)). Rejected: an O(n)-memory rolling conversion row was 4.9% slower; precomputed small binomial rows were 1.4% slower; routing cubic area moments through the generic scaled converter was 2.7% slower; hard-coded area conversion moved only about 0.7%, below the retention threshold. Existing quartic/quintic products and elevation are already direct degree-specialized Bernstein formulas. General rational degree elevation retains every intermediate exact result. GCD, remainder, Sturm, substitution, and resultant consumers operate on a single cached power-form construction; duplicating them in Bernstein form would add a synchronized representation without avoiding repeated conversions, while exact arithmetic removes the paper's floating-point conditioning rationale. The publisher abstract and author-hosted Bernstein overview were audited, and the author publication list confirms the citation but offers no article link; IBM/Elsevier expose no lawful complete copy found by the primary-source search, so this row does not claim the original 1988 text was read completely. |
| Akenine-Moller, “Fast 3D Triangle-Box Overlap Testing” (2001/2005 course reprint) | Read completely; current-stack audit complete | Kept in HyperVoxel: exact 13-axis SAT in doubled cell coordinates, with prepared AABB certificate reuse (`f0fc38f`). Surface voxelization median 57.191 to 32.356 ms (-43.4%); prepared solid 157.02 to 64.927 ms (-58.7%); unprepared solid 402.89 to 341.10 ms (-15.3%). Specialized two-projection formulas were statistically neutral and rejected. The paper's edge-axis-first order was neutral/slightly regressive and rejected. OBB inverse-transform application is architecture-inapplicable until an oriented-box carrier exists; no OBB implementation is present in the workspace. Degenerate-normal robustness is already handled by exact source certification. |
| Kampe, Sintorn, and Assarsson, “High Resolution Sparse Voxel DAGs” (2013) | Read completely; current-stack audit complete | Kept in HyperVoxel (`8553220`): canonical finest-depth sparse imports are Morton-sorted and reduced bottom-up instead of retaining sequential path-copy history. Compaction median 87.170 to 67.108 us (-23.0% across the bucketed and adjacent-range stages); retained nodes 139 to 39 (-71.9%) on the fixed 64-cell workload. A second retained change (`ab1dec0`) replaces per-node public aggregate packets with exact sufficient integer statistics, compact material summaries, and one materialized root packet: contiguous node stride fell 208 to 80 bytes (-61.5%), path-copy edits 75.056 to 46.942 us (-37.5%), storage/report construction 79.675 to 46.793 us (-41.3%), and sparse compaction 68.000 to 54.620 us (-19.7%). A direct allocation-free material-summary union was slower (up to +7.7%) and was rejected. Voxelis interning now resolves equal 64-bit fingerprints by full leaf/branch identity and a full-period collision probe, including empty-sentinel collisions, deserialization, removal, and cluster repair (`88e4530`); it adds no per-node storage, is neutral on random batch updates (14.979 versus 14.961 us), costs about +1.6% on random single updates, and costs about +9% in a 25–30 ns fill/recycle microbenchmark. Corrected uniform-leaf collapse retains parent logical depth and invalid equal-branch level collapse is prohibited. Dynamic edits retain online hash-consing/path-copy semantics. Sub-DAG construction is naturally realized by recursive bottom-up ranges; parallel sort/scan is not justified at current benchmark sizes. Pointerless 4^3 leaves apply only to binary geometry and conflict with rich exact `VoxelCell` payloads. The paper's compact consecutive-child pointer layout conflicts with recyclable dynamic nodes, while HyperVoxel already uses 32-bit child IDs; mixed layouts would require a second synchronized traversal representation whose cost has no current consumer. Beam tracing, approximate solid-angle LOD, and GPU traversal are rendering architecture concerns absent from the current exact semantic SVO API. |
| Edelsbrunner and Mücke, “Simulation of Simplicity” (1990) | Read completely; cross-stack audit complete | The paper's ordered infinitesimal perturbation scans an exact determinant and then a lexicographically ordered sequence of signed subdeterminants until a nonzero coefficient supplies a consistent tie. HyperLimit intentionally returns exact zero and rich boundary/degeneracy classifications; HyperCurve and HyperMesh retain contacts and overlaps; HyperTri preserves collinear vertices where required and represents D-dimensional cospherical cells as a complex. Replacing those zeros with artificial signs would discard public evidence. The paper itself warns that perturbation can lose original boundary points and requires postprocessing. HyperTri's opt-in BRIO schedule already permits a different valid 2D cocircular diagonal without claiming canonical perturbation. No SoS runtime was added: there is no current API that both requires a globally indexed general-position view and permits the changed topology. The existing exact filters and fixed determinant kernels already address the paper's dominant arithmetic cost on nondegenerate inputs. |
| Teschner et al., “Optimized Spatial Hashing for Collision Detection of Deformable Objects” (2003) | Read; HyperDRC transfer audit complete | HyperDRC's regular-grid candidate indexes matched the paper's spatial-hash target but used ordered maps internally. Replacing private `BTreeMap` buckets with `HashMap` while retaining explicit candidate sorting and deduplication improved the fixed 10,003-drill rebuild median from 1605.697 ms to 1261.417 ms (-21.4%). Determinism is exercised across 32 rebuilds. Custom multiplicative hashing, k-d replacement, and distance-update variants were rejected or inapplicable as recorded in `hyperdrc/PERFORMANCE.md`. |
| Tinney and Walker, “Direct Solutions of Sparse Network Equations by Optimally Ordered Triangular Factorization” (1967) | Read; HyperSolve implementation audit complete | Kept as an opt-in symmetric minimum-degree exact Bareiss solve. The report retains both permutation directions, the complete permuted factorization report, source-order solution recovery, and a fresh exact replay of the original sparse system. On the fixed 32-by-32 arrowhead sentinel, authored order took 4.901 ms and minimum degree took 0.790 ms (-83.9%). On an already well-ordered tridiagonal sentinel, the same path regressed 315.46 to 388.12 us (+23.0%), so the authored-order API remains unchanged and callers choose the fill-reducing schedule when their structure warrants its analysis cost. |

## Repository audit coverage

Every algorithm-bearing Hyper repository now has a source-by-source disposition
and retained/rejected measurement record. These files are the detailed ledger;
the repeated papers in their README files are not silently collapsed here.

| Repository | Detailed evidence |
| --- | --- |
| Core scalar, algebra, predicate, triangulation, topology, and curve crates | `hyperreal/PERFORMANCE.md`, `hyperlattice/PERFORMANCE.md`, `hyperlimit/PERFORMANCE.md`, `hypertri/PERFORMANCE.md`, `hypermesh/PERFORMANCE.md`, `hypercurve/PERFORMANCE.md` |
| CSG consumer | `csgrs/PERFORMANCE.md` |
| Other Hyper geometry and domain crates | `hyperbrep/PERFORMANCE.md`, `hypercircuit/PERFORMANCE.md`, `hyperdrc/PERFORMANCE.md`, `hyperevolution/PERFORMANCE.md`, `hypergraphics/PERFORMANCE.md`, `hyperpack/PERFORMANCE.md`, `hyperparts/PERFORMANCE.md`, `hyperpath/PERFORMANCE.md`, `hyperphysics/PERFORMANCE.md`, `hypersdf/PERFORMANCE.md`, `hypersolve/PERFORMANCE.md` |
| Voxel stack and nested crates | `hypervoxel/PERFORMANCE.md`; its nested-crate table covers Akenine-Möller, Guigue–Devillers, Kämpe et al., Laine–Karras, Lysenko, Bevy, and OBJ individually. The legacy floating SAT prototype was both 3.43x slower and occupancy-changing, so it was removed. |
| Video project | Shewchuk and Yap map to the core predicate/scalar audits. The video-only Edelsbrunner–Mücke paper is disposed above; Manim is an authoring API rather than a computational kernel. |

## Focused core-stack disposition

Every README reference in the seven requested repositories has a normalized
code-path disposition in its local performance audit. The retained outcomes are:

| Repository | Retained paper-driven result |
| --- | --- |
| HyperReal | Exact-rational inverse-trig range reduction around the shared `atan(2/3)` identity improved the measured interval sweep by 24–41%, alongside the earlier scalar representation and summation changes. Expansion arithmetic, extreme-precision pi algorithms, and matrix decompositions have explicit cross-crate or measurement triggers rather than open local work. |
| HyperLattice | Dense exact-rational 4x4 determinant dispatch improved 6.4%; the attempted zero-mask multiplication schedule regressed sparse rows and was removed. |
| HyperLimit | Reusing point/ring orientations improved report replay 20.4%; reusing triangle/plane vertex sides improved non-coplanar and coplanar triangle reports 21.6–25.8%. Early Farkas-certificate scheduling regressed 10.7% and was removed. |
| HyperTri | BRIO-style opt-in insertion improved located batches 14.3–25.2%; orientation reuse and removal of immediately repeated builder proofs improved key Delaunay rows 28.4–47.0%. Mutable adjacency, TDS cavity stitching, and new sweep/monotone APIs are architecture changes, not unattempted local substitutions. |
| HyperMesh | Cached BVH keys improved the hull sentinel 44.7%; bounded leaf caches and triangle-side reuse improved Boolean hot paths; a large-leaf exact trace BVH regressed 42.7% and was removed. New certified build-once/extract-many arrangements reduce four overlapping-cube operations from 34.710 ms to 9.421 ms (72.9%). |
| HyperCurve | Shared de Casteljau weights improved split/flatten/prefix rows 7.6–9.6%; exact-rational polynomial evaluation improved representative rows up to 36.6%. A retained conservative x-interval event sweep improves large sparse direct/prepared intersection batches 88.9–99.1% while dense batches fall back to the flat schedule. |
| CSGRS | Direct graphics-buffer streaming improved its sentinel 88.1%; indexed export preallocation improved the all-exporters row 6.9%. Reusable smoothing buffers were output-identical but neutral/slightly slower and were removed. |

For HyperMesh, direct and reusable-arrangement results match exactly for union,
intersection, difference, and symmetric difference on both overlapping and
coincident inputs. Its full post-change matrix passed 952 unit tests, 54 core
tests, 48 regressions (one benchmark smoke ignored), doctests, formatting,
all-target/all-feature checking, strict Clippy, and strict rustdoc.

## Non-paper reference sections

The remaining headings contain standards, hardware datasheets, API manuals,
language grammars, file formats, or implementation-lineage links rather than
research papers:

- `alumina-firmware` references ESP/Rust tooling and the PCF8575 datasheet;
  `alumina-interface` references UI/rendering dependencies, the Web Storage
  standard, and the already-audited CSGRS implementation rather than research
  algorithms.
- `openscad-rs` and `synaps-cad` reference language compatibility corpora,
  parser/UI APIs, and 3MF/STL/OBJ interchange specifications.
- `t-deck-async-drivers-rs` and its 16 nested headings reference component
  datasheets, bus/radio standards, Embassy/embedded-hal APIs, and board examples.
  Their register, timing, and protocol requirements constrain correctness; they
  do not propose host-side algorithms to benchmark across the Hyper stack.
- `tools/hyper-callgraph` references syntax and graph serialization formats.

These sources were classified rather than treated as optimization papers. No
speculative runtime changes were made to projects with no algorithmic paper.

## Experiment policy

- Baselines and candidates use release/bench profiles and existing end-to-end
  workloads where available.
- A local median movement without statistical support is not sufficient to
  retain extra complexity.
- Exact boundary contact, degeneracy behavior, uncertainty propagation, full
  relevant tests, formatting, and strict Clippy are correctness gates.
- Concurrent edits are excluded from paper commits unless independently in
  scope and verified.

## Final validation state (2026-07-15)

- HyperReal, HyperLattice, HyperLimit, HyperTri, HyperMesh, HyperCurve, CSGRS,
  and both HyperVoxel packages passed their ordinary library, integration, and
  example tests with all relevant features. All-target/all-feature compilation
  also passed for the focused stack.
- The broader HyperBreP, HyperCircuit, HyperDRC, HyperEvolution, HyperGraphics,
  HyperPack, HyperParts, HyperPath, HyperPhysics, HyperSDF, and HyperSolve
  library/integration regression pass completed without a failure.
- HyperSolve's retained minimum-degree addition subsequently passed its complete
  all-target/all-feature test and benchmark-smoke matrix, no-default checking,
  formatting, strict Clippy, and strict rustdoc. That pass also repaired a stale
  benchmark fixture that indexed a second distinct root of `(x - 1)^2`; the
  comparison now uses valid represented roots for `sqrt(2)` and `sqrt(3)`.
- Every retained optimization has a dedicated release-profile benchmark in its
  repository evidence file. The final aggregate validation did not replay every
  fixed-loop benchmark executable to completion after their tests and sentinel
  workloads passed; those long measurements were already run for the individual
  retention decisions recorded above.
- `git diff --check` passes across all 19 Hyper/CSGRS repositories. One
  HyperDRC rustdoc-link validation previously reached GNU `ld` host-resource
  exhaustion (SIGBUS); its library, integration, lint, check, and documentation
  builds otherwise pass, so this is recorded as an environment limitation rather
  than a product failure.
