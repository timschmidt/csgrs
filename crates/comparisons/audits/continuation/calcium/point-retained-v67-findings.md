# Checkpoint 67 — demand-gated point-witness repair retained

Retained in live Hypersolve as the sixth Calcium/FLINT continuation transfer.
The full exact-real ecosystem audit is still incomplete.

Only `hypersolve/src/algebraic_binary.rs` changed. Its contents exactly match
the previously qualified 175-file isolated solver candidate; the other 955
recorded live source/support identities are unchanged. The live file was clean
in Git before this integration. Existing resultant/root-isolation edits and the
monic test file were preserved. No API, dependency, scalar layout or cache-field
change, and no donor code was copied into production.

## Why this version is worthwhile

The original binary-image constructor can produce a collapsed interval without
its required exact-point witness. The repair waits for the refiner's typed
InvalidInterval result, proves endpoint equality under STRICT, and retries the
unchanged refiner with the witness. Approximate multiplication/division requires
strict reconstruction of the whole image as well. Polynomial vanishing,
containment, uniqueness and half-open ownership obligations remain intact;
approximate equality and genuine Unknown are not promoted to exact facts.
The rejected report is dropped before retrying.

Earlier independently checked public corpora recover 825 answers per policy
while preserving all other 5,616 records. Repeated policies/histories are not
new independent defect counts. Unlike the unretained eager repair, demand avoids
ordinary-path endpoint probes and removes its unchanged-result allocation/peak
penalties in the measured native corpus. The chosen WASM Unknown control is
about 24–25% faster than eager across four diagnostic passes.

Costs are accepted, not hidden: recovered answers do additional work, retries
allocate and can be slower, and no universal speedup/memory bound is established.
The two qualified stripped Hypercurve examples grow 1,568/1,536 bytes versus
baseline and 432/384 versus eager. Their BSS grows 2,528/2,568 versus baseline;
build-path/linker-layout effects contribute. The production diff has 52 net
algorithm lines and 374 test/helper lines, including eight regression tests.
Completeness with unchanged proof obligations takes priority over those costs.

## Live qualification

The exact source promotion was prepared only after checkpoint 66's final
verification completed at 2026-09-11T03:22:18.866Z. Live gates run from
03:26:15.670 to 03:28:55.616 UTC and all pass:

- All-feature debug and release solver tests: 811 each, eight suites, zero
  failed/ignored. Exact test names/outcomes match the isolated candidate.
- Clippy: all targets/all features with warnings denied; formatting check.
- All-feature release WASM library build (compile-only).
- Full candidate/live dependency metadata: 139 packages and nodes, identical
  after only the expected source-path normalizations; one copy of each of the
  four relevant Hyper crates and an unchanged lockfile.

The separate retention evidence check passes at 03:30:15.722 UTC. Compiler/tool
versions match the consumer qualification. Hypercurve's 1,764 passed/nine ignored
release tests, numerical oracles, allocation campaigns, cold/history execution
and corrected native/WASM timings remain evidence from their frozen qualified
trees; they were not rerun on live paths or relabelled as new runs.

## Historical/current distinction and resources

`point-retained-origin-v67.json` binds both the 956-file pre-retention map and
the one-file-changed current map, plus the complete isolated solver and consumer
bindings. `retainedSources()` rechecks the actual frozen baseline, candidate,
consumer and live files. Earlier verification scripts that assert an older live
map are historical checks; they are not valid current-state entry points after
this retention. Use, from this audit directory:

```sh
node verify-point-retained-v67.mjs --point-live
```

The new verifier preserves earlier captures/manifests and checks their hashes,
not a fresh execution of the full historical 47-chain or 60–65 statistics.
Future audit tools must use the new retained source binding for current identity
instead of treating the older `point-qualified-sources` live map as current.

No new dedicated /tmp artifact or source-tree copy was needed for live adoption;
the shared two-job, nonincremental Cargo cache was reused and its growth is
separate from checkpoint 66's four preserved example binaries. Recorded /tmp
availability after live gates is 15,381,413,888 bytes. Nothing was deleted,
committed or pushed. Donor coverage is unchanged: 1,415 complete files, twenty
partial and 183,653 uniquely read lines. Continue the remaining source/reference
audit, separate power-sum work and full inventory reconciliation.
