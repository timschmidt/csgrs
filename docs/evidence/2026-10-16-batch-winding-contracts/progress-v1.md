# Strict completion of coincident batch cells

The owning invariant is Hypertri's canonical topology-only triangulation.
The old decreasing single-edge flip rule can stop at two distinct valid local
minima beside a collinear boundary vertex. Hypermesh then receives different
triangles for the same protected cell, preventing consistent bundling of
coincident sheets. This is reproduced at `f74a7848`, with both the original
26-box case and its deletion-minimal five-box diagnostic retained.

The repair protects admissible edges in endpoint-pair order and recovers missing
edges through the existing exact constraint-recovery implementation. It keeps
the inexpensive decreasing-flip preparation, native coordinates, protected
constraints, explicit predicate policy and certainty accounting. Mesh winding
propagation and its contradiction check are unchanged. The public Hypertri
documentation identifies the deterministic-artifact migration for previously
noncanonical outputs; affected stored triangulations should be regenerated.
This is a completeness repair, with no optimization claim.

The independent finite oracle enumerates all three nondegenerate triangulations
of the five-point cell under all 120 vertex orders, three invertible affine maps,
and both predicate policies. Three complete face PSLGs additionally verify
that exterior constraints and constraint order do not change their common cell.
Exact integer areas and positive topology checks accompany triangle comparisons.
The mathematical reasoning and its unproved implementation assumptions are
separately stated in `canonicalization-reasoning-v1.md`.

Native regressions retain the original 26 boxes and the minimized five boxes,
with independent volumes 124 and 46. Both batch and checked merged-shell inputs
run under strict policy, including operand/triangle reordering, reversed winding
and multiple Boolean roots. Closed PWN, exact bounds and integer-coordinate-plane
boundaries make the 125 open unit-cell witnesses a complete set criterion for
each represented box result. Twelve primary variants and two downstream
intersections give 1,750 such membership checks per native test-suite execution.
Self-difference must return empty. The public CSGRS regression exercises merged
shell regularization through empty union, the original binary accumulation,
translation, self-difference and reuse of the resulting empty mesh.

The two pre-repair triangulation failures and the intermediate test-field-name
compilation failure remain retained and classified in the diagnostic record.
Neither the original geometry, strict policy nor validation expectations were
weakened. Existing source in 35 Hypercurve/Hyperdrc files remains part of the
pinned validation context and separately unadmitted; this change does not claim
that a clean checkout of the new commit alone qualifies those files.

The broad frozen qualification completes 50 commands: 49 pass, and the existing
Hypertri fuzz harnesses trigger two fixed-size-chunk Clippy warnings. The
five-command final phase passes after replacing those two iteration expressions
with `as_chunks::<3>().0`; every length, index and distinctness assertion remains.
The before/after hashes identify these as the only source changes after the
broad phase. Production and all other regression sources remain identical.
Both failed lint output and the final passing result are retained.

The broad tests pass 196 Hypertri tests, seven minimal Hypertri tests, 249
Hypermesh tests, 131 minimal CSGRS tests, 287 curve-enabled CSGRS tests, 404
all-feature CSGRS tests, ten FFI tests, and 88 CDT-only tests. Seven native mesh
tests and one all-feature root test remain ignored; they are not credited as
executed. Documentation examples, public documentation, affected Clippy
combinations, both native fuzz-manifest checks, and actual C, C++, Python, Go
and JavaScript/WASM consumers pass. The original four construction benchmark
workloads complete with the same recorded output counts and checksums. Their
single-sample times are 2.2–4.6% higher than the prior retained run; these are
uncontrolled observations, not a matched performance campaign.

All six resource-phase commands and all 64 additional measurement children
pass. With the original baseline, this retains 88 children: 66 timing and 22
allocation rows. Every control's ordered serialized artifact is byte-identical
across both baseline epochs and the candidate. Each timed construction asserts
a Certified receipt and materializes its complete output; the unchanged driver
checks topology, exact volume, witnesses and serialization on a separate
post-timing replay after dropping the timed object. Artifacts therefore describe
that replay, not a serialization of the first timed object. Independent native
regressions supply the stronger set-semantic evidence described above.

Each timing cell below is the median of three fresh child processes, in
microseconds. Warm children perform one complete construction before timing.
The host is not pinned; cold describes process state, not filesystem state.

| Control | State | Original baseline | Replayed baseline | Candidate |
| --- | --- | ---: | ---: | ---: |
| Overlap | Cold | 931.2 | 1079.4 | 1044.6 |
| Overlap | Warm | 695.0 | 913.3 | 720.9 |
| Coincident | Cold | 1013.1 | 995.9 | 1026.6 |
| Coincident | Warm | 819.1 | 802.7 | 867.5 |
| Touching | Cold | 545.4 | 475.7 | 520.2 |
| Touching | Warm | 278.0 | 286.7 | 333.5 |

The unfavorable warm touching result is 20.0% above the original baseline and
16.3% above the contemporaneous replay. Overlap incurs 60 additional allocations
and 40 reallocations per construction; its measured peak requested bytes and
retained delta are unchanged. Coincident and touching allocation counts, peak
requested bytes and retained deltas match both baselines. These small samples
support a focused cost disclosure, not a speedup or universal regression bound.

The newly completed full26 case has cold/warm medians 62.22/61.32 ms and peak
additional requested bytes 1,678,044/1,635,684. Merged26 has medians 66.12/60.52 ms
and peaks 1,523,568/1,493,504 bytes. Its retained deltas are 168,624/126,008 bytes,
versus 42,984/208 for full26. These retained differences are reported rather
than treated as a heap-byte limit. Neither case has a successful batch baseline.
The frozen driver includes initial input construction, including its otherwise
unused cube in these two cases. Raw OS resource reports are retained separately
from the requested Rust allocator counters.

Hypertri's selected library grows by 9,396 bytes; timing/allocation executables
grow by 4,960/4,976 bytes. All selected library and changed source sizes are in
the summary. The serialized outputs contain no computable nodes and have maximum
rational numerator-plus-denominator sizes of eight bits for full26/merged26.
Those are output-serialization measurements, not internal expression or
coefficient-growth peaks.

`qualification-summary-v1.json` links every phase, failure, raw measurement,
artifact comparison, library fingerprint and source size. Final closure checks
the inherited artifact expectations and permits changes only to the raw
`current_work` value in the existing ledger. This record does not replace that
audit or establish the still-open requirements below.

Remaining limitations include inherited end-to-end provenance for bare meshes,
internal total-work/heap-byte/cancellation contracts, clean external distribution
and minimum-compiler validation, full workflow verification, formal proof,
instrumented coverage and directed adversarial campaigns. A finite pair scan
does not bound scalar expression growth or total resource consumption. External
test containment is reported separately from those still-open API contracts.
The complete properties plan remains open.
