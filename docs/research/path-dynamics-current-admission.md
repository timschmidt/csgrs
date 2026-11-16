# Path dynamics: current admission and retirement of pending prototypes

The Phase 11 extraction remains an outstanding implementation requirement.
The unaccepted production migration has been removed from active CSGRS and
Interface worktrees under the owner's explicit authorization to manage,
commit or discard pending changes. Its complete source remains recoverable.
The size exception does not waive the reproduced runtime failures.

## Reproduction and alternative

Current source baseline: `e835617c530fd4f5009d4e6c5bc7af9666b0ecf7`.
The exact 41-path pending extraction and eight-path Interface patch were saved
and verified before changes. Rust 1.95, the repository's locked bench profile,
and the same absolute source path reproduce both original owner executables
byte-for-byte against the earlier layout review. Later solver additions do not
change these artifacts. No unchanged timing or cache-model campaign was repeated.

The shared-error-only alternative was previously stopped on size before matched
runtime qualification. It aliases the dynamics error under the compatibility
name and removes sixteen identity conversions, avoiding report-copy work. This
review applies that exact archived patch, with no inline/outline hints or changes
to arithmetic, predicates, policies, fixtures or workload bodies. Its executable
also reproduces the earlier artifact exactly. All 614 current owner tests and
doctests pass, including fresh-process full reports and dispatch counts under
both policies.

| Final owner artifact | File bytes | `.text` bytes | Defined function-range union |
| --- | ---: | ---: | ---: |
| Baseline | 13,312,600 | 8,118,103 | 8,044,187 |
| Original extraction | 13,304,584 | 8,110,279 | 8,036,358 |
| Shared error | 13,316,848 | 8,117,175 | 8,043,152 |

Function ranges merge aliases/overlaps; six unsized symbols remain disclosed.
These are complete owner-artifact measurements, not a universal downstream cost
bound. Data and BSS sizes are unchanged. The shared-error variant adds 4,248 file
bytes while reducing text and the defined function union.

Before measuring runtime, this review proposed a scoped benchmark-file allowance
of 8,192 bytes AND 0.1%, both incremental and cumulative against the existing fixed
program baseline. This permits two pages rather than one increment while reducing
the default cumulative allowance and percentages. All other artifact and code
caps remain unchanged. **That proposal is not adopted or consumed.** Its purpose
was to qualify an evidence-backed boundary change, not excuse a timing failure.
The existing cumulative account is preserved.

## New matched runtime evidence

Two blocks use ABBA and BAAB order, CPU 0 affinity, ASLR disabled, one executable
path and working directory, a busy warmup, then one-second Criterion warmup,
two-second measurement and 30 samples. No build, test or profile ran during these
measurements. Six cases were selected from the previously published failures
before obtaining current results. There are 48 case runs, 1,440 samples and
45,034,785 measured completed callbacks. This is not the full 194-case campaign.

Ratios below are shared-error mean divided by baseline mean. Adverse separation
requires both candidate lower confidence bounds above both baseline upper bounds
within the block. It is a screening rule, not a family-wise confidence claim or
an allowed regression tolerance.

| Case | ABBA ratio | BAAB ratio | Adverse separation in both |
| --- | ---: | ---: | --- |
| Axis-aligned offset | 0.98278 | 0.98024 | no |
| Quadratic hodograph | 1.00799 | 1.00781 | no; first block only |
| Rectangular-pad clearance | 1.01567 | 1.01366 | yes |
| Additive-bead schedule | 0.99167 | 0.99330 | no |
| 256-record parser | 1.02558 | 1.02810 | yes |
| Exact trace import | 1.02476 | 1.02776 | yes |

The three confirmed failures reject the variant independently of file size.
Favorable cases and the hodograph nonconfirmation remain in the raw record.
No unique hardware cause is established; no arbitrary layout retry or unrelated
parser/mathematical optimization follows. Broader timing, downstream native/WASM,
whole-call stack and complete retention accounting are not claimed for the
rejected candidate.

## Recoverable snapshots and restoration

Both repositories retain `refs/interval-unaccepted/path-dynamics-20261116`:

- CSGRS snapshot `f8dd6e1b251e9377f4010268f1ab00824df77a58` contains all 41 pending paths.
- Interface snapshot `3efb35a56278202270d873fbaa56c7995d8d8807` contains all eight pending paths.

These are explicitly unaccepted snapshots outside active release history.
The evidence also contains both exact patches and the combined shared-error
variant. Existing hard cases and failed measurements are not erased.
The independent Interface canonical-digest assertion is separated for its own
qualification; it does not require adopting the production migration.

After restoration, all 599 Hyperpath/Hyperdynamics tests and doctests pass across
25 suites, with no failures or ignored cases. Strict all-feature/all-target
Clippy and workspace formatting pass. The full plan remains active: archiving an
unaccepted extraction does not satisfy its domain-neutral ownership requirement.
No push, publication, full release, browser, HIL or firmware-fit claim is made.


## Independently retained canonical witness

Interface commit `32a59de0d69436427e5b97297c4929fd908bf326` retains only the existing diagonal G1 profile test's
additional 41,795-byte canonical transcript assertion and its original digest
`df893df83e90e6b53452e18586ffcaa2524a021631a395d4a6523be16263669c`.
All prior independent semantic assertions remain. This test does not adopt any
production extraction or replace geometry correctness with a checksum.

All 201 core tests/doctests and 199 configured-release tests pass, as do strict
core Clippy and workspace formatting. Fixed CSGRS c9f1f9ac/current e835617 with
Interface f5341f3 and firmware 2d68e82 yield byte-identical baseline executables.
The candidate final file grows 12,912,864 -> 12,913,296 bytes (+432, 0.00335%);
complete .text grows 9,562,322 -> 9,562,546 bytes (+224, 0.00234%). Both absolute
and percentage caps pass. The stricter code cap applies to all final .text;
there is no named-symbol claim for this stripped artifact. A symbol-preserving
companion changed generated code and is excluded from acceptance measurements.

The new test account leaves 15,952 bytes of cumulative file allowance and 3,872
bytes of cumulative complete-text allowance, subject also to their remaining
percentage caps. All 43 earlier artifact accounts and the fixed program baseline
remain unchanged. No cross-target savings offset this cost.
