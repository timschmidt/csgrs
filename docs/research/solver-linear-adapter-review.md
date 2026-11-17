# Solver linear adapter ownership review

At `15cf09783`, all 445 lines and 25 declarations in
`crates/hypersolve/src/linalg.rs` were reviewed for plan section 7.1. The file
contains dense `f64` numerical proposals and their diagnostic receipts. It has
no interval carrier or outward enclosure arithmetic to move to Hyperinterval
or Hyperlattice. The partial inventory now records every declaration, including
trait signatures and their implementations separately.

The numerical implementation and helpers remain category 4 solver adapters;
errors, precision labels and pivot reports are category 5 diagnostics. Generic
names such as `dot`, `norm2` and `mat_vec` do not establish interval semantics.
A future shared proposal kernel needs its own admission evidence. This review
does not justify replacing primitive calculations with exact interval work or
changing their order.

Normal equations form `JᵀJ + damping I` and `−Jᵀr`, then use partial-pivot
Gauss–Jordan elimination. Dogleg first solves the undamped normal equations,
then selects a Gauss–Newton, Cauchy or interpolated boundary step. BFGS applies
the supplied inverse Hessian and clips the direction. Singular tests use
`f64::EPSILON`; nonpositive trust radii and step limits are floored at that
value. Pivot counts are numerical metadata, not symbolic rank proofs.

A standalone probe compiles a byte-identical copy of the complete module with
Rust 1.95, warnings denied. Nine controls pass: independently solved diagonal
and damped systems, dimension failures, empty and zero-width systems, singular
and tiny pivots, both direct dogleg branches, an independently checked
segment/sphere intersection, BFGS clipping, and current nonpositive-limit
behavior. The last case characterizes existing behavior; it does not approve
the input-validation contract. The probe also passes strict Clippy.

Three missing P6 acceptance witnesses fail on this revision:

| Input and method | Observed result |
| --- | --- |
| Normal equations: `J=[[1e308]]`, `r=[1e308]`, damping `0` | `Ok([NaN], report)` with infinite pivot extrema |
| BFGS: inverse Hessian `[[1e308]]`, gradient `[2]`, limit `1` | `Ok([NaN], report)` |
| Normal equations: `J=[[NaN]]`, `r=[1]`, damping `0` | `Ok([NaN], report)` with NaN pivot extrema |

The first two use only finite inputs. Each missing witness requires a typed
failure instead of successful non-finite proposal data. They are explicitly
run in a separate failing invocation; none is counted as a passing acceptance
test. No production repair or repository test change is retained here.

The public backend is consumed by `solver.rs`; its reports are retained by
`diagnostics::SolveReport`. The solver labels proposal precision as lossy and
uses exact replay separately for proof. Its coordinate-import failure path
currently retains the previous variable value. That source observation is an
additional P6 review concern, not a reproduced end-to-end solver failure in this
corpus. Approximation labeling does not make successful NaN output acceptable.

The file has no policy argument, Hyperreal/Hyperlimit calls, serialization,
resource budget or cancellation mechanism. Its `Vec` allocations are visible
in source; allocation, stack and runtime costs were not measured. Existing
`certification` benches cover Levenberg–Marquardt, modified Newton, dogleg,
Powell hybrid, BFGS and SQP proposal routes, but were not rerun for this
source-unchanged audit. No whole-owner, cross-platform, performance, exact
closure or release qualification follows from the standalone probe.

The archive record and digest are registered in `docs/evidence-archive.tsv` as
`docs/evidence/2026-11-17-interval-linear-adapter-review`. It retains the exact
module, source identity, harness, commands, passing controls, failing witnesses,
caller snapshots and reviewed dispositions. Production source and all 57 size
accounts remain unchanged. The full 15-phase goal remains active.

## Isolated finite-arithmetic repair

Candidate `0caf282ad` adds distinct non-finite input/arithmetic errors and
preserves them through new unit convergence-reason variants. It also checks
norm overflow before clipping can publish an incorrect zero direction. The
three original failures reproduce again at `a32805da2`; all twelve standalone
cases and four new owner tests pass with the candidate. Strict all-feature,
all-target owner Clippy and workspace formatting pass. A fixture initially
used a nonexistent constructor; its failed compile log and correction to the
existing `Expr::real` constructor are preserved.

The candidate is rejected on the unchanged stack gate. An external caller of
the actual Hypersolve crate exercises sixteen unchanged cases in 768 fresh
processes. All 384 paired complete outputs and 128 allocation/drop pairs agree.
Across cold and explicit-warm groups, normal and singular cases reduce their
measured maxima by 16 bytes, but dogleg adds 16 bytes, BFGS adds 48 bytes and a
zero-width normal system adds 24 bytes. Empty dogleg/BFGS paths newly cross the
measurement window; their baseline zero readings are censored and must not be
reported as zero actual stack. The ordinary increases independently reject
retention. No timing or final-size admission was attempted afterward.

The baseline is restored clean, and its rebuilt external artifact is
byte-identical to the original. Six existing solver controls, strict owner
Clippy and formatting pass. The candidate remains under
`refs/interval-unaccepted/linalg-finite-20261117`; no repair, new test or proposed
API migration is adopted by the primary branch. The non-finite failures remain
open. No full-owner test suite, stable/WASM qualification, universal resource
bound or completed-goal claim follows from this experiment.

The immutable record is
`docs/evidence/2026-11-17-interval-linear-finite-rejection`, registered in
`docs/evidence-archive.tsv`. It includes the rejected patch, test logs, both
resource probes, all campaign records and restoration evidence. The inventory
remains at 1,565 rows and all 57 size accounts are unchanged.
