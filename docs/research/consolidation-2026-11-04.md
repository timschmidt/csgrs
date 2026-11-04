# Research integration, 4 November 2026

The working branch advanced from `f5f8f0fb6f` to `a08c698757` by fast-forward.
This brings the simplification/modeling API implementation and the intervening
closure, cancellation and coefficient-growth work into `closure-integration`.
The prior memory-priority correction is retained because it agrees with P36
and the governing ladder. No remote push or publication is included.

## Branch dispositions

The [86-branch register](local-branch-dispositions-2026-11-04.tsv) records every
local branch observed before cleanup, its full tip and disposition. Of these,
74 unused branch references were retired atomically after creating local tags
`research-archive/2026-11-04/<original-branch>`. All their commits remain
reachable. `main`, the integration branch and ten checked-out worktree branches
remain. The archive tags are local and have not been pushed.

The ancestry check alone was insufficient: ten apparent divergent tips map
through `docs/history-rewrite-2026-10-20.tsv` to ancestors of the integrated
revision. Their original evidence and decisions remain in
`docs/properties-status.json`: flat aggregation and rational equality facts
were retained; the superlinear report variant and exact-track variants were
rejected on their original resource gates. The seven October experiment
worktrees contain uncommitted source/tests and samples. Their patches and
untracked files were preserved in the evidence record; the worktrees remain
untouched. They are historical controls, not additional admitted candidates.

The nonancestral curve checkpoints are superseded by the qualified integration:

| Earlier checkpoint | Integrated work |
| --- | --- |
| `abddc05cda` | selected point/field replay and joins: `579cf9ad02`, `97fb12dbbe`, `4f018c0620`, `f3a7e76390` |
| `7b5b61e158` | selected-field root discovery: `fbd75f3139` |
| `4d8b5098a4` | selected-query fibers: `e2ba33065a` |
| `83047f4b0b` | integrated derivatives and original guards: `57a523fdfd` |
| `26916a3c95` | coefficient roots/replay: `146cc0804f` |
| `09a149fb19` | selected circle incidence: `c079baaddd` |
| `d3645365f8` | polynomial chamfer incidence: `4cd115dc46` |
| `9d313498bf`, `eb7ab943b1` | qualified region chamfer/probe continuation: `70cccec462` |
| `6296155cf7` | polynomial SVG replay: `bc826075df` |
| `d31ef199c3` | repeated selected chamfers: `1333a09d16` |
| `f809c8b250`, `d781351ff1`, `b0a950dcfd`, `b6ec1de05f` | zero displacement, multiplicity, empty topology and tensor witnesses: `6ee27becee`, `f140324764`, `2bae38d76f`, `f5f8f0fb6f` |

These are dispositions of earlier variants, not claims of byte-identical
commits. Review of changed paths showed later owner corrections and expanded
qualification in the integration. Reapplying earlier checkpoint trees would
discard those changes. The K06 benchmark file is byte-identical between its
standalone branch and `88bdf6e160`; the later helper migration is already here.
The July direct-predicate adapter (`1784b8d550`) changes the removed
`src/mesh/polygon/triangulation.rs`; that variant is architecture-inapplicable
to the current native Hypermesh carrier.

## Empty expression ancestry

Owner: CSGRS expression lowering (`src/solid_expr.rs`), P9/P18/P19/P25.
Domain: valid mesh expressions, including qualified canonical empty operands,
under both explicit policies. An empty leaf was replaced by truth constant
`false` before its construction evidence was recorded. This violated the
documented promise that an expression inherits every leaf's qualification.

The new regression constructs an approximate empty intersection from touching
cubes using the existing terminal-zero fixture. It reproduces lost ancestry
on `a08c698757`. Lowering now records construction evidence before the empty
shortcut. No predicate order, geometry or local certainty changes. The test
checks an empty-only expression, mixed Boolean operations in both operand
positions, shared empty leaves and subsequent strict translation/union. Local
decisions remain certified while inherited approximation stays queryable.

Validation after the final Rust edit:

- `cargo test --offline --release --features dispatch-trace`: 675 passed
  (including four doctests), seven ignored, zero failures. This includes the
  strict/approximate session chains, empty composition, construction evidence,
  receipt composition, downstream profiles/solids and unchanged arrangement
  count assertions.
- The five focused modeling suites: 47 passed, one timing test ignored.
- `cargo clippy --offline -p csgrs --all-targets --features dispatch-trace -- -D warnings`
  and the corresponding `--no-default-features` command: passed.
- `RUSTDOCFLAGS='-D warnings' cargo doc --offline -p csgrs --no-deps --features dispatch-trace`:
  passed. Workspace format and whitespace checks passed.
- All 86 recorded reference dispositions were checked against Git; each of
  the 74 retired tips resolves to its exact archived tag.

Builds used separate `/tmp` Cargo targets to avoid exhausting the nearly full
workspace filesystem. No performance or binary-size improvement is claimed.
The integrated HyperDRC tokenizer recheck (`cargo test --offline --release
-p hyperdrc sexp --lib`) also passed all seven selected tests. The original
failing regression and initial formatting failure remain beside the passing logs.

Evidence: local evidence branch `research-consolidation-2026-11-04`, commit
`c6bf7692d80cce9f3212a965f1b8c48ad638b4ca`, record
`docs/evidence/2026-11-04-research-consolidation` (48 files, 992,163 bytes),
SHA-256 `9ecf03b87cbf15e1174e0b5fff363c66a3eca8bc5f6ffb884647dbb164957a5a`.
The archive is retained in the existing local bare evidence repository and
has not been pushed. A tree comparison with its parent verifies exactly 48
additions under the new record and no changed or deleted prior archive files.

## Open work

Package dispositions in the simplification plan are historical evidence, not
a fresh universal completion claim. Read-only checks of the workspace sibling
repositories confirm D01/D02/D04 still have uncommitted work. openscad-rs is
clean at `c0d92e6`; the Manifold comparison remains deferred. U06's proposed
expression-node attribution for errors is not implemented by the current
lowering API. The general curve/BRep closure plans, remaining research queue,
full pre-push gates and installed-distribution validation remain open. The
next research changes must reproduce their witnesses on this integrated tree;
none of these open requirements is closed by the branch cleanup.
