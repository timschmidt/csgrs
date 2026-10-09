# Conic source and local parameter frames

A reversed retained quarter-circle returned parameter `1/3` for the line
`y = 3/5`, although its actual intersection is at parameter `2/3`. Substitution
at the returned parameter gives `(5/13, 12/13)`, which is off the line. The
[original-path replay](../original-circle-contact-oracle-replay/decision.json)
reproduces this with the original contact-direction implementation.

The inverse helper selected either cached source controls or current local
controls, then always applied the source-to-local range map. For current
controls, this repeated reversal or trimming. The repair carries the selected
frame's chart identity with its controls and converts parameters only for a
source frame. Shared-point transport and primary/fallback conic parameter maps
use the same distinction. Reading the controls and identity together also
prevents concurrent frame publication from changing a query's interpretation.
The cache's lifetime, admission, retention, and policy rules are unchanged.

The [independent Fraction oracle](oracle.py) derives twelve line intersections
from `C(t) = ((1-t²)/(1+t²), 2t/(1+t²))` and checks their points and crossing
signs. The Rust regression exercises 2,304 contact queries plus 192 shared-point
transports and 192 primary parameter maps: both policies, zero and `(π,e)`
translations, positive and negative common weight scales, three source ranges,
both orientations, and cold/warm source and target frames as applicable.

## Isolated validation

Only the two repaired source files differ from production baseline
`4d1e32fe69c0eb5b71b9949f4b3129e961007d20`. The other pending native-stroke
experiments are absent. Sources were hash-checked before and after each
command; Cargo invocations were serialized. See [the manifest](manifest.json),
[commands and results](status.json), and [public consumer results](public-consumers-status.json).

- Formatting and release Clippy for all HyperCurve targets with `-D warnings` pass.
- 217 curve tests pass: 56 rational Bézier unit tests, 39 region Boolean tests,
  and 122 public consumer tests.
- The unchanged full-board smoke test passes with both original fixture ZIPs:
  12 CPArti findings and 201 HVP109A findings. Process CPU time is 56.33 seconds
  user plus 0.82 seconds system; wall time is 57.42 seconds. Its existing
  120-second worker-time assertion passes. No profiler was attached.

These are correctness checks and one runtime gate, not a speedup claim,
allocation comparison, complete report-identity proof, exhaustive cache audit,
or completion of a property. All remaining plan and crate future work stays
open.

## Reproduction and retained failures

Use a clean checkout of the baseline commit and overlay the two files in
`source-delta.tar.gz`; retain the baseline's nested Rustfmt configuration and
both tracked fixture ZIPs. Run the commands in `status.json` and
`public-consumers-status.json` from that checkout, substituting its generated
test executable for the recorded absolute path. The scripts retain the exact
local workspace preparation, target directory, and hash checks used here.
`python3 oracle.py` regenerates `oracle.json` without calling Hyper code.

The initial extraction omitted nested Rustfmt configuration; `preparation-*`
records the resulting format failure; the large raw log is preserved byte-for-byte
in `preparation-format.log.gz`. The next attempt missed the
`parameter_images.rs` consumer of the changed helper; its compile failure and
source snapshot are in `attempts/incomplete-frame-consumers/`. Neither failed
attempt ran a board benchmark. Both failures were fixed before the successful
run, without weakening tests or the runtime assertion.
