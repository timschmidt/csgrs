# Hidden SVG path admission (2026-11-10)

Hypercurve owns SVG validation and visibility selection under P6, P18 and
P19. The CSGRS adapter carries the typed outcome under P25. Standard hidden
paths use the existing finite SVG command grammar. Exact metadata uses the
existing native decoder and its version, byte, point-definition and replay
limits. The region, wire and path outputs for valid hidden inputs are empty.

On `30ea60023a`, four malformed standard descriptions returned successful
empty geometry under three hidden styles and both predicate policies. The
matched visible descriptions returned `MalformedInput`. A complete baseline
probe recorded 24 invalid-as-empty outcomes and 40 controls. The controls
include a valid visible square, a visible noncircular elliptical arc with a
typed representation blocker, valid hidden counterparts, and the legacy
handling of empty and move-only data. The first two probes incorrectly
expected visible empty and move-only data to succeed. Their failed receipts
are preserved. The visible parser explicitly requires a drawable subpath.

The first owner acceptance run against the same committed library failed the
hidden grammar and native metadata tests. Its valid hidden control passed.
The adapter acceptance run failed because a hidden path without `d` returned
empty geometry.

The correction reuses the visible grammar iterator for hidden standard
commands. It scans the whole input, so a malformed suffix after an otherwise
valid unsupported arc still reports malformed syntax. Hidden native paths
use the existing exact metadata decoder. The extension supplies the path
even when the display projection cannot be parsed.
Visibility selection then yields the canonical empty result.

The acceptance witnesses cover typed malformed, unsupported and configured
size-limit outcomes, plus certified empty under both predicate policies. The
adapter exports and reloads the empty result and consumes it in a subsequent
Boolean identity. Visible standard line, arc and quadratic controls compare
serialized output, exact replay, certainty and predicate dispatch against the
parent. The native decoding limits govern the named resources in `SvgOptions`.
General SVG fidelity and total computation and memory bounds remain open.

The affected all-features suites passed 222 test reports in 25 harnesses.
Formatting, strict Clippy for Hypercurve and CSGRS, the 17 capability runner
tests and six selected catalog witnesses passed. The first dispatch comparison
mixed two Hyperreal feature configurations and reported four different
cache-hit counts. The matched configuration produced byte-identical output,
certainty, dispatch and rational counters in all six visible controls.
