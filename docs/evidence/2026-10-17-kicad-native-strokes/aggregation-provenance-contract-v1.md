# Retained source diagnostics at aggregation

Source inspection of candidate v3 found that `PcbRegion::new_shared` clears
the `had_non_finite_input` history. Boolean operations intentionally propagate
that history to `layer_sanity`, even when their resulting exact geometry has a
valid finite report. Board aggregation must preserve the same diagnostic.
Reproduce this through public constructors, intersection, board aggregation
and the public sanity check before changing the helper. This is a source
history claim, separate from certification of the resulting exact set.

Use an exact circle promoted from finite binary64 inputs whose report samples
overflow, intersect it with an analytically contained unit square, and require
the existing source diagnostic before and after both aggregation entry points.
No unsafe/private-field mutation or fabricated diagnostic is used. Preserve
all prior aggregation tests. Shared metadata/history retention belongs in the
native profile helper, not in board-specific callers.
