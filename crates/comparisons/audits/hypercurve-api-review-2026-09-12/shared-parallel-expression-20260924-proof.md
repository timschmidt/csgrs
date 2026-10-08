# Shared recursive positive-speed expression replay

Parent: 9be44c2eec926e71dc2ea0b140e768e9542008f2.

The chord and selected-radial systems independently implemented sign replay
for A+B*sqrt(S), and separately formed A^2-B^2*S. The chord system also had
two identical native expression evaluators under different method names.
This change moves the positive-speed sign procedure and magnitude polynomial
onto the existing expression value, migrates both systems, and removes the
identical chord evaluator. No representation, cache or compatibility entry
point is added. Production shrinks by 82 lines; every existing test is unchanged.

The generic sign procedure requires strict S>0 before component signs or
magnitude comparison. The chord consumer first strictly requires W!=0 and
applies its source-weight orientation after expression replay. This aligns its
admission with the native and selected-radial evaluators. Joined-field root
replay remains owned by the chord's callback; radial queries retain their
existing parameter authority and weight guard. Native evaluation and global
projection remain their existing specializations, and source-only chord
projection still avoids an unnecessary square.

Seventeen focused cases cover the previous exact tangent-scaling regression,
positive-speed cancellation/opposite-sheet cases, poles and stationary frames,
independent chord coefficient fields and weight gauges, dense norm sheet
replay, finite monotonicity, all cubic crossings, and corner composition.
The physical V1 snapshot binds all 2,044 inputs; the full qualification keeps
the parent's 1,470 cases and unchanged limits. Work is not yet qualified.

V1 formatting, both all-target Clippy feature configurations and all seventeen
focused release tests pass. All source and executable bindings match; every
focused process is reaped. The broader 1,470-case qualification is now live,
with source bytes frozen and all prior timeout limits unchanged.

V1 completed and committed as 9945504d571c15316b9fbca22aa689a63b6dddb2. All 1,470 cases were attempted:
1,460 pass, six remain ignored, and the same four cases time out at unchanged
75-second limits. All 225 public integration tests pass. The isolated long
compositions take 36.92, 145.93 and 59.80 seconds within their unchanged
75/180/75 limits. Qualification preserves every prior passing case and binds
all 2,044 inputs, executables, staged bytes and committed bytes. All owned
processes are reaped, all thirty repositories are clean, and nothing was pushed.
The full implementation goal remains active.
