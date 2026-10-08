# Parameter interval constructor simplification (V558, read-only audit)

No production changes for this follow-up. V557 qualification owns the current
source tree (exact outer94215). V555 caught a new adapter accidentally using
the unit-only public constructor; V557 uses the established ordered finite
constructor. The underlying carrier and root replay already support exterior
and negative intervals.

The qualification manifest contains 141 explicit constructor call sites, all
inside Hypercurve (25 try_new_ordered, 116 ordinary try_new). The exact locations
are in parameter-interval-constructor-uses-v558.json. This is a call-site
inventory, not a completed semantic audit of every occurrence; Self calls and
indirect wrappers also need review before changing the public contract.

Proposed next implementation, subject to source review and tests:
- Give BezierParameterInterval::try_new one meaning: certify ordered finite
  exact endpoints. Remove try_new_ordered and update its callers directly.
- Keep native unit-domain requirements explicit at their actual operation.
  In particular, BezierParameter2::from_algebraic_root_representation_in_domain
  currently chooses the interval constructor according to unit_domain.
  Preserve the required membership proof there; do not silently remove it.
- Most production ordinary calls are unit Bernstein, fixed half-circle chart
  subdivision, complements of certified brackets, or explicit [0,1] bounds.
  Their unit ownership comes from the chart or operation, not the generic
  interval representation. Audit selected dense projection around
  bezier_offset.rs28595 and all retained chart transports individually.
- Update tests/hypercurve_bezier_algebraic_parameter.rs's obsolete rejection
  of negative intervals. Add public exterior/negative exact isolator examples
  while keeping reversed-range and native curve-domain rejection checks.
- No compatibility alias or alternate constructor should survive this migration.

An independent optimization candidate V556 retains one strictly certified
source-tangent factorization across distances and source cells. It is unrun and
requires V557 committed/sealed; its priority depends on V557 timing evidence.
The broader generic stationary point-constraint/ray-barrier audits remain open.

V563 continuation notes (read-only, production frozen):
- BezierParameterRange2::try_new already admits ordered exterior/algebraic
  ranges; its old unit-domain type comment is inaccurate. The underlying
  general parameter/isolator types likewise do not carry a unit-domain
  invariant, because retained corner extensions already use them.
- The exact importer name is from_algebraic_root_representation_with_domain
  (not _in_domain as the initial inventory says). It is the known caller that
  still depends on ordinary interval construction to validate unit bounds;
  preserve that explicit check before constructing its generic interval.
- The ordinary constructor's Self forwarding call, from_monotone_span and
  native point/location admission need review along with the 141 inventory
  entries. Do not treat the textual inventory as a semantic audit.
- Prioritize primitive tangent factorization reuse after the current
  finite-field proof/identity change is qualified and committed. Its source
  owner is distance-independent, strict success is reusable across policy
  calls, and opposite one-sided normals must remain distinct. V564 is a
  prepared copied-source probe, not an executed or approved production patch.
