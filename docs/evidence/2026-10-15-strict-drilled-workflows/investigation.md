# Drilled-solid investigation

The original 16-segment README fixture remains unchanged. Baseline-v1 records
strict undecided and qualified approximate completion at 16 and 32 segments;
8 and 24 segments remain certified. These are diagnostic outcomes, not a
completed semantic acceptance test. Exact volume comparison remains unresolved.

Diagnostic-v1 preserves a failed isolated build: Hyperphysics still referred to
the original Hypermesh type. Diagnostic-v2 rebuilds that dependency consistently
and reproduces the same program outcomes. The first unresolved scalar occurs
while testing coplanarity of adjacent triangles in the drill, before Boolean
selection. Production native sources were not instrumented.

Scalar-baseline-v1 reproduces the reversed-factor cancellation gap. Two new
controls incorrectly requested positive precision exponent 512; that run is
retained as a harness defect. Scalar-baseline-v2 corrects those arguments to
-512, preserving the intended 512-bit bound and every mathematical assertion.
No production implementation has changed between these baselines.

The executed validate_v1.py inherited section-specific metadata from its previous
use. validate_v2.py corrects the contract metadata; all earlier logs and runner
bytes remain preserved. The fixture and outcomes, not inherited metadata, define
the scope of the earlier diagnostic evidence.

Scalar-baseline-v2 passes all three controls and fails both reversed-product
identity tests. The exact ordered-product identity is already satisfied.
The independent bounded expansion in expand_diagnostic_v1.py checks the
captured 16-segment scalar using rational ring arithmetic and opaque atoms;
it makes no assumptions about trigonometric identities.
