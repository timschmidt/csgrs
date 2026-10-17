# Typed KiCad track-load failure contract

This refines the proposed failure carrier in [contract-v1.md](contract-v1.md)
before implementation. Base revision, source domain, original workload gates,
retained regressions, resource containment and full-plan limitations remain.

The public `CopperFeature.location` is an unconditional coordinate pair. A
failed-region placeholder would therefore introduce an apparently valid zero
coordinate when source coordinates cannot be parsed. Track loading will use
the existing fallible `load_kicad_pcb` boundary instead: return a structured
track error through its existing error chain before publishing a BoardModel.
The caller can inspect the segment index, field and error category. Ordinary
domain errors do not panic. App/CLI context must preserve the diagnostic.

This is an explicit failure-contract migration. Missing, malformed, duplicated,
extra-valued or negative-width source data will fail loading instead of being
skipped, defaulted or truncated. A valid positive-width track that the existing
finite geometry adapter cannot construct will fail with an explicit unsupported
or geometry outcome instead of disappearing or becoming successful empty.
Budget exhaustion remains distinct from invalid source and from a strict
predicate that remains undecided. No partial board is returned as complete.

Validate every required source field before the existing exact-zero-width
empty identity. All previously successful valid workload gates and full fixture
resolutions remain. Preserve the existing finite construction for admitted
ordinary inputs, including its exact output and source metadata. Its known
round-cap and exact-fidelity gaps remain unresolved; an honest rejection of an
unsupported valid input is not counted as certified geometric completion.

The archived candidate's malformed-track test expected a returned board with
failed regions. Retain that original complete test file unchanged, record its
result, and do not claim to satisfy that different carrier contract. New public
API tests require typed load failure, no returned board, explicit resource
exhaustion, recovery, and valid/empty controls. Actual CLI consumers must reject
malformed input visibly. This choice follows P6 without fabricating a location
or introducing another geometry carrier.

Increment the readiness implementation version because source failures change
report behavior. Preserve raw valid reports; any comparison must identify the
declared version-only metadata change separately from geometry/findings, and
must not normalize away other differences. This is not a performance-only
patch or a completed claim about every KiCad object or failure category.
