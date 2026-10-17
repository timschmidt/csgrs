# Reconcile retained release expectations with the committed readiness version

Final consumer validation v1 compiles and passes the typed text/fabrication
regressions. Its complete Hypercircuit run reports 341 passes and seven failed
artifact assertions. Preserve that complete failed attempt. The six Easyduino
JSON assertions differ only at 160 coverage check-version fields each: their
expected version is 20, while the committed b1373812 producer is version 21.
The manufacturing archive still expects version 1. Its only changed entries
are the DRC evidence and the manifest that hashes that evidence.

The owning layer is the downstream deterministic-artifact expectation, under
P27, P35, P36 and P37. This is an explicit metadata migration following the
already committed readiness change, not permission to accept changed geometry,
findings, coverage status, policy, capability profile, provenance or fixtures.

Before replacing any expected bytes, decode all seven retained assertions and
verify every JSON field and archive entry. Independently derive the permitted
check-version substitutions from CHECK_IMPLEMENTATION_VERSION = 21, recompute
the DRC byte length and SHA-256 and the manifest core digest, and preserve all
other bytes and archive metadata. Retain both prior and actual artifacts and
the complete difference inventory. Do not use snapshot-update environment flags.

A new frozen snapshot changes only the seven expected artifacts relative to
v5. Production code and test bodies remain byte-identical. Recompile the two
affected unmodified integration test sources with their original all-features
configuration, using the completed candidate's libraries read-only and the new
snapshot's CARGO_MANIFEST_DIR. Run every test in both targets, including the
checks after the former assertion failures. Keep their original inputs,
policies and limits. No completed Cargo target is reopened.

Passing observations outside those two targets remain evidence only where
their source and runtime inputs are unchanged. The original incomplete suite
remains failed; its seven failed cases require successful complete replays.
No metadata update by itself establishes runtime correctness or a performance
improvement. The full published consumer and pre-push validation remain open.
