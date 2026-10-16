# Fresh physical build requalification

The validation infrastructure owns artifact identity (P19, P27, P36, P37).
The nested-overlay counterexample and retained observations are recorded at
`../2026-10-16-native-metadata-isolation/progress-v2.md`. No earlier artifact is
reset, overwritten or reused as a compiler input here.

Before compiling, exercise executable loading, hard-link identity, atomic path
replacement, truncation, and both file lookup orders in a fresh physical tmpfs
directory. This finite storage check does not prove filesystem correctness.
Use a new, empty, physical Cargo target and independent source copies; do not
seed from an earlier target or mount an overlay. Preserve source/lock/toolchain,
commands, process outcomes, exact outputs and retained artifact hashes.

The baseline is the current source at `3b69af324a9642d1f71b9cc38de58b59a236f740`
plus the unchanged 35 pending native files. The metadata-order candidate is not
included. Re-run root and affected native correctness, lint and documentation
checks; run the original full board, copper neck and Gerber gates unchanged.
Each command has external elapsed-time and address-space containment, not a
claim of internal bounded work or cancellation. A failed command stays failed.
This phase is not the complete pre-push workflow matrix or a release check.

Format-boundary hypotheses remain separate until reproduced through public APIs
on independently built artifacts. No production source change is part of this
initial requalification phase.
