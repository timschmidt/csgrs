# Pre-verification draft range correction

An explicit physical line count, obtained while the initial draft binder ran,
showed that `hyperreal/src/real/arithmetic/elementary_functions.rs` ends at line
4200. The read command requested lines 4170..4205 but naturally stopped at EOF.
The initial draft copied that requested endpoint instead of the actual endpoint.

The unverified `e-qualified-experiment-draft.json` and its original binder,
verifier and live-patch generator are preserved. The corrected draft is
`e-qualified-experiment-draft-v2.json`; it credits only 4170..4200. No production
edit, numerical result, source snapshot or donor read total changed. The initial
draft was not presented as a successful verification and its out-of-range entry
was corrected before running the full draft verifier.
