# Checkpoint 87 evidence

Read [findings-v87.md](findings-v87.md) for conclusions and limitations. No
production transfer is retained. Earlier sealed checkpoint directories must
remain unchanged; later work belongs in a new sibling subdirectory.

From this directory, `node evidence-v87.mjs` revalidates the source/evidence
chain, read coverage, independent model, static benchmark check and frozen
native test captures. It requires permission to spawn child processes. It
reexecutes models/checkers, not the recorded Rust tests or donor benchmarks.
The environment capture is historical and is not required to remain unchanged
when a compiler is eventually installed. `--record` creates the manifest only
once, after all files and captures are final.

Results live in ../../calcium/results under coq-aern-*v87. Each capture keeps
its metadata, stdout and stderr, including unusable/failing attempts. The
final verification capture is separate from the ten earlier bound captures.
The root continuation ledger and interim report are mutable summaries and
are not part of the immutable checkpoint artifact set.
