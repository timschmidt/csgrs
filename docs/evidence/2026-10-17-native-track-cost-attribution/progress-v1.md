# E37: full-project cost attribution

This read-only diagnostic identifies the next owning-layer experiments. No
production source changes are admitted and the E36 rejection remains in force.

| Source | Project | Load (s) | Native aggregation (s) | Layer checks (s) | Neck checks (s) |
| --- | --- | ---: | ---: | ---: | ---: |
| baseline | cparti | 1.463 | 28.962 | 0.656 | 5.302 |
| baseline | hvp109a | 0.883 | 6.061 | 1.802 | 7.726 |
| candidate | cparti | 5.519 | 220.173 | 22.665 | 83.258 |
| candidate | hvp109a | 1.756 | 29.254 | 7.874 | 16.187 |

These rows are diagnostic, not optimization benchmarks: baseline rectangles
and candidate capsules represent different source sets. CPArti candidate
timing includes brief debugger pauses and a three-second sampled profile.
Each board runs in a separate process; the original full gate runs both.

Native aggregation accounts for most of the expensive layer-sanity group.
A resolved stack catches the batch circle-support cache scanning retained
entries through arithmetic point equality. Later scalar and retained-ray
samples show other work as well. The profile has 598 cycle samples and no
reported loss, but incomplete call-graph unwinding prevents reliable
inclusive attribution. An exact rational support-key index is an experiment
to measure, not an optimization already established by this evidence.

Neck checks separately enter native offset regularization and exact loop
bounds. Their 60-second original limit still requires an independent repair.
The raw findings retain uncertainty and readiness qualification. No local
Certified receipt promotes sampled reports or historical source intent.

The first stack attempt could not load symbols. Its zero exit code is not
credited as useful attribution. The corrected attempt names the verified
executable explicitly; its namespace warning remains recorded. A later
aggregation-stage guard rejected an attempted snapshot before attachment.

Both original full project archives, exact board bytes, source manifests,
libraries, probe source, commands, findings and raw observations are retained.
Completed Cargo targets were read only. The historical artifact audit still
has 27,093 missing paths. No whole-history recovery, full CI completion,
published-package validation or complete-properties claim is made.
