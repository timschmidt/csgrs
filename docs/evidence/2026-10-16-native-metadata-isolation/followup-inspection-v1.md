# Interchange follow-up: inspection only

This note identifies missing acceptance cases for a subsequent fresh phase.
It does not claim an executed reproduction or change the metadata trial.
The owning layer is the CSGRS format boundary; P18, P25, P26 and P27 apply.
The required criterion is format-specific numerical fidelity and downstream
consumption, separately from topology and inherited certainty.

Current OBJ and ASCII STL coordinate writers use fixed `.17` decimal places.
For a candidate triangle or closed cube with scale `1/10^20`, independently
known nonzero coordinates would render as zero under that formatting rule.
A public export/reimport probe must establish the actual current behavior;
none has run in this phase. Existing round-trip triangle counts are not an
oracle for coordinate or represented-set preservation.

Binary STL computes a normal after binary64 projection, then rounds vertices
to binary32. A candidate unit cube translated by `2^24` along x has distinct
binary64 x coordinates `2^24` and `2^24+1`, which round to the same binary32
value under ties-to-even. Check whether successful export can produce a file
that the supported importer rejects, and distinguish an explicit loss contract
from any promised topology. Do not infer an exact-source certificate from a
validated quantized import.

OBJ emits triangle indices without indexing the position array in the writer.
Exercise missing vertices and index overflow through the public exporter;
prove the failure category before deciding whether shared boundary validation
is missing. These are hypotheses from reading current code, not new reproduced
findings. Do not change output artifacts as an alleged performance-only fix;
any serialization change needs a stated migration and exact expected rows.

Reproduce these cases after the current sources and measurements close, in a
new phase with independently derived expected coordinates and typed errors.
Do not weaken the native runtime gates or discard this trial's unfavorable data.

## Inspected source pins

- `src/io/obj.rs`: `37caa578342c9b618d9e7b70f03d2ee0cf7b33670308a630299f1691b33d5bb0`
- `src/io/stl.rs`: `ab29247046a12a794de8a22294c4bdae05bc523e8d3c0f1ead0d0cece6ac53e8`
- `src/io/mod.rs`: `20d41b34396d8708c1bc6c6f3d07a11b3d36d74a0c86eda6815c9c75bb3e5d02`
