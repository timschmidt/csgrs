# Faceted STEP fixtures

These four files were originally authored for HyperBRep from the elementary
recipes below. They are distributed under the repository's Apache-2.0 license.
They contain no third-party CAD model or copied EXPRESS schema. Entity IDs and
polygon order are deliberate test data, independent of the production mapper.
Each file declares millimetres and a source uncertainty of 1e-7 mm; the latter is
metadata, never a test tolerance for native geometry.

| File | Authored solid | Exact volume, mm³ | Unique V / E / F / bodies |
| --- | --- | ---: | --- |
| [faceted-box.step](faceted-box.step) | `[0,2] × [0,3] × [0,4]` | 24 | 8 / 12 / 6 / 1 |
| [faceted-two-boxes.step](faceted-two-boxes.step) | The box and its translation by `(10,0,0)` | 48 | 16 / 24 / 12 / 2 |
| [faceted-through-hole.step](faceted-through-hole.step) | Box minus the through-hole `(0.5,1.5) × (1,2) × [0,4]`; two cap inner bounds plus four hole walls | 20 | 16 / 24 / 10 / 1 |
| [faceted-tetrahedron.step](faceted-tetrahedron.step) | Vertices `(0,0,0)`, `(4,0,0)`, `(0,3,0)`, `(0,0,6)` | 12 | 4 / 6 / 4 / 1 |

The tetrahedron's oblique plane uses axis `(3,4,2)` and reference `(-4,3,0)`,
requiring exact radical normalization. All other face axes are Cartesian.
Representation #502 owns the body items, #9003 owns the unit context, and
#9500–#9508 provide basic product/root metadata for an independent reader.
Native tests deliberately select #502 and report the untraversed product records.

[Independent OCCT reader results and file hashes](../../../hyperbrep/docs/evidence/2026-10-15-step-faceted/occt-read.json)
check each file's solid validity, unique entity counts and floating-point volume.
Exact volume assertions use the recipes above in Rust, not OCCT values.
This is four elementary original fixtures read by an independent tool; it is
not a corpus of files supplied by independent CAD exporters. The external
[writer probe](../../../hyperbrep/docs/evidence/2026-10-15-step-faceted/occt-writer-experiment.json)
records why the attempted OCCT faceted export was not used as an input oracle.
