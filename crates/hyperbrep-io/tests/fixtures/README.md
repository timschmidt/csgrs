# STEP geometry fixtures

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

## Independently exported advanced inputs

OCP 7.9.3.1 generated three additional original fixtures using OCCT's general
STEP writer (`STEPControl_AsIs`). They contain no copied third-party CAD model.
Units are millimetres and representation #10 is explicitly selected.

| File | Construction | Volume, mm³ | V / E / F |
| --- | --- | ---: | --- |
| [occt-advanced-box.step](occt-advanced-box.step) | `BRepPrimAPI_MakeBox(2,3,4)`, surface-curve mode 1 | 24 | 8 / 12 / 6 |
| [occt-advanced-box-3d.step](occt-advanced-box-3d.step) | Same box, surface-curve mode 0 | 24 | 8 / 12 / 6 |
| [occt-advanced-hole.step](occt-advanced-hole.step) | `BRepAlgoAPI_Cut(MakeBox(4,3,2), MakeBox((1,1,0),(3,2,2)))`, mode 1 | 20 | 16 / 24 / 10 |

Mode 1 retains the writer's paired planar pcurves. The files retain original
timestamps, entity order and whitespace. Exact native tests derive expected
geometry from these dimensional recipes. Mutations operate on copies and test
master/alternate parameterizations, topology identity, orientation, units and
strict incidence. Native healing is never inferred from OCCT's tolerance policy.

## Exact analytic cylinders

These original fixtures are distributed under HyperBRep IO's Apache-2.0 license.
They declare metres and radians, with representation #10 selected by native tests.
Every cylinder has two vertices, three edges (two full circles and one seam used
twice), three faces and one solid. Expected geometry comes from these recipes:

| File | Construction | Exact volume, m³ | Exact area, m² |
| --- | --- | --- | --- |
| [exact-cylinder.step](exact-cylinder.step) | Radius 5, height 7, seam at `(3,4)` | `175*pi` | `120*pi` |
| [supplied-cylinder.step](supplied-cylinder.step) | Same solid with supplied planar circles and cylindrical lines, distinct source parameterizations and cylinder X direction `(3,4,0)` | `175*pi` | `120*pi` |
| [negative-circle-axes.step](negative-circle-axes.step) | Reverse both circle axes and edge senses | `175*pi` | `120*pi` |
| [negative-cylinder-axis.step](negative-cylinder-axis.step) | Reverse the cylindrical surface axis | `175*pi` | `120*pi` |
| [negative-cap-axes.step](negative-cap-axes.step) | Reverse cap axes and face senses | `175*pi` | `120*pi` |
| [oblique-cylinder.step](oblique-cylinder.step) | Rotate by `(x,y,z) -> (x,(3*y-4*z)/5,(4*y+3*z)/5)` | `175*pi` | `120*pi` |
| [radical-frame-cylinder.step](radical-frame-cylinder.step) | Radius 5, axis `(0,1,1)`, reference `(1,1,0)`, height `7*sqrt(2)`; rational source coordinates require irrational normalized frames | `175*sqrt(2)*pi` | `(50+70*sqrt(2))*pi` |

The [variant generator and independent OCCT reader](../../../hyperbrep/docs/evidence/2026-10-15-step-circles/axis_variants.py)
retain the complete source recipes and floating-point interoperability checks.
Their outputs are byte-identical to the corresponding native fixtures. The
[shared acceptance tests](../support/analytic_cylinder.rs) also construct partial
circular edges, independent frame/sense changes, unit conversions and invalid
near-incidence variants. None modifies the original files.

[occt-cylinder-off-carrier.step](occt-cylinder-off-carrier.step) is a byte-identical
copy of the original OCP 7.9.3.1 `BRepPrimAPI_MakeCylinder(2,4)` export retained in
the [periodic-cylinder research](../../../hyperbrep/docs/evidence/2026-10-15-periodic-cylinder/research/occt-cylinder.step).
It declares millimetres. Its tiny nonzero decimal seam coordinate is not exactly
on the declared circle, so native import must reject it. It remains an independent
exporter input and exact-fidelity rejection, never a healed success fixture.
