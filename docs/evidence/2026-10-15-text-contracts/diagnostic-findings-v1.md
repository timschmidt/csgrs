# Candidate diagnostics retained

`candidate-v1` preserves compiler failures in the first implementation draft:
two uses of a nonexistent context constructor, three incorrectly constructed
font tags, and a missing borrow in the native catalog test. No semantic pass is
credited to that build. The next draft corrects these interfaces.

`candidate-v2` passes six of seven public text tests, all six native Hershey
tests, all four dimension tests, and the actual text renderer. Its only failed
assertion is the R-space-R witness in the newly authored synthetic fixture.
The fixture covers R followed by any class-zero glyph, including space, so the
second R starts at 1000 - 200 + 1000 = 1800, not 2000. `layout-witness-v1`
reads the exact pair record and confirms all seven layout witnesses against
the unchanged candidate library, including inside at 1850 and outside at 2050.
The final test corrects this independently derived expectation to 1850 without
altering font bytes, policies or workloads. Original baseline failures and all
full-size README/benchmark inputs remain retained.

The candidate renderer completes with native outlines. Its Hershey PNG is
byte-identical to the existing asset. The TrueType asset intentionally changes
from sampled-chord vertices to native-curve vertices, matching the declared
geometry migration. Both images were visually inspected; visual agreement is
not used as an exact-set or topology oracle. See `render-migration-v1.json`.

`qualification-v1` retains the Clippy failure requiring `NativeOutline::invalid`
to be `const`. Its semantic suites passed, including the full native library
(1760 passed, one existing ignored test). After that correction,
`qualification-v2` passes all 16 commands. `qualification-v3` adds independent
cubic callback area/membership and invalid-coordinate source-chain tests and
passes all 16 commands on the final production sources. The full native-library
run is reused only after verifying that every `crates/` input hash is unchanged.
Historical feasibility output used the label "components" for `CurveRegion2::len`;
that count denotes boundary loops, including holes. Final tests, documents and
measurement columns use the correct term; old raw logs remain untouched.

`resources-v1` retains all 64 predetermined child attempts. All 32 Hershey
children succeed (24 timing and eight allocation rows). All 32 TrueType children
complete construction, then fail in the helper's untimed serialization: it
incorrectly assumed every exact boundary curve had a direct `geometry()` view.
The failed children emit no usable construction measurements; none is credited
as completed cost evidence. The successful Hershey costs are retained alongside
the replacement matrix, including unfavorable rows.

`serialization-v1` passes all seven commands against the unchanged baseline and
final candidate libraries. `resource_probe_v4.rs` fixes only the fixture
serializer: native line-family curves are determined by their exact endpoints.
For endpoints without direct coordinate views, the native enclosure must prove
both coordinate intervals are singletons before they can be serialized. Other
fixture curves retain their native control points. No approximate enclosure,
display chord, production change, timing-scope change or reduced fixture is
introduced. The fresh `resources-v2` phase uses this driver with the original
fixed matrix. The executed v1 scripts and driver remain immutable.

`resources-v2` passes all six phase commands, all 64 matrix children and all four
additional Hershey preservation checks. Both timing sets and allocation sets
remain in the final summary, including cost increases. Production inputs match
final qualification; the helper change does not reopen or replace code checks.
