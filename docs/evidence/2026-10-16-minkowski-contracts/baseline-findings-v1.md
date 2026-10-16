# Current-revision reproduction

The frozen `baseline-v1` diagnostic is compiled against the final qualified
all-feature libraries for `adef9a64a91a7524006ba41527e427e2edb79fbe`.
Sources, libraries, commands and full stdout are pinned in that phase.

* Two unit cubes separated by four units in X, summed with a unit cube, should
  yield two disjoint cubes of side two: volume 16, excluding `(3,1,1)`.
  The implementation certifies volume 24 and includes that gap witness.
* Adding an unreferenced stored position `(20,0,0)` to a unit cube should not
  alter its represented solid or its sum. The implementation certifies volume
  `314/6` rather than 8 and includes `(10,1/2,1/2)`.
* Canonical empty paired with an invalid no-position triangle returns successful
  Certified empty geometry, hiding the invalid index.
* Degenerate point-cloud and unresolved point-cloud inputs return string-only
  Geometry errors with no native source. These fixtures also pin the old
  point-set hull interpretation, which needs an explicit migration.

Native topology validation succeeds for the first two wrong sets: a topology
certificate does not establish fidelity to the intended operation. The original
cube(2)+cube(3) artifact is retained separately as a valid control. The baseline
driver also replays the preceding face/loft diagnostics without changing them.
