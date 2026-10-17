# KiCad shape reference

Consulted 2026-10-17. KiCad constructs a width-bearing segment in
[`PCB_TRACK::GetEffectiveShape`](https://docs.kicad.org/doxygen/pcb__track_8cpp_source.html#l03154).
Its [segment projection](https://docs.kicad.org/doxygen/shape__segment_8cpp_source.html#l00137)
uses the [oval conversion](https://docs.kicad.org/doxygen/convert__basic__shapes__to__polygon_8cpp_source.html#l00118),
whose end arcs use half the width and whose straight length is the endpoint
distance. The mathematical coincident-endpoint case is therefore a disk.

This supports the circular-end interpretation already required by the archived
native-stroke regression. It does not establish equality with KiCad's integer
grid rounding or polygonal approximation. The new native disk retains the
source rational center and half-width; its local mathematical claims are
checked independently by exact area, membership and bounds assertions.
