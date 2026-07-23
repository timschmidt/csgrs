# Circuit and PCB vocabulary migration

`csgrs` is the geometry/materialization layer. It owns profiles, curves,
regions, solids, transforms, offsets, booleans, meshes and geometry-format
adapters. It does not own electrical connectivity, PCB layout intent or design
rules.

The retained semantic owners are:

| Legacy csgrs metadata | Semantic replacement | Geometry handoff |
| --- | --- | --- |
| `InterfaceKind::Package` | `hypercircuit::LandPattern` | Materialize each `LandPatternPad` to a `csgrs::Profile` |
| `InterfaceKind::Electrical` | `hypercircuit::DeviceModel` and `DevicePin` | No geometry required |
| `PartTerminal { role: "pin" }` | `hypercircuit::DevicePin` / `PinRef` | Stable source id only |
| `PartTerminal { role: "pad" }` | `hypercircuit::LandPatternPad` / `PadId` | Source-addressable copper profile |
| Gerber aperture/trace interpretation as circuit intent | `hypercircuit` PCB features on import | `csgrs` continues parsing/writing geometry |

`InterfaceKind::Package` and `InterfaceKind::Electrical` remain temporarily as
deprecated compatibility markers so existing geometry metadata can be read.
Adapters should convert them at load time and must not create them for new
designs. The generic `PartTerminal` carrier remains for mechanical, thermal and
process attachment points; electrical `pin`/`pad` roles are legacy-only.

Removal schedule: delete the two deprecated interface variants in `csgrs`
0.25.0. `hypercircuit::LegacyCsgrsElectronicsImport` now provides the required
versioned persisted handoff, and its representative fixture proves that marker
and terminal claims cross the boundary with typed omissions instead of inferred
semantics. At removal, the `pin` and `pad` spellings of generic terminal roles
cease to carry electrical meaning. Gerber and other geometry I/O remain in
csgrs; net, component, pad, via, zone, stackup and manufacturing-intent
interpretation remains outside it.

No compatibility path may infer nets, connectivity, footprint identity,
placement, layer policy, or manufacturing rules from geometry.

`src/tests/ownership_tests.rs` guards this boundary by rejecting public
circuit/PCB type declarations, a reverse dependency on hypercircuit, or any
expansion of the two-marker compatibility exception.
