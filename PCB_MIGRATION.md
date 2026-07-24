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

Migration status: complete. `InterfaceKind::Package` and
`InterfaceKind::Electrical` were removed immediately under the workspace's
breaking-change policy instead of waiting for csgrs 0.25.0. The generic
`PartTerminal` carrier now describes only mechanical, thermal, and process
attachment points; `pin` and `pad` have no csgrs role semantics.

`hypercircuit::LegacyCsgrsElectronicsImport` remains only as the versioned
reader for migration handoffs captured before live marker removal. It no longer
constructs a handoff from csgrs metadata. Gerber and other geometry I/O remain
in csgrs; net, component, pad, via, zone, stackup, and manufacturing-intent
interpretation remains outside it.

That HyperCircuit reader is scheduled for deletion in HyperCircuit 0.4.0,
recorded by its exported `LEGACY_CSGRS_ELECTRONICS_REMOVAL_VERSION`. Before
then, callers must explicitly author every reported omission and persist a
current HyperCircuit semantic document. csgrs has no compatibility API to
remove: the live markers and electrical terminal roles are already gone.

No compatibility path may infer nets, connectivity, footprint identity,
placement, layer policy, or manufacturing rules from geometry.

`src/tests/ownership_tests.rs` guards this boundary by rejecting public
circuit/PCB type declarations, a reverse dependency on hypercircuit, and any
return of the retired marker or electrical terminal-role vocabulary.
