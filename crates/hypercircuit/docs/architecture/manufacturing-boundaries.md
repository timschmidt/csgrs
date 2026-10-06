# Manufacturing architecture boundary and glossary

HyperCircuit owns electrical/physical design intent and the immutable release
that represents it. HyperDRC owns readiness policy, capability profiles, check
selection, and structured findings. A future factory service may consume an
accepted release to own work orders, WIP, genealogy, recipes, quality records,
and shipping state. Equipment controllers and PLCs retain exclusive authority
over motion, process loops, guards, emergency stops, and safety interlocks.

No HyperCircuit API, release adapter, external service, or language model is a
real-time or safety controller. A release authorizes design intent; it does not
authorize a machine to bypass its local recipe validation or safety system.
`hyperparts` remains the owner of supplier, sourcing, lifecycle, and commercial
part facts. HyperCircuit records only the selected part identity and provenance
needed to make an assembly release reproducible.

## Glossary

- **Design release:** immutable, content-addressed HyperCircuit release core,
  artifact catalog, and optional signature envelopes.
- **Manufacturing package:** directory or deterministic ZIP transporting one
  design release and every catalogued artifact.
- **Panel:** typed arrangement of one or more child boards plus rails, tooling,
  fiducials, separation features, coupons, markings, and exact transforms.
- **Unit:** one physical board instance derived from a panel child.
- **Process plan:** factory-owned ordered operations that consume an accepted
  design release.
- **Recipe:** equipment-specific, revisioned settings authorized for one
  process-plan operation.
- **Work order:** factory-owned authorization to produce a quantity from a
  specific accepted release and process-plan revision.

The release digest is the join key between these domains. It is never replaced
by a filename, moving branch, human revision label, or vendor job number.
