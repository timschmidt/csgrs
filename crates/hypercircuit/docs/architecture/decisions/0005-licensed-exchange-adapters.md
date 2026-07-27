# ADR 0005: Isolate licensed intelligent exchange formats

Status: accepted.

IPC-2581C and ODB++Design support is an explicit interchange adapter boundary.
HyperCircuit defines versioned, content-addressed request/package contracts and
a bounded subprocess adapter. It does not invent fields from unavailable
schemas, redistribute restricted standards, or claim conformance without the
official schema and suitable fixtures.

IPC XML is pre-inspected with DTD/entity rejection and resource limits before a
caller-supplied official-schema validator runs. ODB++ implementation remains
out of process so separately licensed tooling can be connected without adding
proprietary code or assets to the core crate.
