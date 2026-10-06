# ADR 0004: Separate design, readiness, factory, and equipment authority

Status: accepted.

HyperCircuit owns design and release intent. HyperDRC is the only readiness
rule engine and returns complete coverage evidence. A future HyperFactory may
own execution and genealogy but must consume HyperCircuit APIs and immutable
release artifacts rather than recalculate design geometry.

Equipment controllers retain recipe execution and all real-time safety
authority. No cloud service, MES, adapter, or design tool may override local
guards, interlocks, emergency stops, or validated machine limits.
