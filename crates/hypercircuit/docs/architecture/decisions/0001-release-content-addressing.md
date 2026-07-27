# ADR 0001: Content-address manufacturing releases

Status: accepted.

HyperCircuit reuses `PackageDigest` and hashes exact bytes with canonical
`sha256:<lowercase-hex>` JSON/text representation. Every artifact is listed by
a normalized portable relative path, byte length, role, and digest. The
deterministically serialized release core transitively binds the catalog.

Directory names and ZIP bytes are transport details. Detached signatures do
not alter core identity. Missing, extra, duplicate, traversing, or mutated
artifact paths fail verification.
