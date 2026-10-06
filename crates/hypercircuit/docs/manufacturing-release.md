# Manufacturing releases

`SemanticDocument::build_manufacturing_release` is the authoritative entry
point. Its defaults deliberately require no signing key, vendor profile, panel,
DFT plan, or proprietary adapter:

- the generic prototype HyperDRC profile is selected by release reporting;
- absent panel and DFT intent is explicitly `not_required`;
- the unsigned core and artifact catalog are deterministic;
- a path ending in `.zip` selects deterministic ZIP output, while every other
  path selects a directory;
- `verify`, `compare`, and `inspect` accept either representation.

The ownership boundary and domain vocabulary are recorded in
[`architecture/manufacturing-boundaries.md`](architecture/manufacturing-boundaries.md);
the adjacent ADRs cover identity, coordinates, signing, ownership, and licensed
exchange adapters. Every release includes its v1 JSON Schema under `metadata/`.
Callers may add sorted `ManufacturingRequirement` entries for required
inspection, test, or special processes; the default is empty because
HyperCircuit must not invent contractual factory operations.

Unsigned releases are the normal local-development and CI form. Signing is a
separate operation so it cannot change the core digest:

```text
hypercircuit release sign release.zip \
  --key-id production-2026 \
  --seed-file /secure/path/ed25519-seed.hex
```

The built-in Ed25519 envelope proves that the core digest was signed by the
embedded public key. Embedding a key does not establish organizational trust;
production intake must pin or certify acceptable key IDs/public keys. The
`ReleaseSigner` and `ReleaseSignatureVerifier` traits allow an HSM, PKCS#11
bridge, or remote signer to own key custody and trust policy. Advantages of the
built-in path are simple offline verification, deterministic signatures, and
no service dependency. Its disadvantages are seed-custody responsibility and
the lack of certificate lifecycle/revocation semantics. External signing keeps
keys out of the process and can enforce approval/audit policy, at the cost of
more setup, availability dependencies, and a required external verifier.

## Format authority and licensing

Attributed Gerber targets the Ucamco Gerber Layer Format revision recorded in
the fabrication manifest. As of this implementation that is `2026.05`.
Gerber Job output separately records job-format specification revision
`2020.08` and public schema revision `2023.06`; it does not imply
that the older job specification was revised with the layer format.

- Ucamco downloads: <https://www.ucamco.com/en/gerber/downloads>
- Public Gerber Job schema:
  <https://www.ucamco.com/files/downloads/file_en/397/gerber-job-file-schema_en.json>

Ucamco's specification text restricts redistribution, so HyperCircuit does not
vendor the PDF, schema, or conformance archives. Tests exercise the emitted
contract and independent parsers without copying those assets.

IPC-2581C output/import is intentionally an adapter boundary until the caller
supplies the official XSD and confirms its licensing. `inspect_ipc2581_xml`
performs pre-schema security checks (DTD/entity rejection, root/revision
policy, byte/depth/event limits) and records the supplied schema digest; it is
not represented as XSD conformance.

- IPC-2581 validation guidance and official-schema location:
  <https://www.ipc2581.com/ipc-2581-file-validation-tool/>

ODB++Design remains out of process behind
`IntelligentPcbExchangeAdapter`. The package protocol is versioned and
content-addressed, and the subprocess implementation enforces timeout,
response-size, crash, malformed-response, version, and digest checks. No ODB++
fields are guessed or reverse engineered. Obtain the current specification and
implementation rights directly from ODB++Design:

- <https://odbplusplus.com/design/odb-design-format-specification/>
- <https://odbplusplus.com/design/partner-terms-of-use/>
