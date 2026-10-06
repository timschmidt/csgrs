# ADR 0003: Make release signing detached and optional

Status: accepted.

An unsigned deterministic release is the default for local work and CI.
Signatures cover a domain-separated canonical core digest and live outside that
core, so approval does not create a different design identity.

The built-in Ed25519 envelope embeds a public key for offline cryptographic
verification, but that key is not a trust assertion. Production intake must
pin keys or certificates and own rotation/revocation policy. `ReleaseSigner`
and `ReleaseSignatureVerifier` allow HSM or remote-service integration without
placing private keys in design files, command arguments, or release bundles.
