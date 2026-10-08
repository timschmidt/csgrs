# Checkpoint 77: proof-only twelfth-turn candidate

This is an isolated, not-yet-retained experiment based on checkpoint 75's
957-file live map and checkpoint 76's pinned-source/value audit. No new donor
coverage is claimed. Only 181 Hyperreal source/support files are copied.

The candidate attempts signs in Q(sqrt(2),sqrt(3)) for otherwise unresolved sums
containing a prescaled sine/cosine/tangent/cotangent node. A bounded structural
parser proves rational pi/12 arguments. Rational arithmetic, the four field
basis elements, rational square roots provably in that field, and field
inverses may participate. No floating-point comparison, class hint, Unknown
cache, or approximate equality authorizes an answer. Zero denominators and
unproved domains are rejected. Numerical nodes and approximation caches are
not rewritten or populated by the proof.

Admission bounds: 128 visited nodes per proof (including angle parsing),
128 nodes in an allocation-free trig-presence scan, 1,024-bit admitted rational
numerators/denominators, and binary offsets of absolute value at most 1,023.
Intermediate arithmetic has fixed multiplicative overhead relative to these
bounds; field coefficient growth is checked after each accumulated product.
No unbounded integer factorization, DAG traversal or new persistent cache.

The hook runs before storing Unknown in each visited sum, not just the outer
root. Test failed child queries, parent-before-child queries, repeated queries,
shared/independent trees, and existing approximation/serialization behavior.

Qualification stages:

1. Internal field/sign/domain/resource/cache tests with an independent rational
   enclosure oracle, including near cancellation and all conjugate branches.
2. Reuse the *unchanged* checkpoint-76 public capability source against the
   candidate. Compare full ordered rows to baseline: only the 192 repeated
   Unknown rows representing 32 identities may become Equal; all poles,
   periodic controls and unequal perturbations must retain correct outcomes.
3. Broader regression, native/WASM, consumer, matched timing, allocation and
   binary/source-size gates before any adoption. These are not yet complete.

Do not call completion or retain on the basis of focused tests alone. Preserve
failed captures and source versions, bind all reported source and output hashes,
and reuse the existing shared target to conserve /tmp.
