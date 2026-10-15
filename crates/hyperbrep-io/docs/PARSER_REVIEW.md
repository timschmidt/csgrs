# Part 21 dependency decision — 2026-10-15

Implement the required syntax locally. Neither inspected parser preserves exact
numeric lexemes in its AST or exposes caller-selected admission limits. Adopting
one would require replacing the numeric carrier and recursive collection paths.
No upstream implementation, schema file or third-party fixture is copied.

| Candidate | Inspected snapshot | Decision |
| --- | --- | --- |
| Ruststep | `cd89e5a7cd9a6baadcb456392a5888f4cbc6b707` | Not adopted; numeric demotion and oversized-integer panic reproduced |
| step-p21 | `2baab1148970a4a328429a0a5f1e459c979e77ba` | Not adopted; same decisive numeric failures reproduced |

The [Ruststep token parser](https://github.com/ricosjp/ruststep/blob/cd89e5a7cd9a6baadcb456392a5888f4cbc6b707/ruststep/src/parser/token.rs)
converts real tokens to `f64`, integers/exponents to `i64`, and uses `expect` on
integer conversion. Its public parse entry point has no budget argument.
The [fork's token parser](https://github.com/virtualritz/step-p21/blob/2baab1148970a4a328429a0a5f1e459c979e77ba/crates/step-p21/src/parser/token.rs)
retains those choices. Both source snapshots accept the shared hand-authored
escaped-apostrophe, empty-aggregate, complex-instance and forward-reference
fixtures. This observation concerns their repository snapshots, not a claim
about older published releases.

The isolated probe compiles both snapshots and feeds the same literals. Both
round `9007199254740993.0` to `9007199254740992`, convert `1.E400` to infinity and
`1.E-400` to zero. Both panic for the tested integer literals
`9223372036854775808` and `184467440737095516160000`; the probe catches those
panics. These are correctness probes, not comparative performance benchmarks.
They are enough to reject the candidates for authoritative exact import.
No hostile-depth execution or upstream WASM qualification is inferred.

Both manifests identify Apache-2.0 licensing. Their READMEs exclude schema and
STEP fixture directories from the project license's scope. The review copies
none of those resources. Generated schemas, broader entity mapping and external
interoperability fixtures require their own provenance checks.

The local implementation keeps source slices and flat preorder parameter nodes.
Immediate-child iteration skips whole subtrees using recorded end indices;
parsing and writing use explicit heap frames. This design avoids recursive
parameter-tree destruction as well as recursive parsing. IDs use a compact
sorted index; arbitrary entity numbers cannot request sparse arrays. Caller
limits apply before adding nodes, frames, records, instances and sections.
The existing Hyperreal `Rational` parser owns exact numeric conversion after
separate digit/exponent admission; the IO layer adds no scalar backend.

The initial syntax subset preserves encoded string tokens without decoding
control directives. Header/schema validation, legacy text decoding and extended
exchange sections remain explicit follow-up work. The dependency decision does
not establish complete Part 21 or AP242 support.

The [Part 21 edition 3 reference](https://www.steptools.com/stds/step/IS_final_p21e3.html)
was checked for token rules and entity identity. Clauses 6.4.4.3 and 11.2 require
nonzero identifiers unique across the exchange; leading zeros do not change
identity. Clause 6.4.6 specifies zero padding on the left of binary payloads.
The lexer checks both requirements and distinguishes exchange delimiters from
record/type keywords. Added regressions failed on the initial implementation
before correction. The binary regression checks all 64 combinations of padding
count and first hexadecimal digit. This focused review is not full conformance
qualification.
