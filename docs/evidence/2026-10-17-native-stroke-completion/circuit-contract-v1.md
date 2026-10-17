# Complete the typed text migration at the materialization consumer

The v4 downstream Hypercircuit all-features build fails at materialize.rs:3021:
`truetype_text` returns `Result<CurveRegion2, ValidationError>`, but the consumer
passes it to a transform expecting `CurveRegion2`. The text API and consumer
are byte-identical to b137381247a03342ce5b6184710475fcc22c9d63. Reproduce the
compile failure with that baseline's frozen sources before changing code.

The owning layer is Hypercircuit's public materialization adapter. P6, P8,
P9, P18, P19, P25, P30 and P37 require its selected text policy, certainty
receipt and typed constructor failures to survive downstream transforms.
Use the existing context-bearing CSGRS constructor and the existing
MaterializationDecisions receipt consumer and Construction error variant.
Do not unwrap an ordinary domain error or turn an invalid selected font into
an apparently complete materialization with omitted artwork.

The domain is non-whitespace production artwork with a selected font. Preserve
the existing explicit omission when no production-font policy is selected.
Valid text retains its source/font identity through rotation, translation and
fabrication. Invalid fonts and missing glyphs fail the public materialization
operation with their typed cause, including partial-prefix text such as AB
when the font has only A. Exercise STRICT and APPROXIMATE_512; selecting the
latter alone must not weaken a certified successful text result.

The old fabrication test expects invalid selected font data to yield an
omission in a successful report. This expectation must explicitly migrate to
a typed failure with no report available to export. Preserve the complete
successful fabrication fixture and its assertions; strengthen the failure
assertions and retain the original test bytes in the frozen baseline/v4.

Keep completed v4 evidence for unchanged dependency closures. A final source
snapshot adds only the consumer migration, its acceptance tests and its public
documentation to the five frozen stroke files. Requalify Hypercircuit and
strict lint after the final edit. Complete the unchanged curve all-features
corpus with compilation and execution contained separately. Preserve all v4
failures, raw output and prerequisite-guard failures; do not claim v4 passed.

No test, fixture, native workload limit or policy is relaxed. No new public
error category, total-work or memory bound, universal provenance guarantee,
release completion or performance improvement is claimed.
