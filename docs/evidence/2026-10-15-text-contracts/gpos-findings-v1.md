# Pair-adjustment reproduction

The preserved `72cd216cc` library applies a Format 2 GPOS class-zero adjustment
even when the left glyph is absent from the subtable coverage. The synthetic
font covers R only, with an adjustment of -200 font units. Q is a clockwise
quadratic arch with x in [0,100], horizontal advance 1000, and maximum y=50.

`gpos-reproduction-v3` passes its three diagnostic commands: Q contains (50,10),
but QQ incorrectly contains (850,10) and excludes (1050,10), after applying the
baseline's declared unit conversion. The expected second arch is [1000,1100].
This independently establishes the placement error before the coverage fix.
Candidate tests require the opposite two QQ classifications and verify that
covered RR still receives the adjustment. This is an intentional layout fix,
not byte preservation of the incorrect old placement.

The first two trials remain intact. V1 did not compile because the diagnostic
omitted unwrapping the point-classification Result; its invocation therefore
also failed. V2 compiled but its RR control used a counterclockwise square,
which the old importer classified as a hole. V3 uses the clockwise Q control
and the original glyph data; it does not alter a production workload or relax
the expected semantics. The new NonZero importer also accepts the synthetic
square and verifies its exact area independently.
