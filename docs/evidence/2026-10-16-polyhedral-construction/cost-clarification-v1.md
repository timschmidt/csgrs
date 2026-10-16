# Timing boundary clarification

The timing harness is unchanged from the pre-edit baseline. Its interval wraps
`build(case)`, including input construction, the public constructor, its
certainty assertion and output materialization. Thus the candidate's new
internal ring and PWN validation is included in its measured cost. The baseline
executes its original constructor in the same interval.

The phrase "excludes ... topology and oracle checks" in cost-contract-v1 means
the harness's separate acceptance checks, not validation performed inside a
constructor. Exact output serialization/replay and independent area, volume,
membership and topology evidence remain outside both timing intervals. This
clarifies the existing boundary; it changes no command, fixture, policy,
measurement or expected output. Both controls produce the same valid meshes.
The incorrect concave baseline and unusable repeated-section outputs are not
counted as cheaper successful constructions.
