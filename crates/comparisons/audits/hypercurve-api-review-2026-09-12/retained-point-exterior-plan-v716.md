# Coincident exterior point probe

V714 independently failed four chord-normal rectangle queries: the exact target equals the first exterior corner of the certified boundary enclosure. BezierAlgebraicChord2::try_new_with_endpoint_equality returns ZeroLengthLine when equality is decided true, and the probe loop previously propagated that error. The exterior point is already a complete Outside witness.

Handle that specific constructor outcome in the probe loop and return Outside. Preserve all other errors, uncertainty/direction retries, ownership, winding and branch selection. No repeated equality predicate, extra materialization, constructor API or geometric approximation is introduced. The repair applies to both existing LoopParity and FilledRegion callers.

V716 contains two isolated copied files: this bounded Boolean probe repair and the unchanged four V713 representation/oracle tests. Production remains clean. Qualify before promotion or any public point-classification migration. NextV717 driver.
