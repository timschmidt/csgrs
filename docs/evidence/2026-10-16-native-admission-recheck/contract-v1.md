# Native admission recheck

Target revision: `5918d85aa0064614ba11dc3c8bcc6a0a7d4b7191`, with the same 35 separately pending Hypercurve/Hyperdrc files pinned by the preceding checkpoint. No implementation change is made for this reproduction.

Owning layers: Hypercurve exact native linear strokes, retained construction facts and certified predicate/cache machinery; Hyperdrc source import, display boundaries and checked geometry consumers. Relevant properties: P1–P9, P17–P25, P27, P35–P39. The domain is the original complete KiCad CPArti FPGA and HVP109A boards, finite exact native strokes, and the existing strict and approximate-policy contracts.

Missing current acceptance evidence: the previous native admission failed the original full-board 120-second worker-CPU gate and the 60-second copper-neck check-body wall gate. Later admitted scalar and mesh changes require reproducing those findings on the current source. The limits, full fixtures, policies, reports and assertions are unchanged. Both ZIP files and entries must exist and validate; skipping either fixture invalidates the reproduction.

A passing runtime gate is necessary but insufficient for admission. The pending source review, independent semantic evidence, native downstream checks, and unresolved broad library costs remain open. Process wall time is not either internal assertion metric. This single invocation of each gate is a diagnostic reproduction, not a matched speedup claim or release qualification.

All source and standalone Rust inputs remain frozen during a phase. Every command retains its logs, timing and exit status. A fresh copy-on-write target uses the immutable previous qualification as its lower directory; every lower file is hashed before and after. Previous failures and artifacts remain preserved. External 1,800-second wall and 8-GiB per-process address-space containment does not establish internal work or heap bounds.
