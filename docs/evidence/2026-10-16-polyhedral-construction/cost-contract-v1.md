# Fixed construction cost controls

Before production edits, measure retained `67a3daae8` all-feature benchmark
libraries on the original tetrahedron and square loft from reproduction-v1.
Construction includes input construction and output materialization, with a
strict Certified receipt. Timing excludes serialization, topology and oracle
checks, which are performed separately on complete outputs. Cold children have
no warm-up; warm children perform one identical completed construction first.
Use three fresh timing children and one allocation child per case/temperature,
then the same harness and build graph for the candidate. Keep every sample.

The harness records requested allocation/reallocation bytes, live/peak/retained
bytes, exact ordered position/index JSON and process resources. Allocator
overhead is excluded. Full output bytes must agree for these preserved valid
controls. Corrected concave/redundant-section behavior is assessed semantically,
not compared as a speedup against a wrong or unusable baseline output.

This correctness repair may add validation cost. These are fixed focused
probes, not a competitive benchmark or a statistical speedup claim. External
wall/address-space containment does not establish internal resource bounds.
