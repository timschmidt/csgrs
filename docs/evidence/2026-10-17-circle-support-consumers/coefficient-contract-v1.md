# Shared large-coefficient cache reuse

Target main revision: 67a6ba54f1bf25f0ef495972238bba0a0102137e.
The E38 circle-support index remains isolated and unadmitted. Its existing
integer, symbolic, mixed, superrange and subrange cost cases do not measure
repeated reuse of one support pair with very large shared exact coefficients.

Owner: Hypercurve's temporary intersection batch; scalar equality and hashing
remain owned by Hyperreal. Applicable properties: P4, P17–P20, P27 and P35–P39.
Rational clones share an Arc, but numeric hashing reads numerator words. This
creates a possible warm-lookup regression that requires measurement before
retention; no regression is assumed in advance.

Construct two exact circles with a shared center `(2^b + 3, 0)` and radii 5 and
7, for b = 64, 65,536 and 1,048,576. The independent oracle is that concentric
circles of distinct positive radii have no common point. Strict queries must
return Disjoint with a Certified receipt, retaining one cache entry. These
are valid finite rational inputs; no coefficient-size restriction is added.

Compile the same diagnostic in both frozen E38 source variants using exactly
the same verified dependency libraries, compiler and flags. Only the ignored
diagnostic module changes. No production source, fixture, policy or dependency
is changed. Use a fresh private runtime directory, never a closed Cargo target.

Measure one cold query and 20,000 warm queries per fresh process. Include
relation validation and the complete ordered result-row materialization in
each phase; exclude input construction, coefficient inventory, printing and
teardown from phase timing, but include them in process resources. A zero row
encodes the complete Disjoint relation, which has no coordinate witnesses.
Cold single-query timings are diagnostic only. Use seven matched repetitions,
alternate baseline/candidate order, and shuffle the coefficient cases with a
recorded seed. Keep every result, including unfavorable rows. Do not run cost
measurements concurrently with repository builds or tests.

External containment: 1,800 seconds and 8 GiB address space per command;
256 MiB free on the runtime filesystem, 768 MiB on the repository filesystem,
and 2 GiB MemAvailable. The exhausted /tmp and /dev/shm build filesystems are
read-only inputs for this diagnostic. These limits do not establish internal
work, heap, cancellation or coefficient-growth bounds.

This is a private-kernel diagnostic, not a distributed-binary or end-to-end
speed claim. If it finds a meaningful regression, reject or revise the index;
do not conceal the case or redefine the supported mathematical domain.
