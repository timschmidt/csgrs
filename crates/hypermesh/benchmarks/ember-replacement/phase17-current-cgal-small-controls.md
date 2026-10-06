# Phase 17 current CGAL small-control refresh

Date: 2026-08-05

Status: measured boundary; every competitive gap remains open

Implementation: Hypertri `72ddcdc`, Hypermesh `4aa51efc`

## Purpose

This refresh measures the current one-engine exact Boolean path against the
pinned CGAL 6.0.3 EPECK adapter on the two permanent small shared-contract
controls. It does not change production code and it does not select an
algorithm by fixture, size, operation, result, policy, competitor, or measured
performance. The measurements are a gate for subsequent general algorithmic
work, not inputs to production dispatch.

The exact rational OFF inputs were freshly exported from the permanent
Hypermesh fixtures. Their SHA-256 digests are:

| Input | SHA-256 |
| --- | --- |
| `crossing_octahedra-left.off` | `eb772e8f3db1a80a8f8a5981434ae6047b3222d58c9a8a42651ae640eb4ad876` |
| `crossing_octahedra-right.off` | `351fe5898ac5836dc2dcc96a25f0ec6a968ec246f68e81d749e03875741636d0` |
| `affine_boxes-left.off` | `58af4f44315585af2b3d2828c83bd15e9800a08e7a3d47b6b0c5089cb8827ca6` |
| `affine_boxes-right.off` | `60b5235146a8f993a0a07dcf77d6238a94dafae247746aae6b56ef54df393707` |

## Exact output gate

Both engines validate closed, structurally valid outputs for union,
intersection, difference, and reverse difference. CGAL's result-local vertex
counts are not compared with Hypermesh's shared arrangement arena; complete
surface triangle counts and volumes are compared per result.

| Fixture | Hypermesh shared vertices | Triangle counts U/I/A-B/B-A | Exact six-volume U/I/A-B/B-A |
| --- | ---: | ---: | ---: |
| crossing octahedra | 24 | 44 / 20 / 32 / 32 | 351 / 81 / 135 / 135 |
| affine boxes | 24 | 44 / 28 / 20 / 20 | 576 / 192 / 192 / 192 |

CGAL reports ordinary volumes 58.5/13.5/22.5/22.5 and 96/32/32/32,
respectively, exactly corresponding to the rational six-volumes above. Every
Hypermesh run is `Certified` under both `STRICT` and `APPROXIMATE_512`.
Consequently the two policies execute the same topology path here; no
approximate terminal is consumed.

## Fresh-process protocol

The established checkpoint protocol uses 63 fresh CPU-11-pinned Hypermesh
processes per policy, with one timed shared-arrangement iteration after fixture
construction and PWN validation. CGAL uses 63 timed repetitions over decoded
exact EPECK inputs, once with mesh copies outside and once with copies inside
the timer. Values are nanoseconds.

| Fixture | CGAL outside median (range) | CGAL inside median (range) | Hypermesh `STRICT` median (range) | Ratio to CGAL outside | Hypermesh `APPROXIMATE_512` median (range) | Ratio to CGAL outside |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| crossing octahedra | 120,282 (113,282--251,792) | 128,711 (113,042--350,826) | 558,352 (439,740--840,663) | 4.642x | 491,436 (443,219--687,693) | 4.086x |
| affine boxes | 390,933 (372,315--727,200) | 385,234 (370,755--543,283) | 1,040,088 (795,886--1,525,095) | 2.661x | 1,039,839 (802,445--1,533,075) | 2.660x |

These clock rows are frequency-sensitive and much wider than the deterministic
instruction controls. In particular, the apparent policy separation on the
certified crossing case is host noise, not a policy-specific algorithm. The
earlier checkpoint's medians were 3.529--3.545x and 2.105--2.108x CGAL; this
refresh is not sufficient evidence of an algorithmic regression because the
implementation outputs and deterministic current counters are unchanged by
the measurement protocol. No favorable sample or aggregate substitutes for
the losing medians.

## Reused-input protocol

The second protocol measures the scalar ownership model that long-lived callers
can actually exploit. Five paired CPU-11 trials run 1,000 Boolean iterations
over one retained Hypermesh `MeshContext` and exact input pair. Each paired CGAL
trial runs 1,000 EPECK iterations with input copies outside the timer. CGAL is
the median internal iteration; Hypermesh is aggregate elapsed time divided by
1,000. This preserves each engine's intended retained representation without
adding a compatibility layer or changing the Boolean algorithm.

| Trial | Crossing CGAL / Hypermesh / ratio | Affine CGAL / Hypermesh / ratio |
| ---: | ---: | ---: |
| 1 | 119,212 / 303,324 / 2.544x | 375,044 / 674,597 / 1.799x |
| 2 | 122,111 / 301,388 / 2.468x | 374,884 / 677,483 / 1.807x |
| 3 | 119,832 / 298,229 / 2.489x | 375,954 / 677,668 / 1.803x |
| 4 | 119,962 / 301,634 / 2.514x | 375,034 / 674,369 / 1.798x |
| 5 | 120,842 / 298,973 / 2.474x | 377,194 / 671,085 / 1.779x |
| median paired ratio | **2.489x** | **1.799x** |

Reuse narrows both gaps, consistent with Hyperreal retaining exact construction
facts and the Boolean engine reusing one policy context. It does not close
either competitive case. The next work should play to that ownership model by
retaining topology and exact facts that the general algorithm has already
proved, rather than reproducing CGAL's scalar schedule or adding benchmark
special cases.

## Deterministic work and advisory RSS

The current three-run medians for 1,000 retained strict arrangements are
2,916,553,799 instructions / 495,482,144 branches for crossing octahedra and
6,864,519,901 / 1,175,516,130 for affine boxes. These are retained as
Hypermesh regression controls. The CGAL adapter emits one JSON object per
iteration and performs different output copying/formatting, so whole-process
CGAL instruction counts are not timer-equivalent and are not used for a
competitive parity claim.

For the same reason, 1,000-iteration `/usr/bin/time -v` process RSS is advisory:
Hypermesh prints one summary whereas CGAL prints 1,000 JSON records. Hypermesh
reports 4,476 KiB versus CGAL's 6,136 KiB on crossing and 4,528 KiB versus
6,092 KiB on affine. These are 27.05% and 25.67% lower process maxima, but they
are not substituted for the isolated large-fixture allocator/Heaptrack matrix
or claimed as like-for-like kernel heap wins.

## Conclusion

The exact output contract, both-policy behavior, and retained-input advantage
are confirmed. Hypermesh remains slower on every refreshed small shared-contract
row, so Phase 17 stays open. General retained adjacency, arrangement lifetime,
and exact construction scheduling remain legitimate targets; fixture-aware
dispatch, policy-name shortcuts, competitor-aware paths, and weakened exact
validation remain forbidden.
