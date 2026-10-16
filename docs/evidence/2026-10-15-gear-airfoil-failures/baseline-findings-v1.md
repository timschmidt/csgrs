# Target-revision reproduction

All three commands in `reproduction-v1` pass as diagnostics on the retained E15
library and 1,914 pinned dependency inputs. That pass reproduces defects, not
corrected behavior. No implementation edit occurred before this run finished.

The five public families produced 22 successful empty profiles for invalid,
undecided or excessive-count inputs. Five additional capacity cases panicked
before allocation; the probe catches and retains those diagnostics. All three
gear solid wrappers then returned successful Certified empty meshes for zero
module under both STRICT and APPROXIMATE_512: six hidden failures.

All ten original full-resolution README/profile-benchmark inputs complete as
one material contour without holes. Exact serialized coordinate expressions
are retained in the baseline target's `fixtures/` directory. Vertex counts,
which supplement rather than establish semantics, are:

| Family | README | Benchmark |
| --- | ---: | ---: |
| Involute gear | 308 | 360 |
| Cycloidal gear | 672 | 288 |
| Involute rack | 30 | 18 |
| Cycloidal rack | 340 | 132 |
| Airfoil | 192 | 160 |

The independent module-1, two-tooth, 30-degree involute rack oracle passes on
this baseline: area `15*pi/4 + 29*sqrt(3)/24`, tooth-center inclusion, pitch-gap
exclusion and backing inclusion. Keep this exact expectation through migration.
The matched coordinate records are a preservation check, not an independent
oracle for the intended analytic shapes or their approximation error.
