The original complete boards make 21,954,159 coefficient multiplications.
Right-rational scaling occurs 3,067,533 times (13.972%); 1,348,450 calls have
owned left vectors that could potentially be reused. Rational-left scaling of
borrowed right vectors occurs 8,594,476 times. Either direction covers 53.120%
of observed calls, including rational-by-rational operations.

| Left / right vector shape | Rational | Radical | Opaque | Mixed |
| --- | ---: | ---: | ---: | ---: |
| Rational | 1,719,083 | 7,977,215 | 600,243 | 17,018 |
| Radical | 1,307,308 | 8,276,574 | 984,639 | 4,518 |
| Opaque | 38,284 | 161,216 | 691,422 | 14,799 |
| Mixed | 2,858 | 1,736 | 76,610 | 80,636 |

“Rational” means both vectors are empty. The other classes describe radical
and opaque-atom vector presence; they may also have a nonzero rational constant.
The counters read only existing vector lengths. They add no scalar queries,
owners, arithmetic operations, cache publication or recognition passes.

[The checked assessment](assessment-v1.json) retains every count and summed
vector length. Two counter calibration tests, six retained-cache regressions
and five endpoint regressions pass, as do formatting and strict release,
all-feature, all-target Clippy for scalar, curve and DRC crates. The diagnostic
has a separate source and Cargo target with source/executable hashes. All 35
pending main runtime entries remain unchanged.

Both boards emit their reports with the original 12/197 finding counts. The
instrumented run still fails the original worker-CPU assertion at
134.127171383 seconds / 120. Counter synchronization changes execution cost;
this duration and the call frequencies are not a speedup or savings estimate.
Reporting happens after the original assertion, including during unwinding.

[The next experiment](decision-v1.json) evaluates two separate variants from
the uninstrumented parent: reuse owned left vectors for a rational right scale,
and additionally clone/scale borrowed right vectors for a rational left scale.
The variants need independent correctness and fixed cost results because the
second changes different ownership and operand/cache work. No runtime change
is admitted. Existing frontiers, original native limits, endpoint tradeoffs,
complete reports and the full properties/future-work plan remain in scope.
