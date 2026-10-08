# GMP Algorithm Parity

This file is the coverage ledger for Hyperreal's exact-rational algorithm
dispatch. The authoritative taxonomy is GNU MP 6.3.0's
[Algorithms](https://gmplib.org/manual/Algorithms) chapter. “Backend” below
means the locked `num-bigint` 0.4.6 dependency; “rational” means specialization
owned by Hyperreal before or after a magnitude operation.

Parity here means that an operation applicable to Hyperreal's public numeric
domain has a correctness-tested route, an observable dispatch path, and a
benchmark fixture. It does not claim that `num-bigint` uses GMP's exact source
implementation or tuning thresholds.

GMP is a taxonomy and benchmark reference only. Hyperreal does not link to GMP
or use a GMP-backed representation in normal/release builds; Rug/GMP is kept
strictly in dev-dependencies for comparison benchmarks.

## Coverage matrix

| GMP family | Hyperreal route | Trace evidence | Test/benchmark evidence | State |
| --- | --- | --- | --- | --- |
| Multiplication: basecase | checked `u128` products and backend long multiplication | `rational/mul/word-sized`, `rational_algorithm/*/backend-basecase` | word arithmetic tests; `mul_backend_basecase_cold` | Covered |
| Multiplication: Karatsuba | backend balanced Karatsuba and unbalanced half-Karatsuba | `backend-karatsuba`, `backend-half-karatsuba` | threshold tests; cold 40×40 and 33×66-limb benches | Covered |
| Multiplication: Toom-3 | backend Toom-3 above 256 limbs | `backend-toom3` | threshold test; cold 257×257-limb bench | Covered |
| Multiplication: Toom-4 | Hyperreal-owned seven-product `BigInt` evaluation/interpolation above 1,048,576 bits in the 7:6-to-5:4 balance band | selected `rust-native-toom4`; candidate `multiplication-candidate/rust-native-toom4` | differential/crossover/path tests; 6:5 retained-selector bench measures 14.85 ms versus backend 16.42 ms | Covered |
| Multiplication: Toom-6 | Hyperreal-owned eleven-product interpolation above 524,288 bits in the 9:8-to-7:6 balance band | selected `rust-native-toom6`; candidate `multiplication-candidate/rust-native-toom6` | differential/path tests; 8:7 retained-selector bench measures 4.90 ms versus backend 5.49 ms | Covered by benchmark-superior native Toom-6; GMP's exact asymmetric 6.5 shape remains a gap |
| Multiplication: Toom-8 | Hyperreal-owned fifteen-product interpolation above 262,144 bits for operands within a 9:8 balance ratio | selected `rust-native-toom8`; candidate `multiplication-candidate/rust-native-toom8` | differential/path tests and paired candidate/selected/backend benches from 65,536 through 4,194,304 bits; selected 1.54/3.99/10.77/28.95 ms at 262K/524K/1M/2M | Covered by benchmark-superior native Toom-8; GMP's exact asymmetric 8.5 shape remains a gap |
| Multiplication: FFT | exact Rust-native two-prime NTT with CRT reconstruction; production retains Toom-8 because the candidate is slower through 4,194,304 bits | candidate `multiplication-candidate/rust-native-ntt-crt`; production higher-Toom paths | differential tests through 262,144 bits and paired 262K/1M/4M benches: NTT 16.45/76.79/351.76 ms versus selected 1.54/10.77/77.32 ms | Partial: benchmark-rejected for dispatch |
| Multiplication: unbalanced | rational dyadic/general cancellation plus backend half-Karatsuba | `dyadic-general-cross-cancel`, `backend-half-karatsuba` | cross-cancel tests and benches | Covered |
| Division: single limb | backend single-limb division | `backend-single-limb` | classifier and path tests; cold exact-reduction bench | Covered |
| Division: basecase | backend normalized Knuth Algorithm D | `backend-knuth-basecase` | classifier and path tests; cold exact-reduction bench | Covered |
| Division: divide and conquer | production retains normalized Knuth division; a temporary `num-bigint` 0.4.8 Burnikel–Ziegler upgrade was rejected after materially regressing the measured GCD workload | production `backend-knuth-basecase` | locked-backend classifier/path tests; at 262K bits the 0.4.8 upgrade slowed half-GCD from 273.69 ms to 1.237 s and Lehmer from 131.00 ms to 406.32 ms | Partial: benchmark-rejected for release |
| Division: block-wise Barrett | correct Hyperreal-owned `BigUint` candidate with a reusable reciprocal and overflow-free direct product subtraction; production retains locked normalized Knuth division because Barrett loses all measured shapes | candidate `division-candidate/block-wise-barrett`; production backend paths | differential tests from word divisors through 4,096 bits; one-shot and batch-16 benches at 8,192/1,024 and 65,536/4,096 bits | Partial: benchmark-rejected for dispatch |
| Division: exact division | rational cross-cancellation, known-divisor reduction, and dyadic shifts | `reduction-*`, backend division subpaths, and `power-of-two-common-factor` reducer stats | rational oracle, cross-cancel/path tests, and single-limb/small/large Knuth reduction benches | Covered |
| Division: exact remainder | rational fractional remainder delegates to the selected native backend; GCD remainders remain inside their separately traced family | `exact-fractional-remainder/backend-*`; GCD family paths | fractional-remainder path test and wide normalized-Knuth benchmark; parse/format/square tests | Covered |
| Division: small quotient | comparisons and backend trivial/small-quotient exits | `backend-trivial-or-small-quotient` | classifier test and wide zero-quotient benchmark | Covered |
| GCD: binary | tuned `u64`/`u128` Stein reducer | `rational_algorithm/gcd/binary-word` | randomized word-GCD oracle test | Covered |
| GCD: Lehmer | leading-limb quotient batching for balanced magnitudes at and above three 64-bit limbs | `lehmer-leading-limb` | differential/path tests and paired 192/512/1,024/4,096-bit crossover benches | Covered |
| GCD: subquadratic | correct recursive Möller `hgcd-d` candidate with determinant-one matrices and selected Rust-native higher-Toom matrix products; production retains Lehmer because the candidate is slower through 1,048,576 bits | candidate `recursive-half-gcd`; production `lehmer-leading-limb` | matrix/differential/path tests and paired 8,192/16,384/65,536/262,144/1,048,576-bit benches; at 262K and 1M, half-GCD measures 272.27 ms and 3.880 s versus Lehmer's 132.22 ms and 2.044 s | Partial: benchmark-rejected for dispatch |
| GCD: extended GCD | not part of `Rational`'s public arithmetic domain | — | — | Not applicable |
| Jacobi symbol | not part of `Rational`'s public arithmetic domain | — | — | Not applicable |
| Powering: normal | checked word powers, backend binary powers, and retained repeated-squaring chains | `rational_algorithm/powering/*` | `powi` tests and retained/cold benches | Covered |
| Powering: modular | no modular integer API | — | — | Not applicable |
| Root: square root | residue filters, dyadic extraction, and backend Newton root | `square_extraction/*`, `newton-square-root` | extraction tests and retained/cold sqrt benches | Covered |
| Root: nth root | backend Newton root with exact power verification | `newton-nth-root` | exact/non-exact root tests; comparison bench | Covered |
| Root: perfect square | mod-64/mod-63 rejection followed by exact square verification | `mod64-reject`, `mod63-reject`, `newton-square-root` | square extraction tests | Covered |
| Root: perfect power | general prime-degree discovery narrowed by small-factor multiplicity gcd, plus exact fixed-degree extraction | `factor-multiplicity-reject`, `prime-root-candidate`, `prime-root-exact` | definition/path tests and measured reject/general/fixed-degree benches | Covered |
| Radix: binary to radix | backend repeated single-limb division below 32 limbs and divide-and-conquer conversion above it; rational decimals use exact digit division | `binary-to-radix/*` | crossover/path tests and measured small/large/fraction format benches | Covered |
| Radix: radix to binary | backend chunked multiply-add below 8,192 digits and a cached-power divide-and-conquer product tree above it | `backend-chunked-multiply-add`, `divide-conquer-product-tree` | correctness/path tests and paired 10,240/20,480-digit crossover benches | Covered |
| Prime testing | outside Hyperreal's rational/real API | — | — | Not applicable |
| Factorial, binomial, Fibonacci, Lucas | only derived statistical helpers need small fixed cases; no public integer sequence API | — | — | Not applicable |
| Random numbers | outside deterministic exact arithmetic | — | — | Not applicable |

## Retained facts

`RationalData` stores monotonic evidence in one atomic fact word. The first
exact dyadic-denominator query records both “known” and its value; later
dispatch reads that certificate instead of rescanning the immutable
denominator. Existing linear, powering, square-reduction, and exact-binary64
reuse evidence shares the same bounded word. `rational_data_layout_stays_bounded`
guards the node size.

## Dev-only GMP comparison

`benches/gmp_api.rs` compares the Rust-native release representation with raw
GMP and with conversion to and from GMP on every iteration. These July 2026
measurements are evidence, not release dependencies:

| bits | native multiply | raw GMP multiply | GMP round-trip multiply | native divide | raw GMP divide | GMP round-trip divide |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 4,096 | 2.14 µs | 1.47 µs | 5.42 µs | 4.89 µs | 2.64 µs | 6.67 µs |
| 16,384 | 10.10 µs | 12.90 µs | 28.87 µs | 71.61 µs | 31.20 µs | 47.35 µs |
| 65,536 | 113.52 µs | 83.14 µs | 147.27 µs | 1.121 ms | 249.22 µs | 324.96 µs |

The native representation therefore beats a foreign GMP boundary for every
measured multiply and wins raw multiplication at 16,384 bits. It does not beat
GMP's large division, so the ledger keeps divide-and-conquer and Barrett marked
partial rather than claiming performance parity. `cargo tree --edges normal`
contains only the Rust `num` family; Rug, GMP, and MPFR remain dev-only.

## Next implementation order

1. Revisit recursive half-GCD only with a lower-overhead reduction kernel. Even
   after matrix products use the retained higher-Toom selector, it remains
   2.06× slower at 262,144 bits and 1.90× slower at 1,048,576 bits.
2. Revisit the exact NTT only with a materially faster modular butterfly
   (Montgomery/Harvey reduction or equivalent); the current kernel remains
   4.5× slower than selected Toom-8 even at 4,194,304 bits.
3. Revisit the Rust-native block-wise Barrett candidate only with a lower-cost
   block kernel: after direct subtraction it measures 5.58 µs versus 2.85 µs
   one-shot, 84.3 µs versus 49.9 µs for sixteen reused 8,192/1,024-bit
   divisions, and 1.51 ms versus
   1.23 ms for sixteen reused 65,536/4,096-bit divisions.
