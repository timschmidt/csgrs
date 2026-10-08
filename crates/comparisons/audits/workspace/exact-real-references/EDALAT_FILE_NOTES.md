# Edalat interval-domain prototype audit notes

Repository pin: `474b133ad9cd52d94000efcfcb6febb3722edf2f`.

All three tracked files were read line by line: `README.md` (179 lines), `interval_domain_core.py` (1055), and `test_interval_domain.py` (861), 2,095 lines total. Hashes are in `EDALAT_FILE_INVENTORY.tsv`.

## Runtime evidence

The checked-in Python source cannot run directly: line 855 contains a raw byte `0xB2` in a comment without an encoding declaration, causing Python 3's UTF-8 `SyntaxError`. In-memory execution after replacing that comment byte (no source modification) ran all **32 tests: 32 passed, 0 failed, 0 errors**, with warnings that some approximation sequences are not chains.

## Architecture and risks

The prototype models rationals (including ad-hoc infinities), reverse-inclusion interval domains, cached effective interval sequences, computable reals as interval chains, interval extensions of functions, IFS/dynamical examples, signed-binary/decimal representations, and a simplified probability pushforward. Arithmetic is mostly exact `Fraction` endpoint arithmetic, but transcendental/outward rounding paths convert through binary `float` plus a fixed epsilon.

This is a pedagogical domain-theory framework rather than an exact-real scalar runtime. Float construction uses `Fraction.limit_denominator(1_000_000)`, so it is not exact; NaN is treated as negative infinity; bottom/contains/intersection/union semantics are inconsistent for domain order; multiplication drops infinite endpoint products; sequence validation checks only five terms and permits invalid chains; `to_float` returns an unvalidated midpoint; and pushforward measures are explicitly rough overlap ratios rather than preimage measures. Broad exception swallowing maps arithmetic/domain errors to bottom. The tests include many assertions on approximate floats and a dimension test that returns a raw count, so they do not establish exactness.

**No production Hyper change selected.** Hyper's interval/enclosure and structured domain-error contracts are materially stronger; importing this prototype's float-based outward rounding or bottom semantics would reduce exactness.

