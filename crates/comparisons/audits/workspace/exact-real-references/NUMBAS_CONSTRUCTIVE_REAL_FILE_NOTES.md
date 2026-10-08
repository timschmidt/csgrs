# Numbas constructive-real extension audit notes

Repository pin: `b840893fe89308ee2a3e5167a901295dc5c2851b`.

`README.md` (42 lines) and `constructive-real-numbers.js` (2211 lines) were read completely, line by line. The extension embeds a JavaScript/BigInt port of AOSP/Boehm CReal and wraps it in Numbas JME types and notation.

## Architecture

The scalar is an immutable expression graph whose nodes answer binary approximations at requested precision. `get_appr` keeps one highest-precision cache; `slow_CReal` batches requests in 32-bit increments. Arithmetic propagates guard bits, uses msd discovery for products/inverses, and uses Taylor/range-reduction workers for exp/log/trig. Pi uses AGM with a retained sequence of square-root approximations. Serialization is postfix and deserialization reconstructs the graph. A 3-second per-worker timeout prevents runaway evaluation.

## Runtime probe

A Node VM probe extracted the embedded CReal core and verified 20 decimal places for sqrt(2), pi, and exp(1); zero sign/compare and max also behaved as expected. Numbas browser integration itself was not run because the host framework is not present.

## Findings / transfer decision

* Useful ideas: explicit precision overflow checks, batched `slow_CReal` evaluation, postfix graph serialization, AGM state reuse, and a per-evaluation timeout are all clear operational contracts.
* Risks: comparison/signum of zero intentionally diverges; unbounded no-argument sign/compare loops rely on precision overflow or timeout. `select_CReal` caches a coarse selector sign and can retain unresolved state. `CReal_to_JME` emits malformed `max`/`min` strings (missing a closing parenthesis). `select_CReal` writes `selector_sign = 0` on the positive branch, causing repeated tests. Browser-facing equality fixes precision at `-32` bits, so near-equal values may compare equal by tolerance.
* The implementation duplicates the CRCalc/AOSP design already audited and is JavaScript-specific. Hyper has stronger structured outcomes, cache rescaling, and concurrency tests. **No production Hyper change selected.**

