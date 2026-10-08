# Confirmed open coefficient-width defect

After the retained ln1p repair, a source-derived completeness question was
tested independently. Public `Computable::rational(1/16).asin()` and `.asinh()`
both pass their approximation contract at184,000bits in debug and release.
At192,000bits both debug builds panic on coefficient multiplication; both
release builds return finite values disjoint from the correct error enclosure.
None of the eight probes hit its60second runtime cap. There is no coefficient
repair yet at this checkpoint.

The input is exactly dyadic, strictly inside the tiny-series domain. The oracle
uses directed MPFR at requested precision+256bits (184,256/192,256), verifies
exact input construction, and rejects inconclusive overlap. This is not reuse
of a fixed4096bit oracle beyond its capability. Probe source, binaries, logs and
terminal exit codes are frozen; analyze-series-limit.mjs checks them.

The coefficient formulas `(2*n-1)^2` and `(2*n)*(2*n+1)` currently use i32.
At n=23,171, the first numerator is46,341²=2,147,488,281, above i32::MAX. The
same formulas occur in four recurrences: rational and generic asin in
inverse_trig.rs, rational and generic asinh in inverse_hyperbolic.rs. Public
rational probes confirm the issue; the generic counterparts still need their
own high-precision regression tests. The earlier series-domain repair prevents
misuse for large arguments but cannot prevent overflow at a legitimately large
precision request on a small argument.

Next: widen or otherwise safely form these coefficients, prove bounds for the
supported precision arithmetic, test all four paths against precision-matched
MPFR, and compare ordinary timing/allocation/binary size against the frozen
prior source. Do not treat an arbitrary precision cutoff as a completeness fix.
Other precision/counter boundaries remain subject to the broader audit.

All failures are original-code observations at this checkpoint; no donor code
is involved. The independently qualified log-domain repair is retained and is
not credited with fixing this separate defect.
