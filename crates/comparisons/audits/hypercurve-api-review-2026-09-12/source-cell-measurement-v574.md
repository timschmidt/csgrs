# Bounded source-cell measurement (V574, unrun)

V568 removed repeated strict hodograph factorization and shared the oriented
fields, but joined continuous-family time remained 5.471→5.521s. It does not
resolve the earlier 3.017→5.371s regression. Its full qualification V571 is
still running; do not call the cache committed yet.

After the interval migration V572/V577 is qualified and committed, instrument
only a frozen copied source. Never insert these diagnostics in production.
The small module in source-cell-measurement-v574.rs accumulates integer counts
and elapsed time; each exact test prints just three summary rows. No parameter,
Real, curve, point, or recursively formatted error is printed.

- In `PreparedFilletCarrier2::parallel`, a bucket-0 Timer covers the existing
  optional direction preparation. This includes exact early-outs and bounded
  fallback, as actually scheduled today.
- In `parallel_pair_centers`, a bucket-1 Timer lives inside each initial axis
  iteration, covering incident bridge preparation, original-curve singularity
  analysis and regular-subrange extraction, ending before pair elimination.
- A bucket-2 Timer lives inside each finite-cell normal selection iteration.
  This measures repeated derivative-side proof separately from root solving.
- A Session guard lives in each of three existing tests: joined continuous
  family, stationary quartic continuous family, stationary full composition.
  Run each in its own child process with `--exact --test-threads=1` so counters
  have a single workload. Retain the whole-process wall time too.

The fixture module and combined V575 copied-source driver are prepared; no
instrumentation or measurement has been executed. Decide implementation from
measured cost rather than claiming the remaining regression is already
explained by source inspection.

Candidate ownership, subject to that evidence: the prepared original parallel
can own lazy strictly certified cells, borrowed by its signed-distance center
supports. The original finite interval and expanded incident bridge must stay
distinct. Cache only successful exact proofs; preserve bounded optional
scheduling, early center exclusion, source/center derivative distinctions,
one-sided stationary ownership, and fresh fallback after unavailable evidence.
Retained cell roots would also avoid rebuilding equivalent parameter authority.
No public curve/component variant or compatibility layer is needed.
