You have mostly won the scalar war.

So the next steps should be about turning scalar wins into broader algebra wins, and about protecting the fast paths you already have.

The benchmark pattern now says:

scalar transcendental/special-form work is already excellent, often massively ahead of astro-float 128 and sometimes even ahead of Numerica on the rows that matter most to your design, like structured trig and inverse functions.
ordinary matrix kernels are no longer bad at all: mat3 determinant is about 731 ns, mat3 mul mat3 about 3.00 us, mat4 determinant about 2.20 us, and mat4 mul mat4 about 5.51 us.
the remaining weight is concentrated in inverse / reciprocal / division / negative-power style matrix work, where Hyperreal is still clearly behind astro-float and Numerica.

That suggests the next steps are:

1. Make inverse/division kernels your primary target

This is now the obvious hotspot family.

The rows still carrying the most cost are:

mat3 inverse / reciprocal around 15 us
mat3 div matrix around 19 us
mat4 inverse / reciprocal around 25 us
mat4 div matrix around 29 us.

That means your highest-value optimization work is no longer generic scalar math. It is:

inverse construction
right-division
reciprocal
powi for negative exponents
anything that currently expands cofactors/adjugates and then canonicalizes too much

This is where I would keep using trace-driven work.

2. Push delayed canonicalization further into matrix inverse/division

Your success story so far is basically:

keep structure
defer work
only approximate when forced

The same philosophy should probably dominate inverse-heavy matrix code too.

I would keep asking:

can cofactors stay as signed product sums longer?
can adjugate entries remain scaled by a shared factor longer?
can A / B be implemented as solve-with-shared-structure instead of inverse-then-multiply?
can negative powi reuse the same inverse structure without repeatedly materializing expensive intermediates?
3. Keep building short exact-polynomial primitives

Your instinct about difference_of_products / sum_signed_products is exactly what the benchmark shape suggests.

The reason is that “normal” multiply kernels are already much better, while inverse/division-heavy rows still pay for:

short determinant minors
cofactors
signed polynomial combinations
repeated normalization/gcd pressure

That means the next scalar primitive wins are probably not new transcendental tricks. They are more likely:

a*b - c*d
3-term determinant sums
shared-denominator signed accumulators
common-factor-aware short polynomial evaluation

That is the scalar substrate most likely to move matrix inverse/division.

4. Treat from f64 vs rational as a policy problem, not just a benchmark curiosity

The stack keeps showing that representation choice matters enormously.

Sometimes the from f64 path is best. Sometimes rational is best. That means the next optimization layer is probably not just “make one representation faster,” but:

pick the best representation earlier,
avoid unnecessary promotion,
and preserve dyadic structure aggressively where it exists.

The benchmark reference supports that this is a real lever: exact float conversion into Rational and then Real is already cheap, and many of the top-level wins depend on staying in the right structural regime rather than collapsing too early.

5. Keep guarding the fast scalar trig dispatch paths

Even though scalar trig is now excellent, the run history showed how easy it was for generic trig to regress when structural work leaked into the wrong path.

So I would treat those paths as sacred hot loops:

keep the dispatch trace
keep targeted adversarial trig benches
keep cold vs cached microbenches
and make sure future matrix-oriented work does not accidentally reintroduce scalar overhead

This is now less about squeezing more speed out of trig and more about not losing the absurd wins you already got.

6. Build solver/geometry-facing abstractions only after preserving the cheap paths

You are now at the point where the stack is fast because:

sign
zero-status
magnitude
structural facts
and cheap reductions

are all cheap enough to use pervasively.

That means the next application-layer work for geometry and nonlinear solving should be designed to:

consume those facts,
not wrap them in something that hides them,
and not force early approximation.

So the optimization lesson for architecture is:

any new solver/predicate/geometry layer should expose and exploit cheap structural facts, not abstract them away.

My blunt priority order

If I were you, I’d prioritize like this:

matrix inverse / reciprocal / division / negative-powi
short signed polynomial rational primitives
representation policy: dyadic vs rational vs computable
protect scalar trig fast paths with trace + targeted benches
only then broaden solver/expression infrastructure
The bigger conclusion

What this says, overall, is that the project has crossed a line:

You no longer need to prove that the design can be fast.
The benchmarks already show that it can.

Now the job is to:

move the remaining concentrated hotspots,
preserve the absurd scalar wins,
and make the application-facing layers inherit those advantages instead of erasing them.
