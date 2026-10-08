# CDAR (cdar-mBound)

Implementation of computable real numbers, which is also often referred to as Exact Real Arithmetic.

Please see [section at the bottom](#variation-mbound-vs-master) how this branch (mBound) differs from the master branch.

## Computable Real Numbers

Computable real numbers form a countable subset of the real numbers. It contains all real numbers that can be described by a finite program.  This includes all numbers that are commonly used in mathematics, including π and e. It is also closed under all field operations and all common transcendental functions such as exponential and trigonometric functions.

### Interval Arithmetic and Computable Real numbers

This implementation used interval arithmetic internally but that doesn't mean that this is _Interval Arithmetic_. Computable real numbers are _exact_, so we are working under the assumption that arbitrary good approximations of input values can be found. This is obviously different from interval arithmetic where one of the main starting points is that input values may not be exact.

### This implementation

Is based on Centred Dyadic Approximations as described in [Blanck 2006](http://cs.swan.ac.uk/~csjens/pdf/centred.pdf).

It is also heavily inspired by the [iRRAM](http://irram.uni-trier.de/) implementation by Norbert Müller. In particular, it uses nested intervals rather than Cauchy sequences with a computable modulus function.

This is implementation should have comparable efficiency to implementations using Cauchy sequences for shallow expressions. However, for deeply nested expressions, such as iterated functions, it should be significantly faster.

Each computable real can be viewed as an infinite list of rapidly shrinking intervals. Thus, laziness of Haskell is used to keep the computations finite.

### Other Related stuff on Hackage

* [**MPFR**](https://hackage.haskell.org/package/hmpfr-0.4.3/docs/Data-Number-MPFR.html) Arbitrary precision floating point numbers with specified rounding modes. While arbitrary precision can be used it is still fixed for a computation so this is still floating point arithmetic, but with larger precision.

* [**AERN2**](https://hackage.haskell.org/package/aern2-real) Computable real numbers and continuous functions using Cauchy sequences. 

  * AERN2 uses on the CDAR type `Approx` as safely-rounded multi-precision floating-point numbers and builds interval arithmetic on top of them.

* [**ireal**](http://hackage.haskell.org/package/ireal) Computable real numbers using Cauchy sequences with a fixed modulus.

* [**constrible**](http://hackage.haskell.org/package/constructible) From the description this appears to be the real closure of the field of rational numbers. This allows for decidable equality, but excludes transcendental functions.

* [**exact-real**](http://hackage.haskell.org/package/exact-real) Computable real numbers using Cauchy sequences with a fixed modulus.

* **ERA** (Can't find a link at the moment) Computable real number using Cauchy sequences with computable modulus.

## Comparison with ireal and AERN2

(https://github.com/michalkonecny/haskell-reals-comparison)

## Installation

Should build under `stack`.

## Motivation

Although the terminology Exact Real Arithmetic promises the ability to compute arbitrarily the result with huge precision, this is not the real strength of this implementation. It is certainly possible to compute 10000 digits of π, and it doesn't take very long. But, even more useful is the fact that if you ask for 10 digits from your resulting computable real number, then you will get 10 correct digits even if the computation has to compute intermediate values with much higher precision.

Main usage cases:
* To provide _exact_ results for important computations.
* To provide numerical computations without having to understand the problems of floating point computations. See [Goldberg 1991](http://dl.acm.org/citation.cfm?id=103163).
* To provide numerical analysis without having to do error analysis.

A word of warning though: Some operations are, by necessity, partial. In particular, comparisons are partial, and so is 1/x near 0.

<!-- ## Examples -->

## Variation mBound vs master

In the mBounds branch, each `Approx` has an additional integer component, called `mBound`, a bound for the number of bits in the integer component `m`.  Thus there is a limit on the bit size of the results of arithmetic operations.
In the original variation, the size of `m` could grow arbitrarily for large numbers.  
`mBound` plays a similar role for `Approx` numbers as mantissa size does for floating-point numbers.
