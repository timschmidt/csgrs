# Realistic

Realistic is an implementation of Hans Boehm's ["Towards an API for the Real Numbers"](https://dl.acm.org/doi/pdf/10.1145/3385412.3386037) paper

The core idea here is that while by definition almost all Real numbers are non-computable and so we can't expect them to be well behaved in
our computer, many useful Real numbers are quite well behaved if we provide a better API.

The Real type in this crate represents a number which is a composite of a rational (a ratio between two potentially large integers) and some
real value which we may not be able to compute exactly and so must be approximated for display, but which sometimes has a precise description
we can track to take advantage of in further calculations.

The built-in example interactive evaluates [Simple Expressions](#simple-expressions) entered by hand and displays the answer

## Unfinished

Most features from the Java are now replicated or have equivalent functionality in realistic, but there is still tidying up to do.

## Unfaithful

To the extent this implementation reflects the intent of the original paper, credit should go to the author of that paper, Hans Boehm.

On the other hand, if you encounter bugs or inadequacies in this code, chances are blame for those lies entirely with me as its programmer

In some places the natural way to express the API in Rust differs from Java and I intend to explain such deviations below. In several places
the Java uses recursion but I expressed the same algorithm with iteration in Rust, sometimes taking the opportunity to incorporate the signal
checking early termination.

### Conversions

Where applicable the Rust conversion APIs (From, Into, TryFrom, TryInto) are made available for these types.

You can convert f32 or f64 (IEEE floating point values) into a Rational or Real. You can convert a Real back to the closest floating point
value. This should round trip correctly, note that these convert to the closest binary fraction *not* to the decimal fraction as displayed.
Conversions from f32 or f64 are fallible, as both NaN and Infinity do not have equivalents in these types.

You can convert i32 or i64 directly into a Real.

### Arithmetic operators on Rational and Real

Unlike Java, Rust has "Operator overloading" or rather, we can implement many arithmetic operators for user defined types. This means that
where the Java API provides named functions, in many cases the Rust API provides operators, such as + (impl Add) and * (impl Multiply)

I have chosen not to provide this capability for Computable, this might be revisited later

### Naming

`BoundedRational::to_big_integer` has a different name because in Rust `as` signifies that this conversion is cheap

Many APIs use Option<T> where the analogous Java relies on nullability, others use Result<Real, Problem> to reflect the possibility of
unrecoverable errors during either parsing or approximation of results.

## Performance

No attempt has been made to measure the current performance of this Rust, compare it against the Java, or against similar code. Although the
code was not specifically written with performance in mind, some care was taken to avoid needless re-calculations even beyond the core idea of
the Computable type.

Because some Computable reals would evaluate forever, you may want to use the Real::abort method before trying to evaluate a Real, this enables
you to hook a signal (for example from a user interface thread) to the evaluation logic, in places which may run forever it will periodically
check for your signal to abort the calculation.


# Simple Expressions

Parsing for a simple fairly human readable expression syntax is provided, this syntax has a LISP-like structure with each list consisting of
exactly one operator, then one or more sub-expressions consisting of either another list, a numeric literal or a name. The type for a parsed
expression of this type is named Simple.

We can parse these expressions in the usual way, and we can evaluate such an expression.

Currently the provided operators are the + - * and / expected in programming for addition, subtraction, multiplication and division
plus e or exp for the natural exponenitation, l or ln for the natural log, pow or ^ for exponentiation, and s, sqrt or √ for square roots.

The cosine (cos) and sine (sin) functions are provided and expressions which reduce to sin() of some rational multiple of pi are tracked.


(ln (exp 1)) == (e (l 1)) == 1

(sqrt 9) == 3

(cos (* pi 2.5)) == 0

Numeric literals are written in decimal, 2.75 is exactly two and three quarters. The underscore is ignored, so 10\_000 means the same as 10000

You may also write a fraction, including improper fractions such as 11/7 or 4/3 and this will be recognised as a rational number

Today the built-in names are only e (the mathematical constant and base of the natural logarithm) and pi (the mathematical constant π)

However in the built-in evaluator names are also given to the previous answer (last) and all numbered answers (#1, #2 and so on)

When results are not integral, scientific notation is used to display the decimal fraction


(+ 1 2 3 4) == 10

(* 1 2 3 4) == 24

(- 1) == -1  negates the number

(- 1 2) == -1  basic subtraction

(= 1 2 3) == -4  all subsequent arguments are also subtracted

(/ 1) == 1  inverts the number

(/ 1 2) == 0.5  basic division

(/ 1 2 3) == a sixth, all subsequent arguments are also divided
