# Realistic

Realistic is an implementation of Hans Boehm's ["Towards an API for the Real Numbers"](https://dl.acm.org/doi/pdf/10.1145/3385412.3386037) paper

The core idea here is that while by definition almost all Real numbers are non-computable and so we can't expect them to be well behaved in
our computer, many useful Real numbers are quite well behaved if we provide a better API.

The Real type in this crate represents a number which is a composite of a rational (a ratio between two potentially large integers) and some
real value which we may not be able to compute exactly and so must be approximated for display, but which sometimes has a precise description
we can track to take advantage of in further calculations.

The built-in example interactive evaluates [Simple Expressions](#simple-expressions) entered by hand and displays the answer

## Unfinished

This library is incomplete, some features which were whole and useful in the Java API described in the paper are unfinished or partial here.

In particular the trigonometric functions provided in Hans' work are not yet implemented, there is Pi but no Sine etc.

## Unfaithful

To the extent this implementation reflects the intent of the original paper, credit should go to the author of that paper, Hans Boehm.

On the other hand, if you encounter bugs or inadequacies in this code, chances are blame for those lies entirely with me as its programmer

In some places the natural way to express the API in Rust differs from Java and I intend to explain such deviatione below, but for now the
nature of the work often makes that impractical.

Unlike Java, Rust has "Operator overloading" or rather, we can implement many arithmetic operators for user defined types. This means that
where the Java API provides named functions, in many cases the Rust API provides operators, such as + (impl Add) and * (impl Multiply)

I have chosen not to provide this capability for Computable, this might be revisited later

`BoundedRational::to_big_integer` has a different name because in Rust `as` signifies that this conversion is cheap

## Performance

No attempt has been made to measure the current performance of this Rust, compare it against the Java, or against similar code. Although the
code was not specifically written with performance in mind, some care was taken to avoid needless re-calculations even beyond the core idea of
the Computable type.


# Simple Expressions

Parsing for a simple fairly human readable expression syntax is provided, this syntax has a LISP-like structure with each list consisting of
exactly one operator, then one or more sub-expressions consisting of either another list, a numeric literal or a name. The type for a parsed
expression of this type is named Simple.

We can parse these expressions in the usual way, and we can evaluate such an expression.

Currently the provided operators are the + - * and / expected in programming for addition, subtraction, multiplication and division
plus e for the natural exponenitation, l for the natural log, and s or √ for square roots

(l (e 1)) == (e (l 1)) == 1

(s 9) == 3

Numeric literals are written in decimal, 2.75 is exactly two and three quarters.

Today the built-in names are only e (the mathematical constant and base of the natural logarithm) and pi (the mathematical constant π)

However in the built-in evaluator names are also given to the previous answer (last) and all numbered answers (#1, #2 and so on)

(+ 1 2 3 4) == 10

(* 1 2 3 4) == 24

(- 1) == -1  negates the number

(- 1 2) == -1  basic subtraction

(= 1 2 3) == -4  all subsequent arguments are also subtracted

(/ 1) == 1  inverts the number

(/ 1 2) == 0.5  basic division

(/ 1 2 3) == a sixth, all subsequent arguments are also divided
