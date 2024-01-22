# Realistic

Realistic is an implementation of Hans Boehm's ["Towards an API for the Real Numbers"](https://dl.acm.org/doi/pdf/10.1145/3385412.3386037) paper

## Unfinished

This library is incomplete, features which were whole and useful in the Java API described in the paper are unfinished or partial here.

## Unfaithful

To the extent this implementation reflects the intent of the original paper, credit should go to the author of that paper, Hans Boehm.

On the other hand, if you encounter bugs or inadequacies in this code, chances are blame for those lies entirely with me as its programmer

In some places the natural way to express the API in Rust differs from Java and I intend to explain such deviatione below, but for now the
hilariously incomplete nature of the work makes that impractical.

`BoundedRational::to_big_integer` has a different name because in Rust `as` signifies that this conversion is cheap
