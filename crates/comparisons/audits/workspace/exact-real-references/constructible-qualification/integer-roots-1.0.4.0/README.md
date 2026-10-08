# integer-roots [![Hackage](http://img.shields.io/hackage/v/integer-roots.svg)](https://hackage.haskell.org/package/integer-roots) [![Stackage LTS](http://stackage.org/package/integer-roots/badge/lts)](http://stackage.org/lts/package/integer-roots) [![Stackage Nightly](http://stackage.org/package/integer-roots/badge/nightly)](http://stackage.org/nightly/package/integer-roots)

Calculating integer roots and testing perfect powers of arbitrary precision.

## Integer square root

The [integer square root](https://en.wikipedia.org/wiki/Integer_square_root)
(`integerSquareRoot`)
of a non-negative integer
$n$
is the greatest integer
$m$
such that
$m\le\sqrt{n}$.
Alternatively, in terms of the
[floor function](https://en.wikipedia.org/wiki/Floor_and_ceiling_functions),
$m = \lfloor\sqrt{n}\rfloor$.

For example,

```haskell
> integerSquareRoot 99
9
> integerSquareRoot 101
10
```

It is tempting to implement `integerSquareRoot` via `sqrt :: Double -> Double`:

```haskell
integerSquareRoot :: Integer -> Integer
integerSquareRoot = truncate . sqrt . fromInteger
```

However, this implementation is faulty:

```haskell
> integerSquareRoot (3037000502^2)
3037000501
> integerSquareRoot (2^1024) == 2^1024
True
```

The problem here is that `Double` can represent only
a limited subset of integers without precision loss.
Once we encounter larger integers, we lose precision
and obtain all kinds of wrong results.

This library features a polymorphic, efficient and robust routine
`integerSquareRoot :: Integral a => a -> a`,
which computes integer square roots by
[Karatsuba square root algorithm](https://hal.inria.fr/inria-00072854/PDF/RR-3805.pdf)
without intermediate `Double`s.

## Integer cube roots

The integer cube root
(`integerCubeRoot`)
of an integer
$n$
equals to
$\lfloor\sqrt[3]{n}\rfloor$.

Again, a naive approach is to implement `integerCubeRoot`
via `Double`-typed computations:

```haskell
integerCubeRoot :: Integer -> Integer
integerCubeRoot = truncate . (** (1/3)) . fromInteger
```

Here the precision loss is even worse than for `integerSquareRoot`:

```haskell
> integerCubeRoot (4^3)
3
> integerCubeRoot (5^3)
4
```

That is why we provide a robust implementation of
`integerCubeRoot :: Integral a => a -> a`,
which computes roots by
[generalized Heron algorithm](https://en.wikipedia.org/wiki/Nth_root_algorithm).

## Higher powers

In spirit of `integerSquareRoot` and `integerCubeRoot` this library
covers the general case as well, providing
`integerRoot :: (Integral a, Integral b) => b -> a -> a`
to compute integer $k$-th roots of arbitrary precision.

There is also `highestPower` routine, which tries hard to represent
its input as a power with as large exponent as possible. This is a useful function
in number theory, e. g., elliptic curve factorisation.

```haskell
> map highestPower [2..10]
[(2,1),(3,1),(2,2),(5,1),(6,1),(7,1),(2,3),(3,2),(10,1)]
```
