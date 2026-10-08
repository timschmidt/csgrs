A library for computing with exact real numbers in Haskell.

## Prerequisites

* A generally working development environment (`make`, `gcc`)
* Probably the [GMP](https://gmplib.org) header files.
* Haskell version 7.8.4, including Cabal
* The [haskell-mpfr](https://github.com/comius/haskell-mpfr) library by [Ivo List](https://github.com/comius).

The MPFR Haskell bindings in the haskell-mpfr library rely on low-level implementation details
of Haskell. It is likely they will break with newer versions of Haskell.

## Installation & Usage

1. Install the haskell-mpfr library, following the instructions there. Essentially, it is just a `cabal install` inside the haskell-mpfr source directory.

2. To install the library, run

        cabal install

   To get a version of the library which allows profiling, use instead

        cabal install --ghc-option=-fprof-auto -p --enable-executable-profiling

3. Alternatively to the previous step, you can run

        cabal repl

   to get an interactive shell with the library preloaded. This way you can try out the
   library without installing it.

4. To create HTML documentation run

        cabal haddock

   The generated HTML files will be placed in `dist/doc/html/`.

## Example

Here is an example session.

    Preprocessing library haskell-fast-reals-0.1.0.0...
    GHCi, version 7.8.4: http://www.haskell.org/ghc/  :? for help
    <... stuff gets loaded ...>
    *Data.Reals.Reals> let a = 1.3 :: RealNum
    *Data.Reals.Reals> a*(1-a)
    [-3.900000000007822e-1,-3.899999999994179e-1]
    *Data.Reals.Reals> a / a
    [9.999999999972715e-1,1.000000000001819]
    *Data.Reals.Reals> let b = 2.7 :: RealNum
    *Data.Reals.Reals> a * a - b * b - (a + b) * (a - b)
    [-3.128661774106285e-12,4.147295840103611e-12]
    *Data.Reals.Reals> a * a
    [1.689999999996871,1.690000000004147]
    *Data.Reals.Reals> approx (a * a) (precUp 100)
    [1.69,1.69]
    λ> let int =  (ClosedInterval (appFromInteger 0,appFromInteger 1))
    λ> forall int $ \x -> (x * (1 - x)) `less` (0.24 :: RealNum )
    False
    λ> forall int $ \x -> (x * (1 - x)) `less` (0.26 :: RealNum )
    True
