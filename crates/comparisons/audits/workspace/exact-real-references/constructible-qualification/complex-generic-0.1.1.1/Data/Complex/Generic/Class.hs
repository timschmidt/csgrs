{-# LANGUAGE MultiParamTypeClasses, FunctionalDependencies #-}
{- |
Module      :  Data.Complex.Generic.Class
Copyright   :  (c) Claude Heiland-Allen 2012
License     :  BSD3

Maintainer  :  claude@mathr.co.uk
Stability   :  unstable
Portability :  MultiParamTypeClasses, FunctionalDependencies

Classes for complex number operations.
-}
module Data.Complex.Generic.Class where

-- | Rectangular form.
class ComplexRect c r | c -> r where
  -- | Construction.
  mkRect :: r {- ^ real -} -> r {- ^ imaginary -} -> c
  -- | Construction with imagPart 0.
  real :: r {- ^ real -} -> c
  -- | Construction with realPart 0.
  imag :: r {- ^ imaginary -} -> c
  -- | Deconstruction.
  rect :: c -> (r, r)
  -- | Get the real part.
  realPart :: c -> r
  -- | Get the imaginary part.
  imagPart :: c -> r
  -- | Conjugation.
  conjugate :: c -> c
  -- | Squared magnitude.
  magnitudeSquared :: c -> r
  -- | Complex square.
  sqr :: c -> c
  -- | Real-complex multiplication.
  (.*) :: r -> c -> c
  infixl 7 .*
  -- | Complex-real multiplication.
  (*.) :: c -> r -> c
  infixl 7 *.

-- | Complex-real division.
(/.) :: (Fractional r, ComplexRect c r) => c -> r -> c
z /. a = z *. recip a
infixl 6 /.

-- | A synonym for 'mkRect'.
(.+) :: ComplexRect c r => r -> r -> c
(.+) = mkRect
infix 6 .+

-- | Polar form.
class ComplexPolar c r | c -> r where
  -- Construction.
  mkPolar :: r -> r -> c
  -- Construction with magnitude 1.
  cis :: r -> c
  -- Deconstruction.
  polar :: c -> (r, r)
  -- | Magnitude.
  magnitude :: c -> r
  -- | Phase in (-pi, pi].
  phase :: c -> r
