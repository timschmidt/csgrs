{-# LANGUAGE DataKinds #-}
{-# LANGUAGE PolyKinds #-}
{-# LANGUAGE KindSignatures #-}
{-# LANGUAGE FlexibleInstances #-}
{-# LANGUAGE UndecidableInstances #-}
-- | This module uses type-level literals to provide
-- types 'Rounded' /lit/ where /lit/ is a positive integer literal. 
-- Values of this type are 'IReal's where (sub-)expressions are computed
-- with a precision of at most /lit/ decimals. This is very different from 
-- multi-precision floating point numbers; 'Rounded' values are intervals,
-- indicating the precision in the computed result.
--
-- To use this module in ghci you must @:set -XDataKinds@. Example usage:
--
-- >>> import Data.Number.IReal.FAD 
-- >>> import Data.Number.IReal.Rounded
-- >>> let f x = cos x * cos (2*x) + sin x * sin (2 * x)
-- >>> :set +s
-- >>> :set -XDataKinds
-- >>> (deriv 200 f 1 :: Rounded 120) ? 40
-- 0.54030230586813971740[| 0803811043 .. 1069403842 |]
-- (0.13 secs, 114501688 bytes)
-- >>> (deriv 200 f 1 :: Rounded 150) ? 40
-- 0.5403023058681397174009366074429766037324
-- (0.13 secs, 120063280 bytes)
--
-- Note that function f is in fact an obfuscated version of the cosine function, using a trigonometric identity
--  on cos (2x - x). So we can check the result, but the derivatives are computed using the rules for differentiation. 
--
-- We compute the 200'th  derivative of f, evaluated at 1 (i.e., cos 1) with 40 significant digits. First we try 
-- to do this at type Rounded 120, i.e. with 120 decimals in all intermediate computations. As we see, this is not 
-- precision enough; we get as result an interval of width circa 2e-22. Redoing it at type Rounded 150 gives 
-- sufficient precision. 
module Data.Number.IReal.Rounded where

import GHC.TypeLits
import Data.Proxy
import Data.Number.IReal as IR
import Data.Number.IReal.IRealOperations
import Data.Number.IReal.Powers
import Data.Ratio

infix 3 ?, ??, =?=, <!, >!
infix 6 +-, -+-

newtype Rounded p = R {unRound :: IReal}

instance Precision p => VarPrec (Rounded p) where
  precB n (R x) = r (precB n x)

instance Precision p => Powers (Rounded p) where

class Precision p where
  precision :: proxy p -> Int

instance KnownNat n => Precision (n :: Nat) where
  precision p = fromInteger (natVal p)

r :: Precision p => IReal -> Rounded p
r x = t where t = R (prec p x)
              p = precision (proxyPrecision t)

proxyPrecision :: Rounded p -> Proxy p
proxyPrecision _ = Proxy

instance Precision p => Num (Rounded p) where
  R x + R y = r (x+y)
  R x - R y = r (x-y)
  R x * R y = r (x*y)
  negate (R x) = r (negate x)
  abs (R x) = r (abs x)
  signum (R x) = r (signum x)
  fromInteger n = R (fromInteger n)

instance Precision p => Fractional (Rounded p) where
  recip (R x) = r (recip x)
  fromRational x = r (fromRational x)

instance Precision p => Show (Rounded p) where
  show x@(R x') = show x'

instance Precision p => Floating (Rounded p) where
  pi = r pi
  sqrt (R x) = r (sqrt x)
  exp (R x) = r (exp x)
  log (R x) = r (log x)
  sin (R x) = r (sin x)
  cos (R x) = r (cos x)
  tan (R x) = r (tan x)
  asin (R x) = r (asin x)
  acos (R x) = r (acos x)
  atan (R x) = r (atan x)
  sinh (R x) = r (sinh x)
  cosh (R x) = r (cosh x)
  tanh (R x) = r (tanh x)
  asinh (R x) = r (asinh x)
  acosh (R x) = r (acosh x)
  atanh (R x) = r (atanh x)
 
instance Precision p => Eq (Rounded p) where
   R x == R y = x == y

instance Precision p => Ord (Rounded p) where
  compare (R x) (R y) = compare x y
  R x < R y = x < y
  R x > R y = x > y
  max (R x) (R y) = r (max x y)
  min (R x) (R y) = r (min x y)

instance Precision p => Scalable (Rounded p) where
  scale (R x) n = r (scale x n)

R x ? n = x IR.? n
 
R x ?? n = x IR.?? n

(=?=), (<!), (>!) :: Precision p => Rounded p -> Rounded p -> Int -> Bool
(=?=) (R x) (R y) b = (x IR.=?= y) b

(<!) (R x) (R y) b = (x IR.<! y) b
(>!) (R x) (R y) b = (x IR.>! y) b
 
(+-) :: Precision p => Rational -> Rational -> Rounded p
x +- y = r (x IR.+- y)

(-+-) :: Precision p => Rounded p -> Rounded p -> Rounded p
R x -+- R y = r (x IR.-+- y)

showIReal :: Precision p => Int -> Rounded p -> String
showIReal d (R x) = IR.showIReal d x

mid, rad, lower, upper :: Precision p => Rounded p -> Rounded p
hull :: Precision p => [Rounded p] -> Rounded p
containedIn :: Precision p => Rounded p -> Rounded p -> Int -> Bool
intersection :: Precision p => Rounded p -> Rounded p -> Maybe (Rounded p)
mid (R x) = r (IR.mid x)
rad (R x) = r (IR.rad x)
lower (R x) = r (IR.lower x)
upper (R x) = r (IR.upper x)
hull xs = r (IR.hull (map unRound xs))
containedIn (R x) (R y) = IR.containedIn x y
intersection (R x) (R y) = case IR.intersection x y of
                              Nothing -> Nothing
                              Just x -> Just (r x)
