{- | This module contains a purely Haskell implementation of dyadic rationals, suitable
   for interval arithmetic. A faster implementation of dyadic rationals would use
   a fast arbitrary-precision floating-point library, such as MPFR and the related
   hmpfr Haskell bindings for it.

   A dyadic number is a rational whose denominator is a power of 2. We also include
   positive and negative infinity, these are useful for representing infinite intervals.
   The special value 'NaN' (not a number) is included as well in order to follow more closely
   the usual floating-point format, but is not used in the interval computations because
   there we use [-inf, +inf] to represent the completely undefined value.
-}

module Data.Approximate.Floating.Dyadic (
  Dyadic(..),
) where

import Data.Bits
import Data.Approximate.ApproximateField


-- | A dyadic number is of the form @m * 2^e@ where @m@ is the /mantissa/ and @e@ is the /exponent/.
data Dyadic = Dyadic { mant :: Integer, expo :: Int }
            | PositiveInfinity
            | NegativeInfinity
            | NaN -- ^ not a number, result of undefined arithmetical operation

-- | This should be improved so that dyadics are shown in the usual
-- decimal notation. The trouble is: how many digits should we show?
-- MPFR does something reasonable, maybe we can do the same thing
-- here.
instance Show Dyadic where
  show PositiveInfinity = "+inf"
  show NegativeInfinity = "-inf"
  show NaN = "NaN"
  show Dyadic {mant=m, expo=e} = show m ++ "*2^" ++ show e

-- | Suppose @g@ is a map of two dyadic arguments which is invariant
-- under multiplication by a power of two, i.e., @g x y = g (x * 2^e)
-- (y * 2^e)@. Then @g@ is already determined by its action on
-- integers. The map 'shifted2' takes such a @g@ restricted to the
-- integers and extends it to dyadics.
shifted2 :: (Integer -> Integer -> a) -> Dyadic -> Dyadic -> a
shifted2 f (Dyadic {mant=m1, expo=e1}) (Dyadic {mant=m2, expo=e2}) =
  case compare e1 e2 of
    LT -> f m1 (shiftL m2 (e2-e1))
    EQ -> f m1 m2
    GT -> f (shiftL m1 (e1-e2)) m2

-- | zeroCmp q returns the same thing as compare 0 q
zeroCmp :: Dyadic -> Ordering
zeroCmp NegativeInfinity = GT
zeroCmp PositiveInfinity = LT
zeroCmp Dyadic {mant=m, expo=e} = compare 0 m

instance Eq Dyadic where
  PositiveInfinity == PositiveInfinity = True
  NegativeInfinity == NegativeInfinity = True
  a@(Dyadic _ _)   == b@(Dyadic _ _)   = shifted2 (==) a b
  _                == _                = False

  PositiveInfinity /= PositiveInfinity = False
  NegativeInfinity /= NegativeInfinity = False
  a@(Dyadic _ _)   /= b@(Dyadic _ _)   = shifted2 (/=) a b
  _                /= _                = True

instance Ord Dyadic where
  compare NegativeInfinity NegativeInfinity = EQ
  compare NegativeInfinity _                = LT
  compare _                NegativeInfinity = GT
  compare PositiveInfinity PositiveInfinity = EQ
  compare PositiveInfinity _                = GT
  compare _                PositiveInfinity = LT
  compare a@(Dyadic _ _)   b@(Dyadic _ _)   = shifted2 compare a b

instance Num Dyadic where
  -- addition
  NaN + _ = NaN
  _ + NaN = NaN
  NegativeInfinity + PositiveInfinity = NaN
  PositiveInfinity + NegativeInfinity = NaN
  NegativeInfinity + _ = NegativeInfinity
  _ + NegativeInfinity = NegativeInfinity
  PositiveInfinity + _ = PositiveInfinity
  _ + PositiveInfinity = PositiveInfinity
  Dyadic {mant=m1, expo=e1} + Dyadic {mant=m2, expo=e2} = Dyadic {mant = m3, expo = e3}
      where m3 = if e1 < e2 then m1 + shiftL m2 (e2 - e1) else shiftL m1 (e1 - e2) + m2
            e3 = min e1 e2

  -- subtraction
  NaN - _ = NaN
  _ - NaN = NaN
  NegativeInfinity - NegativeInfinity = NaN
  PositiveInfinity - PositiveInfinity = NaN
  NegativeInfinity - _ = NegativeInfinity
  _ - NegativeInfinity = PositiveInfinity
  PositiveInfinity - _ = PositiveInfinity
  _ - PositiveInfinity = NegativeInfinity
  Dyadic {mant=m1, expo=e1} - Dyadic {mant=m2, expo=e2} = Dyadic {mant = m3, expo = e3}
      where m3 = if e1 < e2 then m1 - shiftL m2 (e2 - e1) else shiftL m1 (e1 - e2) - m2
            e3 = min e1 e2

  -- multiplication
  NaN * _ = NaN
  _ * NaN = NaN
  NegativeInfinity * q = case zeroCmp q of
                           LT -> NegativeInfinity -- 0 < q
                           EQ -> 0                -- 0 == q
                           GT -> PositiveInfinity -- q < 0
  PositiveInfinity * q = case zeroCmp q of
                           LT -> PositiveInfinity -- 0 < q
                           EQ -> 0                -- 0 == q
                           GT -> NegativeInfinity -- q < 0
  q@(Dyadic _ _) * NegativeInfinity = NegativeInfinity * q
  q@(Dyadic _ _) * PositiveInfinity = PositiveInfinity * q
  Dyadic {mant=m1, expo=e1} * Dyadic {mant=m2, expo=e2} = Dyadic {mant = m1 * m2, expo = e1 + e2}

  -- absolute value
  abs NaN = NaN
  abs PositiveInfinity = PositiveInfinity
  abs NegativeInfinity = NegativeInfinity
  abs Dyadic {mant=m, expo=e} = Dyadic {mant = abs m, expo = e}

  -- signum
  signum NaN = NaN
  signum PositiveInfinity = 1
  signum NegativeInfinity = -1
  signum Dyadic {mant=m, expo=e} = fromInteger (signum m)

  -- fromInteger
  fromInteger i = Dyadic {mant = i, expo = 0}


-- | This was taken from
-- | <http://www.haskell.org/pipermail/haskell-cafe/2008-February/039640.html>
-- | and it computes the integral logarithm in given base.
ilogb :: Integer -> Integer -> Int
ilogb b n | n < 0      = ilogb b (- n)
          | n < b      = 0
          | otherwise  = up b n 1 - 1
  where up b n a = if n < (b ^ a)
                      then bin b (quot a 2) a
                      else up b n (2*a)
        bin b lo hi = if (hi - lo) <= 1
                         then hi
                         else let av = quot (lo + hi) 2
                              in if n < (b ^ av)
                                    then bin b lo av
                                    else bin b av hi


normalize s NaN = case rounding s of
                      RoundDown -> NegativeInfinity
                      RoundUp -> PositiveInfinity
normalize s PositiveInfinity = PositiveInfinity
normalize s NegativeInfinity = NegativeInfinity
normalize s a@(Dyadic {mant=m, expo=e}) =
      let j = ilogb 2 m
          k = precision s
          r = rounding s
      in  if j <= k
          then a
          else Dyadic {mant = shift_with_round r (j-k) m, expo = e + (j-k) }
      where shift_with_round r k x =
                       let y = shiftR x k
                       in case r of
                         RoundDown -> if signum y > 0 then y else succ y
                         RoundUp -> if signum y > 0 then succ y else y


{- | Dyadics with normalization and rounding form an "approximate"
  field in which operations can be performed up to a given precision.

  We take the easy route: first we perform an exact operation then we
  normalize the result. A better implementation would directly compute
  the approximation, but it's probably not worth doing this with
  Dyadics. If you want speed, use hmpfr, see
  <http://hackage.haskell.org/package/hmpfr>.
-}

instance ApproximateField Dyadic where
  zero = Dyadic {mant=0, expo=1}



{-  toFloat NaN = 0.0 / 0.0
  toFloat PositiveInfinity = 1.0 / 0.0
  toFloat NegativeInfinity = - 1.0 / 0.0
  toFloat Dyadic{mant=m, expo=e} = encodeFloat m e-}

{-  midpoint NaN _ = NaN
  midpoint _ NaN = NaN
  midpoint NegativeInfinity NegativeInfinity = NegativeInfinity
  midpoint NegativeInfinity PositiveInfinity = zero
  midpoint NegativeInfinity Dyadic{mant=m, expo=e} = Dyadic {mant = -1 - abs m, expo= 2 * max 1 e}
  midpoint PositiveInfinity NegativeInfinity = zero
  midpoint PositiveInfinity PositiveInfinity = PositiveInfinity
  midpoint PositiveInfinity Dyadic{mant=m, expo=e} = Dyadic {mant = 1 + abs m, expo= 2 * max 1 e}
  midpoint Dyadic{mant=m,expo=e} NegativeInfinity = Dyadic {mant = -1 - abs m, expo= 2 * max 1 e}
  midpoint Dyadic{mant=m,expo=e} PositiveInfinity = Dyadic {mant = 1 + abs m, expo= 2 * max 1 e}
  midpoint Dyadic{mant=m1,expo=e1} Dyadic{mant=m2,expo=e2} = Dyadic {mant = m3, expo = e3 - 1}
    where m3 = if e1 < e2 then m1 + shiftL m2 (e2 - e1) else shiftL m1 (e1 - e2) + m2
          e3 = min e1 e2
-}

  appAdd s a b = normalize s (a + b)
  appSub s a b = normalize s (a - b)
  appMul s a b = normalize s (a * b)
  appNeg s = negate
  appAbs s a = normalize s (abs a)
--  app_signum s a = normalize s (signum a)-}
  appFromInteger i = fromInteger i
  appFromRational_ r = undefined

  appInv s NaN = normalize s NaN
  appInv s PositiveInfinity = zero
  appInv s NegativeInfinity = zero
  appInv s Dyadic{mant=m, expo=e} =
    let d = precision s
        b = ilogb 2 m
        r = case rounding s of
              RoundDown -> 0
              RoundUp -> 1
    in if signum m == 0
       then normalize s NaN
       else Dyadic {mant = r + (shiftL 1 (d + b)) `div` m, expo = -(b + d + e)}

  appDiv s Dyadic{mant=m1,expo=e1} Dyadic{mant=m2,expo=e2} =
      let e = precision s
          r = case rounding s of
                RoundDown -> 0
                RoundUp -> 1
      in if signum m2 == 0
      then normalize s NaN
      else Dyadic {mant = r + (shiftL 1 e * m1) `div` m2, expo = e1 - e2 - e}
  appDiv s _ _ = normalize s NaN -- can we do better than this in other cases?



instance DyadicField Dyadic where
  posInf = PositiveInfinity
  negInf = NegativeInfinity
  naN = NaN

  isUnordered NaN _ = True
  isUnordered _ NaN = True
  isUnordered _ _ = False

  appMul2 s NaN k = normalize s NaN
  appMul2 s PositiveInfinity k = PositiveInfinity
  appMul2 s NegativeInfinity k = NegativeInfinity
  appMul2 s Dyadic {mant=m, expo=e} k = normalize s Dyadic {mant = m, expo = e + k}

  appPrec NaN = 0
  appPrec PositiveInfinity = 0
  appPrec NegativeInfinity = 0
  appPrec Dyadic{mant=m, expo=e} = ilogb 2 m

  appGetExp NaN = error "log2 of NaN"
  appGetExp PositiveInfinity = error "log2 of +inf"
  appGetExp NegativeInfinity = error "log2 of -inf"
  appGetExp Dyadic{mant=m, expo=e} = e + ilogb 2 m
