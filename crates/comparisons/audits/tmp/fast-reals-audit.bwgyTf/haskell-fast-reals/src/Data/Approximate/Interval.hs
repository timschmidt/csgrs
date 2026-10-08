{-# LANGUAGE FlexibleInstances, UndecidableInstances #-}

{- | This module defines the interval domain, i.e., the space of
  intervals. Actually, what we define as a /base/ for such a domain
  because our intervals have rational endpoints (to be exact, the
  endpoints are elements of an approximate field). The actual interval
  domains is defined in the module "Reals".
-}

module Data.Approximate.Interval (
  Interval (..),
  width, split,
) where

import Data.Approximate.ApproximateField
--import Debug.Trace


{- | An interval is represented by a lower and upper endpoint. We do
  /not/ require that the lower endpoint be smaller or equal to the
  upper one. In other words, we allow the usual as well as
  back-to-front intervals. This is useful in certain kinds of
  computations.

  A possible optimization: represent an interval with its center point
  and radius, where the radius is not a precise number (has a small
  mantissa). This can save up to 50% of space, but it is not clear how
  to treat back-to-front intervals then. Presumably with negative
  radii, except I have never worked out how to implement interval
  multiplication then. -}

data Interval q = Interval { lower :: q, upper :: q }
  deriving Eq

instance (Show q, Eq q) => Show (Interval q) where
  show Interval{lower=a, upper=b} =
    if a == b
    then show a
    else "[" ++ show a ++ "," ++ show b ++ "]"


{- | We define the implementation of intervals in terms of ApproximateField. -}

instance Ord q => Ord (Interval q) where
  i < j = upper i < lower j
  i > j = j < i
  i <= j = upper i <= lower j


instance DyadicField q => ApproximateField (Interval q) where
  appAdd s a b = Interval { lower = appAdd s (lower a) (lower b),
                              upper = appAdd (anti s) (upper a) (upper b)}

  appSub s a b = Interval { lower = appSub s (lower a) (upper b),
                              upper = appSub (anti s) (upper a) (lower b)}

  appNeg s b = Interval { lower = appNeg s (upper b),
                              upper = appNeg (anti s) (lower b)}

  -- Kaucher multiplication
  appMul s Interval{lower=a,upper=b} Interval{lower=c,upper=d} =
    let negative q = q < zero
        lmul = appMul s
        umul = appMul (anti s)
    in {-traceShow s-} Interval { lower =
                                   case (negative a, negative b, negative c, negative d) of
                                     (True, True, _, True) -> lmul b d
                                     (True, True, _, False) -> lmul a d
                                     (True, False, True, True) -> lmul b c
                                     (True, False, True, False) -> min (lmul a d) (lmul b c)
                                     (True, False, False, True) -> zero
                                     (True, False, False, False) -> lmul a d
                                     (False, True, True, True) -> lmul b d
                                     (False, True, True, False) -> zero
                                     (False, True, False, True) -> max (lmul a c) (lmul b d)
                                     (False, True, False, False) -> lmul a c
                                     (False, False, True, _) -> lmul b c
                                     (False, False, False, _) -> lmul a c
                                      {- if negative a
                                          then if negative b
                                               then if negative d
                                                    then lmul b d
                                                    else lmul a d
                                               else if negative c
                                                    then if negative d
                                                         then lmul b c
                                                         else min (lmul a d) (lmul b c)
                                                    else if negative d
                                                         then zero
                                                         else lmul a d
                                          else if negative b
                                               then if negative c
                                                    then if negative d
                                                         then lmul b d
                                                         else zero
                                                    else if negative d
                                                         then max (lmul a c) (lmul b d)
                                                         else lmul a c
                                          else if negative c
                                               then lmul b c
                                               else lmul a c-}
                                ,
                                   upper = if negative a
                                            then if negative b
                                                 then if negative c
                                                      then umul a c
                                                      else umul b c
                                                 else if negative c
                                                      then if negative d
                                                           then umul a c
                                                           else max (umul a c) (umul b d)
                                                      else if negative d
                                                           then zero
                                                           else umul b d
                                            else if negative b
                                                 then if negative c
                                                      then if negative d
                                                           then umul a d
                                                           else zero
                                                      else if negative d
                                                           then min (umul a d) (umul b c)
                                                           else umul b c
                                                 else if negative d
                                                      then umul a d
                                                      else umul b d}

  appInv s Interval{lower=a, upper=b} =
    let sgn q = compare q zero
        linv = appInv s
        uinv = appInv (anti s)
    in Interval { lower = case (sgn a, sgn b) of
                             (LT, LT) -> linv b
                             (EQ, LT) -> linv b
                             (GT, LT) -> posInf
                             (LT, EQ) -> negInf
                             (EQ, EQ) -> negInf
                             (GT, EQ) -> posInf
                             (LT, GT) -> negInf
                             (EQ, GT) -> negInf
                             (GT, GT) -> linv b,
                  upper = case (sgn a, sgn b) of
                             (LT, LT) -> uinv a
                             (EQ, LT) -> negInf
                             (GT, LT) -> negInf
                             (LT, EQ) -> posInf
                             (EQ, EQ) -> posInf
                             (GT, EQ) -> uinv a
                             (LT, GT) -> posInf
                             (EQ, GT) -> posInf
                             (GT, GT) -> uinv a}

  appDiv s a b = appMul s a (appInv s b)

  appAbs s a = Interval { lower = zero,
                          upper = let q = appNeg s (lower a)
                                      r = upper a
                                  in if q < r then r else q }
  appFromInteger k = Interval { lower = appFromInteger k,
                                  upper = appFromInteger k }

  --TODO
  appFromRational_ s r = (Interval { lower = appFromRational s r,
                                                       upper = appFromRational (anti s) r}, False)

width :: DyadicField q => Interval q -> Int
width Interval{lower=a, upper=b} = if zero == diff then maxBound else  (appGetExp diff) --- (appGetExp sum) + 1
    where diff = appSub (precUp 20) b a
          sum = appAdd (precUp 20) a b

split :: DyadicField q => Interval q -> (Interval q, Interval q)
split Interval{lower=a, upper=b} =
    let c = midpoint a b
    in (Interval {lower=a, upper=c}, Interval {lower=c, upper=b})
