{-# LANGUAGE DataKinds #-}
{-# LANGUAGE ScopedTypeVariables #-}
{- | This module contains a faster implementation of dyadic rationals using
   a fast arbitrary-precision floating-point library MPFR via haskel module Data.Rounded
-}

module Data.Approximate.Floating.MPFR (Rounded)
where

import Data.Approximate.ApproximateField hiding (prec)
import Data.Approximate.MPFRLowLevel as MPFR hiding (max)
--import Debug.Trace
import Prelude hiding (isNaN,isInfinite, div)


staged2mpfrRounding RoundUp = Up
staged2mpfrRounding RoundDown = Down

rnd = staged2mpfrRounding . rounding

prec :: Stage -> Precision
prec a = fromInteger (max 2 (toInteger (precision a)))

instance ApproximateField Rounded where
  zero = MPFR.zero

  appFromInteger = fromIntegerA Down 64 -- TODO
  appFromRational_ s r = (fromRationalA (rnd s) (prec s) r, False) --TODO

  appAdd s = add (rnd s) (prec s)
  appSub s = sub (rnd s) (prec s)
  appMul s = mul (rnd s) (prec s)
  appNeg s = neg (rnd s) (prec s)
  appInv s a = div (rnd s) (prec s) (fromIntegerA Near 2 1) a
  appDiv s = div (rnd s) (prec s)
  appAbs s = undefined -- TODO absD (rnd s) (prec s)


instance DyadicField Rounded where
  posInf = MPFR.posInf
  negInf = MPFR.negInf
  naN = MPFR.nan

  appGetExp a = {-traceShow (a,r)-} r
    where r = -fromEnum (getExp a)
  appPrec a = getPrec a

  appMul2 s = mul2i

  isUnordered a b = False -- TODO
