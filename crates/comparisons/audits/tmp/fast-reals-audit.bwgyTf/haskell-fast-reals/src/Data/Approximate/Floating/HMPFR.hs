{-# LANGUAGE DataKinds #-}
{-# LANGUAGE ScopedTypeVariables #-}
{- | This module contains a faster implementation of dyadic rationals using
   a fast arbitrary-precision floating-point library MPFR via haskel module Data.Rounded
-}

module Data.Approximate.Floating.HMPFR  where

import Data.Approximate.ApproximateField hiding (prec)
import Data.Number.MPFR as MPFR
import Debug.Trace
import Prelude hiding (isNaN,isInfinite, div)


staged2mpfrRounding RoundUp = Up
staged2mpfrRounding RoundDown = Down

rnd = staged2mpfrRounding . rounding

prec :: Stage -> Precision
prec a = fromInteger (max 2 (toInteger (precision a)))

instance ApproximateField MPFR where
  zero = MPFR.zero

  appFromInteger = fromIntegerA Down 64 -- TODO
  appFromRational_ s r = undefined --(fromRationalA (rnd s) (prec s) r, False) --TODO

  appAdd s = add (rnd s) (prec s)
  appSub s = sub (rnd s) (prec s)
  appMul s = mul (rnd s) (prec s)
  appNeg s = neg (rnd s) (prec s)
  appInv s a = div (rnd s) (prec s) (fromIntegerA Near 2 1) a
  appDiv s = div (rnd s) (prec s)
  appAbs s = undefined -- TODO absD (rnd s) (prec s)


instance DyadicField MPFR where
  posInf = setInf 2 1
  negInf = setInf 2 (-1)
  naN = setNaN 2

  appGetExp a = traceShow (a,r) r
    where r = -fromEnum (getExp a)
  appPrec a = fromEnum $ getPrec a

  appMul2 s = mul2i (rnd s) (prec s)

  isUnordered a b = False -- TODO
