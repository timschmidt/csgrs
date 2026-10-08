{-# LANGUAGE DataKinds #-}
{-# LANGUAGE ScopedTypeVariables #-}
{-# LANGUAGE RankNTypes #-}
{-# LANGUAGE KindSignatures #-}
{-# LANGUAGE ExistentialQuantification #-}
{-# LANGUAGE StandaloneDeriving #-}
{-# LANGUAGE GeneralizedNewtypeDeriving #-}

{-|
    Module      :  Numeric.AERN.RealArithmetic.Basis.MPFR.Effort
    Description :  MPFR precision is an effort indicator
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    MPFR precision is an effort indicator.

    This is a private module reexported publicly via its parent.
-}

module Numeric.AERN.RealArithmetic.Basis.MPFR.Basics
(
    MPFR(..),
    RoundedTowardInf,
    liftRoundedToMPFR1,
    liftRoundedToMPFR2,
    MPFRPrec,
    defaultPrecision,
    getPrecision,
    samePrecision,
    withPrec,
    withPrecRoundDown
)
where

import Numeric.AERN.Basics.Effort

import qualified Numeric.Rounded as R
import GHC.TypeLits
import Data.Proxy
import Unsafe.Coerce

--import qualified Data.Number.MPFR as M

import Test.QuickCheck

data MPFR = forall p. (R.Precision p) => MPFR (RoundedTowardInf p)

type RoundedTowardInf p = R.Rounded R.TowardInf p

deriving instance Show MPFR

newtype MPFRPrec = MPFRPrec Int
    deriving (Eq, Ord, Show, Enum, EffortIndicator, Num, Real, Integral)

withPrec :: MPFRPrec -> (forall p. (R.Precision p) => R.Rounded R.TowardInf p) -> MPFR
withPrec (MPFRPrec p) computation
    | p < 2 =
        error "MPFR precision has to be at least 2" 
    | otherwise = 
        R.reifyPrecision p (\(_ :: Proxy p) -> MPFR (computation :: R.Rounded R.TowardInf p))

{-|
    This should be needed very rarely, in cases such as to get a lower bound on pi.
-}
withPrecRoundDown :: MPFRPrec -> (forall p. (R.Precision p) => R.Rounded R.TowardNegInf p) -> MPFR
withPrecRoundDown (MPFRPrec p) computation
    | p < 2 =
        error "MPFR precision has to be at least 2" 
    | otherwise = 
        R.reifyPrecision p 
            (\(_ :: Proxy p) -> 
                MPFR (unsafeCoerce (computation :: R.Rounded R.TowardNegInf p) :: R.Rounded R.TowardInf p))

instance Arbitrary MPFRPrec where
    arbitrary =
        do
        p <- choose (10,1000)
        return (MPFRPrec (p :: Int))

defaultPrecision :: MPFRPrec
defaultPrecision = 100

--type DefaultPrecision = 100

getPrecision :: MPFR -> MPFRPrec
getPrecision (MPFR v) = MPFRPrec $ R.precision v

samePrecision :: MPFR -> MPFR -> Bool
samePrecision m1 m2 =
    p1 == p2
    where
    p1 = getPrecision m1
    p2 = getPrecision m2

withSamePrecision ::
    String
    ->
    (forall p. (R.Precision p) => (RoundedTowardInf p) -> (RoundedTowardInf p) -> t) 
    -> 
    MPFR -> MPFR -> t
withSamePrecision context f m1@(MPFR v1) m2@(MPFR v2) 
    | samePrecision m1 m2 = 
        f v1 (unsafeCoerce v2)
    | otherwise =
        error $ mixPrecisionErrorMessage context m1 m2

mixPrecisionErrorMessage context m1 m2 =
    context ++ ": trying to mix precisions: " ++ show (getPrecision m1) ++ " vs " ++ show (getPrecision m2)

instance Eq MPFR where
    (==) = withSamePrecision "==" (==) 

instance Ord MPFR where
    compare = withSamePrecision "compare" compare 

liftRoundedToMPFR1 :: 
    (forall p. (R.Precision p) => (RoundedTowardInf p) -> (RoundedTowardInf p)) 
    -> 
    MPFR -> MPFR
liftRoundedToMPFR1 f (MPFR v) = (MPFR (f v)) 

liftRoundedToMPFR2 ::
    String
    ->
    (forall p. (R.Precision p) => (RoundedTowardInf p) -> (RoundedTowardInf p) -> (RoundedTowardInf p)) 
    -> 
    MPFR -> MPFR -> MPFR
liftRoundedToMPFR2 context f =
    withSamePrecision context (\ v1 v2 -> MPFR (f v1 v2)) 

instance Num MPFR where
  (+) = liftRoundedToMPFR2 "+" (+)
  (-) = liftRoundedToMPFR2 "-" (-)
  (*) = liftRoundedToMPFR2 "*" (*)
  negate = liftRoundedToMPFR1 negate
--  fromInteger n = withPrec defaultPrecision $ fromInteger n -- useless
  fromInteger _ = error "MPFR type: fromInteger not implemented" 
  abs = liftRoundedToMPFR1 abs
  signum = liftRoundedToMPFR1 signum

instance Fractional MPFR where
  (/) = liftRoundedToMPFR2 "/" (/)
  fromRational _ = error "MPFR type: fromRational not implemented" 

instance Floating MPFR where
  pi    = error "MPFR type: pi not implemented"
  exp   = liftRoundedToMPFR1 exp
  sqrt  = liftRoundedToMPFR1 sqrt
  log   = liftRoundedToMPFR1 log
  sin   = liftRoundedToMPFR1 sin
  tan   = liftRoundedToMPFR1 tan
  cos   = liftRoundedToMPFR1 cos
  asin  = liftRoundedToMPFR1 asin
  atan  = liftRoundedToMPFR1 atan
  acos  = liftRoundedToMPFR1 acos
  sinh  = liftRoundedToMPFR1 sinh
  tanh  = liftRoundedToMPFR1 tanh
  cosh  = liftRoundedToMPFR1 cosh
  asinh = liftRoundedToMPFR1 asinh
  atanh = liftRoundedToMPFR1 atanh
  acosh = liftRoundedToMPFR1 acosh

instance Real MPFR where
    toRational (MPFR v) = toRational v

{- local tests as a proof of concept: -}

testRounded =
    R.reifyPrecision 512 (\(_ :: Proxy p) -> show (logBase 10 2 :: R.Rounded R.TowardNearest p))

a :: MPFR
a = withPrec 100 pi

b :: MPFR
b = withPrec 1000 1

aPrec = getPrecision a

mpfrPlus :: MPFR -> MPFR -> MPFR
mpfrPlus = liftRoundedToMPFR2 "mpfrPlus" (+) 

aPa = a `mpfrPlus` a

aPb = a `mpfrPlus` b -- throws exception


