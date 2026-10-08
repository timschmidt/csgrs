{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE UndecidableInstances #-}
{-# LANGUAGE StandaloneDeriving #-}
{-|
    Module      :  Numeric.AERN.RealArithmetic.RefinementOrderRounding.ElementaryFromFieldOps.Exponentiation
    Description :  out-rounded exponentiation using Taylor
    Copyright   :  (c) Michal Konecny, Jan Duracz
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable

    Out rounded exponentiation using Taylor expansion.
-}

module Numeric.AERN.RealArithmetic.RefinementOrderRounding.ElementaryFromFieldOps.Exponentiation where

import qualified 
       Numeric.AERN.RealArithmetic.RefinementOrderRounding 
       as ArithInOut

import qualified 
       Numeric.AERN.RealArithmetic.NumericOrderRounding 
       as ArithUpDn
       (convertUpEff, convertDnEff)

import qualified 
       Numeric.AERN.RefinementOrder 
       as RefOrd

import qualified 
       Numeric.AERN.NumericOrder 
       as NumOrd

import Numeric.AERN.Basics.Effort
import Numeric.AERN.RealArithmetic.ExactOps

import Test.QuickCheck (Arbitrary(..)) -- , arbitrary, vectorOf)

import Control.Monad.ST (ST)

import Control.Applicative

import Debug.Trace
_ = trace

data ExpThinEffortIndicator t =
    ExpThinEffortIndicator
    {
        expeff_arith :: ArithInOut.RoundedRealEffortIndicator t,
        expeff_taylorDeg :: Int -- ^ truncate the Taylor expansion at this degree 
    }

instance 
    (
        Arbitrary (ArithInOut.RoundedRealEffortIndicator e)
    )
    =>
    Arbitrary (ExpThinEffortIndicator e)
    where
    arbitrary =
        ExpThinEffortIndicator <$> arbitrary <*> (fromInt1To10 <$> arbitrary)
        
deriving instance
    (
        Show (ArithInOut.RoundedRealEffortIndicator e)
    )
    =>
    Show (ExpThinEffortIndicator e)

instance
    (
        EffortIndicator (ArithInOut.RoundedRealEffortIndicator e)
    )
    =>
    EffortIndicator (ExpThinEffortIndicator e)
    where
    effortIncrementVariants (ExpThinEffortIndicator e1O e2O) =
        [ExpThinEffortIndicator e1 e2 | 
            (e1, e2) <- effortIncrementVariants (e1O, e2O) ]
    effortRepeatIncrement (ExpThinEffortIndicator i1 i2, ExpThinEffortIndicator j1 j2) = 
        ExpThinEffortIndicator (effortRepeatIncrement (i1, j1)) (effortRepeatIncrement (i2, j2)) 
    effortIncrementSequence (ExpThinEffortIndicator e1O e2O) =
        [ExpThinEffortIndicator e1 e2 | 
            (e1, e2) <- effortIncrementSequence (e1O, e2O) ]
    effortCombine  (ExpThinEffortIndicator i1 i2) (ExpThinEffortIndicator j1 j2) =
        ExpThinEffortIndicator (effortCombine i1 j1) (effortCombine i2 j2)

expThinDefaultEffort :: 
   (ArithInOut.RoundedReal t) 
   =>
   t -> Int -> ExpThinEffortIndicator t
expThinDefaultEffort x deg =
    ExpThinEffortIndicator
    {
        expeff_arith = ArithInOut.roundedRealDefaultEffort x,
        expeff_taylorDeg = deg
    }


expOutThinArg ::
    (ArithInOut.RoundedReal t, Show t) 
    =>
    ExpThinEffortIndicator t ->
    t {-^ @x@ assumed to be a thin approximation -} -> 
    t {-^ @exp(x)@ -}
expOutThinArg eff x =
    -- infinities not handled well by the Taylor formula,
    -- treat them as special cases, adding also 0 for efficiency:
    case (xTooBig, xTooLow, x |>=? (zero sample)) of
        (True, _, _) -> 
            x </\> (plusInfinity sample) -- x almost oo
        (_, True, _) ->
            (zero sample) </\> ((one sample) </> (neg x)) -- x almost -oo
        (_, _, Just True) -> one sample -- x = 0
        _ | excludesPlusInfinity x && excludesMinusInfinity x ->
            expOutViaTaylorForXScaledNearZero
        _ -> -- not equal to infinity but not excluding infinity:
--            unsafePrint
--            (
--                "expOutThinArg: overflow:"
--                ++ "\n x = " ++ show x
--                ++ "\n xTooLow = " ++ show xTooLow
--                ++ "\n xTooBig = " ++ show xTooBig
--                ++ "\n excludesMinusInfinity x = " ++ show (excludesMinusInfinity x)
--                ++ "\n excludesPlusInfinity x = " ++ show (excludesPlusInfinity x)
--            ) $
            (zero sample) </\> (plusInfinity sample)
             -- this is always a valid outer approx
    where

    (xUp, xTooBig) =
        case ArithUpDn.convertUpEff effortToInt sampleI x of
            Just xUp -> (xUp, False)
            _ -> (error "internal error in expOutThinArg", True)
    (xDn, xTooLow) =
        case ArithUpDn.convertDnEff effortToInt sampleI x of
            Just xDn -> (xDn, False)
            _ -> (error "internal error in expOutThinArg", True)
    expOutViaTaylorForXScaledNearZero =
        (expOutViaTaylor degr (x </>| n)) <^> n
        where
        n = -- x / n must fall inside [-1,1] 
            (abs xUp) `max` (abs xDn)
    expOutViaTaylor degr x = -- assuming x inside [-1,1]
        (1::Int) |<+> (te degr (1::Int))
        where
        te steps i =
--            unsafePrint
--            (
--                "expOutThinArg: expOutViaTaylor:"
--                ++ "\n teResult = " ++ show teResult
--                ++ "\n i = " ++ show i
--                ++ "\n x = " ++ show x
--            ) 
            teResult
            where
            teResult 
                | steps > 0 =
                    (x </>| i) <*> (1 |<+> (te (steps - 1) (i + 1)))
                | steps == 0 = 
                    errorBound
            errorBound = 
                (x </>| i) <*> ithDerivBound
            ithDerivBound =
                case (pNonnegNonposEff effortCompare x) of
                    (Just True, _) -> -- x >= 0:
                        (one x) </\> eUp
                    (_, Just True) -> -- x <= 0:
                        recipEDn </\> (one x)
                    _ -> -- near or crossing zero:
                        recipEDn </\> eUp
            eUp =
                ArithInOut.convertOutEff effortFromDouble sample (2.718281829 :: Double)
            recipEDn =
                ArithInOut.convertOutEff effortFromDouble sample (0.367879440 :: Double)

    (|>=?) = RefOrd.pGeqEff effortRefinement
    (</\>) = RefOrd.meetOutEff effortMeet
    (</>) = ArithInOut.divOutEff divInOutEffort
    (<*>) = ArithInOut.multOutEff multInOutEffort
    (<^>) = ArithInOut.powerToNonnegIntOutEff intPowerInOutEffort
    (</>|) = ArithInOut.mixedDivOutEff mixedDivInOutEffort
    (|<+>) = flip $ ArithInOut.mixedAddOutEff mixedAddInOutEffort

    addInOutEffort = ArithInOut.fldEffortAdd x effortField
    multInOutEffort = ArithInOut.fldEffortMult x effortField
    intPowerInOutEffort = ArithInOut.fldEffortPow x effortField
    divInOutEffort = ArithInOut.fldEffortDiv x effortField
    mixedAddInOutEffort = ArithInOut.mxfldEffortAdd x xUp effortMixedField
    mixedMultInOutEffort = ArithInOut.mxfldEffortMult x xUp effortMixedField
    mixedDivInOutEffort = ArithInOut.mxfldEffortDiv x xUp effortMixedField
    
    effortField = ArithInOut.rrEffortField sample effR
    effortMixedField = ArithInOut.rrEffortIntMixedField sample effR
    effortMeet = ArithInOut.rrEffortJoinMeet sample effR
    effortRefinement = ArithInOut.rrEffortRefComp sample effR
    effortCompare = ArithInOut.rrEffortNumComp sample effR
    effortToInt = ArithInOut.rrEffortToInt sample effR
    effortFromDouble = ArithInOut.rrEffortFromDouble sample effR
    
    effR = expeff_arith eff
    degr = expeff_taylorDeg eff
    
    sample = x
    sampleI = 0 :: Int

