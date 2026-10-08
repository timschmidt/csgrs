{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE ScopedTypeVariables #-}
{-|
    Module      :  Numeric.AERN.RmToRn.RefinementOrderRounding.BernsteinPoly
    Description :  approximations of Bernstein polynomials  
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Approximations of Bernstein polynomials.
-}

module Numeric.AERN.RmToRn.RefinementOrderRounding.BernsteinPoly where

import Numeric.AERN.RmToRn.Domain (HasDomainBox(..))

import Numeric.AERN.RealArithmetic.ExactOps (HasOne(..))
import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut

--import Numeric.AERN.Misc.IntegerArithmetic

import Debug.Trace
_ = trace

bernsteinOut ::
    (HasDomainBox f, HasOne f,
     ArithInOut.RoundedRing f,
     ArithInOut.RoundedMixedMultiply f (Domain f),
     ArithInOut.RoundedReal (Domain f), 
     Show f, Show (Domain f)) 
     =>
    (ArithInOut.RingOpsEffortIndicator f,
     ArithInOut.RoundedRealEffortIndicator (Domain f),
     ArithInOut.MixedMultEffortIndicator f (Domain f)) 
    {-^ Effort indicators to guide the effort used for approximately computing the Bernstein polynomial -} ->
    f {-^ @x@ - a function to substitute in the Bernstein polynomial  -} ->
    Int {-^ @n@ - the degree of the Bernstein polynomial -} ->
    Int {-^ @p@ - the index of the Bernstein polynomial  -} ->
    f {-^ @b_{p,n}(x)@ - The Bernstein polynomial of degree @n@ and index @p@ -}
bernsteinOut (effRingF, effRealD, effMultFD) x n p =
--    trace
--    (
--        "bernsteinOut:"
--        ++ "\n x = " ++ show x
--        ++ "\n n = " ++ show n
--        ++ "\n p = " ++ show p
--        ++ "\n (n p) = " ++ show binomialOut
--        ++ "\n result = " ++ show result
--    )
    result
    where
    result = 
        (binomialOut) .<*>~
            ((x ~<^> p) ~<*>~ ((c1 ~<->~ x) ~<^> (n-p)))
    binomialOut 
        | 0 <= p && p <= n =
            productNdownP .</>. factorialP
        | otherwise = 
            error $ 
                "bernsteinOut called with illegal parameters:" 
                ++ "\n n = " ++ show n 
                ++ "\n p = " ++ show p 
        where
        factorialP = 
            foldl (.<*>|) (one sampleD) [2..p]
        productNdownP = 
            foldl (.<*>|) (one sampleD) [(n-p+1)..n]

    c1 = (one sampleF)
    sampleF = x
    sampleD = getSampleDomValue sampleF

--    (~<+>~) = ArithInOut.addOutEff effAddF
    (~<->~) = ArithInOut.subtrOutEff effAddF
    (~<*>~) = ArithInOut.multOutEff effMultF
    (~<^>) = ArithInOut.powerToNonnegIntOutEff effPowF
    (.<*>~) = flip $ ArithInOut.mixedMultOutEff effMultFD

    (.</>.) = ArithInOut.divOutEff effDivD
    (.<*>|) = ArithInOut.mixedMultOutEff effMultIntD

    effAddF = ArithInOut.ringEffortAdd sampleF effRingF
    effMultF = ArithInOut.ringEffortMult sampleF effRingF
    effPowF = ArithInOut.ringEffortPow sampleF effRingF
    effDivD = 
        ArithInOut.fldEffortDiv sampleD $ 
            ArithInOut.rrEffortField sampleD effRealD
    effMultIntD = 
        ArithInOut.mxfldEffortMult sampleD (1::Int) $ 
            ArithInOut.rrEffortIntMixedField sampleD effRealD
    
    
    