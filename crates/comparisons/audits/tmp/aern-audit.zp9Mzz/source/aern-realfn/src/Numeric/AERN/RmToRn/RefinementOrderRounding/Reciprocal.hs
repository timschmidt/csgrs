{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE ScopedTypeVariables #-}
{-|
    Module      :  Numeric.AERN.RmToRn.RefinementOrderRounding.Reciprocal
    Description :  approximate pointwise reciprocal  
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Approximate pointwise reciprocal of a function
-}

module Numeric.AERN.RmToRn.RefinementOrderRounding.Reciprocal 
(
    recipOutUsingTauEff
)
where

import Numeric.AERN.RmToRn.RefinementOrderRounding.ChebyshevPoly (chebyshevOut)

import Numeric.AERN.RmToRn.Domain (HasDomainBox(..))
import Numeric.AERN.RmToRn.Evaluation (CanEvaluate(..), evalAtPointOut)

import Numeric.AERN.RealArithmetic.ExactOps (HasOne(..), HasZero(..), neg)
import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut

import qualified Numeric.AERN.RefinementOrder as RefOrd
import qualified Numeric.AERN.NumericOrder as NumOrd

import Numeric.AERN.Basics.Exception (AERNException(..))
import Control.Exception (throw)

import Debug.Trace
_ = trace

{-|
   Pointwise reciprocal of a function, outwards rounded.
-}
recipOutUsingTauEff ::
    (HasOne f,
     CanEvaluate f, 
     ArithInOut.RoundedRing f, 
     ArithInOut.RoundedMixedField f (Domain f),
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     Show f, Show (Domain f)) 
     =>
    (ArithInOut.RingOpsEffortIndicator f,
     ArithInOut.RoundedRealEffortIndicator (Domain f),
     ArithInOut.MixedFieldOpsEffortIndicator f (Domain f)) 
    {-^ Effort indicators to guide the effort used for approximately computing the reciprocal -} ->
    Int {-^ @k@ - Degree of tau method to use -} ->
    f {-^ @f@ - A function to compute an approximate pointwise reciprocal for; should be thin  -} ->
    f {-^ An outwards approximation of @1/f@ over the same domain as $f$ -}
{-
   The implementation of this function is described at http://michalkonecny.github.io/aern/_site/posts/division.html
   
   The variable names used below correspond to names used in the above page.
-}
recipOutUsingTauEff (effRingF, effRealD, effFldFD) kOrig fOrig =
    case True of
        _ | (bOrig NumOrd.>? d0) == Just True -- f is entirely positive 
            -> stage2 (bOrig, cOrig) fOrig
        _ | (cOrig NumOrd.<? d0) == Just True -- f is entirely negative 
            -> neg $ stage2 (neg cOrig, neg bOrig) (neg fOrig) -- switch sign and call itself
        _ | otherwise
            -> throw $ 
                    AERNMaybeDomViolationException 
                    "Potential division by zero.  (Pointwise reciprocal called for a function that may contain zero in its range.)"
    where
    (bOrig,cOrig) = RefOrd.getEndpointsOut rangeOfF
        where
        rangeOfF = evalAtPointOut domboxOfF fOrig -- TODO: add explicit evaluation effort
        domboxOfF = getDomainBox fOrig

    k = max 2 kOrig

    stage2 (b,c) f = 
--        trace
--        (
--            "recipOut stage2:"
--            ++ "\n f = " ++ show f
--            ++ "\n b = " ++ show b
--            ++ "\n c = " ++ show c
--            ++ "\n d = " ++ show d
--            ++ "\n x = " ++ show x
--            ++ "\n rangeOfX = " ++ show (evalAtPointOut (getDomainBox x) x)
--            ++ "\n k = " ++ show k
--            ++ "\n coeffsPre = " ++ show coeffsPre
--            ++ "\n tau = " ++ show tau
--            ++ "\n coeffs = " ++ show coeffs
--            ++ "\n pmTauErrorBound = " ++ show pmTauErrorBound
--            ++ "\n 1/f = " ++ (show $ result)
--        )
        result -- TODO: check that the computation of coeffs has not overflown
        where
        result = yOfX ~<*>. twoOverCminusB
    
        -- x as in equation (2):
        x = (twoOverCminusB .<*>~ f) ~<+>. (neg d)
        
        yOfX = pOfX ~<+>. pmTauErrorBound
        
        -- the polynomial from equation (5):
        pOfX = foldl1 (~<+>~) $ zipWith (.<*>~) coeffs $ chebyshevOut effRingF x
        
        -- error bound as given by equation (4):
        pmTauErrorBound = 
            (neg tauErrorBound) RefOrd.</\> (tauErrorBound) 
            -- = [-|\tau|/(d-1), |\tau|/(d-1)]
        tauErrorBound = tau .</>. (d .<->. d1) -- = \tau / (d - 1)
    
        -- constants that appear in equation (1):
        d = d1 .<+>. (twoOverCminusB .<*>. b) -- = 1 + 2b/(c-b)
        twoOverCminusB = d2 .</>. cMinusB -- = 2/(c-b)
        cMinusB = c .<->. b
        
        -- the coefficients and tau as a solution of the system in equation (6):
        cKPre = d2 -- = 2 (Assume tau = 1 until we know what it really is.  Coefficients with \tau = 1 have Pre postfix.)
        cKminus1Pre = neg $ d2 .<*>. d .<*>. cKPre -- = -2dc_k
        coeffsKto1Pre = getNextCoeff k cKPre cKminus1Pre
            where
            getNextCoeff i cI cIminus1 
                | i > 2 = cI : getNextCoeff (i-1) cIminus1 cIminus2
                | i == 2 = [cI, cIminus1]
                | otherwise = error $ "recipOut: internal error: k has to be at least 2"
                where
                cIminus2 = neg $ (d2 .<*>. d .<*>. cIminus1) .<+>. cI 
                -- c_{i-2} = - 2dc_{i-1} - c_i
        coeffs1ToKPre = reverse coeffsKto1Pre
        (c1Pre : c2Pre : _) = coeffs1ToKPre
        c0Pre = neg $ (d .<*>. c1Pre) .<+>. (c2Pre .</>. d2) -- = - dc_1 - c_2/2
        coeffsPre = c0Pre : coeffs1ToKPre
        
        -- now we compute \tau using the last equation in (6):
        tau = d1 .</>. ((d .<*>. c0Pre) .<+>. (c1Pre .</>. d2))
            -- = 1 / (dc_0 + c_1/2)
        
        -- adjust the coefficients using the correct tau:
        coeffs = map (.<*>. tau) coeffsPre
        
    -- simple constant numbers:
    d0 = (zero sampleD)
    d1 = (one sampleD)
    d2 = d1 .<+>. d1
    
    -- field operations with the specified effort:
    
    (~<+>~) = ArithInOut.addOutEff effAddF
--    (~<*>~) = ArithInOut.multOutEff effMultF

    (.<*>~) = flip (~<*>.)
    (~<*>.) = ArithInOut.mixedMultOutEff effMultFD
    (~<+>.) = ArithInOut.mixedAddOutEff effAddFD

    (.<->.) = ArithInOut.subtrOutEff effAddD
    (.<+>.) = ArithInOut.addOutEff effAddD
    (.<*>.) = ArithInOut.multOutEff effMultD
    (.</>.) = ArithInOut.divOutEff effDivD

    -- more specific effort indicators:

    effAddF = ArithInOut.ringEffortAdd sampleF effRingF
--    effMultF = ArithInOut.ringEffortMult sampleF effRingF
    
    effAddFD = ArithInOut.mxfldEffortAdd sampleF sampleD effFldFD
    effMultFD = ArithInOut.mxfldEffortMult sampleF sampleD effFldFD
    
    effAddD = ArithInOut.fldEffortAdd sampleD effFldD
    effMultD = ArithInOut.fldEffortMult sampleD effFldD
    effDivD = ArithInOut.fldEffortDiv sampleD effFldD
    effFldD = ArithInOut.rrEffortField sampleD effRealD 
    
    sampleF = fOrig
    sampleD = getSampleDomValue sampleF

    
    