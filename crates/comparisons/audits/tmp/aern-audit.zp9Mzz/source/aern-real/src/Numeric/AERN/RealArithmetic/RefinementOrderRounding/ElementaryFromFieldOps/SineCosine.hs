{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE UndecidableInstances #-}
{-# LANGUAGE StandaloneDeriving #-}
{-# LANGUAGE TypeFamilies #-}
{-|
    Module      :  Numeric.AERN.RealArithmetic.RefinementOrderRounding.ElementaryFromFieldOps.SineCosine
    Description :  sine and cosine using Taylor, outwards rounded
    Copyright   :  (c) Michal Konecny, Jan Duracz
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable

    Outwards rounded sine and cosine using Taylor expansion.
-}

module Numeric.AERN.RealArithmetic.RefinementOrderRounding.ElementaryFromFieldOps.SineCosine where

import qualified 
       Numeric.AERN.RealArithmetic.RefinementOrderRounding 
       as ArithInOut
import Numeric.AERN.RealArithmetic.RefinementOrderRounding.Operators

import qualified 
       Numeric.AERN.RealArithmetic.NumericOrderRounding 
       as ArithUpDn
       (convertDn)

--import Numeric.AERN.RealArithmetic.Measures

import qualified 
       Numeric.AERN.RefinementOrder 
       as RefOrd
import Numeric.AERN.RefinementOrder.Operators

import qualified 
       Numeric.AERN.NumericOrder 
       as NumOrd
import Numeric.AERN.NumericOrder.Operators

import Numeric.AERN.Basics.Effort
import Numeric.AERN.RealArithmetic.ExactOps

import Test.QuickCheck (Arbitrary(..)) -- , arbitrary, vectorOf)

import Control.Applicative 
       as Appl 

import Debug.Trace
_ = trace

data SineCosineThinEffortIndicator real coeff =
    SineCosineThinEffortIndicator
    {
--        sincoseff_real :: ArithInOut.RoundedRealEffortIndicator real,
--        sincoseff_coeff :: ArithInOut.RoundedRealEffortIndicator coeff,
        sincoseff_toCoeff :: ArithInOut.ConvertEffortIndicator real coeff,
        sincoseff_taylorDeg :: Int -- ^ truncate the Taylor expansion at this degree 
    }

instance 
    (
--        Arbitrary (ArithInOut.RoundedRealEffortIndicator real),
--        Arbitrary (ArithInOut.RoundedRealEffortIndicator coeff),
        Arbitrary (ArithInOut.ConvertEffortIndicator real coeff)
    )
    =>
    Arbitrary (SineCosineThinEffortIndicator real coeff)
    where
    arbitrary =
        SineCosineThinEffortIndicator <$> 
            arbitrary 
--            Appl.<*> arbitrary 
--            Appl.<*> arbitrary 
            Appl.<*> (fromInt1To10 <$> arbitrary)
        
deriving instance
    (
--        Show (ArithInOut.RoundedRealEffortIndicator real),
--        Show (ArithInOut.RoundedRealEffortIndicator coeff),
        Show (ArithInOut.ConvertEffortIndicator real coeff)
    )
    =>
    Show (SineCosineThinEffortIndicator real coeff)

instance
    (
--        EffortIndicator (ArithInOut.RoundedRealEffortIndicator real),
--        EffortIndicator (ArithInOut.RoundedRealEffortIndicator coeff),
        EffortIndicator (ArithInOut.ConvertEffortIndicator real coeff)
    )
    =>
    EffortIndicator (SineCosineThinEffortIndicator real coeff)
    where
    effortIncrementVariants (SineCosineThinEffortIndicator e1O e2O) = -- e3O e4O) =
        [SineCosineThinEffortIndicator e1 e2 | 
            (e1, e2) <- effortIncrementVariants (e1O, e2O) ]
    effortRepeatIncrement (SineCosineThinEffortIndicator i1 i2, SineCosineThinEffortIndicator j1 j2) = 
        SineCosineThinEffortIndicator 
            (effortRepeatIncrement (i1, j1)) 
            (effortRepeatIncrement (i2, j2)) 
--            (effortRepeatIncrement (i3, j3)) 
--            (effortRepeatIncrement (i4, j4)) 
    effortIncrementSequence (SineCosineThinEffortIndicator e1O e2O) =
        [SineCosineThinEffortIndicator e1 e2 | 
            (e1, e2) <- effortIncrementSequence (e1O, e2O) ]
    effortCombine
            (SineCosineThinEffortIndicator i1 i2) 
            (SineCosineThinEffortIndicator j1 j2) 
        =
        SineCosineThinEffortIndicator 
            (effortCombine i1 j1) 
            (effortCombine i2 j2)
--            (effortCombine i3 j3)
--            (effortCombine i4 j4)

sineCosineThinDefaultEffort :: 
   (
--    ArithInOut.RoundedReal real,
--    ArithInOut.RoundedReal coeff,
    ArithInOut.Convertible real coeff
   ) 
   =>
   real -> coeff -> Int -> SineCosineThinEffortIndicator real coeff
sineCosineThinDefaultEffort x cf deg =
    SineCosineThinEffortIndicator
    {
--        sincoseff_real = ArithInOut.roundedRealDefaultEffort x,
--        sincoseff_coeff = ArithInOut.roundedRealDefaultEffort cf,
        sincoseff_toCoeff = ArithInOut.convertDefaultEffort x cf,
        sincoseff_taylorDeg = deg
    }



sineCosineIntervalLikeOut :: 
    (RefOrd.IntervalLike real,
     ArithInOut.RoundedReal real,
     ArithInOut.RoundedReal coeff,
     ArithInOut.RoundedSpecialConst coeff,
     ArithInOut.Convertible real coeff,
     ArithInOut.RoundedMixedField real coeff,
     RefOrd.IntervalLike coeff,
     Show coeff, Show real)
     =>
     Bool {-^ True -> sine; False -> cosine -} ->
     SineCosineThinEffortIndicator real coeff ->
     coeff {-^ sample coefficient of the type to use in the Taylor expansion -} -> 
     real {-^ @x@ -} -> 
     real {-^ @sin(x)@ or @cos(x)@ -}
sineCosineIntervalLikeOut isSine eff sampleCoeff x =
--    trace (
--        "sineCosineIntervalLikeOut:"
--        ++ "\n  isSine = " ++ show isSine
--        ++ "\n  x = " ++ show x
--        ++ "\n  xRangeNear0 = " ++ show xRangeNear0
--        ++ "\n  plusMinusPiHalf = " ++ show plusMinusPiHalf
--    ) $
    case True of
        _ | isSine && xIsThin ->
             sineTaylorAt0 eff sampleCoeff xNear0
        -- sin(x) with x in [-pi/2, pi/2]; no need to shift x:
        _ | isSine && (xRangeNear0                  |>=? plusMinusPiHalf) == Just True ->
            sineViaEndpointsNear0 xNear0
            
        -- sin(x) with x in [pi/2, 3pi/2]; use sin(x) = -sin(x - pi):
        _ | isSine && (xRangeNear0 <+> (neg pi)     |>=? plusMinusPiHalf) == Just True ->
            neg $ sineViaEndpointsNear0 (xNear0 <+>| (neg pi))
            
        -- sin(x) with x in [-pi, 0]; use sin(x) = -cos(x + pi/2):
        _ | isSine && (xRangeNear0 <+> piHalf       |>=? plusMinusPiHalf) == Just True ->
            neg $ cosineViaEndpointsNear0 (xNear0 <+>| piHalf)
            
        -- sin(x) with x in [0, pi]; use sin(x) = cos(x - pi/2):
        _ | isSine && (xRangeNear0 <+> (neg piHalf) |>=? plusMinusPiHalf) == Just True ->
            cosineViaEndpointsNear0 (xNear0 <+>| (neg piHalf))
            

        _ | isCosine && xIsThin ->
             cosineTaylorAt0 eff sampleCoeff xNear0
        -- cos(x) with x in [-pi, 0]; use cos(x) = sin(x + pi/2):
        _ | isCosine && (xRangeNear0 <+> piHalf       |>=? plusMinusPiHalf) == Just True ->
            sineViaEndpointsNear0 (xNear0 <+>| piHalf)
            
        -- cos(x) with x in [0, pi]; use cos(x) = -sin(x - pi/2):
        _ | isCosine && (xRangeNear0 <+> (neg piHalf) |>=? plusMinusPiHalf) == Just True ->
            neg $ sineViaEndpointsNear0 (xNear0 <+>| (neg piHalf))

        -- cos(x) with x in [-pi/2, pi/2]; no need to shift x:
        _ | isCosine && (xRangeNear0                  |>=? plusMinusPiHalf) == Just True ->
            cosineViaEndpointsNear0 xNear0

        -- cos(x) with x in [pi/2, 3pi/2]; use cos(x) = -cos(x - pi):
        _ | isCosine && (xRangeNear0 <+> (neg pi)     |>=? plusMinusPiHalf) == Just True ->
            neg $ cosineViaEndpointsNear0 (xNear0 <+>| (neg pi))
            
        _ ->
            RefOrd.fromEndpointsOut (neg $ one x, one x) 
    where
    isCosine = not isSine
    
    xIsThin = 
        (xL ==? xR) == Just True
        where
        (xL, xR) = RefOrd.getEndpointsOut x
    
    (xRangeNear0, xNear0) = 
        (xRange <+> (neg k2pi),
         x <+>| (neg k2pi))
        where
        k2pi = (k * 2) |* pi
        k =
            case ArithUpDn.convertDn (0::Integer) kAsCoeff of 
                Just k2 -> k2
                _ -> error "sineCosineIntervalLike: parameter cannot be bounded"
        kAsCoeff =
            (0.5 :: Double) |<+> (xRange </> ((2 :: Int)|<*>pi))
            
    plusMinusPiHalf = RefOrd.fromEndpointsOut (neg piHalf, piHalf)
    piHalf = pi </>| (2 :: Int)
    pi = ArithInOut.piOut sampleCoeff
    
    xRange =
        ArithInOut.convertOutEff effToCoeff sampleCoeff x
        where
        effToCoeff = sincoseff_toCoeff eff

    sineViaEndpointsNear0 x0 =
        RefOrd.fromEndpointsOut (sLDn, sRUp)
        where
        (sLDn, sLUp) = RefOrd.getEndpointsOut sL
        (sRDn, sRUp) = RefOrd.getEndpointsOut sR
        sL = sineTaylorAt0 eff sampleCoeff x0L
        sR = sineTaylorAt0 eff sampleCoeff x0R
        (x0L, x0R) = RefOrd.getEndpointsOut x0

    cosineViaEndpointsNear0 x0
        | (x0 <=? z) == Just True = cosineViaEndpointsDecreasing
        | (z <=? x0) == Just True = cosineViaEndpointsIncreasing
        | (z |>=? x0) == Just True = cosineViaEndpointsContainsZero
        | otherwise = cosineViaEndpointsGeneral
        where
        z = zero x0
        (x0L, x0R) = RefOrd.getEndpointsOut x0
        cosineViaEndpointsIncreasing = cosineViaEndpoints True
        cosineViaEndpointsDecreasing = cosineViaEndpoints False
        cosineViaEndpoints isIncreasing 
            | isIncreasing =
                RefOrd.fromEndpointsOut (sLDn, sRUp)
            | otherwise =
                RefOrd.fromEndpointsOut (sRDn, sLUp)
            where
        cosineViaEndpointsContainsZero =
            RefOrd.fromEndpointsOut (minLRDn, one x0)
            where
            minLRDn = NumOrd.minOut sLDn sRDn
        cosineViaEndpointsGeneral =
            RefOrd.fromEndpointsOut (minLRDn, one x0)
            where
            minLRDn = NumOrd.minOut sLDn sRDn
        (sLDn, sLUp) = RefOrd.getEndpointsOut sL
        (sRDn, sRUp) = RefOrd.getEndpointsOut sR
        sL = cosineTaylorAt0 eff sampleCoeff x0L
        sR = cosineTaylorAt0 eff sampleCoeff x0R
    
{-|
    Taylor expansion of @sin(x)@, assuming @x@ is within  @[-pi/2, pi/2]@. 
-}
sineTaylorAt0 ::
    (ArithInOut.RoundedAdd real,
     ArithInOut.RoundedMultiply real,
     ArithInOut.RoundedPowerToNonnegInt real,
     HasOne real,
     ArithInOut.RoundedReal coeff,
     ArithInOut.Convertible real coeff,
     ArithInOut.RoundedMixedAdd real coeff,
     ArithInOut.RoundedMixedMultiply real coeff,
     RefOrd.IntervalLike coeff)
     =>
     SineCosineThinEffortIndicator real coeff ->
     coeff {-^ sample coefficient of the type to use in the Taylor expansion -} -> 
     real {-^ @x@ -} -> 
     real {-^ @sin(x)@ -}
sineTaylorAt0 = sineCosineTaylorAt0 True

{-|
    Taylor expansion of @cos(x)@, assuming @x@ is within  @[-pi/2, pi/2]@. 
-}
cosineTaylorAt0 ::
    (ArithInOut.RoundedAdd real,
     ArithInOut.RoundedMultiply real,
     ArithInOut.RoundedPowerToNonnegInt real,
     HasOne real,
     ArithInOut.RoundedReal coeff,
     ArithInOut.Convertible real coeff,
     ArithInOut.RoundedMixedAdd real coeff,
     ArithInOut.RoundedMixedMultiply real coeff,
     RefOrd.IntervalLike coeff)
     =>
     SineCosineThinEffortIndicator real coeff ->
     coeff {-^ sample coefficient of the type to use in the Taylor expansion -} -> 
     real {-^ @x@ -} -> 
     real {-^ @cos(x)@ -}
cosineTaylorAt0 = sineCosineTaylorAt0 False 

sineCosineTaylorAt0 ::
    (ArithInOut.RoundedAdd real,
     ArithInOut.RoundedMultiply real,
     ArithInOut.RoundedPowerToNonnegInt real,
     HasOne real,
     ArithInOut.RoundedReal coeff,
     ArithInOut.Convertible real coeff,
     ArithInOut.RoundedMixedAdd real coeff,
     ArithInOut.RoundedMixedMultiply real coeff,
     RefOrd.IntervalLike coeff)
     =>
     Bool {-^ True -> sine; False -> cosine -} ->
     SineCosineThinEffortIndicator real coeff ->
     coeff {-^ sample coefficient of the type to use in the Taylor expansion -} ->
     real {-^ @x@ -} -> 
     real {-^ @sin(x)@ or @cos(x)@ -}
sineCosineTaylorAt0 isSine eff sampleCoeff x = 
--        | xTooBig || xTooLow = (neg $ one sampleRes) </\> (one sampleRes) -- = [-1,1]
--        trace
--        (
--            "sineTaylor: "
--            ++ "\n x = " ++ show x
--            ++ "\n ranLargerEndpoint = " ++ show ranLargerEndpoint
--            ++ "\n result = " ++ show result
--        ) $
--        | otherwise = 
        result
        where
        result 
            | isSine =
                remainderApprox |<+> (x ArithInOut.<*> taylorExpansion)
            | otherwise =
                remainderApprox |<+> taylorExpansion
        (taylorExpansion, k, lastTermCoeff) 
            | isSine =
                sincosTaylorAux xSqr taylorDegree 1 (one sampleCoeff)
            | otherwise =
                sincosTaylorAux xSqr taylorDegree 0 (one sampleCoeff)
        xSqr = x <^> 2
        
        remainderApprox = -- using the Lagrange form: sin([-x,x])x^(k+1)/(k+1)! <= [-1,1]||x||^(k+2)/(k+1)!
            (neg sineErrorBound) </\> sineErrorBound
            where
            sineErrorBound =
                (ranLargerEndpointUp <^> (k+2)) ArithInOut.<*> errorCoeffUp
            (_, errorCoeffUp) =
                RefOrd.getEndpointsOut $
                    ArithInOut.absOut $ 
                        lastTermCoeff </>| (k+1) 
            (_, ranLargerEndpointUp) =
                RefOrd.getEndpointsOut $
                    ArithInOut.absOut $ 
                        ArithInOut.convertOutEff effToCoeff sampleCoeff x
        effToCoeff = sincoseff_toCoeff eff
        taylorDegree = sincoseff_taylorDeg eff

{-|
    Recursively evaluate a portion of a sine or cosine Taylor expansion.
-}
sincosTaylorAux ::
     (ArithInOut.RoundedAdd real,
      ArithInOut.RoundedMultiply real,
      HasOne real,
      ArithInOut.RoundedReal coeff,
      ArithInOut.RoundedMixedAdd real coeff,
      ArithInOut.RoundedMixedMultiply real coeff
     )
     =>
    real {-^ @x^2@ -} -> 
    Int {-^ @taylorDegree@ how far to go in the Taylor series -} -> 
    Int {-^ degree of the first term -} -> 
    coeff {-^ the coefficient of the first term -} -> 
    (real, Int, coeff)
    {-^ 
        Bounds for the series result and information about the first discarded term,
        from which some bound on the uniform error can be deduced.
        
        (value of Taylor expansion with last term of degree k, k, 1/k!)
    -} 
sincosTaylorAux 
        xSqr tayDegree 
        thisDegreeOrig thisCoeffOrig =
    sct thisDegreeOrig thisCoeffOrig
    where
    sct thisDegree thisCoeff
        | nextDegree > tayDegree =
--            trace
--            (
--                "sincosTaylorAux: "
--                ++ "\n thisCoeff = " ++ show thisCoeff
--                ++ "\n nextDegree = " ++ show nextDegree
--            )
            (thisCoeffRes, thisDegree, thisCoeff)
        | otherwise =
--            trace
--            (
--                "sincosTaylorAux: "
--                ++ "\n thisCoeff = " ++ show thisCoeff
--                ++ "\n nextDegree = " ++ show nextDegree
--                ++ "\n errorTermCoeff = " ++ show errorTermCoeff
--                ++ "\n errorTermDegree = " ++ show errorTermDegree
--            )
            (resultEncl, lastTermDegree, lastTermCoeff) 
        where
        thisCoeffRes = thisCoeff |<*> (one sampleRes)
        resultEncl =
            thisCoeff |<+> (xSqr ArithInOut.<*> restResult)
        
        (restResult, lastTermDegree, lastTermCoeff) =
            sct nextDegree nextCoeff
        
        nextDegree = thisDegree + 2
        
        nextCoeff = thisCoeff </> nextCoeffDenominator
        nextCoeffDenominator =
            ArithInOut.convertOut sampleCoeff $ toInteger $
                negate $ nextDegree * (nextDegree - 1)
                
        sampleRes = xSqr
        sampleCoeff = thisCoeff 

