{-# LANGUAGE FlexibleContexts #-}
{-|
    Module      :  Numeric.AERN.RealArithmetic.Bench
    Description :  benchmarking utilities  
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable

    Benchmarking utilities.
-}

module Numeric.AERN.RealArithmetic.Bench where

import Numeric.AERN.Basics.Consistency
import Numeric.AERN.NumericOrder.Operators

import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as ArithUpDn

import Numeric.AERN.RealArithmetic.ExactOps
import Numeric.AERN.RealArithmetic.Measures

import Numeric.AERN.Misc.Debug

{-| Approximate the imprecision of an operation by measuring
    the distance between its outer rounded result and inner rounded result 
-}
mkCommentImprecision1 ::
    (HasDistance t,
     ArithUpDn.Convertible (Distance t) Double,
     Show (Distance t)) =>
    (ei -> t -> t) ->
    (ei -> t -> t) ->
    ei -> t -> String
mkCommentImprecision1 opOut opIn effort a =
    show $ imprecisionD
    where
    imprecisionD :: Double
    imprecisionD =
        case ArithUpDn.convertUpEff (ArithUpDn.convertDefaultEffort imprecision sampleD) sampleD imprecision of
            Just imprecisionUp -> imprecisionUp
            Nothing -> error $ "mkCommentImprecision: cannot convert up to a Double: " ++ show imprecision
    sampleD = 0 :: Double
    imprecision = distanceBetweenEff (distanceDefaultEffort resultOut) resultOut resultIn
    resultOut = opOut effort a   
    resultIn = opIn effort a   

mkCommentAreaImprecision op effort a =
    unsafePrint
    (
        "mkCommentImprecision: " 
        ++ "\n a = " ++ show a
        ++ "\n effort = " ++ show effort
        ++ "\n aE = " ++ show aE
        ++ "\n aD = " ++ show aD
        ++ "\n aExp = " ++ show aExp
        ++ "\n resultE = " ++ show resultE
        ++ "\n imprecisionE = " ++ show imprecisionE
        ++ "\n imprecisionD = " ++ show imprecisionD
        ++ "\n imprecisionExp = " ++ show imprecisionExp
        ++ "\n resultBinaryDigits = " ++ show resultBinaryDigits
    ) $
    signOfaE ++ "x" ++ show aExp ++ "rd" ++ show resultBinaryDigits
    where
    signOfaE = 
        case (aE >? zero, aE <? zero) of
            (Just True, _) -> "+"
            (_, Just True) -> "-"
            _ -> ""
    aE = getThinRepresentative a
    Just aD =
        ArithUpDn.convertUpEff (ArithUpDn.convertDefaultEffort a sampleD) sampleD aE
    aExp = exponent aD
    
    resultE = op effort aE
    Just resultD =
        ArithUpDn.convertUpEff (ArithUpDn.convertDefaultEffort a sampleD) sampleD resultE
    resultExp = exponent resultD
    
    imprecisionE =
        imprecisionOfEff (imprecisionDefaultEffort a) resultE
    Just imprecisionD =
        ArithUpDn.convertUpEff (ArithUpDn.convertDefaultEffort imprecisionE sampleD) sampleD imprecisionE
    imprecisionExp = exponent imprecisionD
    
    resultBinaryDigits = resultExp - imprecisionExp
    
    imprecisionD, aD, resultD, sampleD :: Double
    sampleD = 0 
