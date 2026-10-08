{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE FlexibleInstances #-}
{-|
    Module      :  Numeric.AERN.RealArithmetic.RefinementOrderRounding.Conversion
    Description :  conversion between approximations and other types  
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Conversion between approximations and other types.
    
    This module is hidden and reexported via its parent RefinementOrderRounding. 
-}
module Numeric.AERN.RealArithmetic.RefinementOrderRounding.Conversion where

import Prelude hiding (EQ, LT, GT)

import Numeric.AERN.RealArithmetic.ExactOps

import Numeric.AERN.Basics.Effort
import Numeric.AERN.Basics.PartialOrdering
import qualified Numeric.AERN.RefinementOrder as RefOrd
import qualified Numeric.AERN.NumericOrder as NumOrd

import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding.Conversion as UpDnConversion

import Numeric.AERN.Misc.Bool
import Numeric.AERN.Misc.Maybe

import Data.Ratio
import Data.Maybe

import Test.QuickCheck
import Test.Framework (testGroup, Test)
import Test.Framework.Providers.QuickCheck2 (testProperty)

class
    (EffortIndicator (ConvertEffortIndicator t1 t2))
    => 
    Convertible t1 t2 
    where
    type ConvertEffortIndicator t1 t2
    convertDefaultEffort :: t1 -> t2 -> ConvertEffortIndicator t1 t2 
    convertInEff :: 
        ConvertEffortIndicator t1 t2 -> 
        t2 {-^ sample value of the target type -} -> 
        t1 -> 
        t2
    convertOutEff :: 
        ConvertEffortIndicator t1 t2 -> 
        t2 {-^ sample value of the target type -} -> 
        t1 -> 
        t2


convertIn :: (Convertible t1 t2) => t2 -> t1 -> t2
convertIn a b = convertInEff (convertDefaultEffort b a) a b

convertOut :: (Convertible t1 t2) => t2 -> t1 -> t2
convertOut a b = convertOutEff (convertDefaultEffort b a) a b

propConvertMonotoneFromNumOrd ::
    (Convertible t1 t2, 
     NumOrd.ArbitraryOrderedTuple t1, 
     NumOrd.PartialComparison t2) 
    =>
    t1 -> t2 ->
    (ConvertEffortIndicator t1 t2, NumOrd.PartialCompareEffortIndicator t2) ->  
    NumOrd.LEPair t1 -> Bool
propConvertMonotoneFromNumOrd sample1 sample2 (effortFrom, effortComp) (NumOrd.LEPair (a, b)) = 
    (trueOrNothing $ let (<=?) = NumOrd.pLeqEff effortComp in aOut <=? bOut)
    &&
    (trueOrNothing $ let (<=?) = NumOrd.pLeqEff effortComp in aIn <=? bIn)
    where
    aOut = convertOutEff effortFrom sample2 a 
    aIn = convertInEff effortFrom sample2 a 
    bOut = convertOutEff effortFrom sample2 b 
    bIn = convertInEff effortFrom sample2 b

propConvertRoundTripNumOrd ::
    (UpDnConversion.Convertible t1 t2, Convertible t2 t1, 
     NumOrd.PartialComparison t1, Show t1, Show t2) 
    =>
    t1 -> t2 -> 
    (NumOrd.PartialCompareEffortIndicator t1, 
     ConvertEffortIndicator t2 t1, 
     UpDnConversion.ConvertEffortIndicator t1 t2) ->
    t1 -> Bool
propConvertRoundTripNumOrd sample1 sample2 (effortComp, effortFrom, effortTo) a =
    (defined maDn && defined maUp) ===>
    let (<=?) = NumOrd.pLeqEff effortComp in
    case (aDnOut <=? a, a <=? aUpOut) of
       (Just False, _) -> printErrorDetail
       (_, Just False) -> printErrorDetail
       _ -> True
    where
    aDnOut = convertOutEff effortFrom sample1 aDn 
    maDn = UpDnConversion.convertDnEff effortTo sample2 a
    Just aDn = maDn 
    aUpOut = convertOutEff effortFrom sample1 aUp
    maUp = UpDnConversion.convertUpEff effortTo sample2 a
    Just aUp = maUp 
    printErrorDetail =
        error $
           "propToFromInteger failed:"
           ++ "\n  a = " ++ show a
           ++ "\n  aDnOut = " ++ show aDnOut
           ++ "\n  aUpOut = " ++ show aUpOut


testsConvertNumOrd (name1, sample1, name2, sample2) =
    testGroup (name1 ++ " -> " ++ name2 ++  " conversions") $
        [
            testProperty "monotone" (propConvertMonotoneFromNumOrd sample1 sample2)
        ,
            testProperty "round trip" (propConvertRoundTripNumOrd sample2 sample1)
        ]

instance Convertible t t
    where
    type ConvertEffortIndicator t t = ()
    convertDefaultEffort _ _ = ()
    convertInEff _ _ x = x
    convertOutEff _ _ x = x

