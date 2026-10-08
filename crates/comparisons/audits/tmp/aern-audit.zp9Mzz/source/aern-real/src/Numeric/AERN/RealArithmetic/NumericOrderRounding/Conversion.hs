{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE FlexibleInstances #-}
{-|
    Module      :  Numeric.AERN.RealArithmetic.NumericOrderRounding.Conversion
    Description :  conversion between approximations and other types  
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Conversion between approximations and other types.
    
    This module is hidden and reexported via its parent NumericOrderRounding. 
-}
module Numeric.AERN.RealArithmetic.NumericOrderRounding.Conversion where

import Prelude hiding (EQ, LT, GT)

import Numeric.AERN.RealArithmetic.ExactOps

import Numeric.AERN.Basics.Effort
import Numeric.AERN.Basics.PartialOrdering
import qualified Numeric.AERN.NumericOrder as NumOrd

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
    convertUpEff :: ConvertEffortIndicator t1 t2 -> t2 -> t1 -> Maybe t2
    convertDnEff :: ConvertEffortIndicator t1 t2 -> t2 -> t1 -> Maybe t2

convertUp :: (Convertible t1 t2) => t2 -> t1 -> Maybe t2
convertUp a b = convertUpEff (convertDefaultEffort b a) a b

convertDn :: (Convertible t1 t2) => t2 -> t1 -> Maybe t2
convertDn a b = convertDnEff (convertDefaultEffort b a) a b


propConvertMonotone ::
    (Convertible t1 t2, 
     NumOrd.ArbitraryOrderedTuple t1, 
     NumOrd.PartialComparison t2) =>
    t1 -> t2 ->
    (ConvertEffortIndicator t1 t2,
     NumOrd.PartialCompareEffortIndicator t2) ->  
    NumOrd.LEPair t1 -> Bool
propConvertMonotone sample1 sample2 (effortConvert, effortComp2) (NumOrd.LEPair (a1, a2)) =
    (defined ma1Dn && defined ma2Up) ===>
    (trueOrNothing $ 
        let (<=?) = NumOrd.pLeqEff effortComp2 in
        a1Dn <=? a2Up)
    where
    ma1Dn = convertDnEff effortConvert sample2 a1 
    ma2Up = convertUpEff effortConvert sample2 a2
    Just a1Dn = ma1Dn
    Just a2Up = ma2Up
    
propConvertRoundTrip ::
    (Convertible t1 t2, Convertible t2 t1, NumOrd.PartialComparison t1) =>
    t1 -> t2 -> 
    (NumOrd.PartialCompareEffortIndicator t1, 
     ConvertEffortIndicator t2 t1, 
     ConvertEffortIndicator t1 t2) ->
    t1 -> Bool
propConvertRoundTrip sample1 sample2 (effortComp, effortFrom2, effortTo2) a =
    (defined maDn2 && defined maDn && defined maUp2 && defined maUp) ===>
    let (<=?) = NumOrd.pLeqEff effortComp in
    case (aDn <=? a, a <=? aUp) of
       (Just False, _) -> False
       (_, Just False) -> False
       _ -> True
    where
    maDn = convertDnEff effortFrom2 sample1 aDn2
    Just aDn = maDn 
    maDn2 = convertDnEff effortTo2 sample2 a
    Just aDn2 = maDn2 
    maUp = convertUpEff effortFrom2 sample1 aUp2
    Just aUp = maUp 
    maUp2 = convertUpEff effortTo2 sample2 a
    Just aUp2 = maUp2
    _ = [sample2, aUp2, aDn2] 
    
testsConvert (name1, sample1, name2, sample2) =
    testGroup (name1 ++ " -> " ++ name2 ++  " conversions") $
        [
            testProperty "monotone" (propConvertMonotone sample1 sample2)
        ,
            testProperty "round trip" (propConvertRoundTrip sample1 sample2)
        ]
    
instance Convertible t t
    where
    type ConvertEffortIndicator t t = ()
    convertDefaultEffort _ _ = ()
    convertUpEff _ _ x = Just x
    convertDnEff _ _ x = Just x
    