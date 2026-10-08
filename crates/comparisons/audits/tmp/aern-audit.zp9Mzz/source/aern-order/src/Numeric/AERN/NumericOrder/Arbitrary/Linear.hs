{-# LANGUAGE TypeSynonymInstances #-}
{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleInstances #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-|
    Module      :  Numeric.AERN.NumericOrder.Arbitrary.Linear
    Description :  instances for linearly ordered types   
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    'ArbitraryOrderedTuple' instances for linearly ordered types.
    
    This module is hidden and reexported via its grand-parent NumericOrder. 
-}
module Numeric.AERN.NumericOrder.Arbitrary.Linear where

import Prelude hiding (EQ, LT, GT)

import Numeric.AERN.NumericOrder.Arbitrary

import Numeric.AERN.Basics.PartialOrdering
import Numeric.AERN.Basics.Arbitrary

import Numeric.AERN.Misc.Debug

import Data.Maybe
import Data.Ratio
import qualified Data.Map as Map 
import qualified Data.Set as Set 


import Test.QuickCheck
import System.Random
import Test.Framework (testGroup, Test)
import Test.Framework.Providers.QuickCheck2 (testProperty)
import Numeric.AERN.Misc.QuickCheck

import Numeric.AERN.Misc.List
import qualified Data.List as List
import System.IO.Unsafe

data AreaLinear t = 
    AreaLinear
    {
        areaLinLowerBound :: Maybe t,
        areaLinLowerBoundStrict :: Bool,
        areaLinUpperBound :: Maybe t,
        areaLinUpperBoundStrict :: Bool,
        areaLinWhole :: AreaWholeOnly t
    }
    deriving Show 

areaLinearWhole :: [t] -> AreaLinear t
areaLinearWhole specialValues = 
    AreaLinear Nothing True Nothing True $ areaWholeOnlyWhole specialValues

areaLinearRestrictToNonNeg ::
    (Ord t) =>
    t {-^ the zero -} ->
    (AreaLinear t) ->
    (AreaLinear t)
areaLinearRestrictToNonNeg z =
    areaLinearAddLowerBound (z, False)

areaLinearAddLowerBound ::
    (Ord t) =>
    (t, Bool) {-^ the lower bound, whether strict -} ->
    (AreaLinear t) ->
    (AreaLinear t)
areaLinearAddLowerBound (lowerBound, isStrict) area = 
    area
        {
            areaLinLowerBound = Just newLowerBound
            ,
            areaLinLowerBoundStrict = newLowerBoundStrict
            ,
            areaLinWhole =
                (areaLinWhole area)
                    {
                        areaWholeSpecialValues =
                            filter valueOK $ areaWholeSpecialValues $ areaLinWhole area
                    }
        }
    where
    (newLowerBound, newLowerBoundStrict) =
        case maybeOldLowerBound of
            Nothing -> (lowerBound, isStrict)
            Just oldLowerBound  
                | lowerBound < oldLowerBound -> (lowerBound, isStrict)
                | oldLowerBound < lowerBound -> (oldLowerBound, oldLowerBoundStrict)
                | otherwise -> (oldLowerBound, isStrict || oldLowerBoundStrict)
    maybeOldLowerBound = areaLinLowerBound area
    oldLowerBoundStrict = areaLinLowerBoundStrict area
    valueOK value 
        | isStrict = lowerBound < value
        | otherwise = lowerBound <= value
        
areaLinearAddUpperBound ::
    (Ord t) =>
    (t, Bool) {-^ the upper bound, whether strict -} ->
    (AreaLinear t) ->
    (AreaLinear t)
areaLinearAddUpperBound (upperBound, isStrict) area = 
    area
        {
            areaLinUpperBound = Just newUpperBound
            ,
            areaLinUpperBoundStrict = newUpperBoundStrict
            ,
            areaLinWhole =
                (areaLinWhole area)
                    {
                        areaWholeSpecialValues =
                            filter valueOK $ areaWholeSpecialValues $ areaLinWhole area
                    }
        }
    where
    (newUpperBound, newUpperBoundStrict) =
        case maybeOldUpperBound of
            Nothing -> (upperBound, isStrict)
            Just oldUpperBound  
                | upperBound > oldUpperBound -> (upperBound, isStrict)
                | oldUpperBound > upperBound -> (oldUpperBound, oldUpperBoundStrict)
                | otherwise -> (oldUpperBound, isStrict || oldUpperBoundStrict)
    maybeOldUpperBound = areaLinUpperBound area
    oldUpperBoundStrict = areaLinUpperBoundStrict area
    valueOK value 
        | isStrict = value < upperBound
        | otherwise = value <= upperBound
        

areaLinearAddForbiddenValues ::
    (Ord t) =>
    [t] {-^ newly forbidden values -} ->
    (AreaLinear t) ->
    (AreaLinear t)
areaLinearAddForbiddenValues values area =
    area
        {
            areaLinWhole =
                areaWholeOnlyAddForbiddenValues values (areaLinWhole area)
        }

arbitraryLinear ::
    (Arbitrary t) =>
    (t,t) {-^ least and greatest element -} -> 
    (t -> t) {-^ successor function -} ->
    (t -> t) {-^ predecessor function -} ->
    ((t,t) -> Gen t) {-^ choose function -} -> 
    AreaLinear t ->
    Gen t
arbitraryLinear (least, greatest) succ pred choose 
        (AreaLinear mlb lbStrict mub ubStrict areaWhole@(AreaWholeOnly _ isForbidden specialValues)) =
    incrSize $ -- at size 0 we get only 0s...
    do
    useSpecial <-
        case specialValues of
            [] -> return False
            _ -> elements [False, True, False, False] 
                 -- 1 in 4 values should be special
    case useSpecial of
        True -> elements specialValues
        False -> avoidForbidden
    where
    avoidForbidden =
        do
        result <-
            case (mlb, mub) of 
                (Nothing, Nothing) -> arbitrary
                _ -> choose (lb, ub)
        if isForbidden result
            then avoidForbidden 
            else return result
    lb = 
        case (mlb, lbStrict) of
            (Nothing, _) -> least
            (Just lb, True) -> lb
            (Just lb, False) -> succ lb
    ub = 
        case (mub, ubStrict) of
            (Nothing, _) -> greatest
            (Just ub, True) -> ub
            (Just ub, False) -> pred ub


instance ArbitraryWithArea Int 
    where
    type (Area Int) = AreaWholeOnly Int
    areaWhole _ = areaWholeOnlyWhole [-1,0,1]
    arbitraryInArea = arbitraryWhole

instance AreaHasForbiddenValues Int 
    where
    areaGetForbiddenValues = areaWholeForbiddenValues
    areaAddForbiddenValues = areaWholeOnlyAddForbiddenValues

instance ArbitraryOrderedTuple Int where
    arbitraryTupleInAreaRelatedBy area = 
        linearArbitraryTupleRelatedBy $ arbitraryWhole area
    arbitraryTupleRelatedBy =
        linearArbitraryTupleRelatedBy $ incrSize arbitrary

instance ArbitraryWithArea Integer where
    type (Area Integer) = AreaWholeOnly Integer
    areaWhole _ = areaWholeOnlyWhole [-1,0,1]
    arbitraryInArea = arbitraryWhole

instance AreaHasForbiddenValues Integer 
    where
    areaGetForbiddenValues = areaWholeForbiddenValues
    areaAddForbiddenValues = areaWholeOnlyAddForbiddenValues

instance ArbitraryOrderedTuple Integer where
    arbitraryTupleInAreaRelatedBy area = 
        linearArbitraryTupleRelatedBy $ arbitraryWhole area
    arbitraryTupleRelatedBy =
        linearArbitraryTupleRelatedBy $ incrSize arbitrary

instance ArbitraryWithArea Rational where
    type (Area Rational) = (AreaLinear Int, AreaLinear Int)
    areaWhole _ = (areaLinearWhole [-1,0,1], areaLinearWhole [0])
    arbitraryInArea (numeratorArea, preDenominatorArea) =
        chooseRational
        where
        chooseRational = 
            do
            num <- arbitraryIntInArea numeratorArea
            preDenom <- arbitraryIntInArea preDenominatorArea
            return $ (toInteger num) % (1 + (abs $ toInteger preDenom))
        arbitraryIntInArea = arbitraryLinear (minInt, maxInt) succ pred choose  
        maxInt = maxBound
        minInt = minBound

instance ArbitraryOrderedTuple Rational where
    arbitraryTupleInAreaRelatedBy area = 
        linearArbitraryTupleRelatedBy $ arbitraryInArea area
    arbitraryTupleRelatedBy =
        linearArbitraryTupleRelatedBy $ incrSize arbitrary

areaDoubleSmall :: AreaLinear Double
areaDoubleSmall =
    AreaLinear (Just $ -256) False (Just 256) False $ areaWholeOnlyWhole [0,-1,1]

instance ArbitraryWithArea Double where
    type (Area Double) = AreaLinear Double
    areaWhole _ = areaLinearWhole [-1/0,-1,0,1,1/0]
    arbitraryInArea area =
        arbitraryLinear (-maxDbl, maxDbl) id id chooseDbl area
        where
        maxDbl = encodeFloat 1 (maxExp - 1)
        chooseDbl (lb, ub) =
            do
            exp <- chooseNearerZero (expLb, expUb)
            case exp == expUb of
                True -> choose (lb, ub)
                False -> chooseWithExp exp
            where
            chooseNearerZero (lo, hi) =
--                unsafePrint ("chooseNearerZero: "
--                    ++ "\n loDT = " ++ show loDT
--                    ++ "\n hiDT = " ++ show hiDT
--                ) $
                do
                eDT <- choose (loDT, hiDT)
                return $ round $ transform eDT 
                where
                loDT = transformInv loD
                hiDT = transformInv hiD
                transform :: Double -> Double
                transform x = x*x*x/1000000 -- 100^3 = 1000000
                transformInv x 
                    | x > 0 = 100 * exp ((log x)/3)
                    | x < 0 = - (transformInv (-x))
                    | otherwise = 0
                loD = fromInteger $ toInteger lo
                hiD = fromInteger $ toInteger hi
            expLb = 
                case (lb <= 0 && ub >= 0) of
                    True -> minExp
                    False -> min lbExp ubExp
            expUb = max lbExp ubExp
            lbExp = case lb == 0 of True -> minExp; False -> exponent lb
            ubExp = case ub == 0 of True -> minExp; False -> exponent ub
            chooseWithExp exp =
                do
                signif <- choose (0.5,2)
                let validCandidates = deriveValidCandidates $ signif * (encodeFloat 1 exp)
                case validCandidates of
                    [] -> chooseWithExp exp
                    _ -> elements validCandidates
            deriveValidCandidates d =
                filter valid [d,-d]
                where
                valid d = lb <= d && d <= ub
        (minExp, maxExp) = floatRange (0 :: Double)
    
instance ArbitraryOrderedTuple Double where
    arbitraryTupleInAreaRelatedBy area = 
        linearArbitraryTupleRelatedBy $ arbitraryInArea area
    arbitraryTupleRelatedBy =
        arbitraryTupleInAreaRelatedBy areaDoubleSmall
       -- When generating Double numbers for testing, try to avoid overflows
       -- as we cannot usually overcome overflows when we cannot increase 
       -- the granularity (aka precision) of the floating point type.
       -- Exp overflows at around 700.


instance (AreaHasNonNegativeOption Double)
    where
    areaRestrictToNonNeg _ =
        areaLinearRestrictToNonNeg 0

instance (AreaHasForbiddenValues Double)
    where
    areaGetForbiddenValues = areaWholeForbiddenValues . areaLinWhole
    areaAddForbiddenValues = areaLinearAddForbiddenValues

instance (AreaHasBoundsConstraints Double)
    where
    areaSetLowerBound = areaLinearAddLowerBound
    areaSetUpperBound = areaLinearAddUpperBound

{-| Default implementation of linearArbitraryTupleRelatedBy for Ord instances -}   
linearArbitraryTupleRelatedBy ::
    (Ord ix, Show ix, Ord a, Show a) 
    =>
    (Gen a) ->
    [ix] ->
    [((ix,ix),[PartialOrdering])] ->
    Maybe (Gen [a])
linearArbitraryTupleRelatedBy givenArbitrary indices constraints =   
        case consistentUnambiguousConstraints of
            [] -> Nothing
            _ -> Just $
                do
                unambiguousConstraints <- elements consistentUnambiguousConstraints
                let cMap = Map.fromList unambiguousConstraints
                let sortedIndices = List.sortBy (turnIntoOrdering cMap) indices
                let sortedIndicesGrouped = List.groupBy (turnIntoEquality cMap) sortedIndices
                ds <- vectorOf (3 * (length sortedIndicesGrouped)) givenArbitrary
                seed <- arbitrary
                let dsDistinctSorted = getNDistinctSorted seed (length sortedIndicesGrouped) ds
                                      -- here we rely on the following property:
                                      -- it is very unlikely to get less than n distinct
                                      -- elements among 3*n elements generated by givenArbitrary 
                return $ makeResultTuple unambiguousConstraints sortedIndicesGrouped dsDistinctSorted 
        where
        makeResultTuple unambiguousConstraints sortedIndicesGrouped dsDistinctSorted =
--            unsafePrintReturn
--            (
--                "linearArbitraryTupleRelatedBy: "
--                ++ "\n constraints = " ++ show constraints
--                ++ "\n chosen linear order = " ++ show unambiguousConstraints
--                ++ "\n dsDistinctSorted = " ++ show dsDistinctSorted
--                ++ "\n result = "
--            ) $
            map snd $ 
                List.sort $ 
                    concat $ 
                        zipWith zip sortedIndicesGrouped $ 
                            map repeat dsDistinctSorted
        consistentUnambiguousConstraints =
            pickConsistentOrderings permittedInLinearOrder indices constraints
        turnIntoOrdering cMap a b =
            case (Map.lookup (a,b) cMap, Map.lookup (b,a) cMap) of
                (Just pord, _) -> fromPartialOrdering pord
                (_, Just pord) -> fromPartialOrdering $ partialOrderingTranspose pord
        turnIntoEquality cMap a b =
            case (Map.lookup (a,b) cMap, Map.lookup (b,a) cMap) of
                (Just pord, _) -> pord == EQ
                (_, Just pord) -> pord == EQ

{-| Default implementation of linearArbitraryTupleRelatedBy for types in which
    sequences of values are first transformed into linearly ordered ones. 
     -}   
forcedLinearArbitraryTupleRelatedBy ::
    (Ord ix, Show ix) =>
    (Gen a) ->
    (Int -> Int -> [a] -> [a]) ->
    [ix] ->
    [((ix,ix),[PartialOrdering])] ->
    Maybe (Gen [a])
forcedLinearArbitraryTupleRelatedBy givenArbitrary sortNSquashFn indices constraints =   
       case consistentUnambiguousConstraints of
          [] -> Nothing
          _ -> Just $
              do
              unambiguousConstraints <- elements consistentUnambiguousConstraints
              let cMap = Map.fromList unambiguousConstraints
              let sortedIndices = List.sortBy (turnIntoOrdering cMap) indices
              let sortedIndicesGrouped = List.groupBy (turnIntoEquality cMap) sortedIndices
              ds <- vectorOf (3 * (length sortedIndicesGrouped)) givenArbitrary
              seed <- arbitrary
              let dsDistinctSorted = sortNSquashFn seed (length sortedIndicesGrouped) ds
                                      -- here we rely on the following property:
                                      -- it is very unlikely to get less than n distinct
                                      -- elements among 3*n elements generated by givenArbitrary 
              return $ 
                  map snd $ 
                      List.sortBy (\(a,_) -> \(b,_) -> compare a b) $ 
                          concat $ 
                              zipWith zip sortedIndicesGrouped $ 
                                  map repeat dsDistinctSorted
       where
       consistentUnambiguousConstraints =
           pickConsistentOrderings permittedInLinearOrder indices constraints
       turnIntoOrdering cMap a b =
           case (Map.lookup (a,b) cMap, Map.lookup (b,a) cMap) of
               (Just pord, _) -> fromPartialOrdering pord
               (_, Just pord) -> fromPartialOrdering $ partialOrderingTranspose pord
       turnIntoEquality cMap a b =
           case (Map.lookup (a,b) cMap, Map.lookup (b,a) cMap) of
               (Just pord, _) -> pord == EQ
               (_, Just pord) -> pord == EQ

