{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE TypeSynonymInstances #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE FlexibleInstances #-}
{-|
    Module      :  Numeric.AERN.Basics.Arbitrary
    Description :  random generation over specific areas
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Random generation over specific areas.
-}

module Numeric.AERN.Basics.Arbitrary where

import Numeric.AERN.Basics.Consistency


import Test.QuickCheck
import Test.Framework (testGroup, Test)
import Test.Framework.Providers.QuickCheck2 (testProperty)
import Numeric.AERN.Misc.QuickCheck

class ArbitraryWithArea t
    where
    {-| a type of meaningful constraints to place on generation of arbitrary values -}
    type Area t
    {-| a special area that puts no constaints on the values -}
    areaWhole :: t -> Area t
    arbitraryInArea :: Area t -> Gen t
    
newtype SingletonInArea t = SingletonInArea t deriving Show
instance
    (ArbitraryWithArea t, Area t ~ a)
    => 
    (ArbitraryWithParam (SingletonInArea t) a)
    where
    arbitraryWithParam area =
        do
        e <- arbitraryInArea area
        return $ SingletonInArea e
    
newtype PairInArea t = PairInArea (t,t) deriving Show
instance
    (ArbitraryWithArea t, Area t ~ a)
    => 
    (ArbitraryWithParam (PairInArea t) a)
    where
    arbitraryWithParam area =
        do
        e1 <- arbitraryInArea area
        e2 <- arbitraryInArea area
        return $ PairInArea (e1, e2)
    
newtype TripleInArea t = TripleInArea (t,t,t) deriving Show
instance
    (ArbitraryWithArea t, Area t ~ a)
    => 
    (ArbitraryWithParam (TripleInArea t) a)
    where
    arbitraryWithParam area =
        do
        e1 <- arbitraryInArea area
        e2 <- arbitraryInArea area
        e3 <- arbitraryInArea area
        return $ TripleInArea (e1, e2, e3)
    
{-| generic mechanism for adding common restriction on the area of random generation -}
class AreaHasForbiddenValues t where
    areaAddForbiddenValues :: [t] -> Area t -> Area t
    areaGetForbiddenValues :: Area t -> [t]
class AreaHasNonNegativeOption t where
    areaRestrictToNonNeg :: t -> Area t -> Area t

class AreaHasConsistencyConstraint t where
    areaSetConsistencyConstraint :: t -> AreaConsistencyConstraint -> (Area t) -> (Area t)

data AreaConsistencyConstraint =
    AreaMaybeAllowOnlyWithConsistencyStatus (Maybe ConsistencyStatus)
    deriving Show

{-| an almost void, simple concrete area specification -}
data AreaWholeOnly t =
    AreaWholeOnly 
    {
        areaWholeForbiddenValues :: [t],
        areaWholeIsValueForbidden :: t -> Bool,
        areaWholeSpecialValues :: [t]
    }

instance (Show t) => Show (AreaWholeOnly t)
    where
    show (AreaWholeOnly forbidden _ special) =
        "AreaWholeOnly { forbidden = " ++ show forbidden
        ++ ", special = " ++ show special ++ "}"

areaWholeOnlyWhole specialValues =
    AreaWholeOnly [] (const False) specialValues

areaWholeOnlyAddForbiddenValues ::
    (Ord t) =>
    [t] {-^ newly forbidden values -} ->
    (AreaWholeOnly t) ->
    (AreaWholeOnly t)
areaWholeOnlyAddForbiddenValues values area =
    area
        {
            areaWholeIsValueForbidden = 
                \value ->
                    (value `elem` values)
                    ||
                    (areaWholeIsValueForbidden area value)
            ,
            areaWholeForbiddenValues =
                areaWholeForbiddenValues area ++ values
        }

arbitraryWhole ::
    (Arbitrary t) =>
    AreaWholeOnly t ->
    Gen t
arbitraryWhole (AreaWholeOnly _ isForbidden specialValues) =
    incrSize $ -- at size 0 we get only 0s...
    do
    useSpecial <- elements [False, True, False, False] 
                        -- 1 in 4 values should be special
    case useSpecial of
        True -> elements specialValues
        False -> avoidForbidden
    where 
    avoidForbidden =
        do
        result <- arbitrary
        if isForbidden result
            then avoidForbidden 
            else return result
            

    