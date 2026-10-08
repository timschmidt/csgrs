{-# LANGUAGE TypeSynonymInstances #-}
{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleInstances #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-|
    Module      :  Numeric.AERN.RefinementOrder.Arbitrary
    Description :  random generation of tuples with various relation constraints  
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Random generation of tuples with various relation constraints.
    
    This module is hidden and reexported via its parent RefinementOrder. 
-}
module Numeric.AERN.RefinementOrder.Arbitrary where

import Prelude hiding (EQ, LT, GT)

import Numeric.AERN.Basics.PartialOrdering
import Numeric.AERN.Basics.Arbitrary

import Data.Maybe
import qualified Data.Map as Map 
import qualified Data.Set as Set 

import Test.QuickCheck
import Test.Framework (testGroup, Test)
import Test.Framework.Providers.QuickCheck2 (testProperty)
import Numeric.AERN.Misc.QuickCheck

import System.IO.Unsafe

class AreaHasBoundsConstraints t where
    areaSetOuterBound :: t -> (Area t) -> (Area t)  
    areaSetInnerBound :: t -> (Area t) -> (Area t)  


{-|
    Comparison with the ability to randomly generate
    pairs and triples of its own elements that are in 
    a specific order relation (eg LT or NC).
    
    This is to help with checking properties that
    make sense only for pairs in a certain relation
    where such pairs are rare.
-}
class (ArbitraryWithArea t) => ArbitraryOrderedTuple t where
    {-| generator of tuples that satisfy the given relation requirements
        and area restriction, 
        nothing if in this structure there are no tuples satisfying these requirements -}
    arbitraryTupleInAreaRelatedBy ::
        (Ord ix, Show ix) =>
        (Area t) -> 
        [ix] 
           {-^ how many elements should be generated and with what names -} -> 
        [((ix, ix),[PartialOrdering])]
           {-^ required orderings for some pairs of elements -} -> 
        Maybe (Gen [t]) {-^ generator for tuples if the requirements make sense -}   
    {-| generator of tuples that satisfy the given relation requirements, 
        nothing if in this structure there are no tuples satisfying these requirements -}
    arbitraryTupleRelatedBy ::
        (Ord ix, Show ix) => 
        [ix]
           {-^ how many elements should be generated and with what names -} -> 
        [((ix, ix),[PartialOrdering])]
           {-^ required orderings for some pairs of elements -} -> 
        Maybe (Gen [t]) {-^ generator for tuples if the requirements make sense -}
    arbitraryTuple ::   
        Int {-^ how many elements should be generated -} -> 
        Maybe (Gen [t]) {-^ generator for tuples if the requirements make sense -}
    arbitraryTuple n = arbitraryTupleRelatedBy [1..n] [] 

arbitraryPairRelatedBy ::
    (ArbitraryOrderedTuple t) => PartialOrdering -> Maybe (Gen (t,t))
arbitraryPairRelatedBy rel =
    case arbitraryTupleRelatedBy [1,2] [((1,2),[rel])] of
        Nothing -> Nothing
        Just gen -> Just $
            do
            ~[e1,e2] <- gen 
            return (e1,e2)

arbitraryPairInAreaRelatedBy ::
    (ArbitraryOrderedTuple t) =>
    Area t -> 
    PartialOrdering -> 
    Maybe (Gen (t,t))
arbitraryPairInAreaRelatedBy area rel =
    case arbitraryTupleInAreaRelatedBy area [1,2] [((1,2),[rel])] of
        Nothing -> Nothing
        Just gen -> Just $
            do
            ~[e1,e2] <- gen 
            return (e1,e2)

arbitraryTripleRelatedBy ::
    (ArbitraryOrderedTuple t) => 
    (PartialOrdering, PartialOrdering, PartialOrdering) -> Maybe (Gen (t,t,t))
arbitraryTripleRelatedBy (r1, r2, r3) =
    case arbitraryTupleRelatedBy [1,2,3] constraints of
        Nothing -> Nothing
        Just gen -> Just $
            do
            ~[e1,e2,e3] <- gen
            return (e1, e2, e3)
    where
    constraints = [((1,2),[r1]), ((2,3),[r2]), ((1,3),[r3])]

arbitraryTripleInAreaRelatedBy ::
    (ArbitraryOrderedTuple t) => 
    Area t ->
    (PartialOrdering, PartialOrdering, PartialOrdering) -> 
    Maybe (Gen (t,t,t))
arbitraryTripleInAreaRelatedBy area (r1, r2, r3) =
    case arbitraryTupleInAreaRelatedBy area [1,2,3] constraints of
        Nothing -> Nothing
        Just gen -> Just $
            do
            ~[e1,e2,e3] <- gen
            return (e1, e2, e3)
    where
    constraints = [((1,2),[r1]), ((2,3),[r2]), ((1,3),[r3])]

{-| type for randomly generating single elements using the distribution of the 'ArbitraryOrderedTuple' instance -}
newtype UniformlyOrderedSingleton t = UniformlyOrderedSingleton t deriving (Show)
{-| type for randomly generating pairs of unrelated elements using the distribution of the 'ArbitraryOrderedTuple' instance -}
data TwoUniformlyOrderedSingletons t = TwoUniformlyOrderedSingletons (t,t) deriving (Show)
{-| type for randomly generating triples of unrelated elements using the distribution of the 'ArbitraryOrderedTuple' instance -}
data ThreeUniformlyOrderedSingletons t = ThreeUniformlyOrderedSingletons (t,t,t) deriving (Show)

{-| type for generating pairs distributed in such a way that all ordering relations 
    permitted by this structure have similar probabilities of occurrence -}
data UniformlyOrderedPair t = UniformlyOrderedPair (t,t) deriving (Show)
data TwoUniformlyOrderedPairs t = TwoUniformlyOrderedPairs ((t,t),(t,t)) deriving (Show)
data ThreeUniformlyOrderedPairs t = ThreeUniformlyOrderedPairs ((t,t),(t,t),(t,t)) deriving (Show)

data LEPair t = LEPair (t,t) deriving (Show)
data TwoLEPairs t = TwoLEPairs ((t,t),(t,t)) deriving (Show)
data ThreeLEPairs t = ThreeLEPairs ((t,t),(t,t),(t,t)) deriving (Show)

{-| type for generating triples distributed in such a way that all ordering relation combinations 
    permitted by this structure have similar probabilities of occurrence -}
data UniformlyOrderedTriple t = UniformlyOrderedTriple (t,t,t) deriving (Show)


instance (ArbitraryOrderedTuple t) => Arbitrary (UniformlyOrderedSingleton t) where
    arbitrary =
        do
        ~[elem] <- gen
        return $ UniformlyOrderedSingleton elem
        where
        Just gen = arbitraryTupleRelatedBy [1] []

instance
    (ArbitraryOrderedTuple t, a ~ Area t) 
    => 
    ArbitraryWithParam (UniformlyOrderedSingleton t) a 
    where
    arbitraryWithParam area =
        do
        ~[elem] <- gen
        return $ UniformlyOrderedSingleton elem
        where
        Just gen = arbitraryTupleInAreaRelatedBy area [1::Int] []

instance
    (ArbitraryOrderedTuple t, a ~ Area t)
    => 
    ArbitraryWithParam (TwoUniformlyOrderedSingletons t) a 
    where
    arbitraryWithParam area =
        do
        (UniformlyOrderedSingleton e1) <- arbitraryWithParam area
        (UniformlyOrderedSingleton e2) <- arbitraryWithParam area
        return $ TwoUniformlyOrderedSingletons (e1,e2)

instance
    (ArbitraryOrderedTuple t, a ~ Area t)
    => 
    ArbitraryWithParam (ThreeUniformlyOrderedSingletons t) a 
    where
    arbitraryWithParam area =
        do
        (UniformlyOrderedSingleton e1) <- arbitraryWithParam area
        (UniformlyOrderedSingleton e2) <- arbitraryWithParam area
        (UniformlyOrderedSingleton e3) <- arbitraryWithParam area
        return $ ThreeUniformlyOrderedSingletons (e1,e2,e3)

instance 
    (ArbitraryOrderedTuple t) 
    => 
    Arbitrary (UniformlyOrderedPair t) where
    arbitrary =
        do
        gen <- elements gens
        pair <- gen
        return $ UniformlyOrderedPair pair
        where
        gens = catMaybes $ map arbitraryPairRelatedBy partialOrderingVariants  

instance
    (ArbitraryOrderedTuple t, a ~ Area t) 
    => 
    ArbitraryWithParam (UniformlyOrderedPair t) a 
    where
    arbitraryWithParam area =
        do
        gen <- elements gens
        pair <- gen
        return $ UniformlyOrderedPair pair
        where
        gens = catMaybes $ map (arbitraryPairInAreaRelatedBy area) partialOrderingVariants  


instance
    (ArbitraryOrderedTuple t) 
    => 
    Arbitrary (LEPair t) 
    where
    arbitrary =
        do
        gen <- elements gens
        pair <- gen
        return $ LEPair pair
        where
        gens = catMaybes $ map arbitraryPairRelatedBy [LT, LT, LT, EQ]  

instance
    (ArbitraryOrderedTuple t, a ~ Area t) 
    => 
    ArbitraryWithParam (TwoUniformlyOrderedPairs t) a 
    where
    arbitraryWithParam area =
        do
        (UniformlyOrderedPair p1) <- arbitraryWithParam area
        (UniformlyOrderedPair p2) <- arbitraryWithParam area
        return $ TwoUniformlyOrderedPairs (p1,p2)

instance
    (ArbitraryOrderedTuple t, a ~ Area t)
    => 
    ArbitraryWithParam (ThreeUniformlyOrderedPairs t) a 
    where
    arbitraryWithParam area =
        do
        (UniformlyOrderedPair p1) <- arbitraryWithParam area
        (UniformlyOrderedPair p2) <- arbitraryWithParam area
        (UniformlyOrderedPair p3) <- arbitraryWithParam area
        return $ ThreeUniformlyOrderedPairs (p1,p2,p3)


instance
    (ArbitraryOrderedTuple t, a ~ Area t) 
    => 
    ArbitraryWithParam (LEPair t) a
    where
    arbitraryWithParam area =
        do
        gen <- elements gens
        pair <- gen
        return $ LEPair pair
        where
        gens = catMaybes $ map (arbitraryPairInAreaRelatedBy area) [LT, LT, LT, EQ]  

instance
    (ArbitraryOrderedTuple t, a ~ Area t) 
    => 
    ArbitraryWithParam (TwoLEPairs t) a 
    where
    arbitraryWithParam area =
        do
        (LEPair p1) <- arbitraryWithParam area
        (LEPair p2) <- arbitraryWithParam area
        return $ TwoLEPairs (p1,p2)

instance
    (ArbitraryOrderedTuple t, a ~ Area t)
    => 
    ArbitraryWithParam (ThreeLEPairs t) a 
    where
    arbitraryWithParam area =
        do
        (LEPair p1) <- arbitraryWithParam area
        (LEPair p2) <- arbitraryWithParam area
        (LEPair p3) <- arbitraryWithParam area
        return $ ThreeLEPairs (p1,p2,p3)

instance 
    (ArbitraryOrderedTuple t)
    => 
    Arbitrary (UniformlyOrderedTriple t) where
    arbitrary = 
        do
        gen <- elements gens
        triple <- gen
        return $ UniformlyOrderedTriple triple
        where
        gens = catMaybes $ map arbitraryTripleRelatedBy partialOrderingVariantsTriples

instance
    (ArbitraryOrderedTuple t, a ~ Area t) 
    => 
    ArbitraryWithParam (UniformlyOrderedTriple t) a 
    where
    arbitraryWithParam area =
        do
        gen <- elements gens
        triple <- gen
        return $ UniformlyOrderedTriple triple
        where
        gens = catMaybes $ map (arbitraryTripleInAreaRelatedBy area) partialOrderingVariantsTriples  

propArbitraryOrderedPair ::
    (ArbitraryOrderedTuple t) =>
    (t -> t -> PartialOrdering) -> PartialOrdering -> Bool 
propArbitraryOrderedPair compare rel =
     case arbitraryPairRelatedBy rel of
        Nothing -> True
        Just gen ->
             and $ map relOK theSample 
             where
             theSample = unsafePerformIO $ sample' gen 
             relOK (e1, e2) = compare e1 e2 == rel

propArbitraryOrderedTriple ::
    (ArbitraryOrderedTuple t) =>
    (t -> t -> PartialOrdering) -> (PartialOrdering, PartialOrdering, PartialOrdering) -> Bool 
propArbitraryOrderedTriple compare rels@(r1,r2,r3) =
     case arbitraryTripleRelatedBy rels of
        Nothing -> True
        Just gen ->
             and $ map relOK theSample 
             where
             theSample = unsafePerformIO $ sample' $ gen
             relOK (e1, e2, e3) = 
                and [compare e1 e2 == r1, compare e2 e3 == r2, compare e1 e3 == r3]

testsArbitraryTuple ::
    (Arbitrary t,
     ArbitraryOrderedTuple t) =>
    (String, t, t -> t -> PartialOrdering) -> Test
testsArbitraryTuple (name, sample, compare)  =
    testGroup (name ++ " arbitrary ordered") $ 
        [
         testProperty "pairs" (propArbitraryOrderedPair compare)
        ,
         testProperty "triples" (propArbitraryOrderedTriple compare)
        ]
