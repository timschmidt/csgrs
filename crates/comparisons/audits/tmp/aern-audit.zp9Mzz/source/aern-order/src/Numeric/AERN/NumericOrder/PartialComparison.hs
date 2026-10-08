{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE TypeSynonymInstances #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE FlexibleInstances #-}
{-|
    Module      :  Numeric.AERN.NumericOrder.ApproxOrder
    Description :  Comparisons in a semidecidable order  
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Comparisons in a semidecidable order.
    
    This module is hidden and reexported via its parent NumericOrder. 
-}

module Numeric.AERN.NumericOrder.PartialComparison 
where

import Prelude hiding (EQ, LT, GT)

import Numeric.AERN.NumericOrder.Extrema
import Numeric.AERN.NumericOrder.Arbitrary

import Numeric.AERN.Basics.Arbitrary
import Numeric.AERN.Basics.Effort
import Numeric.AERN.Basics.PartialOrdering
import Numeric.AERN.Basics.Laws.PartialRelation

import Numeric.AERN.Misc.Maybe
import Numeric.AERN.Misc.Bool

import Test.QuickCheck
import Test.Framework (testGroup, Test)
import Test.Framework.Providers.QuickCheck2 (testProperty)

infix 4 ==?, <==>?, </=>?, <?, <=?, >=?, >?

{-|
    A type with semi-decidable equality and partial order
-}
class 
    (EffortIndicator (PartialCompareEffortIndicator t)) 
    => 
    PartialComparison t 
    where
    type PartialCompareEffortIndicator t
    pCompareDefaultEffort :: t -> PartialCompareEffortIndicator t
    
    pCompareEff :: PartialCompareEffortIndicator t -> t -> t -> Maybe PartialOrdering
    pCompareInFullEff :: PartialCompareEffortIndicator t -> t -> t -> PartialOrderingPartialInfo
    pCompareInFullEff eff a b = partialOrdering2PartialInfo $ pCompareEff eff a b 
    
    -- | Partial equality
    pEqualEff :: (PartialCompareEffortIndicator t) -> t -> t -> Maybe Bool
    -- | Partial `is comparable to`.
    pComparableEff :: (PartialCompareEffortIndicator t) -> t -> t -> Maybe Bool
    -- | Partial `is not comparable to`.
    pIncomparableEff :: (PartialCompareEffortIndicator t) -> t -> t -> Maybe Bool
    pLessEff :: (PartialCompareEffortIndicator t) -> t -> t -> Maybe Bool
    pLeqEff :: (PartialCompareEffortIndicator t) -> t -> t -> Maybe Bool
    pGeqEff :: (PartialCompareEffortIndicator t) -> t -> t -> Maybe Bool
    pGreaterEff :: (PartialCompareEffortIndicator t) -> t -> t -> Maybe Bool
    
    -- defaults for all convenience operations:
    pEqualEff effort a b =
        pOrdInfEQ $ pCompareInFullEff effort a b
    pLessEff effort a b = 
        pOrdInfLT $ pCompareInFullEff effort a b
    pGreaterEff effort a b = 
        pOrdInfGT $ pCompareInFullEff effort a b
    pLeqEff effort a b =
        pOrdInfLEQ $ pCompareInFullEff effort a b
    pGeqEff effort a b =
        pOrdInfGEQ $ pCompareInFullEff effort a b
    pComparableEff effort a b = 
        fmap not $ pOrdInfNC $ pCompareInFullEff effort a b
    pIncomparableEff effort a b =
        pOrdInfNC $ pCompareInFullEff effort a b



-- | Partial comparison with default effort
pCompare :: (PartialComparison t) => t -> t -> Maybe PartialOrdering
pCompare a = pCompareEff (pCompareDefaultEffort a) a

-- | Partial comparison with default effort
pCompareInFull :: (PartialComparison t) => t -> t -> PartialOrderingPartialInfo
pCompareInFull a = pCompareInFullEff (pCompareDefaultEffort a) a

-- | Partial `is comparable to` with default effort
pComparable :: (PartialComparison t) => t -> t -> Maybe Bool
pComparable a = pComparableEff (pCompareDefaultEffort a) a

-- | Partial `is comparable to`
(<==>?) :: (PartialComparison t) => t -> t -> Maybe Bool
(<==>?) = pComparable

-- | Partial `is not comparable to` with default effort
pIncomparable :: (PartialComparison t) => t -> t -> Maybe Bool
pIncomparable a = pIncomparableEff (pCompareDefaultEffort a) a

-- | Partial `is not comparable to`
(</=>?) :: (PartialComparison t) => t -> t -> Maybe Bool
(</=>?) = pIncomparable

-- | Partial equality with default effort
pEqual :: (PartialComparison t) => t -> t -> Maybe Bool
pEqual a = pEqualEff (pCompareDefaultEffort a) a

-- | Partial equality with default effort
(==?) :: (PartialComparison t) => t -> t -> Maybe Bool
(==?) = pEqual

-- | Partial `strictly less than` with default effort
pLess :: (PartialComparison t) => t -> t -> Maybe Bool
pLess a = pLessEff (pCompareDefaultEffort a) a

-- | Partial `strictly less than` with default effort
(<?) :: (PartialComparison t) => t -> t -> Maybe Bool
(<?) = pLess

-- | Partial `less than or equal to` with default effort
pLeq :: (PartialComparison t) => t -> t -> Maybe Bool
pLeq a = pLeqEff (pCompareDefaultEffort a) a

-- | Partial `less than or equal to` with default effort
(<=?) :: (PartialComparison t) => t -> t -> Maybe Bool
(<=?) = pLeq

-- | Partial `strictly greater than` with default effort
pGreater :: (PartialComparison t) => t -> t -> Maybe Bool
pGreater a = pGreaterEff (pCompareDefaultEffort a) a

-- | Partial `strictly greater than` with default effort
(>?) :: (PartialComparison t) => t -> t -> Maybe Bool
(>?) = pGreater

-- | Partial `greater than or equal to` with default effort
pGeq :: (PartialComparison t) => t -> t -> Maybe Bool
pGeq a = pGeqEff (pCompareDefaultEffort a) a

-- | Partial `greater than or equal to` with default effort
(>=?) :: (PartialComparison t) => t -> t -> Maybe Bool
(>=?) = pGeq


instance PartialComparison Int where
    type PartialCompareEffortIndicator Int = ()
    pCompareDefaultEffort _ = ()
    pCompareEff = pComparePreludeCompare    
    
instance PartialComparison Integer where
    type PartialCompareEffortIndicator Integer = ()
    pCompareDefaultEffort _ = ()
    pCompareEff = pComparePreludeCompare    
    
instance PartialComparison Rational where
    type PartialCompareEffortIndicator Rational = ()
    pCompareDefaultEffort _ = ()
    pCompareEff = pComparePreludeCompare    

instance PartialComparison Double where
    type PartialCompareEffortIndicator Double = ()
    pCompareEff _ a b = Just $ toPartialOrdering $ Prelude.compare a b
--        case (isNaN a, isNaN b) of
--           (False, False) -> Just $ toPartialOrdering $ Prelude.compare a b  
--           (True, True) -> Just EQ
--           _ -> Just NC 
    pCompareDefaultEffort _ = ()
    
instance PartialComparison () where
    type PartialCompareEffortIndicator () = ()
    pCompareDefaultEffort _ = ()
    pCompareEff _ _ _ = Just EQ
    
instance
    PartialComparison a
    => 
    PartialComparison (Maybe a) 
    where
    type PartialCompareEffortIndicator (Maybe a) = PartialCompareEffortIndicator a
    pCompareDefaultEffort (Just sample) = pCompareDefaultEffort sample
    pCompareDefaultEffort _ = error "pCompareDefaultEffort Nothing not defined"
    pCompareEff eff ma mb = 
        case (ma,mb) of
            (Just a, Just b) -> pCompareEff eff a b
            (Nothing, Just _) -> Just GT
            (Just _, Nothing) -> Just LT
            (Nothing, Nothing) -> Just EQ
    
pComparePreludeCompare _ a b =
    Just $ toPartialOrdering $ Prelude.compare a b

propPartialComparisonReflexiveEQ :: 
    (PartialComparison t) => 
    t -> 
    (PartialCompareEffortIndicator t) -> 
    (UniformlyOrderedSingleton t) -> 
    Bool
propPartialComparisonReflexiveEQ _ effort (UniformlyOrderedSingleton e) = 
    case pCompareEff effort e e of Just EQ -> True; Nothing -> True; _ -> False 

propPartialComparisonAntiSymmetric :: 
    (PartialComparison t) => 
    t -> 
    UniformlyOrderedPair t -> 
    (PartialCompareEffortIndicator t) -> 
    Bool
propPartialComparisonAntiSymmetric _ (UniformlyOrderedPair (e1,e2)) effort =
    case (pCompareEff effort e2 e1, pCompareEff effort e1 e2) of
        (Just b1, Just b2) -> b1 == partialOrderingTranspose b2
        _ -> True 

propPartialComparisonTransitiveEQ :: 
    (PartialComparison t) => 
    t -> 
    UniformlyOrderedTriple t -> 
    (PartialCompareEffortIndicator t) -> 
    Bool
propPartialComparisonTransitiveEQ _ 
        (UniformlyOrderedTriple (e1,e2,e3)) effort = 
    partialTransitive (pEqualEff effort) e1 e2 e3

propPartialComparisonTransitiveLT :: 
    (PartialComparison t) => 
    t -> 
    UniformlyOrderedTriple t -> 
    (PartialCompareEffortIndicator t) -> 
    Bool
propPartialComparisonTransitiveLT _ 
        (UniformlyOrderedTriple (e1,e2,e3)) effort = 
    partialTransitive (pLessEff effort) e1 e2 e3

propPartialComparisonTransitiveLE :: 
    (PartialComparison t) => 
    t -> 
    UniformlyOrderedTriple t -> 
    (PartialCompareEffortIndicator t) -> 
    Bool
propPartialComparisonTransitiveLE _ 
        (UniformlyOrderedTriple (e1,e2,e3)) effort = 
    partialTransitive (pLeqEff effort) e1 e2 e3

propExtremaInPartialComparison :: 
    (PartialComparison t, HasExtrema t) => 
    t -> 
    UniformlyOrderedSingleton t -> 
    (PartialCompareEffortIndicator t) -> 
    Bool
propExtremaInPartialComparison _ 
        (UniformlyOrderedSingleton e) effort = 
    partialOrderExtrema (pLeqEff effort) (least e) (greatest e) e

testsPartialComparison :: 
    (PartialComparison t,
     HasExtrema t,
     ArbitraryOrderedTuple t, 
     Show t) 
    => 
    (String, t) -> 
    (Area t) ->
    Test
testsPartialComparison (name, sample) area =
    testGroup (name ++ " (>=?)")
        [
         testProperty "anti symmetric" (area, propPartialComparisonAntiSymmetric sample)
        ,
         testProperty "transitive EQ" (area, propPartialComparisonTransitiveEQ sample)
        ,
         testProperty "transitive LE" (area, propPartialComparisonTransitiveLE sample)
        ,
         testProperty "transitive LT" (area, propPartialComparisonTransitiveLT sample)
        ,
         testProperty "extrema" (area, propExtremaInPartialComparison sample)
        ]
        
