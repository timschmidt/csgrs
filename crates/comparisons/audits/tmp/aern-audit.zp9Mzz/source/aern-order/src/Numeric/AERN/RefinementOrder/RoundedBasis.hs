{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-|
    Module      :  Numeric.AERN.RefinementOrder.RoundedBasis
    Description :  domain bases with outwards and inwards rounded operations  
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Domain bases with outwards and inwards rounded operations.
    
    This module is hidden and reexported via its parent RefinementOrder. 
-}
module Numeric.AERN.RefinementOrder.RoundedBasis 
where

import Control.Monad.ST (ST)

import Numeric.AERN.Basics.Effort
import Numeric.AERN.Basics.Exception
import Numeric.AERN.Basics.PartialOrdering
import Numeric.AERN.RefinementOrder.PartialComparison
import Numeric.AERN.RefinementOrder.Arbitrary

import Numeric.AERN.Basics.Laws.OperationRelation
import Numeric.AERN.Basics.Laws.RoundedOperation

import Numeric.AERN.Misc.Maybe

import Test.QuickCheck
import Test.Framework (testGroup, Test)
import Test.Framework.Providers.QuickCheck2 (testProperty)

infixr 2 <\/>?, >\/<?, <⊔>?, >⊔<? 

{-|
    A type with outward-rounding lattice operations.
-}
class
    (EffortIndicator (PartialJoinEffortIndicator t))
    => 
    RoundedBasisEffort t 
    where
    type PartialJoinEffortIndicator t
    partialJoinDefaultEffort :: t -> PartialJoinEffortIndicator t

class (RoundedBasisEffort t) => RoundedBasis t where
    partialJoinOutEff :: PartialJoinEffortIndicator t -> t -> t -> Maybe t
    partialJoinInEff :: PartialJoinEffortIndicator t -> t -> t -> Maybe t

-- | Partial outward rounded join with default effort
partialJoinOut :: (RoundedBasis t) => t -> t -> Maybe t
partialJoinOut a = partialJoinOutEff (partialJoinDefaultEffort a) a

-- | Partial outward rounded join with default effort
(<\/>?) :: (RoundedBasis t) => t -> t -> Maybe t
(<\/>?) = partialJoinOut 

{-| Convenience Unicode notation for '<\/>?' -}
(<⊔>?) :: (RoundedBasis t) => t -> t -> Maybe t 
(<⊔>?) = (<\/>?)

-- | Partial outward rounded join with default effort
partialJoinIn :: (RoundedBasis t) => t -> t -> Maybe t
partialJoinIn a = partialJoinInEff (partialJoinDefaultEffort a) a

-- | Partial inward rounded join with default effort
(>\/<?) :: (RoundedBasis t) => t -> t -> Maybe t
(>\/<?) = partialJoinIn

{-| Convenience Unicode notation for '>\/<?' -}
(>⊔<?) :: (RoundedBasis t) => t -> t -> Maybe t
(>⊔<?) = (>\/<?)


-- properties of RoundedBasis
propRoundedBasisComparisonCompatible :: 
    (PartialComparison t, RoundedBasis t) => 
    t -> 
    (PartialCompareEffortIndicator t, PartialJoinEffortIndicator t) ->
    t -> t -> Bool
propRoundedBasisComparisonCompatible _ (effortComp, effortJoin) =
    downRoundedPartialJoinOfOrderedPair (pLeqEff effortComp) 
        (partialJoinOutEff effortJoin)

-- properties of RoundedBasis:
propRoundedBasisJoinAboveBoth :: 
    (PartialComparison t, RoundedBasis t) => 
    t -> 
    (PartialCompareEffortIndicator t, PartialJoinEffortIndicator t) ->
    t -> t -> Bool
propRoundedBasisJoinAboveBoth _ (effortComp, effortJoin) = 
    partialJoinAboveOperands (pLeqEff effortComp) (partialJoinInEff effortJoin)

-- properties of RoundedBasis:
propRoundedBasisJoinIdempotent :: 
    (PartialComparison t, RoundedBasis t, Show t, HasLegalValues t) => 
    t -> 
    (PartialCompareEffortIndicator t, 
     PartialJoinEffortIndicator t) ->
    (UniformlyOrderedSingleton t) -> 
    Bool
propRoundedBasisJoinIdempotent _ (effortComp, effortJoinInOut) 
        (UniformlyOrderedSingleton e) = 
    partialRoundedIdempotent (pLeqEff effortComp) 
        (partialJoinInEff effortJoinInOut) (partialJoinOutEff effortJoinInOut) e

propRoundedBasisJoinCommutative :: 
    (PartialComparison t, RoundedBasis t, Show t, HasLegalValues t) => 
    t -> 
    (PartialCompareEffortIndicator t, 
     PartialJoinEffortIndicator t) ->
    UniformlyOrderedPair t -> Bool
propRoundedBasisJoinCommutative _ (effortComp, effortJoinInOut)
        (UniformlyOrderedPair (e1,e2)) = 
    partialRoundedCommutative (pLeqEff effortComp) 
        (partialJoinInEff effortJoinInOut) (partialJoinOutEff effortJoinInOut) 
        e1 e2

propRoundedBasisJoinAssociative :: 
    (PartialComparison t, RoundedBasis t, Show t, HasLegalValues t) => 
    t -> 
    (PartialCompareEffortIndicator t, 
     PartialJoinEffortIndicator t) ->
    UniformlyOrderedTriple t -> Bool
propRoundedBasisJoinAssociative _ (effortComp, effortJoinInOut)
        (UniformlyOrderedTriple (e1,e2,e3)) = 
    partialRoundedAssociative (pLeqEff effortComp) 
        (partialJoinInEff effortJoinInOut) (partialJoinOutEff effortJoinInOut) 
        e1 e2 e3


propRoundedBasisJoinMonotone ::
    (Eq t, RoundedBasis t, PartialComparison t) => 
    t -> 
    (PartialCompareEffortIndicator t, 
     PartialJoinEffortIndicator t) -> 
    LEPair t -> 
    LEPair t ->
    Bool
propRoundedBasisJoinMonotone _ (effortComp, effortInOut)
        (LEPair (e1Lower,e1)) 
        (LEPair (e2Lower,e2)) =
    case (maybeRLower, maybeR) of
        (Just rLower, Just r) ->
            case pLeqEff effortComp rLower r of
                Just b -> b
                Nothing -> True
        (_, _) -> True
    where
    maybeRLower = partialJoinOutEff effortInOut e1Lower e2Lower 
    maybeR = partialJoinInEff effortInOut e1 e2 

testsRoundedBasis ::
    (PartialComparison t,
     RoundedBasis t,
     Arbitrary t, Show t, HasLegalValues t,
     ArbitraryOrderedTuple t,
     Eq t) => 
    (String, t) -> Test
testsRoundedBasis (name, sample) =
    testGroup (name ++ " (<⊔>?, >⊔<?)") $
        [
         testProperty "rounded join comparison compatible"  (propRoundedBasisComparisonCompatible sample),
         testProperty "rounded join above both"  (propRoundedBasisJoinAboveBoth sample),
         testProperty "rounded join idempotent" (propRoundedBasisJoinIdempotent sample),
         testProperty "rounded join commutative" (propRoundedBasisJoinCommutative sample),
         testProperty "rounded join associative" (propRoundedBasisJoinAssociative sample),
         testProperty "rounded join monotone" (propRoundedBasisJoinMonotone sample)
        ]

