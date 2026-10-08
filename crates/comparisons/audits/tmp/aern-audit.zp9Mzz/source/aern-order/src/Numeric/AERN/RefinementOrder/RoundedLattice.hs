{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-|
    Module      :  Numeric.AERN.RefinementOrder.RoundedLattice
    Description :  lattices with outwards and inwards rounded operations  
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Lattices with outwards and inwards rounded operations.
    
    This module is hidden and reexported via its parent RefinementOrder. 
-}
module Numeric.AERN.RefinementOrder.RoundedLattice 
where

import Numeric.AERN.Basics.Exception

import Control.Monad.ST (ST)

import Numeric.AERN.Basics.Arbitrary
import Numeric.AERN.Basics.Effort
import Numeric.AERN.Misc.Maybe
import Numeric.AERN.Basics.PartialOrdering
import Numeric.AERN.RefinementOrder.Arbitrary
import Numeric.AERN.RefinementOrder.PartialComparison

import Numeric.AERN.Basics.Laws.RoundedOperation
import Numeric.AERN.Basics.Laws.OperationRelation

import Test.QuickCheck
import Test.Framework (testGroup, Test)
import Test.Framework.Providers.QuickCheck2 (testProperty)

infixr 3 </\>, >/\<, <⊓>, >⊓<, /\, ⊓
infixr 2 <\/>, >\/<, <⊔>, >⊔<, \/, ⊔



{-|
    A type with outward-rounding lattice operations.
-}
class
    (EffortIndicator (JoinMeetEffortIndicator t))
    => 
    RoundedLatticeEffort t 
    where
    type JoinMeetEffortIndicator t
    joinmeetDefaultEffort :: t -> JoinMeetEffortIndicator t

class (RoundedLatticeEffort t) => RoundedLattice t where
    joinOutEff :: JoinMeetEffortIndicator t -> t -> t -> t
    joinInEff :: JoinMeetEffortIndicator t -> t -> t -> t
    meetOutEff :: JoinMeetEffortIndicator t -> t -> t -> t
    meetInEff :: JoinMeetEffortIndicator t -> t -> t -> t

-- | Outward rounded join with default effort
joinOut :: (RoundedLattice t) => t -> t -> t
joinOut a = joinOutEff (joinmeetDefaultEffort a) a 

-- | Outward rounded join with default effort
(<\/>) :: (RoundedLattice t) => t -> t -> t 
(<\/>) = joinOut 
(\/) :: (RoundedLattice t) => t -> t -> t
(\/) = (<\/>)

{-| Convenience Unicode notation for '<\/>' -}
(<⊔>) :: (RoundedLattice t) => t -> t -> t
(<⊔>) = (<\/>)
(⊔) :: (RoundedLattice t) => t -> t -> t
(⊔) = (<\/>)

-- | Inward rounded join with default effort
joinIn :: (RoundedLattice t) => t -> t -> t
joinIn a = joinInEff (joinmeetDefaultEffort a) a 

-- | Inward rounded join with default effort
(>\/<) :: (RoundedLattice t) => t -> t -> t
(>\/<) = joinIn 

{-| Convenience Unicode notation for '>\/<' -}
(>⊔<) :: (RoundedLattice t) => t -> t -> t
(>⊔<) = (>\/<)

-- | Outward rounded meet with default effort
meetOut :: (RoundedLattice t) => t -> t -> t
meetOut a = meetOutEff (joinmeetDefaultEffort a) a 

-- | Outward rounded meet with default effort
(</\>) :: (RoundedLattice t) => t -> t -> t
(</\>) = meetOut 
(/\) :: (RoundedLattice t) => t -> t -> t
(/\) = (</\>)

{-| Convenience Unicode notation for '</\>' -}
(<⊓>) :: (RoundedLattice t) => t -> t -> t
(<⊓>) = (</\>)
(⊓) :: (RoundedLattice t) => t -> t -> t
(⊓) = (<⊓>)

-- | Inward rounded meet with default effort
meetIn :: (RoundedLattice t) => t -> t -> t
meetIn a = meetInEff (joinmeetDefaultEffort a) a 

-- | Inward rounded meet with default effort
(>/\<) :: (RoundedLattice t) => t -> t -> t
(>/\<) = meetIn 

{-| Convenience Unicode notation for '>/\<' -}
(>⊓<) :: (RoundedLattice t) => t -> t -> t
(>⊓<) = (>/\<)


-- properties of RoundedLattice

propRoundedLatticeComparisonCompatible :: 
    (PartialComparison t, RoundedLattice t) => 
    t ->
    UniformlyOrderedPair t -> 
    (PartialCompareEffortIndicator t, 
     JoinMeetEffortIndicator t) -> 
    Bool
propRoundedLatticeComparisonCompatible _ 
        (UniformlyOrderedPair (e1,e2)) 
        (effortComp, effortInOut) 
    =
    (downRoundedJoinOfOrderedPair (pLeqEff effortComp) (joinOutEff effortInOut) e1 e2)
    && 
    (upRoundedMeetOfOrderedPair (pLeqEff effortComp) (meetInEff effortInOut) e1 e2)

propRoundedLatticeJoinAboveBoth :: 
    (PartialComparison t, RoundedLattice t) => 
    t -> 
    UniformlyOrderedPair t -> 
    (PartialCompareEffortIndicator t, 
     JoinMeetEffortIndicator t) -> 
    Bool
propRoundedLatticeJoinAboveBoth _ 
        (UniformlyOrderedPair (e1,e2)) 
        (effortComp, effortInOut)
    = 
    joinAboveOperands (pLeqEff effortComp) (joinInEff effortInOut) e1 e2

propRoundedLatticeMeetBelowBoth :: 
    (PartialComparison t, RoundedLattice t) => 
    t -> 
    UniformlyOrderedPair t -> 
    (PartialCompareEffortIndicator t, 
     JoinMeetEffortIndicator t) -> 
    Bool
propRoundedLatticeMeetBelowBoth _ 
        (UniformlyOrderedPair (e1,e2)) 
        (effortComp, effortInOut)
    = 
    meetBelowOperands (pLeqEff effortComp) (meetOutEff effortInOut) e1 e2

propRoundedLatticeJoinIdempotent :: 
    (PartialComparison t, RoundedLattice t, Show t, HasLegalValues t) => 
    t -> 
    (UniformlyOrderedSingleton t) -> 
    (PartialCompareEffortIndicator t, 
     JoinMeetEffortIndicator t) -> 
    Bool
propRoundedLatticeJoinIdempotent _ 
        (UniformlyOrderedSingleton e) 
        (effortComp, effortInOut) 
    = 
    roundedIdempotent (pLeqEff effortComp) (joinInEff effortInOut) (joinOutEff effortInOut) e

propRoundedLatticeJoinCommutative :: 
    (PartialComparison t, RoundedLattice t, Show t, HasLegalValues t) => 
    t -> 
    UniformlyOrderedPair t -> 
    (PartialCompareEffortIndicator t, 
     JoinMeetEffortIndicator t) -> 
    Bool
propRoundedLatticeJoinCommutative _ 
        (UniformlyOrderedPair (e1,e2)) 
        (effortComp, effortInOut)
    = 
    roundedCommutative (pLeqEff effortComp) (joinInEff effortInOut) (joinOutEff effortInOut) e1 e2

propRoundedLatticeJoinAssocative :: 
    (PartialComparison t, RoundedLattice t, Show t, HasLegalValues t) => 
    t -> 
    UniformlyOrderedTriple t -> 
    (PartialCompareEffortIndicator t, 
     JoinMeetEffortIndicator t) -> 
    Bool
propRoundedLatticeJoinAssocative _ 
        (UniformlyOrderedTriple (e1,e2,e3)) 
        (effortComp, effortInOut)
    = 
    roundedAssociative (pLeqEff effortComp) (joinInEff effortInOut) (joinOutEff effortInOut) e1 e2 e3

propRoundedLatticeJoinMonotone ::
    (RoundedLattice t, PartialComparison t, Show t, HasLegalValues t) => 
    t -> 
    TwoLEPairs t -> 
    (PartialCompareEffortIndicator t, 
     JoinMeetEffortIndicator t) -> 
    Bool
propRoundedLatticeJoinMonotone _ 
        (TwoLEPairs ((e1Lower,e1),(e2Lower,e2))) 
        (effortComp, effortInOut)
    =
    case pLeqEff effortComp rLower r of
        Just b -> b
        Nothing -> True
   where
    rLower = joinOutEff effortInOut e1Lower e2Lower 
    r = joinInEff effortInOut e1 e2 

propRoundedLatticeMeetIdempotent :: 
    (PartialComparison t, RoundedLattice t, Show t, HasLegalValues t) => 
    t -> 
    (UniformlyOrderedSingleton t) -> 
    (PartialCompareEffortIndicator t, 
     JoinMeetEffortIndicator t) -> 
    Bool
propRoundedLatticeMeetIdempotent _ 
        (UniformlyOrderedSingleton e) 
        (effortComp, effortInOut) 
    = 
    roundedIdempotent (pLeqEff effortComp) (meetInEff effortInOut) (meetOutEff effortInOut) e

propRoundedLatticeMeetCommutative :: 
    (PartialComparison t, RoundedLattice t, Show t, HasLegalValues t) => 
    t -> 
    UniformlyOrderedPair t -> 
    (PartialCompareEffortIndicator t, 
     JoinMeetEffortIndicator t) -> 
    Bool
propRoundedLatticeMeetCommutative _ 
        (UniformlyOrderedPair (e1,e2)) 
        (effortComp, effortInOut)
    = 
    roundedCommutative (pLeqEff effortComp) (meetInEff effortInOut) (meetOutEff effortInOut) e1 e2

propRoundedLatticeMeetAssocative :: 
    (PartialComparison t, RoundedLattice t, Show t, HasLegalValues t) => 
    t -> 
    UniformlyOrderedTriple t -> 
    (PartialCompareEffortIndicator t, 
     JoinMeetEffortIndicator t) -> 
    Bool
propRoundedLatticeMeetAssocative _ 
        (UniformlyOrderedTriple (e1,e2,e3)) 
        (effortComp, effortInOut)
    = 
    roundedAssociative (pLeqEff effortComp) (meetInEff effortInOut) (meetOutEff effortInOut) e1 e2 e3

{- optional properties: -}
propRoundedLatticeModular :: 
    (PartialComparison t, RoundedLattice t, Show t, HasLegalValues t) => 
    t -> 
    UniformlyOrderedTriple t -> 
    (PartialCompareEffortIndicator t, 
     JoinMeetEffortIndicator t) -> 
    Bool
propRoundedLatticeModular _ 
        (UniformlyOrderedTriple (e1,e2,e3)) 
        (effortComp, effortInOut)
    = 
    roundedModular (pLeqEff effortComp) (meetInEff effortInOut) (joinInEff effortInOut) (meetOutEff effortInOut) (joinOutEff effortInOut) e1 e2 e3

propRoundedLatticeMeetMonotone ::
    (RoundedLattice t, PartialComparison t, Show t, HasLegalValues t) => 
    t ->
    (TwoLEPairs t) -> 
    (PartialCompareEffortIndicator t, 
     JoinMeetEffortIndicator t) -> 
    Bool
propRoundedLatticeMeetMonotone _ 
        (TwoLEPairs ((e1Lower,e1), (e2Lower,e2))) 
        (effortComp, effortInOut)
    =
    case pLeqEff effortComp rLower r of
        Just b -> b
        Nothing -> True
   where
    rLower = meetOutEff effortInOut e1Lower e2Lower 
    r = meetInEff effortInOut e1 e2 

propRoundedLatticeDistributive :: 
    (PartialComparison t, RoundedLattice t, Show t, HasLegalValues t) => 
    t -> 
    UniformlyOrderedTriple t -> 
    (PartialCompareEffortIndicator t, 
     JoinMeetEffortIndicator t) -> 
    Bool
propRoundedLatticeDistributive _ 
        (UniformlyOrderedTriple (e1,e2,e3)) 
        (effortComp, effortInOut)
    = 
    (roundedLeftDistributive  (pLeqEff effortComp) (meetInEff effortInOut) (joinInEff effortInOut) (meetOutEff effortInOut) (joinOutEff effortInOut) e1 e2 e3)
    && 
    (roundedLeftDistributive  (pLeqEff effortComp) (joinInEff effortInOut) (meetInEff effortInOut) (joinOutEff effortInOut) (meetOutEff effortInOut) e1 e2 e3)


testsRoundedLatticeDistributive :: 
    (PartialComparison t,
     RoundedLattice t,
     Show t, HasLegalValues t,
     ArbitraryOrderedTuple t) 
    => 
    (String, t) ->
    (Area t) -> 
    Test
testsRoundedLatticeDistributive (name, sample) area =
    testGroup (name ++ " (⊓,⊔) rounded") $
        [
         testProperty "Comparison compatible" (area, propRoundedLatticeComparisonCompatible sample)
        ,
         testProperty "join above" (area, propRoundedLatticeJoinAboveBoth sample)
        ,
         testProperty "meet below" (area, propRoundedLatticeMeetBelowBoth sample)
        ,
         testProperty "join idempotent" (area, propRoundedLatticeJoinIdempotent sample)
        ,
         testProperty "join commutative" (area, propRoundedLatticeJoinCommutative sample)
        ,
         testProperty "join associative" (area, propRoundedLatticeJoinAssocative sample)
        ,
         testProperty "join monotone" (area, propRoundedLatticeJoinMonotone sample)
        ,
         testProperty "meet idempotent" (area, propRoundedLatticeMeetIdempotent sample)
        ,
         testProperty "meet commutative" (area, propRoundedLatticeMeetCommutative sample)
        ,
         testProperty "meet associative" (area, propRoundedLatticeMeetAssocative sample)
        ,
         testProperty "meet monotone" (area, propRoundedLatticeMeetMonotone sample)
        ,
         testProperty "distributive" (area, propRoundedLatticeDistributive sample)
        ]
    
-- mutable versions (TODO)    