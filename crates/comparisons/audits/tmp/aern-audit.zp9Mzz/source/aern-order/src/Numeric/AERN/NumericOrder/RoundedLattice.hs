{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-|
    Module      :  Numeric.AERN.NumericOrder.RoundedLattice
    Description :  lattices with directed-rounded operations  
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Lattices with directed-rounded operations.
    
    This module is hidden and reexported via its parent NumericOrder. 
-}
module Numeric.AERN.NumericOrder.RoundedLattice 
where

import Prelude hiding ((<=))

import Numeric.AERN.Basics.Exception

import Numeric.AERN.Basics.Arbitrary
import Numeric.AERN.Basics.Effort
import Numeric.AERN.Basics.PartialOrdering
import Numeric.AERN.NumericOrder.Arbitrary 
import Numeric.AERN.NumericOrder.PartialComparison 
import Numeric.AERN.NumericOrder.Extrema

import Numeric.AERN.Basics.Laws.PartialRelation
import Numeric.AERN.Basics.Laws.RoundedOperation
import Numeric.AERN.Basics.Laws.OperationRelation

import Numeric.AERN.Misc.Maybe

import Test.QuickCheck
import Test.Framework (testGroup, Test)
import Test.Framework.Providers.QuickCheck2 (testProperty)

class
    (EffortIndicator (MinmaxEffortIndicator t))
    => 
    RoundedLatticeEffort t 
    where
    type MinmaxEffortIndicator t
    minmaxDefaultEffort :: t -> MinmaxEffortIndicator t

{-|
    A type with directed-rounding lattice operations.
-}
class (RoundedLatticeEffort t) => RoundedLattice t where
    maxUpEff :: MinmaxEffortIndicator t -> t -> t -> t
    maxDnEff :: MinmaxEffortIndicator t -> t -> t -> t
    minUpEff :: MinmaxEffortIndicator t -> t -> t -> t
    minDnEff :: MinmaxEffortIndicator t -> t -> t -> t

-- default-effort variants:
{-|
    Rounded lattice operation using default effort.
-}
minUp :: (RoundedLattice t) => t -> t -> t
minUp a = minUpEff (minmaxDefaultEffort a) a
minDn :: (RoundedLattice t) => t -> t -> t
minDn a = minDnEff (minmaxDefaultEffort a) a
maxUp :: (RoundedLattice t) => t -> t -> t
maxUp a = maxUpEff (minmaxDefaultEffort a) a
maxDn :: (RoundedLattice t) => t -> t -> t
maxDn a = maxDnEff (minmaxDefaultEffort a) a

propRoundedLatticeComparisonCompatible :: 
    (PartialComparison t, RoundedLattice t) => 
    t -> 
    UniformlyOrderedPair t -> 
    (MinmaxEffortIndicator t, PartialCompareEffortIndicator t) -> 
    Bool
propRoundedLatticeComparisonCompatible _ 
        (UniformlyOrderedPair (e1,e2)) 
        (minmaxEffort, effortComp) 
        =
    (downRoundedJoinOfOrderedPair (pLeqEff effortComp) (minDnEff minmaxEffort) e1 e2)
    && 
    (upRoundedMeetOfOrderedPair (pLeqEff effortComp) (maxUpEff minmaxEffort) e1 e2)

propRoundedLatticeJoinAboveBoth :: 
    (PartialComparison t, RoundedLattice t) => 
    t -> 
    UniformlyOrderedPair t -> 
    (MinmaxEffortIndicator t, PartialCompareEffortIndicator t) -> 
    Bool
propRoundedLatticeJoinAboveBoth _ 
        (UniformlyOrderedPair (e1,e2)) 
        (minmaxEffort, effortComp) 
    = 
    joinAboveOperands (pLeqEff effortComp) (maxUpEff minmaxEffort) e1 e2

propRoundedLatticeMeetBelowBoth :: 
    (PartialComparison t, RoundedLattice t) => 
    t -> 
    UniformlyOrderedPair t -> 
    (MinmaxEffortIndicator t, PartialCompareEffortIndicator t) -> 
    Bool
propRoundedLatticeMeetBelowBoth _ 
        (UniformlyOrderedPair (e1,e2)) 
        (minmaxEffort, effortComp)
    = 
    meetBelowOperands (pLeqEff effortComp) (minDnEff minmaxEffort) e1 e2

propRoundedLatticeJoinIdempotent :: 
    (PartialComparison t, RoundedLattice t, Show t, HasLegalValues t) => 
    t -> 
    UniformlyOrderedSingleton t -> 
    (MinmaxEffortIndicator t, PartialCompareEffortIndicator t) -> 
    Bool
propRoundedLatticeJoinIdempotent _ 
        (UniformlyOrderedSingleton a) 
        (minmaxEffort, effortComp) 
    = 
    roundedIdempotent (pLeqEff effortComp) (maxUpEff minmaxEffort) (maxDnEff minmaxEffort) a

propRoundedLatticeJoinCommutative :: 
    (PartialComparison t, RoundedLattice t, Show t, HasLegalValues t) => 
    t -> 
    UniformlyOrderedPair t -> 
    (MinmaxEffortIndicator t, PartialCompareEffortIndicator t) -> 
    Bool
propRoundedLatticeJoinCommutative _ 
        (UniformlyOrderedPair (e1,e2)) 
        (minmaxEffort, effortComp)
    = 
    roundedCommutative (pLeqEff effortComp) (maxUpEff minmaxEffort) (maxDnEff minmaxEffort) e1 e2

propRoundedLatticeJoinAssocative :: 
    (PartialComparison t, RoundedLattice t, Show t, HasLegalValues t) => 
    t -> 
    UniformlyOrderedTriple t -> 
    (MinmaxEffortIndicator t, PartialCompareEffortIndicator t) -> 
    Bool
propRoundedLatticeJoinAssocative _ 
        (UniformlyOrderedTriple (e1,e2,e3)) 
        (minmaxEffort, effortComp)
    = 
    roundedAssociative (pLeqEff effortComp) (maxUpEff minmaxEffort) (maxDnEff minmaxEffort) e1 e2 e3

propRoundedLatticeMeetIdempotent :: 
    (PartialComparison t, RoundedLattice t, Show t, HasLegalValues t) => 
    t -> 
    UniformlyOrderedSingleton t -> 
    (MinmaxEffortIndicator t, PartialCompareEffortIndicator t) -> 
    Bool
propRoundedLatticeMeetIdempotent _ 
        (UniformlyOrderedSingleton e) 
        (minmaxEffort, effortComp) 
    = 
    roundedIdempotent (pLeqEff effortComp) (minUpEff minmaxEffort) (minDnEff minmaxEffort) e

propRoundedLatticeMeetCommutative :: 
    (PartialComparison t, RoundedLattice t, Show t, HasLegalValues t) => 
    t -> 
    UniformlyOrderedPair t -> 
    (MinmaxEffortIndicator t, PartialCompareEffortIndicator t) -> 
    Bool
propRoundedLatticeMeetCommutative _ 
        (UniformlyOrderedPair (e1,e2)) 
        (minmaxEffort, effortComp)
    = 
    roundedCommutative (pLeqEff effortComp) (minUpEff minmaxEffort) (minDnEff minmaxEffort) e1 e2

propRoundedLatticeMeetAssocative :: 
    (PartialComparison t, RoundedLattice t, Show t, HasLegalValues t) => 
    t -> 
    UniformlyOrderedTriple t -> 
    (MinmaxEffortIndicator t, PartialCompareEffortIndicator t) -> 
    Bool
propRoundedLatticeMeetAssocative _ 
        (UniformlyOrderedTriple (e1,e2,e3)) 
        (minmaxEffort, effortComp)
    = 
    roundedAssociative (pLeqEff effortComp) (minUpEff minmaxEffort) (minDnEff minmaxEffort) e1 e2 e3

{- optional properties: -}
propRoundedLatticeModular :: 
    (PartialComparison t, RoundedLattice t, Show t, HasLegalValues t) => 
    t -> 
    UniformlyOrderedTriple t -> 
    (MinmaxEffortIndicator t, PartialCompareEffortIndicator t) -> 
    Bool
propRoundedLatticeModular _ 
        (UniformlyOrderedTriple (e1,e2,e3)) 
        (minmaxEffort, effortComp)
    = 
    roundedModular (pLeqEff effortComp) 
        (maxUpEff minmaxEffort) (minUpEff minmaxEffort) 
        (maxDnEff minmaxEffort) (minDnEff minmaxEffort) 
        e1 e2 e3

propRoundedLatticeDistributive :: 
    (PartialComparison t, RoundedLattice t, Show t, HasLegalValues t) => 
    t -> 
    UniformlyOrderedTriple t -> 
    (MinmaxEffortIndicator t, PartialCompareEffortIndicator t) -> 
    Bool
propRoundedLatticeDistributive _ 
        (UniformlyOrderedTriple (e1,e2,e3)) 
        (minmaxEffort, effortComp)
    = 
    (roundedLeftDistributive (pLeqEff effortComp) 
        (maxUpEff minmaxEffort) (minUpEff minmaxEffort) 
        (maxDnEff minmaxEffort) (minDnEff minmaxEffort) 
        e1 e2 e3)
    && 
    (roundedLeftDistributive (pLeqEff effortComp) 
        (minUpEff minmaxEffort) (maxUpEff minmaxEffort) 
        (minDnEff minmaxEffort) (maxDnEff minmaxEffort) 
        e1 e2 e3)

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
    testGroup (name ++ " (min,max) rounded") $
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
         testProperty "meet idempotent" (area, propRoundedLatticeMeetIdempotent sample)
        ,
         testProperty "meet commutative" (area, propRoundedLatticeMeetCommutative sample)
        ,
         testProperty "meet associative" (area, propRoundedLatticeMeetAssocative sample)
        ,
         testProperty "distributive" (area, propRoundedLatticeDistributive sample)
        ]

        