{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-|
    Module      :  Numeric.AERN.NumericOrder.RefinementRoundedLattice
    Description :  lattices over numerical order but with refinement order rounding  
    Copyright   :  (c) Michal Konecny, Jan Duracz
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Lattices over numerical order but with refinement order rounding.
    
    This module is hidden and reexported via its parent NumericOrder. 
-}
module Numeric.AERN.NumericOrder.RefinementRoundedLattice 
(
    RefinementRoundedLattice(..),
    minOut, maxOut, minIn, maxIn,
    RefinementRoundedLatticeEffort(..),
    testsRefinementRoundedLattice, 
    testsRefinementRoundedLatticeDistributive,
    testsRefinementRoundedLatticeDistributiveMonotone
)
where

import Prelude hiding ((<=))

import Numeric.AERN.Basics.Arbitrary
import Numeric.AERN.Basics.Exception 

import Numeric.AERN.Basics.Effort
import Numeric.AERN.Basics.PartialOrdering
import Numeric.AERN.NumericOrder.Arbitrary 
import Numeric.AERN.NumericOrder.PartialComparison 
import Numeric.AERN.NumericOrder.Extrema

import qualified Numeric.AERN.RefinementOrder as RefOrd

import Numeric.AERN.Basics.Laws.PartialRelation
import Numeric.AERN.Basics.Laws.RoundedOperation
import Numeric.AERN.Basics.Laws.OperationRelation

import Numeric.AERN.Misc.Maybe

import Test.QuickCheck
import Test.Framework (testGroup, Test)
import Test.Framework.Providers.QuickCheck2 (testProperty)

class
    (EffortIndicator (MinmaxInOutEffortIndicator t))
    => 
    RefinementRoundedLatticeEffort t 
    where
    type MinmaxInOutEffortIndicator t
    minmaxInOutDefaultEffort :: t -> MinmaxInOutEffortIndicator t

{-|
    A type with refinement rounding numerical order lattice operations.
-}
class (RefinementRoundedLatticeEffort t) => RefinementRoundedLattice t where
    maxInEff :: MinmaxInOutEffortIndicator t -> t -> t -> t
    maxOutEff :: MinmaxInOutEffortIndicator t -> t -> t -> t
    minInEff :: MinmaxInOutEffortIndicator t -> t -> t -> t
    minOutEff :: MinmaxInOutEffortIndicator t -> t -> t -> t

-- | Outward rounded minimum with default effort
minOut :: (RefinementRoundedLattice t) => t -> t -> t
minOut a = minOutEff (minmaxInOutDefaultEffort a) a

-- | Outward rounded maximum with default effort
maxOut :: (RefinementRoundedLattice t) => t -> t -> t
maxOut a = maxOutEff (minmaxInOutDefaultEffort a) a

-- | Inward rounded minimum with default effort
minIn :: (RefinementRoundedLattice t) => t -> t -> t
minIn a = minInEff (minmaxInOutDefaultEffort a) a

-- | Inward rounded maximum with default effort
maxIn :: (RefinementRoundedLattice t) => t -> t -> t
maxIn a = maxInEff (minmaxInOutDefaultEffort a) a

propRefinementRoundedLatticeJoinIdempotent :: 
    (RefOrd.PartialComparison t, RefinementRoundedLattice t, Show t, HasLegalValues t) => 
    t ->
    (UniformlyOrderedSingleton t) -> 
    (RefOrd.PartialCompareEffortIndicator t, 
     MinmaxInOutEffortIndicator t) -> 
    Bool
propRefinementRoundedLatticeJoinIdempotent _ 
        (UniformlyOrderedSingleton e) 
        (effortComp, effortInOut) 
    =
    roundedIdempotent (RefOrd.pLeqEff effortComp) 
        (maxInEff effortInOut) (maxOutEff effortInOut) e

propRefinementRoundedLatticeJoinCommutative :: 
    (RefOrd.PartialComparison t, RefinementRoundedLattice t, Show t, HasLegalValues t) => 
    t -> 
    UniformlyOrderedPair t -> 
    (RefOrd.PartialCompareEffortIndicator t, 
     MinmaxInOutEffortIndicator t) -> 
    Bool
propRefinementRoundedLatticeJoinCommutative _ 
        (UniformlyOrderedPair (e1,e2)) 
        (effortComp, effortInOut)
    = 
    roundedCommutative (RefOrd.pLeqEff effortComp) 
        (maxInEff effortInOut) (maxOutEff effortInOut) e1 e2

propRefinementRoundedLatticeJoinAssocative :: 
    (RefOrd.PartialComparison t, RefinementRoundedLattice t, Show t, HasLegalValues t) => 
    t -> 
    UniformlyOrderedTriple t -> 
    (RefOrd.PartialCompareEffortIndicator t, 
     MinmaxInOutEffortIndicator t) -> 
    Bool
propRefinementRoundedLatticeJoinAssocative _ 
        (UniformlyOrderedTriple (e1,e2,e3)) 
        (effortComp, effortInOut)
    = 
    roundedAssociative (RefOrd.pLeqEff effortComp) 
        (maxInEff effortInOut) (maxOutEff effortInOut) e1 e2 e3

propRefinementRoundedLatticeMeetIdempotent :: 
    (RefOrd.PartialComparison t, RefinementRoundedLattice t, Show t, HasLegalValues t) => 
    t -> 
    (UniformlyOrderedSingleton t) -> 
    (RefOrd.PartialCompareEffortIndicator t, 
     MinmaxInOutEffortIndicator t) -> 
    Bool
propRefinementRoundedLatticeMeetIdempotent _ 
        (UniformlyOrderedSingleton e) 
        (effortComp, effortInOut) 
    = 
    roundedIdempotent (RefOrd.pLeqEff effortComp) 
        (minInEff effortInOut) (minOutEff effortInOut) e

propRefinementRoundedLatticeMeetCommutative :: 
    (RefOrd.PartialComparison t, RefinementRoundedLattice t, Show t, HasLegalValues t) => 
    t -> 
    UniformlyOrderedPair t -> 
    (RefOrd.PartialCompareEffortIndicator t, 
     MinmaxInOutEffortIndicator t) -> 
    Bool
propRefinementRoundedLatticeMeetCommutative _  
        (UniformlyOrderedPair (e1,e2)) 
        (effortComp, effortInOut)
    = 
    roundedCommutative (RefOrd.pLeqEff effortComp) 
        (minInEff effortInOut) (minOutEff effortInOut) e1 e2

propRefinementRoundedLatticeMeetAssocative :: 
    (RefOrd.PartialComparison t, RefinementRoundedLattice t, Show t, HasLegalValues t) => 
    t -> 
    UniformlyOrderedTriple t -> 
    (RefOrd.PartialCompareEffortIndicator t, 
     MinmaxInOutEffortIndicator t) -> 
    Bool
propRefinementRoundedLatticeMeetAssocative _ 
        (UniformlyOrderedTriple (e1,e2,e3)) 
        (effortComp, effortInOut)
    = 
    roundedAssociative  (RefOrd.pLeqEff effortComp) 
        (minInEff effortInOut) (minOutEff effortInOut) e1 e2 e3

{- optional properties: -}
propRefinementRoundedLatticeModular :: 
    (RefOrd.PartialComparison t, RefinementRoundedLattice t, Show t, HasLegalValues t) => 
    t -> 
    UniformlyOrderedTriple t -> 
    (RefOrd.PartialCompareEffortIndicator t, 
     MinmaxInOutEffortIndicator t) -> 
    Bool
propRefinementRoundedLatticeModular _ 
        (UniformlyOrderedTriple (e1,e2,e3)) 
        (effortComp, effortInOut)
    = 
    roundedModular (RefOrd.pLeqEff effortComp) 
        (maxInEff effortInOut) (minInEff effortInOut)
        (maxOutEff effortInOut) (minOutEff effortInOut)
        e1 e2 e3

propRefinementRoundedLatticeDistributive :: 
    (RefOrd.PartialComparison t, RefinementRoundedLattice t,
     Show t, HasLegalValues t) => 
    t -> 
    UniformlyOrderedTriple t -> 
    (RefOrd.PartialCompareEffortIndicator t, 
     MinmaxInOutEffortIndicator t) -> 
    Bool
propRefinementRoundedLatticeDistributive _ 
        (UniformlyOrderedTriple (e1,e2,e3)) 
        (effortComp, effortInOut)
    = 
    (roundedModular (RefOrd.pLeqEff effortComp) 
        (maxInEff effortInOut) (minInEff effortInOut)
        (maxOutEff effortInOut) (minOutEff effortInOut)
        e1 e2 e3)
    && 
    (roundedModular (RefOrd.pLeqEff effortComp) 
        (minInEff effortInOut) (maxInEff effortInOut)
        (minOutEff effortInOut) (maxOutEff effortInOut)
        e1 e2 e3)
    
propRefinementRoundedLatticeJoinMonotone ::
    (RefinementRoundedLattice t, RefOrd.PartialComparison t, 
     Show t, HasLegalValues t) => 
    t -> 
    RefOrd.TwoLEPairs t -> 
    (RefOrd.PartialCompareEffortIndicator t, 
     MinmaxInOutEffortIndicator t) ->
    Bool
propRefinementRoundedLatticeJoinMonotone _ 
        (RefOrd.TwoLEPairs ((e1Lower,e1),(e2Lower,e2))) 
        (effortComp, effortInOut)
    =
    case RefOrd.pLeqEff effortComp rLower r of
        Just b -> b
        Nothing -> True
    where
    rLower = maxOutEff effortInOut e1Lower e2Lower 
    r = maxInEff effortInOut e1 e2 
    
propRefinementRoundedLatticeMeetMonotone ::
    (RefinementRoundedLattice t, RefOrd.PartialComparison t, 
     Show t, HasLegalValues t) => 
    t -> 
    RefOrd.TwoLEPairs t -> 
    (RefOrd.PartialCompareEffortIndicator t, 
     MinmaxInOutEffortIndicator t) -> 
    Bool
propRefinementRoundedLatticeMeetMonotone _ 
        (RefOrd.TwoLEPairs ((e1Lower,e1),(e2Lower,e2))) 
        (effortComp, effortInOut)
    =
    case RefOrd.pLeqEff effortComp rLower r of
        Just b -> b
        Nothing -> True
    where
    rLower = minOutEff effortInOut e1Lower e2Lower 
    r = minInEff effortInOut e1 e2 
    
mkTestGroupLattice name = testGroup (name ++ " (min,max) treated as refinement rounded")
    
testsRefinementRoundedLattice :: 
    (RefOrd.PartialComparison t,
     RefOrd.ArbitraryOrderedTuple t,
     HasExtrema t,
     RefinementRoundedLattice t,
     ArbitraryOrderedTuple t,
     Show t, HasLegalValues t
     ) =>
    (String, t) -> 
    Area t -> 
    Test
testsRefinementRoundedLattice (name, sample) area =
    mkTestGroupLattice name (testsRefinementRoundedLatticeL area sample)

testsRefinementRoundedLatticeDistributive :: 
    (RefOrd.PartialComparison t,
     RefOrd.ArbitraryOrderedTuple t,
     HasExtrema t,
     RefinementRoundedLattice t,
     ArbitraryOrderedTuple t,
     Show t, HasLegalValues t
     ) => 
    (String, t) -> 
    Area t -> 
    Test
testsRefinementRoundedLatticeDistributive (name, sample) area =
    mkTestGroupLattice name (testsRefinementRoundedLatticeDistributiveL area sample)

testsRefinementRoundedLatticeDistributiveMonotone :: 
    (RefOrd.PartialComparison t,
     RefOrd.ArbitraryOrderedTuple t,
     HasExtrema t,
     RefinementRoundedLattice t,
     ArbitraryOrderedTuple t,
     Show t, HasLegalValues t 
     ) => 
    (String, t) -> 
    Area t -> 
    Test
testsRefinementRoundedLatticeDistributiveMonotone (name, sample) area =
    mkTestGroupLattice name (testsRefinementRoundedLatticeDistributiveMonotoneL area sample)

testsRefinementRoundedLatticeL area sample =    
        [
         testProperty "join idempotent" (area, propRefinementRoundedLatticeJoinIdempotent sample)
        ,
         testProperty "join commutative" (area, propRefinementRoundedLatticeJoinCommutative sample)
        ,
         testProperty "join associative" (area, propRefinementRoundedLatticeJoinAssocative sample)
        ,
         testProperty "meet idempotent" (area, propRefinementRoundedLatticeMeetIdempotent sample)
        ,
         testProperty "meet commutative" (area, propRefinementRoundedLatticeMeetCommutative sample)
        ,
         testProperty "meet associative" (area, propRefinementRoundedLatticeMeetAssocative sample)
        ]
        
testsRefinementRoundedLatticeDistributiveL area sample =
    testsRefinementRoundedLatticeL area sample ++
        [    
         testProperty "distributive" (area, propRefinementRoundedLatticeDistributive sample)
        ]
        
testsRefinementRoundedLatticeDistributiveMonotoneL area sample =
    testsRefinementRoundedLatticeDistributiveL area sample ++
        [    
         testProperty "join monotone" (area, propRefinementRoundedLatticeJoinMonotone sample)
        ,
         testProperty "meet monotone" (area, propRefinementRoundedLatticeMeetMonotone sample)
        ]
    