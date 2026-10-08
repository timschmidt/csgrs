{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE UndecidableInstances #-}
{-|
    Module      :  Numeric.AERN.RealArithmetic.RefinementOrderRounding.FieldOps
    Description :  rounded addition and multiplication  
    Copyright   :  (c) Michal Konecny, Jan Duracz
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Rounded addition and multiplication.
    
    This module is hidden and reexported via its parent RefinementOrderRounding. 
-}
module Numeric.AERN.RealArithmetic.RefinementOrderRounding.FieldOps 
where

import Prelude hiding (EQ, LT, GT)
import Numeric.AERN.Basics.PartialOrdering

import Numeric.AERN.RealArithmetic.Auxiliary
import Numeric.AERN.RealArithmetic.ExactOps
import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as ArithUpDn
import Numeric.AERN.RealArithmetic.RefinementOrderRounding.Conversion

import Numeric.AERN.Basics.Effort
import Numeric.AERN.Basics.Exception (HasLegalValues)
import Numeric.AERN.Basics.Consistency

import Numeric.AERN.RealArithmetic.Laws
import Numeric.AERN.RealArithmetic.Measures
import qualified Numeric.AERN.RefinementOrder as RefOrd
import qualified Numeric.AERN.NumericOrder as NumOrd

import Test.QuickCheck
import Test.Framework (testGroup, Test)
import Test.Framework.Providers.QuickCheck2 (testProperty)

import Data.Maybe

infixl 6 <+>, >+<, <->, >-<
infixl 7 <*>, >*<
infixl 8 <^>, >^<
infixl 7 </>, >/<

class
    (EffortIndicator (AddEffortIndicator t))
    => 
    RoundedAddEffort t 
    where
    type AddEffortIndicator t
    addDefaultEffort :: t -> AddEffortIndicator t

class (RoundedAddEffort t) => RoundedAdd t where
    addInEff :: AddEffortIndicator t -> t -> t -> t
    addOutEff :: AddEffortIndicator t -> t -> t -> t

-- | Inward rounded addition with default effort
addIn :: (RoundedAdd t) => t -> t -> t
addIn a = addInEff (addDefaultEffort a) a

-- | Inward rounded addition with default effort
(>+<) :: (RoundedAdd t) => t -> t -> t
(>+<) = addIn

-- | Outward rounded addition with default effort
addOut :: (RoundedAdd t) => t -> t -> t
addOut a = addOutEff (addDefaultEffort a) a

-- | Outward rounded addition with default effort
(<+>) :: (RoundedAdd t) => t -> t -> t
(<+>) = addOut

--propAddRefIsotone _ effortDist

propInOutAddZero ::
    (RefOrd.PartialComparison t, RoundedAdd t, HasZero t,
     Show t, HasLegalValues t) 
    =>
    t ->
    (RefOrd.UniformlyOrderedSingleton t) -> 
    (RefOrd.PartialCompareEffortIndicator t, 
     AddEffortIndicator t) -> 
    Bool
propInOutAddZero sample (RefOrd.UniformlyOrderedSingleton e) effort =
    roundedUnit (zero sample) RefOrd.pLeqEff addInEff addOutEff effort e

propInOutAddCommutative ::
    (RefOrd.PartialComparison t, RoundedAdd t, HasZero t,
     Show t, HasLegalValues t)
    =>
    t ->
    (RefOrd.UniformlyOrderedPair t) -> 
    (RefOrd.PartialCompareEffortIndicator t, 
     AddEffortIndicator t) -> 
    Bool
propInOutAddCommutative _ (RefOrd.UniformlyOrderedPair (e1,e2)) effort =
    roundedCommutative RefOrd.pLeqEff addInEff addOutEff effort e1 e2

propInOutAddAssociative ::
    (RefOrd.PartialComparison t, RoundedAdd t, HasZero t,
     Show t, HasLegalValues t) 
    =>
    t ->
    (RefOrd.UniformlyOrderedTriple t) -> 
    (RefOrd.PartialCompareEffortIndicator t, 
     AddEffortIndicator t) -> 
    Bool
propInOutAddAssociative _ (RefOrd.UniformlyOrderedTriple (e1,e2,e3)) effort =
    roundedAssociative RefOrd.pLeqEff addInEff addOutEff effort e1 e2 e3

propInOutAddRefIsotone ::
    (RefOrd.PartialComparison t, RoundedAdd t, 
     Show t, HasLegalValues t,
     RefOrd.ArbitraryOrderedTuple t)
    =>
    t ->
    (RefOrd.TwoLEPairs t) -> 
    (AddEffortIndicator t) -> 
    (RefOrd.PartialCompareEffortIndicator t) ->
    Bool
propInOutAddRefIsotone _ =
    roundedRefinementIsotone2 "addition" addInEff addOutEff

testsInOutAdd (name, sample) area =
    testGroup (name ++ " >+< <+>") $
        [
            testProperty "0 absorbs" (area, propInOutAddZero sample)
        ,
            testProperty "commutative" (area, propInOutAddCommutative sample)
        ,
            testProperty "associative" (area, propInOutAddAssociative sample)
        ,
            testProperty "refinement isotone" (area, propInOutAddRefIsotone sample)
        ]


class (RoundedAdd t, Neg t) => RoundedSubtr t where
    subtrInEff :: (AddEffortIndicator t) -> t -> t -> t
    subtrOutEff :: (AddEffortIndicator t) -> t -> t -> t
    subtrInEff effort a b = addInEff effort a (neg b)
    subtrOutEff effort a b = addOutEff effort a (neg b)

-- | Inward rounded subtraction with default effort
subtrIn :: (RoundedSubtr t) => t -> t -> t
subtrIn d = subtrInEff (addDefaultEffort d) d

-- | Inward rounded subtraction with default effort
(>-<) :: (RoundedSubtr t) => t -> t -> t
(>-<) = subtrIn

-- | Outward rounded subtraction with default effort
subtrOut :: (RoundedSubtr t) => t -> t -> t
subtrOut d = subtrOutEff (addDefaultEffort d) d

-- | Outward rounded subtraction with default effort
(<->) :: (RoundedSubtr t) => t -> t -> t
(<->) = subtrOut


propInOutSubtrElim ::
    (RefOrd.PartialComparison t, RoundedSubtr t, HasZero t,
     Show t, HasLegalValues t) 
    =>
    t ->
    (RefOrd.UniformlyOrderedSingleton t) -> 
    (RefOrd.PartialCompareEffortIndicator t, 
     AddEffortIndicator t) -> 
    Bool
propInOutSubtrElim sample (RefOrd.UniformlyOrderedSingleton e) effort =
    roundedReflexiveCollapse (zero sample) RefOrd.pLeqEff subtrInEff subtrOutEff effort e

propInOutSubtrNegAdd ::
    (RefOrd.PartialComparison t, RoundedSubtr t, Neg t,
     Show t, HasLegalValues t) 
    =>
    t ->
    (RefOrd.UniformlyOrderedPair t) -> 
    (RefOrd.PartialCompareEffortIndicator t, 
     AddEffortIndicator t) -> 
    Bool
propInOutSubtrNegAdd _ (RefOrd.UniformlyOrderedPair (e1, e2)) initEffort =
    equalRoundingUpDn "a+b=a-(-b)"
        expr1Up expr1Dn expr2Up expr2Dn 
        RefOrd.pLeqEff initEffort
    where
    expr1Up eff =
        let (>-<) = subtrInEff eff in e1 >-< (neg e2)
    expr1Dn eff =
        let (<->) = subtrOutEff eff in e1 <-> (neg e2)
    expr2Up eff =
        let (>+<) = addInEff eff in e1 >+< e2
    expr2Dn eff =
        let (<+>) = addOutEff eff in e1 <+> e2

propInOutSubtrRefIsotone ::
    (RefOrd.PartialComparison t, RoundedSubtr t, 
     Show t, HasLegalValues t,
     RefOrd.ArbitraryOrderedTuple t) 
    =>
    t ->
    (RefOrd.TwoLEPairs t) -> 
    (AddEffortIndicator t) -> 
    (RefOrd.PartialCompareEffortIndicator t) ->
    Bool
propInOutSubtrRefIsotone _ =
    roundedRefinementIsotone2 "subtraction" subtrInEff subtrOutEff

testsInOutSubtr (name, sample) area =
    testGroup (name ++ " >-< <->") $
        [
--            testProperty "a-a=0" (propInOutSubtrElim sample)
--            ,
            testProperty "a+b=a-(-b)" (area, propInOutSubtrNegAdd sample)
            ,
            testProperty "refinement isotone" (area, propInOutSubtrRefIsotone sample)
        ]


class
    (EffortIndicator (AbsEffortIndicator t))
    => 
    RoundedAbsEffort t 
    where
    type AbsEffortIndicator t
    absDefaultEffort :: t -> AbsEffortIndicator t

class (RoundedAbsEffort t) => RoundedAbs t where
    absInEff :: (AbsEffortIndicator t) -> t -> t
    absOutEff :: (AbsEffortIndicator t) -> t -> t

-- | Inward rounded absolute value with default effort
absIn :: (RoundedAbs t) => t -> t
absIn d = absInEff (absDefaultEffort d) d

-- | Outward rounded absolute value with default effort
absOut :: (RoundedAbs t) => t -> t
absOut d = absOutEff (absDefaultEffort d) d


absOutUsingCompMax ::
    (HasZero t, Neg t, 
     NumOrd.PartialComparison t, NumOrd.RefinementRoundedLattice t) =>
    (NumOrd.PartialCompareEffortIndicator t,
     NumOrd.MinmaxInOutEffortIndicator t) ->
    t -> t 
absOutUsingCompMax (effortComp, effortMinmax) a =
    case NumOrd.pCompareEff effortComp (zero a) a of
        Just EQ -> a
        Just LT -> a
--        Just LEE -> a
        Just GT -> neg a
--        Just GEE -> neg a
        _ -> (zero a) `max` (a `max` (neg a))
    where
    max = NumOrd.maxOutEff effortMinmax

absInUsingCompMax ::
    (HasZero t, Neg t, 
     NumOrd.PartialComparison t, NumOrd.RefinementRoundedLattice t) 
    =>
    (NumOrd.PartialCompareEffortIndicator t,
     NumOrd.MinmaxInOutEffortIndicator t) ->
    t -> t 
absInUsingCompMax (effortComp, effortMinmax) a =
    case NumOrd.pCompareEff effortComp (zero a) a of
        Just EQ -> a
        Just LT -> a
--        Just LEE -> a
        Just GT -> neg a
--        Just GEE -> neg a
        _ -> (zero a) `max` (a `max` (neg a))
    where
    max = NumOrd.maxInEff effortMinmax

propInOutAbsNegSymmetric ::
    (RefOrd.PartialComparison t, RoundedAbs t, HasZero t, Neg t,
     Show t, HasLegalValues t) 
    =>
    t ->
    (RefOrd.UniformlyOrderedSingleton t) -> 
    (RefOrd.PartialCompareEffortIndicator t, 
     AbsEffortIndicator t) -> 
    Bool
propInOutAbsNegSymmetric _ (RefOrd.UniformlyOrderedSingleton e) effort =
    roundedNegSymmetric RefOrd.pLeqEff absInEff absOutEff effort e

propInOutAbsIdempotent ::
    (RefOrd.PartialComparison t, RoundedAbs t, HasZero t,
     Show t, HasLegalValues t) 
    =>
    t ->
    (RefOrd.UniformlyOrderedSingleton t) -> 
    (RefOrd.PartialCompareEffortIndicator t, 
     AbsEffortIndicator t) -> 
    Bool
propInOutAbsIdempotent _ (RefOrd.UniformlyOrderedSingleton e) effort =
    roundedIdempotent RefOrd.pLeqEff absInEff absOutEff effort e

propInOutAbsRefIsotone ::
    (RefOrd.PartialComparison t, RoundedAbs t,
     RefOrd.ArbitraryOrderedTuple t,  
     Show t, HasLegalValues t) 
    =>
    t ->
    (RefOrd.LEPair t) -> 
    (AbsEffortIndicator t) -> 
    (RefOrd.PartialCompareEffortIndicator t) ->
    Bool
propInOutAbsRefIsotone _ =
    roundedRefinementIsotone1 "abs" absInEff absOutEff

testsInOutAbs (name, sample) area =
    testGroup (name ++ " in/out rounded abs") $
        [
            testProperty "neg -> no change" (area, propInOutAbsNegSymmetric sample)
        ,
            testProperty "idempotent" (area, propInOutAbsIdempotent sample)
        ,
            testProperty "refinement monotone" (area, propInOutAbsRefIsotone sample)
        ]


class
    (EffortIndicator (MultEffortIndicator t))
    => 
    RoundedMultiplyEffort t 
    where
    type MultEffortIndicator t
    multDefaultEffort :: t -> MultEffortIndicator t

class (RoundedMultiplyEffort t) => RoundedMultiply t where
    multInEff :: MultEffortIndicator t -> t -> t -> t
    multOutEff :: MultEffortIndicator t -> t -> t -> t

-- | Inward rounded multiplication with default effort
multIn :: (RoundedMultiply t) => t -> t -> t
multIn a = multInEff (multDefaultEffort a) a

-- | Inward rounded multiplication with default effort
(>*<) :: (RoundedMultiply t) => t -> t -> t
(>*<) = multIn

-- | Outward rounded multiplication with default effort
multOut :: (RoundedMultiply t) => t -> t -> t
multOut a = multOutEff (multDefaultEffort a) a

-- | Outward rounded multiplication with default effort
(<*>) :: (RoundedMultiply t) => t -> t -> t
(<*>) = multOut


propInOutMultRefIsotone ::
    (RefOrd.PartialComparison t, RoundedMultiply t, 
     Show t, HasLegalValues t,
     RefOrd.ArbitraryOrderedTuple t) 
    =>
    t ->
    (RefOrd.TwoLEPairs t) -> 
    (MultEffortIndicator t) -> 
    (RefOrd.PartialCompareEffortIndicator t) ->
    Bool
propInOutMultRefIsotone _ =
    roundedRefinementIsotone2 "multiplication" multInEff multOutEff

propInOutMultOne ::
    (RefOrd.PartialComparison t, RoundedMultiply t, HasOne t,
     Show t, HasLegalValues t) 
    =>
    t ->
    (RefOrd.UniformlyOrderedSingleton t) -> 
    (RefOrd.PartialCompareEffortIndicator t, 
     MultEffortIndicator t) -> 
    Bool
propInOutMultOne sample (RefOrd.UniformlyOrderedSingleton e) effort =
    roundedUnit (one sample) RefOrd.pLeqEff multInEff multOutEff effort e

propInOutMultCommutative ::
    (RefOrd.PartialComparison t, RoundedMultiply t, HasZero t,
     Show t, HasLegalValues t) 
    =>
    t ->
    (RefOrd.UniformlyOrderedPair t) -> 
    (RefOrd.PartialCompareEffortIndicator t, 
     MultEffortIndicator t) -> 
    Bool
propInOutMultCommutative _ (RefOrd.UniformlyOrderedPair (e1,e2)) effort =
    roundedCommutative RefOrd.pLeqEff multInEff multOutEff effort e1 e2
       
propInOutMultAssociative ::
    (RefOrd.PartialComparison t, 
     RoundedMultiply t, HasZero t,
     Show t, HasLegalValues t) 
    =>
    t ->
    (RefOrd.UniformlyOrderedTriple t) -> 
    (RefOrd.PartialCompareEffortIndicator t, 
     MultEffortIndicator t) -> 
    Bool
propInOutMultAssociative _ (RefOrd.UniformlyOrderedTriple (e1,e2,e3)) effort =
    roundedAssociative RefOrd.pLeqEff multInEff multOutEff effort e1 e2 e3

propInOutMultDistributesOverAdd ::
    (RefOrd.PartialComparison t,
     RoundedMultiply t,  RoundedAdd t,
     HasAntiConsistency t, Show t, HasLegalValues t) 
    =>
    t ->
    (RefOrd.UniformlyOrderedTriple t) -> 
    (ConsistencyEffortIndicator t) ->
    (RefOrd.PartialCompareEffortIndicator t, 
     (MultEffortIndicator t, AddEffortIndicator t)) -> 
    Bool
propInOutMultDistributesOverAdd _ (RefOrd.UniformlyOrderedTriple (e1,e2,e3)) effortConst effort =
    roundedDistributive 
        RefOrd.pLeqEff 
        multInEff addInEff multOutEff addOutEff
        effortConst effort e1 e2 e3
       
    
testsInOutMult (name, sample) area =
    testGroup (name ++ " >*< <*>") $
        [
            testProperty "1 absorbs" (area, propInOutMultOne sample)
        ,
            testProperty "commutative" (area, propInOutMultCommutative sample)
        ,
            testProperty "associative" (area, propInOutMultAssociative sample)
        ,
            testProperty "weakly distributes over +" (area, propInOutMultDistributesOverAdd sample)
        ,
            testProperty "refinement monotone" (area, propInOutMultRefIsotone sample)
        ]

class
    (EffortIndicator (PowerToNonnegIntEffortIndicator t))
    => 
    RoundedPowerToNonnegIntEffort t 
    where
    type PowerToNonnegIntEffortIndicator t
    powerToNonnegIntDefaultEffort :: 
        t -> PowerToNonnegIntEffortIndicator t 

class (RoundedPowerToNonnegIntEffort t) => RoundedPowerToNonnegInt t where
    powerToNonnegIntInEff ::
        (PowerToNonnegIntEffortIndicator t) -> 
        t {-^ @x@ -} -> 
        Int {-^ @n@ (assumed >=0)-} -> 
        t {-^ @x^n@ rounded inwards -}
    powerToNonnegIntOutEff ::
        (PowerToNonnegIntEffortIndicator t) -> 
        t {-^ @x@ -} -> 
        Int {-^ @n@ (assumed >=0)-} -> 
        t {-^ @x^n@ rounded outwards -}

-- | Inward rounded integer power with default effort
powerToNonnegIntIn :: (RoundedPowerToNonnegInt t) => t -> Int -> t
powerToNonnegIntIn a = powerToNonnegIntInEff (powerToNonnegIntDefaultEffort a) a

-- | Inward rounded integer power with default effort
(>^<) :: (RoundedPowerToNonnegInt t) => t -> Int -> t 
(>^<) = powerToNonnegIntIn

-- | Outward rounded integer power with default effort
powerToNonnegIntOut :: (RoundedPowerToNonnegInt t) => t -> Int -> t
powerToNonnegIntOut a = powerToNonnegIntOutEff (powerToNonnegIntDefaultEffort a) a

-- | Outward rounded integer power with default effort
(<^>) :: (RoundedPowerToNonnegInt t) => t -> Int -> t
(<^>) = powerToNonnegIntOut


-- functions providing an implementation derived from rounded multiplication: 
        
type PowerToNonnegIntEffortIndicatorFromMult t =
    MultEffortIndicator t
    
powerToNonnegIntDefaultEffortFromMult a =
    multDefaultEffort a

powerToNonnegIntInEffFromMult ::
    (RoundedMultiply t, HasOne t) => 
    PowerToNonnegIntEffortIndicatorFromMult t -> 
    t -> Int -> t
powerToNonnegIntInEffFromMult effMult e n =
    powerFromMult (one e) (multInEff effMult) e n

powerToNonnegIntOutEffFromMult ::
    (RoundedMultiply t, HasOne t) => 
    PowerToNonnegIntEffortIndicatorFromMult t -> 
    t -> Int -> t
powerToNonnegIntOutEffFromMult effMult e n =
    powerFromMult (one e) (multOutEff effMult) e n

propInOutPowerRefIsotone ::
    (RefOrd.PartialComparison t, RoundedPowerToNonnegInt t,
     RefOrd.ArbitraryOrderedTuple t,  
     Show t, HasLegalValues t) 
    =>
    t ->
    (RefOrd.LEPair t) -> 
    Int ->
    (PowerToNonnegIntEffortIndicator t) -> 
    (RefOrd.PartialCompareEffortIndicator t) ->
    Bool
propInOutPowerRefIsotone _ pair nR =
    roundedRefinementIsotone1 "non-neg integer power" powerNInEff powerNOutEff pair
    where
    n = nR `mod` 10
    powerNInEff eff x = powerToNonnegIntInEff eff x n
    powerNOutEff eff x = powerToNonnegIntOutEff eff x n


propInOutPowerSumExponents ::
    (RefOrd.PartialComparison t,
     RoundedPowerToNonnegInt t, RoundedMultiply t, 
     HasOne t, HasAntiConsistency t, Show t, HasLegalValues t) 
    =>
    t ->
    (RefOrd.UniformlyOrderedSingleton t) -> 
    (ConsistencyEffortIndicator t) -> 
    (RefOrd.PartialCompareEffortIndicator t,
     (PowerToNonnegIntEffortIndicator t,
      MultEffortIndicator t)) ->
    Int -> Int -> Bool
propInOutPowerSumExponents _ (RefOrd.UniformlyOrderedSingleton a) 
        effortConsistency initEffort 
        nR mR =
    thinEqualConsLeqRoundingUpDnImprovement "a^n * a^m ⊑/⊒ a^(n+m)" [a]
        expr1Up expr1Dn expr2Up expr2Dn 
        RefOrd.pLeqEff
        effortConsistency 
        initEffort
    where
    n = nR `mod` 10
    m = mR `mod` 10
    expr1Up (effPower, effMult) =
        let (>^<) = powerToNonnegIntInEff effPower in
        let (>*<) = multInEff effMult in
        (a >^< n) >*< (a >^< m)
    expr1Dn (effPower, effMult) =
        let (<^>) = powerToNonnegIntOutEff effPower in
        let (<*>) = multOutEff effMult in
        (a <^> n) <*> (a <^> m)
    expr2Up (effPower, effMult) =
        let (>^<) = powerToNonnegIntInEff effPower in
        a >^< (n + m)
    expr2Dn (effPower, effMult) =
        let (<^>) = powerToNonnegIntOutEff effPower in
        a <^> (n + m)

testsInOutIntPower (name, sample) area =
    testGroup (name ++ " non-negative integer power") $
        [
            testProperty "a^n * a^m ⊑/⊒ a^(n+m)" (area, propInOutPowerSumExponents sample)
            ,
            testProperty "refinement monotone" (area, propInOutPowerRefIsotone sample)
--            ,
--            testProperty "a/b=a*(1/b)" (propUpDnDivRecipMult sample)
        ]

class
    (EffortIndicator (DivEffortIndicator t))
    => 
    RoundedDivideEffort t 
    where
    type DivEffortIndicator t
    divDefaultEffort :: t -> DivEffortIndicator t

class (HasOne t, RoundedDivideEffort t) => RoundedDivide t where
    divInEff :: DivEffortIndicator t -> t -> t -> t
    divOutEff :: DivEffortIndicator t -> t -> t -> t
--    divAssumePositiveOutEff :: DivEffortIndicator t -> t -> t -> t
--    divAssumePositiveInEff :: DivEffortIndicator t -> t -> t -> t
    recipInEff :: DivEffortIndicator t -> t -> t
    recipOutEff :: DivEffortIndicator t -> t -> t
    recipInEff eff a = divInEff eff (one a) a
    recipOutEff eff a = divOutEff eff (one a) a

-- | Inward rounded division with default effort
divIn :: (RoundedDivide t) => t -> t -> t
divIn a = divInEff (divDefaultEffort a) a

-- | Inward rounded division with default effort
(>/<) :: (RoundedDivide t) => t -> t -> t
(>/<) = divIn

-- | Outward rounded division with default effort
divOut :: (RoundedDivide t) => t -> t -> t
divOut a = divOutEff (divDefaultEffort a) a

-- | Outward rounded division with default effort
(</>) :: (RoundedDivide t) => t -> t -> t
(</>) = divOut

{-
-- | Inward rounded division assuming positive argument, with default effort
divAssumePositiveIn :: (RoundedDivide t) => t -> t -> t
divAssumePositiveIn a = divAssumePositiveInEff (divDefaultEffort a) a

(>/+<) :: (RoundedDivide t) => t -> t -> t
(>/+<) = divAssumePositiveIn

-- | Outward rounded division assuming positive argument, with default effort
divAssumePositiveOut :: (RoundedDivide t) => t -> t -> t
divAssumePositiveOut a = divAssumePositiveOutEff (divDefaultEffort a) a

(</+>) :: (RoundedDivide t) => t -> t -> t
(</+>) = divAssumePositiveOut

(/+) :: (RoundedDivide t) => t -> t -> t
(/+) = (</+>)
-}

recipIn :: (RoundedDivide t) => t -> t
recipIn a = recipInEff (divDefaultEffort a) a

recipOut :: (RoundedDivide t) => t -> t
recipOut a = recipOutEff (divDefaultEffort a) a

propInOutDivRefIsotone ::
    (RefOrd.PartialComparison t, RoundedDivide t, 
     Show t, HasLegalValues t,
     RefOrd.ArbitraryOrderedTuple t) 
    =>
    t ->
    (RefOrd.TwoLEPairs t) -> 
    (DivEffortIndicator t) -> 
    (RefOrd.PartialCompareEffortIndicator t) ->
    Bool
propInOutDivRefIsotone _ =
    roundedRefinementIsotone2 "division" divInEff divOutEff

propInOutDivElim ::
    (RefOrd.PartialComparison t, RoundedDivide t, HasOne t, HasZero t,
     Show t, HasLegalValues t) 
    =>
    t ->
    (RefOrd.UniformlyOrderedSingleton t) -> 
    (RefOrd.PartialCompareEffortIndicator t, 
     DivEffortIndicator t) -> 
    Bool
propInOutDivElim sample (RefOrd.UniformlyOrderedSingleton a) efforts2@(effComp, _) =
    roundedReflexiveCollapse 
        (one sample)
        RefOrd.pLeqEff 
        divInEff divOutEff 
        efforts2 
        a

propInOutDivRecipMult ::
    (RefOrd.PartialComparison t,
     RoundedMultiply t, RoundedDivide t, HasOne t, HasZero t,
     Show t, HasLegalValues t) 
    =>
    t ->
    (RefOrd.UniformlyOrderedPair t) -> 
    (RefOrd.PartialCompareEffortIndicator t, 
     (MultEffortIndicator t, DivEffortIndicator t)) -> 
    Bool
propInOutDivRecipMult _ (RefOrd.UniformlyOrderedPair (e1, e2)) initEffort@(effComp,_) =
    equalRoundingUpDnBin2Var2 "a/b=a*(1/b)"
        expr1 expr2 RefOrd.pLeqEff
        multInEff divInEff
        multOutEff divOutEff
        initEffort e1 e2
    where
    expr1 op1Eff op2Eff (effort1, effort2) e1 e2 = 
        e1 * ((one e1) / e2)
        where
        (*) = op1Eff effort1
        (/) = op2Eff effort2
    expr2 op1Eff op2Eff (effort1, effort2) e1 e2 = 
        e1 / e2
        where
        (/) = op2Eff effort2

testsInOutDiv (name, sample) area =
    testGroup (name ++ " </> >/<") $
        [
--            testProperty "a/a=1" (propInOutDivElim sample)
--            ,
            testProperty "a/b=a*(1/b)" (area, propInOutDivRecipMult sample)
        ,
            testProperty "refinement monotone" (area, propInOutDivRefIsotone sample)
        ]

class 
    (RoundedAddEffort t, 
     RoundedMultiplyEffort t, 
     RoundedPowerToNonnegIntEffort t,
     EffortIndicator (RingOpsEffortIndicator t)) 
    => 
    RoundedRingEffort t
    where
    type RingOpsEffortIndicator t
    ringOpsDefaultEffort :: t -> RingOpsEffortIndicator t
    ringEffortAdd :: t -> (RingOpsEffortIndicator t) -> (AddEffortIndicator t)
    ringEffortMult :: t ->  (RingOpsEffortIndicator t) -> (MultEffortIndicator t)
    ringEffortPow :: t -> (RingOpsEffortIndicator t) -> (PowerToNonnegIntEffortIndicator t)

class 
    (RoundedAdd t, 
     RoundedSubtr t, 
     RoundedMultiply t, 
     RoundedPowerToNonnegInt t,
     RoundedRingEffort t) 
    => 
    RoundedRing t
    
class 
    (RoundedRingEffort t, RoundedDivideEffort t,
     EffortIndicator (FieldOpsEffortIndicator t)) 
    => 
    RoundedFieldEffort t 
    where
    type FieldOpsEffortIndicator t
    fieldOpsDefaultEffort :: t -> FieldOpsEffortIndicator t
    fldEffortAdd :: t -> (FieldOpsEffortIndicator t) -> (AddEffortIndicator t)
    fldEffortMult :: t ->  (FieldOpsEffortIndicator t) -> (MultEffortIndicator t)
    fldEffortPow :: t -> (FieldOpsEffortIndicator t) -> (PowerToNonnegIntEffortIndicator t)
    fldEffortDiv :: t -> (FieldOpsEffortIndicator t) -> (DivEffortIndicator t)

class (RoundedRing t, RoundedDivide t, RoundedFieldEffort t) => RoundedField t
