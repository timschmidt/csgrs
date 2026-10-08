{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-|
    Module      :  Numeric.AERN.RealArithmetic.NumericOrderRounding.FieldOps
    Description :  rounded basic arithmetic operations  
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Rounded basic arithmetical operations.
    
    This module is hidden and reexported via its parent NumericOrderRounding. 
-}
module Numeric.AERN.RealArithmetic.NumericOrderRounding.FieldOps 
where

import Prelude hiding (EQ, LT, GT)
import Numeric.AERN.Basics.PartialOrdering

import Numeric.AERN.RealArithmetic.Auxiliary
import Numeric.AERN.RealArithmetic.ExactOps
import Numeric.AERN.RealArithmetic.NumericOrderRounding.Conversion

import Numeric.AERN.Basics.Effort
import Numeric.AERN.Basics.Exception (HasLegalValues)
import Numeric.AERN.RealArithmetic.Laws 
import Numeric.AERN.RealArithmetic.Measures
import qualified Numeric.AERN.NumericOrder as NumOrd

import Test.QuickCheck
import Test.Framework (testGroup, Test)
import Test.Framework.Providers.QuickCheck2 (testProperty)

import Data.Maybe

infixl 6 +., +^, -., -^
infixl 7 *., *^
--infixl 8 ^., ^^
infixl 7 /., /^

class
    (EffortIndicator (AddEffortIndicator t))
    => 
    RoundedAddEffort t 
    where
    type AddEffortIndicator t
    addDefaultEffort :: t -> AddEffortIndicator t

class (RoundedAddEffort t) => RoundedAdd t where
    addUpEff :: AddEffortIndicator t -> t -> t -> t
    addDnEff :: AddEffortIndicator t -> t -> t -> t

addUp :: (RoundedAdd t) => t -> t -> t
addUp a = addUpEff (addDefaultEffort a) a

(+^) :: (RoundedAdd t) => t -> t -> t
(+^) = addUp

addDn :: (RoundedAdd t) => t -> t -> t
addDn a = addDnEff (addDefaultEffort a) a

(+.) :: (RoundedAdd t) => t -> t -> t
(+.) = addDn

propUpDnAddZero ::
    (NumOrd.PartialComparison t, RoundedAdd t, HasZero t,
     Show t, HasLegalValues t) 
    =>
    t ->
    (NumOrd.PartialCompareEffortIndicator t, 
     AddEffortIndicator t) -> 
    (NumOrd.UniformlyOrderedSingleton t) -> 
    Bool
propUpDnAddZero sample effort (NumOrd.UniformlyOrderedSingleton e) =
    roundedUnit (zero sample) NumOrd.pLeqEff addUpEff addDnEff effort e

propUpDnAddCommutative ::
    (NumOrd.PartialComparison t, RoundedAdd t, HasZero t,
     Show t, HasLegalValues t) 
    =>
    t ->
    (NumOrd.PartialCompareEffortIndicator t, 
     AddEffortIndicator t) -> 
    (NumOrd.UniformlyOrderedPair t) -> 
    Bool
propUpDnAddCommutative _ effort (NumOrd.UniformlyOrderedPair (e1,e2)) =
    roundedCommutative NumOrd.pLeqEff addUpEff addDnEff effort e1 e2
       
propUpDnAddAssociative ::
    (NumOrd.PartialComparison t, RoundedAdd t, HasZero t,
     Show t, HasLegalValues t) 
    =>
    t ->
    (NumOrd.PartialCompareEffortIndicator t, 
     AddEffortIndicator t) -> 
    (NumOrd.UniformlyOrderedTriple t) -> 
    Bool
propUpDnAddAssociative _ effort (NumOrd.UniformlyOrderedTriple (e1,e2,e3)) =
    roundedAssociative NumOrd.pLeqEff addUpEff addDnEff effort e1 e2 e3

testsUpDnAdd (name, sample) =
    testGroup (name ++ " +. +^") $
        [
            testProperty "0 absorbs" (propUpDnAddZero sample)
        ,
            testProperty "commutative" (propUpDnAddCommutative sample)
        ,
            testProperty "associative" (propUpDnAddAssociative sample)
        ]
        
class (RoundedAdd t, Neg t) => RoundedSubtr t where
    subtrUpEff :: (AddEffortIndicator t) -> t -> t -> t
    subtrDnEff :: (AddEffortIndicator t) -> t -> t -> t
    subtrUpEff effort a b = addUpEff effort a (neg b)
    subtrDnEff effort a b = addDnEff effort a (neg b)

subtrUp :: (RoundedSubtr t) => t -> t -> t
subtrUp a = subtrUpEff (addDefaultEffort a) a

(-^) :: (RoundedSubtr t) => t -> t -> t
(-^) = subtrUp

subtrDn :: (RoundedSubtr t) => t -> t -> t
subtrDn a = subtrDnEff (addDefaultEffort a) a

(-.) :: (RoundedSubtr t) => t -> t -> t
(-.) = addDn


propUpDnSubtrElim ::
    (NumOrd.PartialComparison t, RoundedSubtr t, HasZero t,
     Show t, HasLegalValues t) 
    =>
    t ->
    (NumOrd.PartialCompareEffortIndicator t, 
     AddEffortIndicator t) -> 
    (NumOrd.UniformlyOrderedSingleton t) -> 
    Bool
propUpDnSubtrElim sample effort (NumOrd.UniformlyOrderedSingleton e) =
    roundedReflexiveCollapse (zero sample) NumOrd.pLeqEff subtrUpEff subtrDnEff effort e

propUpDnSubtrNegAdd ::
    (NumOrd.PartialComparison t, RoundedSubtr t, Neg t,
     Show t, HasLegalValues t) 
    =>
    t ->
    (NumOrd.PartialCompareEffortIndicator t, 
     AddEffortIndicator t) -> 
    (NumOrd.UniformlyOrderedPair t) -> 
    Bool
propUpDnSubtrNegAdd _ initEffort (NumOrd.UniformlyOrderedPair (e1, e2)) =
    equalRoundingUpDn "a+b=a-(-b)"
        expr1Up expr1Dn expr2Up expr2Dn 
        NumOrd.pLeqEff initEffort
    where
    expr1Up eff =
        let (-^) = subtrUpEff eff in e1 -^ (neg e2)
    expr1Dn eff =
        let (-.) = subtrDnEff eff in e1 -. (neg e2)
    expr2Up eff =
        let (+^) = addUpEff eff in e1 +^ e2
    expr2Dn eff =
        let (+.) = addDnEff eff in e1 +. e2


testsUpDnSubtr (name, sample) =
    testGroup (name ++ " -. -^") $
        [
            testProperty "a-a=0" (propUpDnSubtrElim sample)
            ,
            testProperty "a+b=a-(-b)" (propUpDnSubtrNegAdd sample)
        ]

class
    (EffortIndicator (AbsEffortIndicator t))
    => 
    RoundedAbsEffort t 
    where
    type AbsEffortIndicator t
    absDefaultEffort :: t -> AbsEffortIndicator t

class (RoundedAbsEffort t) => RoundedAbs t where
    absUpEff :: (AbsEffortIndicator t) -> t -> t
    absDnEff :: (AbsEffortIndicator t) -> t -> t

absUp :: (RoundedAbs t) => t -> t
absUp a = absUpEff (absDefaultEffort a) a

absDn :: (RoundedAbs t) => t -> t
absDn a = absDnEff (absDefaultEffort a) a


absUpUsingCompMax ::
    (HasZero t, Neg t, 
     NumOrd.PartialComparison t, NumOrd.RoundedLattice t) =>
    (NumOrd.PartialCompareEffortIndicator t,
     NumOrd.MinmaxEffortIndicator t) ->
    t -> t 
absUpUsingCompMax (effortComp, effortMinmax) a =
    case NumOrd.pCompareEff effortComp (zero a) a of
        Just EQ -> a
        Just LT -> a
        Just GT -> neg a
        _ -> (zero a) `max` (a `max` (neg a))
    where
    max = NumOrd.maxUpEff effortMinmax

absDnUsingCompMax ::
    (HasZero t, Neg t, 
     NumOrd.PartialComparison t, NumOrd.RoundedLattice t) =>
    (NumOrd.PartialCompareEffortIndicator t,
     NumOrd.MinmaxEffortIndicator t) ->
    t -> t 
absDnUsingCompMax (effortComp, effortMinmax) a =
    case NumOrd.pCompareEff effortComp (zero a) a of
        Just EQ -> a
        Just LT -> a
        Just GT -> neg a
        _ -> (zero a) `max` (a `max` (neg a))
    where
    max = NumOrd.maxDnEff effortMinmax

propUpDnAbsNegSymmetric ::
    (NumOrd.PartialComparison t, RoundedAbs t, HasZero t,
     Show t, Neg t, HasLegalValues t) 
    =>
    t ->
    (NumOrd.PartialCompareEffortIndicator t, 
     AbsEffortIndicator t) -> 
    (NumOrd.UniformlyOrderedSingleton t) -> 
    Bool
propUpDnAbsNegSymmetric _ effort (NumOrd.UniformlyOrderedSingleton e) =
    roundedNegSymmetric NumOrd.pLeqEff absUpEff absDnEff effort e

propUpDnAbsIdempotent ::
    (NumOrd.PartialComparison t, RoundedAbs t, HasZero t,
     Show t, HasLegalValues t) 
    =>
    t ->
    (NumOrd.PartialCompareEffortIndicator t, 
     AbsEffortIndicator t) -> 
    (NumOrd.UniformlyOrderedSingleton t) -> 
    Bool
propUpDnAbsIdempotent _ effort (NumOrd.UniformlyOrderedSingleton e) =
    roundedIdempotent NumOrd.pLeqEff absUpEff absDnEff effort e

testsUpDnAbs (name, sample) =
    testGroup (name ++ " up/dn rounded abs") $
        [
            testProperty "neg -> no change" (propUpDnAbsNegSymmetric sample)
        ,
            testProperty "idempotent" (propUpDnAbsIdempotent sample)
        ]


class
    (EffortIndicator (MultEffortIndicator t))
    => 
    RoundedMultiplyEffort t 
    where
    type MultEffortIndicator t
    multDefaultEffort :: t -> MultEffortIndicator t

class (RoundedMultiplyEffort t) => RoundedMultiply t where
    multUpEff :: MultEffortIndicator t -> t -> t -> t
    multDnEff :: MultEffortIndicator t -> t -> t -> t

multUp :: (RoundedMultiply t) => t -> t -> t
multUp a = multUpEff (multDefaultEffort a) a

(*^) :: (RoundedMultiply t) => t -> t -> t
(*^) = multUp

multDn :: (RoundedMultiply t) => t -> t -> t
multDn a = multDnEff (multDefaultEffort a) a

(*.) :: (RoundedMultiply t) => t -> t -> t
(*.) = multDn


propUpDnMultOne ::
    (NumOrd.PartialComparison t, RoundedMultiply t, HasOne t,
     Show t, HasLegalValues t) 
    =>
    t ->
    (NumOrd.PartialCompareEffortIndicator t, 
     MultEffortIndicator t) -> 
    (NumOrd.UniformlyOrderedSingleton t) -> 
    Bool
propUpDnMultOne sample effort (NumOrd.UniformlyOrderedSingleton e) =
    roundedUnit (one sample) NumOrd.pLeqEff multUpEff multDnEff effort e

propUpDnMultCommutative ::
    (NumOrd.PartialComparison t, RoundedMultiply t, HasZero t,
     Show t, HasLegalValues t) 
    =>
    t ->
    (NumOrd.PartialCompareEffortIndicator t, 
     MultEffortIndicator t) -> 
    (NumOrd.UniformlyOrderedPair t) -> 
    Bool
propUpDnMultCommutative _ effort (NumOrd.UniformlyOrderedPair (e1,e2)) =
    roundedCommutative NumOrd.pLeqEff multUpEff multDnEff effort e1 e2
       
propUpDnMultAssociative ::
    (NumOrd.PartialComparison t, NumOrd.RoundedLattice t, 
     Show t, HasLegalValues t,
     RoundedMultiply t, HasZero t) 
    =>
    t ->
    (NumOrd.PartialCompareEffortIndicator t, 
     (MultEffortIndicator t, NumOrd.MinmaxEffortIndicator t)) -> 
    (NumOrd.UniformlyOrderedTriple t) -> 
    Bool
propUpDnMultAssociative _ initEffort (NumOrd.UniformlyOrderedTriple (e1, e2, e3)) =
    equalRoundingUpDn "associativity"
        expr1Up expr1Dn expr2Up expr2Dn 
        NumOrd.pLeqEff initEffort
    where
    expr1Up (effMult, effMinmax) =
        let (*^) = multUpEff effMult; (*.) = multDnEff effMult in
        let r1 = e1 *^ (e2 *^ e3) in
        let r2 = e1 *^ (e2 *. e3) in
        NumOrd.maxUpEff effMinmax r1 r2
    expr1Dn (effMult, effMinmax) =
        let (*^) = multUpEff effMult; (*.) = multDnEff effMult in
        let r1 = e1 *. (e2 *^ e3) in
        let r2 = e1 *. (e2 *. e3) in
        NumOrd.minDnEff effMinmax r1 r2
    expr2Up (effMult, effMinmax) =
        let (*^) = multUpEff effMult; (*.) = multDnEff effMult in
        let r1 = (e1 *^ e2) *^ e3 in
        let r2 = (e1 *. e2) *^ e3 in
        NumOrd.maxUpEff effMinmax r1 r2
    expr2Dn (effMult, effMinmax) =
        let (*^) = multUpEff effMult; (*.) = multDnEff effMult in
        let r1 = (e1 *^ e2) *. e3 in
        let r2 = (e1 *. e2) *. e3 in
        NumOrd.minDnEff effMinmax r1 r2

propUpDnMultDistributesOverAdd ::
    (NumOrd.PartialComparison t, NumOrd.RoundedLattice t,
     Show t, HasLegalValues t,
     RoundedMultiply t,  RoundedAdd t) 
    =>
    t ->
    (NumOrd.PartialCompareEffortIndicator t, 
     (MultEffortIndicator t, AddEffortIndicator t, NumOrd.MinmaxEffortIndicator t)) -> 
    (NumOrd.UniformlyOrderedTriple t) -> 
    Bool
propUpDnMultDistributesOverAdd _ initEffort (NumOrd.UniformlyOrderedTriple (e1, e2, e3)) =
    equalRoundingUpDn "distributivity"
        expr1Up expr1Dn expr2Up expr2Dn 
        NumOrd.pLeqEff initEffort
    where
    expr1Up (effMult, effAdd, effMinmax) =
        let (*^) = multUpEff effMult in
        let (+^) = addUpEff effAdd; (+.) = addDnEff effAdd in
        let r1 = e1 *^ (e2 +^ e3) in
        let r2 = e1 *^ (e2 +. e3) in
        NumOrd.maxUpEff effMinmax r1 r2
    expr1Dn (effMult, effAdd, effMinmax) =
        let (*.) = multDnEff effMult in
        let (+^) = addUpEff effAdd; (+.) = addDnEff effAdd in
        let r1 = e1 *. (e2 +^ e3) in
        let r2 = e1 *. (e2 +. e3) in
        NumOrd.minDnEff effMinmax r1 r2
    expr2Up (effMult, effAdd, _) =
        let (*^) = multUpEff effMult in
        let (+^) = addUpEff effAdd in
        (e1 *^ e2) +^ (e1 *^ e3)
    expr2Dn (effMult, effAdd, _) =
        let (*.) = multDnEff effMult in
        let (+.) = addDnEff effAdd in
        (e1 *. e2) +. (e1 *. e3)
       
    
testsUpDnMult (name, sample) =
    testGroup (name ++ " *. *^") $
        [
            testProperty "1 absorbs" (propUpDnMultOne sample)
        ,
            testProperty "commutative" (propUpDnMultCommutative sample)
        ,
            testProperty "associative" (propUpDnMultAssociative sample)
        ,
            testProperty "distributes over +" (propUpDnMultDistributesOverAdd sample)
        ]

-- simpler versions of integer power assuming the base is non-negative:
class
    (EffortIndicator (PowerNonnegToNonnegIntEffortIndicator t))
    => 
    RoundedPowerNonnegToNonnegIntEffort t 
    where
    type PowerNonnegToNonnegIntEffortIndicator t
    powerNonnegToNonnegIntDefaultEffort :: 
        t -> PowerNonnegToNonnegIntEffortIndicator t 

class (RoundedPowerNonnegToNonnegIntEffort t) =>
        RoundedPowerNonnegToNonnegInt t where
    powerNonnegToNonnegIntUpEff :: 
        (PowerNonnegToNonnegIntEffortIndicator t) -> 
        t {-^ @x@ (assumed >=0) -} -> 
        Int {-^ @n@ (assumed >=0)-} -> 
        t {-^ @x^n@ rounded up -}
    powerNonnegToNonnegIntDnEff ::
        (PowerNonnegToNonnegIntEffortIndicator t) -> 
        t {-^ @x@ (assumed >=0) -} -> 
        Int {-^ @n@ (assumed >=0)-} -> 
        t {-^ @x^n@ rounded down -}


-- functions providing an implementation derived from rounded multiplication: 
        
type PowerNonnegToNonnegIntEffortIndicatorFromMult t =
    MultEffortIndicator t
    
powerNonnegToNonnegIntDefaultEffortFromMult a =
    multDefaultEffort a

powerNonnegToNonnegIntUpEffFromMult ::
    (RoundedMultiply t, HasOne t) => 
    PowerNonnegToNonnegIntEffortIndicatorFromMult t -> 
    t -> Int -> t
powerNonnegToNonnegIntUpEffFromMult effMult e n =
    powerFromMult (one e) (multUpEff effMult) e n

powerNonnegToNonnegIntDnEffFromMult ::
    (RoundedMultiply t, HasOne t) => 
    PowerNonnegToNonnegIntEffortIndicatorFromMult t -> 
    t -> Int -> t
powerNonnegToNonnegIntDnEffFromMult effMult e n =
    powerFromMult (one e) (multDnEff effMult) e n

-- general integer power, ie not assuming the base is non-negative:
class
    (EffortIndicator (PowerToNonnegIntEffortIndicator t))
    => 
    RoundedPowerToNonnegIntEffort t 
    where
    type PowerToNonnegIntEffortIndicator t
    powerToNonnegIntDefaultEffort :: 
        t -> PowerToNonnegIntEffortIndicator t 

class 
    (RoundedPowerToNonnegIntEffort t) 
    => 
    RoundedPowerToNonnegInt t 
    where
    powerToNonnegIntUpEff ::
        (PowerToNonnegIntEffortIndicator t) -> 
        t {-^ @x@ -} -> 
        Int {-^ @n@ (assumed >=0)-} -> 
        t {-^ @x^n@ rounded up -}
    powerToNonnegIntDnEff ::
        (PowerToNonnegIntEffortIndicator t) -> 
        t {-^ @x@ -} -> 
        Int {-^ @n@ (assumed >=0)-} -> 
        t {-^ @x^n@ rounded down -}

powerToNonnegIntUp :: (RoundedPowerToNonnegInt t) => t -> Int -> t
powerToNonnegIntUp a = powerToNonnegIntUpEff (powerToNonnegIntDefaultEffort a) a

powerToNonnegIntDn :: (RoundedPowerToNonnegInt t) => t -> Int -> t
powerToNonnegIntDn a = powerToNonnegIntDnEff (powerToNonnegIntDefaultEffort a) a


-- functions providing an implementation derived from rounded multiplication: 

type PowerToNonnegIntEffortIndicatorFromMult t =
    (MultEffortIndicator t, 
     NumOrd.PartialCompareEffortIndicator t, 
     NumOrd.MinmaxEffortIndicator t)
     
powerToNonnegIntDefaultEffortFromMult a =
    (multDefaultEffort a,
     NumOrd.pCompareDefaultEffort a,
     NumOrd.minmaxDefaultEffort a)

powerToNonnegIntUpEffFromMult :: 
    (RoundedMultiply t, HasOne t, 
     NumOrd.PartialComparison t, HasZero t, 
     Neg t, NumOrd.RoundedLattice t) => 
    PowerToNonnegIntEffortIndicatorFromMult t ->
    t -> Int -> t
powerToNonnegIntUpEffFromMult (effMult, effComp, effMinmax) e n =
    powerToNonnegIntDir
        (multUpEff effMult) (multDnEff effMult)
        (NumOrd.maxUpEff effMinmax)
        effComp e n

powerToNonnegIntDnEffFromMult :: 
    (RoundedMultiply t, HasOne t, 
     NumOrd.PartialComparison t, HasZero t, 
     Neg t, NumOrd.RoundedLattice t) => 
    PowerToNonnegIntEffortIndicatorFromMult t ->
    t -> Int -> t
powerToNonnegIntDnEffFromMult (effMult, effComp, effMinmax) e n =
    powerToNonnegIntDir 
        (multDnEff effMult) (multUpEff effMult) 
        (NumOrd.minDnEff effMinmax)
        effComp e n

powerToNonnegIntDir :: 
    (HasOne t, 
     NumOrd.PartialComparison t, HasZero t, 
     Neg t) => 
    (t -> t -> t) {-^ multiplication rounded in the desired direction -} ->
    (t -> t -> t) {-^ multiplication rounded in the opposite direction -} ->
    (t -> t -> t) {-^ safe combination of alternative results -} ->
    (NumOrd.PartialCompareEffortIndicator t) -> 
    t -> Int -> t
powerToNonnegIntDir mult1 mult2 combine effComp x n
    | n == 0 = (one x)
    | n == 1 = x
    | otherwise =
        case (pNonnegNonposEff effComp x) of
            (Just True, _) -> resNonneg
            (_, Just True) -> resNonpos
            _ -> resNonneg `combine` resNonpos
    where
    resNonneg = powerFromMult (one x) mult1 x n
    resNonpos 
        | even n = 
            powerFromMult (one x) mult1 (neg x) n
        | otherwise = 
            neg $ powerFromMult (one x) mult2 (neg x) n 
            -- switching rounding direction

propUpDnPowerSumExponents ::
    (NumOrd.PartialComparison t, NumOrd.RoundedLattice t,
     RoundedPowerToNonnegInt t, RoundedMultiply t, 
     HasOne t, HasZero t, Neg t,
     Show t, HasLegalValues t) 
    =>
    t ->
    (NumOrd.PartialCompareEffortIndicator t,
     (PowerToNonnegIntEffortIndicator t,
      (NumOrd.PartialCompareEffortIndicator t,
       MultEffortIndicator t,
       NumOrd.MinmaxEffortIndicator t))) -> 
    (NumOrd.UniformlyOrderedSingleton t) -> 
    Int -> Int -> Bool
propUpDnPowerSumExponents _ initEffort (NumOrd.UniformlyOrderedSingleton a) nR mR =
    equalRoundingUpDn "a^(n+m) = a^n * a^m"
        expr1Up expr1Dn expr2Up expr2Dn 
        NumOrd.pLeqEff initEffort
    where
    n = nR `mod` 10
    m = mR `mod` 10
    minusA = neg a
    expr1Up (effPower, (effComp, effMult, effMinmax)) =
        let (^^) = powerToNonnegIntUpEff effPower in
        a ^^ (n + m)
    expr1Dn (effPower, (effComp, effMult, effMinmax)) =
        let (^.) = powerToNonnegIntDnEff effPower in
        a ^. (n + m)
    expr2Up (effPower, (effComp, effMult, effMinmax)) =
        case pNonnegNonposEff effComp a of
            (Just True, _) -> rNonneg
            (_, Just True) -> rNonpos
            _ -> rNonneg `max` rNonpos
        where
        max = NumOrd.maxUpEff effMinmax
        (^^) = powerToNonnegIntUpEff effPower
        (^.) = powerToNonnegIntDnEff effPower
        (*^) = multUpEff effMult
        (*.) = multDnEff effMult
        rNonneg = (a ^^ n) *^ (a ^^ m)
        rNonpos =
            case (even (n + m)) of
                True -> (minusA ^^ n) *^ (minusA ^^ m)
                False -> neg $ (minusA ^. n) *. (minusA ^. m)
    expr2Dn (effPower, (effComp, effMult, effMinmax)) =
        case pNonnegNonposEff effComp a of
            (Just True, _) -> rNonneg
            (_, Just True) -> rNonpos
            _ -> rNonneg `min` rNonpos
        where
        min = NumOrd.minDnEff effMinmax
        (^^) = powerToNonnegIntUpEff effPower
        (^.) = powerToNonnegIntDnEff effPower
        (*^) = multUpEff effMult
        (*.) = multDnEff effMult
        rNonneg = (a ^. n) *. (a ^. m)
        rNonpos =
            case (even (n + m)) of
                True -> (minusA ^. n) *. (minusA ^. m)
                False -> neg $ (minusA ^^ n) *^ (minusA ^^ m)

testsUpDnIntPower (name, sample) =
    testGroup (name ++ " non-negative integer power") $
        [
            testProperty "a^(n+m) = a^n * a^m" (propUpDnPowerSumExponents sample)
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
    divUpEff :: DivEffortIndicator t -> t -> t -> t
    divDnEff :: DivEffortIndicator t -> t -> t -> t
    recipUpEff :: DivEffortIndicator t -> t -> t
    recipDnEff :: DivEffortIndicator t -> t -> t
    recipUpEff eff a = divUpEff eff (one a) a
    recipDnEff eff a = divDnEff eff (one a) a

divUp :: (RoundedDivide t) => t -> t -> t
divUp a = divUpEff (divDefaultEffort a) a

(/^) :: (RoundedDivide t) => t -> t -> t
(/^) = divUp

divDn :: (RoundedDivide t) => t -> t -> t
divDn a = divDnEff (divDefaultEffort a) a

(/.) :: (RoundedDivide t) => t -> t -> t
(/.) = divDn

recipUp :: (RoundedDivide t) => t -> t
recipUp a = recipUpEff (divDefaultEffort a) a

recipDn :: (RoundedDivide t) => t -> t
recipDn a = recipDnEff (divDefaultEffort a) a

propUpDnDivElim ::
    (NumOrd.PartialComparison t, RoundedDivide t, HasOne t, HasZero t,
     Show t, HasLegalValues t) 
    =>
    t ->
    (NumOrd.PartialCompareEffortIndicator t, 
     DivEffortIndicator t) -> 
    (NumOrd.UniformlyOrderedSingleton t) -> 
    Bool
propUpDnDivElim sample efforts2@(effComp, _) (NumOrd.UniformlyOrderedSingleton a) =
    roundedReflexiveCollapse 
        (one sample) 
        NumOrd.pLeqEff 
        divUpEff divDnEff 
        efforts2 
        a
        
propUpDnDivRecipMult ::
    (NumOrd.PartialComparison t, NumOrd.RoundedLattice t,
     Show t, HasLegalValues t,
     RoundedMultiply t, RoundedDivide t, HasOne t, HasZero t) 
    =>
    t ->
    (NumOrd.PartialCompareEffortIndicator t,
     (MultEffortIndicator t, DivEffortIndicator t, NumOrd.MinmaxEffortIndicator t)) -> 
    (NumOrd.UniformlyOrderedPair t) -> 
    Bool
propUpDnDivRecipMult sample initEffort@(effComp,_) (NumOrd.UniformlyOrderedPair (e1, e2)) =
    equalRoundingUpDn "a/b=a*(1/b)"
        expr1Up expr1Dn expr2Up expr2Dn 
        NumOrd.pLeqEff initEffort
    where
    expr1Up (effMult, effDiv, effMinmax) =
        let (*^) = multUpEff effMult in
        let (/^) = divUpEff effDiv; (/.) = divDnEff effDiv in
        let r1 = e1 *^ ((one sample) /^ e2) in
        let r2 = e1 *^ ((one sample) /. e2) in
        NumOrd.maxUpEff effMinmax r1 r2
    expr1Dn (effMult, effDiv, effMinmax) =
        let (*.) = multDnEff effMult in
        let (/^) = divUpEff effDiv; (/.) = divDnEff effDiv in
        let r1 = e1 *. ((one sample) /^ e2) in
        let r2 = e1 *. ((one sample) /. e2) in
        NumOrd.minDnEff effMinmax r1 r2
    expr2Up (effMult, effDiv, _) =
        let (/^) = divUpEff effDiv in
        e1 /^ e2
    expr2Dn (effMult, effDiv, _) =
        let (/.) = divDnEff effDiv in
        e1 /. e2

testsUpDnDiv (name, sample) =
    testGroup (name ++ " /. /^") $
        [
            testProperty "a/a=1" (propUpDnDivElim sample)
            ,
            testProperty "a/b=a*(1/b)" (propUpDnDivRecipMult sample)
        ]

class 
    (RoundedAddEffort t,
     RoundedMultiplyEffort t, 
     RoundedPowerNonnegToNonnegIntEffort t, 
     RoundedPowerToNonnegIntEffort t,
     EffortIndicator (RingOpsEffortIndicator t)) 
    =>
    RoundedRingEffort t
    where
    type RingOpsEffortIndicator t
    ringOpsDefaultEffort :: t -> RingOpsEffortIndicator t
    ringEffortAdd :: t -> (RingOpsEffortIndicator t) -> (AddEffortIndicator t)
    ringEffortMult :: t ->  (RingOpsEffortIndicator t) -> (MultEffortIndicator t)
    ringEffortPow :: t -> (RingOpsEffortIndicator t) -> (PowerNonnegToNonnegIntEffortIndicator t)

class (RoundedAdd t, RoundedSubtr t, 
       RoundedMultiply t, 
       RoundedPowerNonnegToNonnegInt t, 
       RoundedPowerToNonnegInt t,
       RoundedRingEffort t) => 
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
    fldEffortPow :: t -> (FieldOpsEffortIndicator t) -> (PowerNonnegToNonnegIntEffortIndicator t)
    fldEffortDiv :: t -> (FieldOpsEffortIndicator t) -> (DivEffortIndicator t)

class (RoundedRing t, RoundedDivide t, RoundedFieldEffort t) => RoundedField t

    