{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-|
    Module      :  Numeric.AERN.RealArithmetic.RefinementOrderRounding.Elementary
    Description :  support for various common elementary functions
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Support for various common elementary functions.
    
    This module is hidden and reexported via its parent RefinementOrderRounding. 
-}

module Numeric.AERN.RealArithmetic.RefinementOrderRounding.Elementary where

import Numeric.AERN.RealArithmetic.ExactOps
import Numeric.AERN.RealArithmetic.RefinementOrderRounding.FieldOps
import Numeric.AERN.RealArithmetic.RefinementOrderRounding.MixedFieldOps

import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding.Conversion as UpDnConversion

import Numeric.AERN.Basics.Consistency
import Numeric.AERN.Basics.Arbitrary
import Numeric.AERN.Basics.Effort
import Numeric.AERN.Basics.Exception
import Numeric.AERN.Basics.ShowInternals
import Numeric.AERN.Basics.Bench
import Numeric.AERN.Basics.Consistency
import Numeric.AERN.RealArithmetic.Laws
import Numeric.AERN.RealArithmetic.Bench
import Numeric.AERN.RealArithmetic.Measures
import qualified Numeric.AERN.NumericOrder as NumOrd
import qualified Numeric.AERN.RefinementOrder as RefOrd

import Numeric.AERN.Misc.Debug

import Test.QuickCheck
import Test.Framework (testGroup, Test)
import Test.Framework.Providers.QuickCheck2 (testProperty)

import Criterion

class
    (EffortIndicator (ExpEffortIndicator t))
    => 
    RoundedExponentiationEffort t 
    where
    type ExpEffortIndicator t
    expDefaultEffort :: t -> ExpEffortIndicator t

class (RoundedExponentiationEffort t) => RoundedExponentiation t where
    expInEff :: (ExpEffortIndicator t) -> t -> t
    expOutEff :: (ExpEffortIndicator t) -> t -> t

-- | Inward rounded exponential with default effort
expIn :: (RoundedExponentiation t) => t -> t
expIn d = expInEff (expDefaultEffort d) d

-- | Outward rounded exponential with default effort
expOut :: (RoundedExponentiation t) => t -> t
expOut d = expOutEff (expDefaultEffort d) d


-- | @e^a*e^(-a) = 1@
propExpOfNegRecip ::
    (RefOrd.PartialComparison t,
     RoundedExponentiation t, RoundedMultiply t, Neg t, HasOne t,
     Show t, HasAntiConsistency t, HasLegalValues t) 
    =>
    t ->
    (ConsistencyEffortIndicator t) -> 
    (RefOrd.PartialCompareEffortIndicator t, 
     (ExpEffortIndicator t, MultEffortIndicator t)) -> 
    (RefOrd.UniformlyOrderedSingleton t) -> 
    Bool
propExpOfNegRecip sample effortConsistency initEffort 
        (RefOrd.UniformlyOrderedSingleton e1) =
    thinEqualConsLeqRoundingUpDnImprovement "e^a * e^(-a) ⊑/⊒ 1" [e1]
        expr1In expr1Out expr2In expr2Out 
        RefOrd.pLeqEff
        effortConsistency 
        initEffort
    where
    expr1In (effExp, effMult) =
--        unsafePrintReturn (
--                "propExpOfNegRecip: expr2In: " 
--                ++ "\n e1 = " ++ (show e1)
--                ++ "\n expInEff effExp e1 = " ++ (show $ expInEff effExp e1)
--                ++ "\n expInEff effExp (neg e1) = " ++ (show $ expInEff effExp (neg e1))
--                ++ "\n product of the above = "
--        ) $
        let (>*<) = multInEff effMult in
        (expInEff effExp e1) >*< (expInEff effExp (neg e1))
    expr1Out (effExp, effMult) =
        let (<*>) = multOutEff effMult in
        (expOutEff effExp e1) <*> (expOutEff effExp (neg e1))
    expr2In (effExp, effMult) = one sample
    expr2Out (effExp, effMult) = one sample

-- | @e^(b+c) = e^b * e^c@
propExpOfAddToMult ::
    (RefOrd.PartialComparison t,
     RoundedExponentiation t, RoundedMultiply t,  RoundedAdd t,
     Show t, HasLegalValues t) 
    =>
    t ->
    (RefOrd.PartialCompareEffortIndicator t, 
     (ExpEffortIndicator t, MultEffortIndicator t, AddEffortIndicator t)) -> 
    (RefOrd.UniformlyOrderedPair t) -> 
    Bool
propExpOfAddToMult _ initEffort (RefOrd.UniformlyOrderedPair (e1, e2)) =
    equalRoundingUpDn "e^(a + b) = e^a * e^b"
        expr1In expr1Out expr2In expr2Out 
        RefOrd.pLeqEff initEffort
    where
    expr1In (effExp, effMult, effAdd) =
        let (+^) = addInEff effAdd in
        (expInEff effExp (e1 +^ e2))
    expr1Out (effExp, effMult, effAdd) =
        let (+.) = addOutEff effAdd in
        (expOutEff effExp (e1 +. e2))
    expr2In (effExp, effMult, effAdd) =
        let (*^) = multInEff effMult in
        (expInEff effExp e1) *^ (expInEff effExp e2)
    expr2Out (effExp, effMult, effAdd) =
        let (*.) = multOutEff effMult in
        (expOutEff effExp e1) *. (expOutEff effExp e2)
    
testsInOutExp (name, sample) =
    testGroup (name ++ " exp in/out") $
        [
            testProperty "e^a * e^(-a) ⊑/⊒ 1" (propExpOfNegRecip sample)
        ,
            testProperty "e^(a + b) = e^a * e^b" (propExpOfAddToMult sample)
        ]
            
benchInOutExp (name, sample) areas =
    bgroup (name ++ " exp") $
        mkBenchAreasSequences1 (mkCommentImprecision1 expOutEff expInEff) 
            expOutEff areas 10 (expDefaultEffort sample) sample 

benchExpAreasReal =
    [
        ("near 0", areaWithBounds (-1/2) (1/2))
    ,
        ("near -10", areaWithBounds (-10.5) (-9.5))
    ,
        ("near 10", areaWithBounds (9.5) (10.5))
    ,
        ("near 20", areaWithBounds (19.5) (20.5))
    ]
    where
    areaWithBounds l r =
         (NumOrd.AreaLinear (Just l) True (Just r) True $ areaWholeOnlyWhole [], 
          AreaMaybeAllowOnlyWithConsistencyStatus $ Just Consistent)

class
    (EffortIndicator (SqrtEffortIndicator t))
    => 
    RoundedSquareRootEffort t 
    where
    type SqrtEffortIndicator t
    sqrtDefaultEffort :: t -> SqrtEffortIndicator t

class (RoundedSquareRootEffort t) => RoundedSquareRoot t where
    sqrtInEff :: (SqrtEffortIndicator t) -> t -> t
    sqrtOutEff :: (SqrtEffortIndicator t) -> t -> t

-- | Inward rounded square root with default effort
sqrtIn :: (RoundedSquareRoot t) => t -> t
sqrtIn d = sqrtInEff (sqrtDefaultEffort d) d

-- | Outward rounded square root with default effort
sqrtOut :: (RoundedSquareRoot t) => t -> t
sqrtOut d = sqrtOutEff (sqrtDefaultEffort d) d

propSqrtSquare ::
    (RefOrd.PartialComparison t,
     RoundedSquareRoot t, RoundedMultiply t, HasZero t,
     UpDnConversion.Convertible t Double,
     RoundedMixedAdd t Double,
     Show t, HasLegalValues t
--     ShowInternals t,
     ) =>
    t ->
    (tInArea -> t) ->
    (UpDnConversion.ConvertEffortIndicator t Double, 
     MixedAddEffortIndicator t Double) ->
    (RefOrd.PartialCompareEffortIndicator t, 
     (SqrtEffortIndicator t, 
      MultEffortIndicator t, 
      RefOrd.PartialCompareEffortIndicator t)) -> 
    tInArea -> Bool
propSqrtSquare _ fromArea (effortToDbl, effortAddDbl) initEffort e1InArea =
    equalRoundingUpDn "sqrt(e)^2 = e"
        expr1In expr1Out expr2In expr2Out 
        RefOrd.pLeqEff initEffort
    where
    e1Pos = fromArea e1InArea
--        case maybeE1LowerBoundD of
--            Just e1LowerBoundD
--                | e1LowerBoundD <= (0 :: Double) -> 
--                    mixedAddOutEff effortAddDbl e1 (0.5 - e1LowerBoundD)
--                | otherwise -> e1
--            _ -> e1
--        where
--        maybeE1LowerBoundD = UpDnConversion.convertDnEff effortToDbl e1  
    expr1In (effSqrt, effMult, effCompare) =
        sqrtE1 >*< sqrtE1
        where
        (>*<) = multInEff effMult
        sqrtE1 = sqrtInEff effSqrt e1Pos
    expr1Out (effSqrt, effMult, effCompare) =
        sqrtE1 <*> sqrtE1
        where
        (<*>) = multOutEff effMult
        sqrtE1 = sqrtOutEff effSqrt e1Pos
    expr2In _ = e1Pos
    expr2Out _ = e1Pos

testsInOutSqrt (name, sample) fromArea =
    testGroup (name ++ " sqrt in/out") $
        [
            testProperty "sqrt(e)^2 = e" (propSqrtSquare sample fromArea)
        ]



class
    (EffortIndicator (SineCosineEffortIndicator t))
    => 
    RoundedSineCosineEffort t 
    where
    type SineCosineEffortIndicator t
    sincosDefaultEffort :: t -> SineCosineEffortIndicator t

class (RoundedSineCosineEffort t) => RoundedSineCosine t where
    sinInEff :: (SineCosineEffortIndicator t) -> t -> t
    sinOutEff :: (SineCosineEffortIndicator t) -> t -> t
    cosInEff :: (SineCosineEffortIndicator t) -> t -> t
    cosOutEff :: (SineCosineEffortIndicator t) -> t -> t

-- | Inward rounded sine and cosine with default effort
sinIn :: (RoundedSineCosine t) => t -> t
sinIn d = sinInEff (sincosDefaultEffort d) d
cosIn :: (RoundedSineCosine t) => t -> t
cosIn d = cosInEff (sincosDefaultEffort d) d

-- | Outward rounded sine and cosine with default effort
sinOut :: (RoundedSineCosine t) => t -> t
sinOut d = sinOutEff (sincosDefaultEffort d) d
cosOut :: (RoundedSineCosine t) => t -> t
cosOut d = cosOutEff (sincosDefaultEffort d) d

--propSineCosineSquare ::
--    (RefOrd.PartialComparison t,
--     RoundedSineCosine t, RoundedMultiply t, HasZero t,
--     UpDnConversion.Convertible t Double,
--     RoundedMixedAdd t Double,
--     Show t, HasLegalValues t
----     ShowInternals t,
--     ) =>
--    t ->
--    (tInArea -> t) ->
--    (UpDnConversion.ConvertEffortIndicator t Double, 
--     MixedAddEffortIndicator t Double) ->
--    (RefOrd.PartialCompareEffortIndicator t, 
--     (SineCosineEffortIndicator t, 
--      MultEffortIndicator t, 
--      RefOrd.PartialCompareEffortIndicator t)) -> 
--    tInArea -> Bool
--propSineCosineSquare _ fromArea (effortToDbl, effortAddDbl) initEffort e1InArea =
--    equalRoundingUpDn "sincos(e)^2 = e"
--        expr1In expr1Out expr2In expr2Out 
--        RefOrd.pLeqEff initEffort
--    where
--    e1Pos = fromArea e1InArea
----        case maybeE1LowerBoundD of
----            Just e1LowerBoundD
----                | e1LowerBoundD <= (0 :: Double) -> 
----                    mixedAddOutEff effortAddDbl e1 (0.5 - e1LowerBoundD)
----                | otherwise -> e1
----            _ -> e1
----        where
----        maybeE1LowerBoundD = UpDnConversion.convertDnEff effortToDbl e1  
--    expr1In (effSineCosine, effMult, effCompare) =
--        sincosE1 >*< sincosE1
--        where
--        (>*<) = multInEff effMult
--        sincosE1 = sincosInEff effSineCosine e1Pos
--    expr1Out (effSineCosine, effMult, effCompare) =
--        sincosE1 <*> sincosE1
--        where
--        (<*>) = multOutEff effMult
--        sincosE1 = sincosOutEff effSineCosine e1Pos
--    expr2In _ = e1Pos
--    expr2Out _ = e1Pos
--
--testsInOutSineCosine (name, sample) fromArea =
--    testGroup (name ++ " sincos in/out") $
--        [
--            testProperty "sincos(e)^2 = e" (propSineCosineSquare sample fromArea)
--        ]
    