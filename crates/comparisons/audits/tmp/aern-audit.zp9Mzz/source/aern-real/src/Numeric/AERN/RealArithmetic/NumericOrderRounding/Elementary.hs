{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-|
    Module      :  Numeric.AERN.RealArithmetic.NumericOrderRounding.Elementary
    Description :  support for various common elementary functions
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Support for various common elementary functions.
    
    This module is hidden and reexported via its parent NumericOrderRounding. 
-}

module Numeric.AERN.RealArithmetic.NumericOrderRounding.Elementary where

import Numeric.AERN.RealArithmetic.ExactOps
import Numeric.AERN.RealArithmetic.NumericOrderRounding.FieldOps

import Numeric.AERN.Basics.Effort
import Numeric.AERN.Basics.Exception
import Numeric.AERN.Basics.ShowInternals
import Numeric.AERN.RealArithmetic.Laws
--import Numeric.AERN.RealArithmetic.Measures
import qualified Numeric.AERN.NumericOrder as NumOrd

import Numeric.AERN.Misc.Debug

import Test.QuickCheck
import Test.Framework (testGroup, Test)
import Test.Framework.Providers.QuickCheck2 (testProperty)

class
    (EffortIndicator (ExpEffortIndicator t))
    => 
    RoundedExponentiationEffort t 
    where
    type ExpEffortIndicator t
    expDefaultEffort :: t -> ExpEffortIndicator t

class (RoundedExponentiationEffort t) => RoundedExponentiation t where
    expUpEff :: (ExpEffortIndicator t) -> t -> t
    expDnEff :: (ExpEffortIndicator t) -> t -> t

expUp :: (RoundedExponentiation t) => t -> t
expUp a = expUpEff (expDefaultEffort a) a

expDn :: (RoundedExponentiation t) => t -> t
expDn a = expDnEff (expDefaultEffort a) a


-- | @e^a*e^(-a) = 1@
propExpOfNegRecip ::
    (NumOrd.PartialComparison t, NumOrd.RoundedLattice t,
     RoundedExponentiation t, RoundedMultiply t, Neg t, HasOne t,
     Show t, HasLegalValues t,
     ShowInternals t) 
    =>
    t ->
    (NumOrd.PartialCompareEffortIndicator t, 
     (ExpEffortIndicator t, MultEffortIndicator t)) -> 
    (NumOrd.UniformlyOrderedSingleton t) -> 
    Bool
propExpOfNegRecip sample initEffort (NumOrd.UniformlyOrderedSingleton e1) =
    equalRoundingUpDn "e^a * e^(-a) = 1"
        expr1Up expr1Dn expr2Up expr2Dn 
        NumOrd.pLeqEff initEffort
    where
    expr1Up (effExp, effMult) = one sample
    expr1Dn (effExp, effMult) = one sample
    expr2Up (effExp, effMult) =
        let (*^) = multUpEff effMult in
        let expE1 = expUpEff effExp e1 in
        let expNegE1 = expUpEff effExp (neg e1) in
        let prod = expE1 *^ expNegE1 in
--        unsafePrintReturn
--        (
--          "propExpOfNegRecip: expr2Up: e1 = " ++ show e1 
--          ++ "; expE1 = " ++ show expE1 
--          ++ "; expNegE1 = " ++ show expNegE1 
--          ++ "; prod = " ++ showUsingShowInternals prod
--          ++ "; result = " 
--        )$
        prod
    expr2Dn (effExp, effMult) =
        let (*.) = multDnEff effMult in
        let expE1 = expDnEff effExp e1 in
        let negE1 = (neg e1) in
        let expNegE1 = expDnEff effExp negE1 in
        let prod = expE1 *. expNegE1 in
--        unsafePrintReturn
--        (
--          "propExpOfNegRecip: expr2Dn: e1 = " ++ show e1 
--          ++ "; expE1 = " ++ show expE1 
--          ++ "; negE1 = " ++ show negE1 
--          ++ "; expNegE1 = " ++ show expNegE1 
--          ++ "; prod = " ++ showUsingShowInternals prod
--          ++ "; result = " 
--        )$
        prod

-- | @e^(b+c) = e^b * e^c@
propExpOfAddToMult ::
    (NumOrd.PartialComparison t,
     RoundedExponentiation t, RoundedMultiply t,  RoundedAdd t,
     Show t, HasLegalValues t) 
    =>
    t ->
    (NumOrd.PartialCompareEffortIndicator t, 
     (ExpEffortIndicator t, MultEffortIndicator t, AddEffortIndicator t)) -> 
    (NumOrd.UniformlyOrderedPair t) -> 
    Bool
propExpOfAddToMult _ initEffort (NumOrd.UniformlyOrderedPair (e1, e2)) =
    equalRoundingUpDn "e^(a + b) = e^a * e^b"
        expr1Up expr1Dn expr2Up expr2Dn 
        NumOrd.pLeqEff initEffort
    where
    expr1Up (effExp, effMult, effAdd) =
        let (+^) = addUpEff effAdd in
        (expUpEff effExp (e1 +^ e2))
    expr1Dn (effExp, effMult, effAdd) =
        let (+.) = addDnEff effAdd in
        (expDnEff effExp (e1 +. e2))
    expr2Up (effExp, effMult, effAdd) =
        let (*^) = multUpEff effMult in
        (expUpEff effExp e1) *^ (expUpEff effExp e2)
    expr2Dn (effExp, effMult, effAdd) =
        let (*.) = multDnEff effMult in
        (expDnEff effExp e1) *. (expDnEff effExp e2)
    
testsUpDnExp (name, sample) =
    testGroup (name ++ " exp up/dn") $
        [
            testProperty "e^a * e^(-a) = 1" (propExpOfNegRecip sample)
        ,
            testProperty "e^(a + b) = e^a * e^b" (propExpOfAddToMult sample)
        ]
    
class
    (EffortIndicator (SqrtEffortIndicator t))
    => 
    RoundedSquareRootEffort t 
    where
    type SqrtEffortIndicator t
    sqrtDefaultEffort :: t -> SqrtEffortIndicator t

class (RoundedSquareRootEffort t) => RoundedSquareRoot t where
    sqrtUpEff :: (SqrtEffortIndicator t) -> t -> t
    sqrtDnEff :: (SqrtEffortIndicator t) -> t -> t

sqrtUp :: (RoundedSquareRoot t) => t -> t
sqrtUp a = sqrtUpEff (sqrtDefaultEffort a) a

sqrtDn :: (RoundedSquareRoot t) => t -> t
sqrtDn a = sqrtDnEff (sqrtDefaultEffort a) a

propSqrtSquare ::
    (NumOrd.PartialComparison t, 
     RoundedSquareRoot t, RoundedMultiply t, HasZero t,
     Show t, HasLegalValues t,
     ShowInternals t) 
    =>
    t ->
    (NumOrd.PartialCompareEffortIndicator t, 
     (SqrtEffortIndicator t, MultEffortIndicator t, NumOrd.PartialCompareEffortIndicator t)) -> 
    (NumOrd.UniformlyOrderedSingleton t) -> 
    Bool
propSqrtSquare sample initEffort (NumOrd.UniformlyOrderedSingleton e1) =
    case evalCatchDomViolationExceptions "checking sqrt(x)^2 = x"
            (equalRoundingUpDn "sqrt(x)^2 = x"
                expr1Up expr1Dn expr2Up expr2Dn 
                NumOrd.pLeqEff initEffort) of
        Left e -> True -- was unlucky with the params
        Right r -> r
    where
    expr1Up (effSqrt, effMult, effCompare) =
        sqrtE1 *^ sqrtE1
        where
        (*^) = multUpEff effMult
        sqrtE1 = sqrtUpEff effSqrt e1
    expr1Dn (effSqrt, effMult, effCompare)
        | sqrtE1DefinitelyPositive = sqrtE1 *. sqrtE1
        | otherwise = zero sample
        where
        sqrtE1DefinitelyPositive =
            case zero sample <=? sqrtE1  of (Just r) -> r; _ -> False
        (<=?) = NumOrd.pLeqEff effCompare
        (*.) = multDnEff effMult
        sqrtE1 = sqrtDnEff effSqrt e1
    expr2Up _ = e1
    expr2Dn _ = e1

testsUpDnSqrt (name, sample) =
    testGroup (name ++ " sqrt up/dn") $
        [
            testProperty "sqrt(e)^2 = e" (propSqrtSquare sample)
        ]
    