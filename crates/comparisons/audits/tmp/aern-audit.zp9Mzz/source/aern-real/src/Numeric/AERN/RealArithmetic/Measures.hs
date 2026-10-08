{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE UndecidableInstances #-}
{-|
    Module      :  Numeric.AERN.RealArithmetic.Measures
    Description :  measures of quality for approximations
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Measures of quality for approximations.
-}
module Numeric.AERN.RealArithmetic.Measures where

import {-# Source #-} Numeric.AERN.RealArithmetic.RefinementOrderRounding.FieldOps

import qualified Numeric.AERN.NumericOrder as NumOrd

import qualified Numeric.AERN.RefinementOrder as RefOrd

import Numeric.AERN.Basics.Effort

import Test.QuickCheck
import Test.Framework (testGroup, Test)
import Test.Framework.Providers.QuickCheck2 (testProperty)

import Debug.Trace
_ = trace

{-|
   Ability to measure a distance.  Distance
   should be a numeric type approximating the
   positive real numbers with Partial comparison.
-}
class
    (EffortIndicator (DistanceEffortIndicator t))
    => 
    HasDistance t 
    where
    type Distance t
    type DistanceEffortIndicator t
    distanceDefaultEffort :: t -> (DistanceEffortIndicator t)
    {-| distance measure -}
    distanceBetweenEff :: 
        DistanceEffortIndicator t -> t -> t -> Distance t

distanceBetween :: (HasDistance t) => t -> t -> Distance t
distanceBetween a b = distanceBetweenEff eff a b
    where
    eff = distanceDefaultEffort a

propDistanceTriangular :: 
    (HasDistance t, 
     NumOrd.PartialComparison (Distance t),
     RoundedAdd (Distance t)
    ) =>
    t ->
    (DistanceEffortIndicator t) ->     
    (NumOrd.PartialCompareEffortIndicator (Distance t)) -> 
    (AddEffortIndicator (Distance t)) ->     
    t -> t -> t -> Bool
propDistanceTriangular _ effortDist effortComp effortAdd e1 e2 e3 =
    let (>=?) = NumOrd.pGeqEff effortComp in 
    let (<+>) = addOutEff effortAdd in 
        let
        d12 = distanceBetweenEff effortDist e1 e2
        d23 = distanceBetweenEff effortDist e2 e3
        d13 = distanceBetweenEff effortDist e1 e3
        in
        case (d12 <+> d23) >=? d13 of
            Nothing -> True
            Just b -> b

testsDistance ::
    (HasDistance t, 
     NumOrd.PartialComparison (Distance t),
     RoundedAdd (Distance t),
     Arbitrary (NumOrd.PartialCompareEffortIndicator (Distance t)), 
     Show (NumOrd.PartialCompareEffortIndicator (Distance t)),
     Arbitrary (AddEffortIndicator (Distance t)), 
     Show (AddEffortIndicator (Distance t)),
     Arbitrary (DistanceEffortIndicator t), 
     Show (DistanceEffortIndicator t), 
     Arbitrary t, Show t) =>
    (String, t) -> Test
testsDistance (name, sample) =
    testGroup (name ++ " distance measure") $ 
        [
         testProperty "triangle inequality" (propDistanceTriangular sample)
        ]

{-|
   A numeric measure of imprecision of approximations.
   The imprecision type should support Partial comparison.
   A zero imprecision usually means that the approximation is exact.
-}
class HasImprecision t where
    type Imprecision t
    type ImprecisionEffortIndicator t
    imprecisionDefaultEffort :: t -> ImprecisionEffortIndicator t
    imprecisionOfEff :: ImprecisionEffortIndicator t -> t -> Imprecision t

imprecisionOf :: (HasImprecision t) => t -> Imprecision t
imprecisionOf v = imprecisionOfEff (imprecisionDefaultEffort v) v

propImprecisionDecreasesWithRefinement ::
    (HasImprecision t, NumOrd.PartialComparison (Imprecision t)) =>
    t -> 
    (ImprecisionEffortIndicator t) -> 
    (NumOrd.PartialCompareEffortIndicator (Imprecision t)) -> 
    RefOrd.LEPair t -> Bool
propImprecisionDecreasesWithRefinement _ effortImpr effortComp (RefOrd.LEPair (e1,e2)) =
    let (>=?) = NumOrd.pGeqEff effortComp in
    case (imprecisionOfEff effortImpr e1) >=? (imprecisionOfEff effortImpr e2) of
        Nothing -> True
        Just b -> b

testsImprecision (name, sample) =
    testGroup (name ++ " imprecision measure") $ 
        [
         testProperty "decreases with refinement" (propImprecisionDecreasesWithRefinement sample)
        ]

instance 
    (HasImprecision t,
     NumOrd.RefinementRoundedLattice (Imprecision t)) 
    => 
    HasImprecision [t]
    where
    type Imprecision [t] = Imprecision t
    type ImprecisionEffortIndicator [t] = 
        (ImprecisionEffortIndicator t, NumOrd.MinmaxInOutEffortIndicator (Imprecision t))
    imprecisionDefaultEffort (h : _) =
        (effImpr, NumOrd.minmaxInOutDefaultEffort (imprecisionOfEff effImpr h))
        where
        effImpr = imprecisionDefaultEffort h
    imprecisionOfEff (effImpr, effMax) list@(_:_) =
        foldl1 (NumOrd.maxOutEff effMax) $ map (imprecisionOfEff effImpr) list
--    isExactEff (eff, _) list =
--        fmap and $ sequence $ map (isExactEff eff) list
         
         
instance
    HasImprecision a
    =>
    HasImprecision (Maybe a)
    where
    type Imprecision (Maybe a) = Maybe (Imprecision a)
    type ImprecisionEffortIndicator (Maybe a) = 
        ImprecisionEffortIndicator a
    imprecisionDefaultEffort maybeSample =
        case maybeSample of
            (Just sample) -> 
                imprecisionDefaultEffort sample
            _ -> error $ "(imprecisionDefaultEffort Nothing) not defined"
    imprecisionOfEff _ Nothing = Nothing
    imprecisionOfEff eff (Just v) = Just $ imprecisionOfEff eff v
    
instance
    HasImprecision Bool
    where
    type Imprecision Bool = ()
    type ImprecisionEffortIndicator Bool = ()
    imprecisionDefaultEffort _ = ()
    imprecisionOfEff _ _ = ()
    
{-|
    A generic function that applies a given approximate function
    repeatedly with increasing effort until the required accuracy
    is reached.
-}
iterateUntilAccurate :: 
    (HasImprecision t,
    NumOrd.PartialComparison (Imprecision t),
    Show (Imprecision t),
    EffortIndicator eff) 
    =>
    eff {-^ @initEff@ the initial effort -} -> 
    Int {-^ @iterLimit@ maximum number of different efforts to try out -} -> 
    (Imprecision t) {-^ @maxImprecision@ imprecision threshold for the result -} -> 
    (eff -> t) {-^ the function to compute -} -> 
    [(eff,t,Imprecision t)] {-^ the efforts that were tried and the corresponding results -}
iterateUntilAccurate initEff iterLimit maxImprecision fnEff =
    iterateWithIncreasingPrecisionUntilOK initEff iterLimit resultOK fnEff
    where
    resultOK _result resultImprecision = 
        (resultImprecision NumOrd.<=? maxImprecision) == Just True 
{-|
    A generic function that applies a given approximate function
    repeatedly with increasing effort until the required accuracy
    is reached.
-}
iterateWithIncreasingPrecisionUntilOK :: 
    (HasImprecision t,
    NumOrd.PartialComparison (Imprecision t),
    Show (Imprecision t),
    EffortIndicator eff) 
    =>
    eff {-^ @initEff@ the initial effort -} -> 
    Int {-^ @iterLimit@ maximum number of different efforts to try out -} -> 
    (t -> Imprecision t -> Bool) {-^ @isResultOK@ function to accept or reject an approximate result -} -> 
    (eff -> t) {-^ the function to compute -} -> 
    [(eff,t,Imprecision t)] {-^ the efforts that were tried and the corresponding results -}
iterateWithIncreasingPrecisionUntilOK initEff iterLimit isResultOK fnEff =
    stopWhenAccurate $ 
        take iterLimit $ 
            map addImprecision $ zip efforts (map fnEff efforts)
    where
    efforts = effortIncrementSequence initEff

    addImprecision (eff, fn) = (eff, fn, imprecisionOf fn)

    stopWhenAccurate [] = []
    stopWhenAccurate ((eff, result, resultImprecision) : rest)
        | accurateEnough = [(eff, result, resultImprecision)]
        | otherwise = (eff, result, resultImprecision) : (stopWhenAccurate rest)
        where
        accurateEnough = isResultOK result resultImprecision
--            (maxImprecision NumOrd.>=? resultImprecision) == Just True
         
{-|
    A generic function that applies a given approximate function
    repeatedly with increasing effort until the required accuracy
    is reached.
    
    This function is a bit more sophisticated than iterateUntilAccurate.
    It tries to keep repeating incrementing individual effort components
    until an improvement is made. 
-}
iterateUntilAccurate2 :: 
    (HasImprecision t,
    NumOrd.PartialComparison (Imprecision t),
    RefOrd.IntervalLike (Imprecision t),
    Show (Imprecision t),
    EffortIndicator eff) 
    =>
    eff {-^ @initEff@ the initial effort -} -> 
    Int {-^ @iterLimit@ maximum number of different efforts to try out -} -> 
    Int {-^ @improvementDepth@ maximum length of effort improvement chain to try -} -> 
    Imprecision t {-^ @maxImprecision@ imprecision threshold for the result -} -> 
    (eff -> t) {-^ the function to compute -} -> 
    [(eff,t,Imprecision t)] {-^ the efforts that were tried and the corresponding results -}
iterateUntilAccurate2 initEff iterLimit maxJump maxImprecision fnEff =
    keepImprovingEffort iterLimit $ useEffort initEff
    where
    useEffort eff =
        (eff, fn, imprecisionOf fn)
        where
        fn = fnEff eff
    keepImprovingEffort remainingAttempts prevResult@(prevEff, _, prevImprecision) 
        | prevImprecision `imprecisionLessThan` maxImprecision = -- done!
            [prevResult]
        | remainingAttempts == 0 = -- ran out of attempts, give up...
            [prevResult]
        | otherwise =
            case maybeNextResultN of
                (Just nextResult@(_, _, nextImprecision)) -> 
--                    trace ("iterateUntilAccurate2: keepImprovingEffort: "
--                        ++ "\n possibleResults = " ++ (unlines $ map show $ map (\(a,_,b) -> (b,a)) possibleResultsN) 
--                    ) $
                    prevResult : keepImprovingEffort (remainingAttempts - 1) nextResult
                _ ->
                    [prevResult]   
        where
        maybeNextResultN = getBestResult Nothing possibleResultsN
        possibleResultsN = map useEffort possibleEffortsN
        
        possibleEfforts1 = effortIncrementVariants prevEff
        possibleEffortsN = concat $ map (take maxJump . repeatTheImprovement) possibleEfforts1
            where
            repeatTheImprovement eff = drop 1 $ aux (prevEff, eff)
                where
                aux (eff1, eff2) = eff1 : aux (eff2, eff3)
                    where
                    eff3 = effortRepeatIncrement (eff1, eff2)  
        
    getBestResult maybeBestSoFar [] = maybeBestSoFar
    getBestResult Nothing (result : rest) = getBestResult (Just result) rest 
    getBestResult (Just bestSoFar@(_,_,bestSoFarImprecision)) (result@(_,_,imprecision) : rest) 
        | imprecision `imprecisionLessThan` bestSoFarImprecision =  
            getBestResult (Just result) rest
        | otherwise =
            getBestResult (Just bestSoFar) rest
         
    i1 `imprecisionLessThan` i2 =
        (i1R NumOrd.<? i2R) == Just True
        where
        (_, i1R) = RefOrd.getEndpointsOut i1
        (_, i2R) = RefOrd.getEndpointsOut i2
            