{-# OPTIONS_GHC -fno-warn-orphans #-}
{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE UndecidableInstances #-}
{-|
    Module      :  Numeric.AERN.Basics.Interval.RefinementOrder
    Description :  interval instances of refinement-ordered structures 
    Copyright   :  (c) Michal Konecny, Jan Duracz
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Interval instances of refinement-ordered structures.
    
    This is a hidden module reexported via its parent.
-}
module Numeric.AERN.Basics.Interval.RefinementOrder where

import Prelude hiding (EQ, LT, GT)

import Numeric.AERN.Basics.Consistency
import Numeric.AERN.Basics.Arbitrary
import Numeric.AERN.Basics.PartialOrdering

import Numeric.AERN.Basics.Interval.Basics
import Numeric.AERN.Basics.Interval.NumericOrder ()

import qualified Numeric.AERN.NumericOrder as NumOrd
import Numeric.AERN.NumericOrder.Operators
import qualified Numeric.AERN.RefinementOrder as RefOrd
import Numeric.AERN.RefinementOrder
        (
                GetEndpointsEffortIndicator,
                FromEndpointsEffortIndicator,
                PartialCompareEffortIndicator,
                PartialJoinEffortIndicator,
                JoinMeetEffortIndicator
        )

import Numeric.AERN.Misc.List

import Test.QuickCheck

import Data.Maybe

instance
    (RefOrd.IntervalLike (Interval e))
    where
    type GetEndpointsEffortIndicator (Interval e) = ()
    type FromEndpointsEffortIndicator (Interval e) = ()
    getEndpointsDefaultEffort _ = ()
    fromEndpointsDefaultEffort _ = ()
--    getEndpointsInEff _ (Interval l r) = (Interval l l,Interval r r)
    getEndpointsOutEff _ (Interval l r) = (Interval l l,Interval r r)
    fromEndpointsInEff _ (Interval _ll lr, Interval rl _rr) = (Interval lr rl) 
    fromEndpointsOutEff _ (Interval ll _lr, Interval _rl rr) = (Interval ll rr)

instance 
    (NumOrd.PartialComparison e, NumOrd.RoundedLatticeEffort e) 
    => 
    (RefOrd.PartialComparison (Interval e))
    where
    type PartialCompareEffortIndicator (Interval e) = 
        IntervalOrderEffort e 
    pCompareDefaultEffort i = 
        defaultIntervalOrderEffort i
    pCompareEff effort i1 i2 =
        case partialInfo2PartialOrdering $ RefOrd.pCompareInFullEff effort i1 i2 of
            [ord] -> Just ord
            _ -> Nothing
    pCompareInFullEff effort (Interval l1 r1) (Interval l2 r2)
        =
        refordPCompareInFullIntervalsEff effComp (l1, r1) (l2, r2)
        where
        effComp = intordeff_eComp effort  
    
refordPCompareInFullIntervalsEff :: 
      (NumOrd.PartialComparison t) 
      =>
      NumOrd.PartialCompareEffortIndicator t
      -> (t, t) 
      -> (t, t) 
      -> PartialOrderingPartialInfo
refordPCompareInFullIntervalsEff effort (l1, r1) (l2, r2)
    = 
    PartialOrderingPartialInfo
    {
        pOrdInfEQ = fromD (eqD, neqD),
        pOrdInfLT = fromD (ltD, nltD),
        pOrdInfLEQ = fromD (leqD, nleqD),
        pOrdInfGT = fromD (gtD, ngtD),
        pOrdInfGEQ = fromD (geqD, ngeqD),
        pOrdInfNC = fromD (nleqD && ngeqD, leqD || geqD)
    }
--        case (c l1 l2, c r1 r2) of
--            (Just EQ, Just EQ) -> Just EQ
--            (Just LT, Just GT) -> Just LT  
--            (Just LT, Just EQ) -> Just LT  
--            (Just EQ, Just GT) -> Just LT  
--            (Just GT, Just LT) -> Just GT  
--            (Just GT, Just EQ) -> Just GT  
--            (Just EQ, Just LT) -> Just GT  
--            (Just _, Just _) -> Just NC  
--            _ -> Nothing
    where
    fromD (definitelyTrue, definitelyFalse) =
        case (definitelyTrue, definitelyFalse) of
            (True, _) -> Just True
            (_, True) -> Just False
            _ -> Nothing
    eqD = leftEQ == jt && rightEQ == jt
    neqD = leftEQ == jf || rightEQ == jf
    leqD = leftLEQ == jt && rightGEQ == jt
    nleqD = leftLEQ == jf || rightGEQ == jf
    ltD = leqD && neqD
    nltD = nleqD || eqD
    gtD = geqD && neqD
    ngtD = ngeqD || eqD
    geqD = leftGEQ == jt && rightLEQ == jt
    ngeqD = leftGEQ == jf || rightLEQ == jf
    
    leftEQ = pOrdInfEQ leftInfo
--    leftLT = pOrdInfLT leftInfo
    leftLEQ = pOrdInfLEQ leftInfo
--    leftGT = pOrdInfGT leftInfo
    leftGEQ = pOrdInfGEQ leftInfo
    leftInfo = NumOrd.pCompareInFullEff effort l1 l2 
    rightEQ = pOrdInfEQ rightInfo
--    rightLT = pOrdInfLT rightInfo
    rightLEQ = pOrdInfLEQ rightInfo
--    rightGT = pOrdInfGT rightInfo
    rightGEQ = pOrdInfGEQ rightInfo
    rightInfo = NumOrd.pCompareInFullEff effort r1 r2 
    jt = Just True
    jf = Just False
        

{-
    Beware, the following instance does not test inclusion of the two approximated intervals.
    This instance compares that one approximation permits a subset of intervals
    that the other approximation permits.
-}
instance 
    (NumOrd.PartialComparison e, NumOrd.RoundedLatticeEffort e) 
    => 
    (RefOrd.PartialComparison (IntervalApprox e))
    where
    type PartialCompareEffortIndicator (IntervalApprox e) = 
        IntervalOrderEffort e 
    pCompareDefaultEffort (IntervalApprox o _) = 
        defaultIntervalOrderEffort o
    pCompareEff effort ia1 ia2 =
        case partialInfo2PartialOrdering $ RefOrd.pCompareInFullEff effort ia1 ia2 of
            [ord] -> Just ord
            _ -> Nothing
    pCompareInFullEff effort (IntervalApprox o1 i1) (IntervalApprox o2 i2)
        =
        compOuter `partialOrderingPartialInfoAnd` compInner
        where
        compOuter = RefOrd.pCompareInFullEff effort o1 o2  
        compInner = RefOrd.pCompareInFullEff effort i2 i1
    
intervalApproxIncludedInEff :: 
    (NumOrd.RoundedLatticeEffort t, NumOrd.PartialComparison t) 
    =>
    IntervalOrderEffort t -> 
    IntervalApprox t -> IntervalApprox t -> Maybe Bool
intervalApproxIncludedInEff eff (IntervalApprox o1 _i1) (IntervalApprox o2 i2) =
    case (o1 `includedIn` i2, o1 `includedIn` o2) of
        (Just True, _) -> Just True -- outer inside inner -> inclusion proved
        (_,Just False) -> Just False -- outer definitely not inside outer, ie outers are disjoint somewhere -> inclusion disproved 
        _ -> Nothing
    where
    includedIn = RefOrd.pGeqEff eff

intervalApproxIncludedIn :: 
    (NumOrd.RoundedLatticeEffort t, NumOrd.PartialComparison t) 
    =>
    IntervalApprox t -> IntervalApprox t -> Maybe Bool
intervalApproxIncludedIn ia1@(IntervalApprox o _) =
    intervalApproxIncludedInEff eff ia1
    where
    eff = RefOrd.pCompareDefaultEffort o

instance (NumOrd.HasExtrema e) => (RefOrd.HasTop (Interval e))
    where
    top (Interval sampleE _) = 
        Interval (NumOrd.greatest sampleE) (NumOrd.least sampleE)
    
instance (NumOrd.HasExtrema e) => (RefOrd.HasBottom (Interval e))
    where
    bottom (Interval sampleE _) = 
        Interval (NumOrd.least sampleE) (NumOrd.greatest sampleE)

instance (NumOrd.HasExtrema e) => (RefOrd.HasExtrema (Interval e))

instance 
    (NumOrd.RoundedLatticeEffort e, NumOrd.PartialComparison e) 
    => RefOrd.RoundedBasisEffort (Interval e)
    where
    type PartialJoinEffortIndicator (Interval e) =
        IntervalOrderEffort e 
    partialJoinDefaultEffort i =
        defaultIntervalOrderEffort i
    
instance 
    (NumOrd.RoundedLattice e, NumOrd.PartialComparison e) 
    => 
    RefOrd.RoundedBasis (Interval e) 
    where
    partialJoinOutEff effort (Interval l1 r1) (Interval l2 r2) = 
            case l <=? r of
                Just True -> Just $ Interval l r
                _ -> Nothing
            where
            (<=?) = NumOrd.pLeqEff effortComp
            l = NumOrd.maxDnEff effortMinmax l1 l2
            r = NumOrd.minUpEff effortMinmax r1 r2
            effortMinmax = intordeff_eMinmax effort
            effortComp = intordeff_eComp effort
    partialJoinInEff effort (Interval l1 r1) (Interval l2 r2) = 
            case l <=? r of
                Just True -> Just $ Interval l r
                _ -> Nothing
            where
            (<=?) = NumOrd.pLeqEff effortComp
            l = NumOrd.maxUpEff effortMinmax l1 l2
            r = NumOrd.minDnEff effortMinmax r1 r2
            effortMinmax = intordeff_eMinmax effort
            effortComp = intordeff_eComp effort



instance 
    (NumOrd.RoundedLattice e, NumOrd.PartialComparison e) 
    => 
    (RefOrd.RoundedLatticeEffort (Interval e)) 
    where
    type JoinMeetEffortIndicator (Interval e) = 
        IntervalOrderEffort e 
    joinmeetDefaultEffort i =
        defaultIntervalOrderEffort i

instance 
    (NumOrd.RoundedLattice e, NumOrd.PartialComparison e) 
    => 
    (RefOrd.RoundedLattice (Interval e)) 
    where
    joinOutEff effort (Interval l1 r1) (Interval l2 r2) =
            Interval l r
            where
            l = NumOrd.maxDnEff effMinmax l1 l2
            r = NumOrd.minUpEff effMinmax r1 r2
            effMinmax = intordeff_eMinmax effort
    meetOutEff effort (Interval l1 r1) (Interval l2 r2) =
            Interval l r
            where
            l = NumOrd.minDnEff effMinmax l1 l2
            r = NumOrd.maxUpEff effMinmax r1 r2
            effMinmax = intordeff_eMinmax effort
    joinInEff effort (Interval l1 r1) (Interval l2 r2) =
            Interval l r
            where
            l = NumOrd.maxUpEff effMinmax l1 l2
            r = NumOrd.minDnEff effMinmax r1 r2
            effMinmax = intordeff_eMinmax effort
    meetInEff effort (Interval l1 r1) (Interval l2 r2) =
            Interval l r
            where
            l = NumOrd.minUpEff effMinmax l1 l2
            r = NumOrd.maxDnEff effMinmax r1 r2
            effMinmax = intordeff_eMinmax effort

intervalApproxUnionEff :: 
    (NumOrd.PartialComparison e, NumOrd.RoundedLattice e) 
    =>
    IntervalOrderEffort e
    -> IntervalApprox e -> IntervalApprox e -> IntervalApprox e
intervalApproxUnionEff effort (IntervalApprox o1 i1) (IntervalApprox o2 i2) =
    IntervalApprox 
        (RefOrd.meetOutEff effort o1 o2) 
        (RefOrd.meetInEff effort i1 i2)

intervalApproxUnion :: 
    (NumOrd.PartialComparison e, NumOrd.RoundedLattice e) 
    =>
    IntervalApprox e -> IntervalApprox e -> IntervalApprox e
intervalApproxUnion ia1@(IntervalApprox o _) ia2 =
    intervalApproxUnionEff effort ia1 ia2
    where
    effort = RefOrd.joinmeetDefaultEffort o


instance
    (NumOrd.AreaHasBoundsConstraints e)
    =>
    (RefOrd.AreaHasBoundsConstraints (Interval e))
    where
    areaSetOuterBound (Interval l r) (areaEndpt, consistency) =
        (NumOrd.areaSetUpperBound (r, False) $ NumOrd.areaSetLowerBound (l, False) areaEndpt, 
         consistency)
      

instance 
    (NumOrd.ArbitraryOrderedTuple e,
     NumOrd.RoundedLattice e,
     AreaHasForbiddenValues e,
     NumOrd.PartialComparison e) 
    => 
    RefOrd.ArbitraryOrderedTuple (Interval e) 
    where
    arbitraryTupleInAreaRelatedBy area = 
        arbitraryIntervalTupleInAreaRefinementRelatedBy (Just area)
    arbitraryTupleRelatedBy = 
        arbitraryIntervalTupleInAreaRefinementRelatedBy Nothing

instance AreaHasConsistencyConstraint (Interval e)
    where
    areaSetConsistencyConstraint _ constraint (areaEndpt, _) = (areaEndpt, constraint)

arbitraryIntervalTupleInAreaRefinementRelatedBy maybeArea indices constraints =
    case endpointGens of
        [] -> Nothing
        _ -> Just $
            do
            gen <- elements endpointGens
            avoidForbidden gen 100 -- maximum tries
    where
    avoidForbidden gen maxTries =
        do
        endpointTuple <- gen
        let results = endpointsToIntervals endpointTuple
        case nothingForbiddenInsideIntervals results of
            True -> return results
            _ | maxTries > 0 -> avoidForbidden gen $ maxTries - 1
            _ -> error "aern-interval: internal error in arbitraryIntervalTupleInAreaRefinementRelatedBy: failed to avoid forbidden values"
    
    nothingForbiddenInsideIntervals intervals =
        case maybeArea of
            Nothing -> True
            Just (areaEndpt, _) ->
                and $ map (nothingForbiddenInsideInterval areaEndpt) intervals
    nothingForbiddenInsideInterval areaEndpt interval =
        and $ map (notInside interval) $ areaGetForbiddenValues areaEndpt
        where
        notInside (Interval l r) value = 
            ((value <? l) == Just True)
            ||
            ((value >? r) == Just True)  

    endpointGens =
        case maybeArea of
            (Just (areaEndpt, _areaConsistency)) ->
                catMaybes $
                   map (NumOrd.arbitraryTupleInAreaRelatedBy areaEndpt endpointIndices)
                       endpointConstraintsVersions
            Nothing ->
                catMaybes $
                   map (NumOrd.arbitraryTupleRelatedBy endpointIndices) 
                       endpointConstraintsVersions
    endpointIndices = 
        concat $ map (\ix -> [(ix,-1), (ix,1)]) indices
    endpointsToIntervals [] = []
    endpointsToIntervals (l : r : rest) =
        (Interval l r) : (endpointsToIntervals rest)
    endpointConstraintsVersions =
--        unsafePrintReturn 
--        ("arbitraryIntervalTupleRelatedBy:"
--         ++ "\n indices = " ++ show indices 
--         ++ "\n constraints = " ++ show constraints 
--         ++ "\n endpointIndices = " ++ show endpointIndices 
--         ++ "\n endpointConstraintsVersions = "
--        ) $
        map concat $ 
--        map ((++ thinnessConstraints) . concat) $ 
            combinations $ map intervalConstraintsToEndpointConstraints constraints
--    thinnessConstraints = map (\ix -> (((ix,-1),(ix,1)),[EQ])) thinIndices
    allowedEndpointRelations =
        case maybeArea of
            Just (_, AreaMaybeAllowOnlyWithConsistencyStatus (Just Consistent)) -> [EQ,LT]
            Just (_, AreaMaybeAllowOnlyWithConsistencyStatus (Just Anticonsistent)) -> [EQ,GT]
            Just (_, AreaMaybeAllowOnlyWithConsistencyStatus (Just Exact)) -> [EQ]
            _ -> [EQ,LT,GT]
    intervalConstraintsToEndpointConstraints :: 
        ((ix, ix), [PartialOrdering]) -> [[(((ix,Int), (ix,Int)), [PartialOrdering])]]
    intervalConstraintsToEndpointConstraints ((ix1, ix2),rels) =
        concat $ map forEachRel rels
        where
        endpoints1Comparable = [(((ix1,-1),(ix1, 1)), allowedEndpointRelations)]
        endpoints2Comparable = [(((ix2,-1),(ix2, 1)), allowedEndpointRelations)]
        endpointsComparable = endpoints1Comparable ++ endpoints2Comparable
        forEachRel EQ = -- both endpoints agree 
            [
                endpointsComparable ++
                [(((ix1,-1),(ix2,-1)), [EQ]), (((ix1,1),(ix2,1)), [EQ])]
            ]
        forEachRel GT =
            -- the interval ix1 is indide ix2, but not equal
            [
                endpointsComparable ++
                [(((ix1,-1),(ix2,-1)), [GT])] ++ 
                [(((ix1,1),(ix2,1)), [EQ, LT])]
            ]
            ++
            [
                endpointsComparable ++
                [(((ix1,-1),(ix2,-1)), [EQ, GT])] ++ 
                [(((ix1,1),(ix2,1)), [LT])]
            ]
        forEachRel LT =  
            -- the interval ix2 is indide ix1, but not equal
            [
                endpointsComparable ++
                [(((ix2,-1),(ix1,-1)), [GT])] ++ 
                [(((ix2,1),(ix1,1)), [EQ, LT])]
            ]
            ++
            [
                endpointsComparable ++
                [(((ix2,-1),(ix1,-1)), [EQ, GT])] ++ 
                [(((ix2,1),(ix1,1)), [LT])]
            ]
        forEachRel NC =
            -- either some pair of endpoints is NC:
            [ endpointsComparable ++ [(((ix1,side1), (ix2, side2)),[NC])]  
               | side1 <- [-1,1], side2 <- [-1,1]
            ]
            ++
            -- or the interval ix1 is to the left of ix2
            [
                endpointsComparable ++
                [(((ix1,-1),(ix2,-1)), [LT]), 
                 (((ix1,1),(ix2,1)), [LT])]
            ]
            ++
            -- or the interval ix2 is to the left of ix1
            [
                endpointsComparable ++
                [(((ix2,-1),(ix1,-1)), [LT]), 
                 (((ix2,1),(ix1,1)), [LT])]
            ]
--        forEachRel _ = []
       
       
    