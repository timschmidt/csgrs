{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE ScopedTypeVariables #-}
{-|
    Module      :  Numeric.AERN.IVP.Specification.Hybrid
    Description :  specification of hybrid system IVP with uncertainty
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Specification of hybrid system IVP with uncertainty.
-}

module Numeric.AERN.IVP.Specification.Hybrid
--(
--)
where

import Numeric.AERN.RmToRn.Domain
--import Numeric.AERN.RmToRn.New

--import Numeric.AERN.RmToRn.Evaluation
--import Numeric.AERN.RmToRn.Integration
--
import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut
import Numeric.AERN.RealArithmetic.ExactOps
import Numeric.AERN.RealArithmetic.Measures
--
--import qualified Numeric.AERN.NumericOrder as NumOrd
--import Numeric.AERN.NumericOrder.Operators
--
import qualified Numeric.AERN.RefinementOrder as RefOrd

--import Numeric.AERN.Misc.Debug

import qualified Data.Set as Set
import qualified Data.Map as Map
import qualified Data.List as List

data HybridIVP f =
    HybridIVP
    {
        hybivp_description :: String
    ,
        hybivp_system :: HybridSystem f
    , 
        hybivp_tVar :: Var f -- ^ @tVar@
    ,
        hybivp_tStart :: Domain f -- ^ @tStart@
    ,
        hybivp_tEnd :: Domain f -- ^ @tEnd@
    ,
        hybivp_initialStateEnclosure :: HybridSystemUncertainState (Domain f) 
        {-^
        -}
    ,
        hybivp_maybeExactStateAtTEnd :: Maybe (HybridSystemUncertainState (Domain f))
    }

data HybridSystem f =
    HybridSystem
    {
        hybsys_componentNames :: [Var f]
    ,
        hybsys_modeFields :: Map.Map HybSysMode ([f] -> [f])
    ,
        hybsys_modeInvariants :: Map.Map HybSysMode ([Domain f] -> Maybe [Domain f])
            {- each invariant is represented by a function that approximates the intersection with the mode domain -}
    ,
        hybsys_eventSpecification :: 
            HybSysMode ->
            Map.Map HybSysEventKind
                (
                 HybSysMode,
                 [Domain f] -> [Domain f],
                 [Bool],
                    {- indication of which components are affected by the event;
                       a component is affected if the reset function changes its value OR
                       if the new mode has a different dynamics for this variable -}
--                 [f] -> f,
--                    {- construction of a zero crossing function -}
--                 [Domain f] -> Maybe Bool,
--                    {- additional reset condition -}
                 Domain f -> [Domain f] -> Maybe [Domain f] 
                    {- intersection with the support of the guard, 
                       used to detect some cases when this kind of event is ruled out 
                       and to prune some impossible states at the switching point;
                       the first parameter is the current time interval
                    -} 
                )  
    }
    
newtype HybSysMode = HybSysMode String deriving (Eq, Ord, Show)
newtype HybSysEventKind = HybSysEventKind String deriving (Eq, Ord, Show)

type HybridSystemUncertainState dom =
    Map.Map HybSysMode [dom]

--data HybridSystemUncertainStateOneMode f =
--    HybridSystemUncertainStateOneMode
--    {
--        hybstate_mode :: HybSysMode
--    ,
--        hybstate_values :: [Domain f]
--    }

mergeHybridStates ::
    (RefOrd.RoundedLattice dom)
    =>
    RefOrd.JoinMeetEffortIndicator dom ->
    HybridSystemUncertainState dom ->
    HybridSystemUncertainState dom ->
    HybridSystemUncertainState dom
mergeHybridStates effJoin state1 state2 =    
    Map.unionWith mergeValues state1 state2
    where
    mergeValues values1 values2 =
        zipWith (</\>) values1 values2
        where
        (</\>) = RefOrd.meetOutEff effJoin

differenceHybridStates ::
    (RefOrd.PartialComparison dom, RefOrd.IntervalLike dom)
    =>
    RefOrd.PartialCompareEffortIndicator dom ->
    HybridSystemUncertainState dom ->
    HybridSystemUncertainState dom ->
    HybridSystemUncertainState dom
differenceHybridStates effComp state1 (state2 :: HybridSystemUncertainState dom) =    
    Map.differenceWith diffValues state1 state2
    where
    diffValues values1 values2 =
        case List.findIndices (== False) containmentFlags of
            [] -> 
                Nothing 
                -- values1 fully inside values2 - the difference is empty
            [ixOnlyNoncontainment] ->
                Just $ differenceOneComponent ixOnlyNoncontainment values1 values2
            _ -> 
                Just values1
                -- more than one component is not fully covered by values2; 
                -- the box hull of the difference is equal to the original box
        where
        containmentFlags =
            map (== Just True) $        
            zipWith (|>=?) values1 (values2 :: [dom])
            where
            (|>=?) = RefOrd.pGeqEff effComp
        differenceOneComponent ix (h1:t1) (h2:t2)
            | ix == 0 = (diffValue h1 h2) : t1
            | otherwise = h1 : (differenceOneComponent (ix - 1) t1 t2)
            where
            diffValue value1 value2 
                | cuttingFromRight = 
                    RefOrd.fromEndpointsOut (value1L, value2L)
                | cuttingFromLeft = 
                    RefOrd.fromEndpointsOut (value2R, value1R)
                | otherwise =
                    value1
                where
                cuttingFromLeft = 
                    (value1 |<=? value2R) == Just True
                cuttingFromRight = 
                    (value1 |<=? value2L) == Just True
                (value1L, value1R) = RefOrd.getEndpointsOut value1
                (value2L, value2R) = RefOrd.getEndpointsOut value2
                (|<=?) = RefOrd.pLeqEff effComp

getHybridStateUnion ::
    (RefOrd.RoundedLattice dom)
    =>
    RefOrd.JoinMeetEffortIndicator dom ->
    HybridSystemUncertainState dom ->
    (Set.Set HybSysMode, [dom])
getHybridStateUnion effJoin state =
    (Set.fromAscList modes, foldl1 mergeValues valueVecs) 
    where
    (modes, valueVecs) =
        unzip $ Map.toAscList state
    mergeValues values1 values2 =
        zipWith (</\>) values1 values2
        where
        (</\>) = RefOrd.meetOutEff effJoin
    
measureImprovementState :: 
      (HasImprecision dom,
       Imprecision dom ~ dom,
       ArithInOut.RoundedReal dom) 
      =>
      dom
      -> ArithInOut.RoundedRealEffortIndicator dom
      -> HybridSystemUncertainState dom
      -> HybridSystemUncertainState dom
      -> Imprecision dom
measureImprovementState sampleDom effDom state1 state2
    | newModeInState2 = neg (one sampleDom) -- =-1 serious deterioration
    | otherwise = 
        modeCountDecreaseInState2 |<+> improvement 
    where
    newModeInState2 = 
        not $ Map.null $ Map.difference state2 state1
    modeCountDecreaseInState2 = 
        Map.size $ Map.difference state1 state2
    improvement =
        Map.foldr (<+>) (zero sampleDom) improvementsPerModeMap
    improvementsPerModeMap =
        Map.intersectionWith measureImprovementVec state1 state2

    measureImprovementVec vec1 vec2 = 
        foldl1 (<+>) improvements
        where
        improvements = map measureImprovement $ zip vec1 vec2
    measureImprovement (encl1, encl2) =
        (imprecisionOfEff effImpr encl1) <-> (imprecisionOfEff effImpr encl2)

    (<+>) = ArithInOut.addOutEff effAddDom
    (<->) = ArithInOut.subtrOutEff effAddDom
    (|<+>) = flip $ ArithInOut.mixedAddOutEff effAddDomInt

    effAddDom = ArithInOut.fldEffortAdd sampleDom $ ArithInOut.rrEffortField sampleDom effDom
    effAddDomInt = 
        ArithInOut.mxfldEffortAdd sampleDom (1 :: Int) $ 
            ArithInOut.rrEffortIntMixedField sampleDom effDom
    effImpr = ArithInOut.rrEffortImprecision sampleDom effDom
    
    
    
