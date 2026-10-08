{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE ScopedTypeVariables #-}
{-|
    Module      :  Numeric.AERN.IVP.Solver.Events.Locate
    Description :  Event localisation logic
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    The logic of searching for the first potential event in a hybrid system.
-}

module Numeric.AERN.IVP.Solver.Events.Locate
--(
--)
where

--import Numeric.AERN.RmToRn.Domain
--import Numeric.AERN.RmToRn.New
--import Numeric.AERN.RmToRn.Evaluation

import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut
import Numeric.AERN.RealArithmetic.RefinementOrderRounding.Operators
--import Numeric.AERN.RealArithmetic.Measures
--import Numeric.AERN.RealArithmetic.ExactOps

--import qualified Numeric.AERN.NumericOrder as NumOrd
import Numeric.AERN.NumericOrder.Operators

import qualified Numeric.AERN.RefinementOrder as RefOrd

--import Numeric.AERN.Basics.Consistency

--import qualified Data.Map as Map
import qualified Data.Set as Set
--import qualified Data.List as List

import Numeric.AERN.Misc.Debug
_ = unsafePrint

    
{-
    Do binary splitting to a certain depth and for each segment get one of the following three values:

    N: event nowhere on this segment
    S: event certainly on this segment
    M: no idea
    
    If a segment returns N, do not split it any further.  
    
    By splitting like this to a certain depth we get a finite sequence over the alphabet {N,S,M}.
    We then reduce the sequence to a normal form using the following rewrite rules:
    
    S_ -> S
    NN -> N
    MNM -> M
    MM -> M
    MNS -> S
    MS -> S

    The following are all the possible normal forms:
    S, NS, N, M, MN, NM, NMN
    
    At the same time, with each S and M, associate a segment within T.  Some of the above rules enlarge this segment.
    
    A quicker way to compute the normal form and the segment of first occurence: 
        ignore all N segments unless there are only N segments, in which case return N
        if there is some S, ignore everything after the first S and merge it with all preceding M's, returning S
        if there is no S, merge all M's, returning M
    
    
    typical simple call:
        locateFirstZeroDip stepLimit (const $ Just True) [dipFn] dipFn 
-}
locateFirstDipAmongMultipleFns :: 
    (RefOrd.IntervalLike (dom),
     ArithInOut.RoundedReal (dom),
     Ord eventId, 
     Show dom, Show eventId)
    =>
    dom  {-^ minimum step size -} ->
    (dom -> Set.Set eventId) {-^ the events that are certainly excluded on the given point/area -} -> 
    (dom -> Bool) {-^ whether the mode invariant is certainly violated on the given point/area -} ->
    (dom -> Bool) {-^ whether the enclosure information is degenerate on the given point/area -} ->
    (dom, dom) {-^ domain to search over -} -> 
    LocateDipResult dom eventId
locateFirstDipAmongMultipleFns 
        stepLimit 
        eventsCertainlyInactiveOnDom
        invariantCertainlyViolatedOnDom
        invariantIndecisiveThroughoutDom
        (tStart, tEnd) =
--    unsafePrint
--    (
--        "locateFirstDipAmongMultipleFns:"
--        ++ "\n stepLimit = " ++ show stepLimit
--        ++ "\n tStart = " ++ show tStart
--        ++ "\n tEnd = " ++ show tEnd
--        ++ "\n result = " ++ show result
--    ) $
    result 
    where
    result = locateBySplitting (tStart, tEnd)
    locateBySplitting d@(dLE, dRE) =
--        unsafePrint
--        (
--            "locateFirstDipAmongMultipleFns: locateBySplitting:"
--            ++ "\n d = " ++ show d
--            ++ "; resD = " ++ show resD
--        ) $
        result2
        where
        result2
            | ((stepLimit <? size) /= Just True) = -- hit the minimum split size 
                resD -- no more splitting
            | otherwise =
                case resD of
                    LDResNone -> resD
                    LDResSome LDResDipPoorEnclosure _ _ -> resD 
                        -- splitting will not give more information because the enclosure is too poor
                    _ -> combineLocateDipResults resL resR
        resD =
            examineEventsOnDom
                eventsCertainlyInactiveOnDom
                invariantCertainlyViolatedOnDom
                invariantIndecisiveThroughoutDom
                d
        size = dRE <-> dLE
        resL = locateBySplitting (dLE, dM)
        resR = locateBySplitting (dM, dRE)
        (dM, _) = RefOrd.getEndpointsOut $ (dLE <+> dRE) </>| (2 :: Int) 
    
    
examineEventsOnDom :: 
    (RefOrd.IntervalLike dom)
    =>
    (dom -> Set.Set eventId) {-^ the events that are certainly excluded on the given point/area -} -> 
    (dom -> Bool) {-^ whether the mode invariant is certainly violated on the given point/area -} ->
    (dom -> Bool) {-^ whether the enclosure information is degenerate on the given point/area -} ->
    (dom, dom) -> 
    LocateDipResult dom eventId
examineEventsOnDom 
        eventsNotRuledOutOnDom
        invariantCertainlyViolatedOnDom
        invariantIndecisiveThroughoutDom
        (dLE, dRE) =
    case (Set.null eventsNotRuledOutOnD, invariantCertainlyViolatedOnDom dRE, invariantIndecisiveThroughoutDom d) of
        (True, _, _) -> LDResNone -- no event
        (_, True, _) -> LDResSome LDResDipCertain (dLE, dRE) eventsNotRuledOutOnD
        (_, _, True) -> LDResSome LDResDipPoorEnclosure (dLE, dRE) eventsNotRuledOutOnD
        _ -> LDResSome LDResDipMaybe (dLE, dRE) eventsNotRuledOutOnD
            -- in all other cases, we declare that we don't know for sure
    where
    d = RefOrd.fromEndpointsOut (dLE, dRE)
    eventsNotRuledOutOnD = eventsNotRuledOutOnDom d
    
    
data LocateDipResult dom eventId =
      LDResNone
    | LDResSome
        { 
            ldresEventCertainty :: LocateDipResultCertainty,
            ldresFirstEventLoc :: (dom, dom),
            ldresFirstEventType :: Set.Set eventId
        }
    deriving Show

data LocateDipResultCertainty =
    LDResDipCertain | LDResDipMaybe | LDResDipPoorEnclosure
    deriving Show

isLDResNone :: LocateDipResult dom eventId -> Bool
isLDResNone LDResNone = True
isLDResNone _ = False

isLDResSure :: LocateDipResult dom eventId -> Bool
isLDResSure (LDResSome LDResDipCertain _ _) = True 
isLDResSure _ = False

getLDResDom ::
    (Show dom, Show eventId)
    => 
    LocateDipResult dom eventId -> (dom, dom)
getLDResDom (LDResSome _ dom _) = dom
getLDResDom ldr = error $ "getLDResDom applied on " ++ show ldr

combineLocateDipResults :: 
  (Ord eventId)
  =>
  LocateDipResult dom eventId -> 
  LocateDipResult dom eventId -> 
  LocateDipResult dom eventId
combineLocateDipResults LDResNone res2 = res2
combineLocateDipResults res1@(LDResSome LDResDipCertain _ _) _ = res1
combineLocateDipResults res1 LDResNone = res1
combineLocateDipResults (LDResSome certainty1 (dL,_) events1) (LDResSome certainty2 (_,dR) events2) =
    LDResSome certainty (dL, dR) (Set.union events1 events2)
    where
    certainty =
        combineLocateDipCertainties certainty1 certainty2

combineLocateDipCertainties :: 
    LocateDipResultCertainty -> 
    LocateDipResultCertainty -> 
    LocateDipResultCertainty
combineLocateDipCertainties LDResDipCertain _ = LDResDipCertain
combineLocateDipCertainties _ LDResDipCertain = LDResDipCertain
combineLocateDipCertainties LDResDipPoorEnclosure LDResDipPoorEnclosure = LDResDipPoorEnclosure
combineLocateDipCertainties _ _ = LDResDipMaybe
