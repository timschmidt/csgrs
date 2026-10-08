{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE ScopedTypeVariables #-}
{-|
    Module      :  Numeric.AERN.IVP.Solver.Events.EventTree
    Description :  hybrid system simulation step using event trees  
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Hybrid system simulation step using event trees.
-}

module Numeric.AERN.IVP.Solver.Events.EventTree
--(
--)
where

import Numeric.AERN.IVP.Solver.Events.Locate

import Numeric.AERN.IVP.Specification.ODE
import Numeric.AERN.IVP.Specification.Hybrid
import Numeric.AERN.IVP.Solver.Picard.UncertainValue
--import Numeric.AERN.IVP.Solver.Picard.UncertainTime

import Numeric.AERN.RmToRn.Domain
import Numeric.AERN.RmToRn.New
import Numeric.AERN.RmToRn.Evaluation
import Numeric.AERN.RmToRn.Integration

import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut
--import Numeric.AERN.RealArithmetic.RefinementOrderRounding.OpsImplicitEffort
import Numeric.AERN.RealArithmetic.ExactOps

import qualified Numeric.AERN.NumericOrder as NumOrd
--import Numeric.AERN.NumericOrder.OpsDefaultEffort

import qualified Numeric.AERN.RefinementOrder as RefOrd

import Numeric.AERN.Basics.SizeLimits
import Numeric.AERN.Basics.Consistency

--import Data.Maybe (catMaybes)
import qualified Data.Map as Map
import qualified Data.Set as Set
import qualified Data.List as List (intercalate)

import Numeric.AERN.Misc.Debug
_ = unsafePrint

solveHybridIVP_UsingPicardAndEventTree ::
    (CanAddVariables f,
     CanRenameVariables f,
     CanEvaluate f,
     CanCompose f,
     CanPartiallyEvaluate f,
     HasProjections f,
     HasConstFns f,
     RefOrd.PartialComparison f,
     RefOrd.IntervalLike f,
     HasAntiConsistency f,
     NumOrd.RefinementRoundedLattice f,
     RoundedIntegration f,
     ArithInOut.RoundedAdd f,
     ArithInOut.RoundedSubtr f,
     ArithInOut.RoundedMultiply f,
     ArithInOut.RoundedMixedAdd f (Domain f),
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f), 
     HasAntiConsistency (Domain f), 
     Show f, Show (Domain f), Show (Var f), Eq (Var f)
     )
    =>
    SizeLimits f 
    ->
    PartialEvaluationEffortIndicator f 
    ->
    CompositionEffortIndicator f 
    ->
    EvaluationEffortIndicator f 
    ->
    IntegrationEffortIndicator f 
    ->
    RefOrd.PartialCompareEffortIndicator f 
    ->
    ArithInOut.AddEffortIndicator f 
    ->
    ArithInOut.MultEffortIndicator f 
    ->
    ArithInOut.MixedAddEffortIndicator f (Domain f) 
    ->
    ArithInOut.RoundedRealEffortIndicator (Domain f)
    ->
    Int {-^ maximum number of potential events to consider -}
    ->
    Domain f {-^ initial widening @delta@ -}
    ->
    Int  {-^ @m@ - limit on the number of Picard iterations after stabilisation -}
    ->
    Var f {-^ @t0@ - the initial time variable -} 
    ->
    HybridIVP f 
    ->
    (Maybe (HybridSystemUncertainState (Domain f)), [(HybSysMode, (EventInfo f))])
solveHybridIVP_UsingPicardAndEventTree
        sizeLimits effPEval effCompose effEval effInteg effInclFn effAddFn _effMultFn effAddFnDom effDom
            maxNodes
                delta m
                    _t0Var
                        (hybivp :: HybridIVP f)
    | givenUp =        
        (Nothing, modeEventInfoList)
    | otherwise =
        (Just finalState, modeEventInfoList)
    where
    effJoinDom = ArithInOut.rrEffortJoinMeet sampleDom effDom
    effInclDom = ArithInOut.rrEffortRefComp sampleDom effDom
    sampleDom = tStart
    tStart = hybivp_tStart hybivp
    tEnd = hybivp_tEnd hybivp
    tDom = RefOrd.fromEndpointsOut (tStart, tEnd)
    tVar = hybivp_tVar hybivp
    hybsys = hybivp_system hybivp
    componentNames = hybsys_componentNames hybsys
    
--    finalState :: HybridSystemUncertainState (Domain f)
    (givenUp, finalState) = 
        case sequence perModeMaybeFinalState of
            Just perModeFinalState | not (null perModeFinalState) ->
                (False, foldl1 (mergeHybridStates effJoinDom) perModeFinalState) 
            _ -> (True, error "internal error in solveEvents: finalState undefined")
    modeEventInfoList = 
        zip modeList perModeEventInfo 
    (perModeMaybeFinalState, perModeEventInfo) = 
        unzip $ map solveEventsOneMode modeValueList 
    modeList = map fst modeValueList
    modeValueList = Map.toList initState 
    initState = hybivp_initialStateEnclosure hybivp
    
    solveEventsOneMode (initialMode, initialValues) =
        (maybeFinalState, eventInfo)
        where
        maybeFinalState =
            case eventInfoCollectFinalStates effEval tVar tEnd eventInfo of
                Just states | not (null states) -> Just $ foldl1 (mergeHybridStates effJoinDom) states
                _ -> Nothing
            
        eventInfo =
            case maybeFnVecNoEvent of
                Nothing -> EventGivenUp
                Just fnVecNoEvent ->
                    collapseRootIfEvents $
                    esolve (EventTODO (initialMode, fnVecNoEvent))
                    where
                    collapseRootIfEvents (EventNextMaybe _state children)
                        | not (Map.null children) =
                            EventNextMaybe collapsedState children
                    collapseRootIfEvents (EventNextSure _state children) =
                            EventNextSure collapsedState children
                    collapseRootIfEvents tree = tree
                    collapsedState =
                        (initialMode, collapsedFnVec)
                        where
                        collapsedFnVec =
                            case wrapFnVecAsBox effEval invariantInitialMode fnVecNoEvent of
                                Just fnVec -> fnVec
                                _ -> error $ "enclosure broke mode invariant: mode = " ++ show initialMode ++ "; enclosure = " ++ show fnVecNoEvent   
                        Just invariantInitialMode = Map.lookup initialMode modeInvariants
        maybeFnVecNoEvent =
            fmap (map removeAllVarsButT) $ -- parameters have to be removed so that we can test inclusion
            fmap (!! m) $
            solveODEIVPUncertainValueExactTime_PicardIterations
                sizeLimits effCompose effInteg effInclFn effAddFn effAddFnDom effDom
                delta
                (odeivp tStart initialMode $ 
                    makeFnVecFromInitialValues componentNames initialValues)
        removeAllVarsButT fn =
            -- substitute domains for all variables except t:
            pEvalAtPointOutEff effPEval domboxNoT fn
            where
            domboxNoT =
                fromAscList $ filter ((/= tVar) . fst) varDoms
            varDoms = 
                toAscList $ getDomainBox fn

        esolve prevEventInfo0 =
            aux stateMapEmpty prevEventInfo0
            where
            aux prevStates prevEventInfo =
                case addOneLayer prevStates prevEventInfo of
                    (newEventInfo, _, False) -> newEventInfo -- nothing left to do
                    (newEventInfo, Nothing, _) -> newEventInfo -- given up
                    (newEventInfo, Just newStates, _) 
                        | eventInfoCountNodes newEventInfo > maxNodes -> newEventInfo -- reached size limit 
                        | otherwise -> aux newStates newEventInfo -- continue adding nodes
        addOneLayer prevStates0 prevEventInfo =
--            unsafePrint
--            (
--                "solveEvents: solveEventsOneMode: "
--                ++ "\n tStart = " ++ show tStart  
--                ++ "\n tEnd = " ++ show tEnd  
--                ++ "\n prevEventInfo = \n" ++ showEventInfo "   " (show . id) prevEventInfo
--            ) $
            processNode prevStates0 prevEventInfo
            where
            processNode prevStates eventInfo2 = 
                case eventInfo2 of
                    EventGivenUp -> (eventInfo2, Nothing, False)
                    EventInconsistent -> (eventInfo2, Just prevStates, False)
                    EventFixedPoint _ -> (eventInfo2, Just prevStates, False)
                    EventNextSure state children ->
                        (EventNextSure state newChildren, maybeNewStates, someChildHasChanged)
                        where
                        (newChildren, maybeNewStates, someChildHasChanged) = 
                            processChildren prevStates $ 
                                Map.toAscList children
                    EventNextMaybe state children -> 
                        (EventNextMaybe state newChildren, maybeNewStates, someChildHasChanged)
                        where
                        (newChildren, maybeNewStates, someChildHasChanged) = 
                            processChildren prevStates $ 
                                Map.toAscList children
                    EventTODO state -> processState prevStates state
            processChildren prevStates [] = (Map.empty, Just prevStates, False)
            processChildren prevStates ((key, child) : rest) =
                 case processChildren prevStates rest of
                    (newRest, Nothing, someChildHasChanged) -> 
                        (Map.insert key child newRest, Nothing, someChildHasChanged)
                    (newRest, Just prevStatesWithRest, someChildHasChanged) ->
                        (Map.insert key newChild newRest, newStatesSoFar, someChildHasChanged || hasChanged)
                        where
                        (newChild, newStatesSoFar, hasChanged) = 
                            processNode prevStatesWithRest child

            processState prevStates state@(modeBeforeEvent, fnVecBeforeEvent) 
                -- first check whether this state is included in a previous state:
                | stateShrinksPreviousOne =
                    (EventFixedPoint state, Just prevStates, True)
--                -- the following case unnecessarily gives up in some cases:
--                | stateExpandsPreviousOne =
--                    unsafePrint
--                    (
--                        "solveHybridIVP_UsingPicardAndEventTree: processState: stateExpandsPreviousOne"
--                        ++ "\n  state = " ++ show state
--                        ++ "\n  previousStates =\n" ++ unlines (map show previousStates)
--                    ) $
--                    (EventGivenUp, Nothing, True)
                | otherwise =
                -- find which events are not ruled out by fnVec and try to determine whether an event is certain
                -- for each potential event, compute an enclosure for the state at the event
                --    then enclosure for the state after that event
                -- build the event info
                -- at the same time sum up the overall events 
                --  (how? need breadth-first with size cut off - need to have partial event info to hold intermediate results;
                --   could use a zipper...)
                    (constructorForNextEvents state eventTasksMap, maybeNewStates, True)
                where
                stateRange = (modeBeforeEvent, getRangeVec fnVecBeforeEvent)
                stateShrinksPreviousOne =
                    stateMapIncludedIn2 effInclDom stateRange prevStates
                maybeNewStates
                    | givenUp2 = Nothing
                    | otherwise = Just $ stateMapAdd stateRange prevStates
                eventList = 
                    case eventExaminationResult of
                        LDResSome _ _ eventSet -> Set.toList eventSet
                        LDResNone -> []
                someEventCertain = isLDResSure eventExaminationResult
                eventExaminationResult =
                    detectEventsWithoutLocalisation 
                        effEval eventSpecMap invariantBeforeEvent (tStart, tEnd) fnVecBeforeEvent
                eventSpecMap = 
                    hybsys_eventSpecification hybsys modeBeforeEvent
                Just invariantBeforeEvent = Map.lookup modeBeforeEvent modeInvariants  

                constructorForNextEvents 
                    | someEventCertain = EventNextSure
                    | otherwise = EventNextMaybe
                
                eventTasksMap =
                    Map.fromAscList eventTasks
                givenUp2 = 
                    or $ map eventInfoIsGivenUp (map snd eventTasks)
                eventTasks =
                    map simulateEvent $ eventKindAffectCompsAndPruneList
                simulateEvent (eventKind, affectedComponents, pruneUsingTheGuard) =
                    case maybeFnVecAfterEventUseBox of
                        (Just fnVecAfterEvent, _) ->
                            (eventKind, EventTODO (modeAfterEvent, fnVecAfterEvent))
                        (Nothing, Just givenUpOrInconsistent) ->
                            (eventKind, givenUpOrInconsistent)
                    where
                    (modeAfterEvent, eventResetMap, _, _) =
                        case Map.lookup eventKind eventSpecMap of
                            Just res -> res
                            Nothing -> error $ "aern-ivp: hybrid system has no information about event kind " ++ show eventKind
                    maybeFnVecAtEventAfterReset =
                        wrapFnVecAsBox effEval pruneAndReset fnVecBeforeEvent
                        where
                        pruneAndReset valVec =
                            do 
                            valVec1 <- pruneUsingTheGuard tDom valVec
                            valVec2 <- invariantBeforeEvent valVec1
                            return $ eventResetMap valVec2 
--                    maybeFnVecAfterEventUseVT = 
--                        fmap (map $ removeAllVarsButT . fst) $
--                        fmap (!! m) $
--                        solveUncertainValueUncertainTime
--                                sizeLimits effCompose effInteg effInclFn effAddFn effAddFnDom effDom
--                                    delta
--                                        t0Var $
--                                            odeivp tEnd modeAfterEvent $
--                                                makeInitialValuesFromFnVecAtEvent
--                        where
--                        makeInitialValuesFromFnVecAtEvent _sizeLimits t0Var2 _t0Domain =
--                            parametriseThickFunctions effAddFn effMultFn componentNames $
--                                map (renameVar tVar t0Var2) fnVecAtEvent
                    maybeFnVecAfterEventUseBox =
                        (maybeFnVec, maybeGivenUpOrInconsistent)
                        where
                        maybeGivenUpOrInconsistent =
                            case (maybeInitialValue_A, maybeFnVec) of
                                (Nothing, _) -> Just EventInconsistent
                                (_, Nothing) -> Just EventGivenUp
                                _ -> Nothing
                        maybeFnVec =
                            do
                            fnVecAfterEvent <- fmap (!! m) $ maybeFnVecIterationsAfterEvent
                            let cleanedFnVecAfterEvent = restoreUnaffectedComponents $ map removeAllVarsButT $ fnVecAfterEvent
                            prunedFnVecAfterEvent <- wrapFnVecAsBox effEval invariantAfterEvent cleanedFnVecAfterEvent 
                            return $ restoreUnaffectedComponents $  prunedFnVecAfterEvent
                        maybeFnVecIterationsAfterEvent =
                            do
                            initialValue_A <- maybeInitialValue_A 
                            solveODEIVPUncertainValueExactTime_PicardIterations
                                sizeLimits effCompose effInteg effInclFn effAddFn effAddFnDom effDom
                                    delta
                                        (odeivp tStart modeAfterEvent $ 
                                            makeFnVecFromInitialValues componentNames initialValue_A) 
                        maybeInitialValue_A =
                            do
                            fnVecAtEvent <- maybeFnVecAtEventAfterReset
                            fnVecAtEventIsectDom <- wrapFnVecAsBox effEval invariantAfterEvent fnVecAtEvent
                            return $ getRangeVec fnVecAtEventIsectDom 
                                    
                        Just invariantAfterEvent = Map.lookup modeAfterEvent modeInvariants  
                        restoreUnaffectedComponents fnVecAfterEvent =
                            zipWith pick affectedComponents $ zip fnVecAfterEvent fnVecBeforeEvent
                            where
                            pick True (fnAfterEvent, _fnBeforeEvent) = fnAfterEvent
                            pick False (_fnAfterEvent, fnBeforeEvent) = fnBeforeEvent  
                    
                getRangeVec fnVec = 
                    map getRange fnVec
                getRange fn =
                    evalAtPointOutEff effEval (getDomainBox fn) fn
    
                eventKindAffectCompsAndPruneList =
                    map lookupSpec eventList
                    where
                    lookupSpec eventKind =
                        case Map.lookup eventKind eventSpecMap of
                            Nothing -> error $ "event " ++ show eventKind ++ " not defined"
                            Just (_, _, affectedComps, pruneFn) -> (eventKind, affectedComps, pruneFn)

        modeInvariants = hybsys_modeInvariants hybsys

    odeivp t0End mode makeInitValueFnVec =
        ODEIVP
        {
            odeivp_description = "ODE for " ++ show mode,
            odeivp_componentNames = componentNames,
            odeivp_intersectDomain = Just,
            odeivp_field = field,
            odeivp_tVar = tVar,
            odeivp_tStart = tStart,
            odeivp_tEnd = tEnd,
            odeivp_makeInitialValueFnVec = makeInitValueFnVec,
            odeivp_t0End = t0End,
            odeivp_maybeExactValuesAtTEnd = Nothing,
            odeivp_valuePlotExtents = error "odeivp_valuePlotExtents deliberately not set",
            odeivp_enclosureRangeWidthLimit = (zero tStart) ArithInOut.<+>| (100000 :: Int) 
        }
        where
        Just field = Map.lookup mode modeFields
        modeFields = hybsys_modeFields hybsys
    
detectEventsWithoutLocalisation :: 
    (RefOrd.IntervalLike (Domain f), CanEvaluate f,
     NumOrd.PartialComparison (Domain f),
     HasZero (Domain f)) 
    =>
    EvaluationEffortIndicator f 
    ->
    Map.Map HybSysEventKind (targetMode, resetMap, [Bool], Domain f -> [Domain f] -> Maybe [Domain f]) 
    ->
    ([Domain f] -> Maybe [Domain f])
    ->
    (Domain f, Domain f) 
    -> 
    [f] 
    -> 
    LocateDipResult (Domain f) HybSysEventKind
detectEventsWithoutLocalisation effEval eventSpecMap modeInvariant (tStart,tEnd) fnVecBeforeEvent =
    examineEventsOnDom 
        eventsNotRuledOutOnDom
        invariantCertainlyViolatedOnDom
        invariantIndecisiveThroughoutDom
        (tStart,tEnd)
    where
    eventSpecList = Map.toList eventSpecMap 
    eventsNotRuledOutOnDom d =
        Set.fromList $ map fst $ filter maybeActiveOnD eventSpecList
        where
        maybeActiveOnD (_, (_, _, _, pruneUsingTheGuard)) =
            case pruneUsingTheGuard d fnVecBeforeEventOnD of
                Nothing -> False
                _ -> True
        fnVecBeforeEventOnD = map (evalAtPointOutEff effEval boxD) fnVecBeforeEvent
        boxD = fromList [(tVar, d)]
    invariantCertainlyViolatedOnDom d =
        case modeInvariant fnVecBeforeEventOnD of
            Nothing -> True
            _ -> False
        where
        fnVecBeforeEventOnD = map (evalAtPointOutEff effEval boxD) fnVecBeforeEvent
        boxD = fromList [(tVar, d)]
    invariantIndecisiveThroughoutDom _d =
        False -- TODO
    [(tVar,_)] = toAscList $ getDomainBox sampleFn
    (sampleFn : _) = fnVecBeforeEvent
            
        
wrapFnVecAsBox ::
    (CanEvaluate f, HasConstFns f) 
    =>
    EvaluationEffortIndicator f -> 
    ([Domain f] -> Maybe [Domain f]) -> 
    [f] -> 
    Maybe [f]
wrapFnVecAsBox effEval transformVec fnVec =
    do
    resultVec <- transformVec rangeVec
    return $ map (newConstFnFromSample sampleFn) $ resultVec 
    where
    rangeVec = map (evalAtPointOutEff effEval dombox) fnVec         
    dombox = getDomainBox sampleFn
    (sampleFn : _) = fnVec


type StateMap d = Map.Map HybSysMode [[d]]

stateMapEmpty :: StateMap d
stateMapEmpty = Map.empty
 
stateMapAdd :: (HybSysMode, [d]) -> StateMap d -> StateMap d
stateMapAdd (mode, box) stateMap =
    case Map.lookup mode stateMap of
        Nothing -> Map.insert mode [box] stateMap
        Just boxes -> Map.insert mode (box : boxes) stateMap 

stateMapIncludedIn ::
    (RefOrd.PartialComparison d)
    =>
    RefOrd.PartialCompareEffortIndicator d -> 
    (HybSysMode, [d]) -> StateMap d -> Bool
stateMapIncludedIn effInclFn (mode, box) stateMap =
    case Map.lookup mode stateMap of
        Nothing -> False
        Just boxes ->
            or $ map (boxIncludedIn effInclFn box) boxes
    
boxIncludedIn :: 
    RefOrd.PartialComparison d 
    =>
    RefOrd.PartialCompareEffortIndicator d -> 
    [d] -> [d] -> Bool
boxIncludedIn effIncl box1 box2 =          
    let (|<=?) = RefOrd.pLeqEff effIncl in
    and $ map (== Just True) $ zipWith (|<=?) box2 box1 

stateMapIncludedIn2 :: 
     (ArithInOut.RoundedReal d, RefOrd.IntervalLike d) 
    =>
    RefOrd.PartialCompareEffortIndicator d -> 
    (HybSysMode, [d]) -> StateMap d -> Bool
stateMapIncludedIn2 effIncl (mode, box0) stateMap =
    case Map.lookup mode stateMap of
        Nothing -> False
        Just boxes -> box0 `coveredBy` boxes
    where
    box `coveredBy` boxes
        | coveredByOne = True
        | otherwise = useIntersection boxes
        where
        coveredByOne =  
            or $ map (boxIncludedIn effIncl box) boxes
        useIntersection [] = False
        useIntersection (b : bb) =
            case splitUpBoxBy b box of
                Nothing -> useIntersection bb -- b and box are disjoint, ignore b
                Just boxFragments -> 
                    and $ map (`coveredBy` bb) boxFragments

splitUpBoxBy ::
    (ArithInOut.RoundedReal d, RefOrd.IntervalLike d)
    =>
    [d] -> [d] -> Maybe [[d]]
splitUpBoxBy b box = aux [] [] b box
    where
    aux prevFragments _prevCoords [] [] = Just prevFragments
    aux prevFragments prevCoords (bEdge : brest) (boxEdge : boxrest) = 
        case splitUpIntervalBy bEdge boxEdge of
            Nothing -> Nothing
            Just (intersectionEdge, outsideEdges) -> 
                aux (prevFragments ++ newFragments) (intersectionEdge : prevCoords) brest boxrest
                where
                newFragments = map makeFragment outsideEdges
                    where
                    makeFragment outsideEdge = reverse prevCoords ++ [outsideEdge] ++ boxrest
    aux _ _ _ _ = error "splitUpBoxBy: aux: internal error"
    splitUpIntervalBy be boxe 
        | be .<. boxe = Nothing
        | boxe .<. be = Nothing
        | boxe `inside` be = Just (boxe, [])
        | be `inside` boxe = Just (be, [boxeLbeL, beRboxeR])
        | beL `inside` boxe = Just (beLboxeR, [boxeLbeL])
        | beR `inside` boxe = Just (boxeLbeR, [beRboxeR])
        | otherwise = error "splitUpBoxBy: splitUpIntervalBy: internal error"
        where
        a1 .<. a2 = (a1 NumOrd.<? a2) == Just True
        a1 `inside` a2 = (a2 RefOrd.|<=? a1) == Just True
        (beL, beR) = RefOrd.getEndpointsOut be
        (boxeL, boxeR) = RefOrd.getEndpointsOut boxe
        boxeLbeL = RefOrd.fromEndpointsOut (boxeL, beL)
        boxeLbeR = RefOrd.fromEndpointsOut (boxeL, beR)
        beLboxeR = RefOrd.fromEndpointsOut (beL, boxeR)
        beRboxeR = RefOrd.fromEndpointsOut (beR, boxeR)
    
data EventInfo f
    = EventNextSure (HybSysMode, [f]) (Map.Map HybSysEventKind (EventInfo f)) -- at least one
    | EventNextMaybe (HybSysMode, [f]) (Map.Map HybSysEventKind (EventInfo f)) -- possibly none
    | EventFixedPoint (HybSysMode, [f]) -- reaches only states that have been considered by a parent node
    | EventInconsistent -- the mode invariant is definitely broken
    | EventGivenUp -- eg when tree has become too large or the ODE solver gave up
    | EventTODO (HybSysMode, [f]) -- to capture intermediate states during breadth-first search 
    
instance
    (Show f)
    => 
    Show (EventInfo f)
    where
    show = showEventInfo "" show
    
showEventInfo :: 
    String -> ((HybSysMode, [f]) -> String) -> EventInfo f -> String
showEventInfo prefix showState (EventTODO state) =
    prefix ++ "EventTODO " ++ showState state
showEventInfo prefix _showState EventInconsistent =
    prefix ++ "EventInconsistent"
showEventInfo prefix _showState EventGivenUp =
    prefix ++ "EventGivenUp"
showEventInfo prefix showState (EventFixedPoint state) =
    prefix ++ "EventFixedPoint " ++ showState state
showEventInfo prefix showState eventInfo =
    prefix ++ eventInfoConstrS ++ " " ++ showState state 
    ++ (showEvents $ Map.toAscList eventsMap)
    where
    (eventInfoConstrS, state, eventsMap) =
        case eventInfo of
            (EventNextMaybe state2 eventsMap2) -> ("EventNextMaybe", state2, eventsMap2)
            (EventNextSure state2 eventsMap2) -> ("EventNextSure", state2, eventsMap2)
            _ -> error ""
    showEvents [] = " []"
    showEvents events =
        "\n" ++ (List.intercalate "\n" $ map showEvent events)
    showEvent (eventKind, eventInfo2) =
        prefix ++ show eventKind ++ " ->\n" 
        ++ showEventInfo (prefix ++ "| ") showState eventInfo2
            
eventInfoIsGivenUp :: EventInfo t -> Bool
eventInfoIsGivenUp EventGivenUp = True
eventInfoIsGivenUp _ = False
    
eventInfoCollectFinalStates ::
    (Show (Domain f), Show f,
     CanEvaluate f,
     RefOrd.IntervalLike f,
     HasAntiConsistency f,
     RefOrd.IntervalLike (Domain f),
     HasAntiConsistency (Domain f))
    =>
    EvaluationEffortIndicator f
    ->
    Var f -> Domain f -> EventInfo f 
    -> 
    Maybe [HybridSystemUncertainState (Domain f)]
eventInfoCollectFinalStates effEval tVar tEnd eventInfo =
    aux eventInfo
    where
    aux (EventFixedPoint state) = Just [stateFromModeFnVec state]
    aux (EventNextSure _state furtherInfo) =
        fmap concat $ sequence $ map aux $ Map.elems furtherInfo
    aux (EventNextMaybe state furtherInfo) =
        fmap concat $ sequence $
            (Just [stateFromModeFnVec state] :
             (map aux $ Map.elems furtherInfo))
    aux _ = Nothing
    stateFromModeFnVec (mode, fnVec) =
        Map.singleton mode valueVec
        where
        valueVec =
            fst $ evalAtEndTimeVec effEval tVar tEnd fnVec

eventInfoCountNodes ::
    EventInfo f -> Int
eventInfoCountNodes eventInfo =
    aux eventInfo
    where
    aux (EventFixedPoint _) = 0 
    aux (EventNextSure _ furtherInfo) = 
        1 + (sum $ map aux $ Map.elems furtherInfo)
    aux (EventNextMaybe _ furtherInfo) =
        1 + (sum $ map aux $ Map.elems furtherInfo)
    aux _ = 0
        
eventInfoCountEvents ::
    (ArithInOut.RoundedReal d)
    =>
    d ->
    ArithInOut.RoundedRealEffortIndicator d ->
    EventInfo f ->
    d
eventInfoCountEvents sampleD effD eventInfo =
    aux eventInfo
    where
    aux (EventFixedPoint _) = nonneg 
    aux (EventNextSure _ furtherInfo) 
        | Map.null furtherInfo = error "eventInfoCountEvents: EventNextSure furtherInfo is empty"
        | otherwise = foldl1 union $ map incr $ map aux $ Map.elems furtherInfo
    aux (EventNextMaybe _ furtherInfo) =
        foldl union c0 $ map incr $ map aux $ Map.elems furtherInfo
    aux _ = nonneg
    
    incr = ArithInOut.addOutEff effAdd c1
    union = RefOrd.meetOutEff effJoinMeet
    c0 = zero sampleD
    c1 = one sampleD
    cInfty = plusInfinity sampleD
    nonneg = c0 `union` cInfty
    
    effJoinMeet =
        ArithInOut.rrEffortJoinMeet sampleD effD
    effAdd = 
        ArithInOut.fldEffortAdd sampleD $ ArithInOut.rrEffortField sampleD effD
        
--leqOverSomeT ::
--    (Show (Domain f),
--    Show f,
--    RefOrd.IntervalLike f,
--    RefOrd.IntervalLike (Domain f),
--    NumOrd.PartialComparison (Domain f),
--    HasAntiConsistency f,
--    HasAntiConsistency (Domain f),
--    CanEvaluate f) 
--    =>
--    EvaluationEffortIndicator f -> 
--    Int -> 
--    Var f 
--    -> 
--    f -> f -> Bool
--leqOverSomeT effEval n tVar f1 f2 =
--    predOverSomeT (\[e1,e2] -> (e1 <=? e2) == Just True) effEval n tVar [f1, f2]
--        
--predOverSomeT :: 
--    (Show (Domain f),
--    Show f,
--    RefOrd.IntervalLike f,
--    RefOrd.IntervalLike (Domain f),
--    NumOrd.PartialComparison (Domain f),
--    HasAntiConsistency f,
--    HasAntiConsistency (Domain f),
--    CanEvaluate f) 
--    =>
--    ([Domain f] -> Bool)
--    ->
--    EvaluationEffortIndicator f -> 
--    Int -> 
--    Var f 
--    -> 
--    [f] -> Bool
--predOverSomeT predicate effEval n tVar fs =
--    or predResults
--    where
--    predResults = map predOnSample tSamples
--        where
--        predOnSample tSample = predicate fsOnT
--            where
--            fsOnT = map onT fs
--            onT f = res
--                where
--                [res] = fst $ evalAtEndTimeVec effEval tVar tSample [f]
--    tSamples = map getTDom tSampleBoxes
--    tSampleBoxes = getNSamplesFromInsideDomainBox sampleF domboxTOnly n
--    domboxTOnly = fromList [(tVar, tDom)]
--    tDom = getTDom dombox
--    getTDom box = 
--        case lookupVar box tVar of
--            Just tDom2 -> tDom2
--            _ -> error "aern-ivp: internal error in predOverSomeT"
--    dombox = getDomainBox sampleF
--    (sampleF : _) = fs
