-------------------------------------
-- THIS DEMO IS CURRENTLY DISABLED --
-------------------------------------
{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE MultiParamTypeClasses #-}
module Main where

import Numeric.AERN.IVP.Examples.Hybrid.Simple

import Numeric.AERN.IVP.Specification.Hybrid
--import Numeric.AERN.IVP.Specification.ODE
import Numeric.AERN.IVP.Solver.Bisection
import Numeric.AERN.IVP.Solver.Events.EventTree
import Numeric.AERN.IVP.Solver.Events.Bisection
import Numeric.AERN.IVP.Solver.Events.SplitNearEvents
import Numeric.AERN.IVP.Plot.UsingFnView 
    (plotHybIVPBisectionEnclosures, plotHybIVPListEnclosures,
     PlotParams(..), readPlotParams)

import Numeric.AERN.Poly.IntPoly
import Numeric.AERN.Poly.IntPoly.Plot ()

import Numeric.AERN.RmToRn
import Numeric.AERN.RmToRn.Interval

import qualified 
       Numeric.AERN.RmToRn.Plot.FnView 
       as FV

import Numeric.AERN.Basics.Interval

import Numeric.AERN.RealArithmetic.Basis.Double ()
import qualified 
       Numeric.AERN.DoubleBasis.Interval 
       as CF
--import Numeric.AERN.RealArithmetic.Basis.MPFR
--import qualified Numeric.AERN.MPFRBasis.Interval as MI

--import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as ArithUpDn

import qualified 
       Numeric.AERN.RealArithmetic.RefinementOrderRounding 
       as ArithInOut
import Numeric.AERN.RealArithmetic.RefinementOrderRounding (dblToReal)
import Numeric.AERN.RealArithmetic.RefinementOrderRounding.Operators
import Numeric.AERN.RealArithmetic.ExactOps
--import Numeric.AERN.RealArithmetic.Measures

--import qualified Numeric.AERN.NumericOrder as NumOrd
--import Numeric.AERN.NumericOrder.OpsDefaultEffort

import qualified Numeric.AERN.RefinementOrder as RefOrd
--import Numeric.AERN.RefinementOrder.OpsDefaultEffort


import Numeric.AERN.Basics.SizeLimits
--import Numeric.AERN.Basics.ShowInternals

--import Data.List (intercalate)
import qualified Data.Set as Set
import qualified Data.Map as Map
import Data.List (isSuffixOf)

import System.IO
import System.Environment
--import System.Directory
--import System.CPUTime
--import System.Timeout

--import qualified Numeric.AERN.RmToRn.Plot.FnView as FV
--import Numeric.AERN.RmToRn.Plot.CairoDrawable

--import qualified Graphics.UI.Gtk as Gtk
--import qualified Control.Concurrent as Concurrent
--import Control.Concurrent.STM

import Numeric.AERN.Misc.Debug
_ = unsafePrint -- stop the unused warning

--import qualified Data.Map as Map
--import qualified Data.List as List

type CF = CF.DI

--type Fn = IntPoly String CF
type Fn = Interval (IntPoly String CF)

sampleCf :: CF
sampleCf = 0

samplePoly :: Fn
samplePoly = makeSampleWithVarsDoms 10 10 ["x"] [sampleCf]

main :: IO ()
main =
    do
    hSetBuffering stdout LineBuffering
    args <- getArgs
    runWithArgs args
        
usage :: IO ()
usage =
    do
--    putStrLn "Usage A: simple-events [locate|bisect] [evtree|pwl] <ivp name> <end time> <output file name>"
    putStrLn $ "Usage: simple-events <ivp name> <end time> \"<PlotArgs>\" <<output>.pdf|GUI> " 
                ++ " [locate|bisect] [evtree|pwl] <LOCmaxUnitSplitDepth> <LOCminUnitSplitDepth>"
                ++ " <maxDeg> <maxTermSize> <ODEmaxUnitSplitDepth> <ODEminUnitSplitDepth>"
    putStrLn "   PlotArgs:  example 1: PlotGraph[True, False, False](0,1,-1,1)"
    putStrLn "   PlotArgs:                      [shouldPlotVar1,...]"
    putStrLn "   PlotArgs:                                      ....(xmin, xmax, ymin, ymax)"
    putStrLn "   PlotArgs:  example 2: BWPlotGraph[True, True, False](-1,1,-1,1)"
    putStrLn "   PlotArgs:  example 3: PlotParam[True, True, False](-1,1,-1,1)"
    putStrLn "   PlotArgs:  example 4: NoPlot"

data TopLevelStrategy =
    TopLevelBisect | TopLevelLocate
    deriving (Show)

topLevelStrategyFromS :: String -> TopLevelStrategy
topLevelStrategyFromS "bisect" = TopLevelBisect
topLevelStrategyFromS "locate" = TopLevelLocate
topLevelStrategyFromS s = error $ "unknown top level strategy: " ++ s 

data BasicStepType =
    BasicStepEvTree | BasicStepPWL
    deriving (Show)

basicStepTypeFromS :: String -> BasicStepType
basicStepTypeFromS "evtree" = BasicStepEvTree 
basicStepTypeFromS "pwl" = BasicStepPWL 
basicStepTypeFromS s = error $ "unknown basic step type: " ++ s 

runWithArgs :: [String] -> IO ()
runWithArgs 
    [ivpName, tEndS, maybePlotDimensS, maybePDFfilenameS,
     topLevelStrategyS, basicStepTypeS, locMaxDepthS, locMinDepthS,
     maxDegS, maxSizeS, odeMaxDepthS, odeMinDepthS] =
    do
    putStrLn $ hybivp_description ivp
    solveEventsPrintSteps ivp 
        maybePlotDimens maybePDFfilename 
        (topLevelStrategy, basicStepType, locMaxDepth, locMinDepth) 
        (maxDeg, maxSize, odeMaxDepth, odeMinDepth)
    return ()
    where
    ivp = ivpByNameReportError ivpName tEndD samplePoly

    topLevelStrategy = topLevelStrategyFromS topLevelStrategyS
    basicStepType = basicStepTypeFromS basicStepTypeS

    maybePlotDimens = readPlotParams maybePlotDimensS :: Maybe PlotParams
    maybePDFfilename = readPDFfilename maybePDFfilenameS :: Maybe String
    maxDeg = read maxDegS :: Int
    maxSize = read maxSizeS :: Int
    odeMaxDepth = read odeMaxDepthS :: Int
    odeMinDepth = read odeMinDepthS :: Int
    locMaxDepth = read locMaxDepthS :: Int
    locMinDepth = read locMinDepthS :: Int
    tEndD = read tEndS :: Double
    readPDFfilename "GUI" = Nothing
    readPDFfilename pdfilename
        | ".pdf" `isSuffixOf` pdfilename = Just pdfilename
    readPDFfilename filename =
        error $ "Unsupported output file format: " ++ filename

runWithArgs _ = usage
    
refinesVec :: [CF] -> [CF] -> Bool
refinesVec vec1 vec2 =
    and $ zipWith refines vec1 vec2
refines :: CF -> CF -> Bool
refines a1 a2 = 
    (a2 CF.|<=? a1) == Just True
    where
--    tolerance = 2 ^^ (-50)

solveEventsPrintSteps :: 
    HybridIVP Fn -> 
    (Maybe PlotParams) ->
    (Maybe FilePath) ->
    (TopLevelStrategy, BasicStepType, Int, Int) ->
    (Int, Int, Int, Int) 
    -> 
    IO ()
solveEventsPrintSteps ivp 
        maybePlotDimens maybePDFfilename 
        (topLevelStrategy, basicStepType, locMaxDepth, locMinDepth) 
        (maxDeg, maxSize, odeMaxDepth, odeMinDepth) =
    do
    case basicStepType of
        BasicStepPWL -> error "PWL not fully implemented yet."
        BasicStepEvTree -> return ()
    putStrLn "---------------------------------------------------------"
    putStrLn "enclosure semantics based on (Konecny, Taha, Duracz 2012)" 
    putStrLn "---------------------------------------------------------"
    putStrLn $ "solving: " ++ description
    putStrLn "-------------------------------------------------"
    putStrLn $ "top level strategy: " ++ show topLevelStrategy
    putStrLn $ "basic step type: " ++ show basicStepType
    putStrLn $ "event location minimum step size = 2^{" ++ show (- locMaxDepth) ++ "}"
    putStrLn $ "event location maximum step size = 2^{" ++ show (- locMinDepth) ++ "}"
    putStrLn $ "substSplitSizeLimit = " ++ show substSplitSizeLimit
    putStrLn $ "maxdeg = " ++ show maxDeg
    putStrLn $ "maxsize = " ++ show maxSize
    putStrLn $ "delta = " ++ show delta
    putStrLn $ "m = " ++ show m
    putStrLn $ "ODE minimum step size = 2^{" ++ show (- odeMaxDepth) ++ "}"
    putStrLn $ "ODE maximum step size = 2^{" ++ show (- odeMinDepth) ++ "}"
    putStrLn $ "split improvement threshold = " ++ show splitImprovementThreshold
    case maybeExactResult of
        Just exactResult ->
            do
            putStrLn "(almost) exact result = "
            putStr $ showState "   " (tEnd, Just exactResult)
        _ -> return ()
    putStrLn "----------  steps: ---------------------------"
    case topLevelStrategy of
        TopLevelBisect -> 
            do
            _ <- printStepsInfoBisect (1:: Int) bisectionInfo
            return ()
        TopLevelLocate -> 
            do
            _ <- printStepsInfoLocate (1:: Int) segmentsInfo
            return ()
    putStrLn "----------  step summary: -----------------------"
    putStrLn $ "number of atomic segments = " ++ 
        case topLevelStrategy of
            TopLevelBisect ->
                (show $ bisectionInfoCountLeafs bisectionInfo)
            TopLevelLocate ->
                (show $ length segmentsInfo)
    putStrLn $ "smallest segment size: " ++ (show smallestSegSize) 
        
    putStrLn "----------  result: -----------------------------"
    putStr $ showState ">>> " (tEnd, maybeEndState)
    case (maybeExactResult, maybeEndState) of
        (Just exactResult, Just resultOut) ->
            putStrLn $ "error = " ++ show (getErrorState exactResult resultOut)
        _ -> return ()
    putStrLn $ "event count = " ++ show eventCount
    putStrLn "-------------------------------------------------"

    case (maybePlotDimens, topLevelStrategy) of
        (Nothing, _) -> return ()
        (Just (PlotParams rectDbl activevars shouldUseParamPlot isBW), TopLevelBisect) ->
            plotHybIVPBisectionEnclosures rect activevars isBW shouldUseParamPlot
                effCf False (2^^(-8 :: Int) :: CF) ivp bisectionInfo maybePDFfilename
            where
            rect = fmap (dblToReal 0) rectDbl :: FV.Rectangle CF
        (Just (PlotParams rectDbl activevars shouldUseParamPlot isBW), TopLevelLocate) -> 
            plotHybIVPListEnclosures rect activevars isBW shouldUseParamPlot
                effCf (2^^(-12 :: Int) :: CF) ivp segmentsInfo maybePDFfilename
            where
            rect = fmap (dblToReal 0) rectDbl :: FV.Rectangle CF
    return ()
--    return (maybeEndState, segmentsInfo)
--    return (maybeEndState, bisectionInfo)
    where
    maybeEndState = case topLevelStrategy of
        TopLevelBisect -> maybeEndStateBisect
        TopLevelLocate -> maybeEndStateLocate
    (maybeEndStateBisect, bisectionInfo) =
        solveHybridIVPBisect
            sizeLimits effCf substSplitSizeLimit
                delta m locMinStepSize locMaxStepSize splitImprovementThreshold
                    "t0" 
                        ivp
    (maybeEndStateLocate, segmentsInfo) =
        solveHybridIVPLocate
            sizeLimits effCf substSplitSizeLimit
                locMinStepSize locMaxStepSize maxNodes
                delta m odeMinStepSize odeMaxStepSize splitImprovementThreshold
                    "t0" 
                        ivp
    -- parameters:
    delta = 1
    m = 200 -- max number of Picard iterations
--    m = 20
    maxNodes = 100 -- max event tree size
    substSplitSizeLimit = 10
--    substSplitSizeLimit = maxSplitSizeParam -- 2^t0maxdeg

    locMinStepSize = 2^^(-locMaxDepth)
    locMaxStepSize = 2^^(-locMinDepth)
    odeMinStepSize = 2^^(-odeMaxDepth)
    odeMaxStepSize = 2^^(-odeMinDepth)
    
--        fst $ RefOrd.getEndpointsOut $ 10^^(-3::Int)
    splitImprovementThreshold = 2^^(-48 :: Int)
    
    -- auxiliary:
    description = hybivp_description ivp
    tStart = hybivp_tStart ivp
    tEnd = hybivp_tEnd ivp
    maybeExactResult = hybivp_maybeExactStateAtTEnd ivp
    hybsys = hybivp_system ivp
    componentNames = hybsys_componentNames hybsys

    effCf = ArithInOut.roundedRealDefaultEffort sampleCf
    effJoinCf = RefOrd.joinmeetDefaultEffort sampleCf
    sizeLimits =
        getSizeLimits $
            makeSampleWithVarsDoms maxDeg maxSize [] []
            
    getErrorState exactState approxState
        | not (exactModeSet `Set.isSubsetOf` approxModeSet) =
            error $ 
                "enclosure error:"
                ++ "\n approxModeSet = " ++ show approxModeSet
                ++ "\n exactModeSet = " ++ show exactModeSet
        | not (refinesVec exactVec approxVec) = -- && refinesVec vecIn vecExact) ->
            error $ 
                "enclosure error:"
                ++ "\n approxVec = " ++ show approxVec
                ++ "\n exactVec = " ++ show exactVec
        | otherwise =
            map getError $ zip exactVec approxVec
        where
        (exactModeSet, exactVec) = getHybridStateUnion effJoinCf exactState
        (approxModeSet, approxVec) = getHybridStateUnion effJoinCf approxState
        getError (valueIn, valueOut) =
            err
            where
            err = snd $ RefOrd.getEndpointsOut $ wOut CF.<-> wIn
            wOut = CF.width valueOut     
            wIn = CF.width valueIn     
    
    eventCount = case topLevelStrategy of
        TopLevelBisect -> eventCountBisect
        TopLevelLocate -> eventCountLocate
    smallestSegSize = case topLevelStrategy of
        TopLevelBisect -> smallestSegSizeBisect  
        TopLevelLocate -> smallestSegSizeLocate  
    eventCountBisect =
        aux bisectionInfo
        where
        aux (BisectionNoSplit (_,_,modeEventInfoList)) 
            | null modeEventInfoList = error "solveEventsPrintSteps: BisectionNoSplit with empty modeEventInfoList" 
            | otherwise = foldl1 (CF.</\>) $ map (eventInfoCountEvents 0 effCf . snd) modeEventInfoList
        aux (BisectionSplit _ left Nothing) =
            aux left
        aux (BisectionSplit _ left (Just right)) =
            (aux left) CF.<+> (aux right)
    (smallestSegSizeBisect, _) =
        aux tStart (tEnd CF.<-> tStart) bisectionInfo
        where
        aux tPrev tSmallestSoFar (BisectionNoSplit (tNow,_,_)) =
            (CF.minOut tSmallestSoFar (tNow CF.<-> tPrev), tNow)
        aux tPrev tSmallestSoFar (BisectionSplit _ left Nothing) =
            aux tPrev tSmallestSoFar left
        aux tPrev tSmallestSoFar (BisectionSplit _ left (Just right)) =
            aux tPrevL tSmallestSoFarL right
            where
            (tSmallestSoFarL, tPrevL) =
                aux tPrev tSmallestSoFar left
    eventCountLocate =
        foldl (<+>) 0 $ map getSegmentEventCount segmentsInfo
        where
        getSegmentEventCount (_, Nothing, _) =  -- something failed on this segment 
            0 CF.</\> (plusInfinity 0)
        getSegmentEventCount (_, Just _, modeMap) = 
            foldl1 (CF.</\>) $ map getModeEventCount $ Map.elems modeMap
        getModeEventCount (_, _, Nothing) = 0 -- no events here
        getModeEventCount (_, _, Just (_, _, eventInfo)) = 
            eventInfoCountEvents 0 effCf eventInfo
    smallestSegSizeLocate =
        aux tStart (tEnd CF.<-> tStart) segmentsInfo
        where
        aux _tPrev tSmallestSoFar [] = tSmallestSoFar
        aux tPrev tSmallestSoFar ((tNow,_,_) : rest) =
            aux tNow tSmallestUpdated rest
            where
            tSmallestUpdated =
                CF.minOut tSmallestSoFar (tNow CF.<-> tPrev)
            
    showStepInfo (n, t) =
        "step " ++ show n ++ ": t = " ++ show t
    printStepsInfoBisect n (BisectionNoSplit (t, _maybeState, _modeEventInfoList)) =
        do
        putStrLn $ showStepInfo (n, t)
        return $ n + 1
    printStepsInfoBisect n (BisectionSplit _ left maybeRight) =
        do
        n2 <- printStepsInfoBisect n left
        case maybeRight of
            Just right -> printStepsInfoBisect n2 right
            Nothing -> return $ n2 + 1
    printStepsInfoLocate _ [] = return ()
    printStepsInfoLocate n ((t,_,_) : rest) =
        do
        putStrLn $ showStepInfo (n, t)
        printStepsInfoLocate (n+1) rest
        
    showState indent (t, maybeState) =
        indent ++ "mode(" ++ show t ++ ") ∊ " ++ modesS ++ "\n" ++
        (unlines $ map (showComponent indent) $ zip componentNames valueSs)
        where
        showComponent indent2 (name, valueS) =
            indent2 ++ name ++ "("  ++ show t ++ ") ∊ " ++ valueS
        (modesS, valueSs) =
            case maybeState of
                Just state ->
                    (show modeSet, map showValue values)
                    where
                    (modeSet, values) = getHybridStateUnion effJoinCf state
                _ ->
                    ("<no result computed>", 
                     replicate (length componentNames) "<no result computed>")
        showValue valueOut =
            show valueOut 
--    showSegInfoBisect indent (t, maybeState, modeEventInfoList) =
--        maybeEventsCountS ++
--        indent ++ "mode(" ++ show t ++ ") ∊ " ++ modesS ++ "\n" ++
--        (unlines $ map (showComponent indent) $ zip componentNames valueSs)
--        ++ (unlines $ map showModeEventInfo modeEventInfoList)
--        where
--        maybeEventsCountS =
--            case modeEventInfoList of
--                [] -> ""
--                _ ->  indent ++ "events on this time segment: " ++ eventsS ++ "\n" 
--        eventsS =
--            show $
--            foldl1 (CF.</\>) $ map (eventInfoCountEvents 0 effCf . snd) modeEventInfoList 
--        showModeEventInfo (mode, eventInfo) =
--            indent ++ "events assuming mode at the start of segment = " ++ show mode ++ ":\n" ++
--            showEventInfo (indent ++ "  ") (show . fst) eventInfo
--        showComponent indent2 (name, valueS) =
--            indent2 ++ name ++ "("  ++ show t ++ ") ∊ " ++ valueS
--        (modesS, valueSs) =
--            case maybeState of
--                Just state ->
--                    (show modeSet, map showValue values)
--                    where
--                    (modeSet, values) = getHybridStateUnion effJoinCf state
--                _ ->
--                    ("<no result computed>", 
--                     replicate (length componentNames) "<no result computed>")
--        showValue valueOut =
--            show valueOut 
----            ++ "(err<=" ++ show err ++ ")"
----            ++ "; valueIn = " ++ show valueIn
----            where
----            err = snd $ RefOrd.getEndpointsOut $ wOut CF.<-> wIn
----            wOut = CF.width valueOut     
----            wIn = CF.width valueIn     
--    showSegInfoLocate indent (t, maybeState, modeSolvingInfoMap) =
--        maybeEventsCountS
--        ++ (showState indent (t, maybeState))
----        ++ (unlines $ map showModeEventInfo modeEventInfoList)
--        where
--        eventInfoList = map getEventInfo $ filter hasEventInfo $ Map.elems modeSolvingInfoMap
--        hasEventInfo (_, _, Nothing) = False
--        hasEventInfo _ = True
--        getEventInfo (_, _, Just eventInfo) = eventInfo
--        maybeEventsCountS 
--            | null eventInfoList = ""
--            | otherwise =
--                indent ++ "events on this time segment: " ++ eventsS ++ "\n" 
--        eventsS =
--            show $
--            foldl1 (CF.</\>) $ map (eventInfoCountEvents 0 effCf) eventInfoList 
----        showModeEventInfo (mode, eventInfo) =
----            indent ++ "events assuming mode at the start of segment = " ++ show mode ++ ":\n" ++
----            showEventInfo (indent ++ "  ") (show . fst) eventInfo

solveHybridIVPBisect ::
    (solvingInfo ~ (CF, Maybe (HybridSystemUncertainState CF), [(HybSysMode, EventInfo Fn)]))
    =>
    SizeLimits Fn -> 
    ArithInOut.RoundedRealEffortIndicator CF ->
    Int -> 
    CF ->
    Int ->
    CF ->
    CF ->
    CF ->
    Var Fn ->
    HybridIVP Fn 
    ->
    (
     Maybe (HybridSystemUncertainState CF)
    ,
     BisectionInfo solvingInfo (solvingInfo, Maybe CF)
    )
solveHybridIVPBisect 
        sizeLimits effCf substSplitSizeLimit
            delta m minStepSize maxStepSize splitImprovementThreshold 
                t0Var
                    hybivp
    =
    result
    where
    result =
        solveHybridIVP_UsingPicardAndEventTree_Bisect
            sizeLimits effPEval effCompose effEval effInteg effInclFn effAddFn effMultFn effAddFnDom effCf
                delta m t0Var minStepSize maxStepSize splitImprovementThreshold
                    hybivp

    effCompose = effIP
    effEval = (effIP, ())
    effPEval = (effIP, ())
    effInteg = (effIP, ())
    effAddFn = effIP
    effMultFn = effIP
    effAddFnDom = effIP
    effInclFn = effIP
    effIP = 
        (ipolylimits_effort sizeLimits)
        {
            ipolyeff_cfRoundedRealEffort = effCf,
            ipolyeff_evalMaxSplitSize = substSplitSizeLimit
        }

solveHybridIVPLocate ::
    (
     solvingInfoODESegment ~ (Maybe ([Fn],[Fn]), (CF, Maybe [CF])),
     solvingInfoODE ~ BisectionInfo solvingInfoODESegment (solvingInfoODESegment, Maybe CF),
     solvingInfoEvents ~ (CF, Maybe (HybridSystemUncertainState CF), EventInfo Fn)
    )
    =>
    SizeLimits Fn -> 
    ArithInOut.RoundedRealEffortIndicator CF ->
    Int -> 
    CF ->
    CF ->
    Int ->
    CF ->
    Int ->
    CF ->
    CF ->
    CF ->
    Var Fn ->
    HybridIVP Fn 
    ->
    (
        Maybe (HybridSystemUncertainState CF)
    ,
        [(
            CF
            -- end time of this segment (including the event resolution sub-segment)  
         ,
            Maybe (HybridSystemUncertainState CF)
         ,
            Map.Map HybSysMode 
                (
                    solvingInfoODE,
                    Maybe (HybridSystemUncertainState CF),
                    Maybe solvingInfoEvents
                )
         )
        ]
    )
solveHybridIVPLocate
        sizeLimits effCf substSplitSizeLimit
            locMinStepSize locMaxStepSize maxNodes
            delta m odeMinStepSize odeMaxStepSize splitImprovementThreshold
                t0Var
                    hybivp
    =
    result
    where
    result =
        solveHybridIVP_UsingPicardAndEventTree_SplitNearEvents
            sizeLimits effSizeLims effPEval effCompose effEval effInteg effDeriv effInclFn 
                effAddFn effMultFn effAbsFn effMinmaxFn 
                effDivFnInt effAddFnDom effMultFnDom effCf
                locMinStepSize locMaxStepSize maxNodes
                delta m t0Var odeMinStepSize odeMaxStepSize splitImprovementThreshold
                    hybivp

    effSizeLims = effIP
    effDeriv = effIP
    effAbsFn = effIP
    effMinmaxFn = effIP

    effCompose = effIP
    effEval = effIP
    effPEval = effIP
    effInteg = effIP
    effAddFn = effIP
    effMultFn = effIP

    effAddFnDom = effIP
    effMultFnDom = effIP
    effDivFnInt = effIP
    effInclFn = effIP
    effIP = 
        (ipolylimits_effort sizeLimits)
        {
            ipolyeff_cfRoundedRealEffort = effCf,
            ipolyeff_evalMaxSplitSize = substSplitSizeLimit
        }



makeSampleWithVarsDoms :: 
     Int -> Int -> [Var Fn] -> [CF] -> Fn
makeSampleWithVarsDoms maxdeg maxsize vars doms =
    newConstFn sizeLimits varDoms sampleCf
    where
    varDoms = zip vars doms 
    sizeLimits =
        (defaultIntPolySizeLimits sampleCf cf_limits arity)
        {
            ipolylimits_maxdeg = maxdeg,
            ipolylimits_maxsize = maxsize
        }
        where
        arity = length vars
        cf_limits = defaultSizeLimits sampleCf
     
    