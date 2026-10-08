{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE ScopedTypeVariables #-}
{-|
    Module      :  Numeric.AERN.IVP.Plot.UsingFnView
    Description :  visualising IVP solutions using FnView 
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Visualising IVP solutions using FnView.
-}

module Numeric.AERN.IVP.Plot.UsingFnView 
(
    plotODEIVPBisectionEnclosures,
    plotHybIVPBisectionEnclosures,
    plotHybIVPListEnclosures,
    plotArgsHelpLines,
    IVPPlotArgs(..),
    readIVPPlotArgs
)
where

import Numeric.AERN.IVP.Solver.Bisection
import Numeric.AERN.IVP.Solver.Events.EventTree
import Numeric.AERN.IVP.Specification.ODE
import Numeric.AERN.IVP.Specification.Hybrid

import Numeric.AERN.RmToRn.New
import Numeric.AERN.RmToRn.Domain
import Numeric.AERN.RmToRn.Evaluation

import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut
--import Numeric.AERN.RealArithmetic.RefinementOrderRounding (dblToReal)
import Numeric.AERN.RealArithmetic.RefinementOrderRounding.Operators

--import Numeric.AERN.RealArithmetic.ExactOps

import qualified Numeric.AERN.NumericOrder as NumOrd
import Numeric.AERN.NumericOrder.Operators

import qualified Numeric.AERN.RefinementOrder as RefOrd
import Numeric.AERN.RefinementOrder.Operators

import Numeric.AERN.Basics.SizeLimits

import qualified Numeric.AERN.RmToRn.Plot.FnView as FV
import Numeric.AERN.RmToRn.Plot.CairoDrawable

import qualified Graphics.UI.Gtk as Gtk
import Control.Concurrent.STM

import qualified Data.Map as Map
import qualified Data.List as List
import Data.List (isPrefixOf)

import Debug.Trace
_ = trace

plotArgsHelpLines :: [String]
plotArgsHelpLines =
    [
        "   PlotArgs:  example 1: PlotGraph[True, False, False](0,1,-1,1)",
        "   PlotArgs:                      [shouldPlotVar1,...] - fill with False when not specified",
        "   PlotArgs:                                          (xmin, xmax, ymin, ymax)",
        "   PlotArgs:  example 2: BWPlotGraph[True, True, False](-1,1,-1,1)",
        "                         BW = black & white",
        "   PlotArgs:  example 3: PlotParam(10,[False, False, True])[True, True, False](-1,1,-1,1)",
        "                                   10 samples per time unit",
        "                                      [isIndependetVar1, ...] - fill with False when not specified",
        "   PlotArgs:  example 4: NoPlot"
    ]

data IVPPlotArgs =
    IVPPlotArgs
    {
        plotp_rect :: FV.Rectangle Double,
        plotp_activevars :: [Bool],
        plotp_isParam :: Maybe (Int, [Bool]), -- sampling frequency for parametric plot
        plotp_isBW :: Bool
    }
    deriving (Show)
    
readIVPPlotArgs ::
    String -> Maybe IVPPlotArgs
readIVPPlotArgs sOrig
    | "Plot" `isPrefixOf` sOrig = readPlotType False $ drop (length "Plot") sOrig
    | "BWPlot" `isPrefixOf` sOrig = readPlotType True $ drop (length "BWPlot") sOrig
    | sOrig == "NoPlot" = Nothing
    | otherwise = errorP
    where
    readPlotType isBW s 
        | "Graph" `isPrefixOf` s = 
            readActiveVars Nothing $ drop (length "Graph") s
        | "Param" `isPrefixOf` s =
            readParamPlotArgs $ drop (length "Param") s 
        | otherwise = errorP
        where
        readActiveVars maybeParamPlotArgs s2 = 
            case reads s2 of
                (activeVars, sRest) : _ ->
                    case reads sRest of
                        (boundsD, []) : _ -> 
                            Just (IVPPlotArgs (boundsFromDoubles boundsD) activeVars maybeParamPlotArgs isBW)
                        _ -> errorP
                _ -> errorP
        readParamPlotArgs ('(' : s2) =
            case reads s2 of
                (samplingFreq, ',' : sRest) : _ ->
                    readScanVars samplingFreq sRest
                _ -> errorP
        readParamPlotArgs _ = errorP
        readScanVars samplingFreq s2 =
            case List.findIndex (== ')') s2 of
                Just i2 ->
                    case reads $ (take i2 s2) of
                        (scanVars, []) : _ -> 
                            readActiveVars (Just (samplingFreq, scanVars)) (drop (i2+1) s2)
                        _ -> errorP
                _ -> errorP
            
    errorP = error $ "Cannot parse plot specification: " ++ sOrig
    boundsFromDoubles (xmin, xmax, ymin, ymax) =
        FV.Rectangle ymax ymin xmin xmax

plotODEIVPBisectionEnclosures :: 
    (Var f ~ String,
     CanEvaluate f,
     CairoDrawableFn f,
     HasSizeLimits f,
     RefOrd.RoundedLattice f,
     HasConstFns f,
     HasDomainBox f,
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     Num (Domain f),
     Show f, Show (Var f), Show (Domain f)
    ) 
    =>
    FV.Rectangle (Domain f) -- ^ initial canvas viewport
    -> [Bool] -- ^ for each variable, whether it should be plotted
    -> Bool -- ^ True -> plot all components in black 
    -> Maybe (Int, [Bool]) 
        -- ^ Just (samplingFreq, scanVars) -> use parametric plot (there have to be exactly two active functions)
        -- ^   scanVars : variables with True are substituted by their domains, otherwise they are sampled  
    -> ArithInOut.RoundedRealEffortIndicator (Domain f)
    -> Domain f
    -> ODEIVP f
    -> BisectionInfo (Maybe ([f],[f]), t) splitReason
    -> Maybe FilePath
    -> IO ()
plotODEIVPBisectionEnclosures 
        rect activevarsPre isBW maybeParamPlotArgs effCF plotMinSegSize 
        (ivp :: ODEIVP f) bisectionInfo 
        maybePDFFilename =
--    trace ("plotODEIVPBisectionEnclosures: maybeParamPlotArgs = " ++  show maybeParamPlotArgs) $
    case maybePDFFilename of
        Nothing ->
            do
            _ <- Gtk.unsafeInitGUIForThreadedRTS
            fnDataTV <- atomically $ newTVar $ FV.FnData $ fnsPlotSpec
            fnMetaTV <- atomically $ newTVar $ fnmeta
            _ <- FV.new sampleFn effDrawFn effCF effEval (fnDataTV, fnMetaTV) Nothing
            Gtk.mainGUI
        Just pdffilename ->
            do
            FV.plotToPDFFile sampleFn effDrawFn effCF canvasParams 512 512 fnsActive fnsPlotSpec fnsStyles pdffilename
            where
            fnsStyles = (map $ const black) $ concat $ FV.dataFnStyles fnmeta
            canvasParams = FV.dataDefaultCanvasParams fnmeta
            fnsActive = concat $ FV.dataDefaultActiveFns fnmeta
    where
    shouldUseParamPlot = case maybeParamPlotArgs of Just _ -> True; _ -> False
    
    fnsPlotSpec = addPlotVar fns
    
    effDrawFn = cairoDrawFnDefaultEffort sampleFn
    effEval = evaluationDefaultEffort sampleFn
    (((sampleFn : _) : _) : _) = fns
--    sampleCf = getSampleDomValue sampleFn
    
    componentNames = odeivp_componentNames ivp
    n = length componentNames
--    tStart = odeivp_tStart ivp
--    tEnd = odeivp_tEnd ivp
    tVar = odeivp_tVar ivp

    -- complete the activevarsPre list with False's to be the same length as componentNames:
    activevars =
        take n $ activevarsPre ++ (repeat False)

    activevarNames = pickByActivevars componentNames 

    -- function to pick from a list of length componentNames those elements that correspond to active vars:
    pickByActivevars :: [a] -> [a]
    pickByActivevars list =
        map snd $ filter fst $ zip activevars list
    -- function to pick from a list of length componentNames those elements that correspond to active vars:
    pickByActivevarsCycle :: [a] -> [[a]]
    pickByActivevarsCycle [] = []
    pickByActivevarsCycle list =
        pickByActivevars batch : pickByActivevarsCycle rest
        where
        (batch, rest) = splitAt n list
--    pickByActivevarsCycle _ = error $ "plotODEIVPBisectionEnclosures: pickByActivevarsCycle: list not divisible by n"
    
    addPlotVar fns2 =
        case maybeParamPlotArgs of
            Just (_, scanVars) ->
                map (map (addVParam scanVarsList) . pickByActivevarsCycle) fns2
                where
                scanVarsList = 
                    map snd $ filter (not . fst) $ zip (scanVars ++ repeat False) componentNames
            _ ->
                map (map addV) fns2
        where
        addV fs = (FV.GraphPlotFn (fs :: [f]), tVar)
        addVParam scanVarsList [fsX, fsY] = (FV.ParamPlotFns (zip fsX fsY) scanVarsList, tVar)
        addVParam scanVarsList _ = errorParamFnCount 
        
    errorParamFnCount =
            error 
            "plotODEIVPBisectionEnclosures: In a parameteric plot there have to be exactly two active functions."
            
    fns = [List.transpose fns3]
    segNames = ["variables"]
    fnNamesPre
        | shouldUseParamPlot = [[concat $ pickByActivevars componentNames]]
        | otherwise = [componentNames]
            
    (fns3, _fnNamesPre3, _segNames3) 
        | shouldUseParamPlot =
            (fns2, fnNamesPre2, segNames2)
        | otherwise = 
            aggregateSequencesOfTinySegments2 fnsAndNames
        where
        (fns2, fnNamesPre2) = unzip $ (map unzip fnsAndNames)
        segNames2 = map snd $ zip fnNamesPre2 $ ["segment " ++ show i | i <- [1..] :: [Int]]
    fnNames
        | shouldUseParamPlot = 
            fnNamesActiveJoined
        | otherwise = 
            fnNamesPre 
        where
        fnNamesActiveJoined =
            map ((map (const joinedActiveVarsName)) . pickByActivevarsCycle) fnNamesPre
            where
            joinedActiveVarsName = "(" ++ List.intercalate "," activevarNames ++ ")"
        
    fnmeta =
        FV.simpleFnMetaData
            sampleFn
            rect
            (Just (1,1,1,1))
            (case maybeParamPlotArgs of Just (freq, _) -> freq; _ -> 200)
            tVar
            (zip segNames $ map addMetaToFnNames fnNames)
        where
        addMetaToFnNames names =
            zip3 names colourList enabledList
        colourList 
            | isBW =
                repeat black
            | otherwise = 
                cycle $ take n $ cycle [blue, green, red, black, orange, purple, magenta]
        enabledList 
            | shouldUseParamPlot = repeat True
            | otherwise = cycle activevars


--    fns = 
    fnsAndNames = 
        map getFnsFromSegInfo $
            bisectionInfoGetLeafSegInfoSequence bisectionInfo
        where
        getFnsFromSegInfo (Just (fnVec, _), _) 
--            | notTooThick 
                =
                zip fnVec componentNames
            where
--            notTooThick =
--                True 
--                and $ map withdOK fnVec
--            withdOK fn =
--                ((ArithInOut.absOut val) <? (one sampleCf)) == (Just True) 
--                where
--                val = zero sampleCf 
----                    evalAtPointOutEff effEval domPt fn
----                domPt = getSampleFromInsideDomainBox fn domBox
----                domBox = getDomainBox fn 
        getFnsFromSegInfo _ = []
    aggregateSequencesOfTinySegments2 fnsAndNames2 = 
        aggrNewSegm [] [] [] $ zip ([1..]::[Int]) fnsAndNames2
        where
        aggrNewSegm 
                prevFns prevFnNames prevSegNames 
                segs@((segNoFirstSeg, fnsNamesFirstSeg@((fn1FirstSeg,_) : _)) : restSegs)
            | noAggregation =
                aggrNewSegm 
                    (fnsFirstSeg : prevFns) 
                    (fnNamesFirstSeg : prevFnNames) 
                    (("segment " ++ show segNoFirstSeg) : prevSegNames) 
                    restSegs 
            | otherwise =
                aggrNewSegm
                    (fnsAggregated : prevFns) 
                    (fnNamesAggregated : prevFnNames) 
                    (("segments " ++ show segNoFirstSeg ++ "-" ++ show segNoLastAggrSeg) : prevSegNames) 
                    restSegsAfterAggr
            where
            noAggregation = length smallSegmentsToAggregate <= 1
            (smallSegmentsToAggregate, restSegsAfterAggr) = 
                span segEndsBeforeLimit segs
                where
                segEndsBeforeLimit (_, ((fn1ThisSeg,_) : _)) =
                    (tEndThisSeg <=? tAggrLimit) == Just True
                    where
                    (_, tEndThisSeg) = getTVarDomEndpoints fn1ThisSeg
                    tAggrLimit = tStartFirstSeg <+> plotMinSegSize
                segEndsBeforeLimit _ = False
            fnNamesAggregated =
                map (++ "(aggr)") componentNames
            fnsAggregated =
                foldl1 (zipWith (</\>)) $
                    chunksOf (length componentNames) $
                        map makeConstFnOverAggrDom $
                            concat $ map getFnsFromSeg smallSegmentsToAggregate
                where
                chunksOf _ [] = []
                chunksOf n list = firstN : (chunksOf n rest)
                    where
                    (firstN, rest) = splitAt n list
                getFnsFromSeg (_, fnsNames) = map fst fnsNames
                makeConstFnOverAggrDom fn =
                    newConstFn sizeLimits [(tVar, aggrDom)] range
                    where
                    range = evalAtPointOutEff effEval dombox fn
                    sizeLimits = getSizeLimits fn         
                    dombox = getDomainBox fn
                    
            aggrDom = RefOrd.fromEndpointsOut (tStartFirstSeg, tEndLastAggrSeg) 
            (tStartFirstSeg, _) = getTVarDomEndpoints fn1FirstSeg
            (_, tEndLastAggrSeg) = getTVarDomEndpoints fn1LastAggrSeg
            (segNoLastAggrSeg, ((fn1LastAggrSeg,_) : _)) = last smallSegmentsToAggregate 
            getTVarDomEndpoints fn =
                case lookupVar (getDomainBox fn) tVar of 
                    Just tDom -> RefOrd.getEndpointsOut tDom 
            (fnsFirstSeg, fnNamesFirstSeg) = unzip fnsNamesFirstSeg
        aggrNewSegm prevFns prevFnNames prevSegNames _ =
            (reverse prevFns, reverse prevFnNames, reverse prevSegNames)

plotHybIVPBisectionEnclosures :: 
    (Var f ~ String,
     CanEvaluate f,
     CairoDrawableFn f,
     HasSizeLimits f,
     RefOrd.RoundedLattice f,
     HasConstFns f,
     HasDomainBox f,
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     Show f, Show (Var f), Show (Domain f)
    ) 
    =>
    FV.Rectangle (Domain f) -- ^ initial canvas viewport
    -> [Bool] -- ^ for each variable, whether it should be plotted
    -> Bool -- ^ True -> plot all components in black 
    -> Maybe (Int, [Bool]) 
        -- ^ Just (samplingFreq, scanVars) -> use parametric plot (there have to be exactly two active functions)
        -- ^   scanVars : variables with True are substituted by their domains, otherwise they are sampled  
    ->
    ArithInOut.RoundedRealEffortIndicator (Domain f)
    -> Bool 
    -> Domain f
    -> HybridIVP f
    -> BisectionInfo (t, t1, [(HybSysMode, EventInfo f)]) splitReason
    -> Maybe FilePath
    -> IO ()
plotHybIVPBisectionEnclosures 
        rect activevarsPre isBW maybeParamPlotArgs effCF 
        shouldShowEventTreeEnclosures plotMinSegSize ivp bisectionInfo 
        maybePDFFilename =
    case maybePDFFilename of
        Nothing ->
            do
            _ <- Gtk.unsafeInitGUIForThreadedRTS
            fnDataTV <- atomically $ newTVar $ FV.FnData $ fnsPlotSpec
            fnMetaTV <- atomically $ newTVar $ fnmeta
            _ <- FV.new sampleFn effDrawFn effCF effEval (fnDataTV, fnMetaTV) Nothing
            Gtk.mainGUI
        Just pdffilename ->
            do
            FV.plotToPDFFile sampleFn effDrawFn effCF canvasParams 512 512 fnsActive fnsPlotSpec fnsStyles pdffilename
            where
            fnsStyles = (map $ const black) $ concat $ FV.dataFnStyles fnmeta
            canvasParams = FV.dataDefaultCanvasParams fnmeta
            fnsActive = concat $ FV.dataDefaultActiveFns fnmeta
    where
    shouldUseParamPlot = case maybeParamPlotArgs of Just _ -> True; _ -> False

    fnsPlotSpec = addPlotVar fns
    
    effDrawFn = cairoDrawFnDefaultEffort sampleFn
    effEval = evaluationDefaultEffort sampleFn
    (((sampleFn : _) : _) : _) = fns 
--    sampleCf = getSampleDomValue sampleFn
    
    componentNames = hybsys_componentNames $ hybivp_system ivp
    n = length componentNames
--    tStart = hybivp_tStart ivp
--    tEnd = hybivp_tEnd ivp
    tVar = hybivp_tVar ivp
    
    activevars =
        take n $ activevarsPre ++ (repeat False)

    -- function to pick from a list of length componentNames those elements that correspond to active vars:
    pickByActivevars :: [a] -> [a]
    pickByActivevars list =
        map snd $ filter fst $ zip activevars list

    addPlotVar = map $ map addV
        where
        addV fs = (FV.GraphPlotFn fs, tVar)
        
    fns = [List.transpose $ splitUpGroups fns3]
        where
        splitUpGroups [] = []
        splitUpGroups ([] : gs) = splitUpGroups gs
        splitUpGroups (g : gs) =
            (take n g) : (splitUpGroups ((drop n g) : gs)) 
    segNames = ["variables"]
    fnNames 
        | shouldUseParamPlot = [[concat $ pickByActivevars componentNames]]
        | otherwise = [componentNames]
        
    (fns3, _fnNames3, _segNames3) = 
        aggregateSequencesOfTinySegments effEval componentNames tVar plotMinSegSize fnsAndNames 
    fnsAndNames =
        concat $ map maybeAggregateSegment fnAndNameVectors
        where
        maybeAggregateSegment fnAndNameVectorsInSegment
            | shouldShowEventTreeEnclosures = fnAndNameVectorsInSegment
            | otherwise = 
                case fnAndNameVectorsInSegment of
                    (_ : _ : _) -> [numberFnVec unionAllVectors ""]
                    _ -> fnAndNameVectorsInSegment
            where
            unionAllVectors =
--                unsafePrint (
--                    "unionAllVectors:"
--                    ++ "\n nameVectorsInSegment = " ++ show nameVectorsInSegment 
--                    ++ "\n fnVectorsInSegment = " ++ show fnVectorsInSegment 
--                    ++ "\n rangeVectors = " ++ 
--                        (concat $ map ("\n    " ++) (map show $ concat rangeVectors)) 
--                    ++ "\n unifiedRangeVector = " ++ show unifiedRangeVector 
--                )$
                result
                where
                result = map (newConstFnFromSample sampleFnInSegment) unifiedRangeVector
                unifiedRangeVector = foldl1 (zipWith (</\>)) rangeVectors 
                rangeVectors = map (map getRange) fnVectorsInSegment
                fnVectorsInSegment@((sampleFnInSegment : _) : _) = map (map fst) fnAndNameVectorsInSegment
--                nameVectorsInSegment = map (map snd) fnAndNameVectorsInSegment
                getRange fn =
                    evalAtPointOutEff effEval2 dombox fn
                    where
                    dombox = getDomainBox fn
                effEval2 = evaluationDefaultEffort sampleFnInSegment
        fnAndNameVectors =
            map getFnsFromSegInfo $ bisectionInfoGetLeafSegInfoSequence bisectionInfo
        getFnsFromSegInfo (_,_,modeEventInfos) =
            concat $ map getFnsFromMEI modeEventInfos
        getFnsFromMEI (HybSysMode modeName, eventInfo) =
            collectFns modeName eventInfo
        collectFns namePrefix (EventNextSure (_, fnVec) eventMap) =
            [numberFnVec fnVec (namePrefix ++ ".")] ++
            (concat $ map perEvent $ toAscList eventMap)
            where
            perEvent (HybSysEventKind eventName, subEventInfo) =
                collectFns (namePrefix ++ "!" ++ eventName) subEventInfo
        collectFns namePrefix (EventNextMaybe (_, fnVec) eventMap) =
            [numberFnVec fnVec (namePrefix ++ ".")] ++
            (concat $ map perEvent $ toAscList eventMap)
            where
            perEvent (HybSysEventKind eventName, subEventInfo) =
                collectFns (namePrefix ++ "?" ++ eventName) subEventInfo
        collectFns namePrefix (EventFixedPoint (_, fnVec)) =
            [numberFnVec fnVec (namePrefix ++ ".")]
        collectFns _ _ = 
            []
        numberFnVec fnVec namePrefix =
            zipWith addName fnVec componentNames
            where
            addName fn compName = (fn, namePrefix ++ compName)
    fnmeta =
        FV.simpleFnMetaData
            sampleFn
            rect
            (Just (1,1,1,1))
            (case maybeParamPlotArgs of Just (freq, _) -> freq; _ -> 200)
            tVar
            (zip segNames $ map addMetaToFnNames fnNames)
        where
        addMetaToFnNames names =
            zip3 names colourList enabledList
        colourList 
            | isBW =
                repeat black
            | otherwise = 
                cycle $ take n $ cycle [blue, green, red, black, orange, purple, magenta]
        enabledList 
            | shouldUseParamPlot = repeat True
            | otherwise = cycle activevars
--    fnmeta = 
--        (FV.defaultFnMetaData sampleFn)
--        {
--            FV.dataFnGroupNames = segNames, -- map ("segment " ++) (map show [1..segs]),
--            FV.dataFnNames = fnNames,
--            FV.dataFnStyles = map giveColours fnNames,
--            FV.dataDomName = "t",
--            FV.dataDomL = tStart,
--            FV.dataDomR = tEnd,
--            FV.dataValLO = neg domainHalf,
--            FV.dataValHI = domainHalf,
--            FV.dataDefaultActiveFns = map whichActive fnNames,
--            FV.dataDefaultEvalPoint = tEnd,
--            FV.dataDefaultCanvasParams =
--                (FV.defaultCanvasParams sampleCf)
--                {
--                    FV.cnvprmCoordSystem = 
--                        FV.CoordSystemLinear $ 
--                            FV.Rectangle  domainHalf (neg domainHalf) tStart tEnd
--                    ,
--                    FV.cnvprmSamplesPerUnit = 200
--                    ,
--                    FV.cnvprmBackgroundColour = Just (1,1,1,1)
--                }
--        }
--        where
--        domainHalf = (tEnd <-> tStart) </>| (2 :: Double)

aggregateSequencesOfTinySegments :: 
      (ArithInOut.RoundedAdd (Domain a),
       RefOrd.IntervalLike (Domain a), CanEvaluate a, HasConstFns a,
       NumOrd.PartialComparison
         (Domain a),
       RefOrd.RoundedLattice a) 
      =>
      EvaluationEffortIndicator a
      -> [String]
      -> Var a
      -> Domain a
      -> [[(a, String)]]
      -> ([[a]], [[String]], [String])
aggregateSequencesOfTinySegments effEval componentNames tVar plotMinSegSize fnsAndNames2 
    = aggrNewSegm [] [] [] $ zip ([1..]::[Int]) fnsAndNames2
    where
    aggrNewSegm 
            prevFns prevFnNames prevSegNames 
            segs@((segNoFirstSeg, fnsNamesFirstSeg@((fn1FirstSeg,_) : _)) : restSegs)
        | noAggregation =
            aggrNewSegm 
                (fnsFirstSeg : prevFns) 
                (fnNamesFirstSeg : prevFnNames) 
                (("segment " ++ show segNoFirstSeg) : prevSegNames) 
                restSegs 
        | otherwise =
            aggrNewSegm
                (fnsAggregated : prevFns) 
                (fnNamesAggregated : prevFnNames) 
                (("segments " ++ show segNoFirstSeg ++ "-" ++ show segNoLastAggrSeg) : prevSegNames) 
                restSegsAfterAggr
        where
        noAggregation = length smallSegmentsToAggregate <= 1
        (smallSegmentsToAggregate, restSegsAfterAggr) = 
            span segEndsBeforeLimit segs
            where
            segEndsBeforeLimit (_, ((fn1ThisSeg,_) : _)) =
                (tEndThisSeg <=? tAggrLimit) == Just True
                where
                (_, tEndThisSeg) = getTVarDomEndpoints fn1ThisSeg
                tAggrLimit = tStartFirstSeg <+> plotMinSegSize
        fnNamesAggregated =
            map (++ "(aggr)") componentNames
        fnsAggregated =
            foldl1 (zipWith (</\>)) $
                chunksOf (length componentNames) $
                    map makeConstFnOverAggrDom $
                        concat $ map getFnsFromSeg smallSegmentsToAggregate
            where
            chunksOf _ [] = []
            chunksOf n list = firstN : (chunksOf n rest)
                where
                (firstN, rest) = splitAt n list
            getFnsFromSeg (_, fnsNames) = map fst fnsNames
            makeConstFnOverAggrDom fn =
                newConstFn sizeLimits [(tVar, aggrDom)] range
                where
                sizeLimits = getSizeLimits fn         
                range = evalAtPointOutEff effEval dombox fn
                dombox = getDomainBox fn
                
        aggrDom = RefOrd.fromEndpointsOut (tStartFirstSeg, tEndLastAggrSeg) 
        (tStartFirstSeg, _) = getTVarDomEndpoints fn1FirstSeg
        (_, tEndLastAggrSeg) = getTVarDomEndpoints fn1LastAggrSeg
        (segNoLastAggrSeg, ((fn1LastAggrSeg,_) : _)) = last smallSegmentsToAggregate 
        getTVarDomEndpoints fn =
            case lookupVar (getDomainBox fn) tVar of 
                Just tDom -> RefOrd.getEndpointsOut tDom 
        (fnsFirstSeg, fnNamesFirstSeg) = unzip fnsNamesFirstSeg
    aggrNewSegm prevFns prevFnNames prevSegNames _ =
        (reverse prevFns, reverse prevFnNames, reverse prevSegNames)


plotHybIVPListEnclosures :: 
    (Var f ~ String,
     CanEvaluate f,
     CairoDrawableFn f,
     HasSizeLimits f,
     RefOrd.RoundedLattice f,
     HasConstFns f,
     HasDomainBox f,
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     Show f, Show (Var f), Show (Domain f),
     solvingInfoODESegment ~ (Maybe ([f],[f]), (Domain f, Maybe [Domain f])),
     solvingInfoODE ~ BisectionInfo solvingInfoODESegment (solvingInfoODESegment, Maybe (Domain f)),
     solvingInfoEvents ~ (Domain f, Maybe (HybridSystemUncertainState (Domain f)), EventInfo f)
     ) 
    =>
    FV.Rectangle (Domain f) -- ^ initial canvas viewport
    -> [Bool] -- ^ for each variable, whether it should be plotted
    -> Bool -- ^ True -> plot all components in black 
    -> Maybe (Int, [Bool]) 
        -- ^ Just (samplingFreq, scanVars) -> use parametric plot (there have to be exactly two active functions)
        -- ^   scanVars : variables with True are substituted by their domains, otherwise they are sampled  
    ->
    ArithInOut.RoundedRealEffortIndicator (Domain f)
    ->
    CairoDrawFnEffortIndicator f 
    ->
    Domain f
    -> 
    HybridIVP f
    -> 
    [(
        Domain f
        -- end time of this segment (including the event resolution sub-segment)  
     ,
        Maybe (HybridSystemUncertainState (Domain f))
     ,
        Map.Map HybSysMode 
            (
                solvingInfoODE,
                Maybe (HybridSystemUncertainState (Domain f)),
                Maybe solvingInfoEvents
            )
     )
    ]
    -> 
    Maybe FilePath 
    -> 
    IO ()
plotHybIVPListEnclosures 
        rect activevarsPre isBW maybeParamPlotArgs 
        effCF effDrawFn _plotMinSegSize (ivp :: HybridIVP f) segmentsInfo 
        maybePDFFilename =
    case maybePDFFilename of
        Nothing ->
            do
            _ <- Gtk.unsafeInitGUIForThreadedRTS
            fnDataTV <- atomically $ newTVar $ FV.FnData $ fnsPlotSpec
            fnMetaTV <- atomically $ newTVar $ fnmeta
            _ <- FV.new sampleFn effDrawFn effCF effEval (fnDataTV, fnMetaTV) Nothing
            Gtk.mainGUI
        Just pdffilename ->
            do
            FV.plotToPDFFile sampleFn effDrawFn effCF canvasParams 512 512 fnsActive fnsPlotSpec fnsStyles pdffilename
            where
            fnsStyles = (map $ const black) $ concat $ FV.dataFnStyles fnmeta
            canvasParams = FV.dataDefaultCanvasParams fnmeta
            fnsActive = concat $ FV.dataDefaultActiveFns fnmeta
    where
    shouldUseParamPlot = case maybeParamPlotArgs of Just _ -> True; _ -> False

    fnsPlotSpec = addPlotVar fns
--    effDrawFn = cairoDrawFnDefaultEffort sampleFn
    effEval = evaluationDefaultEffort sampleFn
    (((sampleFn : _) : _) : _) = fns 
--    sampleCf = getSampleDomValue sampleFn
    
    componentNames = hybsys_componentNames $ hybivp_system ivp
    n = length componentNames

    activevars =
        take n $ activevarsPre ++ (repeat False)

--    activevarNames = pickByActivevars componentNames 

    -- function to pick from a list of length componentNames those elements that correspond to active vars:
    pickByActivevars :: [a] -> [a]
    pickByActivevars list =
        map snd $ filter fst $ zip activevars list
    -- function to pick from a list of length componentNames those elements that correspond to active vars:
    pickByActivevarsCycle :: [a] -> [[a]]
    pickByActivevarsCycle [] = []
    pickByActivevarsCycle list =
        pickByActivevars batch : pickByActivevarsCycle rest
        where
        (batch, rest) = splitAt n list
--    pickByActivevarsCycle _ = error $ "plotODEIVPBisectionEnclosures: pickByActivevarsCycle: list not divisible by n"
    

    tVar = hybivp_tVar ivp

    addPlotVar fns2
        | shouldUseParamPlot
            = map (map addVParam . pickByActivevarsCycle) fns2
        | otherwise 
            = map (map addV) fns2
        where
        addV fs = (FV.GraphPlotFn (fs :: [f]), tVar)
        addVParam [fsX, fsY] = (FV.ParamPlotFns (zip fsX fsY) [], tVar)
        addVParam _ = errorParamFnCount 

    addPlotVar fns2 =
        case maybeParamPlotArgs of
            Just (_, scanVars) ->
                map (map (addVParam scanVarsList) . pickByActivevarsCycle) fns2
                where
                scanVarsList = 
                    map snd $ filter (not . fst) $ zip (scanVars ++ repeat False) componentNames
            _ ->
                map (map addV) fns2
                
        where
        addV fs = (FV.GraphPlotFn (fs :: [f]), tVar)
        addVParam scanVarsList [fsX, fsY] = (FV.ParamPlotFns (zip fsX fsY) scanVarsList, tVar)
        addVParam _scanVarsList _ = errorParamFnCount 

    errorParamFnCount =
            error 
            "plotHybIVPListEnclosures: In a parameteric plot there have to be exactly two active functions."
            
-- The following code has been copied from ODE plotter above:

--    (fns, fnNamesPre, groupNames) 
--        | shouldUseParamPlot =
--            (fns2, fnNamesPre2, segNames2)
--        | otherwise = 
--            aggregateSequencesOfTinySegments2 fnsAndNames
--        where
--        (fns2, fnNamesPre2) = unzip $ (map unzip fnsAndNames)
--        segNames2 = map snd $ zip fnNamesPre2 $ ["segment " ++ show i | i <- [1..] :: [Int]]
--    fnNames
--        | shouldUseParamPlot = 
--            fnNamesActiveJoined
--        | otherwise = 
--            fnNamesPre 
--        where
--        fnNamesActiveJoined =
--            map ((map (const joinedActiveVarsName)) . pickByActivevarsCycle) fnNamesPre
--            where
--            joinedActiveVarsName = "(" ++ List.intercalate "," activevarNames ++ ")"

    fns = [List.transpose $ splitUpGroups fns3]
        where
        splitUpGroups [] = []
        splitUpGroups ([] : gs) = splitUpGroups gs
        splitUpGroups (g : gs) =
            (take n g) : (splitUpGroups ((drop n g) : gs)) 
    groupNames = ["variables"]
    fnNames 
        | shouldUseParamPlot = [[concat $ pickByActivevars componentNames]]
        | otherwise = [componentNames]
            
    (fns3, _fnNames3, _groupNames3) 
--        | shouldUseParamPlot =
--            error "Parametric plot not yet supported."
--        | otherwise 
        = 
            unzip3 $
                map getFnsFromSegModeInfo $ 
                    concat $ map getSegModeInfo
                        $ zip ([1..]::[Int]) segmentsInfo
        where
        getSegModeInfo (segNo, (_, _, modeSolvingInfoMap)) =
            map (\i -> (segNo,i)) $ Map.toList modeSolvingInfoMap
        getFnsFromSegModeInfo (segNo, (HybSysMode modeName, (bisectionInfo, _, maybeEventSolvingInfo))) =
            (groupFns, groupFnNames, groupName)
            where
            groupName = "seg" ++ show segNo ++ "." ++ modeName
            (groupFns, groupFnNames) =
                unzip $ fnsAndNames
            fnsAndNames = 
                fnsAndNamesNoEvents ++ 
                fnsAndNamesEvents
            fnsAndNamesNoEvents =
                concat $
                    map getFnsFromSegInfo $
                        zip ([1..]::[Int]) $
                            bisectionInfoGetLeafSegInfoSequence bisectionInfo
                where
                getFnsFromSegInfo (i, (Just (fnVec, _), _)) =
                    nameFnVec fnVec prefix
                    where
                    prefix = "noev" ++ (show i)
                getFnsFromSegInfo _ = []
            fnsAndNamesEvents =
                case maybeEventSolvingInfo of
                    Nothing -> []
                    Just (_,_,eventInfo) ->
                        getFnsFromEventInfo eventInfo
            getFnsFromEventInfo eventInfo =
                collectFns "" eventInfo
            collectFns namePrefix (EventNextSure (_, fnVec) eventMap) =
                (nameFnVec fnVec namePrefix) ++
                (concat $ map perEvent $ toAscList eventMap)
                where
                perEvent (HybSysEventKind eventName, subEventInfo) =
                    collectFns (namePrefix ++ "!" ++ eventName) subEventInfo
            collectFns namePrefix (EventNextMaybe (_, fnVec) eventMap) =
                (nameFnVec fnVec namePrefix) ++
                (concat $ map perEvent $ toAscList eventMap)
                where
                perEvent (HybSysEventKind eventName, subEventInfo) =
                    collectFns (namePrefix ++ "?" ++ eventName) subEventInfo
            collectFns namePrefix (EventFixedPoint (_, fnVec)) =
                (nameFnVec fnVec namePrefix)
            collectFns _ _ = 
                []
            nameFnVec fnVec namePrefix =
                zipWith addName fnVec componentNames
                where
                addName fn compName = (fn, namePrefix ++ "." ++ compName)
    fnmeta =
        FV.simpleFnMetaData
            sampleFn
            rect
            (Just (1,1,1,1))
            (case maybeParamPlotArgs of Just (freq, _) -> freq; _ -> 500)
            tVar
            (zip groupNames $ map addMetaToFnNames fnNames)
        where
        addMetaToFnNames names =
            zip3 names colourList enabledList
        colourList 
            | isBW =
                repeat black
            | otherwise = 
                cycle $ take n $ cycle [blue, green, red, black, orange, purple, magenta]
        enabledList 
            | shouldUseParamPlot = repeat True
            | otherwise = cycle activevars

baseStyle :: FV.FnPlotStyle
baseStyle = FV.defaultFnPlotStyle
    {
        FV.styleOutlineThickness = 1
    }

black :: FV.FnPlotStyle
black = baseStyle
    { 
        FV.styleOutlineColour = Just (0,0,0,1), 
        FV.styleFillColour = Just (0,0,0,0.1)
    } 
blue :: FV.FnPlotStyle
blue = baseStyle 
    { 
        FV.styleOutlineColour = Just (0.1,0.1,0.8,1), 
        FV.styleFillColour = Just (0.1,0.1,0.8,0.1) 
    } 
green :: FV.FnPlotStyle
green = baseStyle 
    { 
        FV.styleOutlineColour = Just (0.1,0.8,0.1,1), 
        FV.styleFillColour = Just (0.1,0.8,0.1,0.1) 
    } 
red :: FV.FnPlotStyle
red = baseStyle 
    { 
        FV.styleOutlineColour = Just (0.8,0.1,0.1,1), 
        FV.styleFillColour = Just (0.8,0.1,0.1,0.1) 
    } 
orange :: FV.FnPlotStyle
orange = baseStyle 
    { 
        FV.styleOutlineColour = Just (1,0.7,0.2,1), 
        FV.styleFillColour = Just (1,0.7,0.2,0.1) 
    } 
purple :: FV.FnPlotStyle
purple = baseStyle 
    { 
        FV.styleOutlineColour = Just (0.7,0.4,1,1), 
        FV.styleFillColour = Just (0.7,0.4,1,0.1) 
    } 
magenta :: FV.FnPlotStyle
magenta = baseStyle 
    { 
        FV.styleOutlineColour = Just (0, 0.8,0.8,1), 
        FV.styleFillColour = Just (0, 0.8,0.8,0.1) 
    } 
        