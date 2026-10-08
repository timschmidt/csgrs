{-# OPTIONS_GHC -fno-warn-orphans #-}
{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE UndecidableInstances #-}
{-# LANGUAGE StandaloneDeriving #-}
--{-# LANGUAGE ScopedTypeVariables #-}
{-|
    Module      :  Numeric.AERN.RmToRn.Plot.FromEval
    Description :  plotting on gtk canvas using evaluation at some points
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Plotting on gtk canvas using evaluation at some points.
-}

module Numeric.AERN.RmToRn.Plot.FromEval where

import Numeric.AERN.RmToRn.Plot.Params
import Numeric.AERN.RmToRn.Plot.CairoDrawable

import Numeric.AERN.RmToRn.Domain
import Numeric.AERN.RmToRn.Evaluation

import Numeric.AERN.RealArithmetic.ExactOps

import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut

import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as ArithUpDn

import qualified Numeric.AERN.NumericOrder as NumOrd

import qualified Numeric.AERN.RefinementOrder as RefOrd

import Numeric.AERN.Basics.Interval
import Numeric.AERN.RmToRn.Interval ()

import Graphics.Rendering.Cairo

--import qualified Data.Map as Map
--import qualified Data.List as List

import Debug.Trace
_ = trace

instance 
    (CanEvaluate f,
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     Eq (Var f), Ord (Var f),
     Show (Domain f), Show (Var f), Show (VarBox f (Domain f))
    )
    =>
    (CairoDrawableFn (Interval f))
    where
    type CairoDrawFnEffortIndicator (Interval f) =
        CairoDrawEffortIndicatorFnFromEval (Interval f)
    cairoDrawFnDefaultEffort = cairoDrawFnDefaultEffortFromEval
    cairoDrawFnGraph = cairoDrawFnGraphFromEval
    cairoDrawFnParameteric = cairoDrawFnParametericFromEval
    
instance 
    (CanEvaluate f,
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     RefOrd.IntervalLike f,
     Eq (Var f), Ord (Var f),
     Show (Domain f), Show (Var f), Show (VarBox f (Domain f))
    )
    =>
    (CairoDrawableFn (IntervalApprox f))
    where
    type CairoDrawFnEffortIndicator (IntervalApprox f) =
        CairoDrawEffortIndicatorFnFromEval (Interval f)
    cairoDrawFnDefaultEffort (IntervalApprox o _) = cairoDrawFnDefaultEffortFromEval o
    cairoDrawFnGraph
            eff
            canvasParams toScreenCoords 
            style plotVar fns =
        do
        cairoDrawFnGraph eff canvasParams toScreenCoords styleDarker plotVar $ map upperBound fns
        cairoDrawFnGraph eff canvasParams toScreenCoords style plotVar $ map intervalInner fns
        cairoDrawFnGraph eff canvasParams toScreenCoords styleDarker plotVar $ map lowerBound fns
        where
        styleDarker = 
            style
            {
                styleFillColour = fmap darken $ styleFillColour style 
            }
            where
            darken (r,g,b,a) = (r,g,b,(1+a)/2)
        upperBound fn =
            RefOrd.fromEndpointsOut (fnInnerUpper, fnOuterUpper)
            where
            (_fnOuterLower, fnOuterUpper) = RefOrd.getEndpointsOut (intervalOuter fn) 
            (_fnInnerLower, fnInnerUpper) = RefOrd.getEndpointsOut (intervalInner fn) 
        lowerBound fn =
            RefOrd.fromEndpointsOut (fnOuterLower, fnInnerLower)
            where
            (fnOuterLower, _fnOuterUpper) = RefOrd.getEndpointsOut (intervalOuter fn) 
            (fnInnerLower, _fnInnerUpper) = RefOrd.getEndpointsOut (intervalInner fn) 
        
    cairoDrawFnParameteric
            eff
            canvasParams toScreenCoords 
            style plotVar scanVars fnPairs = 
        do
        cairoDrawFnParameteric eff canvasParams toScreenCoords style plotVar scanVars $ map getOuters fnPairs
        cairoDrawFnParameteric eff canvasParams toScreenCoords style plotVar scanVars $ map getInners fnPairs
        where
        getOuters (f1, f2) = (intervalOuter f1, intervalOuter f2)
        getInners (f1, f2) = (intervalInner f1, intervalInner f2)

data CairoDrawEffortIndicatorFnFromEval f =
    CairoDrawEffortIndicatorFnFromEval
    {
        draweff_evalF :: EvaluationEffortIndicator f
    ,
        draweff_realD :: ArithInOut.RoundedRealEffortIndicator (Domain f)
    ,
        draweff_getEndpointsD :: RefOrd.GetEndpointsEffortIndicator (Domain f)
    }

deriving instance
    (Show (EvaluationEffortIndicator f),
     Show (ArithInOut.RoundedRealEffortIndicator (Domain f)),
     Show (RefOrd.GetEndpointsEffortIndicator (Domain f)))
    => 
    Show (CairoDrawEffortIndicatorFnFromEval f) 

cairoDrawFnDefaultEffortFromEval ::
    (CanEvaluate f,
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     Show (Domain f)
    )
    =>
    f ->
    CairoDrawEffortIndicatorFnFromEval f
cairoDrawFnDefaultEffortFromEval sampleF =
    CairoDrawEffortIndicatorFnFromEval
    {
        draweff_evalF = evaluationDefaultEffort sampleF
    ,
        draweff_realD = ArithInOut.roundedRealDefaultEffort sampleDF
    ,
        draweff_getEndpointsD = RefOrd.getEndpointsDefaultEffort sampleDF
    }    
    where
    sampleDF = getSampleDomValue sampleF

cairoDrawFnGraphFromEval ::
    (CanEvaluate f,
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     Show (Domain f), Show (Var f), Show (VarBox f (Domain f))
    )
    =>
    CairoDrawEffortIndicatorFnFromEval f ->
    CanvasParams (Domain f) ->
    ((Domain f, Domain f) -> (Double, Double)) ->
    FnPlotStyle ->
    Var f ->
    [f] ->
    Render ()
cairoDrawFnGraphFromEval 
        (CairoDrawEffortIndicatorFnFromEval effEval effReal effGetE)
        canvasParams toScreenCoords 
        style plotVar fns
    | null fns = return ()
    | otherwise = 
--    unsafePrint (
--        "cairoDrawFnFromEval: starting"
--        ++ "\n dom = " ++ show dom
--        ++ "\n canvasParams = " ++ show canvasParams
--    ) $ 
    case visibleDomL NumOrd.<=? visibleDomR of
        Just True ->
--            unsafePrint (
--                "cairoDrawFnFromEval: visible domain not anti-consistent"
--                ++ "\n partition = " ++ show partition
--            ) $ 
            plotEnclosure
            where
            plotEnclosure =
                do
                -- fill the enclosure first:
                case styleFillColour style of
                    Just (r,g,b,a) ->
                        do 
                        setSourceRGBA r g b a
                        enclosureOutline
                        fill
                    _ -> return ()
                -- then draw the outline:
                case styleOutlineColour style of
                    Just (r,g,b,a) ->
                        do 
                        setSourceRGBA r g b a
                        enclosureOutline
                        setLineWidth $ styleOutlineThickness style
                        setLineCap LineCapRound
                        setLineJoin LineJoinRound
                        stroke
                    _ -> return ()
                where
                enclosureOutline =
                    do
                    moveToPt firstPointUp
                    mapM_ lineToPt pointsDn
                    mapM_ lineToPt (reverse pointsUp)
                (firstPointUp : _) = pointsUp
                pointsUp = zip partition enclosureValuesUp
                pointsDn = zip partition enclosureValuesDn
                (enclosureValuesDn, enclosureValuesUp) =
                    unzip $ map (RefOrd.getEndpointsOutEff effGetE) enclosureValues
                moveToPt = usePt moveTo 
                lineToPt = usePt lineTo 
                usePt fn pt = fn xD yD
                    where
                    (xC,yC) = translateToCoordSystem effReal coordSystem pt
                    (xD, yD) = toScreenCoords (xC,yC)
            enclosureValues =
                map evalPt partition
            evalPt d =
                foldl1 (</\>) $ map  auxEval relevantFns
                where
                auxEval fn = evalAtPointOutEff effEval pt fn
                    where
                    pt = insertVar plotVar d $ getDomainBox fn
                relevantFns = 
                    map snd $ filter hasD $ zip doms fns
                hasD (dom2, _) =
                    (dom2 RefOrd.|<=? d) == Just True

            partition =
                [domLO] ++ (map ithPt [1..(segCnt -1)]) ++ [domHI]
                where
                (domLO, domHI) = 
                    RefOrd.getEndpointsOutEff effGetE visibleDom
                ithPt i =
                    ((domLO <*>| (segCnt - i)) <+> (domHI <*>| i)) </>| segCnt
                segCnt =
                    case ArithUpDn.convertUpEff effToInt 0 (segPerUnit |<*> domWidthScreen) of
                        Just cnt -> (cnt :: Int)
                        _ -> error $ "cairoDrawFnFromEval: segCnt: internal error"
                    where
                    domWidthScreen = 
                        NumOrd.minOutEff effMinmax c1 $ domHIScreen <-> domLOScreen
                    (domLOScreen, _) = 
                        translateToCoordSystem effReal coordSystem (domLO2, c1)
                    (domHIScreen, _) = 
                        translateToCoordSystem effReal coordSystem (domHI2, c1)
                    (domLO2, domHI2) = RefOrd.getEndpointsOutEff effGetE visibleDom
                    c1 = one sampleDF
--            pickActiveVals vals =
--                fst $ unzip $ filter snd $ zip vals activeDimensions
        _ -> return ()
    where
    (visibleDomL, visibleDomR) = RefOrd.getEndpointsOutEff effGetE visibleDom
    visibleDom = 
        dom <\/> (vdomLO </\> vdomHI)
        where
        (_,_,vdomLO, vdomHI) = getVisibleDomExtents coordSystem
    dom = foldl1 (</\>) doms
    doms = 
        map getPlotDom domboxes
        where
        getPlotDom dombox =
            case lookupVar dombox plotVar of 
                Just dom2 -> dom2
                _ -> error $ 
                    "aern-realfn-plot-gtk error: plotVar not present in dombox:"
                    ++ "\n  plotVar = " ++ show plotVar 
                    ++ "\n  dombox = " ++ show dombox 
    domboxes = map getDomainBox fns
    coordSystem = cnvprmCoordSystem canvasParams
    segPerUnit = cnvprmSamplesPerUnit canvasParams
--    activeDimensions = cnvprmPlotDimensions canvasParams
    
    (<\/>) = RefOrd.joinOutEff effJoinMeet
    (</\>) = RefOrd.meetOutEff effJoinMeet
    (<+>) = ArithInOut.addOutEff effAdd
    (<->) = ArithInOut.subtrOutEff effAdd
    (<*>|) = ArithInOut.mixedMultOutEff effMultInt
    (|<*>) = flip $ ArithInOut.mixedMultOutEff effMultInt
    (</>|) = ArithInOut.mixedDivOutEff effDivInt
    
    effMinmax = ArithInOut.rrEffortMinmaxInOut sampleDF effReal
    effJoinMeet = ArithInOut.rrEffortJoinMeet sampleDF effReal
    effToInt = ArithInOut.rrEffortToInt sampleDF effReal
    effAdd =
        ArithInOut.fldEffortAdd sampleDF $ ArithInOut.rrEffortField sampleDF effReal
    effMultInt =
        ArithInOut.mxfldEffortMult sampleDF sampleI $ ArithInOut.rrEffortIntMixedField sampleDF effReal
    effDivInt =
        ArithInOut.mxfldEffortDiv sampleDF sampleI $ ArithInOut.rrEffortIntMixedField sampleDF effReal
    
    sampleDF = dom
    sampleI = 1 :: Int
    


cairoDrawFnParametericFromEval ::
    (CanEvaluate f,
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     Eq (Var f), Ord (Var f), 
     Show (Domain f), Show (Var f), Show (VarBox f (Domain f))
    )
    =>
    CairoDrawEffortIndicatorFnFromEval f ->
    CanvasParams (Domain f) ->
    ((Domain f, Domain f) -> (Double, Double)) ->
    FnPlotStyle ->
    Var f ->
    [Var f] ->
    [(f,f)] ->
    Render ()
cairoDrawFnParametericFromEval 
        (CairoDrawEffortIndicatorFnFromEval effEval effReal effGetE)
        canvasParams toScreenCoords 
        style plotVar scanVars fnPairs 
    | null fnPairs = return ()
    | otherwise =
    plotSampleBoxes
    where
    plotSampleBoxes =
        mapM_ plotValueSample enclosureFacesForPartition
    plotValueSample face =
--        trace
--        (
--            "plotValueSample: " ++ show (length face) ++ " face = "
--            ++ (unlines $ map show face)
--        ) $
        do
        -- fill the box first:
        case styleFillColour style of
            Just (r,g,b,a) ->
                do
                setSourceRGBA r g b a
                drawFace face
                fill
            _ -> return ()
        -- then draw the box outline:
        case styleOutlineColour style of
            Just (r,g,b,a) ->
                do 
                setSourceRGBA r g b a
                drawFace face
                setLineCap LineCapRound
                setLineJoin LineJoinRound
                setLineWidth $ styleOutlineThickness style
                stroke
            _ -> return ()
        where
        drawFace outlinePoints =
            do
            drawOutline
            drawVertices
            where
            drawOutline =
                do
                moveToPt (last points)
                mapM_ lineToPt points
            drawVertices =
                do
                mapM_ drawVertex points
            points = map (\[a,b] -> (a,b)) outlinePoints
            moveToPt = usePt moveTo 
            lineToPt = usePt lineTo 
            usePt pointOp (xI, yI) =
                do
                pointOp xCentreD yCentreD
                where
                (xCentreD, yCentreD) = t (xCentre, yCentre)
                xCentre = (xL <+> xR) </>| (2 :: Int)
                yCentre = (yL <+> yR) </>| (2 :: Int)
                (xL, xR) = RefOrd.getEndpointsOut xI
                (yL, yR) = RefOrd.getEndpointsOut yI
                
            drawVertex (xI, yI) =
                do 
                moveTo x1D y1D
                lineTo x2D y2D
                lineTo x3D y3D
                lineTo x4D y4D
                lineTo x1D y1D
                where
                (xL, xR) = RefOrd.getEndpointsOut xI
                (yL, yR) = RefOrd.getEndpointsOut yI
                (x1D, y1D) = t (xL, yL)
                (x2D, y2D) = t (xR, yL)
                (x3D, y3D) = t (xR, yR)
                (x4D, y4D) = t (xL, yR)
            t pt =
                toScreenCoords $
                    translateToCoordSystem effReal coordSystem pt
                
    enclosureFacesForPartition =
        concat $ map evalAreaFacesUsingSamples partition
    evalAreaFacesUsingSamples d =
        concat $ map auxEval relevantFnPairs
        where
        auxEval (fnX, fnY) =
            evalSamplesAlongFacesEff effEval 2 area scanVars [fnX, fnY]
            where
            area = insertVar plotVar d $ getDomainBox fnX 
        relevantFnPairs = 
            map snd $ filter hasD $ zip doms fnPairs
        hasD (dom2, _) =
            (dom2 RefOrd.|<=? d) == Just True
    partition =
        [domLO] ++ (map ithPt [1..(segCnt -1)]) ++ [domHI]
        where
        (domLO, domHI) = 
            RefOrd.getEndpointsOutEff effGetE dom
        ithPt i =
            ((domLO <*>| (segCnt - i)) <+> (domHI <*>| i)) </>| segCnt
        segCnt :: Int
        segCnt = round $ (fromInteger $ toInteger segPerUnit) * domSize
        domSize :: Double
        Just domSize = ArithUpDn.convertUp 0 $ domHI <-> domLO
    dom = foldl1 (RefOrd.</\>) doms 
    doms = 
        map getPlotDom domboxes
        where
        getPlotDom dombox =
            case lookupVar dombox plotVar of 
                Just dom2 -> dom2
                _ -> error $ 
                    "aern-realfn-plot-gtk error: plotVar not present in dombox:"
                    ++ "\n  plotVar = " ++ show plotVar 
                    ++ "\n  dombox = " ++ show dombox 
    domboxes = map getDomainBox $ map fst fnPairs
    coordSystem = cnvprmCoordSystem canvasParams
    segPerUnit = cnvprmSamplesPerUnit canvasParams
--    activeDimensions = cnvprmPlotDimensions canvasParams
    
    (<->) = ArithInOut.subtrOutEff effAdd
    (<+>) = ArithInOut.addOutEff effAdd
    (<*>|) = ArithInOut.mixedMultOutEff effMultInt
    (</>|) = ArithInOut.mixedDivOutEff effDivInt
    
--    effMinmax = ArithInOut.rrEffortMinmaxInOut sampleDF effReal
--    effJoinMeet = ArithInOut.rrEffortJoinMeet sampleDF effReal
--    effToInt = ArithInOut.rrEffortToInt sampleDF effReal
    effAdd =
        ArithInOut.fldEffortAdd sampleDF $ ArithInOut.rrEffortField sampleDF effReal
    effMultInt =
        ArithInOut.mxfldEffortMult sampleDF sampleI $ ArithInOut.rrEffortIntMixedField sampleDF effReal
    effDivInt =
        ArithInOut.mxfldEffortDiv sampleDF sampleI $ ArithInOut.rrEffortIntMixedField sampleDF effReal
    
--    sampleF = fst $ head fnPairs
    sampleDF = dom
    sampleI = 1 :: Int
    