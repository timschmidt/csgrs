{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE StandaloneDeriving #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE UndecidableInstances #-}
{-|
    Module      :  Numeric.AERN.RmToRn.Plot.FnView.State
    Description :  internal state of a FnView widget
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Internal module for FnView.
    Internal state of a FnView widget.
-}
module Numeric.AERN.RmToRn.Plot.FnView.State
--(
--)
where

import Numeric.AERN.RmToRn.Plot.FnView.FnData

import Numeric.AERN.RmToRn.Plot.Params

import Numeric.AERN.RmToRn.Domain

import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut

import qualified Numeric.AERN.RefinementOrder as RefOrd

data FnViewState f =
    FnViewState
    {
        favstActiveFns :: [[Bool]],
        favstTrackingDefaultEvalPt :: Bool,
        favstCanvasParams :: CanvasParams (Domain f),
        favstZoomPercent :: Double,
        favstPanCentre :: (Domain f, Domain f) 
    }
    
initState :: 
    (ArithInOut.RoundedReal (Domain f))
    =>
    (ArithInOut.RoundedRealEffortIndicator (Domain f)) ->
    (t, FnMetaData f) -> 
    FnViewState f
initState effReal (_, fnmeta) =
    FnViewState
    {
        favstActiveFns = activeFns,
        favstTrackingDefaultEvalPt = True,
        favstCanvasParams = 
            (dataDefaultCanvasParams fnmeta)
            {
                cnvprmCoordSystem = 
                    linearCoordsWithZoomAndCentre effReal defaultZoom centre $
                        getFnExtents fnmeta 
            },
        favstZoomPercent = defaultZoom,
        favstPanCentre = centre
    }
    where
    centre = getDefaultCentre effReal fnmeta
    activeFns = mergeDefaults (mergeDefaults $ \ _a b -> b) allTrue $ dataDefaultActiveFns fnmeta
    allTrue = map (map $ const True) $ dataFnNames fnmeta
    mergeDefaults :: (a -> a -> a) -> [a] -> [a] -> [a]
    mergeDefaults _ [] _ = [] 
    mergeDefaults _ list1 [] = list1
    mergeDefaults mergeElems (h1:t1) (h2:t2) =
        (mergeElems h1 h2) : (mergeDefaults mergeElems t1 t2)
    
updateShowAxes ::
    Bool ->
    (FnViewState f) ->
    (FnViewState f)
updateShowAxes showAxes state = 
    state 
    { 
        favstCanvasParams = 
            (favstCanvasParams state) 
                { cnvprmShowAxes = showAxes }
    }
    
updateFontSize ::
    Maybe Double ->
    (FnViewState f) ->
    (FnViewState f)
updateFontSize maybeFontSize state = 
    state 
    { 
        favstCanvasParams = 
            (favstCanvasParams state) 
                { cnvprmShowSampleValuesFontSize = maybeFontSize }
    }
    
defaultZoom :: Double
defaultZoom = 90
    
updateZoomPanCentreCoordSystem ::
    Double ->
    (Domain f, Domain f) ->
    (CoordSystem (Domain f)) ->
    (FnViewState f) ->
    (FnViewState f)
updateZoomPanCentreCoordSystem zoomPercent panCentre coordSystem state = 
    state 
    { 
        favstCanvasParams = 
            (favstCanvasParams state) 
                { cnvprmCoordSystem = coordSystem },
        favstZoomPercent = zoomPercent,
        favstPanCentre = panCentre
    }
    
updatePanCentreCoordSystem ::
    (Domain f, Domain f) ->
    (CoordSystem (Domain f)) ->
    (FnViewState f) ->
    (FnViewState f)
updatePanCentreCoordSystem = updateZoomPanCentreCoordSystem defaultZoom

updateZoomPercentAndFnExtents :: 
   ArithInOut.RoundedReal (Domain f) 
   =>
   ArithInOut.RoundedRealEffortIndicator (Domain f)
   -> Double
   -> (Domain f, Domain f, Domain f, Domain f)
   -> FnViewState f
   -> FnViewState f
updateZoomPercentAndFnExtents effFromDouble zoomPercent fnExtents state =
    state
    {
        favstCanvasParams = 
            (favstCanvasParams state) 
                { cnvprmCoordSystem = newCoordSystem },
        favstZoomPercent = zoomPercent
    }
    where
    newCoordSystem =
        case cnvprmCoordSystem (favstCanvasParams state) of
            csys@(CoordSystemLogSqueeze _) -> 
                csys
            CoordSystemLinear _ ->
                linearCoordsWithZoomAndCentre effFromDouble zoomPercent centre fnExtents
    centre = favstPanCentre state

updateCentreByRatio ::
    (ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     Show (Domain f))
    =>
    (ArithInOut.RoundedRealEffortIndicator (Domain f)) -> 
    (Double, Double) ->
    FnViewState f -> 
    FnViewState f
updateCentreByRatio effReal (ratX, ratY) state =
    case cnvprmCoordSystem (favstCanvasParams state) of
        CoordSystemLogSqueeze _ -> state
        CoordSystemLinear (Rectangle hi lo l r) ->
--            unsafePrint (
--                "updateCentreByRatio: CoordSystemLinear: "
--                ++ "\n ratX = " ++ show ratX
--                ++ "; ratY = " ++ show ratY
--                ++ "\n shiftX = " ++ show shiftX
--                ++ "; shiftY = " ++ show shiftY
--                ++ "\n old cX = " ++ show cX
--                ++ "; old cY = " ++ show cY
--                ++ "\n old Rect = " ++ show (hi,lo,l,r)
--                ++ "\n new system = " ++ show coordSystem
--            ) $ 
            state
            {
                favstCanvasParams = 
                    (favstCanvasParams state) 
                        { cnvprmCoordSystem = coordSystem },
                favstPanCentre = (shiftX cX, shiftY cY)
            }
            where
            (cX,cY) = favstPanCentre state
            shiftX a =
                fst $ RefOrd.getEndpointsOut $ 
                    a <-> (ratX |<*> fnDomWidth)
            shiftY a = 
                fst $ RefOrd.getEndpointsOut $ 
                    a <-> (ratY |<*> fnRangeHeight)
            fnDomWidth = 
                r <-> l
            fnRangeHeight = 
                lo <-> hi
            coordSystem =
                CoordSystemLinear 
                    (Rectangle 
                        (shiftY hi) (shiftY lo) 
                        (shiftX l) (shiftX r))

            (<->) = ArithInOut.subtrOutEff effAdd
            (|<*>) = flip $ ArithInOut.mixedMultOutEff effMultDbl

            effAdd =
                ArithInOut.fldEffortAdd sampleDom $ ArithInOut.rrEffortField sampleDom effReal
            effMultDbl =
                ArithInOut.mxfldEffortMult sampleDom (1::Double) $ ArithInOut.rrEffortDoubleMixedField sampleDom effReal

            sampleDom = cX
            
    
updateFnActive :: 
    Int -> 
    Int -> 
    Bool -> 
    (FnViewState f) -> 
    (FnViewState f)
updateFnActive fnNo dimNo isActive state =
    state
    {
        favstActiveFns = updateDim $ favstActiveFns state
    }
    where
    updateDim activeDims =
        listUpdate fnNo activeFnDims activeDims
        where
        activeFnDims =
            listUpdate dimNo isActive (activeDims !! fnNo)

linearCoordsWithZoom ::
    (ArithInOut.RoundedReal t)
    =>
    ArithInOut.RoundedRealEffortIndicator t ->
    Double {-^ zoom level in percent -}  ->
    (t,t,t,t) 
        {-^ upper, lower, left, right bounds of the function graph -} ->
    CoordSystem t
linearCoordsWithZoom effReal zoomPercent fnExtents@(fnHI, fnLO, fnL, fnR) =
    linearCoordsWithZoomAndCentre effReal zoomPercent (cX,cY) fnExtents
    where
    cX = 
        (fnL <+> fnR) </>| (2 :: Int) 
    cY = 
        (fnLO <+> fnHI) </>| (2 :: Int) 

    (<+>) = ArithInOut.addOutEff effAdd
    (</>|) = ArithInOut.mixedDivOutEff effDivInt
    effAdd =
        ArithInOut.fldEffortAdd sampleDom $ ArithInOut.rrEffortField sampleDom effReal
    effDivInt =
        ArithInOut.mxfldEffortDiv sampleDom (1::Int) $ 
            ArithInOut.rrEffortIntMixedField sampleDom effReal
    sampleDom = fnL

linearCoordsWithZoomAndCentre ::
    (ArithInOut.RoundedReal t)
    =>
    ArithInOut.RoundedRealEffortIndicator t ->
    Double {-^ zoom level in percent -}  ->
    (t, t) {-^ x,y coordinates of the centre -}  ->
    (t, t, t, t) 
        {-^ upper, lower, left, right bounds of the function graph -} ->
    CoordSystem (t)
linearCoordsWithZoomAndCentre effReal zoomPercent (cX,cY) (fnHI, fnLO, fnL, fnR) =
    CoordSystemLinear $ Rectangle hi lo l r
    where
    hi = 
        cY <+> heighHalf
    lo = 
        cY <-> heighHalf
    l = 
        cX <-> widthHalf
    r = 
        cX <+> widthHalf
    heighHalf = 
        zoomRatio |<*> fnHeightHalf
    widthHalf = 
        zoomRatio |<*> fnWidthHalf
    zoomRatio = 100 / zoomPercent
    fnWidthHalf = 
        (fnR <-> fnL) </>| (2 :: Int)
    fnHeightHalf = 
        (fnHI <-> fnLO) </>| (2 :: Int)

    (<+>) = ArithInOut.addOutEff effAdd
    (<->) = ArithInOut.subtrOutEff effAdd
    (|<*>) = flip $ ArithInOut.mixedMultOutEff effMultDbl
    (</>|) = ArithInOut.mixedDivOutEff effDivInt

    effAdd =
        ArithInOut.fldEffortAdd sampleDom $ ArithInOut.rrEffortField sampleDom effReal
    effMultDbl =
        ArithInOut.mxfldEffortMult sampleDom (1::Double) $ 
            ArithInOut.rrEffortDoubleMixedField sampleDom effReal
    effDivInt =
        ArithInOut.mxfldEffortDiv sampleDom (1::Int) $ 
            ArithInOut.rrEffortIntMixedField sampleDom effReal
    sampleDom = cX

    
listUpdate :: Int -> a -> [a] -> [a]
listUpdate _ _ [] = error "FV: listUpdate: invalid index"
listUpdate i newx (x:xs) 
    | i == 0 = newx : xs
    | i > 0 = x : (listUpdate (i - 1) newx xs)
    | otherwise = error "FV: listUpdate: invalid index"

