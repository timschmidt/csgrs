{-# LANGUAGE CPP #-}
{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE StandaloneDeriving #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE UndecidableInstances #-}
{-# LANGUAGE ScopedTypeVariables #-}
#ifdef GLADE_DIR
#else
#define GLADE_DIR "./"
#endif 
{-|
    Module      :  Numeric.AERN.RmToRn.Plot.Canvas
    Description :  the widget where functions are plotted
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Internal module for FnView.
    The widget where functions are plotted.
-}
module Numeric.AERN.RmToRn.Plot.FnView.Canvas
where

import Numeric.AERN.RmToRn.Plot.FnView.FnData
import Numeric.AERN.RmToRn.Plot.FnView.State
import Numeric.AERN.RmToRn.Plot.FnView.Layout

import Numeric.AERN.RmToRn.Plot.Params
import Numeric.AERN.RmToRn.Plot.CairoDrawable (CairoDrawableFn(..))

import Numeric.AERN.RmToRn.Domain
--import Numeric.AERN.RmToRn.Evaluation

import Numeric.AERN.RealArithmetic.ExactOps

import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut

import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as ArithUpDn

import Numeric.AERN.Misc.IntegerArithmetic

--import qualified Numeric.AERN.RefinementOrder as RefOrd

import qualified Numeric.AERN.NumericOrder as NumOrd

import Graphics.Rendering.Cairo as Cairo
import qualified Graphics.UI.Gtk as Gtk
import Graphics.UI.Gtk (AttrOp((:=)))
--import qualified Graphics.UI.Gtk.Gdk.EventM as GdkEv

import Control.Concurrent.STM -- as STM

makeCanvas :: 
    (Show (Domain f), ArithInOut.RoundedReal (Domain f),
     CairoDrawableFn f, HasDomainBox f) 
    =>
    f
    -> CairoDrawFnEffortIndicator f
    -> ArithInOut.RoundedRealEffortIndicator (Domain f)
    -> Widgets
    -> (TVar (FnData f), TVar (FnMetaData f))
    -> TVar (FnViewState f)
    -> IO Widgets
makeCanvas sampleF effDraw effReal widgets _fndataTVs@(fadataTV, fndataTV) stateTV =
    do
    -- create canvas:
    canvas <- Gtk.drawingAreaNew
    -- set canvas properties:
--    Gtk.onRealize canvas $
--        do
----        Gtk.widgetModifyBg canvas Gtk.StateNormal 
----        Gtk.set canvas []
--        return () 
    -- open font for labels:
    let font = () -- a dummy
    -- set the canvas repaint handler:
    _ <- Gtk.onExpose canvas $ \ _event ->
        do
        (fndatas, state) <- atomically $
            do
            fadata <- readTVar fadataTV
            fndata <- readTVar fndataTV
            state <- readTVar stateTV
            return ((fadata, fndata), state)  
        repaintCanvas sampleF effDraw effReal canvas font fndatas state
--    Gtk.timeoutAdd (Gtk.widgetQueueDraw canvas >> return True) 500 >> return ()
    -- plug the canvas in the GUI:
    Gtk.set (wgt_canvasAlignment widgets)
        [ Gtk.containerChild := canvas ]
    return $ widgets { wgt_canvas = canvas }
    
repaintCanvas ::
    (CairoDrawableFn f,
     ArithInOut.RoundedReal (Domain f),
     HasDomainBox f,
     Show (Domain f))
    =>
    f ->
    (CairoDrawFnEffortIndicator f) ->
    (ArithInOut.RoundedRealEffortIndicator (Domain f)) ->
    Gtk.DrawingArea ->
--    FTGL.Font ->
    () ->
    ((FnData f),
     FnMetaData f) ->
    (FnViewState f) ->
    IO Bool
repaintCanvas (sampleF :: f) effDraw effReal da _font (fndata, fnmeta) state =
    do
--    putStrLn "repaintCanvas: starting"
    drawWin <- Gtk.widgetGetDrawWindow da
    (wi,hi) <- Gtk.widgetGetSize da
    let w = realToFrac wi; h = realToFrac hi
    Gtk.renderWithDrawable drawWin $
        do
--        background w h
        -- draw outlines of all function enclosures:
        drawFunctions sampleF effDraw effReal canvasParams w h fnsActive fns fnsStyles
--    putStrLn "repaintCanvas: done"
    return True
    where
    canvasParams = favstCanvasParams state
    fnsActive = concat $ favstActiveFns state
    fnsStyles = concat $ dataFnStyles fnmeta
    fns = concat $ dataFns fndata
    
drawFunctions ::
    (CairoDrawableFn f,
     ArithInOut.RoundedReal (Domain f),
     HasDomainBox f,
     Show (Domain f))
    =>
    f ->
    (CairoDrawFnEffortIndicator f) ->
    (ArithInOut.RoundedRealEffortIndicator (Domain f)) ->
    CanvasParams (Domain f) ->
    Double -> Double ->
    [Bool] ->
    [(GraphOrParamPlotFn f, Var f)] ->
    [FnPlotStyle] ->
    Render ()
drawFunctions (sampleF :: f) effDraw effReal canvasParams w h fnsActive fns fnsStyles =
    do
    background
    drawAxes
    drawSampleValues
    mapM_ drawFn $ collectFns fnsActive fns fnsStyles
    where
    background =
        case cnvprmBackgroundColour canvasParams of
            Just (r,g,b,a) ->
                do
                moveTo 0 0
                mapM_ (uncurry lineTo) [(w,0),(w,h),(0,h),(0,0)]
                setSourceRGBA r g b a
                fill
            Nothing ->
                return ()
    
    collectFns [] _ _ = []
    collectFns (False:restActive) (_:restFns) (_:restStyles) = 
        collectFns restActive restFns restStyles
    collectFns (True:restActive) (fn:restFns) (col:restStyles) = 
        (fn, col) : (collectFns restActive restFns restStyles)
    collectFns _ _ _ = 
        error $ 
            "FV: Canvas: mismatch in fnMeta: |fns| = " ++ show (length fns) 
            ++ "; |fnsStyles| = " ++ show (length fnsStyles)
            ++ "; fnsActive = " ++ show fnsActive 
    
    drawFn ::
        ((GraphOrParamPlotFn f, Var f), FnPlotStyle) ->
        Render ()
    drawFn ((GraphPlotFn fs, plotVar), style) =
        cairoDrawFnGraph effDraw canvasParams toScreenCoordsFromUnit style plotVar fs
    drawFn ((ParamPlotFns fsXY scanVars, plotVar), style) =
        cairoDrawFnParameteric effDraw canvasParams toScreenCoordsFromUnit style plotVar scanVars fsXY
        
    drawAxes 
        | cnvprmShowAxes canvasParams =
            do
            setSourceRGBA 0 0 0 1
            setLineWidth 0.5
            extents <- getExtents
            uncurry moveTo $ toScreenCoordsFromOrig (domLO,c0)  
            uncurry lineTo $ toScreenCoordsFromOrig (domHI,c0)
            stroke  
            uncurry moveTo $ makeSpaceForXLabels extents $ toScreenCoordsFromOrig (c0,valLO)  
            uncurry lineTo $ toScreenCoordsFromOrig (c0,valHI)
            stroke
        | otherwise = return ()
        where
        getExtents =
            case cnvprmShowSampleValuesFontSize canvasParams of
                Just fontSize ->
                    do
                    setFontSize fontSize
                    extents <- textExtents "0"
                    return (Just extents)
                _ -> return Nothing
            
--        makeSpaceForYLabels Nothing pt = pt
--        makeSpaceForYLabels (Just extents) (x,y) = (x+d, y)
--            where
--            d = 1.2 * (textExtentsWidth extents)
        makeSpaceForXLabels Nothing pt = pt
        makeSpaceForXLabels (Just extents) (x,y) = (x, y-d)
            where
            d = 15 + 1.3 * (textExtentsHeight extents)
    drawSampleValues =
        case cnvprmShowSampleValuesFontSize canvasParams of
            Just fontSize ->
                do
                setFontSize fontSize
                setSourceRGBA 0 0 0 1
                setLineWidth 0.5
                mapM_ (showPt True) xPts
                mapM_ (showPt False) yPts
                return ()
            _ -> return ()

    showPt isX (pt,text) =
        do
        -- work out the size of the text:
        extents <- textExtents text
        let textWidthHalf = (textExtentsWidth extents)/2  
        let textHeight = textExtentsHeight extents
        -- fix the base position of the label:
        let (x,y) = toScreenCoordsFromUnit pt
        let pX = if isX then x else x + textWidthHalf + 2
        let pY = if isX then y - 1.3*textHeight - 5 else y
        -- draw a small line at the point:
        case isX of
            True ->
                do  
                moveTo pX (pY - 5)
                lineTo pX (pY + 5)
                stroke
            False ->
                do  
                moveTo (pX - 5) pY
                lineTo (pX + 5) pY
                stroke
        -- center the label below the cross:
        moveTo (pX - textWidthHalf) (pY + 1.3*textHeight + if isX then 5 else 0)
        showText text
        
    yPts = map translYtoPt $ pickAFewDyadicBetween sampleDom effReal valLO valHI
    xPts = map translXtoPt $ pickAFewDyadicBetween sampleDom effReal domLO domHI
    translXtoPt (x, xText) = ((xUnit, c0), xText)
        where
        (xUnit,_) = translateToCoordSystem effReal coordSystem (x,x)
    translYtoPt (y, yText) = ((c0, yUnit), yText)
        where
        (_,yUnit) = translateToCoordSystem effReal coordSystem (y,y)

    (valHI,valLO,domLO,domHI) = getVisibleDomExtents coordSystem 
    _ = [valLO,valHI,domLO,domHI,sampleDom]
    
    toScreenCoordsFromOrig =
        toScreenCoordsFromUnit . translateToCoordSystem effReal coordSystem
    
    toScreenCoordsFromUnit ::
        (Domain f, Domain f) ->
        (Double, Double)        
    toScreenCoordsFromUnit (xUnit,yUnit) =
        (w*xUnitD,h*(1-yUnitD)) -- Cairo's origin is the top left corner 
        where
        _ = [sampleDom, xUnit, yUnit]
        Just xUnitD = ArithUpDn.convertUpEff effToDouble 0 xUnit
        Just yUnitD = ArithUpDn.convertUpEff effToDouble 0 yUnit

    coordSystem = cnvprmCoordSystem canvasParams
    c0 = zero sampleDom
    sampleDom = getSampleDomValue sampleF
    effToDouble = ArithInOut.rrEffortToDouble sampleDom effReal


pickAFewDyadicBetween ::
    (ArithInOut.RoundedReal t)
    =>
    t ->
    (ArithInOut.RoundedRealEffortIndicator t) ->
    t ->
    t ->
    [(t,String)]
pickAFewDyadicBetween sampleDom effReal a b =
    map mkPt [aCount..bCount]
    where
    mkPt count =
        (value, text)
        where
        value = 
            count |<*> partitionBase
        text =
            case logOfSizeIMinusOne of
--                _ | count == 0 -> "0" 
                lgs | 0 <= lgs && lgs < 10 -> show (count * (2^lgs))
                lgs | -10 < lgs && lgs < 0 -> show count ++ "/" ++ (show $ (2::Int)^(-lgs))
                lgs | lgs < 0 -> show count ++ "*2^(" ++ show lgs ++ ")"
                lgs -> show count ++ "*2^" ++ show lgs
    
    aCount, bCount :: Integer
    Just aCount =
        ArithUpDn.convertUpEff effToInteger 0 $
            a </> partitionBase
    Just bCount = 
        ArithUpDn.convertDnEff effToInteger 0 $
            b </> partitionBase
    
    partitionBase =
        case logOfSizeIMinusOne < 0 of
            True ->
                c1 </> (c2 <^> (negate logOfSizeIMinusOne))
            _ ->
                c2 <^> logOfSizeIMinusOne
                      
    logOfSizeIMinusOne = logOfSizeI - 1
    
    logOfSizeI :: Int
    logOfSizeI 
        | sizeBelowOne =
            negate $ intLogUp (2::Integer) sizeRecipUpI
        | otherwise =
            intLogDown (2::Integer) sizeDnI
    
    sizeRecipUpI, sizeDnI :: Integer
    Just sizeRecipUpI = ArithUpDn.convertUpEff effToInteger 0 sizeRecip
    Just sizeDnI = ArithUpDn.convertDnEff effToInteger 0 size

    sizeBelowOne = (size <? c1) == Just True
        
    sizeRecip = c1 </> size
    size = b <-> a

    c1 = one sampleDom
    c2 = c1 <+> c1

    (<?) = NumOrd.pLessEff effComp
    
    (<+>) = ArithInOut.addOutEff effAdd
    (<->) = ArithInOut.subtrOutEff effAdd
    (</>) = ArithInOut.divOutEff effDiv
    (<^>) = ArithInOut.powerToNonnegIntOutEff effPow
    (|<*>) = flip $ ArithInOut.mixedMultOutEff effMultInteger

    effPow =
        ArithInOut.fldEffortPow sampleDom $ ArithInOut.rrEffortField sampleDom effReal
    effAdd =
        ArithInOut.fldEffortAdd sampleDom $ ArithInOut.rrEffortField sampleDom effReal
    effDiv =
        ArithInOut.fldEffortDiv sampleDom $ ArithInOut.rrEffortField sampleDom effReal
    effMultInteger =
        ArithInOut.mxfldEffortMult sampleDom (1::Integer) $ ArithInOut.rrEffortIntegerMixedField sampleDom effReal
    effToInteger = ArithInOut.rrEffortToInteger sampleDom effReal
    effComp = ArithInOut.rrEffortNumComp sampleDom effReal
    _ = [a,b,c1,c2,size,partitionBase,sampleDom]
    