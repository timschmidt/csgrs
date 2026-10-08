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
    Module      :  Numeric.AERN.IVP.Plot.PicardView
    Description :  UI for controlling and visualising Picard iterations 
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    UI for controlling and visualising Picard iterations.
    
    NOT COMPLETED, CURRENTLY STALLED
-}
module Numeric.AERN.IVP.Plot.PicardView
(
    new
)
where

import Numeric.AERN.IVP.Plot.PicardView.State
import Numeric.AERN.IVP.Plot.PicardView.Layout

import qualified Numeric.AERN.RmToRn.Plot.FnView as FV

import Numeric.AERN.RmToRn.Plot.Params
import Numeric.AERN.RmToRn.Plot.CairoDrawable (CairoDrawableFn(..))

import Numeric.AERN.RmToRn.New
import Numeric.AERN.RmToRn.Domain
import Numeric.AERN.RmToRn.Evaluation

import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut
--import Numeric.AERN.RealArithmetic.RefinementOrderRounding.OpsImplicitEffort

--import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as ArithUpDn

import qualified Numeric.AERN.RefinementOrder as RefOrd
--import Numeric.AERN.RefinementOrder.OpsImplicitEffort

import Graphics.Rendering.Cairo as Cairo
import qualified Graphics.UI.Gtk as Gtk
import Graphics.UI.Gtk (AttrOp((:=)))
import qualified Graphics.UI.Gtk.Gdk.EventM as GdkEv

import Control.Concurrent (forkIO)
import Control.Concurrent.STM -- as STM
import Data.IORef

import qualified System.FilePath as FilePath

{-|
    Create a new Picard IVP simulation widget.
-}
new ::
    (CairoDrawableFn f,
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     HasEvalOps f (Domain f),
     HasConstFns f,
     RefOrd.PartialComparison (Domain f),
     Show f, Show (Domain f))
    =>
    f {-^ sample value -} ->
    (CairoDrawFnEffortIndicator f) ->
    ArithInOut.RoundedRealEffortIndicator (Domain f) ->
    EvalOpsEffortIndicator f (Domain f) ->
    IVP f ->
    (Maybe Gtk.Window) {- ^ parent window -} -> 
    IO Gtk.Window
new (sampleF :: f) effDraw effReal effEval ivp maybeParentWindow =
    do
    -- create initial state objects:
    (stateTV, fnTV) <- atomically $
        do
        let (fns, state) = initState sampleF effReal ivp
        stateTV <- newTVar state
        fnTV <- newTVar fns
        return (stateTV, fnTV)
    dynWidgetsRef <- newIORef initPicardViewDynWidgets
    -- create most widgets:
    widgets <- loadGlade (FilePath.combine GLADE_DIR "PicardView.glade")
    -- create a function view:
    -- TODO
    
    -- add dynamic function label widgets:
--    updateSegmentInfoWidgets widgets dynWidgetsRef stateTV
    
    -- attach handlers to widgets
    Gtk.onDestroy (window widgets) $
        do
        Gtk.mainQuit
--    setHandlers sampleF effDraw effReal effEval widgets dynWidgetsRef stateTV

    Gtk.widgetShowAll $ window widgets
    return $ window widgets
    where
    initialValues = ivpInitialValues ivp
    sampleDom = getSampleDomValue sampleF

--setHandlers :: 
--    (CairoDrawableFn f,
--     ArithInOut.RoundedReal (Domain f),
--     RefOrd.IntervalLike (Domain f),
--     HasEvalOps f (Domain f), 
--     Show f, Show (Domain f))
--    =>
--    f {-^ sample value -} ->
--    (CairoDrawFnEffortIndicator f) ->
--    ArithInOut.RoundedRealEffortIndicator (Domain f) ->
--    EvalOpsEffortIndicator f (Domain f) ->
--    Widgets -> 
--    IORef PicardViewDynWidgets -> 
--    TVar (PicardViewState f) -> 
--    IO ()
--setHandlers (sampleF :: f) effDraw effReal effEval widgets dynWidgetsRef fndataTVs@(fndataTV, fnmetaTV) stateTV =
--    do
--    setHandlerCoordSystem
--    setHandlerZoomAndPanEntries
--    setHandlerPanByMouse
--    setHandlerZoomByMouse
--    setHandlerDefaultEvalPointButton
--    setHandlerEvalPointEntry
--    setHandlerPrintTXTButton
--    setHandlerExportPDFButton
--    setHandlerExportSVGButton
--    setHandlerExportPNGButton
--
--    state <- atomically $ readTVar stateTV
--    updateZoomWidgets toDbl widgets state
----    putStrLn $ "setHandlers: " ++ (show $ cnvprmCoordSystem $ favstCanvasParams state)
--    return ()
--    where        
--    toDbl :: (Domain f) -> Double
--    toDbl a = d
--        where
--        (Just d) = ArithUpDn.convertUpEff effToDouble a
--    effToDouble = ArithInOut.rrEffortToDouble sampleDom effReal
--    effFromDouble = ArithInOut.rrEffortFromDouble sampleDom effReal
--    sampleDom = getSampleDomValue sampleF
--    
--    setHandlerCoordSystem =
--        do
--        Gtk.on (coorSystemCombo widgets) Gtk.changed resetZoomPanFromCoordSystem
--        Gtk.onClicked (defaultZoomPanButton widgets) resetZoomPanFromCoordSystem
--        where
--        resetZoomPanFromCoordSystem =
--            do
--            maybeCSysIx <- Gtk.comboBoxGetActive (coorSystemCombo widgets)
----            putStrLn $ "resetZoomPanFromCoordSystem: maybeCSysIx = " ++ show maybeCSysIx
--            case maybeCSysIx of
--                -1 -> return ()
--                ix ->
--                    do
--                    state <-
--                        atomically $
--                            do
--                            fnmeta <- readTVar fnmetaTV
--                            let coordSystem = case ix of
--                                    0 -> CoordSystemLogSqueeze sampleDom
--                                    1 -> 
--                                        linearCoordsWithZoom effReal defaultZoom (getFnExtents fnmeta)
--                            state <- modifyTVar stateTV $ 
--                                updatePanCentreCoordSystem (getDefaultCentre effReal fnmeta) coordSystem
--                            return state
--                    Gtk.widgetQueueDraw (canvas widgets)
--                    updateZoomWidgets toDbl widgets state
--        
--    setHandlerZoomAndPanEntries =
--        do
--        Gtk.onEntryActivate (zoomEntry widgets) (zoomHandler ())
--        Gtk.onFocusOut (zoomEntry widgets) (\ e -> zoomHandler False)
--        Gtk.onEntryActivate (centreXEntry widgets) (zoomHandler ())
--        Gtk.onFocusOut (centreXEntry widgets) (\ e -> zoomHandler False)
--        Gtk.onEntryActivate (centreYEntry widgets) (zoomHandler ())
--        Gtk.onFocusOut (centreYEntry widgets) (\ e -> zoomHandler False)
--        where
--        zoomHandler returnValue =
--            do
--            zoomS <- Gtk.entryGetText (zoomEntry widgets)
--            centreXS <- Gtk.entryGetText (centreXEntry widgets)
--            centreYS <- Gtk.entryGetText (centreYEntry widgets)
--            case (reads zoomS, reads centreXS, reads centreYS)  of
--                ([(zoomPercent,"")], [(centreXD,"")], [(centreYD,"")]) -> 
--                    atomically $
--                        do
--                        fnmeta <- readTVar fnmetaTV
--                        modifyTVar stateTV $ 
--                            updateZoomPanCentreCoordSystem zoomPercent (centreX, centreY) $ 
--                                linearCoordsWithZoomAndCentre effReal zoomPercent (centreX, centreY) $
--                                    getFnExtents fnmeta 
--                        return ()
--                    where
--                    centreX = ArithInOut.convertOutEff effFromDouble (centreXD :: Double)
--                    centreY = ArithInOut.convertOutEff effFromDouble (centreYD :: Double)
--                wrongParseResults ->
--                    do
--                    putStrLn $ "zoomHandler: parse error: " ++ show wrongParseResults 
--                    return ()
----            putStrLn $ "zoomHandler"
--            Gtk.widgetQueueDraw (canvas widgets)
--            return returnValue
--
--    setHandlerPanByMouse =
--        do
--        -- a variable to keep track of position before each drag movement:
--        panOriginTV <- atomically $ newTVar Nothing
--        -- setup the canvas to receive various mouse events:
--        Gtk.widgetAddEvents (canvas widgets)
--            [Gtk.ButtonPressMask, Gtk.ButtonReleaseMask, Gtk.PointerMotionMask]
--        -- what to do when the left mouse button is pressed:
--        Gtk.on (canvas widgets) Gtk.buttonPressEvent $
--            do
--            button <- GdkEv.eventButton
--            coords <- GdkEv.eventCoordinates
--            case button of
--                GdkEv.LeftButton ->
--                    liftIO $
--                        do
--                        -- remember the position and indicate that dragging is underway:
--                        atomically $ writeTVar panOriginTV (Just coords)
--                        return ()
--                _ -> return ()
--            return False
--        -- what to do when the left mouse button is released:
--        Gtk.on (canvas widgets) Gtk.buttonReleaseEvent $
--            do
--            button <- GdkEv.eventButton 
--            case button of
--                GdkEv.LeftButton ->
--                    liftIO $
--                        do
--                        -- indicate no dragging is underway:
--                        atomically $ writeTVar panOriginTV Nothing
--                        return ()
--                _ -> return ()
--            return False
--        -- what to do when mouse moves:
--        Gtk.on (canvas widgets) Gtk.motionNotifyEvent $
--            do
--            coords@(x,y) <- GdkEv.eventCoordinates
--            liftIO $
--                do
--                -- update the dragging information variable:
--                maybePanOrigin <- atomically $
--                    do
--                    maybePanOrigin <- readTVar panOriginTV
--                    case maybePanOrigin of
--                        Nothing ->
--                            return maybePanOrigin
--                        Just _ ->
--                            do
--                            writeTVar panOriginTV (Just coords)
--                            return maybePanOrigin
--                -- check whether dragging or not:
--                case maybePanOrigin of
--                    Nothing -> return ()
--                    Just panOrigin@(oX,oY) -> -- yes, dragging occurred
--                        do
--                        -- find out the size of the canvas:
--                        (canvasX, canvasY) <- Gtk.widgetGetSize (canvas widgets)
--                        -- recalculate the centre and coordinate bounds 
--                        -- based on the drag amount relative to the size fo the canvas:
--                        state <- atomically $ modifyTVar stateTV $
--                            updateCentreByRatio
--                                effReal
--                                ((x - oX) / (int2dbl canvasX), 
--                                 (y - oY) / (int2dbl canvasY))
----                        putStrLn $ "motionNotifyEvent: coords = " ++ show coords 
----                            ++ "; panOrigin = " ++ show panOrigin
----                            ++ "; (canvasX,canvasY) = " ++ show (canvasX,canvasY)
--                        return ()
--                        -- make sure the text in the zoom and pan entries are updated:
--                        updateZoomWidgets toDbl widgets state
--                        -- schedule the canvas for redraw:
--                        Gtk.widgetQueueDraw (canvas widgets)
--                        where
--                        int2dbl :: Int -> Double
--                        int2dbl = realToFrac
--            return False
--        return ()
--            
--    setHandlerZoomByMouse =
--        do -- IO
--        Gtk.widgetAddEvents (canvas widgets) [Gtk.ScrollMask]
--        Gtk.on (canvas widgets) Gtk.scrollEvent $
--            do -- ReaderTV
--            scrollDirection <- GdkEv.eventScrollDirection
--            liftIO $
--                do -- IO
--                state <- atomically $
--                    do -- STM
--                    state <- readTVar stateTV 
--                    let zoomPercent = favstZoomPercent state
--                    let newZoomPercent = case scrollDirection of
--                                            GdkEv.ScrollUp ->  1.25 * zoomPercent
--                                            GdkEv.ScrollDown -> 0.8 * zoomPercent
--                                            _ -> zoomPercent
--                    fnmeta <- readTVar fnmetaTV
--                    state <- modifyTVar stateTV $ 
--                        updateZoomPercentAndFnExtents effReal newZoomPercent $ getFnExtents fnmeta
--                    return state
--                updateZoomWidgets toDbl widgets state
--                Gtk.widgetQueueDraw (canvas widgets)
--                return False
--        return () -- TODO
--            
--    setHandlerDefaultEvalPointButton =
--        Gtk.onClicked (defaultEvalPointButton widgets) $
--            do
--            (state, fnmeta) <- 
--                atomically $
--                    do
--                    state <- readTVar stateTV
--                    fnmeta <- readTVar fnmetaTV
--                    return (state, fnmeta)
--            case favstTrackingDefaultEvalPt state of
--                False ->
--                    do
--                    Gtk.entrySetText (evalPointEntry widgets) $ 
--                        show $ dataDefaultEvalPoint fnmeta
--                    atomically $ modifyTVar stateTV $
--                        \ st -> st { favstTrackingDefaultEvalPt = True }
--                    updateValueDisplayTV effFromDouble effEval widgets dynWidgetsRef fndataTVs stateTV 
--                True -> -- already tracking the default
--                    return ()
--    setHandlerEvalPointEntry =
--        do
--        Gtk.onEntryActivate (evalPointEntry widgets) $ 
--            do
--            updateEvalPointHandler
--        Gtk.onFocusOut (evalPointEntry widgets) $ \ _ -> 
--            do
--            updateEvalPointHandler
--            return False
--        where
--        updateEvalPointHandler =
--            do
--            -- indicate that we no longer wish to track the default point:  
--            atomically $ modifyTVar stateTV $ 
--                \ st -> st { favstTrackingDefaultEvalPt = False }
--            -- update the values for the new point:  
--            updateValueDisplayTV effFromDouble effEval widgets dynWidgetsRef fndataTVs stateTV
--
--    setHandlerPrintTXTButton =
--        Gtk.onClicked (printTXTButton widgets) $
--            do
--            (state, FnData fns) <- 
--                atomically $
--                    do
--                    state <- readTVar stateTV
--                    fns <- readTVar fndataTV
--                    return (state, fns)
--            putStrLn $
--                unlines $ map show $ fns
--
--    setHandlerExportPDFButton =
--        Gtk.onClicked (exportPDFButton widgets) $
--            do
--            (state, FnData fns, fnmeta) <- 
--                atomically $
--                    do
--                    state <- readTVar stateTV
--                    fndata <- readTVar fndataTV
--                    fnmeta <- readTVar fnmetaTV
--                    return (state, fndata, fnmeta)
--            let fnsActive = concat $ favstActiveFns state
--            let canvasParams = favstCanvasParams state
--            let fnsStyles = concat $ dataFnStyles fnmeta
--            maybeFilepath <- letUserChooseFileToSaveInto "PDF" "pdf"
--            case maybeFilepath of
--                Just filepath ->
--                    withPDFSurface filepath w h $ \ surface ->
--                        renderWith surface $
--                            drawFunctions sampleF effDraw effReal canvasParams state w h fnsActive (concat fns) fnsStyles
--                _ -> return ()
--            where
--            w = 360 :: Double -- in 1/72 inch TODO: ask user
--            h = 360 :: Double -- in 1/72 inch TODO: ask user
--
--    setHandlerExportSVGButton =
--        Gtk.onClicked (exportSVGButton widgets) $
--            do
--            (state, FnData fns, fnmeta) <- 
--                atomically $
--                    do
--                    state <- readTVar stateTV
--                    fndata <- readTVar fndataTV
--                    fnmeta <- readTVar fnmetaTV
--                    return (state, fndata, fnmeta)
--            let fnsActive = concat $ favstActiveFns state
--            let canvasParams = favstCanvasParams state
--            let fnsStyles = concat $ dataFnStyles fnmeta
--            maybeFilepath <- letUserChooseFileToSaveInto "SVG" "svg"
--            case maybeFilepath of
--                Just filepath ->
--                    withSVGSurface filepath w h $ \ surface ->
--                        renderWith surface $
--                            drawFunctions sampleF effDraw effReal canvasParams state w h fnsActive (concat fns) fnsStyles
--                _ -> return ()
--            where
--            w = 360 :: Double -- in 1/72 inch TODO: ask user
--            h = 360 :: Double -- in 1/72 inch TODO: ask user
--
--    setHandlerExportPNGButton =
--        Gtk.onClicked (exportPNGButton widgets) $
--            do
--            (state, FnData fns, fnmeta) <- 
--                atomically $
--                    do
--                    state <- readTVar stateTV
--                    fndata <- readTVar fndataTV
--                    fnmeta <- readTVar fnmetaTV
--                    return (state, fndata, fnmeta)
--            let fnsActive = concat $ favstActiveFns state
--            let canvasParams = favstCanvasParams state
--            let fnsStyles = concat $ dataFnStyles fnmeta
--            maybeFilepath <- letUserChooseFileToSaveInto "PNG" "png"
--            case maybeFilepath of
--                Just filepath ->
--                    withImageSurface FormatARGB32 wI hI $ \ surface ->
--                        do
--                        renderWith surface $
--                            drawFunctions sampleF effDraw effReal canvasParams state wD hD fnsActive (concat fns) fnsStyles
--                        surfaceWriteToPNG surface filepath
--                _ -> return ()
--            where
--            wI = 1024 :: Int -- in pixels TODO: ask user
--            hI = 1024 :: Int -- in pixels TODO: ask user
--            wD = realToFrac wI
--            hD = realToFrac hI
--
    
{-
    initial plans for the GUI:

      controller ODE IVP:
        fields showing and allowing editing of maxDeg and maxSize
        label showing the list of numbers of picard iterations in each segment
        plotting the best enclosure for each segment
          black for previous segments
          red or green for current segment
        button to perform another Picard iteration on the current segment
        indicator whether inclusion within the previous enclosure has been detected
          if inclusion detected, plot in green, otherwise in red
        when inclusion is detected, plot also the improvement
        button to jump to the next segment
          active only when inclusion within the previous enclosure has been detected
        undo button
      controller hybrid/ODE IVP:
        indicator of the current segment
        button to perform another Picard iteration on the current segment
        indicator whether inclusion within the previous enclosure has been detected
        indicator of the possible sequences of events that have been dealt with
        indicator of the possible types of the next event (including the type "none")
        button to jump to the next segment
          active only when inclusion within the previous enclosure has been detected
          and all events have been dealt with either explicitly or by recursion
        button to refocus on a small segment around the first event
          not active if it is clear that there is no event in this segment
        button to add another event into the event sequence
          not active if it is clear that there is no further event in this small segment
        undo button
-}
