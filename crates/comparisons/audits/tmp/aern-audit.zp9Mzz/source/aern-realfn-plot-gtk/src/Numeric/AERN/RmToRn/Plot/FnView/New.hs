{-# OPTIONS_GHC -fno-warn-unused-do-bind #-}
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
    Module      :  Numeric.AERN.RmToRn.Plot.FnView.New
    Description :  widget constructor
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Widget constructor
-}
module Numeric.AERN.RmToRn.Plot.FnView.New
where

import Numeric.AERN.RmToRn.Plot.FnView.FnData
import Numeric.AERN.RmToRn.Plot.FnView.State
import Numeric.AERN.RmToRn.Plot.FnView.Layout
import Numeric.AERN.RmToRn.Plot.FnView.WatchData
import Numeric.AERN.RmToRn.Plot.FnView.Canvas

import Numeric.AERN.RmToRn.Plot.Params
import Numeric.AERN.RmToRn.Plot.CairoDrawable (CairoDrawableFn(..))

import Numeric.AERN.RmToRn.Domain
import Numeric.AERN.RmToRn.Evaluation

import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut
--import Numeric.AERN.RealArithmetic.RefinementOrderRounding.OpsImplicitEffort

import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as ArithUpDn

import qualified Numeric.AERN.RefinementOrder as RefOrd
--import Numeric.AERN.RefinementOrder.OpsImplicitEffort

import Graphics.Rendering.Cairo as Cairo
import qualified Graphics.UI.Gtk as Gtk
--import Graphics.UI.Gtk (AttrOp((:=)))
import qualified Graphics.UI.Gtk.Gdk.EventM as GdkEv

import Control.Concurrent (forkIO)
import Control.Concurrent.STM (TVar, newTVar, atomically, readTVar, writeTVar) -- as STM
import Data.IORef

import qualified System.FilePath as FilePath

{-|
    Create a new viewer linked to the given data variable.
-}
new ::
    (CairoDrawableFn f,
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     CanEvaluate f,
     RefOrd.PartialComparison (Domain f),
     Show f, Show (Var f), Show (Domain f))
    =>
    f {-^ sample value -} ->
    (CairoDrawFnEffortIndicator f) ->
    ArithInOut.RoundedRealEffortIndicator (Domain f) ->
    EvaluationEffortIndicator f ->
    (TVar (FnData f),
     TVar (FnMetaData f)) ->
    (Maybe Gtk.Window) {- ^ parent wgt_window -} -> 
    IO Gtk.Window
new (sampleF :: f) effDraw effReal effEval fndataTVs@(fndataTV, fnmetaTV) _maybeParentWindow =
    do
    -- create initial state objects:
    (stateTV, state, fnmeta) <- atomically $
        do
        fndata <- readTVar fndataTV
        fnmeta <- readTVar fnmetaTV
        let state = initState effReal (fndata, fnmeta)
        stateTV <- newTVar state
        return (stateTV, state, fnmeta)
    -- create most widgets:
    widgetsPre <- loadGlade (FilePath.combine GLADE_DIR "FnView.glade")
    -- create plotting wgt_canvas:
    widgets <- makeCanvas sampleF effDraw effReal widgetsPre fndataTVs stateTV
    -- add dynamic function label widgets:
    dynWidgetsRef <- newIORef initFnViewDynWidgets
    updateFnWidgets toDbl widgets dynWidgetsRef fnmeta state fndataTVs stateTV
    -- initialise controls according to state:
    updateZoomWidgets toDbl widgets state
    updateAxesWidgets widgets state
    -- attach handlers to widgets:
    Gtk.onDestroy (wgt_window widgets) $
        do
        atomically $ modifyTVar fnmetaTV $ \fnmeta2 -> fnmeta2 { dataDestroyed = True }
        Gtk.mainQuit
    setHandlers sampleF effDraw effReal effEval widgets dynWidgetsRef fndataTVs stateTV
    -- start thread that reponds to changes in fndataTVs:
    forkIO $
        dataWatchThread 
            sampleF effReal effEval 
            widgets dynWidgetsRef fndataTVs stateTV
    -- make the window visible
    Gtk.widgetShowAll $ wgt_window widgets
    --
    return $ wgt_window widgets
    where
    sampleDom = getSampleDomValue sampleF
    effToDouble = ArithInOut.rrEffortToDouble sampleDom effReal
    toDbl :: (Domain f) -> Double
    toDbl a = d
        where
        (Just d) = ArithUpDn.convertUpEff effToDouble 0 a

setHandlers :: 
    (CairoDrawableFn f,
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     CanEvaluate f, 
     Show f, Show (Var f), Show (Domain f))
    =>
    f {-^ sample value -} ->
    (CairoDrawFnEffortIndicator f) ->
    ArithInOut.RoundedRealEffortIndicator (Domain f) ->
    EvaluationEffortIndicator f ->
    Widgets -> 
    IORef FnViewDynWidgets -> 
    (TVar (FnData f), TVar (FnMetaData f)) -> 
    TVar (FnViewState f) -> 
    IO ()
setHandlers (sampleF :: f) effDraw effReal effEval widgets dynWidgetsRef fndataTVs@(fndataTV, fnmetaTV) stateTV =
    do
    setHandlerAxesSetting
    setHandlerCoordSystem
    setHandlerZoomAndPanEntries
    setHandlerPanByMouse
    setHandlerZoomByMouse
    setHandlerDefaultEvalPointButton
    setHandlerEvalPointEntry
    setHandlerPrintTXTButton
    setHandlerExportPDFButton
    setHandlerExportSVGButton
    setHandlerExportPNGButton

--    putStrLn $ "setHandlers: " ++ (show $ cnvprmCoordSystem $ favstCanvasParams state)
    return ()
    where        
    toDbl :: (Domain f) -> Double
    toDbl a = d
        where
        (Just d) = ArithUpDn.convertUpEff effToDouble 0 a
    effToDouble = ArithInOut.rrEffortToDouble sampleDom effReal
    effFromDouble = ArithInOut.rrEffortFromDouble sampleDom effReal
    sampleDom = getSampleDomValue sampleF

    setHandlerAxesSetting =
        do
        Gtk.onToggled (wgt_showAxesCheckbutton widgets) showAxesHandler   
        Gtk.onEntryActivate (wgt_fontSizeEntry widgets) (fontSizeHandler ())
        Gtk.onFocusOut (wgt_fontSizeEntry widgets) (\ _e -> fontSizeHandler False)
        where
        showAxesHandler =
            do
            showAxes <- Gtk.toggleButtonGetActive (wgt_showAxesCheckbutton widgets)
            _ <- atomically $ modifyTVar stateTV $ updateShowAxes showAxes
            Gtk.widgetQueueDraw (wgt_canvas widgets)
        fontSizeHandler :: t -> IO t
        fontSizeHandler returnValue =
            do
            fontSizeS <- Gtk.entryGetText (wgt_fontSizeEntry widgets)
            case reads fontSizeS of
                ((fontSize, _): _)
                    | fontSize > 0 -> 
                        do
                        _ <- atomically $ modifyTVar stateTV $ updateFontSize (Just fontSize)
                        Gtk.widgetQueueDraw (wgt_canvas widgets)
                    | fontSize == 0 ->
                        do
                        _ <- atomically $ modifyTVar stateTV $ updateFontSize Nothing
                        Gtk.widgetQueueDraw (wgt_canvas widgets)
                _ -> return ()
            return returnValue
        
    setHandlerCoordSystem =
        do
--        Gtk.on (wgt_coorSystemCombo widgets) Gtk.changed resetZoomPanFromCoordSystem
        -- disable the coord system combo because only one coord system is currently supported:
        Gtk.widgetSetSensitivity (wgt_coorSystemCombo widgets) False   
        Gtk.onClicked (wgt_defaultZoomPanButton widgets) resetZoomPanFromCoordSystem
        where
        resetZoomPanFromCoordSystem =
            do
            maybeCSysIx <- Gtk.comboBoxGetActive (wgt_coorSystemCombo widgets)
--            putStrLn $ "resetZoomPanFromCoordSystem: maybeCSysIx = " ++ show maybeCSysIx
            case maybeCSysIx of
                -1 -> return ()
                ix ->
                    do
                    state <-
                        atomically $
                            do
                            fnmeta <- readTVar fnmetaTV
                            let coordSystem = case ix of
                                    0 -> CoordSystemLogSqueeze sampleDom
                                    1 -> 
                                        linearCoordsWithZoom effReal defaultZoom (getFnExtents fnmeta)
                                    _ -> error "FnView: unsupported choice of coordSystem"
                            state <- modifyTVar stateTV $ 
                                updatePanCentreCoordSystem (getDefaultCentre effReal fnmeta) coordSystem
                            return state
                    Gtk.widgetQueueDraw (wgt_canvas widgets)
                    updateZoomWidgets toDbl widgets state
    
    setHandlerZoomAndPanEntries =
        do
        Gtk.onEntryActivate (wgt_zoomEntry widgets) (zoomHandler ())
        Gtk.onFocusOut (wgt_zoomEntry widgets) (\ _e -> zoomHandler False)
        Gtk.onEntryActivate (wgt_centreXEntry widgets) (zoomHandler ())
        Gtk.onFocusOut (wgt_centreXEntry widgets) (\ _e -> zoomHandler False)
        Gtk.onEntryActivate (wgt_centreYEntry widgets) (zoomHandler ())
        Gtk.onFocusOut (wgt_centreYEntry widgets) (\ _e -> zoomHandler False)
        where
        zoomHandler :: t -> IO t
        zoomHandler returnValue =
            do
            zoomS <- Gtk.entryGetText (wgt_zoomEntry widgets)
            centreXS <- Gtk.entryGetText (wgt_centreXEntry widgets)
            centreYS <- Gtk.entryGetText (wgt_centreYEntry widgets)
            case (reads zoomS, reads centreXS, reads centreYS)  of
                ([(zoomPercent,"")], [(centreXD,"")], [(centreYD,"")]) -> 
                    atomically $
                        do
                        fnmeta <- readTVar fnmetaTV
                        modifyTVar stateTV $ 
                            updateZoomPanCentreCoordSystem zoomPercent (centreX, centreY) $ 
                                linearCoordsWithZoomAndCentre effReal zoomPercent (centreX, centreY) $
                                    getFnExtents fnmeta 
                        return ()
                    where
                    centreX = ArithInOut.convertOutEff effFromDouble sampleDom (centreXD :: Double)
                    centreY = ArithInOut.convertOutEff effFromDouble sampleDom (centreYD :: Double)
                wrongParseResults ->
                    do
                    putStrLn $ "zoomHandler: parse error: " ++ show wrongParseResults 
                    return ()
--            putStrLn $ "zoomHandler"
            Gtk.widgetQueueDraw (wgt_canvas widgets)
            return returnValue

    setHandlerPanByMouse =
        do
        -- a variable to keep track of position before each drag movement:
        panOriginTV <- atomically $ newTVar Nothing
        -- setup the wgt_canvas to receive various mouse events:
        Gtk.widgetAddEvents (wgt_canvas widgets)
            [Gtk.ButtonPressMask, Gtk.ButtonReleaseMask, Gtk.PointerMotionMask]
        -- what to do when the left mouse button is pressed:
        Gtk.on (wgt_canvas widgets) Gtk.buttonPressEvent $
            do
            button <- GdkEv.eventButton
            coords <- GdkEv.eventCoordinates
            case button of
                GdkEv.LeftButton ->
                    liftIO $
                        do
                        -- remember the position and indicate that dragging is underway:
                        atomically $ writeTVar panOriginTV (Just coords)
                        return ()
                _ -> return ()
            return False
        -- what to do when the left mouse button is released:
        Gtk.on (wgt_canvas widgets) Gtk.buttonReleaseEvent $
            do
            button <- GdkEv.eventButton 
            case button of
                GdkEv.LeftButton ->
                    liftIO $
                        do
                        -- indicate no dragging is underway:
                        atomically $ writeTVar panOriginTV Nothing
                        return ()
                _ -> return ()
            return False
        -- what to do when mouse moves:
        Gtk.on (wgt_canvas widgets) Gtk.motionNotifyEvent $
            do
            coords@(x,y) <- GdkEv.eventCoordinates
            liftIO $
                do
                -- update the dragging information variable:
                maybePanOrigin <- atomically $
                    do
                    maybePanOrigin <- readTVar panOriginTV
                    case maybePanOrigin of
                        Nothing ->
                            return maybePanOrigin
                        Just _ ->
                            do
                            writeTVar panOriginTV (Just coords)
                            return maybePanOrigin
                -- check whether dragging or not:
                case maybePanOrigin of
                    Nothing -> return ()
                    Just _panOrigin@(oX,oY) -> -- yes, dragging occurred
                        do
                        -- find out the size of the wgt_canvas:
                        (canvasX, canvasY) <- Gtk.widgetGetSize (wgt_canvas widgets)
                        -- recalculate the centre and coordinate bounds 
                        -- based on the drag amount relative to the size fo the wgt_canvas:
                        state <- atomically $ modifyTVar stateTV $
                            updateCentreByRatio
                                effReal
                                ((x - oX) / (int2dbl canvasX), 
                                 (y - oY) / (int2dbl canvasY))
--                        putStrLn $ "motionNotifyEvent: coords = " ++ show coords 
--                            ++ "; panOrigin = " ++ show panOrigin
--                            ++ "; (canvasX,canvasY) = " ++ show (canvasX,canvasY)
                        return ()
                        -- make sure the text in the zoom and pan entries are updated:
                        updateZoomWidgets toDbl widgets state
                        -- schedule the wgt_canvas for redraw:
                        Gtk.widgetQueueDraw (wgt_canvas widgets)
                        where
                        int2dbl :: Int -> Double
                        int2dbl = realToFrac
            return False
        return ()
            
    setHandlerZoomByMouse =
        do -- IO
        Gtk.widgetAddEvents (wgt_canvas widgets) [Gtk.ScrollMask]
        Gtk.on (wgt_canvas widgets) Gtk.scrollEvent $
            do -- ReaderTV
            scrollDirection <- GdkEv.eventScrollDirection
            liftIO $
                do -- IO
                state <- atomically $
                    do -- STM
                    state <- readTVar stateTV 
                    let zoomPercent = favstZoomPercent state
                    let newZoomPercent = case scrollDirection of
                                            GdkEv.ScrollUp ->  1.25 * zoomPercent
                                            GdkEv.ScrollDown -> 0.8 * zoomPercent
                                            _ -> zoomPercent
                    fnmeta <- readTVar fnmetaTV
                    state2 <- modifyTVar stateTV $ 
                        updateZoomPercentAndFnExtents effReal newZoomPercent $ getFnExtents fnmeta
                    return state2
                updateZoomWidgets toDbl widgets state
                Gtk.widgetQueueDraw (wgt_canvas widgets)
                return False
        return ()
            
    setHandlerDefaultEvalPointButton =
        Gtk.onClicked (wgt_defaultEvalPointButton widgets) $
            do
            (state, fnmeta) <- 
                atomically $
                    do
                    state <- readTVar stateTV
                    fnmeta <- readTVar fnmetaTV
                    return (state, fnmeta)
            case favstTrackingDefaultEvalPt state of
                False ->
                    do
                    Gtk.entrySetText (wgt_evalPointEntry widgets) $ 
                        show $ dataDefaultEvalPoint fnmeta
                    atomically $ modifyTVar stateTV $
                        \ st -> st { favstTrackingDefaultEvalPt = True }
                    updateValueDisplayTV effFromDouble effEval widgets dynWidgetsRef fndataTVs stateTV 
                True -> -- already tracking the default
                    return ()
    setHandlerEvalPointEntry =
        do
        Gtk.onEntryActivate (wgt_evalPointEntry widgets) $ 
            do
            updateEvalPointHandler
        Gtk.onFocusOut (wgt_evalPointEntry widgets) $ \ _ -> 
            do
            updateEvalPointHandler
            return False
        where
        updateEvalPointHandler =
            do
            -- indicate that we no longer wish to track the default point:  
            atomically $ modifyTVar stateTV $ 
                \ st -> st { favstTrackingDefaultEvalPt = False }
            -- update the values for the new point:  
            updateValueDisplayTV effFromDouble effEval widgets dynWidgetsRef fndataTVs stateTV

    setHandlerPrintTXTButton =
        Gtk.onClicked (wgt_printTXTButton widgets) $
            do
            (_state, FnData fns) <- 
                atomically $
                    do
                    state <- readTVar stateTV
                    fns <- readTVar fndataTV
                    return (state, fns)
            putStrLn $
                unlines $ map show $ fns

    setHandlerExportPDFButton =
        Gtk.onClicked (wgt_exportPDFButton widgets) $
            do
            (state, FnData fns, fnmeta) <- 
                atomically $
                    do
                    state <- readTVar stateTV
                    fndata <- readTVar fndataTV
                    fnmeta <- readTVar fnmetaTV
                    return (state, fndata, fnmeta)
            let fnsActive = concat $ favstActiveFns state
            let canvasParams = favstCanvasParams state
            let fnsStyles = concat $ dataFnStyles fnmeta
            maybeFilepath <- letUserChooseFileToSaveInto "PDF" "pdf"
            case maybeFilepath of
                Just filepath ->
                    withPDFSurface filepath w h $ \ surface ->
                        renderWith surface $
                            drawFunctions sampleF effDraw effReal canvasParams w h fnsActive (concat fns) fnsStyles
                _ -> return ()
            where
            w = 360 :: Double -- in 1/72 inch TODO: ask user
            h = 360 :: Double -- in 1/72 inch TODO: ask user

    setHandlerExportSVGButton =
        Gtk.onClicked (wgt_exportSVGButton widgets) $
            do
            (state, FnData fns, fnmeta) <- 
                atomically $
                    do
                    state <- readTVar stateTV
                    fndata <- readTVar fndataTV
                    fnmeta <- readTVar fnmetaTV
                    return (state, fndata, fnmeta)
            let fnsActive = concat $ favstActiveFns state
            let canvasParams = favstCanvasParams state
            let fnsStyles = concat $ dataFnStyles fnmeta
            maybeFilepath <- letUserChooseFileToSaveInto "SVG" "svg"
            case maybeFilepath of
                Just filepath ->
                    withSVGSurface filepath w h $ \ surface ->
                        renderWith surface $
                            drawFunctions sampleF effDraw effReal canvasParams w h fnsActive (concat fns) fnsStyles
                _ -> return ()
            where
            w = 360 :: Double -- in 1/72 inch TODO: ask user
            h = 360 :: Double -- in 1/72 inch TODO: ask user

    setHandlerExportPNGButton =
        Gtk.onClicked (wgt_exportPNGButton widgets) $
            do
            (state, FnData fns, fnmeta) <- 
                atomically $
                    do
                    state <- readTVar stateTV
                    fndata <- readTVar fndataTV
                    fnmeta <- readTVar fnmetaTV
                    return (state, fndata, fnmeta)
            let fnsActive = concat $ favstActiveFns state
            let canvasParams = favstCanvasParams state
            let fnsStyles = concat $ dataFnStyles fnmeta
            maybeFilepath <- letUserChooseFileToSaveInto "PNG" "png"
            case maybeFilepath of
                Just filepath ->
                    withImageSurface FormatARGB32 wI hI $ \ surface ->
                        do
                        renderWith surface $
                            drawFunctions sampleF effDraw effReal canvasParams wD hD fnsActive (concat fns) fnsStyles
                        surfaceWriteToPNG surface filepath
                _ -> return ()
            where
            wI = 1024 :: Int -- in pixels TODO: ask user
            hI = 1024 :: Int -- in pixels TODO: ask user
            wD = realToFrac wI
            hD = realToFrac hI


letUserChooseFileToSaveInto :: 
    [Char] -> [Char] -> IO (Maybe FilePath)
letUserChooseFileToSaveInto formatName extension =
    do
    chooser <- Gtk.fileChooserDialogNew 
        (Just $ "Save a plot in " ++ formatName) 
        Nothing -- no parent wgt_window specified 
        Gtk.FileChooserActionSave 
        [("Cancel", Gtk.ResponseCancel),
         ("Save as " ++ formatName, Gtk.ResponseAccept)]
    filter2 <- Gtk.fileFilterNew
    Gtk.fileFilterAddPattern filter2 $ "*." ++ extension
    Gtk.fileChooserSetFilter chooser filter2
    response <- Gtk.dialogRun chooser
    maybeFilename <- Gtk.fileChooserGetFilename chooser
    Gtk.widgetDestroy chooser
    case response of
        Gtk.ResponseAccept -> return maybeFilename
        _ -> return Nothing
    
plotToPDFFile :: 
    (Show (Domain f), ArithInOut.RoundedReal (Domain f),
     HasDomainBox f, CairoDrawableFn f) =>
    f
    -> CairoDrawFnEffortIndicator f
    -> ArithInOut.RoundedRealEffortIndicator (Domain f)
    -> CanvasParams (Domain f)
    -> Double
    -> Double
    -> [Bool]
    -> [[(GraphOrParamPlotFn f, Var f)]]
    -> [FnPlotStyle]
    -> FilePath
    -> IO ()
plotToPDFFile sampleF effDraw effReal canvasParams w h fnsActive fns fnsStyles filepath =
    withPDFSurface filepath w h $ \ surface ->
        renderWith surface $
            drawFunctions sampleF effDraw effReal canvasParams w h fnsActive (concat fns) fnsStyles
    