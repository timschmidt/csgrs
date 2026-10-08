{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE StandaloneDeriving #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE UndecidableInstances #-}
{-# LANGUAGE ScopedTypeVariables #-}
{-|
    Module      :  Numeric.AERN.RmToRn.Plot.FnView.WatchData
    Description :  dynamically respond to a change in the function data
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Internal module for FnView.
    Dynamically respond to a change in the function data. 
-}
module Numeric.AERN.RmToRn.Plot.FnView.WatchData
--(
--)
where

import Numeric.AERN.RmToRn.Plot.FnView.FnData
import Numeric.AERN.RmToRn.Plot.FnView.State
import Numeric.AERN.RmToRn.Plot.FnView.Layout

import Numeric.AERN.RmToRn.Plot.Params

import Numeric.AERN.RmToRn.Domain
import Numeric.AERN.RmToRn.Evaluation

import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut
--import Numeric.AERN.RealArithmetic.RefinementOrderRounding.OpsImplicitEffort

import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as ArithUpDn

import qualified Numeric.AERN.RefinementOrder as RefOrd
--import Numeric.AERN.RefinementOrder.OpsImplicitEffort

import qualified Graphics.UI.Gtk as Gtk
import Graphics.UI.Gtk (AttrOp((:=)))

import qualified Control.Concurrent as Concurrent
import Control.Concurrent.STM (STM, TVar, atomically, retry, readTVar, writeTVar) -- as STM
import Data.IORef

dataWatchThread ::
    (ArithInOut.RoundedReal (Domain f),
     CanEvaluate f,
     RefOrd.PartialComparison (Domain f),
     Show (Domain f))
    =>
    f ->
    ArithInOut.RoundedRealEffortIndicator (Domain f) ->
    EvaluationEffortIndicator f -> 
    Widgets ->
    IORef FnViewDynWidgets ->
    (TVar (FnData f),
     TVar (FnMetaData f)) ->
    (TVar (FnViewState f)) ->
    IO ()
dataWatchThread 
        (sampleF :: f)
        effReal effEval 
        widgets dynWidgetsRef fndataTVs@(fndataTV, fnmetaTV) stateTV =
    do
    (fndatas@(_, fnmeta), state) <- atomically $ readAll3TVars fndataTVs stateTV
    -- initial update:
    action DataChangeMeta fndatas state
    -- enter watch loop:
    dataWatchLoop fnmeta
    where
    dataWatchLoop fnmetaOld =
        do
--        putStrLn "dataWatchLoop: start"
        ((dataChange, fndatas@(_, fnmeta)), state) <- waitForChange fnmetaOld
--        putStrLn "dataWatchLoop: fnmeta change detected"
        _ <- Gtk.timeoutAdd (do { action dataChange fndatas state; return False }) 10
        Concurrent.yield
        case dataChange of
            DataChangeClose -> return ()
            _ -> dataWatchLoop fnmeta
    action DataChangeClose (_fndata, _fnmeta) _state =
        do
        return ()
    action DataChangeMeta (fndata, fnmeta) state =
        do
--        putStrLn $ "DataChangeMeta"
--        putStrLn $ show $ dataFAs fndata
        updateFnWidgets toDbl widgets dynWidgetsRef fnmeta state fndataTVs stateTV
        let initialisedState = initState effReal (fndata, fnmeta)
        atomically $ writeTVar stateTV initialisedState
        updateView sampleF effReal effEval widgets dynWidgetsRef initialisedState (fndata, fnmeta)
    action DataChangeFn (fndata, fnmeta) state =
        do
--        putStrLn $ "DataChangeFn"
--        putStrLn $ show $ dataFAs fndata
        case favstTrackingDefaultEvalPt state of
            True -> 
                Gtk.entrySetText (wgt_evalPointEntry widgets) $ 
                    show $ (dataDefaultEvalPoint fnmeta)
            False -> 
                return () 
        updateFnWidgets toDbl widgets dynWidgetsRef fnmeta state fndataTVs stateTV
        updateView sampleF effReal effEval widgets dynWidgetsRef state (fndata, fnmeta)
    action DataChangeDefaultEvalPoint (fndata, fnmeta) state =
        do
--        putStrLn $ "DataChangeDefaultEvalPoint"
        case favstTrackingDefaultEvalPt state of
            True ->
                do
                Gtk.entrySetText (wgt_evalPointEntry widgets) $ 
                    show $ (dataDefaultEvalPoint fnmeta) 
                updateView sampleF effReal effEval widgets dynWidgetsRef state (fndata, fnmeta)
            False -> return ()
    waitForChange fnmetaOld =
        do
        waitFC
        where
        waitFC =
            atomically $
            do
            fnmeta <- readTVar fnmetaTV
            (change, fndatas) <- 
                case fnmeta == fnmetaOld of
                    True -> retry
                    False ->
                        case dataFnsUpdated fnmeta of
                            True ->
                                do
                                fndata <- readTVar fndataTV
                                let change = returnChange fnmeta fndata
                                let fnmetaNew = fnmeta { dataFnsUpdated = False }
                                writeTVar fnmetaTV fnmetaNew
                                return (change, (fndata, fnmetaNew))
                            False ->
                                do
                                let change = returnChange fnmeta undefined
                                return (change, (undefined, fnmeta))
            state <- readTVar stateTV
            return ((change, fndatas), state) 
        returnChange fnmeta _fndata
            | dataDestroyed fnmeta =
                DataChangeClose
            | namesChanged || fnStylesChanged = 
                DataChangeMeta
            | dataFnsUpdated fnmeta =
                DataChangeFn
            | evalPtChanged =
                DataChangeDefaultEvalPoint
            | otherwise =
                error $ 
                    "AERN internal error: FnView: returnChange: cannot detect type of change:\n" 
                        ++ show fnmeta ++ "\n" ++ show fnmetaOld  
            where
            changed :: (Eq t) => (FnMetaData f -> t) -> Bool 
            changed field = 
                field fnmeta /= field fnmetaOld
            namesChanged =
                domNameChanged || resNamesChanged || fnNamesChanged
            domNameChanged = changed dataDomName
            resNamesChanged = changed dataFnNames
            fnNamesChanged = changed dataFnGroupNames 
            fnStylesChanged = changed dataFnStyles 
            evalPtChanged = changed (toDbl . dataDefaultEvalPoint)
    toDbl :: (Domain f) -> Double
    toDbl a = d
        where
        (Just d) = ArithUpDn.convertUpEff effToDouble 0 a
    effToDouble = ArithInOut.rrEffortToDouble sampleDom effReal
    sampleDom = getSampleDomValue sampleF
    
    
data DataChange 
    = DataChangeClose -- signals the end...
    | DataChangeMeta -- all change
    | DataChangeFn -- only fn & eval point may have changed
    | DataChangeDefaultEvalPoint -- only eval point has changed


{-| 
    update the values shown against dimension names
-}
updateValueDisplay ::
    (ArithInOut.Convertible Double (Domain f),
     RefOrd.PartialComparison (Domain f),
     CanEvaluate f, 
     Show (Domain f))
    =>
    ArithInOut.ConvertEffortIndicator Double (Domain f) ->
    EvaluationEffortIndicator f ->
    Widgets ->
    IORef FnViewDynWidgets ->
    (FnViewState f) ->
    ((FnData f),
      FnMetaData f) ->
    IO ()
updateValueDisplay effFromDouble effEval widgets dynWidgetsRef _state (fndata, _) =
    do
    evalPointText <- Gtk.entryGetText $ wgt_evalPointEntry widgets
    let maybeFnValueTexts = getFnValueTexts evalPointText 
    case maybeFnValueTexts of
        Nothing -> do return () -- putStrLn $ "failed to parse eval point: " ++ evalPointText
        Just fnValueTexts ->
            do
            dynWidgets <- readIORef dynWidgetsRef
            mapM_ (mapM $ uncurry Gtk.labelSetText) $ 
                zipWith zip (dynwgt_valueLabels dynWidgets) fnValueTexts
            return ()
    where
    getFnValueTexts evalPointText =
        fmap eval $ readMaybe evalPointText
        where
        eval :: Double -> [[String]] 
        eval evalPointD =
            map (map $ getDimValue1Or2) $ dataFns fndata
            where
            getDimValue1Or2 (GraphPlotFn fns, plotVar) =
                show $ 
                    map (getDimValue plotVar) $ 
                        filter (pointInDom plotVar) fns
            getDimValue1Or2 (ParamPlotFns fnPairs _, plotVar) =
                show $ 
                    map (getDimValue2 plotVar) $ 
                        filter (pointInDom2 plotVar) fnPairs
            pointInDom2 var (fnX, _fnY) = pointInDom var fnX
            pointInDom var fn = 
                case lookupVar dombox var of
                    Just dom -> (dom RefOrd.|<=? evalPointDom) == Just True
                    Nothing -> False
                where
                dombox = getDomainBox fn
                sampleDom = getSampleDomValue fn
                evalPointDom =
                    ArithInOut.convertOutEff effFromDouble sampleDom evalPointD
            getDimValue2 plotVar  (fnX, fnY) =
                (getDimValue plotVar fnX, getDimValue plotVar fnY) 
            getDimValue plotVar fn =
                evalAtPointOutEff effEval evalDombox fn
                where
                evalDombox =
                    insertVar plotVar evalPointDom dombox 
                sampleDom = getSampleDomValue fn
                dombox = getDomainBox fn
                evalPointDom =
                    ArithInOut.convertOutEff effFromDouble sampleDom evalPointD

updateValueDisplayTV :: 
      (Show (Domain f), 
       ArithInOut.Convertible Double (Domain f),
       RefOrd.PartialComparison (Domain f),
       CanEvaluate f) 
      =>
      ArithInOut.ConvertEffortIndicator Double (Domain f)
      -> EvaluationEffortIndicator f
      -> Widgets
      -> IORef FnViewDynWidgets
      -> (TVar (FnData f), TVar (FnMetaData f))
      -> TVar (FnViewState f)
      -> IO ()
updateValueDisplayTV effFromDouble effEval widgets dynWidgetsRef fndataTVs stateTV =
    do
--    putStrLn "updateValueDisplayTVERFA"
    (fndatas, state) <- atomically $ readAll3TVars fndataTVs stateTV
    updateValueDisplay effFromDouble effEval widgets dynWidgetsRef state fndatas    


updateZoomWidgets :: 
   Show a 
   =>
   (Domain f -> a) -> 
   Widgets -> 
   FnViewState f -> 
   IO ()
updateZoomWidgets toDbl widgets state =
    case coordSystem of 
        CoordSystemLogSqueeze _ ->
            do
            Gtk.comboBoxSetActive (wgt_coorSystemCombo widgets) 0 
            Gtk.editableSetEditable (wgt_zoomEntry widgets) False
            Gtk.editableSetEditable (wgt_centreXEntry widgets) False
            Gtk.editableSetEditable (wgt_centreYEntry widgets) False
            Gtk.entrySetText (wgt_zoomEntry widgets) ""
            Gtk.entrySetText (wgt_centreXEntry widgets) ""
            Gtk.entrySetText (wgt_centreYEntry widgets) ""
        (CoordSystemLinear (Rectangle _hi _lo _l _r)) ->
            do
            Gtk.comboBoxSetActive (wgt_coorSystemCombo widgets) 1 
            Gtk.editableSetEditable (wgt_zoomEntry widgets) True
            Gtk.editableSetEditable (wgt_centreXEntry widgets) True
            Gtk.editableSetEditable (wgt_centreYEntry widgets) True
            Gtk.entrySetText (wgt_zoomEntry widgets) $ show zoomPercent
            Gtk.entrySetText (wgt_centreXEntry widgets) $ show cX
            Gtk.entrySetText (wgt_centreYEntry widgets) $ show cY
    where
    zoomPercent = favstZoomPercent state
    cX = toDbl cXDF
    cY = toDbl cYDF
    (cXDF,cYDF) = favstPanCentre state
    coordSystem = cnvprmCoordSystem $ favstCanvasParams state

updateAxesWidgets :: Widgets -> FnViewState f -> IO ()
updateAxesWidgets widgets state =
    do
    Gtk.entrySetText (wgt_fontSizeEntry widgets) (show fontSize)
    Gtk.toggleButtonSetActive (wgt_showAxesCheckbutton widgets) showAxes
    where
    fontSize = case maybeFontSize of Just sz -> sz; _ -> 0
    maybeFontSize = cnvprmShowSampleValuesFontSize $ favstCanvasParams state
    showAxes = cnvprmShowAxes $ favstCanvasParams state

{-|
    Reconfigure the GUI to show variable names appropriate
    for the given fnmeta.
-}
updateFnWidgets ::
    (Domain f -> Double) ->
    Widgets ->
    IORef FnViewDynWidgets ->
    FnMetaData f ->
    FnViewState f ->
    ((TVar (FnData f)),
      TVar (FnMetaData f)) ->
    (TVar (FnViewState f)) ->
    IO ()
updateFnWidgets toDbl widgets dynWidgetsRef fnmeta state _fndataTVs stateTV =
    do
    -- update the name of the domain variable:
    Gtk.labelSetText (wgt_domVarLabel widgets) $ domName ++ "="
    -- set the default evaluation point:
    Gtk.entrySetText (wgt_evalPointEntry widgets) $ show $ toDbl $ dataDefaultEvalPoint fnmeta
    -- remove any old dim rows from dimTable:
    children <- Gtk.containerGetChildren table
    mapM_ (Gtk.containerRemove table) children  
    -- add new dim rows:
    Gtk.tableResize table (fnRowCount + 1) 3
    -- fill each row with widgets and return all newly created value entries:
    let activeFns = favstActiveFns state
    valueLabels <- addGroupLabels 0 $ zip3 [0..] grpNames $ zipWith zip fnNames activeFns
    -- layout the table:
    Gtk.widgetShowAll table
    Gtk.containerResizeChildren table
    -- remember valueEntries for later use: 
    modifyIORef dynWidgetsRef $ \ dynWidgets ->
        dynWidgets
        {
            dynwgt_valueLabels = valueLabels
        } 
    where
    table = wgt_dimTable widgets
    domName = dataDomName fnmeta
    grpNames = dataFnGroupNames fnmeta
    fnNames = dataFnNames fnmeta
    fnRowCount = (length grpNames) + (sum $ map length fnNames)
    addGroupLabels _nextRowNo [] = return []
    addGroupLabels nextRowNo ((grpNo, grpName, fnNamesActive):rest) =
        do
        -- add a function label:
        grpLabel <- Gtk.labelNew (Just grpName)        
        Gtk.tableAttachDefaults table grpLabel 1 2 nextRowNo (nextRowNo + 1)
        Gtk.set table [ Gtk.tableChildXOptions grpLabel := []]
        Gtk.miscSetAlignment grpLabel 0 0.5
        -- add all result labels:
        labels <- addFnLabels (nextRowNo + 1) (grpNo, grpName) fnNamesActive
        -- recurse for the following functions:
        restLabels <- addGroupLabels (nextRowNo + 1 + (length fnNamesActive)) rest
        return $ labels : restLabels
        
    addFnLabels nextRowNo0 (grpNo, _grpName) fnNamesActive =
        do
        mapM addFnLabel $ zip3 [nextRowNo0..] [0..] fnNamesActive
        where
        addFnLabel (nextRowNo, fnNo, (fnName, isActive0)) =
            do
            -- add variable label:
            fnLabel <- Gtk.labelNew (Just labelText)
            Gtk.tableAttachDefaults table fnLabel 1 2 nextRowNo nextRowNoPlus1
            Gtk.miscSetAlignment fnLabel 0 0.5
            -- add value label:
            valLabel <- Gtk.labelNew (Nothing :: Maybe String)
            Gtk.tableAttachDefaults table valLabel 2 3 nextRowNo nextRowNoPlus1
            -- add a check button:
            showCheckButton <- Gtk.checkButtonNew
            Gtk.tableAttachDefaults table showCheckButton 0 1 nextRowNo nextRowNoPlus1
            -- make it ticked:
            Gtk.toggleButtonSetActive showCheckButton isActive0
            -- give the check button a handler:
            _ <- Gtk.onToggled showCheckButton $
                do
                isActive <- Gtk.toggleButtonGetActive showCheckButton
                _state <- atomically $ modifyTVar stateTV $ updateFnActive grpNo fnNo isActive
--                fndatas <- atomically $ readBothTVars fndataTVs
                Gtk.widgetQueueDraw (wgt_canvas widgets)
                return ()
            return valLabel
            where
            labelText = " [" ++ show fnNo ++ "]" ++ fnName ++ "(" ++ domName ++ ")="
            nextRowNoPlus1 = nextRowNo + 1
        
updateView ::
    (ArithInOut.RoundedReal (Domain f),
     CanEvaluate f, 
     Show (Domain f))
    =>
    f ->
    ArithInOut.RoundedRealEffortIndicator (Domain f) ->
    EvaluationEffortIndicator f ->
    Widgets ->
    IORef FnViewDynWidgets ->
    (FnViewState f) ->
    ((FnData f),
     FnMetaData f) ->
    IO ()
updateView (sampleF :: f) effReal effEval widgets dynWidgetsRef state (fndata, fnmeta) =
    do
    updateValueDisplay effFromDouble effEval widgets dynWidgetsRef state (fndata, fnmeta)
    updateZoomWidgets toDbl widgets state
    updateAxesWidgets widgets state
    Gtk.widgetQueueDraw (wgt_canvas widgets)
    return ()
    where
    toDbl :: (Domain f) -> Double
    toDbl a = d
        where
        (Just d) = ArithUpDn.convertUpEff effToDouble 0 a
    effToDouble = ArithInOut.rrEffortToDouble sampleDom effReal
    effFromDouble = ArithInOut.rrEffortFromDouble sampleDom effReal
    sampleDom = getSampleDomValue sampleF
    


-- auxiliaries:
readMaybe :: (Read a) => String -> Maybe a
readMaybe s =
    case reads s of
        [] -> Nothing
        (val,_) : _ -> Just val

-- stm:

modifyTVar :: TVar b -> (b -> b) -> STM b
modifyTVar tv update =
    do
    value <- readTVar tv
    let newValue = update value
    writeTVar tv newValue
    return newValue

readAll3TVars :: 
    (TVar a1, TVar a2) -> 
    TVar a -> 
    STM ((a1, a2), a)
readAll3TVars fndataTVs stateTV =
    do
    state <- readTVar stateTV
    fndatas <- readBothTVars fndataTVs 
    return (fndatas, state)

readBothTVars :: 
    (TVar a, TVar a1) -> 
    STM (a, a1)
readBothTVars (fndataTV, fnmetaTV) =
    do
    fndata <- readTVar fndataTV
    fnmeta <- readTVar fnmetaTV
    return (fndata, fnmeta)

    