{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE StandaloneDeriving #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE UndecidableInstances #-}
{-|
    Module      :  Numeric.AERN.RmToRn.Plot.FnView.Layout
    Description :  layout of the FnView widget
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Internal module for FnView.
    Layout of the FnView widget. 
-}
module Numeric.AERN.RmToRn.Plot.FnView.Layout
--(
--)
where

--import Numeric.AERN.RmToRn.Plot.FnView.State

--import Numeric.AERN.RmToRn.Plot.Params

import qualified Graphics.UI.Gtk as Gtk

import System.Directory (doesFileExist)

loadGlade :: 
    FilePath ->
    IO Widgets
loadGlade gladeFileName =
    do
    gotGladeFile <- doesFileExist gladeFileName
    case gotGladeFile of
        True -> return ()
        False -> error $ "AERN: RmToRn.Plot.FnView: glade file " ++ gladeFileName ++ " not found" 
    gui <- Gtk.builderNew
    Gtk.builderAddFromFile gui gladeFileName
    window <- Gtk.builderGetObject gui Gtk.castToWindow "window1"
    canvasAlignment <- Gtk.builderGetObject gui Gtk.castToAlignment "canvasAlignment1"
    coorSystemCombo <- Gtk.builderGetObject gui Gtk.castToComboBox "coorSystemCombo1"
    showAxesCheckbutton <- Gtk.builderGetObject gui Gtk.castToCheckButton "showAxesCheckbutton1"
    fontSizeEntry <- Gtk.builderGetObject gui Gtk.castToEntry "fontSizeEntry1"
    evalPointEntry <- Gtk.builderGetObject gui Gtk.castToEntry "evalPointEntry1"
    defaultEvalPointButton <- Gtk.builderGetObject gui Gtk.castToButton "defaultEvalPointButton1"
    dimTable <- Gtk.builderGetObject gui Gtk.castToTable "dimTable1"
    domVarLabel <- Gtk.builderGetObject gui Gtk.castToLabel "domVarLabel1"
    zoomEntry <- Gtk.builderGetObject gui Gtk.castToEntry "zoomEntry1"
    defaultZoomPanButton <- Gtk.builderGetObject gui Gtk.castToButton "defaultZoomPanButton1"
    centreXEntry <- Gtk.builderGetObject gui Gtk.castToEntry "centreXEntry1"
    centreYEntry <- Gtk.builderGetObject gui Gtk.castToEntry "centreYEntry1"
    exportPNGButton <- Gtk.builderGetObject gui Gtk.castToButton "exportPNGButton1"
    exportSVGButton <- Gtk.builderGetObject gui Gtk.castToButton "exportSVGButton1"
    exportPDFButton <- Gtk.builderGetObject gui Gtk.castToButton "exportPDFButton1"
    printTXTButton <- Gtk.builderGetObject gui Gtk.castToButton "printTXTButton1"
    return $ Widgets
        {
            wgt_window = window,
            wgt_canvasAlignment = canvasAlignment,
            wgt_coorSystemCombo = coorSystemCombo,
            wgt_showAxesCheckbutton = showAxesCheckbutton,
            wgt_fontSizeEntry = fontSizeEntry,
            wgt_evalPointEntry = evalPointEntry,
            wgt_defaultEvalPointButton = defaultEvalPointButton,
            wgt_dimTable = dimTable,
            wgt_domVarLabel = domVarLabel,
            wgt_zoomEntry = zoomEntry,
            wgt_defaultZoomPanButton = defaultZoomPanButton,
            wgt_centreXEntry = centreXEntry,
            wgt_centreYEntry = centreYEntry,
            wgt_exportPNGButton = exportPNGButton,
            wgt_exportSVGButton = exportSVGButton,
            wgt_exportPDFButton = exportPDFButton,
            wgt_printTXTButton = printTXTButton,
            wgt_canvas = error "canvas not created yet"
        }

data Widgets = 
    Widgets
    {
        wgt_window :: Gtk.Window,
        wgt_canvasAlignment :: Gtk.Alignment,
        wgt_coorSystemCombo :: Gtk.ComboBox,
        wgt_showAxesCheckbutton :: Gtk.CheckButton,
        wgt_fontSizeEntry :: Gtk.Entry,
        wgt_evalPointEntry :: Gtk.Entry,
        wgt_defaultEvalPointButton :: Gtk.Button,
        wgt_dimTable :: Gtk.Table,
        wgt_domVarLabel :: Gtk.Label,
        wgt_zoomEntry :: Gtk.Entry,
        wgt_defaultZoomPanButton :: Gtk.Button,
        wgt_centreXEntry :: Gtk.Entry,
        wgt_centreYEntry :: Gtk.Entry,
        wgt_exportPNGButton :: Gtk.Button,
        wgt_exportSVGButton :: Gtk.Button,
        wgt_exportPDFButton :: Gtk.Button,
        wgt_printTXTButton :: Gtk.Button,
        wgt_canvas :: Gtk.DrawingArea
    }

data FnViewDynWidgets = 
    FnViewDynWidgets
    {
        dynwgt_valueLabels :: [[Gtk.Label]]
    }

initFnViewDynWidgets :: FnViewDynWidgets
initFnViewDynWidgets =
    FnViewDynWidgets []    

    