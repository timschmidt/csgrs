{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE StandaloneDeriving #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE UndecidableInstances #-}
{-|
    Module      :  Numeric.AERN.IVP.Plot.PicardView.Layout
    Description :  layout of the PicardView window
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Internal module for PicardView.
    Layout of the PicardView window. 
    
    NOT COMPLETED, CURRENTLY STALLED
-}
module Numeric.AERN.IVP.Plot.PicardView.Layout
--(
--)
where

--import Numeric.AERN.RmToRn.Plot.PicardView.State

import qualified Graphics.UI.Gtk as Gtk

import System.Directory (doesFileExist)

--loadGlade :: 
--    FilePath ->
--    IO Widgets
--loadGlade gladeFileName =
--    do
--    gotGladeFile <- doesFileExist gladeFileName
--    case gotGladeFile of
--        True -> return ()
--        False -> error $ "aern-ivp: ...Plot.PicardView: glade file " ++ gladeFileName ++ " not found" 
--    Just xml <- Glade.xmlNew gladeFileName
--    window <- Glade.xmlGetWidget xml Gtk.castToWindow "window1"
--    segmentTable <- Glade.xmlGetWidget xml Gtk.castToTable "segmentTable1"
--    nextPicardIterationButton <- Glade.xmlGetWidget xml Gtk.castToButton "nextPicardIterationButton1"
--    nextSegmentButton <- Glade.xmlGetWidget xml Gtk.castToButton "nextSegmentButton1"
--    undoButton <- Glade.xmlGetWidget xml Gtk.castToButton "undoButton1"
--    return $ Widgets
--        {
--            window = window,
--            segmentTable = segmentTable,
--            nextPicardIterationButton = nextPicardIterationButton,
--            nextSegmentButton = nextSegmentButton,
--            undoButton = undoButton
--        }

data Widgets = 
    Widgets
    {
        window :: Gtk.Window,
        segmentTable :: Gtk.Table,
        nextPicardIterationButton :: Gtk.Button,
        nextSegmentButton :: Gtk.Button,
        undoButton :: Gtk.Button
    }

data PicardViewDynWidgets = 
    PicardViewDynWidgets
    {
        iterationCountLabels :: [[Gtk.Label]],
        precisionLabels :: [[Gtk.Label]]
    }

initPicardViewDynWidgets :: PicardViewDynWidgets
initPicardViewDynWidgets =
    PicardViewDynWidgets [] []

    