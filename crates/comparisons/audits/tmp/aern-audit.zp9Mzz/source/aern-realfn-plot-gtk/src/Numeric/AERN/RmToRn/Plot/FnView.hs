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
    Module      :  Numeric.AERN.RmToRn.Plot.FnView
    Description :  a gtk widget inspecting cairo drawable functions
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    A gtk widget inspecting function enclosures.
-}
module Numeric.AERN.RmToRn.Plot.FnView 
(
    FnData (..),
    GraphOrParamPlotFn (..),
    FnMetaData (..),
    defaultFnData,
    defaultFnMetaData,
    new,
    plotToPDFFile,
    module Numeric.AERN.RmToRn.Plot.Simple,
    module Numeric.AERN.RmToRn.Plot.Params
)
where

import Numeric.AERN.RmToRn.Plot.FnView.New
import Numeric.AERN.RmToRn.Plot.FnView.FnData
import Numeric.AERN.RmToRn.Plot.FnView.State
import Numeric.AERN.RmToRn.Plot.FnView.Layout
import Numeric.AERN.RmToRn.Plot.FnView.WatchData
import Numeric.AERN.RmToRn.Plot.FnView.Canvas

import Numeric.AERN.RmToRn.Plot.Params
import Numeric.AERN.RmToRn.Plot.Simple
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

