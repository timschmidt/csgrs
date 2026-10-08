{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE ScopedTypeVariables #-}
{-|
    Module      :  Numeric.AERN.RmToRn.Plot.Simple
    Description :  simple utilities for plotting functions
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Simple utilities for plotting functions.
-}
module Numeric.AERN.RmToRn.Plot.Simple
(
    plotFns,
    simpleFnMetaData
)
where

import Numeric.AERN.RmToRn.Plot.FnView.FnData
import Numeric.AERN.RmToRn.Plot.FnView.New
import Numeric.AERN.RmToRn.Plot.Params

import Numeric.AERN.RmToRn.Plot.CairoDrawable

import Numeric.AERN.RmToRn

import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut
--import Numeric.AERN.RealArithmetic.RefinementOrderRounding.Operators

import qualified Numeric.AERN.RefinementOrder as RefOrd

import Numeric.AERN.RealArithmetic.ExactOps

import qualified Graphics.UI.Gtk as Gtk
import Control.Concurrent.STM

{-|
    To make the following plotting code work, the file FnView.glade
    has to be available in the current directory.  The master copy of this file
    is in the root folder of the aern-realfn-plot-gtk package.
-}
plotFns ::
    (fnInfo ~ (String, FnPlotStyle, Bool),
     CairoDrawableFn f,
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     CanEvaluate f,
     RefOrd.PartialComparison (Domain f),
     Show f, Show (Domain f), Show (Var f)) 
    =>
    [(String, [(fnInfo, f)])] -> 
    IO ()
plotFns fnGroups =
    do
    -- enable multithreaded GUI:
    _ <- Gtk.unsafeInitGUIForThreadedRTS
    fnDataTV <- atomically $ newTVar $ FnData $ addPlotVar fns
    fnMetaTV <- atomically $ newTVar $ fnmeta
    _ <- new sampleFn effDrawFn effCF effEval (fnDataTV, fnMetaTV) Nothing
--    Concurrent.forkIO $ signalFn fnMetaTV
    Gtk.mainGUI
    where
    ((sampleFn :_) :_) = fns 
    sampleCF = getSampleDomValue sampleFn
    effDrawFn = cairoDrawFnDefaultEffort sampleFn
    effEval = evaluationDefaultEffort sampleFn
    effCF = ArithInOut.roundedRealDefaultEffort sampleCF
    --effCF = (100, (100,())) -- MPFR

    fnmeta =
        simpleFnMetaData sampleFn rect Nothing 100 (show var) groupInfo
        where
        rect = Rectangle valHI valLO domL domR
        (domL, domR) = RefOrd.getEndpointsOut dom
        _varDoms@((var, dom) : _) = getVarDoms sampleFn
        groupInfo = zip groupNames fnInfos
        
        (valLO, valHI) = RefOrd.getEndpointsOut rangeUnion
        rangeUnion = foldl1 (RefOrd.</\>) ranges
        ranges = concat $ map (map getRange) fns
        getRange fn = evalAtPointOut (getDomainBox fn) fn
        
    (fnInfos, fns) = unzip $ map unzip groupFns
    (groupNames, groupFns) = unzip fnGroups

addPlotVar :: 
    (HasDomainBox f)
    =>
    [[f]] -> [[(GraphOrParamPlotFn f, Var f)]]
addPlotVar fns =
    map (map addV) fns
    where
    addV fn = (GraphPlotFn [fn], plotVar)
        where
        (plotVar : _) = vars
        vars = map fst $ getVarDoms fn   
 
    
simpleFnMetaData :: 
      (fnInfo ~ (String, FnPlotStyle, Bool), 
       HasZero (Domain t), HasOne (Domain t), HasDomainBox t) 
      =>
      t
      -> Rectangle (Domain f) -- ^ initial crop
      -> Maybe ColourRGBA -- ^ background colour
      -> Int -- ^ number of samples to take of each function per viewable region
      -> String -- ^ the name of the variable that ranges over the plotted function domain
      -> [(String, [fnInfo])] -- ^ information on plotted function groups (names, plot colours, whether shown initially)
      -> FnMetaData f
simpleFnMetaData sampleFn rect bgrColour samplesPerUnit domVarName (groups :: [(String, [fnInfo])]) =
        (defaultFnMetaData sampleFn)
        {
            dataFnGroupNames = map getGroupName groups,
            dataFnNames = mapGroupsFns getFnName, 
            dataFnStyles = mapGroupsFns getFnStyle,
            dataDefaultActiveFns = mapGroupsFns getFnEnabled,
            dataDomName = domVarName,
            dataDomL = domL,
            dataDomR = domR,
            dataValLO = valLO,
            dataValHI = valHI,
            dataDefaultEvalPoint = domR,
            dataDefaultCanvasParams =
                (defaultCanvasParams sampleDom)
                {
                    cnvprmCoordSystem = CoordSystemLinear rect,
                    cnvprmBackgroundColour = bgrColour,
                    cnvprmSamplesPerUnit = samplesPerUnit
                }
        }
        where
        (Rectangle valHI valLO domL domR) = rect
        sampleDom = getSampleDomValue sampleFn
        getGroupName (name, _) = name
        getGroupContent (_, content) = content
        mapGroupsFns :: (fnInfo -> t) -> [[t]]
        mapGroupsFns f = map (map f . getGroupContent) groups 
        getFnName (name, _, _) = name
        getFnStyle (_, style, _) = style
        getFnEnabled (_, _, enabled) = enabled
    

   