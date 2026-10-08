{-# LANGUAGE TypeFamilies #-}
{-|
    Module      :  Numeric.AERN.RmToRn.Plot.CairoDrawable
    Description :  abstraction for drawing on a Cairo canvas
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Abstraction for drawing on a Cairo canvas.
-}
module Numeric.AERN.RmToRn.Plot.CairoDrawable 
(
    CairoDrawableFn(..),
)
where

import Numeric.AERN.RmToRn.Plot.Params (CanvasParams,FnPlotStyle)

import Numeric.AERN.RmToRn.Domain

import Graphics.Rendering.Cairo (Render)

class
    CairoDrawableFn f
    where
    type CairoDrawFnEffortIndicator f
    cairoDrawFnDefaultEffort ::
        f -> (CairoDrawFnEffortIndicator f)
    {-|
        Plot the graph of a uni-variate function on the active cairo canvas.
        A multi-variate function is transformed into an univariate function
        by ranging all other variables over their entire domains and taking
        the union of all the uni-variate functions thus obtained.
     -}
    cairoDrawFnGraph ::
        CairoDrawFnEffortIndicator f -> 
        CanvasParams (Domain f) ->
        ((Domain f, Domain f) -> (Double, Double)) {-^ conversion from [0,1]^2 (origin bottom left) to screen coords -} ->
        FnPlotStyle ->
        Var f {-^ the variable to map to the horizontal axis; the remaining variables are all substituted with their full range -} ->
        [f] {-^ a function to plot, given piecewise -} -> 
        Render ()
    {-|
        Plot the graph of a uni-variate function on the active cairo canvas.
        A multi-variate function is transformed into an univariate function
        by ranging all other variables over their entire domains and taking
        the union of all the uni-variate functions thus obtained.
     -}
    cairoDrawFnParameteric ::
        CairoDrawFnEffortIndicator f -> 
        CanvasParams (Domain f) ->
        ((Domain f, Domain f) -> (Double, Double)) {-^ conversion from [0,1]^2 (origin bottom left) to screen coords -} ->
        FnPlotStyle ->
        Var f {-^ the variable to parametrise by; the remaining variables are all substituted with their full range -} ->
        [Var f] {-^ variables that should not be substituted with their full domain but sampled, potentially resulting in a non-rectangular shape -} ->
        [(f, f)] {-^ one function to map to the x axis, one to the y axis, the function pair is given piecewise -} -> 
        Render ()

