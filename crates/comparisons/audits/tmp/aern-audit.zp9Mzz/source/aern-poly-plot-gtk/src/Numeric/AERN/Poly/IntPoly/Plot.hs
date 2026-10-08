{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE FlexibleInstances #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE UndecidableInstances #-}
{-|
    Module      :  Numeric.AERN.Poly.IntPoly.Plot
    Description :  Cairo plotting support
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Cairo plotting support.
-}

module Numeric.AERN.Poly.IntPoly.Plot
(
    CairoDrawEffortIndicatorFnFromEval(..)
)
where
    
import Numeric.AERN.Poly.IntPoly

import Numeric.AERN.RmToRn.Plot.CairoDrawable
import Numeric.AERN.RmToRn.Plot.FromEval

import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut
--import Numeric.AERN.RealArithmetic.RefinementOrderRounding.OpsImplicitEffort

import qualified Numeric.AERN.RefinementOrder as RefOrd
--import Numeric.AERN.RefinementOrder.OpsImplicitEffort

import Numeric.AERN.Basics.Consistency

--import Numeric.AERN.Misc.Debug

instance
    (Show cf,
     RefOrd.IntervalLike cf,
     ArithInOut.RoundedReal cf,
     Ord var,
     Show var,
     HasAntiConsistency cf)
    =>
    CairoDrawableFn (IntPoly var cf)
    where
    type CairoDrawFnEffortIndicator (IntPoly var cf) =
        IntPolyEffort cf
    cairoDrawFnDefaultEffort (IntPoly cfg _) =
        ipolycfg_effort cfg
    cairoDrawFnGraph effIP = 
        cairoDrawFnGraphFromEval $ convertEff effIP 
    cairoDrawFnParameteric effIP = 
        cairoDrawFnParametericFromEval $ convertEff effIP 
        
convertEff :: 
    HasConsistency cf 
    =>
    IntPolyEffort cf
    -> 
    CairoDrawEffortIndicatorFnFromEval (IntPoly var cf)
convertEff effIP =
    CairoDrawEffortIndicatorFnFromEval
    {
        draweff_evalF = effIP
    ,
        draweff_realD = ipolyeff_cfRoundedRealEffort effIP
    ,
        draweff_getEndpointsD = ipolyeff_cfGetEndpointsEffort effIP
    }    
    
        