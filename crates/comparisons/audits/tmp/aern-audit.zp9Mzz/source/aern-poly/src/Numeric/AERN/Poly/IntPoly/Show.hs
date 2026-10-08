{-# OPTIONS_GHC -fno-warn-orphans #-}
{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-|
    Module      :  Numeric.AERN.Poly.IntPoly.Show
    Description :  formatting functions  
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Formatting functions.
-}

module Numeric.AERN.Poly.IntPoly.Show
--    (
--    )
where
    
import Numeric.AERN.Poly.IntPoly.Config
import Numeric.AERN.Poly.IntPoly.IntPoly
--import Numeric.AERN.Poly.IntPoly.Composition
    
import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut
--import Numeric.AERN.RealArithmetic.RefinementOrderRounding.OpsDefaultEffort
import Numeric.AERN.RealArithmetic.ExactOps
--import Numeric.AERN.RealArithmetic.Measures (HasImprecision(..))

import qualified Numeric.AERN.RefinementOrder as RefOrd
import Numeric.AERN.RefinementOrder.Operators ((|==?))

import Numeric.AERN.Basics.ShowInternals
--import Numeric.AERN.Basics.SizeLimits
--import Numeric.AERN.Basics.Exception
--import Numeric.AERN.Basics.Consistency

import qualified Data.IntMap as IntMap

import Data.List (intercalate)

instance 
    (Show var, Show cf, Show (IntPolySizeLimits cf), ArithInOut.RoundedReal cf) => 
    Show (IntPoly var cf)
    where
    show p
        = 
        showPolyInternals show False False p
--        "IntPoly{" ++ showPoly show show p ++ "; " ++ show cfg ++ "; " ++ show terms ++ "}" 

instance
    (Show var, Show cf, ShowInternals cf, Show (IntPolySizeLimits cf),
     ArithInOut.RoundedReal cf,
     HasZero cf, RefOrd.PartialComparison cf) 
    =>
    (ShowInternals (IntPoly var cf))
    where
    type ShowInternalsIndicator (IntPoly var cf) 
        = (ShowInternalsIndicator cf, Bool, Bool)
    defaultShowIndicator (IntPoly cfg _) 
        = (defaultShowIndicator $ ipolycfg_sample_cf cfg, True, False)
    showInternals (cfIndicator, shouldShowCfg, shouldShowTerms) p =
        showPolyInternals showCf shouldShowCfg shouldShowTerms p 
        where
        showCf = showInternals cfIndicator
    
showPolyInternals :: 
    (Show cf, Show (IntPolySizeLimits cf), Show var, ArithInOut.RoundedReal cf) 
    =>
    (cf -> String) -> Bool -> Bool -> IntPoly var cf 
    -> 
    String
showPolyInternals showCf shouldShowCfg shouldShowTerms p@(IntPoly cfg terms) =
        "IntPoly{" ++ valueS ++ cfgS ++ termsS ++ "}"
        where
        valueS = showPoly show showCf p
        cfgS 
            | shouldShowCfg = "; " ++ show cfg
            | otherwise = ""
        termsS
            | shouldShowTerms = "; [" ++ showTerms showCf terms ++ "]"
            | otherwise = ""
    
showPoly ::
    (HasZero cf, 
     RefOrd.PartialComparison cf)
    =>
    (var -> String) -> 
    (cf -> String) -> 
    (IntPoly var cf -> String)
showPoly showVar showCoeff (IntPoly cfg terms) =
    usingVariableDifferences
--    expandedToNormalForm
    where
--    expandedToNormalForm =
--        composeAllVarsOutEff -- TODO
    usingVariableDifferences =
        sp "" domsLE terms
    domsLE = ipolycfg_domsLE cfg
    sp vars _ (IntPolyC value) 
        = (showCoeff value) ++ vars
    sp otherVars (domLE : restDomsLE) (IntPolyV var powers)
        = intercalate " + " $ map showTerm $ reverse $ IntMap.toAscList $ powers
        where
        showTerm (n,p) = sp (otherVars ++ showVarPower n) restDomsLE p
        showVarPower 0 = ""
        showVarPower 1 = showVarShift
        showVarPower n = showVarShift ++ "^" ++ show n
        showVarShift =
            case domLE |==? zero domLE of
                Just True -> showVar var
                _ -> "(" ++ showVar var ++ "-" ++ showCoeff domLE ++ ")"
    sp _ _ _ =
        error $ "aern-poly: internal error in IntPoly.showPoly" 
    
