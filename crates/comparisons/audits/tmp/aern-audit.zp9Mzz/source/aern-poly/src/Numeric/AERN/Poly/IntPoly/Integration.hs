{-# OPTIONS_GHC -fno-warn-orphans #-}
{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE ScopedTypeVariables #-}
{-# LANGUAGE UndecidableInstances #-}
{-|
    Module      :  Numeric.AERN.Poly.IntPoly.Integration
    Description :  integration of interval polynomials  
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Integration of interval polynomials.
-}

module Numeric.AERN.Poly.IntPoly.Integration
    (
        primitiveFnOutPoly
    )
where
    
import Numeric.AERN.Poly.IntPoly.Config
import Numeric.AERN.Poly.IntPoly.IntPoly
import Numeric.AERN.Poly.IntPoly.New
import Numeric.AERN.Poly.IntPoly.Reduction
import Numeric.AERN.Poly.IntPoly.Addition ()
import Numeric.AERN.Poly.IntPoly.Multiplication ()

import Numeric.AERN.RmToRn.Integration

import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut
--import Numeric.AERN.RealArithmetic.RefinementOrderRounding.OpsImplicitEffort
import Numeric.AERN.RealArithmetic.ExactOps

import qualified Numeric.AERN.RefinementOrder as RefOrd
--import Numeric.AERN.RefinementOrder.OpsImplicitEffort
import qualified Numeric.AERN.NumericOrder as NumOrd
--import Numeric.AERN.NumericOrder.OpsImplicitEffort

import Numeric.AERN.RealArithmetic.Measures

import Numeric.AERN.Basics.Consistency

import qualified Data.IntMap as IntMap

instance 
    (Ord var, Show var, 
     ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf,
     HasAntiConsistency cf,
     NumOrd.PartialComparison (Imprecision cf), 
     Show cf) 
    => 
    RoundedIntegration (IntPoly var cf)
    where
    type IntegrationEffortIndicator (IntPoly var cf) =
         IntPolyEffort cf
    integrationDefaultEffort (IntPoly cfg _) =
        ipolycfg_effort cfg
    primitiveFunctionOutEff = primitiveFnOutPoly
    primitiveFunctionInEff =
        error "inner rounded integration not defined for IntPoly"

primitiveFnOutPoly ::
    (Ord var, Show var, 
     ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf,
     HasAntiConsistency cf,
     NumOrd.PartialComparison (Imprecision cf), 
     Show cf) 
    => 
    (IntPolyEffort cf) ->
    IntPoly var cf {- polynomial to integrate in its main variable -} ->
    var {- variable to integrate in -} ->
    IntPoly var cf
primitiveFnOutPoly eff _p@(IntPoly cfgTop terms) integVar =
    reducePolyDegreeOut effCf $
    IntPoly cfgTop $ primitiveFnOutTerms cfgTop terms
    where
    effCf = ipolyeff_cfRoundedRealEffort eff
    primitiveFnOutTerms cfg (IntPolyV var powers) 
        | integVar == var =
        --    unsafePrint
        --    (
        --        "integratePoly:"
        --        ++ "\n initPoly = " ++ showPoly initPoly
        --        ++ "\n p = " ++ showPoly p
        --        ++ "\n result = " ++ showPoly result
        --    )
                result
        | otherwise =
            (IntPolyV var $ IntMap.map (primitiveFnOutTerms cfgR) powers)
        where
        result = IntPolyV var $ IntMap.insert 0 zP powersFractions 
        cfgR = cfgRemFirstVar cfg
        zP = mkConstTerms (zero sample) $ ipolycfg_vars cfgR
        powersFractions =
            IntMap.fromDistinctAscList $
                map integrateTerms $
                    IntMap.toAscList powers
            where
            integrateTerms (n,t) =
                (n+1, intpoly_terms $ (IntPoly cfgR t) </>| (n+1))
            (</>|) = ArithInOut.mixedDivOutEff eff
    --    effAdd = ArithInOut.fldEffortAdd sample $ ArithInOut.rrEffortField sample eff
        sample = ipolycfg_sample_cf cfg
    primitiveFnOutTerms _ _ = 
        error "aern-poly: primitiveFnOutPoly: integrating by a variable not present in the domain"        
