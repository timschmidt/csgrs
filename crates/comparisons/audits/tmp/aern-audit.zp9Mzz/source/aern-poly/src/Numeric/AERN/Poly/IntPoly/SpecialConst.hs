{-# OPTIONS_GHC -fno-warn-orphans #-}
{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-|
    Module      :  Numeric.AERN.Poly.IntPoly.SpecialConst
    Description :  constant pi, e polynomials etc  
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Creating constant polynomials with special values such as pi and e.
-}

module Numeric.AERN.Poly.IntPoly.SpecialConst
--    (
--    )
where
    
import Numeric.AERN.Poly.IntPoly.IntPoly
import Numeric.AERN.Poly.IntPoly.New ()
import Numeric.AERN.Poly.IntPoly.Show ()
    
import Numeric.AERN.RmToRn.New
import Numeric.AERN.RmToRn.Domain

import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut

import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as ArithUpDn

import qualified Numeric.AERN.RefinementOrder as RefOrd
--import Numeric.AERN.RefinementOrder.OpsDefaultEffort

import Numeric.AERN.Basics.Consistency

instance
    (Ord var, Show var, Show cf,
     HasConsistency cf,
     ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf,
     ArithInOut.RoundedSpecialConstEffort cf)
    =>
    ArithInOut.RoundedSpecialConstEffort (IntPoly var cf)
    where
    type SpecialConstEffortIndicator (IntPoly var cf) = 
        ArithInOut.SpecialConstEffortIndicator cf
    specialConstDefaultEffort sampleP =
        ArithInOut.specialConstDefaultEffort sampleCf
        where
        sampleCf = getSampleDomValue sampleP 

instance
    (Ord var, Show var, Show cf,
     HasConsistency cf,
     ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf,
     ArithInOut.RoundedSpecialConst cf)
    =>
    ArithInOut.RoundedSpecialConst (IntPoly var cf)
    where
    piOutEff eff sampleP =
        newConstFnFromSample sampleP $ ArithInOut.piOutEff eff sampleCf
        where
        sampleCf = getSampleDomValue sampleP 
    piInEff eff sampleP = 
        newConstFnFromSample sampleP $ ArithInOut.piInEff eff sampleCf
        where
        sampleCf = getSampleDomValue sampleP 
    eOutEff eff sampleP =
        newConstFnFromSample sampleP $ ArithInOut.eOutEff eff sampleCf
        where
        sampleCf = getSampleDomValue sampleP 
    eInEff eff sampleP = 
        newConstFnFromSample sampleP $ ArithInOut.eInEff eff sampleCf
        where
        sampleCf = getSampleDomValue sampleP 
    
instance
    (Ord var, Show var, Show cf,
     HasConsistency cf,
     ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf,
     ArithInOut.RoundedSpecialConstEffort cf)
    =>
    ArithUpDn.RoundedSpecialConstEffort (IntPoly var cf)
    where
    type SpecialConstEffortIndicator (IntPoly var cf) = 
        ArithInOut.SpecialConstEffortIndicator cf
    specialConstDefaultEffort sampleP =
        ArithInOut.specialConstDefaultEffort sampleCf
        where
        sampleCf = getSampleDomValue sampleP 
    

instance
    (Ord var, Show var, Show cf,
     HasConsistency cf,
     ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf,
     ArithInOut.RoundedSpecialConst cf)
    =>
    ArithUpDn.RoundedSpecialConst (IntPoly var cf)
    where
    piUpEff eff sampleP = 
        newConstFnFromSample sampleP $ ArithInOut.piOutEff eff sampleCf
        where
        sampleCf = getSampleDomValue sampleP 
    piDnEff eff sampleP = 
        newConstFnFromSample sampleP $ ArithInOut.piOutEff eff sampleCf
        where
        sampleCf = getSampleDomValue sampleP 
    eUpEff eff sampleP = 
        newConstFnFromSample sampleP $ ArithInOut.eOutEff eff sampleCf
        where
        sampleCf = getSampleDomValue sampleP 
    eDnEff eff sampleP = 
        newConstFnFromSample sampleP $ ArithInOut.eOutEff eff sampleCf
        where
        sampleCf = getSampleDomValue sampleP 

