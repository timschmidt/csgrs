{-# OPTIONS_GHC -fno-warn-orphans #-}
{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE FlexibleInstances #-}
{-# LANGUAGE UndecidableInstances #-}
{-|
    Module      :  Numeric.AERN.Poly.IntPoly
    Description :  datatype of polynomials with consistent interval coefficients  
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Datatype of polynomials with consistent interval coefficients.
-}

module Numeric.AERN.Poly.IntPoly 
    (
--        sineOutPoly, sineOutPolyThin,
        module Numeric.AERN.Poly.IntPoly.Config,
        module Numeric.AERN.Poly.IntPoly.IntPoly,
        module Numeric.AERN.Poly.IntPoly.New,
        module Numeric.AERN.Poly.IntPoly.Integration,
        module Numeric.AERN.Poly.IntPoly.Evaluation,
        module Numeric.AERN.Poly.IntPoly.Show,
--        module Numeric.AERN.Poly.IntPoly.NumericOrder, -- nothing to reexport
        module Numeric.AERN.Poly.IntPoly.Reduction,
        module Numeric.AERN.Poly.IntPoly.Addition,
        module Numeric.AERN.Poly.IntPoly.Multiplication,
--        module Numeric.AERN.Poly.IntPoly.Composition,
--        module Numeric.AERN.Poly.IntPoly.RefinementOrder, -- nothing to reexport
        module Numeric.AERN.Poly.IntPoly.Minmax
--        module Numeric.AERN.Poly.IntPoly.IntervalComposition
    )
where

import Numeric.AERN.Poly.IntPoly.Config
import Numeric.AERN.Poly.IntPoly.IntPoly
import Numeric.AERN.Poly.IntPoly.New
import Numeric.AERN.Poly.IntPoly.Conversion ()
import Numeric.AERN.Poly.IntPoly.Integration
import Numeric.AERN.Poly.IntPoly.Evaluation
import Numeric.AERN.Poly.IntPoly.Reduction
import Numeric.AERN.Poly.IntPoly.Addition
import Numeric.AERN.Poly.IntPoly.Multiplication
import Numeric.AERN.Poly.IntPoly.Division ()
import Numeric.AERN.Poly.IntPoly.UpDnField ()
import Numeric.AERN.Poly.IntPoly.Composition ()
import Numeric.AERN.Poly.IntPoly.Show
import Numeric.AERN.Poly.IntPoly.NumericOrder ()
import Numeric.AERN.Poly.IntPoly.RefinementOrder ()
import Numeric.AERN.Poly.IntPoly.Minmax
import Numeric.AERN.Poly.IntPoly.SpecialConst ()
import Numeric.AERN.Poly.IntPoly.Elementary ()

import Numeric.AERN.RmToRn.Domain
import Numeric.AERN.RmToRn.New

import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as ArithUpDn
import Numeric.AERN.RealArithmetic.NumericOrderRounding 
    (RoundedRealEffortIndicator) -- needed for ghc 6.12
import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut

import Numeric.AERN.RealArithmetic.Measures
import Numeric.AERN.RealArithmetic.ExactOps

import qualified Numeric.AERN.RefinementOrder as RefOrd
import qualified Numeric.AERN.NumericOrder as NumOrd

import Numeric.AERN.Basics.Interval
import Numeric.AERN.Basics.Consistency
--import Numeric.AERN.Basics.Effort

import Test.QuickCheck.Arbitrary

instance
    (
     ArithInOut.RoundedReal (Interval e),
     NumOrd.PartialComparison (Imprecision (Interval e)),
     ArithUpDn.Convertible Int (Interval e),
     ArithUpDn.Convertible Integer (Interval e),
     ArithUpDn.Convertible Rational (Interval e),
     ArithUpDn.Convertible Double (Interval e),
     ArithInOut.RoundedMixedField (Interval e) (Interval e),
     ArithUpDn.Convertible (Interval e) (Interval e),
     RefOrd.IntervalLike (Interval e),
     HasAntiConsistency (Interval e),     
     Arbitrary (Interval e),
     GeneratableVariables var, Ord var, Show var,
     Show (Interval e), Show (Imprecision (Interval e))
    )
    => 
    ArithUpDn.RoundedReal (IntPoly var (Interval e)) 
    where
    type RoundedRealEffortIndicator (IntPoly var (Interval e)) =
        IntPolyEffort (Interval e)
    roundedRealDefaultEffort (IntPoly cfg _) = ipolycfg_effort cfg
    rrEffortComp _ eff = eff
    rrEffortMinmax _ eff = eff
    rrEffortDistance _ eff = eff
    rrEffortToSelf _sampleP _eff = ()
    rrEffortToInt _sampleP eff = eff
    rrEffortFromInt _sampleP eff = eff
    rrEffortToInteger _sampleP eff = eff
    rrEffortFromInteger _sampleP eff = eff
    rrEffortToDouble _sampleP eff = eff
    rrEffortFromDouble _sampleP eff = eff
    rrEffortToRational _sampleP eff = eff
    rrEffortFromRational _sampleP eff = eff
    rrEffortAbs _ eff = eff
    rrEffortField _ eff = eff
    rrEffortSelfMixedField _ eff = eff
    rrEffortIntMixedField _sampleP eff = eff
    rrEffortIntegerMixedField _sampleP eff = eff
    rrEffortRationalMixedField _sampleP eff = eff
    rrEffortDoubleMixedField _sampleP eff = eff

instance
    (
     ArithInOut.RoundedReal (Interval e),
     NumOrd.PartialComparison (Imprecision (Interval e)),
     ArithUpDn.Convertible Int (Interval e),
     ArithUpDn.Convertible Integer (Interval e),
     ArithUpDn.Convertible Rational (Interval e),
     ArithUpDn.Convertible Double (Interval e),
     ArithInOut.RoundedMixedField (Interval e) (Interval e),
     ArithUpDn.Convertible (Interval e) (Interval e),
     RefOrd.IntervalLike (Interval e),
     HasAntiConsistency (Interval e),     
     Arbitrary (Interval e),
     GeneratableVariables var, Ord var, Show var,
     Show (Interval e), Show (Imprecision (Interval e))
    )
    => 
    ArithInOut.RoundedReal (IntPoly var (Interval e)) 
    where
    type RoundedRealEffortIndicator (IntPoly var (Interval e)) =
        IntPolyEffort (Interval e)
    roundedRealDefaultEffort (IntPoly cfg _) = ipolycfg_effort cfg
    rrEffortNumComp _ eff = eff
    rrEffortRefComp _ eff = eff
    rrEffortMinmaxInOut _ eff = eff
--    rrEffortPartialJoin _ eff = eff
    rrEffortJoinMeet _ eff = eff
    rrEffortDistance _ eff = eff
    rrEffortToSelf _sampleP _eff = ()
    rrEffortToInt _sampleP eff = eff
    rrEffortFromInt _sampleP eff = eff
    rrEffortToInteger _sampleP eff = eff
    rrEffortFromInteger _sampleP eff = eff
    rrEffortToDouble _sampleP eff = eff
    rrEffortFromDouble _sampleP eff = eff
    rrEffortToRational _sampleP eff = eff
    rrEffortFromRational _sampleP eff = eff
    rrEffortAbs _ eff = eff
    rrEffortField _ eff = eff
    rrEffortSelfMixedField _ eff = eff
    rrEffortIntMixedField _sampleP eff = eff
    rrEffortIntegerMixedField _sampleP eff = eff
    rrEffortRationalMixedField _sampleP eff = eff
    rrEffortDoubleMixedField _sampleP eff = eff


instance 
    (ArithInOut.RoundedReal cf, Num cf,
     HasSampleFromContext cf,
     HasSampleFromContext var,
     HasAntiConsistency cf,
     RefOrd.IntervalLike cf,
     Arbitrary cf,
     Show var, Ord var, Show cf,
     NumOrd.PartialComparison (Imprecision cf), Show (Imprecision cf))
    =>
    Num (IntPoly var cf)
    where
    fromInteger n =
        newConstFnFromSample sampleP $ fromInteger n 
        where
        sampleP = sampleFromContext
    negate p = neg p
    p1 + p2 = ArithInOut.addOut p1 p2
    p1 * p2 = ArithInOut.multOut p1 p2
    
    
instance 
    (ArithInOut.RoundedReal cf, Num cf,
     HasSampleFromContext cf,
     HasSampleFromContext var,
     HasAntiConsistency cf,
     RefOrd.IntervalLike cf,
     Arbitrary cf,
     Show var, Ord var, Show cf,
     cf ~ Interval e,
     NumOrd.PartialComparison (Imprecision cf), Show (Imprecision cf))
    =>
    Fractional (IntPoly var cf)
    where
    p1 / p2 = ArithInOut.divOut p1 p2
    

        