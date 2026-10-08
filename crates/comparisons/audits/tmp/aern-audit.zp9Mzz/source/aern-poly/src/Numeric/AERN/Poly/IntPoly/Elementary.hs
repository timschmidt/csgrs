{-# OPTIONS_GHC -fno-warn-orphans #-}
{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE UndecidableInstances #-}
{-|
    Module      :  Numeric.AERN.Poly.IntPoly.Elementary
    Description :  out-rounded exp, sine, cosine etc on IntPoly   
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Outwards-rounded exp, sine, cosine etc on IntPoly.  Works only with relatively thin IntPolys.
-}

module Numeric.AERN.Poly.IntPoly.Elementary
--    (
--    )
where
    
import Numeric.AERN.Poly.IntPoly.IntPoly
import Numeric.AERN.Poly.IntPoly.Conversion
import Numeric.AERN.Poly.IntPoly.Addition
import Numeric.AERN.Poly.IntPoly.Multiplication
import Numeric.AERN.Poly.IntPoly.New ()
import Numeric.AERN.Poly.IntPoly.Show ()
    
import Numeric.AERN.RmToRn.New
import Numeric.AERN.RmToRn.Domain

import qualified 
       Numeric.AERN.RealArithmetic.RefinementOrderRounding
       as ArithInOut

import Numeric.AERN.RealArithmetic.RefinementOrderRounding.ElementaryFromFieldOps.SineCosine

import qualified 
       Numeric.AERN.RealArithmetic.NumericOrderRounding
       as ArithUpDn

import Numeric.AERN.RealArithmetic.NumericOrderRounding.ElementaryFromFieldOps.Sqrt

import Numeric.AERN.RealArithmetic.ExactOps

--import Numeric.AERN.RealArithmetic.Interval ()

import Numeric.AERN.Basics.Interval
import Numeric.AERN.Basics.SizeLimits


import qualified 
       Numeric.AERN.RefinementOrder 
       as RefOrd
--import Numeric.AERN.RefinementOrder.OpsDefaultEffort

import Numeric.AERN.Basics.Consistency

instance
    (Ord var, Show var,
     cf ~ Interval e,
     ArithInOut.RoundedReal cf,
     HasAntiConsistency cf,
     Show cf,  Show (SizeLimits cf))
    =>
    ArithInOut.RoundedSineCosineEffort (IntPoly var cf)
    where
    type SineCosineEffortIndicator (IntPoly var cf) = 
        SineCosineThinEffortIndicator (IntPoly var cf) cf
    sincosDefaultEffort sampleP =
        sineCosineThinDefaultEffort sampleP sampleCf 3
        where
        sampleCf = getSampleDomValue sampleP 

instance
    (Ord var, Show var,
     cf ~ Interval e,
     real ~ IntPoly var cf,
     ArithInOut.RoundedReal cf,
     HasAntiConsistency cf,
     ArithInOut.RoundedAdd real,
     ArithInOut.RoundedMultiply real,
     ArithInOut.RoundedPowerToNonnegInt real,
     Show cf,  Show (SizeLimits cf))
    =>
    ArithInOut.RoundedSineCosine (IntPoly var cf)
    where
    sinOutEff eff p = sineTaylorAt0 eff sampleCf p
        where
        sampleCf = getSampleDomValue p
    cosOutEff eff p = cosineTaylorAt0 eff sampleCf p
        where
        sampleCf = getSampleDomValue p
       
instance
    (Ord var, Show var,
     cf ~ Interval e,
     real ~ IntPoly var cf,
     ArithUpDn.RoundedReal real,
     ArithInOut.RoundedReal cf,
     HasAntiConsistency cf,
     Show cf,  Show (SizeLimits cf))
    =>
    ArithInOut.RoundedSquareRootEffort (IntPoly var cf)
    where
    type SqrtEffortIndicator (IntPoly var cf) = 
        SqrtThinEffortIndicator (IntPoly var cf)
    sqrtDefaultEffort sampleP =
        sqrtThinDefaultEffort sampleP 5

       