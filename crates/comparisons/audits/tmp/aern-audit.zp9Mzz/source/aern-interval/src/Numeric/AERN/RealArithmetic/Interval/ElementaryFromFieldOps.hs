{-# OPTIONS_GHC -fno-warn-orphans #-}
{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE UndecidableInstances #-}
{-# LANGUAGE FlexibleContexts #-}
{-|
    Module      :  Numeric.AERN.RealArithmetic.Interval.ElementaryFromFieldOps
    Description :  elementary operations using generic direct implementation
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable

    Elementary operations using generic direct implementation.
-}

module Numeric.AERN.RealArithmetic.Interval.ElementaryFromFieldOps 
(
    ExpThinEffortIndicator(..), 
    SqrtThinEffortIndicator(..),
    SineCosineThinEffortIndicator(..)
)
where

import Numeric.AERN.RealArithmetic.Interval.SpecialConst ()
import Numeric.AERN.RealArithmetic.Interval.MixedFieldOps ()

import Numeric.AERN.RealArithmetic.RefinementOrderRounding.ElementaryFromFieldOps.Exponentiation
import Numeric.AERN.RealArithmetic.RefinementOrderRounding.ElementaryFromFieldOps.SineCosine
import Numeric.AERN.RealArithmetic.NumericOrderRounding.ElementaryFromFieldOps.Sqrt

import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as ArithUpDn
import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut
import Numeric.AERN.RealArithmetic.RefinementOrderRounding
        (
                ExpEffortIndicator,
                SqrtEffortIndicator,
                SineCosineEffortIndicator
        )

import qualified Numeric.AERN.NumericOrder as NumOrd
import  Numeric.AERN.NumericOrder.Operators

import qualified Numeric.AERN.RefinementOrder as RefOrd

--import Numeric.AERN.RealArithmetic.ExactOps
--import Numeric.AERN.RealArithmetic.Interval.FieldOps
import Numeric.AERN.RealArithmetic.Measures

import Numeric.AERN.Basics.Interval
--import Numeric.AERN.Basics.Consistency
import Numeric.AERN.Basics.Effort
import Numeric.AERN.Basics.ShowInternals


instance
    (ArithInOut.RoundedReal (Interval e),
     -- MK has no idea why the following three are not automatically deduced from the above...
     ArithUpDn.RoundedReal e,
     ArithInOut.RoundedAddEffort (Distance e),
     RefOrd.RoundedLatticeEffort (Distance e),
     EffortIndicator (ArithInOut.FieldOpsEffortIndicator (Distance e))
     )
    => 
    (ArithInOut.RoundedExponentiationEffort (Interval e))
    where
    type ExpEffortIndicator (Interval e) =
        ExpThinEffortIndicator (Interval e) 
    expDefaultEffort i =
        expThinDefaultEffort i 10

instance
    (ArithInOut.RoundedReal (Interval e),
     -- MK has no idea why the following three are not automatically deduced from the above...
     ArithUpDn.RoundedReal e,
     ArithInOut.RoundedAddEffort (Distance e),
     RefOrd.RoundedLatticeEffort (Distance e),
     EffortIndicator (ArithInOut.FieldOpsEffortIndicator (Distance e))
     )
    => 
    (ArithInOut.RoundedExponentiationEffort (IntervalApprox e))
    where
    type ExpEffortIndicator (IntervalApprox e) =
        ExpThinEffortIndicator (Interval e) 
    expDefaultEffort (IntervalApprox o _i) =
        ArithInOut.expDefaultEffort o


instance
    (ArithInOut.RoundedReal (Interval e), 
     -- MK has no idea why the following three are not automatically deduced from the above...
     ArithUpDn.RoundedReal e,
     ShowInternals e,
     ArithInOut.RoundedAddEffort (Distance e),
     RefOrd.RoundedLatticeEffort (Distance e),
     EffortIndicator (ArithInOut.FieldOpsEffortIndicator (Distance e)),
     --
     NumOrd.HasExtrema e)
    => 
    (ArithInOut.RoundedExponentiation (Interval e))
    where
    expOutEff eff (Interval l r) 
        = Interval resL resR
        where
        Interval resL _ = 
            expOutThinArg 
                eff 
                (Interval l l)
        Interval _ resR =
            expOutThinArg 
                eff
                (Interval r r)
    expInEff eff (Interval l r) 
        = Interval resL resR
        where
        Interval _ resL = 
            expOutThinArg 
                eff 
                (Interval l l)
        Interval resR _ =
            expOutThinArg 
                eff 
                (Interval r r)

instance
    (ArithInOut.RoundedReal (Interval e), 
     -- MK has no idea why the following three are not automatically deduced from the above...
     ArithUpDn.RoundedReal e,
     ShowInternals e,
     ArithInOut.RoundedAddEffort (Distance e),
     RefOrd.RoundedLatticeEffort (Distance e),
     EffortIndicator (ArithInOut.FieldOpsEffortIndicator (Distance e)),
     --
     NumOrd.HasExtrema e)
    => 
    (ArithInOut.RoundedExponentiation (IntervalApprox e))
    where
    expOutEff eff (IntervalApprox o i) 
        = IntervalApprox resO resI
        where
        resO = ArithInOut.expOutEff eff o
        resI = ArithInOut.expInEff eff i
    expInEff = error "AERN: expInEff not defined for IntervalApprox"

instance 
    (ArithUpDn.RoundedReal e) 
    => 
    (ArithInOut.RoundedSquareRootEffort (Interval e))
    where
    type SqrtEffortIndicator (Interval e) = SqrtThinEffortIndicator e 
    sqrtDefaultEffort (Interval l _) =
        sqrtThinDefaultEffort l 10 

instance 
    (ArithUpDn.RoundedReal e) 
    => 
    (ArithInOut.RoundedSquareRootEffort (IntervalApprox e))
    where
    type SqrtEffortIndicator (IntervalApprox e) = 
        SqrtThinEffortIndicator e 
    sqrtDefaultEffort (IntervalApprox o _i) =
        ArithInOut.sqrtDefaultEffort o
    
instance 
    (ArithUpDn.RoundedReal e, Show e) 
    => 
    (ArithInOut.RoundedSquareRoot (Interval e))
    where
    sqrtOutEff eff (Interval l r) =
        case NumOrd.pEqualEff effComp l r of
            Just True -> Interval sqrtLL sqrtLR
            _ -> Interval sqrtLL sqrtRR
        where
        (sqrtLL, sqrtLR) = sqrtOutThinArg eff l 
        (_, sqrtRR) = sqrtOutThinArg eff r
        effComp = 
            ArithUpDn.rrEffortComp l $ sqrteff_arith eff 
    sqrtInEff eff (Interval l r) =
        case NumOrd.pEqualEff effComp l r of
            Just True -> Interval sqrtLR sqrtLL -- invert
            _ -> Interval sqrtLR sqrtRL
        where
        (sqrtLL, sqrtLR) = sqrtOutThinArg eff l 
        (sqrtRL, _) = sqrtOutThinArg eff r
        effComp = 
            ArithUpDn.rrEffortComp l $ sqrteff_arith eff 
                
instance 
    (ArithUpDn.RoundedReal e, Show e) 
    => 
    (ArithInOut.RoundedSquareRoot (IntervalApprox e))
    where
    sqrtOutEff eff (IntervalApprox o i) 
        = IntervalApprox resO resI
        where
        resO = ArithInOut.sqrtOutEff eff o
        resI = ArithInOut.sqrtInEff eff i
    sqrtInEff = error "AERN: expInEff not defined for IntervalApprox"


instance 
    (ArithUpDn.RoundedReal e) 
    => 
    (ArithInOut.RoundedSineCosineEffort (Interval e))
    where
    type SineCosineEffortIndicator (Interval e) = SineCosineThinEffortIndicator (Interval e) (Interval e)
        -- TODO: have a Scalar associated type and use this here for the coeff parameter
    sincosDefaultEffort i =
        sineCosineThinDefaultEffort i i 10

--instance 
--    (ArithUpDn.RoundedReal e) 
--    => 
--    (ArithInOut.RoundedSquareRootEffort (IntervalApprox e))
--    where
--    type SqrtEffortIndicator (IntervalApprox e) = 
--        SqrtThinEffortIndicator e 
--    sqrtDefaultEffort (IntervalApprox o _i) =
--        ArithInOut.sqrtDefaultEffort o

instance 
    (ArithInOut.RoundedReal (Interval e),
     ArithUpDn.RoundedReal e, 
     Show e,
     ArithUpDn.RoundedSpecialConst e, 
     HasDistance e, ArithInOut.RoundedReal (Distance e),
     ShowInternals e) 
    => 
    (ArithInOut.RoundedSineCosine (Interval e))
    where
    sinOutEff eff i =
        sineCosineIntervalLikeOut True eff i i
    sinInEff eff i =
        error "inner rounded sine not defined yet"
    cosOutEff eff i =
        sineCosineIntervalLikeOut False eff i i
    cosInEff eff i =
        error "inner rounded cosine not defined yet"
                
--instance 
--    (ArithUpDn.RoundedReal e, Show e) 
--    => 
--    (ArithInOut.RoundedSquareRoot (IntervalApprox e))
--    where
--    sqrtOutEff eff (IntervalApprox o i) 
--        = IntervalApprox resO resI
--        where
--        resO = ArithInOut.sqrtOutEff eff o
--        resI = ArithInOut.sqrtInEff eff i
--    sqrtInEff = error "AERN: expInEff not defined for IntervalApprox"
--                