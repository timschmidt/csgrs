{-# LANGUAGE TypeFamilies #-}
--{-# LANGUAGE FlexibleContexts #-}
{-|
    Module      :  Numeric.AERN.RealArithmetic.Interval.ElementaryFromBasis
    Description :  elementary operations using basis-level operations
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable

    Elementary operations using basis-level operations.
-}

module Numeric.AERN.RealArithmetic.Interval.ElementaryFromBasis where

import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut
import Numeric.AERN.RealArithmetic.RefinementOrderRounding
        (
                ExpEffortIndicator,
                SqrtEffortIndicator
        )
        
import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as ArithUpDn

import Numeric.AERN.Basics.Interval

newtype IntervalElementaryFromBasis t = 
    IntervalElementaryFromBasis { unIntervalElementaryFromBasis :: Interval t }

asIntervalElementaryFromBasis ::
    ((IntervalElementaryFromBasis t) -> (IntervalElementaryFromBasis t))
    ->
    ((Interval t) -> (Interval t))
asIntervalElementaryFromBasis op interval =
    unIntervalElementaryFromBasis $ op (IntervalElementaryFromBasis interval)

instance (ArithUpDn.RoundedExponentiationEffort e) => 
    (ArithInOut.RoundedExponentiationEffort (IntervalElementaryFromBasis e)) 
    where
    type ExpEffortIndicator (IntervalElementaryFromBasis e) = 
        ArithUpDn.ExpEffortIndicator e
    expDefaultEffort (IntervalElementaryFromBasis (Interval l _)) = ArithUpDn.expDefaultEffort l

instance (ArithUpDn.RoundedExponentiation e) => 
    (ArithInOut.RoundedExponentiation (IntervalElementaryFromBasis e)) 
    where
    expInEff effort (IntervalElementaryFromBasis (Interval l r)) =
        IntervalElementaryFromBasis $
            Interval (ArithUpDn.expUpEff effort l) (ArithUpDn.expDnEff effort r)
    expOutEff effort (IntervalElementaryFromBasis (Interval l r)) =
        IntervalElementaryFromBasis $
            Interval (ArithUpDn.expDnEff effort l) (ArithUpDn.expUpEff effort r)

instance (ArithUpDn.RoundedSquareRootEffort e) => 
    (ArithInOut.RoundedSquareRootEffort (IntervalElementaryFromBasis e)) 
    where
    type SqrtEffortIndicator (IntervalElementaryFromBasis e) = ArithUpDn.SqrtEffortIndicator e
    sqrtDefaultEffort (IntervalElementaryFromBasis (Interval l _)) = ArithUpDn.sqrtDefaultEffort l

instance (ArithUpDn.RoundedSquareRoot e) => 
    (ArithInOut.RoundedSquareRoot (IntervalElementaryFromBasis e)) 
    where
    sqrtInEff effort (IntervalElementaryFromBasis (Interval l r)) =
        IntervalElementaryFromBasis $
            Interval (ArithUpDn.sqrtUpEff effort l) (ArithUpDn.sqrtDnEff effort r)
    sqrtOutEff effort (IntervalElementaryFromBasis (Interval l r)) =
        IntervalElementaryFromBasis $
            Interval (ArithUpDn.sqrtDnEff effort l) (ArithUpDn.sqrtUpEff effort r)
        