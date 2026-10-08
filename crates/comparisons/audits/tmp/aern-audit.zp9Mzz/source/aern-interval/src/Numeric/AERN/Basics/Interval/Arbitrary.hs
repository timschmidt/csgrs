{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE UndecidableInstances #-}
{-|
    Module      :  Numeric.AERN.Basics.Interval.Arbitrary
    Description :  interval instances of arbitrary with area 
    Copyright   :  (c) Michal Konecny, Jan Duracz
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Interval instances of arbitrary with area.
    
    This is a hidden module reexported via its parent.
-}
module Numeric.AERN.Basics.Interval.Arbitrary where

import Prelude hiding (EQ, LT, GT)

import Numeric.AERN.Basics.Consistency
import Numeric.AERN.Basics.Arbitrary

import Numeric.AERN.Basics.Interval.Basics

import qualified Numeric.AERN.NumericOrder as NumOrd
import Numeric.AERN.NumericOrder.Operators

instance 
    (NumOrd.ArbitraryOrderedTuple e,
     NumOrd.RoundedLattice e,
     AreaHasForbiddenValues e,
     NumOrd.PartialComparison e) 
    =>
    ArbitraryWithArea (Interval e)
    where
    type Area (Interval e) = (Area e, AreaConsistencyConstraint)
    areaWhole (Interval l _) = (areaWhole l, AreaMaybeAllowOnlyWithConsistencyStatus Nothing)
    arbitraryInArea (area, AreaMaybeAllowOnlyWithConsistencyStatus maybeConsistencyStatus) =
        avoidForbidden (100 :: Int)
        where
        avoidForbidden maxTries =
            do
            e1 <- arbitraryInArea area
            e2 <- arbitraryInArea area
            let l = NumOrd.minDnEff (NumOrd.minmaxDefaultEffort e1) e1 e2
            let r = NumOrd.maxUpEff (NumOrd.minmaxDefaultEffort e1) e1 e2
            let result = case maybeConsistencyStatus of
                            Nothing -> Interval e1 e2
                            Just Consistent -> Interval l r
                            Just Anticonsistent -> Interval r l
                            Just Exact -> Interval l l
            case nothingForbiddenInsideInterval area result of
                True -> return result
                _ | maxTries > 0 -> avoidForbidden $ maxTries - 1
                _ -> error "aern-interval: internal error in arbitraryInArea: failed to avoid forbidden values"
        
nothingForbiddenInsideInterval :: 
     (NumOrd.PartialComparison e,
      AreaHasForbiddenValues e) 
     =>
     Area e -> Interval e -> Bool
nothingForbiddenInsideInterval area interval =
    and $ map (notInside interval) $ areaGetForbiddenValues area
    where
    notInside (Interval l r) value = 
        ((value <? l) == Just True)
        ||
        ((value >? r) == Just True)  
        

