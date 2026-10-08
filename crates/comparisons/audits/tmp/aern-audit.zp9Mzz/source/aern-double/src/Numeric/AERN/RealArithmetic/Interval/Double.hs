{-# LANGUAGE FlexibleInstances, MultiParamTypeClasses, TypeFamilies #-}
{-|
    Module      :  Numeric.AERN.RealArithmetic.Interval.Double
    Description :  Interval Double utilities and tests  
    Copyright   :  (c) Michal Konecny, Jan Duracz
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Interval Double utilities and tests.
-}
module Numeric.AERN.RealArithmetic.Interval.Double 
(
    DI,
    sampleDI,
    PositiveDI(..),
    width,
    bisect,
    module Numeric.AERN.Basics.Interval
)
where

import Numeric.AERN.RealArithmetic.NumericOrderRounding

import Numeric.AERN.Basics.Interval

import Numeric.AERN.RealArithmetic.Interval

import Numeric.AERN.RealArithmetic.Basis.Double.ShowInternals
import Numeric.AERN.RealArithmetic.Basis.Double.FieldOps

import qualified Numeric.AERN.NumericOrder as NumOrd
import Numeric.AERN.RealArithmetic.RefinementOrderRounding.Operators

import Test.QuickCheck

type DI = Interval Double 

sampleDI :: DI
sampleDI = Interval 0 0

newtype PositiveDI = PositiveDI { unPositiveDI :: DI }

instance Show PositiveDI where
    show (PositiveDI i) = show i

instance Arbitrary PositiveDI
    where
    arbitrary =
        do
        NumOrd.UniformlyOrderedPair (l,r) <- arbitrary
        return $ PositiveDI (Interval (pos l) (pos r))
        where
        pos e 
            | e > 0 =  e
            | e == 0 =  1
            | otherwise = (-e) 

{-|
    Calculate the width of an interval.
    The result may not be thin since the width calculation uses outwards rounding.
-}
width :: DI -> DI
width i = 
  irI <-> ilI
  where
  irI = Interval ir ir  
  ilI = Interval il il
  Interval il ir = i  

{-|
    Split an interval into two subintervals that share only one point 
    and whose union is the original interval.
-}
bisect ::
    Maybe Double {-^ optional parameter, indicating where to split the interval -} -> 
    DI {-^ the interval to split -} -> 
    (DI, DI)
bisect maybeMidpoint i =
  (l,r)
  where
  r = Interval midpoint ir
  l = Interval il midpoint
  midpoint 
    = case maybeMidpoint of
        Nothing -> 0.5*(il + ir)
        Just m -> m
  Interval il ir = i
            