{-# OPTIONS_GHC -fno-warn-orphans #-}
{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE FlexibleInstances #-}
{-# LANGUAGE TypeSynonymInstances #-}
{-|
    Module      :  Numeric.AERN.RealArithmetic.Interval.Conversion
    Description :  conversions between intervals and standard numeric types
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Conversion between intervals and standard numeric types.
    
    This module is hidden and reexported via its parent Interval. 
-}

module Numeric.AERN.RealArithmetic.Interval.Conversion 
()
where

import Numeric.AERN.RealArithmetic.RefinementOrderRounding

import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as ArithUpDn

import Numeric.AERN.Basics.Exception
import Numeric.AERN.Basics.Interval

import Control.Exception

--instance 
--    (ArithUpDn.Convertible t e, Show t) 
--    => 
--    Convertible t (Interval e) 
--    where
--    type ConvertEffortIndicator t (Interval e) = 
--        ArithUpDn.ConvertEffortIndicator t e
--    convertDefaultEffort i (Interval sampleE _) = ArithUpDn.convertDefaultEffort i sampleE 
--    convertInEff effort (Interval sampleE _) x =
--        Interval xUp xDn
--        where
--        xUp = convertUpEffException effort sampleE x
--        xDn = convertDnEffException effort sampleE x
--    convertOutEff effort (Interval sampleE _) x =
--        Interval xDn xUp
--        where
--        xUp = convertUpEffException effort sampleE x
--        xDn = convertDnEffException effort sampleE x

instance 
    (ArithUpDn.Convertible Rational e) 
    => 
    Convertible Rational (Interval e) 
    where
    type ConvertEffortIndicator Rational (Interval e) = 
        ArithUpDn.ConvertEffortIndicator Rational e
    convertDefaultEffort i (Interval sampleE _) = ArithUpDn.convertDefaultEffort i sampleE 
    convertInEff effort (Interval sampleE _) x =
        Interval xUp xDn
        where
        xUp = convertUpEffException effort sampleE x
        xDn = convertDnEffException effort sampleE x
    convertOutEff effort (Interval sampleE _) x =
        Interval xDn xUp
        where
        xUp = convertUpEffException effort sampleE x
        xDn = convertDnEffException effort sampleE x

instance 
    (ArithUpDn.Convertible Integer e) 
    => 
    Convertible Integer (Interval e) 
    where
    type ConvertEffortIndicator Integer (Interval e) = 
        ArithUpDn.ConvertEffortIndicator Integer e
    convertDefaultEffort i (Interval sampleE _) = ArithUpDn.convertDefaultEffort i sampleE 
    convertInEff effort (Interval sampleE _) x =
        Interval xUp xDn
        where
        xUp = convertUpEffException effort sampleE x
        xDn = convertDnEffException effort sampleE x
    convertOutEff effort (Interval sampleE _) x =
        Interval xDn xUp
        where
        xUp = convertUpEffException effort sampleE x
        xDn = convertDnEffException effort sampleE x


instance 
    (ArithUpDn.Convertible Int e) 
    => 
    Convertible Int (Interval e) 
    where
    type ConvertEffortIndicator Int (Interval e) = 
        ArithUpDn.ConvertEffortIndicator Int e
    convertDefaultEffort i (Interval sampleE _) = ArithUpDn.convertDefaultEffort i sampleE 
    convertInEff effort (Interval sampleE _) x =
        Interval xUp xDn
        where
        xUp = convertUpEffException effort sampleE x
        xDn = convertDnEffException effort sampleE x
    convertOutEff effort (Interval sampleE _) x =
        Interval xDn xUp
        where
        xUp = convertUpEffException effort sampleE x
        xDn = convertDnEffException effort sampleE x

instance 
    (ArithUpDn.Convertible Double e) 
    => 
    Convertible Double (Interval e) 
    where
    type ConvertEffortIndicator Double (Interval e) = 
        ArithUpDn.ConvertEffortIndicator Double e
    convertDefaultEffort i (Interval sampleE _) = ArithUpDn.convertDefaultEffort i sampleE 
    convertInEff effort (Interval sampleE _) x =
        Interval xUp xDn
        where
        xUp = convertUpEffException effort sampleE x
        xDn = convertDnEffException effort sampleE x
    convertOutEff effort (Interval sampleE _) x =
        Interval xDn xUp
        where
        xUp = convertUpEffException effort sampleE x
        xDn = convertDnEffException effort sampleE x



convertUpEffException ::
    (Show t1, ArithUpDn.Convertible t1 t2) 
    =>
    ArithUpDn.ConvertEffortIndicator t1 t2 -> t2 -> t1 -> t2
convertUpEffException effort sample x =
    case ArithUpDn.convertUpEff effort sample x of
       Just xUp -> xUp
       _ -> throw $ AERNException $
                "failed to convert to interval: x = " ++ show x
    
convertDnEffException ::
    (Show t1, ArithUpDn.Convertible t1 t2) 
    =>
    ArithUpDn.ConvertEffortIndicator t1 t2 -> t2 -> t1 -> t2
convertDnEffException effort sample x =
    case ArithUpDn.convertDnEff effort sample x of
       Just xDn -> xDn
       _ -> throw $ AERNException $
                "failed to convert to interval: x = " ++ show x
           
