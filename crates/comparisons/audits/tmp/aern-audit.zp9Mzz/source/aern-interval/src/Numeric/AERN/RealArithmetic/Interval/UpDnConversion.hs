{-# OPTIONS_GHC -fno-warn-orphans #-}
{-# LANGUAGE CPP #-}
{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE FlexibleInstances #-}
{-# LANGUAGE TypeSynonymInstances #-}
{-|
    Module      :  Numeric.AERN.RealArithmetic.Interval.UpDnConversion
    Description :  conversions between intervals and standard numeric types, rounded up/dn
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Conversion between intervals and standard numeric types.
    
    This module is hidden and reexported via its parent Interval. 
-}

module Numeric.AERN.RealArithmetic.Interval.UpDnConversion 
()
where

import Numeric.AERN.RealArithmetic.NumericOrderRounding

import Numeric.AERN.Basics.Interval


--instance (Convertible e t) => 
--        Convertible (Interval e) t where
--    type ConvertEffortIndicator (Interval e) t = 
--        ConvertEffortIndicator e t
--    convertDefaultEffort (Interval sampleE _) i = convertDefaultEffort sampleE i 
--    convertUpEff effort sampleT (Interval _ r) = convertUpEff effort sampleT r
--    convertDnEff effort sampleT (Interval l _) = convertDnEff effort sampleT l

instance (Convertible e Int) => 
        Convertible (Interval e) Int where
    type ConvertEffortIndicator (Interval e) Int = 
        ConvertEffortIndicator e Int
    convertDefaultEffort (Interval sampleE _) i = convertDefaultEffort sampleE i 
    convertUpEff effort sampleT (Interval _ r) = convertUpEff effort sampleT r
    convertDnEff effort sampleT (Interval l _) = convertDnEff effort sampleT l

instance (Convertible e Integer) => 
        Convertible (Interval e) Integer where
    type ConvertEffortIndicator (Interval e) Integer = 
        ConvertEffortIndicator e Integer
    convertDefaultEffort (Interval sampleE _) i = convertDefaultEffort sampleE i 
    convertUpEff effort sampleT (Interval _ r) = convertUpEff effort sampleT r
    convertDnEff effort sampleT (Interval l _) = convertDnEff effort sampleT l

instance (Convertible e Rational) => 
        Convertible (Interval e) Rational where
    type ConvertEffortIndicator (Interval e) Rational = 
        ConvertEffortIndicator e Rational
    convertDefaultEffort (Interval sampleE _) i = convertDefaultEffort sampleE i 
    convertUpEff effort sampleT (Interval _ r) = convertUpEff effort sampleT r
    convertDnEff effort sampleT (Interval l _) = convertDnEff effort sampleT l

instance (Convertible e Double) => 
        Convertible (Interval e) Double where
    type ConvertEffortIndicator (Interval e) Double = 
        ConvertEffortIndicator e Double
    convertDefaultEffort (Interval sampleE _) i = convertDefaultEffort sampleE i 
    convertUpEff effort sampleT (Interval _ r) = convertUpEff effort sampleT r
    convertDnEff effort sampleT (Interval l _) = convertDnEff effort sampleT l

--instance (Convertible e e) => 
--        Convertible (Interval e) e where
--    type ConvertEffortIndicator (Interval e) e = 
--        ConvertEffortIndicator e e
--    convertDefaultEffort (Interval sampleE _) i = convertDefaultEffort sampleE i 
--    convertUpEff effort sampleT (Interval _ r) = convertUpEff effort sampleT r
--    convertDnEff effort sampleT (Interval l _) = convertDnEff effort sampleT l


instance
    Convertible Int e
    =>
    Convertible Int (Interval e)
    where
    type ConvertEffortIndicator Int (Interval e) = 
        ConvertEffortIndicator Int e
    convertDefaultEffort sampleI (Interval sampleE _) =
        convertDefaultEffort sampleI sampleE
    convertUpEff eff (Interval sampleE _) n =
        fmap makeSingleton $ convertUpEff eff sampleE n
    convertDnEff eff (Interval sampleE _) n =
        fmap makeSingleton $ convertDnEff eff sampleE n

instance
    Convertible Integer e
    =>
    Convertible Integer (Interval e)
    where
    type ConvertEffortIndicator Integer (Interval e) = 
        ConvertEffortIndicator Integer e
    convertDefaultEffort sampleI (Interval sampleE _) =
        convertDefaultEffort sampleI sampleE
    convertUpEff eff (Interval sampleE _) n =
        fmap makeSingleton $ convertUpEff eff sampleE n
    convertDnEff eff (Interval sampleE _) n =
        fmap makeSingleton $ convertDnEff eff sampleE n

instance
    Convertible Rational e
    =>
    Convertible Rational (Interval e)
    where
    type ConvertEffortIndicator Rational (Interval e) = 
        ConvertEffortIndicator Rational e
    convertDefaultEffort sampleI (Interval sampleE _) =
        convertDefaultEffort sampleI sampleE
    convertUpEff eff (Interval sampleE _) n =
        fmap makeSingleton $ convertUpEff eff sampleE n
    convertDnEff eff (Interval sampleE _) n =
        fmap makeSingleton $ convertDnEff eff sampleE n

instance
    Convertible Double e
    =>
    Convertible Double (Interval e)
    where
    type ConvertEffortIndicator Double (Interval e) = 
        ConvertEffortIndicator Double e
    convertDefaultEffort sampleD (Interval sampleE _) =
        convertDefaultEffort sampleD sampleE
    convertUpEff eff (Interval sampleE _) d =
        fmap makeSingleton $ convertUpEff eff sampleE d
    convertDnEff eff (Interval sampleE _) d =
        fmap makeSingleton $ convertDnEff eff sampleE d

--instance 
--    (Convertible e e) 
--    => 
--    Convertible e (Interval e) 
--    where
--    type ConvertEffortIndicator e (Interval e) = 
--        ConvertEffortIndicator e e
--    convertDefaultEffort i (Interval sampleE _) = convertDefaultEffort i sampleE 
--    convertUpEff effort (Interval sampleE _) x =
--        case maybeXUp of
--            Just xUp -> Just $ Interval xUp xUp
--            _ -> Nothing
--        where
--        maybeXUp = convertUpEff effort sampleE x
--    convertDnEff effort (Interval sampleE _) x =
--        case maybeXDn of
--            Just xDn -> Just $ Interval xDn xDn
--            _ -> Nothing
--        where
--        maybeXDn = convertDnEff effort sampleE x

makeSingleton :: e -> Interval e
makeSingleton x = Interval x x
