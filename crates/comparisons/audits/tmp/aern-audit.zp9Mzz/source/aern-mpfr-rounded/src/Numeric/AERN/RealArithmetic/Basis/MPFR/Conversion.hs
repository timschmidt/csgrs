{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE TypeSynonymInstances #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE FlexibleInstances #-}
{-|
    Module      :  Numeric.AERN.RealArithmetic.Basis.MPFR.Conversion
    Description :  conversions between MPFR and standard numeric types
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable

    Conversions between MPFR and standard numeric types.

    This is a private module reexported publicly via its parent.
-}

module Numeric.AERN.RealArithmetic.Basis.MPFR.Conversion where

import Numeric.AERN.RealArithmetic.Basis.MPFR.Basics
import Numeric.AERN.RealArithmetic.Basis.MPFR.ExactOps
import Numeric.AERN.RealArithmetic.Basis.MPFR.FieldOps

import qualified Numeric.Rounded as R

import Numeric.AERN.RealArithmetic.NumericOrderRounding
import Numeric.AERN.RealArithmetic.ExactOps

import Numeric.AERN.Basics.Exception
import Control.Exception

import Data.Ratio



instance Convertible Integer MPFR where
    type ConvertEffortIndicator Integer MPFR = ()
    convertDefaultEffort n sampleD = ()
    convertUpEff _ sampleD n =
        Just $ withPrec (getPrecision sampleD) $ fromInteger n 
    convertDnEff _ sampleD n = 
        Just $ negate $ withPrec (getPrecision sampleD) $ fromInteger (-n) 

instance Convertible Int MPFR where
    type ConvertEffortIndicator Int MPFR = ()
    convertDefaultEffort n sampleD = ()
    convertUpEff _ sampleD n =
        Just $ withPrec (getPrecision sampleD) $ fromInteger $ toInteger n 
    convertDnEff _ sampleD n = 
        Just $ negate $ withPrec (getPrecision sampleD) $ fromInteger $ toInteger (-n) 

instance Convertible MPFR Integer where
    type ConvertEffortIndicator MPFR Integer = ()
    convertDefaultEffort _ _ = ()
    convertUpEff _ _ d@(MPFR v)
        | isInfinite v = Nothing
        | otherwise = Just $ ceiling v
    convertDnEff _ _ d@(MPFR v)
        | isInfinite v = Nothing
        | otherwise = Just $ floor v

instance Convertible MPFR Int where
    type ConvertEffortIndicator MPFR Int = ()
    convertDefaultEffort _ _ = ()
    convertUpEff effort _ d =
        case mdUpInteger of
            Nothing -> Nothing
            Just dUpInteger
                | dUpInteger > (toInteger intMax) -> Nothing
                | otherwise -> Just $ fromInteger dUpInteger
        where
        mdUpInteger = convertUpEff effort (0 :: Integer) d
        intMax = maxBound :: Int
    convertDnEff effort _ d =
        case mdDnInteger of
            Nothing -> Nothing
            Just dDnInteger
                | dDnInteger < (toInteger intMin) -> Nothing
                | otherwise -> Just $ fromInteger dDnInteger
        where
        mdDnInteger = convertDnEff effort (0 :: Integer) d
        intMin = minBound :: Int

instance Convertible MPFR Double where
    type ConvertEffortIndicator MPFR Double = ()
    convertDefaultEffort _ _ = ()
-- TODO
--    convertUpEff _ _ d
--        | d == plusInfinity d = Just (1/0) 
--        | d == minusInfinity d = Just (-1/0)
--        | otherwise =
--            case M.toDouble2exp M.Up d of
--                (s1,e) ->
--                   case encodeFloat (fst $ decodeFloat s1) (e - 53) of
--                       d2 | d2 == -(1/0) -> Just dblMinBound
--                       d2 | d2 == 0 && d > 0 -> Just dblEpsilon
--                       d2 -> Just d2 
--    convertDnEff _ _ d
--        | d == plusInfinity d = Just (1/0) 
--        | d == minusInfinity d = Just (-1/0)
--        | otherwise =
--            case M.toDouble2exp M.Down d of
--                (s1,e) ->
--                   case encodeFloat (fst $ decodeFloat s1) (e - 53) of
--                       d2 | d2 == (1/0) -> Just dblMaxBound
--                       d2 | d2 == 0 && d < 0 -> Just (- dblEpsilon)
--                       d2 -> Just d2 


(dblSigMaxBound, dblExpMaxBound) = decodeFloat $ (0/1 :: Double)

dblMinBound, dblMaxBound :: Double
dblMinBound = encodeFloat (negate $ dblSigMaxBound - 1) dblExpMaxBound
dblMaxBound = - dblMinBound

dblEpsilon :: Double
dblEpsilon = encodeFloat 1 (-1074)

instance Convertible Double MPFR where
    type ConvertEffortIndicator Double MPFR = ()
    convertDefaultEffort n sampleD = ()
    convertUpEff _ sampleD d =
        Just $ withPrec (getPrecision sampleD) $ R.fromDouble d 
    convertDnEff _ sampleD d = 
        Just $ negate $ withPrec (getPrecision sampleD) $ R.fromDouble (-d) 


instance Convertible Rational MPFR where
    type ConvertEffortIndicator Rational MPFR = ()
    convertDefaultEffort n sampleD = ()
    convertUpEff _ sampleD d =
        Just $ withPrec (getPrecision sampleD) $ fromRational d 
    convertDnEff _ sampleD d = 
        Just $ negate $ withPrec (getPrecision sampleD) $ fromRational (-d) 
        
positiveRational2MPFRUp prec r =        
        case (convertUpEff prec 0 $ numerator r, convertDnEff prec 0 $ denominator r) of
            (Just num, Just den) -> Just $ divUpEff () num den
            _ -> Nothing
positiveRational2MPFRDn prec r =
        case (convertDnEff prec 0 $ numerator r, convertUpEff prec 0 $ denominator r) of
            (Just num, Just den) -> Just $ divDnEff () num den
            _ -> Nothing

instance Convertible MPFR Rational where
    type ConvertEffortIndicator MPFR Rational = ()
    convertDefaultEffort _ _ = ()
    convertUpEff _ _ d@(MPFR v)
        | isInfinite v = Nothing
        | otherwise = Just $ toRational v
    convertDnEff = convertUpEff

