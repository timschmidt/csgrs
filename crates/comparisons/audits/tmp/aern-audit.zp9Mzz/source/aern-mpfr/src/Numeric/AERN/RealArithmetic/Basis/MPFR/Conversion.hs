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

import Numeric.AERN.RealArithmetic.Basis.MPFR.ExactOps
import Numeric.AERN.RealArithmetic.Basis.MPFR.FieldOps

import Numeric.AERN.RealArithmetic.NumericOrderRounding
import Numeric.AERN.RealArithmetic.ExactOps

import Numeric.AERN.Basics.Exception
import Control.Exception

import Data.Ratio

import qualified Data.Number.MPFR as M
import Data.Number.MPFR (MPFR)
import Data.Number.MPFR.Instances.Up
import qualified Data.Number.MPFR.Mutable as MM


instance Convertible Integer MPFR where
    type ConvertEffortIndicator Integer MPFR = M.Precision
    convertDefaultEffort n sampleD =
        max precD $ 1 + M.bitsInInteger n -- see hmpfr Num instance
        where
        precD = M.getPrec sampleD
    convertUpEff prec sampleD n = 
        Just $ M.fromIntegerA M.Up (max prec precD) n 
        where
        precD = M.getPrec sampleD
    convertDnEff prec sampleD n = 
        Just $ M.fromIntegerA M.Down (max prec precD) n
        where
        precD = M.getPrec sampleD

instance Convertible Int MPFR where
    type ConvertEffortIndicator Int MPFR = M.Precision
    convertDefaultEffort _ _ = 100
    convertUpEff prec sampleD n = Just $ M.fromInt M.Up (max prec precD) n 
        where
        precD = M.getPrec sampleD
    convertDnEff prec sampleD n = Just $ M.fromInt M.Down (max prec precD) n
        where
        precD = M.getPrec sampleD

instance Convertible MPFR Integer where
    type ConvertEffortIndicator MPFR Integer = ()
    convertDefaultEffort _ _ = ()
    convertUpEff _ _ d 
        | M.isInfinite d = Nothing
        | otherwise = Just $ ceiling d
    convertDnEff _ _ d 
        | M.isInfinite d = Nothing
        | otherwise = Just $ floor d

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
    convertUpEff _ _ d
        | d == plusInfinity d = Just (1/0) 
        | d == minusInfinity d = Just (-1/0)
        | otherwise =
            case M.toDouble2exp M.Up d of
                (s1,e) ->
                   case encodeFloat (fst $ decodeFloat s1) (e - 53) of
                       d2 | d2 == -(1/0) -> Just dblMinBound
                       d2 | d2 == 0 && d > 0 -> Just dblEpsilon
                       d2 -> Just d2 
    convertDnEff _ _ d
        | d == plusInfinity d = Just (1/0) 
        | d == minusInfinity d = Just (-1/0)
        | otherwise =
            case M.toDouble2exp M.Down d of
                (s1,e) ->
                   case encodeFloat (fst $ decodeFloat s1) (e - 53) of
                       d2 | d2 == (1/0) -> Just dblMaxBound
                       d2 | d2 == 0 && d < 0 -> Just (- dblEpsilon)
                       d2 -> Just d2 


(dblSigMaxBound, dblExpMaxBound) = decodeFloat $ (0/1 :: Double)

dblMinBound, dblMaxBound :: Double
dblMinBound = encodeFloat (negate $ dblSigMaxBound - 1) dblExpMaxBound
dblMaxBound = - dblMinBound

dblEpsilon :: Double
dblEpsilon = encodeFloat 1 (-1074)

instance Convertible Double MPFR where
    type ConvertEffortIndicator Double MPFR = M.Precision
    convertDefaultEffort _ _ = 100
    convertUpEff prec _ d = Just $ M.fromDouble M.Up prec d 
    convertDnEff prec _ d = Just $ M.fromDouble M.Down prec d

instance Convertible Rational MPFR where
    type ConvertEffortIndicator Rational MPFR = M.Precision
    convertDefaultEffort _ _ = 100
    convertUpEff prec sampleD r 
        | r < 0 = fmap negate (positiveRational2MPFRDn (max prec precD) (- r))
        | otherwise = positiveRational2MPFRUp prec r
        where
        precD = M.getPrec sampleD
    convertDnEff prec sampleD r 
        | r < 0 = fmap negate (positiveRational2MPFRUp (max prec precD) (- r))
        | otherwise = positiveRational2MPFRDn prec r
        where
        precD = M.getPrec sampleD
        
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
    convertUpEff _ _ d
        | M.isInfinite d = Nothing
        | otherwise = Just $ toRational d
    convertDnEff = convertUpEff

