{-# LANGUAGE ForeignFunctionInterface #-}
-----------------------------------------------------------------------------
-- |
-- Module      :  Numeric.MPFR.Raw.Unsafe
-- Copyright   :  (C) 2012-2014 Edward Kmett, Daniel Peebles
--                (C) 2013-2017 Claude Heiland-Allen
-- License     :  LGPL
-- Maintainer  :  Claude Heiland-Allen <claude@mathr.co.uk>
-- Stability   :  experimental
-- Portability :  non-portable
--
-- This module contains FFI imports as unsafe ccalls.
----------------------------------------------------------------------------
module Numeric.MPFR.Raw.Unsafe where

import Foreign.C (CInt(..), CIntMax(..), CSize(..), CChar(..), CLong(..))

import GHC.Exts (Ptr)

import Numeric.LongDouble (LongDouble)
import Numeric.GMP.Types (MPZ, MPQ)

import Numeric.MPFR.Types

foreign import ccall unsafe "mpfr_init2" mpfr_init2 :: Ptr MPFR -> MPFRPrec -> IO ()
foreign import ccall unsafe "mpfr_clear" mpfr_clear :: Ptr MPFR -> IO ()

foreign import ccall unsafe "mpfr_sgn" mpfr_sgn :: Ptr MPFR -> IO CInt
foreign import ccall unsafe "mpfr_get_d" mpfr_get_d :: Ptr MPFR -> MPFRRnd -> IO Double
foreign import ccall unsafe "wrapped_mpfr_get_z" wrapped_mpfr_get_z :: Ptr MPZ -> Ptr MPFR -> MPFRRnd -> Ptr CInt -> IO CInt

foreign import ccall unsafe "mpfr_get_str" mpfr_get_str :: Ptr CChar -> Ptr MPFRExp -> Int -> CSize -> Ptr MPFR -> MPFRRnd -> IO (Ptr CChar)
foreign import ccall unsafe "mpfr_free_str" mpfr_free_str :: Ptr CChar -> IO ()

foreign import ccall unsafe "mpfr_set_z" mpfr_set_z :: Ptr MPFR -> Ptr MPZ -> MPFRRnd -> IO CInt
foreign import ccall unsafe "__gmpfr_set_sj" mpfr_set_sj :: Ptr MPFR -> CIntMax -> MPFRRnd -> IO CInt
foreign import ccall unsafe "mpfr_set_q" mpfr_set_q :: Ptr MPFR -> Ptr MPQ -> MPFRRnd -> IO CInt
foreign import ccall unsafe "mpfr_set_d" mpfr_set_d :: Ptr MPFR -> Double -> MPFRRnd -> IO CInt

type Test = Ptr MPFR -> IO CInt

foreign import ccall unsafe "mpfr_nan_p" mpfr_nan_p :: Test
foreign import ccall unsafe "mpfr_inf_p" mpfr_inf_p :: Test
foreign import ccall unsafe "mpfr_zero_p" mpfr_zero_p :: Test
foreign import ccall unsafe "mpfr_signbit" mpfr_signbit :: Test

foreign import ccall unsafe "wrapped_mpfr_get_z_2exp" wrapped_mpfr_get_z_2exp :: Ptr MPZ -> Ptr MPFR -> Ptr CInt -> IO MPFRExp
foreign import ccall unsafe "mpfr_set_z_2exp" mpfr_set_z_2exp :: Ptr MPFR -> Ptr MPZ -> MPFRExp -> MPFRRnd -> IO CInt

type Constant = Ptr MPFR -> MPFRRnd -> IO CInt

foreign import ccall unsafe "mpfr_const_pi" mpfr_const_pi :: Constant
foreign import ccall unsafe "mpfr_const_log2" mpfr_const_log2 :: Constant
foreign import ccall unsafe "mpfr_const_euler" mpfr_const_euler :: Constant
foreign import ccall unsafe "mpfr_const_catalan" mpfr_const_catalan :: Constant

type Unary = Ptr MPFR -> Ptr MPFR -> MPFRRnd -> IO CInt

foreign import ccall unsafe "mpfr_set" mpfr_set :: Unary

foreign import ccall unsafe "mpfr_abs" mpfr_abs :: Unary
foreign import ccall unsafe "mpfr_neg" mpfr_neg :: Unary
foreign import ccall unsafe "mpfr_log" mpfr_log :: Unary
foreign import ccall unsafe "mpfr_exp" mpfr_exp :: Unary
foreign import ccall unsafe "mpfr_sqrt" mpfr_sqrt :: Unary
foreign import ccall unsafe "mpfr_sin" mpfr_sin :: Unary
foreign import ccall unsafe "mpfr_cos" mpfr_cos :: Unary
foreign import ccall unsafe "mpfr_tan" mpfr_tan :: Unary
foreign import ccall unsafe "mpfr_asin" mpfr_asin :: Unary
foreign import ccall unsafe "mpfr_acos" mpfr_acos :: Unary
foreign import ccall unsafe "mpfr_atan" mpfr_atan :: Unary
foreign import ccall unsafe "mpfr_sinh" mpfr_sinh :: Unary
foreign import ccall unsafe "mpfr_cosh" mpfr_cosh :: Unary
foreign import ccall unsafe "mpfr_tanh" mpfr_tanh :: Unary
foreign import ccall unsafe "mpfr_asinh" mpfr_asinh :: Unary
foreign import ccall unsafe "mpfr_acosh" mpfr_acosh :: Unary
foreign import ccall unsafe "mpfr_atanh" mpfr_atanh :: Unary
foreign import ccall unsafe "mpfr_log1p" mpfr_log1p :: Unary
foreign import ccall unsafe "mpfr_expm1" mpfr_expm1 :: Unary
foreign import ccall unsafe "mpfr_rint" mpfr_rint :: Unary

type Unary' = Ptr MPFR -> Ptr MPFR -> IO CInt

foreign import ccall unsafe "mpfr_trunc" mpfr_trunc :: Unary'
foreign import ccall unsafe "mpfr_ceil" mpfr_ceil :: Unary'
foreign import ccall unsafe "mpfr_floor" mpfr_floor :: Unary'

type Binary = Ptr MPFR -> Ptr MPFR -> Ptr MPFR -> MPFRRnd -> IO CInt

foreign import ccall unsafe "mpfr_add" mpfr_add :: Binary
foreign import ccall unsafe "mpfr_sub" mpfr_sub :: Binary
foreign import ccall unsafe "mpfr_mul" mpfr_mul :: Binary
foreign import ccall unsafe "mpfr_div" mpfr_div :: Binary
foreign import ccall unsafe "mpfr_min" mpfr_min :: Binary
foreign import ccall unsafe "mpfr_max" mpfr_max :: Binary
foreign import ccall unsafe "mpfr_atan2" mpfr_atan2 :: Binary

foreign import ccall unsafe "mpfr_modf" mpfr_modf :: Binary

type Comparison = Ptr MPFR -> Ptr MPFR -> IO CInt

foreign import ccall unsafe "mpfr_cmp"            mpfr_cmp            :: Comparison
foreign import ccall unsafe "mpfr_equal_p"        mpfr_equal_p        :: Comparison
foreign import ccall unsafe "mpfr_lessgreater_p"  mpfr_lessgreater_p  :: Comparison
foreign import ccall unsafe "mpfr_less_p"         mpfr_less_p         :: Comparison
foreign import ccall unsafe "mpfr_greater_p"      mpfr_greater_p      :: Comparison
foreign import ccall unsafe "mpfr_lessequal_p"    mpfr_lessequal_p    :: Comparison
foreign import ccall unsafe "mpfr_greaterequal_p" mpfr_greaterequal_p :: Comparison

foreign import ccall unsafe "mpfr_nextabove" mpfr_nextabove :: Ptr MPFR -> IO ()
foreign import ccall unsafe "mpfr_nextbelow" mpfr_nextbelow :: Ptr MPFR -> IO ()

foreign import ccall unsafe "wrapped_mpfr_get_ld" wrapped_mpfr_get_ld :: Ptr LongDouble -> Ptr MPFR -> MPFRRnd -> Ptr CInt -> IO CInt
foreign import ccall unsafe "wrapped_mpfr_get_ld_2exp" wrapped_mpfr_get_ld_2exp :: Ptr LongDouble -> Ptr CLong -> Ptr MPFR -> MPFRRnd -> Ptr CInt -> IO CInt
foreign import ccall unsafe "wrapped_mpfr_set_ld" wrapped_mpfr_set_ld :: Ptr MPFR -> Ptr LongDouble -> MPFRRnd -> IO CInt
foreign import ccall unsafe "wrapped_mpfr_cmp_ld" wrapped_mpfr_cmp_ld :: Ptr MPFR -> Ptr LongDouble -> IO CInt
