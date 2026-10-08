{-# LANGUAGE ForeignFunctionInterface #-}
#include <hsmpfr.h>
#include <mpfr.h>

module Data.Number.FFIhelper where

import Data.Word

import Data.Int

import Foreign.C.String(CString)
import Foreign.C.Types(CULong, CLong, CInt, CUInt, CDouble, CChar)
import Foreign.Ptr(FunPtr, Ptr)


import Foreign.Storable
import Foreign.ForeignPtr (ForeignPtr, withForeignPtr)

data RoundMode = Near | Zero | Up | Down | GMP_RND_MAX | GMP_RNDNA 

instance Enum RoundMode where
    fromEnum Near        = #{const GMP_RNDN} 
    fromEnum Zero        = #{const GMP_RNDZ} 
    fromEnum Up          = #{const GMP_RNDU} 
    fromEnum Down        = #{const GMP_RNDD} 
    fromEnum GMP_RND_MAX = #{const GMP_RND_MAX}
    fromEnum GMP_RNDNA    = #{const GMP_RNDNA}
    
    toEnum #{const GMP_RNDN}    = Near
    toEnum #{const GMP_RNDZ}    = Zero
    toEnum #{const GMP_RNDU}    = Up
    toEnum #{const GMP_RNDD}    = Down
    toEnum #{const GMP_RND_MAX} = GMP_RND_MAX
    toEnum (#{const GMP_RNDNA}) = GMP_RNDNA
    toEnum i                    = error $ "RoundMode.toEnum called with illegal argument :" ++ show i 


data MPFR_T = MP CPrecision Sign Exp !(ForeignPtr Limb)

instance Storable MPFR_T where
    sizeOf _ = #size __mpfr_struct
    alignment _ = (undefined :: Int)
    peek = error "Not needed and not applicable"
    poke p (MP prec s e fp) = do withForeignPtr fp $ \p1 -> do 
                                      #{poke __mpfr_struct, _mpfr_prec} p prec
                                      #{poke __mpfr_struct, _mpfr_sign} p s 
                                      #{poke __mpfr_struct, _mpfr_exp} p e
                                      #{poke __mpfr_struct, _mpfr_d} p p1

peekP      :: Ptr MPFR_T -> ForeignPtr Limb -> IO MPFR_T
peekP p fp = do r11 <- #{peek __mpfr_struct, _mpfr_prec} p
	        r21 <- #{peek __mpfr_struct, _mpfr_sign} p
		r22 <- #{peek __mpfr_struct, _mpfr_exp} p
                return (MP r11 r21 r22 fp)

bitsPerMPLimb :: Int 
bitsPerMPLimb = 8 * #size mp_limb_t

type CRoundMode = CInt

type Limb = #type mp_limb_t

type Sign = #type mpfr_sign_t

type CPrecision = #type mpfr_prec_t

type Exp = #type mp_exp_t

type MpSize = #type mp_size_t

--data MPFR_T = MPFR_T

-- utility functions from hsmpfr.h
foreign import ccall unsafe "initS"
        initS :: CPrecision -> IO (Ptr MPFR_T)

foreign import ccall unsafe "&clear"
        clear :: FunPtr (Ptr MPFR_T -> IO ())

--------------------
foreign import ccall unsafe "mpfr_get_prec_wrap"
        mpfr_get_prec :: Ptr MPFR_T -> IO CPrecision 

----------------------------------------------------------------

-- assignment functions
foreign import ccall unsafe "mpfr_set_wrap"
        mpfr_set :: Ptr MPFR_T -> Ptr MPFR_T -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_set_ui_wrap"
        mpfr_set_ui :: Ptr MPFR_T -> CULong -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_set_si_wrap"
        mpfr_set_si :: Ptr MPFR_T -> CLong -> CRoundMode -> IO CInt

--TODO set_uj, set_sj

foreign import ccall unsafe "mpfr_set_d"
        mpfr_set_d :: Ptr MPFR_T -> CDouble -> CRoundMode -> IO CInt

--foreign import ccall unsafe "mfpr_set_ld"
  --      mpfr_set_ld :: Ptr MPFR_T -> #{type long double} -> CRoundMode -> IO CInt
--long double does not seem to be supported

--TODO set_decimal64, set_z, set_q, set_f

foreign import ccall unsafe "mpfr_set_ui_2exp"
        mpfr_set_ui_2exp :: Ptr MPFR_T -> CULong -> CInt -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_set_si_2exp"
        mpfr_set_si_2exp :: Ptr MPFR_T -> CLong -> CInt -> CRoundMode -> IO CInt

--TODO set_uj_2exp, set_sj_2exp

foreign import ccall unsafe "mpfr_set_str"
        mpfr_set_str :: Ptr MPFR_T -> CString -> CInt -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_strtofr"
        mpfr_strtofr :: Ptr MPFR_T  ->  CString -> Ptr (Ptr CChar) -> CInt -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_set_inf"
        mpfr_set_inf :: Ptr MPFR_T -> CInt -> IO ()

foreign import ccall unsafe "mpfr_set_nan"
        mpfr_set_nan :: Ptr MPFR_T -> IO ()

foreign import ccall unsafe "mpfr_swap"
        mpfr_swap :: Ptr MPFR_T -> Ptr MPFR_T -> IO ()

-- THINK combined initialization and assignment functions are non-applicable
-- with custom interface?

--------------------------------------------------------------------------------


-- conversion functions
foreign import ccall unsafe "mpfr_get_d"
        mpfr_get_d :: Ptr MPFR_T -> CRoundMode -> IO CDouble

-- TODO get_decimal64

foreign import ccall unsafe "mpfr_get_d_2exp"
        mpfr_get_d_2exp :: Ptr CLong -> Ptr MPFR_T -> CRoundMode -> IO CDouble

-- TODO get_ld_2exp
-- !!!!!!! next 4 set erange flags
foreign import ccall unsafe "mpfr_get_si" 
        mpfr_get_si :: Ptr MPFR_T -> CRoundMode -> IO CLong

foreign import ccall unsafe "mpfr_get_ui" 
        mpfr_get_ui :: Ptr MPFR_T -> CRoundMode -> IO CULong

{-
foreign import ccall unsafe "mpfr_get_sj_wrap"
        mpfr_get_sj :: Ptr MPFR_T -> CRoundMode -> IO #type intmax_t

foreign import ccall unsafe "mpfr_get_uj_wrap"
        mpft_get_uj :: Ptr MPFR_T -> CRoundMode -> IO #type uintmax_t
-}
--TODO get_z_exp, get_z, get_f, 

foreign import ccall unsafe "mpfr_get_str"
        mpfr_get_str :: CString -> Ptr CInt -> CInt -> CUInt -> Ptr MPFR_T ->  CRoundMode -> IO CString

foreign import ccall unsafe "mpfr_free_str"
        mpfr_free_str :: CString -> IO ()

foreign import ccall unsafe "mpfr_fits_ulong_p"
        mpfr_fits_ulong_p :: Ptr MPFR_T -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_fits_slong_p"
        mpfr_fits_slong_p :: Ptr MPFR_T -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_fits_uint_p"
        mpfr_fits_uint_p :: Ptr MPFR_T -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_fits_sint_p"
        mpfr_fits_sint_p :: Ptr MPFR_T -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_fits_ushort_p"
        mpfr_fits_ushort_p :: Ptr MPFR_T -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_fits_sshort_p"
        mpfr_fits_sshort_p :: Ptr MPFR_T -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_fits_intmax_p"
        mpfr_fits_intmax_p :: Ptr MPFR_T -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_fits_uintmax_p"
        mpfr_fits_uintmax_p :: Ptr MPFR_T -> CRoundMode -> IO CInt


-------------------------------------------------------------------------------

-- basic arithmetic functions

foreign import ccall unsafe "mpfr_add"
        mpfr_add :: Ptr MPFR_T -> Ptr MPFR_T -> Ptr MPFR_T -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_add_ui"
        mpfr_add_ui :: Ptr MPFR_T -> Ptr MPFR_T -> CULong -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_add_si"
        mpfr_add_si :: Ptr MPFR_T -> Ptr MPFR_T -> CLong -> CRoundMode -> IO CInt

-- TODO add_z, add_q

foreign import ccall unsafe "mpfr_sub"
        mpfr_sub :: Ptr MPFR_T -> Ptr MPFR_T -> Ptr MPFR_T -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_ui_sub" 
        mpfr_ui_sub :: Ptr MPFR_T -> CULong -> Ptr MPFR_T -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_sub_ui"
        mpfr_sub_ui :: Ptr MPFR_T -> Ptr MPFR_T -> CULong -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_si_sub" 
        mpfr_si_sub :: Ptr MPFR_T -> CLong -> Ptr MPFR_T -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_sub_si"
        mpfr_sub_si :: Ptr MPFR_T -> Ptr MPFR_T -> CLong -> CRoundMode -> IO CInt

--TODO sub_z, sub_q

foreign import ccall unsafe "mpfr_mul"
        mpfr_mul :: Ptr MPFR_T -> Ptr MPFR_T -> Ptr MPFR_T -> CRoundMode -> IO CInt 

foreign import ccall unsafe "mpfr_mul_ui"
        mpfr_mul_ui :: Ptr MPFR_T -> Ptr MPFR_T -> CULong -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_mul_si"
        mpfr_mul_si :: Ptr MPFR_T -> Ptr MPFR_T -> CLong -> CRoundMode -> IO CInt

--TODO mul_z, mul_q

foreign import ccall unsafe "mpfr_sqr"
        mpfr_sqr :: Ptr MPFR_T -> Ptr MPFR_T -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_div"
        mpfr_div :: Ptr MPFR_T -> Ptr MPFR_T -> Ptr MPFR_T -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_ui_div"
        mpfr_ui_div :: Ptr MPFR_T -> CULong -> Ptr MPFR_T -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_div_ui"
        mpfr_div_ui :: Ptr MPFR_T -> Ptr MPFR_T -> CULong -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_si_div"
        mpfr_si_div :: Ptr MPFR_T -> CLong -> Ptr MPFR_T -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_div_si"
        mpfr_div_si :: Ptr MPFR_T -> Ptr MPFR_T -> CLong -> CRoundMode -> IO CInt

-- TODO div_z, div_q

foreign import ccall unsafe "mpfr_sqrt"
        mpfr_sqrt :: Ptr MPFR_T -> Ptr MPFR_T -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_sqrt_ui"
        mpfr_sqrt_ui :: Ptr MPFR_T -> CULong -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_cbrt"
        mpfr_cbrt :: Ptr MPFR_T -> Ptr MPFR_T -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_root"
        mpfr_root :: Ptr MPFR_T -> Ptr MPFR_T -> CULong -> CRoundMode -> IO CInt 

foreign import ccall unsafe "mpfr_pow"
        mpfr_pow :: Ptr MPFR_T -> Ptr MPFR_T -> Ptr MPFR_T -> CRoundMode -> IO CInt 

foreign import ccall unsafe "mpfr_pow_ui"
        mpfr_pow_ui :: Ptr MPFR_T -> Ptr MPFR_T -> CULong -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_pow_si"
        mpfr_pow_si :: Ptr MPFR_T -> Ptr MPFR_T -> CLong -> CRoundMode -> IO CInt

--TODO pow_z

foreign import ccall unsafe "mpfr_ui_pow_ui"
        mpfr_ui_pow_ui :: Ptr MPFR_T -> CULong -> CULong -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_ui_pow"
        mpfr_ui_pow :: Ptr MPFR_T -> CULong -> Ptr MPFR_T -> CRoundMode -> IO CInt


foreign import ccall unsafe "mpfr_neg"
        mpfr_neg :: Ptr MPFR_T -> Ptr MPFR_T -> CRoundMode -> IO CInt 

foreign import ccall unsafe "mpfr_abs_wrap"
        mpfr_abs :: Ptr MPFR_T -> Ptr MPFR_T -> CRoundMode -> IO CInt 

foreign import ccall unsafe "mpfr_dim"
        mpfr_dim :: Ptr MPFR_T -> Ptr MPFR_T -> Ptr MPFR_T -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_mul_2ui"
        mpfr_mul_2ui :: Ptr MPFR_T -> Ptr MPFR_T -> CULong -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_mul_2si"
        mpfr_mul_2si :: Ptr MPFR_T -> Ptr MPFR_T -> CLong -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_div_2ui"
        mpfr_div_2ui :: Ptr MPFR_T -> Ptr MPFR_T -> CULong -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_div_2si"
        mpfr_div_2si :: Ptr MPFR_T -> Ptr MPFR_T -> CLong -> CRoundMode -> IO CInt



--------------------------------------------------------------------------------
-- comparison functions
-- !!!!!!!! these set erange flags
foreign import ccall unsafe "mpfr_cmp_wrap"
        mpfr_cmp :: Ptr MPFR_T -> Ptr MPFR_T -> IO CInt

foreign import ccall unsafe "mpfr_cmp_ui_wrap"
        mpfr_cmp_ui :: Ptr MPFR_T -> CULong -> IO CInt

foreign import ccall unsafe "mpfr_cmp_si_wrap"
        mpfr_cmp_si :: Ptr MPFR_T -> CLong -> IO CInt

foreign import ccall unsafe "mpfr_cmp_d"
        mpfr_cmp_d :: Ptr MPFR_T -> CDouble -> IO CInt

--TODO cmp_ld, cmp_z, cmp_q, cmp_f

foreign import ccall unsafe "mpfr_cmp_ui_2exp"
        mpfr_cmp_ui_2exp :: Ptr MPFR_T -> CULong -> CInt -> IO CInt

foreign import ccall unsafe "mpfr_cmp_si_2exp"
        mpfr_cmp_si_2exp :: Ptr MPFR_T -> CLong -> CInt -> IO CInt

foreign import ccall unsafe "mpfr_cmpabs"
        mpfr_cmpabs :: Ptr MPFR_T -> Ptr MPFR_T -> IO CInt

foreign import ccall unsafe "mpfr_nan_p_wrap"
        mpfr_nan_p :: Ptr MPFR_T -> IO CInt

foreign import ccall unsafe "mpfr_inf_p_wrap"
        mpfr_inf_p :: Ptr MPFR_T -> IO CInt

foreign import ccall unsafe "mpfr_number_p"
        mpfr_number_p :: Ptr MPFR_T -> IO CInt

foreign import ccall unsafe "mpfr_zero_p_wrap"
        mpfr_zero_p :: Ptr MPFR_T -> IO CInt

foreign import ccall unsafe "mpfr_sgn_wrap"
        mpfr_sgn :: Ptr MPFR_T -> IO CInt 

foreign import ccall unsafe "mpfr_greater_p"
        mpfr_greater_p :: Ptr MPFR_T ->  Ptr MPFR_T -> IO CInt

foreign import ccall unsafe "mpfr_greaterequal_p"
        mpfr_greaterequal_p :: Ptr MPFR_T -> Ptr MPFR_T -> IO CInt 

foreign import ccall unsafe "mpfr_less_p"
        mpfr_less_p :: Ptr MPFR_T -> Ptr MPFR_T -> IO CInt 

foreign import ccall unsafe "mpfr_lessequal_p"
        mpfr_lessequal_p :: Ptr MPFR_T -> Ptr MPFR_T -> IO CInt 

foreign import ccall unsafe "mpfr_lessgreater_p"
        mpfr_lessgreater_p :: Ptr MPFR_T -> Ptr MPFR_T -> IO CInt 

foreign import ccall unsafe "mpfr_equal_p"
        mpfr_equal_p :: Ptr MPFR_T -> Ptr MPFR_T -> IO CInt 

foreign import ccall unsafe "mpfr_unordered_p"
        mpfr_unordered_p :: Ptr MPFR_T -> Ptr MPFR_T -> IO CInt 

-- special functions 

foreign import ccall unsafe "mpfr_log"
        mpfr_log :: Ptr MPFR_T -> Ptr MPFR_T ->  CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_log2"
        mpfr_log2 :: Ptr MPFR_T -> Ptr MPFR_T ->  CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_log10"
        mpfr_log10 :: Ptr MPFR_T -> Ptr MPFR_T ->  CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_exp"
        mpfr_exp :: Ptr MPFR_T -> Ptr MPFR_T ->  CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_exp2"
        mpfr_exp2 :: Ptr MPFR_T -> Ptr MPFR_T ->  CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_exp10"
        mpfr_exp10 :: Ptr MPFR_T -> Ptr MPFR_T ->  CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_sin"
        mpfr_sin :: Ptr MPFR_T -> Ptr MPFR_T ->  CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_cos"
        mpfr_cos :: Ptr MPFR_T -> Ptr MPFR_T ->  CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_tan"
        mpfr_tan :: Ptr MPFR_T -> Ptr MPFR_T ->  CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_sec"
        mpfr_sec :: Ptr MPFR_T -> Ptr MPFR_T ->  CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_csc"
        mpfr_csc :: Ptr MPFR_T -> Ptr MPFR_T ->  CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_cot"
        mpfr_cot :: Ptr MPFR_T -> Ptr MPFR_T ->  CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_sin_cos"
        mpfr_sin_cos :: Ptr MPFR_T -> Ptr MPFR_T -> Ptr MPFR_T -> CRoundMode -> IO CInt


foreign import ccall unsafe "mpfr_asin"
        mpfr_asin :: Ptr MPFR_T -> Ptr MPFR_T ->  CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_acos"
        mpfr_acos :: Ptr MPFR_T -> Ptr MPFR_T ->  CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_atan"
        mpfr_atan :: Ptr MPFR_T -> Ptr MPFR_T ->  CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_atan2"
        mpfr_atan2 :: Ptr MPFR_T -> Ptr MPFR_T -> Ptr MPFR_T -> CRoundMode -> IO CInt


foreign import ccall unsafe "mpfr_cosh"
        mpfr_cosh :: Ptr MPFR_T -> Ptr MPFR_T ->  CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_sinh"
        mpfr_sinh :: Ptr MPFR_T -> Ptr MPFR_T ->  CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_tanh"
        mpfr_tanh :: Ptr MPFR_T -> Ptr MPFR_T ->  CRoundMode -> IO CInt


foreign import ccall unsafe "mpfr_sech"
        mpfr_sech :: Ptr MPFR_T -> Ptr MPFR_T ->  CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_csch"
        mpfr_csch :: Ptr MPFR_T -> Ptr MPFR_T ->  CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_coth"
        mpfr_coth :: Ptr MPFR_T -> Ptr MPFR_T ->  CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_asinh"
        mpfr_asinh :: Ptr MPFR_T -> Ptr MPFR_T ->  CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_acosh"
        mpfr_acosh :: Ptr MPFR_T -> Ptr MPFR_T ->  CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_atanh"
        mpfr_atanh :: Ptr MPFR_T -> Ptr MPFR_T ->  CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_fac_ui"
        mpfr_fac_ui :: Ptr MPFR_T -> CULong -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_log1p"
        mpfr_log1p :: Ptr MPFR_T -> Ptr MPFR_T ->  CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_expm1"
        mpfr_expm1 :: Ptr MPFR_T -> Ptr MPFR_T ->  CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_eint"
        mpfr_eint :: Ptr MPFR_T -> Ptr MPFR_T ->  CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_gamma"
        mpfr_gamma :: Ptr MPFR_T -> Ptr MPFR_T ->  CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_lngamma"
        mpfr_lngamma :: Ptr MPFR_T -> Ptr MPFR_T ->  CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_lgamma"
        mpfr_lgamma :: Ptr MPFR_T -> Ptr CInt -> Ptr MPFR_T ->  CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_zeta"
        mpfr_zeta :: Ptr MPFR_T -> Ptr MPFR_T ->  CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_zeta_ui"
        mpfr_zeta_ui :: Ptr MPFR_T -> CULong ->  CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_erf"
        mpfr_erf :: Ptr MPFR_T -> Ptr MPFR_T ->  CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_erfc"
        mpfr_erfc :: Ptr MPFR_T -> Ptr MPFR_T ->  CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_j0"
        mpfr_j0 :: Ptr MPFR_T -> Ptr MPFR_T ->  CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_j1"
        mpfr_j1 :: Ptr MPFR_T -> Ptr MPFR_T ->  CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_jn"
        mpfr_jn :: Ptr MPFR_T -> CLong -> Ptr MPFR_T -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_y0"
        mpfr_y0 :: Ptr MPFR_T -> Ptr MPFR_T ->  CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_y1"
        mpfr_y1 :: Ptr MPFR_T -> Ptr MPFR_T ->  CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_yn"
        mpfr_yn :: Ptr MPFR_T -> CLong -> Ptr MPFR_T -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_fma"
        mpfr_fma :: Ptr MPFR_T -> Ptr MPFR_T -> Ptr MPFR_T -> Ptr MPFR_T -> CRoundMode -> IO CInt  

foreign import ccall unsafe "mpfr_fms"
        mpfr_fms :: Ptr MPFR_T -> Ptr MPFR_T -> Ptr MPFR_T -> Ptr MPFR_T -> CRoundMode -> IO CInt


foreign import ccall unsafe "mpfr_agm"
        mpfr_agm :: Ptr MPFR_T -> Ptr MPFR_T -> Ptr MPFR_T -> CRoundMode -> IO CInt  

foreign import ccall unsafe "mpfr_hypot"
        mpfr_hypot :: Ptr MPFR_T -> Ptr MPFR_T -> Ptr MPFR_T -> CRoundMode -> IO CInt  

-- constants
foreign import ccall unsafe "mpfr_const_log2"
        mpfr_const_log2 :: Ptr MPFR_T -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_const_pi"
        mpfr_const_pi :: Ptr MPFR_T -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_const_euler"
        mpfr_const_euler :: Ptr MPFR_T -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_const_catalan"
        mpfr_const_catalan :: Ptr MPFR_T -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_free_cache"
        mpfr_free_cache :: IO ()

foreign import ccall unsafe "mpfr_sum"
        mpfr_sum :: Ptr MPFR_T -> Ptr (Ptr MPFR_T) -> CULong -> CRoundMode -> IO CInt

-- TODO input and output functions

-- integer related functions

foreign import ccall unsafe "mpfr_rint"
        mpfr_rint :: Ptr MPFR_T -> Ptr MPFR_T -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_ceil_wrap"
        mpfr_ceil :: Ptr MPFR_T -> Ptr MPFR_T  -> IO CInt

foreign import ccall unsafe "mpfr_floor_wrap"
        mpfr_floor :: Ptr MPFR_T -> Ptr MPFR_T -> IO CInt

foreign import ccall unsafe "mpfr_round_wrap"
        mpfr_round :: Ptr MPFR_T -> Ptr MPFR_T -> IO CInt

foreign import ccall unsafe "mpfr_trunc_wrap"
        mpfr_trunc :: Ptr MPFR_T -> Ptr MPFR_T -> IO CInt
 
foreign import ccall unsafe "mpfr_rint_ceil"
        mpfr_rint_ceil :: Ptr MPFR_T -> Ptr MPFR_T -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_rint_floor"
        mpfr_rint_floor :: Ptr MPFR_T -> Ptr MPFR_T -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_rint_round"
        mpfr_rint_round :: Ptr MPFR_T -> Ptr MPFR_T -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_rint_trunc"
        mpfr_rint_trunc :: Ptr MPFR_T -> Ptr MPFR_T -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_frac"
        mpfr_frac :: Ptr MPFR_T -> Ptr MPFR_T -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_remainder" 
        mpfr_remainder :: Ptr MPFR_T -> Ptr MPFR_T -> Ptr MPFR_T -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_remquo" 
        mpfr_remquo :: Ptr MPFR_T -> Ptr CLong -> Ptr MPFR_T -> Ptr MPFR_T -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_integer_p"
        mpfr_integer_p :: Ptr MPFR_T -> IO CInt

--------------------
-- miscellaneus functions

foreign import ccall unsafe "mpfr_nexttoward"
        mpfr_nexttoward ::  Ptr MPFR_T -> Ptr MPFR_T -> IO ()

foreign import ccall unsafe "mpfr_nextabove"
        mpfr_nextabove ::  Ptr MPFR_T -> IO ()

foreign import ccall unsafe "mpfr_nextbelow"
        mpfr_nextbelow ::  Ptr MPFR_T -> IO ()

foreign import ccall unsafe "mpfr_min"
        mpfr_min :: Ptr MPFR_T -> Ptr MPFR_T -> Ptr MPFR_T -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_max"
        mpfr_max :: Ptr MPFR_T -> Ptr MPFR_T -> Ptr MPFR_T -> CRoundMode -> IO CInt

-- TODO urandomb

foreign import ccall unsafe "mpfr_random2"
        mpfr_random2 :: Ptr MPFR_T -> MpSize -> Exp -> IO ()


foreign import ccall unsafe "mpfr_get_exp_wrap"
        mpfr_get_exp :: Ptr MPFR_T -> IO Exp

foreign import ccall unsafe "mpfr_set_exp"
        mpfr_set_exp :: Ptr MPFR_T -> Exp -> IO CInt

foreign import ccall unsafe "mpfr_signbit_wrap"
        mpfr_signbit :: Ptr MPFR_T -> IO CInt

foreign import ccall unsafe "mpfr_setsign_wrap"
        mpfr_setsign :: Ptr MPFR_T -> Ptr MPFR_T -> CInt -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_copysign_wrap"
        mpfr_copysign :: Ptr MPFR_T -> Ptr MPFR_T -> Ptr MPFR_T -> CRoundMode -> IO CInt 

---------------------------------------------------------------
-- rounding mode related functions

foreign import ccall unsafe "mpfr_get_emin"
        mpfr_get_emin :: IO Exp

foreign import ccall unsafe "mpfr_get_emax"
        mpfr_get_emax :: IO Exp

foreign import ccall unsafe "mpfr_set_emin"
        mpfr_set_emin :: Exp -> IO CInt

foreign import ccall unsafe "mpfr_set_emax"
        mpfr_set_emax :: Exp -> IO CInt

foreign import ccall unsafe "mpfr_get_emin_min"
        mpfr_get_emin_min :: IO Exp

foreign import ccall unsafe "mpfr_get_emin_max"
        mpfr_get_emin_max :: IO Exp

foreign import ccall unsafe "mpfr_get_emax_min"
        mpfr_get_emax_min :: IO Exp

foreign import ccall unsafe "mpfr_get_emax_max"
        mpfr_get_emax_max :: IO Exp

foreign import ccall unsafe "mpfr_check_range"
        mpfr_check_range :: Ptr MPFR_T -> CInt -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_subnormalize"
        mpfr_subnormalize :: Ptr MPFR_T -> CInt -> CRoundMode -> IO CInt

foreign import ccall unsafe "mpfr_clear_underflow"
        mpfr_clear_underflow :: IO ()

foreign import ccall unsafe "mpfr_clear_overflow"
        mpfr_clear_overflow :: IO ()

foreign import ccall unsafe "mpfr_clear_nanflag"
        mpfr_clear_nanflag :: IO ()

foreign import ccall unsafe "mpfr_clear_inexflag"
        mpfr_clear_inexflag :: IO ()

foreign import ccall unsafe "mpfr_clear_erangeflag"
        mpfr_clear_erangeflag :: IO ()

foreign import ccall unsafe "mpfr_set_underflow"
        mpfr_set_underflow :: IO ()

foreign import ccall unsafe "mpfr_set_overflow"
        mpfr_set_overflow :: IO ()

foreign import ccall unsafe "mpfr_set_nanflag"
        mpfr_set_nanflag :: IO ()

foreign import ccall unsafe "mpfr_set_inexflag"
        mpfr_set_inexflag :: IO ()

foreign import ccall unsafe "mpfr_set_erangeflag"
        mpfr_set_erangeflag :: IO ()

foreign import ccall unsafe "mpfr_clear_flags"
        mpfr_clear_flags :: IO ()


foreign import ccall unsafe "mpfr_underflow_p"
        mpfr_underflow_p :: IO CInt

foreign import ccall unsafe "mpfr_overflow_p"
        mpfr_overflow_p :: IO CInt

foreign import ccall unsafe "mpfr_nanflag_p"
        mpfr_nanflag_p :: IO CInt

foreign import ccall unsafe "mpfr_inexflag_p"
        mpfr_inexflag_p :: IO CInt

foreign import ccall unsafe "mpfr_erangeflag_p"
        mpfr_erangeflag_p :: IO CInt

---------------------------------------------------------------
-- custom interface
foreign import ccall unsafe "mpfr_custom_get_size_wrap" 
        mpfr_custom_get_size :: CPrecision -> IO #{type size_t}

foreign import ccall unsafe "mpfr_custom_init_wrap"
        mpfr_custom_init :: Ptr #{type mp_limb_t} -> CPrecision -> IO ()

foreign import ccall unsafe "mpfr_custom_init_set_wrap"
        mpfr_custom_init_set :: Ptr MPFR_T -> CInt -> Exp -> CPrecision -> Ptr Limb -> IO ()

foreign import ccall unsafe "mpfr_custom_get_kind_wrap"
        mpfr_custom_get_kind :: Ptr MPFR_T -> IO CInt

foreign import ccall unsafe "mpfr_custom_get_mantissa_wrap"
        mpfr_custom_get_mantissa :: Ptr MPFR_T -> IO (Ptr Limb)

foreign import ccall unsafe "mpfr_custom_get_exp_wrap"
        mpfr_custom_get_exp :: Ptr MPFR_T -> IO CInt

foreign import ccall unsafe "mpfr_custom_move_wrap"
        mpfr_custom_move :: Ptr MPFR_T -> Ptr #{type mp_limb_t} -> IO ()

-------------------------------------------------

