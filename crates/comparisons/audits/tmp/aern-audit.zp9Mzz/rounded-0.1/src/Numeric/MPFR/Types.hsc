#include <ghc-gmp.h>
#include <mpfr.h>

#if __GLASGOW_HASKELL__ < 800
#let alignment t = "%lu", (unsigned long)offsetof(struct {char x__; t (y__); }, y__)
#endif

{-# LANGUAGE DeriveDataTypeable #-}
{-# LANGUAGE GeneralizedNewtypeDeriving #-}
{-# LANGUAGE PatternSynonyms #-}
-----------------------------------------------------------------------------
-- |
-- Module      :  Numeric.MPFR.Types
-- Copyright   :  (C) 2015-2018 Claude Heiland-Allen
-- License     :  LGPL
-- Stability   :  experimental
-- Portability :  non-portable
--
-- MPFR types.
----------------------------------------------------------------------------
module Numeric.MPFR.Types where

import Data.Data
import Data.Typeable
import Data.Bits
import Data.Ix
import Data.Int
import Data.Word

import Foreign (Storable(..), Ptr)

import Numeric.GMP.Types (MPLimb(..))

-- | Version numbers.
pattern MPFR_VERSION_MAJOR = #const MPFR_VERSION_MAJOR
pattern MPFR_VERSION_MINOR = #const MPFR_VERSION_MINOR
pattern MPFR_VERSION_PATCHLEVEL = #const MPFR_VERSION_PATCHLEVEL
pattern MPFR_VERSION = #const MPFR_VERSION

-- | @mpfr_t@
data MPFR = MPFR
  { mpfrPrec :: !MPFRPrec
  , mpfrSign :: !MPFRSign
  , mpfrExp :: !MPFRExp
  , mpfrD :: !(Ptr MPLimb)
  }

instance Storable MPFR where
  sizeOf _ = (#size __mpfr_struct)
  alignment _ = (#alignment __mpfr_struct)
  peek ptr = do
    prec <- (#peek __mpfr_struct, _mpfr_prec) ptr
    sign <- (#peek __mpfr_struct, _mpfr_sign) ptr
    exp' <- (#peek __mpfr_struct, _mpfr_exp) ptr
    d <- (#peek __mpfr_struct, _mpfr_d) ptr
    return (MPFR{ mpfrPrec = prec, mpfrSign = sign, mpfrExp = exp', mpfrD = d })
  poke ptr (MPFR{ mpfrPrec = prec, mpfrSign = sign, mpfrExp = exp', mpfrD = d }) = do
    (#poke __mpfr_struct, _mpfr_prec) ptr prec
    (#poke __mpfr_struct, _mpfr_sign) ptr sign
    (#poke __mpfr_struct, _mpfr_exp) ptr exp'
    (#poke __mpfr_struct, _mpfr_d) ptr d

-- | @mpfr_int@
newtype MPFRInt = MPFRInt (#type mpfr_int)
  deriving (Eq, Ord, Read, Show, Enum, Bounded, Num, Integral, Real, Ix, Bits, FiniteBits, Data, Typeable, Storable)
-- | @mpfr_uint@
newtype MPFRUInt = MPFRUInt (#type mpfr_uint)
  deriving (Eq, Ord, Read, Show, Enum, Bounded, Num, Integral, Real, Ix, Bits, FiniteBits, Data, Typeable, Storable)

-- | @mpfr_long@
newtype MPFRLong = MPFRLong (#type mpfr_long)
  deriving (Eq, Ord, Read, Show, Enum, Bounded, Num, Integral, Real, Ix, Bits, FiniteBits, Data, Typeable, Storable)
-- | @mpfr_ulong@
newtype MPFRULong = MPFRULong (#type mpfr_ulong)
  deriving (Eq, Ord, Read, Show, Enum, Bounded, Num, Integral, Real, Ix, Bits, FiniteBits, Data, Typeable, Storable)

-- | @mpfr_size_t@
newtype MPFRSize = MPFRSize (#type mpfr_size_t)
  deriving (Eq, Ord, Read, Show, Enum, Bounded, Num, Integral, Real, Ix, Bits, FiniteBits, Data, Typeable, Storable)

-- | @mpfr_prec_t@
newtype MPFRPrec = MPFRPrec (#type mpfr_prec_t)
  deriving (Eq, Ord, Read, Show, Enum, Bounded, Num, Integral, Real, Ix, Bits, FiniteBits, Data, Typeable, Storable)
-- | @mpfr_uprec_t@
newtype MPFRUPrec = MPFRUPrec (#type mpfr_uprec_t)
  deriving (Eq, Ord, Read, Show, Enum, Bounded, Num, Integral, Real, Ix, Bits, FiniteBits, Data, Typeable, Storable)
pattern MPFR_PREC_MIN = (#const MPFR_PREC_MIN)
pattern MPFR_PREC_MAX = (#const MPFR_PREC_MAX)

-- | @mpfr_sign_t@
newtype MPFRSign = MPFRSign (#type mpfr_sign_t)
  deriving (Eq, Ord, Read, Show, Enum, Bounded, Num, Integral, Real, Ix, Bits, FiniteBits, Data, Typeable, Storable)

-- | @mpfr_exp_t@
newtype MPFRExp = MPFRExp (#type mpfr_exp_t)
  deriving (Eq, Ord, Read, Show, Enum, Bounded, Num, Integral, Real, Ix, Bits, FiniteBits, Data, Typeable, Storable)
-- | @mpfr_uexp_t@
newtype MPFRUExp = MPFRUExp (#type mpfr_uexp_t)
  deriving (Eq, Ord, Read, Show, Enum, Bounded, Num, Integral, Real, Ix, Bits, FiniteBits, Data, Typeable, Storable)
pattern MPFR_EMAX_DEFAULT = (#const MPFR_EMAX_DEFAULT)
pattern MPFR_EMIN_DEFAULT = (#const MPFR_EMIN_DEFAULT)

-- | @mpfr_rnd_t@
newtype MPFRRnd = MPFRRnd (#type mpfr_rnd_t)
  deriving (Eq, Ord, Read, Show, Enum, Bounded, Num, Integral, Real, Ix, Bits, FiniteBits, Data, Typeable, Storable)
-- | round to nearest, with ties to even
pattern MPFR_RNDN = (#const MPFR_RNDN)
-- | round toward zero
pattern MPFR_RNDZ = (#const MPFR_RNDZ)
-- | round toward +Inf
pattern MPFR_RNDU = (#const MPFR_RNDU)
-- | round toward -Inf
pattern MPFR_RNDD = (#const MPFR_RNDD)
-- | round away from zero
pattern MPFR_RNDA = (#const MPFR_RNDA)
-- | faithful rounding (not implemented yet)
pattern MPFR_RNDF = (#const MPFR_RNDF)
-- | round to nearest, with ties away from zero (mpfr_round) (do not use)
pattern MPFR_RNDNA = (#const MPFR_RNDNA)

-- | @mpfr_kind_t@
newtype MPFRKind = MPFRKind (#type mpfr_kind_t)
  deriving (Eq, Ord, Read, Show, Enum, Bounded, Num, Integral, Real, Ix, Bits, FiniteBits, Data, Typeable, Storable)
pattern MPFR_NAN_KIND = (#const MPFR_NAN_KIND)
pattern MPFR_INF_KIND = (#const MPFR_INF_KIND)
pattern MPFR_ZERO_KIND = (#const MPFR_ZERO_KIND)
pattern MPFR_REGULAR_KIND = (#const MPFR_REGULAR_KIND)
