{-# LANGUAGE EmptyDataDecls #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE MagicHash #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE Rank2Types #-}
{-# LANGUAGE UndecidableInstances #-}
{-# LANGUAGE DataKinds #-}
{-# LANGUAGE PolyKinds #-}
{-# LANGUAGE KindSignatures #-}
{-# LANGUAGE ScopedTypeVariables #-}
{-# LANGUAGE FlexibleInstances #-}
{-# OPTIONS_HADDOCK not-home #-}
-----------------------------------------------------------------------------
-- |
-- Module      :  Numeric.Rounded.Precision
-- Copyright   :  (C) 2012 Edward Kmett
-- License     :  LGPL
-- Maintainer  :  Claude Heiland-Allen <claude@mathr.co.uk>
-- Stability   :  experimental
-- Portability :  non-portable
--
----------------------------------------------------------------------------
module Numeric.Rounded.Precision
  ( Precision(..)
  , reifyPrecision
  , Bytes
  ) where

import Data.Proxy
import Data.Reflection
import Foreign.C.Types
import GHC.TypeLits

import Numeric.LongDouble (LongDouble)
import Numeric.MPFR.Types

-- | This class is used to specify the number of bits of precision that are maintained in the
-- significand of a properly 'Numeric.Rounded.Rounded' floating point number.
class Precision p where
  precision :: proxy p -> Int

floatPrecision :: RealFloat a => p a -> Int
floatPrecision p = fromIntegral (floatDigits (proxyArg p)) where
  proxyArg :: p a -> a
  proxyArg _ = undefined
{-# INLINE floatPrecision #-}

instance Precision Float where
  precision = floatPrecision

instance Precision CFloat where
  precision = floatPrecision

instance Precision Double where
  precision = floatPrecision

instance Precision CDouble where
  precision = floatPrecision

instance Precision LongDouble where
  precision = floatPrecision

instance KnownNat n => Precision (n :: Nat) where
  precision p = max MPFR_PREC_MIN . min MPFR_PREC_MAX $ fromInteger (natVal p)

data Bytes (n :: Nat)

instance KnownNat n => Precision (Bytes n) where
  precision _ = max MPFR_PREC_MIN . min MPFR_PREC_MAX $ 8 * fromInteger (natVal (undefined :: Bytes n))

data ReifiedPrecision (s :: *)

retagReifiedPrecision :: (Proxy s -> a) -> proxy (ReifiedPrecision s) -> a
retagReifiedPrecision f _ = f Proxy
{-# INLINE retagReifiedPrecision #-}

instance Reifies s Int => Precision (ReifiedPrecision s) where
  precision = retagReifiedPrecision reflect

reifyPrecision :: Int -> (forall (p :: *). Precision p => Proxy p -> a) -> a
reifyPrecision m f = reify m (go f) where
  go :: Reifies p Int => (Proxy (ReifiedPrecision p) -> a) -> proxy p -> a
  go g _ = g Proxy
{-# INLINE reifyPrecision #-}
