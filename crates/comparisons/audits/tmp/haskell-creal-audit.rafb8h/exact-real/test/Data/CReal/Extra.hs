{-# OPTIONS_GHC -fno-warn-orphans #-}

module Data.CReal.Extra
  ( module Data.CReal
  ) where

import Data.CReal
import GHC.TypeLits
import Test.QuickCheck (Arbitrary(..), chooseAny)
import Test.QuickCheck.Checkers (EqProp(..), eq)

instance KnownNat n => EqProp (CReal n) where
  (=-=) = eq

instance KnownNat n => Arbitrary (CReal n) where
  arbitrary = do
    integralPart <- fromInteger <$> arbitrary
    fractionalPart <- (subtract 0.5) <$> chooseAny
    pure (integralPart + fractionalPart)

