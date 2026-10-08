{-|
Defines the 'Scalable' type class.
-}
module Data.CDAR.Classes where

import Data.Bits

-- | 'Scalable' allows scaling numerical data types by powers of 2.
class Scalable a where
  scale :: a -> Int -> a

-- | The 'Integer' instance.
instance Scalable Integer where
  scale x n
    | n >= 0 = unsafeShiftL x n
    | otherwise = unsafeShiftR (x + bit (-n-1)) (-n)
