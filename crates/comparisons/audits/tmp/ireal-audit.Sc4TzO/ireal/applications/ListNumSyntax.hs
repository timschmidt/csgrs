module ListNumSyntax where

instance Num a => Num [a] where 
  (+) = zipWith (+)
  (-) = zipWith (-)
  (*) = zipWith (*)
  negate = map negate
  abs = map abs
  signum = map signum
  fromInteger = repeat . fromInteger


instance Fractional a => Fractional [a] where
  recip = map recip
  (/) = zipWith (/)
  fromRational = repeat . fromRational
