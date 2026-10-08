{-| This module contains specification of approximate field.
-}

module Data.Approximate.ApproximateField (
       RoundingMode (..),
       Stage (..),
       anti,
       precDown,
       precUp,
       prec,
       ApproximateField (..),
       DyadicField (..)
) where

-- | The rounding mode tells us whether we should under- or over-approximate the exact result.
data RoundingMode = RoundUp | RoundDown
                  deriving (Eq, Show)

-- | A stage of computation tells us how hard we should try to compute the result. The 'stage' component
-- is a measure of precisions. As it goes to infinity, the approximation should converge to the exact
-- value (in the sense of Scott topology on the underlying domain model).
data Stage = Stage { precision :: Int, rounding :: RoundingMode }
             deriving Show

-- | 'anti' reverses the rounding mode
anti :: Stage -> Stage
anti s = Stage {precision = precision s, rounding = case rounding s of { RoundUp -> RoundDown ; RoundDown -> RoundUp}}

-- | @precDown k@ sets precision to @k@ and the rounding mode to 'RoundDown'
precDown :: Int -> Stage
precDown k = Stage {precision = k, rounding = RoundDown}

-- | @precUp k@ sets precision to @k@ and the rounding mode to 'RoundUp'
precUp :: Int -> Stage
precUp k = Stage {precision = k, rounding = RoundUp}

-- | @prec r k@ is the stage with given rounding @r@ and precision @k@
prec :: RoundingMode -> Int -> Stage
prec r k = Stage {precision = k, rounding = r}


{- | An approximate field is a structure in which we can perform approximate
arithmetical operations. The typical example is the ring of dyadic rational
numbers: division of dyadic rationals is only approximate, and even though the
other operations (+, -, *) can be peformed exactly, it is too expensive and
unecessary to do so in an interval computation. Therefore, we want approximate
versions of all operations.

The approximate operations take a 'Stage' argument which tells whether the
result of the operation should be rounded up or down, in the sense of the
linear ordering of the structure, and how precise the result should be.

(Missing explanation of what exactly an approximate field is supposed to be.)
-}
class (Show q, Ord q) => ApproximateField q where
  zero :: q
  zero = appFromInteger 0

  -- conversions from
  appFromInteger :: Integer -> q
  appFromRational_ :: Stage -> Rational -> (q, Bool)
  -- ^ returns also whether this rational has an exact representation
  appFromRational :: Stage -> Rational -> q
  appFromRational s q = fst $ appFromRational_ s q

  -- approximate operations
  appAdd :: Stage -> q -> q -> q
  appSub :: Stage -> q -> q -> q
  appMul :: Stage -> q -> q -> q
  appInv :: Stage -> q -> q
  appDiv :: Stage -> q -> q -> q
  appNeg :: Stage -> q -> q
  appAbs :: Stage -> q -> q


class ApproximateField q => DyadicField q where
  -- special values
  posInf :: q
  negInf :: q
  naN :: q

  isUnordered :: q -> q -> Bool

  -- accessing internal structure of the numbers
  appGetExp :: q -> Int -- ^ returns a number @k@ such that @2^k <= abs q <= 2^(k+1)@.
  appPrec :: q -> Int -- ^ the size of the number (memory usage)

  -- additional operations
  --  toFloat :: q -> Double
  appMul2 :: Stage -> q -> Int -> q  -- ^ shift by a power of 2
  appMul2 s q i = appMul s q (appFromInteger (toInteger i))

  {-| Exact midpoint -}
  midpoint :: q -> q -> q

  midpoint a b
    | isUnordered a b            = naN
    | cmp == EQ                  = a
    | c == negInf && d < zero    = appMul2 p d 1
    | c == negInf && d == zero   = appFromInteger (-1)
    | c > zero && d == posInf    = appMul2 p c 1
    | c == zero && d == posInf   = appFromInteger 1
    | c == negInf || d == posInf = zero  -- careful: order of statements should not change
    | otherwise                  = appMul2 p (appAdd p a b) (-1)
    where
      cmp    = compare a b
      (c, d) = if cmp == LT then (a, b) else (b, a)
      p = precDown $ 1 + max (appPrec a) (appPrec b)
