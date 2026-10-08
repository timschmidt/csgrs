{-# LANGUAGE StandaloneDeriving #-}
{-# LANGUAGE
    EmptyDataDecls,
    GeneralizedNewtypeDeriving,
    ScopedTypeVariables,
    Rank2Types #-}

-- | Numbers with a fixed number of decimals.
module Data.Number.Fixed(
    Fixed,
    Epsilon(eps), Eps1, EpsDiv10, Prec10, Prec50, PrecPlus20,
    Prec500, convertFixed, dynamicEps, precision, with_added_precision) where
import Numeric
import Data.Char
import Data.Ratio
import qualified Data.Number.FixedFunctions as F

-- | The 'Epsilon' class contains the types that can be used to determine the
-- precision of a 'Fixed' number.
class Epsilon e where
    eps :: e -> Rational

-- | An epsilon of 1, i.e., no decimals.
data Eps1
instance Epsilon Eps1 where
    eps _ = 1

-- | A type construct that gives one more decimals than the argument.
data EpsDiv10 p
instance (Epsilon e) => Epsilon (EpsDiv10 e) where
    eps e = eps (un e) / 10
       where un :: EpsDiv10 e -> e
             un = undefined

-- | Ten decimals.
data Prec10
instance Epsilon Prec10 where
    eps _ = 1e-10

-- | 50 decimals.
data Prec50
instance Epsilon Prec50 where
    eps _ = 1e-50

-- | 500 decimals.
data Prec500
instance Epsilon Prec500 where
    eps _ = 1e-500

-- A type that gives 20 more decimals than the argument.
data PrecPlus20 e
instance (Epsilon e) => Epsilon (PrecPlus20 e) where
    eps e = 1e-20 * eps (un e)
       where un :: PrecPlus20 e -> e
             un = undefined

-----------

-- The type of fixed precision numbers.  The type /e/ determines the precision.
newtype Fixed e = F Rational deriving (Eq, Ord, Enum)
deriving instance Epsilon e => Real (Fixed e)
deriving instance Epsilon e => RealFrac (Fixed e)

-- Get the accuracy (the epsilon) of the type.
precision :: (Epsilon e) => Fixed e -> Rational
precision = getEps

instance (Epsilon e) => Num (Fixed e) where
    (+) = lift2 (+)
    (-) = lift2 (-)
    (*) = lift2 (*)
    negate (F x) = F (negate x)
    abs (F x) = F (abs x)
    signum (F x) = F (signum x)
    fromInteger = F . fromInteger

instance (Epsilon e) => Fractional (Fixed e) where
    (/) = lift2 (/)
    fromRational x = r
        where r = F $ approx x (getEps r)

lift2 :: (Epsilon e) => (Rational -> Rational -> Rational) -> Fixed e -> Fixed e -> Fixed e
lift2 op fx@(F x) (F y) = F $ approx (x `op` y) (getEps fx)

approx :: Rational -> Rational -> Rational
approx x eps = approxRational x (eps/2)

-- | Convert between two arbitrary fixed precision types.
convertFixed :: (Epsilon e, Epsilon f) => Fixed e -> Fixed f
convertFixed e@(F x) = f
  where f = F $ if feps > eeps then approx x feps else x
        feps = getEps f
        eeps = getEps e

getEps :: (Epsilon e) => Fixed e -> Rational
getEps = eps . un
  where un :: Fixed e -> e
        un = undefined

instance (Epsilon e) => Show (Fixed e) where
    showsPrec = showSigned showFixed
      where showFixed f@(F x) = showString $ show q ++ "." ++ decimals r e
              where q :: Integer
                    (q, r) = properFraction (x + e/2)
                    e = getEps f
            decimals a e | e >= 1 = ""
                         | otherwise = intToDigit b : decimals c (10 * e)
                              where (b, c) = properFraction (10 * a)

instance (Epsilon e) => Read (Fixed e) where
    readsPrec _ = readSigned readFixed
      where readFixed s = [ (toFixed0 (approxRational x), s') | (x, s') <- readFloat s ]

instance (Epsilon e) => Floating (Fixed e) where
    pi = toFixed0 F.pi
    sqrt = toFixed1 F.sqrt
    exp x = with_added_precision r (convertFixed . (toFixed1 F.exp)) x where
      r = if x < 0 then 1 else 0.1 ^ (ceiling (x * 0.45))
    log = toFixed1 F.log
    sin = toFixed1 F.sin
    cos = toFixed1 F.cos
    tan = toFixed1 F.tan
    asin = toFixed1 F.asin
    acos = toFixed1 F.acos
    atan = toFixed1 F.atan
    sinh = toFixed1 F.sinh
    cosh = toFixed1 F.cosh
    tanh = toFixed1 F.tanh
    asinh = toFixed1 F.asinh
    acosh = toFixed1 F.acosh
    atanh = toFixed1 F.atanh

toFixed0 :: (Epsilon e) => (Rational -> Rational) -> Fixed e
toFixed0 f = r
    where r = F $ f $ getEps r

toFixed1 :: (Epsilon e) => (Rational -> Rational -> Rational) -> Fixed e -> Fixed e
toFixed1 f x@(F r) = F $ f (getEps x) r

instance (Epsilon e) => RealFloat (Fixed e) where
    exponent _ = 0
    scaleFloat 0 x = x
    isNaN _ = False
    isInfinite _ = False
    isDenormalized _ = False
    isNegativeZero _ = False
    isIEEE _ = False
    -- Explicitly undefine these rather than omitting them; this
    -- prevents a compiler warning at least.
    floatRadix = undefined
    floatDigits = undefined
    floatRange = undefined
    decodeFloat = undefined
    encodeFloat = undefined

-----------

-- | The call @dynmicEps r f v@ evaluates @f v@ to a precsion of @r@.
dynamicEps :: forall a . Rational -> (forall e . Epsilon e => Fixed e -> a) -> Rational -> a
dynamicEps r f v = loop (undefined :: Eps1)
  where loop :: forall x . (Epsilon x) => x -> a
        loop e = if eps e <= r then f (fromRational v :: Fixed x) else loop (undefined :: EpsDiv10 x)

-- | The call @with_added_precision r f v@ evaluates @f v@, while
-- temporarily multiplying the precision of /v/ by /r/.
with_added_precision :: forall a f.(Epsilon f) => Rational -> (forall e.(Epsilon e) => Fixed e -> a) -> Fixed f -> a
with_added_precision r f v = dynamicEps (p*r) f (toRational v) where
  p = precision v
