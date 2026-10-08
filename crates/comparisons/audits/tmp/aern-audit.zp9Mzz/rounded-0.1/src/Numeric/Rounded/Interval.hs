{-# LANGUAGE DataKinds #-}
{-# LANGUAGE DeriveDataTypeable #-}
{-# LANGUAGE DeriveGeneric #-}
{-# LANGUAGE RankNTypes #-}
{-# LANGUAGE PolyKinds #-}
module Numeric.Rounded.Interval where

import Control.Applicative
import Numeric.Rounded
import Data.Coerce
import Data.Typeable
import GHC.Generics
import Prelude hiding (elem, notElem)

data Interval p
  = I (Rounded TowardNegInf p) (Rounded TowardInf p)
  | Empty
  deriving (Typeable, Generic)


-- TODO: take from mpfr
fmod :: RealFrac a => a -> a -> a
fmod a b = a - q*b where
  q = realToFrac (truncate $ a / b :: Integer)
{-# INLINE fmod #-}


instance Precision p => Num (Interval p) where
  I a b + I a' b' = I (a + a') (b + b')
  _ + _ = Empty
  I a b - I a' b' = I (a - coerce b') (b - coerce a')
  _ - _ = Empty
  negate (I a b) = I (coerce (negate b)) (coerce (negate a))
  negate Empty = Empty
  I a b * I a' b' =
    I (minimum [a * a', a * coerce b', coerce b * a', coerce b * coerce b'])
      (maximum [coerce a * coerce a', coerce a * b', b * coerce a', b * b'])
  _ * _ = Empty
  abs x@(I a b)
    | a >= 0    = x
    | b <= 0    = negate x
    | otherwise = I 0 (max (negate (coerce a)) b)
  abs Empty = Empty
  {-# INLINE abs #-}
  signum = increasing signum
  {-# INLINE signum #-}
  fromInteger = I <$> fromInteger <*> fromInteger

-- | lift a monotone increasing function over a given interval
increasing :: (forall r. Rounding r => Rounded r a -> Rounded r b) -> Interval a -> Interval b
increasing f (I a b) = I (f a) (f b)
increasing _ Empty = Empty

-- -- | lift a monotone decreasing function over a given interval
decreasing :: (forall r. Rounding r => Rounded r a -> Rounded r b) -> Interval a -> Interval b
decreasing f (I a b) = I (coerce (f b)) (coerce (f a))
decreasing _ Empty = Empty
--

(...) :: Rounded TowardNegInf p -> Rounded TowardInf p -> Interval p
a ... b
  | coerce a <= b = I a b
  | otherwise = Empty

infixl 6 +/-

(+/-) :: Rounded r p -> Rounded r' p -> Interval p
a +/- b = (coerce a .-. coerce b) ... (coerce a .+. coerce b)

negInfinity :: Fractional a => a
negInfinity = (-1)/0
{-# INLINE negInfinity #-}

posInfinity :: Fractional a => a
posInfinity = 1/0
{-# INLINE posInfinity #-}

-- | create a non-empty interval or fail
interval :: Rounded TowardNegInf p -> Rounded TowardInf p -> Maybe (Interval p)
interval a b
  | coerce a <= b = Just $ I a b
  | otherwise     = Nothing
{-# INLINE interval #-}

-- | The whole real number line
--
-- >>> whole
-- -Infinity ... Infinity
whole :: Precision p => Interval p
whole = I negInfinity posInfinity
{-# INLINE whole #-}

-- | An empty interval
--
-- >>> empty
-- Empty
empty :: Interval p
empty = Empty
{-# INLINE empty #-}

-- | Check if an interval is empty
--
-- >>> null (1 ... 5)
-- False
--
-- >>> null (1 ... 1)
-- False
--
-- >>> null empty
-- True
null :: Interval p -> Bool
null Empty = True
null _ = False
{-# INLINE null #-}

-- | The infimum (lower bound) of an interval
--
-- >>> inf (1.0 ... 20.0)
-- 1.0
--
-- >>> inf empty
-- *** Exception: empty interval
inf :: Interval p -> Rounded TowardNegInf p
inf (I a _) = a
inf Empty = error "empty interval"
{-# INLINE inf #-}

-- | The supremum (upper bound) of an interval
--
-- >>> sup (1.0 ... 20.0)
-- 20.0
--
-- >>> sup empty
-- *** Exception: empty interval
sup :: Interval p -> Rounded TowardInf p
sup (I _ b) = b
sup Empty = error "empty interval"
{-# INLINE sup #-}

-- | Is the interval a singleton point?
-- N.B. This is fairly fragile and likely will not hold after
-- even a few operations that only involve singletons
--
-- >>> singular (singleton 1)
-- True
--
-- >>> singular (1.0 ... 20.0)
-- False
singular :: Interval p -> Bool
singular Empty = False
singular (I a b) = coerce a == b
{-# INLINE singular #-}

instance Eq (Interval p) where
  (==) = (==!)
  {-# INLINE (==) #-}

instance Precision p => Ord (Interval p) where
  compare Empty Empty = EQ
  compare Empty _ = LT
  compare _ Empty = GT
  compare (I ax bx) (I ay by)
    | coerce bx < ay = LT
    | coerce ax > by = GT
    | coerce bx == ay && coerce ax == by = EQ
    | otherwise = error "ambiguous comparison"
  {-# INLINE compare #-}

  max (I a b) (I a' b') = I (max a a') (max b b')
  max Empty i = i
  max i Empty = i
  {-# INLINE max #-}

  min (I a b) (I a' b') = I (min a a') (min b b')
  min Empty _ = Empty
  min _ Empty = Empty
  {-# INLINE min #-}

-- | 'realToFrac' will use the midpoint
instance Precision p => Real (Interval p) where
  toRational Empty = error "empty interval"
  toRational (I ra rb) = a + (b - a) / 2 where
    a = toRational ra
    b = toRational rb
  {-# INLINE toRational #-}

instance Precision p => Show (Interval p) where
  showsPrec _ Empty = showString "Empty"
  showsPrec n (I a b) =
    showParen (n > 3) $
      showsPrec 3 a .
      showString " ... " .
      showsPrec 3 b

-- | Calculate the width of an interval.
--
-- >>> width (1 ... 20)
-- 19 ... 19
--
-- >>> width (singleton 1)
-- 0 ... 0
--
-- >>> width empty
-- 0 ... 0
width :: Precision p => Interval p -> Rounded TowardInf p
width (I a b) = b - coerce a
width Empty   = 0
{-# INLINE width #-}

-- | Magnitude
--
-- >>> magnitude (1 ... 20)
-- 20
--
-- >>> magnitude (-20 ... 10)
-- 20
--
-- >>> magnitude (singleton 5)
-- 5
--
-- throws 'EmptyInterval' if the interval is empty.
--
-- >>> magnitude empty
-- *** Exception: empty interval
magnitude :: Precision p => Interval p -> Rounded TowardInf p
magnitude = sup . abs
{-# INLINE magnitude #-}

-- | \"mignitude\"
--
-- >>> mignitude (1 ... 20)
-- 1
--
-- >>> mignitude (-20 ... 10)
-- 0
--
-- >>> mignitude (singleton 5)
-- 5
--
-- throws 'EmptyInterval' if the interval is empty.
--
-- >>> mignitude empty
-- *** Exception: empty interval
mignitude :: Precision p => Interval p -> Rounded TowardNegInf p -- TowardZero?
mignitude = inf . abs
{-# INLINE mignitude #-}

-- | Construct a symmetric interval.
--
-- >>> symmetric 3
-- -3 ... 3
symmetric :: Rounded TowardInf p -> Interval p
symmetric b = coerce (negate' b) ... b

-- | Hausdorff distance between intervals.
--
-- >>> distance (1 ... 7) (6 ... 10)
-- 0
--
-- >>> distance (1 ... 7) (15 ... 24)
-- 8
--
-- >>> distance (1 ... 7) (-10 ... -2)
-- 3
--
-- >>> distance Empty (1 ... 1)
-- *** Exception: empty interval
distance :: Precision p => Interval p -> Interval p -> Rounded TowardNegInf p -- TowardZero?
distance i1 i2 = mignitude (i1 - i2)

-- | Inflate an interval by enlarging it at both ends.
--
-- >>> inflate 3 (-1 ... 7)
-- -4 ... 10
--
-- >>> inflate (-2) (0 ... 4)
-- -2 ... 6
--
-- >>> inflate 1 empty
-- Empty
inflate :: Precision p => Rounded TowardInf p -> Interval p -> Interval p
inflate x y = symmetric x + y

{-

-- | Deflate an interval by shrinking it from both ends.
--
-- >>> deflate 3.0 (-4.0 ... 10.0)
-- -1.0 ... 7.0
--
-- >>> deflate 2.0 (-1.0 ... 1.0)
-- Empty
--
-- >>> deflate 1.0 empty
-- Empty
deflate :: => a -> Interval a -> Interval a
deflate _ Empty               = Empty
deflate x (I a b) | a' <= b'  = I a' b'
                  | otherwise = Empty
  where
    a' = a + x
    b' = b - x

-}

-- | For all @x@ in @X@, @y@ in @Y@. @x '<' y@
--
-- >>> (5 ... 10 :: Interval Double) <! (20 ... 30 :: Interval Double)
-- True
--
-- >>> (5 ... 10 :: Interval Double) <! (10 ... 30 :: Interval Double)
-- False
--
-- >>> (20 ... 30 :: Interval Double) <! (5 ... 10 :: Interval Double)
-- False
(<!)  :: Precision p => Interval p -> Interval p -> Bool
I _ bx <! I ay _ = coerce bx < ay
_ <! _ = True
{-# INLINE (<!) #-}

-- | For all @x@ in @X@, @y@ in @Y@. @x '<=' y@
--
-- >>> (5 ... 10 :: Interval Double) <=! (20 ... 30 :: Interval Double)
-- True
--
-- >>> (5 ... 10 :: Interval Double) <=! (10 ... 30 :: Interval Double)
-- True
--
-- >>> (20 ... 30 :: Interval Double) <=! (5 ... 10 :: Interval Double)
-- False
(<=!) :: Precision p => Interval p -> Interval p -> Bool
I _ bx <=! I ay _ = coerce bx <= ay
_ <=! _ = True
{-# INLINE (<=!) #-}

-- | For all @x@ in @X@, @y@ in @Y@. @x '==' y@
--
-- Only singleton intervals or empty intervals can return true
--
-- >>> (singleton 5 :: Interval Double) ==! (singleton 5 :: Interval Double)
-- True
--
-- >>> (5 ... 10 :: Interval Double) ==! (5 ... 10 :: Interval Double)
-- False
(==!) :: Interval p -> Interval p -> Bool
I ax bx ==! I ay by = coerce bx == ay && coerce ax == by
_ ==! _ = True
{-# INLINE (==!) #-}

-- | For all @x@ in @X@, @y@ in @Y@. @x '/=' y@
--
-- >>> (5 ... 15 :: Interval Double) /=! (20 ... 40 :: Interval Double)
-- True
--
-- >>> (5 ... 15 :: Interval Double) /=! (15 ... 40 :: Interval Double)
-- False
(/=!) :: Interval p -> Interval p -> Bool
I ax bx /=! I ay by = bx < coerce ay || coerce ax > by
_ /=! _ = True
{-# INLINE (/=!) #-}

-- | For all @x@ in @X@, @y@ in @Y@. @x '>' y@
--
-- >>> (20 ... 40 :: Interval Double) >! (10 ... 19 :: Interval Double)
-- True
--
-- >>> (5 ... 20 :: Interval Double) >! (15 ... 40 :: Interval Double)
-- False
(>!) :: Precision p => Interval p -> Interval p -> Bool
I ax _ >! I _ by = ax > coerce by
_ >! _ = True
{-# INLINE (>!) #-}

-- | For all @x@ in @X@, @y@ in @Y@. @x '>=' y@
--
-- >>> (20 ... 40 :: Interval Double) >=! (10 ... 20 :: Interval Double)
-- True
--
-- >>> (5 ... 20 :: Interval Double) >=! (15 ... 40 :: Interval Double)
-- False
(>=!) :: Precision p => Interval p -> Interval p -> Bool
I ax _ >=! I _ by = coerce ax >= by
_ >=! _ = True

-- | Determine if a point is in the interval.
--
-- >>> elem 3.2 (1 ... 5)
-- True
--
-- >>> elem 5 (1 ... 5)
-- True
--
-- >>> elem 1 (1 ... 5)
-- True
--
-- >>> elem 8 (1 ... 5)
-- False
--
-- >>> elem 5 empty
-- False
--
elem :: Rounded TowardZero p -> Interval p -> Bool
elem x (I a b) = coerce x >= a && coerce x <= b
elem _ Empty = False
{-# INLINE elem #-}

-- | Determine if a point is not included in the interval
--
-- >>> notElem 8 (1.0 ... 5.0)
-- True
--
-- >>> notElem 1.4 (1.0 ... 5.0)
-- False
--
-- And of course, nothing is a member of the empty interval.
--
-- >>> notElem 5 empty
-- True
notElem :: Rounded TowardZero p -> Interval p -> Bool
notElem x xs = not (elem x xs)
{-# INLINE notElem #-}


-- | For all @x@ in @X@, @y@ in @Y@. @x `op` y@
certainly :: Precision p => (forall b. Ord b => b -> b -> Bool) -> Interval p -> Interval p -> Bool
certainly cmp l r
    | lt && eq && gt = True
    | lt && eq       = l <=! r
    | lt &&       gt = l /=! r
    | lt             = l <!  r
    |       eq && gt = l >=! r
    |       eq       = l ==! r
    |             gt = l >!  r
    | otherwise      = False
    where
        lt = cmp False True
        eq = cmp True True
        gt = cmp True False
{-# INLINE certainly #-}

-- | Does there exist an @x@ in @X@, @y@ in @Y@ such that @x '<' y@?
(<?) :: Precision p => Interval p -> Interval p -> Bool
Empty <? _ = False
_ <? Empty = False
I ax _ <? I _ by = coerce ax < by
{-# INLINE (<?) #-}

-- | Does there exist an @x@ in @X@, @y@ in @Y@ such that @x '<=' y@?
(<=?) :: Precision p => Interval p -> Interval p -> Bool
Empty <=? _ = False
_ <=? Empty = False
I ax _ <=? I _ by = coerce ax <= by
{-# INLINE (<=?) #-}

-- | Does there exist an @x@ in @X@, @y@ in @Y@ such that @x '==' y@?
(==?) :: Interval a -> Interval a -> Bool
I ax bx ==? I ay by = coerce ax <= by && coerce bx >= ay
_ ==? _ = False
{-# INLINE (==?) #-}

-- | Does there exist an @x@ in @X@, @y@ in @Y@ such that @x '/=' y@?
(/=?) :: Interval a -> Interval a -> Bool
I ax bx /=? I ay by = coerce ax /= by || coerce bx /= ay
_ /=? _ = False
{-# INLINE (/=?) #-}

-- | Does there exist an @x@ in @X@, @y@ in @Y@ such that @x '>' y@?
(>?) :: Precision p => Interval p -> Interval p -> Bool
I _ bx >? I ay _ = bx > coerce ay
_ >? _ = False
{-# INLINE (>?) #-}

-- | Does there exist an @x@ in @X@, @y@ in @Y@ such that @x '>=' y@?
(>=?) :: Precision p => Interval p -> Interval p -> Bool
I _ bx >=? I ay _ = bx >= coerce ay
_ >=? _ = False
{-# INLINE (>=?) #-}

-- | Does there exist an @x@ in @X@, @y@ in @Y@ such that @x `op` y@?
possibly :: Precision p => (forall b. Ord b => b -> b -> Bool) -> Interval p -> Interval p -> Bool
possibly cmp l r
    | lt && eq && gt = True
    | lt && eq       = l <=? r
    | lt &&       gt = l /=? r
    | lt             = l <? r
    |       eq && gt = l >=? r
    |       eq       = l ==? r
    |             gt = l >? r
    | otherwise      = False
    where
        lt = cmp LT EQ
        eq = cmp EQ EQ
        gt = cmp GT EQ
{-# INLINE possibly #-}

-- | Check if interval @X@ totally contains interval @Y@
--
-- >>> (20 ... 40 :: Interval Double) `contains` (25 ... 35 :: Interval Double)
-- True
--
-- >>> (20 ... 40 :: Interval Double) `contains` (15 ... 35 :: Interval Double)
-- False
contains :: Precision p => Interval p -> Interval p -> Bool
contains _ Empty = True
contains (I ax bx) (I ay by) = ax <= ay && by <= bx
contains Empty I{} = False
{-# INLINE contains #-}

-- | Flipped version of `contains`. Check if interval @X@ a subset of interval @Y@
--
-- >>> (25 ... 35 :: Interval Double) `isSubsetOf` (20 ... 40 :: Interval Double)
-- True
--
-- >>> (20 ... 40 :: Interval Double) `isSubsetOf` (15 ... 35 :: Interval Double)
-- False
isSubsetOf :: Precision p => Interval p -> Interval p -> Bool
isSubsetOf = flip contains
{-# INLINE isSubsetOf #-}

-- | Calculate the intersection of two intervals.
--
-- >>> intersection (1 ... 10 :: Interval Double) (5 ... 15 :: Interval Double)
-- 5.0 ... 10.0
intersection :: Precision p => Interval p -> Interval p -> Interval p
intersection x@(I a b) y@(I a' b')
  | x /=! y   = Empty
  | otherwise = I (max a a') (min b b')
intersection _ _ = Empty
{-# INLINE intersection #-}

-- | Calculate the convex hull of two intervals
--
-- >>> hull (0 ... 10 :: Interval Double) (5 ... 15 :: Interval Double)
-- 0.0 ... 15.0
--
-- >>> hull (15 ... 85 :: Interval Double) (0 ... 10 :: Interval Double)
-- 0.0 ... 85.0
hull :: Precision p => Interval p -> Interval p -> Interval p
hull (I a b) (I a' b') = I (min a a') (max b b')
hull Empty x = x
hull x Empty = x
{-# INLINE hull #-}

-- | Bisect an interval at its midpoint.
--
-- >>> bisect (10.0 ... 20.0)
-- (10.0 ... 15.0,15.0 ... 20.0)
--
-- >>> bisect (singleton 5.0)
-- (5.0 ... 5.0,5.0 ... 5.0)
--
-- >>> bisect Empty
-- (Empty,Empty)
bisect :: Precision p => Interval p -> (Interval p, Interval p)
bisect Empty = (Empty,Empty)
bisect (I a b) = (a...coerce m, succUlp m...b) where m = a + (coerce b - a) / 2
{-# INLINE bisect #-}

-- @'divNonZero' X Y@ assumes @0 `'notElem'` Y@
divNonZero :: Precision p => Interval p -> Interval p -> Interval p
divNonZero (I a b) (I a' b') =
  minimum [a / a', a / coerce b', coerce b / a', coerce b / coerce b']
  ...
  maximum [coerce a / coerce a', coerce a / b', b / coerce a', b / b']
divNonZero _ _ = Empty

-- @'divPositive' X y@ assumes y > 0, and divides @X@ by [0 ... y]
divPositive :: Precision p => Interval p -> Rounded TowardInf p -> Interval p
divPositive Empty _ = Empty
divPositive x@(I a b) y
  | a == 0 && b == 0 = x
  | b < 0 || isNegativeZero b = negInfinity ... (b / y)
  | a < 0 = whole
  | otherwise = (a / coerce y) ... posInfinity
{-# INLINE divPositive #-}

-- divNegative assumes y < 0 and divides the interval @X@ by [y ... 0]
divNegative :: Precision p => Interval p -> Rounded TowardNegInf p -> Interval p
divNegative Empty _ = Empty
divNegative x@(I a b) y
  | a == 0 && b == 0 = negate x -- flip negative zeros
  | b < 0 || isNegativeZero b = (coerce b / y) ... posInfinity
  | a < 0     = whole
  | otherwise = negInfinity ... (coerce a / coerce y)
{-# INLINE divNegative #-}

divZero :: Precision p => Interval p -> Interval p
divZero x@(I a b)
  | a == 0 && b == 0 = x
  | otherwise        = whole
divZero Empty = Empty
{-# INLINE divZero #-}

instance Precision p => Fractional (Interval p) where
  -- TODO: check isNegativeZero properly?
  _ / Empty = Empty
  x / y@(I a b)
    | 0 `notElem` y = divNonZero x y
    | iz && sz  = error "divide by zero"
    | iz        = divPositive x b
    |       sz  = divNegative x a
    | otherwise = divZero x
    where
      iz = a == 0
      sz = b == 0
  recip Empty = Empty
  recip (I a b) = min (recip $ coerce a) (recip $ coerce b) ... max (recip $ coerce a) (recip $ coerce b)
  {-# INLINE recip #-}
  fromRational = I <$> fromRational <*> fromRational
  {-# INLINE fromRational #-}

midpoint :: Precision p => Interval p -> Rounded TowardNegInf p
midpoint (I a b) = a + (coerce b - a) / 2
midpoint _ = 0/0 -- TODO: use mpfr's nan

instance Precision p => RealFrac (Interval p) where
  properFraction x = (b, x - fromIntegral b)
    where b = truncate (midpoint x)
  {-# INLINE properFraction #-}
  ceiling x = ceiling (sup x)
  {-# INLINE ceiling #-}
  floor x = floor (inf x)
  {-# INLINE floor #-}
  round x = round (midpoint x)
  {-# INLINE round #-}
  truncate x = truncate (midpoint x)
  {-# INLINE truncate #-}

instance Precision p => Floating (Interval p) where
  pi = I pi pi
  {-# INLINE pi #-}

  exp = increasing exp
  {-# INLINE exp #-}

  log (I a b) = (if a > 0 then log a else negInfinity) ... log b
  log Empty = Empty
  {-# INLINE log #-}

  cos Empty = Empty
  cos x
    | width t >= pi = negate 1 ... 1
    | inf t >= pi = negate $ cos (t - pi)
    | sup t <= pi = decreasing cos t
    | sup t <= 2 * pi = negate 1 ... cos (((pi * 2 - sup t) `min` coerce (inf t)))
    | otherwise = negate 1 ... 1
    where
      t = fmod x (pi * 2)
  {-# INLINE cos #-}

  sin Empty = Empty
  sin x = cos (x - pi / 2)
  {-# INLINE sin #-}

  tan Empty = Empty
  tan x
    | inf t' <= negate pi / 2 || sup t' >= pi / 2 = whole
    | otherwise = increasing tan x
    where
      t = x `fmod` pi
      t' | t >= pi / 2 = t - pi
         | otherwise   = t
  {-# INLINE tan #-}

  asin Empty = Empty
  asin (I a b)
    | b < -1 || a > 1 = Empty
    | otherwise =
      (if a <= -1 then negate pi / 2 else asin a)
      ...
      (if b >= 1 then pi / 2 else asin b)
  {-# INLINE asin #-}

  acos Empty = Empty
  acos (I a b)
    | b < -1 || a > 1 = Empty
    | otherwise =
      (if b >= 1 then 0 else acos (coerce b))
      ...
      (if a < -1 then pi else acos (coerce a))
  {-# INLINE acos #-}

  atan = increasing atan
  {-# INLINE atan #-}

  sinh = increasing sinh
  {-# INLINE sinh #-}

  cosh Empty = Empty
  cosh x@(I a b)
    | b < 0  = decreasing cosh x
    | a >= 0 = increasing cosh x
    | otherwise  = I 0 $ cosh $ if negate a > coerce b then coerce a else b
  {-# INLINE cosh #-}

  tanh = increasing tanh
  {-# INLINE tanh #-}

  asinh = increasing asinh
  {-# INLINE asinh #-}

  acosh Empty = Empty
  acosh (I a b)
    | b < 1 = Empty
    | otherwise = I lo $ acosh b
    where lo | a <= 1    = 0
             | otherwise = acosh a
  {-# INLINE acosh #-}

  atanh Empty = Empty
  atanh (I a b)
    | b < -1 || a > 1 = Empty
    | otherwise =
      (if a <= - 1 then negInfinity else atanh a)
      ...
      (if b >= 1 then posInfinity else atanh b)
  {-# INLINE atanh #-}
