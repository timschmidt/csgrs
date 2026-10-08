{-# LANGUAGE DataKinds #-}
{-# LANGUAGE BangPatterns #-}
{-# LANGUAGE KindSignatures #-}
{-# LANGUAGE MagicHash #-}
{-# LANGUAGE MultiWayIf #-}
{-# LANGUAGE PostfixOperators #-}
{-# LANGUAGE ScopedTypeVariables #-}
{-# LANGUAGE BangPatterns #-}

-----------------------------------------------------------------------------
-- | This module exports a bunch of utilities for working inside the CReal
-- datatype. One should be careful to maintain the CReal invariant when using
-- these functions
----------------------------------------------------------------------------
module Data.CReal.Internal
  (
    -- * The CReal type
    CReal(..)
    -- ** Memoization
  , Cache(..)
    -- ** Simple utilities
  , atPrecision
  , crealPrecision

    -- * More efficient variants of common functions
    -- Note that the preconditions to these functions are not checked
    -- ** Additive
  , plusInteger
    -- ** Multiplicative
  , mulBounded
  , (.*.)
  , mulBoundedL
  , (.*)
  , (*.)
  , recipBounded
  , shiftL
  , shiftR
  , square
  , squareBounded

    -- ** Exponential
  , expBounded
  , expPosNeg
  , logBounded

    -- ** Trigonometric
  , atanBounded
  , sinBounded
  , cosBounded

    -- * Utilities for operating inside CReals
  , crMemoize
  , powerSeries
  , alternateSign

    -- ** Integer operations
  , (/.)
  , (/^)
  , log2
  , log10
  , isqrt

    -- * Utilities for converting CReals to Strings
  , showAtPrecision
  , decimalDigitsAtPrecision
  , rationalToDecimal
  ) where

import Data.List (scanl')
import qualified Data.Bits as B
import Data.Bits hiding (shiftL, shiftR)
import GHC.Base (Int(..))
import GHC.Integer.Logarithms (integerLog2#, integerLogBase#)
import GHC.Real (Ratio(..), (%))
import GHC.TypeLits
import Text.Read
import qualified Text.Read.Lex as L
import System.Random (Random(..))
import Control.Concurrent.MVar
import Control.Exception
import System.IO.Unsafe (unsafePerformIO)

{-# ANN module ("HLint: ignore Reduce duplication" :: String) #-}

-- $setup
-- >>> :set -XDataKinds
-- >>> :set -XPostfixOperators

default ()

-- | The Cache type represents a way to memoize a `CReal`. It holds the largest
-- precision the number has been evaluated that, as well as the value. Rounding
-- it down gives the value for lower numbers.
data Cache
  = Never
  | Current {-# UNPACK #-} !Int !Integer
  deriving (Show)

-- | The type CReal represents a fast binary Cauchy sequence. This is a Cauchy
-- sequence with the invariant that the pth element divided by 2^p will be
-- within 2^-p of the true value. Internally this sequence is represented as a
-- function from Ints to Integers, as well as an `MVar` to hold the highest
-- precision cached value.
data CReal (n :: Nat) = CR {-# UNPACK #-} !(MVar Cache) (Int -> Integer)

-- | 'crMemoize' takes a fast binary Cauchy sequence and returns a CReal
-- represented by that sequence which will memoize the values at each
-- precision. This is essential for getting good performance.
crMemoize :: (Int -> Integer) -> CReal n
crMemoize fn = unsafePerformIO $ do
  mvc <- newMVar Never
  return $ CR mvc fn

-- | crealPrecision x returns the type level parameter representing x's default
-- precision.
--
-- >>> crealPrecision (1 :: CReal 10)
-- 10
crealPrecision :: KnownNat n => CReal n -> Int
crealPrecision = fromInteger . natVal

-- | @x \`atPrecision\` p@ returns the numerator of the pth element in the
-- Cauchy sequence represented by x. The denominator is 2^p.
--
-- >>> 10 `atPrecision` 10
-- 10240
atPrecision :: CReal n -> Int -> Integer
(CR mvc f) `atPrecision` (!p) = unsafePerformIO $ modifyMVar mvc $ \vc -> do
  vc' <- evaluate vc
  case vc' of
    Current j v | j >= p ->
      pure (vc', v /^ (j - p))
    _ -> do
      v <- evaluate $ f p
      let !vcn = Current p v
      pure (vcn, v)
{-# INLINABLE atPrecision #-}

-- | A CReal with precision p is shown as a decimal number d such that d is
-- within 2^-p of the true value.
--
-- >>> show (47176870 :: CReal 0)
-- "47176870"
--
-- >>> show (pi :: CReal 230)
-- "3.1415926535897932384626433832795028841971693993751058209749445923078164"
instance KnownNat n => Show (CReal n) where
  show x = showAtPrecision (crealPrecision x) x

-- | The instance of Read will read an optionally signed number expressed in
-- decimal scientific notation
instance Read (CReal n) where
  readPrec = parens $ do
    lit <- lexP
    case lit of
      Number n -> return $ fromRational $ L.numberToRational n
      Symbol "-" -> prec 6 $ do
        lit' <- lexP
        case lit' of
          Number n -> return $ fromRational $ negate $ L.numberToRational n
          _ -> pfail
      _ -> pfail
  {-# INLINE readPrec #-}

  readListPrec = readListPrecDefault
  {-# INLINE readListPrec #-}

  readsPrec = readPrec_to_S readPrec
  {-# INLINE readsPrec #-}

  readList = readPrec_to_S readListPrec 0
  {-# INLINE readList #-}

-- | @signum (x :: CReal p)@ returns the sign of @x@ at precision @p@. It's
-- important to remember that this /may not/ represent the actual sign of @x@ if
-- the distance between @x@ and zero is less than 2^-@p@.
--
-- This is a little bit of a fudge, but it's probably better than failing to
-- terminate when trying to find the sign of zero. The class still respects the
-- abs-signum law though.
--
-- >>> signum (0.1 :: CReal 2)
-- 0.0
--
-- >>> signum (0.1 :: CReal 3)
-- 1.0
instance Num (CReal n) where
  {-# INLINE fromInteger #-}
  fromInteger i = let
    !vc = Current 0 i
    in unsafePerformIO $ do
      mvc <- newMVar vc
      return $ CR mvc (B.shiftL i)

  -- @negate@ and @abs@ try to give initial guesses, but don't wait if the
  -- @\'MVar\'@ is being used elsewhere.
  {-# INLINE negate #-}
  negate (CR mvc fn) = unsafePerformIO $ do
    vcc <- tryReadMVar mvc
    let
      !vcn = case vcc of
        Nothing -> Never
        Just Never -> Never
        Just (Current p v) -> Current p (negate v)
    mvn <- newMVar vcn
    return $ CR mvn (negate . fn)

  {-# INLINE abs #-}
  abs (CR mvc fn) = unsafePerformIO $ do
    vcc <- tryReadMVar mvc
    let
      !vcn = case vcc of
        Nothing -> Never
        Just Never -> Never
        Just (Current p v) -> Current p (abs v)
    mvn <- newMVar vcn
    return $ CR mvn (abs . fn)

  {-# INLINE (+) #-}
  x1 + x2 = crMemoize (\p -> let n1 = atPrecision x1 (p + 2)
                                 n2 = atPrecision x2 (p + 2)
                                 in (n1 + n2) /^ 2)

  {-# INLINE (-) #-}
  x1 - x2 = crMemoize (\p -> let n1 = atPrecision x1 (p + 2)
                                 n2 = atPrecision x2 (p + 2)
                                 in (n1 - n2) /^ 2)

  {-# INLINE (*) #-}
  x1 * x2 = let
    s1 = log2 (abs (atPrecision x1 0) + 2) + 3
    s2 = log2 (abs (atPrecision x2 0) + 2) + 3
    in crMemoize (\p -> let n1 = atPrecision x1 (p + s2)
                            n2 = atPrecision x2 (p + s1)
                        in (n1 * n2) /^ (p + s1 + s2))

  signum x = crMemoize (\p -> B.shiftL (signum (x `atPrecision` p)) p)

-- | Taking the reciprocal of zero will not terminate
instance Fractional (CReal n) where
  {-# INLINE fromRational #-}
  -- Use @roundD@ instead of @/.@ because we know @d > 0@ for a valid Rational.
  fromRational (n :% d) = crMemoize (\p -> roundD (B.shiftL n p) d)

  {-# INLINE recip #-}
  -- TODO: Make recip 0 throw an error (if, for example, it would take more
  -- than 4GB of memory to represent the result)
  recip x = let
    s = findFirstMonotonic ((3 <=) . abs . atPrecision x)
    in crMemoize (\p -> let n = atPrecision x (p + 2 * s + 2)
                        in bit (2 * p + 2 * s + 2) /. n)

instance Floating (CReal n) where
  -- TODO: Could we use something faster such as Ramanujan's formula
  pi = piBy4 `shiftL` 2

  exp x = let o = shiftL (x *. recipBounded (shiftL ln2 1)) 1
              l = atPrecision o 0
              y = x - fromInteger l *. ln2
          in if l == 0
               then expBounded x
               else expBounded y `shiftL` fromInteger l

  -- | Range reduction on the principle that ln (a * b) = ln a + ln b
  log x = let l = log2 (atPrecision x 2) - 2
          in if   -- x <= 0.75
                | l < 0  -> - log (recip x)
                  -- 0.75 <= x <= 2
                | l == 0 -> logBounded x
                  -- x >= 2
                | otherwise  -> let a = x `shiftR` l
                                in logBounded a + fromIntegral l *. ln2

  sqrt x = crMemoize (\p -> let n = atPrecision x (2 * p)
                            in isqrt n)

  -- | This will diverge when the base is not positive
  x ** y = exp (log x * y)

  logBase x y = log y / log x

  sin x = cos (x - piBy2)

  cos x = let o = shiftL (x *. recipBounded pi) 2
              s = atPrecision o 1 /^ 1
              octant = fromInteger $ s .&. 7
              offset = x - (fromIntegral s *. piBy4)
              fs = [          cosBounded
                   , negate . sinBounded . subtract piBy4
                   , negate . sinBounded
                   , negate . cosBounded . (piBy4-)
                   , negate . cosBounded
                   ,          sinBounded . subtract piBy4
                   ,          sinBounded
                   ,          cosBounded . (piBy4-)]
          in (fs !! octant) offset

  tan x = sin x .* recip (cos x)

  asin x = atan (x .*. recipBounded (1 + sqrt (1 - squareBounded x))) `shiftL` 1

  acos x = piBy2 - asin x

  atan x = let -- q is 4 times x to within 1/4
               q = x `atPrecision` 2
           in if   -- x <= -1
                 | q <  -4 -> atanBounded (negate (recipBounded x)) - piBy2
                   -- -1.25 <= x <= -0.75
                 | q == -4 -> -(piBy4 + atanBounded ((x + 1) .*. recipBounded (x - 1)))
                   -- 0.75 <= x <= 1.25
                 | q ==  4 -> piBy4 + atanBounded ((x - 1) .*. recipBounded (x + 1))
                   -- x >= 1
                 | q >   4 -> piBy2 - atanBounded (recipBounded x)
                   -- -0.75 <= x <= 0.75
                 | otherwise -> atanBounded x

  -- TODO: benchmark replacing these with their series expansion
  sinh x = let (expX, expNegX) = expPosNeg x
           in (expX - expNegX) `shiftR` 1
  cosh x = let (expX, expNegX) = expPosNeg x
           in (expX + expNegX) `shiftR` 1
  tanh x = let e2x = exp (x `shiftL` 1)
           in (e2x - 1) *. recipBounded (e2x + 1)

  asinh x = log (x + sqrt (square x + 1))
  acosh x = log (x + sqrt (x + 1) * sqrt (x - 1))
  atanh x = (log (1 + x) - log (1 - x)) `shiftR` 1

-- | 'toRational' returns the CReal n evaluated at a precision of 2^-n
instance KnownNat n => Real (CReal n) where
  toRational x = let p = crealPrecision x
                 in x `atPrecision` p % bit p

instance KnownNat n => RealFrac (CReal n) where
  properFraction x = let p = crealPrecision x
                         v = x `atPrecision` p
                         r = v .&. (bit p - 1)
                         c = unsafeShiftR (v - r) p
                         n = if c < 0 && r /= 0 then c + 1 else c
                         f = plusInteger x (negate n)
                     in (fromInteger n, f)

  truncate x = let p = crealPrecision x
                   v = x `atPrecision` p
                   r = v .&. (bit p - 1)
                   c = unsafeShiftR (v - r) p
                   n = if c < 0 && r /= 0 then c + 1 else c
                   in fromInteger n

  round x = let p = crealPrecision x
                n = (x `atPrecision` p) /^ p
                in fromInteger n

  ceiling x = let p = crealPrecision x
                  v = x `atPrecision` p
                  r = v .&. (bit p - 1)
                  n = unsafeShiftR (v - r) p
                  in if r /= 0 then fromInteger $ n + 1 else fromInteger n

  floor x = let p = crealPrecision x
                v = x `atPrecision` p
                r = v .&. (bit p - 1)
                n = unsafeShiftR (v - r) p
                in fromInteger n

-- | Several of the functions in this class ('floatDigits', 'floatRange',
-- 'exponent', 'significand') only make sense for floats represented by a
-- mantissa and exponent. These are bound to error.
--
-- @atan2 y x `atPrecision` p@ performs the comparison to determine the
-- quadrant at precision p. This can cause atan2 to be slightly slower than atan
instance KnownNat n => RealFloat (CReal n) where
  floatRadix _ = 2
  floatDigits = crealPrecision
  floatRange _ = error "Data.CReal.Internal floatRange"
  decodeFloat x = let p = crealPrecision x
                  in (x `atPrecision` p, -p)
  encodeFloat m n = if n <= 0
    then fromRational (m % bit (negate n))
    else fromRational (unsafeShiftL m n :% 1)
  exponent _ = 0
  significand = id
  scaleFloat = flip shiftL
  isNaN _ = False
  isInfinite _ = False
  isDenormalized _ = False
  isNegativeZero _ = False
  isIEEE _ = False
  atan2 y x = crMemoize (\p ->
    let y' = y `atPrecision` p
        x' = x `atPrecision` p
        θ = if | x' > 0            ->  atan (y/x)
               | x' == 0 && y' > 0 ->  piBy2
               | x' <  0 && y' > 0 ->  pi + atan (y/x)
               | x' <= 0 && y' < 0 -> -atan2 (-y) x
               | y' == 0 && x' < 0 ->  pi    -- must be after the previous test on zero y
               | x'==0 && y'==0    ->  0     -- must be after the other double zero tests
               | otherwise         ->  error "Data.CReal.Internal atan2"
    in θ `atPrecision` p)

-- | Values of type @CReal p@ are compared for equality at precision @p@. This
-- may cause values which differ by less than 2^-p to compare as equal.
--
-- >>> 0 == (0.1 :: CReal 1)
-- True
instance KnownNat n => Eq (CReal n) where
  -- TODO, should this try smaller values first?
  CR mvx _ == CR mvy _ | mvx == mvy = True
  x == y = let p = crealPrecision x
           in ((x - y) `atPrecision` p) == 0


-- | Like equality, values of type @CReal p@ are compared at precision @p@.
instance KnownNat n => Ord (CReal n) where
  compare (CR mvx _) (CR mvy _) | mvx == mvy = EQ
  compare x y =
    let p = crealPrecision x
    in compare ((x - y) `atPrecision` p) 0
  max x y = crMemoize (\p -> max (atPrecision x p) (atPrecision y p))
  min x y = crMemoize (\p -> min (atPrecision x p) (atPrecision y p))

-- | The 'Random' instance for @\'CReal\' p@ will return random number with at
-- least @p@ digits of precision, every digit after that is zero.
instance KnownNat n => Random (CReal n) where
  randomR (lo, hi) g = let d = hi - lo
                           l = 1 + log2 (abs d `atPrecision` 0)
                           p = l + crealPrecision lo
                           (n, g') = randomR (0, 2^p) g
                           r = fromRational (n % 2^p)
                       in (r * d + lo, g')
  random g = let p = 1 + crealPrecision (undefined :: CReal n)
                 (n, g') = randomR (0, max 0 (2^p - 2)) g
                 r = fromRational (n % 2^p)
             in (r, g')

--------------------------------------------------------------------------------
-- Some utility functions
--------------------------------------------------------------------------------

--
-- Constants
--

piBy4 :: CReal n
piBy4 = atanBounded (recipBounded 5) `shiftL` 2 - atanBounded (recipBounded 239) -- Machin Formula

piBy2 :: CReal n
piBy2 = piBy4 `shiftL` 1

ln2 :: CReal n
ln2 = logBounded 2

--
-- Bounded multiplication
--

infixl 7 `mulBounded`, `mulBoundedL`, .*, *., .*.

-- | Alias for @'mulBoundedL'@
(.*) :: CReal n -> CReal n -> CReal n
(.*) = mulBoundedL

-- | Alias for @flip 'mulBoundedL'@
(*.) :: CReal n -> CReal n -> CReal n
(*.) = flip mulBoundedL

-- | Alias for @'mulBounded'@
(.*.) :: CReal n -> CReal n -> CReal n
(.*.) = mulBounded

-- | A more efficient multiply with the restriction that the first argument
-- must be in the closed range [-1..1]
mulBoundedL :: CReal n -> CReal n -> CReal n
mulBoundedL x1 x2 = let
  s1 = 4
  s2 = log2 (abs (atPrecision x2 0) + 2) + 3
  in crMemoize (\p -> let n1 = atPrecision x1 (p + s2)
                          n2 = atPrecision x2 (p + s1)
                      in (n1 * n2) /^ (p + s1 + s2))

-- | A more efficient multiply with the restriction that both values must be
-- in the closed range [-1..1]
mulBounded :: CReal n -> CReal n -> CReal n
mulBounded x1 x2 = let
  s1 = 4
  s2 = 4
  in crMemoize (\p -> let n1 = atPrecision x1 (p + s2)
                          n2 = atPrecision x2 (p + s1)
                      in (n1 * n2) /^ (p + s1 + s2))

-- | A more efficient 'recip' with the restriction that the input must have
-- absolute value greater than or equal to 1
recipBounded :: CReal n -> CReal n
recipBounded x = crMemoize (\p -> let s = 2
                                      n = atPrecision x (p + 2 * s + 2)
                                  in bit (2 * p + 2 * s + 2) /. n)

-- | Return the square of the input, more efficient than @('*')@
{-# INLINABLE square #-}
square :: CReal n -> CReal n
square x = let
  s = log2 (abs (atPrecision x 0) + 2) + 3
  in crMemoize (\p -> let n = atPrecision x (p + s)
                      in (n * n) /^ (p + 2 * s))

-- | A more efficient 'square' with the restrictuion that the value must be in
-- the closed range [-1..1]
{-# INLINABLE squareBounded #-}
squareBounded :: CReal n -> CReal n
squareBounded x@(CR mvc _) = unsafePerformIO $ do
  vcc <- tryReadMVar mvc
  let
    !s = 4
    !vcn = case vcc of
      Nothing -> Never
      Just Never -> Never
      Just (Current j n) -> case j - s of
        p | p < 0 -> Never
        p -> Current p ((n * n) /^ (p + 2 * s))
    fn' !p = let n = atPrecision x (p + s)
             in (n * n) /^ (p + 2 * s)
  mvn <- newMVar vcn
  return $ CR mvn fn'

--
-- Bounded exponential functions and expPosNeg
--

-- | A more efficient 'exp' with the restriction that the input must be in the
-- closed range [-1..1]
expBounded :: CReal n -> CReal n
expBounded x = let q = (1%) <$> scanl' (*) 1 [1..]
               in powerSeries q (max 5) x

-- | A more efficient 'log' with the restriction that the input must be in the
-- closed range [2/3..2]
logBounded :: CReal n -> CReal n
logBounded x = let q = [1 % n | n <- [1..]]
                   y = (x - 1) .* recip x
               in y .* powerSeries q id y

-- | @expPosNeg x@ returns @(exp x, exp (-x))#
expPosNeg :: CReal n -> (CReal n, CReal n)
expPosNeg x = let o = x / ln2
                  l = atPrecision o 0
                  y = x - fromInteger l * ln2
              in if l == 0
                   then (expBounded x, expBounded (-x))
                   else (expBounded y `shiftL` fromInteger l,
                         expBounded (negate y) `shiftR` fromInteger l)

--
-- Bounded trigonometric functions
--

-- | A more efficient 'sin' with the restriction that the input must be in the
-- closed range [-1..1]
sinBounded :: CReal n -> CReal n
sinBounded x = let q = alternateSign (scanl' (*) 1 [ 1 % (n*(n+1)) | n <- [2,4..]])
               in x .* powerSeries q (max 1) (squareBounded x)

-- | A more efficient 'cos' with the restriction that the input must be in the
-- closed range [-1..1]
cosBounded :: CReal n -> CReal n
cosBounded x = let q = alternateSign (scanl' (*) 1 [1 % (n*(n+1)) | n <- [1,3..]])
               in powerSeries q (max 1) (squareBounded x)

-- | A more efficient 'atan' with the restriction that the input must be in the
-- closed range [-1..1]
atanBounded :: CReal n -> CReal n
atanBounded x = let q = scanl' (*) 1 [n % (n + 1) | n <- [2,4..]]
                    s = squareBounded x
                    rd = recipBounded (plusInteger s 1)
                in (x .*. rd) .* powerSeries q (+1) (s .*. rd)

--
-- Integer addition
--

infixl 6 `plusInteger`

-- | @x \`plusInteger\` n@ is equal to @x + fromInteger n@, but more efficient
{-# INLINE plusInteger #-}
plusInteger :: CReal n -> Integer -> CReal n
plusInteger x 0 = x
plusInteger (CR mvc fn) n = unsafePerformIO $ do
  vcc <- tryReadMVar mvc
  let
    !vcn = case vcc of
      Nothing -> Never
      Just Never -> Never
      Just (Current j v) -> Current j (v + unsafeShiftL n j)
    fn' !p = fn p + B.shiftL n p
  mvc' <- newMVar vcn
  return $ CR mvc' fn'

--
-- Multiplication with powers of two
--

infixl 8 `shiftL`, `shiftR`

-- | @x \`shiftR\` n@ is equal to @x@ divided by 2^@n@
--
-- @n@ can be negative or zero
--
-- This can be faster than doing the division
shiftR :: CReal n -> Int -> CReal n
shiftR x n = crMemoize (\p -> let p' = p - n
                              in if p' >= 0
                                 then atPrecision x p'
                                 else atPrecision x 0 /^ negate p')

-- | @x \`shiftL\` n@ is equal to @x@ multiplied by 2^@n@
--
-- @n@ can be negative or zero
--
-- This can be faster than doing the multiplication
shiftL :: CReal n -> Int -> CReal n
shiftL x = shiftR x . negate


--
-- Showing CReals
--

-- | Return a string representing a decimal number within 2^-p of the value
-- represented by the given @CReal p@.
showAtPrecision :: Int -> CReal n -> String
showAtPrecision p x = let places = decimalDigitsAtPrecision p
                          r      = atPrecision x p % bit p
                      in rationalToDecimal places r

-- | How many decimal digits are required to represent a number to within 2^-p
decimalDigitsAtPrecision :: Int -> Int
decimalDigitsAtPrecision 0 = 0
decimalDigitsAtPrecision p = log10 (bit p) + 1

-- | @rationalToDecimal p x@ returns a string representing @x@ at @p@ decimal
-- places.
rationalToDecimal :: Int -> Rational -> String
rationalToDecimal places (n :% d) = p ++ is ++ if places > 0 then "." ++ fs else ""
  where p = case signum n of
              -1 -> "-"
              _  -> ""
        ds = show (roundD (abs n * 10^places) d)
        l = length ds
        (is, fs) = if l <= places then ("0", replicate (places - l) '0' ++ ds) else splitAt (l - places) ds


--
-- Integer operations
--

divZeroErr :: a
divZeroErr = error "Division by zero"
{-# NOINLINE divZeroErr #-}

roundD :: Integer -> Integer -> Integer
roundD n d = case divMod n d of
  (q, r) -> case compare (unsafeShiftL r 1) d of
    LT -> q
    EQ -> if testBit q 0 then q + 1 else q
    GT -> q + 1
{-# INLINE roundD #-}

infixl 7 /.
-- | Division rounding to the nearest integer and rounding half integers to the
-- nearest even integer.
(/.) :: Integer -> Integer -> Integer
(!n) /. (!d) = case compare d 0 of
  LT -> roundD (negate n) (negate d)
  EQ -> divZeroErr
  GT -> roundD n d
{-# INLINABLE (/.) #-}

infixl 7 /^
-- | @n /^ p@ is equivalent to @n \'/.\' (2^p)@, but faster, and it works for
-- negative values of p.
(/^) :: Integer -> Int -> Integer
(!n) /^ (!p) = case compare p 0 of
  LT -> unsafeShiftL n (negate p)
  EQ -> n
  GT -> let
    !bp = bit p
    !r = n .&. (bp - 1)
    !q = unsafeShiftR (n - r) p
    in case compare (unsafeShiftL r 1) bp of
      LT -> q
      EQ -> if testBit q 0 then q + 1 else q
      GT -> q + 1

-- | @log2 x@ returns the base 2 logarithm of @x@ rounded towards zero.
--
-- The input must be positive
{-# INLINE log2 #-}
log2 :: Integer -> Int
log2 x = I# (integerLog2# x)

-- | @log10 x@ returns the base 10 logarithm of @x@ rounded towards zero.
--
-- The input must be positive
{-# INLINE log10 #-}
log10 :: Integer -> Int
log10 x = I# (integerLogBase# 10 x)

-- | @isqrt x@ returns the square root of @x@ rounded towards zero.
--
-- The input must not be negative
{-# INLINABLE isqrt #-}
isqrt :: Integer -> Integer
isqrt x | x < 0     = error "Sqrt applied to negative Integer"
        | x == 0    = 0
        | otherwise = until satisfied improve initialGuess
  where improve r    = unsafeShiftR (r + x `div` r) 1
        satisfied r  = let r2 = r * r in r2 <= x && r2 + unsafeShiftL r 1 >= x
        initialGuess = bit (unsafeShiftR (log2 x) 1)

--
-- Searching
--

-- | Given a monotonic function
{-# INLINABLE findFirstMonotonic #-}
findFirstMonotonic :: (Int -> Bool) -> Int
findFirstMonotonic p = findBounds 0 1
  where findBounds !l !u = if p u then binarySearch l u
                                  else findBounds u (u * 2)
        binarySearch !l !u = let !m = l + (u - l) `div` 2
                             in if | l+1 == u  -> l
                                   | p m       -> binarySearch l m
                                   | otherwise -> binarySearch m u


--
-- Power series
--

-- | Apply 'negate' to every other element, starting with the second
--
-- >>> alternateSign [1..5]
-- [1,-2,3,-4,5]
{-# INLINABLE alternateSign #-}
alternateSign :: Num a => [a] -> [a]
alternateSign ls = foldr
  (\a r b -> if b then negate a : r False else a : r True)
  (const [])
  ls
  False

-- | @powerSeries q f x `atPrecision` p@ will evaluate the power series with
-- coefficients @q@ up to the coefficient at index @f p@ at value @x@
--
-- @f@ should be a function such that the CReal invariant is maintained. This
-- means that if the power series @y = a[0] + a[1] + a[2] + ...@ is evaluated
-- at precision @p@ then the sum of every @a[n]@ for @n > f p@ must be less than
-- 2^-p.
--
-- This is used by all the bounded transcendental functions.
--
-- >>> let (!) x = product [2..x]
-- >>> powerSeries [1 % (n!) | n <- [0..]] (max 5) 1 :: CReal 218
-- 2.718281828459045235360287471352662497757247093699959574966967627724
powerSeries :: [Rational] -> (Int -> Int) -> CReal n -> CReal n
powerSeries q termsAtPrecision x = crMemoize
     (\p -> let t = termsAtPrecision p
                d = log2 (toInteger t) + 2
                p' = p + d
                p'' = p' + d
                m = atPrecision x p''
                xs = (% 1) <$> iterate (\e -> m * e /^ p'') (bit p')
                r = sum . take (t + 1) . fmap (round . (* fromInteger (bit d))) $ zipWith (*) q xs
            in r /^ (2 * d))

