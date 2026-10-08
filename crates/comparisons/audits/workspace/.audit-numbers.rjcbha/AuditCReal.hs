-- ERA: Exact Real Arithmetic (version 1.0)
--
-- A tolerably efficient and possibly correct implementation of the computable
-- reals using Haskell 1.2.
--
-- David Lester, Department of Computer Science, Manchester University, M13 9PL.
--           (2000-2001)

module AuditCReal where
import Data.Ratio
import Numeric(readFloat, readSigned)

-- |The 'CReal' type implements (constructive) real numbers.
--
-- Note that the comparison operations on 'CReal' may diverge
-- since it is (by necessity) impossible to implementent them
-- correctly and always terminating.
--
-- This implementation is really David Lester's ERA package.
data CReal = CR (Int -> Integer)

instance Eq  CReal where
  x == y = s' (digitsToBits digits) == 0 where (CR s') = x-y

instance Ord CReal where
  x <= y = s' (digitsToBits digits) <= 0 where (CR s') = x-y
  x <  y = s' (digitsToBits digits) <  0 where (CR s') = x-y
  x >= y = s' (digitsToBits digits) >= 0 where (CR s') = x-y
  x >  y = s' (digitsToBits digits) >  0 where (CR s') = x-y
  max (CR x') (CR y') = CR (\p -> max (x' p) (y' p))
  min (CR x') (CR y') = CR (\p -> min (x' p) (y' p))

instance Num CReal where
  (CR x') + (CR y') = CR (\p -> round_uk ((x' (p+2) + y' (p+2)) % 4))
  (CR x') * (CR y') = CR (\p -> round_uk ((x' (p+sy)*y' (p+sx)) % 2^(p+sx+sy)))
                        where x0 = abs (x' 0)+2; y0 = abs (y' 0)+2
                              sx = sizeinbase x0 2+3; sy = sizeinbase y0 2+3
  negate (CR x')    = CR (\p -> negate (x' p))
  abs x             = max x (negate x)
  signum (CR x')    = fromInteger (signum (x' (digitsToBits digits)))
  fromInteger n     = CR (\p -> n*2^p)

instance Fractional CReal where
  recip (CR x') = CR (\p -> let s = head [n | n <- [0..], 3 <= abs (x' n)]
                              in round_uk (2^(2*p+2*s+2) % (x' (p+2*s+2))))
  fromRational x = fromInteger (numerator x) / fromInteger (denominator x)

-- two useful scaling functions:

div2n :: CReal -> Int -> CReal
div2n (CR x') n = CR (\p -> if p >= n then x' (p-n) else round_uk (x' p % 2^n))

mul2n :: CReal -> Int -> CReal
mul2n (CR x') n = CR (\p -> x' (p+n))

-- transcendental functions (mostly range reductions):

instance Floating CReal where
  pi = 16 * atan (fromRational (1 % 5)) 
                - 4 * atan (fromRational (1 % 239))
  sqrt x  = CR (\p -> floorsqrt (x' (2*p))) where (CR x') = x

  log x   = if t < 0 then error "log of negative number\n" else
            if t < 4 then - log (recip x)                  else
            if t < 8 then log_dr x                         else
            {- 7 < t -}   log_dr (div2n x n) + fromIntegral n * log2
            where (CR x') = x; t = x' 2; n = sizeinbase t 2 - 3
  exp x   = if n < 0 then div2n (exp_dr s) (fromInteger (-n)) else
            if n > 0 then mul2n (exp_dr s) (fromInteger n) else exp_dr s
            where (CR u') = x/log2; n = u' 0; s = x-fromInteger n*log2
  sin x   = if n == 0 then sin_dr y                           else
            if n == 1 then sqrt1By2 * (cos_dr y + sin_dr y)   else
            if n == 2 then cos_dr y                           else
            if n == 3 then sqrt1By2 * (cos_dr y - sin_dr y)   else
            if n == 4 then - sin_dr y                         else
            if n == 5 then - sqrt1By2 * (cos_dr y + sin_dr y) else
            if n == 6 then - cos_dr y                         else
            {- n == 7 -}   - sqrt1By2 * (cos_dr y - sin_dr y)
            where (CR z') = x/piBy4; s = round_uk (z' 2 % 4); n = s `mod` 8
                  y = x - piBy4 * fromInteger s
  cos x   = if n == 0 then cos_dr y                           else
            if n == 1 then sqrt1By2 * (cos_dr y - sin_dr y)   else
            if n == 2 then - sin_dr y                           else
            if n == 3 then - sqrt1By2 * (cos_dr y + sin_dr y)   else
            if n == 4 then - cos_dr y                         else
            if n == 5 then - sqrt1By2 * (cos_dr y - sin_dr y) else
            if n == 6 then sin_dr y                         else
            {- n == 7 -}   sqrt1By2 * (cos_dr y + sin_dr y)
            where (CR z') = x/piBy4; s = round_uk (z' 2 % 4); n = s `mod` 8
                  y = x - piBy4 * fromInteger s
  atan x  = if t <  -5 then atan_dr (negate (recip x)) - piBy2 else
            if t == -4 then -piBy4 - atan_dr (xp1/xm1)         else
            if t <   4 then atan_dr x                          else
            if t ==  4 then piBy4 + atan_dr (xm1/xp1)          else
            {- t >   4 -}   piBy2 - atan_dr (recip x)
            where (CR x') = x; t = x' 2
                  xp1 = x+1; xm1 = x-1
  asin x  = if x0 >  0 then pi / 2 - atan (s/x) else
            if x0 == 0 then atan (x/s)                      else
            {- x0 <  0 -}   - atan (s/x) - pi / 2
            where (CR x') = x; x0 = x' 0; s = sqrt (1 - x*x)
  acos x  = pi / 2 - asin x
  sinh x  = (y - recip y) / 2 where y = exp x
  cosh x  = (y + recip y) / 2 where y = exp x
  tanh x  = (y - y') / (y + y') where y = exp x; y' = recip y
  asinh x = log (x + sqrt (x*x + 1))
  acosh x = log (x + sqrt (x*x - 1))
  atanh x = log ((1 + x) / (1 - x)) / 2


acc_seq :: (Rational -> Integer -> Rational) -> [Rational]
acc_seq f = scanl f (1 % 1) [1..]

exp_dr :: CReal -> CReal
exp_dr = power_series (acc_seq (\a n -> a*(1 % n))) id

log_dr :: CReal -> CReal
log_dr x = y * log_drx y where y = (x - 1) / x

log_drx :: CReal -> CReal
log_drx = power_series [1 % n | n <- [1..]] (+1)

sin_dr :: CReal -> CReal
sin_dr x = x*power_series (acc_seq (\a n -> -a*(1 % (2*n*(2*n+1))))) id (x*x)

cos_dr :: CReal -> CReal
cos_dr x = power_series (acc_seq (\a n -> -a*(1 % (2*n*(2*n-1))))) id (x*x)

atan_dr :: CReal -> CReal
atan_dr x = (x/y) * atan_drx ((x*x)/y) where y = x*x+1

atan_drx :: CReal -> CReal
atan_drx = power_series (acc_seq (\a n -> a*((2*n) % (2*n+1)))) (+1)

-- power_series takes as arguments:
--   a (rational) list of the coefficients of the power series
--   a function from the desired accuracy to the number of terms needed
--   the argument x

power_series :: [Rational] -> (Int -> Int) -> CReal -> CReal
power_series ps terms (CR x')
  = CR (\p -> let t = terms p; l2t = 2*sizeinbase (toInteger t+1) 2+6; p' = p + l2t
                  xr = x' p'; xn = 2^p'; g yn = round_uk ((yn*xr) % (2^p'))
               in round_uk (accumulate (iterate g xn) (take t ps) % (2^l2t)))
    where accumulate _      []     = 0
          accumulate []     _      = error "CReal.power_series.accumulate"
          accumulate (x:xs) (c:cs) = let t = round_uk (c*(x % 1)) in
                                     if t == 0 then 0 else t + accumulate xs cs

-- Some useful constants:

piBy2 :: CReal
piBy2 = div2n pi 1

piBy4 :: CReal
piBy4 = div2n pi 2

log2 :: CReal
log2 = div2n (log_drx (recip 2)) 1

sqrt1By2 :: CReal
sqrt1By2 = sqrt (recip 2)

instance Enum CReal where
  toEnum i         = fromIntegral i
  fromEnum _       = error "Cannot fromEnum CReal"
  enumFrom         = iterate (+ 1)
  enumFromTo n e   = takeWhile (<= e) $ iterate (+ 1)n
  enumFromThen n m = iterate (+(m-n)) n
  enumFromThenTo n m e = if m >= n then takeWhile (<= e) $ iterate (+(m-n)) n
                          else takeWhile (>= e) $ iterate (+(m-n)) n

instance Real CReal where
 -- toRational x@(CR x') = x' n % 2^n where n = digitsToBits digits
  toRational _ = error "CReal.toRational"
instance RealFrac CReal where
  properFraction x@(CR x') = (fromInteger n, x - fromInteger n) where n = x' 0

instance RealFloat CReal where
  floatRadix _ = error "CReal.floatRadix"
  floatDigits _ = error "CReal.floatDigits"
  floatRange _ = error "CReal.floatRange"
  decodeFloat _ = error "CReal.decodeFloat"
  encodeFloat _ _ = error "CReal.encodeFloat"
  exponent _ = 0
  scaleFloat 0 x = x
  significand x = x
  isNaN _ = False
  isInfinite _ = False
  isDenormalized _ = False
  isNegativeZero _ = False
  isIEEE _ = False

-- printing and reading the reals:

-- |The 'showCReal' function connverts a 'CReal' to a 'String'.
showCReal :: Int                -- ^ The number of decimals
          -> CReal              -- ^ The real number
          -> String             -- ^ The resulting string
showCReal d (CR x')
  = (if s then "-" else "") ++ zs ++ (if d /= 0 then '.':fs' else "")
    where b  = digitsToBits d
          n  = x' b
          ds = show (round_uk ((n*10^d) % 2^b))
          (s,ds') = let sgn = head ds == '-' in (sgn, if sgn then tail ds else ds)
          ds'' = take (max (d+1-length ds') 0) (repeat '0') ++ ds'
          (zs,fs) = splitAt (length ds'' -d) ds''
          fs' = case reverse $ dropWhile (== '0') $ reverse fs of
                "" -> "0"
                xs -> xs

digitsToBits :: Int -> Int
digitsToBits d = ceiling (fromIntegral d * (logBase 2.0 10.0 :: Double)) + 4

digits :: Int
digits = 40

instance Read CReal where
  readsPrec _p = readSigned readFloat

instance Show CReal where
  showsPrec p x = let xs = showCReal digits x in
                  if head xs == '-' then showParen (p > 6) (showString xs)
                                    else showString xs

-- GMP functions not provided by Haskell

sizeinbase :: Integer -> Int -> Int
sizeinbase i b = f (abs i)
                 where f n = if n <= 1 then 1 else 1 + f (n `div` toInteger b)

floorsqrt :: Integer -> Integer
floorsqrt x = until satisfy improve x
              where improve y = floor ((y*y+x) % (2*y))
                    satisfy y = y*y <= x && x <= (y+1)*(y+1)

round_uk :: Rational -> Integer
round_uk x = floor (x+1 % 2)
