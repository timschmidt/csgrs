module Data.Number.IReal.IRealOperations where

import Data.Number.IReal.IReal
import Data.Number.IReal.Scalable
import Data.Number.IReal.IntegerInterval
import Data.Number.IReal.Auxiliary
import Data.Number.IReal.Powers

import Data.Ratio
import Data.Bits

infix 3 ?, ??, =?=, <!, >!
infix 6 +-, -+-

-- Real intervals --------------------------------------------------------------

-- | Constructs an interval from midpoint and radius.
(+-) :: Rational -> Rational -> IReal
a +- b 
  |b < 0 = error "IReal.+-: negative radius"
  |otherwise = ir f
  where f p = floor (scale (a-b) p) `upto` ceiling (scale (a+b) p)

-- | Constructs an interval from end points (which can be given in any order).
(-+-) :: IReal -> IReal -> IReal
x -+- y = ir f
           where f p = min (lowerI xp) (lowerI yp) `upto` max (upperI xp) (upperI yp)
                  where xp = appr x p
                        yp = appr y p

lower, upper, mid, rad :: IReal -> IReal
-- | Returns midpoint of argument.
mid x   = ir (fromInteger . midI . appr x)

-- | Returns radius of argument.
rad x   = ir (fromInteger . radI . appr x)

-- | Returns left end point of argument.
lower x = ir (\p -> fromInteger (lowerI (appr x p)))

-- | Returns right end point of argument.
upper x = ir (\p -> fromInteger (upperI (appr x p)))

-- | Convex hull of a list of intervals.
hull :: [IReal] -> IReal
hull xs = minimum (map lower xs) -+- maximum (map upper xs)

-- | Tests whether first arg is contained in second, using total tests of given precision.
containedIn :: IReal -> IReal -> Precision -> Bool
containedIn i1 i2 p = (lower i2 <! lower i1) p && (upper i1 <! upper i2) p

-- | Intersection of intervals; empty intersection gives 'Nothing'
intersection :: IReal -> IReal -> Maybe IReal
intersection x y 
   | l > u = Nothing
   | otherwise = Just (l -+- u)
   where l = max (lower x) (lower y)
         u = min (upper x) (upper y)

-- Basic real arithmetic ------------------------------------------------------------

instance Num IReal where
  x + y = ir f 
    where f p = scale (xp+yp) (-2)
             where xp = appr x (p+2)
                   yp = appr y (p+2)
  x * y = ir f 
   where x0 = appr x 0
         y0 = appr y 0
         sx = lg2 (upperI (abs x0)) + 3
         sy = lg2 (upperI (abs y0)) + 3
         f p =  scale (xp * yp) (-(p+sx+sy)) 
           where xp = appr x (p + sy)
                 yp = appr y (p + sx)
  negate x      = ir (negate . appr x)
  abs x         = ir (abs . appr x)
  signum x      = ir (\p -> ivalCase (signum (appr x p)) 
                                      (fromInteger (bit p))
                                      ( -fromInteger (bit p))
                                      (error "IReal.signum: value includes 0"))
  fromInteger n = ir (fromInteger . shift n)

-- | Division by zero is non-terminating.
instance Fractional IReal where
  recip x = ir f
      where f p = l `upto` u
             where s = head [ n | n <- 0 : map bit [0..],
                                  let xn = appr x n, signum (lowerI xn) == signum (upperI xn)]
                   k = 2 * (p + s) + 1
                   xp = appr x (k - p)
                   l = bit k `div` upperI xp
                   u = bit k `div` lowerI xp + 1
  fromRational r = ir (\p -> fromInteger (shift (numerator r) p `div` denominator r))

-- Elementary functions --------------------------------------------------------

instance Floating IReal where
  pi = scale (atinv 5) 4 - scale (atinv 239) 2
    where atinv = ps negate

  sqrt x           = ir f
    where 
       f p
         |lowerI xp < 0  = error "IReal.sqrt: negative argument"
         |isThin xp     = fromInteger u
         |otherwise     = l `upto` u
          where (xp,q) = head [(xq,q) | q <- iterate (*2) (max 1 (2*p)), 
                                        let xq = appr x q, midI xq /= radI xq-1]
                u = isqrt (scale (upperI xp) (2*p-q)) x0
                l = isqrt (scale (lowerI xp) (2*p-q)) x0
                x0 = if p < 100 then 1 else max 1 (shift (midI (f p')) (p - p'))
                p' = shift p (-1) 
                isqrt x = until ok improve
                   where improve y = shift (y + x `div` y) (-1)
                         ok y      = y^2 <= x && x < (y+1)^2

  log x
     |not (isThin x1)
                   = log (lower x) -+- log (upper x)
     |r < 0        = error "IReal.log: negative argument"      --  x < 0
     |r < 2        = -log (recip x)                            -- -0.5 < x < 1. 
     |r == 2       = powerSeries ack 2 id (x-1) - 1             -- 0.5 < x < 1.5
     |r < 10       = scale (log0 y) 1                           -- 1 < x < 5; hence 0 < y < 2/3. 
     |otherwise    = log (scale x (-n)) + log2 * fromIntegral n
      where x1     = appr x 1
            r      = midI x1
            n      = lg2 r - 1
            y      = 1 - scale (recip (x+1)) 1
            log0 x = x * g0 (sq x)                              -- log0 used only for positive x, so does not lose interval
                                                                -- precision in spite of multiplication (g0 . sq positive, increasing)
            ack a 1 = a
            ack a n = - a * (n-1) `div` n

  exp x            = scale (powerSeries div 2 id s) (fromInteger n) 
       where x0    = appr (x * recip log2) 0
             n     = midI x0
             s     = x - fromInteger n * log2

  atan x = ir f 
    where f p
           | isThin i  = appr (atan' x) p 
           | otherwise = lowerI (appr (atan' (scale (fromInteger l) (-p))) p) `upto` 
                         upperI (appr (atan' (scale (fromInteger u) (-p))) p)
             where i@(I (l,u)) = appr x p
  -- We cannot avoid losing precision in intervals with atan' (because of the two occurrences of x in atan0).
  -- Hence the function above, which treats intervals, using that atan is strictly increasing and has
  -- derivative at most 1. atan' is used for end points (and thin intervals).
          atan' x                            
           | r < 0        = - atan' (-x)                                     --  x < 0
           | r > 8        = halfPi - atan0 (recip x)                         --  x > 2             atan0 y for 0 < y < 0.5
           | r > 1        = quarterPi + atan0 (1 - scale (recip (x+1)) 1)    --  0.25 < x < 2.25   atan0 y for -3/5 < y < 5/13
           | otherwise    = atan0 x                                          --  r = 0 or 1        -0.25 < x < 0.5
            where x2      = appr x 2
                  r       = midI x2 
                  atan0 x = x * g0 (negate (sq x))                          -- converges for |x| < 1/sqrt 2


  cos x
    | rad x >! pi `atDecimals` 2 = 0 +- 1
    | otherwise = if even n then cos0 y else -cos0 y
   where n = midI (appr (x/pi) 0)
         y = abs (x - pi * fromInteger n)  
         -- upper y < 2*pi, so by scaling with (-1) we get into 
         -- a region where cos is monotonic.
         cos0 x = f (powerSeries (\a n -> -a `div` (2*n*(2*n-1))) 1 (`div` 2) (sq (scale x (-1))))
         f x = scale (sq x) 1 - 1          

  sin x            = cos (halfPi - x) 

  asin x           = atan ( x / sqrt (1 - sq x))
  acos x           = halfPi - asin x
  sinh x           = scale (y - recip y) (-1) where y = exp x
  cosh x           = sqrt (1 + sq (sinh x))
  tanh x           = 1 - scale (recip (y + 1)) 1 where y = sq (exp x)
  asinh x          = log (x + sqrt (sq x + 1))
  acosh x          = log (x + sqrt (sq x - 1))
  atanh x          = scale (log (scale (recip (1-x)) 1 - 1)) (-1)

-- Real, RealFrac and RealFloat ------------------------------------------------------

instance Real IReal where
  toRational x = midI (appr x 100) % bit 100

-- We choose to give an error when trying to use this function for the probably common
-- case of an argument which is an integer, rather than non-termination (or, worse, 
-- risk the wrong result).
instance RealFrac IReal where
  properFraction x
     | y =?= 0 `atDecimals` 100 = error "RealFrac.properFraction: argument value differs from integer with at most 10^(-100)"
     | x > 0 = if y > 0 then (i',y) else (i'-1,y+1)
     | otherwise = if y < 0 then (i',y) else (i'+1,y-1)
     where i = midI (appr x 0)
           i' = fromIntegral i
           y = x - fromInteger i


toDouble :: IReal -> Double
toDouble x = encodeFloat (scale (midI i) (-5)) (-p)
  where p = floatDigits undefined - ceiling (logBase 2 (abs x))
        i = appr x (p+5)

-- Unfortunately, to allow us to define an instance of Floating for Complex IReal, 
-- Haskell requires (for dubious reasons) that IReal is an instance of RealFloat
instance RealFloat IReal where
   floatRadix = undefined
   floatDigits = undefined
   floatRange = undefined
   decodeFloat = undefined
   encodeFloat = undefined
   isNaN = undefined
   isInfinite = undefined
   isDenormalized = undefined
   isNegativeZero = undefined
   isIEEE = undefined

-- More functions --------------------------------------------------------------

instance Powers IReal where
  pow x 0 = 1
  pow x n = ir f
     where x0 = appr x 0
           f p = scale (pow xp n) (p - n*q)
             where xp = appr x q
                   q = p + ceiling (logBase 2 (fromIntegral n) :: Double) 
                         + (n-1) * lg2 (upperI (abs x0)) + n

-- Scaling ---------------------------------------------------------------------

instance Scalable IReal where
  scale x n = ir (\p -> if p + n >= 0 then appr x (p+n) else scale (appr x 0) (p+n))

-- Equality, ordering and enumerations -----------------------------------------

-- | Equality test for overlapping values is non-terminating.
instance Eq IReal where 
   x /= y = compare x y /= EQ

instance Ord IReal where
  compare x y = case comp x y  of
    EQ -> error "IReal.compare: overlapping values"
    x -> x
  x < y = comp x y == LT
  x > y = comp x y == GT

  max x y = ir (\p -> max' (appr x p) (appr y p))
    where max' xp yp = max (lowerI xp) (lowerI yp) `upto` max (upperI xp) (upperI yp)

  min x y = ir (\p -> min' (appr x p) (appr y p))
    where min' xp yp = min (lowerI xp) (lowerI yp) `upto` min (upperI xp) (upperI yp)

comp x y = cf 1 
   where cf p
          | lowerI dp >= 0 = GT
          | upperI dp <= 0 = LT
          | isThin dp = cf (shift p 1)
          | otherwise = EQ -- this result signifies overlapping interval(s); caught in compare
            where dp = appr (x - y) p

instance Enum IReal where
  toEnum           = fromIntegral 
  fromEnum _       = error "IReal.fromEnum: not implemented"
  enumFrom a       = [ a + fromInteger n | n <- [0..] ]
  enumFromTo a b   = takeWhile (<= b + 1/2) $ enumFrom a
  enumFromThen n m = [ n + (m - n) * fromInteger k | k <- [0..] ]
  enumFromThenTo n m e 
     | m >= n      = takeWhile (<= e + (m-n)/2) $ enumFromThen n m
     | otherwise   = takeWhile (>= e + (m-n)/2) $ enumFromThen n m

-- | Total, approximate equality test. If @x =?= y `atDecimals` d@  returns 'False', then @x@ and @y@ are definitely not equal.
-- If it returns 'True', then the absolute value of their difference is less than @10^(-d)@ (but they may be non-equal).
(=?=) :: IReal -> IReal -> Precision -> Bool
(=?=) x y p = isThin dp && midI dp == 0
            where dp = appr (x-y) p

-- | Total, approximate inequality test. If @x <! y `atDecimals` d@ returns 'True', then @x@ is definitely smaller than @y@, 
-- If it returns 'False', @x@ may still be smaller than @y@, but their difference is then at most @10^(-d)@. 
(<!) :: IReal -> IReal -> Precision -> Bool
(<!) x y p = upperI dp <= 0
   where dp = appr (x-y) p
-- | @x >! y@ is the same as @y <! x@.
(>!) :: IReal -> IReal -> Precision -> Bool
x >! y = y <! x

-- Printing real numbers -------------------------------------------------------

showIReal :: Int -> IReal -> String 
showIReal d x = 
   case compare d 0 of
      EQ -> if isThin x0 then show (midI x0) else show (midI x0) ++ " +- " ++ show (radI x0)
        where x0 = appr x 0
      LT -> error "IReal.? : second argument negative"
      GT| l==u -> concat l
        | s==s1 && is==is1 -> s ++ is ++ take n fs ++ 
                              "[| " ++ take 10 (drop n fs) ++ " .. " ++ take 10 (drop n fs1) ++ " |]"
        | otherwise -> "[| " ++ s ++ is ++ tryInt (take 11 fs) ++ " .. " ++ s1 ++ is1 ++ tryInt(take 11 fs1) ++ " |]"
    where xd = appr (scale (x*fromInteger (5^d)) d) 0
          tryInt "" = ""
          tryInt fs =  if (all (=='0') (tail fs)) then "" else fs
          f m = [s, is, tryInt ('.':fs)]
            where
             ds = show m
             (s,ds') = if head ds == '-' then ("-",tail ds) else ("",ds)
             miss = d - length ds'  
             (is,fs) = if miss >= 0 then ("0",replicate miss '0' ++ ds') else splitAt (-miss) ds'
          l@[s,is,fs]    = f (lowerI xd+1) 
          u@[s1,is1,fs1] = f (upperI xd-1) 
          n = length (takeWhile id (zipWith (==) fs fs1))

-- | IReal is an instance of 'Show' but it should be avoided; see introduction. 
-- Use @x ? n@ to print @x@ with @n@ decimals.
instance Show IReal where
 showsPrec p x 
    | head xs == '-' = showParen (p > 6) (showString xs) 
    | otherwise      = showString xs
    where xs = showIReal digits x 
          digits = 30

-- | Prints an 'IReal' with given number of decimals. Rounding error is up to one unit in the last position.
(?) :: IReal -> Int -> IO ()
x ? d = putStrLn (showIReal d x)

-- | Prints an 'IReal' in scientific notation with given number of digits. Rounding error is up to one unit in the last position.
(??) :: IReal -> Int -> IO ()
x ?? d
  | d < 0 = error "IReal.??: second argument negative"
  | x < 0 = do putStr "-"
               (-x) ?? d
  | otherwise = do let z = midI (appr (logBase 10 x) 0)
                   case compare z 0 of
                      GT -> putStrLn (showIReal d (x/10^z) ++ "e" ++ show z)
                      EQ -> x ? d
                      LT -> putStrLn (showIReal d (x*10^(-z)) ++ "e" ++ show z)

-- | Forces evaluation of second argument to given number of decimals; returns second argument.
force :: Int -> IReal -> IReal
force d x = seq (appr x `atDecimals` d) x



-- Auxiliary functions and constants --------------------------------------------

-- terms p is N such that |sum_{N+1}^{\infty}| < 2^{-(p+3)}
-- c is max |f'| in interval
powerSeries ackFun c terms x = ir f 
   where f p
           |isThin xq    = fromInteger (scale (ss (midI xq)) (-s))
           |otherwise    = scale (min x1 x2 `upto` max x1 x2) (-s)           
          where nmax     = toInteger (terms p)
                s        = lg2 (max nmax c) + 4
                q        = p + s
                xq       = appr x q
                ss t     = sum (takeWhile (/=0) (scanl (g t) (bit q) [1..]))
                g m a n  = scale (ackFun (m * a) n) (-q)
                x1       = ss (lowerI xq)
                x2       = ss (upperI xq)

-- used for both log and atan; converges only for |x| <= 0.5
-- Monotonically increasing for 0 <= x < 0.5
g0 = powerSeries (\a n -> a * (shift n 1 - 1) `div` (shift n 1 + 1)) 2 id

ps sign n = ir f
  where f p = fromInteger (scale (sum (takeWhile (/=0) ts) `div` n) (-s))
         where s = lg2 (fromIntegral p) + 3
               ts  = zipWith div (iterate g (bit (p + s))) [1,3..]
               g x = sign (x `div` (n*n))

halfPi, quarterPi, log2 :: IReal
halfPi = scale pi (-1)
quarterPi = scale pi (-2)

log2 = 18 * at 26 - scale (at 4801) 1 + scale (at 8749) 3
    where at = ps id
