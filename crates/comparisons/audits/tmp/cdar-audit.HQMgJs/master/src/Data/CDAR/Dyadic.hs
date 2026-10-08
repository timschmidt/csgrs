{- |The Dyadic module provides dyadic numbers. Syntax for dyadic number
   'a * 2 ^ s' is 'a :^ s'. The exponent 's' is an 'Int', but the 'a' is an
   arbitrary 'Integer'.
-}
module Data.CDAR.Dyadic (Dyadic(..),shiftD,sqrtD,sqrtD',sqrtRecD,sqrtRecD',initSqrtRecD,initSqrtRecDoubleD,piMachinD,piBorweinD,ln2D,logD,atanhD,divD,divD') where

import Data.Ratio
import Data.Bits
import Data.CDAR.Classes
import Data.CDAR.IntegerLog

-- Dyadic numbers

infix 8 :^

-- |The Dyadic data type.
data Dyadic = Integer :^ Int deriving (Read,Show)

instance Eq Dyadic where
    a :^ s == b :^ t | s <= t    = a == unsafeShiftL b (t-s)
                     | otherwise = unsafeShiftL a (s-t) == b

instance Ord Dyadic where
    compare (a :^ s) (b :^ t) | s <= t    = compare a (unsafeShiftL b (t-s))
                              | otherwise = compare (unsafeShiftL a (s-t)) b

instance Scalable Dyadic where
  scale (a :^ s) n = a :^ (s+n)

instance Num Dyadic where
    a :^ s + b :^ t 
        | s <= t    = (a + scale b (t-s)) :^ s
        | otherwise = (scale a (s-t) + b) :^ t
    a :^ s * b :^ t = (a * b) :^ (s+t)
    negate (a :^ s) = (negate a) :^ s
    abs (a :^ s) = (abs a) :^ s
    signum (a :^ _) = (signum a):^ 0
    fromInteger i = i :^ 0

instance Real Dyadic where
    toRational (a :^ s) = (toRational a)*2^^s

-- | Shift a dyadic number to a given base and round in case of right shifts.
shiftD :: Int -> Dyadic -> Dyadic
shiftD t (m:^s) =
    if t <= s
    then unsafeShiftL m (s-t) :^ t
    else unsafeShiftR (m + bit (t-s-1)) (t-s) :^ t

-- Square root

divD :: Dyadic -> Dyadic -> Dyadic
divD a (n:^t) = let (m:^_) = shiftD (2*t) a
                 in round (m % n) :^ t

{- |Computes the square root of a Dyadic number to the specified base. The
   Newton-Raphson method may overestimates the square root, but the
   overestimate is bounded by 1 ulp. For example, sqrtD 0 2 will give 2,
   whereas the closest integer to the square root is 1. Need double precision
   to guarantee correct rounding, which was not considered worthwhile.

   This is actually Heron's method and is no longer used in Approx as it is
   faster to use sqrtRecD.
-}
sqrtD :: Int -> Dyadic -> Dyadic
sqrtD t x = sqrtD' t x $ initSqrtD x
    where
      -- Initial approximation of square root to about 3 bits
      initSqrtD :: Dyadic -> Dyadic
      initSqrtD (0:^_) = (0:^0)
      initSqrtD (m:^s) = let i = integerLog2 m
                             n = shift m (2-i)
                             s' = (s+i) `div` 2 - 3
                         in if odd (s+i)
                            then (n+8):^s'
                            else (n+4):^s'

-- |Square root with initial approximation as third argument.
sqrtD' :: Int -> Dyadic -> Dyadic -> Dyadic
sqrtD' t x@(m:^_) y
    | m == 0 = 0:^0
    | m > 0  = converge . iterate (newtonStep x t) $ shiftD t y
    | otherwise = error "Attempting sqrt of negative dyadic number."
    where
      -- One step of Newton iteration to find square root.
      -- The base of a need to be the same as t', and the result will have base t'.
      newtonStep :: Dyadic -> Int -> Dyadic -> Dyadic
      newtonStep d t' a = shiftD t' $ (1:^(-1)) * (a + divD d a)

-- |Reciprocal of square root using Newton's iteration.
sqrtRecD :: Int -> Dyadic -> Dyadic
sqrtRecD t a = sqrtRecD' t a $ initSqrtRecDoubleD a

-- |Gives initial values for the iteration. Based on the three most
-- significant bits of the argument. Should consider to use machine floating
-- point to give initial value.
initSqrtRecD :: Dyadic -> Dyadic
initSqrtRecD (0 :^ _) = error "Divide by zero in initSqrtRecD"
initSqrtRecD (m :^ s) =
  let i = integerLog2 m
      n = shift m (3-i)
      s' = (-i-s-1) `div` 2 - 5
  in if even (s+i)
     then ([62,59,56,53,51,49,48,46]!!(fromIntegral n-8)) :^ s'
     else ([44,42,40,38,36,35,34,33]!!(fromIntegral n-8)) :^ s'

-- Using Double to find a good first approximation to SqrtRec. Extracts 52
-- bits from the dyadic number and encodes a float in the range [1/2,2).
-- Decodes the result of the Double computation as a dyadic taking care of the
-- exponent explicitly (as the range of Double exponents is too small).
initSqrtRecDoubleD :: Dyadic -> Dyadic
initSqrtRecDoubleD (0 :^ _) = error "Divide by zero in initSqrtRecDoubleD"
initSqrtRecDoubleD (m :^ s) =
  let i = integerLog2 m
      n = shift m (52-i)
      s' = (-i-s) `div` 2 - 52
      (m',_) = decodeFloat . (1/) . sqrt $ encodeFloat n (if even (s+i) then (-52) else (-51))
      t = if m' /= bit 52 && even (s+i) then s'-1 else s'
  in m' :^ t

-- |Reciprocal of square root using Newton's iteration with inital value as third argument.
sqrtRecD' :: Int -> Dyadic -> Dyadic -> Dyadic
sqrtRecD' t a x0 =
  let step x = shiftD t $ x + shiftD t (x * (1 - shiftD t (x * x * a)) * (1 :^ (-1)))
      xs = iterate step x0
  in converge xs

-- isqrt :: Integral t => t -> t
-- isqrt 0 = 0
-- isqrt 1 = 1
-- isqrt n = head $ dropWhile (\x -> x*x > n) $ iterate (\x -> (x + n `div` x) `div` 2) (n `div` 2)

-- Assuming a seqence of dyadic numbers with the same exponent converges
-- quadratically. Then this returns the first element where it is known that
-- the sequence will become essentially constant (differing by at most 1). If
-- the numbers contain sufficiently many bits we need only wait until the
-- difference has less than half as many bits (with a small margin).
converge :: [Dyadic] -> Dyadic
converge ((0:^s):_) = (0:^s)
converge ((m:^_):ds@(d@(n:^_):_)) =
  let a = integerLog2 m
      b = integerLog2 (abs (m - n))
  in if a <= 6
     then if abs (m-n) <= 1 then d else converge ds
     else if 2 * b <= a - 3 then d else converge ds
converge _ = error "List terminating in converge."

divD' :: Int -> Dyadic -> Dyadic -> Dyadic
divD' p a b = let (m:^_) = shiftD (2*p) a
                  (n:^_) = shiftD p b
              in round (m%n) :^ p

-- |Compute dyadic values close to pi by Machin's formula.
piMachinD :: Int -> Dyadic
piMachinD t = let t' = t-10-integerLog2 (fromIntegral (abs t))
                  a = map ((:^t') . round . (bit (-t') %)) $ iterate ((-25)*) (5 :: Integer)
                  b = map ((:^t') . round . (bit (-t') %)) $ iterate ((-57121)*) (239 :: Integer)
                  c = map ((:^t') . round . (bit (-t') %)) ([1,3..] :: [Integer])
                  d = takeWhile (/= 0) . map (shiftD t') $ zipWith (*) a c
                  e = takeWhile (/= 0) . map (shiftD t') $ zipWith (*) b c
              in shiftD t $ 4 * (4 * sum d - sum e)

-- |Compute dyadic values close to pi by Borwein's formula.
piBorweinD :: Int -> Dyadic
piBorweinD t = let t' = t-10-integerLog2 (fromIntegral (abs t))
                   s = sqrtD t' 2
                   d = head . dropWhile (\(_,y,_) -> y /= 0) $ iterate (f t') (6-4*s,s-1,1 :: Int)
               in shiftD t . (\(a,_,_) -> divD' t' 1 a) $ d
    where
      f :: Int -> (Dyadic, Dyadic, Int) -> (Dyadic, Dyadic, Int)
      f l (a,y,k) = let u = sqrtD l . sqrtD l $ 1-y^(4::Int)
                        y' = shiftD l $ divD (1-u) (1+u)
                        a' = shiftD l $ a*(1+y')^(4::Int)-(bit (2*k+1):^0)*y'*(1+y'+y'*y')
                    in (a',y',k+1)

-- |Compute dyadic values close to ln 2.
ln2D :: Int -> Dyadic
ln2D t = let t' = t - 10 - 2 * integerLog2 (fromIntegral (abs t))
             a = map ((:^t') . round . (bit (-t') %)) $ iterate (3*) (3 :: Integer)
             b = map ((:^t') . round . (bit (-t') %)) $ iterate (4*) (4 :: Integer)
             c = map ((:^t') . round . (bit (-t') %)) ([1..] :: [Integer])
             d = zipWith (+) a b
             e = takeWhile (/= 0) . map (shiftD t') $ zipWith (*) d c
         in shiftD t $ sum e

logD :: Int -> Dyadic -> Dyadic
logD t x@(a :^ s) =
  if a <= 0 then error "logD: Non-positive argument"
  else let t' = t - 10 - max 0 (integerLog2 a + s) -- (5 + size of argument) guard digits
           r = s + integerLog2 (3*a) - 1
           x' = scale x (-r) -- 2/3 <= x' <= 4/3
           y = divD' t' (x' - 1) (x' + 1) -- so |y| <= 1/5
           z = fromIntegral r * ln2D t'
       in  shiftD t . (+z) . flip scale 1 $ atanhD t' y

atanhD :: Int -> Dyadic -> Dyadic
atanhD t x@(a :^ s) =
  if integerLog2 (abs a) + s >= 0 then error "atanhD: Argument outside domain, (-1,1)"
  else let 
    -- number of guard digits is 5+k where k depends on the precision [how do we know if this is enough?]
    t' = t - 5 - integerLog2 (abs $ fromIntegral t) `div` 2
    g _x _y = shiftD t' (_x * _y)
    x2 = g x x
    b = iterate (g x2) 1
    c = (map (divD' t' 1 . fromIntegral) [1,3..])
    in shiftD t . g x . sum . takeWhile (/= 0) $ zipWith g b c

{-
agmD :: Int -> Dyadic -> Dyadic -> Dyadic
agmD t a b = let t' = t - 5
                 agmStep (c,d) = ((1:^(-1)) * (c+d), sqrtD t' (c*d))
                 close (c,d) = abs (c-d) < 1:^t
             in fst . last . takeWhile (not . close) $ iterate agmStep (shiftD t' a, shiftD t' b)

theta2D :: Int -> Dyadic -> Dyadic
theta2D t q = let q2 = shiftD t $ q*q
                  q4 = sqrtD t . sqrtD t $ q
                  a = iterate (shiftD t . (q2*)) 1
                  b = scanl1 (\c d -> shiftD t (c*d)) a
              in (2*) . shiftD t . (q4*) . sum $ takeWhile (/= 0) b

theta3D :: Int -> Dyadic -> Dyadic
theta3D t q = let q2 = shiftD t $ q*q
                  -- q4 = sqrtD t . sqrtD t $ q
                  a = iterate (shiftD t . (q2*)) q
                  b = scanl1 (\c d -> shiftD t (c*d)) a
              in (1+) . (2*) . shiftD t . sum $ takeWhile (/= 0) b

lnBigD :: Int -> Dyadic -> Dyadic
lnBigD t x = let t' = t-10-integerLog2 (fromIntegral (abs t))
                 t2 = theta2D t' x
                 t3 = theta3D t' x
                 a = agmD t' (t2*t2) (t3*t3)
             in shiftD t $ divD' t' (piBorweinD t') a
-}
{-
agm a b = let step (a,b) = (0.5 * (a+b), sqrt (a*b))
              close (a,b) = abs (a-b) < 1e-6
          in fst . last . takeWhile (not . close) $ iterate step (a, b)

theta2 q = let q2 = q^2
               q4 = sqrt . sqrt $ q
               a = iterate (q2*) 1
               b = scanl1 (*) a
           in (2*) . (q4*) . sum $ takeWhile (> 1e-6) b

theta3 q = let q2 = q^2
               q4 = sqrt . sqrt $ q
               a = iterate (q2*) q
               b = scanl1 (*) a
           in (1+) . (2*) . sum $ takeWhile (> 1e-6) b
-}

{-
-- checking that converging dyadic numbers only differ in last bit
check ((1:^_):(1:^_):_) = False
check ((1:^_):xs) = check xs
check ((-1:^_):(-1:^_):_) = False
check ((-1:^_):xs) = check xs
check ((0:^_):xs) = check xs
check ((_:^_):_) = False
check [] = True
check [_] = True
-}
