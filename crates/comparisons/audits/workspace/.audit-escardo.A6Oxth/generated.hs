{-# LANGUAGE NPlusKPatterns #-}
{-# OPTIONS_GHC -Wno-x-partial #-}

import System.IO

main = do
        hSetBuffering stdout NoBuffering
        putStr ((show (take 500 example4)) ++ "\n")

type Digit = Int

type I  = [Digit]

one, zero, minusOne, half,  minusHalf :: I
minusOne  = repeat (-1)
minusHalf = -1 : repeat 0
zero      = repeat 0
half      = 1 : zero
one       = repeat 1

type Digit2  = Int

type Digit4  = Int

type Digitn = Int

type I2 = [Digit2]
type I4 = [Digit4]
type In = [Digitn]

divideBy :: Int -> In -> I
divideBy n (0:x) = 0 : divideBy n x -- Added 5 Feb 2015. Makes everything way faster.
divideBy n (a:x) | abs a == n
  = (if a < 0 then -1 else 1) : divideBy n x
divideBy n (a:b:x) =
  let d = 2 * a + b
  in if      d < -n then -1 : divideBy n (d+2*n:x)
     else if d >  n then  1 : divideBy n (d-2*n:x)
                    else  0 : divideBy n (d:x)

divideBy2 :: I2 -> I -- Added 5 Feb 2015 again to make things faster.
divideBy2 ( 0:x) =  0 : divideBy2 x
divideBy2 ( 2:x) =  1 : divideBy2 x
divideBy2 (-2:x) = -1 : divideBy2 x
divideBy2 (a:b:x) =
  let d = 2*a+b
  in if      d < -2 then -1 : divideBy2 (d+4:x)
     else if d >  2 then  1 : divideBy2 (d-4:x)
                    else  0 : divideBy2 (d:x)

add2 :: I -> I -> I2
add2 = zipWith (+)

mid :: I -> I -> I
mid x y = divideBy2 (add2 x y)

compl :: I -> I
compl = map (\d -> -d)

divByInt :: I -> Int -> I
divByInt x n = f x 0
  where
    f (a:x) s = let t = a + 2 * s
                in if t >=  n then  1 : f x (t - n)
              else if t <= -n then -1 : f x (t + n)
                              else  0 : f x t

bigMid :: [I] -> I
bigMid = divideBy 4 . bigMid'
 where bigMid'((a:b:x):(c:y):zs) = 2*a + b + c : bigMid'((mid x y):zs)

addOne :: I -> I
addOne ( 1 : x) = one
addOne ( 0 : x) = 1 : addOne x
addOne (-1 : x) = 1 : x

subOne :: I -> I
subOne ( 1 : x) = -1 : x
subOne ( 0 : x) = -1 : subOne x
subOne (-1 : x) = minusOne

oneMinus :: I -> I
oneMinus = addOne.compl

mulBy2 :: I -> I
mulBy2 ( 1 : x)  = addOne x
mulBy2 ( 0 : x)  = x
mulBy2 (-1 : x)  = subOne x

mulBy4 :: I -> I
mulBy4 = mulBy2.mulBy2

tMulByInt :: I -> Int -> I
tMulByInt x 0 = zero
tMulByInt x 1 = x
tMulByInt x n = if even n
                then mulBy2(tMulByInt x (n `div` 2))
                else tadd x (mulBy2(tMulByInt x (n `div` 2)))

tadd :: I -> I -> I
tadd x y = mulBy2(mid x y)

piDividedBy32 :: I
piDividedBy32 = bigMid [f k (mid (mid (g1 k) (g2 k))(mid (g3 k) (g4 k))) | k <- [0..]]
 where
   f k x = if k == 0 then x else 0:0:0: f (k-1) x
   g1 k = divByInt (repeat  1)      (8*k+1)
   g2 k = divByInt (-1 : zero)      (8*k+4)
   g3 k = divByInt ( 0 : -1 : zero) (8*k+5)
   g4 k = divByInt ( 0 : -1 : zero) (8*k+6)

example1 = piDividedBy32

toDouble :: I -> Double
toDouble = f 55
  where
    f 0 x = 0.0
    f k (-1 : x) = (-1.0 + f (k-1) x)/2.0
    f k ( 0 : x) = (       f (k-1) x)/2.0
    f k ( 1 : x) = ( 1.0 + f (k-1) x)/2.0

fromDouble :: Double -> I
fromDouble = f 55
  where
    f 0 x   = zero
    f k x   = if x  < 0.0 then -1 : f (k-1) (2.0 * x + 1)
                          else  1 : f (k-1) (2.0 * x - 1)

example2 = 32 * toDouble piDividedBy32
example3 = example2 - pi

mulByInt :: I -> Int -> (Int,I)
mulByInt x n = f n
  where
    f 1 = (0, x)
    f n = let (a,u) = f (n `div` 2)
              d:y = u
              b = 2*a+d
          in if even n
             then (b,y)
             else let e:t = (mid x y)
                  in (b+e,t)

type Decimal = [Int]

signed2Decimal :: I -> Decimal
signed2Decimal x = let (d,y) = mulByInt x 10
                   in d : signed2Decimal y

normalize :: Decimal -> Decimal
normalize x = f x
  where
    f(d:x) = if wpositive x
             then d:f x
             else (d-1): g x
    g(0:x) = 9: g(x)
    g(d:x) = if wpositive x
             then (10+d) : f x
             else (10+d-1) : g x
    wpositive (d:x) =
             if d > 0 then True
        else if d < 0 then False
                      else wpositive x

decimal :: I -> Decimal
decimal = normalize.signed2Decimal

decimalString :: I -> String
decimalString = concat . map show . decimal

example4 = let (m,x) = mulByInt piDividedBy32 32
           in show m ++ "." ++ decimalString x

mul :: I -> I -> I
mul = mul_version2

digitMul :: Digit -> I -> I
digitMul (-1) x = compl x
digitMul   0  x = zero
digitMul   1  x = x

mul_version0 :: I -> I -> I
mul_version0 x y = bigMid (zipWith digitMul x (repeat y))

mul_version1 :: I -> I -> I
mul_version1 (a0 : a1 : x) (b0 : b1 : y) = mid p q
  where
    p  = mid p' p''
    p' = (a0*b1): mid (digitMul b1 x) (digitMul a1 y)
    p''= mid (digitMul b0 x) (digitMul a0 y)
    q = (a0*b0) : (a1*b0) : (a1*b1) : mul_version1 x y

mul_version2 :: I -> I -> I
mul_version2 (0:x) y = 0 : mul_version2 x y
mul_version2 x (0:y) = 0 : mul_version2 x y
mul_version2 (a0 : 0 : x) (b0 : 0 : y) = mid p q
  where
    p  = 0 : mid (digitMul b0 x) (digitMul a0 y)
    q = (a0*b0) : 0 : 0 : mul_version2 x y
mul_version2 (a0 : 0 : x) (b0 : 1 : y) = mid p q
  where
   p  = mid p' p''
   p' = 0 : 0 : x
   p''= mid (digitMul b0 x) (digitMul a0 y)
   q = (a0*b0) : 0 : 0 : mul_version2 x y
mul_version2 (a0 : 0 : x) (b0 : b1 : y) = mid p q
  where
    p  = mid p' p''
    p' = (a0*b1): 0 : digitMul b1 x
    p''= mid (digitMul b0 x) (digitMul a0 y)
    q = (a0*b0) : 0 : 0 : mul_version2 x y
mul_version2 (a0 : a1 : x) (b0 : 0 : y) = mid p q
  where
    p  = mid p' p''
    p' = 0 : 0 : digitMul a1 y
    p''= mid (digitMul b0 x) (digitMul a0 y)
    q = (a0*b0) : (a1*b0) : 0 : mul_version2 x y
mul_version2 (a0 : a1 : x) (b0 : b1 : y) = mid p q
  where
    p  = mid p' p''
    p' = (a0*b1): mid (digitMul b1 x) (digitMul a1 y)
    p''= mid (digitMul b0 x) (digitMul a0 y)
    q = (a0*b0) : (a1*b0) : (a1*b1) : mul_version2 x y

sqr :: I -> I
sqr (0:x) = 0 : 0 : sqr x
sqr (a0 : 0 : x) = mid p q
  where
   p  = 0 : digitMul a0 x
   q = (a0*a0) : 0 : 0 : sqr x
sqr (a0 : a1 : x) = mid p q
  where
    p  = mid p' p''
    p' = (a0*a1): digitMul a1 x
    p''= digitMul a0 x
    q  = (a0*a0) : (a1*a0) : (a1*a1) : sqr x

logistic, logistic' :: I -> I

logistic  x = mulBy4 (mul x (oneMinus x))

logistic' x = oneMinus (sqr(g x))   -- 1-(2x-1)^2
  where
    g ( 1 : x) = x                  -- g(x)= max(-1,2x-1)
    g ( 0 : x) = subOne x
    g (-1 : x) = minusOne

x0 = 0.671875

d0 :: I
d0 = [1,0,1,0,1,1] ++ zero

logistics = map toDouble (iterate logistic' d0)

dlogistic' :: Double -> Double
dlogistic' x = 1.0-(2.0 * x - 1.0)^2

logisticsDouble = iterate dlogistic' x0

logisticsError = zipWith (-) logistics logisticsDouble

example5 = logisticsError

mexp :: I -> I
mexp x = bigMid (series one 1)
  where series y n = y : series (divByInt (mul x y) n) (n+1)

msin :: I -> I
msin x = bigMid (series x 2)
  where
    x2 = compl(sqr x)
    series y n = zero : y : series(divByInt(mul x2 y)(n*(n+1)))(n+2)

mcos :: I -> I
mcos x = bigMid (series one 1)
  where
    x2 = compl(sqr x)
    series y n = y : zero : series(divByInt(mul x2 y)(n*(n+1)))(n+2)

marctan :: I -> I
marctan x = bigMid (series x 1)
  where
    x2 = compl(sqr x)
    series y n = zero : divByInt y n : series (mul x2 y) (n+2)

piDividedBy4 :: I
piDividedBy4 = let inverse n = divByInt one n
                   arctan = mulBy2.marctan.mulBy2
                   y1 = tMulByInt (arctan (inverse 49)) 12
                   y2 = tMulByInt (arctan (inverse 57)) 32
                   y3 = compl(tMulByInt (arctan (inverse 239)) 5)
                   y4 = tMulByInt (arctan (inverse 110443)) 12
               in tadd (tadd y1 y2) (tadd y3 y4)

example13 = let (m,x) = mulByInt piDividedBy4 4
            in show m ++ "." ++ decimalString x

marcsin :: I -> I
marcsin x = bigMid (series x 1)
  where
    x2 = sqr x
    series y n = zero : divByInt y n :
                 series (tMulByInt (divByInt (mul x2 y) (n+1)) n) (n+2)

mlni :: I -> I

mlni x = bigMid (series one 1)
  where
    x2 = compl x
    series y n = divByInt y n : series (mul x2 y) (n+1)

mln :: I -> I
mln x = mul x (mlni x)

inv :: I -> I
inv x = bigMid (series one)
  where series y = y : series (mul x y)

affine :: I -> I -> I -> I
affine a b x = bigMid (map h x)
  where
   h (-1) = a
   h   0  = mid a b
   h   1  = b

mul_version3 :: I -> I -> I
mul_version3 y = affine (compl y) y

complAffine :: I -> I
complAffine = affine one minusOne

idAffine :: I -> I
idAffine = affine minusOne one

findI :: (I -> Bool) -> I
findI p = if p left then left else right
 where
  left  = -1 : findI(\x -> p(-1:x))
  right =  1 : findI(\x -> p( 1:x))

forEveryI, forSomeI :: (I -> Bool) -> Bool
forSomeI p = p(findI p)
forEveryI p = not(forSomeI(not.p))

findI' :: (I -> Bool) -> I

findI' p | p(left)   = left
         | p(centre) = centre
         | otherwise = right
 where
  left   = -1 : findI'(\x -> p(-1:x))
  centre =  0 : findI'(\x -> p( 0:x))
  right  =  1 : findI'(\x -> p( 1:x))

forEveryI', forSomeI' :: (I -> Bool) -> Bool
forSomeI' p = p(findI' p)
forEveryI' p = not(forSomeI'(not.p))

modulus :: (I -> I) -> (Int -> Int)
modulus f 0 = 0
modulus f n = if forEveryI(\x -> head(f x) == head (f zero))
              then modulus (tail.f) (n-1)
              else 1 + max (modulus (f.((-1):)) n) (modulus (f.(1:)) n)

example11 = modulus sqr 2

supremum :: (I -> I) -> I

supremum f =
 let h = head(f zero)
 in if forEveryI(\x -> head(f x) == h)
    then h : supremum(tail.f)
    else imax (supremum(f.((-1):))) (supremum(f.(1:)))

infimum :: (I -> I) -> I
infimum f =
  let h = head(f zero)
 in if forEveryI(\x -> head(f x) == h)
    then h : infimum(tail.f)
    else imin (infimum(f.((-1):))) (infimum(f.(1:)))

example12 = supremum (\x -> mid (0:x) (sqr x))

imin :: I -> I -> I
imin ( a : x) ( b : y) | a == b = a : imin x y
imin (-1 : x) ( 1 : y) = -1 : x
imin ( 1 : x) (-1 : y) = -1 : y
imin (-1 : x) ( 0 : y) = -1 : imin           x     (oneMinus y)
imin ( 0 : x) (-1 : y) = -1 : imin (oneMinus x)              y
imin ( 1 : x) ( 0 : y) =  0 : imin (addOne   x)              y
imin ( 0 : x) ( 1 : y) =  0 : imin           x       (addOne y)

imax :: I -> I -> I
imax x y = compl (imin (compl x) (compl y))

iabs :: I -> I
iabs x = imax (compl x) x

average :: [I] -> [I] -> [I]
average = zipWith mid

halfIntegral' :: (I -> I) -> [I]
halfIntegral' f =
 let h = head(f zero)
 in if forEveryI(\x -> head(f x) == h)
    then (repeat h) : halfIntegral'(tail.f)
    else average (halfIntegral'(f.((-1):))) (halfIntegral'(f.(1:)))

halfIntegral :: (I -> I) -> I
halfIntegral f = bigMid (halfIntegral' f)

example10 = halfIntegral iabs

znorm :: I -> I
znorm (0:x)      = 0:znorm x
znorm (-1:  1:x) = 0:znorm (-1:x)
znorm ( 1: -1:x) = 0:znorm ( 1:x)
znorm x          = x

negative :: I -> Bool
negative x = f(znorm x)
 where
  f( 0 : x) = f x
  f(-1 : x) = True
  f( 1 : x) = False

smaller :: I -> I -> Bool
smaller x y = negative(mid x (compl y))

bisection :: (I -> I) -> I
bisection f =
 if negative(f zero)
 then  1 : bisection(f.( 1:))
 else -1 : bisection(f.(-1:))

ztrichot :: I -> I -> Bool
ztrichot x y = f (znorm x) (znorm y)
 where
  f (0  : x) (0 : y) = f x y
  f (-1 : x)      y  = True
  f       x       y  = False

trisection :: (I -> I) -> I
trisection f =
 let l = f minusHalf
     c = f zero
     r = f half
 in if (ztrichot c r)
    then 1 : trisection(f.(1:))       -- f(0) < 0,      so root in [0,1]
    else if (ztrichot l c)            -- 0 < f(1/2),    so root in [-1,  1/2]
         then  0: trisection(f.( 0:)) --    f(-1/2) < 0,so root in [-1/2,1/2]
         else -1: trisection(f.(-1:)) --    0 < f(0),   so root in [-1,0]

inverse :: (I -> I) -> (I -> I)
inverse f y = trisection(\x -> mid (f x) (compl y))

squareRoot :: I -> I
squareRoot = inverse sqr

example6 = toDouble(squareRoot half)

example7 = trisection f
 where
  tiny n = if n == 0 then one else 0 : tiny(n-1)
  epsilon = tiny (10^3)
  f = affine (compl epsilon) epsilon

trisectionInterval :: I -> I -> (I -> I) -> I
trisectionInterval a b f = g(trisection (f.g))
  where g = affine a b

trisection01 :: (I -> I) -> I
trisection01 f = 1 : trisection(f.(1:))

zppif :: I -> I -> I
zppif x y = c (znorm x) (znorm y)
  where
    c (0:x) (0:y) = 0 : c x y
    c (0:x)    y  = c x y
    c (-1:x)   _  = zero
    c ( 1:x)   y  = y

ppif :: I -> I -> I -> I -> I
ppif x y u v =
   mulBy4(mid (0:u) (zppif (mid x (compl y)) (mid (compl u) v)))

ppifz :: I -> I -> I -> I
ppifz x u v = mulBy4(mid (0:u) (zppif (0:x) (mid (compl u) v)))

pabs :: I -> I
pabs x = ppifz x (compl x) x

pmax :: I -> I -> I
pmax x y = ppif x y y x

buggyMul (0:x) y = 0 : buggyMul x y
buggyMul x (0:y) = 0 : buggyMul x y
buggyMul (a0 : 0 : x) (b0 : 0 : y) = mid p q
  where
    p  = 0 : mid (digitMul b0 x) (digitMul a0 y)
    q = (a0*b0) : 0 : 0 : buggyMul x y
buggyMul (a0 : 0 : x) (b0 : 1 : y) = mid p q
  where
    p  = mid p' p''
    p' = 0 : 0 : x
    p''= mid (digitMul b0 x) (digitMul a0 y)
    q = (a0*b0) : 0 : 0 : buggyMul x y
buggyMul (a0 : 0 : x) (b0 : b1 : y) = mid p q
  where
    p  = mid p' p''
    p' = (a0*b1): 0 : digitMul b1 x
    p''= mid (digitMul b0 x) (digitMul a0 y)
    q = (a0*b0) : 0 : 0 : buggyMul x y
buggyMul (a0 : a1 : x) (b0 : 0 : y) = mid p q
  where
    p  = mid p' p''
    p' = 0 : 0 : digitMul a1 y
    p''= mid (digitMul b0 x) (digitMul a0 y)
    q = (a0*b0) : (a1*b0) : 0 : buggyMul x y
buggyMul (a0 : a1 : x) (b0 : b1 : y) = mid p q
  where
    p  = mid p' p''
    p' = (a0*b1): mid (digitMul b1 x) (digitMul a1 y)
    p''= mid (digitMul b0 x) (digitMul a0 y)
    q  = (a0*b0) : (a1*b0) : (a1*a1) : buggyMul x y

close :: Int -> I -> I -> Bool
close n x y = closez n (mid x (compl y))

closez :: Int -> I -> Bool
closez n x = all (==0) (take n (znorm x))

example8 = forEveryI(\x -> forEveryI(\y -> close 4 (mul x y) (buggyMul x y)))

example9 = (take 5 x, take 5 y)
 where
  x = findI(\x -> forSomeI(\y -> not(close 4 (mul x y) (buggyMul x y))))
  y = findI(\y -> not(close 4 (mul x y) (buggyMul x y)))

type R = (I,Integer)

