module MoreDigits where

import Data.Number.IReal
import Data.Number.IReal.IReal
import Data.Number.IReal.IntegerInterval
import LinAlg
import Newton
import Integrals
import Erf

{-

This file contains solutions to some of the problems from the MoreDigits friendly competition,
held at LORIA, Nancy, France, in July 2006. See http://rnc7.loria.fr/competition.html. 
We use the example parameter sets rather than the actual competition sets,since the former 
have published correct solutions we can check our results against.

Timings are from running ghci 7.8.3 on a 2.5 Ghz MacBook Pro.

Each of the 16 problems has a simpler and a harder version. The results with our package are as follows (see below for improved results):
- We can solve both versions of problems 2, 3, 6, 7, 8, 9, 12 and 14.
- We can solve the simpler versions of problems 1, 11, 13 and 16, but the harder versions are out of reach, for excessive memory or time requirements
- Problems 4 and 5 concern two special functions, the zeta and gamma functions. We have not implemented these and, consequently, cannot solve these problems.
- Problem 10 is a linear algebra problem requiring inverting a matrix (of size 80x80 for the simpler problem). Our simple-minded linear algebra module cannot invert matrices larger than ca 40x40.
- Problem 15 is an integral with a heavily oscillating integrand. We can get the correct result for the simpler problem using Clenshaw-Curtis quadrature, but with a shaky error analysis based on noting that the results for 512 and 1024 points agree to the required number of decimals, and thus the common result is probably correct. We do not consider that a satisfactory solution.

Addendum March 14, 2015: See file MoreDigitsRounded.hs for solutions to the harder versions of problems 10, 13, 15 and 16 using the new module Data.Number.IReal.Rounded.

-}

-- Auxiliary stream of pseudo-random integers used in several problems below
chi :: Integer -> [Integer]
chi s = tail (iterate f s)
  where f s = (69069 * s + 3) `mod` 2^31


p1 n a b c = exp (log (a/b)/c) ? n

{- 

p1 15000 123456 10000 1000
1.002516460709654427070669458074954176353693511207522646674439914525351574088850747808768859
...
90067298469713697479049799670230992994517233320216647880388126823764450957720301326122793745235802
(1.84 secs, 944380208 bytes)

We have no specialized implementation of nth roots, so the harder problem is far beyond reach. An implementation of nthRoot along the lines of sqrt would probaby handle this problem well.
-}

p2 n a b = exp (cos (a/b)) ? n

{-

p2 5000 6 7
1.924372742668343802802699839922863404917063482409596698496497263030972263567957417927207928
...
7064785022769449560946036292536382
(0.08 secs, 37847656 bytes)


p2 50000 2348 11
2.677658027419916799656791778743205279006831584447909394138911371168661727256596447351001419-
...
0028465996312865188159383181798097287773457362
(11.63 secs, 5915475144 bytes)

-}

p3 n a b c = acos (a/c) + asin (b/c) ? n

{-
p3 5000 2923 2813 3000
1.442910413467789456239386067859780011674672727154695225153220712260067164227074953642684611
...
8818481631797890652501951269818691
(0.27 secs, 187639840 bytes)

p3 40000 3922 813 4000
0.402482572546625665922717515202191441139775563729442129892515278209805324753500764115577856
...
53391820989810762240599096232837234036181276719563105086718718647336476260
(23.17 secs, 8288882608 bytes)

-}

{-

We do not attempt to solve p4 and p5 since we have not implemented the special functions zeta and gamma.

-}

p6 n a b = erf(sin(a/b)) ? n

{-

p6 30000 1512 1000
0.8419822444250350722500465556032049076535282226505613076319708107454641865199854581422396
...
97379590474161895111388647662559
(5.09 secs, 1769452000 bytes)

-}

p7 n s a = bsum (map (recip . fromIntegral) (take a (chi s))) ? n

{-

p7 15 12324 20000 
0.000089400791092
(0.29 secs, 121306240 bytes)

p7 15 12314 2000000
0.016451150554244
(29.21 secs, 11332463760 bytes)

Note that the latter is *not* the harder problem; that has 100 times more terms!! 

On the other hand, the required precision for the harder problem is only five significant digits, 
so we can actually work in type Double to solve the problem! Compiling and running 

main = print (sum (take 200000000 (map (recip . fromIntegral) (chi 12314))))

we get the output 1.9378131134366972 in ca 16 seconds.  (This seems a bit too much for ghci)
-}

p8 n s a b = bsum [prec (n + 10) $ abs (sin (fromInteger k/b)) | k <- take a (chi s) ] ? n

{-

p8 16 12344 5000 2
3178.8937977675151612
(1.17 secs, 604932260 bytes)

p8 150 12384 100000 10
63837.783124646381213503757555288914770713872985612438616744139740933585253689694540737815815402040378950007180540637644340700133080182949505340280227608713
(8.51 secs, 9719354792 bytes)

-}

p9 n a b = bsum [scale 1 (-k) | (k,x) <- zip [1..ceiling (logBase 2 10 * fromIntegral n)] (tail (iterate f 0.5)), x < (0.5 :: IReal)] ? n
   where f x =  prec 20000 (c*x*(1-x))
         c = a/b

{-
p9 100 161 43
0.2893154707647130135806131784714035265898339804693509562693961960506866365030222558951473454014980657
(0.30 secs, 68561352 bytes)

p9 7000 15 4
0.2893697060801703747946860518794294747419158436765556621286276182515676860828551
...
304596295681017400188911416039082717
(20.55 secs, 4806753488 bytes)

Remark: This is not entirely satisfactory. The successive values in the logistic sequence are narrow intervals (starting from width 10^(-20000)) and it is conceivable that one such interval includes 0.5, while the exact value would not. But since we got the correct answer, we were lucky... (But, of course, the probability for failure is negligible, if we believe that values are randomly distributed). 

But also the problem itself is dubious, since it involves the < relation, which is not computable. Of course, the organizers made sure that no x_n is exactly 0.5, but anyhow...
-}

p10 n s a = bsum (map (bsum . map abs) inv) 
  where cs = map fromInteger (chi s)
        mat = take a (group a cs)
        group a xs = take a xs : group a (drop a xs)
        inv = inverse mat

{- Can only use be used for n up to ca 40, i.e. not even close to solve the simple problem (n=80). -}

p11 :: IReal -> IReal -> Int -> (Integer,Int)
p11 a b c = (last as, length (filter (==1) as))
    where as = take (c+1) (cf (sqrt(sqrt(a/b))))
          cf x = r:cf (prec 10000 (recip s))
                 where (r,s) = properFraction x
  
{-

p11 82 13 5246
(10532,2132)
(2.67 secs, 610580736 bytes)

-}
      
p12 n s a = f 0 0 (chi s) ? n
  where f k y (c:cs)
          |k == a = y
          |otherwise = f (k+1) (prec 20 $ sin (y+fromInteger c)) cs

{-

p12 11 24905 1000
-0.95250313722
(0.29 secs, 207889240 bytes)

p12 11 14905 200000
-0.80338154922
(11.55 secs, 9981154672 bytes)

-}

p13 n s a b = bsum (zipWith3 (\a b c -> a*b/c) as bs cs) ?? (n+1)
  where as = take (n `div` 2) (map (\x -> fromInteger x - scale 1 30) (chi s))
        bs = tail (iterate (* sqrt (a/b)) 1)
        cs = map fromInteger (scanl (*) 1 [2..])

{-

p13 2000 102348 9999 1001
1.3148975627779447163107569531450272105470501991442659409013069882493811911013720
...
14096037221926209702149402897608454262629229624730552707920e10
(23.76 secs, 8757132408 bytes)
-}

p14 n a b = head (allZeros (n+1) f (0 -+- (val b*pi/4))) ? n 
  where f x = exp (-x/a) - tan (x/b)

{-

p14 1000 10 10
5.313908566521572046202664406047153136830749994680350179440416642864202502440058660714165198
...
1993742013577685970083189334559355797301908569071064365753611935955962832815667242
(0.06 secs, 43362384 bytes)

p14 20000 1234 4321
1372.607407915898039275620096163136526889749503325903743299297616743666337996084985896175294
...
89660209714061591467119432351273335633380
(14.95 secs, 5790678848 bytes)

-}

p15 n a b c = integral 8 n (\x -> sin (a * cos (b*x+c))) (0 +- 1) ?? n

{-

Above attempt can only solve the trivial problem, taking almost 3 seconds. The one below, from Clenshaw-Curtis, solves simple problem, but error analysis is non-trivial.
cctest can easily be modified to a solution that indicates that 512 pts are sufficient, but it's not a proof.
 
quad  (\x -> sin (a * cos (b*x+c))) (cpss!!9) (wss!!9) ?? 20
1.01356447296047236253e-2
(10.16 secs, 3480716500 bytes)

-}

p16 n a b c = deriv c f (recip (sqrt a)) ?? n
  where f x = sqrt (sin x + recip (sqrt b))

{-

p16 20 3210 5432 30
-1.19844645066450855152e74
(1.20 secs, 811510936 bytes)

-}
