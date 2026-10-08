{-# LANGUAGE DataKinds #-}

module MoreDigitsRounded where

import Data.Number.IReal.Rounded
import Data.Number.IReal (bsum,scale)
import Data.Number.IReal.FAD
import ClenshawRounded
import LinAlg


{-

This file contains solutions to the harder versions of problems 10, 13, 15 and 16 in the MoreDigits competition. See file MoreDigits.hs for the other problems. Probably some of these can be solved faster with module Data.Number.IReal.Rounded, but we haven't tried.

-}

chi :: Integer -> [Integer]
chi s = tail (iterate f s)
  where f s = (69069 * s + 3) `mod` 2^31

p10 n s a = bsum (map (bsum . map abs) inv) 
  where cs = map fromInteger (chi s)
        mat = take a (group a cs)
        group a xs = take a xs : group a (drop a xs)
        inv = inverse mat

{-

 (p10 10 12165 80 :: Rounded 90) ?? 10
0.7880687075e-6
(7.62 secs, 7416272616 bytes)

(p10 10 12385 250 :: Rounded 200) ?? 10
0.5297164525e-5
(282.02 secs, 252926826520 bytes)

-}

p13 n s a b = bsum (zipWith3 (\a b c -> a*b/c) as bs cs) 
  where as = take (n `div` 2) (map (\x -> fromInteger x - scale 1 30) (chi s))
        bs = tail (iterate (* sqrt (a/b)) 1)
        cs = map fromInteger (scanl (*) 1 [2..])

{-

(p13 2000 102348 9999 1001 :: Rounded 3000) ?? 1999
1.31489756277794471631075695314502721054705019914426594090130698824938119110137
...
096037221926209702149402897608454262629229624730552707920e10
(0.19 secs, 95604096 bytes)

(p13 20000 102317 999999 1001 :: Rounded 35000) ?? 20000
0.8321021150455468552974202017116086110
...
636210307261174886399423816332873222e22
(50.18 secs, 9929549512 bytes)
-}

p15 k a b c = quad  (\x -> sin (a * cos (b*x+c))) (cpss!!k) (wss!!k) 

{-
ghci MoreDigitsRounded
> import Data.Number.IReal.Rounded
:set +s
:set -XDataKinds
(p15 13 20 20 20 :: Rounded 265) ?? 254
0.71608688553056848008081092230917458779137963176367305213525499620635442269804039553481261557978602143549436015055422778396950304469931102251569240476886870118455258641437467302292796143016953525336574898319125551548729281893014937880076135276345390154853e-2
(21.49 secs, 29031071328 bytes)

The above solves the harder problem using Clenshaw-Curtis quadrature in 2^13+1 points. The correctness has to be argued independently (using well-known error estimates for analytic integrands or, less stringent, by noting the results for 12 and 13 points and relying on geometric convergence.

-}

p16 a b c = deriv c f (recip (sqrt a))
  where f x = sqrt (sin x + recip (sqrt b))

{-

 (p16 4321 543210 200 :: Rounded 25) ?? 20 
-1.39326746432557879171e726
(0.85 secs, 828010472 bytes)
-}
