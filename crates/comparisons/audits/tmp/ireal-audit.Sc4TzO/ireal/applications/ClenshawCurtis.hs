module ClenshawCurtis where

import FFT
import Data.Number.IReal
import Text.Printf
import Data.Bits
import ListNumSyntax

-- Chebyshev points
chebpts ::  Floating a => Int -> [a]
chebpts n = [sin (fromIntegral k*(pi/fromIntegral (2*n'))) | k <- [n',n'-2 .. -n']]
  where n' = bit n :: Int

-- Clenshaw-Curtis weights following Waldvogel
weights :: (VarPrec a, RealFloat a) => Int -> [a]
weights 0 = [2]
weights n = w/2 : init ws ++ rs
   where w : ws = dct 1100 (map f [0..n'])
         rs = reverse (w/2 : ws)
         n' = bit (n-1) :: Int
         f k = 2/fromIntegral n'/(1 - 4 * fromIntegral k^2)

cpss :: Floating a => [[a]]
cpss = [chebpts n | n <- [0..]]

wss :: (VarPrec a, RealFloat a) => [[a]]
wss = [weights n | n <- [0..]]

-- General quadrature formula
quad f xs ws = bsum (map f xs * ws)

cctest f = do let is = zipWith (quad f) cpss wss
                  diffs = is - tail is
                  ns = map (+1) (iterate (*2) 1)
                  row n d = do printf "%5d   " (n :: Integer)
                               d ?? 10
              printf "%5s   %s\n" "n" "cc(n) - cc(2*n-1)" 
              sequence_ (take 9 (zipWith row ns diffs))

{- 
cctest f computes Clenshaw-Curtis quadrature of f for 2, 3, 5, 9 .. 257, 513 points and
tabulates the differences between successive values with ten significant digits.

Example:
> let f x = 1/sqrt pi*exp(-x^2)
> cctest f
    n   cc(n) - cc(2*n-1)
    2   -0.4755144465
    3   0.4658494488e-1
    5   1.3360200833e-3
    9   0.1859662191e-6
   17   0.2786578649e-13
   33   0.5387431708e-28
   65   0.6451787408e-61
  129   0.4623338430e-135
  257   0.2823741182e-301
(2.42 secs, 1912637760 bytes)
This indicates geometric convergence. We can use this (or, of course, well-known error bounds 
for Clenshaw-Curtis with  analytic integrands) to argue that 128 points are enough to compute the integral 
of f over [-1,1] to 100 decimals:

> quad f (cpss!!7) (wss!!7)?100
0.8427007929497148693412206350826092592960669979663029084599378978347172540960108412619833253481448885
(0.72 secs, 649544664 bytes)

As another example, we can see the slow convergence when a low-order derivative of the integrand is
discontinuous:

> cctest ((^3) . abs . (+0.5))
    n   cc(n) - cc(2*n-1)
    2   0.5416666667e1
    3   0.5719095842e-1
    5   -0.5061390681e-2
    9   -0.3745316495e-4
   17   -0.8422334880e-5
   33   -0.3307784812e-6
   65   -0.2654695584e-7
  129   -1.4743605157e-9
  257   -0.9789596620e-10
(0.48 secs, 239041584 bytes)

-}

