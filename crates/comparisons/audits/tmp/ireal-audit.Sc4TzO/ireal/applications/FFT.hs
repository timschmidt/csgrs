module FFT where

import ListNumSyntax
import Data.Complex
import Data.Number.IReal

-- All functions in this file take a precision argument for (less in-)efficiency.
-- To get d correct decimals in result, slightly higher precision 
-- argument should be used.

-- basic fast Fourier transform, for power of 2 lengths only
fft :: (VarPrec a, RealFloat a) => Int -> [Complex a] -> [Complex a]
fft d [] = error "fft: empty list"
fft d [a] = [prec d a]
fft d [a,b] = let a' = prec d a; b' = prec d b in [a'+b',a'-b']
fft d as = interleave (fft d cs) (fft d ds)
  where
    n = length as
    (ls,rs) = splitAt (n `div` 2)  (prec d as)

    a = -2*pi/fromIntegral n
    tws = [ cis (a * fromIntegral k) | k <-  [0..n `div` 2 - 1] ]

    cs =  ls + rs
    ds =  tws * (ls - rs)

    interleave [] bs = bs
    interleave (a : as) bs = a : interleave bs as

-- inverse FFT
ifft :: (VarPrec a, RealFloat a) => Int -> [Complex a] -> [Complex a]
ifft d as = (fft d . rev) as / repeat n
  where n = fromIntegral (length as)
        rev (a : as) = a : reverse as

-- type 1 discrete cosine transform (for lengths 2^n + 1)
dct :: (VarPrec a, RealFloat a) => Int -> [a] -> [a]
dct d as =  (map ((/2) . realPart) . take (length as) . fft d . map (:+ 0)) as'
   where as' = as ++ tail (reverse (tail as))

-- inverse DCT
idct :: (VarPrec a, RealFloat a) => Int -> [a] -> [a]
idct d cs = dct d cs / repeat n'
   where n' = fromIntegral ((length cs - 1) `div` 2)

-- unscaled type 1 DCT 
dctu d [x0,x1] = [x0+x1,x0-x1]
dctu d xs = dct d (eDouble xs)
  where eDouble (x:xs) = 2*x : t xs
        t [x] = [2*x]
        t (x:xs) = x : t xs

-- its inverse
idctu d [c0,c1] = [(c0+c1)/2,(c0-c1)/2]
idctu d xs = eHalve (idct d xs)
   where eHalve (x:xs) = x/2 : t xs
         t [x] = [x/2]
         t (x:xs) = x : t xs

-- discrete sine transform (for lengths 2^n - 1)
dst d as =  (map ((/2) . negate . imagPart) . tail . take (length as+1) . fft d . map (:+ 0)) as'
   where as' = 0 : as ++ 0 : map negate (reverse as)

-- its inverse
idst d cs = dst d cs / repeat n'
  where n' =  fromIntegral ((length cs + 1) `div` 2)

instance VarPrec a => VarPrec (Complex a) where
  precB p (a :+ b) = precB p a :+ precB p b
 
